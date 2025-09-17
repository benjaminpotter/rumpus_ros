use chrono::prelude::*;
use clap::Parser;
use rayon::prelude::*;
use rosbags_rs::Reader;
use rosbags_rs::cdr::CdrDeserializer;
use rosbags_rs::messages::FromCdr;
use rosbags_rs::types::Connection;
use rumpus::prelude::*;
use rumpus_ros::messages::*;
use sguaba::engineering::Orientation;
use sguaba::systems::{EquivalentTo, Wgs84};
use sguaba::{Coordinate, system};
use std::path::PathBuf;
use uom::si::{
    angle::{Angle, degree},
    length::{Length, meter, micron, millimeter},
};

#[derive(Parser)]
struct Cli {
    bag_path: PathBuf,

    #[arg(short, long)]
    output_dir: Option<PathBuf>,

    #[arg(short, long)]
    max_images: Option<usize>,
}

fn main() {
    let args = Cli::parse();
    let output_dir = args.output_dir.unwrap_or("./".into());
    let topic_name = "/novatel/oem7/inspvax";
    let pixel_size = Length::new::<micron>(3.45 * 2.);
    let focal_length = Length::new::<millimeter>(8.);
    let image_rows = 1024;
    let image_cols = 1224;

    std::fs::create_dir_all(&output_dir).unwrap();

    let mut reader = Reader::new(&args.bag_path).unwrap();
    reader.open().unwrap();

    // Find connections with correct topic.
    let connections: Vec<Connection> = reader
        .connections()
        .iter()
        .filter(|conn| &conn.topic == &topic_name)
        .cloned()
        .collect();

    let messages = reader
        .messages_filtered(Some(&connections), None, None)
        .unwrap();

    let lens = Lens::from_focal_length(focal_length).expect("positive focal length");
    let image_sensor = ImageSensor::new(pixel_size, pixel_size, image_rows, image_cols);
    let coords: Vec<Coordinate<CameraFrd>> = (0..image_rows)
        .flat_map(|row| (0..image_cols).map(move |col| (row, col)))
        .map(|(row, col)| image_sensor.at_pixel(row, col).unwrap())
        .collect();

    for (i, result) in messages
        .enumerate()
        .take_while(|(i, _result)| args.max_images.is_none_or(|ref max| i < max))
    {
        let message = result.unwrap();
        let mut deserializer = CdrDeserializer::new(&message.data).unwrap();
        let inspvax = InsPvaX::from_cdr(&mut deserializer).unwrap();
        let state: State<InsEnu> = inspvax.into();

        let sky_model = SkyModel::from_wgs84_and_time(state.position(), state.time());

        // Camera aligned with the INS sensor reference frame.
        let cam_orientation = state.orientation().cast::<CameraEnu>();

        let camera = Camera::new(lens.clone(), cam_orientation);
        let rays: Vec<Ray<_>> = coords
            .par_iter()
            .filter_map(|coord| {
                let bearing_cam_enu = camera
                    .trace_from_sensor(*coord)
                    .expect("coord on sensor plane");
                let aop = sky_model.aop(bearing_cam_enu)?;

                Some(Ray::new(*coord, aop, Dop::new(0.0)))
            })
            .collect();

        // Could add RayImage::from_camera_with_sensor
        let ray_image = RayImage::from_rays_with_sensor(rays, &image_sensor)
            .expect("no ray hits the same pixel");

        // Map the AoP values in the RayImage to RGB colours.
        // Draw missing pixels as white.
        let aop_image: Vec<u8> = ray_image
            .ray_pixels()
            .flat_map(|pixel| match pixel {
                Some(ray) => to_rgb(ray.aop().angle().get::<degree>(), -90.0, 90.0)
                    .expect("aop in between -90 and 90"),
                None => [255, 255, 255],
            })
            .collect();

        let frame_id = format!("frame_{}.png", i);
        let mut image_path = PathBuf::new();
        image_path.push(&output_dir);
        image_path.push(&frame_id);

        // Save the buffer of RGB pixels as a PNG.
        image::save_buffer(
            &image_path,
            &aop_image,
            image_cols.into(),
            image_rows.into(),
            image::ExtendedColorType::Rgb8,
        )
        .expect("valid image and path");

        println!("wrote image");
    }
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct State<In> {
    orientation: Orientation<In>,
    position: Wgs84,
    time: DateTime<Utc>,
}

impl<In> State<In> {
    pub fn new(orientation: Orientation<In>, position: Wgs84, time: DateTime<Utc>) -> Self {
        Self {
            orientation,
            position,
            time,
        }
    }

    pub fn orientation(&self) -> Orientation<In> {
        self.orientation
    }

    pub fn position(&self) -> Wgs84 {
        self.position
    }

    pub fn time(&self) -> DateTime<Utc> {
        self.time
    }
}

system!(pub struct InsEnu using ENU);

// SAFETY: Since camera is simulated, we can place it directly on top of INS.
unsafe impl EquivalentTo<CameraEnu> for InsEnu {}
unsafe impl EquivalentTo<InsEnu> for CameraEnu {}

impl From<InsPvaX> for State<InsEnu> {
    fn from(inspvax: InsPvaX) -> Self {
        let orientation = Orientation::<InsEnu>::tait_bryan_builder()
            .yaw(Angle::new::<degree>(inspvax.azimuth))
            .pitch(Angle::new::<degree>(inspvax.pitch))
            .roll(Angle::new::<degree>(inspvax.roll))
            .build();

        let position = Wgs84::builder()
            .longitude(Angle::new::<degree>(inspvax.longitude))
            .latitude(Angle::new::<degree>(inspvax.latitude))
            .expect("inspvax message provides latitude between -90 and 90 degrees")
            .altitude(Length::new::<meter>(inspvax.height))
            .build();

        // chrono_gpst expects week seconds.
        let time = chrono_gpst::from_gpst(
            inspvax.nov_header.gps_week_number.into(),
            inspvax.nov_header.gps_week_milliseconds as f64 / 1000.0,
            true,
        )
        .expect("inspvax message provides valid gpst time");

        Self::new(orientation, position, time)
    }
}

// Map an f64 on the interval [x_min, x_max] to an RGB color.
pub fn to_rgb(x: f64, x_min: f64, x_max: f64) -> Option<[u8; 3]> {
    if x < x_min || x > x_max {
        return None;
    }

    let interval_width = x_max - x_min;
    let x_norm = ((x - x_min) / interval_width * 255.).floor() as u8;

    let r = vec![
        255,
        x_norm
            .checked_sub(96)
            .unwrap_or(u8::MIN)
            .checked_mul(4)
            .unwrap_or(u8::MAX),
        255 - x_norm
            .checked_sub(224)
            .unwrap_or(u8::MIN)
            .checked_mul(4)
            .unwrap_or(u8::MAX),
    ]
    .into_iter()
    .min()
    .unwrap();

    let g = vec![
        255,
        x_norm
            .checked_sub(32)
            .unwrap_or(u8::MIN)
            .checked_mul(4)
            .unwrap_or(u8::MAX),
        255 - x_norm
            .checked_sub(160)
            .unwrap_or(u8::MIN)
            .checked_mul(4)
            .unwrap_or(u8::MAX),
    ]
    .into_iter()
    .min()
    .unwrap();

    let b = vec![
        255,
        x_norm
            .checked_add(127)
            .unwrap_or(u8::MIN)
            .checked_mul(4)
            .unwrap_or(u8::MAX),
        255 - x_norm
            .checked_sub(96)
            .unwrap_or(u8::MIN)
            .checked_mul(4)
            .unwrap_or(u8::MAX),
    ]
    .into_iter()
    .min()
    .unwrap();

    Some([r, g, b])
}
