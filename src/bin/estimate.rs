use chrono::prelude::*;
use clap::Parser;
use rayon::prelude::*;
use rosbags_rs::Reader;
use rosbags_rs::cdr::CdrDeserializer;
use rosbags_rs::messages::FromCdr;
use rosbags_rs::messages::Image;
use rosbags_rs::types::Connection;
use rumpus::prelude::*;
use rumpus::ray::RayFrame;
use rumpus_ros::messages::*;
use sguaba::engineering::Orientation;
use sguaba::systems::{EquivalentTo, Wgs84};
use sguaba::{Bearing, Coordinate, system, vector};
use std::io::Read;
use std::path::PathBuf;
use uom::{
    ConstZero,
    si::{
        angle::{degree, radian},
        f64::{Angle, Length},
        length::{meter, micron, millimeter},
        ratio::ratio,
    },
};

const INS_TOPIC_NAME: &str = "/novatel/oem7/inspvax";
const CAM_TOPIC_NAME: &str = "/arena_camera_node/image_raw";

#[derive(Parser)]
struct Cli {
    bag_path: PathBuf,

    #[arg(short, long)]
    params: PathBuf,

    #[arg(short, long)]
    max_msgs: Option<usize>,
}

#[derive(serde::Serialize, serde::Deserialize)]
struct SimulationParams {
    pixel_size: Length,
    focal_length: Length,
    min_dop: f64,
    max_iters: usize,
    /// The rate at which the current estimate descends to the optimal estimate.
    ///
    /// A larger learning rate can decrease the number of steps to the optimal estimate, but may lead to oscillation
    /// about the optimal estimate, preventing convergence.
    /// The best learning rate is the largest rate that still converges.
    learning_rate: f64,

    /// The threshold on the magnitude of the gradient that is considered the optimal estimate.
    convergence_threshold: f64,
}

#[derive(serde::Serialize, Clone, Copy)]
struct Candidate {
    yaw: f64,
    pitch: f64,
    roll: f64,
    /// The x coordinate of the up vector when oriented at yaw, pitch, roll.
    optical_axis_east: f64,
    /// The y coordinate of the up vector when oriented at yaw, pitch, roll.
    optical_axis_north: f64,
    loss: f64,
    gradient: f64,
    iters: usize,
    converged: bool,
    epoch: usize,
}

#[derive(serde::Serialize)]
struct Estimate {
    timestamp: DateTime<Utc>,
    nanosec: u32,
    message_index: usize,
    ins_yaw: f64,
    ins_pitch: f64,
    ins_roll: f64,
    ins_latitude: f64,
    ins_longitude: f64,
    yaw: f64,
    pitch: f64,
    roll: f64,
    /// The x coordinate of the up vector when oriented at yaw, pitch, roll.
    optical_axis_east: f64,
    /// The y coordinate of the up vector when oriented at yaw, pitch, roll.
    optical_axis_north: f64,
    loss: f64,
    gradient: f64,
    iters: usize,
    converged: bool,
    epoch: usize,
}

fn main() {
    let args = Cli::parse();
    let params = parse_params(&args.params).expect("readable and parsable params");

    let mut reader = Reader::new(&args.bag_path).unwrap();
    reader.open().unwrap();

    println!("opened bag");

    // Find connections with correct topic.
    let connections: Vec<Connection> = reader
        .connections()
        .iter()
        .filter(|conn| [INS_TOPIC_NAME, CAM_TOPIC_NAME].contains(&conn.topic.as_str()))
        .cloned()
        .collect();

    // for conn in &connections {
    //     println!("{}", conn.message_definition.data);
    // }

    let messages = reader
        .messages_filtered(Some(&connections), None, None)
        .unwrap();

    println!("filtered messages");

    // Can I make a Kalman filter library that structures filters like iterator consumers?
    // Collect messages into the filter and advance the state in time.

    let lens =
        Lens::from_focal_length(params.focal_length).expect("focal length is greater than zero");
    let zenith_bearing = Bearing::<CameraEnu>::builder()
        .azimuth(Angle::ZERO)
        .elevation(Angle::HALF_TURN / 2.)
        .expect("elevation is on range -90 to 90")
        .build();

    // There are two minima in the loss occuring at the solar and anti solar alignment.
    // They will occur at a 180 phase offset.
    // One of them will be the global minima.
    // Start with two seeds with a 180 phase offset to determine global minima.
    let seed_orientations = vec![
        Orientation::<CameraEnu>::tait_bryan_builder()
            .yaw(Angle::new::<degree>(0.0))
            .pitch(Angle::new::<degree>(0.0))
            .roll(Angle::new::<degree>(0.0))
            .build(),
        Orientation::<CameraEnu>::tait_bryan_builder()
            .yaw(Angle::new::<degree>(180.0))
            .pitch(Angle::new::<degree>(0.0))
            .roll(Angle::new::<degree>(0.0))
            .build(),
    ];

    let mut state: Option<State<InsEnu>> = None;
    let bag_csv_filename = format!("{}.csv", "bag");
    let mut bag_csv = csv::Writer::from_path(&bag_csv_filename).unwrap();

    for (i, result) in messages
        .enumerate()
        .take_while(|(i, _result)| args.max_msgs.is_none_or(|ref max| i < max))
    {
        let message = result.unwrap();
        let mut deserializer = CdrDeserializer::new(&message.data).unwrap();

        match message.topic.as_str() {
            INS_TOPIC_NAME => {
                println!("received ins message");

                let inspvax = InsPvaX::from_cdr(&mut deserializer).unwrap();
                state = Some(inspvax.into());
                println!("updated state");

                println!("handled ins message");
            }
            CAM_TOPIC_NAME => {
                println!("received cam message");

                if let Some(state) = state {
                    let image = Image::from_cdr(&mut deserializer).unwrap();
                    assert_eq!(&image.encoding, "mono8");
                    let rays: Vec<Ray<SensorFrame>> = IntensityImage::from_bytes(
                        image.width as u16,
                        image.height as u16,
                        &image.data[..],
                    )
                    .expect("image dimensions are even")
                    .rays(params.pixel_size, params.pixel_size)
                    .ray_filter(DopFilter::new(params.min_dop))
                    .collect();

                    let ray_count = rays.len();
                    if ray_count == 0 {
                        println!("no rays met dop threshold");
                        std::process::exit(0);
                    }

                    let model = SkyModel::from_wgs84_and_time(state.position(), state.time());
                    let solar_azimuth = model.solar_bearing().azimuth();
                    let solar_zenith = Angle::HALF_TURN / 2. - model.solar_bearing().elevation();

                    // Save the gradient descent steps with the timestamp as the file name.
                    let filename = format!("{}_{}.csv", i, image.header.stamp.nanosec);
                    let mut image_csv = csv::Writer::from_path(&filename).unwrap();
                    let mut estimates = Vec::new();

                    let mut epoch = 0;
                    for seed in &seed_orientations {
                        let mut orientation = *seed;

                        epoch += 1;
                        println!("start epoch {}", epoch);

                        let mut estimate: Option<Candidate> = None;
                        let mut converged = false;
                        let mut iters = 0;
                        loop {
                            iters += 1;
                            if iters > params.max_iters {
                                println!("reached max_iters");
                                break;
                            }

                            // Construct a camera at the new orientation.
                            // TODO: Probably don't need to clone the orientation here.
                            let cam = Camera::new(lens.clone(), orientation.clone());

                            // Find the zenith coordinate in CameraFrd.
                            let zenith_coord = cam
                                .trace_from_sky(zenith_bearing.clone())
                                .expect("zenith is always above the horizon");

                            struct RayInfo<Frame: RayFrame> {
                                difference: Aop<Frame>,
                                partial_derivative: Aop<Frame>,
                                weight: f64,
                            }

                            let ray_info: Vec<RayInfo<_>> = rays
                                .par_iter()
                                .filter_map(|ray_sensor| {
                                    // Model a ray with the same CameraFrd coordinate as the measured ray.
                                    let ray_bearing = cam
                                        .trace_from_sensor(*ray_sensor.coord())
                                        .expect("ray coordinate should always have Z of zero");

                                    // Ignore rays from below the horizon.
                                    let modelled_aop = model.aop(ray_bearing)?;
                                    let modelled_ray_global =
                                        Ray::new(*ray_sensor.coord(), modelled_aop, Dop::zero());

                                    if ray_bearing.elevation() < Angle::ZERO {
                                        return None;
                                    }

                                    let azimuth = ray_bearing.azimuth();
                                    let zenith = Angle::HALF_TURN / 2. - ray_bearing.elevation();

                                    let a = (zenith.sin() * solar_zenith.cos()).get::<ratio>();
                                    let b = zenith.cos().get::<ratio>();
                                    let c = solar_zenith.sin().get::<ratio>();
                                    let d = (azimuth - solar_azimuth).get::<radian>();

                                    // \frac{c\left(bc\sin ^2\left(x-d\right)-\cos \left(x-d\right)\left(-bc\cos \left(x-d\right)+a\right)\right)}{\left(-bc\cos \left(x-d\right)+a\right)^2+c^2\sin ^2\left(x-d\right)}
                                    let angle = c
                                        * (b * c * d.sin().powf(2.)
                                            - d.cos() * (-1. * b * c * d.cos() + a))
                                        / ((-1. * b * c * d.cos() + a).powf(2.)
                                            + c.powf(2.) * d.sin().powf(2.));

                                    let partial_derivative =
                                        Aop::from_angle_wrapped(Angle::new::<radian>(angle));

                                    // Transform the measured ray from the sensor frame into the global frame.
                                    let ray_global = ray_sensor
                                        .into_global_frame(zenith_coord.clone())
                                        // Camera trace_from_sky always returns a coordinate with a zenith of zero which enforces this expect.
                                        .expect("zenith coord has a Z of zero");

                                    let difference = *modelled_ray_global.aop() - *ray_global.aop();
                                    let weight = 1. / (*ray_global.dop()).into_inner();

                                    Some(RayInfo {
                                        difference,
                                        partial_derivative,
                                        weight,
                                    })
                                })
                                .collect();

                            let loss = ray_info
                                .par_iter()
                                .map(|ri| {
                                    let sq_diff =
                                        ri.difference.into_inner().get::<radian>().powf(2.);
                                    let weighted_sq_diff = ri.weight * sq_diff;

                                    weighted_sq_diff
                                })
                                // Take the mean of the weighted, squared differences.
                                .sum::<f64>()
                                / rays.len() as f64;

                            let gradient = ray_info
                                .par_iter()
                                .map(|ri| {
                                    let weighted_delta = ri.weight
                                        * ri.difference.into_inner().get::<radian>()
                                        * ri.partial_derivative.into_inner().get::<radian>();

                                    weighted_delta
                                })
                                .sum::<f64>()
                                / rays.len() as f64
                                * 2.;

                            if gradient.abs() < params.convergence_threshold {
                                converged = true;
                            }

                            // Create a unit vector along the camera's optical axis.
                            let optical_axis_frd = vector!(
                                f = Length::new::<meter>(0.0),
                                r = Length::new::<meter>(0.0),
                                d = Length::new::<meter>(1.0);
                                in CameraFrd
                            );

                            // Transform the unit vector by orientation.
                            let camera_frd_to_enu =
                                unsafe { orientation.map_as_zero_in::<CameraFrd>() }.inverse();
                            let optical_axis_enu = camera_frd_to_enu.transform(optical_axis_frd);

                            let (yaw, pitch, roll) = orientation.to_tait_bryan_angles();
                            let candidate = Candidate {
                                yaw: yaw.get::<degree>(),
                                pitch: pitch.get::<degree>(),
                                roll: roll.get::<degree>(),
                                optical_axis_east: optical_axis_enu.enu_east().get::<meter>(),
                                optical_axis_north: optical_axis_enu.enu_north().get::<meter>(),
                                loss,
                                gradient,
                                iters,
                                converged,
                                epoch,
                            };
                            image_csv.serialize(candidate.clone()).unwrap();

                            estimate = Some(candidate);

                            if converged {
                                println!("estimate converged");
                                break;
                            }

                            // Gradient is the expected delta loss per delta yaw.
                            // We want to move towards a yaw that minimizes the loss.
                            // If delta loss is positive that implies increasing yaw will increase loss.
                            // If delta loss is negative that implies decreasing yaw will increase loss.
                            // We want to do the opposite.
                            // If we haven't converged, then compute the next orientation to check.
                            // The next orientation should be the current orientation, but shifted by:
                            //   O' = O - learning_rate * gradient
                            // To start, we only vary the yaw and assume pitch and roll are zero.
                            //   O'.yaw = O.yaw - learning_rate * gradient
                            let (yaw, _pitch, _roll) = orientation.to_tait_bryan_angles();
                            let delta_yaw = Angle::new::<radian>(params.learning_rate * gradient);
                            orientation = Orientation::<CameraEnu>::tait_bryan_builder()
                                .yaw(yaw + delta_yaw)
                                .pitch(Angle::new::<degree>(0.0))
                                .roll(Angle::new::<degree>(0.0))
                                .build();
                        }

                        if let Some(estimate) = estimate {
                            estimates.push(estimate);
                        }
                    }

                    let estimate = estimates.into_iter().min_by(|a, b| {
                        a.loss
                            .partial_cmp(&b.loss)
                            // In this case, we have a NaN or Inf loss value...
                            .unwrap_or(std::cmp::Ordering::Less)
                    });

                    if let Some(candidate) = estimate {
                        let (ins_yaw, ins_pitch, ins_roll) =
                            state.orientation().to_tait_bryan_angles();
                        let estimate = Estimate {
                            timestamp: state.time(),
                            nanosec: image.header.stamp.nanosec,
                            message_index: i,
                            ins_yaw: ins_yaw.get::<degree>(),
                            ins_pitch: ins_pitch.get::<degree>(),
                            ins_roll: ins_roll.get::<degree>(),
                            ins_latitude: state.position().latitude().get::<degree>(),
                            ins_longitude: state.position().longitude().get::<degree>(),
                            yaw: candidate.yaw,
                            pitch: candidate.pitch,
                            roll: candidate.roll,
                            optical_axis_east: candidate.optical_axis_east,
                            optical_axis_north: candidate.optical_axis_north,
                            loss: candidate.loss,
                            gradient: candidate.gradient,
                            iters: candidate.iters,
                            converged: candidate.converged,
                            epoch: candidate.epoch,
                        };

                        bag_csv.serialize(estimate).unwrap();

                        // Write a simulated image from the candidate.

                        let sky_model =
                            SkyModel::from_wgs84_and_time(state.position(), state.time());

                        // Camera aligned with the INS sensor reference frame.
                        let cam_orientation = Orientation::<CameraEnu>::tait_bryan_builder()
                            .yaw(Angle::new::<degree>(candidate.yaw))
                            .pitch(Angle::new::<degree>(candidate.pitch))
                            .roll(Angle::new::<degree>(candidate.roll))
                            .build();

                        let camera = Camera::new(lens.clone(), cam_orientation);
                        let image_sensor = ImageSensor::new(
                            params.pixel_size,
                            params.pixel_size,
                            image.height as u16,
                            image.width as u16,
                        );

                        let coords: Vec<Coordinate<CameraFrd>> = (0..image.height as u16)
                            .flat_map(|row| (0..image.width as u16).map(move |col| (row, col)))
                            .map(|(row, col)| image_sensor.at_pixel(row, col).unwrap())
                            .collect();

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

                        // Save the buffer of RGB pixels as a PNG.
                        let aop_image_filename =
                            format!("{}_{}.png", i, image.header.stamp.nanosec);
                        image::save_buffer(
                            &aop_image_filename,
                            &aop_image,
                            image.width,
                            image.height,
                            image::ExtendedColorType::Rgb8,
                        )
                        .expect("valid image and path");
                    }

                    println!("wrote estimate for camera frame");
                } else {
                    println!("state is uninitialized, skipping cam message");
                }

                println!("handled cam message");
            }
            _ => unreachable!(),
        }
    }
}

fn parse_params(path: &PathBuf) -> Option<SimulationParams> {
    let mut buffer = String::new();
    std::fs::File::open(path)
        .ok()?
        .read_to_string(&mut buffer)
        .ok()?;
    toml::from_str(&buffer).ok()
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
