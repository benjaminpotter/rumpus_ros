use chrono::prelude::*;
use clap::Parser;
use rayon::prelude::*;
use rosbags_rs::Reader;
use rosbags_rs::cdr::CdrDeserializer;
use rosbags_rs::messages::FromCdr;
use rosbags_rs::messages::Image;
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

const INS_TOPIC_NAME: &str = "/novatel/oem7/inspvax";
const CAM_TOPIC_NAME: &str = "/arena_camera_node/image_raw";

#[derive(Parser)]
struct Cli {
    bag_path: PathBuf,

    #[arg(short, long)]
    max_msgs: Option<usize>,
}

fn main() {
    let args = Cli::parse();

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

    for conn in &connections {
        println!("{}", conn.message_definition.data);
    }

    let messages = reader
        .messages_filtered(Some(&connections), None, None)
        .unwrap();

    println!("filtered messages");

    // Can I make a Kalman filter library that structures filters like iterator consumers?
    // Collect messages into the filter and advance the state in time.

    for (i, result) in messages
        .enumerate()
        .take_while(|(i, _result)| args.max_msgs.is_none_or(|ref max| i < max))
    {
        let message = result.unwrap();
        let mut deserializer = CdrDeserializer::new(&message.data).unwrap();

        match message.topic.as_str() {
            INS_TOPIC_NAME => {
                let inspvax = InsPvaX::from_cdr(&mut deserializer).unwrap();
                let _state: State<InsEnu> = inspvax.into();

                println!("handled ins message");
            }
            CAM_TOPIC_NAME => {
                let image = Image::from_cdr(&mut deserializer).unwrap();
                let image_name = format!("frame_{}.png", i);

                image::save_buffer(
                    &image_name,
                    &image.data[..],
                    image.width,
                    image.height,
                    image::ExtendedColorType::L8,
                )
                .unwrap();

                println!("handled cam message");
            }
            _ => unreachable!(),
        }
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
