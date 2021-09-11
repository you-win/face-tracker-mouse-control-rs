use std::{
    error::Error,
    f32::consts::PI,
    io::{Cursor, Read, Seek},
    mem,
    net::SocketAddr,
};

use byteorder::{NativeEndian, ReadBytesExt};
use enigo::*;
use glam::{Quat, Vec2, Vec3};
use tokio::net::UdpSocket;

const FACE_TRACKER_ADDRESS: &str = "127.0.0.1:11573";
const TRACKING_POINTS: usize = 68;
const PACKET_FRAME_SIZE: usize = 8
    + 4
    + 2 * 4
    + 2 * 4
    + 1
    + 4
    + 3 * 4
    + 3 * 4
    + 4 * 4
    + 4 * 68
    + 4 * 2 * 68
    + 4 * 3 * 70
    + 4 * 14;

#[derive(Default)]
struct OpenSeeData {
    time: f64,
    id: i32,
    camera_resolution: Vec2,

    right_eye_open: f32,
    left_eye_open: f32,
    right_gaze: Quat,
    left_gaze: Quat,
    // right_gaze: Vec3,
    // left_gaze: Vec3,
    got_3d_points: bool,
    fit_3d_error: f32,

    // rotation: Vector3,
    translation: Vec3,
    raw_quaternion: Quat,
    raw_euler: Vec3,

    confidence: Vec<f32>,
    points: Vec<Vec2>,
    points_3d: Vec<Vec3>,

    open_see_features: OpenSeeFeatures,
}

impl OpenSeeData {
    pub fn new() -> Self {
        let mut osd = OpenSeeData::default();

        osd.confidence = Vec::with_capacity(TRACKING_POINTS);
        osd.points = Vec::with_capacity(TRACKING_POINTS);
        osd.points_3d = Vec::with_capacity(TRACKING_POINTS + 2);

        return osd;
    }
}

#[derive(Default)]
struct OpenSeeFeatures {
    eye_left: f32,
    eye_right: f32,

    eyebrow_steepness_left: f32,
    eyebrow_up_down_left: f32,
    eyebrow_quirk_left: f32,

    eyebrow_steepness_right: f32,
    eyebrow_up_down_right: f32,
    eyebrow_quirk_right: f32,

    mouth_corner_up_down_left: f32,
    mouth_corner_in_out_left: f32,

    mouth_corner_up_down_right: f32,
    mouth_corner_in_out_right: f32,

    mouth_open: f32,
    mouth_wide: f32,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    println!("Starting receive at {}", FACE_TRACKER_ADDRESS);

    let socket = UdpSocket::bind(FACE_TRACKER_ADDRESS).await?;

    let mut buffer = [0; 1785];

    loop {
        let valid_bytes = Some(socket.recv(&mut buffer).await?);

        if let Some(size) = valid_bytes {
            if size < 1 || size % PACKET_FRAME_SIZE != 0 {
                continue;
            }
            let mut osd = OpenSeeData::new();
            let mut osd_features = OpenSeeFeatures::default();
            let mut reader = Cursor::new(buffer.clone());

            osd.time = read_double(&mut reader);
            osd.id = read_int(&mut reader);
            osd.camera_resolution = read_vec2(&mut reader);

            osd.right_eye_open = read_float(&mut reader);
            osd.left_eye_open = read_float(&mut reader);

            osd.got_3d_points = read_bool(&mut reader);
            osd.fit_3d_error = read_float(&mut reader);

            osd.raw_quaternion = read_quaternion(&mut reader);
            osd.raw_euler = read_vec3(&mut reader);
            osd.translation = read_vec3(&mut reader);

            for _ in 0..TRACKING_POINTS {
                osd.confidence.push(read_float(&mut reader));
            }
            for _ in 0..TRACKING_POINTS {
                osd.points.push(read_vec2(&mut reader));
            }
            for _ in 0..TRACKING_POINTS + 2 {
                osd.points_3d.push(read_vec3(&mut reader));
            }

            // osd.right_gaze = Quat::from_axis_angle(osd.points_3d[66] - osd.points_3d[68], 0.0);
            // osd.left_gaze = Quat::from_axis_angle(osd.points_3d[67] - osd.points_3d[69], 0.0);
            osd.right_gaze = Quat::from_scaled_axis(osd.points_3d[66] - osd.points_3d[68])
                * Quat::from_axis_angle(Vec3::Z, PI)
                * Quat::from_axis_angle(Vec3::X, PI);
            osd.left_gaze = Quat::from_scaled_axis(osd.points_3d[67] - osd.points_3d[69])
                * Quat::from_axis_angle(Vec3::Z, PI)
                * Quat::from_axis_angle(Vec3::X, PI);
            // osd.right_gaze = osd.points_3d[66] - osd.points_3d[68];
            // osd.left_gaze = osd.points_3d[67] - osd.points_3d[69];

            osd_features.eye_left = read_float(&mut reader);
            osd_features.eye_right = read_float(&mut reader);

            osd_features.eyebrow_steepness_left = read_float(&mut reader);
            osd_features.eyebrow_up_down_left = read_float(&mut reader);
            osd_features.eyebrow_quirk_left = read_float(&mut reader);

            osd_features.eyebrow_steepness_right = read_float(&mut reader);
            osd_features.eyebrow_up_down_right = read_float(&mut reader);
            osd_features.eyebrow_quirk_right = read_float(&mut reader);

            osd_features.mouth_corner_up_down_left = read_float(&mut reader);
            osd_features.mouth_corner_in_out_left = read_float(&mut reader);
            osd_features.mouth_corner_up_down_right = read_float(&mut reader);
            osd_features.mouth_corner_in_out_right = read_float(&mut reader);

            osd_features.mouth_open = read_float(&mut reader);
            osd_features.mouth_wide = read_float(&mut reader);

            osd.open_see_features = osd_features;

            // println!("position: {}", reader.position());

            // println!(
            //     "left_eyebrow: {}",
            //     osd.open_see_features.eyebrow_up_down_left
            // );

            // println!("left_gaze: {}", osd.left_gaze);

            // println!("0: {}, 1: {}", osd.points_3d[66], osd.points_3d[68]);

            // println!("right_eye_open: {}", osd.right_eye_open);

            println!("translation: {}", osd.translation);
        }
    }

    Ok(())
}

fn read_int<R: Read + Seek>(reader: &mut R) -> i32 {
    reader.read_i32::<NativeEndian>().unwrap()
}

fn read_float<R: Read + Seek>(reader: &mut R) -> f32 {
    reader.read_f32::<NativeEndian>().unwrap()
}

fn read_double<R: Read + Seek>(reader: &mut R) -> f64 {
    reader.read_f64::<NativeEndian>().unwrap()
}

fn read_vec2<R: Read + Seek>(reader: &mut R) -> Vec2 {
    Vec2::new(read_float(reader), read_float(reader))
}

fn read_vec3<R: Read + Seek>(reader: &mut R) -> Vec3 {
    Vec3::new(read_float(reader), read_float(reader), read_float(reader))
}

fn read_quaternion<R: Read + Seek>(reader: &mut R) -> Quat {
    Quat::from_xyzw(
        read_float(reader),
        read_float(reader),
        read_float(reader),
        read_float(reader),
    )
}

fn read_bool<R: Read + Seek>(reader: &mut R) -> bool {
    match reader.read_u8().unwrap() {
        0 => return false,
        1 => return true,
        _ => return true,
    }
}
