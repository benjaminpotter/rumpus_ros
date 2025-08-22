use bitflags::bitflags;
use rosbags_rs::cdr::CdrDeserializer;
use rosbags_rs::messages::{FromCdr, Header};

/// novatel_oem7_msgs/msg/INSPVAX
///
/// Definitions and coordinate system information is found at [2] and [3].
///
/// ### Message Definition [1]
///
/// ```text
/// std_msgs/Header             header
/// Oem7Header                  nov_header
/// InertialSolutionStatus      ins_status
/// PositionOrVelocityType      pos_type
/// float64                     latitude
/// float64                     longitude  
/// float64                     height
/// float32                     undulation
/// float64                     north_velocity
/// float64                     east_velocity
/// float64                     up_velocity
/// float64                     roll
/// float64                     pitch
/// float64                     azimuth
/// float32                     latitude_stdev
/// float32                     longitude_stdev
/// float32                     height_stdev
/// float32                     north_velocity_stdev
/// float32                     east_velocity_stdev
/// float32                     up_velocity_stdev
/// float32                     roll_stdev
/// float32                     pitch_stdev
/// float32                     azimuth_stdev
/// INSExtendedSolutionStatus   ext_sol_status
/// uint16                      time_since_update
/// ```
///
/// [1] https://docs.ros.org/en/humble/p/novatel_oem7_msgs/msg/INSPVAX.html
/// [2] https://docs.novatel.com/OEM7/Content/SPAN_Logs/INSPVAX.htm
/// [3] https://docs.novatel.com/OEM7/Content/SPAN_Operation/Definition_Reference_Frames.htm
#[derive(Debug, Clone, PartialEq)]
pub struct InsPvaX {
    pub header: Header,
    pub nov_header: Oem7Header,
    pub ins_status: InertialSolutionStatus,
    pub pos_type: PositionOrVelocityType,
    pub latitude: f64,
    pub longitude: f64,
    pub height: f64,
    pub undulation: f32,
    pub north_velocity: f64,
    pub east_velocity: f64,
    pub up_velocity: f64,
    pub roll: f64,
    pub pitch: f64,
    pub azimuth: f64,
    pub latitude_stdev: f32,
    pub longitude_stdev: f32,
    pub height_stdev: f32,
    pub north_velocity_stdev: f32,
    pub east_velocity_stdev: f32,
    pub up_velocity_stdev: f32,
    pub roll_stdev: f32,
    pub pitch_stdev: f32,
    pub azimuth_stdev: f32,
    pub ext_sol_status: InsExtendedSolutionStatus,
    pub time_since_update: u16,
}

///
/// ### Message Definition [1]
///
/// ```text
/// uint16 OEM7MSGTYPE_LOG =  0
///
/// string   message_name
/// uint16   message_id
/// uint8    message_type
/// uint32   sequence_number
/// uint8    time_status
/// uint16   gps_week_number
/// uint32   gps_week_milliseconds
/// ```
///
/// ```text
/// uint32   receiver_status
/// float32  idle_time
/// ```
/// These two fields are included in the online documentation [1], but not in
/// the message definition used when testing.
///
/// [1] https://docs.ros.org/en/humble/p/novatel_oem7_msgs/msg/Oem7Header.html
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Oem7Header {
    pub message_name: String,
    pub message_id: u16,
    // TODO: Make pub enum.
    pub message_type: u8,
    pub sequence_number: u32,
    // TODO: Make pub enum.
    // https://docs.novatel.com/OEM7/Content/Messages/GPS_Reference_Time_Statu.htm#Table_GPSReferenceTimeStatus
    pub time_status: u8,
    pub gps_week_number: u16,
    pub gps_week_milliseconds: u32,
}

/// Represents the Inertial Solution Status as reported by the INSATT log.
///
/// [1] https://docs.ros.org/en/humble/p/novatel_oem7_msgs/msg/InertialSolutionStatus.html
/// [2] https://docs.novatel.com/OEM7/Content/SPAN_Logs/INSATT.htm#InertialSolutionStatus
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InertialSolutionStatus {
    /// IMU logs are present, but the alignment routine has not started; INS is inactive.
    Inactive = 0,

    /// INS is in alignment mode.
    Aligning = 1,

    /// The INS solution uncertainty contains outliers and the solution may be outside specifications. The solution is still valid but you should monitor the solution uncertainty in the INSSTDEV log. It may be encountered during times when GNSS is absent or poor.
    HighVariance = 2,

    /// The INS filter is in navigation mode and the INS solution is good.
    SolutionGood = 3,

    /// The INS Filter is in navigation mode and the GNSS solution is suspected to be in error. The inertial filter will report this status when there are no available updates (GNSS or other) being accepted and used.
    SolutionFree = 6,

    /// The INS filter is in navigation mode, but not enough vehicle dynamics have been experienced for the system to be within specifications.
    AlignmentComplete = 7,

    /// INS is determining the IMU axis aligned with gravity.
    DeterminingOrientation = 8,

    /// The INS filter has determined the IMU orientation and is awaiting an initial position estimate to begin the alignment process.
    WaitingInitialPos = 9,

    /// The INS filter has orientation, initial biases, initial position and valid roll/pitch estimated. Will not proceed until initial azimuth is entered.
    WaitingAzimuth = 10,

    /// The INS filter is estimating initial biases during the first 10 seconds of stationary data.
    InitializingBiases = 11,

    /// The INS filter has not completely aligned, but has detected motion.
    MotionDetect = 12,
}

impl InertialSolutionStatus {
    /// Attempts to convert a `u32` into an `InertialSolutionStatus`.
    ///
    /// Returns `Some(status)` if the value matches a known variant,
    /// or `None` if the value is unrecognized.
    pub fn from_u32(value: u32) -> Option<Self> {
        match value {
            0 => Some(InertialSolutionStatus::Inactive),
            1 => Some(InertialSolutionStatus::Aligning),
            2 => Some(InertialSolutionStatus::HighVariance),
            3 => Some(InertialSolutionStatus::SolutionGood),
            6 => Some(InertialSolutionStatus::SolutionFree),
            7 => Some(InertialSolutionStatus::AlignmentComplete),
            8 => Some(InertialSolutionStatus::DeterminingOrientation),
            9 => Some(InertialSolutionStatus::WaitingInitialPos),
            10 => Some(InertialSolutionStatus::WaitingAzimuth),
            11 => Some(InertialSolutionStatus::InitializingBiases),
            12 => Some(InertialSolutionStatus::MotionDetect),
            _ => None,
        }
    }
}

/// [1] https://docs.ros.org/en/humble/p/novatel_oem7_msgs/msg/PositionOrVelocityType.html
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PositionOrVelocityType {
    /// No solution
    No = 0,

    /// Position has been fixed by the FIX position command or by position averaging.
    FixedPos = 1,

    /// Position has been fixed by the FIX height or FIX auto command or by position averaging
    FixedHeight = 2,

    // 3–7 Reserved
    /// Velocity computed using instantaneous Doppler
    DopplerVelocity = 8,

    // 9–15 Reserved
    /// Solution calculated using only data supplied by the GNSS satellites
    Single = 16,

    /// Solution calculated using pseudorange differential (DGPS, DGNSS) corrections
    PsrDiff = 17,

    /// Solution calculated using corrections from an SBAS satellite
    Waas = 18,

    /// Propagated by a Kalman filter without new observations
    Propagated = 19,

    // 20–31 Reserved
    /// Single‑frequency RTK solution with unresolved, float carrier phase ambiguities
    L1Float = 32,

    // 33 Reserved
    /// Multi‑frequency RTK solution with unresolved, float carrier phase ambiguities
    NarrowFloat = 34,

    // 35–47 Reserved
    /// Single‑frequency RTK solution with carrier phase ambiguities resolved to integers
    L1Int = 48,

    /// Multi‑frequency RTK solution with carrier phase ambiguities resolved to wide‑lane integers
    WideInt = 49,

    /// Multi‑frequency RTK solution with carrier phase ambiguities resolved to narrow‑lane integers
    NarrowInt = 50,

    // 51 Reserved
    /// INS position, where the last applied position update used a GNSS solution computed using corrections from an SBAS (WAAS) solution
    InsSbas = 52,

    /// INS position, where the last applied position update used a single point GNSS (SINGLE) solution
    InsPsrSp = 53,

    /// INS position, where the last applied position update used a pseudorange differential GNSS (PSRDIFF) solution
    InsPsrDiff = 54,

    /// INS position, where the last applied position update used a floating ambiguity RTK (L1_FLOAT or NARROW_FLOAT) solution
    InsRtkFloat = 55,

    /// INS position, where the last applied position update used a fixed integer ambiguity RTK (L1_INT, WIDE_INT or NARROW_INT) solution
    InsRtkFixed = 56,

    // 57–66 Reserved
    /// INS position, where the last applied position update used an external source (entered using the EXTERNALPVAS command)
    ExtConstrained = 67,

    /// Converging TerraStar‑C PRO or TerraStar‑X solution
    PppConverging = 68,

    /// Converged TerraStar‑C PRO or TerraStar‑X solution
    Ppp = 69,

    /// Solution accuracy is within UAL operational limit
    Operational = 70,

    /// Solution accuracy is outside UAL operational limit but within warning limit
    Warning = 71,

    /// Solution accuracy is outside UAL limits
    OutOfBounds = 72,

    /// INS position, where the last applied position update used a converging TerraStar‑C PRO or TerraStar‑X PPP (PPP_CONVERGING) solution
    InsPppConverging = 73,

    /// INS position, where the last applied position update used a converged TerraStar‑C PRO or TerraStar‑X PPP (PPP) solution
    InsPpp = 74,

    /// Converging TerraStar‑L solution
    PppBasicConverging = 77,

    /// Converged TerraStar‑L solution
    PppBasic = 78,

    /// INS position, where the last applied position update used a converging TerraStar‑L PPP (PPP_BASIC) solution
    InsPppBasicConverging = 79,

    /// INS position, where the last applied position update used a converged TerraStar‑L PPP (PPP_BASIC) solution
    InsPppBasic = 80,
}

impl PositionOrVelocityType {
    /// Attempts to convert a u32 value into a `PositionVelocityType`.
    pub fn from_u32(value: u32) -> Option<Self> {
        match value {
            0 => Some(Self::No),
            1 => Some(Self::FixedPos),
            2 => Some(Self::FixedHeight),
            8 => Some(Self::DopplerVelocity),
            16 => Some(Self::Single),
            17 => Some(Self::PsrDiff),
            18 => Some(Self::Waas),
            19 => Some(Self::Propagated),
            32 => Some(Self::L1Float),
            34 => Some(Self::NarrowFloat),
            48 => Some(Self::L1Int),
            49 => Some(Self::WideInt),
            50 => Some(Self::NarrowInt),
            52 => Some(Self::InsSbas),
            53 => Some(Self::InsPsrSp),
            54 => Some(Self::InsPsrDiff),
            55 => Some(Self::InsRtkFloat),
            56 => Some(Self::InsRtkFixed),
            67 => Some(Self::ExtConstrained),
            68 => Some(Self::PppConverging),
            69 => Some(Self::Ppp),
            70 => Some(Self::Operational),
            71 => Some(Self::Warning),
            72 => Some(Self::OutOfBounds),
            73 => Some(Self::InsPppConverging),
            74 => Some(Self::InsPpp),
            77 => Some(Self::PppBasicConverging),
            78 => Some(Self::PppBasic),
            79 => Some(Self::InsPppBasicConverging),
            80 => Some(Self::InsPppBasic),
            _ => None, // unknown or reserved values
        }
    }
}

bitflags! {

    /// Extended solution status flags from the INSATTX log, indicating
    /// which updates and conditions have been used or are active.
    /// [1] https://docs.ros.org/en/humble/p/novatel_oem7_msgs/msg/INSExtendedSolutionStatus.html
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct InsExtendedSolutionStatus: u32 {
        /// Position update used (bit 0).
        const POSITION_UPDATE = 0x0000_0001;

        /// Phase update used (bit 1).
        const PHASE_UPDATE = 0x0000_0002;

        /// Zero velocity update used (bit 2).
        const ZERO_VELOCITY_UPDATE = 0x0000_0004;

        /// Wheel sensor update used (bit 3).
        const WHEEL_SENSOR_UPDATE = 0x0000_0008;

        /// ALIGN (heading) update used (bit 4).
        const ALIGN_UPDATE = 0x0000_0010;

        /// External position update used (bit 5).
        const EXTERNAL_POSITION_UPDATE = 0x0000_0020;

        /// INS solution convergence flag (bit 6): 1 = Converged.
        const INS_SOLUTION_CONVERGENCE_FLAG = 0x0000_0040;

        /// Doppler update used (bit 7).
        const DOPPLER_UPDATE = 0x0000_0080;

        /// Pseudorange update used (bit 8).
        const PSEUDORANGE_UPDATE = 0x0000_0100;

        /// Velocity update used (bit 9).
        const VELOCITY_UPDATE = 0x0000_0200;

        /// Dead reckoning update used (bit 11).
        const DEAD_RECKONING_UPDATE = 0x0000_0800;

        /// Phase wind-up update used (bit 12).
        const PHASE_WIND_UP_UPDATE = 0x0000_1000;

        /// Course over ground update used (bit 13).
        const COURSE_OVER_GROUND_UPDATE = 0x0000_2000;

        /// External velocity update used (bit 14).
        const EXTERNAL_VELOCITY_UPDATE = 0x0000_4000;

        /// External attitude update used (bit 15).
        const EXTERNAL_ATTITUDE_UPDATE = 0x0000_8000;

        /// External heading update used (bit 16).
        const EXTERNAL_HEADING_UPDATE = 0x0001_0000;

        /// External height update used (bit 17).
        const EXTERNAL_HEIGHT_UPDATE = 0x0002_0000;

        /// Secondary INS solution used (bit 22).
        const SECONDARY_INS_SOLUTION = 0x0040_0000;

        /// Turn-on biases estimated (bit 24): Static turn-on biases have been estimated.
        const TURN_ON_BIASES_ESTIMATED = 0x0100_0000;

        /// Alignment direction verified (bit 25): Alignment direction has been verified.
        const ALIGNMENT_DIRECTION_VERIFIED = 0x0200_0000;

        /// Alignment Indication 1 set (bit 26).
        const ALIGNMENT_INDICATION_1 = 0x0400_0000;

        /// Alignment Indication 2 set (bit 27).
        const ALIGNMENT_INDICATION_2 = 0x0800_0000;

        /// Alignment Indication 3 set (bit 28).
        const ALIGNMENT_INDICATION_3 = 0x1000_0000;

        /// NVM Seed Indication 1 set (bit 29).
        const NVM_SEED_INDICATION_1 = 0x2000_0000;

        /// NVM Seed Indication 2 set (bit 30).
        const NVM_SEED_INDICATION_2 = 0x4000_0000;

        /// NVM Seed Indication 3 set (bit 31).
        const NVM_SEED_INDICATION_3 = 0x8000_0000;
    }
}

/// Helper function to manually read f64 without automatic alignment
///
/// This function provides optimized f64 reading with proper error handling
/// and bounds checking for better performance and safety.
fn read_f64_manual(deserializer: &mut CdrDeserializer) -> rosbags_rs::error::Result<f64> {
    let position = deserializer.position();
    let data_len = deserializer.data_len();

    // Check bounds before reading to provide better error messages
    if !deserializer.has_remaining(8) {
        return Err(rosbags_rs::error::ReaderError::cdr_deserialization(
            "Not enough data for f64",
            position,
            data_len,
        ));
    }

    // Read 8 bytes for f64 with optimized error handling
    let mut bytes = [0u8; 8];

    for (i, byte) in bytes.iter_mut().enumerate().take(8) {
        *byte = deserializer.read_u8().map_err(|e| {
            rosbags_rs::error::ReaderError::cdr_deserialization(
                format!("Failed to read byte {} of f64: {}", i, e),
                position + i,
                data_len,
            )
        })?;
    }

    Ok(f64::from_le_bytes(bytes))
}

impl FromCdr for InsPvaX {
    fn from_cdr(deserializer: &mut CdrDeserializer) -> rosbags_rs::error::Result<Self> {
        let header = Header::from_cdr(deserializer)?;

        let nov_header = Oem7Header::from_cdr(deserializer)?;
        let ins_status = InertialSolutionStatus::from_cdr(deserializer)?;
        let pos_type = PositionOrVelocityType::from_cdr(deserializer)?;

        // For some reason, there seems to be 4 bytes of padding between these
        // two fields.
        // The deserializer is at byte 56 before reading the latitude, which is
        // 8-byte aligned.
        // The actual data for the latitude f64 starts at byte 60, which is _not_
        // 8-byte aligned.
        // I am not sure why there is 4 bytes of padding.

        for _ in 0..4 {
            deserializer.read_u8()?;
        }

        let latitude = read_f64_manual(deserializer)?;
        let longitude = read_f64_manual(deserializer)?;
        let height = read_f64_manual(deserializer)?;
        let undulation = deserializer.read_f32()?;

        // Another four bytes of padding.
        for _ in 0..4 {
            deserializer.read_u8()?;
        }

        let north_velocity = read_f64_manual(deserializer)?;
        let east_velocity = read_f64_manual(deserializer)?;
        let up_velocity = read_f64_manual(deserializer)?;

        let roll = read_f64_manual(deserializer)?;
        let pitch = read_f64_manual(deserializer)?;
        let azimuth = read_f64_manual(deserializer)?;

        let latitude_stdev = deserializer.read_f32()?;
        let longitude_stdev = deserializer.read_f32()?;
        let height_stdev = deserializer.read_f32()?;

        let north_velocity_stdev = deserializer.read_f32()?;
        let east_velocity_stdev = deserializer.read_f32()?;
        let up_velocity_stdev = deserializer.read_f32()?;

        let roll_stdev = deserializer.read_f32()?;
        let pitch_stdev = deserializer.read_f32()?;
        let azimuth_stdev = deserializer.read_f32()?;

        let ext_sol_status = InsExtendedSolutionStatus::from_cdr(deserializer)?;

        let time_since_update = deserializer.read_u16()?;

        Ok(Self {
            header,
            nov_header,
            ins_status,
            pos_type,
            latitude,
            longitude,
            height,
            undulation,
            north_velocity,
            east_velocity,
            up_velocity,
            roll,
            pitch,
            azimuth,
            latitude_stdev,
            longitude_stdev,
            height_stdev,
            north_velocity_stdev,
            east_velocity_stdev,
            up_velocity_stdev,
            roll_stdev,
            pitch_stdev,
            azimuth_stdev,
            ext_sol_status,
            time_since_update,
        })
    }
}

impl FromCdr for Oem7Header {
    fn from_cdr(deserializer: &mut CdrDeserializer) -> rosbags_rs::error::Result<Self> {
        let message_name = deserializer.read_string()?;
        let message_id = deserializer.read_u16()?;
        let message_type = deserializer.read_u8()?;
        let sequence_number = deserializer.read_u32()?;
        let time_status = deserializer.read_u8()?;
        let gps_week_number = deserializer.read_u16()?;
        let gps_week_milliseconds = deserializer.read_u32()?;

        Ok(Self {
            message_name,
            message_id,
            message_type,
            sequence_number,
            time_status,
            gps_week_number,
            gps_week_milliseconds,
        })
    }
}

impl FromCdr for InertialSolutionStatus {
    fn from_cdr(deserializer: &mut CdrDeserializer) -> rosbags_rs::error::Result<Self> {
        Ok(InertialSolutionStatus::from_u32(deserializer.read_u32()?).unwrap())
    }
}

impl FromCdr for PositionOrVelocityType {
    fn from_cdr(deserializer: &mut CdrDeserializer) -> rosbags_rs::error::Result<Self> {
        Ok(PositionOrVelocityType::from_u32(deserializer.read_u32()?).unwrap())
    }
}

impl FromCdr for InsExtendedSolutionStatus {
    fn from_cdr(deserializer: &mut CdrDeserializer) -> rosbags_rs::error::Result<Self> {
        Ok(InsExtendedSolutionStatus::from_bits(deserializer.read_u32()?).unwrap())
    }
}
