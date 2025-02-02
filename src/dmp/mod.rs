// Guide:
// https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary/blob/main/DMP.md#how-is-the-dmp-loaded-and-started

use bitfield::bitfield;
use embedded_hal_async::i2c::Operation;
use nalgebra::Quaternion;

use crate::*;

mod blob;
use blob::DMP_FIRWARE_BLOB;

const MAX_SERIAL_WRITE: usize = 16;
const DMP_LOAD_START: u8 = 0x90;
const DMP_START_ADDR: u16 = 0x1000;
const DMP_MOTION_EVENT_CONTROL_ACCEL_CALIBR: u16 = 0x0200;
const DMP_MOTION_EVENT_CONTROL_GYRO_CALIBR: u16 = 0x0100;
const DMP_MOTION_EVENT_CONTROL_COMPASS_CALIBR: u16 = 0x0080;
const DMP_DATA_OUT_CTL: u16 = 4 * 16;
const DMP_DATA_OUT_CTL2: u16 = 4 * 16 + 2;
const DMP_MOTION_EVENT_CTL: u16 = 4 * 16 + 14;
const DATA_RDY_STATUS: u16 = 8 * 16 + 10; // 16-bit: indicates to DMP which sensors are available

const INV_ANDROID_SENSOR_TO_CONTROL_BITS: &[u16] = &[
    // Data output control 1 register bit definition
    // 16-bit accel                                0x8000
    // 16-bit gyro                                 0x4000
    // 16-bit compass                              0x2000
    // 16-bit ALS                                  0x1000
    // 32-bit 6-axis quaternion                    0x0800
    // 32-bit 9-axis quaternion + heading accuracy 0x0400
    // 16-bit pedometer quaternion                 0x0200
    // 32-bit Geomag rv + heading accuracy         0x0100
    // 16-bit Pressure                             0x0080
    // 32-bit calibrated gyro                      0x0040
    // 32-bit calibrated compass                   0x0020
    // Pedometer Step Detector                     0x0010
    // Header 2                                    0x0008
    // Pedometer Step Indicator Bit 2              0x0004
    // Pedometer Step Indicator Bit 1              0x0002
    // Pedometer Step Indicator Bit 0              0x0001
    // Unsupported Sensors are 0xFFFF
    0xFFFF, // 0  Meta Data
    0x8008, // 1  Accelerometer
    0x0028, // 2  Magnetic Field
    0x0408, // 3  Orientation
    0x4048, // 4  Gyroscope
    0x1008, // 5  Light
    0x0088, // 6  Pressure
    0xFFFF, // 7  Temperature
    0xFFFF, // 8  Proximity <----------- fixme
    0x0808, // 9  Gravity
    0x8808, // 10 Linear Acceleration
    0x0408, // 11 Rotation Vector
    0xFFFF, // 12 Humidity
    0xFFFF, // 13 Ambient Temperature
    0x2008, // 14 Magnetic Field Uncalibrated
    0x0808, // 15 Game Rotation Vector
    0x4008, // 16 Gyroscope Uncalibrated
    0x0000, // 17 Significant Motion
    0x0018, // 18 Step Detector
    0x0010, // 19 Step Counter <----------- fixme
    0x0108, // 20 Geomagnetic Rotation Vector
    0xFFFF, // 21 ANDROID_SENSOR_HEART_RATE,
    0xFFFF, // 22 ANDROID_SENSOR_PROXIMITY,
    0x8008, // 23 ANDROID_SENSOR_WAKEUP_ACCELEROMETER,
    0x0028, // 24 ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD,
    0x0408, // 25 ANDROID_SENSOR_WAKEUP_ORIENTATION,
    0x4048, // 26 ANDROID_SENSOR_WAKEUP_GYROSCOPE,
    0x1008, // 27 ANDROID_SENSOR_WAKEUP_LIGHT,
    0x0088, // 28 ANDROID_SENSOR_WAKEUP_PRESSURE,
    0x0808, // 29 ANDROID_SENSOR_WAKEUP_GRAVITY,
    0x8808, // 30 ANDROID_SENSOR_WAKEUP_LINEAR_ACCELERATION,
    0x0408, // 31 ANDROID_SENSOR_WAKEUP_ROTATION_VECTOR,
    0xFFFF, // 32 ANDROID_SENSOR_WAKEUP_RELATIVE_HUMIDITY,
    0xFFFF, // 33 ANDROID_SENSOR_WAKEUP_AMBIENT_TEMPERATURE,
    0x2008, // 34 ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED,
    0x0808, // 35 ANDROID_SENSOR_WAKEUP_GAME_ROTATION_VECTOR,
    0x4008, // 36 ANDROID_SENSOR_WAKEUP_GYROSCOPE_UNCALIBRATED,
    0x0018, // 37 ANDROID_SENSOR_WAKEUP_STEP_DETECTOR,
    0x0010, // 38 ANDROID_SENSOR_WAKEUP_STEP_COUNTER,
    0x0108, // 39 ANDROID_SENSOR_WAKEUP_GEOMAGNETIC_ROTATION_VECTOR
    0xFFFF, // 40 ANDROID_SENSOR_WAKEUP_HEART_RATE,
    0x0000, // 41 ANDROID_SENSOR_WAKEUP_TILT_DETECTOR,
    0x8008, // 42 Raw Acc
    0x4048, // 43 Raw Gyr
];

#[repr(u8)]
pub enum DmpSensor {
    Orientation = 18,
}

impl DmpSensor {
    fn to_android_sensor(&self) -> u8 {
        match self {
            DmpSensor::Orientation => 3,
        }
    }
}

enum Peripheral {
    Zero,
    One,
}

struct PeripheralRegGroup {
    addr: Bank3,
    reg: Bank3,
    ctrl: Bank3,
    r#do: Bank3,
}

impl Peripheral {
    fn regs(&self) -> PeripheralRegGroup {
        match self {
            Peripheral::Zero => PeripheralRegGroup {
                addr: Bank3::I2cSlv0Addr,
                reg: Bank3::I2cSlv0Reg,
                ctrl: Bank3::I2cSlv0Ctrl,
                r#do: Bank3::I2cSlv0Do,
            },
            Peripheral::One => PeripheralRegGroup {
                addr: Bank3::I2cSlv1Addr,
                reg: Bank3::I2cSlv1Reg,
                ctrl: Bank3::I2cSlv1Ctrl,
                r#do: Bank3::I2cSlv1Do,
            },
        }
    }
}

impl<BUS, DELAY, E> Icm20948<IcmBusI2c<BUS>, MagDisabled, NotInit, DELAY, E>
where
    BUS: I2c<Error = E>,
    E: Into<IcmError<E>>,
    DELAY: DelayNs,
{
    // ICM_20948_i2c_controller_configure_peripheral in sparkfun
    async fn configure_i2c(
        &mut self,
        peripheral: Peripheral,
        addr: u8,
        reg: u8,
        len: u8,
        rnw: bool,
        enable: bool,
        data_only: bool,
        grp: bool,
        swap: bool,
        data_out: u8,
    ) -> Result<(), E> {
        let regs = peripheral.regs();
        /*
        {
          uint8_t ID : 7;
          uint8_t RNW : 1;
        } ICM_20948_I2C_PERIPHX_ADDR_t;
         */
        let periphx_addr_val: u8 = (addr & !0x80) | ((rnw as u8) << 7);

        self.write_to(regs.addr, periphx_addr_val).await?;

        if !rnw {
            self.write_to(regs.r#do, data_out).await?;
        }

        self.write_to(regs.reg, reg).await?;
        /*
        {
          uint8_t LENG : 4;
          uint8_t GRP : 1;
          uint8_t REG_DIS : 1;
          uint8_t BYTE_SW : 1;
          uint8_t EN : 1;
        } ICM_20948_I2C_PERIPHX_CTRL_t;
        */
        let mut periphx_ctrl = PeriphxCtrl(0);
        periphx_ctrl.set_leng(len);
        periphx_ctrl.set_grp(grp);
        periphx_ctrl.set_reg_dis(data_only);
        periphx_ctrl.set_byte_sw(swap);
        periphx_ctrl.set_en(enable);
        self.write_to(regs.ctrl, periphx_ctrl.0).await?;

        Ok(())
    }
    pub async fn load_dmp(&mut self) -> Result<(), E> {
        self.configure_i2c(
            Peripheral::Zero,
            MAGNET_ADDR,
            MagBank::Rsv2.reg(),
            10,
            true,
            true,
            false,
            true,
            true,
            0,
        )
        .await?;

        self.configure_i2c(
            Peripheral::One,
            MAGNET_ADDR,
            MagBank::Control2.reg(),
            1,
            false,
            true,
            false,
            false,
            false,
            0x01, // AK09916_mode_single
        )
        .await?;

        self.write_to(Bank3::I2cMstOdrConfig, 0x04).await?;

        // set clock source auto
        let [pwr_mgmt] = self.read_from(Bank0::PwrMgmt1).await?;
        let mut pwr_mgmt = PwrMgmt1(pwr_mgmt);
        pwr_mgmt.set_clk_sel(0x01);
        self.write_to(Bank0::PwrMgmt1, pwr_mgmt.0).await?;

        self.write_to(Bank0::PwrMgmt2, 0x40).await?;

        // Set i2c master to cycled sample mode
        let [lp_config] = self.read_from(Bank0::LpConfig).await?;
        let mut lp_config = LpConfigVal(lp_config);
        lp_config.set_i2c_mst(true);
        self.write_to(Bank0::LpConfig, lp_config.0).await?;
        self.delay.delay_ms(1).await;

        self.set_fifo(false).await?;
        self.set_dmp(false).await?;

        self.set_full_scale(true, true).await?;
        // gyro DLPF
        self.enable_gyro_fchoice().await?;

        self.write_to(Bank0::FifoEn1, 0).await?;
        self.write_to(Bank0::FifoEn2, 0).await?;

        self.write_to(Bank0::IntEnable1, 0).await?;
        self.write_to(Bank0::FifoRst, 1).await?;

        self.set_sample_rate(&SampleRateConfig {
            accel: 19,
            gyro: 19,
        })
        .await?;

        self.write_two(
            Bank2::PrgmStartAddr,
            [(DMP_START_ADDR >> 8) as u8, (DMP_START_ADDR & 0xFF) as u8],
        )
        .await?;

        self.load_dmp_firmware().await?;
        self.verify_dmp_firmware().await?;

        self.write_two(
            Bank2::PrgmStartAddr,
            [(DMP_START_ADDR >> 8) as u8, (DMP_START_ADDR & 0xFF) as u8],
        )
        .await?;

        self.write_to(Bank0::HwFixDisable, 0x48).await?;

        self.write_to(Bank0::SingleFifoPrioritySel, 0xE4).await?;

        const ACCEL_SCALE: u16 = 30 * 16;
        self.write_mems(ACCEL_SCALE, &[0x04, 0x00, 0x00, 0x00])
            .await?;

        const ACCEL_SCALE2: u16 = 79 * 16 + 4;
        self.write_mems(ACCEL_SCALE2, &[0x00, 0x04, 0x00, 0x00])
            .await?;

        const CPASS_MTX_00: u16 = 23 * 16;
        let mount_mult_zero = &[0u8, 0, 0, 0];
        let mount_mult_plus = &[0x09, 0x99, 0x99, 0x99];
        let mount_mult_minus = &[0xf6, 0x66, 0x66, 0x67];
        self.write_mems(CPASS_MTX_00, mount_mult_plus).await?;
        self.write_mems(CPASS_MTX_00 + 4, mount_mult_zero).await?;
        self.write_mems(CPASS_MTX_00 + 8, mount_mult_zero).await?;
        self.write_mems(CPASS_MTX_00 + 12, mount_mult_zero).await?;
        self.write_mems(CPASS_MTX_00 + 16, mount_mult_minus).await?;
        self.write_mems(CPASS_MTX_00 + 20, mount_mult_zero).await?;
        self.write_mems(CPASS_MTX_00 + 24, mount_mult_zero).await?;
        self.write_mems(CPASS_MTX_00 + 28, mount_mult_zero).await?;
        self.write_mems(CPASS_MTX_00 + 32, mount_mult_minus).await?;

        const B2S_MTX_00: u16 = 208 * 16;
        let b2s_mount_mult_zero = &[0x00, 0x00, 0x00, 0x00];
        let b2s_mount_mult_plus = &[0x40, 0x00, 0x00, 0x00];
        self.write_mems(B2S_MTX_00, b2s_mount_mult_plus).await?;
        self.write_mems(B2S_MTX_00 + 4, b2s_mount_mult_zero).await?;
        self.write_mems(B2S_MTX_00 + 8, b2s_mount_mult_zero).await?;
        self.write_mems(B2S_MTX_00 + 12, b2s_mount_mult_zero)
            .await?;
        self.write_mems(B2S_MTX_00 + 16, b2s_mount_mult_plus)
            .await?;
        self.write_mems(B2S_MTX_00 + 20, b2s_mount_mult_zero)
            .await?;
        self.write_mems(B2S_MTX_00 + 24, b2s_mount_mult_zero)
            .await?;
        self.write_mems(B2S_MTX_00 + 28, b2s_mount_mult_zero)
            .await?;
        self.write_mems(B2S_MTX_00 + 32, b2s_mount_mult_plus)
            .await?;

        // Configure the DMP Gyro Scaling Factor
        let [pll] = self.read_from(Bank1::TimebaseCorrectionPll).await?;
        const MAGIC_CONSTANT: u64 = 264446880937391;
        const MAGIC_CONSTANT_SCALE: u64 = 100000;
        const GYRO_LEVEL: u64 = 4;
        let result_ll: u64;
        let div = 19u64;
        if pll & 0x80 != 0x00 {
            result_ll = MAGIC_CONSTANT * (1u64 << GYRO_LEVEL) * (1 + div)
                / (1270 - ((pll as u64) & 0x7Fu64))
                / MAGIC_CONSTANT_SCALE;
        } else {
            result_ll = MAGIC_CONSTANT * (1u64 << GYRO_LEVEL) * (1 + div)
                / (1270 + pll as u64)
                / MAGIC_CONSTANT_SCALE;
        }
        let gyro_sf = result_ll.min(0x7FFFFFFF) as u32;
        const GYRO_SF: u16 = 19 * 16;
        self.write_mems(GYRO_SF, &gyro_sf.to_be_bytes()).await?;

        const GYRO_FULLSCALE: u16 = 72 * 16 + 12;
        self.write_mems(GYRO_FULLSCALE, &[0x10, 0x00, 0x00, 0x00])
            .await?;

        const ACCEL_ONLY_GAIN: u16 = 16 * 16 + 12;
        self.write_mems(ACCEL_ONLY_GAIN, &[0x03, 0xa4, 0x92, 0x49])
            .await?;

        const ACCEL_ALPHA_VAR: u16 = 91 * 16;
        self.write_mems(ACCEL_ALPHA_VAR, &[0x34, 0x92, 0x49, 0x25])
            .await?;

        const ACCEL_A_VAR: u16 = 92 * 16;
        self.write_mems(ACCEL_A_VAR, &[0x0B, 0x6D, 0xB6, 0xDB])
            .await?;

        const ACCEL_CAL_RATE: u16 = 94 * 16 + 4;
        self.write_mems(ACCEL_CAL_RATE, &[0x00, 0x00]).await?;

        const CPASS_TIME_BUFFER: u16 = 112 * 16 + 14;
        self.write_mems(CPASS_TIME_BUFFER, &[0x00, 0x45]).await?;

        Ok(())
    }

    pub async fn enable_dmp_sensor(&mut self, sensor: DmpSensor) -> Result<(), crate::IcmError<E>> {
        self.set_low_power(false).await?;
        let android_sensor = sensor.to_android_sensor();
        let delta: u16 = INV_ANDROID_SENSOR_TO_CONTROL_BITS
            .get(android_sensor as usize)
            .cloned()
            .ok_or(IcmError::DmpSetupError)?;

        // assume all sensors needed for now
        const DMP_MOTION_EVENT_CONTROL_9AXIS: u16 = 0x0040;

        // ACTUALLY we assume just quat9 + header2 for now (that's delta for Orientation)

        let event_control: u16 = DMP_MOTION_EVENT_CONTROL_ACCEL_CALIBR
            | DMP_MOTION_EVENT_CONTROL_GYRO_CALIBR
            | DMP_MOTION_EVENT_CONTROL_COMPASS_CALIBR
            | DMP_MOTION_EVENT_CONTROL_9AXIS;
        const DMP_DATA_READY_ACCEL: u16 = 0x0002;
        const DMP_DATA_READY_GYRO: u16 = 0x0001;
        const DMP_DATA_READY_SECONDARY_COMPASS: u16 = 0x0008;
        let data_rdy_status: u16 =
            DMP_DATA_READY_ACCEL | DMP_DATA_READY_GYRO | DMP_DATA_READY_SECONDARY_COMPASS;
        let data_output_control = delta;
        const DMP_DATA_OUTPUT_CONTROL_2_ACCEL_ACCURACY: u16 = 0x4000;
        const DMP_DATA_OUTPUT_CONTROL_2_GYRO_ACCURACY: u16 = 0x2000;
        const DMP_DATA_OUTPUT_CONTROL_2_COMPASS_ACCURACY: u16 = 0x1000;

        let data_output_control_2 = 
        // DMP_DATA_OUTPUT_CONTROL_2_GYRO_ACCURACY
        //     | DMP_DATA_OUTPUT_CONTROL_2_ACCEL_ACCURACY
            DMP_DATA_OUTPUT_CONTROL_2_COMPASS_ACCURACY;
        self.write_mems(DMP_DATA_OUT_CTL, &data_output_control.to_be_bytes())
            .await?;
        self.write_mems(DMP_DATA_OUT_CTL2, &data_output_control_2.to_be_bytes())
            .await?;
        self.write_mems(DATA_RDY_STATUS, &data_rdy_status.to_be_bytes())
            .await?;
        self.write_mems(DMP_MOTION_EVENT_CTL, &event_control.to_be_bytes())
            .await?;

        self.set_low_power(true).await?;
        Ok(())
    }

    pub async fn set_dmp_odr(&mut self) -> Result<(), E> {
        self.set_sleep(false).await?;
        self.set_low_power(false).await?;
        const ODR_QUAT9: u16 = 10 * 16 + 8;
        const ODR_CNTR_QUAT9: u16 = 8 * 16 + 8;
        // Can be changed based on this equation:
        // https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary/blob/9a10c510ddb694f08aa93c12d586358cb45abd2b/src/util/ICM_20948_C.c#L1621.
        self.write_mems(ODR_QUAT9, &[0, 0]).await?;
        self.write_mems(ODR_CNTR_QUAT9, &[0, 0]).await?;
        self.set_low_power(true).await?;

        Ok(())
    }

    async fn set_fifo(&mut self, enable: bool) -> Result<(), E> {
        let [user_ctrl] = self.read_from(Bank0::UserCtrl).await?;
        let mut user_ctrl = UserCtrlVal(user_ctrl);
        user_ctrl.set_fifo_en(enable);
        self.write_to(Bank0::UserCtrl, user_ctrl.0).await?;
        Ok(())
    }
    async fn set_dmp(&mut self, enable: bool) -> Result<(), E> {
        let [user_ctrl] = self.read_from(Bank0::UserCtrl).await?;
        let mut user_ctrl = UserCtrlVal(user_ctrl);
        user_ctrl.set_dmp_en(enable);
        self.write_to(Bank0::UserCtrl, user_ctrl.0).await?;
        Ok(())
    }

    async fn reset_dmp(&mut self) -> Result<(), E> {
        let [user_ctrl] = self.read_from(Bank0::UserCtrl).await?;
        let mut user_ctrl = UserCtrlVal(user_ctrl);
        user_ctrl.set_dmp_rst(true);
        self.write_to(Bank0::UserCtrl, user_ctrl.0).await?;
        Ok(())
    }

    async fn reset_fifo(&mut self) -> Result<(), E> {
        let [mut fifo_rst] = self.read_from(Bank0::FifoRst).await?;
        fifo_rst |= 0x1F;
        self.write_to(Bank0::FifoRst, fifo_rst).await?;
        fifo_rst = (fifo_rst & !0x1F) | 0x1E;
        self.write_to(Bank0::FifoRst, fifo_rst).await?;
        Ok(())
    }

    async fn set_full_scale(&mut self, accel: bool, gyro: bool) -> Result<(), E> {
        if accel {
            let [acc_config] = self.read_from(Bank2::AccelConfig).await?;
            let mut acc_config = AccelConfig(acc_config);
            acc_config.set_fs_sel(AccelConfigFsSel::Gpm4 as u8);
            self.write_to(Bank2::AccelConfig, acc_config.0).await?;
        }
        if gyro {
            let [gyr_config] = self.read_from(Bank2::GyroConfig1).await?;
            let mut gyr_config = GyroConfig(gyr_config);
            gyr_config.set_fs_sel(GyroConfigFsSel::Dps2000 as u8);
            self.write_to(Bank2::GyroConfig1, gyr_config.0).await?;
        }
        Ok(())
    }
    async fn enable_gyro_fchoice(&mut self) -> Result<(), E> {
        let [gyr_config] = self.read_from(Bank2::GyroConfig1).await?;
        let mut gyr_config = GyroConfig(gyr_config);
        gyr_config.set_fchoice(true);
        self.write_to(Bank2::GyroConfig1, gyr_config.0).await?;
        Ok(())
    }

    async fn set_sample_rate(&mut self, sr: &SampleRateConfig) -> Result<(), E> {
        // https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary/blob/9a10c510ddb694f08aa93c12d586358cb45abd2b/src/util/ICM_20948_C.c#L832
        self.write_to(Bank2::AccelSmplrtDiv1, (sr.accel >> 8) as u8)
            .await?;
        self.write_to(Bank2::AccelSmplrtDiv2, (sr.accel & 0xff) as u8)
            .await?;
        self.write_to(Bank2::GyroSmplrtDiv, sr.gyro).await?;
        Ok(())
    }

    async fn load_dmp_firmware(&mut self) -> Result<(), E> {
        self.set_low_power(false).await?;

        let data = DMP_FIRWARE_BLOB;
        let mut write_start = data;
        let mut write_addr = DMP_LOAD_START as u16;
        while write_start.len() > 0 {
            let mut write_size = write_start.len().min(MAX_SERIAL_WRITE);
            if (write_addr & 0xff) + (write_size as u16) > 0x100 {
                write_size = ((write_addr & 0xff) + (write_size as u16) - 0x100) as usize;
                // write_size = (0x100 - (write_addr & 0xff)) as usize;
            }
            if write_size == 0 {
                panic!("write size zero");
            }
            self.write_mems(write_addr, &write_start[..write_size])
                .await?;
            write_start = &write_start[write_size..];
            write_addr += write_size as u16;
        }

        self.set_low_power(true).await?;

        Ok(())
    }

    async fn verify_dmp_firmware(&mut self) -> Result<(), E> {
        let mut expected_data = DMP_FIRWARE_BLOB;
        let mut read_loc = DMP_LOAD_START as usize;

        while !expected_data.is_empty() {
            let mut read_len = expected_data.len().min(MAX_SERIAL_WRITE);
            if (read_loc & 0xff) + read_len > 0x100 {
                // Moved across a bank
                read_len = (read_loc & 0xff) + read_len - 0x100;
            }
            let mut buf = &mut [0u8; MAX_SERIAL_WRITE][..read_len];
            self.read_mems(read_loc as u16, &mut buf).await?;
            if buf != &expected_data[..read_len] {
                panic!("OHNO");
            }
            expected_data = &expected_data[read_len..];
            read_loc += read_len;
        }
        // panic!("DONE :) ");

        Ok(())
    }

    async fn write_mems(&mut self, mut reg: u16, data: &[u8]) -> Result<(), E> {
        self.write_to(Bank0::MemBankSel, (reg >> 8) as u8).await?;

        let mut bytes_written = 0;
        while bytes_written < data.len() {
            let start_addr = (reg & 0xff) as u8;
            self.write_to(Bank0::MemStartAddr, start_addr).await?;

            let to_write = (data.len() - bytes_written).min(MAX_SERIAL_WRITE);
            self.set_user_bank(&Bank0::MemRW, false).await?;
            // Pack register and data in to buf.
            let mut buf = [0u8; MAX_SERIAL_WRITE + 1];
            buf[0] = Bank0::MemRW.reg();
            buf[1..to_write + 1].copy_from_slice(&data[bytes_written..bytes_written + to_write]);
            self.bus.bus_write(&buf[..to_write + 1]).await?;
            bytes_written += to_write;
            if let Some(next) = reg.checked_add(to_write as u16) {
                reg = next;
            } else {
                panic!("{}", bytes_written);
            }
            // NOTE: this loop is currently only ever executed once since the caller limits writes to MAX_SERIAL_WRITE
            reg += to_write as u16;
        }
        Ok(())
    }

    async fn read_mems(&mut self, mut reg: u16, mut buf: &mut [u8]) -> Result<(), E> {
        self.write_to(Bank0::MemBankSel, (reg >> 8) as u8).await?;

        while !buf.is_empty() {
            self.write_to(Bank0::MemStartAddr, (reg & 0xff) as u8)
                .await?;
            let read_len = buf.len().min(MAX_SERIAL_WRITE) as usize;
            self.read_slice_from(Bank0::MemRW, &mut buf[..read_len])
                .await?;
            buf = &mut buf[read_len..];
            reg += read_len as u16;
        }

        Ok(())
    }

    async fn set_low_power(&mut self, enable: bool) -> Result<(), E> {
        let [pwr_mgmt] = self.read_from(Bank0::PwrMgmt1).await?;
        let mut pwr_mgmt = PwrMgmt1(pwr_mgmt);
        pwr_mgmt.set_lp_en(enable);
        self.write_to(Bank0::PwrMgmt1, pwr_mgmt.0).await?;
        Ok(())
    }

    async fn set_sleep(&mut self, enable: bool) -> Result<(), E> {
        let [pwr_mgmt] = self.read_from(Bank0::PwrMgmt1).await?;
        let mut pwr_mgmt = PwrMgmt1(pwr_mgmt);
        pwr_mgmt.set_sleep(enable);
        self.write_to(Bank0::PwrMgmt1, pwr_mgmt.0).await?;
        Ok(())
    }
    async fn reset(&mut self) -> Result<(), E> {
        let [pwr_mgmt] = self.read_from(Bank0::PwrMgmt1).await?;
        let mut pwr_mgmt = PwrMgmt1(pwr_mgmt);
        pwr_mgmt.set_device_reset(true);
        self.write_to(Bank0::PwrMgmt1, pwr_mgmt.0).await?;
        Ok(())
    }

    pub async fn full_dmp_setup(&mut self) -> Result<(), IcmError<E>> {
        // self.reset().await?;
        // self.delay.delay_ms(50).await;
        self.set_sleep(false).await?;
        self.load_dmp().await?;
        self.enable_dmp_sensor(DmpSensor::Orientation).await?;
        self.set_dmp_odr().await?;
        self.set_fifo(true).await?;
        self.set_dmp(true).await?;
        self.reset_dmp().await?;
        self.reset_fifo().await?;
        Ok(())
    }
    pub async fn read_dmp(&mut self) -> Result<(Option<((f64, f64, f64), u16)>, Option<u16>), E> {
        const MAX_DMP_BYTES: usize = 14;
        const DMP_HEADER_BYTES: u16 = 2u16;
        const DMP_HEADER2_BYTES: u16 = 2u16;
        const DMP_HEADER_BITMAP_HEADER2: u16 = 0x0008;
        const DMP_HEADER_BITMAP_ACCEL: u16 = 0x8000;
        const DMP_HEADER_BITMAP_GYRO: u16 = 0x4000;
        const DMP_HEADER_BITMAP_COMPASS: u16 = 0x2000;
        const DMP_HEADER_BITMAP_ALS: u16 = 0x1000;
        const DMP_HEADER_BITMAP_QUAT6: u16 = 0x0800;
        const DMP_HEADER_BITMAP_QUAT9: u16 = 0x0400;
        const DMP_HEADER_BITMAP_PQUAT6: u16 = 0x0200;
        const DMP_HEADER_BITMAP_GEOMAG: u16 = 0x0100;
        const DMP_HEADER_BITMAP_PRESSURE: u16 = 0x0080;
        const DMP_HEADER_BITMAP_GYRO_CALIB: u16 = 0x0040;
        const DMP_HEADER_BITMAP_COMPASS_CALIB: u16 = 0x0020;
        const DMP_HEADER_BITMAP_STEP_DETECTOR: u16 = 0x0010;

        let mut out: Option<((f64, f64, f64), u16)> = None;

        let mut count = self.fifo_count().await?;
        if count < DMP_HEADER_BYTES {
            return Ok((None, None));
        }

        let [header_hi, header_lo] = self.fifo_read().await?;
        let header = u16::from_be_bytes([header_hi, header_lo]);
        count -= DMP_HEADER_BYTES;

        let header2 = if header & DMP_HEADER_BITMAP_HEADER2 > 0 {
            if count < DMP_HEADER2_BYTES {
                count = self.fifo_count().await?;
                if count < DMP_HEADER2_BYTES {
                    return Ok((None, Some(header)));
                }
            }
            let [header_hi, header_lo] = self.fifo_read().await?;
            let header2 = u16::from_be_bytes([header_hi, header_lo]);
            Some(header2)
        } else {
            None
        };
        // TODO: will these panic if fifo is not adequately filled?
        if header & DMP_HEADER_BITMAP_ACCEL > 0 {
            let _: [u8; 6] = self.fifo_read().await?;
        }
        if header & DMP_HEADER_BITMAP_GYRO > 0 {
            let _: [u8; 12] = self.fifo_read().await?;
        }
        if header & DMP_HEADER_BITMAP_COMPASS > 0 {
            let _: [u8; 6] = self.fifo_read().await?;
        }
        if header & DMP_HEADER_BITMAP_ALS > 0 {
            let _: [u8; 8] = self.fifo_read().await?;
        }
        if header & DMP_HEADER_BITMAP_QUAT6 > 0 {
            let _: [u8; 12] = self.fifo_read().await?;
        }
        if header & DMP_HEADER_BITMAP_QUAT9 > 0 {
            let quat9_bytes: [u8; 14] = self.fifo_read().await?;
            let q1 =
                u32::from_be_bytes(quat9_bytes[0..4].try_into().unwrap()) as f64 / 1073741824.0;
            let q2 =
                u32::from_be_bytes(quat9_bytes[4..8].try_into().unwrap()) as f64 / 1073741824.0;
            let q3 =
                u32::from_be_bytes(quat9_bytes[8..12].try_into().unwrap()) as f64 / 1073741824.0;
            use num_traits::Float as _;
            let q0: f64 = (1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3))).sqrt();

            let accuracy = u16::from_be_bytes(quat9_bytes[12..14].try_into().unwrap());
            let q = nalgebra::Quaternion::from_parts(q0, [q1, q2, q3].into());
            out = Some(((q1, q2, q3), accuracy))
        }

        if header & DMP_HEADER_BITMAP_PQUAT6 > 0 {
            let _: [u8; 6] = self.fifo_read().await?;
        }
        if header & DMP_HEADER_BITMAP_GEOMAG > 0 {
            let _: [u8; 14] = self.fifo_read().await?;
        }
        if header & DMP_HEADER_BITMAP_PRESSURE > 0 {
            let _: [u8; 6] = self.fifo_read().await?;
        }
        if header & DMP_HEADER_BITMAP_GYRO_CALIB > 0 {}
        if header & DMP_HEADER_BITMAP_COMPASS_CALIB > 0 {
            let _: [u8; 12] = self.fifo_read().await?;
        }
        if header & DMP_HEADER_BITMAP_STEP_DETECTOR > 0 {
            let _: [u8; 4] = self.fifo_read().await?;
        }

        if let Some(header2) = header2 {
            let skipped_header2_bits_and_lens: &[(u16, usize)] = &[
                (0x4000, 2),
                (0x2000, 2),
                (0x1000, 2),
                (0x0400, 2),
                (0x0080, 6),
                (0x0040, 2),
            ];

            for (skip_mask, skip_len) in skipped_header2_bits_and_lens {
                if header2 & skip_mask > 0 {
                    for _ in 0..*skip_len {
                        if (count as usize) < *skip_len {
                            count = self.fifo_count().await?;
                            if (count as usize) < *skip_len {
                                return Ok((None, Some(header)));
                            }
                        }
                        // TODO: is this slow?
                        let [_] = self.fifo_read().await?;
                    }
                }
            }
        }

        let _footer: [u8; 2] = self.fifo_read().await?;

        Ok((out, Some(header)))
    }

    pub async fn fifo_count(&mut self) -> Result<u16, E> {
        let [high, low] = self.read_from(Bank0::FifoCounth).await?;
        // let [low] = self.read_from(Bank0::FifoCountl).await?;
        Ok(((high as u16) << 8) + low as u16)
    }

    async fn fifo_read<const N: usize>(&mut self) -> Result<[u8; N], E> {
        self.read_from(Bank0::FifoRW).await
    }
}

bitfield! {
    /*
      {
    uint8_t LENG : 4;
    uint8_t GRP : 1;
    uint8_t REG_DIS : 1;
    uint8_t BYTE_SW : 1;
    uint8_t EN : 1;
  } ICM_20948_I2C_PERIPHX_CTRL_t;
    */
    struct PeriphxCtrl(u8);
    get_leng, set_leng: 3, 0;
    get_grp, set_grp: 4;
    get_reg_dis, set_reg_dis: 5;
    get_byte_sw, set_byte_sw: 6;
    get_en, set_en: 7;
}

bitfield! {
    /*
  {
    uint8_t reserved_0 : 4;
    uint8_t GYRO_CYCLE : 1;
    uint8_t ACCEL_CYCLE : 1;
    uint8_t I2C_MST_CYCLE : 1;
    uint8_t reserved_1 : 1;
  } ICM_20948_LP_CONFIG_t;
    */
    struct LpConfigVal(u8);
    _, set_gyro: 4;
    _, set_accel: 5;
    _, set_i2c_mst: 6;
}

bitfield! {
    /*
  {
    uint8_t reserved_0 : 1;
    uint8_t I2C_MST_RST : 1;
    uint8_t SRAM_RST : 1;
    uint8_t DMP_RST : 1;
    uint8_t I2C_IF_DIS : 1;
    uint8_t I2C_MST_EN : 1;
    uint8_t FIFO_EN : 1;
    uint8_t DMP_EN : 1;
  } ICM_20948_USER_CTRL_t;
    */
    struct UserCtrlVal(u8);
    _, set_i2c_mst_rst: 1;
    _, set_sram_rst: 2;
    _, set_dmp_rst: 3;
    _, set_i2c_if_dis: 4;
    _, set_i2c_mst_en: 5;
    _, set_fifo_en: 6;
    _, set_dmp_en: 7;
}

bitfield! {
    /* {
    uint8_t a : 2;
    uint8_t g : 2;
    uint8_t reserved_0 : 4;
  } ICM_20948_fss_t;
    */
    struct Fss(u8);
    _, set_a: 1, 0;
    _, set_g: 3, 2;
}

bitfield! {
    /*
     {
    uint8_t ACCEL_FCHOICE : 1;
    uint8_t ACCEL_FS_SEL : 2;
    uint8_t ACCEL_DLPFCFG : 3;
    uint8_t reserved_0 : 2;
  } ICM_20948_ACCEL_CONFIG_t;
    */
    struct AccelConfig(u8);
    _, set_fchoice: 0;
    _, set_fs_sel: 2, 1;
    _, set_dlpf_cfg: 5, 3;
}
bitfield! {
    /*
     {
    uint8_t GYRO_FCHOICE : 1;
    uint8_t GYRO_FS_SEL : 2;
    uint8_t GYRO_DLPFCFG : 3;
    uint8_t reserved_0 : 2;
  } ICM_20948_GYRO_CONFIG_1_t;
    */
    struct GyroConfig(u8);
    _, set_fchoice: 0;
    _, set_fs_sel: 2, 1;
    _, set_dlpf_cfg: 5, 3;
}

#[repr(u8)]
enum AccelConfigFsSel {
    Gpm2 = 0x0,
    Gpm4,
    Gpm8,
    Gpm16,
}

#[repr(u8)]
enum GyroConfigFsSel {
    Dps250 = 0x0,
    Dps500,
    Dps1000,
    Dps2000,
}

struct SampleRateConfig {
    accel: u16,
    gyro: u8,
}

bitfield! {
    /*
    {
        uint8_t CLKSEL : 3;
        uint8_t TEMP_DIS : 1;
        uint8_t reserved_0 : 1;
        uint8_t LP_EN : 1;
        uint8_t SLEEP : 1;
        uint8_t DEVICE_RESET : 1;
    } ICM_20948_PWR_MGMT_1_t;
    */
    struct PwrMgmt1(u8);
    _, set_clk_sel: 2, 0;
    _, set_lp_en: 5;
    _, set_sleep: 6;
    _, set_device_reset: 7;
}
