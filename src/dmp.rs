// Guide:
// https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary/blob/main/DMP.md#how-is-the-dmp-loaded-and-started

use bitfield::bitfield;
use embedded_hal_async::i2c::Operation;

use crate::*;

const FIRMWARE: &[u8] = include_bytes!("dmp.raw");
const MAX_SERIAL_WRITE: usize = 16;
const DMP_LOAD_START: u8 = 0x90;
const DMP_START_ADDR: u16 = 0x1000;

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

        self.set_fifo(false).await?;
        self.set_dmp(false).await?;

        self.set_full_scale(true, true).await?;
        self.enable_gyro_fchoice().await?;

        self.write_to(Bank0::FifoEn1, 0).await?;
        self.write_to(Bank0::FifoEn2, 0).await?;

        self.write_to(Bank0::IntEnable1, 0).await?;
        self.write_to(Bank0::FifoRst, 1).await?;

        self.set_sample_rate(&SampleRateConfig {
            accel: 16,
            gyro: 16,
        })
        .await?;

        let start_addr = DMP_START_ADDR;
        self.write_two(
            Bank2::PrgmStartAddr,
            [(start_addr >> 8) as u8, start_addr as u8],
        )
        .await?;

        self.load_dmp_firmware().await?;

        self.write_two(
            Bank2::PrgmStartAddr,
            [(DMP_START_ADDR >> 8) as u8, (DMP_START_ADDR & 0xFF) as u8],
        )
        .await?;

        self.write_to(Bank0::HwFixDisable, 0x48).await?;

        self.write_to(Bank0::SingleFifoPrioritySel, 0xE4).await?;

        

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
        // Disable lowpower
        let [pwr_mgmt] = self.read_from(Bank0::PwrMgmt1).await?;
        let mut pwr_mgmt = PwrMgmt1(pwr_mgmt);
        pwr_mgmt.set_lp_en(false);
        self.write_to(Bank0::PwrMgmt1, pwr_mgmt.0).await?;

        let data = FIRMWARE;
        let mut write_start = data;
        let mut write_addr = DMP_LOAD_START as u16;
        while write_start.len() > 0 {
            let mut write_size = write_start.len().min(MAX_SERIAL_WRITE);
            if (write_addr & 0xff) + (write_size as u16) > 0x100 {
                write_size = ((write_addr & 0xff) + (write_size as u16) - 0x100) as usize;
            }
            self.write_mems(write_addr, &write_start[..write_size])
                .await?;
            write_start = &write_start[write_size..];
            write_addr += write_size as u16;
        }

        // Enable lowpower
        let [pwr_mgmt] = self.read_from(Bank0::PwrMgmt1).await?;
        let mut pwr_mgmt = PwrMgmt1(pwr_mgmt);
        pwr_mgmt.set_lp_en(true);
        self.write_to(Bank0::PwrMgmt1, pwr_mgmt.0).await?;

        Ok(())
    }

    async fn write_mems(&mut self, mut reg: u16, data: &[u8]) -> Result<(), E> {
        self.write_to(Bank0::MemBankSel, (reg >> 8) as u8).await?;

        let mut bytes_written = 0;
        while bytes_written < data.len() {
            let start_addr = (reg & 0xff) as u8;
            self.write_to(Bank0::MemStartAddr, start_addr).await?;

            let to_write = data.len().min(MAX_SERIAL_WRITE);
            self.bus.bus_write(&[Bank0::MemRW.reg()]).await?;
            self.bus
                .bus_inner
                .transaction(
                    self.bus.address.get(),
                    &mut [
                        Operation::Write(&[Bank0::MemRW.reg()]),
                        Operation::Write(&data[bytes_written..bytes_written + to_write]),
                    ],
                )
                .await?;
            bytes_written += to_write;
            reg += to_write as u16;
        }
        Ok(())
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

}
