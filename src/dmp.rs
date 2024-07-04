// Guide:
// https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary/blob/main/DMP.md#how-is-the-dmp-loaded-and-started

use bitfield::bitfield;

use crate::*;

const FIRMWARE: &[u8] = include_bytes!("dmp.raw");

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
    pub async fn load_dmp(&mut self) {}
}

#[test]
fn count_firmware() {
    assert_eq!(14301, FIRMWARE.len());
}

bitfield! {
    struct PeriphxCtrl(u8);
    get_leng, set_leng: 4, 0;
    get_grp, set_grp: 4;
    get_reg_dis, set_reg_dis: 5;
    get_byte_sw, set_byte_sw: 6;
    get_en, set_en: 7;
}
