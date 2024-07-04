/// Register bank select register
pub const REG_BANK_SEL: u8 = 0x7F;

#[repr(u8)]
#[derive(PartialEq, Copy, Clone)]
pub enum UserBank {
    Bank0 = 0b00,
    Bank1 = 0b01,
    Bank2 = 0b10,
    Bank3 = 0b11,
}

#[allow(unused)]
#[derive(PartialEq, Copy, Clone)]
pub enum Bank0 {
    WhoAmI = 0x00,
    UserCtrl = 0x03,
    LpConfig = 0x05,
    PwrMgmt1 = 0x06,
    PwrMgmt2 = 0x07,
    IntPinCfg = 0x0F,
    IntEnable = 0x10,
    IntEnable1 = 0x11,
    IntEnable2 = 0x12,
    IntEnable3 = 0x13,
    I2cMstStatus = 0x17,
    DmpIntStatus = 0x18,
    IntStatus = 0x19,
    IntStatus1 = 0x1A,
    IntStatus2 = 0x1B,
    IntStatus3 = 0x1C,
    DelayTimeh = 0x28,
    DelayTimel = 0x29,
    AccelXoutH = 0x2D,
    AccelYoutH = 0x2F,
    AccelZoutH = 0x31,
    GyroXoutH = 0x33,
    GyroYoutH = 0x35,
    GyroZoutH = 0x37,
    TempOutH = 0x39,
    ExtSlvSensData00 = 0x3B,
    FifoEn1 = 0x66,
    FifoEn2 = 0x67,
    FifoRst = 0x68,
    FifoMode = 0x69,
    FifoCounth = 0x70,
    FifoCountl = 0x71,
    FifoRW = 0x72,
    DataRdyStatus = 0x74,
    FifoCfg = 0x76,
    MemStartAddr = 0x7C,
    MemRW = 0x7D,
    MemBankSel = 0x7E,
    RegBankSel = 0x7F,
}

#[allow(unused)]
#[derive(PartialEq, Copy, Clone)]
pub enum Bank1 {
    SelfTestXGyro = 0x02,
    SelfTestYGyro = 0x03,
    SelfTestZGyro = 0x04,
    SelfTestXAccel = 0x0E,
    SelfTestYAccel = 0x0F,
    SelfTestZAccel = 0x10,
    XaOffsH = 0x14,
    YaOffsH = 0x17,
    ZaOffsH = 0x1A,
    TimebaseCorrectionPll = 0x28,
}

#[allow(unused)]
#[derive(PartialEq, Copy, Clone)]
pub enum Bank2 {
    GyroSmplrtDiv = 0x00,
    GyroConfig1 = 0x01,
    GyroConfig2 = 0x02,
    XgOffsUsrh = 0x03,
    YgOffsUsrh = 0x05,
    ZgOffsUsrh = 0x07,
    OdrAlignEn = 0x09,
    AccelSmplrtDiv1 = 0x10,
    AccelSmplrtDiv2 = 0x11,
    AccelIntelCtrl = 0x12,
    AccelWomThrCtrl = 0x13,
    AccelConfig = 0x14,
    AccelConfig2 = 0x15,
    FsyncConfig = 0x52,
    TempConfig = 0x53,
    ModCtrlUsr = 0x54,
}

#[allow(unused)]
#[derive(PartialEq, Copy, Clone)]
pub enum Bank3 {
    I2cMstOdrConfig = 0x00,
    I2cMstCtrl = 0x01,
    I2cMstDelayCtrl = 0x02,
    I2cSlv0Addr = 0x03,
    I2cSlv0Reg = 0x04,
    I2cSlv0Ctrl = 0x05,
    I2cSlv0Do = 0x06,
    I2cSlv1Addr = 0x07,
    I2cSlv1Reg = 0x08,
    I2cSlv1Ctrl = 0x09,
    I2cSlv1Do = 0x0A,
}

pub trait Register {
    fn bank(self) -> UserBank;
    fn reg(self) -> u8;
}

impl Register for Bank0 {
    fn bank(self) -> UserBank {
        UserBank::Bank0
    }

    fn reg(self) -> u8 {
        self as u8
    }
}

impl Register for Bank1 {
    fn bank(self) -> UserBank {
        UserBank::Bank1
    }

    fn reg(self) -> u8 {
        self as u8
    }
}

impl Register for Bank2 {
    fn bank(self) -> UserBank {
        UserBank::Bank2
    }

    fn reg(self) -> u8 {
        self as u8
    }
}

impl Register for Bank3 {
    fn bank(self) -> UserBank {
        UserBank::Bank3
    }

    fn reg(self) -> u8 {
        self as u8
    }
}

#[allow(unused)]
#[derive(Copy, Clone)]
pub enum MagBank {
    DeviceId = 0x01,
    Status1 = 0x10,
    XDataLow = 0x11,
    YDataLow = 0x13,
    ZDataLow = 0x15,
    Status2 = 0x18,
    Control2 = 0x31,
    Control3 = 0x32,
}

impl MagBank {
    pub fn reg(self) -> u8 {
        self as u8
    }
}
