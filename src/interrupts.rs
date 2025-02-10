use bitfield::bitfield;

bitfield! {
    /// Interrupt enable flags from ICM-20948 datasheet
    pub struct Icm20948IntEnable(u8);
    impl Debug;
    pub i2c_mst_int_en, set_i2c_mst_int_en: 0;
    pub dmp_int1_en, set_dmp_int1_en: 1;
    pub pll_ready_en, set_pll_ready_en: 2;
    pub wom_int_en, set_wom_int_en: 3;
    pub reserved, set_reserved: 6, 4;
    pub reg_wof_en, set_reg_wof_en: 7;
}
