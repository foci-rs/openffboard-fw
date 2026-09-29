::ankyra::klipper_enumeration! {
    #[allow(non_camel_case_types)]
    pub enum Pins(name = "pin") {
        Range(PA, 0, 16),
        Range(PB, 0, 16),
        Range(PC, 0, 16),
        Range(PD, 0, 16),
        Range(PE, 0, 16),
        STEP0,
        DIR0,
        ENA0,
        STALL0,
        MCU_TEMP,
    }
}
