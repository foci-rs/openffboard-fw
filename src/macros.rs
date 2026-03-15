macro_rules! write_filter_subreg {
    ($tmc:expr, $reg:expr, $value:expr, $label:literal) => {
        if $tmc.write_sub_register($reg, $value).await.is_err() {
            defmt::warn!("{} biquad write failed", $label);
        }
    };
}
