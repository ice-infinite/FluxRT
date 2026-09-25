//! Prints the exact application-owned runtime baseline and its canonical CRC.
//!
//! This is an audit helper for the offline production-profile workflow. It does
//! not write firmware sources and it cannot set an approval bit.

use foc_rt_bridge::{
    foc_rust_default_st_config, foc_rust_runtime_config_crc32, FocRuntimeConfig, FocStatus,
};

fn main() {
    let mut config = FocRuntimeConfig::default();
    // SAFETY: both pointers reference live, writable/readable host values for
    // the duration of each FFI call.
    let status = unsafe { foc_rust_default_st_config(&mut config) };
    assert_eq!(status, FocStatus::Ok);

    // Keep these overrides in lock-step with foc_production_profile_apply().
    config.observer_backend = 0;
    config.observer_enable = 1;
    config.closed_loop_enable = 0;
    config.observer_update_divider = 1;
    config.voltage_utilization = 0.90;

    let mut crc32 = 0;
    // SAFETY: both pointers reference live host values for the call duration.
    let status = unsafe { foc_rust_runtime_config_crc32(&config, &mut crc32) };
    assert_eq!(status, FocStatus::Ok);

    println!("{config:#?}");
    println!("runtime_config_crc32=0x{crc32:08X}");
}
