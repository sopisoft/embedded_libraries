#![cfg_attr(all(target_arch = "arm", target_os = "none"), no_std)]
#![cfg_attr(all(target_arch = "arm", target_os = "none"), no_main)]

#[cfg(all(target_arch = "arm", target_os = "none"))]
mod embedded_stub {
    use defmt_rtt as _;
    use panic_probe as _;
    use rp235x_hal as hal;

    #[unsafe(link_section = ".start_block")]
    #[used]
    pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

    #[hal::entry]
    fn main() -> ! {
        loop {
            core::hint::spin_loop();
        }
    }
}

#[cfg(not(all(target_arch = "arm", target_os = "none")))]
mod host;

#[cfg(not(all(target_arch = "arm", target_os = "none")))]
fn main() -> Result<(), String> {
    host::run()
}
