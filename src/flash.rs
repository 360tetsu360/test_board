#![allow(dead_code)]

use core::ops::{Range, RangeFrom};
use rp_pico as bsp;

use bsp::hal::rom_data;

pub const NAUTILUS_ID: &[u8; 8] = b"N4ut1lu5";
pub const BLOCK_SIZE: u32 = 65536;
pub const SECTOR_SIZE: usize = 4096;
pub const PAGE_SIZE: u32 = 256;
pub const SECTOR_ERASE: u8 = 0x20;
pub const BLOCK32_ERASE: u8 = 0x52;
pub const BLOCK64_ERASE: u8 = 0xD8;
pub const FLASH_XIP_BASE: u32 = 0x1000_0000;

pub const FLASH_END: u32 = 0x0020_0000;
pub const FLASH_USER_SIZE: u32 = 4096;

#[inline(never)]
#[link_section = ".data.ram_func"]
fn write_flash(data: &[u8]) {
    let addr = FLASH_END - FLASH_USER_SIZE;
    unsafe {
        cortex_m::interrupt::free(|_cs| {
            rom_data::connect_internal_flash();
            rom_data::flash_exit_xip();
            rom_data::flash_range_erase(addr, SECTOR_SIZE, BLOCK_SIZE, SECTOR_ERASE);
            rom_data::flash_range_program(addr, data.as_ptr(), data.len());
            rom_data::flash_flush_cache(); // Get the XIP working again
            rom_data::flash_enter_cmd_xip(); // Start XIP back up
        });
    }
    defmt::println!("write_flash() Complete"); // TEMP
}

fn read_flash() -> *const u8 {
    (FLASH_XIP_BASE + FLASH_END - FLASH_USER_SIZE) as *const u8
}

pub struct FlashMemory {
    data: [u8; FLASH_USER_SIZE as usize],
    initialized: bool,
}

impl FlashMemory {
    pub fn new() -> Self {
        let mut data = [0u8; FLASH_USER_SIZE as usize];
        unsafe {
            core::ptr::copy(read_flash(), data.as_mut_ptr(), data.len());
        }

        let initialized = data[..NAUTILUS_ID.len()].eq(NAUTILUS_ID);
        Self { data, initialized }
    }

    pub fn initialized(&self) -> bool {
        self.initialized
    }

    pub fn read_range(&self, range: Range<usize>) -> &[u8] {
        &self.data[range]
    }

    pub fn write_from(&mut self, from: RangeFrom<usize>, v: &[u8]) {
        self.data[from].copy_from_slice(v);
    }

    pub fn save(mut self) {
        if !self.initialized {
            self.data[..NAUTILUS_ID.len()].copy_from_slice(NAUTILUS_ID);
        }
        write_flash(&self.data)
    }
}
