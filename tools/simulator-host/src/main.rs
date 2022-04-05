use std::env;
use std::fs::{self};
use std::time::Duration;

use cobs::CobsDecoder;
use rand::seq::index;
use rand::Rng;
use serialport::SerialPort;

fn encode_cmd(addr: u16, value: u8, cmd: u8) -> Vec<u8> {
    let cmd = [addr as u8, (addr >> 8) as u8, value, cmd];
    cobs::encode_vec(&cmd)
}

fn read(serial: &mut dyn SerialPort, addr: u16) -> u8 {
    let mut cmd = encode_cmd(addr, 0xFF, 0);
    cmd.push(0);
    serial.write_all(&cmd).unwrap();

    let mut value = [0_u8; 1];
    let mut decoder = CobsDecoder::new(&mut value);
    loop {
        let mut rsp = [0_u8; 1];
        let n = serial.read(&mut rsp).unwrap();
        if let Some((1, _)) = decoder.push(&rsp[..n]).unwrap() {
            return value[0];
        }
    }
}

fn write(serial: &mut dyn SerialPort, addr: u16, value: u8) {
    let mut cmd = encode_cmd(addr, value, 1);
    cmd.push(0);
    serial.write_all(&cmd).unwrap();
}

fn main() {
    let mut serial = serialport::new("COM13", 115200).open().unwrap();
    serial.set_timeout(Duration::ZERO).unwrap();
    // Pico usb_stdio requires this for the terminal to be detected as connected.
    serial.write_data_terminal_ready(true).unwrap();

    let cmd = env::args().nth(1).unwrap_or(String::default());
    if &cmd != "test" {
        for i in 0..2 {
            let addr: u16 = 0x4001 + i;
            println!("[{:#04X}]={:#02X}", addr, read(serial.as_mut(), addr));
        }
        return;
    }

    let rom = fs::read("10-print.gb").unwrap();

    let n_rom = 128;

    print!("Read ROM linear... ");
    for (addr, a) in rom.iter().enumerate().take(n_rom) {
        assert_eq!(*a, read(serial.as_mut(), addr as u16), "addr={:#04X}", addr);
    }
    println!("OK");

    print!("Read ROM bank 1 alias linear... ");
    for (addr, a) in rom.iter().enumerate().take(n_rom) {
        let addr = addr + 0x4000;
        assert_eq!(*a, read(serial.as_mut(), addr as u16), "addr={:#04X}", addr);
    }
    println!("OK");

    print!("Read ROM and bank 1 alias random... ");
    let mut rng = rand::thread_rng();
    let random_idx = index::sample(&mut rng, rom.len(), n_rom);
    for i in random_idx {
        let a = rom[i];
        let addr = if rng.gen_bool(0.5) { i } else { 0x4000 + i };
        assert_eq!(a, read(serial.as_mut(), addr as u16), "addr={:#04X}", addr);
    }
    println!("OK");

    let n_ram = 128;
    let ram_random_idx = index::sample(&mut rng, 8 * 1024, n_ram).into_vec();

    print!("Write RAM random... ");
    for (v, i) in (0..n_ram).zip(&ram_random_idx) {
        let addr = 0xA000 + *i;
        write(serial.as_mut(), addr as u16, v as u8);
    }
    print!("Read RAM random reverse... ");
    for (v, i) in (0..n_ram).zip(&ram_random_idx).rev() {
        let addr = 0xA000 + *i;
        assert_eq!(
            v as u8,
            read(serial.as_mut(), addr as u16),
            "addr={:#04X}",
            addr
        );
    }
    println!("OK");
}
