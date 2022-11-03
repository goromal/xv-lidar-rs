use clap::Parser;
use std::{thread, time};
use colored::Colorize;

/// XV LiDAR Interface Daemon.
#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Device name
    #[arg(short, long, default_value = "/dev/ttyACM0")]
    device: String,
}

fn main() {
    let args = Args::parse();
    println!("Connecting to {} at 115200 baud...", args.device.yellow().bold());
    let mut port = serialport::new(args.device, 115_200)
        .timeout(time::Duration::from_millis(10))
        .open().expect("Failed to open port");
    
    let mut init_level: i32 = 0;
    let mut index: u8 = 0;

    loop {
        thread::sleep(time::Duration::from_micros(10));
        if init_level == 0 {
            let mut serial_buf: Vec<u8> = vec![0; 1];
            let b: u8;
            match port.read(serial_buf.as_mut_slice()) {
                Ok(_) => {
                    b = serial_buf[0];
                    if b == 0xFA {
                        init_level = 1;
                    } else {
                        init_level = 0;
                    }
                },
                Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => (),
                Err(e) => eprintln!("{:?}", e),
            }
        } else if init_level == 1 {
            let mut serial_buf: Vec<u8> = vec![0; 1];
            let b: u8;
            match port.read(serial_buf.as_mut_slice()) {
                Ok(_) => {
                    b = serial_buf[0];
                    if b >= 0xA0 && b <= 0xF9 {
                        index = b - 0xA0;
                        init_level = 2;
                    }
                    else if b != 0xFA {
                        init_level = 0;
                    }
                },
                Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => (),
                Err(e) => eprintln!("{:?}", e),
            }
        } else if init_level == 2 {
            let mut b_speed: Vec<u8> = vec![0; 2];
            let mut b_data0: Vec<u8> = vec![0; 4];
            let mut b_data1: Vec<u8> = vec![0; 4];
            let mut b_data2: Vec<u8> = vec![0; 4];
            let mut b_data3: Vec<u8> = vec![0; 4];
            let mut b_checksum: Vec<u8> = vec![0; 2];
            port.read(b_speed.as_mut_slice()).expect("Broken data stream");
            port.read(b_data0.as_mut_slice()).expect("Broken data stream");
            port.read(b_data1.as_mut_slice()).expect("Broken data stream");
            port.read(b_data2.as_mut_slice()).expect("Broken data stream");
            port.read(b_data3.as_mut_slice()).expect("Broken data stream");
            port.read(b_checksum.as_mut_slice()).expect("Broken data stream");
            let incoming_checksum: u32 = Into::<u32>::into(b_checksum[0]) + 
                                         (Into::<u32>::into(b_checksum[1]) << 8);
            println!("Breaking loop; level 2 with index {}", index);
            break; // TODO
        } else {
            init_level = 0;
        }
    }
}
