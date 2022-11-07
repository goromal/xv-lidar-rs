use clap::Parser;
use std::{thread, time, io, fs};
use colored::Colorize;
use chrono;
use io::Write;

/// XV LiDAR Interface Daemon.
#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Device name
    #[arg(short, long, default_value = "/dev/ttyACM0")]
    device: String,
}

fn checksum(idx: u8, b_speed: &[u8], b_data0: &[u8], b_data1: &[u8], b_data2: &[u8], b_data3: &[u8]) -> u32 {
    fn increment_checksum(chk32: u32, data0: u8, data1: u8) -> u32 {
        (chk32 << 1) + Into::<u32>::into(data0) + (Into::<u32>::into(data1) << 8)
    }
    let mut chk32: u32 = 0;
    chk32 = increment_checksum(chk32, 0xFA, idx + 0xA0);
    chk32 = increment_checksum(chk32, b_speed[0], b_speed[1]);
    chk32 = increment_checksum(chk32, b_data0[0], b_data0[1]);
    chk32 = increment_checksum(chk32, b_data0[2], b_data0[3]);
    chk32 = increment_checksum(chk32, b_data1[0], b_data1[1]);
    chk32 = increment_checksum(chk32, b_data1[2], b_data1[3]);
    chk32 = increment_checksum(chk32, b_data2[0], b_data2[1]);
    chk32 = increment_checksum(chk32, b_data2[2], b_data2[3]);
    chk32 = increment_checksum(chk32, b_data3[0], b_data3[1]);
    chk32 = increment_checksum(chk32, b_data3[2], b_data3[3]);
    ((chk32 & 0x7FFF) + (chk32 >> 15)) & 0x7FFF
}

fn parse_data_packet(b_data: &[u8]) -> (i32, i32) {
    let dist_mm: i32 = Into::<i32>::into(b_data[0]) | (Into::<i32>::into(b_data[1] & 0x3F) << 8);
    let quality: i32 = Into::<i32>::into(b_data[2]) | (Into::<i32>::into(b_data[3]) << 8);
    (dist_mm, quality)
}

fn main() {
    let args = Args::parse();
    println!("Connecting to {} at 115200 baud...", args.device.yellow().bold());
    let mut port = serialport::new(args.device, 115_200)
        .timeout(time::Duration::from_millis(20))
        .open().expect("Failed to open port");
    
    let mut datafile = fs::OpenOptions::new()
                          .write(true)
                          .create(true)
                          .open(format!("xv_meas_{}.csv", chrono::offset::Utc::now()))
                          .expect("Unable to open data file");
    
    let mut init_level: u8 = 0;
    let mut index: u8 = 0;
    let mut num_errors: i32 = 0;
    let mut distances_mm: Vec<i32> = vec![-1; 360];
    let mut qualities: Vec<i32> = vec![-1; 360];

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
            let calculated_checksum: u32 = checksum(index, &b_speed, &b_data0, &b_data1, &b_data2, &b_data3);
            let (dist_mm_0, dist_mm_1, dist_mm_2, dist_mm_3): (i32, i32, i32, i32);
            let (qual_0, qual_1, qual_2, qual_3): (i32, i32, i32, i32);
            if calculated_checksum == incoming_checksum {
                (dist_mm_0, qual_0) = parse_data_packet(&b_data0);
                (dist_mm_1, qual_1) = parse_data_packet(&b_data1);
                (dist_mm_2, qual_2) = parse_data_packet(&b_data2);
                (dist_mm_3, qual_3) = parse_data_packet(&b_data3);
            }
            else {
                num_errors += 1;
                (dist_mm_0, qual_0) = (-1, -1);
                (dist_mm_1, qual_1) = (-1, -1);
                (dist_mm_2, qual_2) = (-1, -1);
                (dist_mm_3, qual_3) = (-1, -1);
            }
            distances_mm[Into::<usize>::into(index) * 4 + 0] = dist_mm_0;
            distances_mm[Into::<usize>::into(index) * 4 + 1] = dist_mm_1;
            distances_mm[Into::<usize>::into(index) * 4 + 2] = dist_mm_2;
            distances_mm[Into::<usize>::into(index) * 4 + 3] = dist_mm_3;
            qualities[Into::<usize>::into(index) * 4 + 0] = qual_0;
            qualities[Into::<usize>::into(index) * 4 + 1] = qual_1;
            qualities[Into::<usize>::into(index) * 4 + 2] = qual_2;
            qualities[Into::<usize>::into(index) * 4 + 3] = qual_3;
            init_level = 0;
            if index == 89 {
                datafile.write_all(format!("{},", chrono::offset::Utc::now()).as_bytes()).expect("Unable to write to data file");
                for i in 0..360 {
                    datafile.write_all(format!("{},{},", distances_mm[i], qualities[i]).as_bytes()).expect("Unable to write to data file");
                }
                datafile.write_all("\n".as_bytes()).expect("Unable to write to data file");
            }
        } else {
            init_level = 0;
        }
    }
}
