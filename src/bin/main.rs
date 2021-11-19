#![allow(unused_imports)]
// Async for threading on each device
use std::error::Error;
use std::io::{self, Write};
use std::result::Result;
use std::sync::mpsc;
use std::thread;
use std::time::Duration;
use std::time::Instant;

pub use quadcopter::*;

// For terminating early
use simple_signal::{self, Signal};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

// IMU imports
use bno055::Bno055;
use linux_embedded_hal::{Delay, I2cdev};
use mint::{EulerAngles, Quaternion};

// Altimeter import
use bmp280::Bmp280Builder;

// PWM imports
use sysfs_pwm::Pwm;

fn main() {
    let mut angles: EulerAngles<f32, ()> = mint::EulerAngles::<f32, ()>::from([0.0, 0.0, 0.0]);
    let (tx_imu, rx_imu) = mpsc::channel();
    let (tx_stop_imu, rx_stop_imu) = mpsc::channel(); // Sending a stop command across threads

    let mut altitude: f32;
    let (tx_altimeter, rx_altimeter) = mpsc::channel();
    let (tx_stop_altimeter, rx_stop_altimeter) = mpsc::channel(); // Sending a stop command across threads

    let p = 1000.0;
    let i = 0.00;
    let d = 0.0;
    // Start pulses at 1_590_000 and 1_360_000
    // BlueRobotics BasicESCs operate over a hard-coded range of 1100-1900us, with a neutral value of 1500us
    // Instantiate Motor A
    let pwm_a = Pwm::new(0, 0).unwrap();
    let mut esc_a: ESCParamsNs = ESCParamsNs::default();
    esc_a.set_pulse_flight(1_690_000);
    let pid_a = PIDController::new(p, i, d);
    let mut motor_a = Motor::new(pwm_a, esc_a, pid_a);

    // Instantiate Motor B
    let pwm_b = Pwm::new(0, 1).unwrap();
    let mut esc_b: ESCParamsNs = ESCParamsNs::default();
    esc_b.set_pulse_flight(1_340_000);
    let pid_b = PIDController::new(p, i, d);
    let mut motor_b = Motor::new(pwm_b, esc_b, pid_b);

    // Instantiate Motor C
    let pwm_c = Pwm::new(4, 0).unwrap();
    let mut esc_c: ESCParamsNs = ESCParamsNs::default();
    esc_c.set_pulse_flight(1_310_000);
    let pid_c = PIDController::new(p, i, d);
    let mut motor_c = Motor::new(pwm_c, esc_c, pid_c);

    // Instantiate Motor D
    let pwm_d = Pwm::new(4, 1).unwrap();
    let mut esc_d: ESCParamsNs = ESCParamsNs::default();
    esc_d.set_pulse_flight(1_600_000);
    let pid_d = PIDController::new(p, i, d);
    let mut motor_d = Motor::new(pwm_d, esc_d, pid_d);

    let running = Arc::new(AtomicBool::new(true));
    simple_signal::set_handler(&[Signal::Int, Signal::Term], {
        let running = running.clone();
        move |_| {
            running.store(false, Ordering::SeqCst);
        }
    });

    let thread_imu = thread::spawn(move || {
        let dev = I2cdev::new("/dev/i2c-0").unwrap();
        let mut delay = Delay {};
        let mut imu = Bno055::new(dev).with_alternative_address();
        imu.init(&mut delay).expect("An error occurred while building the IMU");

        imu.set_mode(bno055::BNO055OperationMode::NDOF, &mut delay)
            .expect("An error occurred while setting the IMU mode");

        loop {
            match imu.euler_angles() {
                Ok(angles) => {
                    tx_imu.send(angles).unwrap();
                    // println!("count: {} at {}",count,now.elapsed().as_millis());
                }
                Err(e) => {
                    eprintln!("{:?}", e);
                }
            }

            match rx_stop_imu.try_recv() {
                Ok(val) => {
                    println!("received a {} on the stop loop", val);
                    break;
                }
                _ => continue,
            }
        }
    });

    let thread_altimeter = thread::spawn(move || {
        let mut altimeter = Bmp280Builder::new()
            .path("/dev/i2c-1")
            .address(0x77)
            .build()
            .expect("Failed to build device");

        altimeter.zero().expect("Device failed to zero");

        loop {
            match altimeter.altitude_m() {
                Ok(altitude) => {
                    tx_altimeter.send(altitude).unwrap();
                    // println!("count: {} at {}",count,now.elapsed().as_millis());
                }
                Err(e) => {
                    eprintln!("{:?}", e);
                }
            }

            match rx_stop_altimeter.try_recv() {
                Ok(val) => {
                    println!("received a {} on the stop loop", val);
                    break;
                }
                _ => continue,
            }
        }
    });

    // Init and get motors up to appx. flight
    motor_a.init().unwrap();
    //thread::sleep(Duration::from_millis(1000));
    motor_b.init().unwrap();
    //thread::sleep(Duration::from_millis(1000));
    motor_c.init().unwrap();
    //thread::sleep(Duration::from_millis(1000));
    motor_d.init().unwrap();
    //thread::sleep(Duration::from_millis(1000));

    motor_c.get_to_flight(3).unwrap();
    thread::sleep(Duration::from_millis(100));
    //motor_b.get_to_flight(50).unwrap();
    thread::sleep(Duration::from_millis(100));
    motor_a.get_to_flight(3).unwrap();
    thread::sleep(Duration::from_millis(100));
    //motor_d.get_to_flight(50).unwrap();
    thread::sleep(Duration::from_millis(100));

    while running.load(Ordering::SeqCst) {
        match rx_imu.try_recv() {
            Ok(val) => {
                angles = val;
                //println!("EulerAngles: {:.2?}", angles);
            }
            Err(_e) => {
                // println!("{:?}", e);
            }
        }

        match rx_altimeter.try_recv() {
            Ok(val) => {
                altitude = val;
                //println!("Altitude: {:.2?}", altitude);
            }
            Err(_e) => {
                // println!("{:?}", e);
            }
        }

        // Currently working with AC/BD motor same-side,opposite rotation pairs
        motor_a.update_pid_actual(-angles.a).unwrap();
        motor_a.update_control_signal().unwrap();

        motor_c.update_pid_actual(angles.a).unwrap();
        motor_c.update_control_signal().unwrap();

        //motor_b.update_pid_actual(0.5 * -angles.b + 0.5 * -angles.a).unwrap();
        //motor_b.update_control_signal().unwrap();

        //motor_d.update_pid_actual(0.5 * angles.b + 0.5 * angles.a).unwrap();
        //motor_d.update_control_signal().unwrap();


        println!("Motor Actual, a) {}    c) {}", motor_a.esc.pulse_actual, motor_c.esc.pulse_actual);
    }

    // Shutdown all motors
    motor_a.shutdown().unwrap();
    motor_b.shutdown().unwrap();
    motor_c.shutdown().unwrap();
    motor_d.shutdown().unwrap();

    // Stop the IMU thread and join it to main before exiting
    tx_stop_imu.send(false).unwrap();
    thread_imu.join().expect("Error joining IMU thread");
    tx_stop_altimeter.send(false).unwrap();
    thread_altimeter.join().expect("Error joining altimeter thread");
}
