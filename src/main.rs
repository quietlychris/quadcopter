#![allow(unused_imports)]
// Async for threading on each device
use std::error::Error;
use std::io::{self, Write};
use std::result::Result;
use std::sync::mpsc;
use std::thread;
use std::time::Duration;
use std::time::Instant;

mod pid;
use pid::*;
mod motor;
use motor::*;

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

    // Start pulses at 1_590_000 and 1_360_000
    // Instantiate Motor A
    let pwm_a = Pwm::new(0, 0).unwrap();
    let mut esc_a: ESCParamsNs = ESCParamsNs::default();
    esc_a.set_pulse_flight(1_590_000);
    let pid_a = PIDController::new(50.0, 0.05, 0.8);
    let mut motor_a = Motor::new(pwm_a, esc_a, pid_a);

    // Instantiate Motor B
    let pwm_b = Pwm::new(0, 1).unwrap();
    let mut esc_b: ESCParamsNs = ESCParamsNs::default();
    esc_b.set_pulse_flight(1_360_000);
    let pid_b = PIDController::new(50.0, 0.05, 0.8);
    let mut motor_b = Motor::new(pwm_b, esc_b, pid_b);

    // Instantiate Motor C
    let pwm_c = Pwm::new(4, 0).unwrap();
    let mut esc_c: ESCParamsNs = ESCParamsNs::default();
    esc_c.set_pulse_flight(1_360_000);
    let pid_c = PIDController::new(60.0, 0.005, 2.2);
    let mut motor_c = Motor::new(pwm_c, esc_c, pid_c);

    // Instantiate Motor D
    let pwm_d = Pwm::new(4, 1).unwrap();
    let mut esc_d: ESCParamsNs = ESCParamsNs::default();
    esc_d.set_pulse_flight(1_590_000);
    let pid_d = PIDController::new(50.0, 0.05, 1.6);
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
        imu.init(&mut delay)
            .expect("An error occurred while building the IMU");

        imu.set_mode(bno055::BNO055OperationMode::NDOF, &mut delay)
            .expect("An error occurred while setting the IMU mode");

        // calibrate_imu(&mut imu);
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

    // Init and get motors up to appx. flight
    motor_a.init().unwrap();
    thread::sleep(Duration::from_millis(100));
    motor_b.init().unwrap();
    thread::sleep(Duration::from_millis(100));
    motor_c.init().unwrap();
    thread::sleep(Duration::from_millis(100));
    motor_d.init().unwrap();

    
    motor_a.get_to_flight(10).unwrap();
    thread::sleep(Duration::from_millis(100));
    motor_b.get_to_flight(10).unwrap();
    thread::sleep(Duration::from_millis(100));
    motor_c.get_to_flight(10).unwrap();
    thread::sleep(Duration::from_millis(100));
    motor_d.get_to_flight(10).unwrap();
    

    while running.load(Ordering::SeqCst) {
        match rx_imu.try_recv() {
            Ok(val) => {
                angles = val;
                println!("EulerAngles: {:.2?}", angles);
            }
            Err(_e) => {
                // println!("{:?}", e);
            }
        }

        
        motor_a.update_pid_actual(-angles.b).unwrap();
        motor_a.update_control_signal().unwrap();

        motor_b.update_pid_actual(-angles.b).unwrap();
        motor_b.update_control_signal().unwrap();
        
        motor_c.update_pid_actual(angles.a).unwrap();
        motor_c.update_control_signal().unwrap();

        motor_d.update_pid_actual(angles.a).unwrap();
        motor_d.update_control_signal().unwrap();

    }

    // Shutdown all motors
    motor_a.shutdown().unwrap();
    motor_b.shutdown().unwrap();
    motor_c.shutdown().unwrap();
    motor_d.shutdown().unwrap();
    
    // Stop the IMU thread and join it to main before exiting
    tx_stop_imu.send(false).unwrap();
    thread_imu.join().expect("Error joining IMU thread");
}
