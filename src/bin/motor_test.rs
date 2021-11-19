use quadcopter::{ESCParamsNs, Motor, PIDController};
use sysfs_pwm::Pwm;

// For terminating early
use simple_signal::{self, Signal};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

fn main() {
    let p = 150.0;
    let i = 0.00;
    let d = 0.0;
    // Start pulses at 1_590_000 and 1_360_000
    // Instantiate Motor A
    let pwm_a = Pwm::new(0, 0).unwrap();
    let mut esc_a: ESCParamsNs = ESCParamsNs::default();
    esc_a.set_pulse_flight(1_590_000);
    let pid_a = PIDController::new(p, i, d);
    let mut motor_a = Motor::new(pwm_a, esc_a, pid_a);

    motor_a.init().unwrap();
    let running = Arc::new(AtomicBool::new(true));
    simple_signal::set_handler(&[Signal::Int, Signal::Term], {
        let running = running.clone();
        move |_| {
            running.store(false, Ordering::SeqCst);
        }
    });

    motor_a.get_to_flight(50).unwrap();
    motor_a.shutdown().unwrap();
    
}
