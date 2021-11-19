use crate::PIDController;
use std::error::Error;
use std::thread;
use std::time::Duration;
use sysfs_pwm::Pwm;

/// Electronic Speed Controller parameters, in nanoseconds
#[derive(Debug, Clone)]
pub struct ESCParamsNs {
    period: u32,
    pulse_min: u32,
    pulse_neutral: u32,
    pulse_max: u32,
    pulse_flight: u32,
    pub pulse_actual: u32,
}

impl ESCParamsNs {
    pub fn default() -> Self {
        let pulse_neutral = 1_500_000;
        ESCParamsNs {
            period: 20_000_000,
            pulse_min: 1_200_000,
            pulse_neutral,
            pulse_max: 1_800_000,
            pulse_flight: pulse_neutral,
            pulse_actual: pulse_neutral,
        }
    }

    // PULSE_FLIGHT is initially set to pulse neutral, and will need to be
    // guesstimated for each vehicle/payload
    // Approximately the signal required for initial lift
    pub fn set_pulse_flight(&mut self, pulse_flight: u32) {
        self.pulse_flight = pulse_flight;
    }
}

#[derive(Debug)]
pub struct Motor {
    pwm: Pwm,
    pub esc: ESCParamsNs,
    pub pid: PIDController,
}

impl Motor {
    pub fn new(pwm: Pwm, esc: ESCParamsNs, pid: PIDController) -> Self {
        Motor { pwm, esc, pid }
    }

    #[inline]
    pub fn update_pid_actual(&mut self, actual_value: f32) -> Result<(), Box<dyn Error>> {
        self.pid.update_actual(actual_value);

        Ok(())
    }

    #[inline]
    pub fn update_control_signal(&mut self) -> Result<(), Box<dyn Error>> {
        let signal = self.pid.control_signal();
        let signal = (signal * 10.) as i32;
        let pulse = (self.esc.pulse_flight as i32 - signal) as u32;

        // dbg!(pulse);
        self.pwm.set_duty_cycle_ns(pulse)?;
        self.esc.pulse_actual = pulse;

        Ok(())
    }

    pub fn init(&mut self) -> Result<(), Box<dyn Error>> {
        self.pwm.export()?;
        self.pwm.set_period_ns(self.esc.period)?;
        self.pwm.set_duty_cycle_ns(self.esc.pulse_min)?;
        self.pwm.enable(true)?;

        thread::sleep(Duration::from_millis(100));
        self.pwm.set_duty_cycle_ns(self.esc.pulse_min)?;
        thread::sleep(Duration::from_millis(500));
        self.pwm.set_duty_cycle_ns(self.esc.pulse_neutral).unwrap();
        thread::sleep(Duration::from_millis(250));

        Ok(())
    }

    pub fn get_to_flight(&mut self, step_time_ms: u64) -> Result<(), Box<dyn Error>> {
        if self.esc.pulse_flight == self.esc.pulse_neutral {
            panic!("Error: please set pulse_flight != pulse_neutral. Currently: {}", self.esc.pulse_flight);
        }

        match self.esc.pulse_flight < self.esc.pulse_neutral {
            true => {
                println!("Motor direction: CCW");
                for step in (self.esc.pulse_flight..self.esc.pulse_neutral).step_by(100).rev() {
                    println!("{}", step);
                    self.pwm.set_duty_cycle_ns(step)?;
                    thread::sleep(Duration::from_millis(step_time_ms));
                }
                println!("At flight speed for CCW");
            }
            false => {
                println!("Motor direction: CC");
                for step in (self.esc.pulse_neutral..self.esc.pulse_flight).step_by(100) {
                    println!("{}", step);
                    self.pwm.set_duty_cycle_ns(step)?;
                    thread::sleep(Duration::from_millis(step_time_ms));
                }
                println!("At flight speed for CC");
            }
        }

        Ok(())
    }

    pub fn shutdown(&mut self) -> Result<(), Box<dyn Error>> {
        self.pwm.set_duty_cycle_ns(self.esc.pulse_neutral)?;
        self.pwm.enable(false)?;
        Ok(())
    }
}
