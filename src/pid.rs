#![allow(dead_code)]
#![allow(unused_parens)]

#[derive(Debug, Clone)]
pub struct PIDController {
    setpoint: f32,
    actual: f32,
    kp: f32,
    ki: f32,
    kd: f32,
    pre_error: f32,
    integral: f32,
    dt: f32,
}

impl PIDController {
    pub fn new(kp: f32, ki: f32, kd: f32) -> Self {
        PIDController {
            actual: 0.0,
            setpoint: 0.0,
            kp,
            ki,
            kd,
            pre_error: 0.0,
            integral: 0.0,
            dt: 0.05, // 20 Hz
        }
    }

    pub fn default() -> Self {
        PIDController {
            actual: 0.0,
            setpoint: 0.0,
            kp: 0.0,
            ki: 0.0,
            kd: 0.0,
            pre_error: 0.0,
            integral: 0.0,
            dt: 0.05, // 20 Hz
        }
    }

    pub fn set_kp(&mut self, kp: f32) {
        self.kp = kp;
    }

    pub fn set_ki(&mut self, ki: f32) {
        self.ki = ki;
    }

    pub fn set_kd(&mut self, kd: f32) {
        self.kd = kd;
    }

    pub fn set_dt(&mut self, dt: f32) {
        self.dt = dt;
    }

    pub fn set_setpoint(&mut self, setpoint: f32) {
        self.setpoint = setpoint;
    }

    #[inline]
    pub fn control_signal(&mut self) -> f32 {
        let error = self.setpoint - self.actual;
        self.integral += (error * self.dt);
        let derivative = (error - self.pre_error) / self.dt;
        self.pre_error = error;

        (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
    }

    pub fn update_actual(&mut self, actual: f32) {
        self.actual = actual;
    }
}

#[test]
fn build_controller() {
    let mut controller = PIDController::default();
    controller.set_kp(1.0);
    controller.set_ki(0.1);
    controller.set_kd(0.01);
    controller.set_setpoint(0.0);
}
