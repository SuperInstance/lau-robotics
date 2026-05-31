use serde::{Deserialize, Serialize};

/// PID controller with anti-windup.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct PIDController {
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
    pub setpoint: f64,
    pub integral: f64,
    pub prev_error: f64,
    pub output_min: f64,
    pub output_max: f64,
    pub integral_limit: f64,
    pub initialized: bool,
}

impl PIDController {
    pub fn new(kp: f64, ki: f64, kd: f64) -> Self {
        Self {
            kp,
            ki,
            kd,
            setpoint: 0.0,
            integral: 0.0,
            prev_error: 0.0,
            output_min: f64::NEG_INFINITY,
            output_max: f64::INFINITY,
            integral_limit: f64::MAX,
            initialized: false,
        }
    }

    pub fn with_limits(mut self, min: f64, max: f64) -> Self {
        self.output_min = min;
        self.output_max = max;
        self
    }

    pub fn with_integral_limit(mut self, limit: f64) -> Self {
        self.integral_limit = limit;
        self
    }

    pub fn set_setpoint(&mut self, setpoint: f64) {
        self.setpoint = setpoint;
    }

    /// Compute one PID step. `measurement` is the current process variable.
    /// `dt` is the time step in seconds.
    pub fn update(&mut self, measurement: f64, dt: f64) -> f64 {
        let error = self.setpoint - measurement;

        // Proportional
        let p = self.kp * error;

        // Integral with anti-windup
        self.integral += error * dt;
        self.integral = self.integral.clamp(-self.integral_limit, self.integral_limit);
        let i = self.ki * self.integral;

        // Derivative
        let d = if self.initialized {
            self.kd * (error - self.prev_error) / dt
        } else {
            0.0
        };

        self.prev_error = error;
        self.initialized = true;

        let output = p + i + d;
        output.clamp(self.output_min, self.output_max)
    }

    /// Reset the controller state.
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_error = 0.0;
        self.initialized = false;
    }
}
