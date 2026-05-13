// Continuous gyro bias estimation, ported from
// JibbSmart/GamepadMotionHelpers (`AutoCalibration::AddSampleStillness` in
// `GamepadMotion.hpp` v9, MIT-licensed). The controller is judged stationary
// from gyro + accel deltas across a growing collection window; while it
// remains stationary the bias is exponential-LERP'd toward the window-mean
// gyro, with a half-time scaled by accumulated confidence so the first lock
// is instant and subsequent updates are slow. Accel is part of the stillness
// gate so a slow constant rotation cannot masquerade as zero motion.

const MIN_STILLNESS_SAMPLES: u32 = 10;
const MIN_STILLNESS_COLLECTION_TIME: f32 = 0.5;
const MIN_STILLNESS_CORRECTION_TIME: f32 = 2.0;
const MAX_STILLNESS_ERROR: f32 = 2.0;
const STILLNESS_SAMPLE_DETERIORATION_RATE: f32 = 0.2;
const STILLNESS_ERROR_CLIMB_RATE: f32 = 0.1;
const STILLNESS_ERROR_DROP_ON_RECALIBRATE: f32 = 0.1;
const STILLNESS_CALIBRATION_EASE_IN_TIME: f32 = 3.0;
const STILLNESS_CALIBRATION_HALF_TIME: f32 = 0.1;
const STILLNESS_CONFIDENCE_RATE: f32 = 1.0;

const INITIAL_MIN_DELTA_GYRO: f32 = 1.0;
const INITIAL_MIN_DELTA_ACCEL: f32 = 0.25;

#[derive(Debug, Clone, Copy)]
struct Window {
    min_gyro: [f32; 3],
    max_gyro: [f32; 3],
    mean_gyro: [f32; 3],
    min_accel: [f32; 3],
    max_accel: [f32; 3],
    mean_accel: [f32; 3],
    num_samples: u32,
    time_sampled: f32,
}

impl Window {
    const fn new() -> Self {
        Self {
            min_gyro: [0.0; 3],
            max_gyro: [0.0; 3],
            mean_gyro: [0.0; 3],
            min_accel: [0.0; 3],
            max_accel: [0.0; 3],
            mean_accel: [0.0; 3],
            num_samples: 0,
            time_sampled: 0.0,
        }
    }

    fn reset(&mut self) {
        self.num_samples = 0;
        self.time_sampled = 0.0;
    }

    fn add(&mut self, gyro: [f32; 3], accel: [f32; 3], dt: f32) {
        if self.num_samples == 0 {
            self.min_gyro = gyro;
            self.max_gyro = gyro;
            self.mean_gyro = gyro;
            self.min_accel = accel;
            self.max_accel = accel;
            self.mean_accel = accel;
            self.num_samples = 1;
            self.time_sampled = dt;
            return;
        }
        for i in 0..3 {
            self.min_gyro[i] = self.min_gyro[i].min(gyro[i]);
            self.max_gyro[i] = self.max_gyro[i].max(gyro[i]);
            self.min_accel[i] = self.min_accel[i].min(accel[i]);
            self.max_accel[i] = self.max_accel[i].max(accel[i]);
        }
        self.num_samples += 1;
        self.time_sampled += dt;
        let n_inv = 1.0 / self.num_samples as f32;
        for i in 0..3 {
            self.mean_gyro[i] += (gyro[i] - self.mean_gyro[i]) * n_inv;
            self.mean_accel[i] += (accel[i] - self.mean_accel[i]) * n_inv;
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct GyroCalibration {
    bias: [f32; 3],
    confidence: f32,
    time_steady: f32,
    min_delta_gyro: [f32; 3],
    min_delta_accel: [f32; 3],
    recalibrate_threshold: f32,
    is_steady: bool,
    window: Window,
}

impl Default for GyroCalibration {
    fn default() -> Self {
        Self::new()
    }
}

impl GyroCalibration {
    pub const fn new() -> Self {
        Self {
            bias: [0.0; 3],
            confidence: 0.0,
            time_steady: 0.0,
            min_delta_gyro: [INITIAL_MIN_DELTA_GYRO; 3],
            min_delta_accel: [INITIAL_MIN_DELTA_ACCEL; 3],
            recalibrate_threshold: 1.0,
            is_steady: false,
            window: Window::new(),
        }
    }

    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// Take a fresh (post-axis-mapping) gyro/accel sample and dt seconds since
    /// the previous sample. Updates the bias estimate when the controller is
    /// detected stationary, and returns the bias-corrected gyro.
    pub fn correct(&mut self, gyro_dps: [f32; 3], accel_g: [f32; 3], dt: f32) -> [f32; 3] {
        if dt > 0.0 {
            self.add_sample(gyro_dps, accel_g, dt);
        }
        [
            gyro_dps[0] - self.bias[0],
            gyro_dps[1] - self.bias[1],
            gyro_dps[2] - self.bias[2],
        ]
    }

    fn add_sample(&mut self, gyro: [f32; 3], accel: [f32; 3], dt: f32) {
        if gyro == [0.0; 3] && accel == [0.0; 3] {
            return;
        }

        self.window.add(gyro, accel, dt);
        let gyro_delta = [
            self.window.max_gyro[0] - self.window.min_gyro[0],
            self.window.max_gyro[1] - self.window.min_gyro[1],
            self.window.max_gyro[2] - self.window.min_gyro[2],
        ];
        let accel_delta = [
            self.window.max_accel[0] - self.window.min_accel[0],
            self.window.max_accel[1] - self.window.min_accel[1],
            self.window.max_accel[2] - self.window.min_accel[2],
        ];

        if self.confidence < 1.0 {
            let climb = STILLNESS_SAMPLE_DETERIORATION_RATE * dt;
            for i in 0..3 {
                self.min_delta_gyro[i] += climb;
                self.min_delta_accel[i] += climb;
            }
        }

        if self.window.num_samples < MIN_STILLNESS_SAMPLES
            || self.window.time_sampled < MIN_STILLNESS_COLLECTION_TIME
        {
            self.recalibrate_threshold = (self.recalibrate_threshold
                + STILLNESS_ERROR_CLIMB_RATE * dt)
                .min(MAX_STILLNESS_ERROR);
            return;
        }

        for i in 0..3 {
            self.min_delta_gyro[i] = self.min_delta_gyro[i].min(gyro_delta[i]);
            self.min_delta_accel[i] = self.min_delta_accel[i].min(accel_delta[i]);
        }

        let thr = self.recalibrate_threshold;
        let still = gyro_delta[0] <= self.min_delta_gyro[0] * thr
            && gyro_delta[1] <= self.min_delta_gyro[1] * thr
            && gyro_delta[2] <= self.min_delta_gyro[2] * thr
            && accel_delta[0] <= self.min_delta_accel[0] * thr
            && accel_delta[1] <= self.min_delta_accel[1] * thr
            && accel_delta[2] <= self.min_delta_accel[2] * thr;

        if still {
            if self.window.time_sampled < MIN_STILLNESS_CORRECTION_TIME {
                self.recalibrate_threshold = (self.recalibrate_threshold
                    + STILLNESS_ERROR_CLIMB_RATE * dt)
                    .min(MAX_STILLNESS_ERROR);
                self.is_steady = false;
                return;
            }

            self.time_steady = (self.time_steady + dt).min(STILLNESS_CALIBRATION_EASE_IN_TIME);
            let ease_in = if STILLNESS_CALIBRATION_EASE_IN_TIME <= 0.0 {
                1.0
            } else {
                self.time_steady / STILLNESS_CALIBRATION_EASE_IN_TIME
            };
            let half_time = STILLNESS_CALIBRATION_HALF_TIME * self.confidence;
            // exp2(-x) -> 1 when x small, -> 0 when x large. At confidence=0
            // the half_time is 0 and we treat the factor as 0, which snaps
            // bias to the window mean on the first lock.
            let lerp_factor = if half_time <= 0.0 {
                0.0
            } else {
                (-ease_in * dt / half_time).exp2()
            };
            let calibrated = self.window.mean_gyro;
            let old_bias = self.bias;
            self.bias = [
                calibrated[0] + (old_bias[0] - calibrated[0]) * lerp_factor,
                calibrated[1] + (old_bias[1] - calibrated[1]) * lerp_factor,
                calibrated[2] + (old_bias[2] - calibrated[2]) * lerp_factor,
            ];
            self.confidence = (self.confidence + dt * STILLNESS_CONFIDENCE_RATE).min(1.0);
            self.is_steady = true;
        } else if self.time_steady > 0.0 {
            self.recalibrate_threshold =
                (self.recalibrate_threshold - STILLNESS_ERROR_DROP_ON_RECALIBRATE).max(1.0);
            self.time_steady = 0.0;
            self.window.reset();
            self.is_steady = false;
        } else {
            self.recalibrate_threshold = (self.recalibrate_threshold
                + STILLNESS_ERROR_CLIMB_RATE * dt)
                .min(MAX_STILLNESS_ERROR);
            self.window.reset();
            self.is_steady = false;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f32 = 1.0 / 250.0;
    const GRAVITY: [f32; 3] = [0.0, 1.0, 0.0];

    fn feed_stationary(cal: &mut GyroCalibration, bias: [f32; 3], seconds: f32) {
        let n = (seconds / DT) as u32;
        for _ in 0..n {
            cal.correct(bias, GRAVITY, DT);
        }
    }

    #[test]
    fn passes_through_before_lock() {
        let mut cal = GyroCalibration::new();
        let raw = [0.7, -0.4, 0.2];
        let out = cal.correct(raw, GRAVITY, DT);
        assert_eq!(out, raw);
        assert!(!cal.is_steady);
        assert_eq!(cal.bias, [0.0; 3]);
    }

    #[test]
    fn rejects_all_zero_sample() {
        let mut cal = GyroCalibration::new();
        for _ in 0..200 {
            cal.correct([0.0; 3], [0.0; 3], DT);
        }
        // Window must remain empty -> never enters collection phase.
        assert_eq!(cal.window.num_samples, 0);
        assert!(!cal.is_steady);
    }

    #[test]
    fn locks_onto_constant_bias_when_stationary() {
        let mut cal = GyroCalibration::new();
        let true_bias = [0.5, -0.3, 0.1];
        // 3 s of perfectly still input — well past the 2.0 s correction window.
        feed_stationary(&mut cal, true_bias, 3.0);
        assert!(cal.is_steady, "should be steady after 3s stationary");
        for i in 0..3 {
            assert!(
                (cal.bias[i] - true_bias[i]).abs() < 0.05,
                "bias[{i}]={} not within 0.05 of {}",
                cal.bias[i],
                true_bias[i]
            );
        }
        let out = cal.correct(true_bias, GRAVITY, DT);
        for i in 0..3 {
            assert!(out[i].abs() < 0.05, "out[{i}]={} not near 0", out[i]);
        }
    }

    #[test]
    fn does_not_calibrate_while_moving() {
        let mut cal = GyroCalibration::new();
        for i in 0..2000 {
            // High-amplitude oscillation: clearly not stationary.
            let g = ((i as f32) * 0.3).sin() * 50.0;
            cal.correct([g, g, g], GRAVITY, DT);
        }
        assert!(!cal.is_steady);
        assert_eq!(cal.bias, [0.0; 3]);
    }

    #[test]
    fn reset_clears_state() {
        let mut cal = GyroCalibration::new();
        feed_stationary(&mut cal, [0.5, -0.3, 0.1], 3.0);
        assert!(cal.is_steady);
        assert_ne!(cal.bias, [0.0; 3]);
        cal.reset();
        assert_eq!(cal.bias, [0.0; 3]);
        assert_eq!(cal.confidence, 0.0);
        assert!(!cal.is_steady);
    }

    #[test]
    fn movement_after_lock_does_not_destroy_bias_immediately() {
        let mut cal = GyroCalibration::new();
        let true_bias = [0.4, 0.0, -0.2];
        feed_stationary(&mut cal, true_bias, 3.0);
        let locked_bias = cal.bias;
        // Brief shake — bias should not be recomputed from non-stationary samples.
        for i in 0..50 {
            let g = ((i as f32) * 0.9).sin() * 80.0;
            cal.correct(
                [g + true_bias[0], g + true_bias[1], g + true_bias[2]],
                GRAVITY,
                DT,
            );
        }
        assert!(!cal.is_steady);
        for i in 0..3 {
            assert!(
                (cal.bias[i] - locked_bias[i]).abs() < 0.05,
                "bias drifted during motion: was {:?} now {:?}",
                locked_bias,
                cal.bias
            );
        }
    }
}
