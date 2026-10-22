//! Walking, from the step rhythm in the accelerometer, a port of `python/experiments/sensor_trips.py`'s `walking`:
//! in each [`WINDOW_S`] window, |a| shakes by over [`MIN_STD_MPS2`], mostly at step rates.
//! On seven rides, [`IN_A_ROW`] such windows in a row came 1489 times, 5 of them aboard a train, none while it moved
//! (GNSS speed, where confident), so the rider is on foot, at walking speed, not on a train.

use std::sync::Arc;

use rustfft::num_complex::Complex64;
use rustfft::{Fft, FftPlanner};

const WINDOW_S: f64 = 4.0;
const MIN_STD_MPS2: f64 = 1.0;
/// Step rates (Hz), and the share of the power above [`MIN_HZ`] they must hold.
const STEP_HZ: (f64, f64) = (1.4, 2.6);
const MIN_HZ: f64 = 0.3;
const STEP_SHARE: f64 = 0.35;
const MIN_SAMPLES: usize = 100;
pub const IN_A_ROW: u32 = 2;

#[derive(Default)]
pub struct WalkingDetector {
    window_start_ns: Option<i64>,
    mags: Vec<f64>,
    in_a_row: u32,
    planner: Option<(usize, Arc<dyn Fft<f64>>)>,
}

impl WalkingDetector {
    /// An accelerometer sample's magnitude (m/s^2, bias removed); at the end of each window,
    /// whether the rider has been walking for [`IN_A_ROW`] windows.
    pub fn on_accel(&mut self, t_ns: i64, magnitude: f64) -> Option<bool> {
        let start = *self.window_start_ns.get_or_insert(t_ns);
        if ((t_ns - start) as f64) < WINDOW_S * 1e9 {
            self.mags.push(magnitude);
            return None;
        }
        let steps = self.steps();
        self.in_a_row = if steps { self.in_a_row + 1 } else { 0 };
        // Windows stay on a fixed grid from the first sample, as in Python.
        let windows = ((t_ns - start) as f64 / (WINDOW_S * 1e9)).floor() as i64;
        self.window_start_ns = Some(start + windows * (WINDOW_S * 1e9) as i64);
        self.mags.clear();
        self.mags.push(magnitude);
        Some(self.in_a_row >= IN_A_ROW)
    }

    fn steps(&mut self) -> bool {
        let n = self.mags.len();
        if n < MIN_SAMPLES {
            return false;
        }
        let mean = self.mags.iter().sum::<f64>() / n as f64;
        let var = self.mags.iter().map(|m| (m - mean).powi(2)).sum::<f64>() / n as f64;
        if var.sqrt() <= MIN_STD_MPS2 {
            return false;
        }
        let fft = match &self.planner {
            Some((size, fft)) if *size == n => fft.clone(),
            _ => {
                let fft = FftPlanner::new().plan_fft_forward(n);
                self.planner = Some((n, fft.clone()));
                fft
            }
        };
        let mut buf: Vec<Complex64> = self
            .mags
            .iter()
            .map(|m| Complex64::new(m - mean, 0.0))
            .collect();
        fft.process(&mut buf);
        let df = 1.0 / WINDOW_S;
        let (mut step, mut total) = (0.0, 0.0);
        for (k, c) in buf.iter().take(n / 2 + 1).enumerate() {
            let f = k as f64 * df;
            let p = c.norm_sqr();
            if f > MIN_HZ {
                total += p;
            }
            if f > STEP_HZ.0 && f < STEP_HZ.1 {
                step += p;
            }
        }
        total > 0.0 && step > STEP_SHARE * total
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run(mut magnitude: impl FnMut(f64) -> f64) -> Vec<bool> {
        let mut d = WalkingDetector::default();
        (0..400 * 12)
            .filter_map(|i| {
                let t = f64::from(i) / 400.0;
                d.on_accel((t * 1e9) as i64, magnitude(t))
            })
            .collect()
    }

    #[test]
    fn steps_two_windows_running_are_walking() {
        let walking = run(|t| 9.8 + 2.5 * (2.0 * std::f64::consts::PI * 1.9 * t).sin());
        assert_eq!(walking, [false, true]);
    }

    #[test]
    fn a_rattling_train_is_not() {
        // Broadband vibration, strong but without a step rhythm.
        let mut x: u32 = 1;
        let train = run(|_| {
            x = x.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            9.8 + (f64::from(x >> 16) / 32768.0 - 1.0) * 4.0
        });
        assert!(train.iter().all(|w| !w), "{train:?}");
    }
}
