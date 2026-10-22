//! Walking, from the step rhythm in the accelerometer, a port of `python/experiments/sensor_trips.py`'s `walking`:
//! in each [`WINDOW_S`] window, |a| shakes by over [`MIN_STD_MPS2`], mostly at step rates.
//! On seven rides, two such windows in a row came 1489 times, 5 of them aboard a train, none while it moved
//! (GNSS speed, where confident). The rider sometimes walks a few cars on a moving train, though, which would pull its
//! speed down to a walk, so it takes [`IN_A_ROW`] windows (24 s): longer than a walk through a car or two,
//! shorter than a transfer or a walk on the street.

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
pub const IN_A_ROW: u32 = 6;
/// Running: a faster rhythm (Hz) and harder impacts (|a| std, m/s^2) than walking.
const RUN_HZ: (f64, f64) = (2.6, 3.8);
const RUN_MIN_STD_MPS2: f64 = 3.0;

/// What the last window's rhythm was.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Gait {
    Walking,
    Running,
}

#[derive(Default)]
pub struct WalkingDetector {
    window_start_ns: Option<i64>,
    mags: Vec<f64>,
    in_a_row: u32,
    planner: Option<(usize, Arc<dyn Fft<f64>>)>,
    /// The last window's end and its gait, if it had one.
    last: Option<(i64, Option<Gait>)>,
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
        let gait = self.gait();
        self.last = Some((t_ns, gait));
        let steps = gait.is_some();
        self.in_a_row = if steps { self.in_a_row + 1 } else { 0 };
        // Windows stay on a fixed grid from the first sample, as in Python.
        let windows = ((t_ns - start) as f64 / (WINDOW_S * 1e9)).floor() as i64;
        self.window_start_ns = Some(start + windows * (WINDOW_S * 1e9) as i64);
        self.mags.clear();
        self.mags.push(magnitude);
        Some(self.in_a_row >= IN_A_ROW)
    }

    /// The last window's end and the gait in it, if any.
    pub fn last_gait(&self) -> Option<(i64, Option<Gait>)> {
        self.last
    }

    fn gait(&mut self) -> Option<Gait> {
        let n = self.mags.len();
        if n < MIN_SAMPLES {
            return None;
        }
        let mean = self.mags.iter().sum::<f64>() / n as f64;
        let var = self.mags.iter().map(|m| (m - mean).powi(2)).sum::<f64>() / n as f64;
        let std = var.sqrt();
        if std <= MIN_STD_MPS2 {
            return None;
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
        let (mut step, mut run, mut total) = (0.0, 0.0, 0.0);
        for (k, c) in buf.iter().take(n / 2 + 1).enumerate() {
            let f = k as f64 * df;
            let p = c.norm_sqr();
            if f > MIN_HZ {
                total += p;
            }
            if f > STEP_HZ.0 && f < STEP_HZ.1 {
                step += p;
            }
            if f >= RUN_HZ.0 && f < RUN_HZ.1 {
                run += p;
            }
        }
        if total <= 0.0 {
            None
        } else if std > RUN_MIN_STD_MPS2 && run > STEP_SHARE * total && run > step {
            Some(Gait::Running)
        } else if step > STEP_SHARE * total {
            Some(Gait::Walking)
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run(mut magnitude: impl FnMut(f64) -> f64) -> Vec<bool> {
        let mut d = WalkingDetector::default();
        (0..400 * 25)
            .filter_map(|i| {
                let t = f64::from(i) / 400.0;
                d.on_accel((t * 1e9) as i64, magnitude(t))
            })
            .collect()
    }

    #[test]
    fn steps_for_24_s_are_walking() {
        let walking = run(|t| 9.8 + 2.5 * (2.0 * std::f64::consts::PI * 1.9 * t).sin());
        assert_eq!(walking, [false, false, false, false, false, true]);
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
