//! Door cycles in the sound, a port of `python/experiments/stops.py`'s `door_cycles`:
//! the doors opening is a step up in the 1 to 4 kHz level (platform and HVAC noise come in),
//! and closing is a step down with the door chime in the 8 s before it.
//! On seven rides, cycles of at most [`MAX_CYCLE_S`] found 38 of 57 tapped stops and none while the train moved,
//! so a cycle means the train stood still from the opening to the closing.
//!
//! Streaming: samples go in as they're recorded, and a cycle comes out once its closing step is complete,
//! a few seconds after it. Nothing here depends on how the audio is split into chunks.

use std::collections::VecDeque;
use std::sync::Arc;

use rustfft::num_complex::Complex32;
use rustfft::{Fft, FftPlanner};

/// The level's rate (Hz), and its analysis window: 0.2 s windows every 0.1 s.
const LEVEL_RATE_HZ: f64 = 10.0;
/// The level band (Hz).
const LEVEL_BAND_HZ: (f64, f64) = (1000.0, 4000.0);
/// A step compares the mean level over this long after a point with the same before it.
const STEP_SPAN_S: f64 = 2.0;
const UP_DB: f64 = 3.0;
const DOWN_DB: f64 = 2.0;
/// Steps the same way closer than this to the last are the same step.
const STEP_SEPARATION_S: f64 = 5.0;
/// The chime: tones at these pitches (Hz; the 441/555 and 516/645 Hz families, by car type)
/// at least [`CHIME_PROMINENT_DB`] over the median of their +-[`CHIME_NEIGHBORHOOD_HZ`], for [`CHIME_MIN_S`].
const CHIME_HZ: [(f64, f64); 4] = [
    (436.0, 456.0),
    (545.0, 565.0),
    (512.0, 526.0),
    (630.0, 650.0),
];
/// Which car type's chime each of [`CHIME_HZ`] is.
const CHIME_CARS: [ChimeCars; 4] = [
    ChimeCars::Ntt,
    ChimeCars::Ntt,
    ChimeCars::R62,
    ChimeCars::R62,
];

/// The door chime's pitches tell the car type (`python/experiments/chime_omny.py`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ChimeCars {
    /// 441 and 555 Hz, heard on the 7, 2, F, M, and R: the New Technology Trains (R142, R160, R188, and kin).
    Ntt,
    /// 516 and 645 Hz, heard on the 1 (R62A) and the 3 (R62), which sound the same.
    R62,
}
const CHIME_WINDOW: usize = 2048;
const CHIME_HOP: usize = 512;
const CHIME_PROMINENT_DB: f64 = 15.0;
const CHIME_NEIGHBORHOOD_HZ: f64 = 60.0;
const CHIME_MIN_S: f64 = 0.25;
/// A closing needs a chime this long before it.
const CHIME_BEFORE_S: f64 = 8.0;
/// A cycle's opening and closing are this far apart (s): most dwells are 13 to 41 s,
/// and the longer cycles found on one ride were at speed.
const MIN_CYCLE_S: f64 = 8.0;
pub const MAX_CYCLE_S: f64 = 60.0;

/// The train standing with its doors open, from `open_ns` to `close_ns`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DoorCycle {
    pub open_ns: i64,
    pub close_ns: i64,
}

/// A spectrum of a fixed window size, Hann-windowed, as power per bin.
struct Spectrum {
    fft: Arc<dyn Fft<f32>>,
    window: Vec<f32>,
    buf: Vec<Complex32>,
    scratch: Vec<Complex32>,
}

impl Spectrum {
    fn new(n: usize) -> Self {
        let fft = FftPlanner::new().plan_fft_forward(n);
        let scratch = vec![Complex32::default(); fft.get_inplace_scratch_len()];
        // Periodic Hann, as `scipy.signal.stft`'s default.
        let window = (0..n)
            .map(|i| (0.5 - 0.5 * (2.0 * std::f64::consts::PI * i as f64 / n as f64).cos()) as f32)
            .collect();
        Self {
            fft,
            window,
            buf: vec![Complex32::default(); n],
            scratch,
        }
    }

    /// Power in bins `lo..hi` of the last `n` samples.
    fn power(&mut self, samples: impl Iterator<Item = f32>, lo: usize, hi: usize) -> Vec<f32> {
        for ((b, s), w) in self.buf.iter_mut().zip(samples).zip(&self.window) {
            *b = Complex32::new(s * w, 0.0);
        }
        self.fft
            .process_with_scratch(&mut self.buf, &mut self.scratch);
        self.buf[lo..hi].iter().map(|c| c.norm_sqr()).collect()
    }
}

/// One way's steps (up or down), found as each run of the step size over its threshold ends.
#[derive(Default)]
struct Steps {
    /// The current run's sharpest point (time, size), if in one.
    run: Option<(i64, f64)>,
    last_ns: Option<i64>,
}

impl Steps {
    /// Feeds the step size at a point; returns a step's time when a run ends.
    fn on(&mut self, t_ns: i64, size: f64, threshold: f64) -> Option<i64> {
        if size > threshold {
            // A run starting too soon after the last step is part of it.
            if self.run.is_none()
                && self
                    .last_ns
                    .is_some_and(|l| (t_ns - l) as f64 * 1e-9 < STEP_SEPARATION_S)
            {
                return None;
            }
            match &mut self.run {
                Some((_, best)) if *best >= size => {}
                run => *run = Some((t_ns, size)),
            }
            return None;
        }
        let (t, _) = self.run.take()?;
        self.last_ns = Some(t);
        Some(t)
    }
}

pub struct DoorDetector {
    rate_hz: f64,
    /// The latest (frame, elapsed realtime ns) the recorder gave, to place frames on the clock.
    clock: Option<(i64, i64)>,
    /// Recent samples, and the frame index of the first.
    samples: VecDeque<f32>,
    first_frame: i64,
    /// The next frame each analysis window starts at.
    next_level_frame: i64,
    next_chime_frame: i64,
    level_window: usize,
    level: Spectrum,
    level_bins: (usize, usize),
    chime: Spectrum,
    /// For each chime pitch bin, the bins its neighborhood median is over.
    chime_bins: Vec<(usize, usize, usize, ChimeCars)>,
    chime_range: (usize, usize),
    /// Recent levels (time, dB), enough for a step either side of a point.
    levels: VecDeque<(i64, f64)>,
    ups: Steps,
    downs: Steps,
    /// Openings not yet paired with a closing, within [`MAX_CYCLE_S`].
    open_steps: VecDeque<i64>,
    /// The current chime-like tone's start (time, frames), and recent chimes' start times and pitch families.
    tone: Option<(i64, usize)>,
    chimes: VecDeque<(i64, Option<ChimeCars>)>,
    /// The current tone's windows led by each family (NTT, R62), and the latest confirmed chime's time and car type.
    tone_votes: [usize; 2],
    last_chime: Option<(i64, ChimeCars)>,
}

impl DoorDetector {
    pub fn new(rate_hz: u32) -> Self {
        let rate = f64::from(rate_hz);
        let level_hop = (rate / LEVEL_RATE_HZ).round() as usize;
        let level_window = 2 * level_hop;
        let bin = |n: usize, hz: f64| (hz * n as f64 / rate).ceil() as usize;
        let chime_df = rate / CHIME_WINDOW as f64;
        let k = (CHIME_NEIGHBORHOOD_HZ / chime_df) as usize;
        let mut chime_bins = Vec::new();
        for i in 0..CHIME_WINDOW / 2 {
            let f = i as f64 * chime_df;
            if let Some(c) = CHIME_HZ.iter().position(|&(a, b)| f >= a && f <= b) {
                chime_bins.push((i, i.saturating_sub(k), i + k + 1, CHIME_CARS[c]));
            }
        }
        let chime_range = (
            chime_bins.iter().map(|b| b.1).min().unwrap_or(0),
            chime_bins.iter().map(|b| b.2).max().unwrap_or(0),
        );
        Self {
            rate_hz: rate,
            clock: None,
            samples: VecDeque::new(),
            first_frame: 0,
            next_level_frame: 0,
            next_chime_frame: 0,
            level_window,
            level: Spectrum::new(level_window),
            level_bins: (
                bin(level_window, LEVEL_BAND_HZ.0),
                bin(level_window, LEVEL_BAND_HZ.1),
            ),
            chime: Spectrum::new(CHIME_WINDOW),
            chime_bins,
            chime_range,
            levels: VecDeque::new(),
            ups: Steps::default(),
            downs: Steps::default(),
            open_steps: VecDeque::new(),
            tone: None,
            chimes: VecDeque::new(),
            tone_votes: [0; 2],
            last_chime: None,
        }
    }

    /// The latest door chime a closing followed: its time, and the car type its pitches tell.
    /// Only confirmed chimes count: the motor's tone sweeps through the chime pitches with speed
    /// (32.5 Hz per m/s on the R188), which made a third of the 7's status read R62.
    pub fn last_chime(&self) -> Option<(i64, ChimeCars)> {
        self.last_chime
    }

    /// Where `frame` falls on the elapsed realtime clock.
    fn time_of(&self, frame: f64) -> Option<i64> {
        let (f0, t0) = self.clock?;
        Some(t0 + ((frame - f0 as f64) / self.rate_hz * 1e9).round() as i64)
    }

    /// Mono 16-bit samples starting at stream frame `first_frame`, with any (frame, elapsed realtime ns)
    /// timestamps the recorder gave with them. Returns the cycles completed by them.
    pub fn on_audio(
        &mut self,
        first_frame: i64,
        samples: &[i16],
        timestamps: &[(i64, i64)],
    ) -> Vec<DoorCycle> {
        if let Some(&ts) = timestamps.last() {
            self.clock = Some(ts);
        }
        // A gap (frames lost) restarts the analysis windows after it.
        if first_frame != self.first_frame + self.samples.len() as i64 {
            self.samples.clear();
            self.first_frame = first_frame;
            self.next_level_frame = first_frame;
            self.next_chime_frame = first_frame;
        }
        self.samples.extend(samples.iter().map(|&s| f32::from(s)));
        let mut out = Vec::new();
        let end = self.first_frame + self.samples.len() as i64;
        let level_hop = self.level_window as i64 / 2;
        // Analyze in time order, whichever window is due first.
        loop {
            let level_due = self.next_level_frame + self.level_window as i64 <= end;
            let chime_due = self.next_chime_frame + CHIME_WINDOW as i64 <= end;
            if level_due && (!chime_due || self.next_level_frame <= self.next_chime_frame) {
                let start = self.next_level_frame;
                self.next_level_frame += level_hop;
                if let Some(cycle) = self.on_level_window(start) {
                    out.push(cycle);
                }
            } else if chime_due {
                let start = self.next_chime_frame;
                self.next_chime_frame += CHIME_HOP as i64;
                self.on_chime_window(start);
            } else {
                break;
            }
        }
        let keep_from = self.next_level_frame.min(self.next_chime_frame);
        while self.first_frame < keep_from && !self.samples.is_empty() {
            self.samples.pop_front();
            self.first_frame += 1;
        }
        out
    }

    fn window(&self, start: i64, n: usize) -> impl Iterator<Item = f32> + '_ {
        self.samples
            .range((start - self.first_frame) as usize..)
            .take(n)
            .copied()
    }

    fn on_level_window(&mut self, start: i64) -> Option<DoorCycle> {
        let t = self.time_of(start as f64 + self.level_window as f64 / 2.0)?;
        let (lo, hi) = self.level_bins;
        let n = self.level_window;
        let samples: Vec<f32> = self.window(start, n).collect();
        let p: f32 = self.level.power(samples.into_iter(), lo, hi).iter().sum();
        self.levels
            .push_back((t, 10.0 * (f64::from(p) + 1e-9).log10()));
        let span = (STEP_SPAN_S * LEVEL_RATE_HZ) as usize;
        if self.levels.len() > 2 * span {
            self.levels.pop_front();
        }
        if self.levels.len() < 2 * span {
            return None;
        }
        // The step at the middle point: the mean after it minus the mean before it.
        let (t_mid, _) = self.levels[span];
        let before: f64 = self.levels.range(..span).map(|l| l.1).sum::<f64>() / span as f64;
        let after: f64 = self.levels.range(span..).map(|l| l.1).sum::<f64>() / span as f64;
        let d = after - before;
        if let Some(up) = self.ups.on(t_mid, d, UP_DB) {
            self.open_steps.push_back(up);
        }
        while self
            .open_steps
            .front()
            .is_some_and(|&u| (t_mid - u) as f64 * 1e-9 > MAX_CYCLE_S + 10.0)
        {
            self.open_steps.pop_front();
        }
        let down = self.downs.on(t_mid, -d, DOWN_DB)?;
        let chime = self
            .chimes
            .iter()
            .rev()
            .find(|&&(c, _)| {
                let before = (down - c) as f64 * 1e-9;
                before > 0.0 && before < CHIME_BEFORE_S
            })
            .copied()?;
        if let (c, Some(cars)) = chime {
            self.last_chime = Some((c, cars));
        }
        // The earliest opening within a cycle's length: later steps up are mostly noise while the doors are open
        // (announcements, people), and in the Python scoring no opening paired this way came while moving.
        let gap = |u: i64| (down - u) as f64 * 1e-9;
        let open = self
            .open_steps
            .iter()
            .copied()
            .filter(|&u| gap(u) > MIN_CYCLE_S && gap(u) < MAX_CYCLE_S)
            .min()?;
        self.open_steps.retain(|&u| gap(u) <= MIN_CYCLE_S);
        Some(DoorCycle {
            open_ns: open,
            close_ns: down,
        })
    }

    fn on_chime_window(&mut self, start: i64) {
        let Some(t) = self.time_of(start as f64 + CHIME_WINDOW as f64 / 2.0) else {
            return;
        };
        let (lo, hi) = self.chime_range;
        let samples: Vec<f32> = self.window(start, CHIME_WINDOW).collect();
        let db: Vec<f64> = self
            .chime
            .power(samples.into_iter(), lo, hi)
            .iter()
            .map(|&p| 10.0 * (f64::from(p) + 1e-9).log10())
            .collect();
        // Each family's most prominent bin; the window counts for the family that leads.
        let mut best = [f64::NEG_INFINITY; 2];
        for &(i, a, b, cars) in &self.chime_bins {
            let mut near: Vec<f64> = db[a - lo..b - lo].to_vec();
            near.sort_by(f64::total_cmp);
            let k = cars as usize;
            best[k] = best[k].max(db[i - lo] - near[near.len() / 2]);
        }
        let tone = best.iter().any(|&p| p > CHIME_PROMINENT_DB);
        if tone {
            self.tone_votes[usize::from(best[1] > best[0])] += 1;
        }
        let min_frames = (CHIME_MIN_S * self.rate_hz / CHIME_HOP as f64).ceil() as usize;
        match (&mut self.tone, tone) {
            (Some((_, n)), true) => *n += 1,
            (None, true) => self.tone = Some((t, 1)),
            (Some((start, n)), false) => {
                if *n >= min_frames {
                    let [ntt, r62] = self.tone_votes;
                    let cars = match ntt.cmp(&r62) {
                        std::cmp::Ordering::Greater => Some(ChimeCars::Ntt),
                        std::cmp::Ordering::Less => Some(ChimeCars::R62),
                        std::cmp::Ordering::Equal => None,
                    };
                    self.chimes.push_back((*start, cars));
                }
                self.tone_votes = [0; 2];
                self.tone = None;
            }
            (None, false) => {}
        }
        while self
            .chimes
            .front()
            .is_some_and(|&(c, _)| (t - c) as f64 * 1e-9 > MAX_CYCLE_S)
        {
            self.chimes.pop_front();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Quiet noise, louder from 10 s to 40 s (doors open), with a 445 Hz chime from 35 s to 36 s.
    fn ride(rate: u32) -> Vec<i16> {
        let mut x: u32 = 1;
        (0..rate * 60)
            .map(|i| {
                let s = f64::from(i) / f64::from(rate);
                x = x.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                let noise = (f64::from(x >> 16) / 32768.0 - 1.0)
                    * if (10.0..40.0).contains(&s) {
                        3000.0
                    } else {
                        600.0
                    };
                let chime = if (35.0..36.0).contains(&s) {
                    4000.0 * (2.0 * std::f64::consts::PI * 445.0 * s).sin()
                } else {
                    0.0
                };
                (noise + chime) as i16
            })
            .collect()
    }

    #[test]
    fn finds_a_door_cycle_however_the_audio_is_split() {
        let rate = 16_000;
        let pcm = ride(rate);
        let t0 = 1_000_000_000_000;
        for read in [1600, 32_000, 777] {
            let mut d = DoorDetector::new(rate);
            let mut found = Vec::new();
            for (i, part) in pcm.chunks(read).enumerate() {
                let first = (i * read) as i64;
                found.extend(d.on_audio(first, part, &[(0, t0)]));
            }
            assert_eq!(found.len(), 1, "{read}: {found:?}");
            let s = |t: i64| (t - t0) as f64 * 1e-9;
            assert!((s(found[0].open_ns) - 10.0).abs() < 0.5, "{found:?}");
            assert!((s(found[0].close_ns) - 40.0).abs() < 0.5, "{found:?}");
        }
    }

    #[test]
    fn no_cycle_without_the_chime() {
        let rate = 16_000;
        let mut pcm = ride(rate);
        // The same, but the chime's second is the loud noise alone.
        let mut x: u32 = 7;
        for s in &mut pcm[35 * 16_000..36 * 16_000] {
            x = x.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            *s = ((f64::from(x >> 16) / 32768.0 - 1.0) * 3000.0) as i16;
        }
        let mut d = DoorDetector::new(rate);
        assert!(d.on_audio(0, &pcm, &[(0, 0)]).is_empty());
    }
}
