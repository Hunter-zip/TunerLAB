//! Thin `cpal` wrapper: opens the default output device and runs the synthesizer in its
//! callback.
//!
//! The callback never blocks: commands arrive through a bounded lock-free channel drained
//! with `try_iter`, the clock and health flags are atomics, and the synthesizer renders a
//! bounded amount of work into a fixed buffer.

use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering::Relaxed};
use std::sync::mpsc::{sync_channel, SyncSender, TrySendError};
use std::sync::Arc;

use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use cpal::{BufferSize, ErrorKind, FromSample, SampleFormat, SizedSample, StreamConfig};

use super::synth::{Command, Synth};

/// Commands the channel holds; at 5× speed one frame brings well under a hundred.
const QUEUE: usize = 1024;
/// Output periods per second requested when the driver's own buffer size is refused: 20 ms.
/// The driver default comes first: a forced short buffer underruns (audible dropouts) on
/// busy machines and on many Windows devices.
const PERIODS_PER_S: u32 = 50;
/// Samples rendered at once inside the callback.
const CHUNK: usize = 512;

/// State the callback shares with the UI thread.
#[derive(Debug, Default)]
struct Shared {
    /// Samples rendered so far.
    clock: AtomicU64,
    /// Frames of the last callback.
    period: AtomicU32,
    /// The stream reported a fatal error.
    failed: AtomicBool,
}

/// A running output stream.
pub struct Output {
    _stream: cpal::Stream,
    tx: SyncSender<Command>,
    shared: Arc<Shared>,
    sample_rate: u32,
}

impl Output {
    /// Opens the default output device, or `None` when there is none or it refuses every
    /// configuration tried.
    pub fn open() -> Option<Self> {
        let device = cpal::default_host().default_output_device()?;
        let supported = device.default_output_config().ok()?;
        let format = supported.sample_format();
        let mut config = supported.config();
        let wanted = (config.sample_rate / PERIODS_PER_S).max(64);
        let fixed = match supported.buffer_size() {
            cpal::SupportedBufferSize::Range { min, max } if min <= max => {
                Some(BufferSize::Fixed(wanted.clamp(*min, *max)))
            }
            _ => None,
        };
        for buffer in [BufferSize::Default].into_iter().chain(fixed) {
            config.buffer_size = buffer;
            let built = match format {
                SampleFormat::F32 => build::<f32>(&device, &config),
                SampleFormat::I16 => build::<i16>(&device, &config),
                SampleFormat::U16 => build::<u16>(&device, &config),
                SampleFormat::I32 => build::<i32>(&device, &config),
                _ => None,
            };
            if built.is_some() {
                return built;
            }
        }
        None
    }

    pub fn sample_rate(&self) -> u32 {
        self.sample_rate
    }

    /// Samples the callback has rendered so far.
    pub fn clock(&self) -> u64 {
        self.shared.clock.load(Relaxed)
    }

    /// Length of the last callback period \[s\] (zero before the first one).
    pub fn period_s(&self) -> f64 {
        f64::from(self.shared.period.load(Relaxed)) / f64::from(self.sample_rate.max(1))
    }

    /// The stream is running (no fatal error reported).
    pub fn is_alive(&self) -> bool {
        !self.shared.failed.load(Relaxed)
    }

    /// Queues a command; when the queue is full the command is dropped, never waited for.
    pub fn send(&self, command: Command) {
        match self.tx.try_send(command) {
            Ok(()) | Err(TrySendError::Full(_)) => {}
            Err(TrySendError::Disconnected(_)) => self.shared.failed.store(true, Relaxed),
        }
    }
}

fn build<T>(device: &cpal::Device, config: &StreamConfig) -> Option<Output>
where
    T: SizedSample + FromSample<f32>,
{
    let (tx, rx) = sync_channel::<Command>(QUEUE);
    let shared = Arc::new(Shared::default());
    let mut synth = Synth::new(config.sample_rate as f32);
    let channels = usize::from(config.channels.max(1));
    let mut mono = [0.0_f32; CHUNK];
    let callback_shared = Arc::clone(&shared);
    let data = move |data: &mut [T], _: &cpal::OutputCallbackInfo| {
        for command in rx.try_iter().take(QUEUE) {
            synth.apply(command);
        }
        for chunk in data.chunks_mut(channels * CHUNK) {
            let frames = chunk.len() / channels;
            synth.render(&mut mono[..frames]);
            for (frame, &s) in chunk.chunks_mut(channels).zip(&mono[..frames]) {
                let v = T::from_sample(s);
                frame.fill(v);
            }
        }
        callback_shared.clock.store(synth.clock(), Relaxed);
        let period = u32::try_from(data.len() / channels).unwrap_or(u32::MAX);
        callback_shared.period.store(period, Relaxed);
    };
    let error_shared = Arc::clone(&shared);
    let error = move |err: cpal::Error| match err.kind() {
        // Recoverable: the stream keeps running.
        ErrorKind::Xrun | ErrorKind::DeviceChanged | ErrorKind::RealtimeDenied => {}
        _ => {
            // Report once; the UI then shows sound as unavailable.
            if !error_shared.failed.swap(true, Relaxed) {
                eprintln!("TunerLAB: audio output stopped: {err}");
            }
        }
    };
    let stream = device
        .build_output_stream(*config, data, error, None)
        .ok()?;
    stream.play().ok()?;
    Some(Output {
        _stream: stream,
        tx,
        shared,
        sample_rate: config.sample_rate,
    })
}
