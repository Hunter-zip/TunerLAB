//! Fixed-capacity event stores: the diagnostic log (for the UI / course engine) and the
//! per-combustion event ring (for the Phase 2 procedural audio engine).

use super::dtc::DtcCode;
use super::status::{EngineCondition, FailureCause, Warning};

/// Capacity of the diagnostic event log.
pub const EVENT_LOG_CAPACITY: usize = 64;
/// Capacity of the combustion event ring. At 8000 rpm a V8 fires 533 times per second, so
/// 128 slots cover > 200 ms – far more than one audio buffer period.
pub const CYLINDER_EVENT_CAPACITY: usize = 128;

/// What happened.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EventKind {
    /// Operating condition changed.
    ConditionChanged(EngineCondition),
    /// Instructor warning became active.
    Warning(Warning),
    /// The ECU stored a diagnostic trouble code.
    Dtc(DtcCode),
    /// Mechanical damage event.
    Damage(FailureCause),
}

/// Time-stamped diagnostic event.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DiagnosticEvent {
    /// Simulation time \[s\].
    pub time_s: f64,
    /// Monotonic sequence number.
    pub seq: u64,
    /// Event payload.
    pub kind: EventKind,
}

/// Ring buffer of the most recent diagnostic events.
///
/// Sequence numbers are monotonic for the lifetime of the simulation, including across
/// [`EngineSim::reset`](super::EngineSim::reset), so a consumer's stored cursor stays valid:
/// a reset only discards the events logged before it.
#[derive(Debug, Clone)]
pub struct EventLog {
    buf: [DiagnosticEvent; EVENT_LOG_CAPACITY],
    next_seq: u64,
    /// Events with a lower sequence number were discarded by a reset.
    first_seq: u64,
}

impl Default for EventLog {
    fn default() -> Self {
        Self {
            buf: [DiagnosticEvent {
                time_s: 0.0,
                seq: 0,
                kind: EventKind::ConditionChanged(EngineCondition::Off),
            }; EVENT_LOG_CAPACITY],
            next_seq: 0,
            first_seq: 0,
        }
    }
}

impl EventLog {
    pub(crate) fn push(&mut self, time_s: f64, kind: EventKind) {
        let idx = (self.next_seq % EVENT_LOG_CAPACITY as u64) as usize;
        self.buf[idx] = DiagnosticEvent {
            time_s,
            seq: self.next_seq,
            kind,
        };
        self.next_seq += 1;
    }

    /// Sequence number the next event will receive. Store it and pass it to
    /// [`since`](Self::since) to poll incrementally.
    pub fn next_seq(&self) -> u64 {
        self.next_seq
    }

    /// Events with `seq >= from_seq` still held in the ring, oldest first.
    pub fn since(&self, from_seq: u64) -> impl Iterator<Item = &DiagnosticEvent> + '_ {
        let oldest = self
            .next_seq
            .saturating_sub(EVENT_LOG_CAPACITY as u64)
            .max(self.first_seq);
        let start = from_seq.max(oldest);
        (start..self.next_seq).map(move |s| &self.buf[(s % EVENT_LOG_CAPACITY as u64) as usize])
    }

    /// All retained events, oldest first.
    pub fn iter(&self) -> impl Iterator<Item = &DiagnosticEvent> + '_ {
        self.since(0)
    }

    /// Discards every retained event; sequence numbers keep counting.
    pub(crate) fn clear(&mut self) {
        self.first_seq = self.next_seq;
    }
}

/// One combustion (or misfire) event, published for audio synthesis and cycle analysis.
///
/// The closed cycle is computed, and the event emitted, at the **start of the compression
/// stroke** (bottom dead centre). The combustion it describes happens later:
/// [`time_to_tdc_s`](Self::time_to_tdc_s) gives the delay to firing TDC and
/// [`time_to_evo_s`](Self::time_to_evo_s) the delay to exhaust valve opening (the exhaust
/// pulse), both at the engine speed of the event. Schedule sounds with these offsets.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct CylinderEvent {
    /// Monotonic sequence number.
    pub seq: u64,
    /// Simulation time at which the cycle was computed (start of compression) \[s\].
    pub time_s: f64,
    /// Delay from `time_s` to this cylinder's firing TDC \[s\].
    pub time_to_tdc_s: f32,
    /// Delay from `time_s` to exhaust valve opening (start of the exhaust pulse) \[s\].
    pub time_to_evo_s: f32,
    /// Zero-based cylinder index.
    pub cylinder: u8,
    /// Engine speed at the event \[rpm\].
    pub rpm: f32,
    /// Peak cylinder pressure \[bar\].
    pub peak_pressure_bar: f32,
    /// Crank angle of peak pressure [deg ATDC].
    pub peak_pressure_angle_deg: f32,
    /// Knock intensity (pressure-oscillation amplitude equivalent) \[bar\].
    pub knock_intensity: f32,
    /// No combustion took place.
    pub misfire: bool,
    /// Net indicated work of the closed cycle \[J\].
    pub indicated_work_j: f32,
    /// Mass-averaged exhaust gas temperature at the exhaust valve, before the port heat
    /// loss \[K\].
    pub exhaust_temp_k: f32,
    /// Energy released by afterburning in the exhaust manifold \[J\] ("pops & bangs").
    pub afterburn_j: f32,
    /// Blow-down pressure at exhaust valve opening \[bar\] (exhaust pulse amplitude).
    pub blowdown_pressure_bar: f32,
}

/// Ring buffer of recent combustion events. Sequence numbers are monotonic across resets,
/// as for [`EventLog`].
#[derive(Debug, Clone)]
pub struct CylinderEventRing {
    buf: [CylinderEvent; CYLINDER_EVENT_CAPACITY],
    next_seq: u64,
    first_seq: u64,
}

impl Default for CylinderEventRing {
    fn default() -> Self {
        Self {
            buf: [CylinderEvent::default(); CYLINDER_EVENT_CAPACITY],
            next_seq: 0,
            first_seq: 0,
        }
    }
}

impl CylinderEventRing {
    pub(crate) fn push(&mut self, mut ev: CylinderEvent) {
        ev.seq = self.next_seq;
        let idx = (self.next_seq % CYLINDER_EVENT_CAPACITY as u64) as usize;
        self.buf[idx] = ev;
        self.next_seq += 1;
    }

    /// Sequence number the next event will receive.
    pub fn next_seq(&self) -> u64 {
        self.next_seq
    }

    /// Events with `seq >= from_seq` still held in the ring, oldest first.
    pub fn since(&self, from_seq: u64) -> impl Iterator<Item = &CylinderEvent> + '_ {
        let oldest = self
            .next_seq
            .saturating_sub(CYLINDER_EVENT_CAPACITY as u64)
            .max(self.first_seq);
        let start = from_seq.max(oldest);
        (start..self.next_seq)
            .map(move |s| &self.buf[(s % CYLINDER_EVENT_CAPACITY as u64) as usize])
    }

    /// All retained events, oldest first.
    pub fn iter(&self) -> impl Iterator<Item = &CylinderEvent> + '_ {
        self.since(0)
    }

    /// Discards every retained event; sequence numbers keep counting.
    pub(crate) fn clear(&mut self) {
        self.first_seq = self.next_seq;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn log_wraps_and_keeps_newest() {
        let mut log = EventLog::default();
        for i in 0..(EVENT_LOG_CAPACITY as u64 + 10) {
            log.push(i as f64, EventKind::Warning(Warning::Knock));
        }
        let v: Vec<u64> = log.iter().map(|e| e.seq).collect();
        assert_eq!(v.len(), EVENT_LOG_CAPACITY);
        assert_eq!(v[0], 10);
        assert_eq!(*v.last().unwrap(), EVENT_LOG_CAPACITY as u64 + 9);
        assert_eq!(log.since(EVENT_LOG_CAPACITY as u64 + 8).count(), 2);
    }

    #[test]
    fn clear_keeps_sequence_monotonic() {
        let mut log = EventLog::default();
        for i in 0..5 {
            log.push(i as f64, EventKind::Warning(Warning::Knock));
        }
        let cursor = log.next_seq();
        log.clear();
        assert_eq!(log.iter().count(), 0);
        log.push(9.0, EventKind::Warning(Warning::Misfire));
        let new: Vec<u64> = log.since(cursor).map(|e| e.seq).collect();
        assert_eq!(new, vec![5]);

        let mut ring = CylinderEventRing::default();
        for _ in 0..3 {
            ring.push(CylinderEvent::default());
        }
        let cursor = ring.next_seq();
        ring.clear();
        ring.push(CylinderEvent::default());
        assert_eq!(ring.since(cursor).count(), 1);
        assert_eq!(ring.iter().count(), 1);
    }
}
