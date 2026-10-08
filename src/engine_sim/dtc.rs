//! SAE J2012 / ISO 15031-6 diagnostic trouble codes emitted by the emulated ECU.

/// Diagnostic trouble codes the ECU can store. Codes and meanings follow SAE J2012 so that
/// what students learn here transfers directly to a real scan tool.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum DtcCode {
    /// Crankshaft/camshaft position correlation (bank 1, sensor A).
    P0016 = 0,
    /// Fuel rail/system pressure too low.
    P0087,
    /// Manifold absolute pressure circuit range/performance.
    P0106,
    /// Intake air temperature circuit high input.
    P0113,
    /// Coolant temperature below thermostat regulating temperature.
    P0128,
    /// O2 sensor circuit slow response (bank 1, sensor 1).
    P0133,
    /// System too lean (bank 1).
    P0171,
    /// System too rich (bank 1).
    P0172,
    /// Engine coolant over-temperature condition.
    P0217,
    /// Engine overspeed condition.
    P0219,
    /// Turbocharger overboost condition.
    P0234,
    /// Turbocharger underboost condition.
    P0299,
    /// Random/multiple cylinder misfire detected.
    P0300,
    /// Cylinder 1 misfire detected.
    P0301,
    /// Cylinder 2 misfire detected.
    P0302,
    /// Cylinder 3 misfire detected.
    P0303,
    /// Cylinder 4 misfire detected.
    P0304,
    /// Cylinder 5 misfire detected.
    P0305,
    /// Cylinder 6 misfire detected.
    P0306,
    /// Cylinder 7 misfire detected.
    P0307,
    /// Cylinder 8 misfire detected.
    P0308,
    /// Knock sensor 1 circuit malfunction.
    P0325,
    /// Catalyst system efficiency below threshold (bank 1).
    P0420,
    /// Idle air control system RPM higher than expected.
    P0507,
    /// Engine oil pressure too low.
    P0524,
}

impl DtcCode {
    /// Number of defined codes.
    pub const COUNT: usize = 25;

    /// Every code in storage order.
    pub const ALL: [DtcCode; Self::COUNT] = [
        DtcCode::P0016,
        DtcCode::P0087,
        DtcCode::P0106,
        DtcCode::P0113,
        DtcCode::P0128,
        DtcCode::P0133,
        DtcCode::P0171,
        DtcCode::P0172,
        DtcCode::P0217,
        DtcCode::P0219,
        DtcCode::P0234,
        DtcCode::P0299,
        DtcCode::P0300,
        DtcCode::P0301,
        DtcCode::P0302,
        DtcCode::P0303,
        DtcCode::P0304,
        DtcCode::P0305,
        DtcCode::P0306,
        DtcCode::P0307,
        DtcCode::P0308,
        DtcCode::P0325,
        DtcCode::P0420,
        DtcCode::P0507,
        DtcCode::P0524,
    ];

    /// Five-character code as shown by a scan tool.
    pub fn code(&self) -> &'static str {
        match self {
            Self::P0016 => "P0016",
            Self::P0087 => "P0087",
            Self::P0106 => "P0106",
            Self::P0113 => "P0113",
            Self::P0128 => "P0128",
            Self::P0133 => "P0133",
            Self::P0171 => "P0171",
            Self::P0172 => "P0172",
            Self::P0217 => "P0217",
            Self::P0219 => "P0219",
            Self::P0234 => "P0234",
            Self::P0299 => "P0299",
            Self::P0300 => "P0300",
            Self::P0301 => "P0301",
            Self::P0302 => "P0302",
            Self::P0303 => "P0303",
            Self::P0304 => "P0304",
            Self::P0305 => "P0305",
            Self::P0306 => "P0306",
            Self::P0307 => "P0307",
            Self::P0308 => "P0308",
            Self::P0325 => "P0325",
            Self::P0420 => "P0420",
            Self::P0507 => "P0507",
            Self::P0524 => "P0524",
        }
    }

    /// Cylinder-specific misfire code for a zero-based cylinder index.
    pub fn misfire_for_cylinder(cylinder: usize) -> Option<DtcCode> {
        const MISFIRE: [DtcCode; 8] = [
            DtcCode::P0301,
            DtcCode::P0302,
            DtcCode::P0303,
            DtcCode::P0304,
            DtcCode::P0305,
            DtcCode::P0306,
            DtcCode::P0307,
            DtcCode::P0308,
        ];
        MISFIRE.get(cylinder).copied()
    }

    #[inline]
    pub(crate) fn index(self) -> usize {
        self as usize
    }
}

/// Stored trouble codes (bit set, allocation-free).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct DtcSet(u32);

impl DtcSet {
    /// Membership test.
    #[inline]
    pub fn contains(&self, code: DtcCode) -> bool {
        self.0 & (1 << code.index()) != 0
    }

    /// Inserts a code; returns `true` if it was not stored before.
    #[inline]
    pub(crate) fn insert(&mut self, code: DtcCode) -> bool {
        let new = !self.contains(code);
        self.0 |= 1 << code.index();
        new
    }

    /// Number of stored codes.
    #[inline]
    pub fn len(&self) -> usize {
        self.0.count_ones() as usize
    }

    /// `true` when no code is stored.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.0 == 0
    }

    /// Iterates stored codes without allocating.
    pub fn iter(&self) -> impl Iterator<Item = DtcCode> + '_ {
        DtcCode::ALL
            .iter()
            .copied()
            .filter(move |c| self.contains(*c))
    }

    pub(crate) fn clear(&mut self) {
        self.0 = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn codes_are_indexed_consistently() {
        for (i, c) in DtcCode::ALL.iter().enumerate() {
            assert_eq!(c.index(), i);
        }
        assert_eq!(DtcCode::misfire_for_cylinder(2), Some(DtcCode::P0303));
        assert_eq!(DtcCode::misfire_for_cylinder(8), None);
    }
}
