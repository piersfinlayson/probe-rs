//! STMicroelectronics vendor support.

use probe_rs_target::Chip;

use crate::{
    config::DebugSequence,
    vendor::{
        Vendor,
        st::sequences::{
            stm32_armv6::{Stm32Armv6, Stm32Armv6Family},
            stm32_armv7::Stm32Armv7,
            stm32_armv8::Stm32Armv8,
            stm32h7::{Stm32h7, Stm32h7Line},
            stm32n6::Stm32n6,
        },
    },
};

pub mod sequences;

/// STMicroelectronics
#[derive(docsplay::Display)]
pub struct St;

impl Vendor for St {
    fn try_create_debug_sequence(&self, chip: &Chip) -> Option<DebugSequence> {
        let sequence = if chip.name.starts_with("STM32F0") {
            DebugSequence::Arm(Stm32Armv6::create(Stm32Armv6Family::F0))
        } else if chip.name.starts_with("STM32L0") {
            DebugSequence::Arm(Stm32Armv6::create(Stm32Armv6Family::L0))
        } else if chip.name.starts_with("STM32G0") {
            DebugSequence::Arm(Stm32Armv6::create(Stm32Armv6Family::G0))
        } else if chip.name.starts_with("STM32F1")
            || chip.name.starts_with("STM32F2")
            || chip.name.starts_with("STM32F3")
            || chip.name.starts_with("STM32F4")
            || chip.name.starts_with("STM32F7")
            || chip.name.starts_with("STM32G4")
            || chip.name.starts_with("STM32L1")
            || chip.name.starts_with("STM32L4")
            || chip.name.starts_with("STM32WB")
            || chip.name.starts_with("STM32WL")
        {
            DebugSequence::Arm(Stm32Armv7::create())
        } else if chip.name.starts_with("STM32H7S") || chip.name.starts_with("STM32H7R") {
            DebugSequence::Arm(Stm32h7::create(Stm32h7Line::H7S))
        } else if chip.name.starts_with("STM32H7") {
            DebugSequence::Arm(Stm32h7::create(Stm32h7Line::H7))
        } else if chip.name.starts_with("STM32H5")
            || chip.name.starts_with("STM32L5")
            || chip.name.starts_with("STM32U5")
        {
            DebugSequence::Arm(Stm32Armv8::create())
        } else if chip.name.starts_with("STM32N6") {
            DebugSequence::Arm(Stm32n6::create())
        } else {
            return None;
        };

        Some(sequence)
    }
}
