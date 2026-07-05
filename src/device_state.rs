//! Device state machine (`DeviceState` / `DeviceEvent`).

// Device State Machine
// ---------------------------------------------------------------------------

/// State of a `LoRaWAN` end device.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DeviceState {
    Idle,
    Transmitting,
    WaitingRx1,
    WaitingRx2,
    Receiving,
    Processing,
    Sleep,
}

impl DeviceState {
    /// Whether the device is in a receive-capable state.
    #[must_use]
    pub const fn can_receive(self) -> bool {
        matches!(self, Self::WaitingRx1 | Self::WaitingRx2 | Self::Receiving)
    }

    /// Whether the device is idle or sleeping.
    #[must_use]
    pub const fn is_low_power(self) -> bool {
        matches!(self, Self::Idle | Self::Sleep)
    }
}

/// Transition the device state machine for Class A.
#[must_use]
pub const fn class_a_next_state(current: DeviceState, event: DeviceEvent) -> DeviceState {
    match (current, event) {
        (DeviceState::Idle, DeviceEvent::TxRequest) => DeviceState::Transmitting,
        (DeviceState::Transmitting, DeviceEvent::TxComplete) => DeviceState::WaitingRx1,
        (DeviceState::WaitingRx1, DeviceEvent::Rx1Open) => DeviceState::Receiving,
        (DeviceState::WaitingRx1, DeviceEvent::Rx1Timeout) => DeviceState::WaitingRx2,
        (DeviceState::WaitingRx2, DeviceEvent::Rx2Open) => DeviceState::Receiving,
        (DeviceState::WaitingRx2, DeviceEvent::Rx2Timeout) => DeviceState::Idle,
        (DeviceState::Receiving, DeviceEvent::RxSuccess) => DeviceState::Processing,
        (DeviceState::Receiving, DeviceEvent::RxFail) => {
            // If was in RX1, go to WaitingRx2; simplified: go idle
            DeviceState::Idle
        }
        (DeviceState::Processing, DeviceEvent::ProcessComplete) => DeviceState::Idle,
        (DeviceState::Idle, DeviceEvent::GoSleep) => DeviceState::Sleep,
        (DeviceState::Sleep, DeviceEvent::WakeUp) => DeviceState::Idle,
        _ => current,
    }
}

/// Events driving the device state machine.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DeviceEvent {
    TxRequest,
    TxComplete,
    Rx1Open,
    Rx1Timeout,
    Rx2Open,
    Rx2Timeout,
    RxSuccess,
    RxFail,
    ProcessComplete,
    GoSleep,
    WakeUp,
}
