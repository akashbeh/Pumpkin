use log::log;
use pumpkin_inventory::InventoryError;
use pumpkin_protocol::ser::ReadingError;
use pumpkin_world::data::player_data::PlayerDataError;
use std::fmt::Display;

pub trait PumpkinError: Send + std::error::Error + Display {
    fn is_kick(&self) -> bool;

    fn log(&self) {
        log!(self.severity(), "{self}");
    }

    fn severity(&self) -> log::Level;

    fn client_kick_reason(&self) -> Option<String>;
}

impl<ErrorType: PumpkinError + 'static> From<ErrorType> for Box<dyn PumpkinError> {
    fn from(error: ErrorType) -> Self {
        Box::new(error)
    }
}
impl PumpkinError for InventoryError {
    fn is_kick(&self) -> bool {
        use InventoryError::{
            ClosedContainerInteract, InvalidPacket, InvalidSlot, LockError,
            MultiplePlayersDragging, OutOfOrderDragging, PermissionError,
        };
        match self {
            InvalidSlot | ClosedContainerInteract(..) | InvalidPacket | PermissionError => true,
            LockError | OutOfOrderDragging | MultiplePlayersDragging => false,
        }
    }
    fn severity(&self) -> log::Level {
        use InventoryError::{
            ClosedContainerInteract, InvalidPacket, InvalidSlot, LockError,
            MultiplePlayersDragging, OutOfOrderDragging, PermissionError,
        };
        match self {
            LockError
            | InvalidSlot
            | ClosedContainerInteract(..)
            | InvalidPacket
            | PermissionError => log::Level::Error,
            OutOfOrderDragging => log::Level::Info,
            MultiplePlayersDragging => log::Level::Warn,
        }
    }

    fn client_kick_reason(&self) -> Option<String> {
        None
    }
}

impl PumpkinError for ReadingError {
    fn is_kick(&self) -> bool {
        true
    }

    fn severity(&self) -> log::Level {
        log::Level::Error
    }

    fn client_kick_reason(&self) -> Option<String> {
        None
    }
}

impl PumpkinError for PlayerDataError {
    fn is_kick(&self) -> bool {
        false
    }

    fn severity(&self) -> log::Level {
        log::Level::Warn
    }

    fn client_kick_reason(&self) -> Option<String> {
        match self {
            Self::Io(err) => Some(format!("Failed to load player data: {err}")),
            Self::Nbt(err) => Some(format!("Failed to parse player data: {err}")),
        }
    }
}
