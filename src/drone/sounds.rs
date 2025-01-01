use crate::RustBustersDrone;
use log::warn;
use rodio::{Decoder, Sink};
use std::io::Cursor;

#[cfg(feature = "sounds")]
pub(crate) const SPAWN_SOUND: &[u8] = include_bytes!("../../sfx/spawn.mp3");
#[cfg(feature = "sounds")]
pub(crate) const CRASH_SOUND: &[u8] = include_bytes!("../../sfx/crash.mp3");
#[cfg(feature = "sounds")]
pub(crate) const NACK_SOUND: &[u8] = include_bytes!("../../sfx/nack.mp3");
#[cfg(feature = "sounds")]
pub(crate) const DROP_SOUND: &[u8] = include_bytes!("../../sfx/drop.mp3");
#[cfg(feature = "sounds")]
pub(crate) const HUNT_SOUND: &[u8] = include_bytes!("../../sfx/hunt.mp3");

impl RustBustersDrone {
    #[cfg(feature = "sounds")]
    pub(crate) fn play_sound(&self, sound: &'static [u8]) {
        if let Some((_stream, handle)) = &self.sound_sys {
            if let Ok(sink) = Sink::try_new(handle) {
                let cursor = Cursor::new(sound);
                if let Ok(source) = Decoder::new(cursor) {
                    sink.append(source);
                    sink.detach();
                } else {
                    warn!(
                        "Error - Failed to decoded audio file"
                    );
                }
            } else {
                warn!(
                    "Error - Failed to create audio sink"
                );
            }
        }
    }
}
