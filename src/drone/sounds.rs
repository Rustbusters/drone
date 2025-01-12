#[cfg(feature = "sounds")]
pub mod sounds_feat {
    use crate::RustBustersDrone;
    use crossbeam_channel::{unbounded, Receiver, Sender};
    use log::warn;

    use lazy_static::lazy_static;
    use rodio::{Decoder, OutputStream, OutputStreamHandle, Sink};
    use std::io::Cursor;

    pub(crate) const SPAWN_SOUND: &[u8] = include_bytes!("../../sfx/spawn.mp3");
    pub(crate) const CRASH_SOUND: &[u8] = include_bytes!("../../sfx/crash.mp3");
    pub(crate) const NACK_SOUND: &[u8] = include_bytes!("../../sfx/nack.mp3");
    pub(crate) const DROP_SOUND: &[u8] = include_bytes!("../../sfx/drop.mp3");
    pub(crate) const HUNT_SOUND: &[u8] = include_bytes!("../../sfx/hunt.mp3");

    lazy_static! {
        pub(crate) static ref SOUND_SYS: Option<ThreadSafeAudio> = None;
    }

    impl RustBustersDrone {
        pub(crate) fn play_sound(&self, sound: &'static [u8]) {
            if let Some(sound_sys) = &self.sound_sys {
                sound_sys.play_sound(sound);
            } else {
                warn!("Error - Sound system not initialized");
            }
        }
    }

    pub enum AudioCommand {
        PlaySound(&'static [u8]),
    }

    #[derive(Clone)]
    pub struct ThreadSafeAudio {
        command_sender: Sender<AudioCommand>,
    }

    impl ThreadSafeAudio {
        pub fn new() -> Self {
            if let Some(sound_sys) = &*SOUND_SYS {
                sound_sys.clone()
            } else {
                let (tx, rx) = unbounded::<AudioCommand>();

                // Spawn del thread audio dedicato
                std::thread::spawn(move || {
                    if let Ok((stream, handle)) = OutputStream::try_default() {
                        Self::audio_thread_loop(&rx, &stream, &handle);
                    }
                });

                Self { command_sender: tx }
            }
        }

        fn audio_thread_loop(
            receiver: &Receiver<AudioCommand>,
            _stream: &OutputStream,
            handle: &OutputStreamHandle,
        ) {
            while let Ok(cmd) = receiver.recv() {
                match cmd {
                    AudioCommand::PlaySound(sound_data) => {
                        // Logica per riprodurre il suono usando handle
                        if let Ok(sink) = Sink::try_new(handle) {
                            let cursor = Cursor::new(sound_data);
                            if let Ok(source) = Decoder::new(cursor) {
                                sink.append(source);
                                sink.detach();
                            } else {
                                warn!("Error - Failed to decoded audio file");
                            }
                        } else {
                            warn!("Error - Failed to create audio sink");
                        }
                    }
                }
            }
        }

        pub fn play_sound(&self, sound_data: &'static [u8]) {
            if let Err(e) = self
                .command_sender
                .send(AudioCommand::PlaySound(sound_data))
            {
                warn!("Error - Failed to send audio command: {:?}", e);
            }
        }
    }
}
