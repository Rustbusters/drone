#[cfg(feature = "sounds")]
pub mod sounds_feat {
    use crossbeam_channel::{unbounded, Receiver, Sender};
    use lazy_static::lazy_static;
    use log::warn;
    use rodio::{Decoder, OutputStream, OutputStreamHandle, Sink};
    use std::io::Cursor;
    use std::sync::Mutex;

    pub(crate) const SPAWN_SOUND: &[u8] = include_bytes!("../../sfx/spawn.mp3");
    pub(crate) const CRASH_SOUND: &[u8] = include_bytes!("../../sfx/crash.mp3");
    pub(crate) const NACK_SOUND: &[u8] = include_bytes!("../../sfx/nack.mp3");
    pub(crate) const DROP_SOUND: &[u8] = include_bytes!("../../sfx/drop.mp3");
    pub(crate) const HUNT_SOUND: &[u8] = include_bytes!("../../sfx/hunt.mp3");

    lazy_static! {
        static ref AUDIO_SYSTEM: Mutex<AudioSystem> = Mutex::new(AudioSystem::new());
    }

    pub struct AudioSystem {
        command_sender: Option<Sender<AudioCommand>>,
    }

    impl AudioSystem {
        fn new() -> Self {
            let (tx, rx) = unbounded::<AudioCommand>();
            
            // Spawn del thread audio dedicato una sola volta
            std::thread::Builder::new()
                .name("audio-thread".to_string())
                .spawn(move || {
                    if let Ok((stream, handle)) = OutputStream::try_default() {
                        Self::audio_thread_loop(&rx, &stream, &handle);
                    }
                })
                .expect("Failed to spawn audio thread");

            Self {
                command_sender: Some(tx),
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
                        if let Ok(sink) = Sink::try_new(handle) {
                            let cursor = Cursor::new(sound_data);
                            if let Ok(source) = Decoder::new(cursor) {
                                sink.append(source);
                                sink.detach();
                            } else {
                                warn!("Error - Failed to decode audio file");
                            }
                        } else {
                            warn!("Error - Failed to create audio sink");
                        }
                    }
                }
            }
        }

        pub fn play_sound(&self, sound_data: &'static [u8]) {
            if let Some(sender) = &self.command_sender {
                if let Err(e) = sender.send(AudioCommand::PlaySound(sound_data)) {
                    warn!("Error - Failed to send audio command: {:?}", e);
                }
            }
        }
    }

    pub fn play_sound(sound_data: &'static [u8]) {
        if let Ok(audio_system) = AUDIO_SYSTEM.lock() {
            audio_system.play_sound(sound_data);
        }
    }

    pub enum AudioCommand {
        PlaySound(&'static [u8]),
    }
}
