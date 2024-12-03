use super::RustBustersDrone;
use crate::drone::sounds::CRASH_SOUND;
use log::info;
use std::thread;
use wg_2024::controller::DroneCommand;

impl RustBustersDrone {
    pub fn handle_command(&mut self, command: DroneCommand) {
        info!("Drone {}: Handling command {:?}", self.id, command);
        match command {
            DroneCommand::Crash => {
                info!("Drone {}: Received Crash command. Shutting down.", self.id);
                self.play_sound(CRASH_SOUND);
                thread::sleep(std::time::Duration::from_millis(500));
                println!("Drone {}: Shutting down.", self.id);
                self.running = false;
            }
            DroneCommand::AddSender(node_id, sender) => {
                self.packet_send.insert(node_id, sender);
                info!("Drone {}: Added sender for node_id {}", self.id, node_id);
            }
            DroneCommand::SetPacketDropRate(new_pdr) => {
                self.play_sound(CRASH_SOUND);

                self.pdr = ((new_pdr * 100.0).round() as u8).clamp(0, 100);
                info!("Drone {}: Set Packet Drop Rate to {}%", self.id, self.pdr);
            }
        }
    }
}
