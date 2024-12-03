use super::RustBustersDrone;
use log::info;
use wg_2024::controller::DroneCommand;

impl RustBustersDrone {
    /// Handles the given command
    ///
    /// #### Arguments
    /// - `command`: The command to handle
    ///
    /// > Note:
    /// > - The `Crash` command will shut down the drone
    /// > - The `AddSender` command will add a sender for the given node ID
    /// > - The `SetPacketDropRate` command will set the Packet Drop Rate to the given value
    /// > - The `RemoveSender` command will remove the sender for the given node ID
    pub fn handle_command(&mut self, command: DroneCommand) {
        info!("Drone {}: Handling command {:?}", self.id, command);
        match command {
            DroneCommand::Crash => {
                info!("Drone {}: Received Crash command. Shutting down.", self.id);
                self.running = false;
            }
            DroneCommand::AddSender(node_id, sender) => {
                self.packet_send.insert(node_id, sender);
                info!("Drone {}: Added sender for node_id {}", self.id, node_id);
            }
            DroneCommand::SetPacketDropRate(new_pdr) => {
                self.pdr = ((new_pdr * 100.0).round() as u8).clamp(0, 100);
                info!("Drone {}: Set Packet Drop Rate to {}%", self.id, self.pdr);
            }
            DroneCommand::RemoveSender(node_id) => {
                self.packet_send.remove(&node_id);
                info!("Drone {}: Removed sender for node_id {}", self.id, node_id);
            }
        }
    }
}
