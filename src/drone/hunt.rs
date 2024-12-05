use crate::drone::sounds::HUNT_SOUND;
use crate::RustBustersDrone;
use wg_2024::controller::DroneEvent;
use wg_2024::network::{NodeId, SourceRoutingHeader};
use wg_2024::packet::{Fragment, Packet, PacketType, FRAGMENT_DSIZE};

pub const PACKET_CONST: u8 = 169;

impl RustBustersDrone {
    /// Sends a hunt packet to the specified target
    ///
    /// #### Arguments
    /// - `target_id`: The ID of the target node
    ///
    /// #### Returns
    /// A Result containing `()` if the packet was sent successfully, or an error message if the packet could not be sent
    ///
    /// #### Errors
    /// - If the packet could not be sent, an error message is returned
    pub fn hunt_ghost(&self, target_id: NodeId) -> Result<(), String> {
        // Construct the hunt_packet
        // You can recognize the packet for the fragment_index: 0, total_n_fragments: 0, length: 0

        // Step 1: construct the data:
        let mut data = [0; FRAGMENT_DSIZE];
        self.set_data(&mut data, target_id);

        // Step 2: construct the packet with the specified data
        let hunt_packet = Packet {
            pack_type: PacketType::MsgFragment(Fragment {
                fragment_index: 0,
                total_n_fragments: 0,
                length: PACKET_CONST,
                data,
            }),
            routing_header: SourceRoutingHeader {
                hop_index: 0,
                hops: vec![],
            },
            session_id: 0,
        };
        // Create DroneEvent
        let kill_node_event = DroneEvent::PacketSent(hunt_packet);

        // Step 3: send the packet to the SC
        if self.controller_send.send(kill_node_event).is_ok() {
            self.play_sound(HUNT_SOUND);
            Ok(())
        } else {
            Err("Error in sending Hunt Packet".to_string())
        }
    }

    /// Sets the data of the packet
    ///
    /// #### Arguments
    /// - `data`: The data to be set
    /// - `target_id`: The ID of the target node
    pub fn set_data(&self, data: &mut [u8; FRAGMENT_DSIZE], target_id: NodeId) {
        data[0] = self.id;
        data[1] = target_id;
    }
}
