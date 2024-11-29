use super::RustBustersDrone;
use log::{debug, error, info, trace, warn};
use wg_2024::network::NodeId;
use wg_2024::network::SourceRoutingHeader;
use wg_2024::packet::{Ack, Packet, PacketType};

impl RustBustersDrone {
    pub fn send_ack(
        &mut self,
        session_id: u64,
        fragment_index: u64,
        packet_routing_header: &SourceRoutingHeader,
    ) {
        debug!(
            "Drone {}: Sending Ack for fragment {}",
            self.id, fragment_index
        );
        let hop_index = packet_routing_header.hop_index - 1; // hop_index: actual drone

        if hop_index == 0 {
            error!("Drone {}: Error: hop_index is 0 in send_ack", self.id);
            return;
        }

        let path_to_client = &packet_routing_header.hops[0..=hop_index];
        let reversed_path: Vec<NodeId> = path_to_client.iter().rev().cloned().collect();

        let hops = if self.optimized_routing {
            self.optimize_route(&reversed_path)
        } else {
            reversed_path
        };

        let ack = Ack { fragment_index };

        let ack_packet = Packet {
            pack_type: PacketType::Ack(ack),
            routing_header: SourceRoutingHeader { hop_index: 1, hops },
            session_id,
        };

        let next_hop = ack_packet.routing_header.hops[1];

        if let Some(next_sender) = self.packet_send.get(&next_hop).cloned() {
            if let Err(e) = next_sender.send(ack_packet) {
                error!(
                    "Drone {}: Error sending Ack to {}: {}",
                    self.id, next_hop, e
                );
                self.packet_send.remove(&next_hop);
                warn!(
                    "Drone {}: Neighbor {} has been removed from packet_send due to channel closure",
                    self.id, next_hop
                );
            } else {
                info!("Drone {}: Ack sent to {}", self.id, next_hop);
            }
        } else {
            warn!(
                "Drone {}: Cannot send Ack, next hop {} is not a neighbor",
                self.id, next_hop
            );
            trace!("Drone {}: Ack packet: {:?}", self.id, ack_packet);
        }
    }
}
