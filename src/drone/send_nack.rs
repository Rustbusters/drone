use super::RustBustersDrone;
use crate::drone::sounds::{DROP_SOUND, NACK_SOUND};
use log::{debug, error, info, trace, warn};
use wg_2024::network::{NodeId, SourceRoutingHeader};
use wg_2024::packet::NackType::Dropped;
use wg_2024::packet::{Nack, Packet, PacketType};

impl RustBustersDrone {
    /// Send a Nack packet to the sender of the given packet
    ///
    /// #### Arguments
    /// - `packet`: The packet for which the Nack is being sent
    /// - `nack`: The Nack to be sent
    /// - `allow_optimized`: A boolean indicating whether optimized routing is allowed
    pub fn send_nack(&mut self, packet: &Packet, nack: Nack, allow_optimized: bool) {
        debug!("Drone {}: Sending Nack: {:?}", self.id, nack);
        let hop_index = packet.routing_header.hop_index - 1; // hop_index: actual drone
        let nack_type = nack.nack_type;

        if hop_index == 0 {
            error!("Drone {}: Error: hop_index is 0 in send_nack", self.id);
            return;
        }

        let path_to_sender = &packet.routing_header.hops[0..=hop_index];
        let reversed_path = path_to_sender
            .iter()
            .rev()
            .cloned()
            .collect::<Vec<NodeId>>();

        let hops = if self.optimized_routing && allow_optimized {
            self.optimize_route(&reversed_path)
        } else {
            reversed_path
        };

        let source_routing_header = SourceRoutingHeader { hop_index: 1, hops };

        let nack_packet = Packet {
            pack_type: PacketType::Nack(nack),
            routing_header: source_routing_header,
            session_id: packet.session_id,
        };

        let next_hop = if nack_packet.routing_header.hops.len() > 1 {
            nack_packet.routing_header.hops[1]
        } else {
            error!(
                "Drone {}: Error: hops len is 1 in nack {:?}",
                self.id, nack_packet
            );
            return;
        };

        if nack_type == Dropped {
            self.play_sound(DROP_SOUND);
        } else {
            self.play_sound(NACK_SOUND);
        }

        // TODO: inviare il Nack al SC se si verifica un errore durante l'invio al presunto vicino
        if let Some(next_sender) = self.packet_send.get(&next_hop).cloned() {
            if let Err(e) = next_sender.send(nack_packet) {
                error!(
                    "Drone {}: Error sending Nack to {}: {}",
                    self.id, next_hop, e
                );
                self.packet_send.remove(&next_hop);
                warn!(
                    "Drone {}: Neighbor {} has been removed from packet_send due to channel closure",
                    self.id, next_hop
                );
            } else {
                info!("Drone {}: Nack sent to {}", self.id, next_hop);
            }
        } else {
            warn!(
                "Drone {}: Cannot send Nack, next hop {} is not a neighbor",
                self.id, next_hop
            );
            trace!("Drone {}: Nack packet: {:?}", self.id, nack_packet);
        }
    }
}
