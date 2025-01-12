use super::RustBustersDrone;
#[cfg(feature = "sounds")]
use crate::drone::sounds::sounds_feat::{DROP_SOUND, NACK_SOUND};
use log::{debug, error, info, trace, warn};
use wg_2024::controller::DroneEvent;
use wg_2024::controller::DroneEvent::ControllerShortcut;
use wg_2024::network::{NodeId, SourceRoutingHeader};
use wg_2024::packet::{Nack, Packet, PacketType};

impl RustBustersDrone {
    /// Send a Nack packet to the sender of the given packet
    ///
    /// #### Arguments
    /// - `packet`: The packet for which the Nack is being sent
    /// - `nack`: The Nack to be sent
    /// - `allow_optimized`: A boolean indicating whether optimized routing is allowed
    pub fn send_nack(&mut self, packet: &Packet, nack: Nack, allow_optimized: bool) {
        debug!("Drone {} - Send Nack: {:?}", self.id, nack);
        let hop_index = packet.routing_header.hop_index - 1; // hop_index: actual drone
        let nack_type = nack.nack_type;

        if hop_index == 0 || hop_index >= packet.routing_header.hops.len() {
            error!(
                "Drone {} - hop_index out of range in Nack {:?}",
                self.id, nack
            );
            return;
        }

        let path_to_sender = &packet.routing_header.hops[0..=hop_index];
        let reversed_path = path_to_sender
            .iter()
            .rev()
            .copied()
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
            unreachable!(
                "Drone {}: Error: hops len is 1 in nack {:?}",
                self.id, nack_packet
            );
        };

        #[cfg(feature = "sounds")]
        {
            use wg_2024::packet::NackType::Dropped;
            if nack_type == Dropped {
                self.play_sound(DROP_SOUND);
            } else {
                self.play_sound(NACK_SOUND);
            }
        }

        if let Some(next_sender) = self.packet_send.get(&next_hop).cloned() {
            if let Err(e) = next_sender.send(nack_packet.clone()) {
                error!(
                    "Drone {} - Error in sending Nack to {}: {}",
                    self.id, next_hop, e
                );
                self.packet_send.remove(&next_hop);
                self.send_to_sc(ControllerShortcut(nack_packet.clone()));
                warn!(
                    "Drone {} - Neighbor {} has been removed from packet_send due to channel closure",
                    self.id, next_hop
                );
            } else {
                info!(
                    "Drone {} - Forwarded Nack to next hop: {}",
                    self.id, next_hop
                );
            }
        } else {
            warn!(
                "Drone {} - Unable to send Nack: next hop {} is not a neighbor",
                self.id, next_hop
            );
            trace!("Drone {} - Nack Packet: {:?}", self.id, nack_packet);
            self.send_to_sc(ControllerShortcut(nack_packet.clone()));
        }
        self.send_to_sc(DroneEvent::PacketSent(nack_packet));
    }
}
