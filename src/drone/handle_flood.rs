use super::RustBustersDrone;
use crossbeam_channel::Sender;
use log::{debug, error, info, warn};
use wg_2024::network::{NodeId, SourceRoutingHeader};
use wg_2024::packet::{FloodResponse, NodeType, Packet, PacketType};

impl RustBustersDrone {
    pub fn handle_flood_request(&mut self, packet: Packet) {
        debug!("Drone {}: Handling FloodRequest", self.id);
        if let PacketType::FloodRequest(mut flood_request) = packet.pack_type {
            let sender_id = if let Some(&(last_node_id, _)) = flood_request.path_trace.last() {
                last_node_id
            } else {
                error!(
                    "Drone {}: Error: path_trace is empty in handle_flood",
                    self.id
                );
                flood_request.initiator_id
            };

            // Add self to path_trace
            flood_request.path_trace.push((self.id, NodeType::Drone));

            if self.received_floods.contains(&flood_request.flood_id) {
                debug!(
                    "Drone {}: Already received flood_id {}",
                    self.id, flood_request.flood_id
                );
                // Send FloodResponse back to sender
                let response = FloodResponse {
                    flood_id: flood_request.flood_id,
                    path_trace: flood_request.path_trace.clone(),
                };
                let response_packet = Packet {
                    pack_type: PacketType::FloodResponse(response),
                    routing_header: SourceRoutingHeader {
                        hop_index: 1,
                        hops: flood_request
                            .path_trace
                            .iter()
                            .map(|(id, _)| *id)
                            .rev()
                            .collect(),
                    },
                    session_id: packet.session_id,
                };
                if let Some(sender) = self.packet_send.get(&sender_id).cloned() {
                    if let Err(e) = sender.send(response_packet) {
                        error!(
                            "Drone {}: Error sending FloodResponse({}) to {}: {}",
                            self.id, flood_request.flood_id, sender_id, e
                        );
                        self.packet_send.remove(&sender_id);
                        warn!(
                            "Drone {}: Neighbor {} has been removed from packet_send due to channel closure",
                            self.id, sender_id
                        );
                    } else {
                        info!(
                            "Drone {}: FloodResponse({}) sent to {}",
                            self.id, flood_request.flood_id, sender_id
                        );
                    }
                } else {
                    warn!(
                        "Drone {}: Sender {} not found in packet_send.",
                        self.id, sender_id
                    );
                }
            } else {
                debug!(
                    "Drone {}: Processing new flood_id {}",
                    self.id, flood_request.flood_id
                );
                self.received_floods.insert(flood_request.flood_id);
                // Collect neighbor IDs and Senders into a separate vector
                let neighbors: Vec<(NodeId, Sender<Packet>)> = self
                    .packet_send
                    .iter()
                    .filter(|(&neighbor_id, _)| neighbor_id != sender_id)
                    .map(|(&neighbor_id, sender)| (neighbor_id, sender.clone()))
                    .collect();

                // Forward FloodRequest to neighbors except the sender
                for (neighbor_id, neighbor_sender) in neighbors {
                    let packet = Packet {
                        pack_type: PacketType::FloodRequest(flood_request.clone()),
                        routing_header: SourceRoutingHeader {
                            hop_index: 1,
                            hops: vec![self.id, neighbor_id],
                        },
                        session_id: packet.session_id,
                    };
                    if let Err(e) = neighbor_sender.send(packet) {
                        error!(
                            "Drone {}: Error sending FloodRequest({}) to {}: {}",
                            self.id, flood_request.flood_id, neighbor_id, e
                        );
                        // Remove the neighbor from packet_send
                        self.packet_send.remove(&neighbor_id);
                        warn!(
                            "Drone {}: Neighbor {} has been removed from packet_send due to channel closure",
                            self.id, neighbor_id
                        );
                    } else {
                        info!(
                            "Drone {}: FloodRequest({}) forwarded to {}",
                            self.id, flood_request.flood_id, neighbor_id
                        );
                    }
                }
            }
        } else {
            error!(
                "Drone {}: Expected FloodRequest, but got different packet type.",
                self.id
            );
        }
    }
}
