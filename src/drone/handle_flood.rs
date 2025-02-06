use super::RustBustersDrone;
use crossbeam_channel::Sender;
use log::{debug, error, info, warn};
use wg_2024::controller::DroneEvent;
use wg_2024::network::{NodeId, SourceRoutingHeader};
use wg_2024::packet::{FloodRequest, FloodResponse, NodeType, Packet, PacketType};

impl RustBustersDrone {
    /// Handle a `FloodRequest` packet
    ///
    /// #### Arguments
    /// - `packet`: The `FloodRequest` packet to be handled
    pub(crate) fn handle_flood_request(&mut self, packet: Packet) {
        // If the drone has crashed, the request can be dropped
        if !self.running {
            return;
        }

        debug!("Drone {} - Received FloodRequest", self.id);
        if let PacketType::FloodRequest(mut flood_request) = packet.pack_type {
            let sender_id = if let Some(&(last_node_id, _)) = flood_request.path_trace.last() {
                last_node_id
            } else {
                error!("Drone {} - path_trace is empty in handle_flood", self.id);
                flood_request.initiator_id
            };

            // Add self to path_trace
            flood_request.path_trace.push((self.id, NodeType::Drone));

            if self
                .received_floods
                .contains(&(flood_request.flood_id, flood_request.initiator_id))
            {
                self.send_flood_response(&flood_request, packet.session_id, sender_id);
            } else {
                self.spread_flood_request(&flood_request, packet.session_id, sender_id);
            }
        } else {
            error!(
                "Drone {} - Expected FloodRequest, but got different packet type.",
                self.id
            );
        }
    }

    /// Send a `FloodResponse` packet to the sender of the given `FloodRequest`
    ///
    /// #### Arguments
    /// - `flood_request`: The `FloodRequest` for which the `FloodResponse` is being sent
    /// - `session_id`: The session ID of the `FloodRequest`
    /// - `sender_id`: The ID of the sender of the `FloodRequest`
    pub(crate) fn send_flood_response(
        &mut self,
        flood_request: &FloodRequest,
        session_id: u64,
        sender_id: NodeId,
    ) {
        debug!(
            "Drone {} - Already processed FloodRequest(flood_id={}, sender_id={})",
            self.id, flood_request.flood_id, sender_id
        );
        // Send FloodResponse back to sender
        let response = FloodResponse {
            flood_id: flood_request.flood_id,
            path_trace: flood_request.path_trace.clone(),
        };

        let mut return_path = flood_request
            .path_trace
            .iter()
            .map(|(id, _)| *id)
            .rev()
            .collect::<Vec<NodeId>>();

        // if the return path does not contain the initiator, add it
        if return_path.last() != Some(&flood_request.initiator_id) {
            return_path.push(flood_request.initiator_id);
        }

        let response_packet = Packet {
            pack_type: PacketType::FloodResponse(response),
            routing_header: SourceRoutingHeader {
                hop_index: 1,
                hops: if self.optimized_routing {
                    self.optimize_route(&return_path)
                } else {
                    return_path
                },
            },
            session_id,
        };
        if let Some(sender) = self.packet_send.get(&sender_id) {
            if let Err(e) = sender.send(response_packet.clone()) {
                self.packet_send.remove(&sender_id);
                error!(
                    "Drone {} - Error in sending FloodResponse(flood_id={}, sender_id={}): {}",
                    self.id, flood_request.flood_id, sender_id, e
                );
                warn!(
                    "Drone {} - Removed neighbor with ID {} from packet_send due to channel closure",
                    self.id,
                    sender_id
                );

                self.send_to_sc(DroneEvent::ControllerShortcut(response_packet.clone()));
            } else {
                info!(
                    "Drone {} - Sent FloodResponse(flood_id={}, sender_id={})",
                    self.id, flood_request.flood_id, sender_id
                );
                self.send_to_sc(DroneEvent::PacketSent(response_packet));
            }
        } else {
            warn!(
                "Drone {} - Not found Sender {} in packet_send",
                self.id, sender_id
            );

            self.send_to_sc(DroneEvent::ControllerShortcut(response_packet.clone()));
        }
    }

    /// Spread a `FloodRequest` packet to neighbors
    ///
    /// #### Arguments
    /// - `flood_request`: The `FloodRequest` packet to be spread
    /// - `session_id`: The session ID of the `FloodRequest`
    /// - `sender_id`: The ID of the sender of the `FloodRequest`
    ///
    /// > Note: The `FloodRequest` is spread to all neighbors except the sender
    pub(crate) fn spread_flood_request(
        &mut self,
        flood_request: &FloodRequest,
        session_id: u64,
        sender_id: NodeId,
    ) {
        debug!(
            "Drone {} - FloodRequest(flood_id={}, sender_id={}) is being processed",
            self.id, flood_request.flood_id, sender_id
        );
        self.received_floods
            .insert((flood_request.flood_id, flood_request.initiator_id));
        // Collect neighbor IDs and Senders into a separate vector (excluding the sender)
        let neighbors: Vec<(NodeId, Sender<Packet>)> = self
            .packet_send
            .iter()
            .filter(|(&neighbor_id, _)| neighbor_id != sender_id)
            .map(|(&neighbor_id, sender)| (neighbor_id, sender.clone()))
            .collect();

        if neighbors.is_empty() {
            debug!(
                "Drone {} - No neighbors to forward FloodRequest(flood_id={}, sender_id={}) to",
                self.id, flood_request.flood_id, sender_id
            );
            self.send_flood_response(flood_request, session_id, sender_id);

            return;
        }

        // Forward FloodRequest to neighbors except the sender
        for (neighbor_id, neighbor_sender) in neighbors {
            let packet = Packet {
                pack_type: PacketType::FloodRequest(flood_request.clone()),
                routing_header: SourceRoutingHeader {
                    hop_index: 1,
                    hops: vec![self.id, neighbor_id],
                },
                session_id,
            };
            if let Err(e) = neighbor_sender.send(packet.clone()) {
                // Remove the neighbor from packet_send
                self.packet_send.remove(&neighbor_id);
                error!(
                    "Drone {} - Error in sending FloodRequest(flood_id={}, sender_id={}) to {}: {}",
                    self.id, flood_request.flood_id, sender_id, neighbor_id, e
                );
                warn!(
                    "Drone {} - Removed neighbor with ID {} from packet_send due to channel closure",
                    self.id,
                    neighbor_id
                );
            } else {
                info!(
                    "Drone {} - Forwarded FloodRequest(flood_id={}, sender_id={}) to neighbor: {}",
                    self.id, flood_request.flood_id, sender_id, neighbor_id
                );
                self.send_to_sc(DroneEvent::PacketSent(packet.clone()));
            }
        }
    }
}
