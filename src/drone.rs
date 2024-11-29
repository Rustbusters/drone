use crossbeam_channel::{select, Receiver, Sender};
use log::{debug, error, info, warn, trace}; // Import logging macros
use rand::Rng;
use std::collections::{HashMap, HashSet};
use wg_2024::controller::{DroneCommand, NodeEvent};
use wg_2024::drone::{Drone, DroneOptions};
use wg_2024::network::{NodeId, SourceRoutingHeader};
use wg_2024::packet::{Ack, FloodResponse, Nack, NackType, NodeType, Packet, PacketType};

pub struct RustBustersDrone {
    id: NodeId,
    controller_send: Sender<NodeEvent>,
    controller_recv: Receiver<DroneCommand>,
    packet_recv: Receiver<Packet>,
    pdr: u8, // Packet Drop Rate in percentage (0-100)
    packet_send: HashMap<NodeId, Sender<Packet>>,
    received_floods: HashSet<u64>,
    optimized_routing: bool,
    running: bool,
}

impl Drone for RustBustersDrone {
    fn new(options: DroneOptions) -> Self {
        info!("Initializing drone with ID {}", options.id);
        Self {
            id: options.id,
            controller_send: options.controller_send,
            controller_recv: options.controller_recv,
            packet_recv: options.packet_recv,
            pdr: (options.pdr * 100.0) as u8,
            packet_send: options.packet_send,
            received_floods: HashSet::new(),
            optimized_routing: false,
            running: true,
        }
    }

    fn run(&mut self) {
        info!("Drone {} starting to run.", self.id);
        while self.running {
            select! {
                recv(self.packet_recv) -> packet_res => {
                    match packet_res {
                        Ok(packet) => {
                            trace!("Drone {} received packet: {:?}", self.id, packet);
                            match packet.pack_type {
                                PacketType::FloodRequest(_) => self.handle_flood(packet),
                                _ => self.forward_packet(packet, true),
                            }
                        }
                        Err(e) => {
                            warn!("Drone {} packet receive error: {}", self.id, e);
                        }
                    }
                },
                recv(self.controller_recv) -> command_res => {
                    match command_res {
                        Ok(command) => {
                            debug!("Drone {} received command: {:?}", self.id, command);
                            self.handle_command(command);
                        }
                        Err(e) => {
                            warn!("Drone {} controller receive error: {}", self.id, e);
                        }
                    }
                }
            }
        }
        info!("Drone {} has stopped running.", self.id);
    }
}

impl RustBustersDrone {
    fn forward_packet(&mut self, mut packet: Packet, allow_optimized: bool) {
        trace!("Drone {} forwarding packet: {:?}", self.id, packet);
        let hop_index = packet.routing_header.hop_index;

        // Step 1: Check if hops[hop_index] matches self.id
        if packet.routing_header.hops[hop_index] != self.id {
            warn!(
                "Drone {}: Unexpected recipient. Expected {}, got {}",
                self.id, packet.routing_header.hops[hop_index], self.id
            );
            self.send_nack(packet, Nack {
                fragment_index: 0, // TODO: Set fragment index
                nack_type: NackType::UnexpectedRecipient(self.id),
            }, allow_optimized);
            return;
        }

        // Step 2: Increment hop_index
        packet.routing_header.hop_index += 1;

        // Step 3: Check if drone is the final destination
        let next_hop_index = packet.routing_header.hop_index;
        if next_hop_index >= packet.routing_header.hops.len() {
            warn!("Drone {}: Destination is drone, sending Nack.", self.id);
            self.send_nack(packet, Nack {
                fragment_index: 0, // TODO: Set fragment index
                nack_type: NackType::DestinationIsDrone,
            }, allow_optimized);
            return;
        }

        // Step 4: Identify next hop and check if it's a neighbor
        let next_hop = packet.routing_header.hops[next_hop_index];
        if !self.packet_send.contains_key(&next_hop) {
            warn!(
                "Drone {}: Next hop {} is not a neighbor.",
                self.id, next_hop
            );
            self.send_nack(packet, Nack {
                fragment_index: 0, // TODO: Set fragment index
                nack_type: NackType::ErrorInRouting(next_hop),
            }, allow_optimized);
            return;
        }

        // Step 5: Proceed based on packet type
        match &packet.pack_type {
            PacketType::MsgFragment(fragment) => {
                // Check for packet drop
                let should_drop = {
                    let mut rng = rand::thread_rng();
                    rng.gen_range(0..100) < self.pdr
                };

                if should_drop {
                    info!(
                        "Drone {}: Dropping packet due to PDR. Fragment index: {}",
                        self.id, fragment.fragment_index
                    );
                    self.send_nack(
                        packet.clone(),
                        Nack {
                            fragment_index: fragment.fragment_index,
                            nack_type: NackType::Dropped,
                        },
                        allow_optimized,
                    );
                    // Send PacketDropped event to the controller
                    if let Err(e) = self
                        .controller_send
                        .send(NodeEvent::PacketDropped(packet.clone()))
                    {
                        error!(
                            "Drone {}: Error sending PacketDropped event: {}",
                            self.id, e
                        );
                    }
                    return;
                }

                // Forward the packet to next_hop
                if let Some(next_sender) = self.packet_send.get(&next_hop).cloned() {
                    if let Err(e) = next_sender.send(packet.clone()) {
                        error!(
                            "Drone {}: Error sending packet to {}: {}",
                            self.id, next_hop, e
                        );
                        self.packet_send.remove(&next_hop);
                        warn!(
                            "Drone {}: Neighbor {} has been removed from packet_send due to channel closure",
                            self.id, next_hop
                        );
                        // Optionally, send a Nack back to the sender
                        self.send_nack(packet.clone(), Nack {
                            fragment_index: fragment.fragment_index,
                            nack_type: NackType::ErrorInRouting(next_hop),
                        }, allow_optimized);
                    } else {
                        info!("Drone {}: Packet forwarded to {}", self.id, next_hop);
                        // Send PacketSent event to the controller
                        if let Err(e) = self
                            .controller_send
                            .send(NodeEvent::PacketSent(packet.clone()))
                        {
                            error!("Drone {}: Error sending PacketSent event: {}", self.id, e);
                        }
                        // Send Ack to client TODO: remove this
                        self.send_ack(
                            packet.session_id,
                            fragment.fragment_index,
                            &packet.routing_header,
                        );
                    }
                } else {
                    warn!(
                        "Drone {}: Neighbor {} not found in packet_send.",
                        self.id, next_hop
                    );
                    // Neighbor not found in packet_send, send Nack
                    self.send_nack(packet.clone(), Nack {
                        fragment_index: fragment.fragment_index,
                        nack_type: NackType::ErrorInRouting(next_hop),
                    }, allow_optimized);
                }
            }
            PacketType::Ack(_) | PacketType::Nack(_) | PacketType::FloodResponse(_) => {
                // Forward these packets without dropping
                if let Some(next_sender) = self.packet_send.get(&next_hop).cloned() {
                    if let Err(e) = next_sender.send(packet.clone()) {
                        error!(
                            "Drone {}: Error sending packet to {}: {}",
                            self.id, next_hop, e
                        );
                        self.packet_send.remove(&next_hop);
                        warn!(
                            "Drone {}: Neighbor {} has been removed from packet_send due to channel closure",
                            self.id, next_hop
                        );
                    } else {
                        info!("Drone {}: Packet forwarded to {}", self.id, next_hop);
                    }
                } else {
                    warn!(
                        "Drone {}: Cannot forward packet, next hop {} is not a neighbor",
                        self.id, next_hop
                    );
                }
            }
            PacketType::FloodRequest(_) => {
                error!(
                    "Drone {}: Received FloodRequest in forward_packet, which should be handled in handle_flood.",
                    self.id
                );
            }
        }
    }

    fn send_ack(&mut self, session_id: u64, fragment_index: u64, packet_routing_header: &SourceRoutingHeader) {
        debug!(
            "Drone {}: Sending Ack for fragment {}",
            self.id, fragment_index
        );
        let hop_index = packet_routing_header.hop_index-1; // hop_index: actual drone

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

    fn send_nack(&mut self, packet: Packet, nack: Nack, allow_optimized: bool) {
        debug!("Drone {}: Sending Nack: {:?}", self.id, nack);
        let hop_index = packet.routing_header.hop_index - 1; // hop_index: actual drone

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

        let next_hop = nack_packet.routing_header.hops[1];

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
    fn optimize_route(&self, path: &[NodeId]) -> Vec<NodeId> {
        debug!("Drone {}: Optimizing route {:?}", self.id, path);

        if let Some((index, &last_nearby_node)) = path
            .iter()
            .enumerate()
            .rev()
            .find(|&(_, &node_id)| self.packet_send.contains_key(&node_id))
        {
            let mut optimized_path = path[index..].to_vec();

            optimized_path.insert(0, last_nearby_node);

            debug!("Drone {}: Optimized route {:?}", self.id, optimized_path);
            return optimized_path;
        }

        debug!(
            "Drone {}: No optimization possible, returning original path",
            self.id
        );
        path.to_vec()
    }

    fn handle_flood(&mut self, packet: Packet) {
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

    fn handle_command(&mut self, command: DroneCommand) {
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
        }
    }

    #[allow(dead_code)]
    pub fn set_optimized_routing(&mut self, optimized_routing: bool) {
        self.optimized_routing = optimized_routing;
        debug!(
            "Drone {}: Set optimized routing to {}",
            self.id, optimized_routing
        );
    }

    pub fn kill_drone(&self, target_drone_id: NodeId) {
        // Construct the kill_packet
        let kill_packet = Packet {
            pack_type: PacketType::Nack(Nack { fragment_index: 0, nack_type: NackType::DestinationIsDrone }),
            routing_header: SourceRoutingHeader { hop_index: 0, hops: vec![] },
            session_id: 0
        };
        let kill_node_event = NodeEvent::PacketDropped(kill_packet);
        // Send kill to SC with target_drone_id
        self.controller_send.send(kill_node_event).expect("Error in sending Kill Packet");
    }
}
