use crossbeam_channel::{select, Receiver, Sender};
use rand::Rng;
use std::collections::{HashMap, HashSet};
use wg_2024::controller::{DroneCommand, NodeEvent};
use wg_2024::drone::{Drone, DroneOptions};
use wg_2024::network::{NodeId, SourceRoutingHeader};
use wg_2024::packet::{Ack, FloodResponse, Nack, NodeType, Packet, PacketType};

pub struct RustBustersDrone {
    id: NodeId,
    controller_send: Sender<NodeEvent>,
    controller_recv: Receiver<DroneCommand>,
    packet_recv: Receiver<Packet>,
    pdr: u8, // Packet Drop Rate in percentage (0-100)
    packet_send: HashMap<NodeId, Sender<Packet>>,
    received_floods: HashSet<u64>,
    optimized_routing: bool,
}

impl Drone for RustBustersDrone {
    fn new(options: DroneOptions) -> Self {
        Self {
            id: options.id,
            controller_send: options.controller_send,
            controller_recv: options.controller_recv,
            packet_recv: options.packet_recv,
            pdr: (options.pdr * 100.0) as u8,
            packet_send: options.packet_send,
            received_floods: HashSet::new(),
            optimized_routing: true,
        }
    }

    fn run(&mut self) {
        loop {
            select! {
                recv(self.packet_recv) -> packet_res => {
                    if let Ok(packet) = packet_res {
                        match packet.pack_type {
                            PacketType::FloodRequest(_) => self.handle_flood(packet),
                            _ => self.forward_packet(packet, true),
                        }
                    }
                },
                recv(self.controller_recv) -> command_res => {
                    if let Ok(command) = command_res {
                        self.handle_command(command);
                    }
                }
            }
        }
    }
}

impl RustBustersDrone {
    fn forward_packet(&mut self, mut packet: Packet, allow_optimized: bool) {
        let hop_index = packet.routing_header.hop_index;

        // Step 1: Check if hops[hop_index] matches self.id
        if packet.routing_header.hops[hop_index] != self.id {
            self.send_nack(packet, Nack::UnexpectedRecipient(self.id), allow_optimized);
            return;
        }

        // Step 2: Increment hop_index
        packet.routing_header.hop_index += 1;

        // Step 3: Check if drone is the final destination
        let next_hop_index = packet.routing_header.hop_index;
        if next_hop_index >= packet.routing_header.hops.len() {
            self.send_nack(packet, Nack::DestinationIsDrone, allow_optimized);
            return;
        }

        // Step 4: Identify next hop and check if it's a neighbor
        let next_hop = packet.routing_header.hops[next_hop_index];
        if !self.packet_send.contains_key(&next_hop) {
            self.send_nack(packet, Nack::ErrorInRouting(next_hop), allow_optimized);
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
                    self.send_nack(
                        packet.clone(),
                        Nack::Dropped(fragment.fragment_index),
                        allow_optimized,
                    );
                    // Send PacketDropped event to the controller
                    if let Err(e) = self
                        .controller_send
                        .send(NodeEvent::PacketDropped(packet.clone()))
                    {
                        eprintln!("Error sending PacketDropped event: {}", e);
                    }
                    return;
                }

                // Forward the packet to next_hop
                let next_sender = self.packet_send.get(&next_hop);
                if let Some(next_sender) = next_sender {
                    if let Err(e) = next_sender.send(packet.clone()) {
                        eprintln!("Error sending packet to {}: {}", next_hop, e);

                        self.packet_send.remove(&next_hop); // TODO: lo rimuovo subito?
                        eprintln!(
                            "Neighbor {} has been removed from packet_send due to channel closure",
                            next_hop
                        );
                        // Optionally, you might want to send a Nack back to the sender
                        self.send_nack(packet, Nack::ErrorInRouting(next_hop), allow_optimized);

                    } else {
                        // Send PacketSent event to the controller
                        if let Err(e) = self
                            .controller_send
                            .send(NodeEvent::PacketSent(packet.clone()))
                        {
                            eprintln!("Error sending PacketSent event: {}", e);
                        }
                        // Send Ack to client TODO: remove when issue #82 is resolved
                        self.send_ack(
                            packet.session_id,
                            fragment.fragment_index,
                            &packet.routing_header,
                        );
                    }
                } else {
                    // Neighbor not found in packet_send, send Nack
                    self.send_nack(packet, Nack::ErrorInRouting(next_hop), allow_optimized);
                }
                
                
            }
            PacketType::Ack(_) | PacketType::Nack(_) | PacketType::FloodResponse(_) => {
                // Forward these packets without dropping
                let next_sender = self.packet_send.get(&next_hop).unwrap();
                if let Err(e) = next_sender.send(packet.clone()) {
                    eprintln!("Error sending packet to {}: {}", next_hop, e);
                }
            }
            PacketType::FloodRequest(_) => {
                // Should not reach here
                eprintln!(
                    "Received FloodRequest in forward_packet, which should be handled in handle_flood."
                );
            }
        }
    }

    fn send_ack(
        &mut self,
        session_id: u64,
        fragment_index: u64,
        packet_routing_header: &SourceRoutingHeader,
    ) {
        let hop_index = packet_routing_header.hop_index;

        if hop_index == 0 {
            eprintln!("Error: hop_index is 0 in send_ack");
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
        
        if let Some(next_sender) = self.packet_send.get(&next_hop) {
            if let Err(e) = next_sender.send(ack_packet) {
                eprintln!("Error sending Ack to {}: {}", next_hop, e);
                self.packet_send.remove(&next_hop);
                eprintln!(
                    "Neighbor {} has been removed from packet_send due to channel closure",
                    next_hop
                );
            }
        } else { // TODO: to handle loss of undroppable packets
            eprintln!("Cannot send Ack, next hop {} is not a neighbor", next_hop);
        }
    }

    fn send_nack(&mut self, packet: Packet, nack: Nack, allow_optimized: bool) {
        let hop_index = packet.routing_header.hop_index;

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

        if let Some(next_sender) = self.packet_send.get(&next_hop) {
            if let Err(e) = next_sender.send(nack_packet) {
                eprintln!("Error sending Nack to {}: {}", next_hop, e);
                // Remove the neighbor from packet_send
                self.packet_send.remove(&next_hop);
                eprintln!(
                    "Neighbor {} has been removed from packet_send due to channel closure",
                    next_hop
                );
            }
        } else { // TODO: to handle loss of undroppable packets
            eprintln!("Cannot send Nack, next hop {} is not a neighbor", next_hop);
        }
    }

    fn optimize_route(&self, path: &[NodeId]) -> Vec<NodeId> {
        for (index, &node_id) in path.iter().enumerate() {
            if self.packet_send.contains_key(&node_id) {
                return path[index..].to_vec();
            }
        }
        path.to_vec()
    }

    fn handle_flood(&mut self, packet: Packet) {
        if let PacketType::FloodRequest(mut flood_request) = packet.pack_type {
            let sender_id = if let Some(&(last_node_id, _)) = flood_request.path_trace.last() {
                last_node_id
            } else {
                eprintln!("Error: path_trace is empty in handle_flood");
                flood_request.initiator_id
            };

            // Add self to path_trace
            flood_request.path_trace.push((self.id, NodeType::Drone));

            if self.received_floods.contains(&flood_request.flood_id) {
                // Send FloodResponse back to sender
                let response = FloodResponse {
                    flood_id: flood_request.flood_id,
                    path_trace: flood_request.path_trace.clone(),
                };
                let response_packet = Packet {
                    pack_type: PacketType::FloodResponse(response),
                    routing_header: SourceRoutingHeader {
                        hop_index: 0,
                        hops: flood_request
                            .path_trace
                            .iter()
                            .map(|(id, _)| *id)
                            .rev()
                            .collect(),
                    },
                    session_id: packet.session_id,
                };
                if let Some(sender) = self.packet_send.get(&sender_id) {
                    if let Err(e) = sender.send(response_packet) {
                        eprintln!("Error sending FloodResponse to {}: {}", sender_id, e);
                    }
                }
            } else {
                self.received_floods.insert(flood_request.flood_id);
                // Forward FloodRequest to neighbors except the sender
                for (&neighbor_id, neighbor_sender) in &self.packet_send {
                    if neighbor_id != sender_id {
                        let packet = Packet {
                            pack_type: PacketType::FloodRequest(flood_request.clone()),
                            routing_header: SourceRoutingHeader {
                                hop_index: 1,
                                hops: vec![self.id, neighbor_id],
                            },
                            session_id: packet.session_id,
                        };
                        if let Err(e) = neighbor_sender.send(packet) {
                            eprintln!("Error sending FloodRequest to {}: {}", neighbor_id, e);
                            // Remove the neighbor from packet_send
                            // self.packet_send.remove(&neighbor_id); // TODO: handle removal of neighbors
                            eprintln!(
                                "Neighbor {} has been removed from packet_send due to channel closure",
                                neighbor_id
                            );
                        }
                    }
                }
            }
        }
    }

    fn handle_command(&mut self, command: DroneCommand) {
        match command {
            DroneCommand::Crash => {
                // Terminate the drone's thread
                std::process::exit(0); // TODO: handle this more gracefully
            }
            DroneCommand::AddSender(node_id, sender) => {
                self.packet_send.insert(node_id, sender);
                eprintln!("Drone {} added sender for node_id {}", self.id, node_id);
            }
            DroneCommand::SetPacketDropRate(new_pdr) => {
                self.pdr = (new_pdr * 100.0) as u8;
                eprintln!("Drone {} set Packet Drop Rate to {}%", self.id, self.pdr);
            }
        }
    }
}
