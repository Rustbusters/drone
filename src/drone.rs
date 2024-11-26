use crossbeam_channel::{select, Receiver, Sender};
use rand::Rng;
use std::collections::{HashMap, HashSet};
use wg_2024::controller::Command;
use wg_2024::drone::{Drone, DroneOptions};
use wg_2024::network::{NodeId, SourceRoutingHeader};
use wg_2024::packet::{Ack, FloodResponse, Nack, NackType, NodeType, Packet, PacketType};

pub struct RustBustersDrone {
    id: NodeId,
    sim_contr_send: Sender<Command>,
    sim_contr_recv: Receiver<Command>,
    packet_recv: Receiver<Packet>,
    pdr: u8,
    packet_send: HashMap<NodeId, Sender<Packet>>,
    received_floods: HashSet<u64>,
    optimized_routing: bool,
}

impl Drone for RustBustersDrone {
    fn new(options: DroneOptions) -> Self {
        Self {
            id: options.id,
            sim_contr_send: options.sim_contr_send,
            sim_contr_recv: options.sim_contr_recv,
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
                recv(self.sim_contr_recv) -> command_res => {
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
            self.send_nack(packet, NackType::UnexpectedRecipient(self.id), allow_optimized);
            return;
        }

        // Step 2: Increment hop_index
        packet.routing_header.hop_index += 1;

        // Step 3: Check if drone is the final destination
        let nex_hop_index = packet.routing_header.hop_index;
        if nex_hop_index >= packet.routing_header.hops.len() {
            self.send_nack(packet, NackType::DestinationIsDrone, allow_optimized);
            return;
        }

        // Step 4: Identify next hop and check if it's a neighbor
        let next_hop = packet.routing_header.hops[nex_hop_index];
        if !self.packet_send.contains_key(&next_hop) {
            self.send_nack(packet, NackType::ErrorInRouting(next_hop), allow_optimized);
            return;
        }

        // Step 5: Proceed based on packet type
        match packet.pack_type {
            PacketType::MsgFragment(_) => {
                // Check for packet drop
                let should_drop = {
                    let mut rng = rand::thread_rng();
                    rng.gen_range(0..100) < self.pdr
                };

                if should_drop {
                    self.send_nack(packet, NackType::Dropped, allow_optimized);
                    return;
                }

                // Forward the packet to next_hop
                let next_sender = self.packet_send.get(&next_hop).unwrap();
                if let Err(e) = next_sender.send(packet.clone()) {
                    eprintln!("Error sending packet to {}: {}", next_hop, e);
                } else {
                    // Send Ack to client
                    if let PacketType::MsgFragment(ref fragment) = packet.pack_type {
                        self.send_ack(
                            packet.session_id,
                            fragment.fragment_index,
                            &packet.routing_header,
                        );
                    }
                }
            },
            PacketType::Ack(_) | PacketType::Nack(_) | PacketType::FloodResponse(_) => {
                // Forward these packets without dropping
                let next_sender = self.packet_send.get(&next_hop).unwrap();
                if let Err(e) = next_sender.send(packet.clone()) {
                    eprintln!("Error sending packet to {}: {}", next_hop, e);
                }
            },
            PacketType::FloodRequest(_) => {
                // Should not reach here
                eprintln!("Received FloodRequest in forward_packet, which should be handled in handle_flood.");
            },
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

        let path_to_client = &packet_routing_header.hops[0..hop_index];
        let reversed_path: Vec<NodeId> = path_to_client.iter().rev().cloned().collect();

        let hops = if self.optimized_routing {
            self.optimize_route(&reversed_path)
        } else {
            reversed_path
        };

        let ack = Ack {
            fragment_index,
        };

        let ack_packet = Packet {
            pack_type: PacketType::Ack(ack),
            routing_header: SourceRoutingHeader { hop_index: 0, hops },
            session_id,
        };

        let next_hop = ack_packet.routing_header.hops[0];

        if !self.packet_send.contains_key(&next_hop) {
            eprintln!("Cannot send Ack, next hop {} is not a neighbor", next_hop);
            return;
        }

        let next_sender = self.packet_send.get(&next_hop).unwrap();

        if let Err(e) = next_sender.send(ack_packet) {
            eprintln!("Error sending Ack to {}: {}", next_hop, e);
        }
    }

    fn send_nack(&mut self, packet: Packet, nack_type: NackType, allow_optimized: bool) {
        let hop_index = packet.routing_header.hop_index;

        let path_to_sender = &packet.routing_header.hops[0..=hop_index];
        let reversed_path = path_to_sender.iter().rev().cloned().collect::<Vec<NodeId>>();

        let hops = if self.optimized_routing && allow_optimized {
            self.optimize_route(&reversed_path)
        } else {
            reversed_path.clone()
        };

        let source_routing_header = SourceRoutingHeader { hop_index: 0, hops };

        let fragment_index = Self::get_fragment_index(&packet);

        let nack = Nack {
            fragment_index,
            nack_type,
        };

        let nack_packet = Packet {
            pack_type: PacketType::Nack(nack),
            routing_header: source_routing_header,
            session_id: packet.session_id,
        };

        let next_hop = nack_packet.routing_header.hops[0];

        if !self.packet_send.contains_key(&next_hop) {
            eprintln!("Cannot send Nack, next hop {} is not a neighbor", next_hop);
            return;
        }

        let next_sender = self.packet_send.get(&next_hop).unwrap();

        if let Err(e) = next_sender.send(nack_packet) {
            eprintln!("Error sending Nack to {}: {}", next_hop, e);
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

    fn get_fragment_index(packet: &Packet) -> u64 {
        match &packet.pack_type {
            PacketType::MsgFragment(fragment) => fragment.fragment_index,
            PacketType::Ack(ack) => ack.fragment_index,
            PacketType::Nack(nack) => nack.fragment_index,
            _ => 0,
        }
    }

    fn handle_flood(&mut self, packet: Packet) {
        if let PacketType::FloodRequest(mut flood_request) = packet.pack_type {
            // Extract sender_id from the last node in path_trace
            let sender_id = if let Some(&(last_node_id, _)) = flood_request.path_trace.last() {
                last_node_id
            } else {
                // If path_trace is empty, this is the initiator
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
                        hops: flood_request.path_trace.iter().map(|(id, _)| *id).rev().collect(),
                    },
                    session_id: packet.session_id,
                };
                if let Some(sender) = self.packet_send.get(&sender_id) {
                    let _ = sender.send(response_packet);
                }
            } else {
                self.received_floods.insert(flood_request.flood_id);
                // Forward FloodRequest to neighbors except the sender
                for (&neighbor_id, neighbor_sender) in &self.packet_send {
                    if neighbor_id != sender_id {
                        let packet = Packet {
                            pack_type: PacketType::FloodRequest(flood_request.clone()),
                            routing_header: SourceRoutingHeader {
                                hop_index: 0,
                                hops: vec![self.id, neighbor_id],
                            },
                            session_id: packet.session_id,
                        };
                        let _ = neighbor_sender.send(packet);
                    }
                }
            }
        }
    }

    fn handle_command(&mut self, command: Command) {
        match command {
            // Command::AddSender(sender, dst_id) => {
            //     self.packet_send.insert(dst_id, sender);
            // },
            Command::Crash => {
                // Terminate the drone's thread
                std::process::exit(0);
            },
            // Command::SetPacketDropRate(new_pdr) => {
            //     self.pdr = (new_pdr * 100.0) as u8;
            // },
            // Handle other commands as needed
            _ => {}
        }
    }

    // Additional methods can be added as needed
}
