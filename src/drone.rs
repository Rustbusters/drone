#![allow(unused)]

use crossbeam_channel::{select, Receiver, Sender};
use rand::Rng;
use std::cmp::max;
use std::collections::{HashMap, HashSet};
use std::thread;
use wg_2024::controller::Command;
use wg_2024::drone::{Drone, DroneOptions};
use wg_2024::network::{NodeId, SourceRoutingHeader};
use wg_2024::packet::{
    Ack, FloodRequest, FloodResponse, Fragment, Nack, NackType, NodeType, Packet, PacketType,
};

pub struct RustBustersDrone {
    id: NodeId,
    sim_contr_send: Sender<Command>,
    sim_contr_recv: Receiver<Command>,
    packet_recv: Receiver<Packet>,
    pdr: u8,
    packet_send: HashMap<NodeId, Sender<Packet>>,
    recived_floods: HashSet<u64>,
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
            packet_send: HashMap::new(),
            recived_floods: HashSet::new(),
            optimized_routing: true,
        }
    }

    fn run(&mut self) {
        loop {
            select! {
                recv(self.packet_recv) -> packet_res => {
                    if let Ok(packet) = packet_res {
                        match packet.pack_type {
                            PacketType::MsgFragment(_) => {
                                self.forward_packet(packet, false);
                            },
                            PacketType::Nack(_) | PacketType::Ack(_) | PacketType::FloodResponse(_) => {
                                self.forward_packet(packet, true);
                            },
                            PacketType::FloodRequest(query) => self.handle_flood(query),
                            PacketType::FloodResponse(_) => unimplemented!(),
                        }
                    }
                },
                recv(self.sim_contr_recv) -> command_res => {
                    if let Ok(_command) = command_res {
                        Self::handle_command()
                    }
                }
            }
        }
    }
}

impl RustBustersDrone {
    fn forward_packet(&mut self, mut packet: Packet, allow_optimized: bool) {
        packet.routing_header.hop_index += 1;

        let hop_index = packet.routing_header.hop_index;
        if hop_index >= packet.routing_header.hops.len() {
            self.send_nack(packet, NackType::DestinationIsDrone, allow_optimized);
            return;
        }
        let next_hop = packet.routing_header.hops[hop_index];

        if !self.packet_send.contains_key(&next_hop) {
            self.send_nack(packet, NackType::ErrorInRouting(next_hop), allow_optimized);
            return;
        }

        let should_drop = match packet.pack_type {
            PacketType::MsgFragment(_) => {
                let mut rng = rand::thread_rng();
                rng.gen_range(0..100) < self.pdr
            }
            _ => false,
        };

        if should_drop {
            self.send_nack(packet, NackType::Dropped, allow_optimized);
            return;
        }

        let next_sender = self.packet_send.get(&next_hop).unwrap(); // Cannot fail because we checked before
        if let Err(e) = next_sender.send(packet.clone()) {
            eprintln!("Error sending packet to {}: {}", next_hop, e);
        } else if let PacketType::MsgFragment(ref fragment) = packet.pack_type {
            self.send_ack(
                packet.session_id,
                fragment.fragment_index,
                &packet.routing_header,
            );
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

        let path_to_client = packet_routing_header.hops[0..hop_index].to_vec();
        let reversed_path: Vec<NodeId> = path_to_client.into_iter().rev().collect();

        let hops = if self.optimized_routing {
            self.optimize_route(&reversed_path)
        } else {
            reversed_path
        };

        let ack = Ack {
            fragment_index,
            time_received: std::time::Instant::now(),
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

        let path_to_sender = packet.routing_header.hops[0..=hop_index].to_vec();
        let reversed_path = path_to_sender.into_iter().rev().collect::<Vec<NodeId>>();

        let hops = if self.optimized_routing && allow_optimized {
            self.optimize_route(&reversed_path)
        } else {
            reversed_path.clone()
        };

        let source_routing_header = SourceRoutingHeader { hop_index: 0, hops };

        let fragment_index = Self::get_fragment_index(&packet);

        let nack = Nack {
            fragment_index,
            time_of_fail: std::time::Instant::now(),
            nack_type,
        };

        let nack_packet = Packet {
            pack_type: PacketType::Nack(nack),
            routing_header: source_routing_header,
            session_id: packet.session_id,
        };

        let next_hop = nack_packet.routing_header.hops[0]; // TODO: check if this is correct

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
        if let Some((index, _)) = path
            .iter()
            .enumerate()
            .rev()
            .find(|&(_, &node_id)| self.packet_send.contains_key(&node_id))
        {
            path[index..].to_vec()
        } else {
            path.to_vec()
        }
    }

    fn get_fragment_index(packet: &Packet) -> u64 {
        match &packet.pack_type {
            PacketType::MsgFragment(fragment) => fragment.fragment_index,
            PacketType::Ack(ack) => ack.fragment_index,
            PacketType::Nack(nack) => nack.fragment_index,
            _ => 0,
        }
    }

    // Todo: Possiamo ottimizzare la risposta del FloodResponse inviandolo al primo che ho tra i vicini (forse?)
    fn handle_flood(&mut self, mut query: FloodRequest) {
        query.path_trace.push((self.id, NodeType::Drone));

        if self.recived_floods.contains(&query.flood_id) {
            let response = FloodResponse {
                flood_id: query.flood_id,
                // TODO: remove with Ricky's PR
                source_routing_header: SourceRoutingHeader {
                    hop_index: 0,
                    hops: vec![],
                },
                path_trace: query.path_trace.clone(),
            };
            let packet = Packet {
                pack_type: PacketType::FloodResponse(response),
                routing_header: SourceRoutingHeader {
                    hop_index: 0,
                    hops: query.path_trace.iter().map(|(id, _)| *id).rev().collect(),
                },
                session_id: 1,
            };
        } else {
            self.recived_floods.insert(query.flood_id);
        }
    }

    fn handle_command() {
        // TODO
        unimplemented!()
    }

    fn add_channel(&mut self, id: NodeId, sender: Sender<Packet>) {
        self.packet_send.insert(id, sender);
    }

    // fn remove_channel(...) {...}
}

#[test]
fn main() {
    let handler = thread::spawn(move || {
        let id = 1;
        let (sim_contr_send, sim_contr_recv) = crossbeam_channel::unbounded();
        let (_packet_send, packet_recv) = crossbeam_channel::unbounded();
        let mut drone = RustBustersDrone::new(DroneOptions {
            id,
            sim_contr_recv,
            sim_contr_send,
            packet_recv,
            pdr: 0.1,
        });

        drone.run();
    });
    handler.join().ok();
}
