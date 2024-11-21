#![allow(unused)]

use crossbeam_channel::{select, Receiver, Sender};
use std::cmp::max;
use std::collections::{HashMap, HashSet};
use std::thread;
use wg_2024::controller::Command;
use wg_2024::drone::{Drone, DroneOptions};
use wg_2024::network::{NodeId, SourceRoutingHeader};
use wg_2024::packet::{FloodRequest, FloodResponse, NodeType, Packet, PacketType};

pub struct RustBustersDrone {
    id: NodeId,
    sim_contr_send: Sender<Command>,
    sim_contr_recv: Receiver<Command>,
    packet_recv: Receiver<Packet>,
    pdr: u8,
    packet_send: HashMap<NodeId, Sender<Packet>>,
    recived_floods: HashSet<u64>,
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
        }
    }

    fn run(&mut self) {
        loop {
            select! {
                recv(self.packet_recv) -> packet_res => {
                    if let Ok(packet) = packet_res {
                        match packet.pack_type {
                            PacketType::MsgFragment(_fragment) => unimplemented!(),
                            PacketType::Nack(_nack) => unimplemented!(),
                            PacketType::Ack(_ack) => unimplemented!(),
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
    // Todo: Possiamo ottimizzare la risposta del FloodResponse inviandolo al primo che ho tra i vicini (forse?)
    fn handle_flood(&mut self, mut query: FloodRequest) {
        
    }

    fn handle_command() {
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
