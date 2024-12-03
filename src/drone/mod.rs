pub mod forward_packet;
pub mod handle_command;
pub mod handle_flood;
pub mod hunt;
pub mod optimize_route;
pub mod send_nack;
mod test;

use crossbeam_channel::{select_biased, Receiver, Sender};
use log::{debug, info, trace, warn};
use std::collections::{HashMap, HashSet};
use wg_2024::controller::{DroneCommand, DroneEvent};
use wg_2024::drone::Drone;
use wg_2024::network::NodeId;
use wg_2024::packet::{Packet, PacketType};

pub type ShotRange = u8;

pub struct RustBustersDrone {
    id: NodeId,
    controller_send: Sender<DroneEvent>,
    controller_recv: Receiver<DroneCommand>,
    packet_recv: Receiver<Packet>,
    pdr: u8, // Packet Drop Rate in percentage (0-100)
    packet_send: HashMap<NodeId, Sender<Packet>>,
    received_floods: HashSet<u64>,
    optimized_routing: bool,
    running: bool,
    shot_range: ShotRange,
}

impl Drone for RustBustersDrone {
    fn new(
        id: NodeId,
        controller_send: Sender<DroneEvent>,
        controller_recv: Receiver<DroneCommand>,
        packet_recv: Receiver<Packet>,
        packet_send: HashMap<NodeId, Sender<Packet>>,
        pdr: f32, // Packet Drop Rate in percentage (0-100)
    ) -> Self {
        info!("Initializing drone with ID {}", id);
        Self {
            id,
            controller_send,
            controller_recv,
            packet_recv,
            pdr: (pdr * 100.0) as u8,
            packet_send,
            received_floods: HashSet::new(),
            optimized_routing: false,
            running: true,
            shot_range: 0, // TODO: set by SC
        }
    }

    fn run(&mut self) {
        info!("Drone {} starting to run.", self.id);
        while self.running {
            select_biased! {
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
                },
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
            }
        }
        info!("Drone {} has stopped running.", self.id);
    }
}

impl RustBustersDrone {
    #[allow(dead_code)]
    pub fn set_optimized_routing(&mut self, optimized_routing: bool) {
        self.optimized_routing = optimized_routing;
        debug!(
            "Drone {}: Set optimized routing to {}",
            self.id, optimized_routing
        );
    }
}
