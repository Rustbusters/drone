pub mod forward_packet;
pub mod handle_command;
pub mod handle_flood;
pub mod optimize_route;
pub mod send_ack;
pub mod send_nack;
mod hunt;

use crossbeam_channel::{select_biased, Receiver, Sender};
use log::{debug, info, trace, warn};
use std::collections::{HashMap, HashSet};
use wg_2024::controller::{DroneCommand, NodeEvent};
use wg_2024::drone::{Drone, DroneOptions};
use wg_2024::network::{NodeId, SourceRoutingHeader};
use wg_2024::packet::{Nack, NackType, Packet, PacketType};

pub type ShotRange = u8;

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
    shot_range: ShotRange
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
            hunt_range: 1 // default to 1, TODO: can be set by the SC
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
