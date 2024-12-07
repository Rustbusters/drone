pub mod forward_packet;
pub mod handle_command;
pub mod handle_flood;
pub mod hunt;
pub mod optimize_route;
pub mod send_nack;
mod sounds;
mod test;

use crate::drone::sounds::SPAWN_SOUND;
use crossbeam_channel::{select_biased, Receiver, Sender};
use log::{debug, info, trace, warn};
use rodio::{OutputStream, OutputStreamHandle};
use std::collections::{HashMap, HashSet};
use wg_2024::controller::{DroneCommand, DroneEvent};
use wg_2024::drone::Drone;
use wg_2024::network::NodeId;
use wg_2024::packet::{Packet, PacketType};

pub struct RustBustersDrone {
    id: NodeId,
    controller_send: Sender<DroneEvent>,
    controller_recv: Receiver<DroneCommand>,
    packet_recv: Receiver<Packet>,
    pdr: u8, // Packet Drop Rate in percentage (0-100)
    packet_send: HashMap<NodeId, Sender<Packet>>,
    received_floods: HashSet<(u64, NodeId)>, // flood_id, initiator_id
    optimized_routing: bool,
    running: bool,
    hunt_mode: bool,
    sound_sys: Option<(OutputStream, OutputStreamHandle)>,
}

impl Drone for RustBustersDrone {
    /// Creates a new drone with the given parameters
    /// #### Arguments
    /// - `id`: The ID of the drone
    /// - `controller_send`: The sender to send events to the controller
    /// - `controller_recv`: The receiver to receive commands from the controller
    /// - `packet_recv`: The receiver to receive packets from the network
    /// - `packet_send`: The map of node IDs to senders to send packets to the network
    /// - `pdr`: The Packet Drop Rate in percentage (0-100)
    ///
    /// #### Returns
    /// A new instance of `RustBustersDrone`
    ///
    /// > Note:
    /// > - The `running` field is set to `true` by default
    /// > - The `optimized_routing` field is set to `false` by default
    /// > - The `shot_range` field is set to `0` by default
    fn new(
        id: NodeId,
        controller_send: Sender<DroneEvent>,
        controller_recv: Receiver<DroneCommand>,
        packet_recv: Receiver<Packet>,
        packet_send: HashMap<NodeId, Sender<Packet>>,
        pdr: f32,
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
            hunt_mode: false,
            sound_sys: None,
        }
    }

    /// Runs the drone
    fn run(&mut self) {
        info!("Drone {} starting to run.", self.id);
        self.play_sound(SPAWN_SOUND);
        while self.running || !self.packet_recv.is_empty() {
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
                                PacketType::FloodRequest(_) => self.handle_flood_request(packet),
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
    /// Sets the `optimized_routing` field to the given value
    /// #### Arguments
    /// - `optimized_routing`: The value to set the `optimized_routing` field to
    pub fn set_optimized_routing(&mut self, optimized_routing: bool) {
        self.optimized_routing = optimized_routing;
        debug!(
            "Drone {}: Set optimized routing to {}",
            self.id, optimized_routing
        );
    }

    /// Sets the `hunt_mode` field to the given value
    ///
    /// #### Arguments
    /// - `hunt_mode`: The value to set the `hunt_mode` field to
    pub fn set_hunt_mode(&mut self, hunt_mode: bool) {
        self.hunt_mode = hunt_mode;
        debug!("Drone {}: Set hunt mode to {}", self.id, hunt_mode);
    }

    /// Enables the sound system for the drone
    pub fn enable_sound(&mut self) {
        if let Ok((stream, handle)) = OutputStream::try_default() {
            self.sound_sys = Some((stream, handle));
            info!("Drone {}: Sound system enabled", self.id);
        } else {
            warn!("Drone {}: Error enabling sound system", self.id);
        }
    }
}
