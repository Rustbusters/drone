#![allow(unused)]
use crate::RustBustersDrone;
use crossbeam_channel::{unbounded, Receiver, Sender};
use std::collections::{HashMap, HashSet};
use wg_2024::controller::DroneEvent;
use wg_2024::network::NodeId;

pub const RB_DRONE_ID: NodeId = 10;
pub const UNKNOWN_NODE: NodeId = 99;

pub fn setup_drone() -> (RustBustersDrone, Sender<DroneEvent>, Receiver<DroneEvent>) {
    let (controller_send, controller_recv) = unbounded();
    let (_cmd_send, cmd_recv) = unbounded();
    let (_packet_send_to_drone, packet_recv) = unbounded();
    let packet_send = HashMap::new();

    let drone = RustBustersDrone {
        id: RB_DRONE_ID,
        controller_send: controller_send.clone(),
        controller_recv: cmd_recv,
        packet_recv,
        pdr: 10,
        packet_send,
        received_floods: HashSet::default(),
        optimized_routing: false,
        running: true,
        hunt_mode: false,
    };

    (drone, controller_send, controller_recv)
}
