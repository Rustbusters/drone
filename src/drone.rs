use crossbeam_channel::{Receiver, Sender};
use wg_2024::controller::Command;
use wg_2024::drone::{Drone, DroneOptions};
use wg_2024::network::NodeId;
use wg_2024::packet::Packet;

pub struct RustBustersDrone {
    id: NodeId,
    sim_contr_send: Sender<Command>,
    sim_contr_recv: Receiver<Command>,
    packet_recv: Receiver<Packet>,
    pdr: f32,
}

impl Drone for RustBustersDrone {
    fn new(options: DroneOptions) -> Self {
        todo!()
    }

    fn run(&mut self) {
        todo!()
    }
}
