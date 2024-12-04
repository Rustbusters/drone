use crate::drone::ShotRange;
use crate::RustBustersDrone;
use wg_2024::controller::DroneEvent;
use wg_2024::network::{NodeId, SourceRoutingHeader};
use wg_2024::packet::{Fragment, Packet, PacketType, FRAGMENT_DSIZE};
use wg_2024::packet::NodeType::Drone;

pub enum HuntMode {
    NormalShot(NodeId, NodeId),
    LongShot(NodeId, ShotRange),
    EMPBlast(NodeId),
}

impl RustBustersDrone {
    pub fn hunt_ghost(&self, hunt_mode: HuntMode) -> Result<(), String> {
        // Construct the hunt_packet
        // You can recognize the packet for the fragment_index: 0, total_n_fragments: 0, length: 0

        // Step 1: construct the data:
        // - NormalShot encoded as 'n' followed by the provided src_node_id and target_node_id
        // - LongShot encoded as 'l' followed by the provided src_node_id and shot_range
        // - EMPBlast encoded as 'e' followed by the provided src_node_id
        let mut data = [0; FRAGMENT_DSIZE];
        self.set_data(&mut data, hunt_mode);

        // Step 2: construct the packet with the specified data
        let hunt_packet = Packet {
            pack_type: PacketType::MsgFragment(Fragment {
                fragment_index: 0,
                total_n_fragments: 0,
                length: 0,
                data,
            }),
            routing_header: SourceRoutingHeader {
                hop_index: 0,
                hops: vec![],
            },
            session_id: 0,
        };
        // Create DroneEvent
        let kill_node_event = DroneEvent::PacketSent(hunt_packet);

        // Step 3: send the packet to the SC
        self.controller_send
            .send(kill_node_event)
            .expect("Error in sending Hunt Packet");

        Ok(())
    }

    pub fn set_data(&self, data: &mut [u8; FRAGMENT_DSIZE], hunt_mode: HuntMode) {
        match hunt_mode {
            HuntMode::NormalShot(src_node_id, target_drone_id) => RustBustersDrone::set_normal_shot_data(data, src_node_id, target_drone_id),
            HuntMode::LongShot(src_node_id, shot_range) => RustBustersDrone::set_long_shot_data(data, src_node_id, shot_range),
            HuntMode::EMPBlast(src_node_id) => RustBustersDrone::set_emp_blast_data(data, src_node_id),
        }
    }

    fn set_normal_shot_data(data: &mut [u8; FRAGMENT_DSIZE], src_node_id: NodeId, target_drone_id: NodeId) {
        data[0] = 'n' as u8; // encoding of normal shot
        data[1] = src_node_id;
        data[2] = target_drone_id;
    }

    fn set_long_shot_data(data: &mut [u8; FRAGMENT_DSIZE], src_node_id: NodeId, shot_range: ShotRange) {
        data[0] = 'l' as u8; // encoding of long shot
        data[1] = src_node_id;
        data[2] = shot_range;
    }

    fn set_emp_blast_data(data: &mut [u8; FRAGMENT_DSIZE], src_node_id: NodeId) {
        data[0] = 'e' as u8; // encoding of emp blast
        data[1] = src_node_id;
    }
}
