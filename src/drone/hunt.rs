use wg_2024::controller::NodeEvent;
use wg_2024::network::{NodeId, SourceRoutingHeader};
use wg_2024::packet::{Fragment, Nack, NackType, Packet, PacketType};
use crate::drone::{ShotRange};
use crate::RustBustersDrone;


pub enum HuntMode {
    NormalShot,
    LongShot(ShotRange),
    EMPBlast
}

impl RustBustersDrone {
    pub fn hunt_ghost(&self, hunt_mode: HuntMode, target_drone_id: Option<NodeId>) -> Result<(), String> {
        // Construct the hunt_packet in disguise
        // You can recognize the packet for the length: 0

        // Step 1: construct the data:
        // - NormalShot encoded as 'n'
        // - LongShot encoded as 'l' followed by the provided range '<range>'
        // - EMPBlast encoded as 'e'
        let mut data = [0; 80];
        match hunt_mode {
            HuntMode::NormalShot => {
                if let Some(target_drone_id) = target_drone_id {
                    RustBustersDrone::set_normal_shot_data(&mut data, target_drone_id);
                } else {
                    return Err("Error: no target_drone_id specified".to_string());
                }
            },
            HuntMode::LongShot(shot_range) => {
                RustBustersDrone::set_long_shot_data(&mut data, shot_range);
            },
            HuntMode::EMPBlast => {
                RustBustersDrone::set_emp_blast_data(&mut data);
            },
        }

        // Step 2: construct the packet with the specified data
        let hunt_packet = Packet {
            pack_type: PacketType::MsgFragment(Fragment {
                fragment_index: 0,
                total_n_fragments: 0,
                length: 0,
                data,
            }),
            routing_header: SourceRoutingHeader { hop_index: 0, hops: vec![] },
            session_id: 0,
        };
        // Create NodeEvent
        let kill_node_event = NodeEvent::PacketSent(hunt_packet);

        // Step 3: send the packet to the SC
        self.controller_send.send(kill_node_event).expect("Error in sending Kill Packet");

        Ok(())
    }

    pub fn set_normal_shot_data(data: &mut [u8; 80], target_drone_id: NodeId) {
        data[0] = 'n' as u8; // encoding of normal shot
        data[1] = target_drone_id;
    }

    pub fn set_long_shot_data(data: &mut [u8; 80], shot_range: ShotRange) {
        data[0] = 'l' as u8; // encoding of long shot
        data[1] = shot_range;
    }

    pub fn set_emp_blast_data(data: &mut [u8; 80]) {
        data[0] = 'e' as u8; // encoding of emp blast
    }
}