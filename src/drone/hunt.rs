use crate::drone::ShotRange;
use crate::RustBustersDrone;
use wg_2024::controller::NodeEvent;
use wg_2024::network::{NodeId, SourceRoutingHeader};
use wg_2024::packet::{Fragment, Packet, PacketType, FRAGMENT_DSIZE};
pub enum HuntMode {
    NormalShot(NodeId),
    LongShot(ShotRange),
    EMPBlast,
}

impl RustBustersDrone {
    pub fn hunt_ghost(&self, hunt_mode: HuntMode) -> Result<(), String> {
        // Construct the hunt_packet in disguise
        // You can recognize the packet for the length: 0

        // Step 1: construct the data:
        // - NormalShot encoded as 'n' followed by the provided target_node_id
        // - LongShot encoded as 'l' followed by the provided shot_range
        // - EMPBlast encoded as 'e'
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
        // Create NodeEvent
        let kill_node_event = NodeEvent::PacketSent(hunt_packet);

        // Step 3: send the packet to the SC
        self.controller_send
            .send(kill_node_event)
            .expect("Error in sending Hunt Packet");

        Ok(())
    }

    pub fn set_data(&self, data: &mut [u8; FRAGMENT_DSIZE], hunt_mode: HuntMode) {
        match hunt_mode {
            HuntMode::NormalShot(target_drone_id) => {
                RustBustersDrone::set_normal_shot_data(data, target_drone_id)
            }
            HuntMode::LongShot(shot_range) => {
                RustBustersDrone::set_long_shot_data(data, shot_range)
            }
            HuntMode::EMPBlast => RustBustersDrone::set_emp_blast_data(data),
        }
    }

    fn set_normal_shot_data(data: &mut [u8; FRAGMENT_DSIZE], target_drone_id: NodeId) {
        data[0] = 'n' as u8; // encoding of normal shot
        data[1] = target_drone_id;
    }

    fn set_long_shot_data(data: &mut [u8; FRAGMENT_DSIZE], shot_range: ShotRange) {
        data[0] = 'l' as u8; // encoding of long shot
        data[1] = shot_range;
    }

    fn set_emp_blast_data(data: &mut [u8; FRAGMENT_DSIZE]) {
        data[0] = 'e' as u8; // encoding of emp blast
    }
}
