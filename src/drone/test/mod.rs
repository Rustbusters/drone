#[cfg(test)]
mod tests {
    use crate::RustBustersDrone;
    use crossbeam_channel::unbounded;
    use std::collections::{HashMap, HashSet};
    use wg_2024::controller::DroneEvent;
    use wg_2024::network::{NodeId, SourceRoutingHeader};
    use wg_2024::packet::{Fragment, Nack, NackType, Packet, PacketType, FRAGMENT_DSIZE};

    fn setup_drone_with_channels() -> RustBustersDrone {
        let id = 10; // ID del drone
        let (controller_send, _controller_recv) = unbounded();
        let (_cmd_send, cmd_recv) = unbounded();
        let (packet_send_to_2, packet_recv) = unbounded();
        let (packet_send_to_3, _packet_recv_3) = unbounded();
        let mut packet_send = HashMap::new();
        packet_send.insert(2, packet_send_to_2); // Nodo 2 come "neighbor"
        packet_send.insert(3, packet_send_to_3); // Nodo 3 come "neighbor"

        RustBustersDrone {
            id,
            controller_send,
            controller_recv: cmd_recv,
            packet_recv,
            pdr: 10,
            packet_send,
            received_floods: HashSet::default(),
            optimized_routing: false,
            running: false,
            hunt_mode: false,
            sound_sys: None,
        }
    }

    #[test]
    fn test_send_nack_success() {
        // Setup
        let mut drone = setup_drone_with_channels();
        let hops = vec![10, 2, 1, 11]; // Nodo 1 (attuale), Nodo 2 (destinazione)
        let packet = Packet {
            pack_type: PacketType::MsgFragment(Fragment {
                fragment_index: 0,
                total_n_fragments: 1,
                data: [0; FRAGMENT_DSIZE],
                length: FRAGMENT_DSIZE as u8,
            }),
            routing_header: SourceRoutingHeader { hop_index: 3, hops },
            session_id: 123,
        };
        let nack = Nack {
            fragment_index: 0,
            nack_type: NackType::Dropped,
        };

        drone.send_nack(&packet, nack.clone(), false);

        if let Ok(packet) = drone
            .packet_recv
            .recv_timeout(std::time::Duration::from_secs(1))
        {
            match packet.pack_type {
                PacketType::Nack(nack) => {
                    assert_eq!(nack.fragment_index, 0);
                    assert_eq!(nack.nack_type, NackType::Dropped);
                }
                _ => panic!("Pacchetto non atteso: {:?}", packet.pack_type),
            }
        } else {
            panic!("Timeout: nessun pacchetto ricevuto");
        }
    }

    #[test]
    /*fn test_path_optimization_with_loop() {
        let drone = setup_drone_with_channels();

        // in some test cases 2 was not included in the path because it is a neighbor and it produces a different output (correct but different)
        let cycle_path: Vec<NodeId> = vec![drone.id, 3, 4, 5, 3, 6, 7];
        assert_eq!(drone.optimize_route(&cycle_path), vec![drone.id, 3, 6, 7]);

        let cycle_path: Vec<NodeId> = vec![drone.id, 3, drone.id, 5, 3, 6, 7];
        assert_eq!(drone.optimize_route(&cycle_path), vec![drone.id, 3, 6, 7]);

        let optimizable_path: Vec<NodeId> = vec![drone.id, 2, 3, 12];
        assert_eq!(
            drone.optimize_route(&optimizable_path),
            vec![drone.id, 3, 12]
        );

        let non_optimizable_path: Vec<NodeId> = vec![drone.id, 3, 4, 5, 6, 7];
        assert_eq!(
            drone.optimize_route(&non_optimizable_path),
            vec![drone.id, 3, 4, 5, 6, 7]
        );

        let pair_path: Vec<NodeId> = vec![drone.id, drone.id];
        assert_eq!(drone.optimize_route(&pair_path), vec![drone.id]);
    }*/
    fn test_path_optimization() {
        // drone with neighbors 2 and 3
        let drone = setup_drone_with_channels();

        let path: Vec<NodeId> = vec![drone.id, 1, 2, 11];
        assert_eq!(drone.optimize_route(&path), vec![drone.id, 2, 11]);

        let path: Vec<NodeId> = vec![drone.id, 1, 4, 5, 6, 3, 11];
        assert_eq!(drone.optimize_route(&path), vec![drone.id, 3, 11]);
    }

    #[test]
    fn test_hunt_mode() {
        let mut drone = setup_drone_with_channels();
        let (controller_send, controller_recv) = unbounded();
        drone.controller_send = controller_send;

        match drone.hunt_ghost(2) {
            Ok(_) => {}
            Err(e) => {
                panic!("Error: {e}");
            }
        }

        if let Ok(packet) = controller_recv.recv_timeout(std::time::Duration::from_secs(1)) {
            match packet {
                DroneEvent::PacketSent(packet) => match packet.pack_type {
                    PacketType::MsgFragment(fragment) => {
                        assert_eq!(fragment.fragment_index, 0);
                        assert_eq!(fragment.total_n_fragments, 0);
                        assert_eq!(fragment.length, 169);
                        assert_eq!(fragment.data[0], drone.id);
                        assert_eq!(fragment.data[1], 2);
                    }
                    _ => panic!("Pacchetto non atteso: {:?}", packet.pack_type),
                },
                _ => panic!("Evento non atteso: {packet:?}"),
            }
        } else {
            panic!("Timeout: nessun pacchetto ricevuto");
        }
    }
}
