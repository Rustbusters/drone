#[cfg(test)]
mod nack {
    use crate::drone::test::common::{setup_drone, RB_DRONE_ID};
    use crossbeam_channel::unbounded;
    use wg_2024::controller::DroneEvent;
    use wg_2024::network::SourceRoutingHeader;
    use wg_2024::packet::{Fragment, Nack, NackType, Packet, PacketType, FRAGMENT_DSIZE};

    #[test]
    fn test_nack_sent_successfully() {
        // Setup
        let (mut drone, _, _) = setup_drone();
        let (neighbor_2_sender, _) = unbounded();
        drone.packet_send.insert(2, neighbor_2_sender); // Nodo 2 come "neighbor"
        let (neighbor_5_sender, check_recv) = unbounded();
        drone.packet_send.insert(5, neighbor_5_sender); // Nodo 5 come "neighbor"

        let packet = Packet {
            pack_type: PacketType::MsgFragment(Fragment {
                fragment_index: 0,
                total_n_fragments: 1,
                data: [0; FRAGMENT_DSIZE],
                length: FRAGMENT_DSIZE as u8,
            }),
            routing_header: SourceRoutingHeader {
                hop_index: 2,
                hops: vec![5, RB_DRONE_ID, 2, 1, 11],
            },
            session_id: 123,
        };
        let nack = Nack {
            fragment_index: 0,
            nack_type: NackType::Dropped,
        };

        drone.send_nack(&packet, nack.clone(), false);

        if let Ok(packet) = check_recv.recv_timeout(std::time::Duration::from_secs(1)) {
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
    fn test_sending_nack_for_packet_with_hop_index_out_of_range() {
        // Setup
        let (mut drone, _, _) = setup_drone();
        let (neighbor_2_sender, check_recv) = unbounded();
        drone.packet_send.insert(2, neighbor_2_sender); // Nodo 2 come "neighbor"
        let (neighbor_5_sender, _) = unbounded();
        drone.packet_send.insert(5, neighbor_5_sender); // Nodo 5 come "neighbor"

        let packet = Packet {
            pack_type: PacketType::MsgFragment(Fragment {
                fragment_index: 0,
                total_n_fragments: 1,
                data: [0; FRAGMENT_DSIZE],
                length: FRAGMENT_DSIZE as u8,
            }),
            routing_header: SourceRoutingHeader {
                hop_index: 15,
                hops: vec![5, RB_DRONE_ID, 2, 1, 11],
            },
            session_id: 123,
        };
        let nack = Nack {
            fragment_index: 0,
            nack_type: NackType::Dropped,
        };

        drone.send_nack(&packet, nack.clone(), false);

        assert!(check_recv
            .recv_timeout(std::time::Duration::from_secs(1))
            .is_err());
    }

    #[test]
    fn test_sending_nack_for_packet_with_only_one_hop() {
        // Setup
        let (mut drone, _, _) = setup_drone();
        let (neighbor_2_sender, check_recv) = unbounded();
        drone.packet_send.insert(2, neighbor_2_sender); // Nodo 2 come "neighbor"

        let packet = Packet {
            pack_type: PacketType::MsgFragment(Fragment {
                fragment_index: 0,
                total_n_fragments: 1,
                data: [0; FRAGMENT_DSIZE],
                length: FRAGMENT_DSIZE as u8,
            }),
            routing_header: SourceRoutingHeader {
                hop_index: 2,
                hops: vec![RB_DRONE_ID],
            },
            session_id: 123,
        };
        let nack = Nack {
            fragment_index: 0,
            nack_type: NackType::DestinationIsDrone,
        };

        drone.send_nack(&packet, nack.clone(), false);

        assert!(check_recv
            .recv_timeout(std::time::Duration::from_secs(1))
            .is_err());
    }

    #[test]
    fn test_sending_nack_to_sc_if_neighbor_absent() {
        // Setup
        let (mut drone, _, controller_recv) = setup_drone();

        let packet = Packet {
            pack_type: PacketType::MsgFragment(Fragment {
                fragment_index: 0,
                total_n_fragments: 1,
                data: [0; FRAGMENT_DSIZE],
                length: FRAGMENT_DSIZE as u8,
            }),
            routing_header: SourceRoutingHeader {
                hop_index: 2,
                hops: vec![5, RB_DRONE_ID, 2, 1, 11],
            },
            session_id: 123,
        };
        let nack = Nack {
            fragment_index: 0,
            nack_type: NackType::Dropped,
        };

        drone.send_nack(&packet, nack.clone(), false);

        if let Ok(event) = controller_recv.recv_timeout(std::time::Duration::from_secs(1)) {
            match event {
                DroneEvent::ControllerShortcut(packet) => {
                    assert_eq!(packet.session_id, 123);
                    match packet.pack_type {
                        PacketType::Nack(nack) => {
                            assert_eq!(nack.fragment_index, 0);
                            assert_eq!(nack.nack_type, NackType::Dropped);
                        }
                        _ => panic!("Unexpected event: {:?}", packet.pack_type),
                    }
                }
                _ => panic!("Unexpected event: {event:?}"),
            }
        } else {
            panic!("Timeout: no event received");
        }
    }

    #[test]
    fn test_sending_nack_to_sc_if_channel_closed() {
        // Setup
        let (mut drone, _, controller_recv) = setup_drone();
        let (neighbor_2_sender, _) = unbounded();
        drone.packet_send.insert(2, neighbor_2_sender); // Nodo 2 come "neighbor"
        let (neighbor_5_sender, neighbor_5_recv) = unbounded();
        drop(neighbor_5_recv); // Nodo 5 come "neighbor"
        drone.packet_send.insert(5, neighbor_5_sender); // Nodo 5 come "neighbor"

        let packet = Packet {
            pack_type: PacketType::MsgFragment(Fragment {
                fragment_index: 0,
                total_n_fragments: 1,
                data: [0; FRAGMENT_DSIZE],
                length: FRAGMENT_DSIZE as u8,
            }),
            routing_header: SourceRoutingHeader {
                hop_index: 2,
                hops: vec![5, RB_DRONE_ID, 2, 1, 11],
            },
            session_id: 123,
        };
        let nack = Nack {
            fragment_index: 0,
            nack_type: NackType::Dropped,
        };

        drone.send_nack(&packet, nack.clone(), false);

        if let Ok(event) = controller_recv.recv_timeout(std::time::Duration::from_secs(1)) {
            match event {
                DroneEvent::ControllerShortcut(packet) => {
                    assert_eq!(packet.session_id, 123);
                    match packet.pack_type {
                        PacketType::Nack(nack) => {
                            assert_eq!(nack.fragment_index, 0);
                            assert_eq!(nack.nack_type, NackType::Dropped);
                        }
                        _ => panic!("Unexpected event: {:?}", packet.pack_type),
                    }
                }
                _ => panic!("Unexpected event: {event:?}"),
            }
        } else {
            panic!("Timeout: no event received");
        }
    }

    #[test]
    fn test_sending_nack_with_optimized_routing() {
        // Setup
        let (mut drone, _, _) = setup_drone();
        let (neighbor_2_sender, _) = unbounded();
        drone.packet_send.insert(2, neighbor_2_sender); // Nodo 2 come "neighbor"
        let (neighbor_5_sender, check_recv) = unbounded();
        drone.packet_send.insert(5, neighbor_5_sender); // Nodo 5 come "neighbor"
        let (neighbor_3_sender, _) = unbounded();
        drone.packet_send.insert(3, neighbor_3_sender); // Nodo 5 come "neighbor"

        drone.set_optimized_routing(true);

        let packet = Packet {
            pack_type: PacketType::MsgFragment(Fragment {
                fragment_index: 0,
                total_n_fragments: 1,
                data: [0; FRAGMENT_DSIZE],
                length: FRAGMENT_DSIZE as u8,
            }),
            routing_header: SourceRoutingHeader {
                hop_index: 5,
                hops: vec![5, 12, 13, 3, RB_DRONE_ID, 2],
            },
            session_id: 123,
        };
        let nack = Nack {
            fragment_index: 0,
            nack_type: NackType::Dropped,
        };

        drone.send_nack(&packet, nack.clone(), true);

        if let Ok(packet) = check_recv.recv_timeout(std::time::Duration::from_secs(1)) {
            match packet.pack_type {
                PacketType::Nack(nack) => {
                    assert_eq!(nack.fragment_index, 0);
                    assert_eq!(nack.nack_type, NackType::Dropped);
                    assert_eq!(packet.routing_header.hops, vec![RB_DRONE_ID, 5]);
                }
                _ => panic!("Pacchetto non atteso: {:?}", packet.pack_type),
            }
        } else {
            panic!("Timeout: nessun pacchetto ricevuto");
        }
    }
}
