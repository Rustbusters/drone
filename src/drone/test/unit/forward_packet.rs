#[cfg(test)]
mod forward {
    use crate::drone::test::common::{setup_drone, RB_DRONE_ID, UNKNOWN_NODE};
    use crate::hunt::PACKET_CONST;
    use crossbeam_channel::unbounded;
    use wg_2024::controller::DroneEvent;
    use wg_2024::network::SourceRoutingHeader;
    use wg_2024::packet::{Ack, Fragment, Nack, NackType, Packet, PacketType, FRAGMENT_DSIZE};

    #[test]
    fn test_forward_packet_invalid_hop_index() {
        let (mut drone, _, _) = setup_drone();
        let (neighbor_5_sender, neighbor_5_receiver) = unbounded();
        drone.packet_send.insert(5, neighbor_5_sender);
        let (neighbor_2_sender, _) = unbounded();
        drone.packet_send.insert(2, neighbor_2_sender);

        // Packet with hop_index greater than the number of hops
        let packet = Packet {
            pack_type: PacketType::MsgFragment(Fragment {
                fragment_index: 0,
                total_n_fragments: 1,
                data: [0; FRAGMENT_DSIZE],
                length: FRAGMENT_DSIZE as u8,
            }),
            routing_header: SourceRoutingHeader {
                hop_index: 10, // Invalid hop_index
                hops: vec![5, drone.id, 2, 3],
            },
            session_id: 123,
        };

        drone.forward_packet(packet, false);

        if let Ok(packet) = neighbor_5_receiver.recv_timeout(std::time::Duration::from_secs(1)) {
            match packet.pack_type {
                PacketType::Nack(nack) => {
                    assert_eq!(nack.fragment_index, 0);

                    match nack.nack_type {
                        NackType::UnexpectedRecipient(e) => {
                            assert_eq!(e, RB_DRONE_ID);
                        }
                        _ => panic!("Unexpected nack type: {:?}", nack.nack_type),
                    }
                }
                _ => panic!("Unexpected packet: {:?}", packet.pack_type),
            }
        } else {
            panic!("Timeout: no packet received");
        }
    }

    #[test]
    fn test_forward_packet_with_non_corresponding_hop_index() {
        let (mut drone, _controller_send, controller_recv) = setup_drone();

        let (neighbor_1_sender, neighbor_1_receiver) = unbounded();
        drone.packet_send.insert(1, neighbor_1_sender);

        let packet = Packet {
            pack_type: PacketType::MsgFragment(Fragment {
                fragment_index: 0,
                total_n_fragments: 1,
                data: [0; FRAGMENT_DSIZE],
                length: FRAGMENT_DSIZE as u8,
            }),
            routing_header: SourceRoutingHeader {
                hop_index: 2,
                hops: vec![1, drone.id, 2, 3],
            },
            session_id: 123,
        };

        drone.forward_packet(packet, false);

        if let Ok(packet) = neighbor_1_receiver.recv_timeout(std::time::Duration::from_secs(1)) {
            match packet.pack_type {
                PacketType::Nack(nack) => {
                    assert_eq!(nack.fragment_index, 0);

                    match nack.nack_type {
                        NackType::UnexpectedRecipient(e) => {
                            assert_eq!(e, drone.id);
                        }
                        _ => panic!("Unexpected nack type: {:?}", nack.nack_type),
                    }
                }
                _ => panic!("Unexpected packet: {:?}", packet.pack_type),
            }
        } else {
            panic!("Timeout: no packet received");
        }

        if let Ok(event) = controller_recv.recv_timeout(std::time::Duration::from_secs(1)) {
            match event {
                DroneEvent::PacketSent(packet) => match packet.pack_type {
                    PacketType::Nack(nack) => {
                        assert_eq!(nack.fragment_index, 0);

                        match nack.nack_type {
                            NackType::UnexpectedRecipient(e) => {
                                assert_eq!(e, 1);
                            }
                            _ => panic!("Unexpected nack type: {:?}", nack.nack_type),
                        }
                    }
                    _ => panic!("Unexpected packet: {:?}", packet.pack_type),
                },
                _ => panic!("Unexpected event: {:?}", event),
            }
        }
    }

    #[test]
    fn test_forward_packet_after_crash() {
        let (mut drone, _, _) = setup_drone();
        let (neighbor_1_sender, neighbor_1_receiver) = unbounded();
        drone.packet_send.insert(1, neighbor_1_sender);
        let (neighbor_3_sender, _neighbor_3_receiver) = unbounded();
        drone.packet_send.insert(3, neighbor_3_sender);

        drone.running = false;

        let packet = Packet {
            pack_type: PacketType::MsgFragment(Fragment {
                fragment_index: 0,
                total_n_fragments: 1,
                data: [0; FRAGMENT_DSIZE],
                length: FRAGMENT_DSIZE as u8,
            }),
            routing_header: SourceRoutingHeader {
                hop_index: 1,
                hops: vec![1, RB_DRONE_ID, 3],
            },
            session_id: 123,
        };

        drone.forward_packet(packet, false);

        if let Ok(packet) = neighbor_1_receiver.recv_timeout(std::time::Duration::from_secs(1)) {
            match packet.pack_type {
                PacketType::Nack(nack) => {
                    assert_eq!(nack.fragment_index, 0);

                    match nack.nack_type {
                        NackType::ErrorInRouting(RB_DRONE_ID) => {}
                        _ => panic!("Unexpected nack type: {:?}", nack.nack_type),
                    }
                }
                _ => panic!("Unexpected packet: {:?}", packet.pack_type),
            }
        } else {
            panic!("Timeout: no packet received");
        }
    }

    #[test]
    fn test_forward_packet_when_drone_is_destination() {
        let (mut drone, _, _) = setup_drone();
        let (neighbor_1_sender, neighbor_1_receiver) = unbounded();
        drone.packet_send.insert(1, neighbor_1_sender);

        let packet = Packet {
            pack_type: PacketType::MsgFragment(Fragment {
                fragment_index: 0,
                total_n_fragments: 1,
                data: [0; FRAGMENT_DSIZE],
                length: FRAGMENT_DSIZE as u8,
            }),
            routing_header: SourceRoutingHeader {
                hop_index: 1,
                hops: vec![1, drone.id],
            },
            session_id: 123,
        };

        drone.forward_packet(packet, false);

        if let Ok(packet) = neighbor_1_receiver.recv_timeout(std::time::Duration::from_secs(1)) {
            match packet.pack_type {
                PacketType::Nack(nack) => {
                    assert_eq!(nack.fragment_index, 0);

                    match nack.nack_type {
                        NackType::DestinationIsDrone => {
                            assert_eq!(nack.fragment_index, 0);
                            assert_eq!(packet.session_id, 123);
                        }
                        _ => panic!("Unexpected nack type: {:?}", nack.nack_type),
                    }
                }
                _ => panic!("Unexpected packet: {:?}", packet.pack_type),
            }
        } else {
            panic!("Timeout: no packet received");
        }
    }

    #[test]
    fn test_forward_packet_when_next_hop_is_not_neighbor() {
        let (mut drone, _, _) = setup_drone();
        let (neighbor_1_sender, neighbor_1_receiver) = unbounded();
        drone.packet_send.insert(1, neighbor_1_sender);

        let packet = Packet {
            pack_type: PacketType::MsgFragment(Fragment {
                fragment_index: 0,
                total_n_fragments: 1,
                data: [0; FRAGMENT_DSIZE],
                length: FRAGMENT_DSIZE as u8,
            }),
            routing_header: SourceRoutingHeader {
                hop_index: 1,
                hops: vec![1, drone.id, 3],
            },
            session_id: 123,
        };

        drone.forward_packet(packet, false);

        if let Ok(packet) = neighbor_1_receiver.recv_timeout(std::time::Duration::from_secs(1)) {
            match packet.pack_type {
                PacketType::Nack(nack) => {
                    assert_eq!(nack.fragment_index, 0);

                    match nack.nack_type {
                        NackType::ErrorInRouting(id) => {
                            assert_eq!(id, 3);
                        }
                        _ => panic!("Unexpected nack type: {:?}", nack.nack_type),
                    }
                }
                _ => panic!("Unexpected packet: {:?}", packet.pack_type),
            }
        } else {
            panic!("Timeout: no packet received");
        }
    }

    #[test]
    fn test_forward_nack_without_hunt_mode() {
        let (mut drone, _, _) = setup_drone();
        let (neighbor_1_sender, _neighbor_1_receiver) = unbounded();
        drone.packet_send.insert(1, neighbor_1_sender);
        let (neighbor_3_sender, neighbor_3_receiver) = unbounded();
        drone.packet_send.insert(3, neighbor_3_sender);

        let packet = Packet {
            pack_type: PacketType::Nack(Nack {
                fragment_index: 0,
                nack_type: NackType::Dropped,
            }),
            routing_header: SourceRoutingHeader {
                hop_index: 1,
                hops: vec![1, drone.id, 3],
            },
            session_id: 123,
        };

        drone.forward_packet(packet, false);

        if let Ok(packet) = neighbor_3_receiver.recv_timeout(std::time::Duration::from_secs(1)) {
            match packet.pack_type {
                PacketType::Nack(nack) => {
                    assert_eq!(nack.fragment_index, 0);

                    match nack.nack_type {
                        NackType::Dropped => {
                            assert_eq!(nack.fragment_index, 0);
                            assert_eq!(packet.session_id, 123);
                        }
                        _ => panic!("Unexpected nack type: {:?}", nack.nack_type),
                    }
                }
                _ => panic!("Unexpected packet: {:?}", packet.pack_type),
            }
        } else {
            panic!("Timeout: no packet received");
        }
    }

    #[test]
    fn test_forward_nack_with_hunt_mode() {
        let (mut drone, _, controller_recv) = setup_drone();
        let (neighbor_2_sender, _neighbor_2_receiver) = unbounded();
        drone.packet_send.insert(2, neighbor_2_sender);
        let (neighbor_3_sender, neighbor_3_receiver) = unbounded();
        drone.packet_send.insert(3, neighbor_3_sender);

        drone.set_hunt_mode(true);

        let packet = Packet {
            pack_type: PacketType::Nack(Nack {
                fragment_index: 0,
                nack_type: NackType::Dropped,
            }),
            routing_header: SourceRoutingHeader {
                hop_index: 2,
                hops: vec![1, 2, drone.id, 3],
            },
            session_id: 123,
        };

        drone.forward_packet(packet, false);

        if let Ok(packet) = neighbor_3_receiver.recv_timeout(std::time::Duration::from_secs(1)) {
            match packet.pack_type {
                PacketType::Nack(nack) => {
                    assert_eq!(nack.fragment_index, 0);

                    match nack.nack_type {
                        NackType::Dropped => {
                            assert_eq!(nack.fragment_index, 0);
                            assert_eq!(packet.session_id, 123);
                        }
                        _ => panic!("Unexpected nack type: {:?}", nack.nack_type),
                    }
                }
                _ => panic!("Unexpected packet: {:?}", packet.pack_type),
            }
        } else {
            panic!("Timeout: no packet received");
        }

        if let Ok(event) = controller_recv.recv_timeout(std::time::Duration::from_secs(1)) {
            match event {
                DroneEvent::PacketSent(packet) => match packet.pack_type {
                    PacketType::MsgFragment(fragment) => {
                        assert_eq!(fragment.fragment_index, 0);
                        assert_eq!(fragment.total_n_fragments, 0);
                        assert_eq!(fragment.length, PACKET_CONST);
                        assert_eq!(fragment.data[0], drone.id);
                        assert_eq!(fragment.data[1], 1);
                    }
                    _ => panic!("Unexpected packet: {:?}", packet.pack_type),
                },
                _ => panic!("Unexpected event: {:?}", event),
            }
        }
    }

    #[test]
    fn test_forward_nack_ack_or_flood_response_without_optimized_routing() {
        let (mut drone, _, _) = setup_drone();
        let (neighbor_1_sender, _neighbor_1_receiver) = unbounded();
        drone.packet_send.insert(1, neighbor_1_sender);
        let (neighbor_3_sender, neighbor_3_receiver) = unbounded();
        drone.packet_send.insert(3, neighbor_3_sender);
        let (neighbor_4_sender, neighbor_4_receiver) = unbounded();
        drone.packet_send.insert(4, neighbor_4_sender);

        let packet = Packet {
            pack_type: PacketType::Ack(wg_2024::packet::Ack { fragment_index: 0 }),
            routing_header: SourceRoutingHeader {
                hop_index: 1,
                hops: vec![1, drone.id, 3, 6, 7, 4, 5],
            },
            session_id: 123,
        };

        drone.forward_packet(packet, false);

        if let Ok(packet) = neighbor_3_receiver.recv_timeout(std::time::Duration::from_secs(1)) {
            match packet.pack_type {
                PacketType::Ack(ack) => {
                    assert_eq!(ack.fragment_index, 0);
                    assert_eq!(packet.session_id, 123);
                }
                _ => panic!("Unexpected packet: {:?}", packet.pack_type),
            }
        } else {
            panic!("Timeout: no packet received");
        }

        assert!(neighbor_4_receiver
            .recv_timeout(std::time::Duration::from_secs(1))
            .is_err());
    }

    #[test]
    fn test_forward_nack_or_flood_response_with_optimized_routing() {
        let (mut drone, _, _) = setup_drone();
        let (neighbor_1_sender, _neighbor_1_receiver) = unbounded();
        drone.packet_send.insert(1, neighbor_1_sender);
        let (neighbor_3_sender, neighbor_3_receiver) = unbounded();
        drone.packet_send.insert(3, neighbor_3_sender);
        let (neighbor_4_sender, neighbor_4_receiver) = unbounded();
        drone.packet_send.insert(4, neighbor_4_sender);

        drone.set_optimized_routing(true);

        let packet = Packet {
            pack_type: PacketType::Nack(Nack {
                fragment_index: 0,
                nack_type: NackType::DestinationIsDrone,
            }),
            routing_header: SourceRoutingHeader {
                hop_index: 1,
                hops: vec![1, drone.id, 3, 6, 7, 4, 5],
            },
            session_id: 123,
        };

        drone.forward_packet(packet, false);

        if let Ok(packet) = neighbor_4_receiver.recv_timeout(std::time::Duration::from_secs(1)) {
            match packet.pack_type {
                PacketType::Nack(nack) => {
                    assert_eq!(nack.fragment_index, 0);
                    assert_eq!(nack.nack_type, NackType::DestinationIsDrone);
                    assert_eq!(packet.session_id, 123);
                }
                _ => panic!("Unexpected packet: {:?}", packet.pack_type),
            }
        } else {
            panic!("Timeout: no packet received");
        }

        assert!(neighbor_3_receiver
            .recv_timeout(std::time::Duration::from_secs(1))
            .is_err());
    }

    #[test]
    fn test_forward_ack_with_optimized_routing() {
        let (mut drone, _, _) = setup_drone();
        let (neighbor_1_sender, _neighbor_1_receiver) = unbounded();
        drone.packet_send.insert(1, neighbor_1_sender);
        let (neighbor_3_sender, neighbor_3_receiver) = unbounded();
        drone.packet_send.insert(3, neighbor_3_sender);

        drone.set_optimized_routing(true);

        let packet = Packet {
            pack_type: PacketType::Ack(Ack { fragment_index: 0 }),
            routing_header: SourceRoutingHeader {
                hop_index: 1,
                hops: vec![1, drone.id, 3, 6, 7, 4, 5],
            },
            session_id: 123,
        };

        drone.forward_packet(packet, false);

        if let Ok(packet) = neighbor_3_receiver.recv_timeout(std::time::Duration::from_secs(1)) {
            match packet.pack_type {
                PacketType::Ack(ack) => {
                    assert_eq!(ack.fragment_index, 0);
                    assert_eq!(packet.session_id, 123);
                }
                _ => panic!("Unexpected packet: {:?}", packet.pack_type),
            }
        } else {
            panic!("Timeout: no packet received");
        }
    }

    #[test]
    fn test_forward_nack_ack_or_flood_response_when_neighbor_channel_closed() {
        let (mut drone, _, controller_recv) = setup_drone();
        let (neighbor_1_sender, _neighbor_1_receiver) = unbounded();
        drone.packet_send.insert(1, neighbor_1_sender);
        let (neighbor_3_sender, neighbor_3_receiver) = unbounded();
        drone.packet_send.insert(3, neighbor_3_sender);

        drop(neighbor_3_receiver);

        let packet = Packet {
            pack_type: PacketType::Ack(Ack {
                fragment_index: 3243,
            }),
            routing_header: SourceRoutingHeader {
                hop_index: 1,
                hops: vec![1, drone.id, 3],
            },
            session_id: 123,
        };

        drone.forward_packet(packet, false);

        if let Ok(event) = controller_recv.recv_timeout(std::time::Duration::from_secs(1)) {
            match event {
                DroneEvent::ControllerShortcut(packet) => {
                    assert_eq!(packet.session_id, 123);
                    match packet.pack_type {
                        PacketType::Ack(ack) => {
                            assert_eq!(ack.fragment_index, 3243);
                        }
                        _ => panic!("Unexpected event: {:?}", packet.pack_type),
                    }
                }
                _ => panic!("Unexpected event: {event:?}"),
            }
        } else {
            panic!("Timeout: no event received");
        }
        assert!(!drone.packet_send.contains_key(&UNKNOWN_NODE));
    }

    #[test]
    fn test_forward_nack_ack_or_flood_response_when_neighbor_is_removed_after_first_check() {
        let (mut drone, _, controller_recv) = setup_drone();
        let (neighbor_1_sender, _neighbor_1_receiver) = unbounded();
        drone.packet_send.insert(1, neighbor_1_sender);
        let (neighbor_3_sender, neighbor_3_receiver) = unbounded();
        drone.packet_send.insert(3, neighbor_3_sender);

        drop(neighbor_3_receiver);

        let mut packet = Packet {
            pack_type: PacketType::Ack(Ack {
                fragment_index: 3243,
            }),
            routing_header: SourceRoutingHeader {
                hop_index: 1,
                hops: vec![1, drone.id, 4, 3],
            },
            session_id: 123,
        };

        drone.forward_other_packet(&mut packet);

        if let Ok(event) = controller_recv.recv_timeout(std::time::Duration::from_secs(1)) {
            match event {
                DroneEvent::ControllerShortcut(packet) => {
                    assert_eq!(packet.session_id, 123);
                    match packet.pack_type {
                        PacketType::Ack(ack) => {
                            assert_eq!(ack.fragment_index, 3243);
                        }
                        _ => panic!("Unexpected event: {:?}", packet.pack_type),
                    }
                }
                _ => panic!("Unexpected event: {event:?}"),
            }
        } else {
            panic!("Timeout: no event received");
        }
        assert!(!drone.packet_send.contains_key(&UNKNOWN_NODE));
    }

    #[test]
    fn test_forward_fragment_with_100_as_pdr() {
        let (mut drone, _, controller_recv) = setup_drone();
        let (neighbor_1_sender, neighbor_1_receiver) = unbounded();
        drone.packet_send.insert(1, neighbor_1_sender);
        let (neighbor_5_sender, _neighbor_5_receiver) = unbounded();
        drone.packet_send.insert(5, neighbor_5_sender);

        drone.pdr = 100;

        let packet = Packet {
            pack_type: PacketType::MsgFragment(Fragment {
                fragment_index: 0,
                total_n_fragments: 1,
                data: [0; FRAGMENT_DSIZE],
                length: FRAGMENT_DSIZE as u8,
            }),
            routing_header: SourceRoutingHeader {
                hop_index: 1,
                hops: vec![1, drone.id, 5],
            },
            session_id: 123,
        };

        drone.forward_packet(packet, false);

        if let Ok(packet) = neighbor_1_receiver.recv_timeout(std::time::Duration::from_secs(1)) {
            match packet.pack_type {
                PacketType::Nack(nack) => {
                    assert_eq!(nack.fragment_index, 0);

                    match nack.nack_type {
                        NackType::Dropped => {
                            assert_eq!(nack.fragment_index, 0);
                            assert_eq!(packet.session_id, 123);
                        }
                        _ => panic!("Unexpected nack type: {:?}", nack.nack_type),
                    }
                }
                _ => panic!("Unexpected packet: {:?}", packet.pack_type),
            }
        } else {
            panic!("Timeout: no packet received");
        }

        if let Ok(event) = controller_recv.recv_timeout(std::time::Duration::from_secs(1)) {
            match event {
                DroneEvent::PacketDropped(packet) => match packet.pack_type {
                    PacketType::MsgFragment(fragment) => {
                        assert_eq!(fragment.fragment_index, 0);
                        assert_eq!(fragment.total_n_fragments, 1);
                        assert_eq!(fragment.length, FRAGMENT_DSIZE as u8);
                        assert_eq!(fragment.data[0], 0);
                    }
                    _ => panic!("Unexpected packet: {:?}", packet.pack_type),
                },
                _ => panic!("Unexpected event: {event:?}"),
            }
        } else {
            panic!("Timeout: no event received");
        }
    }

    #[test]
    fn test_forward_fragment_with_0_as_pdr() {
        let (mut drone, _, _) = setup_drone();
        let (neighbor_1_sender, _neighbor_1_receiver) = unbounded();
        drone.packet_send.insert(1, neighbor_1_sender);
        let (neighbor_5_sender, neighbor_5_receiver) = unbounded();
        drone.packet_send.insert(5, neighbor_5_sender);

        drone.pdr = 0;

        let packet = Packet {
            pack_type: PacketType::MsgFragment(Fragment {
                fragment_index: 0,
                total_n_fragments: 1,
                data: [0; FRAGMENT_DSIZE],
                length: FRAGMENT_DSIZE as u8,
            }),
            routing_header: SourceRoutingHeader {
                hop_index: 1,
                hops: vec![1, drone.id, 5],
            },
            session_id: 123,
        };

        drone.forward_packet(packet, false);

        if let Ok(packet) = neighbor_5_receiver.recv_timeout(std::time::Duration::from_secs(1)) {
            match packet.pack_type {
                PacketType::MsgFragment(fragment) => {
                    assert_eq!(fragment.fragment_index, 0);
                    assert_eq!(fragment.total_n_fragments, 1);
                    assert_eq!(fragment.length, FRAGMENT_DSIZE as u8);
                    assert_eq!(fragment.data[0], 0);
                }
                _ => panic!("Unexpected packet: {:?}", packet.pack_type),
            }
        } else {
            panic!("Timeout: no packet received");
        }
    }

    #[test]
    fn test_forward_fragment_with_optimized_routing_when_drone_is_destination() {
        let (mut drone, _, _) = setup_drone();
        let (neighbor_1_sender, neighbor_1_receiver) = unbounded();
        drone.packet_send.insert(1, neighbor_1_sender);
        let (neighbor_3_sender, _neighbor_3_receiver) = unbounded();
        drone.packet_send.insert(3, neighbor_3_sender);

        drone.set_optimized_routing(true);

        let packet = Packet {
            pack_type: PacketType::MsgFragment(Fragment {
                fragment_index: 0,
                total_n_fragments: 1,
                data: [0; FRAGMENT_DSIZE],
                length: FRAGMENT_DSIZE as u8,
            }),
            routing_header: SourceRoutingHeader {
                hop_index: 1,
                hops: vec![1, drone.id, 3, 6, 7, drone.id],
            },
            session_id: 123,
        };

        drone.forward_packet(packet, false);

        if let Ok(packet) = neighbor_1_receiver.recv_timeout(std::time::Duration::from_secs(1)) {
            match packet.pack_type {
                PacketType::Nack(nack) => {
                    assert_eq!(nack.fragment_index, 0);

                    match nack.nack_type {
                        NackType::DestinationIsDrone => {
                            assert_eq!(nack.fragment_index, 0);
                            assert_eq!(packet.session_id, 123);
                        }
                        _ => panic!("Unexpected nack type: {:?}", nack.nack_type),
                    }
                }
                _ => panic!("Unexpected packet: {:?}", packet.pack_type),
            }
        } else {
            panic!("Timeout: no packet received");
        }
    }

    #[test]
    fn test_forward_fragment_when_neighbor_channel_closed() {
        let (mut drone, _, _) = setup_drone();
        let (neighbor_1_sender, neighbor_1_receiver) = unbounded();
        drone.packet_send.insert(1, neighbor_1_sender);
        let (neighbor_5_sender, neighbor_5_receiver) = unbounded();
        drone.packet_send.insert(5, neighbor_5_sender);

        drop(neighbor_5_receiver);

        let packet = Packet {
            pack_type: PacketType::MsgFragment(Fragment {
                fragment_index: 0,
                total_n_fragments: 1,
                data: [0; FRAGMENT_DSIZE],
                length: FRAGMENT_DSIZE as u8,
            }),
            routing_header: SourceRoutingHeader {
                hop_index: 1,
                hops: vec![1, drone.id, 5],
            },
            session_id: 123,
        };

        drone.forward_packet(packet, false);

        if let Ok(packet) = neighbor_1_receiver.recv_timeout(std::time::Duration::from_secs(1)) {
            match packet.pack_type {
                PacketType::Nack(nack) => {
                    assert_eq!(nack.fragment_index, 0);

                    match nack.nack_type {
                        NackType::ErrorInRouting(5) => {
                            assert_eq!(nack.fragment_index, 0);
                            assert_eq!(packet.session_id, 123);
                        }
                        _ => panic!("Unexpected nack type: {:?}", nack.nack_type),
                    }
                }
                _ => panic!("Unexpected packet: {:?}", packet.pack_type),
            }
        } else {
            panic!("Timeout: no packet received");
        }
    }

    #[test]
    fn test_forward_fragment_when_neighbor_is_removed_after_first_check() {
        let (mut drone, _, _) = setup_drone();
        let (neighbor_1_sender, neighbor_1_receiver) = unbounded();
        drone.packet_send.insert(1, neighbor_1_sender);

        drone.pdr = 0;

        let fragment = Fragment {
            fragment_index: 0,
            total_n_fragments: 1,
            data: [0; FRAGMENT_DSIZE],
            length: FRAGMENT_DSIZE as u8,
        };
        let packet = Packet {
            pack_type: PacketType::MsgFragment(fragment.clone()),
            routing_header: SourceRoutingHeader {
                hop_index: 2, // 2 because handle_fragment is called after the index increment
                hops: vec![1, drone.id, 5],
            },
            session_id: 123,
        };

        drone.handle_fragment(&packet, &fragment, 5, drone.optimized_routing);

        if let Ok(packet) = neighbor_1_receiver.recv_timeout(std::time::Duration::from_secs(1)) {
            match packet.pack_type {
                PacketType::Nack(nack) => {
                    assert_eq!(nack.fragment_index, 0);

                    match nack.nack_type {
                        NackType::ErrorInRouting(5) => {
                            assert_eq!(nack.fragment_index, 0);
                            assert_eq!(packet.session_id, 123);
                        }
                        _ => panic!("Unexpected nack type: {:?}", nack.nack_type),
                    }
                }
                _ => panic!("Unexpected packet: {:?}", packet.pack_type),
            }
        } else {
            panic!("Timeout: no packet received");
        }
    }
}
