#[cfg(test)]
mod forward {
    use crate::drone::test::common::setup_drone;
    use crossbeam_channel::unbounded;
    use wg_2024::controller::DroneEvent;
    use wg_2024::network::SourceRoutingHeader;
    use wg_2024::packet::{Ack, Fragment, NackType, Packet, PacketType, FRAGMENT_DSIZE};

    #[test]
    fn forward_packet_invalid_hop_index() {
        let (mut drone, _, _) = setup_drone();

        // Packet with hop_index greater than the number of hops
        let packet = Packet {
            pack_type: PacketType::Ack(Ack { fragment_index: 0 }),
            routing_header: SourceRoutingHeader {
                hop_index: 10, // Invalid hop_index
                hops: vec![drone.id, 2, 3],
            },
            session_id: 123,
        };

        drone.forward_packet(packet, false);

        // TODO: finish this test and test with negative hop_index
    }

    #[test]
    fn forward_packet_with_non_corresponding_hop_index() {
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
    

    // TODO: test forward if drone is not running
}
