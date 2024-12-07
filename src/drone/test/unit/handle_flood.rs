#[cfg(test)]
mod flooding {
    use crate::drone::test::common::{setup_drone, UNKNOWN_NODE};
    use crossbeam_channel::unbounded;
    use wg_2024::controller::DroneEvent;
    use wg_2024::network::SourceRoutingHeader;
    use wg_2024::packet::NodeType::{Client, Drone, Server};
    use wg_2024::packet::{FloodRequest, Fragment, Packet, PacketType, FRAGMENT_DSIZE};

    #[test]
    fn test_flood_response_to_sc_if_neighbor_absent() {
        // Tests the flood response is sent to SC when the neighbor is absent
        let (mut drone, _, controller_recv) = setup_drone();

        let flood_request = FloodRequest {
            flood_id: 123,
            initiator_id: 1,
            path_trace: vec![(1, Client), (drone.id, Drone)],
        };

        drone.send_flood_response(&flood_request, 42, UNKNOWN_NODE);

        if let Ok(event) = controller_recv.recv_timeout(std::time::Duration::from_secs(1)) {
            match event {
                DroneEvent::ControllerShortcut(packet) => {
                    assert_eq!(packet.session_id, 42);
                    match packet.pack_type {
                        PacketType::FloodResponse(response) => {
                            assert_eq!(response.flood_id, 123);
                            assert_eq!(response.path_trace, vec![(1, Client), (drone.id, Drone)]);
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
    fn test_flood_response_to_sc_if_neighbor_channel_closed() {
        // Tests the flood response is sent to SC when the neighbor's channel is closed
        let (mut drone, _, controller_recv) = setup_drone();

        let (neighbor_sender, _) = unbounded(); // Immediately closed channel
        drone.packet_send.insert(UNKNOWN_NODE, neighbor_sender);

        let flood_request = FloodRequest {
            flood_id: 123,
            initiator_id: 1,
            path_trace: vec![(1, Client), (drone.id, Drone)],
        };

        drone.send_flood_response(&flood_request, 42, UNKNOWN_NODE);

        if let Ok(event) = controller_recv.recv_timeout(std::time::Duration::from_secs(1)) {
            match event {
                DroneEvent::ControllerShortcut(packet) => {
                    assert_eq!(packet.session_id, 42);
                    match packet.pack_type {
                        PacketType::FloodResponse(response) => {
                            assert_eq!(response.flood_id, 123);
                            assert_eq!(response.path_trace, vec![(1, Client), (drone.id, Drone)]);
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
    fn test_no_flood_request_to_unknown_neighbor() {
        // Tests no flood request is sent to an unknown neighbor
        let (mut drone, _, controller_recv) = setup_drone();
        let (neighbor_1_sender, _neighbor_1_recv) = unbounded();
        drone.packet_send.insert(1, neighbor_1_sender);

        let flood_request = FloodRequest {
            flood_id: 123,
            initiator_id: 1,
            path_trace: vec![(1, Server), (drone.id, Drone)],
        };

        drone.spread_flood_request(&flood_request, 42, UNKNOWN_NODE);

        assert!(
            controller_recv.try_recv().is_err(),
            "No event should be sent to the SC"
        );

        assert!(
            !drone.packet_send.contains_key(&UNKNOWN_NODE),
            "Unknown node should not be added"
        );
    }

    #[test]
    fn test_flood_request_if_neighbor_channel_closed() {
        // Tests the removal of a neighbor when its channel is closed
        let (mut drone, _, controller_recv) = setup_drone();

        let (neighbor_sender, neighbor_recv) = unbounded();
        drop(neighbor_recv); // Close the receiver
        drone.packet_send.insert(UNKNOWN_NODE, neighbor_sender);

        let flood_request = FloodRequest {
            flood_id: 123,
            initiator_id: 1,
            path_trace: vec![(1, Client), (drone.id, Drone)],
        };

        let packet = Packet {
            pack_type: PacketType::FloodRequest(flood_request.clone()),
            routing_header: SourceRoutingHeader {
                hop_index: 0,
                hops: vec![],
            },
            session_id: 42,
        };

        drone.handle_flood_request(packet);

        assert!(
            controller_recv.try_recv().is_err(),
            "No event should be sent to the SC"
        );

        assert!(
            !drone.packet_send.contains_key(&UNKNOWN_NODE),
            "Neighbor with closed channel should be removed"
        );
    }

    #[test]
    fn test_flood_request_to_all_neighbors() {
        // Tests flood request is sent to all neighbors except the initiator
        let (mut drone, _, _) = setup_drone();

        let (neighbor_1_sender, neighbor_1_recv) = unbounded();
        let (neighbor_2_sender, neighbor_2_recv) = unbounded();
        drone.packet_send.insert(1, neighbor_1_sender);
        drone.packet_send.insert(2, neighbor_2_sender);

        let flood_request = FloodRequest {
            flood_id: 123,
            initiator_id: 1,
            path_trace: vec![(1, Drone)],
        };

        let packet = Packet {
            pack_type: PacketType::FloodRequest(flood_request.clone()),
            routing_header: SourceRoutingHeader {
                hop_index: 0,
                hops: vec![],
            },
            session_id: 42,
        };

        drone.handle_flood_request(packet);

        assert!(neighbor_1_recv.try_recv().is_err());
        if let Ok(packet) = neighbor_2_recv.recv_timeout(std::time::Duration::from_secs(1)) {
            match packet.pack_type {
                PacketType::FloodRequest(request) => {
                    assert_eq!(request.flood_id, 123);
                    assert!(request.path_trace.contains(&(drone.id, Drone)));
                }
                _ => panic!("Unexpected packet: {:?}", packet.pack_type),
            }
        } else {
            panic!("Timeout: no packet received");
        }
    }

    #[test]
    fn sends_flood_response_if_has_only_sender_as_a_neighbor() {
        // Test that the drone sends a FloodResponse if it has no one to send a FloodRequest to
        let (mut drone, _, controller_recv) = setup_drone();

        let (sender, receiver) = unbounded();
        drone.packet_send.insert(1, sender);

        let packet = Packet {
            pack_type: PacketType::FloodRequest(FloodRequest {
                flood_id: 123,
                initiator_id: 1,
                path_trace: vec![(1, Client)],
            }),
            routing_header: SourceRoutingHeader {
                hop_index: 0,
                hops: vec![],
            },
            session_id: 4,
        };

        drone.handle_flood_request(packet);

        // Verify that packet FloodResponse is sent to neighbor 1
        if let Ok(packet) = receiver.recv_timeout(std::time::Duration::from_secs(1)) {
            match packet.pack_type {
                PacketType::FloodResponse(response) => {
                    assert_eq!(response.flood_id, 123);
                    assert_eq!(response.path_trace, vec![(1, Client), (drone.id, Drone)]);
                }
                _ => panic!("Unexpected packet: {:?}", packet.pack_type),
            }
        } else {
            panic!("Timeout: no packet received");
        }

        // Verify that packet FloodResponse is sent to sc
        if let Ok(event) = controller_recv.recv_timeout(std::time::Duration::from_secs(1)) {
            match event {
                DroneEvent::PacketSent(packet) => match packet.pack_type {
                    PacketType::FloodResponse(response) => {
                        assert_eq!(response.flood_id, 123);
                        assert_eq!(response.path_trace, vec![(1, Client), (drone.id, Drone)]);
                    }
                    _ => panic!("Unexpected event: {:?}", packet.pack_type),
                },
                _ => panic!("Unexpected event: {event:?}"),
            }
        } else {
            panic!("Timeout: no event received");
        }
    }

    #[test]
    fn flood_request_already_received() {
        // Setup
        let (mut drone, _, controller_recv) = setup_drone();

        // Add neighbor 1
        let (sender, receiver) = unbounded();
        drone.packet_send.insert(1, sender);

        // Add neighbor 2
        let (sender2, _receiver2) = unbounded();
        drone.packet_send.insert(2, sender2);

        // Simulate a flood that has already been received
        drone.received_floods.insert((123, 1));

        // Create the FloodRequest packet
        let packet = Packet {
            pack_type: PacketType::FloodRequest(FloodRequest {
                flood_id: 123,
                initiator_id: 1,
                path_trace: vec![(1, Client)],
            }),
            routing_header: SourceRoutingHeader {
                hop_index: 0,
                hops: vec![],
            },
            session_id: 4,
        };

        // Handle the FloodRequest
        drone.handle_flood_request(packet);

        // Verify that a FloodResponse is sent to neighbor 1
        if let Ok(packet) = receiver.recv_timeout(std::time::Duration::from_secs(1)) {
            match packet.pack_type {
                PacketType::FloodResponse(response) => {
                    assert_eq!(response.flood_id, 123);
                    assert_eq!(response.path_trace, vec![(1, Client), (drone.id, Drone)]);
                }
                _ => panic!("Unexpected packet: {:?}", packet.pack_type),
            }
        } else {
            panic!("Timeout: no packet received");
        }

        // Verify that a FloodResponse is sent to sc
        if let Ok(event) = controller_recv.recv_timeout(std::time::Duration::from_secs(1)) {
            match event {
                DroneEvent::PacketSent(packet) => match packet.pack_type {
                    PacketType::FloodResponse(response) => {
                        assert_eq!(response.flood_id, 123);
                        assert_eq!(response.path_trace, vec![(1, Client), (drone.id, Drone)]);
                    }
                    _ => panic!("Unexpected packet type: {:?}", packet.pack_type),
                },
                _ => panic!("Unexpected event: {:?}", event),
            }
        } else {
            panic!("Timeout: no packet received");
        }
    }

    #[test]
    fn flood_request_with_empty_path_trace() {
        // Setup
        let (mut drone, _, _) = setup_drone();

        // Add neighbor 1
        let (sender, receiver) = unbounded();
        drone.packet_send.insert(1, sender);

        // Create a FloodRequest with an empty path_trace
        let packet = Packet {
            pack_type: PacketType::FloodRequest(FloodRequest {
                flood_id: 123,
                initiator_id: 1,
                path_trace: vec![],
            }),
            routing_header: SourceRoutingHeader {
                hop_index: 0,
                hops: vec![],
            },
            session_id: 4,
        };

        drone.handle_flood_request(packet);

        if let Ok(packet) = receiver.recv_timeout(std::time::Duration::from_secs(1)) {
            match packet.pack_type {
                PacketType::FloodResponse(response) => {
                    assert_eq!(response.flood_id, 123);
                    assert_eq!(response.path_trace, vec![(drone.id, Drone)]); // TODO: think if we want to add the initiator
                }
                _ => panic!("Unexpected packet: {:?}", packet.pack_type),
            }
        } else {
            panic!("Timeout: no packet received");
        }
    }

    #[test]
    fn handle_flood_request_with_invalid_packet_type() {
        // Setup
        let (mut drone, _controller_send, controller_recv) = setup_drone();

        // Create a non-valid packet (e.g., MsgFragment)
        let packet = Packet {
            pack_type: PacketType::MsgFragment(Fragment {
                fragment_index: 0,
                total_n_fragments: 1,
                data: [0; FRAGMENT_DSIZE],
                length: FRAGMENT_DSIZE as u8,
            }),
            routing_header: SourceRoutingHeader {
                hop_index: 0,
                hops: vec![],
            },
            session_id: 4,
        };

        // Capture the initial state of the drone
        let initial_received_floods = drone.received_floods.clone();

        // Pass the packet to handle_flood_request
        drone.handle_flood_request(packet);

        // Validate the state remains unchanged
        assert_eq!(
            drone.received_floods, initial_received_floods,
            "Drone state should not be modified when processing an invalid packet type"
        );

        // Verify that no events were sent to the controller
        assert!(
            controller_recv.try_recv().is_err(),
            "No events should be sent to the controller for an invalid packet type"
        );

        // Validate that no packets were sent to any neighbors
        assert!(
            drone.packet_send.is_empty(),
            "No packets should be sent to neighbors for an invalid packet type"
        );
    }

    #[test]
    fn flood_request_then_response_next_hop_not_a_neighbor() {
        // test that the drone receives a FloodRequest, builds the FloodResponse,
        // and if it does not have the node it is supposed to send it to as a neighbor, sends it to the SC
        let (mut drone, _, controller_recv) = setup_drone();

        let (sender, _receiver) = unbounded();
        drone.packet_send.insert(1, sender);
        drone.received_floods.insert((123, 1));

        let packet = Packet {
            pack_type: PacketType::FloodRequest(FloodRequest {
                flood_id: 123,
                initiator_id: 1,
                path_trace: vec![(1, Client), (UNKNOWN_NODE, Drone)],
            }),
            routing_header: SourceRoutingHeader {
                hop_index: 0,
                hops: vec![],
            },
            session_id: 4,
        };
        drone.handle_flood_request(packet);

        // Verify that packet FloodResponse is sent through SC
        if let Ok(event) = controller_recv.recv_timeout(std::time::Duration::from_secs(1)) {
            match event {
                DroneEvent::ControllerShortcut(packet) => match packet.pack_type {
                    PacketType::FloodResponse(response) => {
                        assert_eq!(response.flood_id, 123);
                        assert_eq!(
                            response.path_trace,
                            vec![(1, Client), (UNKNOWN_NODE, Drone), (drone.id, Drone)]
                        );
                    }
                    _ => panic!("Unexpected event: {:?}", packet.pack_type),
                },
                _ => panic!("Unexpected event: {event:?}"),
            }
        } else {
            panic!("Timeout: no event received");
        }
    }
}
