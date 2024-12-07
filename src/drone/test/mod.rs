#[cfg(test)]
mod tests {
    use crate::RustBustersDrone;
    use crossbeam_channel::{unbounded, Receiver};
    use std::collections::{HashMap, HashSet};
    use wg_2024::controller::{DroneCommand, DroneEvent};
    use wg_2024::network::{NodeId, SourceRoutingHeader};
    use wg_2024::packet::NodeType::{Client, Drone};
    use wg_2024::packet::{
        Ack, FloodRequest, Fragment, Nack, NackType, Packet, PacketType, FRAGMENT_DSIZE,
    };

    const RB_DRONE_ID: NodeId = 10;

    fn setup_drone() -> RustBustersDrone {
        let id = RB_DRONE_ID; // ID del drone
        let (controller_send, _controller_recv) = unbounded();
        let (_cmd_send, cmd_recv) = unbounded();
        let (_packet_send_to_2, packet_recv) = unbounded();
        let packet_send = HashMap::new();

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
        let mut drone = setup_drone();
        let (neighbor_2_sender, check_recv) = unbounded();
        drone.packet_send.insert(2, neighbor_2_sender); // Nodo 2 come "neighbor"

        let hops = vec![RB_DRONE_ID, 2, 1, 11]; // Nodo 1 (attuale), Nodo 2 (destinazione)
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
    fn test_path_optimization() {
        // drone with neighbors 2 and 3
        let mut drone = setup_drone();
        let (neighbor_2_sender, _check_recv) = unbounded();
        drone.packet_send.insert(2, neighbor_2_sender); // Nodo 2 come "neighbor"
        let (neighbor_3_sender, _check_recv) = unbounded();
        drone.packet_send.insert(3, neighbor_3_sender); // Nodo 3 come "neighbor"

        let path: Vec<NodeId> = vec![drone.id, 1, 2, 11];
        assert_eq!(drone.optimize_route(&path), vec![drone.id, 2, 11]);

        let path: Vec<NodeId> = vec![drone.id, 1, 4, 5, 6, 3, 11];
        assert_eq!(drone.optimize_route(&path), vec![drone.id, 3, 11]);

        // no neighbors in the path
        let path: Vec<NodeId> = vec![drone.id, 2, 4, 5, 6, 11];
        assert_eq!(drone.optimize_route(&path), vec![drone.id, 2, 4, 5, 6, 11]);
    }

    #[test]
    fn test_hunt_mode() {
        let mut drone = setup_drone();
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

    #[test]
    fn test_forward_other_packets() {
        fn check_ack_recv(recv: &Receiver<Packet>) {
            if let Ok(packet) = recv.recv_timeout(std::time::Duration::from_secs(1)) {
                match packet.pack_type {
                    PacketType::Ack(ack) => {
                        assert_eq!(ack.fragment_index, 0);
                    }
                    _ => panic!("Pacchetto non atteso: {:?}", packet.pack_type),
                }
            } else {
                panic!("Timeout: nessun pacchetto ricevuto");
            }
        }

        let mut drone = setup_drone();
        // Nodo 200 come "neighbor"
        let (neighbor_200_sender, check_200_recv) = unbounded();
        drone.packet_send.insert(200, neighbor_200_sender);
        // Nodo 3 come "neighbor"
        let (neighbor_3_sender, check_3_recv) = unbounded();
        drone.packet_send.insert(3, neighbor_3_sender);

        let mut packet = Packet {
            pack_type: PacketType::Ack(Ack { fragment_index: 0 }),
            routing_header: SourceRoutingHeader {
                hop_index: 3,
                hops: vec![100, 99, drone.id, 3, 5, 200, 1, 11],
            },
            session_id: 123,
        };

        // Test non-optimized routing
        drone.optimized_routing = false;
        drone.forward_other_packet(&mut packet);

        check_ack_recv(&check_3_recv);
        assert_eq!(
            packet.routing_header.hops,
            vec![100, 99, drone.id, 3, 5, 200, 1, 11]
        );

        // Test optimized routing
        drone.optimized_routing = true;
        drone.forward_other_packet(&mut packet);

        check_ack_recv(&check_200_recv);
        assert_eq!(
            packet.routing_header.hops,
            vec![100, 99, drone.id, 200, 1, 11]
        );
    }

    #[test]
    fn test_handle_command() {
        let mut drone = setup_drone();

        drone.running = true;
        drone.handle_command(DroneCommand::Crash);
        assert!(!drone.running);

        let (neighbor_2_sender, _check_recv) = unbounded();
        drone.handle_command(DroneCommand::AddSender(2, neighbor_2_sender));
        assert_eq!(drone.packet_send.len(), 1);
        assert!(drone.packet_send.contains_key(&2));

        drone.handle_command(DroneCommand::RemoveSender(2));
        assert_eq!(drone.packet_send.len(), 0);

        drone.pdr = 0;
        drone.handle_command(DroneCommand::SetPacketDropRate(0.5));
        assert_eq!(drone.pdr, 50);
    }

    #[test]
    fn test_handle_flood_request() {
        let mut drone = setup_drone();
        let (neighbor_2_sender, _check_recv) = unbounded();
        drone.packet_send.insert(2, neighbor_2_sender); // Nodo 2 come "neighbor"
        let (neighbor_3_sender, check_3_recv) = unbounded();
        drone.packet_send.insert(3, neighbor_3_sender); // Nodo 3 come "neighbor"

        let mut packet = Packet {
            pack_type: PacketType::FloodRequest(FloodRequest {
                flood_id: 123,
                initiator_id: 1,
                path_trace: vec![(1, Client), (2, Drone)],
            }),
            routing_header: SourceRoutingHeader {
                hop_index: 0,
                hops: vec![],
            },
            session_id: 123,
        };

        drone.handle_flood_request(packet.clone());

        if let Ok(packet) = check_3_recv.recv_timeout(std::time::Duration::from_secs(1)) {
            match packet.pack_type {
                PacketType::FloodRequest(request) => {
                    assert_eq!(request.flood_id, 123);
                    assert_eq!(
                        request.path_trace,
                        vec![(1, Client), (2, Drone), (drone.id, Drone)]
                    );
                }
                _ => panic!("Pacchetto non atteso: {:?}", packet.pack_type),
            }
        } else {
            panic!("Timeout: nessun pacchetto ricevuto");
        }

        // After receiving a flood request with the same flood_id
        packet.pack_type = PacketType::FloodRequest(FloodRequest {
            flood_id: 123,
            initiator_id: 1,
            path_trace: vec![(1, Client), (3, Drone)],
        });
        drone.handle_flood_request(packet);

        if let Ok(packet) = check_3_recv.recv_timeout(std::time::Duration::from_secs(1)) {
            match packet.pack_type {
                PacketType::FloodResponse(response) => {
                    assert_eq!(response.flood_id, 123);
                    assert_eq!(
                        response.path_trace,
                        vec![(1, Client), (3, Drone), (drone.id, Drone)]
                    );
                }
                _ => panic!("Pacchetto non atteso: {:?}", packet.pack_type),
            }
        } else {
            panic!("Timeout: nessun pacchetto ricevuto");
        }
    }
}



#[cfg(test)]
mod flooding_tests {
    use crate::RustBustersDrone;
    use crossbeam_channel::{unbounded, Receiver, Sender};
    use std::collections::{HashMap, HashSet};
    use wg_2024::controller::DroneEvent;
    use wg_2024::network::NodeId;
    use wg_2024::network::SourceRoutingHeader;
    use wg_2024::packet::NodeType::{Client, Drone, Server};
    use wg_2024::packet::{FloodRequest, Packet, PacketType};

    const RB_DRONE_ID: NodeId = 10;
    const UNKNOWN_NODE: NodeId = 99;

    fn setup_drone() -> (RustBustersDrone, Sender<DroneEvent>, Receiver<DroneEvent>) {
        let (controller_send, controller_recv) = unbounded();
        let (_cmd_send, cmd_recv) = unbounded();
        let (_packet_send_to_2, packet_recv) = unbounded();
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
            running: false,
            hunt_mode: false,
            sound_sys: None,
        };

        (drone, controller_send, controller_recv)
    }

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

        drone.spread_flood_request(&flood_request, 42, UNKNOWN_NODE);

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

        drone.spread_flood_request(&flood_request, 42, 1);

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
    fn test_single_neighbor_sends_flood_response() {
        // Tests a single neighbor sends a flood response
        let (mut drone, _, _) = setup_drone();

        let (neighbor_sender, neighbor_recv) = unbounded();
        drone.packet_send.insert(2, neighbor_sender);

        let flood_request = FloodRequest {
            flood_id: 123,
            initiator_id: 1,
            path_trace: vec![(1, Drone), (2, Drone)],
        };

        drone.handle_flood_request(Packet {
            pack_type: PacketType::FloodRequest(flood_request),
            routing_header: SourceRoutingHeader {
                hop_index: 0,
                hops: vec![2],
            },
            session_id: 123,
        });

        if let Ok(packet) = neighbor_recv.recv_timeout(std::time::Duration::from_secs(1)) {
            match packet.pack_type {
                PacketType::FloodResponse(response) => {
                    assert_eq!(response.flood_id, 123);
                }
                _ => panic!("Unexpected packet: {:?}", packet.pack_type),
            }
        } else {
            panic!("Timeout: no packet received");
        }
    }
    
    
}
