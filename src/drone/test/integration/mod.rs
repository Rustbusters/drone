mod flooding {
    use crate::drone::test::common::{RB_DRONE_ID, UNKNOWN_NODE};
    use crate::RustBustersDrone;
    use crossbeam_channel::unbounded;
    use std::collections::HashMap;
    use wg_2024::controller::{DroneCommand, DroneEvent};
    use wg_2024::drone::Drone;
    use wg_2024::network::SourceRoutingHeader;
    use wg_2024::packet::{FloodRequest, FloodResponse, NodeType, Packet, PacketType};

    #[test]
    fn test_flood_response_back_to_sender_if_no_other_neighbor() {
        use std::thread;

        let (controller_send, controller_recv) = unbounded();
        let (cmd_send, cmd_recv) = unbounded();
        let (packet_send_to_drone, packet_recv) = unbounded();
        let mut packet_send = HashMap::new();

        let (drone1_send, drone1_recv) = unbounded();
        packet_send.insert(1, drone1_send);

        // start drone
        let drone_handle = thread::spawn(move || {
            let mut drone = RustBustersDrone::new(
                RB_DRONE_ID,
                controller_send,
                cmd_recv,
                packet_recv,
                packet_send,
                0.0,
            );

            drone.run();
        });

        let packet = Packet {
            pack_type: PacketType::FloodRequest(FloodRequest {
                flood_id: 123,
                initiator_id: 1,
                path_trace: vec![(1, NodeType::Client)],
            }),
            routing_header: SourceRoutingHeader {
                hop_index: 0,
                hops: vec![],
            },
            session_id: 0,
        };

        // send packet through _packet_send_to_drone
        packet_send_to_drone
            .send(packet)
            .expect("Failed to send packet to the drone");

        if let Ok(packet) = drone1_recv.recv_timeout(std::time::Duration::from_secs(1)) {
            match packet.pack_type {
                PacketType::FloodResponse(flood_response) => {
                    assert_eq!(flood_response.flood_id, 123);
                    assert_eq!(
                        flood_response.path_trace,
                        vec![(1, NodeType::Client), (RB_DRONE_ID, NodeType::Drone)]
                    );
                }
                _ => panic!("Expected FloodResponse packet"),
            }
        } else {
            panic!("Failed to receive packet from drone");
        }

        if let Ok(event) = controller_recv.recv_timeout(std::time::Duration::from_secs(1)) {
            match event {
                DroneEvent::PacketSent(packet) => match packet.pack_type {
                    PacketType::FloodResponse(flood_response) => {
                        assert_eq!(flood_response.flood_id, 123);
                        assert_eq!(
                            flood_response.path_trace,
                            vec![(1, NodeType::Client), (RB_DRONE_ID, NodeType::Drone)]
                        );
                    }
                    _ => panic!("Expected FloodResponse packet"),
                },
                _ => panic!("Expected PacketSent event"),
            }
        } else {
            panic!("Failed to receive command from controller");
        }

        cmd_send
            .send(DroneCommand::Crash)
            .expect("Failed to send crash command to the drone");
        drone_handle.join().expect("Failed to join drone thread");
    }

    #[test]
    fn flood_resposnse_to_sc_if_neighbor_is_absent() {
        use std::thread;

        let (controller_send, controller_recv) = unbounded();
        let (cmd_send, cmd_recv) = unbounded();
        let (packet_send_to_drone, packet_recv) = unbounded();
        let packet_send = HashMap::new();

        // start drone
        let drone_handle = thread::spawn(move || {
            let mut drone = RustBustersDrone::new(
                RB_DRONE_ID,
                controller_send,
                cmd_recv,
                packet_recv,
                packet_send,
                0.0,
            );

            drone.run();
        });

        let packet = Packet {
            pack_type: PacketType::FloodResponse(FloodResponse {
                flood_id: 123,
                path_trace: vec![
                    (1, NodeType::Client),
                    (RB_DRONE_ID, NodeType::Drone),
                    (UNKNOWN_NODE, NodeType::Drone),
                ],
            }),
            routing_header: SourceRoutingHeader {
                hop_index: 1,
                hops: vec![1, RB_DRONE_ID, UNKNOWN_NODE],
            },
            session_id: 0,
        };

        // send packet through _packet_send_to_drone
        packet_send_to_drone
            .send(packet)
            .expect("Failed to send packet to the drone");

        if let Ok(event) = controller_recv.recv_timeout(std::time::Duration::from_secs(1)) {
            match event {
                DroneEvent::ControllerShortcut(packet) => match packet.pack_type {
                    PacketType::FloodResponse(flood_response) => {
                        assert_eq!(flood_response.flood_id, 123);
                        assert_eq!(
                            flood_response.path_trace,
                            vec![
                                (1, NodeType::Client),
                                (RB_DRONE_ID, NodeType::Drone),
                                (UNKNOWN_NODE, NodeType::Drone)
                            ]
                        );
                    }
                    _ => panic!("Expected FloodResponse packet"),
                },
                _ => panic!("Expected DroneEvent event"),
            }
        }

        cmd_send
            .send(DroneCommand::Crash)
            .expect("Failed to send crash command to the drone");
        drone_handle.join().expect("Failed to join drone thread");
    }
}
