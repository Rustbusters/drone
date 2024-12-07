#[cfg(test)]
mod hunt {
    use crate::drone::test::common::setup_drone;
    use crate::hunt::PACKET_CONST;
    use wg_2024::controller::DroneEvent;
    use wg_2024::packet::PacketType;

    #[test]
    fn test_hunt_ghost_on_hunt_mode_active() {
        let (mut drone, _, controller_recv) = setup_drone();
        drone.set_hunt_mode(true);

        let result = drone.hunt_ghost(1);

        assert_eq!(result, Ok(()));
        if let Ok(packet) = controller_recv.recv_timeout(std::time::Duration::from_secs(1)) {
            match packet {
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
                _ => panic!("Unexpected event: {packet:?}"),
            }
        } else {
            panic!("Timeout: no packet received");
        }
    }

    #[test]
    fn test_hunt_ghost_on_hunt_mode_inactive() {
        let (drone, _, controller_recv) = setup_drone();

        let result = drone.hunt_ghost(1);

        assert_eq!(result, Err("Drone is not in hunt mode".to_string()));

        if let Ok(packet) = controller_recv.recv_timeout(std::time::Duration::from_secs(1)) {
            panic!("Unexpected packet: {packet:?}",);
        }
    }

    #[test]
    fn test_error_return_on_controller_not_reachable() {
        let (mut drone, _, _) = setup_drone();
        drone.set_hunt_mode(true);

        let result = drone.hunt_ghost(1);

        assert_eq!(result, Err("Error in sending Hunt Packet".to_string()));
    }
}
