#[cfg(test)]
mod commands {
    use crate::drone::test::common::{setup_drone, UNKNOWN_NODE};
    use crossbeam_channel::unbounded;
    use wg_2024::controller::DroneCommand::{AddSender, Crash, RemoveSender, SetPacketDropRate};
    use wg_2024::network::SourceRoutingHeader;
    use wg_2024::packet::{Ack, Packet, PacketType};

    #[test]
    fn test_crash_command() {
        let (mut drone, _, _) = setup_drone();

        drone.running = true;
        drone.handle_command(Crash);
        assert!(!drone.running);
    }

    #[test]
    fn test_add_sender_command() {
        let (mut drone, _, _) = setup_drone();

        let (unknown_node_sender, unknown_node_receiver) = unbounded();

        drone.handle_command(AddSender(UNKNOWN_NODE, unknown_node_sender));
        assert_eq!(drone.packet_send.len(), 1);
        assert!(drone.packet_send.contains_key(&UNKNOWN_NODE));

        let sample_packet = Packet {
            pack_type: PacketType::Ack(Ack { fragment_index: 0 }),
            session_id: 0,
            routing_header: SourceRoutingHeader {
                hop_index: 1,
                hops: vec![drone.id, UNKNOWN_NODE],
            },
        };

        drone
            .packet_send
            .get(&UNKNOWN_NODE)
            .unwrap()
            .send(sample_packet)
            .unwrap();

        if let Ok(packet) = unknown_node_receiver.recv_timeout(std::time::Duration::from_secs(1)) {
            match packet.pack_type {
                PacketType::Ack(ack) => {
                    assert_eq!(ack.fragment_index, 0);
                    assert_eq!(packet.routing_header.hops, vec![drone.id, UNKNOWN_NODE]);
                }
                _ => panic!("Pacchetto non atteso: {:?}", packet.pack_type),
            }
        } else {
            panic!("Timeout: nessun pacchetto ricevuto");
        }
    }

    #[test]
    fn test_remove_sender_command() {
        let (mut drone, _, _) = setup_drone();

        let (unknown_node_sender, _) = unbounded();
        drone.packet_send.insert(UNKNOWN_NODE, unknown_node_sender);

        drone.handle_command(RemoveSender(UNKNOWN_NODE));
        assert_eq!(drone.packet_send.len(), 0);
    }

    #[test]
    fn test_set_packet_drop_rate_command() {
        let (mut drone, _, _) = setup_drone();

        drone.handle_command(SetPacketDropRate(0.5));
        assert_eq!(drone.pdr, 50);

        drone.handle_command(SetPacketDropRate(0.75));
        assert_eq!(drone.pdr, 75);

        drone.handle_command(SetPacketDropRate(0.0));
        assert_eq!(drone.pdr, 0);

        drone.handle_command(SetPacketDropRate(1.0));
        assert_eq!(drone.pdr, 100);
    }
}
