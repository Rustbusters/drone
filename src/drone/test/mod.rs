#[cfg(test)]
mod tests {
    use crate::RustBustersDrone;
    use crossbeam_channel::unbounded;
    use std::collections::{HashMap, HashSet};
    use wg_2024::network::SourceRoutingHeader;
    use wg_2024::packet::{Fragment, Nack, NackType, Packet, PacketType, FRAGMENT_DSIZE};

    fn setup_drone_with_channels() -> RustBustersDrone {
        let id = 1; // ID del drone
        let (controller_send, controller_recv) = unbounded();
        let (cmd_send, cmd_recv) = unbounded();
        let (packet_send_1, packet_recv) = unbounded();
        let mut packet_send = HashMap::new();
        packet_send.insert(2, packet_send_1); // Nodo 2 come "neighbor"

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
            shot_range: 0,
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

        drone.send_nack(packet, nack.clone(), false);

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
}
