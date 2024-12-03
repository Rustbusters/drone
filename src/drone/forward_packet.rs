use super::RustBustersDrone;
use log::{error, info, trace, warn};
use rand::Rng;
use wg_2024::controller::DroneEvent;
use wg_2024::packet::{Nack, NackType, Packet, PacketType};

impl RustBustersDrone {
    pub fn forward_packet(&mut self, mut packet: Packet, allow_optimized: bool) {
        trace!("Drone {} forwarding packet: {:?}", self.id, packet);
        let hop_index = packet.routing_header.hop_index;

        // Step 1: Check if hops[hop_index] matches self.id
        if packet.routing_header.hops[hop_index] != self.id {
            warn!(
                "Drone {}: Unexpected recipient. Expected {}, got {}",
                self.id, packet.routing_header.hops[hop_index], self.id
            );
            self.send_nack(
                packet,
                Nack {
                    fragment_index: 0, // TODO: Set fragment index
                    nack_type: NackType::UnexpectedRecipient(self.id),
                },
                allow_optimized,
            );
            return;
        }

        // Step 2: Increment hop_index
        packet.routing_header.hop_index += 1;

        // Step 3: Check if drone is the final destination
        let next_hop_index = packet.routing_header.hop_index;
        if next_hop_index >= packet.routing_header.hops.len() {
            warn!("Drone {}: Destination is drone, sending Nack.", self.id);
            self.send_nack(
                packet,
                Nack {
                    fragment_index: 0, // TODO: Set fragment index
                    nack_type: NackType::DestinationIsDrone,
                },
                allow_optimized,
            );
            return;
        }

        // Step 4: Identify next hop and check if it's a neighbor
        let next_hop = packet.routing_header.hops[next_hop_index];
        if !self.packet_send.contains_key(&next_hop) {
            warn!(
                "Drone {}: Next hop {} is not a neighbor.",
                self.id, next_hop
            );
            trace!("Drone {}: Packet: {:?}", self.id, packet);
            self.send_nack(
                packet,
                Nack {
                    fragment_index: 0, // TODO: Set fragment index
                    nack_type: NackType::ErrorInRouting(next_hop),
                },
                allow_optimized,
            );
            return;
        }

        // Step 5: Proceed based on packet type
        match &packet.pack_type {
            PacketType::MsgFragment(fragment) => {
                // Check for packet drop
                let should_drop = {
                    let mut rng = rand::thread_rng();
                    rng.gen_range(0..100) < self.pdr
                };

                if should_drop {
                    info!(
                        "Drone {}: Dropping packet due to PDR. Fragment index: {}",
                        self.id, fragment.fragment_index
                    );
                    self.send_nack(
                        packet.clone(),
                        Nack {
                            fragment_index: fragment.fragment_index,
                            nack_type: NackType::Dropped,
                        },
                        allow_optimized,
                    );
                    // Send PacketDropped event to the controller
                    if let Err(e) = self
                        .controller_send
                        .send(DroneEvent::PacketDropped(packet.clone()))
                    {
                        error!(
                            "Drone {}: Error sending PacketDropped event: {}",
                            self.id, e
                        );
                    }
                    return;
                }

                // Forward the packet to next_hop
                if let Some(next_sender) = self.packet_send.get(&next_hop).cloned() {
                    if let Err(e) = next_sender.send(packet.clone()) {
                        error!(
                            "Drone {}: Error sending packet to {}: {}",
                            self.id, next_hop, e
                        );
                        self.packet_send.remove(&next_hop);
                        warn!(
                            "Drone {}: Neighbor {} has been removed from packet_send due to channel closure",
                            self.id, next_hop
                        );
                        // Optionally, send a Nack back to the sender
                        self.send_nack(
                            packet.clone(),
                            Nack {
                                fragment_index: fragment.fragment_index,
                                nack_type: NackType::ErrorInRouting(next_hop),
                            },
                            allow_optimized,
                        );
                    } else {
                        info!("Drone {}: Packet forwarded to {}", self.id, next_hop);
                        // Send PacketSent event to the controller
                        if let Err(e) = self
                            .controller_send
                            .send(DroneEvent::PacketSent(packet.clone()))
                        {
                            error!("Drone {}: Error sending PacketSent event: {}", self.id, e);
                        }
                    }
                } else {
                    warn!(
                        "Drone {}: Neighbor {} not found in packet_send.",
                        self.id, next_hop
                    );
                    // Neighbor not found in packet_send, send Nack
                    self.send_nack(
                        packet.clone(),
                        Nack {
                            fragment_index: fragment.fragment_index,
                            nack_type: NackType::ErrorInRouting(next_hop),
                        },
                        allow_optimized,
                    );
                }
            }
            PacketType::Ack(_) | PacketType::Nack(_) | PacketType::FloodResponse(_) => {
                // Forward these packets without dropping
                if let Some(next_sender) = self.packet_send.get(&next_hop).cloned() {
                    if let Err(e) = next_sender.send(packet.clone()) {
                        error!(
                            "Drone {}: Error sending packet to {}: {}",
                            self.id, next_hop, e
                        );
                        self.packet_send.remove(&next_hop);
                        warn!(
                            "Drone {}: Neighbor {} has been removed from packet_send due to channel closure",
                            self.id, next_hop
                        );
                    } else {
                        info!("Drone {}: Packet forwarded to {}", self.id, next_hop);
                    }
                } else {
                    warn!(
                        "Drone {}: Cannot forward packet, next hop {} is not a neighbor",
                        self.id, next_hop
                    );
                }
            }
            PacketType::FloodRequest(_) => {
                error!(
                    "Drone {}: Received FloodRequest in forward_packet, which should be handled in handle_flood.",
                    self.id
                );
            }
        }
    }
}
