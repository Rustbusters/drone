use super::RustBustersDrone;
use log::debug;
use wg_2024::network::NodeId;

impl RustBustersDrone {
    pub fn optimize_route(&self, path: &[NodeId]) -> Vec<NodeId> {
        // FIXME: from [1, 5] to [5, 5]
        debug!("Drone {}: Optimizing route {:?}", self.id, path);

        if let Some((index, &last_nearby_node)) = path
            .iter()
            .enumerate()
            .rev()
            .find(|&(_, &node_id)| self.packet_send.contains_key(&node_id))
        {
            let mut optimized_path = path[index..].to_vec();

            debug!("Drone {}: Optimized route {:?}", self.id, optimized_path);
            return optimized_path;
        }

        debug!(
            "Drone {}: No optimization possible, returning original path",
            self.id
        );
        path.to_vec()
    }
}
