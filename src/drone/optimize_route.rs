use super::RustBustersDrone;
use std::collections::HashMap;
use wg_2024::network::NodeId;

impl RustBustersDrone {
    /// Optimize the route when possible if it finds a shorter path to the destination
    ///
    /// #### Arguments
    /// - `path`: The path to optimize
    ///
    /// #### Returns
    /// The optimized path
    pub(crate) fn optimize_route(&self, path: &[NodeId]) -> Vec<NodeId> {
        if path.len() < 2 {
            return path.to_vec();
        }

        // flag for skipping nodes when a neighbor is found until the current node is reached
        let mut neighbor_found = false;
        let mut ret_path = vec![];
        let mut visited = HashMap::new();

        for (index, &node_id) in path.iter().rev().enumerate() {
            // If the node is a neighbor, add it to the path and set the flag to true
            if self.packet_send.contains_key(&node_id) {
                if !neighbor_found {
                    ret_path.push(node_id);
                }
                neighbor_found = true;
            }
            // If the node is the current node, set the flag to false
            else if node_id == self.id {
                neighbor_found = false;
            }

            // If the node is not a neighbor and the flag is false, add it to the path
            if !neighbor_found {
                ret_path.push(node_id);
            }

            // If the node is already visited, remove the nodes of the found loop
            if let Some(i) = visited.get(&node_id) {
                let (correct, trash) = ret_path.split_at(*i + 1);
                // remove the nodes that are not part of the correct path
                for t in trash {
                    visited.remove(t);
                }
                ret_path = correct.to_vec();
            }

            visited.insert(node_id, index);
        }

        ret_path.reverse();
        ret_path
    }
}
