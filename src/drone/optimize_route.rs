use super::RustBustersDrone;
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

        if let Some(pos) = path[1..]
            .iter()
            .rev()
            .position(|&node_id| self.packet_send.contains_key(&node_id))
        {
            let mut ret_path = vec![self.id];

            ret_path.append(path[(path.len() - pos - 1)..].to_vec().as_mut());
            ret_path
        } else {
            path.to_vec()
        }
    }
}
