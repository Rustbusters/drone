#[cfg(test)]
mod route {
    use crate::drone::test::common::setup_drone;
    use crossbeam_channel::unbounded;
    use wg_2024::network::NodeId;

    #[test]
    fn test_optimize_route() {
        let (mut drone, _, _) = setup_drone();
        let (neighbor_2_sender, _check_recv) = unbounded();
        drone.packet_send.insert(2, neighbor_2_sender);
        let (neighbor_3_sender, _check_recv) = unbounded();
        drone.packet_send.insert(3, neighbor_3_sender);

        let path: Vec<NodeId> = vec![drone.id, 1, 2, 11];
        assert_eq!(drone.optimize_route(&path), vec![drone.id, 2, 11]);

        let path: Vec<NodeId> = vec![drone.id, 1, 4, 5, 6, 3, 11];
        assert_eq!(drone.optimize_route(&path), vec![drone.id, 3, 11]);

        // no neighbors in the path
        let path: Vec<NodeId> = vec![drone.id, 2, 4, 5, 6, 11];
        assert_eq!(drone.optimize_route(&path), vec![drone.id, 2, 4, 5, 6, 11]);
    }
}
