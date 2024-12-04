# RustBusters Drone Capabilities

The `RustBustersDrone` is a software-defined drone designed to work in a distributed network simulation.
It was developed for the Advanced Programming course 2024 at the University of Trento.
It provides robust capabilities for handling packet routing, communication, and control commands.

## Basic drone features

These features are the one standardized for every single drone implementation.

### **Packet Handling**

- **Packet Forwarding**: Routes packets through a defined path, with validation of recipient and hop indices.
- **Nack Management**: Sends negative acknowledgments (Nack) based on routing outcomes or errors.
- **Packet Drop Simulation**: Implements a configurable Packet Drop Rate (PDR) to simulate real-world communication
  failures.

### **Flood Management**

- **Flood Request Handling**: Processes and forwards `FloodRequest` packets to neighboring drones while preventing
  redundant processing of the same request.
- **Flood Response**: Sends responses back to the initiator of the flood request with the path trace.

### **Optimized Routing**

- **Path Optimization**: Optimizes routing paths by leveraging nearby nodes, improving efficiency and reducing latency.

### **Control Commands**

- **Crash**: Stops the drone's operations gracefully.
- **Add Sender**: Dynamically adds communication channels for new neighbors.
- **Set Packet Drop Rate**: Configures the PDR dynamically to test network resilience.

### **Event Logging**

Our drone provides comprehensive logging with levels: `debug`, `info`, `warn`, `error`, and `trace` for detailed runtime
monitoring and troubleshooting.

## RustBusters features 😎

### **Hunt the ghosts 👻**

The `hunt` command allows a `RustBustersDrone` or **hunter** to kill a non-`RustBustersDrone` or **ghost drone** via a
request to the **Simulation Controller**. \
This is the basic idea of how it works:

1. Whenever a `RustBustersDrone` receives a `Nack` packet
2. The drone sends a `Hunt Command` to the simulation controller to kill the drone.
3. The Simulation Controller receives the command and makes the following controls:
    - If the network isn't partitioned after the `hunt`, then a `Crash` command is sent to the ghost drone.
    - Otherwise, the command is canceled and the `hunt` initiator is notified.

This feature of the drone uses the same `Packet` structure as specified in the protocol standard.\
The only thing that changes is the encoding. The `HuntPacket` looks like this:

```rust
Packet {
    pack_type: PacketType::MsgFragment(
        Fragment {
            fragment_index: 0,
            total_n_fragments: 0,
            length: 0,
            data: data_for_specific_mode // changes based on the mode you choose (NormalShot, LongShot or EMPBlast) 
        }
    ),
    routing_header: SourceRoutingHeader { hop_index: 0, hops: vec![] },
    session_id: 0,
}
```

It is then put inside a `PacketSent` `NodeEvent`.

### **Play some music 🎶**

Our magnificent drone allows to reproduce sounds based on the packets received by the drone:

- **Nack**: plays a "quack" sound 🦆
- **Dropped**: plays a "Windows Error" sound 🪟
- **Start**: plays a "YAHOO" Mario sound 🍄
- **Crash**: plays a "Windows Shut Down" 💥
- **Hunt Mode**: TBD 👻

### **Optimized path**

For the `FloodResponse` packets the drone is able to analyze the path and to optimize it by removing unnecessary hops.


## Configurable Options

- **Node ID**: Unique identifier for the drone.
- **Packet Drop Rate (PDR)**: Probability of dropping packets (0-100%).
- **Optimized Routing**: Toggle for enabling route optimization.
- **Shot Range**: Adjusts the max range for the drone to shoot.

This drone is part of the `RustBusters` project and integrates seamlessly into the `wg_2024` simulation framework for
distributed network experiments.