# RustBusters Drone

The `RustBustersDrone` is a software-defined drone designed to work in a distributed network simulation.
It was developed for the Advanced Programming course 2024 at the University of Trento.
It provides robust capabilities for packet handling, flood management and control commands.

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

### **Control Commands**

- **Crash**: Stops the drone's operations.
- **Add Sender**: Dynamically adds communication channels for new neighbors.
- **Set Packet Drop Rate**: Configures the PDR dynamically to test network resilience.

## RustBusters features 😎

### **Optimized path 🛣️**

The drone optimizes routes by removing unnecessary hops for `Nack`s and `FloodResponse`s.

#### Example Scenario

- Route: `[1, 2, 3, 4, 5, 6]`
- Current drone: `2` 
- Received a `Nack`/`FloodResponse` from drone `1`. 
- Neighbors of `2`: `1`, `3`, and `5`. 

#### Algorithm Steps

1. Start checking the route from the end.
2. If a neighbor is found between the current drone (`2`) and the analyzed node (`6`,`5`,...), skip intermediate nodes and connect directly to the neighbor.
3. If no neighbors are found, keep the original route.

This ensures efficient path analysis and reduced hops where possible.

So in our example (we're on drone `2`):
- Analyze `6`, not neighbor.
- Analyze `5`, neighbor and thus skip the edges `2-3`, `3-4`, `4-5`
- Attach `2` to `[5,6]`
- The whole route becomes `[1,2,5,6]`

![image](./assets/optimzed-route.jpg)

Code example for setting the optimized routing:

```rust
let mut drone = RustBustersDrone::new(...);
drone.set_optimized_routing(true); // enables optimized routing 
drone.set_optimized_routing(false); // disables optimized routing
```

### **Ghost hunter 👻**

The `hunt` command allows a `RustBustersDrone` or **hunter drone** to eliminate a **ghost drone** 
from the network via a request to the **Simulation Controller**.\
This is how it works:

1. The `RustBustersDrone` receives a `Nack::Dropped` packet from another drone.
2. The `RustBustersDrone` sends a **hunt** `Packet` to the simulation controller to eliminate the drone it received the `Nack` from.
3. The Simulation Controller receives the packet, processes it and makes the following controls:
    - If the network isn't partitioned after the drone removal and if the target is not a Rustbusters drone, then a `Crash` command is sent to the drone.
    - Otherwise, the operation is aborted.


![image info](./assets/ghost-hunter.jpg)

Code example for setting the hunt mode:

```rust
let mut drone = RustBustersDrone::new(...);
drone.set_hunt_mode(true); // enables hunt mode
drone.set_hunt_mode(false); // disables hunt mode
```

#### Hunter Drone to Simulation Controller

This feature of the drone uses the same `Packet` structure as the one specified in the protocol standard.\
The only thing that changes is the encoding. The **hunt** `Packet` looks like this:

```rust
pub const PACKET_CONST: u8 = 169;

Packet {
    pack_type: PacketType::MsgFragment(
        Fragment {
            fragment_index: 0,
            total_n_fragments: 0,
            length: PACKET_CONST,
            data: [src_node_id, target_node_id, ...] // set data with the specified source and target drone ids
        }
    ),
    routing_header: SourceRoutingHeader { hop_index: 0, hops: vec![] },
    session_id: 0,
}
```

It is then put inside a `PacketSent` `DroneEvent`:

```rust
let hunt_node_event = DroneEvent::PacketSent(hunt_packet);
```

And sent to the Simulation Controller:

```rust
self.controller_send.send(hunt_node_event)
```

#### Simulation Controller to Ghost Drone

> The following instructions are for the member that implements the Simulation Controller.

The Simulation Controller is asked to:

1. Implement a handler for the hunt packet.
2. Verify the network integrity on ghost drone removal.
3. Send a `Crash` command to the ghost drone.

A code example can look like this:

```rust
use drone::hunt::PACKET_CONST;

fn handle_drone_commands(packet: Packet) {
    match &packet.pack_type {
        PacketType::MsgFragment(fragment) => {
            // Hunt Packet
            if fragment.fragment_index == 0 && fragment.total_n_fragments == 0 && fragment.length == PACKET_CONST {
                let target_node_id = fragment.data[1];
                handle_hunt(simulation_controller, target_node_id);
            }
        }
        _ => {}
    }
}

fn handle_hunt(simulation_controller: &mut RustBustersSimulationController, target_drone_id: NodeId) -> Result<(), String> {
    // Try to remove node
    simulation_controller.graph.remove_node(target_drone_id);

    // Verify graph integrity
    if is_network_connected(simulation_controller) {
        // Send crash command to the target_drone_id
        println!("Network integrity is guaranteed. Proceeding with crash command.");

        if let Ok(()) = simulation_controller.send_crash_command(target_drone_id) {
            Ok(())
        } else {
            Err("Cannot send crash command to drone".to_string())
        }
    } else {
        // Abort operation, reestablish previous topology by readding the removed drone and send error
        // RESET the graph to the original state
        Err("Cannot guarantee network integrity. Aborting hunt.".to_string())
    }
}

fn is_network_connected(simulation_controller: &RustBustersSimulationController) -> bool {
    // Verify graph integrity
    unimplemented!() // Implement your own network integrity check
}
```



### **Play some music 🎶**

Our magnificent drone allows to reproduce sounds based on the packets received by the drone:

- **Start**: When the drone `start`s it reproduces the **“YAHOO”** Mario sound 🍄.\
    <img src="./assets/1.jpg" width="508" height="400">
- **Nack**: Whenever the Rusbusters drone produces a `Nack` that is **not a `Dropped`** it plays the **"Windows Error"** sound 🪟.\
  <img src="./assets/2.jpg" width="508" height="400">
- **Hunt Mode**: On `Nack` receipt the drone activates the ghost hunter mode and reproduces the **“PIUPIUPIU”** sound 🔫 (like Colt from Brawl Stars).\
  <img src="./assets/3.jpg" width="508" height="400">
- **Crash**: Whenever the Rusbusters drone receives a `Crash` command from the mighty Simulation Controller the drone plays the **"Windows Shut Down"** sound 🪟.\
  <img src="./assets/4.jpg" width="508" height="400">
- **Dropped**: On packet `Nack` `Dropped` the drone plays the original **Marco Patrignani “QUACK”** sound 🦆 and proceeds with the drop of the packet.\
  <img src="./assets/5.jpg" width="508" height="400">

Code example for activating sounds:

In `Cargo.toml`:
```toml
[dependencies]
rustbusters-drone = { git = "...", features = ["sounds"] }
```

In your code:
```rust
let mut drone = RustBustersDrone::new(...);
drone.enable_sound(); // enables sounds
```

### **Telegram Bot 🤖**

The Rustbusters team provides a full customer support via a Telegram Bot.
It's super easy to use and is accessible on the following username or from the QR code below:
```rust
@rustbusters_bot
```

QR code:

![image info](./assets/tg-qr.png)

Don't hesitate if you have issues, our group is here to help.

### **Strong Unit/Integration Tests 🧪**

Our drone also provides a whole bunch of strong unit and integration tests that cover almost 80% of the code.\
However, if we consider only the features relevant to **packet handling**, **flood management** and **control commands** the **coverage** goes up to **90%**.

<img src="./assets/test-coverage.png" width="952">

### **Event Logging ✏️**

Our drone provides comprehensive logging with levels: `debug`, `info`, `warn`, `error`, and `trace` for detailed runtime
monitoring and troubleshooting.

## Configurable Options

- **Optimized Routing**: Toggle for enabling route optimization.
- **Hunt Mode**: Toggle for enabling hunt mode.
- **Sounds**: Toggle for enabling sounds.

This drone is part of the `RustBusters` project and integrates seamlessly into the `wg_2024` simulation framework for
distributed network experiments.