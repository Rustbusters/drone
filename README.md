# RustBusters Drone Capabilities

The `RustBustersDrone` is a software-defined drone designed to work in a distributed network simulation.
It was developed for the Advanced Programming course 2024 at the University of Trento.
It provides robust capabilities for handling packet routing, communication, and control commands.

## Features
The features we provide are divided in basics and advanced.

### Basic features
These features are the one standardized for every single drone implementation.

#### **Packet Handling**
- **Packet Forwarding**: Routes packets through a defined path, with validation of recipient and hop indices.
- **Ack/Nack Management**: Sends acknowledgments (Ack) or negative acknowledgments (Nack) based on routing outcomes or errors.
- **Packet Drop Simulation**: Implements a configurable Packet Drop Rate (PDR) to simulate real-world communication failures.

#### **Flood Management**
- **Flood Request Handling**: Processes and forwards `FloodRequest` packets to neighboring drones while preventing redundant processing of the same request.
- **Flood Response**: Sends responses back to the initiator of the flood request with the path trace.

#### **Optimized Routing**
- **Path Optimization**: Optimizes routing paths by leveraging nearby nodes, improving efficiency and reducing latency.

#### **Control Commands**
- **Crash**: Stops the drone's operations gracefully.
- **Add Sender**: Dynamically adds communication channels for new neighbors.
- **Set Packet Drop Rate**: Configures the PDR dynamically to test network resilience.

### Our cool features

#### **Event Logging ✏️**
Our drone provides comprehensive logging with levels: `debug`, `info`, `warn`, `error`, and `trace` for detailed runtime monitoring and troubleshooting.

#### **Kill drone 💥**
Sends a command to simulate the shutdown of a target drone.
This is how it works:
1. The RustBusters drone chooses a drone to shut down.
2. Sends a specific command to the Simulation Controller.
3. The Simulation Controller receives the command and verifies the network integrity.
   - If the network isn't partitioned after the `kill`, then a `Crash` command is sent to the `target_drone`.
   - Otherwise, the command is canceled and the `kill` initiator is notified.

## Configurable Options
- **Node ID**: Unique identifier for the drone.
- **Packet Drop Rate (PDR)**: Probability of dropping packets (0-100%).
- **Optimized Routing**: Toggle for enabling route optimization.

This drone is part of the `RustBusters` project and integrates seamlessly into the `wg_2024` simulation framework for distributed network experiments.


