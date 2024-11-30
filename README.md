# RustBusters Drone Capabilities

The `RustBustersDrone` is a software-defined drone designed to work in a distributed network simulation.
It was developed for the Advanced Programming course 2024 at the University of Trento.
It provides robust capabilities for handling packet routing, communication, and control commands.

## Basic drone features
These features are the one standardized for every single drone implementation.

### **Packet Handling**
- **Packet Forwarding**: Routes packets through a defined path, with validation of recipient and hop indices.
- **Ack/Nack Management**: Sends acknowledgments (Ack) or negative acknowledgments (Nack) based on routing outcomes or errors.
- **Packet Drop Simulation**: Implements a configurable Packet Drop Rate (PDR) to simulate real-world communication failures.

### **Flood Management**
- **Flood Request Handling**: Processes and forwards `FloodRequest` packets to neighboring drones while preventing redundant processing of the same request.
- **Flood Response**: Sends responses back to the initiator of the flood request with the path trace.

### **Optimized Routing**
- **Path Optimization**: Optimizes routing paths by leveraging nearby nodes, improving efficiency and reducing latency.

### **Control Commands**
- **Crash**: Stops the drone's operations gracefully.
- **Add Sender**: Dynamically adds communication channels for new neighbors.
- **Set Packet Drop Rate**: Configures the PDR dynamically to test network resilience.

## RustBusters features 😎

### **Event Logging ✏️**
Our drone provides comprehensive logging with levels: `debug`, `info`, `warn`, `error`, and `trace` for detailed runtime monitoring and troubleshooting.

### **Hunt the ghost 👻**
The `hunt` command allows a `RustBustersDrone` or **hunter** to kill a non-`RustBustersDrone` or **ghost drone** via a request to the **Simulation Controller**. \
This is the basic idea of how it works:

1. The `RustBustersDrone`chooses a **hunt modality**. There are 3 main modalities:
   1. **Normal shot**: the hunter crashes a ghost neighbor.
   2. **Long shot**: the drone is given a max range within which its **proton pack** operates and it crashes a random ghost drone within that range decided by the simulation controller.
   3. **EMP blast**: the proton pack of the RustBustersDrone has a unique functionality that can be activated once during its lifetime with which every ghost neighbor will be crashed.
2. The hunter drone sends a specific command to the Simulation Controller.
3. The Simulation Controller receives the command and verifies if the target is not a`RustBustersDrone` and if the action maintains the network integrity.
   - If the network isn't partitioned after the `hunt`, then a `Crash` command is sent to the ghost drone.
   - Otherwise, the command is canceled and the `hunt` initiator is notified.

## Configurable Options
- **Node ID**: Unique identifier for the drone.
- **Packet Drop Rate (PDR)**: Probability of dropping packets (0-100%).
- **Optimized Routing**: Toggle for enabling route optimization.

This drone is part of the `RustBusters` project and integrates seamlessly into the `wg_2024` simulation framework for distributed network experiments.