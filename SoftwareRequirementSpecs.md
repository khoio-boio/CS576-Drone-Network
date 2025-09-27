## Purpose and Scope
- **Purpose:** Build a discrete-event simulator of a UAV ad hoc network with simplified PHY/MAC/NET layers, 2D mobility, an interactive GUI, and reproducible experiments
- **In-scope:** 2D mobility (Random Waypoint, Leader-Follower, CSMA/CA-like MAC with ACK/retries, simplified radio model with PRR vs distance and TX power, AODV and OSLR routing (minimal viable), buffering for disconnections, GUI with topolocy and KPI's, E1-E3 experiments.
- **Out-of-scope:** Real radio drivers, GPS/IMU fidelity, true RF propogation/PHY bit level modeling, security/auth, persistence beyond CSV/PNG.

### Stakeholders and Users
- Primary user: Student/researcher running scenarios, viewing topology, exporting results.
- Secondary: Instructor/grader verifying rubric items; teammate modifying mobility or routing.

### Definitions & Abbreviations
- PRR: Packet Reception Ratio
- PDR: Packet Delivery Ratio (delivered/sent)
- Jitter: Variation in inter-arrival delay (RFC3550 style)
- RREQ/RREP/RERR: AODV control messages
- HELLO/TC: OLSR neighbor & topology control messages

### System Context & Interfaces
- Execution: Single process, single-thread Qt GUI; headless CLIs for experiments.
- External interfaces:
- File system (read configs; write CSV/PNG).
- Optional: static HTML export (plots) for sharing.

## Functional Requirements
### Configuration
- Load YAML/JSON config defining world bounds, radio/rate/power, MAC params, routing toggles, mobility model & parameters, flows, seeds, GUI options.
- CLI flags override config (e.g., --nodes, --seed, --duration).

### Mobility (2D)
- Random Waypoint 2D: uniform waypoint selection, speed ∈ [vmin, vmax], optional pause at waypoint.
- Leader–Follower: leader follows path; followers maintain relative offsets with small noise.
- Formation Switch Event: at configured t_switch trigger formation/topology change (e.g., new offsets or insert relay).

### Radio/PHY
- PRR model: PRR(d, tx_power) smooth function (e.g., sigmoid around nominal range r0 scaled by power).
- Airtime: payload size / data rate + header.
- Energy: per-mode power (TX/RX/IDLE/SLEEP); battery Wh accounting.

### MAC
- CSMA/CA-like access: carrier sense, random backoff, DIFS/SIFS timing.
- ACKs with timeout; retry up to limit; optional RTS/CTS threshold.
- Per-node FIFO queue with capacity and drop policy (tail-drop default).

### Routing
- AODV (minimal): RREQ broadcast on no route, reverse path, first RREP wins, lifetime timers; RERR on break; rediscovery.
- OLSR (simplified): HELLO for 1-hop; TC flooding (simple MPR or approximate); periodic recompute via Dijkstra.
- Buffering: hold-buffers for packets awaiting route, with drop policy (e.g., drop-oldest or tail after cap).
