# CS576-Drone-Network
Team members: Khoi Tran, Aiden Suarez, Aidan Baker, Niki Liao
### Physical (PHY) - Aiden Suarez
- Realistic physical layer implementation is not required.
- Tech profiles: Wi-Fi 802.11n/ac and Wi-Fi Direct (or 802.11s-like mesh), or custom radio:
- Channel: Simply simulate data loss with a preset probability. 
- Power: Define energy levels and simulate energy consumption at different TX power levels; model TX/RX/idle/sleep energy.
### Link / MAC - Aidan Baker
- Access: Imitate CSMA/CA, ACKs, retry limit; optional RTS/CTS.
- Queues: per-node FIFO + capacity; beacons for neighbor table & expiry.
### Network - Aiden Suarez, Niki Liao
- Study routing options, e.g., AODV/DSR (reactive) or OLSR (proactive).
- Support recovery from disconnections and rediscovery (reactive) / table repair (proactive);
- Implement buffering for disconnection tolerance (consider drop policy).
### Mobility - Niki Liao
- 2D. Implement Random Waypoint (3D) and Leader–Follower/Formation.
- Formation/topology event: at mid-run (e.g., t=300 s) switch formation or insert a relay/obstacle.
- Logs: route churn, time-to-steady-state after the switch.
### GUI - will be doing 2D - Khoi Tran
- Topology view (2D or 3D): UAV icons, link color by quality (optional), show per-UAV energy, route overlay for a selected flow.
- Controls: start/pause/step/reset; seed; N× speed; button to trigger Formation Change.
- Panels: live PDR/latency/jitter, queue sizes, energy/time series; export PNG/CSV.
### Experiments - Aidan Baker
- E1 Mobility vs Latency: formation vs random waypoint at different node counts, e.g., N= {5,25,100}.
- E2 Energy–Throughput tradeoff: TX power levels vs lifetime/PDR.
- E3 Formation transition: KPIs before/during/after; route churn; time-to-restore.
