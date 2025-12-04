"""
Integrated Simulator for Drone Network
Integrates PHY, MAC, Mobility, and Metrics layers
"""

# ============================================================================
# ORIGINAL CODE (preserved for reference - do not delete)
# ============================================================================
# Original simulator.py code from other team member:
#
# from .phy import Drone
# from .mobility import Mobility2D, RandomWaypoint2D, Formation2D, step_formation2d, v_offsets, line_offsets, grid_offsets
#
# def run_sim_2d():
#     # Build a small swarm
#     drones = [
#         Drone(1,  0,  0, "802.11ac"),
#         Drone(2, 10,  0, "802.11ac"),
#         Drone(3, 20,  0, "802.11ac"),
#         Drone(4, 30,  0, "802.11ac"),
#         Drone(5, 40,  0, "802.11ac"),
#     ]
#
#     ctrls = {d.id: Mobility2D(d, max_speed=6.0) for d in drones}
#
#     # Random Waypoint until t=300s
#     rwp = RandomWaypoint2D(area=(200, 200), pause_range=(0.0, 0.5), seed=42)
#
#     # Then switch to V-formation with drone 1 as leader
#     leader_id = 1
#     followers = [d.id for d in drones if d.id != leader_id]
#     fm = Formation2D(leader_id, v_offsets(followers, spacing=10.0, angle_deg=28.0))
#
#     dt = 0.2
#     T_end = 600.0
#     t = 0.0
#     switched = False
#
#     while t < T_end:
#         if t < 300.0:
#             for d in drones:
#                 rwp.apply(d, ctrls[d.id], dt)
#                 ctrls[d.id].step(dt)
#         else:
#             if not switched:
#                 print(f"[EVENT] t={t:.1f}s switch to V-formation")
#                 switched = True
#             wp = (180.0, 100.0)  # leader waypoint
#             step_formation2d(drones, ctrls, fm, dt, leader_waypoint=wp, leader_speed=5.5)
#
#         if int(t) % 10 == 0 and abs(t - round(t)) < 1e-9:
#             pos = [ (round(d.position[0],1), round(d.position[1],1)) for d in drones ]
#             e   = ", ".join([f"{d.id}:{d.energy:.0f}mWh" for d in drones])
#             print(f"t={t:5.1f}s  pos={pos}  energy[{e}]")
#
#         t += dt
#
# if __name__ == "__main__":
#     run_sim_2d()
# ============================================================================

import random
from typing import List, Dict, Optional, Tuple

try:
    from .phy import Drone, wifi_profiles, transmit_packet
    from .mac import MACLayer, MACPacket, PacketType
    from .mobility import Mobility2D, RandomWaypoint2D, Formation2D, step_formation2d, v_offsets, line_offsets, grid_offsets
    from .metrics import MetricsCollector
except ImportError:
    from drone_sim.phy import Drone, wifi_profiles, transmit_packet
    from drone_sim.mac import MACLayer, MACPacket, PacketType
    from drone_sim.mobility import Mobility2D, RandomWaypoint2D, Formation2D, step_formation2d, v_offsets, line_offsets, grid_offsets
    from drone_sim.metrics import MetricsCollector


class IntegratedSimulator:
    """Integrated simulator that coordinates PHY, MAC, Mobility, and Metrics"""
    
    def __init__(self, 
                 num_nodes: int = 5,
                 area: Tuple[float, float] = (500.0, 500.0),
                 comm_profile: str = "802.11ac",
                 initial_energy: float = 5000.0,
                 mac_config: Optional[Dict] = None,
                 seed: Optional[int] = None):
        self.num_nodes = num_nodes
        self.area = area
        self.comm_profile = comm_profile
        self.initial_energy = initial_energy
        self.mac_config = mac_config or {}
        self.seed = seed
        
        if seed is not None:
            random.seed(seed)
        
        # Initialize drones, MAC layers, and mobility controllers
        self.drones: List[Drone] = []
        self.mac_layers: Dict[int, MACLayer] = {}
        self.mobility_controllers: Dict[int, Mobility2D] = {}
        
        for i in range(num_nodes):
            x = random.uniform(0, area[0])
            y = random.uniform(0, area[1])
            drone = Drone(i + 1, x, y, comm_profile, energy=initial_energy)
            self.drones.append(drone)
            self.mac_layers[drone.id] = MACLayer(drone, **self.mac_config)
            self.mobility_controllers[drone.id] = Mobility2D(drone, max_speed=6.0)
        
        self.metrics = MetricsCollector()
        self.current_time = 0.0
        self.dt = 0.1
        self.channel_state: Dict[int, float] = {}
        self.traffic_flows: List[Dict] = []
        self.packet_counter = 0
        
    def add_traffic_flow(self, src_id: int, dst_id: int, 
                         packet_rate: float = 1.0, 
                         start_time: float = 0.0,
                         end_time: Optional[float] = None):
        """Add a traffic flow between two nodes"""
        self.traffic_flows.append({
            'src': src_id, 'dst': dst_id, 'rate': packet_rate,
            'start': start_time, 'end': end_time, 'last_send': start_time
        })
    
    def discover_neighbors_phy(self):
        """Update PHY layer neighbor discovery"""
        for drone in self.drones:
            drone.discover_neighbors(self.drones)
    
    def _try_transmit(self, sender: Drone, receiver: Drone, packet: MACPacket, 
                     duration: float, collision: bool) -> bool:
        """Helper to attempt packet transmission"""
        if not receiver.online:
            return False
        
        distance = sender.distance_to(receiver)
        max_range = wifi_profiles[sender.comm_profile]["range"]
        
        if distance > max_range:
            return False
        
        if collision and packet.packet_type == PacketType.DATA:
            self.mac_layers[sender.id].stats['collisions'] += 1
            return False
        
        if collision and packet.packet_type in [PacketType.ACK, PacketType.RTS, PacketType.CTS]:
            return False
        
        success = transmit_packet(sender, receiver, duration_s=duration)
        if success:
            self.mac_layers[receiver.id].process_received_packet(packet, sender, self.current_time)
            if packet.packet_type == PacketType.DATA:
                packet_id = f"{sender.id}_{packet.seq_num}"
                self.metrics.record_packet_received(packet_id, self.current_time)
        return success
    
    def process_mac_transmissions(self):
        """Process MAC layer transmissions and handle packet delivery"""
        all_packets: List[Tuple[int, MACPacket]] = []
        
        # Collect all packets to transmit
        for drone in self.drones:
            if drone.online:
                packets = self.mac_layers[drone.id].step(self.current_time, self.drones, self.channel_state)
                all_packets.extend([(drone.id, p) for p in packets])
        
        # Check for collisions (simplified: multiple transmitters = collision)
        collision = len(set(sid for sid, _ in all_packets)) > 1
        
        # Process each transmission
        packet_durations = {
            PacketType.BEACON: 0.005,
            PacketType.DATA: 0.01,
            PacketType.ACK: 0.001,
            PacketType.RTS: 0.002,
            PacketType.CTS: 0.002
        }
        
        for sender_id, packet in all_packets:
            sender = next(d for d in self.drones if d.id == sender_id)
            duration = packet_durations.get(packet.packet_type, 0.01)
            
            if packet.packet_type == PacketType.BEACON:
                # Broadcast to all neighbors
                for receiver in self.drones:
                    if receiver.id != sender_id and receiver.id in sender.neighbors:
                        self._try_transmit(sender, receiver, packet, duration, collision)
            else:
                # Unicast to destination
                receiver = next((d for d in self.drones if d.id == packet.dst_id), None)
                if receiver:
                    self._try_transmit(sender, receiver, packet, duration, collision)
    
    def generate_traffic(self):
        """Generate traffic based on configured flows"""
        for flow in self.traffic_flows:
            if flow['end'] is not None and self.current_time > flow['end']:
                continue
            if self.current_time < flow['start']:
                continue
            
            time_since_last = self.current_time - flow['last_send']
            if time_since_last >= 1.0 / flow['rate']:
                src_drone = next((d for d in self.drones if d.id == flow['src']), None)
                if src_drone and src_drone.online:
                    mac = self.mac_layers[flow['src']]
                    data = {'flow_id': self.packet_counter, 'payload': 'data'}
                    if mac.enqueue_packet(flow['dst'], data):
                        packet_id = f"{flow['src']}_{mac.seq_num - 1}"
                        self.metrics.record_packet_sent(packet_id, self.current_time, flow['src'], flow['dst'])
                        flow['last_send'] = self.current_time
    
    def step(self, dt: Optional[float] = None):
        """Perform one simulation step"""
        if dt is None:
            dt = self.dt
        
        self.current_time += dt
        self.discover_neighbors_phy()
        self.generate_traffic()
        self.process_mac_transmissions()
        self.metrics.record_step(self.current_time, self.drones, self.mac_layers)
        
        # Update channel state
        self.channel_state.clear()
        for drone in self.drones:
            mac = self.mac_layers[drone.id]
            if mac.channel_busy_until > self.current_time:
                self.channel_state[drone.id] = mac.channel_busy_until
    
    def run(self, duration: float, dt: Optional[float] = None):
        """Run simulation for specified duration"""
        if dt is None:
            dt = self.dt
        steps = int(duration / dt)
        for _ in range(steps):
            self.step(dt)
    
    def get_summary(self) -> Dict:
        """Get summary of simulation results"""
        active = sum(1 for d in self.drones if d.online)
        return {
            'duration': self.current_time,
            'metrics': self.metrics.get_summary(),
            'active_nodes': active,
            'avg_energy': sum(d.energy for d in self.drones if d.online) / max(active, 1),
        }


def run_sim_2d():
    """Simple 2D simulation example"""
    drones = [
        Drone(1,  0,  0, "802.11ac"),
        Drone(2, 10,  0, "802.11ac"),
        Drone(3, 20,  0, "802.11ac"),
        Drone(4, 30,  0, "802.11ac"),
        Drone(5, 40,  0, "802.11ac"),
    ]

    ctrls = {d.id: Mobility2D(d, max_speed=6.0) for d in drones}
    rwp = RandomWaypoint2D(area=(200, 200), pause_range=(0.0, 0.5), seed=42)
    leader_id = 1
    followers = [d.id for d in drones if d.id != leader_id]
    fm = Formation2D(leader_id, v_offsets(followers, spacing=10.0, angle_deg=28.0))

    dt = 0.2
    T_end = 600.0
    t = 0.0
    switched = False

    while t < T_end:
        if t < 300.0:
            for d in drones:
                rwp.apply(d, ctrls[d.id], dt)
                ctrls[d.id].step(dt)
        else:
            if not switched:
                print(f"[EVENT] t={t:.1f}s switch to V-formation")
                switched = True
            wp = (180.0, 100.0)
            step_formation2d(drones, ctrls, fm, dt, leader_waypoint=wp, leader_speed=5.5)

        if int(t) % 10 == 0 and abs(t - round(t)) < 1e-9:
            pos = [ (round(d.position[0],1), round(d.position[1],1)) for d in drones ]
            e   = ", ".join([f"{d.id}:{d.energy:.0f}mWh" for d in drones])
            print(f"t={t:5.1f}s  pos={pos}  energy[{e}]")

        t += dt


def run():
    """Main entry point for simulator (called by main.py)"""
    run_sim_2d()


if __name__ == "__main__":
    run()
