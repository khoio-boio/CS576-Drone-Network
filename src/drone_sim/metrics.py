"""
Metrics Collection for Drone Network Simulation

This module provides a MetricsCollector class that tracks and calculates
key performance indicators (KPIs) for the drone network experiments.

Key Metrics Tracked:
- Packet Delivery Ratio (PDR): Percentage of successfully delivered packets
- Latency: End-to-end packet delivery time
- Route Churn: Number of route changes in the network
- Energy Consumption: Average energy usage per node
- Throughput: Data delivery rate
- Network Stability: Tracks network state before/during/after transitions

Used by all three experiments (E1, E2, E3) to collect performance data
and generate results for analysis.
"""

from typing import List, Dict, Optional
from collections import defaultdict, deque
import time

try:
    from .phy import Drone
    from .mac import MACLayer
except ImportError:
    from drone_sim.phy import Drone
    from drone_sim.mac import MACLayer


class MetricsCollector:
    """Collects and calculates network performance metrics"""
    
    def __init__(self):
        # Packet tracking
        self.sent_packets = {}  # packet_id -> send_time
        self.received_packets = {}  # packet_id -> receive_time
        self.packet_latencies = []  # List of latencies
        
        # Route tracking
        self.route_history = defaultdict(list)  # node_id -> [(time, route_table), ...]
        self.route_changes = 0
        self.last_route_snapshot = {}  # node_id -> route_table
        
        # Energy tracking
        self.energy_history = defaultdict(list)  # node_id -> [(time, energy), ...]
        self.initial_energy = {}
        
        # Throughput tracking
        self.bytes_sent = 0
        self.bytes_received = 0
        self.packets_sent_count = 0
        self.packets_received_count = 0
        
        # Time windows for phase analysis
        self.transition_time = None
        self.before_transition_packets = {'sent': 0, 'received': 0}
        self.after_transition_packets = {'sent': 0, 'received': 0}
        
        # Network stability tracking
        self.stability_window = deque(maxlen=10)  # Last 10 steps
        self.stable_threshold = 0.95  # PDR threshold for stability
        
    def record_step(self, current_time: float, drones: List[Drone], 
                   mac_layers: Dict[int, MACLayer]) -> dict:
        """Record metrics for a single simulation step"""
        step_metrics = {
            'time': current_time,
            'pdr': 0.0,
            'latency': 0.0,
            'route_churn': 0,
            'active_nodes': 0,
            'avg_energy': 0.0,
        }
        
        # Track active nodes
        active_nodes = sum(1 for d in drones if d.online)
        step_metrics['active_nodes'] = active_nodes
        
        # Track energy
        total_energy = sum(d.energy for d in drones if d.online)
        step_metrics['avg_energy'] = total_energy / max(active_nodes, 1)
        
        # Track routes
        route_changes_this_step = 0
        for drone in drones:
            if drone.id not in self.last_route_snapshot:
                self.last_route_snapshot[drone.id] = dict(drone.route_table)
            else:
                old_routes = self.last_route_snapshot[drone.id]
                new_routes = dict(drone.route_table)
                
                # Count route changes
                for dst, next_hop in new_routes.items():
                    if dst not in old_routes or old_routes[dst] != next_hop:
                        route_changes_this_step += 1
                
                self.last_route_snapshot[drone.id] = new_routes
        
        step_metrics['route_churn'] = route_changes_this_step
        self.route_changes += route_changes_this_step
        
        # Calculate PDR and latency from collected data
        if self.packets_sent_count > 0:
            step_metrics['pdr'] = self.packets_received_count / self.packets_sent_count
        
        if self.packet_latencies:
            step_metrics['latency'] = sum(self.packet_latencies) / len(self.packet_latencies)
        
        # Store stability metric
        self.stability_window.append(step_metrics['pdr'])
        
        return step_metrics
    
    def record_packet_sent(self, packet_id: int, send_time: float, 
                          src_id: int, dst_id: int):
        """Record a packet being sent"""
        self.sent_packets[packet_id] = {
            'send_time': send_time,
            'src': src_id,
            'dst': dst_id
        }
        self.packets_sent_count += 1
        self.bytes_sent += 1024  # Assume 1KB per packet
        
        # Track before/after transition
        if self.transition_time is not None:
            if send_time < self.transition_time:
                self.before_transition_packets['sent'] += 1
            else:
                self.after_transition_packets['sent'] += 1
    
    def record_packet_received(self, packet_id: int, receive_time: float):
        """Record a packet being received"""
        if packet_id in self.sent_packets:
            send_time = self.sent_packets[packet_id]['send_time']
            latency = receive_time - send_time
            self.packet_latencies.append(latency)
            self.received_packets[packet_id] = receive_time
            self.packets_received_count += 1
            self.bytes_received += 1024  # Assume 1KB per packet
            
            # Track before/after transition
            if self.transition_time is not None:
                if send_time < self.transition_time:
                    self.before_transition_packets['received'] += 1
                else:
                    self.after_transition_packets['received'] += 1
    
    def get_pdr(self) -> float:
        """Get Packet Delivery Ratio"""
        if self.packets_sent_count == 0:
            return 0.0
        return self.packets_received_count / self.packets_sent_count
    
    def get_avg_latency(self) -> float:
        """Get average packet latency"""
        if not self.packet_latencies:
            return 0.0
        return sum(self.packet_latencies) / len(self.packet_latencies)
    
    def get_total_received(self) -> int:
        """Get total number of received packets"""
        return self.packets_received_count
    
    def get_route_churn(self) -> int:
        """Get total route changes"""
        return self.route_changes
    
    def get_total_route_churn(self) -> int:
        """Get total route churn (alias)"""
        return self.get_route_churn()
    
    def get_throughput(self) -> float:
        """Get throughput in bytes per second"""
        # This would need total simulation time
        # For now, return bytes received
        return self.bytes_received
    
    def get_avg_energy_consumption(self) -> float:
        """Get average energy consumption per node"""
        if not self.initial_energy:
            return 0.0
        
        total_consumed = sum(init - final for init, final in 
                           zip(self.initial_energy.values(), 
                               [e[-1][1] if e else 0 for e in self.energy_history.values()]))
        return total_consumed / max(len(self.initial_energy), 1)
    
    def set_transition_time(self, transition_time: float):
        """Set the time when formation transition occurs"""
        self.transition_time = transition_time
    
    def get_pdr_before_transition(self) -> float:
        """Get PDR before formation transition"""
        if self.before_transition_packets['sent'] == 0:
            return 0.0
        return (self.before_transition_packets['received'] / 
                self.before_transition_packets['sent'])
    
    def get_pdr_after_transition(self) -> float:
        """Get PDR after formation transition"""
        if self.after_transition_packets['sent'] == 0:
            return 0.0
        return (self.after_transition_packets['received'] / 
                self.after_transition_packets['sent'])
    
    def is_network_stable(self) -> bool:
        """Check if network has reached stability (PDR consistently high)"""
        if len(self.stability_window) < 10:
            return False
        
        avg_pdr = sum(self.stability_window) / len(self.stability_window)
        return avg_pdr >= self.stable_threshold
    
    def get_summary(self) -> dict:
        """Get summary of all metrics"""
        return {
            'pdr': self.get_pdr(),
            'avg_latency': self.get_avg_latency(),
            'total_sent': self.packets_sent_count,
            'total_received': self.packets_received_count,
            'route_churn': self.get_route_churn(),
            'throughput_bytes': self.bytes_received,
            'avg_energy_consumption': self.get_avg_energy_consumption(),
        }
    
    def reset(self):
        """Reset all metrics"""
        self.sent_packets.clear()
        self.received_packets.clear()
        self.packet_latencies.clear()
        self.route_history.clear()
        self.route_changes = 0
        self.last_route_snapshot.clear()
        self.energy_history.clear()
        self.initial_energy.clear()
        self.bytes_sent = 0
        self.bytes_received = 0
        self.packets_sent_count = 0
        self.packets_received_count = 0
        self.before_transition_packets = {'sent': 0, 'received': 0}
        self.after_transition_packets = {'sent': 0, 'received': 0}
        self.stability_window.clear()

