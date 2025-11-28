"""
MAC Layer Implementation
- CSMA/CA (Carrier Sense Multiple Access with Collision Avoidance)
- ACKs (Acknowledgments)
- Retry limit
- Optional RTS/CTS
- Per-node FIFO queues with capacity
- Beacons for neighbor table & expiry
"""

import random
import time
from collections import deque
from typing import Dict, List, Optional, Tuple, Set
from enum import Enum

try:
    from .phy import Drone, transmit_packet, wifi_profiles
except ImportError:
    from drone_sim.phy import Drone, transmit_packet, wifi_profiles


class PacketType(Enum):
    """Packet types for MAC layer"""
    DATA = "DATA"
    ACK = "ACK"
    RTS = "RTS"  # Request to Send
    CTS = "CTS"  # Clear to Send
    BEACON = "BEACON"


class MACState(Enum):
    """MAC layer states"""
    IDLE = "IDLE"
    SENSING = "SENSING"  # Carrier sensing
    BACKOFF = "BACKOFF"  # Exponential backoff
    TRANSMITTING = "TRANSMITTING"
    WAITING_ACK = "WAITING_ACK"
    RECEIVING = "RECEIVING"
    SENDING_ACK = "SENDING_ACK"


class MACPacket:
    """Represents a packet in the MAC layer"""
    def __init__(self, src_id: int, dst_id: int, packet_type: PacketType, 
                 data: Optional[dict] = None, seq_num: int = 0):
        self.src_id = src_id
        self.dst_id = dst_id
        self.packet_type = packet_type
        self.data = data or {}
        self.seq_num = seq_num
        self.timestamp = time.time()
        self.retry_count = 0

    def __repr__(self):
        return f"{self.packet_type.value}({self.src_id}->{self.dst_id}, seq={self.seq_num})"


class NeighborEntry:
    """Entry in neighbor table with expiry"""
    def __init__(self, neighbor_id: int, last_seen: float, link_quality: float = 1.0):
        self.neighbor_id = neighbor_id
        self.last_seen = last_seen
        self.link_quality = link_quality  # 0.0 to 1.0
        self.expired = False

    def update(self, current_time: float, link_quality: float = None):
        """Update last seen time and optionally link quality"""
        self.last_seen = current_time
        if link_quality is not None:
            self.link_quality = link_quality
        self.expired = False

    def check_expiry(self, current_time: float, expiry_time: float):
        """Check if entry has expired"""
        if current_time - self.last_seen > expiry_time:
            self.expired = True
        return self.expired


class MACLayer:
    """
    MAC Layer with CSMA/CA, ACKs, retry, optional RTS/CTS, queues, and beacons
    """
    def __init__(self, drone: Drone, 
                 queue_capacity: int = 50,
                 cw_min: int = 15,  # Minimum contention window
                 cw_max: int = 1023,  # Maximum contention window
                 max_retries: int = 7,
                 ack_timeout: float = 0.01,  # seconds
                 beacon_interval: float = 1.0,  # seconds
                 neighbor_expiry: float = 3.0,  # seconds
                 use_rts_cts: bool = False,
                 slot_time: float = 0.000009,  # 9 microseconds (802.11)
                 sifs: float = 0.000016,  # 16 microseconds
                 difs: float = 0.000034):  # 34 microseconds
        self.drone = drone
        self.queue = deque(maxlen=queue_capacity)  # FIFO queue
        self.queue_capacity = queue_capacity
        
        # CSMA/CA parameters
        self.cw_min = cw_min
        self.cw_max = cw_max
        self.current_cw = cw_min
        self.backoff_counter = 0
        self.max_retries = max_retries
        
        # Timing parameters
        self.ack_timeout = ack_timeout
        self.slot_time = slot_time
        self.sifs = sifs
        self.difs = difs
        
        # State management
        self.state = MACState.IDLE
        self.current_packet: Optional[MACPacket] = None
        self.pending_ack: Optional[MACPacket] = None
        self.pending_ack_to_send: Optional[MACPacket] = None  # ACK waiting to be sent
        self.ack_timer: float = 0.0
        self.sensing_timer: float = 0.0
        self.backoff_timer: float = 0.0
        
        # Neighbor table with expiry
        self.neighbor_table: Dict[int, NeighborEntry] = {}
        self.beacon_interval = beacon_interval
        self.neighbor_expiry = neighbor_expiry
        self.last_beacon_time: float = 0.0
        
        # RTS/CTS
        self.use_rts_cts = use_rts_cts
        self.rts_timer: float = 0.0
        self.cts_received: bool = False
        
        # Sequence numbers
        self.seq_num = 0
        self.received_seqs: Set[int] = set()  # Track received sequence numbers
        
        # Statistics
        self.stats = {
            'tx_success': 0,
            'tx_failed': 0,
            'tx_retries': 0,
            'collisions': 0,
            'queue_drops': 0,
            'beacons_sent': 0,
            'beacons_received': 0,
        }
        
        # Channel state (for carrier sensing)
        self.channel_busy = False
        self.channel_busy_until: float = 0.0

    def enqueue_packet(self, dst_id: int, data: dict) -> bool:
        """Add a packet to the transmission queue"""
        if len(self.queue) >= self.queue_capacity:
            self.stats['queue_drops'] += 1
            return False
        
        packet = MACPacket(self.drone.id, dst_id, PacketType.DATA, data, self.seq_num)
        self.seq_num += 1
        self.queue.append(packet)
        return True

    def carrier_sense(self, current_time: float) -> bool:
        """Check if channel is busy (carrier sensing)"""
        if current_time < self.channel_busy_until:
            return True
        return False

    def start_transmission(self, packet: MACPacket, current_time: float):
        """Start transmitting a packet"""
        self.state = MACState.TRANSMITTING
        self.current_packet = packet
        
        # Mark channel as busy
        duration = 0.01  # Transmission duration (simplified)
        self.channel_busy_until = current_time + duration
        
        # Update energy
        self.drone.update_energy("tx", duration)

    def send_ack(self, src_id: int, seq_num: int, current_time: float):
        """Send acknowledgment packet"""
        ack = MACPacket(self.drone.id, src_id, PacketType.ACK, {'ack_seq': seq_num})
        self.state = MACState.SENDING_ACK
        
        # Mark channel as busy
        duration = 0.001  # ACK is shorter
        self.channel_busy_until = current_time + duration
        
        # Update energy
        self.drone.update_energy("tx", duration)
        
        return ack

    def send_rts(self, dst_id: int, current_time: float) -> MACPacket:
        """Send RTS (Request to Send)"""
        rts = MACPacket(self.drone.id, dst_id, PacketType.RTS)
        self.state = MACState.TRANSMITTING
        
        duration = 0.002
        self.channel_busy_until = current_time + duration
        self.drone.update_energy("tx", duration)
        
        return rts

    def send_cts(self, src_id: int, current_time: float) -> MACPacket:
        """Send CTS (Clear to Send)"""
        cts = MACPacket(self.drone.id, src_id, PacketType.CTS)
        self.state = MACState.TRANSMITTING
        
        duration = 0.002
        self.channel_busy_until = current_time + duration
        self.drone.update_energy("tx", duration)
        
        return cts

    def send_beacon(self, current_time: float) -> MACPacket:
        """Send beacon packet for neighbor discovery"""
        beacon = MACPacket(self.drone.id, 0, PacketType.BEACON, {
            'position': self.drone.position,
            'energy': self.drone.energy,
            'neighbors': list(self.neighbor_table.keys())
        })
        
        self.state = MACState.TRANSMITTING
        duration = 0.005
        self.channel_busy_until = current_time + duration
        self.drone.update_energy("tx", duration)
        
        self.last_beacon_time = current_time
        self.stats['beacons_sent'] += 1
        
        return beacon

    def exponential_backoff(self):
        """Implement exponential backoff for CSMA/CA"""
        if self.current_cw < self.cw_max:
            self.current_cw = min(self.current_cw * 2 + 1, self.cw_max)
        self.backoff_counter = random.randint(0, self.current_cw)

    def reset_backoff(self):
        """Reset backoff to initial state"""
        self.current_cw = self.cw_min
        self.backoff_counter = 0

    def process_received_packet(self, packet: MACPacket, sender: Drone, current_time: float) -> bool:
        """Process a received packet"""
        if not self.drone.online:
            return False
        
        # Update energy for receiving
        self.drone.update_energy("rx", 0.01)
        
        # Handle different packet types
        if packet.packet_type == PacketType.BEACON:
            self.stats['beacons_received'] += 1
            # Update neighbor table
            if packet.src_id not in self.neighbor_table:
                self.neighbor_table[packet.src_id] = NeighborEntry(packet.src_id, current_time)
            else:
                self.neighbor_table[packet.src_id].update(current_time)
            return True
            
        elif packet.packet_type == PacketType.RTS:
            if packet.dst_id == self.drone.id:
                # Send CTS
                cts = self.send_cts(packet.src_id, current_time)
                return True
            return False
            
        elif packet.packet_type == PacketType.CTS:
            if packet.dst_id == self.drone.id:
                self.cts_received = True
                return True
            return False
            
        elif packet.packet_type == PacketType.DATA:
            if packet.dst_id == self.drone.id:
                # Check for duplicate
                if packet.seq_num in self.received_seqs:
                    # Duplicate, send ACK anyway
                    self.send_ack(packet.src_id, packet.seq_num, current_time)
                    return True
                
                # New packet, add to received set
                self.received_seqs.add(packet.seq_num)
                # Clean old sequence numbers (keep last 1000)
                if len(self.received_seqs) > 1000:
                    self.received_seqs = set(list(self.received_seqs)[-1000:])
                
                # Send ACK (will be handled in step function)
                ack = self.send_ack(packet.src_id, packet.seq_num, current_time)
                # Store ACK to be sent in next step
                self.pending_ack_to_send = ack
                return True
            return False
            
        elif packet.packet_type == PacketType.ACK:
            if packet.dst_id == self.drone.id and self.pending_ack:
                ack_seq = packet.data.get('ack_seq')
                if ack_seq == self.pending_ack.seq_num:
                    # ACK received successfully
                    self.stats['tx_success'] += 1
                    self.pending_ack = None
                    self.ack_timer = 0.0
                    self.reset_backoff()
                    self.state = MACState.IDLE
                    return True
            return False
        
        return False

    def update_neighbor_table(self, current_time: float):
        """Update neighbor table and remove expired entries"""
        expired_ids = []
        for neighbor_id, entry in self.neighbor_table.items():
            if entry.check_expiry(current_time, self.neighbor_expiry):
                expired_ids.append(neighbor_id)
        
        for neighbor_id in expired_ids:
            del self.neighbor_table[neighbor_id]

    def step(self, current_time: float, all_drones: List[Drone], channel_state: Dict) -> List[MACPacket]:
        """
        Main MAC layer step function - called each simulation step
        Returns list of packets to transmit
        """
        packets_to_transmit = []
        
        if not self.drone.online:
            return packets_to_transmit
        
        # Update neighbor table expiry
        self.update_neighbor_table(current_time)
        
        # Send pending ACK if available
        if self.pending_ack_to_send and self.state == MACState.IDLE:
            packets_to_transmit.append(self.pending_ack_to_send)
            self.pending_ack_to_send = None
        
        # Send beacon if interval elapsed
        if current_time - self.last_beacon_time >= self.beacon_interval:
            if self.state == MACState.IDLE:
                beacon = self.send_beacon(current_time)
                packets_to_transmit.append(beacon)
        
        # Check channel state
        self.channel_busy = self.carrier_sense(current_time)
        
        # State machine
        if self.state == MACState.IDLE:
            # Check if we have packets to send
            if len(self.queue) > 0:
                self.current_packet = self.queue[0]
                
                # Check if destination is in neighbor table
                if self.current_packet.dst_id not in self.neighbor_table:
                    # Destination not known, drop packet or wait
                    # For now, we'll try anyway
                    pass
                
                if not self.channel_busy:
                    # Start DIFS
                    self.state = MACState.SENSING
                    self.sensing_timer = current_time + self.difs
                else:
                    # Channel busy, wait
                    pass
        
        elif self.state == MACState.SENSING:
            if current_time >= self.sensing_timer:
                if not self.channel_busy:
                    # DIFS complete, start backoff
                    self.state = MACState.BACKOFF
                    self.exponential_backoff()
                    self.backoff_timer = current_time + (self.backoff_counter * self.slot_time)
                else:
                    # Channel became busy, restart sensing
                    self.sensing_timer = current_time + self.difs
        
        elif self.state == MACState.BACKOFF:
            if current_time >= self.backoff_timer:
                if not self.channel_busy:
                    # Backoff complete, can transmit
                    if self.use_rts_cts and self.current_packet:
                        # Send RTS first
                        rts = self.send_rts(self.current_packet.dst_id, current_time)
                        packets_to_transmit.append(rts)
                        self.rts_timer = current_time + 0.01  # Wait for CTS
                        self.cts_received = False
                    else:
                        # Direct transmission
                        if self.current_packet:
                            self.start_transmission(self.current_packet, current_time)
                            packets_to_transmit.append(self.current_packet)
                            # Set up ACK waiting
                            self.pending_ack = self.current_packet
                            self.ack_timer = current_time + self.ack_timeout
                            self.state = MACState.WAITING_ACK
                else:
                    # Channel busy during backoff, pause and resume later
                    remaining = self.backoff_timer - current_time
                    self.backoff_timer = current_time + remaining
        
        elif self.state == MACState.WAITING_ACK:
            if current_time >= self.ack_timer:
                # ACK timeout
                if self.current_packet:
                    self.current_packet.retry_count += 1
                    self.stats['tx_retries'] += 1
                    
                    if self.current_packet.retry_count >= self.max_retries:
                        # Max retries exceeded, drop packet
                        self.stats['tx_failed'] += 1
                        if len(self.queue) > 0 and self.queue[0] == self.current_packet:
                            self.queue.popleft()  # Remove from queue
                        self.current_packet = None
                        self.pending_ack = None
                        self.reset_backoff()
                        self.state = MACState.IDLE
                    else:
                        # Retry
                        self.exponential_backoff()
                        self.state = MACState.BACKOFF
                        self.backoff_timer = current_time + (self.backoff_counter * self.slot_time)
            # ACK received is handled in process_received_packet
            elif self.pending_ack is None:
                # ACK was received (handled in process_received_packet)
                # Remove packet from queue
                if len(self.queue) > 0 and self.current_packet and self.queue[0] == self.current_packet:
                    self.queue.popleft()
                self.current_packet = None
        
        elif self.state == MACState.TRANSMITTING:
            # Transmission complete
            if current_time >= self.channel_busy_until:
                if self.current_packet and self.current_packet.packet_type == PacketType.DATA:
                    # Data packet sent, wait for ACK
                    self.state = MACState.WAITING_ACK
                else:
                    # Other packet types (RTS, CTS, ACK, BEACON) complete
                    self.state = MACState.IDLE
                    self.current_packet = None
        
        elif self.state == MACState.SENDING_ACK:
            # ACK transmission complete
            if current_time >= self.channel_busy_until:
                self.state = MACState.IDLE
                self.current_packet = None
        
        return packets_to_transmit

    def transmit_packet_via_phy(self, packet: MACPacket, receiver: Drone, current_time: float) -> bool:
        """
        Transmit a packet via PHY layer and process the result
        Returns True if transmission was successful
        """
        if not self.drone.online or not receiver.online:
            return False
        
        # Use PHY layer transmission
        success = transmit_packet(self.drone, receiver, duration_s=0.01)
        
        # If successful, process at receiver's MAC layer
        if success and packet.dst_id == receiver.id:
            # Find receiver's MAC layer (would need to be passed in or stored)
            # For now, we'll handle this in the simulator
            pass
        
        return success

    def get_stats(self) -> dict:
        """Get MAC layer statistics"""
        return {
            **self.stats,
            'queue_size': len(self.queue),
            'neighbor_count': len(self.neighbor_table),
            'current_cw': self.current_cw,
        }

    def get_neighbors(self) -> Set[int]:
        """Get set of neighbor IDs from neighbor table"""
        return set(self.neighbor_table.keys())

