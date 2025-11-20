"""
Node wrapper that integrates PHY, MAC, and Network layers
"""

try:
    from .phy import Drone
    from .mac import MACLayer
except ImportError:
    from drone_sim.phy import Drone
    from drone_sim.mac import MACLayer


class NetworkNode:
    """
    Wrapper class that combines Drone (PHY) with MAC layer
    Provides unified interface for network operations
    """
    
    def __init__(self, drone: Drone, mac_config: dict = None):
        self.drone = drone
        mac_config = mac_config or {}
        self.mac = MACLayer(drone, **mac_config)
        
    @property
    def id(self):
        return self.drone.id
    
    @property
    def position(self):
        return self.drone.position
    
    @property
    def energy(self):
        return self.drone.energy
    
    @property
    def online(self):
        return self.drone.online
    
    @property
    def neighbors(self):
        """Get neighbors from both PHY discovery and MAC neighbor table"""
        phy_neighbors = self.drone.neighbors
        mac_neighbors = self.mac.get_neighbors()
        return phy_neighbors.union(mac_neighbors)
    
    def send_packet(self, dst_id: int, data: dict) -> bool:
        """Send a packet via MAC layer"""
        return self.mac.enqueue_packet(dst_id, data)
    
    def get_mac_stats(self) -> dict:
        """Get MAC layer statistics"""
        return self.mac.get_stats()
    
    def __repr__(self):
        return f"NetworkNode({self.drone.id}, energy={self.drone.energy:.1f}, " \
               f"neighbors={len(self.neighbors)}, queue={len(self.mac.queue)})"

