import random
import math
from collections import deque

# ==========================================================
# Wi-Fi PHY Profiles
# ==========================================================
# Define Wi-Fi tech profiles
wifi_profiles = {
    "802.11n":     {"range": 100, "max_rate": 54,  "tx_power": 15},
    "802.11ac":    {"range": 200, "max_rate": 433, "tx_power": 20},
    "wifi_direct": {"range": 120, "max_rate": 250, "tx_power": 15},
}

# Power consumption per state in mW
power_profile = {
    "tx": 1500,    # Transmit
    "rx": 900,     # Receive
    "idle": 250,   # Listening but not receiving
    "sleep": 20,   # Low power state
    "move": 500,   # Additional power when moving
}

# ==========================================================
# Basic Drone Structure with Formation & Network Control
# ==========================================================
class Drone:
    def __init__(self, drone_id, x, y, comm_profile, energy=5000):
        self.id = drone_id
        self.position = [x, y]               # current position
        self.comm_profile = comm_profile
        self.energy = energy                 # mWh
        self.state = "idle"
        self.tx_power = wifi_profiles[comm_profile]["tx_power"]

        # -------------------------------
        # Network-layer attributes
        # -------------------------------
        self.neighbors = set()               # reachable drones
        self.parent_id = None                # parent in topology tree
        self.children = []                   # children nodes
        self.route_table = {}                # destination -> next hop
        self.buffer = deque(maxlen=20)       # buffered packets for disconnection

        # -------------------------------
        # Formation & movement attributes
        # -------------------------------
        self.role = "follower"               # "leader" or "follower"
        self.formation_offset = (0, 0)       # relative offset to leader
        self.speed = 5.0                     # m/s (default)
        self.velocity = [0.0, 0.0]           # vx, vy components
        self.speed_scale = 1.0               # global speed scaling factor

        # -------------------------------
        # Status flags
        # -------------------------------
        self.online = True                   # False if energy depleted

    # ----------------------------------------------------------
    # Distance calculation
    # ----------------------------------------------------------
    def distance_to(self, other):
        x1, y1 = self.position
        x2, y2 = other.position
        return math.hypot(x2 - x1, y2 - y1)

    # ----------------------------------------------------------
    # Energy consumption and state updates
    # ----------------------------------------------------------
    def update_energy(self, state, duration_s):
        """Update drone energy based on power profile and duration."""
        if not self.online:
            return
        power_mW = power_profile.get(state, 0)
        energy_used = power_mW * duration_s / 3600  # convert to mWh
        self.energy = max(self.energy - energy_used, 0)
        self.state = state
        if self.energy <= 0:
            self.energy = 0
            self.online = False
            self.state = "offline"

    def sleep_mode(self):
        """Enter low-power sleep state."""
        self.update_energy("sleep", 1)
        self.state = "sleep"

    # ----------------------------------------------------------
    # Motion model (simple constant-velocity model)
    # ----------------------------------------------------------
    def move(self, dt=1.0):
        """Update position based on velocity and speed scale."""
        if not self.online:
            return
        vx, vy = self.velocity
        self.position[0] += vx * dt * self.speed_scale
        self.position[1] += vy * dt * self.speed_scale
        self.update_energy("move", dt)

    # ----------------------------------------------------------
    # Neighbor discovery (PHY range)
    # ----------------------------------------------------------
    def discover_neighbors(self, all_drones):
        """Find all reachable drones within PHY communication range."""
        self.neighbors.clear()
        if not self.online:
            return
        max_range = wifi_profiles[self.comm_profile]["range"]
        for other in all_drones:
            if other.id != self.id and other.online:
                if self.distance_to(other) <= max_range:
                    self.neighbors.add(other.id)

    def __repr__(self):
        return (f"Drone {self.id} | Energy: {self.energy:.1f} mWh | State: {self.state} "
                f"| Role: {self.role} | Pos: ({self.position[0]:.1f}, {self.position[1]:.1f})")

# ==========================================================
# RSSI + SNR Based Link Quality Estimator
# ==========================================================
def estimate_rssi(distance, tx_power_dbm, path_loss_exp=2.2):
    if distance < 1:
        distance = 1
    pl = 40 + 10 * path_loss_exp * math.log10(distance)
    return tx_power_dbm - pl

def estimate_snr(rssi_dbm, noise_floor_dbm=-90):
    return rssi_dbm - noise_floor_dbm

def link_quality_metric(distance, tx_power):
    rssi = estimate_rssi(distance, tx_power)
    snr = estimate_snr(rssi)
    snr = max(-10, min(snr, 40))
    return max(0.0, min(1.0, (snr + 10) / 50))

# ==========================================================
# Simulate a Lossy Channel (PHY)
# ==========================================================
def simulate_packet_loss(distance, max_range, base_loss=0.1):
    """Simulate distance-dependent packet loss."""
    if distance > max_range:
        return True  # Always lost if out of range
    # Probability increases with distance
    loss_prob = base_loss + 0.4 * (distance / max_range)
    return random.random() < loss_prob

# ==========================================================
# Simulate a Transmission between Two Drones (PHY)
# ==========================================================
def transmit_packet(sender, receiver, duration_s=0.01):
    """Simulate a single packet transmission event."""
    if not sender.online or not receiver.online:
        return False

    profile = wifi_profiles[sender.comm_profile]
    distance = sender.distance_to(receiver)
    max_range = profile["range"]

    lost = simulate_packet_loss(distance, max_range)
    success = not lost

    sender.update_energy("tx", duration_s)
    receiver.update_energy("rx" if success else "idle", duration_s)

    print(f"[TX] Drone {sender.id} -> Drone {receiver.id} | Distance: {int(distance)}m | Success: {success}")
    return success

# ==========================================================
# Simple Reactive Routing (AODV-like)
# ==========================================================
def route_discovery(src, dst, drones):
    """Flood-based reactive route discovery."""
    print(f"[Route Discovery] Drone {src.id} -> Drone {dst.id}")
    visited = set()
    queue = [(src.id, [])]

    while queue:
        current_id, path = queue.pop(0)
        if current_id == dst.id:
            final_path = path + [dst.id]
            for i in range(len(final_path) - 1):
                a, b = final_path[i], final_path[i + 1]
                drones[a - 1].route_table[dst.id] = b
            return final_path

        visited.add(current_id)
        for neighbor_id in drones[current_id - 1].neighbors:
            if neighbor_id not in visited:
                queue.append((neighbor_id, path + [current_id]))
    return None

# ==========================================================
# Formation Generator Utilities
# ==========================================================
def assign_formation(drones, formation="line", spacing=50):
    """Assign positions and formation offsets based on chosen shape."""
    leader = drones[0]
    leader.role = "leader"
    leader.formation_offset = (0, 0)

    if formation == "line":
        for i, d in enumerate(drones[1:], start=1):
            d.role = "follower"
            d.formation_offset = (i * spacing, 0)

    elif formation == "v":
        for i, d in enumerate(drones[1:], start=1):
            d.role = "follower"
            side = -1 if i % 2 == 0 else 1
            d.formation_offset = (i * spacing * 0.8, side * spacing * 0.6)

    elif formation == "grid":
        rows = int(math.sqrt(len(drones)))
        for idx, d in enumerate(drones):
            row, col = divmod(idx, rows)
            d.formation_offset = (col * spacing, row * spacing)
            d.role = "leader" if idx == 0 else "follower"

    elif formation == "square":
        for i, d in enumerate(drones):
            d.role = "follower" if i > 0 else "leader"
        offsets = [(0, 0), (spacing, 0), (0, spacing), (spacing, spacing)]
        for d, offset in zip(drones, offsets):
            d.formation_offset = offset

# ==========================================================
# Dynamic Leader Handoff
# ==========================================================
def dynamic_leader_handoff(drones, battery_threshold=300):
    leader = drones[0]

    if leader.energy > battery_threshold and len(leader.neighbors) >= 1:
        return  # no handoff

    print("\n[Leader Handoff Triggered] Leader too weak or disconnected")

    candidates = [d for d in drones if d.online]
    candidates.sort(key=lambda d: (d.energy, len(d.neighbors)), reverse=True)

    new_leader = candidates[0]
    idx_old = drones.index(leader)
    idx_new = drones.index(new_leader)

    drones[idx_old], drones[idx_new] = drones[idx_new], drones[idx_old]

    drones[0].role = "leader"
    for d in drones[1:]:
        d.role = "follower"

    print(f"[Leader Handoff] New leader is Drone {drones[0].id}")

# ==========================================================
# Network Simulation Loop
# ==========================================================

def run_simulation(drones, steps=10, dt=1.0):
    # Initial neighbor discovery
    for d in drones:
        d.discover_neighbors(drones)

    for step in range(steps):
        print(f"\n--- Step {step + 1} ---")

        # Leader moves forward; followers maintain relative offsets
        leader = drones[0]
        leader.velocity = [1.0, 0.0]  # move in +x direction
        leader.move(dt)

        for drone in drones[1:]:
            # Maintain formation relative to leader
            dx, dy = drone.formation_offset
            target_x = leader.position[0] - dx
            target_y = leader.position[1] + dy
            vx = (target_x - drone.position[0]) * 0.1
            vy = (target_y - drone.position[1]) * 0.1
            drone.velocity = [vx, vy]
            drone.move(dt)

        # Communication phase
        for drone in drones:
            if not drone.online:
                continue
            if drone.energy <= 50:
                drone.sleep_mode()
                continue

            if drone.neighbors:
                target_id = random.choice(list(drone.neighbors))
                target = next(d for d in drones if d.id == target_id)
                success = transmit_packet(drone, target)
                if not success:
                    path = route_discovery(drone, target, drones)
                    if path:
                        print(f"  Route found: {path}")
                    else:
                        print(f"  No route available between {drone.id} and {target.id}")
                        drone.buffer.append((target.id, "data_packet"))

        # Rediscover neighbors dynamically after movement
        for d in drones:
            d.discover_neighbors(drones)

        # Display network and energy info
        for d in drones:
            print(d, "| Neighbors:", d.neighbors)

# ==========================================================
# Example Usage (Static + Formation Example)
# ==========================================================
if __name__ == "__main__":
    N = 12  # scalable to 100+
    drones = []

    for i in range(N):
        drones.append(Drone(i + 1, i * 20, 0, "802.11ac"))

    assign_formation(drones, formation="v", spacing=50)
    run_simulation(drones, steps=12, dt=1.0)

# ==========================================================
# Future / To-Do Notes (kept and extended)
# ==========================================================
# sleep(): have sleep only receive data from main drone
#
# Tree Topology:
#   - Show battery consumption per link
#   - Display which drone is connected to whom
#   - Display distance with main drone and children
#
# Network:
#   - Study routing options (AODV/DSR reactive)
#   - Support recovery from disconnections and rediscovery (reactive)
#   - Implement buffering for disconnection tolerance (drop policy)
#
# Formation Control:
#   - formations: grid / square / V / line
#   - leader-follower structure with formation offsets
#   - speed, velocity (vx, vy), and global speed scaling
#   - role (leader/follower)
#   - neighbors, parent_id, children links
#
# Added Features:
#   - Neighbor discovery (dynamic link awareness)
#   - Basic AODV-like route discovery and caching
#   - Energy-aware sleep states
#   - Buffering for data loss tolerance
#   - Extendable PHY model for range, path loss, or interference
#   - Basic mobility and formation control
#
# Next Steps (Recommended):
#   - Add dynamic leader handoff
#   - Integrate control laws for formation stability
#   - ability to handle 100 drones / choose the amount needed.
#
# New Next Step:
#   - Add dynamic link-quality estimator (RSSI/SNR model)
