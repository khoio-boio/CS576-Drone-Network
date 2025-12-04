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
def dynamic_leader_handoff(drones, battery_threshold=300, min_neighbors=2, hysteresis=50):
    """Select a new leader if current one is weak or disconnected."""
    leader = drones[0]
    # Conditions for handoff: low energy or poor connectivity
    needs_handoff = (
        (leader.energy <= battery_threshold) or
        (len(leader.neighbors) < min_neighbors)
    )

    # Hysteresis: avoid flapping near threshold
    if not needs_handoff and leader.energy > (battery_threshold + hysteresis):
        return

    print("\n[Leader Handoff Triggered] Leader weak/disconnected")

    # Candidates: online, reasonably connected
    candidates = [d for d in drones if d.online]
    if not candidates:
        print("[Leader Handoff] No candidates available")
        return

    # Score: energy, connectivity, centrality (distance to centroid)
    cx = sum(d.position[0] for d in candidates) / len(candidates)
    cy = sum(d.position[1] for d in candidates) / len(candidates)

    def score(d):
        return (
            0.6 * d.energy +                    # prioritize battery
            0.3 * len(d.neighbors) -            # prioritize connectivity
            0.1 * math.hypot(d.position[0] - cx, d.position[1] - cy)  # centrality
        )

    candidates.sort(key=score, reverse=True)
    new_leader = candidates[0]

    if new_leader.id == leader.id:
        return  # same leader

    # Swap positions in the list to keep drones[0] as the leader
    idx_old = drones.index(leader)
    idx_new = drones.index(new_leader)
    drones[idx_old], drones[idx_new] = drones[idx_new], drones[idx_old]

    # Update roles
    drones[0].role = "leader"
    for d in drones[1:]:
        d.role = "follower"

    print(f"[Leader Handoff] New leader is Drone {drones[0].id}")

# ==========================================================
# Formation Control Laws (PD + consensus + avoidance)
# ==========================================================
def formation_control_update(drones, dt, kp=0.25, kd=0.40, vmax=3.0, avoid_gain=0.5, avoid_radius=20.0, consensus_weight=0.2):
    """Update follower velocities to maintain stable formation relative to leader."""
    if not drones or not drones[0].online:
        return
    leader = drones[0]

    # Leader velocity can be set externally (mission plan)
    # Example: constant forward motion
    # leader.velocity is assumed already set before calling this function

    for d in drones[1:]:
        if not d.online:
            continue

        # Desired position relative to leader
        dx_off, dy_off = d.formation_offset
        target_x = leader.position[0] - dx_off
        target_y = leader.position[1] + dy_off

        # Errors
        ex = target_x - d.position[0]
        ey = target_y - d.position[1]

        # PD control (damping uses drone's current velocity)
        ux = kp * ex - kd * d.velocity[0]
        uy = kp * ey - kd * d.velocity[1]

        # Consensus term: align with average of neighbors to reduce drift
        if d.neighbors:
            nx = 0.0
            ny = 0.0
            count = 0
            for nid in d.neighbors:
                n = next(dr for dr in drones if dr.id == nid)
                nx += n.position[0]
                ny += n.position[1]
                count += 1
            nx /= count
            ny /= count
            # Pull slightly toward neighbor centroid
            ux += consensus_weight * (nx - d.position[0])
            uy += consensus_weight * (ny - d.position[1])

        # Collision avoidance: repulsion from too-close neighbors
        for nid in d.neighbors:
            n = next(dr for dr in drones if dr.id == nid)
            r = math.hypot(n.position[0] - d.position[0], n.position[1] - d.position[1])
            if r < avoid_radius and r > 1e-6:
                rx = (d.position[0] - n.position[0]) / r
                ry = (d.position[1] - n.position[1]) / r
                ux += avoid_gain * rx * (avoid_radius - r) / avoid_radius
                uy += avoid_gain * ry * (avoid_radius - r) / avoid_radius

        # Velocity saturation for stability and energy efficiency
        speed = math.hypot(ux, uy)
        if speed > vmax:
            ux *= vmax / speed
            uy *= vmax / speed

        d.velocity = [ux, uy]


# ==========================================================
# Network Simulation Loop
# ==========================================================

def run_simulation(drones, steps=10, dt=1.0):
    # Initial neighbor discovery
    for d in drones:
        d.discover_neighbors(drones)

    for step in range(steps):
        print(f"\n--- Step {step + 1} ---")

        # Leader mission plan (you can vary over time)
        leader = drones[0]
        leader.velocity = [1.5, 0.0]  # forward motion
        leader.move(dt)

        # Refresh neighbors before control (needed for consensus/avoidance)
        for d in drones:
            d.discover_neighbors(drones)

        # Formation control (stable PD + consensus + avoidance)
        formation_control_update(drones, dt)

        # Move followers after updating control velocities
        for d in drones[1:]:
            d.move(dt)

        # Periodic leader handoff (e.g., every step; you can do every N steps)
        dynamic_leader_handoff(drones, battery_threshold=300, min_neighbors=2, hysteresis=50)

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
    N = int(input("Enter number of drones: ")) # whole numbers only max 100
    formation = input("Enter formation (line, v, grid, square): ") # respond with word as shown
    spacing = int(input("Enter spacing between drones: ")) # whole numbers only

    drones = [Drone(i + 1, i * 20, 0, "802.11ac") for i in range(N)]
    assign_formation(drones, formation=formation, spacing=spacing)
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
#   - Can chose the formation and amount of drones.
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
#
# New Next Step:
#   - Add dynamic link-quality estimator (RSSI/SNR model)

