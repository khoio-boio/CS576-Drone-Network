import random
import math

# Define Wi-Fi tech profiles
wifi_profiles = {
    "802.11n":     {"range": 100, "max_rate": 54,  "tx_power": 15},
    "802.11ac":    {"range": 200, "max_rate": 433, "tx_power": 20},
    "wifi_direct": {"range": 100, "max_rate": 250, "tx_power": 15},
}

# Power consumption per state in mW
power_profile = {
    "tx": 1500,    # Transmit
    "rx": 900,     # Receive
    "idle": 250,   # Listening but not receiving
    "sleep": 20,   # Low power state
}

# Basic drone structure
class Drone:
    def __init__(self, drone_id, x, y, comm_profile, energy=5000):
        self.id = drone_id
        self.position = (x, y)
        self.comm_profile = comm_profile
        self.energy = energy  # in mWh
        self.state = "idle"
        self.tx_power = wifi_profiles[comm_profile]["tx_power"]
    
    def distance_to(self, other):
        x1, y1 = self.position
        x2, y2 = other.position
        return math.hypot(x2 - x1, y2 - y1)
    
    def update_energy(self, state, duration_s):
        power_mW = power_profile.get(state, 0)
        energy_used = power_mW * duration_s / 3600  # convert to mWh
        self.energy = max(self.energy - energy_used, 0)
        self.state = state

# Simulate a lossy channel
def simulate_packet_loss(distance, max_range, base_loss=0.1):
    if distance > max_range:
        return True  # Always lost if out of range
    # Probability increases with distance
    loss_prob = base_loss + 0.4 * (distance / max_range)
    return random.random() < loss_prob

# Simulate a transmission between two drones
def transmit_packet(sender, receiver, duration_s=0.01):
    profile = wifi_profiles[sender.comm_profile]
    distance = sender.distance_to(receiver)
    max_range = profile["range"]

    lost = simulate_packet_loss(distance, max_range)
    success = not lost

    sender.update_energy("tx", duration_s)
    receiver.update_energy("rx" if success else "idle", duration_s)

    print(f"[TX] Drone {sender.id} -> Drone {receiver.id} | Distance: {int(distance)}m | Success: {success}")
    return success

# Simulation loop
def run_simulation(drones, steps=10):
    for step in range(steps):
        print(f"\n--- Step {step + 1} ---")
        for i in range(len(drones)):
            for j in range(i + 1, len(drones)):
                transmit_packet(drones[i], drones[j])

        for drone in drones:
            print(f"Drone {drone.id} Energy: {drone.energy:.2f} mWh | State: {drone.state}")

# Example usage
if __name__ == "__main__":
    drone1 = Drone(1, 0, 0, "802.11ac")
    drone2 = Drone(2, 50, 0, "802.11ac")
    drone3 = Drone(3, 150, 0, "802.11ac")

    drones = [drone1, drone2, drone3]
    run_simulation(drones, steps=5)
    
def sleep():
    # have sleep only recive data from main drone


# tree Topology: battery comsumption, what drone is conected with who
# display distance with main drone and childern
# formations: grid / square formation, V fomation, line formaiton, optional possible shapes

