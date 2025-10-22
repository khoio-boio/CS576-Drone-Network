
from .phy import Drone
from .mobility import Mobility2D, RandomWaypoint2D, Formation2D, step_formation2d, v_offsets, line_offsets, grid_offsets

def run_sim_2d():
    # Build a small swarm
    drones = [
        Drone(1,  0,  0, "802.11ac"),
        Drone(2, 10,  0, "802.11ac"),
        Drone(3, 20,  0, "802.11ac"),
        Drone(4, 30,  0, "802.11ac"),
        Drone(5, 40,  0, "802.11ac"),
    ]

    ctrls = {d.id: Mobility2D(d, max_speed=6.0) for d in drones}

    # Random Waypoint until t=300s
    rwp = RandomWaypoint2D(area=(200, 200), pause_range=(0.0, 0.5), seed=42)

    # Then switch to V-formation with drone 1 as leader
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
            wp = (180.0, 100.0)  # leader waypoint
            step_formation2d(drones, ctrls, fm, dt, leader_waypoint=wp, leader_speed=5.5)

        if int(t) % 10 == 0 and abs(t - round(t)) < 1e-9:
            pos = [ (round(d.position[0],1), round(d.position[1],1)) for d in drones ]
            e   = ", ".join([f"{d.id}:{d.energy:.0f}mWh" for d in drones])
            print(f"t={t:5.1f}s  pos={pos}  energy[{e}]")

        t += dt

if __name__ == "__main__":
    run_sim_2d()
