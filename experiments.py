"""
Experiments for CS576 Drone Network Project
E1: Mobility vs Latency
E2: Energy–Throughput tradeoff
E3: Formation transition
"""

import json
import csv
import os
import math
from typing import List, Dict, Tuple
import random

try:
    from src.drone_sim.simulator import IntegratedSimulator
    from src.drone_sim.phy import Drone
    from src.drone_sim.mobility import RandomWaypoint2D, Formation2D, step_formation2d, v_offsets, Mobility2D
    from src.drone_sim.metrics import MetricsCollector
except ImportError:
    from drone_sim.simulator import IntegratedSimulator
    from drone_sim.phy import Drone
    from drone_sim.mobility import RandomWaypoint2D, Formation2D, step_formation2d, v_offsets, Mobility2D
    from drone_sim.metrics import MetricsCollector


def run_experiment_e1(output_dir: str = "results/e1"):
    """
    E1: Mobility vs Latency
    Compare formation vs random waypoint at different node counts: N = {5, 25, 100}
    """
    print("=" * 60)
    print("Experiment E1: Mobility vs Latency")
    print("=" * 60)
    
    os.makedirs(output_dir, exist_ok=True)
    
    node_counts = [5, 25, 100]
    mobility_models = ['formation', 'random_waypoint']
    area = (500.0, 500.0)
    duration = 300.0  # 5 minutes
    dt = 0.1
    
    results = []
    
    for n in node_counts:
        print(f"\n--- Testing with N={n} nodes ---")
        
        for mobility in mobility_models:
            print(f"  Mobility model: {mobility}")
            
            # Create simulator
            sim = IntegratedSimulator(
                num_nodes=n,
                area=area,
                comm_profile="802.11ac",
                initial_energy=5000.0,
                mac_config={'queue_capacity': 50, 'beacon_interval': 1.0},
                seed=42
            )
            
            # Setup mobility
            if mobility == 'formation':
                # V-formation
                leader_id = 1
                followers = list(range(2, n + 1))
                fm = Formation2D(leader_id, v_offsets(followers, spacing=15.0, angle_deg=30.0))
                rwp = None
            else:
                # Random waypoint
                rwp = RandomWaypoint2D(area=area, pause_range=(0.0, 2.0), seed=42)
                fm = None
            
            # Add traffic flows (leader to followers)
            if mobility == 'formation':
                # Leader sends to all followers
                for follower_id in range(2, min(n + 1, 6)):  # Limit to 5 flows for scalability
                    sim.add_traffic_flow(1, follower_id, packet_rate=2.0, start_time=10.0)
            else:
                # Random pairs
                pairs = [(i, (i % n) + 1) for i in range(1, min(n + 1, 6))]
                for src, dst in pairs:
                    if src != dst:
                        sim.add_traffic_flow(src, dst, packet_rate=2.0, start_time=10.0)
            
            # Run simulation with mobility updates
            steps = int(duration / dt)
            for step in range(steps):
                t = step * dt
                
                # Update mobility
                if mobility == 'formation' and fm:
                    # Formation movement
                    leader = next(d for d in sim.drones if d.id == 1)
                    leader_waypoint = (250.0 + 100.0 * math.sin(t / 50.0), 250.0 + 100.0 * math.cos(t / 50.0))
                    step_formation2d(sim.drones, sim.mobility_controllers, fm, dt, 
                                   leader_waypoint=leader_waypoint, leader_speed=5.0)
                elif mobility == 'random_waypoint' and rwp:
                    # Random waypoint movement
                    for drone in sim.drones:
                        rwp.apply(drone, sim.mobility_controllers[drone.id], dt)
                        sim.mobility_controllers[drone.id].step(dt)
                
                # Simulation step
                sim.step(dt)
                
                # Progress indicator
                if step % 100 == 0:
                    pdr = sim.metrics.get_pdr()
                    latency = sim.metrics.get_avg_latency()
                    print(f"    t={t:.1f}s: PDR={pdr:.3f}, Latency={latency:.3f}s")
            
            # Collect results
            summary = sim.get_summary()
            metrics = summary['metrics']
            
            result = {
                'node_count': n,
                'mobility_model': mobility,
                'pdr': metrics['pdr'],
                'avg_latency': metrics['avg_latency'],
                'total_sent': metrics['total_sent'],
                'total_received': metrics['total_received'],
                'route_churn': metrics['route_churn'],
                'avg_energy': summary['avg_energy'],
                'active_nodes': summary['active_nodes']
            }
            results.append(result)
            
            print(f"    Final PDR: {metrics['pdr']:.3f}")
            print(f"    Final Latency: {metrics['avg_latency']:.3f}s")
    
    # Save results
    csv_path = os.path.join(output_dir, "e1_results.csv")
    with open(csv_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=results[0].keys())
        writer.writeheader()
        writer.writerows(results)
    
    print(f"\nResults saved to {csv_path}")
    return results


def run_experiment_e2(output_dir: str = "results/e2"):
    """
    E2: Energy–Throughput tradeoff
    Test different TX power levels vs lifetime/PDR
    """
    print("=" * 60)
    print("Experiment E2: Energy–Throughput Tradeoff")
    print("=" * 60)
    
    os.makedirs(output_dir, exist_ok=True)
    
    # TX power levels (as multipliers of base power)
    tx_power_levels = [0.5, 0.75, 1.0, 1.25, 1.5]
    num_nodes = 25
    area = (500.0, 500.0)
    duration = 600.0  # 10 minutes
    dt = 0.1
    
    results = []
    
    for tx_mult in tx_power_levels:
        print(f"\n--- Testing TX power multiplier: {tx_mult}x ---")
        
        # Create simulator
        sim = IntegratedSimulator(
            num_nodes=num_nodes,
            area=area,
            comm_profile="802.11ac",
            initial_energy=5000.0,
            mac_config={'queue_capacity': 50, 'beacon_interval': 1.0},
            seed=42
        )
        
        # Adjust TX power for all drones
        base_power = sim.drones[0].tx_power
        for drone in sim.drones:
            drone.tx_power = base_power * tx_mult
            # Update range based on power (simplified model)
            profile = sim.drones[0].comm_profile
            from src.drone_sim.phy import wifi_profiles
            base_range = wifi_profiles[profile]["range"]
            # Range scales with sqrt of power (simplified)
            new_range = base_range * math.sqrt(tx_mult)
            # Update neighbor discovery range (approximate)
            # Note: This is a simplified model
        
        # Setup random waypoint mobility
        rwp = RandomWaypoint2D(area=area, pause_range=(0.0, 2.0), seed=42)
        
        # Add traffic flows
        for i in range(1, min(num_nodes + 1, 11)):
            dst = ((i * 3) % num_nodes) + 1
            if dst != i:
                sim.add_traffic_flow(i, dst, packet_rate=1.0, start_time=10.0)
        
        # Track energy over time
        energy_history = []
        pdr_history = []
        
        # Run simulation
        steps = int(duration / dt)
        for step in range(steps):
            t = step * dt
            
            # Update mobility
            for drone in sim.drones:
                rwp.apply(drone, sim.mobility_controllers[drone.id], dt)
                sim.mobility_controllers[drone.id].step(dt)
            
            # Simulation step
            sim.step(dt)
            
            # Record metrics periodically
            if step % 100 == 0:
                avg_energy = sum(d.energy for d in sim.drones if d.online) / max(sum(1 for d in sim.drones if d.online), 1)
                pdr = sim.metrics.get_pdr()
                energy_history.append((t, avg_energy))
                pdr_history.append((t, pdr))
                
                active = sum(1 for d in sim.drones if d.online)
                print(f"    t={t:.1f}s: Active={active}/{num_nodes}, Avg Energy={avg_energy:.1f}mWh, PDR={pdr:.3f}")
                
                # Check if all nodes dead
                if active == 0:
                    print(f"    All nodes depleted at t={t:.1f}s")
                    break
        
        # Calculate lifetime (time until 50% nodes dead)
        lifetime = duration
        for t, _ in energy_history:
            active = sum(1 for d in sim.drones if d.online)
            if active < num_nodes * 0.5:
                lifetime = t
                break
        
        # Final metrics
        summary = sim.get_summary()
        metrics = summary['metrics']
        
        result = {
            'tx_power_multiplier': tx_mult,
            'pdr': metrics['pdr'],
            'lifetime': lifetime,
            'throughput_bytes': metrics['throughput_bytes'],
            'total_sent': metrics['total_sent'],
            'total_received': metrics['total_received'],
            'final_avg_energy': summary['avg_energy'],
            'final_active_nodes': summary['active_nodes']
        }
        results.append(result)
        
        print(f"    Final PDR: {metrics['pdr']:.3f}")
        print(f"    Lifetime: {lifetime:.1f}s")
    
    # Save results
    csv_path = os.path.join(output_dir, "e2_results.csv")
    with open(csv_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=results[0].keys())
        writer.writeheader()
        writer.writerows(results)
    
    print(f"\nResults saved to {csv_path}")
    return results


def run_experiment_e3(output_dir: str = "results/e3"):
    """
    E3: Formation transition
    Measure KPIs before/during/after transition; route churn; time-to-restore
    """
    print("=" * 60)
    print("Experiment E3: Formation Transition")
    print("=" * 60)
    
    os.makedirs(output_dir, exist_ok=True)
    
    num_nodes = 25
    area = (500.0, 500.0)
    duration = 600.0  # 10 minutes
    transition_time = 300.0  # Transition at 5 minutes
    dt = 0.1
    
    # Create simulator
    sim = IntegratedSimulator(
        num_nodes=num_nodes,
        area=area,
        comm_profile="802.11ac",
        initial_energy=5000.0,
        mac_config={'queue_capacity': 50, 'beacon_interval': 1.0},
        seed=42
    )
    
    # Setup initial formation (V-formation)
    leader_id = 1
    followers = list(range(2, num_nodes + 1))
    fm = Formation2D(leader_id, v_offsets(followers, spacing=15.0, angle_deg=30.0))
    
    # Add traffic flows
    for follower_id in range(2, min(num_nodes + 1, 11)):
        sim.add_traffic_flow(1, follower_id, packet_rate=2.0, start_time=10.0)
    
    # Set transition time in metrics
    sim.metrics.set_transition_time(transition_time)
    
    # Track metrics over time
    metrics_history = []
    route_churn_history = []
    
    transition_occurred = False
    new_formation = None
    
    # Run simulation
    steps = int(duration / dt)
    for step in range(steps):
        t = step * dt
        
        # Trigger formation transition
        if t >= transition_time and not transition_occurred:
            print(f"\n[TRANSITION] t={t:.1f}s - Switching formation")
            transition_occurred = True
            
            # Switch to new formation (line formation)
            new_offsets = line_offsets(followers, spacing=20.0)
            new_formation = Formation2D(leader_id, new_offsets)
            fm = new_formation
        
        # Update mobility
        if t < transition_time:
            # Original V-formation
            leader_waypoint = (250.0 + 100.0 * math.sin(t / 50.0), 250.0 + 100.0 * math.cos(t / 50.0))
            step_formation2d(sim.drones, sim.mobility_controllers, fm, dt,
                           leader_waypoint=leader_waypoint, leader_speed=5.0)
        else:
            # New formation after transition
            if new_formation:
                leader_waypoint = (250.0 + 100.0 * math.sin(t / 50.0), 250.0 + 100.0 * math.cos(t / 50.0))
                step_formation2d(sim.drones, sim.mobility_controllers, new_formation, dt,
                               leader_waypoint=leader_waypoint, leader_speed=5.0)
        
        # Simulation step
        sim.step(dt)
        
        # Record metrics
        if step % 50 == 0:
            pdr = sim.metrics.get_pdr()
            latency = sim.metrics.get_avg_latency()
            route_churn = sim.metrics.get_route_churn()
            active = sum(1 for d in sim.drones if d.online)
            
            phase = "before" if t < transition_time - 30 else "during" if t < transition_time + 30 else "after"
            
            metrics_history.append({
                'time': t,
                'phase': phase,
                'pdr': pdr,
                'latency': latency,
                'route_churn': route_churn,
                'active_nodes': active
            })
            
            route_churn_history.append((t, route_churn))
            
            print(f"    t={t:.1f}s [{phase}]: PDR={pdr:.3f}, Latency={latency:.3f}s, Route Churn={route_churn}")
    
    # Calculate time-to-restore (time until PDR returns to pre-transition level)
    before_pdr = sim.metrics.get_pdr_before_transition()
    time_to_restore = None
    
    for entry in metrics_history:
        if entry['time'] > transition_time and entry['pdr'] >= before_pdr * 0.95:
            time_to_restore = entry['time'] - transition_time
            break
    
    if time_to_restore is None:
        time_to_restore = duration - transition_time
    
    # Calculate metrics by phase
    before_metrics = [m for m in metrics_history if m['phase'] == 'before']
    during_metrics = [m for m in metrics_history if m['phase'] == 'during']
    after_metrics = [m for m in metrics_history if m['phase'] == 'after']
    
    def avg_metric(metrics_list, key):
        if not metrics_list:
            return 0.0
        return sum(m[key] for m in metrics_list) / len(metrics_list)
    
    # Final summary
    summary = sim.get_summary()
    final_metrics = summary['metrics']
    
    results = {
        'before_transition': {
            'avg_pdr': avg_metric(before_metrics, 'pdr'),
            'avg_latency': avg_metric(before_metrics, 'latency'),
            'route_churn': sum(m['route_churn'] for m in before_metrics)
        },
        'during_transition': {
            'avg_pdr': avg_metric(during_metrics, 'pdr'),
            'avg_latency': avg_metric(during_metrics, 'latency'),
            'route_churn': sum(m['route_churn'] for m in during_metrics)
        },
        'after_transition': {
            'avg_pdr': avg_metric(after_metrics, 'pdr'),
            'avg_latency': avg_metric(after_metrics, 'latency'),
            'route_churn': sum(m['route_churn'] for m in after_metrics)
        },
        'time_to_restore': time_to_restore,
        'total_route_churn': final_metrics['route_churn'],
        'final_pdr': final_metrics['pdr']
    }
    
    # Save detailed history
    csv_path = os.path.join(output_dir, "e3_history.csv")
    with open(csv_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=metrics_history[0].keys())
        writer.writeheader()
        writer.writerows(metrics_history)
    
    # Save summary
    summary_path = os.path.join(output_dir, "e3_summary.json")
    with open(summary_path, 'w') as f:
        json.dump(results, f, indent=2)
    
    print(f"\nResults saved to {csv_path} and {summary_path}")
    print(f"Time to restore: {time_to_restore:.1f}s")
    print(f"Before PDR: {results['before_transition']['avg_pdr']:.3f}")
    print(f"During PDR: {results['during_transition']['avg_pdr']:.3f}")
    print(f"After PDR: {results['after_transition']['avg_pdr']:.3f}")
    
    return results


if __name__ == "__main__":
    import math
    
    print("Running all experiments...")
    print("\n")
    
    # Run E1
    e1_results = run_experiment_e1()
    
    print("\n")
    
    # Run E2
    e2_results = run_experiment_e2()
    
    print("\n")
    
    # Run E3
    e3_results = run_experiment_e3()
    
    print("\n" + "=" * 60)
    print("All experiments completed!")
    print("=" * 60)

