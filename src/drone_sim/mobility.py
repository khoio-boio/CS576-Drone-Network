import math, random
from typing import Dict, List, Optional, Tuple

try:
    from .phy import Drone, power_profile         
except ImportError:
    from drone_sim.phy import Drone, power_profile 


Vector = Tuple[float, float]


def _norm(x: float, y: float) -> float:
    return math.hypot(x,y)

def _unit(x: float, y: float) -> Vector:
    distance = _norm(x,y)
    return (0.0, 0.0) if distance == 0.0 else (x/distance, y/distance)

def _rotate(x: float, y: float, degrees: float) -> Vector:
    rad = math.radians(degrees)
    cos = math.cos(rad)
    sin = math.sin(rad)
    return (cos*x - sin*y, sin*x + cos*y)

# Simple2D kinematics with speed and acceleration caps and motion energy draw
class Mobility2D:    
    def __init__(self, drone: Drone, max_speed: float = 7.0, max_accel: float = 3.5, motion_mW_per_m: float = 60.0):
        self.d = drone
        self.vx, self.vy = 0.0, 0.0
        self.max_speed = max_speed
        self.max_accel = max_accel
        self.motion_mW_per_m = motion_mW_per_m
        self.target: Optional[Vector] = None

    def set_target(self, xy: Vector):
        self.target = xy

    def step(self, dt: float):
        x, y = self.d.position
        vx, vy = self.vx, self.vy

        # Move toward target if set
        if self.target is not None:
            tx, ty = self.target
            dx, dy = tx - x, ty - y
            dist = _norm(dx, dy)

            if dist < 0.2:  # arrive
                self.vx, self.vy = 0.0, 0.0
                return

            ux, uy = _unit(dx, dy)
            desired_speed = min(self.max_speed, dist / max(dt, 1e-3))
            ax, ay = ux*desired_speed - vx, uy*desired_speed - vy

            # clamp acceleration
            amag = _norm(ax, ay)
            if amag > self.max_accel:
                ax, ay = _unit(ax, ay)
                ax, ay = ax*self.max_accel, ay*self.max_accel

            vx += ax
            vy += ay

        # clamp speed
        sp = _norm(vx, vy)
        if sp > self.max_speed:
            ux, uy = _unit(vx, vy)
            vx, vy = ux*self.max_speed, uy*self.max_speed

        # integrate
        nx, ny = x + vx*dt, y + vy*dt
        self.vx, self.vy = vx, vy
        self.d.position = (nx, ny)

        # simple motion energy draw (mW per meter → mWh for this step)
        dist = sp * dt
        motion_mWh = (self.motion_mW_per_m * dist) / 3600.0
        self.d.energy = max(self.d.energy - motion_mWh, 0.0)
        self.d.state = "move"


class RandomWaypoint2D:
    def __init__(self, area: Vector = (200.0, 200.0), min_speed=2.0, max_speed=7.0,
                 pause_range: Tuple[float, float] = (0.0, 1.0), seed: Optional[int] = None):
        self.W, self.H = area
        self.min_speed, self.max_speed = min_speed, max_speed
        self.pause_range = pause_range
        self.rng = random.Random(seed)
        self.pause_left: Dict[int, float] = {}

    def _new_point(self) -> Vector:
        return (self.rng.uniform(0, self.W), self.rng.uniform(0, self.H))

    def apply(self, d: Drone, ctrl: Mobility2D, dt: float):
        # handle pause timer
        pid = d.id
        p = self.pause_left.get(pid, 0.0)
        if p > 0:
            self.pause_left[pid] = max(0.0, p - dt)
            ctrl.set_target(d.position)  # stay put
            return

        # pick new waypoint if none or arrived
        if ctrl.target is None or _norm(ctrl.target[0]-d.position[0], ctrl.target[1]-d.position[1]) < 0.5:
            tgt = self._new_point()
            ctrl.set_target(tgt)
            # random cruise speed (we clamp via max_speed)
            ctrl.max_speed = self.rng.uniform(self.min_speed, self.max_speed)
            self.pause_left[pid] = self.rng.uniform(*self.pause_range)


def line_offsets(ids: List[int], spacing: float = 8.0) -> Dict[int, Vector]:
    # leader is at (0,0); followers at (-spacing,0), (-2*spacing,0), ...
    return {did: (-i*spacing, 0.0) for i, did in enumerate(ids, start=1)}

def v_offsets(ids: List[int], spacing: float = 8.0, angle_deg: float = 28.0) -> Dict[int, Vector]:
    offs: Dict[int, Vector] = {}
    left = True
    for k, did in enumerate(ids, start=1):
        r = k * spacing
        ax = -r * math.cos(math.radians(angle_deg))
        ay =  r * math.sin(math.radians(angle_deg)) * (1 if left else -1)
        offs[did] = (ax, ay)
        left = not left
    return offs

def grid_offsets(ids: List[int], cols: int, spacing: float = 8.0) -> Dict[int, Vector]:
    offs: Dict[int, Vector] = {}
    for idx, did in enumerate(ids):
        row, col = divmod(idx, cols)
        offs[did] = (-(row+1)*spacing, (col-(cols-1)/2)*spacing)
    return offs

class Formation2D:
    def __init__(self, leader_id: int, offsets: Dict[int, Vector]):
        self.leader_id = leader_id
        self.offsets = offsets
        self.heading_deg = 0.0  # leader heading

    def update_heading_from(self, vx: float, vy: float):
        if _norm(vx, vy) > 1e-6:
            self.heading_deg = math.degrees(math.atan2(vy, vx))

    def target_for(self, leader_pos: Vector, follower_id: int) -> Optional[Vector]:
        if follower_id not in self.offsets:
            return None
        ox, oy = self.offsets[follower_id]
        rx, ry = _rotate(ox, oy, self.heading_deg)
        return (leader_pos[0] + rx, leader_pos[1] + ry)

def step_formation2d(
    drones: List[Drone],
    ctrls: Dict[int, Mobility2D],
    fm: Formation2D,
    dt: float,
    leader_waypoint: Optional[Vector] = None,
    leader_speed: float = 5.0,
):
    # leader
    leader = next(d for d in drones if d.id == fm.leader_id)
    lc = ctrls[leader.id]
    if leader_waypoint:
        lc.set_target(leader_waypoint)
    lc.max_speed = leader_speed
    lc.step(dt)
    fm.update_heading_from(lc.vx, lc.vy)

    # followers
    for d in drones:
        if d.id == fm.leader_id:
            continue
        t = fm.target_for(leader.position, d.id)
        if t:
            ctrls[d.id].set_target(t)
            ctrls[d.id].step(dt)

