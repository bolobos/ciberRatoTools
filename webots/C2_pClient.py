"""sample2 controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

# create the Robot instance.
robot = Robot()

black_rgb_threshold = 120  # For the camera (RGB < 120)

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
gps = robot.getDevice('gps')
gps.enable(timestep)

camera = robot.getDevice('camera')
camera.enable(timestep)

compass = robot.getDevice('compass')
compass.enable(timestep)


def analyze_borders(image, width, height):
    center_x = int(width / 2)
    start_y = max(height - 2, 21)

    for y in range(start_y, 20, -1):
        # Read the 3 central pixels in the vertical column
        rgb_center_3x3 = []

        for offset_y in (-1, 0, 1):
            check_y = y + offset_y
            if 0 <= check_y < height:
                r_val = camera.imageGetRed(image, width, center_x, check_y)
                g_val = camera.imageGetGreen(image, width, center_x, check_y)
                b_val = camera.imageGetBlue(image, width, center_x, check_y)
                rgb_center_3x3.append([r_val, g_val, b_val])
            else:
                rgb_center_3x3.append([0, 0, 0])

        # Ignore rows where the center path is not black
        if (
            rgb_center_3x3[1][0] > black_rgb_threshold
            or rgb_center_3x3[1][1] > black_rgb_threshold
            or rgb_center_3x3[1][2] > black_rgb_threshold
        ) or (
            rgb_center_3x3[0][0] > black_rgb_threshold
            or rgb_center_3x3[0][1] > black_rgb_threshold
            or rgb_center_3x3[0][2] > black_rgb_threshold
        ) or (
            rgb_center_3x3[2][0] > black_rgb_threshold
            or rgb_center_3x3[2][1] > black_rgb_threshold
            or rgb_center_3x3[2][2] > black_rgb_threshold
        ):
            continue

        left_detected = False
        right_detected = False
        straight_detected = False

        # -------------------------------------------------------------
        # LEFT SIDE SCAN
        # -------------------------------------------------------------
        for x in range(center_x, 0, -1):
            r = camera.imageGetRed(image, width, x, y)
            g = camera.imageGetGreen(image, width, x, y)
            b = camera.imageGetBlue(image, width, x, y)

            is_not_black = (r > black_rgb_threshold or g > black_rgb_threshold or b > black_rgb_threshold)

            if is_not_black:
                # Yellow border
                if r > 150 and g > 150 and b < 100:
                    left_detected = True
                    break

                # White line: stop scanning this side
                elif r > 180 and g > 180 and b > 180:
                    break

                # Green border
                elif g > 150 and r < 100:
                    left_detected = True
                    break

        # -------------------------------------------------------------
        # RIGHT SIDE SCAN
        # -------------------------------------------------------------
        for x in range(center_x, width):
            r = camera.imageGetRed(image, width, x, y)
            g = camera.imageGetGreen(image, width, x, y)
            b = camera.imageGetBlue(image, width, x, y)

            is_not_black = (r > black_rgb_threshold or g > black_rgb_threshold or b > black_rgb_threshold)

            if is_not_black:
                # Yellow border
                if r > 150 and g > 150 and b < 100:
                    right_detected = True
                    break

                # White line: stop scanning this side
                elif r > 180 and g > 180 and b > 180:
                    break

                # Green border
                elif g > 150 and r < 100:
                    right_detected = True
                    break

        # -------------------------------------------------------------
        # STRAIGHT CHECK
        # Detect whether the center remains free (straight is possible)
        # -------------------------------------------------------------
        straight_clear = True
        for offset_x in range(-6, 7):
            scan_x = center_x + offset_x
            if 0 <= scan_x < width:
                r = camera.imageGetRed(image, width, scan_x, y)
                g = camera.imageGetGreen(image, width, scan_x, y)
                b = camera.imageGetBlue(image, width, scan_x, y)

                if r > black_rgb_threshold or g > black_rgb_threshold or b > black_rgb_threshold:
                    straight_clear = False
                    break

        if straight_clear:
            straight_detected = True

        # -------------------------------------------------------------
        # Final classification
        # -------------------------------------------------------------
        if left_detected and right_detected:
            return "LEFT_RIGHT"
        elif left_detected and straight_detected:
            return "LEFT_STRAIGHT"
        elif right_detected and straight_detected:
            return "RIGHT_STRAIGHT"
        elif left_detected:
            return "LEFT_TURN"
        elif right_detected:
            return "RIGHT_TURN"
        elif straight_detected:
            return "STRAIGHT"

    return "NONE"


# ==========================================================================
# ADDED: explore the maze, write mapping.out and return to the start
#
# C2 controller: explore the line maze, map it and return to the start.
#
# The robot only uses its own sensors:
#   * GPS + compass give the pose. The supervisor shifts the maze by a whole
#     number of cells, so maze nodes always sit on multiples of CELL in the
#     world and the GPS position can be snapped to the node grid.
#   * The camera (tilted towards the floor) is projected onto the ground to
#     check, for every nearby segment of the grid, whether a black line is
#     drawn there. Each frame adds a vote for "line" or "no line".
#   * The ground sensors confirm the segment currently being driven on.
#
# Exploration: always head for the closest node that still has a segment
# we have not seen, using only segments we know exist. The path is followed
# with pure pursuit, so the robot takes corners as smooth arcs instead of
# stopping and turning on the spot (it only spins in place for U-turns).
# When nothing is left to explore it returns to the start node, writes the
# map to mapping.out and stops.
# ==========================================================================


import heapq
import math
import os


# --- Geometry of the maze and of the e-puck -------------------------------
CELL = 0.15                 # distance between maze nodes (m)
GPS_OFFSET = 0.0095         # the GPS sits 9.5 mm behind the robot centre
WHEEL_RADIUS = 0.0205
AXLE_LENGTH = 0.052
MAX_WHEEL_SPEED = 6.27

CAM_X = 0.03                # camera position in the robot frame (m)
CAM_Z = 0.028
CAM_TILT = 0.47             # camera_rotation 0 1 0 0.47 in the world file
DARK_LEVEL = 90             # a pixel is "line" if max(R, G, B) is below this
VIEW_MIN, VIEW_MAX = 0.04, 0.22   # trusted ground distance in front (m)
SAMPLE_POINTS = (0.35, 0.5, 0.65)   # where along a segment we look
VOTES_TO_DECIDE = 2       # votes needed to say "there is a line"
VOTES_TO_REJECT = 4       # votes needed to say "no line here"
MAX_TURN_RATE = 1.0       # ignore the camera while turning faster (rad/s)

GROUND_DARK = 600           # ground sensor reading over a black line

# --- Motion ----------------------------------------------------------------
CRUISE_SPEED = 0.125         # m/s on straight lines
LOOKAHEAD = 0.045           # pure pursuit look-ahead distance (m)
CORNER_SPEED = 0.06         # speed while going round a corner
REPLAN_DIST = 0.07          # replan this far before reaching the next node
STOP_TOLERANCE = 0.008
SPIN_SPEED = 6.0            # wheel speed when turning on the spot (rad/s)

# --- Map output --------------------------------------------------------------
MAP_FILE = "mapping.out"
MAP_COLS, MAP_ROWS = 49, 21

DIRS = [(1, 0), (0, 1), (-1, 0), (0, -1)]   # east, north, west, south


def wrap(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def edge(a, b):
    return (a, b) if a <= b else (b, a)


def node_xy(n):
    return n[0] * CELL, n[1] * CELL


class MazeMap:
    """Evidence about which grid segments carry a black line."""

    def __init__(self):
        self.votes = {}
        self.driven = set()     # segments we have driven along: certain

    def add(self, e, vote):
        if e in self.driven:
            return
        v = self.votes.get(e, 0) + vote
        self.votes[e] = max(-8, min(8, v))

    def confirm(self, e):
        self.votes[e] = 8
        self.driven.add(e)

    def state(self, e):
        v = self.votes.get(e, 0)
        if v >= VOTES_TO_DECIDE:
            return True
        if v <= -VOTES_TO_REJECT:
            return False
        return None

    def neighbours(self, n):
        for d in DIRS:
            m = (n[0] + d[0], n[1] + d[1])
            if self.state(edge(n, m)):
                yield m

    def unknown_dirs(self, n):
        return [d for d in DIRS
                if self.state(edge(n, (n[0] + d[0], n[1] + d[1]))) is None]

    def write(self, start, path=MAP_FILE):
        grid = [[" "] * MAP_COLS for _ in range(MAP_ROWS)]
        cx, cy = MAP_COLS // 2, MAP_ROWS // 2
        for (a, b), v in self.votes.items():
            if v < VOTES_TO_DECIDE:
                continue
            dx, dy = a[0] - start[0], a[1] - start[1]
            if a[1] == b[1]:          # horizontal segment
                row, col, ch = cy + 2 * dy, cx + 2 * dx + 1, "-"
            else:                     # vertical segment
                row, col, ch = cy + 2 * dy + 1, cx + 2 * dx, "|"
            if 0 <= row < MAP_ROWS and 0 <= col < MAP_COLS:
                grid[row][col] = ch
        grid[cy][cx] = "I"
        with open(path, "w") as out:
            for row in reversed(grid):    # top line of the file = north
                out.write("".join(row) + "\n")


class Explorer:
    def __init__(self, robot, gps, compass, camera):
        self.robot = robot
        self.dt_ms = int(self.robot.getBasicTimeStep())
        self.dt = self.dt_ms / 1000.0

        self.gps = gps              # enabled in the sample code above
        self.compass = compass
        self.camera = camera
        self.ground = []
        for name in ("gs0", "gs1", "gs2"):
            sensor = self.robot.getDevice(name)
            if sensor is not None:
                sensor.enable(self.dt_ms)
                self.ground.append(sensor)

        self.left = self.robot.getDevice("left wheel motor")
        self.right = self.robot.getDevice("right wheel motor")
        for motor in (self.left, self.right):
            motor.setPosition(float("inf"))
            motor.setVelocity(0.0)

        self.cam_w = self.camera.getWidth()
        self.cam_h = self.camera.getHeight()
        self.focal = (self.cam_w / 2.0) / math.tan(self.camera.getFov() / 2.0)

        self.final_time = 300.0
        data = self.robot.getCustomData().split()
        if "FinalTime" in data:
            self.final_time = float(data[data.index("FinalTime") + 1])

        self.map = MazeMap()
        self.path = []          # list of nodes, path[0] = node we left
        self.target = 0         # index in path of the next node
        self.mode = "plan"
        self.spin_heading = None
        self.look_dirs = []
        self.going_home = False
        self.frame = 0
        self.started = False
        self.last_write = 0.0

    # --- sensing -------------------------------------------------------------
    def read_pose(self):
        g = self.gps.getValues()
        c = self.compass.getValues()
        th = math.atan2(c[0], c[1])
        self.x = g[0] + GPS_OFFSET * math.cos(th)
        self.y = g[1] + GPS_OFFSET * math.sin(th)
        if hasattr(self, "th"):
            self.turn_rate = abs(wrap(th - self.th)) / self.dt
        else:
            self.turn_rate = 0.0
        self.th = th
        self.node = (int(round(self.x / CELL)), int(round(self.y / CELL)))

    def to_pixel(self, wx, wy):
        """Project a floor point (world frame) into the camera image."""
        dx = wx - self.x
        dy = wy - self.y
        fx = dx * math.cos(self.th) + dy * math.sin(self.th)
        fy = -dx * math.sin(self.th) + dy * math.cos(self.th)
        if not VIEW_MIN <= fx <= VIEW_MAX:
            return None
        s, c = math.sin(CAM_TILT), math.cos(CAM_TILT)
        ax, az = fx - CAM_X, -CAM_Z
        depth = c * ax - s * az
        up = s * ax + c * az
        if depth <= 0.01:
            return None
        u = self.cam_w / 2.0 - self.focal * fy / depth
        v = self.cam_h / 2.0 - self.focal * up / depth
        if not (1 <= u < self.cam_w - 1 and 1 <= v < self.cam_h - 1):
            return None
        return int(u), int(v)

    def look_with_camera(self):
        image = self.camera.getImage()
        if image is None:
            return
        cam, w = self.camera, self.cam_w
        n = self.node
        for i in range(n[0] - 2, n[0] + 3):
            for j in range(n[1] - 2, n[1] + 3):
                for d in ((1, 0), (0, 1)):
                    a, b = (i, j), (i + d[0], j + d[1])
                    # sample a few points along the segment, away from the
                    # crossing lines at its two ends
                    for f in SAMPLE_POINTS:
                        pix = self.to_pixel((a[0] + d[0] * f) * CELL,
                                            (a[1] + d[1] * f) * CELL)
                        if pix is None:
                            continue
                        u, v = pix
                        # darkest pixel of a small cross around the sample,
                        # so thin far-away lines are not missed
                        level = min(
                            max(cam.imageGetRed(image, w, x, y),
                                cam.imageGetGreen(image, w, x, y),
                                cam.imageGetBlue(image, w, x, y))
                            for x, y in ((u, v), (u - 1, v), (u + 1, v),
                                         (u, v - 1), (u, v + 1)))
                        # a black pixel right on a segment is strong evidence,
                        # a bright one may just be a slightly misplaced sample
                        self.map.add(edge(a, b), 2 if level < DARK_LEVEL else -1)

    def track_nodes(self):
        """Remember every segment the robot actually drove along."""
        cx, cy = node_xy(self.node)
        if math.hypot(cx - self.x, cy - self.y) > 0.03:
            return
        last = self.last_node
        if last is not None and abs(last[0] - self.node[0]) + abs(last[1] - self.node[1]) == 1:
            self.map.confirm(edge(last, self.node))
        self.last_node = self.node

    def check_ground(self):
        """Centre ground sensor on black while mid-segment confirms the line."""
        if len(self.ground) < 3:
            return
        fx = self.x + 0.03 * math.cos(self.th)
        fy = self.y + 0.03 * math.sin(self.th)
        gx, gy = fx / CELL, fy / CELL
        on_x = abs(gx - round(gx)) < 0.05
        on_y = abs(gy - round(gy)) < 0.05
        if on_x == on_y:
            return              # near a node or off the grid lines
        if self.ground[1].getValue() > GROUND_DARK:
            return
        if on_x:
            a = (int(round(gx)), int(math.floor(gy)))
            self.map.add(edge(a, (a[0], a[1] + 1)), 1)
        else:
            a = (int(math.floor(gx)), int(round(gy)))
            self.map.add(edge(a, (a[0] + 1, a[1])), 1)

    # --- planning ------------------------------------------------------------
    def plan(self, start, heading, goal_test):
        """Dijkstra over known segments; turning costs a little extra."""
        h0 = int(round(heading / (math.pi / 2))) % 4
        best = {(start, h0): 0.0}
        prev = {(start, h0): None}
        queue = [(0.0, start, h0)]
        while queue:
            cost, n, h = heapq.heappop(queue)
            if cost > best[(n, h)]:
                continue
            if goal_test(n):
                path = []
                state = (n, h)
                while state is not None:
                    path.append(state[0])
                    state = prev[state]
                return list(reversed(path))
            for k, d in enumerate(DIRS):
                m = (n[0] + d[0], n[1] + d[1])
                if not self.map.state(edge(n, m)):
                    continue
                turn = (k - h) % 4
                penalty = {0: 0.0, 1: 0.3, 3: 0.3, 2: 1.5}[turn]
                c2 = cost + 1.0 + penalty
                if c2 < best.get((m, k), 1e9):
                    best[(m, k)] = c2
                    prev[(m, k)] = (n, h)
                    heapq.heappush(queue, (c2, m, k))
        return None

    def choose_route(self, start, heading):
        """Route to the closest node with unseen segments, or back home."""
        t_left = self.final_time - self.robot.getTime()
        if not self.going_home:
            route = self.plan(start, heading, self.map.unknown_dirs)
            if route is not None:
                home = self.plan(route[-1], heading, lambda n: n == self.start)
                needed = (len(route) + len(home or [])) * CELL / (0.8 * CRUISE_SPEED) + 5
                if needed < t_left:
                    return route
            self.going_home = True
            print("Exploration finished at %.1f s, going home" % self.robot.getTime())
        return self.plan(start, heading, lambda n: n == self.start)

    # --- motion ----------------------------------------------------------------
    def drive(self, v, w):
        left = (v - w * AXLE_LENGTH / 2.0) / WHEEL_RADIUS
        right = (v + w * AXLE_LENGTH / 2.0) / WHEEL_RADIUS
        scale = max(1.0, abs(left) / MAX_WHEEL_SPEED, abs(right) / MAX_WHEEL_SPEED)
        self.left.setVelocity(left / scale)
        self.right.setVelocity(right / scale)

    def stop(self):
        self.left.setVelocity(0.0)
        self.right.setVelocity(0.0)

    def lookahead_point(self):
        """Point LOOKAHEAD ahead along the path, from the closest path point."""
        pts = [node_xy(n) for n in self.path]
        i = max(1, self.target)
        ax, ay = pts[i - 1]
        bx, by = pts[i]
        sx, sy = bx - ax, by - ay
        t = ((self.x - ax) * sx + (self.y - ay) * sy) / (CELL * CELL)
        t = max(0.0, min(1.0, t))
        remaining = LOOKAHEAD + 0.0
        px, py = ax + sx * t, ay + sy * t
        seg_left = (1.0 - t) * CELL
        while remaining > seg_left and i + 1 < len(pts):
            remaining -= seg_left
            i += 1
            px, py = pts[i - 1]
            seg_left = CELL
        if i < len(pts):
            bx, by = pts[i]
            ux, uy = (bx - px) / max(1e-9, math.hypot(bx - px, by - py)), \
                (by - py) / max(1e-9, math.hypot(bx - px, by - py))
            step = min(remaining, math.hypot(bx - px, by - py))
            px, py = px + ux * step, py + uy * step
        return px, py

    def follow(self):
        gx, gy = node_xy(self.path[self.target])
        dist_next = math.hypot(gx - self.x, gy - self.y)
        last = self.target == len(self.path) - 1

        if not last and dist_next < REPLAN_DIST:
            self.replan_from(self.path[self.target])
            return

        if last:
            if dist_next < STOP_TOLERANCE:
                self.stop()
                self.mode = "arrived"
                return
            # aim straight at the final node and slow down near it
            alpha = wrap(math.atan2(gy - self.y, gx - self.x) - self.th)
            if abs(alpha) > math.pi / 2 and dist_next < 0.02:
                self.stop()             # overshot slightly: good enough
                self.mode = "arrived"
                return
            v = min(CRUISE_SPEED, max(0.02, 1.2 * dist_next))
            curvature = 2.0 * math.sin(alpha) / max(dist_next, LOOKAHEAD)
        else:
            px, py = self.lookahead_point()
            alpha = wrap(math.atan2(py - self.y, px - self.x) - self.th)
            dist = math.hypot(px - self.x, py - self.y)
            curvature = 2.0 * math.sin(alpha) / max(dist, 1e-3)
            v = CRUISE_SPEED / (1.0 + 0.04 * abs(curvature))
            for i in (self.target - 1, self.target):
                cx, cy = node_xy(self.path[i])
                if self.turns_at(i) and math.hypot(cx - self.x, cy - self.y) < 0.09:
                    v = min(v, CORNER_SPEED)

        if abs(alpha) > 1.6:
            self.after_spin = "follow"
            self.start_spin(math.atan2(gy - self.y, gx - self.x))
            return
        self.drive(v, v * curvature)

    def turns_at(self, i):
        if i <= 0 or i + 1 >= len(self.path):
            return False
        a, b, c = self.path[i - 1], self.path[i], self.path[i + 1]
        return (b[0] - a[0], b[1] - a[1]) != (c[0] - b[0], c[1] - b[1])

    def replan_from(self, node):
        """Called just before reaching `node`: decide where to go next."""
        incoming = self.path[self.target - 1] if self.target > 0 else None
        heading = self.th
        if incoming is not None:
            heading = math.atan2(node[1] - incoming[1], node[0] - incoming[0])
        route = self.choose_route(node, heading)
        if route is None or len(route) < 2:
            # stop at this node (goal reached, or nothing reachable)
            self.path = ([incoming] if incoming else []) + [node]
            self.target = len(self.path) - 1
            return
        nxt = route[1]
        if incoming is not None and nxt == incoming:
            # dead end ahead: stop on the node, then turn around
            self.path = [incoming, node]
            self.target = 1
            return
        self.path = ([incoming] if incoming else []) + route
        self.target = len(self.path) - len(route) + 1

    def start_spin(self, heading):
        self.spin_heading = heading
        self.mode = "spin"

    def spin(self):
        err = wrap(self.spin_heading - self.th)
        if abs(err) < 0.06:
            self.stop()
            self.mode = self.after_spin
            return
        speed = max(0.8, min(SPIN_SPEED, 6.0 * abs(err)))
        speed = speed if err > 0 else -speed
        self.left.setVelocity(-speed)
        self.right.setVelocity(speed)

    def at_node(self):
        """Robot is stopped on a node: look at unseen segments, then go on."""
        here = self.node
        unknown = self.map.unknown_dirs(here)
        if unknown and not self.going_home:
            # face the unseen segment nearest to the current heading
            d = min(unknown, key=lambda d: abs(wrap(math.atan2(d[1], d[0]) - self.th)))
            target = math.atan2(d[1], d[0])
            if abs(wrap(target - self.th)) > 0.1:
                self.after_spin = "arrived"
                self.start_spin(target)
                return
            # facing it and still unsure after a moment: trust what we see
            self.look_dirs.append(d)
            if self.look_dirs.count(d) > 10:
                e = edge(here, (here[0] + d[0], here[1] + d[1]))
                self.map.votes[e] = -VOTES_TO_REJECT if self.map.votes.get(e, 0) <= 0 else VOTES_TO_DECIDE
            self.stop()
            return
        self.look_dirs = []

        route = self.choose_route(here, self.th)
        if route is None or len(route) < 2:
            if here == self.start:
                self.mode = "done"
            return
        self.path = route
        self.target = 1
        first = math.atan2(route[1][1] - here[1], route[1][0] - here[0])
        if abs(wrap(first - self.th)) > 0.25:
            self.after_spin = "follow"
            self.start_spin(first)
        else:
            self.mode = "follow"

    # --- main loop -------------------------------------------------------------
    def step(self):
        """One control step. Call it once per robot.step(); False = finished."""
        if not self.started:
            # wait until the supervisor has placed the robot and GPS is steady
            g = self.gps.getValues()
            prev, self.prev_gps = getattr(self, "prev_gps", None), g
            self.wait_steps = getattr(self, "wait_steps", 0) + 1
            if (self.wait_steps < 15 or prev is None or any(v != v for v in g)
                    or abs(g[0] - prev[0]) + abs(g[1] - prev[1]) > 1e-4):
                self.stop()
                return True
            self.started = True
            self.read_pose()
            self.start = self.node
            self.last_node = self.node
            self.after_spin = "arrived"
            self.mode = "arrived"
            print("C2 explorer: start node", self.start, "time limit", self.final_time)
            return True

        self.read_pose()
        self.frame += 1
        if self.frame % 2 == 0 and self.turn_rate < MAX_TURN_RATE:
            self.look_with_camera()
        self.check_ground()
        self.track_nodes()

        t = self.robot.getTime()
        if t - self.last_write > 2.0:
            self.map.write(self.start)
            self.last_write = t

        if self.mode == "follow":
            if self.target >= len(self.path):
                self.mode = "arrived"
            else:
                self.follow()
        elif self.mode == "spin":
            self.spin()
        elif self.mode == "arrived":
            self.stop()
            self.at_node()
        elif self.mode == "done":
            return False

        if t > self.final_time - 1.0:
            return False
        return True

    def finish(self):
        """Stop the robot and write the output files."""
        self.stop()
        self.map.write(self.start)
        print("C2 explorer: finished at %.1f s, map written to %s"
              % (self.robot.getTime(), os.path.abspath(MAP_FILE)))


explorer = Explorer(robot, gps, compass, camera)


# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    
    # print("customData",robot.getCustomData())

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    frame = camera.getImage()

    # (added) mapping / planning / driving
    if not explorer.step():
        break
    # print('GPS', gps.getValues(),end="")
    # print('compass', compass.getValues())

# Enter here exit cleanup code.
explorer.finish()
