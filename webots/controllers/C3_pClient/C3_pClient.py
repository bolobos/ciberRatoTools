"""
Webots controller (C3_pClient.py) - PLANNING: fastest route through all the checkpoints.

1. Reads 'planning.out' (same 21 x 49 format as the mapping export): '-' and '|' are the links
   between two neighbouring junctions, the checkpoints are digits written on the junctions.
2. Plans the route with a Dijkstra search over the states (cell, heading, checkpoints visited).
   It is your BFS (same states, same actions STRAIGHT / LEFT / RIGHT / TURN180 through the
   junction ahead) with a priority queue instead of a FIFO queue, because the actions
   do not all cost the same:
     - start = checkpoint 0, the order of the other checkpoints is free,
     - the route ends ON the last checkpoint (no return to the start),
     - LEFT / RIGHT last 14 % longer than STRAIGHT.
3. Executes the route with the same movement functions as the mapping controller (C2_pClient.py).
"""

import heapq
import math
from controller import Robot

# -------------------------------------------------------------
# CONSTANTS & CONFIGURATION
# -------------------------------------------------------------
DEBUG = False              # True: detailed console output (plan, route drawn on the map, steps)

def debug_print(*args, **kwargs):
    if DEBUG:
        print(*args, **kwargs)

robot = Robot()
timestep = int(robot.getBasicTimeStep())

MAX_SPEED = 6.28
GROUND_BLACK_THRESHOLD = 400    # Ground sensors: low value = black line

# Map file (same geometry as the mapping export)
PLANNING_FILENAME = "../C3_supervisor/planning.out"
PATH_FILENAME = "../C3_supervisor/planning.path"   # Written at each execution: closed tour, coordinates relative to the start
RETURN_TO_START = False           # False: the robot stops on the last checkpoint. True: it also drives back to 0
MAP_ROWS = 21
MAP_COLS = 49
CENTER_COL = MAP_COLS // 2      # column 24
CENTER_ROW = MAP_ROWS // 2      # row 10

# Physical distances (in meters) - identical to the mapping controller
FIRST_STRAIGHT_DISTANCE = 0.082    # Only for the very first straight movement
DEFAULT_STRAIGHT_DISTANCE = 0.146  # Subsequent straight movements (2 cells: through a junction)
POST_TURN_DISTANCE = 0.016         # Forward distance after a turn
FINAL_APPROACH_DISTANCE = DEFAULT_STRAIGHT_DISTANCE / 2   # 1 cell: cell centre -> checkpoint centre

# Costs of the actions (unit = one STRAIGHT movement)
COST_STRAIGHT = 1.0
COST_TURN = 1.14 * COST_STRAIGHT        # LEFT / RIGHT: 14 % longer than a straight
COST_TURN180 = 1.5 * COST_STRAIGHT      # U-turn in place: ESTIMATE, time it and adjust
COST_FINAL_APPROACH = 0.5 * COST_STRAIGHT   # last step: stops on the checkpoint (half a straight)

ACTION_COST = {
    "STRAIGHT": COST_STRAIGHT,
    "LEFT": COST_TURN,
    "RIGHT": COST_TURN,
    "TURN180": COST_TURN180,
    # VISIT: go to the checkpoint ahead, turn around ON it, come back to the cell centre.
    # Needed for a checkpoint at a dead end, which can never be crossed.
    "VISIT": 2 * COST_FINAL_APPROACH + COST_TURN180,
}
ACTION_ORDER = ("STRAIGHT", "LEFT", "RIGHT", "TURN180", "VISIT")   # Exploration order = tie-breaking priority
HEADING_NAMES = {0: "North", 1: "East", 2: "South", 3: "West"}

is_first_movement = True    # The first straight movement uses FIRST_STRAIGHT_DISTANCE

# Motors
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Sensors (no camera needed: the route is already planned)
gps = robot.getDevice('gps')
gps.enable(timestep)
compass = robot.getDevice('compass')
compass.enable(timestep)

gs_left = robot.getDevice('gs0')      # Left ground sensor
gs_center = robot.getDevice('gs1')    # Center ground sensor
gs_right = robot.getDevice('gs2')     # Right ground sensor
gs_left.enable(timestep)
gs_center.enable(timestep)
gs_right.enable(timestep)


# -------------------------------------------------------------
# NAVIGATION AND COORDINATE TOOLS (same conventions as the mapping)
# -------------------------------------------------------------
def get_bearing():
    north = compass.getValues()
    rad = math.atan2(north[0], north[1])
    return (math.degrees(rad) + 360) % 360

def get_delta_angle(current, previous):
    return (current - previous + 180) % 360 - 180

def get_heading_error(target_deg, current_deg):
    return (target_deg - current_deg + 180) % 360 - 180

def get_gps_position():
    vals = gps.getValues()
    return vals[0], vals[1]

def bearing_to_grid_heading(bearing_deg):
    """Converts compass bearing to logical heading (0=North, 1=East, 2=South, 3=West)."""
    return int(round((90.0 - bearing_deg) / 90.0)) % 4

def step_from_heading(x, y, heading):
    """Returns the neighbouring cell from a logical heading."""
    if heading == 0:
        return x, y + 1
    if heading == 1:
        return x + 1, y
    if heading == 2:
        return x, y - 1
    if heading == 3:
        return x - 1, y
    raise ValueError(f"Invalid logical heading: {heading}")


# =============================================================
# MAP READING
# =============================================================
def load_planning_map(filename=PLANNING_FILENAME):
    """Reads the first MAP_ROWS lines of the file (the lines after the map are ignored).

    Planning frame: x to the East, y to the North, (0, 0) = centre of the file.
    Junctions are on even (x, y); a link is the cell between two junctions.

    Returns (rows, links, nodes, checkpoints):
      rows        : the map as MAP_ROWS strings of MAP_COLS characters
      links       : set of (x, y) link cells ('-' or '|')
      nodes       : set of (x, y) junctions (ends of the links + checkpoints)
      checkpoints : {number: (x, y)}
    """
    with open(filename, "r", encoding="utf-8") as file:
        raw = file.read().split("\n")
    rows = [(raw[r].rstrip("\r") if r < len(raw) else "").ljust(MAP_COLS)[:MAP_COLS]
            for r in range(MAP_ROWS)]

    links, checkpoints = set(), {}
    for r, line in enumerate(rows):
        for c, ch in enumerate(line):
            if ch == " ":
                continue
            x, y = c - CENTER_COL, CENTER_ROW - r
            if ch == "-" and x % 2 != 0 and y % 2 == 0:
                links.add((x, y))
            elif ch == "|" and x % 2 == 0 and y % 2 != 0:
                links.add((x, y))
            elif ch.isdigit() and x % 2 == 0 and y % 2 == 0:
                checkpoints[int(ch)] = (x, y)
            else:
                print(f"[MAP] Character '{ch}' ignored (row {r}, column {c}): inconsistent position")

    nodes = set(checkpoints.values())
    for (x, y) in links:
        if x % 2 != 0:      # horizontal link: junctions on its left and right
            nodes.update({(x - 1, y), (x + 1, y)})
        else:               # vertical link: junctions above and below
            nodes.update({(x, y - 1), (x, y + 1)})

    return rows, links, nodes, checkpoints


# =============================================================
# PLANNING (Dijkstra = BFS with costs)
# =============================================================
# The robot's state is (x, y, heading, mask): 'x, y' is the link cell where the robot stands,
# the junction is one step ahead, 'mask' = bit field of the checkpoints already visited.

def apply_plan_move(x, y, heading, action, nodes, links):
    """Simulates an action on the known map (same logic as apply_virtual_move of the mapping).
    Returns (nx, ny, nheading, junction_passed) or None if the action is impossible."""
    if action == "TURN180":
        return x, y, (heading + 2) % 4, None

    junction = step_from_heading(x, y, heading)
    if junction not in nodes:            # Nothing ahead of the robot
        return None

    if action == "VISIT":                # Reach the junction, U-turn on it, back to the same cell
        return x, y, (heading + 2) % 4, junction

    if action == "STRAIGHT":
        new_heading = heading
    elif action == "LEFT":
        new_heading = (heading - 1) % 4
    else:  # RIGHT
        new_heading = (heading + 1) % 4

    nx, ny = step_from_heading(junction[0], junction[1], new_heading)
    if (nx, ny) not in links:            # No link on this side of the junction
        return None
    return nx, ny, new_heading, junction

def plan_route(nodes, links, checkpoints, start_cell, start_heading, return_to_start=False):
    """Fastest route: starts at checkpoint 0, visits all the checkpoints (free order),
    then stops ON the last one (return_to_start=False) or back ON checkpoint 0
    (return_to_start=True, closed tour).

    Returns (steps, total_cost), or (None, None) if a checkpoint is unreachable.
    Each step is a dict: action, cell, heading (before the action), node (junction passed),
    checkpoint (its number or None), to_cell (cell reached).
    """
    numbers = sorted(checkpoints)
    bit = {checkpoints[n]: 1 << i for i, n in enumerate(numbers)}      # junction -> bit
    number_of = {coord: n for n, coord in checkpoints.items()}
    full_mask = (1 << len(numbers)) - 1
    start_mask = bit[checkpoints[0]]                                   # Start = checkpoint 0

    if start_mask == full_mask:
        return [], 0.0

    GOAL = "GOAL"
    start = (start_cell[0], start_cell[1], start_heading, start_mask)
    best = {start: 0.0}
    parent = {start: None}
    heap = [(0.0, 0, start)]
    tie = 1                              # Tie-breaker: states are never compared

    while heap:
        cost, _, state = heapq.heappop(heap)
        if state == GOAL:
            break
        if cost > best[state]:
            continue
        x, y, h, mask = state

        # Open route: the junction ahead is the last missing checkpoint -> go and stop on it.
        # Closed tour: everything is visited and the junction ahead is the start -> stop on it.
        junction = step_from_heading(x, y, h)
        junction_bit = bit.get(junction, 0)
        if return_to_start:
            finished = mask == full_mask and junction == checkpoints[0]
        else:
            finished = bool(junction_bit) and not (mask & junction_bit) and (mask | junction_bit) == full_mask
        if finished:
            total = cost + COST_FINAL_APPROACH
            if total < best.get(GOAL, float("inf")):
                best[GOAL] = total
                parent[GOAL] = (state, "FINAL")
                heapq.heappush(heap, (total, tie, GOAL))
                tie += 1
            continue

        for action in ACTION_ORDER:
            if action == "VISIT" and (not junction_bit or (mask & junction_bit)):
                continue                 # VISIT only makes sense on a checkpoint not yet collected
            result = apply_plan_move(x, y, h, action, nodes, links)
            if result is None:
                continue
            nx, ny, nh, passed = result
            new_mask = mask | bit.get(passed, 0)
            next_state = (nx, ny, nh, new_mask)
            new_cost = cost + ACTION_COST[action]
            if new_cost < best.get(next_state, float("inf")):
                best[next_state] = new_cost
                parent[next_state] = (state, action)
                heapq.heappush(heap, (new_cost, tie, next_state))
                tie += 1

    if GOAL not in parent:
        return None, None

    # Backtrack through the parents (same principle as reconstruct_actions of the mapping)
    chain = []
    state = GOAL
    while parent[state] is not None:
        previous, action = parent[state]
        chain.append((action, previous))
        state = previous
    chain.reverse()

    steps = []
    for action, (x, y, h, _) in chain:
        if action == "FINAL":
            node, to_cell = step_from_heading(x, y, h), step_from_heading(x, y, h)
        else:
            nx, ny, _, node = apply_plan_move(x, y, h, action, nodes, links)
            to_cell = (nx, ny)
        steps.append({"action": action, "cell": (x, y), "heading": h, "node": node,
                      "checkpoint": number_of.get(node), "to_cell": to_cell})
    return steps, best[GOAL]

def checkpoint_order(steps):
    """Order in which the checkpoints are reached (the start, 0, comes first)."""
    order = [0]
    for step in steps:
        number = step["checkpoint"]
        if number is not None and number not in order:
            order.append(number)
    return order

def build_path_cells(steps, checkpoints):
    """Junctions (= cells of the path file) visited by the route, in order, starting at checkpoint 0.
    A U-turn made on a link cell counts the junction the robot was facing, so that
    two consecutive cells are always neighbours."""
    cells = [checkpoints[0]]
    for step in steps:
        if step["action"] == "TURN180":
            target = step_from_heading(step["cell"][0], step["cell"][1], step["heading"])
        else:
            target = step["node"]
        if target != cells[-1]:
            cells.append(target)
    return cells

def write_path_file(steps, checkpoints, filename=PATH_FILENAME):
    """Writes the .path file: one line 'x y' per visited cell (+ ' #n' on a target cell).
    Coordinates are in cell units, relative to the start (checkpoint 0): first and last line = '0 0'."""
    cells = build_path_cells(steps, checkpoints)
    number_of = {coord: n for n, coord in checkpoints.items()}
    x0, y0 = checkpoints[0]
    lines = []
    for (x, y) in cells:
        line = f"{(x - x0) // 2} {(y - y0) // 2}"       # Junctions are 2 logical cells apart = 1 path cell
        if (x, y) in number_of:
            line += f" #{number_of[(x, y)]}"
        lines.append(line)
    if len(lines) == 1:                                  # Only the start: it is also the end
        lines.append(lines[0])
    try:
        with open(filename, "w", encoding="utf-8", newline="\n") as file:
            file.write("\n".join(lines) + "\n")
    except OSError as error:
        print(f"[PATH] Unable to write '{filename}': {error}")
    return lines

def render_route(rows, steps, start_cell):
    """The file map with the route drawn with '*' (junctions and links used). Digits are kept."""
    canvas = [list(line) for line in rows]

    def mark(cell):
        if cell is None:
            return
        c, r = CENTER_COL + cell[0], CENTER_ROW - cell[1]
        if 0 <= r < MAP_ROWS and 0 <= c < MAP_COLS and not canvas[r][c].isdigit():
            canvas[r][c] = "*"

    mark(start_cell)
    for step in steps:
        mark(step["node"])
        mark(step["to_cell"])
    lines = ["".join(line) for line in canvas]
    return "\n".join(line for line in lines if line.strip())


# -------------------------------------------------------------
# PHYSICAL MOVEMENT FUNCTIONS (same as the mapping controller)
# -------------------------------------------------------------
def goStraight(distance=0.5, speed=MAX_SPEED):
    start_x, start_y = get_gps_position()
    target_heading = get_bearing()
    kp = 0.08

    while robot.step(timestep) != -1:
        is_left_black = gs_left.getValue() < GROUND_BLACK_THRESHOLD
        is_right_black = gs_right.getValue() < GROUND_BLACK_THRESHOLD

        # 1. Only the left sensor touches black -> recenter left
        if is_left_black and not is_right_black:
            left_motor.setVelocity(speed * 0.5)
            right_motor.setVelocity(speed * 0.9)
        # 2. Only the right sensor touches black -> recenter right
        elif is_right_black and not is_left_black:
            left_motor.setVelocity(speed * 0.9)
            right_motor.setVelocity(speed * 0.5)
        # 3. Otherwise: compass navigation
        else:
            error = get_heading_error(target_heading, get_bearing())
            left_motor.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, speed + (kp * error))))
            right_motor.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, speed - (kp * error))))

        curr_x, curr_y = get_gps_position()
        traveled = math.sqrt((curr_x - start_x) ** 2 + (curr_y - start_y) ** 2)
        if traveled >= distance:
            break

def turn180(forward_distance=0.0, speed=MAX_SPEED):
    accumulated_angle, previous_heading = 0.0, get_bearing()
    left_motor.setVelocity(-speed * 0.5)
    right_motor.setVelocity(speed * 0.5)
    while robot.step(timestep) != -1:
        current = get_bearing()
        accumulated_angle += abs(get_delta_angle(current, previous_heading))
        previous_heading = current
        remaining = 180.0 - accumulated_angle

        if remaining < 15.0:
            reduced = max(1.0, speed * (remaining / 15.0))
            left_motor.setVelocity(-reduced * 0.5)
            right_motor.setVelocity(reduced * 0.5)
        if accumulated_angle >= 180.0:
            break
    if forward_distance > 0:
        goStraight(distance=forward_distance, speed=speed)

def goLeft(forward_distance=0.0, speed=MAX_SPEED):
    accumulated_angle, previous_heading = 0.0, get_bearing()
    left_motor.setVelocity(speed * 0.4)
    right_motor.setVelocity(speed)
    while robot.step(timestep) != -1:
        current = get_bearing()
        accumulated_angle += abs(get_delta_angle(current, previous_heading))
        previous_heading = current
        if accumulated_angle >= 88.0:
            break
    if forward_distance > 0:
        goStraight(distance=forward_distance, speed=speed)

def goRight(forward_distance=0.0, speed=MAX_SPEED):
    accumulated_angle, previous_heading = 0.0, get_bearing()
    left_motor.setVelocity(speed)
    right_motor.setVelocity(speed * 0.4)
    while robot.step(timestep) != -1:
        current = get_bearing()
        accumulated_angle += abs(get_delta_angle(current, previous_heading))
        previous_heading = current
        if accumulated_angle >= 88.0:
            break
    if forward_distance > 0:
        goStraight(distance=forward_distance, speed=speed)


# -------------------------------------------------------------
# ROUTE EXECUTION
# -------------------------------------------------------------
def execute_plan(steps):
    global is_first_movement

    for index, step in enumerate(steps, 1):
        action = step["action"]

        # Safety check: the real heading must match the planned one
        real_heading = bearing_to_grid_heading(get_bearing())
        if real_heading != step["heading"]:
            print(f"[WARNING] Step {index}: heading {HEADING_NAMES[real_heading]} "
                  f"instead of {HEADING_NAMES[step['heading']]} planned")

        debug_print(f"[STEP {index}/{len(steps)}] {action} | cell {step['cell']} "
                    f"facing {HEADING_NAMES[step['heading']]}"
                    + (f" | checkpoint {step['checkpoint']}" if step["checkpoint"] is not None else ""))

        if action == "STRAIGHT":
            if is_first_movement:
                goStraight(distance=FIRST_STRAIGHT_DISTANCE)
            else:
                goStraight(distance=DEFAULT_STRAIGHT_DISTANCE)
        elif action == "LEFT":
            goLeft(forward_distance=POST_TURN_DISTANCE)
        elif action == "RIGHT":
            goRight(forward_distance=POST_TURN_DISTANCE)
        elif action == "TURN180":
            turn180(forward_distance=POST_TURN_DISTANCE)
        elif action == "VISIT":
            goStraight(distance=FINAL_APPROACH_DISTANCE)   # Up to the checkpoint centre
            turn180(forward_distance=FINAL_APPROACH_DISTANCE)   # U-turn on it, back to the cell centre
        elif action == "FINAL":
            goStraight(distance=FINAL_APPROACH_DISTANCE)   # Stops ON the last checkpoint

        is_first_movement = False

    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)
    robot.step(timestep)


# -------------------------------------------------------------
# MAIN
# -------------------------------------------------------------
def main():
    robot.step(timestep)    # First step: the sensors become valid

    rows, links, nodes, checkpoints = load_planning_map(PLANNING_FILENAME)
    if 0 not in checkpoints:
        print(f"[PLAN] Checkpoint 0 (start) not found in '{PLANNING_FILENAME}'")
        return

    # Like in the mapping: the robot stands on the cell just before the first junction
    # (= checkpoint 0), facing it. The heading comes from the compass.
    start_heading = bearing_to_grid_heading(get_bearing())
    start_node = checkpoints[0]
    start_cell = step_from_heading(start_node[0], start_node[1], (start_heading + 2) % 4)

    # Closed tour (visit everything, come back to 0): this is what the .path file must contain
    closed_steps, closed_cost = plan_route(nodes, links, checkpoints, start_cell, start_heading,
                                           return_to_start=True)
    if closed_steps is None:
        print("[PLAN] No route: at least one checkpoint is unreachable from checkpoint 0")
        return
    path_lines = write_path_file(closed_steps, checkpoints)
    print(f"[PATH] '{PATH_FILENAME}' written ({len(path_lines)} cells, closed tour)")

    # Route driven by the robot
    if RETURN_TO_START:
        steps, total_cost = closed_steps, closed_cost
    else:
        steps, total_cost = plan_route(nodes, links, checkpoints, start_cell, start_heading)

    order = checkpoint_order(steps)
    print(f"[PLAN] Checkpoints {sorted(checkpoints)} | start facing {HEADING_NAMES[start_heading]}")
    print(f"[PLAN] Route: {' -> '.join(str(n) for n in order)} | {len(steps)} movements | "
          f"cost = {total_cost:.2f} (1.00 = one straight)")
    debug_print(f"[PLAN] Actions: {[s['action'] for s in steps]}")
    debug_print("[PLAN] Route on the map ('*' = route, digits = checkpoints):")
    debug_print(render_route(rows, steps, start_cell))

    execute_plan(steps)
    print("End of the planning: last checkpoint reached")


if __name__ == "__main__":
    main()