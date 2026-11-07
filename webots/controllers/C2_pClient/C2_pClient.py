"""
Webots controller (C2_pClient.py) with Trémaux's Algorithm and Adaptive Distances.
1. First straight movement: FIRST_STRAIGHT_DISTANCE (0.082 m).
2. Subsequent straight movements: DEFAULT_STRAIGHT_DISTANCE (0.140 m).
3. Virtual mapping (X, Y) with dead-end handling, backtracking, and continuous display.
"""

import math
from collections import deque
from controller import Robot

# -------------------------------------------------------------
# CONSTANTS & CONFIGURATION
# -------------------------------------------------------------
DEBUG = False              # Set to False to disable all debug console output

def debug_print(*args, **kwargs):
    if DEBUG:
        print(*args, **kwargs)

robot = Robot()
timestep = int(robot.getBasicTimeStep())

MAX_SPEED = 6.28
BLACK_RGB_THRESHOLD = 120
UNEXPLORED_SYMBOL = "u"   # Rest of the grid (completely unknown)
FRONTIER_SYMBOL = "?"     # Paths detected at junctions but not yet explored

# Distance parameters (in meters)
FIRST_STRAIGHT_DISTANCE = 0.082    # Distance only for the very first straight movement
DEFAULT_STRAIGHT_DISTANCE = 0.146  # Higher distance for subsequent straight lines
POST_TURN_DISTANCE = 0.016         # Forward distance after a turn

step_counter = 0
is_first_movement = True           # Flag to identify the first movement
path_history = []

# =============================================================
# MAPPING VARIABLES (VIRTUAL MAP)
# =============================================================
grid_map = {}      # Dictionary {(x, y): visit_count}
blocked_cells = set()  # Cells known to be inaccessible (wall)
midpoint_markers = set()  # Midpoints indicating a junction/turn
frontier_cells = set()  # Open cells detected but not yet visited
grid_x = 0         # Logical X position
grid_y = 0         # Logical Y position
grid_heading = 0   # Logical heading: 0=North, 1=East, 2=South, 3=West
start_junction = None  # First [o] junction encountered = 'I' (start) cell in the map file

# Motors
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Sensors
gps = robot.getDevice('gps')
gps.enable(timestep)
camera = robot.getDevice('camera')
camera.enable(timestep)
compass = robot.getDevice('compass')
compass.enable(timestep)

# Ground Sensors
gs_left = robot.getDevice('gs0')      # Left Sensor
gs_center = robot.getDevice('gs1')    # Center Sensor
gs_right = robot.getDevice('gs2')     # Right Sensor

gs_left.enable(timestep)
gs_center.enable(timestep)
gs_right.enable(timestep)

# Black detection threshold for ground sensors (low value = black line)
GROUND_BLACK_THRESHOLD = 400


# -------------------------------------------------------------
# NAVIGATION AND COORDINATE TOOLS
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
    """Converts compass bearing to logical heading. Required convention: 0° -> East (right)."""
    # 0=North in the logical map. Offset by +90° so that 0° compass corresponds to East.
    return int(round((90.0 - bearing_deg) / 90.0)) % 4

def sync_grid_heading_from_compass():
    """Synchronizes the logical heading with the real heading to prevent mapping drift."""
    global grid_heading
    grid_heading = bearing_to_grid_heading(get_bearing())

def step_from_heading(x, y, heading):
    """Returns the neighboring cell from a logical heading."""
    if heading == 0:
        return x, y + 1
    if heading == 1:
        return x + 1, y
    if heading == 2:
        return x, y - 1
    if heading == 3:
        return x - 1, y
    raise ValueError(f"Invalid logical heading: {heading}")

def get_target_coord(x, y, heading, action):
    """Computes the target cell by systematically advancing 2 cells (midpoint [o] + target cell)."""
    mid_x, mid_y = step_from_heading(x, y, heading)

    if action == "STRAIGHT":
        return step_from_heading(mid_x, mid_y, heading)

    if action == "LEFT":
        new_h = (heading - 1) % 4
        return step_from_heading(mid_x, mid_y, new_h)

    if action == "RIGHT":
        new_h = (heading + 1) % 4
        return step_from_heading(mid_x, mid_y, new_h)

    if action == "TURN180":
        new_h = (heading + 2) % 4
        return step_from_heading(x, y, new_h)

    raise ValueError(f"Unknown action: {action}")

def get_branch_probe_coord(x, y, heading, action):
    """Target cell observed by the camera (always 2 cells away via the midpoint)."""
    mid_x, mid_y = step_from_heading(x, y, heading)

    if action == "STRAIGHT":
        return step_from_heading(mid_x, mid_y, heading)
    if action == "LEFT":
        return step_from_heading(mid_x, mid_y, (heading - 1) % 4)
    if action == "RIGHT":
        return step_from_heading(mid_x, mid_y, (heading + 1) % 4)

    raise ValueError(f"Unknown branch action: {action}")

def mark_lateral_corridor_walls(x, y, heading):
    """Marks lateral cells as empty for a corridor, unless already known to be open/visited."""
    global blocked_cells, frontier_cells
    for side_heading in ((heading - 1) % 4, (heading + 1) % 4):
        sx, sy = step_from_heading(x, y, side_heading)
        if (sx, sy) in frontier_cells:
            continue
        if (sx, sy) in grid_map:
            continue
        if (sx, sy) in midpoint_markers:
            continue
        blocked_cells.add((sx, sy))

# =============================================================
# BFS: SHORTEST PATH TO THE NEAREST '?' FRONTIER
# =============================================================
# The robot's state is (x, y, heading) because the heading determines where the
# midpoint ahead is located. Actions: STRAIGHT / LEFT / RIGHT (via the midpoint)
# and TURN180 (in-place U-turn). Each action costs 1.
DEBUG_BFS = True          # Detailed BFS logs (active only when DEBUG is True)
ACTION_ORDER = ("STRAIGHT", "LEFT", "RIGHT", "TURN180")   # Exploration order = tie-breaking priority
HEADING_NAMES = {0: "North", 1: "East", 2: "South", 3: "West"}

def bfs_log(message):
    if DEBUG and DEBUG_BFS:
        print(message)

def is_walkable_node(cell):
    """Possible destination cell: start, visited cell (excluding dead end), or frontier."""
    if cell in blocked_cells:
        return False
    if grid_map.get(cell, 0) >= 999:   # Permanent dead end
        return False
    return (cell == (0, 0)
            or cell in grid_map
            or cell in frontier_cells
            or cell == (grid_x, grid_y))

def apply_virtual_move(x, y, heading, action):
    """Simulates an action on the map. Returns (nx, ny, nheading, mid) or None if impossible."""
    if action == "TURN180":
        return x, y, (heading + 2) % 4, None

    mid = step_from_heading(x, y, heading)
    if mid not in midpoint_markers:      # No known midpoint ahead -> cannot pass
        return None

    if action == "STRAIGHT":
        new_heading = heading
    elif action == "LEFT":
        new_heading = (heading - 1) % 4
    else:  # RIGHT
        new_heading = (heading + 1) % 4

    nx, ny = step_from_heading(mid[0], mid[1], new_heading)
    if not is_walkable_node((nx, ny)):
        return None
    return nx, ny, new_heading, mid

def bfs_nearest_frontier(start_state, available_moves, forbid_turn180=False):
    """BFS from start_state. Returns (found_frontiers, parents, distances, num_expanded_states).

    - At the start, only actually open actions (camera) are allowed + TURN180.
    - Never expands beyond a frontier (its neighborhood is still unknown).
    - 'found_frontiers' is sorted by increasing distance (BFS discovery order).
    """
    sx, sy, _ = start_state
    queue = deque([start_state])
    parent = {start_state: None}      # state -> (previous_state, action)
    dist = {start_state: 0}
    found = []
    expanded = 0

    while queue:
        state = queue.popleft()
        x, y, h = state
        expanded += 1

        if (x, y) in frontier_cells and (x, y) != (sx, sy):
            found.append(state)
            continue

        for action in ACTION_ORDER:
            if state == start_state:
                if action == "TURN180":
                    if forbid_turn180:
                        continue
                elif action not in available_moves:
                    continue

            result = apply_virtual_move(x, y, h, action)
            if result is None:
                continue
            nx, ny, nh, _ = result
            next_state = (nx, ny, nh)
            if next_state in parent:
                continue
            parent[next_state] = (state, action)
            dist[next_state] = dist[state] + 1
            queue.append(next_state)

    return found, parent, dist, expanded

def reconstruct_actions(parent, state):
    """Backtracks through parents. Returns [(action, state_after_action), ...] in execution order."""
    steps = []
    while parent[state] is not None:
        previous, action = parent[state]
        steps.append((action, state))
        state = previous
    steps.reverse()
    return steps

def plan_to_nearest_frontier(available_moves):
    """Plans the path to the nearest frontier. Returns the 1st action, or None."""
    start = (grid_x, grid_y, grid_heading)
    # If we just made a U-turn, do not repeat it (avoids going in circles).
    forbid_turn180 = len(path_history) > 0 and path_history[-1] == "TURN180"

    bfs_log("\n" + "=" * 70)
    bfs_log("=== BFS: SEARCHING FOR THE NEAREST FRONTIER ===")
    bfs_log("=" * 70)
    bfs_log(f"  [BFS] Start: cell ({grid_x}, {grid_y}) | heading {HEADING_NAMES[grid_heading]}")
    bfs_log(f"  [BFS] Allowed actions at start: {list(available_moves)}"
            f"{'' if forbid_turn180 else ' + TURN180'}")
    bfs_log(f"  [BFS] Known frontiers ({len(frontier_cells)}): {sorted(frontier_cells)}")

    found, parent, dist, expanded = bfs_nearest_frontier(start, available_moves, forbid_turn180)

    bfs_log(f"  [BFS] States explored: {expanded} | Reachable frontiers: {len(found)}")
    for state in found:
        bfs_log(f"    - frontier ({state[0]}, {state[1]}) at {dist[state]} action(s)")

    reachable = {(s[0], s[1]) for s in found}
    unreachable = sorted(frontier_cells - reachable)
    if unreachable:
        bfs_log(f"  [BFS] UNREACHABLE frontiers with current map: {unreachable}")

    if not found:
        bfs_log("  [BFS] No reachable frontier -> no path found.")
        return None

    best = found[0]
    steps = reconstruct_actions(parent, best)

    # Print path + compute map overlay
    overlay = {}
    chain = f"({grid_x}, {grid_y})"
    previous_state = start
    for action, state in steps:
        chain += f" --{action}--> ({state[0]}, {state[1]})"
        result = apply_virtual_move(previous_state[0], previous_state[1], previous_state[2], action)
        if result is not None and result[3] is not None:
            overlay[result[3]] = "*"
        overlay[(state[0], state[1])] = "*"
        previous_state = state
    overlay[(best[0], best[1])] = "X"

    bfs_log(f"  [BFS] CHOSEN TARGET: ({best[0]}, {best[1]}) at {dist[best]} action(s)")
    bfs_log(f"  [BFS] Path: {chain}")
    bfs_log(f"  [BFS] Actions: {[a for a, _ in steps]}")
    if DEBUG and DEBUG_BFS:
        print("  [BFS] Map with path ('*' = path, 'X' = target):")
        print(render_map(overlay))

    return steps[0][0]

def record_branch_information(available_moves):
    global blocked_cells, midpoint_markers, frontier_cells, grid_map, start_junction
    heading_for_scan = bearing_to_grid_heading(get_bearing())

    mid_x, mid_y = step_from_heading(grid_x, grid_y, heading_for_scan)
    midpoint_markers.add((mid_x, mid_y))
    if start_junction is None:
        start_junction = (mid_x, mid_y)   # 1st intersection = 'I' cell in the map file
    blocked_cells.discard((mid_x, mid_y))

    # Retroactive cleanup
    for fx, fy in list(frontier_cells):
        if is_between_two_midpoints(fx, fy):
            frontier_cells.discard((fx, fy))
            grid_map[(fx, fy)] = 1

    for action in ("LEFT", "STRAIGHT", "RIGHT"):
        tx, ty = get_branch_probe_coord(grid_x, grid_y, heading_for_scan, action)
        if action in available_moves:
            blocked_cells.discard((tx, ty))
            if (tx, ty) not in grid_map and (tx, ty) not in midpoint_markers:
                if is_between_two_midpoints(tx, ty):
                    grid_map[(tx, ty)] = 1
                    frontier_cells.discard((tx, ty))
                else:
                    frontier_cells.add((tx, ty))
        else:
            frontier_cells.discard((tx, ty))
            if (tx, ty) not in grid_map and (tx, ty) not in midpoint_markers:
                blocked_cells.add((tx, ty))


def recenter_on_line(speed=MAX_SPEED * 0.5):
    """
    Adjusts motor speeds based on the 3 ground sensors
    to keep the black line centered under the robot.
    """
    val_left = gs_left.getValue()
    val_center = gs_center.getValue()
    val_right = gs_right.getValue()

    is_left_black = val_left < GROUND_BLACK_THRESHOLD
    is_center_black = val_center < GROUND_BLACK_THRESHOLD
    is_right_black = val_right < GROUND_BLACK_THRESHOLD

    # If the line deviates to the left (only the left sensor sees black)
    if is_left_black and not is_center_black:
        left_speed = speed * 0.2
        right_speed = speed * 0.8
    # If the line deviates to the right (only the right sensor sees black)
    elif is_right_black and not is_center_black:
        left_speed = speed * 0.8
        right_speed = speed * 0.2
    # If the center sensor is on the line (well centered)
    elif is_center_black:
        left_speed = speed * 0.6
        right_speed = speed * 0.6
    # If no sensor sees the line
    else:
        left_speed = speed * 0.5
        right_speed = speed * 0.5

    return left_speed, right_speed


def is_between_two_midpoints(x, y):
    """Checks if cell (x, y) lies directly between two [o] junctions (horizontally or vertically)."""
    horizontal = ((x - 1, y) in midpoint_markers) and ((x + 1, y) in midpoint_markers)
    vertical = ((x, y - 1) in midpoint_markers) and ((x, y + 1) in midpoint_markers)
    return horizontal or vertical

def render_map(overlay=None):
    """Displays a compact ASCII map around the robot.
    overlay: optional dict {(x, y): character} drawn over the map (e.g., BFS path)."""
    if not grid_map:
        return "Empty map"

    half_window = 6
    all_known = set(grid_map.keys()) | blocked_cells | midpoint_markers | frontier_cells | {(grid_x, grid_y)}
    xs = [x for x, y in all_known]
    ys = [y for x, y in all_known]

    min_x = min(xs) - half_window
    max_x = max(xs) + half_window
    min_y = min(ys) - half_window
    max_y = max(ys) + half_window

    heading_symbols = {0: "^", 1: ">", 2: "v", 3: "<"}
    display_heading = bearing_to_grid_heading(get_bearing())
    lines = []

    for y in range(max_y, min_y - 1, -1):
        row = []
        for x in range(min_x, max_x + 1):
            if (x, y) == (grid_x, grid_y):
                row.append(heading_symbols.get(display_heading, "R"))
            elif overlay and (x, y) in overlay:
                row.append(overlay[(x, y)])
            elif (x, y) in midpoint_markers:
                # Keep the junction visible, even if this cell also has a visit score.
                row.append("o")
            elif (x, y) in grid_map:
                value = grid_map[(x, y)]
                if value >= 999:
                    row.append("#")
                elif value > 0:
                    row.append(str(min(value, 9)))
                else:
                    row.append(".")
            elif (x, y) in frontier_cells:
                row.append(FRONTIER_SYMBOL)
            elif (x, y) in blocked_cells:
                row.append(" ")
            else:
                row.append(UNEXPLORED_SYMBOL)
        lines.append(" ".join(row))

    return "\n" + "\n".join(lines)

# =============================================================
# PATH EXPORT (only '-' and '|')
# =============================================================
DEBUG_EXPORT_MAP = True   # Displays the path in '-'/'|' format in debug output (active only when DEBUG is True)

def is_floor_cell(x, y):
    """True if cell (x, y) is a corridor: visited, midpoint, frontier, or robot position."""
    return ((x, y) in grid_map
            or (x, y) in midpoint_markers
            or (x, y) in frontier_cells
            or (x, y) == (grid_x, grid_y))

def render_export_map():
    """Draws the PATH in '-' and '|' format: 1 character per logical cell.

      - '-' : path cell connected horizontally (corridor to the left or right)
      - '|' : path cell connected vertically (corridor above or below)
      - space : no path (wall, blocked cell, or unknown 'u')
    The path = visited cells (1..9, #), midpoints 'o', frontiers '?',
    and robot position.
    """
    all_known = set(grid_map.keys()) | midpoint_markers | frontier_cells | {(grid_x, grid_y)}
    if not all_known:
        return "Empty map"

    xs = [x for x, y in all_known]
    ys = [y for x, y in all_known]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)

    lines = []
    for y in range(max_y, min_y - 1, -1):
        row = []
        for x in range(min_x, max_x + 1):
            if not is_floor_cell(x, y):
                row.append(" ")   # No path here
                continue
            if is_floor_cell(x - 1, y) or is_floor_cell(x + 1, y):
                row.append("-")   # Horizontal path segment
            elif is_floor_cell(x, y - 1) or is_floor_cell(x, y + 1):
                row.append("|")   # Vertical path segment
            else:
                row.append("-")   # Isolated cell (e.g., starting point)
        lines.append(" ".join(row))

    return "\n" + "\n".join(lines)

# -------------------------------------------------------------
# MAP FILE (mapping.out format)
# -------------------------------------------------------------
MAPPING_FILENAME = "maze.out"
MAPPING_ROWS = 21      # File rows
MAPPING_COLS = 49      # File columns (start 'I' is in the center: column 24, row 10)

def render_mapping_grid():
    """Builds the map in mapping.out format (MAPPING_ROWS x MAPPING_COLS).

      - 'I' : start, at the center of the grid (= first [o] junction)
      - '-' : horizontal link between two neighboring junctions (odd column, even row)
      - '|' : vertical link between two neighboring junctions (even column, odd row)
      - ' ' : everything else (junctions themselves, walls, unknown)

    The [o] junctions are spaced 2 logical cells apart: the link is the cell located
    between the two. Returns (lines, num_out_of_grid_links, num_inconsistent_links).
    """
    grid = [[" "] * MAPPING_COLS for _ in range(MAPPING_ROWS)]
    center_col, center_row = MAPPING_COLS // 2, MAPPING_ROWS // 2
    out_of_grid = 0
    inconsistent = 0

    if start_junction is not None:
        sx, sy = start_junction
        # (0,0) is never in grid_map: test it separately (case of an exit behind start)
        for (x, y) in set(grid_map) | {(0, 0)}:
            if (x - 1, y) in midpoint_markers and (x + 1, y) in midpoint_markers:
                symbol = "-"
            elif (x, y - 1) in midpoint_markers and (x, y + 1) in midpoint_markers:
                symbol = "|"
            else:
                continue   # Not a link between two junctions

            col = center_col + (x - sx)
            row = center_row - (y - sy)   # logical y goes up, file rows go down
            if not (0 <= col < MAPPING_COLS and 0 <= row < MAPPING_ROWS):
                out_of_grid += 1
                continue

            # Consistency check: '-' => odd column / even row, '|' => the opposite
            col_odd = (col - center_col) % 2 == 1
            row_odd = (row - center_row) % 2 == 1
            if (symbol == "-" and not (col_odd and not row_odd)) or (symbol == "|" and not (row_odd and not col_odd)):
                inconsistent += 1
            grid[row][col] = symbol

    grid[center_row][center_col] = "I"
    return ["".join(row) for row in grid], out_of_grid, inconsistent

def export_map_to_txt(filename=MAPPING_FILENAME):
    """Exports the map in mapping.out format: 21 rows of 49 characters ('I', '-', '|', spaces)."""
    lines, out_of_grid, inconsistent = render_mapping_grid()
    try:
        # newline="\n": same line ending on Windows, Linux, and macOS
        with open(filename, "w", encoding="utf-8", newline="\n") as file:
            file.write("\n".join(lines) + "\n")
        debug_print(f"[EXPORT] Map written to '{filename}' ({len(lines)} rows x {MAPPING_COLS} columns)")
    except OSError as error:
        print(f"[EXPORT] Unable to write '{filename}': {error}")
    if out_of_grid:
        debug_print(f"[EXPORT] WARNING: {out_of_grid} link(s) outside the {MAPPING_ROWS}x{MAPPING_COLS} grid (ignored)")
    if inconsistent:
        debug_print(f"[EXPORT] WARNING: {inconsistent} link(s) at an inconsistent position (parity) -> check logical map")


def move_on_grid(action):
    """Updates the robot's position on the virtual map (all moves are 2 cells)."""
    global grid_x, grid_y, grid_heading, midpoint_markers, blocked_cells, frontier_cells

    # The midpoint ahead of the robot (the junction) is 1 step away
    mid_x, mid_y = step_from_heading(grid_x, grid_y, grid_heading)
    midpoint_markers.add((mid_x, mid_y))

    if action == "LEFT":
        for block_heading in (grid_heading, (grid_heading + 1) % 4):
            bx, by = step_from_heading(mid_x, mid_y, block_heading)
            if (bx, by) not in frontier_cells and (bx, by) not in grid_map and (bx, by) not in midpoint_markers:
                blocked_cells.add((bx, by))

        grid_heading = (grid_heading - 1) % 4
        grid_x, grid_y = step_from_heading(mid_x, mid_y, grid_heading)

    elif action == "RIGHT":
        for block_heading in (grid_heading, (grid_heading - 1) % 4):
            bx, by = step_from_heading(mid_x, mid_y, block_heading)
            if (bx, by) not in frontier_cells and (bx, by) not in grid_map and (bx, by) not in midpoint_markers:
                blocked_cells.add((bx, by))

        grid_heading = (grid_heading + 1) % 4
        grid_x, grid_y = step_from_heading(mid_x, mid_y, grid_heading)

    elif action == "TURN180":
        grid_heading = (grid_heading + 2) % 4

    else:  # "STRAIGHT" action
        grid_x, grid_y = step_from_heading(mid_x, mid_y, grid_heading)

    # Instant update of the new destination cell (excludes 0,0)
    frontier_cells.discard((grid_x, grid_y))
    if (grid_x, grid_y) != (0, 0):
        if (grid_x, grid_y) not in grid_map:
            grid_map[(grid_x, grid_y)] = 1
        elif grid_map[(grid_x, grid_y)] < 999:
            grid_map[(grid_x, grid_y)] += 1

    mark_lateral_corridor_walls(grid_x, grid_y, grid_heading)
    sync_grid_heading_from_compass()


def log_step_status(action_name="", details=""):
    if not DEBUG:
        return
    gps_x, gps_y = get_gps_position()
    display_heading = bearing_to_grid_heading(get_bearing())
    orientations = {0: "North", 1: "East", 2: "South", 3: "West"}
    print(f"\n" + "-" * 70)
    print(f"[STEP #{step_counter:04d}] Action: {action_name}")
    print(f"  Logical Position  : Cell ({grid_x}, {grid_y}) | Facing: {orientations[display_heading]}")

    print("  Virtual map:")
    print(render_map())
    if DEBUG_EXPORT_MAP and action_name == "Decision making":
        print("  Path map ('-' horizontal, '|' vertical):")
        print(render_export_map())
    print(f"  Legend            : R=robot, .=already seen, #=dead end, {FRONTIER_SYMBOL}=unexplored(frontier), {UNEXPLORED_SYMBOL}=empty, 1..9=visits")
    print(f"  Map (grid_map)    : {grid_map}")
    print(f"  Sensors (Real)    : GPS=(X={gps_x:.3f}, Y={gps_y:.3f}) | Compass={get_bearing():.1f}°")
    if details: print(f"  Details           : {details}")
    print("-" * 70)


# -------------------------------------------------------------
# CAMERA IMAGE ANALYSIS
# -------------------------------------------------------------
def analyze_borders(image, width, height):
    """Returns a list of physically open directions with RGB debug info."""
    center_x = int(width / 2)
    scan_rows = [16, 19, 20, 21, 22, 23, 24, 25]   # Multi-scan: 20 then 21, 22, 23 if not detected

    debug_print("\n" + "=" * 70)
    debug_print(f"=== CAMERA ANALYSIS | Current Cell: ({grid_x}, {grid_y}) ===")
    debug_print("=" * 70)

    left_vals, right_vals, straight_vals = [], [], []

    def scan_row(x_range, y):
        """Scans a horizontal row. Returns (detected, max_rgb_values)."""
        detected, vals = True, []
        for x in x_range:
            if 0 <= x < width and 0 <= y < height:
                r = camera.imageGetRed(image, width, x, y)
                g = camera.imageGetGreen(image, width, x, y)
                b = camera.imageGetBlue(image, width, x, y)
                max_rgb = max(r, g, b)
                vals.append(max_rgb)
                if max_rgb > BLACK_RGB_THRESHOLD:
                    detected = False
                    break
        return detected, vals

    # --- LEFT Scan (multi-row) ---
    left_detected, left_debug = False, []
    for y in scan_rows:
        row_detected, row_vals = scan_row(range(11, 0, -1), y)
        left_vals.extend(row_vals)
        left_debug.append(f"y={y}:{'OPEN' if row_detected else 'closed'}")
        if row_detected:
            left_detected = True   # A single black row is enough
            break                  # No need to scan further

    # --- RIGHT Scan (multi-row) ---
    right_detected, right_debug = False, []
    for y in scan_rows:
        row_detected, row_vals = scan_row(range(21, 31), y)
        right_vals.extend(row_vals)
        right_debug.append(f"y={y}:{'OPEN' if row_detected else 'closed'}")
        if row_detected:
            right_detected = True
            break

    # --- STRAIGHT Scan (multi-column: at least 1 valid column is enough) ---
    straight_detected = False

    for x in range(center_x - 6, center_x + 6):
        column_is_black = True
        for y in range(13, 6, -1):
            if 0 <= x < width and 0 <= y < height:
                r = camera.imageGetRed(image, width, x, y)
                g = camera.imageGetGreen(image, width, x, y)
                b = camera.imageGetBlue(image, width, x, y)
                max_rgb = max(r, g, b)
                straight_vals.append(max_rgb)
                if max_rgb > BLACK_RGB_THRESHOLD:
                    column_is_black = False
                    break

        # If at least one vertical column is completely black, the straight path is validated
        if column_is_black:
            straight_detected = True
            break

    available_moves = []
    if straight_detected: available_moves.append("STRAIGHT")
    if left_detected: available_moves.append("LEFT")
    if right_detected: available_moves.append("RIGHT")

    # --- DEBUG OUTPUT ---
    debug_print(f"  [CAMERA DEBUG] Black detection threshold: Must be <= {BLACK_RGB_THRESHOLD}")
    debug_print(f"    - LEFT     : scanned rows [{', '.join(left_debug)}] | "
                f"Max RGB = {max(left_vals) if left_vals else 'N/A'} -> "
                f"{'OPEN' if left_detected else 'CLOSED (Wall detected)'}")
    debug_print(f"    - STRAIGHT : Max scanned = {max(straight_vals) if straight_vals else 'N/A'} -> "
                f"{'OPEN' if straight_detected else 'CLOSED (Wall detected)'}")
    debug_print(f"    - RIGHT    : scanned rows [{', '.join(right_debug)}] | "
                f"Max RGB = {max(right_vals) if right_vals else 'N/A'} -> "
                f"{'OPEN' if right_detected else 'CLOSED (Wall detected)'}")

    return available_moves


# -------------------------------------------------------------
# CONTINUOUS PHYSICAL MOVEMENT FUNCTIONS
# -------------------------------------------------------------
def goStraight(distance=0.5, speed=MAX_SPEED):
    global step_counter
    start_x, start_y = get_gps_position()
    target_heading = get_bearing()
    kp = 0.08

    # Initialize ground sensors
    gs_left = robot.getDevice('gs0')
    gs_center = robot.getDevice('gs1')
    gs_right = robot.getDevice('gs2')
    gs_left.enable(timestep)
    gs_center.enable(timestep)
    gs_right.enable(timestep)

    BLACK_THRESHOLD = 400  # Detection threshold (< 400 = black)

    while robot.step(timestep) != -1:
        step_counter += 1

        v_left = gs_left.getValue()
        v_center = gs_center.getValue()
        v_right = gs_right.getValue()

        is_left_black = v_left < BLACK_THRESHOLD
        is_center_black = v_center < BLACK_THRESHOLD
        is_right_black = v_right < BLACK_THRESHOLD

        correction_state = ""

        # 1. Left sensor touches black (even if center also touches) -> Recenter left
        if is_left_black and not is_right_black:
            left_motor.setVelocity(speed * 0.5)
            right_motor.setVelocity(speed * 0.9)
            correction_state = "RECENTERING LEFT"

        # 2. Right sensor touches black (even if center also touches) -> Recenter right
        elif is_right_black and not is_left_black:
            left_motor.setVelocity(speed * 0.9)
            right_motor.setVelocity(speed * 0.5)
            correction_state = "RECENTERING RIGHT"

        # 3. Only center sensor is on black (or none) -> Classic compass navigation
        else:
            error = get_heading_error(target_heading, get_bearing())
            left_motor.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, speed + (kp * error))))
            right_motor.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, speed - (kp * error))))

            if is_center_black and not is_left_black and not is_right_black:
                correction_state = "PERFECTLY CENTERED"
            else:
                correction_state = "COMPASS ALIGNMENT"

        curr_x, curr_y = get_gps_position()
        traveled = math.sqrt((curr_x - start_x)**2 + (curr_y - start_y)**2)

        # Debug output
        if DEBUG:
            debug_gs = (
                f"GS [L:{v_left:.0f} {'[BLACK]' if is_left_black else '[WHITE]'} | "
                f"C:{v_center:.0f} {'[BLACK]' if is_center_black else '[WHITE]'} | "
                f"R:{v_right:.0f} {'[BLACK]' if is_right_black else '[WHITE]'}] "
                f"-> Status: {correction_state}"
            )
            debug_print(f"  [GROUND SENSORS DEBUG] {debug_gs}")

        log_step_status("goStraight", f"Traveled: {traveled:.3f}m / {distance:.3f}m | {correction_state}")

        if traveled >= distance:
            break

def turn180(forward_distance=0.0, speed=MAX_SPEED):
    global step_counter
    accumulated_angle, previous_heading = 0.0, get_bearing()
    left_motor.setVelocity(-speed * 0.5)
    right_motor.setVelocity(speed * 0.5)
    while robot.step(timestep) != -1:
        step_counter += 1
        current = get_bearing()
        accumulated_angle += abs(get_delta_angle(current, previous_heading))
        previous_heading = current
        remaining = 180.0 - accumulated_angle

        log_step_status("turn180", f"Remaining: {remaining:.2f}°")

        if remaining < 15.0:
            reduced = max(1.0, speed * (remaining / 15.0))
            left_motor.setVelocity(-reduced * 0.5)
            right_motor.setVelocity(reduced * 0.5)
        if accumulated_angle >= 180.0:
            break
    if forward_distance > 0: goStraight(distance=forward_distance, speed=speed)

def goLeft(forward_distance=0.0, speed=MAX_SPEED):
    global step_counter
    accumulated_angle, previous_heading = 0.0, get_bearing()
    left_motor.setVelocity(speed * 0.4)
    right_motor.setVelocity(speed)
    while robot.step(timestep) != -1:
        step_counter += 1
        current = get_bearing()
        accumulated_angle += abs(get_delta_angle(current, previous_heading))
        previous_heading = current

        log_step_status("goLeft", f"Remaining: {88.0 - accumulated_angle:.2f}°")

        if accumulated_angle >= 88.0:
            break
    if forward_distance > 0: goStraight(distance=forward_distance, speed=speed)

def goRight(forward_distance=0.0, speed=MAX_SPEED):
    global step_counter
    accumulated_angle, previous_heading = 0.0, get_bearing()
    left_motor.setVelocity(speed)
    right_motor.setVelocity(speed * 0.4)
    while robot.step(timestep) != -1:
        step_counter += 1
        current = get_bearing()
        accumulated_angle += abs(get_delta_angle(current, previous_heading))
        previous_heading = current

        log_step_status("goRight", f"Remaining: {88.0 - accumulated_angle:.2f}°")

        if accumulated_angle >= 88.0:
            break
    if forward_distance > 0: goStraight(distance=forward_distance, speed=speed)

# -------------------------------------------------------------
# MAIN LOOP: EXPLORATION ALGORITHM (TRÉMAUX)
# -------------------------------------------------------------
robot.step(timestep)

while robot.step(timestep) != -1:
    step_counter += 1

    # Re-sync at the start of each decision cycle.
    sync_grid_heading_from_compass()

    # 1. Update current cell (cell (0,0) is ignored)
    frontier_cells.discard((grid_x, grid_y))
    if (grid_x, grid_y) != (0, 0) and (grid_x, grid_y) not in grid_map:
        grid_map[(grid_x, grid_y)] = 1
    # elif grid_map[(grid_x, grid_y)] < 999:
    #     grid_map[(grid_x, grid_y)] += 1

    log_step_status("Decision making", "Analyzing the next intersection")

    # 2. Capture and Analysis
    image = camera.getImage()
    width = camera.getWidth()
    height = camera.getHeight()
    available_moves = analyze_borders(image, width, height)

    # --- FIX: POST-TURN FILTER ---
    # If the last action was a turn, the robot is still too close to the intersection.
    # Ignore false lateral paths to force it to clear the cell.
    # if len(path_history) > 0 and path_history[-1] in ["LEFT", "RIGHT", "TURN180"]:
    #     if "STRAIGHT" in available_moves:
    #         available_moves = ["STRAIGHT"]

    record_branch_information(available_moves)

    debug_print(f"  -> Physically open paths: {available_moves}")

    chosen_move = None

    # If it's an absolute dead end
    if len(available_moves) == 0:
        debug_print("  -> DEAD END DETECTED. Closing cell.")
        grid_map[(grid_x, grid_y)] = 999  # Marks the cell as a permanent dead end

        # NEW: Explicitly mark front, left, and right as blocked
        for direction_offset in [0, -1, 1]:  # 0=front, -1=left, 1=right
            block_heading = (grid_heading + direction_offset) % 4
            bx, by = step_from_heading(grid_x, grid_y, block_heading)
            blocked_cells.add((bx, by))
            frontier_cells.discard((bx, by)) # In case a '?' got stuck there

        chosen_move = "TURN180"

    # Stop when no '?' frontier remains after scanning the current cell
    if not frontier_cells:
        export_map_to_txt(MAPPING_FILENAME)
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        robot.step(timestep)
        print("End of the mapping")
        break

    if chosen_move is None:
        # Evaluate accessible adjacent cells ahead of us
        move_scores = {}
        for move in available_moves:
            tx, ty = get_target_coord(grid_x, grid_y, grid_heading, move)
            move_scores[move] = grid_map.get((tx, ty), 0)

        # Look for cells with 0 visits (unknown)
        unvisited = [m for m, v in move_scores.items() if v == 0]

        if len(unvisited) > 0:
            # Unknown paths found: Priority Straight > Left > Right
            debug_print(f"  -> Unknown paths found: {unvisited}")
            if "STRAIGHT" in unvisited: chosen_move = "STRAIGHT"
            elif "LEFT" in unvisited: chosen_move = "LEFT"
            else: chosen_move = unvisited[0]

        else:
            # Backtracking
            debug_print("  -> Everything ahead is known. Launching BFS to nearest frontier...")

            chosen_move = plan_to_nearest_frontier(available_moves)
            if chosen_move is not None:
                debug_print(f"  -> BFS: next move = {chosen_move}")
            else:
                debug_print("  -> BFS: no path to a frontier. Falling back to visit-count selection (Trémaux).")

            bx, by = get_target_coord(grid_x, grid_y, grid_heading, "TURN180")
            move_scores["TURN180"] = grid_map.get((bx, by), 0)

            min_visits = min(move_scores.values())

            if min_visits >= 999:
                grid_map[(grid_x, grid_y)] = 999

            best_moves = [m for m, v in move_scores.items() if v == min_visits]

            if chosen_move is None:
                if "TURN180" in best_moves: chosen_move = "TURN180"
                elif "STRAIGHT" in best_moves: chosen_move = "STRAIGHT"
                elif "LEFT" in best_moves: chosen_move = "LEFT"
                else: chosen_move = best_moves[0]

    # 3. Validation and Recording
    debug_print(f"\n>>> FINAL DECISION: '{chosen_move}'")
    path_history.append(chosen_move)

    # 4. Immediate logical update to reflect the new position during movement
    move_on_grid(chosen_move)

    # 5. Physical movement execution (with adaptive distance for STRAIGHT)
    if chosen_move == "STRAIGHT":
        if is_first_movement:
            distance_to_drive = FIRST_STRAIGHT_DISTANCE
            debug_print(f"  -> First straight movement: distance = {distance_to_drive} m")
            is_first_movement = False
        else:
            distance_to_drive = DEFAULT_STRAIGHT_DISTANCE
            debug_print(f"  -> Subsequent straight movement: distance = {distance_to_drive} m")

        goStraight(distance=distance_to_drive)

    elif chosen_move == "LEFT":
        is_first_movement = False
        goLeft(forward_distance=POST_TURN_DISTANCE)

    elif chosen_move == "RIGHT":
        is_first_movement = False
        goRight(forward_distance=POST_TURN_DISTANCE)

    elif chosen_move == "TURN180":
        is_first_movement = False
        turn180(forward_distance=POST_TURN_DISTANCE)

    export_map_to_txt(MAPPING_FILENAME)