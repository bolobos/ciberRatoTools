"""my_controller controller."""

from controller import Robot

# -------------------------------------------------------------------------
# Config
# -------------------------------------------------------------------------

robot = Robot()
timeStep = int(robot.getBasicTimeStep())

# Constants
cruiseVelocity = 6.28
black_rgb_threshold = 120  # For the camera (RGB < 120)
black_gs_threshold = 400   # For the ground sensors (< 400 = Black)
white_threshold = 500      # For the ground sensors (> 500 = White)
turn_buffer_limit = 15     # Minimum number of steps to ignore at the start of a turn
turn_threshold = 14 

# Motors
leftMotor = robot.getDevice("left wheel motor")
rightMotor = robot.getDevice("right wheel motor")
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

# Camera
camera = robot.getDevice("camera")
camera.enable(timeStep)

# Ground sensors
ground_sensors = [robot.getDevice('gs' + str(x)) for x in range(3)]
for gs in ground_sensors:
    gs.enable(timeStep)

# -------------------------------------------------------------------------
# Different states
# -------------------------------------------------------------------------
STATE_GO_FORWARD    = "GO_FORWARD"
STATE_CORRECT_LEFT  = "CORRECT_LEFT"    
STATE_CORRECT_RIGHT = "CORRECT_RIGHT"   
STATE_TURN_LEFT     = "TURN_LEFT"
STATE_TURN_RIGHT    = "TURN_RIGHT"
STATE_FULL_ROTATION = "FULL_ROTATION"

error = 0
current_state = STATE_GO_FORWARD
rotation_counter = 0

# -------------------------------------------------------------------------
# Analyze (Colors / Edges)
# -------------------------------------------------------------------------
# Function that analyzes colors on both the left and right sides of the central axis
def analyze_borders(image, width, height):
    centre_x = int(width / 2)
    #print(f"[DEBUG] Starting analyze_borders | width: {width}, height: {height}, centre_x: {centre_x}")

    # Set initial scan position to start near the bottom of the image (y = height - 2)
    start_y = max(height - 2, 21)

    # Vertical loop to navigate through y-coordinates down to y = 21 (stops after 21)
    for y in range(start_y, 20, -1):

        # Matrix to store RGB values for 3 consecutive vertical pixels at centre_x
        rgb_center_3x3 = []

        for offset_y in (-1, 0, 1):
            check_y = y + offset_y
            if 0 <= check_y < height:
                # Retrieve RGB channels from Webots camera API
                r_val = camera.imageGetRed(image, width, centre_x, check_y)
                g_val = camera.imageGetGreen(image, width, centre_x, check_y)
                b_val = camera.imageGetBlue(image, width, centre_x, check_y)
                rgb_center_3x3.append([r_val, g_val, b_val])
            else:
                rgb_center_3x3.append([0, 0, 0])

        # Ignore rows where the central vertical block is not strictly black
        if (rgb_center_3x3[1][0] > black_rgb_threshold or rgb_center_3x3[1][1] > black_rgb_threshold or rgb_center_3x3[1][2] > black_rgb_threshold) or \
           (rgb_center_3x3[0][0] > black_rgb_threshold or rgb_center_3x3[0][1] > black_rgb_threshold or rgb_center_3x3[0][2] > black_rgb_threshold) or \
           (rgb_center_3x3[2][0] > black_rgb_threshold or rgb_center_3x3[2][1] > black_rgb_threshold or rgb_center_3x3[2][2] > black_rgb_threshold):
            #print(f"[DEBUG] y={y}: Skipping row, center column (x={centre_x}) is not black.")
            continue

        #print(f"[DEBUG] y={y}: Center column is black. Scanning left and right from x={centre_x}...")

        # -----------------------------------------------------------------
        # 1. LEFT SIDE SCAN (from center_x down to 0)
        # -----------------------------------------------------------------
        for x in range(centre_x, 0, -1):
            r = camera.imageGetRed(image, width, x, y)
            g = camera.imageGetGreen(image, width, x, y)
            b = camera.imageGetBlue(image, width, x, y)

            is_not_black = (r > black_rgb_threshold or g > black_rgb_threshold or b > black_rgb_threshold)

            if is_not_black:
                #print(f"[DEBUG] [LEFT] Found non-black pixel at (x={x}, y={y}) -> RGB: ({r}, {g}, {b})")

                # Yellow detection condition
                if r > 150 and g > 150 and b < 100:
                    #print(f"[DEBUG] [LEFT] Match -> YELLOW detected at (x={x}, y={y}) with RGB ({r}, {g}, {b})")
                    return "YELLOW"
                # White line stop condition
                elif r > 180 and g > 180 and b > 180:
                    #print(f"[DEBUG] [LEFT] Match -> WHITE line detected at (x={x}, y={y}). Stopping left scan.")
                    break
                # Green detection condition
                elif g > 150 and r < 100:
                    #print(f"[DEBUG] [LEFT] Match -> GREEN detected at (x={x}, y={y}) with RGB ({r}, {g}, {b})")
                    return "GREEN"
                else:
                    pass
                    #print(f"[DEBUG] [LEFT] Non-black pixel at (x={x}, y={y}) did not match YELLOW, WHITE, or GREEN.")

        # -----------------------------------------------------------------
        # 2. RIGHT SIDE SCAN (from center_x up to width - 1)
        # -----------------------------------------------------------------
        for x in range(centre_x, width):
            r = camera.imageGetRed(image, width, x, y)
            g = camera.imageGetGreen(image, width, x, y)
            b = camera.imageGetBlue(image, width, x, y)

            is_not_black = (r > black_rgb_threshold or g > black_rgb_threshold or b > black_rgb_threshold)

            if is_not_black:
                #print(f"[DEBUG] [RIGHT] Found non-black pixel at (x={x}, y={y}) -> RGB: ({r}, {g}, {b})")

                # Yellow detection condition
                if r > 150 and g > 150 and b < 100:
                    #print(f"[DEBUG] [RIGHT] Match -> YELLOW detected at (x={x}, y={y}) with RGB ({r}, {g}, {b})")
                    return "YELLOW"
                # White line stop condition
                elif r > 180 and g > 180 and b > 180:
                    #print(f"[DEBUG] [RIGHT] Match -> WHITE line detected at (x={x}, y={y}). Stopping right scan.")
                    break
                # Green detection condition
                elif g > 150 and r < 100:
                    #print(f"[DEBUG] [RIGHT] Match -> GREEN detected at (x={x}, y={y}) with RGB ({r}, {g}, {b})")
                    return "GREEN"
                else:
                    pass
                    #print(f"[DEBUG] [RIGHT] Non-black pixel at (x={x}, y={y}) did not match YELLOW, WHITE, or GREEN.")

    #print("[DEBUG] analyze_borders finished: No matching border color found (returning NONE).")
    return "NONE"
# -------------------------------------------------------------------------
# Turn Detection via Camera (Preshot)
# -------------------------------------------------------------------------
def check_camera_turns(image, width, height):
    y_horiz = 21  
    x_vert = int(width / 2)    
    
    black_left = 0
    black_right = 0

    # Check between center and each side to see if we have several balck pixels
    for x in range(0, x_vert):
        r = camera.imageGetRed(image, width, x, y_horiz)
        g = camera.imageGetGreen(image, width, x, y_horiz)
        b = camera.imageGetBlue(image, width, x, y_horiz)
        if (r < black_rgb_threshold and g < black_rgb_threshold and b < black_rgb_threshold):
            black_left += 1

    for x in range(x_vert, width):
        r = camera.imageGetRed(image, width, x, y_horiz)
        g = camera.imageGetGreen(image, width, x, y_horiz)
        b = camera.imageGetBlue(image, width, x, y_horiz)
        if (r < black_rgb_threshold and g < black_rgb_threshold and b < black_rgb_threshold):
            black_right += 1

    return black_left, black_right


# -------------------------------------------------------------------------
# Actions 
# -------------------------------------------------------------------------
# Mainly settings of velocity for each actions with motors
# Tuning possible (normally already optimize)
def action_go_forward():
    leftMotor.setVelocity(cruiseVelocity)
    rightMotor.setVelocity(cruiseVelocity)

def action_correct_left():
    leftMotor.setVelocity(0.1 * cruiseVelocity)  
    rightMotor.setVelocity(0.67 * cruiseVelocity) 

def action_correct_right():
    leftMotor.setVelocity(0.67 * cruiseVelocity)  
    rightMotor.setVelocity(0.1 * cruiseVelocity) 

def action_turn_left():
    leftMotor.setVelocity(0.3 * cruiseVelocity)
    rightMotor.setVelocity(0.67 * cruiseVelocity)

def action_turn_right():
    leftMotor.setVelocity(0.67 * cruiseVelocity)
    rightMotor.setVelocity(0.3 * cruiseVelocity)

def action_full_rotation():
    leftMotor.setVelocity(0.4 * cruiseVelocity)
    rightMotor.setVelocity(-0.4 * cruiseVelocity)


# -------------------------------------------------------------------------
# 4. LOOP
# -------------------------------------------------------------------------
while robot.step(timeStep) != -1:

    # Get image at each timestep
    image = camera.getImage()
    if image is None:
        continue
        
    # print(error)

    # 1. READ GROUND SENSORS
    gs_left   = ground_sensors[0].getValue()
    gs_center = ground_sensors[1].getValue()
    gs_right  = ground_sensors[2].getValue()
    
    # 2. READ CAMERA
    b_left, b_right = check_camera_turns(image, camera.getWidth(), camera.getHeight())

    # --- DEBUGGING ---
    # print("\n---------------------------------------------------")
    # print(f"CURRENT STATE : {current_state} | Timer: {rotation_counter}")
    # print(f"[CAMERA (Far)]   Black Pixels -> Left: {b_left:2d}/16 | Right: {b_right:2d}/16")
    
    # Conditions to detect the alignment with ground sensors
    gs_L_black = gs_left < black_gs_threshold
    gs_C_black = gs_center < black_gs_threshold
    gs_R_black = gs_right < black_gs_threshold
    is_aligned = (not gs_L_black) and gs_C_black and (not gs_R_black)
    
    # print(f"[GROUND (Present)] Raw Values   -> L: {gs_left:.0f} | C: {gs_center:.0f} | R: {gs_right:.0f}")
    # print(f"                   See Black ?  -> L: {gs_L_black} | C: {gs_C_black} | R: {gs_R_black} | PERFECTLY ALIGNED : {is_aligned}")

    # Yellow / Green Detection 
    if error == 0:
        detected_color = analyze_borders(image, camera.getWidth(), camera.getHeight())
        if detected_color == "YELLOW":
            #print(">>> STATE CHANGE: YELLOW -> FULL_ROTATION")
            error = 2
            current_state = STATE_FULL_ROTATION
            rotation_counter = 0
        elif detected_color == "GREEN":
            error = 1

    # ---------------------------------------------------------------------
    # Transitions between states
    # ---------------------------------------------------------------------

    # ROTATION : use a counter first then ground sensor alignment
    if current_state == STATE_FULL_ROTATION:
        rotation_counter += 1
        if rotation_counter > 20 and is_aligned:
            #print(">>> STATE CHANGE: END ROTATION -> GO_FORWARD")
            current_state = STATE_GO_FORWARD
            rotation_counter = 0
            
    # TURN LEFT : counter then -> already new turn detected or alignment
    elif current_state == STATE_TURN_LEFT:
        rotation_counter += 1  
        if rotation_counter > turn_buffer_limit: 
            # Check the camera as absolute priority
            if b_left >= turn_threshold:
                #print(">>> RAPID SEQUENCE : LEFT TURN -> NEW TURN_LEFT")
                current_state = STATE_TURN_LEFT
                rotation_counter = 0
            elif b_right >= turn_threshold:
                #print(">>> RAPID SEQUENCE : LEFT TURN -> NEW TURN_RIGHT")
                current_state = STATE_TURN_RIGHT
                rotation_counter = 0
            # If no new turn, wait for alignment to go straight
            elif is_aligned:
                #print(">>> STATE CHANGE: END LEFT TURN -> GO_FORWARD")
                current_state = STATE_GO_FORWARD
                rotation_counter = 0
            
    # TURN RIGHT : same as left 
    elif current_state == STATE_TURN_RIGHT:
        rotation_counter += 1  
        if rotation_counter > turn_buffer_limit:
            # Check the camera as absolute priority
            if b_left >= turn_threshold:
                #print(">>> RAPID SEQUENCE : RIGHT TURN -> NEW TURN_LEFT")
                current_state = STATE_TURN_LEFT
                rotation_counter = 0
            elif b_right >= turn_threshold:
                #print(">>> RAPID SEQUENCE : RIGHT TURN -> NEW TURN_RIGHT")
                current_state = STATE_TURN_RIGHT
                rotation_counter = 0
            # If no new turn, wait for alignment to go straight
            elif is_aligned:
                #print(">>> STATE CHANGE: END RIGHT TURN -> GO_FORWARD")
                current_state = STATE_GO_FORWARD
                rotation_counter = 0

    # DEFAULT BEHAVIOR : when more forward for example
    else:
        # Camera Preshot
        if b_left >= turn_threshold:
            #print(">>> STATE CHANGE: CAMERA DETECTS LEFT TURN -> TURN_LEFT")
            current_state = STATE_TURN_LEFT
            rotation_counter = 0  
        elif b_right >= turn_threshold:
            #print(">>> STATE CHANGE: CAMERA DETECTS RIGHT TURN -> TURN_RIGHT")
            current_state = STATE_TURN_RIGHT
            rotation_counter = 0  
            
        # Line Following
        elif gs_L_black and not gs_R_black:
            if current_state != STATE_CORRECT_LEFT: 
                #print(">>> STATE CHANGE: DRIFT DETECTED -> CORRECT_LEFT")
                pass
            current_state = STATE_CORRECT_LEFT
            
        elif gs_R_black and not gs_L_black:
            if current_state != STATE_CORRECT_RIGHT: 
                #print(">>> STATE CHANGE: DRIFT DETECTED -> CORRECT_RIGHT")
                pass
            current_state = STATE_CORRECT_RIGHT
            
        # Everything is centered
        else:
            if current_state != STATE_GO_FORWARD: 
                #print(">>> STATE CHANGE: ALIGNED -> GO_FORWARD")
                pass
            current_state = STATE_GO_FORWARD

    # ---------------------------------------------------------------------
    # Actions for each state
    # ---------------------------------------------------------------------
    match current_state:
        case "GO_FORWARD":
            action_go_forward()
        case "CORRECT_LEFT":
            action_correct_left()
        case "CORRECT_RIGHT":
            action_correct_right()
        case "TURN_LEFT":
            action_turn_left()
        case "TURN_RIGHT":
            action_turn_right()
        case "FULL_ROTATION":
            action_full_rotation()
        case _:
            action_go_forward()