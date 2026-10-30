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









# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    
    print("customData",robot.getCustomData())

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    frame = camera.getImage()
    print('GPS', gps.getValues(),end="")
    print('compass', compass.getValues())

# Enter here exit cleanup code.
