"""
Contrôleur Webots (C2_pClient.py) avec Algorithme de Trémaux et Distances Adaptatives.
1. Premier mouvement tout droit : FIRST_STRAIGHT_DISTANCE (0.082 m).
2. Mouvements tout droit suivants : DEFAULT_STRAIGHT_DISTANCE (0.140 m).
3. Cartographie virtuelle (X, Y) avec gestion des impasses, backtracking et affichage continu.
"""

import math
from controller import Robot

# -------------------------------------------------------------
# CONSTANTES & CONFIGURATION
# -------------------------------------------------------------
robot = Robot()
timestep = int(robot.getBasicTimeStep())

MAX_SPEED = 6.28
BLACK_RGB_THRESHOLD = 120

# Paramètres de distance (en mètres)
FIRST_STRAIGHT_DISTANCE = 0.082    # Distance uniquement pour le tout premier mouvement tout droit
DEFAULT_STRAIGHT_DISTANCE = 0.140  # Distance plus élevée pour les lignes droites suivantes
POST_TURN_DISTANCE = 0.016         # Distance d'avancée après un virage

step_counter = 0
is_first_movement = True           # Drapeau pour identifier le premier déplacement
path_history = []

# =============================================================
# VARIABLES DE CARTOGRAPHIE (MAP VIRTUELLE)
# =============================================================
grid_map = {}      # Dictionnaire {(x, y): nombre_de_visites}
grid_x = 0         # Position logique X
grid_y = 0         # Position logique Y
grid_heading = 0   # Cap logique : 0=Nord, 1=Est, 2=Sud, 3=Ouest

# Moteurs
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Capteurs
gps = robot.getDevice('gps')
gps.enable(timestep)
camera = robot.getDevice('camera')
camera.enable(timestep)
compass = robot.getDevice('compass')
compass.enable(timestep)


# -------------------------------------------------------------
# OUTILS DE NAVIGATION ET COORDONNÉES
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

def get_target_coord(x, y, heading, action):
    """Calcule les coordonnées (X, Y) de la case cible selon l'action."""
    new_h = heading
    if action == "LEFT": new_h = (heading - 1) % 4
    elif action == "RIGHT": new_h = (heading + 1) % 4
    elif action == "TURN180": new_h = (heading + 2) % 4

    if new_h == 0: return x, y + 1
    elif new_h == 1: return x + 1, y
    elif new_h == 2: return x, y - 1
    elif new_h == 3: return x - 1, y

def move_on_grid(action):
    """Met à jour la position du robot sur la carte virtuelle."""
    global grid_x, grid_y, grid_heading
    if action == "LEFT": grid_heading = (grid_heading - 1) % 4
    elif action == "RIGHT": grid_heading = (grid_heading + 1) % 4
    elif action == "TURN180": grid_heading = (grid_heading + 2) % 4
    
    if grid_heading == 0: grid_y += 1
    elif grid_heading == 1: grid_x += 1
    elif grid_heading == 2: grid_y -= 1
    elif grid_heading == 3: grid_x -= 1

def log_step_status(action_name="", details=""):
    gps_x, gps_y = get_gps_position()
    orientations = {0: "Nord", 1: "Est", 2: "Sud", 3: "Ouest"}
    print(f"\n" + "-" * 70)
    print(f"[STEP #{step_counter:04d}] Action: {action_name}")
    print(f"  Position Logique  : Case ({grid_x}, {grid_y}) | Orienté: {orientations[grid_heading]}")
    print(f"  Carte (grid_map)  : {grid_map}")
    print(f"  Capteurs (Réels)  : GPS=(X={gps_x:.3f}, Y={gps_y:.3f}) | Boussole={get_bearing():.1f}°")
    if details: print(f"  Détails           : {details}")
    print("-" * 70)


# -------------------------------------------------------------
# ANALYSE D'IMAGE CAMÉRA
# -------------------------------------------------------------
def analyze_borders(image, width, height):
    """Retourne une liste des directions physiquement ouvertes (ex: ['LEFT', 'STRAIGHT'])."""
    center_x = int(width / 2)
    y_scan = 21
    
    print("\n" + "=" * 70)
    print(f"=== ANALYSE CAMÉRA | Case Actuelle : ({grid_x}, {grid_y}) ===")
    print("=" * 70)

    # Scan Gauche
    left_detected = True
    for x in range(11, 4, -1):
        if 0 <= x < width and 0 <= y_scan < height:
            r, g, b = camera.imageGetRed(image, width, x, y_scan), camera.imageGetGreen(image, width, x, y_scan), camera.imageGetBlue(image, width, x, y_scan)
            if r > BLACK_RGB_THRESHOLD or g > BLACK_RGB_THRESHOLD or b > BLACK_RGB_THRESHOLD:
                left_detected = False
                break

    # Scan Droit
    right_detected = True
    for x in range(21, 28):
        if 0 <= x < width and 0 <= y_scan < height:
            r, g, b = camera.imageGetRed(image, width, x, y_scan), camera.imageGetGreen(image, width, x, y_scan), camera.imageGetBlue(image, width, x, y_scan)
            if r > BLACK_RGB_THRESHOLD or g > BLACK_RGB_THRESHOLD or b > BLACK_RGB_THRESHOLD:
                right_detected = False
                break

    # Scan Tout Droit
    straight_detected = True
    for y in range(13, 6, -1):
        for x in range(center_x - 1, center_x + 2):
            if 0 <= x < width and 0 <= y < height:
                r, g, b = camera.imageGetRed(image, width, x, y), camera.imageGetGreen(image, width, x, y), camera.imageGetBlue(image, width, x, y)
                if r > BLACK_RGB_THRESHOLD or g > BLACK_RGB_THRESHOLD or b > BLACK_RGB_THRESHOLD:
                    straight_detected = False
                    break
        if not straight_detected:
            break

    available_moves = []
    if straight_detected: available_moves.append("STRAIGHT")
    if left_detected: available_moves.append("LEFT")
    if right_detected: available_moves.append("RIGHT")
    
    return available_moves


# -------------------------------------------------------------
# FONCTIONS DE DÉPLACEMENT PHYSIQUE CONTINU
# -------------------------------------------------------------
def goStraight(distance=0.5, speed=MAX_SPEED):
    global step_counter
    start_x, start_y = get_gps_position()
    target_heading = get_bearing()
    kp = 0.08
    while robot.step(timestep) != -1:
        step_counter += 1
        error = get_heading_error(target_heading, get_bearing())
        left_motor.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, speed + (kp * error))))
        right_motor.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, speed - (kp * error))))
        curr_x, curr_y = get_gps_position()
        traveled = math.sqrt((curr_x - start_x)**2 + (curr_y - start_y)**2)
        
        log_step_status("goStraight", f"Parcouru: {traveled:.3f}m / {distance:.3f}m")
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
        
        log_step_status("turn180", f"Reste: {remaining:.2f}°")
        
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
        
        log_step_status("goLeft", f"Reste: {88.0 - accumulated_angle:.2f}°")
        
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
        
        log_step_status("goRight", f"Reste: {88.0 - accumulated_angle:.2f}°")
        
        if accumulated_angle >= 88.0:
            break
    if forward_distance > 0: goStraight(distance=forward_distance, speed=speed)


# -------------------------------------------------------------
# BOUCLE PRINCIPALE : ALGORITHME D'EXPLORATION (TRÉMAUX)
# -------------------------------------------------------------
robot.step(timestep)

while robot.step(timestep) != -1:
    step_counter += 1

    # 1. Mise à jour de la case actuelle
    if (grid_x, grid_y) not in grid_map:
        grid_map[(grid_x, grid_y)] = 1
    elif grid_map[(grid_x, grid_y)] < 999:
        grid_map[(grid_x, grid_y)] += 1

    log_step_status("Prise de décision", "Analyse de la prochaine intersection")

    # 2. Capture et Analyse
    image = camera.getImage()
    width = camera.getWidth()
    height = camera.getHeight()
    available_moves = analyze_borders(image, width, height)

    print(f"  -> Voies physiquement ouvertes : {available_moves}")

    chosen_move = None

    # Si c'est un cul-de-sac absolu
    if len(available_moves) == 0:
        print("  -> IMPASSE DÉTECTÉE. Fermeture de la case.")
        grid_map[(grid_x, grid_y)] = 999  # Marque la case comme impasse définitive
        chosen_move = "TURN180"

    else:
        # Évaluation des cases adjacentes accessibles devant nous
        move_scores = {}
        for move in available_moves:
            tx, ty = get_target_coord(grid_x, grid_y, grid_heading, move)
            move_scores[move] = grid_map.get((tx, ty), 0)

        # Cherche les cases avec 0 visite (inconnues)
        unvisited = [m for m, v in move_scores.items() if v == 0]

        if len(unvisited) > 0:
            # Voies inconnues trouvées : Priorité Tout droit > Gauche > Droite
            print(f"  -> Voies inconnues trouvées : {unvisited}")
            if "STRAIGHT" in unvisited: chosen_move = "STRAIGHT"
            elif "LEFT" in unvisited: chosen_move = "LEFT"
            else: chosen_move = unvisited[0]
            
        else:
            # Backtracking
            print("  -> Partout est connu devant. Évaluation du backtracking...")
            
            bx, by = get_target_coord(grid_x, grid_y, grid_heading, "TURN180")
            move_scores["TURN180"] = grid_map.get((bx, by), 0)
            
            min_visits = min(move_scores.values())
            
            if min_visits >= 999:
                grid_map[(grid_x, grid_y)] = 999

            best_moves = [m for m, v in move_scores.items() if v == min_visits]
            
            if "TURN180" in best_moves: chosen_move = "TURN180"
            elif "STRAIGHT" in best_moves: chosen_move = "STRAIGHT"
            elif "LEFT" in best_moves: chosen_move = "LEFT"
            else: chosen_move = best_moves[0]

    # 3. Validation et Enregistrement
    print(f"\n>>> DÉCISION FINALE : '{chosen_move}'")
    path_history.append(chosen_move)

    # 4. Exécution du mouvement physique (avec distance adaptative pour STRAIGHT)
    if chosen_move == "STRAIGHT":
        if is_first_movement:
            distance_to_drive = FIRST_STRAIGHT_DISTANCE
            print(f"  -> Premier mouvement tout droit : distance = {distance_to_drive} m")
            is_first_movement = False
        else:
            distance_to_drive = DEFAULT_STRAIGHT_DISTANCE
            print(f"  -> Mouvement tout droit suivant : distance = {distance_to_drive} m")
        
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

    # 5. Mise à jour de la grille pour le prochain pas de temps
    move_on_grid(chosen_move)