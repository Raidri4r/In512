__author__ = "Aybuke Ozturk Suri, Johvany Gustave"
__copyright__ = "Copyright 2023, IN512, IPSA 2024"
__credits__ = ["Aybuke Ozturk Suri", "Johvany Gustave"]
__license__ = "Apache License 2.0"
__version__ = "1.0.0"

from network import Network
from my_constants import *

from threading import Thread
import numpy as np
from time import sleep


class Agent:
    """ Class that implements the behaviour of each agent based on their perception and communication with other agents """
    def __init__(self, server_ip):
        #TODO: DEINE YOUR ATTRIBUTES HERE
        self.my_key = () #store coordinates of the agent key
        self.my_chest = () #same for the chest
        self.mission_completed = 0 #Nb of agent that have got their key and reach the chest to know when the game is done
        self.map = None
        self.direction = None
        self.vertical_direction = 'down'
        self.min_border_distance = 2
        self.vertical_step = 5
        self.state = 'init'
        self.vertical_moves_remaining = 0
        self.cell_val = 0
        self.waiting_for_move = False

        # Suivi des items découverts par tous les agents
        self.all_keys = {}  # {agent_id: (x, y)}
        self.all_boxes = {}  # {agent_id: (x, y)}
        self.has_collected_my_key = False  # True si l'agent est passé sur SA clé
        self.known_walls = set()  # Ensemble des positions de murs connus

        # Mode contournement de mur
        self.bypass_mode = False  # True quand on contourne un mur
        self.bypass_original_direction = None  # Direction horizontale avant le mur
        self.bypass_steps = 0  # Nombre de pas verticaux faits pour contourner

        #DO NOT TOUCH THE FOLLOWING INSTRUCTIONS
        self.network = Network(server_ip=server_ip)
        self.agent_id = self.network.id
        self.running = True
        self.network.send({"header": GET_DATA})
        self.msg = {}
        env_conf = self.network.receive()
        self.nb_agent_expected = None
        self.nb_agent_connected = 0
        self.x, self.y = env_conf["x"], env_conf["y"]   #initial agent position
        self.w, self.h = env_conf["w"], env_conf["h"]   #environment dimensions
        cell_val = env_conf["cell_val"] #value of the cell the agent is located in
        self.map = np.full((self.h, self.w), -1.0)  # Initialiser map avec -1
        print(cell_val)
        Thread(target=self.msg_cb, daemon=True).start()
        print("hello")
        self.wait_for_connected_agent()

        
    def msg_cb(self): 
        """ Method used to handle incoming messages """
        while self.running:
            msg = self.network.receive()
            self.msg = msg
            if msg["header"] == MOVE:
                self.x, self.y =  msg["x"], msg["y"]
                self.cell_val = msg["cell_val"]
                self.waiting_for_move = False  
            elif msg["header"] == GET_NB_AGENTS:
                self.nb_agent_expected = msg["nb_agents"]
            elif msg["header"] == GET_NB_CONNECTED_AGENTS:
                self.nb_agent_connected = msg["nb_connected_agents"]
            elif msg["header"] == BROADCAST_MSG:
                owner = msg['owner']
                pos = msg['position']
                if msg['Msg type'] == KEY_DISCOVERED:
                    # Ne marquer que si c'est un nouvel item
                    if owner not in self.all_keys:
                        self.all_keys[owner] = pos
                        self.mark_item_gradient_on_map(pos, is_key=True)
                        print(f"Key of agent {owner} was found at {pos}")
                    if owner == self.agent_id:
                        self.my_key = pos
                elif msg['Msg type'] == BOX_DISCOVERED:
                    # Ne marquer que si c'est un nouvel item
                    if owner not in self.all_boxes:
                        self.all_boxes[owner] = pos
                        self.mark_item_gradient_on_map(pos, is_key=False)
                        print(f"Box of agent {owner} was found at {pos}")
                    if owner == self.agent_id:
                        self.my_chest = pos
                elif msg['Msg type'] == COMPLETED:
                    self.mission_completed += 1
                    print(f"Agent {owner} completed their mission!")
                elif msg['Msg type'] == WALL_DISCOVERED:
                    # Marquer le mur si pas encore connu
                    if pos not in self.known_walls:
                        self.mark_wall_on_map(pos)
                        print(f"Wall discovered at {pos}")
            elif msg['header'] == GET_DATA:
                self.map = np.full((msg['h'], msg['w']), -1.0)  # -1 = non exploré
                
            elif msg['header'] == GET_ITEM_OWNER:
                owner = msg['owner']
                pos = (self.x, self.y)
                is_key = (self.state == 'on_key')

                # Ajouter à son propre tracking (le broadcast ne revient pas à l'émetteur)
                if is_key:
                    if owner not in self.all_keys:
                        self.all_keys[owner] = pos
                        print(f"Agent {self.agent_id} - J'ai trouvé la clé de l'agent {owner} à {pos}")
                    if owner == self.agent_id:
                        self.my_key = pos
                        self.has_collected_my_key = True  # Je suis sur MA clé, elle est collectée!
                        print(f"Agent {self.agent_id} - MA CLÉ COLLECTÉE!")
                else:
                    if owner not in self.all_boxes:
                        self.all_boxes[owner] = pos
                        print(f"Agent {self.agent_id} - J'ai trouvé la boîte de l'agent {owner} à {pos}")
                    if owner == self.agent_id:
                        self.my_chest = pos

                # Envoyer broadcast aux autres agents
                cmds = {"header": BROADCAST_MSG}
                cmds["Msg type"] = KEY_DISCOVERED if is_key else BOX_DISCOVERED
                cmds["position"] = pos
                cmds["owner"] = owner
                self.network.send(cmds)

            if msg['header'] != MOVE:
                print(f"Received by {self.agent_id}: {msg}")
      
            

    def wait_for_connected_agent(self):
        self.network.send({"header": GET_NB_AGENTS})
        check_conn_agent = True
        while check_conn_agent:
            self.network.send({"header": GET_NB_CONNECTED_AGENTS})
            sleep(0.5)
            if self.nb_agent_expected and self.nb_agent_expected == self.nb_agent_connected:
                print("both connected!")
                check_conn_agent = False

                  

    #TODO: CREATE YOUR METHODS HERE...

    def init_map(self):
        """Initialise la map avec -1 partout (non exploré)."""
        if self.map is None:
            self.map = np.full((self.h, self.w), -1.0)

    def mark_item_gradient_on_map(self, item_pos, is_key=True):
        """
        Marque le rayonnement d'un item découvert sur la map.
        Cela permet d'ignorer ces zones lors de l'exploration.
        """
        if self.map is None:
            self.init_map()

        x, y = item_pos
        # Valeurs de gradient selon le type d'item
        if is_key:
            center_val = 1.0
            close_val = KEY_NEIGHBOUR_PERCENTAGE  # 0.5
            far_val = KEY_NEIGHBOUR_PERCENTAGE / 2  # 0.25
        else:
            center_val = 1.0
            close_val = BOX_NEIGHBOUR_PERCENTAGE  # 0.6
            far_val = BOX_NEIGHBOUR_PERCENTAGE / 2  # 0.3

        # Marquer le centre (position exacte de l'item)
        if 0 <= y < self.h and 0 <= x < self.w:
            self.map[y, x] = center_val

        # Marquer les voisins proches (rayon 1)
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                nx, ny = x + dx, y + dy
                if 0 <= ny < self.h and 0 <= nx < self.w:
                    self.map[ny, nx] = close_val

        # Marquer les voisins éloignés (rayon 2)
        for dx in [-2, -1, 0, 1, 2]:
            for dy in [-2, -1, 0, 1, 2]:
                if abs(dx) <= 1 and abs(dy) <= 1:
                    continue  # Déjà marqué
                nx, ny = x + dx, y + dy
                if 0 <= ny < self.h and 0 <= nx < self.w:
                    if self.map[ny, nx] == -1:  # Ne pas écraser une valeur existante
                        self.map[ny, nx] = far_val

        print(f"Agent {self.agent_id} - Rayonnement marqué pour item à {item_pos}")

    def mark_wall_on_map(self, wall_pos):
        """
        Marque un mur et son rayonnement sur la map.
        Les murs ont un gradient de 0.35 autour d'eux.
        """
        if self.map is None:
            self.init_map()

        x, y = wall_pos
        self.known_walls.add(wall_pos)

        # Marquer le centre (position exacte du mur) avec une valeur spéciale (2.0 pour mur)
        if 0 <= y < self.h and 0 <= x < self.w:
            self.map[y, x] = 2.0  # 2.0 = mur (différent de 1.0 pour items)

        # Marquer les voisins proches (rayon 1) avec le gradient de mur
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                nx, ny = x + dx, y + dy
                if 0 <= ny < self.h and 0 <= nx < self.w:
                    # Ne pas écraser un mur ou un item
                    if self.map[ny, nx] == -1:
                        self.map[ny, nx] = WALL_NEIGHBOUR_PERCENTAGE  # 0.35

        print(f"Agent {self.agent_id} - Mur marqué à {wall_pos}")

    def broadcast_wall(self, wall_pos):
        """Envoie un broadcast pour informer les autres agents d'un mur découvert."""
        cmds = {"header": BROADCAST_MSG}
        cmds["Msg type"] = WALL_DISCOVERED
        cmds["position"] = wall_pos
        cmds["owner"] = self.agent_id
        self.network.send(cmds)

    def is_wall(self, x, y):
        """Vérifie si une position est un mur connu ou une zone de gradient de mur."""
        if self.map is None:
            return False
        if 0 <= y < self.h and 0 <= x < self.w:
            val = self.map[y, x]
            # 2.0 = mur, 0.35 = gradient de mur (à éviter aussi)
            return val == 2.0 or val == WALL_NEIGHBOUR_PERCENTAGE
        return True  # Hors limites = considéré comme mur

    def handle_wall_detection(self):
        """
        Gère la détection d'un mur proche.
        STRATEGIE: Contournement - on recule puis on contourne le mur
        pour reprendre la trajectoire originale.
        """
        # Déterminer la direction actuelle de l'agent
        if self.state == 'horizontal' or self.bypass_mode:
            dx = 1 if self.direction == 'right' else -1
            dy = 0
        elif self.state == 'vertical':
            dx = 0
            dy = 1 if self.vertical_direction == 'down' else -1
        else:
            dx, dy = 1, 0  # Par défaut

        # Position probable du mur (devant l'agent)
        wall_pos = (self.x + dx, self.y + dy)

        # Marquer la position actuelle comme zone de mur sur la map
        if self.map is not None and self.map[self.y, self.x] == -1:
            self.map[self.y, self.x] = WALL_NEIGHBOUR_PERCENTAGE

        # Marquer le mur si pas déjà connu
        if wall_pos not in self.known_walls:
            self.mark_wall_on_map(wall_pos)
            self.broadcast_wall(wall_pos)

        # Si on était en mode horizontal, entrer en mode bypass
        if self.state == 'horizontal' and not self.bypass_mode:
            self.bypass_mode = True
            self.bypass_original_direction = self.direction
            self.bypass_steps = 0
            print(f"Agent {self.agent_id} - Mur détecté, début contournement")

        # Reculer d'abord
        back_dx = -dx
        back_dy = -dy
        direction = self.get_move_direction(back_dx, back_dy)

        print(f"Agent {self.agent_id} - Recul vers ({self.x + back_dx}, {self.y + back_dy})")

        self.network.send({"header": MOVE, "direction": direction})
        self.waiting_for_move = True

    def is_position_already_known(self):
        """
        Vérifie si la position actuelle a déjà été marquée comme zone d'item connu.
        Retourne True si on est sur une zone déjà explorée/connue.
        """
        if self.map is None:
            return False
        if self.map[self.y, self.x] != -1:
            return True
        return False

    def all_items_found(self):
        """Vérifie si toutes les clés et boîtes de tous les agents ont été trouvées."""
        if self.nb_agent_expected is None:
            return False
        return (len(self.all_keys) == self.nb_agent_expected and
                len(self.all_boxes) == self.nb_agent_expected)

    def is_at_position(self, target_pos):
        """Vérifie si l'agent est à la position cible."""
        if not target_pos:
            return False
        return self.x == target_pos[0] and self.y == target_pos[1]

    def move_towards_target(self, target_pos):
        """
        Déplace l'agent d'une case vers la position cible en évitant les murs.
        Retourne True si un mouvement a été initié, False si déjà à destination.
        """
        if not target_pos or self.is_at_position(target_pos):
            return False

        target_x, target_y = target_pos

        # Calculer la direction idéale
        dx = 0
        dy = 0
        if self.x < target_x:
            dx = 1
        elif self.x > target_x:
            dx = -1
        if self.y < target_y:
            dy = 1
        elif self.y > target_y:
            dy = -1

        # Liste des mouvements à essayer (priorité: diagonal, puis horizontal/vertical)
        moves_to_try = []
        if dx != 0 and dy != 0:
            moves_to_try.append((dx, dy))  # Diagonal idéal
        if dx != 0:
            moves_to_try.append((dx, 0))   # Horizontal
        if dy != 0:
            moves_to_try.append((0, dy))   # Vertical
        # Alternatives si bloqué
        if dy != 0:
            moves_to_try.append((1, dy))   # Contourner par droite
            moves_to_try.append((-1, dy))  # Contourner par gauche
        if dx != 0:
            moves_to_try.append((dx, 1))   # Contourner par bas
            moves_to_try.append((dx, -1))  # Contourner par haut

        # Essayer chaque mouvement
        for try_dx, try_dy in moves_to_try:
            next_x = self.x + try_dx
            next_y = self.y + try_dy
            # Vérifier limites et murs
            if 0 <= next_x < self.w and 0 <= next_y < self.h:
                if not self.is_wall(next_x, next_y):
                    direction = self.get_move_direction(try_dx, try_dy)
                    self.waiting_for_move = True
                    self.network.send({"header": MOVE, "direction": direction})
                    return True

        # Aucun mouvement possible (très rare)
        print(f"Agent {self.agent_id} - Bloqué, aucun chemin vers {target_pos}")
        return False

    def wait_for_move_completion(self, timeout=5.0):
        """
        Attend que le mouvement soit terminé avec un timeout de sécurité.
        Retourne True si le mouvement est terminé, False si timeout.
        """
        wait_time = 0
        step = 0.05
        while self.waiting_for_move and wait_time < timeout:
            sleep(step)
            wait_time += step
        return not self.waiting_for_move

    def send_move_and_wait(self, direction, timeout=5.0):
        """
        Envoie une commande de mouvement et attend la réponse du serveur.
        """
        self.waiting_for_move = True
        self.network.send({"header": MOVE, "direction": direction})
        return self.wait_for_move_completion(timeout)

    def explore_towards_target(self, close_threshold, very_close_threshold, target_value,
                               close_state, very_close_state, on_target_state):
        """
        Generic method to explore and navigate towards a target (key or box).

        Args:
            close_threshold: Value indicating we're close to target (e.g., BOX_NEIGHBOUR_PERCENTAGE/2)
            very_close_threshold: Value indicating we're very close (e.g., BOX_NEIGHBOUR_PERCENTAGE)
            target_value: Value indicating we're on the target (1)
            close_state: State name when close (e.g., 'close_to_box')
            very_close_state: State name when very close (e.g., 'very_close_to_box')
            on_target_state: State name when on target (e.g., 'on_box')
        """
        already_explored = []
        movements = [[1, 0], [-1, 0], [0, -1], [0, 1]]

        # Phase 1: Close to target - search for very close signal
        while self.state == close_state and self.running:
            found_progress = False
            for movement in movements:
                dx, dy = movement[0], movement[1]

                if [self.x + dx, self.y + dy] not in already_explored:
                    direction = self.get_move_direction(dx, dy)
                    already_explored.append([self.x, self.y])

                    if not self.send_move_and_wait(direction):
                        continue  # Timeout, essayer une autre direction

                    if self.cell_val == very_close_threshold:
                        self.state = very_close_state
                        found_progress = True
                        break
                    elif self.cell_val == close_threshold:
                        found_progress = True
                        break
                    else:
                        # Move back if nothing found
                        direction = self.get_move_direction(-dx, -dy)
                        self.send_move_and_wait(direction)

            if not found_progress:
                break  # Éviter boucle infinie

        # Phase 2: Very close to target - search for exact location
        while self.state == very_close_state and self.running:
            found_progress = False
            for movement in movements:
                dx, dy = movement[0], movement[1]

                if [self.x + dx, self.y + dy] not in already_explored:
                    direction = self.get_move_direction(dx, dy)
                    already_explored.append([self.x, self.y])

                    if not self.send_move_and_wait(direction):
                        continue  # Timeout, essayer une autre direction

                    if self.cell_val == target_value:
                        self.state = on_target_state
                        # Marquer IMMEDIATEMENT la map (pas attendre le broadcast)
                        is_key = (on_target_state == 'on_key')
                        self.mark_item_gradient_on_map((self.x, self.y), is_key=is_key)
                        self.network.send({"header": GET_ITEM_OWNER})
                        found_progress = True
                        break
                    elif self.cell_val == very_close_threshold:
                        found_progress = True
                        break
                    else:
                        # Move back if nothing found
                        direction = self.get_move_direction(-dx, -dy)
                        self.send_move_and_wait(direction)

            if not found_progress:
                break  # Éviter boucle infinie

    def get_move_direction(self, dx, dy):
        """"Return the msg type for the move command"""

        if dx == -1 and dy == 0:
            return LEFT
        elif dx == 1 and dy == 0:
            return RIGHT
        elif dx == 0 and dy == -1:
            return UP
        elif dx == 0 and dy == 1:
            return DOWN
        elif dx == -1 and dy == -1:
            return UP_LEFT
        elif dx == 1 and dy == -1:
            return UP_RIGHT
        elif dx == -1 and dy == 1:
            return DOWN_LEFT
        elif dx == 1 and dy == 1:
            return DOWN_RIGHT
        return STAND

    def move_to_initial_position(self):
        """"Move the agent to an appropriate initial position 3 cell away from edge"""
        target_x = self.x
        target_y = self.y

        if self.x < self.min_border_distance:
            target_x = self.min_border_distance
        elif self.x > self.w - 1 - self.min_border_distance:
            target_x = self.w - 1 - self.min_border_distance

        if self.y < self.min_border_distance:
            target_y = self.min_border_distance
        elif self.y > self.h - 1 - self.min_border_distance:
            target_y = self.h - 1 - self.min_border_distance

        if self.x == target_x and self.y == target_y:
            return False  

        dx = 0
        dy = 0

        if self.x < target_x:
            dx = 1
        elif self.x > target_x:
            dx = -1

        if self.y < target_y:
            dy = 1
        elif self.y > target_y:
            dy = -1

        direction = self.get_move_direction(dx, dy)
        cmds = {"header": MOVE, "direction": direction}
        self.network.send(cmds)
        self.waiting_for_move = True

        return True

    def determine_initial_direction(self):
        """"Determine the initial direction that the agent will follow to explore"""
        center_x = self.w / 2
        center_y = self.h / 2

        if self.x <= center_x:
            self.direction = 'right'
        else:
            self.direction = 'left'

        if self.y <= center_y:
            self.vertical_direction = 'down'
        else:
            self.vertical_direction = 'up'

        print(f"Agent {self.agent_id} - Direction: {self.direction}, Vertical: {self.vertical_direction}")

    def is_near_horizontal_border(self):
        """"Chekc if agent is close to border to change direction"""
        if self.direction == 'right':
            return self.x >= self.w - 1 - self.min_border_distance
        else:
            return self.x <= self.min_border_distance

    def is_near_vertical_border(self):
        """"Chekc if agent is close to border to change direction"""
        if self.vertical_direction == 'down':
            return self.y >= self.h - 1 - self.min_border_distance
        else:
            return self.y <= self.min_border_distance

    def run(self):

        if self.waiting_for_move:  # Wait for server response
            return

        # === DETECTION MUR (TOUJOURS en premier, peu importe l'état) ===
        if self.cell_val == WALL_NEIGHBOUR_PERCENTAGE:
            # Marquer la position actuelle sur la map
            if self.map is not None and self.map[self.y, self.x] == -1:
                self.map[self.y, self.x] = WALL_NEIGHBOUR_PERCENTAGE
            # En phase finale, juste reculer sans entrer en bypass
            if self.state in ['going_to_my_key', 'going_to_my_box', 'exploration_ended']:
                # Trouver une direction de recul
                print(f"Agent {self.agent_id} - Mur détecté pendant navigation, recul")
                # Essayer de reculer dans une direction safe
                for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    next_x, next_y = self.x + dx, self.y + dy
                    if 0 <= next_x < self.w and 0 <= next_y < self.h:
                        if not self.is_wall(next_x, next_y):
                            direction = self.get_move_direction(dx, dy)
                            self.network.send({"header": MOVE, "direction": direction})
                            self.waiting_for_move = True
                            return
            else:
                # En exploration, utiliser le bypass normal
                self.handle_wall_detection()
            return

        # === PHASE FINALE: Tous les items trouvés ===
        if self.all_items_found():
            # Marquer la fin de l'exploration si pas encore fait
            if self.state not in ['exploration_ended', 'going_to_my_key', 'going_to_my_box', 'completed']:
                self.state = 'exploration_ended'
                print(f"Agent {self.agent_id} - EXPLORATION TERMINEE! Tous les items trouvés.")
                print(f"  Clés: {self.all_keys}")
                print(f"  Boîtes: {self.all_boxes}")
                return

            # Étape 1: Aller sur MA clé si pas encore fait
            if not self.has_collected_my_key:
                if self.is_at_position(self.my_key):
                    self.has_collected_my_key = True
                    print(f"Agent {self.agent_id} - Clé collectée!")
                else:
                    self.state = 'going_to_my_key'
                    if self.move_towards_target(self.my_key):
                        return

            # Étape 2: Aller sur MA boîte
            if self.has_collected_my_key:
                if self.is_at_position(self.my_chest):
                    # Mission accomplie!
                    self.state = 'completed'
                    cmds = {"header": BROADCAST_MSG}
                    cmds["Msg type"] = COMPLETED
                    cmds["position"] = (self.x, self.y)
                    cmds["owner"] = self.agent_id
                    self.network.send(cmds)
                    print(f"Agent {self.agent_id} - MISSION ACCOMPLIE!")
                    self.running = False
                    return
                else:
                    self.state = 'going_to_my_box'
                    if self.move_towards_target(self.my_chest):
                        return
            return

        # === PHASE EXPLORATION ===
        # Après avoir trouvé un item, reprendre l'exploration
        if self.state in ['on_key', 'on_box']:
            print(f"Agent {self.agent_id} - Item trouvé, reprise exploration")
            self.state = 'horizontal'
            return

        # === DETECTION ITEMS (ignorer si position déjà connue) ===
        if self.cell_val != 0 and not self.is_position_already_known():
            # Handle BOX detection (gradient 0.3)
            if self.cell_val == BOX_NEIGHBOUR_PERCENTAGE / 2:
                self.state = 'close_to_box'
                self.explore_towards_target(
                    close_threshold=BOX_NEIGHBOUR_PERCENTAGE / 2,
                    very_close_threshold=BOX_NEIGHBOUR_PERCENTAGE,
                    target_value=1,
                    close_state='close_to_box',
                    very_close_state='very_close_to_box',
                    on_target_state='on_box'
                )
                return

            # Handle KEY detection (gradient 0.25)
            elif self.cell_val == KEY_NEIGHBOUR_PERCENTAGE / 2:
                self.state = 'close_to_key'
                self.explore_towards_target(
                    close_threshold=KEY_NEIGHBOUR_PERCENTAGE / 2,
                    very_close_threshold=KEY_NEIGHBOUR_PERCENTAGE,
                    target_value=1,
                    close_state='close_to_key',
                    very_close_state='very_close_to_key',
                    on_target_state='on_key'
                )
                return

        if self.state == 'init':
            if self.move_to_initial_position():
                return

            self.determine_initial_direction()
            self.state = 'horizontal'
            print(f"Agent {self.agent_id} - Position initiale atteinte: ({self.x}, {self.y})")
            return

        # === MODE BYPASS (contournement de mur) ===
        if self.bypass_mode:
            # Direction horizontale qu'on veut reprendre
            dx = 1 if self.bypass_original_direction == 'right' else -1
            next_x = self.x + dx

            # Vérifier si on peut avancer horizontalement maintenant
            if not self.is_wall(next_x, self.y) and 0 < next_x < self.w - 1:
                # Voie libre! Sortir du bypass et avancer
                print(f"Agent {self.agent_id} - Contournement réussi, reprise horizontale")
                self.bypass_mode = False
                self.direction = self.bypass_original_direction
                self.state = 'horizontal'
                direction = self.get_move_direction(dx, 0)
                self.network.send({"header": MOVE, "direction": direction})
                self.waiting_for_move = True
                return

            # Sinon, continuer à contourner verticalement
            self.bypass_steps += 1

            # Si on a trop contourné (limite de sécurité), abandonner le bypass
            if self.bypass_steps > self.vertical_step * 2:
                print(f"Agent {self.agent_id} - Contournement trop long, abandon")
                self.bypass_mode = False
                self.direction = 'left' if self.bypass_original_direction == 'right' else 'right'
                self.state = 'horizontal'
                return

            # Vérifier si on peut descendre/monter
            dy = 1 if self.vertical_direction == 'down' else -1
            next_y = self.y + dy

            # Si on atteint un bord vertical, inverser
            if self.is_near_vertical_border():
                self.vertical_direction = 'up' if self.vertical_direction == 'down' else 'down'
                dy = -dy
                next_y = self.y + dy

            # Si mur vertical aussi, abandonner bypass
            if self.is_wall(self.x, next_y):
                print(f"Agent {self.agent_id} - Mur vertical aussi, abandon bypass")
                self.bypass_mode = False
                self.direction = 'left' if self.bypass_original_direction == 'right' else 'right'
                self.state = 'horizontal'
                return

            # Avancer verticalement pour contourner
            print(f"Agent {self.agent_id} - Contournement step {self.bypass_steps}")
            direction = self.get_move_direction(0, dy)
            self.network.send({"header": MOVE, "direction": direction})
            self.waiting_for_move = True
            return

        if self.state == 'horizontal':
            if self.is_near_horizontal_border():
                self.direction = 'left' if self.direction == 'right' else 'right'

                if self.is_near_vertical_border():
                    self.vertical_direction = 'up' if self.vertical_direction == 'down' else 'down'

                self.vertical_moves_remaining = self.vertical_step
                self.state = 'vertical'
                return

            dx = 1 if self.direction == 'right' else -1
            next_x = self.x + dx

            # Vérifier si la prochaine position est un mur connu
            if self.is_wall(next_x, self.y):
                # Mur devant, changer de direction
                self.direction = 'left' if self.direction == 'right' else 'right'
                self.vertical_moves_remaining = self.vertical_step
                self.state = 'vertical'
                print(f"Agent {self.agent_id} - Mur détecté devant, changement de direction")
                return

            direction = self.get_move_direction(dx, 0)
            cmds = {"header": MOVE, "direction": direction}
            self.network.send(cmds)
            self.waiting_for_move = True
            return

        if self.state == 'vertical':
            if self.vertical_moves_remaining <= 0:
                self.state = 'horizontal'
                return

            if self.is_near_vertical_border():
                self.vertical_direction = 'up' if self.vertical_direction == 'down' else 'down'
                self.state = 'horizontal'
                return

            dy = 1 if self.vertical_direction == 'down' else -1
            next_y = self.y + dy

            # Vérifier si la prochaine position est un mur connu
            if self.is_wall(self.x, next_y):
                # Mur devant, changer de direction
                self.vertical_direction = 'up' if self.vertical_direction == 'down' else 'down'
                self.state = 'horizontal'
                print(f"Agent {self.agent_id} - Mur détecté devant, changement de direction")
                return

            direction = self.get_move_direction(0, dy)
            cmds = {"header": MOVE, "direction": direction}
            self.network.send(cmds)
            self.waiting_for_move = True
            self.vertical_moves_remaining -= 1

 
if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--server_ip", help="Ip address of the server", type=str, default="localhost")
    args = parser.parse_args()

    agent = Agent(args.server_ip)

    try:
        while agent.running:
            agent.run()
            sleep(0.1)
        print(f"Agent {agent.agent_id} - Programme terminé.")
    except KeyboardInterrupt:
        agent.running = False
        print(f"Agent {agent.agent_id} - Interruption par l'utilisateur.")