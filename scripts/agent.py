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
import cv2
import heapq # Nécessaire pour l'algorithme A*

class Agent:
    """ Class that implements the behaviour of each agent based on their perception and communication with other agents """
    def __init__(self, server_ip, show_gui=True):
        self.show_gui = show_gui
        self.my_key = None
        self.my_chest = None
        self.has_key = False
        self.mission_completed = 0 
        
        self.found_keys_owners = set()
        self.found_boxes_owners = set()
        self.all_keys = 0
        self.all_box = 0
        self.current_path = [] # Stocke le chemin calculé par A*

        self.map = None
        self.direction = None 
        self.vertical_direction = 'down'
        self.min_border_distance = 2
        self.vertical_step = 5 
        self.state = 'init' # 'init', 'horizontal', 'vertical', 'retrieval_key', 'retrieval_box', 'finished'
        self.vertical_moves_remaining = 0
        self.cell_val = 0  
        self.waiting_for_move = False

        #DO NOT TOUCH THE FOLLOWING INSTRUCTIONS
        self.network = Network(server_ip=server_ip)
        self.agent_id = self.network.id
        self.running = True
        self.network.send({"header": GET_DATA})
        self.msg = {}
        env_conf = self.network.receive()
        self.nb_agent_expected = None
        self.nb_agent_connected = 0
        self.x, self.y = env_conf["x"], env_conf["y"]   
        self.w, self.h = env_conf["w"], env_conf["h"]   
        cell_val = env_conf["cell_val"] 

        self.map = np.zeros((self.h, self.w))

        print(cell_val)
        Thread(target=self.msg_cb, daemon=True).start()
        print("hello")
        self.wait_for_connected_agent()

    # -------------------------------------------------------------------------
    # ----------------------- ALGORITHME A* (PATHFINDING) ---------------------
    # -------------------------------------------------------------------------
    def calculate_path(self, start, goal):
        """ 
        Implémente A* pour trouver le chemin le plus court.
        start: tuple (x, y)
        goal: tuple (x, y)
        Retourne une liste de directions (constantes MOVE)
        """
        def heuristic(a, b):
            # Distance de Manhattan
            return abs(a[0] - b[0]) + abs(a[1] - b[1])

        # Priority queue: (f_score, x, y)
        open_set = []
        heapq.heappush(open_set, (0, start))
        
        came_from = {}
        g_score = {start: 0}
        f_score = {start: heuristic(start, goal)}
        
        # Mouvements possibles (Dx, Dy, Constante Direction)
        neighbors = [
            (0, -1, UP), (0, 1, DOWN), (-1, 0, LEFT), (1, 0, RIGHT),
            (-1, -1, UP_LEFT), (1, -1, UP_RIGHT), (-1, 1, DOWN_LEFT), (1, 1, DOWN_RIGHT)
        ]

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                return self.reconstruct_path(came_from, current, start)

            cx, cy = current
            
            for dx, dy, direction in neighbors:
                neighbor = (cx + dx, cy + dy)
                nx, ny = neighbor

                # Vérifier limites map
                if 0 <= nx < self.w and 0 <= ny < self.h:
                    # Vérifier obstacles
                    # On considère comme obstacle tout ce qui vaut 1.0 (Mur ou Objet)
                    # SAUF si c'est notre destination (la clé ou la boite)
                    is_obstacle = (self.map[ny, nx] == 1.0) and (neighbor != goal)
                    
                    if not is_obstacle:
                        # Coût du mouvement (1 pour orthogonal, 1.414 pour diagonal mais ici on compte 1)
                        tentative_g_score = g_score[current] + 1
                        
                        if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                            came_from[neighbor] = (current, direction)
                            g_score[neighbor] = tentative_g_score
                            f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                            heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        print(f"Agent {self.agent_id}: Aucun chemin trouvé vers {goal}")
        return []

    def reconstruct_path(self, came_from, current, start):
        """ Reconstruit la liste des directions à partir du dictionnaire came_from """
        path = []
        while current != start:
            prev, direction = came_from[current]
            path.append(direction)
            current = prev
        path.reverse() # On veut le chemin du début à la fin
        return path

    # -------------------------------------------------------------------------
    # ----------------------- NOUVELLES FONCTIONS DE MOUVEMENT ----------------
    # -------------------------------------------------------------------------

    def move_to_key(self):
        """ Déplace l'agent vers sa clé en utilisant A* """
        if not self.my_key:
            return # On ne sait pas où elle est

        target = (int(self.my_key[0]), int(self.my_key[1]))
        current = (int(self.x), int(self.y))

        # Si on est arrivé sur la clé
        if current == target:
            self.has_key = True
            print(f"Agent {self.agent_id} a récupéré sa clé !")
            # Transition vers la boite
            self.state = 'retrieval_box'
            self.current_path = [] # Reset path
            return

        # Calcul du chemin si pas encore fait ou si terminé
        if not self.current_path:
            self.current_path = self.calculate_path(current, target)
        
        # Exécuter le prochain mouvement
        if self.current_path:
            next_dir = self.current_path.pop(0)
            self.network.send({"header": MOVE, "direction": next_dir})
            self.waiting_for_move = True
        else:
            # Cas rare : recalculer (bloqué ou recalcul nécessaire)
            self.current_path = self.calculate_path(current, target)

    def move_to_box(self):
        """ Déplace l'agent vers sa boite en utilisant A* """
        if not self.my_chest:
            return 

        target = (int(self.my_chest[0]), int(self.my_chest[1]))
        current = (int(self.x), int(self.y))

        # Si on est arrivé sur la boite
        if current == target:
            self.state = 'finished'
            print(f"Agent {self.agent_id} a atteint sa boite ! MISSION ACCOMPLIE.")
            self.network.send({"header": MOVE, "direction": STAND}) # On s'arrête
            return

        if not self.current_path:
            self.current_path = self.calculate_path(current, target)
        
        if self.current_path:
            next_dir = self.current_path.pop(0)
            self.network.send({"header": MOVE, "direction": next_dir})
            self.waiting_for_move = True

    # -------------------------------------------------------------------------
    # -------------------------------------------------------------------------
    def display_map(self):
        """ 
        Affiche la carte locale de l'agent avec OpenCV.
        Les valeurs de la map (0.0 à 1.0) sont converties en niveaux de gris.
        L'agent est représenté par un pixel rouge.
        """
        if not self.show_gui:
            return
        # 1. Normaliser la map pour l'affichage (0-1 float -> 0-255 uint8)
        # On multiplie par 255 pour avoir des niveaux de gris
        visual_map = (self.map * 255).astype(np.uint8)
         # 2. Convertir en BGR pour pouvoir mettre de la couleur (Agent en rouge)
        visual_map_color = cv2.cvtColor(visual_map, cv2.COLOR_GRAY2BGR)
        
        # 3. Marquer la position de l'agent (Pixel Rouge : B=0, G=0, R=255)
        # Attention : map numpy est [y, x]
        if 0 <= self.y < self.h and 0 <= self.x < self.w:
            visual_map_color[int(self.y), int(self.x)] = [0, 0, 255] # Rouge
        
        # Dessiner la destination si en mode récupération
        target = None
        if self.state == 'retrieval_key' and self.my_key:
            target = self.my_key
            color = [0, 255, 255] # Jaune
        elif self.state == 'retrieval_box' and self.my_chest:
            target = self.my_chest
            color = [255, 0, 0] # Bleu

        if target:
            tx, ty = int(target[0]), int(target[1])
            if 0 <= ty < self.h and 0 <= tx < self.w:
                 visual_map_color[ty, tx] = color
        # 4. Agrandir l'image (Zoom x20) pour que ce soit visible sur l'écran
        scale = 20
        h, w = visual_map_color.shape[:2]
        large_map = cv2.resize(visual_map_color, (w * scale, h * scale), interpolation=cv2.INTER_NEAREST)
        
        # 5. Afficher la fenêtre
        cv2.imshow(f"Agent {self.agent_id} Map", large_map)
        cv2.waitKey(1)

    def msg_cb(self): 
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
                self.update_map_from_broadcast(msg)

                # --- MISE A JOUR DES COMPTEURS ---
                if 'owner' in msg and 'Msg type' in msg:
                    owner_id = msg['owner']
                    if msg['Msg type'] == KEY_DISCOVERED:
                        self.found_keys_owners.add(owner_id)
                    elif msg['Msg type'] == BOX_DISCOVERED:
                        self.found_boxes_owners.add(owner_id)
                    
                    self.all_keys = len(self.found_keys_owners)
                    self.all_box = len(self.found_boxes_owners)
                # ---------------------------------

                if msg['owner'] == self.agent_id:
                    if msg['Msg type'] == KEY_DISCOVERED:
                        self.my_key = msg['position']
                        print(f"Key of agent {self.agent_id} was found in {msg['position']}")
                    elif msg['Msg type'] == BOX_DISCOVERED:
                        self.my_chest = msg['position']
                        print(f"Chest of agent {self.agent_id} was found in {msg['position']}")
                elif msg['Msg type'] == 3:
                    self.mission_completed +=1

            elif msg['header'] == GET_DATA:
                self.map = np.zeros((msg['h'], msg['w']))
            elif msg['header'] == GET_ITEM_OWNER:

                cmds = {"header": 0}
                cmds["Msg type"] = 1 if self.state == 'on_key' else 2
                cmds["position"] = (agent.x, agent.y)
                cmds["owner"] = msg['owner']
                agent.network.send(cmds)

            # Debug print pour suivre l'état
            print(f"Agent {self.agent_id}: Keys={self.all_keys}/{self.nb_agent_expected}, Boxes={self.all_box}/{self.nb_agent_expected}")

    def wait_for_connected_agent(self):
        self.network.send({"header": GET_NB_AGENTS})
        check_conn_agent = True
        while check_conn_agent:
            self.network.send({"header": GET_NB_CONNECTED_AGENTS})
            sleep(0.5)
            if self.nb_agent_expected and self.nb_agent_expected == self.nb_agent_connected:
                print("both connected!")
                check_conn_agent = False

    def update_map(self):
        """ Mise à jour de la map locale avec la valeur courante. """
        if 0 <= self.x < self.w and 0 <= self.y < self.h:
            self.map[self.y, self.x] = self.cell_val

    def compare_with_map(self):
        """ Compare la valeur reçue avec celle stockée. """
        if 0 <= self.x < self.w and 0 <= self.y < self.h:
            known_val = self.map[self.y, self.x]
            if known_val != 0 and abs(known_val - self.cell_val) < 0.03:
                return True
        return False

    def update_map_from_broadcast(self, msg):
        """ Met à jour la map locale et marque la zone autour de l'objet trouvé """
        if 'position' not in msg or 'Msg type' not in msg: 
            return

        target_x, target_y = msg['position']
        msg_type = msg['Msg type']
        
        if msg_type == KEY_DISCOVERED:
            center_val = 1.0
            neighbour_val = KEY_NEIGHBOUR_PERCENTAGE
        elif msg_type == BOX_DISCOVERED:
            center_val = 1.0
            neighbour_val = BOX_NEIGHBOUR_PERCENTAGE
        else:
            return 

        offsets_1 = [(-1, -1), (0, -1), (1, -1), (-1, 0), (1, 0), (-1, 1), (0, 1), (1, 1)]
        offsets_2 = [(-2, -2), (-1, -2), (0, -2), (1, -2), (2, -2), 
                     (-2, -1), (2, -1), (-2, 0), (2, 0), (-2, 1), (2, 1), 
                     (-2, 2), (-1, 2), (0, 2), (1, 2), (2, 2)]

        if 0 <= target_x < self.w and 0 <= target_y < self.h:
            self.map[target_y, target_x] = center_val

        for dx, dy in offsets_1:
            nx, ny = target_x + dx, target_y + dy
            if 0 <= nx < self.w and 0 <= ny < self.h:
                self.map[ny, nx] = neighbour_val 

        for dx, dy in offsets_2:
            nx, ny = target_x + dx, target_y + dy
            if 0 <= nx < self.w and 0 <= ny < self.h:
                self.map[ny, nx] = neighbour_val / 2.0


    def explore_towards_target(self, close_threshold, very_close_threshold, target_value,
                               close_state, very_close_state, on_target_state):
        already_explored = []
        movements = [[1, 0], [-1, 0], [0, -1], [0, 1]]

        while self.state == close_state:
            for movement in movements:
                dx, dy = movement[0], movement[1]
                if [self.x + dx, self.y + dy] not in already_explored:
                    direction = self.get_move_direction(dx, dy)
                    self.network.send({"header": MOVE, "direction": direction})
                    already_explored.append([self.x, self.y])
                    sleep(1)
                    
                    self.update_map()
                    self.display_map() # Visualization update

                    if self.cell_val == very_close_threshold:
                        self.state = very_close_state
                        break
                    elif self.cell_val == close_threshold:
                        break
                    else:
                        direction = self.get_move_direction(-dx, -dy)
                        self.network.send({"header": MOVE, "direction": direction})
                        sleep(1)
                        self.update_map()
                        self.display_map() # Visualization update

        while self.state == very_close_state:
            for movement in movements:
                dx, dy = movement[0], movement[1]
                if [self.x + dx, self.y + dy] not in already_explored:
                    direction = self.get_move_direction(dx, dy)
                    self.network.send({"header": MOVE, "direction": direction})
                    already_explored.append([self.x, self.y])
                    sleep(1)
                    
                    self.update_map()
                    self.display_map() # Visualization update

                    if self.cell_val == target_value:
                        self.state = on_target_state
                        self.network.send({"header": GET_ITEM_OWNER})
                        msg_type = KEY_DISCOVERED if 'key' in on_target_state else BOX_DISCOVERED
                        fake_msg = {'position': (self.x, self.y), 'Msg type': msg_type}
                        self.update_map_from_broadcast(fake_msg)
                        
                        # IMPORTANT: Si c'est MA clé/boite, je la note immédiatement
                        # Le retour du serveur (BROADCAST) le fera aussi, mais par sécurité :
                        # La logique principale est dans msg_cb qui gère l'ID
                        break

                    elif self.cell_val == very_close_threshold:
                        break
                    else:
                        direction = self.get_move_direction(-dx, -dy)
                        self.network.send({"header": MOVE, "direction": direction})
                        sleep(1)
                        self.update_map()
                        self.display_map() # Visualization update

    def get_move_direction(self, dx, dy):
        if dx == -1 and dy == 0: return LEFT
        elif dx == 1 and dy == 0: return RIGHT
        elif dx == 0 and dy == -1: return UP
        elif dx == 0 and dy == 1: return DOWN
        elif dx == -1 and dy == -1: return UP_LEFT
        elif dx == 1 and dy == -1: return UP_RIGHT
        elif dx == -1 and dy == 1: return DOWN_LEFT
        elif dx == 1 and dy == 1: return DOWN_RIGHT
        return STAND

    def move_to_initial_position(self):
        target_x = self.x
        target_y = self.y
        if self.x < self.min_border_distance: target_x = self.min_border_distance
        elif self.x > self.w - 1 - self.min_border_distance: target_x = self.w - 1 - self.min_border_distance
        if self.y < self.min_border_distance: target_y = self.min_border_distance
        elif self.y > self.h - 1 - self.min_border_distance: target_y = self.h - 1 - self.min_border_distance

        if self.x == target_x and self.y == target_y: return False  

        dx = 1 if self.x < target_x else -1 if self.x > target_x else 0
        dy = 1 if self.y < target_y else -1 if self.y > target_y else 0
        direction = self.get_move_direction(dx, dy)
        self.network.send({"header": MOVE, "direction": direction})
        self.waiting_for_move = True
        return True

    def determine_initial_direction(self):
        center_x = self.w / 2
        center_y = self.h / 2
        self.direction = 'right' if self.x <= center_x else 'left'
        self.vertical_direction = 'down' if self.y <= center_y else 'up'
        print(f"Agent {self.agent_id} - Direction: {self.direction}, Vertical: {self.vertical_direction}")

    def is_near_horizontal_border(self):
        if self.direction == 'right': return self.x >= self.w - 1 - self.min_border_distance
        else: return self.x <= self.min_border_distance

    def is_near_vertical_border(self):
        if self.vertical_direction == 'down': return self.y >= self.h - 1 - self.min_border_distance
        else: return self.y <= self.min_border_distance

    def run(self):
        # Update Visualisation at start of run loop
        self.display_map()
        if self.waiting_for_move: 
            return

        # ---------------- GESTION DES MODES DE JEU ----------------
        
        # Cas 1 : Tout le monde a trouvé tout -> Mode récupération
        if self.nb_agent_expected and self.all_keys >= self.nb_agent_expected and self.all_box >= self.nb_agent_expected:
            
            # Si j'ai fini, je ne fais rien
            if self.state == 'finished':
                return

            # Transition initiale vers le mode récupération
            if self.state not in ['retrieval_key', 'retrieval_box', 'finished']:
                print(f"Agent {self.agent_id}: Tous les objets trouvés ! Passage en mode récupération.")
                self.state = 'retrieval_key'
            
            # Logique de récupération
            if self.state == 'retrieval_key':
                if self.has_key:
                    self.state = 'retrieval_box'
                else:
                    self.move_to_key()
            
            if self.state == 'retrieval_box':
                self.move_to_box()
                
            return # On sort de la boucle pour ne pas faire d'exploration
        
        # ---------------- FIN GESTION DES MODES ----------------

        if self.state in ['on_key', 'on_box']:
            self.state = 'horizontal'

        is_known = self.compare_with_map()
        self.update_map()

        if self.cell_val != 0 and not is_known:
            if abs(self.cell_val - BOX_NEIGHBOUR_PERCENTAGE/2) < 0.001:
                self.state = 'close_to_box'
                self.explore_towards_target(
                    close_threshold=BOX_NEIGHBOUR_PERCENTAGE/2,
                    very_close_threshold=BOX_NEIGHBOUR_PERCENTAGE,
                    target_value=1,
                    close_state='close_to_box',
                    very_close_state='very_close_to_box',
                    on_target_state='on_box'
                )
                return
            elif abs(self.cell_val - KEY_NEIGHBOUR_PERCENTAGE/2) < 0.001:
                self.state = 'close_to_key'
                self.explore_towards_target(
                    close_threshold=KEY_NEIGHBOUR_PERCENTAGE/2,
                    very_close_threshold=KEY_NEIGHBOUR_PERCENTAGE,
                    target_value=1,
                    close_state='close_to_key',
                    very_close_state='very_close_to_key',
                    on_target_state='on_key'
                )
                return

        # Logique d'exploration (balayage)
        if self.state == 'init':
            if self.move_to_initial_position(): return  
            self.determine_initial_direction()
            self.state = 'horizontal'
            print(f"Agent {self.agent_id} - Position initiale atteinte: ({self.x}, {self.y})")
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
            direction = self.get_move_direction(dx, 0)
            self.network.send({"header": MOVE, "direction": direction})
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
            direction = self.get_move_direction(0, dy)
            self.network.send({"header": MOVE, "direction": direction})
            self.waiting_for_move = True
            self.vertical_moves_remaining -= 1

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--server_ip", help="Ip address of the server", type=str, default="localhost")
    args = parser.parse_args()

    agent = Agent(args.server_ip, show_gui=True)
    
    try:
        while True:
            agent.run()
            sleep(0.1) 
    except KeyboardInterrupt:
        cv2.destroyAllWindows()