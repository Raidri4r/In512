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

DIRECTION = DOWN_RIGHT
DIAGONAL = (-1, 1)

class Agent:
    """ Class that implements the behaviour of each agent based on their perception and communication with other agents """
    def __init__(self, server_ip):
        self.my_key = ()
        self.my_chest = ()
        self.total_items_found = 0
        self.mission_completed = 0 # Nb of agent that have their key and reached the chest
        self.status = INIT
        self.direction = DIAGONAL
        self.start_pos = {}
        self.target = (0,0)
        self.next_target = None

        self.direction = DOWN_LEFT 
        self.step = 9
        
        self.steps_remaining = 0
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

        self.x, self.y = env_conf["x"], env_conf["y"]   #initial agent position
        self.w, self.h = env_conf["w"], env_conf["h"]   #environment dimensions
        self.cell_val = env_conf["cell_val"] #value of the cell the agent is located in
        self.map = np.empty((self.w, self.h), dtype=object)
        self.map[:] = None

        Thread(target=self.msg_cb, daemon=True).start()
        self.wait_for_connected_agent()

        self.start_pos[self.agent_id] = (self.x, self.y)
        self.map[self.x, self.y] = self.cell_val
   
    def msg_cb(self): 
        """ Method used to handle incoming messages """
        while self.running:
            msg = self.network.receive()
            self.msg = msg
            if msg["header"] == MOVE:
                self.x, self.y =  msg["x"], msg["y"]
                self.map[self.x, self.y] = msg["cell_val"]
                self.cell_val = msg["cell_val"]
                self.waiting_for_move = False

            elif msg["header"] == GET_NB_AGENTS:
                self.nb_agent_expected = msg["nb_agents"]

            elif msg["header"] == GET_NB_CONNECTED_AGENTS:
                self.nb_agent_connected = msg["nb_connected_agents"]

            elif msg["header"] == BROADCAST_MSG:
                if msg['owner'] == self.agent_id:
                    if msg['type'] == KEY_DISCOVERED:
                        self.my_key = msg['position']
                        print(f"Key of agent {self.agent_id} was found in {msg['position']}")

                    elif msg['type'] == BOX_DISCOVERED:
                        self.my_chest = msg['position']
                        print(f"Chest of agent {self.agent_id} was found in {msg['position']}")

                elif msg['type'] == 3:
                    self.mission_completed +=1

                elif msg['type'] == START_POSITION:
                    self.start_pos[msg['owner']] = (msg['x'], msg['y'])

            elif msg['header'] == GET_DATA:
                self.map = np.zeros((msg['h'], msg['w']))

            elif msg['header'] == GET_ITEM_OWNER:
                if msg['owner']:
                    cmds = {"header": 0}
                    cmds["type"] = 1 if self.state == 'on_key' else 2
                    cmds["position"] = (agent.x, agent.y)
                    cmds["owner"] = msg['owner']
                    agent.network.send(cmds)
      
    def wait_for_connected_agent(self):
        self.network.send({"header": GET_NB_AGENTS})
        check_conn_agent = True
        while check_conn_agent:
            self.network.send({"header": GET_NB_CONNECTED_AGENTS})
            sleep(0.5)
            if self.nb_agent_expected and self.nb_agent_expected == self.nb_agent_connected:
                print("All agents connected")
                check_conn_agent = False

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
        while self.state == close_state:
            for movement in movements:
                dx, dy = movement[0], movement[1]

                if [self.x + dx, self.y + dy] not in already_explored:
                    direction = self.get_move_direction(dx, dy)
                    self.network.send({"header": MOVE, "direction": direction})
                    already_explored.append([self.x, self.y])
                    sleep(1)

                    if self.cell_val == very_close_threshold:
                        self.state = very_close_state
                        break
                    elif self.cell_val == close_threshold:
                        break
                    else:
                        # Move back if nothing found
                        direction = self.get_move_direction(-dx, -dy)
                        self.network.send({"header": MOVE, "direction": direction})
                        sleep(1)

        # Phase 2: Very close to target - search for exact location
        while self.state == very_close_state:
            for movement in movements:
                dx, dy = movement[0], movement[1]

                if [self.x + dx, self.y + dy] not in already_explored:
                    direction = self.get_move_direction(dx, dy)
                    self.network.send({"header": MOVE, "direction": direction})
                    already_explored.append([self.x, self.y])
                    sleep(1)

                    if self.cell_val == target_value:
                        self.state = on_target_state
                        self.network.send({"header": GET_ITEM_OWNER})
                        break
                    elif self.cell_val == very_close_threshold:
                        break
                    else:
                        # Move back if nothing found
                        direction = self.get_move_direction(-dx, -dy)
                        self.network.send({"header": MOVE, "direction": direction})
                        sleep(1)

    def get_direction(self, dx, dy):
        """"Return the message type for the move command"""

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
    
    def move(self, dx, dy):
        """"Move the agent in the specified direction"""
        direction = self.get_direction(dx, dy)
        cmds = {"header": MOVE, "direction": direction}
        self.network.send(cmds)
        self.waiting_for_move = True

    def move_towards(self, tx: int, ty: int) -> bool:
        """"Move the agent towards its target. Return True if the target is reached"""

        dx = tx - self.x
        dy = ty - self.y

        # Return True if we are already at the target
        if dx == 0 and dy == 0:
            return True

        dx = int(dx > 0) - int(dx < 0)
        dy = int(dy > 0) - int(dy < 0)

        self.move(dx, dy)

        return False

        """"Chekc if agent is close to border to change direction"""
        if self.vertical_direction == 'down':
            return self.y >= self.h - 1 - self.min_border_distance
        else:
            return self.y <= self.min_border_distance

    def divide_map(self):
        """ Divide the map into nb_agent_expected regions and return the target position and direction for the agent """
        match self.nb_agent_expected:
            case 2:
                start = [(2, 2), (self.w -3, 2)]
                
            case 3:
                if self.w >= self.h:
                    d = int(np.sqrt((self.w - self.h) * self.h + self.h**2/2))
                    start = [(2, 2), (d, 2), (self.h - d, self.w - 3)]
                
                else:
                    d = int(np.sqrt((self.h - self.w) * self.w + self.w**2/2))
                    start = [(2, 2), (2, d), (self.h -3, self.w - d)]

            case 4:
                if self.w >= self.h:
                    d = int(np.sqrt(((self.w - self.h) * self.h + self.h**2)/2))
                    start = [(2, 2), (self.w - d, 2), (2, self.h -3), (self.w -3, self.h - d)]

                else:
                    d = int(np.sqrt(((self.h - self.w) * self.w + self.w**2)/2))
                    start = [(2, 2), (self.h - d, 2), (2, self.w -3), (self.h -3, self.w - d)]


            case _:
                pass

        for pos in start:
            min = float('inf')
            min_id = -1
            for agent_id, agent_pos in self.start_pos.items():
                dist = np.sqrt((pos[0] - agent_pos[0])**2 + (pos[1] - agent_pos[1])**2)
                if dist < min:
                    min = dist
                    min_id = agent_id
        
            if min_id == self.agent_id:
                return pos[0], pos[1]
            else:
                self.start_pos.pop(min_id)
                
    def find_object(self):
        if self.cell_val == BOX_NEIGHBOUR_PERCENTAGE or self.cell_val == KEY_NEIGHBOUR_PERCENTAGE:
            higher_val = 1
        elif self.cell_val == BOX_NEIGHBOUR_PERCENTAGE/2:
            higher_val = BOX_NEIGHBOUR_PERCENTAGE
        elif self.cell_val == KEY_NEIGHBOUR_PERCENTAGE/2:
            higher_val = KEY_NEIGHBOUR_PERCENTAGE
        else:
            return

        window = np.full((3, 3), None, dtype=object)

        for i, dx in enumerate([-1, 0, 1]):
            for j, dy in enumerate([-1, 0, 1]):
                nx, ny = self.x + dx, self.y + dy
                if 0 <= nx < self.w and 0 <= ny < self.h:
                    window[i, j] = self.map[nx, ny]

        print(f"Agent {self.agent_id} - Perception window:\n{window}")

        # Directions in the 3x3 grid
        directions = {
            (0,0): (-1,-1), (0,1): (-1,0), (0,2): (-1,1),
            (1,0): ( 0,-1),               (1,2): ( 0, 1),
            (2,0): ( 1,-1), (2,1): ( 1,0), (2,2): ( 1,1)
        }

        zero_dirs = []

        # Identify all surrounding cells that are zero
        for (i,j), (dx,dy) in directions.items():
            if window[i, j] == higher_val: # If a higher value is in the surroundings, move towards
                print(f"Agent {self.agent_id} - Moving towards higher value at direction ({dx}, {dy})")
                self.move(dx, dy)
                return
            elif window[i, j] is not None:
                zero_dirs.append((dx, dy))


        if not zero_dirs:
            # No zeros around: go to a random direction
            self.move(1, 0)
            return

        # Compute the average of zero directions
        avg_dx = sum(dx for dx, dy in zero_dirs) / len(zero_dirs)
        avg_dy = sum(dy for dx, dy in zero_dirs) / len(zero_dirs)

        # Move to the opposite direction
        print(f"Agent {self.agent_id} - No higher value found, moving away from average zero direction ({avg_dx}, {avg_dy})")
        self.move(-int(avg_dx), -int(avg_dy))


    def run(self):
        if self.waiting_for_move: 
            return # Wait for server response
        
        if self.status == INIT:
            self.network.send({"header": BROADCAST_MSG, "type": START_POSITION ,"owner": self.agent_id, "x": self.x, "y": self.y})
            if len(self.start_pos) == self.nb_agent_expected:
                self.target = self.divide_map()
                print(f"Agent {self.agent_id} - Target position: {self.target}")
                self.status = TOSTART
            return
        
        if self.cell_val == 1:
            print(f"Agent {self.agent_id} - On object in cell value {self.cell_val}")
            return
        
        elif self.cell_val == WALL_NEIGHBOUR_PERCENTAGE:
            print(f"Agent {self.agent_id} - Near wall in cell value {self.cell_val}")
            self.status = AVOID_WALL
            return
        
        if self.cell_val != 0:
            print(f"Agent {self.agent_id} - Found object in cell value {self.cell_val}")
            self.status = FIND_OBJECT
            self.find_object()

        # if self.cell_val == WALL_NEIGHBOUR_PERCENTAGE:
        #     self.status = AVOID_WALL


        if self.status == TOSTART:
            if self.move_towards(self.target[0], self.target[1]):
                self.status = EXPLORING
                self.direction = LEFT
            return
        
        if self.status == EXPLORING:
            print(f"Agent {self.agent_id} - Exploring towards {self.target} in direction {self.direction}")
            if self.move_towards(self.target[0], self.target[1]):

                if self.next_target is not None:
                    self.target = self.next_target
                    self.next_target = None

                elif self.direction == DOWN_LEFT:
                    vertical_move = min(self.step, (self.h - 2) - self.y)
                    horizontal_move = self.step - vertical_move

                    if  0 < vertical_move < self.step:
                        self.target = (self.x, self.y + vertical_move)
                        self.next_target = (self.x + horizontal_move, self.y + vertical_move)
                    else:
                        self.target = (self.x + horizontal_move, self.y + vertical_move)

                    self.direction = DOWN

                elif self.direction == DOWN:
                    dist = min(self.y - 2, self.w - 3 - self.x)
                    self.target = (self.x + dist, self.y - dist)
                    self.direction = UP_RIGHT

                elif self.direction == UP_RIGHT:
                    horizontal_move = min(self.step, (self.w - 2) - self.x)
                    vertical_move = self.step - horizontal_move

                    if 0 < horizontal_move < self.step:
                        self.target = (self.x + horizontal_move, self.y)
                        self.next_target = (self.x + horizontal_move, self.y + vertical_move)
                    else:
                        self.target = (self.x + horizontal_move, self.y + vertical_move)
                    self.direction = LEFT

                elif self.direction == LEFT:
                    dist = min(self.x - 2, self.h - 3 -self.y)
                    self.target = (self.x - dist, self.y + dist)
                    self.direction = DOWN_LEFT
            return

 
if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--server_ip", help="Ip address of the server", type=str, default="localhost")
    args = parser.parse_args()

    agent = Agent(args.server_ip)
    
    try:
        while True:
            agent.run()
            sleep(0.1) 
    except KeyboardInterrupt:
        pass
# it is always the same location of the agent first location