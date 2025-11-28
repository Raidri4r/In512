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
                if msg['owner']:
                    cmds = {"header": 0}
                    cmds["Msg type"] = 1 if self.state == 'on_key' else 2
                    cmds["position"] = (agent.x, agent.y)
                    cmds["owner"] = msg['owner']
                    agent.network.send(cmds)


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

        if self.waiting_for_move: #Wait for server response
            return

        if self.cell_val != 0:
            # Handle BOX detection
            if self.cell_val == BOX_NEIGHBOUR_PERCENTAGE/2:
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

            # Handle KEY detection
            elif self.cell_val == KEY_NEIGHBOUR_PERCENTAGE/2:
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


                


        if self.state == 'init':
            if self.move_to_initial_position():
                return  

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
        while True:
            agent.run()
            sleep(0.1) 
    except KeyboardInterrupt:
        pass
# it is always the same location of the agent first location