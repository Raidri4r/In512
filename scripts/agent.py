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
    def __init__(self, server_ip):
        # Exploration state
        self.state = 'init'
        self.direction = None
        self.vertical_direction = 'down'
        self.min_border_distance = 2
        self.vertical_step = 5
        self.vertical_moves_remaining = 0
        self.cell_val = 0
        self.waiting_for_move = False

        # Item tracking
        self.my_key = ()
        self.my_chest = ()
        self.all_keys = {}
        self.all_boxes = {}
        self.has_collected_my_key = False
        self.current_path = None  # BFS path to target

        # Wall tracking
        self.known_walls = set()
        self.bypass_mode = False
        self.bypass_original_direction = None
        self.bypass_steps = 0

        # Network setup (DO NOT TOUCH)
        self.network = Network(server_ip=server_ip)
        self.agent_id = self.network.id
        self.running = True
        self.network.send({"header": GET_DATA})
        env_conf = self.network.receive()
        self.nb_agent_expected = None
        self.nb_agent_connected = 0
        self.x, self.y = env_conf["x"], env_conf["y"]
        self.w, self.h = env_conf["w"], env_conf["h"]
        self.map = np.full((self.h, self.w), -1.0)

        Thread(target=self.msg_cb, daemon=True).start()
        self.wait_for_connected_agent()

    def msg_cb(self):
        """Handle incoming messages from server."""
        while self.running:
            msg = self.network.receive()

            if msg["header"] == MOVE:
                self.x, self.y = msg["x"], msg["y"]
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
                    if owner not in self.all_keys:
                        self.all_keys[owner] = pos
                        self.mark_item_gradient_on_map(pos, is_key=True)
                        print(f"[Agent {self.agent_id}] Key of agent {owner} found at {pos}")
                    if owner == self.agent_id:
                        self.my_key = pos

                elif msg['Msg type'] == BOX_DISCOVERED:
                    if owner not in self.all_boxes:
                        self.all_boxes[owner] = pos
                        self.mark_item_gradient_on_map(pos, is_key=False)
                        print(f"[Agent {self.agent_id}] Box of agent {owner} found at {pos}")
                    if owner == self.agent_id:
                        self.my_chest = pos

                elif msg['Msg type'] == COMPLETED:
                    print(f"[Agent {self.agent_id}] Agent {owner} completed mission!")

                elif msg['Msg type'] == WALL_DISCOVERED:
                    if pos not in self.known_walls:
                        self.mark_wall_on_map(pos)

            elif msg['header'] == GET_ITEM_OWNER:
                owner = msg['owner']
                pos = (self.x, self.y)
                is_key = (self.state == 'on_key')

                if is_key:
                    if owner not in self.all_keys:
                        self.all_keys[owner] = pos
                        print(f"[Agent {self.agent_id}] Found key of agent {owner} at {pos}")
                    if owner == self.agent_id:
                        self.my_key = pos
                        self.has_collected_my_key = True
                else:
                    if owner not in self.all_boxes:
                        self.all_boxes[owner] = pos
                        print(f"[Agent {self.agent_id}] Found box of agent {owner} at {pos}")
                    if owner == self.agent_id:
                        self.my_chest = pos

                self.network.send({
                    "header": BROADCAST_MSG,
                    "Msg type": KEY_DISCOVERED if is_key else BOX_DISCOVERED,
                    "position": pos,
                    "owner": owner
                })

    def wait_for_connected_agent(self):
        """Wait until all agents are connected."""
        self.network.send({"header": GET_NB_AGENTS})
        while True:
            self.network.send({"header": GET_NB_CONNECTED_AGENTS})
            sleep(0.5)
            if self.nb_agent_expected and self.nb_agent_expected == self.nb_agent_connected:
                print(f"[Agent {self.agent_id}] All agents connected!")
                break

    def mark_item_gradient_on_map(self, item_pos, is_key=True):
        """Mark item gradient on internal map to avoid re-exploring."""
        x, y = item_pos
        if is_key:
            close_val, far_val = KEY_NEIGHBOUR_PERCENTAGE, KEY_NEIGHBOUR_PERCENTAGE / 2
        else:
            close_val, far_val = BOX_NEIGHBOUR_PERCENTAGE, BOX_NEIGHBOUR_PERCENTAGE / 2

        if 0 <= y < self.h and 0 <= x < self.w:
            self.map[y, x] = 1.0

        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                nx, ny = x + dx, y + dy
                if 0 <= ny < self.h and 0 <= nx < self.w:
                    self.map[ny, nx] = close_val

        for dx in [-2, -1, 0, 1, 2]:
            for dy in [-2, -1, 0, 1, 2]:
                if abs(dx) <= 1 and abs(dy) <= 1:
                    continue
                nx, ny = x + dx, y + dy
                if 0 <= ny < self.h and 0 <= nx < self.w and self.map[ny, nx] == -1:
                    self.map[ny, nx] = far_val

    def mark_wall_on_map(self, wall_pos):
        """Mark wall and its gradient on internal map."""
        x, y = wall_pos
        self.known_walls.add(wall_pos)

        if 0 <= y < self.h and 0 <= x < self.w:
            self.map[y, x] = 2.0

        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                nx, ny = x + dx, y + dy
                if 0 <= ny < self.h and 0 <= nx < self.w and self.map[ny, nx] == -1:
                    self.map[ny, nx] = WALL_NEIGHBOUR_PERCENTAGE

    def broadcast_wall(self, wall_pos):
        """Broadcast wall position to other agents."""
        self.network.send({
            "header": BROADCAST_MSG,
            "Msg type": WALL_DISCOVERED,
            "position": wall_pos,
            "owner": self.agent_id
        })

    def is_wall(self, x, y):
        """Check if position is a wall or wall gradient."""
        if 0 <= y < self.h and 0 <= x < self.w:
            val = self.map[y, x]
            return val == 2.0 or val == WALL_NEIGHBOUR_PERCENTAGE
        return True

    def is_position_already_known(self):
        """Check if current position is already explored."""
        return self.map[self.y, self.x] != -1

    def all_items_found(self):
        """Check if all keys and boxes have been discovered."""
        if self.nb_agent_expected is None:
            return False
        return len(self.all_keys) == self.nb_agent_expected and len(self.all_boxes) == self.nb_agent_expected

    def is_at_position(self, target_pos):
        """Check if agent is at target position."""
        return target_pos and self.x == target_pos[0] and self.y == target_pos[1]

    def get_move_direction(self, dx, dy):
        """Convert dx/dy to move command constant."""
        directions = {
            (-1, 0): LEFT, (1, 0): RIGHT, (0, -1): UP, (0, 1): DOWN,
            (-1, -1): UP_LEFT, (1, -1): UP_RIGHT, (-1, 1): DOWN_LEFT, (1, 1): DOWN_RIGHT
        }
        return directions.get((dx, dy), STAND)

    def send_move(self, dx, dy):
        """Send move command and set waiting flag."""
        self.network.send({"header": MOVE, "direction": self.get_move_direction(dx, dy)})
        self.waiting_for_move = True

    def send_move_and_wait(self, direction, timeout=5.0):
        """Send move and wait for completion."""
        self.waiting_for_move = True
        self.network.send({"header": MOVE, "direction": direction})
        wait_time = 0
        while self.waiting_for_move and wait_time < timeout:
            sleep(0.05)
            wait_time += 0.05
        return not self.waiting_for_move

    def find_path_bfs(self, target_pos):
        """Find path to target using BFS. Returns list of positions or None."""
        if not target_pos:
            return None

        start = (self.x, self.y)
        if start == target_pos:
            return []

        from collections import deque
        queue = deque([(start, [start])])
        visited = {start}

        all_dirs = [(1,0), (-1,0), (0,1), (0,-1), (1,1), (1,-1), (-1,1), (-1,-1)]

        while queue:
            (cx, cy), path = queue.popleft()

            for dx, dy in all_dirs:
                nx, ny = cx + dx, cy + dy

                if (nx, ny) == target_pos:
                    return path + [(nx, ny)]

                if (nx, ny) not in visited and 0 <= nx < self.w and 0 <= ny < self.h:
                    if not self.is_wall(nx, ny):
                        visited.add((nx, ny))
                        queue.append(((nx, ny), path + [(nx, ny)]))

        return None  # No path found

    def move_towards_target(self, target_pos):
        """Move one step towards target using BFS pathfinding."""
        if not target_pos or self.is_at_position(target_pos):
            return False

        if not self.current_path or self.current_path[-1] != target_pos:
            self.current_path = self.find_path_bfs(target_pos)

        if not self.current_path:
            return False

        next_pos = self.current_path.pop(0)
        self.send_move(next_pos[0] - self.x, next_pos[1] - self.y)
        return True

    def move_to_initial_position(self):
        """Move agent to safe starting position away from borders."""
        target_x = max(self.min_border_distance, min(self.x, self.w - 1 - self.min_border_distance))
        target_y = max(self.min_border_distance, min(self.y, self.h - 1 - self.min_border_distance))

        if self.x == target_x and self.y == target_y:
            return False

        dx = 1 if self.x < target_x else (-1 if self.x > target_x else 0)
        dy = 1 if self.y < target_y else (-1 if self.y > target_y else 0)
        self.send_move(dx, dy)
        return True

    def determine_initial_direction(self):
        """Set initial exploration direction based on position."""
        self.direction = 'right' if self.x <= self.w / 2 else 'left'
        self.vertical_direction = 'down' if self.y <= self.h / 2 else 'up'

    def is_near_horizontal_border(self):
        """Check if near horizontal border."""
        if self.direction == 'right':
            return self.x >= self.w - 1 - self.min_border_distance
        return self.x <= self.min_border_distance

    def is_near_vertical_border(self):
        """Check if near vertical border."""
        if self.vertical_direction == 'down':
            return self.y >= self.h - 1 - self.min_border_distance
        return self.y <= self.min_border_distance

    def handle_wall_detection(self):
        """Handle wall detection during exploration with bypass mode."""
        if self.state == 'horizontal' or self.bypass_mode:
            dx = 1 if self.direction == 'right' else -1
            dy = 0
        elif self.state == 'vertical':
            dx, dy = 0, 1 if self.vertical_direction == 'down' else -1
        else:
            dx, dy = 1, 0

        wall_pos = (self.x + dx, self.y + dy)

        if self.map[self.y, self.x] == -1:
            self.map[self.y, self.x] = WALL_NEIGHBOUR_PERCENTAGE

        if wall_pos not in self.known_walls:
            self.mark_wall_on_map(wall_pos)
            self.broadcast_wall(wall_pos)

        if self.state == 'horizontal' and not self.bypass_mode:
            self.bypass_mode = True
            self.bypass_original_direction = self.direction
            self.bypass_steps = 0

        self.send_move(-dx, -dy)

    def explore_towards_target(self, close_threshold, very_close_threshold, target_value,
                               close_state, very_close_state, on_target_state):
        """Follow gradient to find item (key or box)."""
        already_explored = []
        movements = [[1, 0], [-1, 0], [0, -1], [0, 1]]

        while self.state == close_state and self.running:
            found = False
            for dx, dy in movements:
                if [self.x + dx, self.y + dy] not in already_explored:
                    already_explored.append([self.x, self.y])
                    if not self.send_move_and_wait(self.get_move_direction(dx, dy)):
                        continue
                    if self.cell_val == very_close_threshold:
                        self.state = very_close_state
                        found = True
                        break
                    elif self.cell_val == close_threshold:
                        found = True
                        break
                    else:
                        self.send_move_and_wait(self.get_move_direction(-dx, -dy))
            if not found:
                break

        while self.state == very_close_state and self.running:
            found = False
            for dx, dy in movements:
                if [self.x + dx, self.y + dy] not in already_explored:
                    already_explored.append([self.x, self.y])
                    if not self.send_move_and_wait(self.get_move_direction(dx, dy)):
                        continue
                    if self.cell_val == target_value:
                        self.state = on_target_state
                        self.mark_item_gradient_on_map((self.x, self.y), is_key=(on_target_state == 'on_key'))
                        self.network.send({"header": GET_ITEM_OWNER})
                        found = True
                        break
                    elif self.cell_val == very_close_threshold:
                        found = True
                        break
                    else:
                        self.send_move_and_wait(self.get_move_direction(-dx, -dy))
            if not found:
                break

    def run(self):
        """Main agent loop."""
        if self.waiting_for_move:
            return

        if self.cell_val == WALL_NEIGHBOUR_PERCENTAGE:
            if self.map[self.y, self.x] == -1:
                self.map[self.y, self.x] = WALL_NEIGHBOUR_PERCENTAGE
            if self.state in ['going_to_my_key', 'going_to_my_box', 'exploration_ended']:
                for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    next_x, next_y = self.x + dx, self.y + dy
                    if 0 <= next_x < self.w and 0 <= next_y < self.h and not self.is_wall(next_x, next_y):
                        self.send_move(dx, dy)
                        return
            else:
                self.handle_wall_detection()
            return

        if self.all_items_found():
            if self.state not in ['exploration_ended', 'going_to_my_key', 'going_to_my_box', 'completed']:
                self.state = 'exploration_ended'
                print(f"[Agent {self.agent_id}] EXPLORATION COMPLETE - Going to collect key and box")
                return

            if not self.has_collected_my_key:
                if self.is_at_position(self.my_key):
                    self.has_collected_my_key = True
                    print(f"[Agent {self.agent_id}] Key collected!")
                else:
                    self.state = 'going_to_my_key'
                    self.move_towards_target(self.my_key)
                    return

            if self.has_collected_my_key:
                if self.is_at_position(self.my_chest):
                    self.state = 'completed'
                    self.network.send({
                        "header": BROADCAST_MSG,
                        "Msg type": COMPLETED,
                        "position": (self.x, self.y),
                        "owner": self.agent_id
                    })
                    print(f"[Agent {self.agent_id}] MISSION COMPLETE!")
                    self.running = False
                else:
                    self.state = 'going_to_my_box'
                    self.move_towards_target(self.my_chest)
            return

        # Resume exploration after finding item
        if self.state in ['on_key', 'on_box']:
            print(f"[Agent {self.agent_id}] Item found, resuming exploration")
            self.state = 'horizontal'
            return

        # Item detection
        if self.cell_val != 0 and not self.is_position_already_known():
            if self.cell_val == BOX_NEIGHBOUR_PERCENTAGE / 2:
                self.state = 'close_to_box'
                self.explore_towards_target(
                    BOX_NEIGHBOUR_PERCENTAGE / 2, BOX_NEIGHBOUR_PERCENTAGE, 1,
                    'close_to_box', 'very_close_to_box', 'on_box')
                return
            elif self.cell_val == KEY_NEIGHBOUR_PERCENTAGE / 2:
                self.state = 'close_to_key'
                self.explore_towards_target(
                    KEY_NEIGHBOUR_PERCENTAGE / 2, KEY_NEIGHBOUR_PERCENTAGE, 1,
                    'close_to_key', 'very_close_to_key', 'on_key')
                return

        # Init state
        if self.state == 'init':
            if self.move_to_initial_position():
                return
            self.determine_initial_direction()
            self.state = 'horizontal'
            print(f"[Agent {self.agent_id}] Starting exploration at ({self.x}, {self.y})")
            return

        if self.bypass_mode:
            dx = 1 if self.bypass_original_direction == 'right' else -1
            next_x = self.x + dx

            if not self.is_wall(next_x, self.y) and 0 < next_x < self.w - 1:
                self.bypass_mode = False
                self.direction = self.bypass_original_direction
                self.state = 'horizontal'
                self.send_move(dx, 0)
                return

            self.bypass_steps += 1

            if self.bypass_steps > self.vertical_step * 2:
                self.bypass_mode = False
                self.direction = 'left' if self.bypass_original_direction == 'right' else 'right'
                self.state = 'horizontal'
                return

            dy = 1 if self.vertical_direction == 'down' else -1
            if self.is_near_vertical_border():
                self.vertical_direction = 'up' if self.vertical_direction == 'down' else 'down'
                dy = -dy

            if self.is_wall(self.x, self.y + dy):
                self.bypass_mode = False
                self.direction = 'left' if self.bypass_original_direction == 'right' else 'right'
                self.state = 'horizontal'
                return

            self.send_move(0, dy)
            return

        # Horizontal movement
        if self.state == 'horizontal':
            if self.is_near_horizontal_border():
                self.direction = 'left' if self.direction == 'right' else 'right'
                if self.is_near_vertical_border():
                    self.vertical_direction = 'up' if self.vertical_direction == 'down' else 'down'
                self.vertical_moves_remaining = self.vertical_step
                self.state = 'vertical'
                return

            dx = 1 if self.direction == 'right' else -1
            if self.is_wall(self.x + dx, self.y):
                self.direction = 'left' if self.direction == 'right' else 'right'
                self.vertical_moves_remaining = self.vertical_step
                self.state = 'vertical'
                return

            self.send_move(dx, 0)
            return

        # Vertical movement
        if self.state == 'vertical':
            if self.vertical_moves_remaining <= 0:
                self.state = 'horizontal'
                return

            if self.is_near_vertical_border():
                self.vertical_direction = 'up' if self.vertical_direction == 'down' else 'down'
                self.state = 'horizontal'
                return

            dy = 1 if self.vertical_direction == 'down' else -1
            if self.is_wall(self.x, self.y + dy):
                self.vertical_direction = 'up' if self.vertical_direction == 'down' else 'down'
                self.state = 'horizontal'
                return

            self.send_move(0, dy)
            self.vertical_moves_remaining -= 1


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--server_ip", help="Server IP address", type=str, default="localhost")
    args = parser.parse_args()

    agent = Agent(args.server_ip)

    try:
        while agent.running:
            agent.run()
            sleep(0.1)
        print(f"[Agent {agent.agent_id}] Program ended.")
    except KeyboardInterrupt:
        agent.running = False
        print(f"[Agent {agent.agent_id}] Interrupted by user.")
