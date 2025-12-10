__author__ = "Aybuke Ozturk Suri, Johvany Gustave"
__copyright__ = "Copyright 2023, IN512, IPSA 2024"
__credits__ = ["Aybuke Ozturk Suri", "Johvany Gustave"]
__license__ = "Apache License 2.0"
__version__ = "1.0.0"

import numpy as np

""" This file contains all the 'constants' shared by all the scripts """

""" MSG HEADERS """
BROADCAST_MSG = 0
GET_DATA = 1    #get the current location of the agent and the dimension of the environment (width and height)
MOVE = 2
GET_NB_CONNECTED_AGENTS = 3
GET_NB_AGENTS = 4
GET_ITEM_OWNER = 5

""" ALLOWED MOVES """
STAND = 0   #do not move
LEFT = 1
RIGHT = 2
UP = 3
DOWN = 4
UP_LEFT = 5
UP_RIGHT = 6
DOWN_LEFT = 7
DOWN_RIGHT = 8

""" BROADCAST TYPES """
START_POSITION = 0
KEY_DISCOVERED = 1  #inform other agents that you discovered a key
BOX_DISCOVERED = 2
COMPLETED = 3   #inform other agents that you discovered your key and you reached your own box

""" GAME """
GAME_ID = -1    #id of the game when it sends a message to an agent
KEY_NEIGHBOUR_PERCENTAGE = 0.5  #value of an adjacent cell to a key
BOX_NEIGHBOUR_PERCENTAGE = 0.6  #value of an adjacent cell to a key
OBJECT_FOUND = 1
KEY_TYPE = 0    #one of the types of item that is output by the 'Get item owner' request
BOX_TYPE = 1
WALL_NEIGHBOUR_PERCENTAGE = 0.35

""" GUI """
BG_COLOR = (255, 255, 255)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)

""" STATUS """
INIT = 1
TOSTART = 2
EXPLORING = 3
FIND_OBJECT = 4
AVOID_WALL = 5
TO_KEY = 6
TO_BOX = 7
COMPLETED = 8

""" OBJECTS"""

KEY = np.array([[0.25, 0.25, 0.25, 0.25, 0.25],
                [0.25, 0.50, 0.50, 0.50, 0.25],
                [0.25, 0.50, 1.00, 0.50, 0.25],
                [0.25, 0.50, 0.50, 0.50, 0.25],
                [0.25, 0.25, 0.25, 0.25, 0.25]])

BOX = np.array([[0.30, 0.30, 0.30, 0.30, 0.30],
                [0.30, 0.60, 0.60, 0.60, 0.30],
                [0.30, 0.60, 1.00, 0.60, 0.30],
                [0.30, 0.60, 0.60, 0.60, 0.30],
                [0.30, 0.30, 0.30, 0.30, 0.30]])

WALL = np.array([[0.35, 0.35, 0.35, 0.00, 0.00],
                 [0.35, 1.00, 0.35, 0.00, 0.00],
                 [0.35, 1.00, 0.35, 0.35, 0.35],
                 [0.35, 1.00, 1.00, 1.00, 0.35],
                 [0.35, 0.35, 0.35, 0.35, 0.35]])