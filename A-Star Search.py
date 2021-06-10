# Implementing A* Search Route Planner

from helpers import Map, load_map_40, show_map
import math

"""
The `Map` object has two properties: `intersections` and `roads`

The `intersections` are represented as a dictionary. 

The `roads` property is a list where `roads[i]` contains a list of the intersections that intersection `i` connects to.
"""


## The Algorithm

"""
# PathPlanner class
`__init__` - initializes the path planner with a map, M, and typically a start and goal node. If either of these are `None`, the rest of the variables here are also set to none. If you don't have both a start and a goal, there's no path to plan!
- `closedSet` includes any explored/visited nodes. 
- `openSet` are any nodes on the frontier for potential future exploration. 
- `cameFrom` will hold the previous node that best reaches a given node
- `gScore` is the `g` in the `f = g + h` equation, or the actual cost to reach the current node
- `fScore` is the combination of `g` and `h`, i.e. the `gScore` plus a heuristic; total cost to reach the goal
- `path` comes from the `run_search` function

`reconstruct_path` - This function just rebuilds the path after search is run, going from the goal node backwards using each node's `cameFrom` information.

`_reset` - Resets *most* of the initialized variables for PathPlanner. This *does not* reset the map, start or goal variables.

`run_search` - This does a lot of the legwork to run search. First, it checks whether the map, goal and start have been added to the class. Then, it will also check if the other variables, other than `path` are initialized (note that these are only needed to be re-run if the goal or start were not originally given when initializing the class, based on what is discussed above for `__init__`.

From here, we use a function `is_open_empty`, to check that there are still nodes to explore. If we're at our goal, we reconstruct the path. If not, we move our current node from the frontier (`openSet`) and into explored (`closedSet`). Then, we check out the neighbors of the current node, check out their costs, and plan our next move.
"""

class PathPlanner():
    """Construct a PathPlanner Object"""
    def __init__(self, M, start=None, goal=None):
        """ """
        self.map = M
        self.start= start
        self.goal = goal
        self.closedSet = self.create_closedSet() if goal != None and start != None else None
        self.openSet = self.create_openSet() if goal != None and start != None else None
        self.cameFrom = self.create_cameFrom() if goal != None and start != None else None
        self.gScore = self.create_gScore() if goal != None and start != None else None
        self.fScore = self.create_fScore() if goal != None and start != None else None
        self.path = self.run_search() if self.map and self.start != None and self.goal != None else None
    
    def reconstruct_path(self, current):
        """ Reconstructs path after search """
        total_path = [current]
        while current in self.cameFrom.keys():
            current = self.cameFrom[current]
            total_path.append(current)
        return total_path
    
    def _reset(self):
        """Private method used to reset the closedSet, openSet, cameFrom, gScore, fScore, and path attributes"""
        self.closedSet = None
        self.openSet = None
        self.cameFrom = None
        self.gScore = None
        self.fScore = None
        self.path = self.run_search() if self.map and self.start and self.goal else None

    def run_search(self):
        """ """
        if self.map == None:
            raise(ValueError, "Must create map before running search. Try running PathPlanner.set_map(start_node)")
        if self.goal == None:
            raise(ValueError, "Must create goal node before running search. Try running PathPlanner.set_goal(start_node)")
        if self.start == None:
            raise(ValueError, "Must create start node before running search. Try running PathPlanner.set_start(start_node)")

        self.closedSet = self.closedSet if self.closedSet != None else self.create_closedSet()
        self.openSet = self.openSet if self.openSet != None else  self.create_openSet()
        self.cameFrom = self.cameFrom if self.cameFrom != None else  self.create_cameFrom()
        self.gScore = self.gScore if self.gScore != None else  self.create_gScore()
        self.fScore = self.fScore if self.fScore != None else  self.create_fScore()

        while not self.is_open_empty():
            current = self.get_current_node()

            if current == self.goal:
                self.path = [x for x in reversed(self.reconstruct_path(current))]
                return self.path
            else:
                self.openSet.remove(current)
                self.closedSet.add(current)

            for neighbor in self.get_neighbors(current):
                if neighbor in self.closedSet:
                    continue    # Ignore the neighbor which is already evaluated.

                if not neighbor in self.openSet:    # Discover a new node
                    self.openSet.add(neighbor)
                
                # The distance from start to a neighbor
                #the "dist_between" function may vary as per the solution requirements.
                if self.get_tentative_gScore(current, neighbor) >= self.get_gScore(neighbor):
                    continue        # This is not a better path.

                # This path is the best until now. Record it!
                self.record_best_path_to(current, neighbor)
        print("No Path Found")
        self.path = None
        return False


def create_closedSet(self):
    """ Creates and returns a set to hold the set of nodes already evaluated"""
    return set()

def create_openSet(self):
    """ Creates and returns a set to hold the set of currently discovered nodes 
    that are not evaluated yet. Initially, only the start node is known."""
    openset = set()
    if self.start != None:
        openset.add(self.start)
        return openset
    
    raise(ValueError, "Must create start node before creating an open set. Try running PathPlanner.set_start(start_node)")

def create_cameFrom(self):
    """Creates and returns a dictionary that shows which node can most efficiently be reached from another,
    for each node."""
    return dict()

import math
def create_gScore(self):
    """Creates and returns a dictionary that holds the cost of getting from the start node to that node, 
    for each node. The cost of going from start to start is zero. The rest of the node's values are set to infinity."""
    gscore = {}
    for i in self.map.intersections:
        if i == self.start:
            gscore[i] = 0
        else:
            gscore[i] = math.inf
    return gscore

def create_fScore(self):
    """Creates and returns a dictionary that holds the total cost of getting from the start node to the goal
    by passing by that node, for each node. That value is partly known, partly heuristic.
    For the first node, that value is completely heuristic."""
    fscore = {}
    for i in self.map.intersections:
        if i == self.start:
            fscore[i] = self.heuristic_cost_estimate(i)
        else:
            fscore[i] = math.inf
    return fscore

## Set certain variables
"""
The below functions help set certain variables if they weren't a part of initializating our `PathPlanner` class, or if they need to be changed for anothe reason.
"""

def set_map(self, M):
    """Method used to set map attribute """
    self._reset(self)
    self.start = None
    self.goal = None
    self.map = M

def set_start(self, start):
    """Method used to set start attribute """
    self._reset(self)
    self.start = start

def set_goal(self, goal):
    """Method used to set goal attribute """
    self._reset(self)
    self.goal = goal

## Get node information
"""
The below functions concern grabbing certain node information. In `is_open_empty`, you are checking whether there are still nodes on the frontier to explore. In `get_current_node()`, you'll want to come up with a way to find the lowest `fScore` of the nodes on the frontier. In `get_neighbors`, you'll need to gather information from the map to find the neighbors of the current node.
"""

def is_open_empty(self):
    """returns True if the open set is empty. False otherwise. """
    return len(self.openSet) == 0

def get_current_node(self):
    """ Returns the node in the open set with the lowest value of f(node)."""
    openf = {}
    for node in self.openSet:
        openf[node] = self.calculate_fscore(node)
    return min(openf,key=openf.get)

def get_neighbors(self, node):
    """Returns the neighbors of a node"""
    return self.map.roads[node]

## Scores and Costs
"""
Calculating the various parts of the `fScore`.
"""

def get_gScore(self, node):
    """Returns the g Score of a node"""
    return self.gScore[node]

def distance(self, node_1, node_2):
    """ Computes the Euclidean L2 Distance"""
    x1 = self.map.intersections[node_1][0]
    y1 = self.map.intersections[node_1][1]
    x2 = self.map.intersections[node_2][0]
    y2 = self.map.intersections[node_2][1]
    return math.sqrt((x2-x1)**2+(y2-y1)**2)

def get_tentative_gScore(self, current, neighbor):
    """Returns the tentative g Score of a node"""
    return self.get_gScore(current)+self.distance(current, neighbor)

def heuristic_cost_estimate(self, node):
    """ Returns the heuristic cost estimate of a node """
    return self.distance(node, self.goal)

def calculate_fscore(self, node):
    """Calculate the f score of a node. """
    return self.get_gScore(node)+self.heuristic_cost_estimate(node)



## Recording the best path
"""
Recording the best path to a given neighbor node from the current node.
"""

def record_best_path_to(self, current, neighbor):
    """Record the best path to a node """
    self.cameFrom[neighbor] = current
    self.gScore[neighbor] = self.get_tentative_gScore(current, neighbor)
    self.fScore[neighbor] = self.calculate_fscore(neighbor)

## Associating the functions with the `PathPlanner` class
PathPlanner.create_closedSet = create_closedSet
PathPlanner.create_openSet = create_openSet
PathPlanner.create_cameFrom = create_cameFrom
PathPlanner.create_gScore = create_gScore
PathPlanner.create_fScore = create_fScore
PathPlanner.set_map = set_map
PathPlanner.set_start = set_start
PathPlanner.set_goal = set_goal
PathPlanner.is_open_empty = is_open_empty
PathPlanner.get_current_node = get_current_node
PathPlanner.get_neighbors = get_neighbors
PathPlanner.get_gScore = get_gScore
PathPlanner.distance = distance
PathPlanner.get_tentative_gScore = get_tentative_gScore
PathPlanner.heuristic_cost_estimate = heuristic_cost_estimate
PathPlanner.calculate_fscore = calculate_fscore
PathPlanner.record_best_path_to = record_best_path_to

## Preliminary Test
"""
The below is the first test case, just based off of one set of inputs. If some of the functions above aren't implemented yet, or are implemented incorrectly, you likely will get an error from running this cell. Try debugging the error to help you figure out what needs further revision!
"""

planner = PathPlanner(map_40, 5, 34)
path = planner.path
if path == [5, 16, 37, 12, 34]:
    print("great! Your code works for these inputs!")
else:
    print("something is off, your code produced the following:")
    print(path)


# Visualize the result of the above test!
start = 5
goal = 34

show_map(map_40, start=start, goal=goal, path=PathPlanner(map_40, start, goal).path)


## Testing your Code
from test import test

test(PathPlanner)