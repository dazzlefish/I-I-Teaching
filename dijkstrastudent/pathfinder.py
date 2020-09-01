'''
An implemenation of a breadth-first-search pathfinding algorithm. The pathfinder can be
configured in many ways, but the default configuration leads to the use of the A* pathfinding
algorithm (say 'A-star').

A well known configuration is the lazy breadth first search algorithm, better known under the
name of Dijkstra's algorithm. This configuration is more optimal when the requirement is to
find the shortest possible path. However, this configuraiont is the least optimal when the
requirement is to use the least possible iterations. Dijkstra's algorithm basically explores
the surroundings by staying as close as possible to the starting point. This leads to many
iterations, but always the shortest path.

Another well known configuration is the greedy breadth first search algorithm. Instead of
staying close to the starting point, this algorithm rather keeps the estimated route length
to the endpoint as small as possible. This leads to finding a path in a very little amount of
iterations, but it may not always find the shortest path.

A* combines the two algorithms by keeping the weighted sum of the traveled route length and
the estimated distance to the end point as small as possible. Weighting of both values is
based on the priority of both values. If the priority of a shorter path is higher than the
priority of less iterations, the traveled route is weighted more. If the priority of less
iterations is higher than the priority of the shortest path, the estimated distance is
weighted more.

The algorithm is based on recursive searching, but recursive searching requires depth-first
searching and that will not work except if the grid is limited. Also, DFS will not work if
there is a specific order in which individual branches need to be explored.

In the algorithm, we use an open list and a closed list. We fill these with nodes. Each node
has a parent node, except the node at the starting point. Parent nodes are always in the
closed list. Nodes in the closed list never mutate, while nodes in the open list may change
their parents and therefore their priority of being explored earlier or later. Nodes in the
closed list are considered as explored. Nodes in the open list are considered candidates to
be explored. Nodes in the open list have a priority of being explored. These priorities
become higher as the weights of the nodes become lower (so, heavier nodes sink deeper in the 
queue and thus have less priority). The pathfinding is then simply done by moving the open
node with the highest priority to the closed list. This is repeated until either no open
nodes are left (the start point is trapped), when the iteration limit is reached, or when an
endpoint is in the open list. When the endpoint node is in the open list, the route is found.

See pathfindertest.py for an example of how to use this pathfinder algorithm.

---------------------------------------------------------------------------------------------
LEGAL INFORMATION:

Copyright (c) 2020 RGSW

This program is free software: you can redistribute it and/or modify it under the terms of
the GNU General Public License as published by the Free Software Foundation, either version 3
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program. If
not, see <https://www.gnu.org/licenses/>.
'''

# Because we need infinity, and we can only get it by parsing it from strings, we compute it
# and store it in a constant before we do any computations.
__INFINITY__ = float("inf")

# This function computes the absolute value. For a given value x, it returns -x if x is a 
# negative number ('x < 0'), and x otherwise. It basically removes the sign from the given
# number.
def abs(val):
    return -val if val < 0 else val

# Returns the manhattan distance between two points. This distance is defined as the distance
# along the x-axis plus the distance along the y-axis. In formula form:
# d = |x2 - x1| + |y2 - y1|
def manhattanDistance(x1, y1, x2, y2):
    return abs(x2 - x1) + abs(y2 - y1)

# Returns the sum of the two parameters. We only use this as default weight calculator: the
# original A* algorithm uses this simple function to compute the final weight of a node.
def add(a, b):
    return a + b

# Grid is a class that represents an abstract grid. This base class represents a completely
# empty grid, going infinitely in all directions. This class must be overridden to make a
# more complex grid with walls.
# See LimitedGrid below for a configurable grid, limited to a bounding box.
class Grid:

    # This method returns the cost of moving into the cell at the given coordinates - the
    # parameters passed to the function. The default implementation simply returns 1 for
    # every pair of coordinates passed into this function. This method returns 0 or any
    # negative value for a wall.
    def getCell(self, x, y):
        return 1

    # Utility method that prints a part of this grid into the console. This part is the 
    # bounding box that must be given as parameters: x, y, width, height. It outputs the part
    # of the grid as an ASCII text, considering a monospace font. Open places are rendered
    # with spaces (' '), walls with hashes ('#'). If an additional pathfinder is specified,
    # it renders the start point as an 'S' and the endpoints as 'E's. If a path is given (or
    # another list of coordinate pairs), each coordinate pair of the path is rendered as a
    # period ('.').
    def printSelf(self, x, y, w, h, pathfinder=None, path=[]):
        # Append all the cells to this
        out = ""

        # Render row by row, iterate over the y axis first
        for i in range(h):
            # Iterate over the x axis for each row
            for j in range(w):
                # 'i' and 'j' are coordinates local to the top-left corner of the bounding 
                # box. Compute our global coordinates by adding the specified top-left corner
                # to our local coords.
                px = x + j
                py = y + i

                # Get the value of our cell. '1' in this class, but can be different in
                # overriding classes.
                cell = self.getCell(px, py)

                # Assume that the cell is an empty place -> ' '
                char = ' '

                # Check if the cell is a wall -> '#'
                if(cell <= 0):
                    char = '#'

                # Loop over the points in the path to see if there is a path node here
                for e in path:
                    # If there is -> '.'
                    if(px == e[0] and py == e[1]):
                        char = '.'
                        break # We can break the loop: we have found a path point

                # If we have a pathfinder, we must also render start and end points
                if(pathfinder != None):
                    # If this position is the starting point of the pathfinder -> 'S'
                    if(px == pathfinder.start[0] and py == pathfinder.start[1]):
                        char = 'S'

                    # Loop over the endpoints of the pathfinder
                    for e in pathfinder.endpoints:
                        # If there is an endpoint at this position -> 'E'
                        if( px == e[0] and py == e[1] ):
                            char = 'E'
                            break # We can break the loop: we have found an endpoint

                # Append the computed character to the output string. We add a space before
                # it to make our rendered cell wrap a more square-shaped box.
                out += ' ' + char
            
            out += '\n' # At the end of a row, we append a newline

        # If we have rendered the grid as a string, we can print it to the console.
        print(out)

# LimitedGrid is a Grid that limits space to a bounding box. Any cell outside this bounding
# box is considered as a wall. Cells inside the bounding box may be modified using setCell.
class LimitedGrid(Grid):
    # These fields form the bounding box. 'x' and 'y' are the coordinates of the corner that
    # are closest to negative infinity. 'w' and 'h' are respectivelythe width and the height
    # of the bounding box, going towards positive infinity.
    # These values should not change, otherwise the grid generates undefined behaviour.
    x = 0
    y = 0
    w = 0
    h = 0

    # This field holds the list of cell values inside the bounding box.
    cells = None

    # The constructor takes the four values of the bounding box: x, y, width and height. It
    # initializes the bounding box with these values. It also initializes the list of cell
    # values inside the box.
    def __init__(self, x, y, w, h):
        self.x = x
        self.y = y
        self.w = w
        self.h = h

        self.cells = []
        # Fill the cell costs with 1: no wall, just a basic cost value.
        for i in range(w * h):
            self.cells.append(1)
    
    # Overrides 'getCell' of the Grid class above. Reference there for a full specification
    # of this method.
    def getCell(self, x, y):
        if(x < self.x or x >= self.x + self.w or y < self.y or y >= self.y + self.h):
            return 0 # Return 0 (wall) if we are outside the bounding box.
        
        # Look up the cell from our array
        return self.cells[x * self.h + y]

    # Sets the cell at the specified position in our grid. If this position is outside the
    # bounding box, this method does nothing. If the position is inside the bounding box, the
    # given weight value can be queried by calling getCell.
    def setCell(self, x, y, wgt):
        if(x < self.x or x >= self.x + self.w or y < self.y or y >= self.y + self.h):
            return # We can't modify outside the bounding box

        # Set the cell cost in our array
        self.cells[x * self.h + y] = wgt
        

            

# Node represents a pathfinding node. This is either a node in the open list or a node in the
# closed list. A node is a pair of coordinates and, except if this is the root node, a parent
# node.
#
# There are 9 fields in this class:
# - grid:              We store the grid we are pathfinding on so that we can retrieve the 
#                      movement cost at our node once we need to compute our weight. We only 
#                      use this value if we are not the root node (if we are, we are the 
#                      start point so we have already been moved to that place).
# - coords:            This is our pair of coordinates, in a tuple: (x, y).
# - parent:            This is our parent. If we are the root node (starting point), we have
#                      no parent and this field is None. 
# - estimatedDistance: The estimated distance to the nearest endpoint: H. This value is set
#                      in the setEstimated weight is not computed by the node itself: the 
#                      pathfinding context computes this for us.
# - traveledDistance:  The distance we have traveled to get here: G. This is 0 if we are the
#                      root node (starting point). If we are not the root node, it is
#                      computed by taking the same value of our parent and adding the cost
#                      to move here (we can get this cost right away from the grid).
# - weigth:            This is our total weight: F. This value is computed by getWeights, by
#                      computing G + H (see the above two fields).
# - validated:         This is a tricky one: it stores whether our weight values are up to
#                      date. We set this to False when our parent has been updated. Once we 
#                      have called updateWeights, we may safely set it back to True. When we
#                      call getWeights while the node is validated (this field is true), we
#                      can save time by just skipping any calculation because our values were
#                      up to date according to this field: they will not change after all,
#                      even if we had done the calculations.
#                      At construction time, we are definitely invalid, therefore the initial
#                      value of this field is False.
# - id:                The ID of this node. The ID is 0 for a root node and every node that
#                      is created later in a pathfinding operation gets this value one larger
#                      than the previously created node. This value is used to determine the
#                      next open node to check out: the one with the highest ID and the 
#                      lowest weight is eventually the node that is being chosen.
# - weightCalc:        A function that computes the weight of nodes: it computes F. This
#                      function must take exactly two arguments: G and H. In A*, this
#                      function defaults to calculating G + H.
class Node:

    grid = None           # Our grid to query costs from
    coords = None         # The coordinates of this node
    parent = None         # The parent node, or None if we are root
    estimatedDistance = 0 # H
    traveledDistance = 0  # G
    weight = 0            # F
    validated = False     # Flag that stores if the F and G values are up to date
    id = 0                # The ID of this node
    weightCalc = None     # The function that computes the weight of this node

    # Our constructor function. Given are our grid, our coordinates, and optionally a parent.
    def __init__(self, grid, coords, weightCalc, parent=None, id=0):
        # Initialize our fields
        self.grid = grid
        self.coords = coords
        self.parent = parent
        self.id = id
        self.weightCalc = weightCalc

    # Sets the parent of this node. The parent weight is determining for our weights, so our
    # weights may be no longer valid after calling this function. To ensure that our weight 
    # values are up to date once we ask for them, we call invalidate.
    def setParent(self, parent):
        # Skip if we are setting our parent to the same value. In that case we do not have to
        # update anything and we can pass.
        if(self.parent == parent):
            return
        
        self.parent = parent
        self.invalidate()

    # Sets the estimated distance between this node and the nearest endpoint. This value is
    # determining for our weights, so our weights may be no longer valid after calling this
    # function. To ensure that our weight values are up to date once we ask for them, we call
    # invalidate.
    def setEstimatedDistance(self, estDist):
        # Skip if we are setting our estimated distance to the same value. In that case we do
        # not have to update anything and we can pass.
        if(self.estimatedDistance == estDist):
            return

        self.estimatedDistance = estDist
        self.invalidate()

    # Marks that the values of the node may not be up to date anymore. We do this by setting
    # our 'validated' field to False.
    def invalidate(self):
        self.validated = False
    
    # Gets or recomputes our weight values (F and G) and returns F ('weight'). We
    # deliberately skip computing H: this value is computed by the pathfinding context and
    # may vary between implementations.
    # 
    # This function takes four steps:
    # 1. We return if we are validated (for performance reasons).
    # 2. We update G: we (re)compute the value of 'traveledDistance'.
    # 3. We update F: we (re)compute the value of 'weight'.
    #
    # Q: Shouldn't we ensure that our parent is up to date before we can update?
    # A: No. In the algorithm, parents are only assigned once they are determined forever
    #    (they are in the always in the closed list). We never invalidate parents.
    def getWeight(self):
        # Step 1: We should not do anything after all if our weight values are up to date. We
        # return the weight value stored in 'weight', as this is the up-to-date F value.
        if(self.validated):
            return self.weight


        # Step 2: Compute G
        # We should separate two different cases here:

        # In the first case we have no parent and we are the starting point. As we are the
        # starting point, we don't have any distance traveled: G = 0.
        if(self.parent == None):
            # We didn't travel: G = 0
            self.traveledDistance = 0
        
        # In the second case we have a parent. In this case we should compute G by taking the
        # G value of our parent and adding the cost to move here to that value. The cost is
        # taken from the grid by using getCell.
        else:
            # Take our cost from the grid. Consider that 'coords' is our pair of coordinates
            # in a tuple: (x, y). 'getCell' has the same signature so we can unpack them
            # directly with the '*' operator.
            cost = self.grid.getCell(*self.coords)
            self.traveledDistance = self.parent.traveledDistance + cost


        # Step 3: Compute F
        # F is defined by our weightCalc fuction. In A*, this fuction defines F as F = G + H.
        # We can use the currently stored values of G and H:
        # - H is given. It can be any value, and is determined by the pathfinding context.
        # - G is up to date, as we have yet computed it.
        self.weight = self.weightCalc(self.traveledDistance, self.estimatedDistance)

        # We are done. Return our weight value as we promised that value to the caller.
        return self.weight

    # Get or recomputes the route length (G) of the route to this node. This calls getWeight,
    # which ensures that our G value is up to date.
    def getRouteLength(self):
        self.getWeight()
        return self.traveledDistance


# PathfinderOperation represents the operation of pathfinding and is used to hold the states
# of the algorithm during the operation.
#
# There are a 5 fields in this class:
# - pathfinder:   The pathfinder instance instantiated this instance. This value is therefore
#                 given in constructor parameter, and this field is set that value.
# - closedList:   The list of closed nodes. This holds all the nodes that have been checked 
#                 out by the algorithm. Their states, including their weights, are determined
#                 and they are never invalidated again: they are closed after all. The
#                 parents of these nodes are also in the closed list.
# - openList:     The list of open nodes. This holds all the nodes that the algorithm can 
#                 check out. The parents of these nodes are always in the closed list. All
#                 open-list nodes are directly reachable from one or more closed-list nodes.
# - nextOpenNode: The next node that will be checked out by the algorithm.
# - nodeCount:    The amount of nodes that were created for this operation. This value is
#                 incremented each time a node is created, and is then used as ID for the
#                 next node that will be created.
#
# The open and closed lists are rather dictionaries than a lists. This is because we don't
# need to have any order in these lists, and we only need to access individual nodes by their
# coordinates.
class PathfinderOperation:
    pathfinder = None
    closedList = None
    openList = None
    nextOpenNode = None
    nodeCount = 0

    # The constructor. This takes a pathfinder as argument to assign the 'pathfinder' field.
    # This also initializes the openList and closedList dictionaries to an empty dictionary.
    def __init__(self, pathfinder):
        self.pathfinder = pathfinder
        self.closedList = {}
        self.openList = {}
    
    # Adds the given node to the closed list and updates the open list accordingly.
    # This method takes a few steps:
    # 1. If the node is in the open list: remove it from the open list.
    # 2. Add the node to the closed list
    # 3. For each direction we can move to (up, right, down or left), update the open list
    #    nodes at those locations. This is done by calling updateOpenListNode
    # 4. We update which node will be the next node to move to the closed list.
    def addToClosedList(self, node):
        (x, y) = node.coords # Unwrap the coordinates of the node into two locals

        # 1. Remove the node from the open list if it is in there
        if(node.coords in self.openList):
            self.openList.pop(node.coords)
        
        # 2. Add the node to the closed list. We don't care about a node already being there
        #    as the algorithm would not create two nodes with the same coordinates.
        self.closedList[node.coords] = node

        # 3. Update the open list nodes that are directly reachable from the moved node
        self.updateOpenListNode(node, (x, y - 1)) # Moving against y axis
        self.updateOpenListNode(node, (x, y + 1)) # Moving with y axis
        self.updateOpenListNode(node, (x - 1, y)) # Moving against x axis
        self.updateOpenListNode(node, (x + 1, y)) # Moving with x axis

        # 4. Update the next node to be moved to the closed list. We reset the value to None
        #    and loop over all the open list nodes to find the best candidate. See the
        #    isBetterCandidateForNextNode for a specification of how candidates are selected.
        self.nextOpenNode = None 
        for n in self.openList.values():
            if(self.isBetterCandidateForNextNode(n)):
                self.nextOpenNode = n
    
    # Updates the open list node at the specified coordinates, considering the given closed
    # list node. This method can either create, update or remove the open list node, or do 
    # neither of these actions. The following procedure is followed in this method:
    # 1. Compute the grid cost of moving to the specified coordinates.
    # 2. If the grid cell is a wall (cost is 0 or less), or if the specified coordinates are
    #    a node in the closed list:
    #    - If the open list specifies a node at the given coordinates: remove and discard it
    #    - Return and end this procedure here
    # 3. If the open list specifies no node at the given coordinates:
    #    - Create and add a new open list node and use the given closed list node as a 
    #      parent. Increment the node count by doing so.
    #    - Compute the estimated distance from the new node to the nearest endpoint and set
    #      the H value of the node to this value.
    #    Else, if the open list does specify a node:
    #    - If the current parent of the open node has a larger route length than the given
    #      closed list node: assign the given closed list node as new parent of the open list
    #      node.
    def updateOpenListNode(self, closedListNode, coords):
        # Lazily get the open list node. Assign to None if there is no open list node.
        openListNode = None if not coords in self.openList else self.openList[coords]

        # 1. Compute the grid cell cost.
        gridCell = self.pathfinder.grid.getCell(*coords)

        # 2. If there is a wall or a closed node at our location: discard any open node
        if(gridCell <= 0 or coords in self.closedList):
            if(openListNode != None):
                del openListNode[coords] # Discard!
            return # End the procedure as we should not be updating an open node here
        
        # 3. If our open list node does not exist, we create it
        if(openListNode == None): 
            openListNode = Node(self.pathfinder.grid, coords, self.pathfinder.weightCalculator, \
                parent=closedListNode, id=self.nodeCount)

            self.nodeCount += 1 # Ensure an unique ID for the next node that will be created.
            self.openList[coords] = openListNode # Add to the open list

            # Compute and set the H value of the newly created node
            dist = self.estimateDistanceToEndpoint(*coords)
            openListNode.setEstimatedDistance(dist)

        # If our open list node does exist, we update it if necessary
        else:
            # Set the parent of the open node to the one with the shortest route.
            if(openListNode.parent.getRouteLength() > closedListNode.getRouteLength()):
                openListNode.setParent(closedListNode)

    # Checks if the given candidate node is a better next open node than our current next
    # open node.
    def isBetterCandidateForNextNode(self, candidate):
        # If there is no next node determined yet, the candidate node wins
        if(self.nextOpenNode == None):
            return True

        # If the current node is on an endpoint, the current node wints
        if(self.pathfinder.isEndpoint(self.nextOpenNode.coords)):
            return False

        # If the candidate node is on an endpoint, the candidate node wins
        if(self.pathfinder.isEndpoint(candidate.coords)):
            return True

        # If the candidate node has a smaller weight than the current node, the candidate
        # node wins
        if(self.nextOpenNode.getWeight() > candidate.getWeight()):
            return True

        # If the candidate node and the current node have equal weights, the one with the
        # highest ID wins
        if(self.nextOpenNode.getWeight() == candidate.getWeight()):
            return candidate.id > self.nextOpenNode.id

    # Estimates the distance from the specified coordinates to the nearest endpoint. This
    # method references the distance estimation function of the pathfinder instance to
    # compute the distance between two points. The distance is computed for all the
    # endpoints. The shorted distance is returned from this method.
    def estimateDistanceToEndpoint(self, x, y):
        leastDistance = __INFINITY__ # All distances are less than infinity

        for e in self.pathfinder.endpoints:
            # Compute the distance to an endpoint
            dist = self.pathfinder.distanceEstimator(x, y, *e)
            if(dist < leastDistance): # Overwrite the current return value by the computed
                leastDistance = dist  # distance if it is less than the current

        return leastDistance

    # Returns true if an endpoint has been found. The endpoint has been found once the next
    # open node is an endpoint.
    def hasFoundEnd(self):
        if(self.nextOpenNode == None):
            return False
        return self.pathfinder.isEndpoint(self.nextOpenNode.coords)

    # Returns true if the algorithm ended up in an exceptional state where the open list is
    # empty. We can only fail in this case.
    def hasFailed(self):
        return not self.hasFoundEnd() and len(self.openList) == 0



# Pathfinder is a class that stores the pathfinding context and runs the actual pathfinding
# algorithm. The Pathfinder class specifies 5 fields:
# - grid:              The grid we are pathfinding on. We query this grid for computing the
#                      movement costs of nodes.
# - start:             The starting point. The algorithm begins finding a path from this
#                      point. The starting point is specified as a tuple of coordinates:
#                      (x, y).
# - endpoints:         A set of endpoints. The algorithm tries to find the easiest path to
#                      one of the endpoints. The endpoint that is reached first is the
#                      endpoint the computed path leads to. If no endpoints are specified,
#                      a pathfinding operation moves directly into the failed state.
# - distanceEstimator: A function that estimates the length of the path between two points.
#                      The better the estimation, the more optimal the algorithm works.
# - weightCalculator:  A function that calculates the total weight (F) of a node, given the G
#                      and H values.
# A pathfinder can be used to find a path multiple times, but it's state should not change
# during an operation. Changing the state during an operation may lead to undefined
# behaviour of the pathfinding operation.
class Pathfinder:
    grid = None
    start = None
    endpoints = None
    distanceEstimator = None
    weightCalculator = None

    # The constructor of a pathfinder. This takes the grid and the starting point as required
    # arguments, but these may be changed later. The constructor also takes the distance 
    # estimator and the weight calculator functions. If no distance estimator is specified,
    # the distance estimator is defaulted to a manhattan distance function. If no weight
    # calculator is specified, the weight calculator is defaulted to a simple sum function.
    def __init__(self, grid, start, distanceEstimator=manhattanDistance, weightCalculator=add):
        self.grid = grid
        self.start = start
        self.distanceEstimator = distanceEstimator
        self.weightCalculator = weightCalculator
        self.endpoints = set()

    # Returns True if the given tuple of coordinates holds the coordinates of an endpoint.
    def isEndpoint(self, coords):
        return coords in self.endpoints

    # This method does the actual pathfinding algorithm. This is basically done by moving the
    # next queued open node to the closed list, repeatingly until the algorithm has entered
    # the end or failed state. If the algorithm reaches the end state, the node that found an
    # endpoint is followed up until the root node, prepending the coordinates of each node to
    # a list. The resulting list specifies the found path from the starting point to one of
    # the endpoints.
    # This method returns a dictionary holding the following keys:
    # - failed:     A boolean that is True when the algorithm ended up in a failed state. If
    #               the algorithm ended up in the finished state, this boolean is False.
    # - iterations: The amount of iterations the pathfinder took to get into a finished or a
    #               failed state. If the algorithm failed because no endpoints were
    #               specified, this value is -1.
    # - endpoint:   The endpoint that the found path leads to, or None if the algorithm ended
    #               up in the failed state.
    # - pathCoords: The coordinates of the found path, ordered from start to end. If the
    #               algorithm failed to find a path, this value is an empty list: [].
    # An optional parameter maxItr specifies how many iterations may be taken to find a path.
    # If the algorithm uses more iterations than specified in this parameter, the algorithm
    # is intentionally pushed into the failed state. The default value is set to infinity,
    # but it is recommended to specify a finite number here if you are not sure if a path
    # exists.
    def findPath(self, maxItr=__INFINITY__):
        # Fail immediately if no endpoints were specified or if we may not iterate.
        if(len(self.endpoints) == 0 or maxItr <= 0):
            # :(
            return {
                "failed": True,
                "iterations": -1,
                "endpoint": None,
                "pathCoords": []
            }
        
        # Count the amount of iterations with this variable
        iterations = 0

        # Create the pathfinding operation instance and add the starting point to the closed
        # list. This automatically updates the open list.
        operation = PathfinderOperation(self)
        operation.addToClosedList(Node(self.grid, self.start, self.weightCalculator, id=0))
        operation.nodeCount += 1 # Update the node count as we created a node.

        # Loop until the algorithm ends. It may happen that the algorithm may search
        # infinitely when all the endpoints are locked up. This can be solved by either
        # limiting the grid to a specific boundary, or by specifying a maximum amount of
        # iterations.
        while(not operation.hasFoundEnd() and not operation.hasFailed() and iterations < maxItr):
            # Move the next node to the closed list and update the open list accordingly.
            operation.addToClosedList(operation.nextOpenNode)
            iterations += 1

            
        # If we left the while loop, end if the algorithm had entered the failed state.
        if(operation.hasFailed() or iterations >= maxItr):
            # :(
            return {
                "failed": True,
                "iterations": iterations,
                "endpoint": None,
                "pathCoords": []
            }

        # From here we are sure that a path is found and we can backtrace the node tree to
        # construct the path.
        pathCoords = []
        node = operation.nextOpenNode

        while(node != None): # Backtrace all parents up until the root node
            # As we start backtracing at the end node, the path we follow must be reversed
            # in order to get a path from start to end. Therefore we prepend to the list
            # instead of appending
            pathCoords.insert(0, node.coords)
            node = node.parent
        
        # At last, when we have constructed our path, we can return with that path as our
        # result: the route has successfully been found!
        # :)
        return {
            "failed": False,
            "pathCoords": pathCoords,
            "endpoint": operation.nextOpenNode.coords,
            "iterations": iterations
        }

    # Adds an endpoint to the pathfinder. Endpoints may also be added by adding tuples of
    # coordinates to the 'endpoints' field. Adding other things directly to the 'endpoitns'
    # field may lead to undefined behaviour, and possibly errors.
    def addEndpoint(self, x, y):
        self.endpoints.add((x, y))
