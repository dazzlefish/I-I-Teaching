from IPython.display import *
from pathfinder import *
from time import sleep

# A pixel grid to be rendered as SVG, using the IPython display module.
class IPGrid:
    cells = None  # The cells, initialized to an array in the constructor.
    width = 0     # The width of the grid in pixels. Changing this value requires a call to 'clear'.
    height = 0    # The height of the grid in pixels. Changing this value requires a call to 'clear'.
    cellSize = 0  # The size of a cell in this grid, in pixels.

    def __init__(self, width, height, cellSize):
        self.width = width
        self.height = height
        self.cellSize = cellSize
        self.clear()

    # Clears the grid, this makes every cell transparent again
    def clear(self):
        cells = []
        for i in range(self.width*self.height):
            cells.append(None)
        self.cells = cells

    # Sets the color of a specific cell. If the color is None, the cell is erased.
    def setCell(self, x, y, color):
        self.cells[x * self.height + y] = color

    # Used by IPython to render the SVG.
    def _repr_svg_(self):
        out = "<svg width='" + str(self.width * self.cellSize) + "px' height='" + str(self.height * self.cellSize) + "px'>"
        for x in range(self.width):
            for y in range(self.height):
                cell = self.cells[x * self.height + y]
                if(cell != None):
                    out += "<rect x='" + str(self.cellSize * x) + "' y='" + str(self.cellSize * y) + "' width='" + str(self.cellSize) + "px' height='" + str(self.cellSize) + "px' style='fill:" + cell + ";' />"
        out += "</svg>"
        return out

    # Renders a grid of the 'pathfinding' module into this IPGrid instance.
    def renderGrid(self, grid, cx, cy, pathfinder=None, path=[], closed=[], openl=[]):
        self.clear()

        for x in range(self.width):
            for y in range(self.height):
                px = cx + x
                py = cy + y
                cell = grid.getCell(px, py)
                
                color = None # Nothing: empty

                if(cell <= 0):
                    color = '#000' # Wall: black

                for e in path: # Path nodes: grey
                    if(px == e[0] and py == e[1]):
                        color = '#999'
                        break

                for e in closed: # Closed nodes: light red
                    if(px == e.coords[0] and py == e.coords[1]):
                        color = '#f88'
                        break

                for e in openl: # Open nodes: light blue
                    if(px == e.coords[0] and py == e.coords[1]):
                        color = '#88f'
                        break

                if(pathfinder != None):
                    if(px == pathfinder.start[0] and py == pathfinder.start[1]):
                        color = '#F00' # Start point: red

                    for e in pathfinder.endpoints:
                        if( px == e[0] and py == e[1] ):
                            color = '#00F' # End point: blue
                            break

                if( color != None ):
                    self.setCell(x, y, color)

# Pathfinds and displays the resulting path (if any).
def pathfindAndDisplay(x, y, w, h, grid, pathfinder, maxItr=float("inf")):
    ipgrid = IPGrid(w, h, 440 / w)
    ipgrid.renderGrid(grid, x, y, pathfinder=pathfinder)

    result = pathfinder.findPath(maxItr)

    if(result["failed"]):
        print("Failed after " + str(result["iterations"]) + " iterations!")
    else:
        print("Finished after " + str(result["iterations"]) + " iterations:")
        ipgrid.renderGrid(grid, x, y, pathfinder=pathfinder, path=result["pathCoords"])
        
    display(ipgrid)

# Pathfinds and displays the effect of the algorithm step by step.
# This is kind of a copy from Pathfinder.findPath in the pathfinder module.
def animatedPathfindAndDisplay(x, y, w, h, grid, pathfinder, maxItr=float("inf"), additionalDelay=0):
    ipgrid = IPGrid(w, h, 440 / w)
    ipgrid.renderGrid(grid, x, y, pathfinder=pathfinder)

    display(ipgrid)

    # Fail immediately if no endpoints were specified or if we may not iterate.
    if(len(pathfinder.endpoints) == 0 or maxItr <= 0):
        print("Failed!")
        return
    
    # Count the amount of iterations with this variable
    iterations = 0

    # Create the pathfinding operation instance and add the starting point to the closed
    # list. This automatically updates the open list.
    operation = PathfinderOperation(pathfinder)
    operation.addToClosedList(Node(pathfinder.grid, pathfinder.start, pathfinder.weightCalculator, id=0))
    operation.nodeCount += 1 # Update the node count as we created a node.

    # Loop until the algorithm ends. It may happen that the algorithm may search
    # infinitely when all the endpoints are locked up. This can be solved by either
    # limiting the grid to a specific boundary, or by specifying a maximum amount of
    # iterations.
    while(not operation.hasFoundEnd() and not operation.hasFailed() and iterations < maxItr):
        # Move the next node to the closed list and update the open list accordingly.
        operation.addToClosedList(operation.nextOpenNode)
        iterations += 1

        ipgrid.renderGrid(grid, x, y, pathfinder=pathfinder, closed=operation.closedList.values(), openl=operation.openList.values())
        clear_output(wait=True)
        display(ipgrid)
        sleep(additionalDelay)


        
    # If we left the while loop, end if the algorithm had entered the failed state.
    if(operation.hasFailed() or iterations >= maxItr):
        print("Failed!")
        return

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
    
    ipgrid.renderGrid(grid, x, y, pathfinder=pathfinder, path=pathCoords)
    clear_output(wait=True)
    display(ipgrid)

    print("Finished after " + str(iterations) + " iterations.")
        
