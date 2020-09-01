'''
Example usage of the pathfinder algorithm implemented in pathfiner.py.

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

# We import our own pathfinding module. Ensure that pathfinder.py is in the same directory as
# this file to keep it working!!!
from pathfinder import *

# Ok. Now we have our algorithm implemented, we can test it

# Create a limited grid. Hereby we ensure that the pathfinder does not search outside the
# bounding box [0, 0, 20, 20].
grid = LimitedGrid(0, 0, 20, 20)
for i in range(15):           # Add two walls of 15 cells.
    grid.setCell(7, i, 0)     # One reaches from the top at x = 7
    grid.setCell(14, 19-i, 0) # One reaches from the bottom at x = 14

# We create a pathfinder for our grid, starting at the point (5, 3). We use the default
# weight calculation function (sum) and the default distance estimation function
# (manhattan distance).
pathfinder = Pathfinder(grid, (5, 3))

# Let the pathfinder find the path to the point (17, 15).
pathfinder.addEndpoint(17, 15)

# Print the grid before we start off so that we can get a view of what we are pathfinding in.
grid.print(0, 0, 20, 20, pathfinder=pathfinder)

# Then, we call the algorithm! We don't specify any max iterations as we already limited the
# search area by using LimitedGrid.
result = pathfinder.findPath()

# Print the result.
if(result["failed"]):
    print("Failed after", result["iterations"], "iterations!")
else:
    print("Finished after", result["iterations"], "iterations:")

    # Visualize our path when we have found it by printing our grid once again
    grid.print(0, 0, 20, 20, pathfinder=pathfinder, path=result["pathCoords"])