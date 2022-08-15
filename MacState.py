
from Graphs import WeightedListGraph

class MacState(object):
    """This represents the state of a search in a maze.  It does not
    represent the maze, just the current location in the maze, and the
    series of cells that have been traversed to get to this location.  That
    is represented in the pathToMe instance variable inherited from the parent
    class.  The cost is determined externally, by the task advisor.
    NOTE: for many search algorithms, we need to store the state object in a set, and to be able to find equivalent
    states that may have different costs, but fundamentally refer to the same place in the map.
    Thus any state class you create MUST implement the __eq__ and __hash__ methods, which are used by Python's set
    data type. These methods should be based on the information about the location in the map. For this
    state, that means the row and column indices of the grid square this state represents."""

    def __init__(self, numLabel, path=None, cost=None):
        """Given the row and column location of the current state, and optional path
        and cost, initializes the state for the search"""
        if path is None:
            self.pathToMe = []
        else:
            self.pathToMe = path
        self.myCost = cost
        self.numLabel = numLabel

    def setPath(self, newPath):
        """This is a method that Dijkstra's needs."""
        self.pathToMe = newPath

    def getPath(self):
        """Access the value of the pathToMe instance variable"""
        return self.pathToMe

    def getCost(self):
        """Access the value of the myCost instance variable"""
        return self.myCost

    def getLocation(self):
        """Return the row and column location of this state"""
        return (self.numLabel)

    def __eq__(self, state):
        """Check if the input is the same type, and if it has the same row and column
        Overloads the == operator."""
        if type(state) is type(self):
            return (state.numLabel == self.numLabel)
        else:
            return False

    def __hash__(self):
        """Makes the state hashable by hashing a tuple of its row and column, so that it can be stored in
        a set or dictionary. Note that states that are == will produce the same hash value."""
        return hash((self.numLabel))

    def __str__(self):
        """To print this object, print the row and column in brackets, followed by the
        path and cost"""
        strng = "[" + str(self.numLabel) + "]"
        strng += "  " + str(self.pathToMe) + " " + str(self.myCost)
        return strng

class AStarMacState(MacState):
    """This represents the state of a search in a maze.  It does not
represent the maze, just the current location in the maze, and the
series of cells that have been traversed to get to this location.  That
is represented in the pathToMe instance variable inherited from the parent
class.  The cost is determined externally."""

    def __init__(self, numLabel, path=None, costToHere=None, costToGoal=None):
        """Given the row and column, the current path, and the two costs (cost so far and heuristic
        cost to come, this creates a state/node for the search"""

        MacState.__init__(self, numLabel, path, costToHere + costToGoal)
        self.costToHere = costToHere
        self.costToGoal = costToGoal
        self.myCost = self.costToHere + self.costToGoal

    def getCostToHere(self):
        """Return the cost so far"""
        return self.costToHere

    def getCostToGoal(self):
        """Return the heuristic estimate cost to the goal"""
        return self.costToGoal

    def __str__(self):
        """Create a string for printing that contains the row, col plus path and costs"""
        strng = "[" + str(self.numLabel) + "]"
        strng += "  " + str(self.pathToMe) + " (" + str(self.costToHere)
        strng += " + " + str(self.costToGoal) + ") = " + str(self.myCost)
        return strng


class MacTaskAdvisor(object):
        """This is the task advisor for the Maze task in general. it knows how to determine what a goal
        is, and how to work with the MazeState, and how to generate neighbors in general. There will be
        subclasses of this class for each kind of search, because the details of costs for neighbors vary from
        one algorithm to the next."""

        def __init__(self, mazeMap, startNum, goalNum):
            """Given a map of a maze, the starting and goal locations, this initializes the variables
            that hold details of the problem"""
            self.maze = mazeMap
            self.goalNum= goalNum
            self.startState = self._setupInitialState(startNum)

        def _setupInitialState(self, startNum):
            """This creates and returns a proper start state for this particular
            class. In this case cost is the distance travelled so far, and that
            starts at whatever the starting position has in it."""
            return MacState(startNum, [startNum])

        def getStartState(self):
            """Returns the start state, in the proper form to be used by the search."""
            return self.startState

        def isGoal(self, state):
            """Given a state, check if it is a goal state.  It must have the same row and column
            as the goal"""
            numLabel = state.getLocation()
            if numLabel == self.goalNum:
                return True
            else:
                return False

        def generateNeighbors(self, state):
            """Given a state, determine all legal neighbor states.  This assumes that movements are
            restricted to north, south, east, and west.  It asks the maze map to determine which moves
            are legal for this given map, and it generates the resulting state for each legal move.
            It returns a list of neighbors."""
            (numLabel) = state.getLocation()
            # Move north, if it can
            list = []
            neighbors = self.maze.getNeighbors(numLabel)
            for n in neighbors:                  #loop through list of tuples
                        x = self._buildNeighborState(state, n)
                        list.append(x)           #build neighbors, with function below
            return list

        def _buildNeighborState(self, currState, neighborTuple):
            """Given the current state and the location of the neighbor, this builds
            a new state, computing the cost as appropriate for the class.
            This will be overridden by most subclasses!"""
            newPath = currState.getPath()[:]
            newPath.append(neighborTuple[0])      #append first value in tuple,
            numLabel = neighborTuple[0]        #use the node number for the neighbor value
            return MacState(numLabel, newPath)

class UCSMazeAdvisor(MacTaskAdvisor):
    """This class is a subclass of the MazeTaskAdvisor. It implements the cost calculations
    used for UCS search, and is intended to be paired with a BestFirstSearchSolver."""

    def _setupInitialState(self, startNum):
        """This creates and returns a proper start state for this particular
        class. In this case cost is the distance travelled so far, and that
        starts at whatever the starting position has in it."""
        return MacState(startNum, [startNum], 0)

    def _buildNeighborState(self, currState, neighTuple):
        """Given the current state and the location of the neighbor, this builds
        a new state, computing the cost as appropriate for the class.
        In this case, the cost is the cost in currState plus the cost in the neighbor."""
        newPath = currState.getPath()[:]
        numLabel = neighTuple[0]
        newPath.append(numLabel)
        oldCost = currState.getCost()
        newCost = neighTuple[1]
        newTest = MacState(numLabel, newPath)
        return MacState(numLabel, newPath, oldCost + newCost)


class AStarMacAdvisor(MacTaskAdvisor):
    """This class is a subclass of the MazeTaskAdvisor. It implements the cost calculations
    used for A* search, using the AStarState, which maintains both g and h costs. It is intended to
    be paired with a BestFirstSearchSolver."""

    def _setupInitialState(self, startNum):
        """This creates and returns a proper start state for this particular
        class. In this case, it computes all the two values, g, and h:
        g = the cost of the starting cell
        h = the heuristic distance to goal
        The f cost is automatically computed by the AStarMazeState (f = g + h)
        """
        g = 0 #cost
        h = self._calcDistToGoal(startNum) #heuristic
        return AStarMacState(startNum, [startNum], g, h)
    #g+w*h

    def _buildNeighborState(self, currState, neighTuple):
        """Given the current state and the location of the neighbor, this builds
        a new state, computing the cost as appropriate for the class.
        In this case, we need to update both g and h costs for the new state:
        new g = old g + new cell's weight,
        new h = distance to goal of new cell"""
        newPath = currState.getPath()[:]
        numLabel = neighTuple[0]
        newPath.append(numLabel)
        newG = currState.getCostToHere() + neighTuple[1]
        newH = self._calcDistToGoal(neighTuple[0])
        return AStarMacState(numLabel, newPath, newG, newH)

    def _calcDistToGoal(self, num):
        """Compute the distance to the goal using the city block metric.  Compute
        the difference in row values and in column values, and add them up"""
        Dist = abs(num - self.goalNum)
        return Dist

#
