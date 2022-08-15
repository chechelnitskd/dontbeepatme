"""  =================================================================
File: SearchSolver.py
This file contains generic definitions for a state space SearchState class,
and a general SearchSolver class.  These classes should be subclassed to make
solvers for a specific problem.
 ==================================================================="""


verbose = False


class BestFirstSearchSolver(object):
    """This class contains a priority-queue based search algorithm. The Priority-Queue Search can act like
    any best-first search, including UCS and A*, depending on how the "cost" is calculated.  This class contains
    stubs for the helper methods that those algorithms need.  Only the isGoal and generateNeighbors methods
    should be overridden by the subclass.
    These algorithms assume that the qData stored in the states implement the equality operators properly!"""

    def __init__(self, taskAdvisor):
        """Creates a Best-First search solver, with the given task advisor. This takes in a "task advisor" and
        sets up the qData needed for the search, the fringe and visited sets, and the counts of
        how many nodes were created and visited.
        The only generic qData are instance variables that count the number of nodes created and the number of
        nodes visited (that corresponds more or less to the number of nodes added to the queue and the number of nodes
        removed from the queue (and not found to be redundant)).  In addition, there are instance variables for the
        search queues for both BFS and PQSearch, so that we can step through the algorithms rather than just running
        them all at once"""
        self.taskAdvisor = taskAdvisor
        self.nodesCreated = 0
        self.nodesVisited = 0
        self.fringe = None
        self.visited = None

    def _initializeCounts(self):
        """A private helper to initialize the counts, since they need to be initialized anew each time the
        search algorithms are called"""
        self.nodesCreated = 0
        self.nodesVisited = 0

    def getNodesCreated(self):
        """Returns the value of self.nodesCreated"""
        return self.nodesCreated

    def getNodesVisited(self):
        """Returns the value of self.nodesVisited"""
        return self.nodesVisited

    def initSearch(self):
        """This method sets up a priority-queue-based search process, initializing the fringe queue, the set of
        visited states, and adding the start state to the fringe queue."""
        self._initializeCounts()
        startState = self.taskAdvisor.getStartState()
        print(startState)
        if self.taskAdvisor.isGoal(startState):
            return startState.getPath()
        self.visited = set()
        self.fringe = PriorityQueue()
        self.fringe.insert(startState, startState.getCost())
        self.nodesCreated += 1

    def searchLoop(self):
        """This method runs the search, repeatedly calling for the next step until either
        the search fails and False is returned, or the search completes"""
        while True:
            (nextState, neighbors, isDone) = self.searchStep()
            if nextState == "Fail":
                return False
            elif isDone == "Done":
                # is search is done then nextState actually holds the result
                return nextState
            # Otherwise just do another step of the search

    def searchStep(self):
        """This method performs one step of a priority-queue search. It finds the next node in
        the priority queue, generates its children, and adds the appropriate ones to the priority queue
        It returns three values: the current state, the neighbors of the current state, and a status
        message.  The message is either "Done", "Fail", or "Not Done" for a normal step."""
        newNeighbors = []
        if self.fringe.isEmpty():
            return (False, False, "Fail")
        nextState, priority = self.fringe.delete()
        if self.taskAdvisor.isGoal(nextState):
            return (nextState, [], "Done")  # when hit goal, neighbors are irrelevant

        # Otherwise, go on
        if verbose:
            print("----------------------")
            print("Current state:", nextState)
        neighbors = self.taskAdvisor.generateNeighbors(nextState)
        self.visited.add(nextState)
        self.nodesVisited += 1

        for n in neighbors:
            visitedMatch = self._hasBeenVisited(n)
            fringeMatch = self._hasBeenFringed(n)

            if (not visitedMatch) and (not fringeMatch):
                if verbose:
                    print("    Neighbor never seen before:", n)
                # this node has not been generated before, add it to the fringe
                self.fringe.insert(n, n.getCost())
                newNeighbors.append(n)
                self.nodesCreated += 1
            elif visitedMatch and visitedMatch.getCost() > n.getCost():
                # if state was visited before but this one is better, add this one to fringe set
                if verbose:
                    print("    Neighbor was already in explored, cost is lower now", n, visitedMatch.getCost(),
                          n.getCost())
                self.fringe.insert(n, n.getCost())
                self.visited.discard(visitedMatch)  # Must update the cost associated with this node in the visited set
                self.visited.add(n)
                newNeighbors.append(n)
                self.nodesCreated += 1
            elif fringeMatch and fringeMatch.getCost() > n.getCost():
                # if state is in fringe but this one is better, add this one to fringe AND
                if verbose:
                    print("    Neighbor state was already in fringe, cost is lower now", n, fringeMatch.getCost(),
                          n.getCost())

                # remove the old one from the fringe
                self.fringe.removeValue(fringeMatch)
                self.fringe.insert(n, n.getCost())
                newNeighbors.append(n)
                self.nodesCreated += 1
            elif visitedMatch:
                if verbose:
                    print("    Neighbor was already in explored, skipping", n)
            elif fringeMatch:
                if verbose:
                    print("    Neighbor was already in fringe, skipping", n)

        # end for
        return nextState, newNeighbors, "Not Done"

    def _hasBeenVisited(self, state):
        """Given a state, it looks through the visited set and seeks a node
        that is "equal" to the input state. It is up to the subclass to
        define what it means for them to be equal. It returns the matching
        state, if any, or False if none"""
        for s in self.visited:
            if s == state:
                return s
        return False

    def _hasBeenFringed(self, state):
        """Given a state, it looks through the fringe set and seeks a node that is "equal" to the
        input state.  It is up to the objects to define what it means for them to be equal.  It
        returns the matching state, if any, or False if none"""
        foundInfo = self.fringe.contains(state)
        if foundInfo:
            return foundInfo
        else:
            return False


class NoCostSearchSolver(object):
    """This class contains a stack or queue search algorithm, so that it can do either BFS or DFS depending on whether
    it is instructed to use a stack or queue. This class contains stubs for the helper methods that those algorithms
    need.  Only the isGoal and generateNeighbors methods should be overridden by the subclass.  BFS is not
    guaranteed to give the best solution on a weighted graph, though it will always give the solution with the least
    edges.  DFS is not guaranteed to give the best solution, ever, but it is more memory-efficient.
    These algorithms assume that the states implement the comparison operators correctly!"""

    def __init__(self, taskAdvisor, mode='BFS'):
        """Takes in the task advisor, and an optional mode, which selects DFS or BFS.
        Sets up the qData needed for the search, the fringe and visited sets, and the counts of
        how many nodes were created and visited. The only generic qData are instance variables that count
        the number of nodes created and the number of nodes visited (that corresponds more or less to the
        number of nodes added to the queue and the number of nodes removed from the queue (and not found to be
        redundant))."""
        """This takes in a "task advisor" and sets up the qData needed for the search, the fringe and visited sets, and the counts of
        how many nodes were created and visited.
        The only generic qData are instance variables that count the number of nodes created and the number of
        nodes visited (that corresponds more or less to the number of nodes added to the queue and the number of nodes
        removed from the queue (and not found to be redundant)).  In addition, there are instance variables for the
        search queues for both BFS and PQSearch, so that we can step through the algorithms rather than just running
        them all at once"""
        self.taskAdvisor = taskAdvisor
        self.nodesCreated = 0
        self.nodesVisited = 0
        self.fringe = None
        self.visited = None
        self.mode = mode

    def _initializeCounts(self):
        """A private helper to initialize the counts, since they need to be initialized anew each time the
        search algorithms are called"""
        self.nodesCreated = 0
        self.nodesVisited = 0

    def getNodesCreated(self):
        """Returns the value of self.nodesCreated"""
        return self.nodesCreated

    def getNodesVisited(self):
        """Returns the value of self.nodesVisited"""
        return self.nodesVisited

    def initSearch(self):
        """This method sets up a priority-queue-based search process, initializing the fringe queue, the set of
        visited states, and adding the start state to the fringe queue."""
        self._initializeCounts()
        startState = self.taskAdvisor.getStartState()
        if self.taskAdvisor.isGoal(startState):
            return startState.getPath()
        self.visited = set()
        if self.mode == "BFS":
            self.fringe = Queue()
        else:
            self.fringe = Stack()
        self.fringe.insert(startState)
        self.nodesCreated += 1

    def _setupFringe(self, startState):
        """This method sets up the proper kind of fringe set for this particular search.
        In this case, it creates either a Queue or a Stack, depending on whether we are doing
        BFS or DFS, and it inserts the start state into it."""
        if self.mode == "BFS":
            self.fringe = Queue()
        else:
            self.fringe = Stack()
        self.fringe.insert(startState)

    def searchLoop(self):
        """This method runs the search, repeatedly calling for the next step until either
        the search fails and False is returned, or the search completes"""
        while True:
            (nextState, neighbors, isDone) = self.searchStep()
            if nextState == "Fail":
                return False
            elif isDone == "Done":
                # is search is done then nextState actually holds the result
                return nextState
            # Otherwise just do another step of the search

    def searchStep(self):
        """This method performs one step of a stack or queue search. It finds the next node in
        the stack/queue, generates its children, and adds the appropriate ones to the stack/queue.
        It returns three values: the current state, the neighbors of the current state, and a status
        message.  The message is either "Done", "Fail", or "Not Done" for a normal step."""
        newNeighbors = []
        if self.fringe.isEmpty():
            return (False, False, "Fail")
        nextState = self.fringe.delete()
        if self.taskAdvisor.isGoal(nextState):
            return (nextState, [], "Done")  # when hit goal, neighbors are irrelevant

        # Otherwise, go one
        if verbose:
            print("----------------------")
            print("Current state:", nextState)
        neighbors = self.taskAdvisor.generateNeighbors(nextState)
        self.visited.add(nextState)
        self.nodesVisited += 1

        for n in neighbors:
            visitedMatch = self._hasBeenVisited(n)
            fringeMatch = self._hasBeenFringed(n)

            if (not visitedMatch) and (not fringeMatch):
                if verbose:
                    print("    Neighbor never seen before", n)
                # this node has not been generated before, add it to the fringe
                self.fringe.insert(n)
                newNeighbors.append(n)
                self.nodesCreated += 1
            elif visitedMatch:
                if verbose:
                    print("    Neighbor was already in explored, skipping", n)
            elif fringeMatch:
                if verbose:
                    print("    Neighbor was already in fringe, skipping", n)

        # end for
        return nextState, newNeighbors, "Not Done"

    def _hasBeenVisited(self, state):
        """Given a state, it looks through the visited set and seeks a node
        that is "equal" to the input state. It is up to the state class to
        define what it means for them to be equal. It returns the matching
        state, if any, or False if none"""
        for s in self.visited:
            if s == state:
                return s
        return False

    def _hasBeenFringed(self, state):
        """Given a state, it looks through the fringe set and seeks a node that is "equal" to the
        input state.  It is up to the objects to define what it means for them to be equal.  It
        returns the matching state, if any, or False if none"""
        foundInfo = self.fringe.contains(state)
        if foundInfo:
            return foundInfo
        else:
            return False


"""####################################################
A Queue class
Susan Fox
Spring 2007
Updated Spring 2014 to fix comment style
Updated Spring 2016 to add methods to priority queue for removing qData from the queue.
Updated Fall 2018 to make classes consistent, to switch the ordering of inputs to the insert methods,
and to add a contains method. Now the classes assume that the object being stored implements the == operator
in a meaningful way.
"""


class Queue:
    """A queue is a linear collection used to hold qData that is waiting
    for some purpose.  The first to enter the queue is the first to
    leave it."""

    def __init__(self, valList=None):
        """When creating a new queue, you can give a list of values to
        insert in the queue at the start."""
        if valList is None:
            self.qData = []
        else:
            self.qData = valList[:]
        self.size = len(self.qData)

    def getSize(self):
        """Return the size of the queue."""
        return self.size

    def isEmpty(self):
        """Returns true if the queue is empty, or false otherwise."""
        return self.size == 0

    def firstElement(self):
        """Returns the first value in the queue, without removing it."""
        if self.isEmpty():
            return None
        else:
            return self.qData[0]

    def peek(self):
        """Another name for firstElement."""
        return self.firstElement()


    def insert(self, val):
        """Inserts a new value at the end of the queue."""
        self.qData.append(val)
        self.size = self.size + 1

    def enqueue(self, val):
        """Another name for inserting."""
        self.insert(val)

    def delete(self):
        """Removes the first element from the queue, returning it as its value."""
        if self.isEmpty():
            return None
        else:
            firstData = self.qData.pop(0)
            self.size = self.size - 1
            return firstData

    def dequeue(self):
        """Another name for deleting: removes the first element from the queue, returning it as its value."""
        return self.delete()

    def contains(self, value):
        """Takes in a value and returns true if the item is in the queue. Assumes the item implements
        the __eq__ operation."""
        for item in self.qData:
            if value == item:
                return item
        return False


    def __str__(self):
        """Creates a string containing the qData, just for debugging."""
        qstr = "Queue: <- "
        if self.size <= 3:
            for val in self.qData:
                qstr = qstr + str(val) + " "
        else:
            for i in range(3):
                qstr = qstr + str(self.qData[i]) + " "
            qstr = qstr + "..."
        qstr = qstr + "<-"
        return qstr
# end class Queue



class PriorityQueue(Queue):
    """A priority queue puts lowest-cost elements first.
    Implemented with a MinHeap, which is internal to the class"""

    def __init__(self, valList=None):
        """Has two optional inputs. The first is a list to populate the queue with, which must
        be a list of tuples, where each tuple contains a value and that value's priority. The second
        optional input is a function used to compare elements of the priority queue to determine which has
        higher priority. This allows more complex priorities, including lists of priority values to be handled.
        """
        Queue.__init__(self)
        self.qData = []
        self.size = 0
        if valList is not None:
            for (val, prior) in valList:
                self.insert(val, prior)


    def insert(self, value, priority):
        """Inserts a new value at the end of the queue."""
        self.qData.append((value, priority))
        self.size = self.size + 1
        self._walkUp(self.size - 1)

    def enqueue(self, val, priority):
        """Another name for inserting"""
        self.insert(val, priority)

    def _walkUp(self, index):
        """Walk a value up the heap until it is larger than its parent
        This is really a *private* method, no one outside should call it.
        Thus the underscore leading the name."""
        inPlace = False
        while not(index == 0) and not inPlace:
            parentIndex = self._parent(index)
            curr = self.qData[index]
            par = self.qData[parentIndex]
            if curr[1] >= par[1]:
                inPlace = True
            else:
                self.qData[index] = par
                self.qData[parentIndex] = curr
                index = parentIndex

    def delete(self):
        """Removes the first element from the queue, returning it as its value, or returning None if
        the queue is already empty."""
        if self.size == 0:
            return None
        elif self.size == 1:
            poppedElement = self.qData[0]
            self.size = self.size - 1
            self.qData = []
            return poppedElement
        else:
            poppedElement = self.qData[0]
            self.size = self.size - 1
            lastItem = self.qData.pop(self.size)
            self.qData[0] = lastItem
            self._walkDown(0)
            return poppedElement

    def dequeue(self):
        """Another name for deleting, removes the first element from the queue, returning it as its value"""
        return self.delete()

    def _walkDown(self, index):
        """A private method, walks a value down the tree until it is smaller than both its children."""
        inPlace = False
        leftInd = self._leftChild(index)
        rightInd = self._rightChild(index)
        while not(leftInd >= self.size) and not inPlace:
            curr = self.qData[index]

            if (rightInd >= self.size):
                minInd = leftInd
            else:
                leftVal = self.qData[leftInd]
                rightVal = self.qData[rightInd]
                if leftVal[1] <= rightVal[1]:
                    minInd = leftInd
                else:
                    minInd = rightInd

            minVal = self.qData[minInd]
            if curr[1] <= minVal[1]:
                inPlace = True
            else:
                self.qData[minInd] = curr
                self.qData[index] = minVal
                index = minInd
                leftInd = self._leftChild(index)
                rightInd = self._rightChild(index)


    def update(self, value, newP):
        """Update finds the given value in the queue, changes its priority value, and then moves it
        up or down the tree as appropriate."""
        pos = self._findValue(value)
        [oldP, v] = self.qData[pos]
        self.qData[pos] = [newP, value]
        if oldP > newP:
            self._walkUp(pos)
        else:
            self._walkDown(pos)


    def contains(self, value):
        """Takes in a value and searches for it in the priority queue. If it is there, it returns True,
        otherwise False."""
        pos = self._findValue(value)
        if pos < 0:
            # value not found
            return False
        else:
            return self.qData[pos][0]


    def removeValue(self, value):
        """Takes in a value and searches for it, and then removes it from the queue, wherever it is."""
        pos = self._findValue(value)
        if pos < 0:
            # If value not found
            print("Value not found:", value)
        elif self.size == 1:
            # If only one value left, make heap empty
            self.size = self.size - 1
            self.qData = []
        elif pos == (self.size - 1):
            # if removed value is last one, just remove it
            self.size = self.size - 1
            self.qData.pop(self.size)
        else:
            self.size = self.size - 1
            lastItem = self.qData.pop(self.size)
            self.qData[pos] = lastItem
            self._walkDown(pos)


    def _findValue(self, value):
        """Find the position of a value in the priority queue."""
        i = 0
        for [val, prior] in self.qData:
            if val == value:
                return i
            i = i + 1
        return -1



    # The following helpers allow us to figure out
    # which value is the parent of a given value, and which
    # is the right child or left child.

    def _parent(self, index):
        """Private: find position of parent given position of heap node."""
        return (index - 1) // 2

    def _leftChild(self, index):
        """Private method: find position of left child given position of heap node."""
        return (index * 2) + 1

    def _rightChild(self, index):
        """Private method: find position of right child given position of heap node."""
        return (index + 1) * 2

    def __str__(self):
        """Provides a string with just the first element."""
        val = "PQueue: "
        if self.isEmpty():
            val += "<empty>"
        else:
            p, v = self.firstElement()
            val = val + "priority: " + str(p) + ", value: " + str(v)
        return val

"""####################################################
A Stack class
Susan Fox
Spring 2007
Revised Spring 2014 to update style.
Revised Fall 2018 to match changes to Queue and PriorityQueue classes, adding a "contains" method.
"""

class Stack:
    """A stack is a linear collection used to hold qData that is waiting
    for some purpose.  Values are added at one end and removed from the
    same end, like a stack of plates"""

    def __init__(self, vallist=[]):
        """When creating a new stack, you can give a list of values to
        insert in the stack at the start.  The front of the list becomes
        the top of the stack."""
        self.data = vallist[:]
        self.size = len(self.data)

    def getSize(self):
        """Returns the size of the stack"""
        return self.size


    def isEmpty(self):
        """Returns true if the stack is empty, or false otherwise."""
        return self.size == 0


    def firstElement(self):
        """Another name for top"""
        return self.top()

    def peek(self):
        """Same as firstElement or top"""
        return self.top()

    def top(self):
        """Returns the first value in the stack, without removing it."""
        if self.isEmpty():
            return None
        else:
            return self.data[0]


    def insert(self, val):
        """Inserts a new value at the end of the stack."""
        self.data.insert(0,val)
        self.size = self.size + 1

    def push(self, val):
        """Another name for inserting"""
        self.insert(val)


    def delete(self):
        """Removes the first element from the stack, returning its value."""
        first = self.data[0]
        self.data.pop(0)
        self.size = self.size - 1
        return first

    def pop(self):
        """Another name for deleting"""
        return self.delete()


    def contains(self, value):
        """Given a value, it searches the stack for a matching value, returning it if found, or returning False if not found"""
        for item in self.data:
            if item == value:
                return item
        return False


    def __str__(self):
        """Creates a string containing the qData, just for debugging."""
        stackStr = "Stack: <- "
        if self.size <= 3:
            for val in self.data:
                stackStr = stackStr + str(val) + " "
        else:
            for i in range(3):
                stackStr = stackStr + str(self.data[i]) + " "
            stackStr = stackStr + "..."
        stackStr = stackStr + "]"
        return stackStr
# end class Stack