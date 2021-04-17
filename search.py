#1 John Eagan
#2 110565786
#3 4/5/21
# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util
from game import Directions


class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"
    node = (problem.getStartState(), 0, 0)
    if problem.isGoalState(node[0]):
        return []
    frontier = util.Stack()
    frontier.push(node)
    reached = [node[0]]
    nodeParents = [(node[0], 0, 0)]  # child state, parent state, action
    actions = []
    while not(frontier.isEmpty()):
        parent = frontier.pop()
        if problem.isGoalState(parent[0]):
            currNode = parent[0]
            startState = problem.getStartState()
            while currNode != startState:
                parentToChild = [(x, y, z) for (x, y, z) in nodeParents if x == currNode]
                action = parentToChild[0][2]
                actions.append(action)
                currNode = parentToChild[0][1]
            actions.reverse()
            return actions
        for child in problem.getSuccessors(parent[0]):
            childState = child[0]
            childAction = child[1]
            if childState not in reached:
                reached.append(childState)
                frontier.push(child)
                nodeParents.append((childState, parent[0], childAction))
            else:
                childParentsIndex = [x for (x, y, z) in nodeParents].index(childState)
                parentParentsIndex = [x for (x, y, z) in nodeParents].index(parent[0])
                if nodeParents[parentParentsIndex][1] != child[0]:
                    nodeParents[childParentsIndex] = (childState, parent[0], childAction)
    return "Failure"
    util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    node = (problem.getStartState(), 0, 0)
    if problem.isGoalState(node[0]):
        return []
    frontier = util.Queue()
    frontier.push(node)
    reached = [node[0]]
    nodeParents = [(node[0], 0, 0)]
    actions = []
    while not (frontier.isEmpty()):
        parent = frontier.pop()
        if problem.isGoalState(parent[0]):
            currNode = parent[0]
            startState = problem.getStartState()
            while currNode != startState:
                parentToChild = [(x, y, z) for (x, y, z) in nodeParents if x == currNode]
                action = parentToChild[0][2]
                actions.append(action)
                currNode = parentToChild[0][1]
            actions.reverse()
            return actions
        for child in problem.getSuccessors(parent[0]):
            childState = child[0]
            childAction = child[1]
            if childState not in reached:
                reached.append(childState)
                frontier.push(child)
                nodeParents.append((childState, parent[0], childAction))
    return "Failure"
    util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    node = (problem.getStartState(), 0, 0)
    frontier = util.PriorityQueue()
    frontier.push(node, node[2])
    reached = [(node[0], 0)]
    nodeParents = []
    actions = []
    while not frontier.isEmpty():
        parent = frontier.pop()
        if problem.isGoalState(parent[0]):
            currNode = parent[0]
            startState = problem.getStartState()
            while currNode != startState:
                parentToChild = [(x, y, z) for (x, y, z) in nodeParents if x == currNode]
                action = parentToChild[0][2]
                actions.append(action)
                currNode = parentToChild[0][1]
            actions.reverse()
            return actions
        for child in problem.getSuccessors(parent[0]):
            if parent[0] == problem.getStartState():
                parentPathCost = 0
            else:
                parentPathCost = [y for (x, y) in reached if x == parent[0]][0]
            childState = child[0]
            childAction = child[1]
            actionCost = child[2]
            childPathCost = parentPathCost + actionCost
            child = (child, childPathCost)

            childInReached = childState in [x for (x, y) in reached]
            if childInReached:
                childReachedIndex = [x for (x, y) in reached].index(childState)
                if childState != problem.getStartState():
                    nodeParentsIndex = [x for (x, y, z) in nodeParents].index(childState)
                childReachedPathCost = reached[childReachedIndex][1]
            else:
                childReachedPathCost = 9999999999
            if not childInReached:
                reached.append((childState, childPathCost))
                frontier.push(child[0], child[1])
                nodeParents.append((childState, parent[0], childAction))
            elif childPathCost < childReachedPathCost:
                reached[childReachedIndex] = (childState, childPathCost)
                frontier.push(child[0], child[1])
                if childState != problem.getStartState():
                    nodeParents[nodeParentsIndex] = (childState, parent[0], childAction)
    return "Failure"
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    node = (problem.getStartState(), 0, 0)
    frontier = util.PriorityQueue()
    frontier.push(node, node[2] + heuristic(node[0], problem))
    reached = [(node[0], 0, heuristic(node[0], problem))]
    nodeParents = []
    actions = []
    expanded = []
    while not frontier.isEmpty():
        parent = frontier.pop()
        #print("Parent:", parent[0], "h:", heuristic(parent[0],problem))
        if parent[0] in expanded:
            continue
        expanded.append(parent[0])
        if problem.isGoalState(parent[0]):
            currNode = parent[0]
            startState = problem.getStartState()
            while currNode != startState:
                parentToChild = [(x, y, z) for (x, y, z) in nodeParents if x == currNode]
                action = parentToChild[0][2]
                actions.append(action)
                currNode = parentToChild[0][1]
            actions.reverse()
            return actions
        for child in problem.getSuccessors(parent[0]):
            if parent[0] == problem.getStartState():
                parentPathCost = 0
            else:
                parentPathCost = [y for (x, y, z) in reached if x == parent[0]][0]
            childState = child[0]
            childAction = child[1]
            actionCost = child[2]
            childPathCost = parentPathCost + actionCost
            childTotalCost = childPathCost + heuristic(childState, problem)
            #print("Child:", child[0], "f:", childTotalCost, "g:", childPathCost, "h:", heuristic(childState,problem))

            child = (child, childPathCost, childTotalCost)

            childInReached = childState in [x for (x, y, z) in reached]
            if childInReached:
                childReachedIndex = [x for (x, y, z) in reached].index(childState)
                if childState != problem.getStartState():
                    nodeParentsIndex = [x for (x, y, z) in nodeParents].index(childState)
                childReachedTotalCost = reached[childReachedIndex][2]
            else:
                childReachedTotalCost = 9999999999
            if not childInReached:
                reached.append((childState, childPathCost, childTotalCost))
                frontier.push(child[0], child[2])
                nodeParents.append((childState, parent[0], childAction))
            elif childTotalCost < childReachedTotalCost:
                reached[childReachedIndex] = (childState, childPathCost, childTotalCost)
                frontier.push(child[0], child[2])
                if childState != problem.getStartState():
                    nodeParents[nodeParentsIndex] = (childState, parent[0], childAction)
    return "Failure"
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
