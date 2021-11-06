# Yael Simhis 209009604
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


class Node:
    def __init__(self, state, path=None):
        if path is None:
            path = []
        self.state = state
        self.path = path.copy()

    def get_state(self):
        return self.state

    def get_path(self):
        return self.path


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
    closed_list = set()  # Set to keep track of visited nodes of graph
    open_list = util.Stack()
    open_list.push(Node(problem.getStartState()))  # Push the first state

    # while the stack is not empty
    while not open_list.isEmpty():
        get_node = open_list.pop()
        if problem.isGoalState(get_node.get_state()):
            return get_node.get_path()
        # get the next node
        for next_step in problem.getSuccessors(get_node.get_state()):
            path = get_node.get_path().copy()
            path.append(next_step[1])
            next_node = Node(next_step[0], path)
            # check if we already visited in this node
            if next_node.get_state() not in closed_list:
                # if not then push to the stack
                open_list.push(next_node)
        # add node to the visited list
        closed_list.add(get_node.get_state())
    return None


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    closed_list = set()  # Set to keep track of visited nodes of graph
    open_list = util.Queue()
    open_list.push(Node(problem.getStartState()))  # Push the first state

    # while the queue is not empty
    while not open_list.isEmpty():
        get_node = open_list.pop()
        if problem.isGoalState(get_node.get_state()):
            return get_node.get_path()
        # add node to the visited list
        closed_list.add(get_node.get_state())
        # get the next node
        for next_step in problem.getSuccessors(get_node.get_state()):
            path = get_node.get_path().copy()
            path.append(next_step[1])
            next_node = Node(next_step[0], path)
            # check if we already visited in this node
            if next_node.get_state() not in closed_list:
                # if not then push to the queue
                open_list.push(next_node)
    return None

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    closed_list = set()  # Set to keep track of visited nodes of graph
    open_list = util.PriorityQueue()
    open_list.push(Node(problem.getStartState()), 0)  # Push the first state and cost 0

    # while the queue is not empty
    while not open_list.isEmpty():
        get_node = open_list.pop()
        if problem.isGoalState(get_node.get_state()):
            return get_node.get_path()
        if get_node.get_state() in closed_list:
            continue
        # get the next node
        for next_step in problem.getSuccessors(get_node.get_state()):
            path = get_node.get_path().copy()
            path.append(next_step[1])
            next_node = Node(next_step[0], path)
            # check if we already visited in this node
            if next_node.get_state() not in closed_list:
                # if not then push to the queue and add the new cost
                open_list.push(next_node, problem.getCostOfActions(next_node.get_path()))
        # add node to the visited list
        closed_list.add(get_node.get_state())
    return None

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    closed_list = set()  # Set to keep track of visited nodes of graph
    open_list = util.PriorityQueue()
    start_node = problem.getStartState()
    open_list.push(Node(start_node), nullHeuristic(start_node, problem))  # Push the first state

    # while the queue is not empty
    while not open_list.isEmpty():
        get_node = open_list.pop()
        if problem.isGoalState(get_node.get_state()):
            return get_node.get_path()
        if get_node.get_state() in closed_list:
            continue
        # get the next node
        for next_step in problem.getSuccessors(get_node.get_state()):
            path = get_node.get_path().copy()
            path.append(next_step[1])
            next_node = Node(next_step[0], path)
            # check if we already visited in this node
            if next_node.get_state() not in closed_list:
                # if not then push to the queue and add the new cost
                heuristic_cost = problem.getCostOfActions(next_node.get_path())\
                                 + heuristic(next_node.get_state(), problem)
                open_list.push(next_node, heuristic_cost)
        # add node to the visited list
        closed_list.add(get_node.get_state())
    return None


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
