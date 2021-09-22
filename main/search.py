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

from builtins import object
import util
from game import Directions


class SearchProblem(object):
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

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """

    visited = []
    moves = []

    depth = 0

    start_coord = problem.getStartState()
    visited.append(start_coord)

    if (problem.isGoalState(start_coord) == True):
        return moves
    else:
        stack = util.Stack()
        for child in problem.getSuccessors(start_coord):
            stack.push((child, depth + 1))
            

        while (stack.isEmpty() == False):
            node, dep = stack.pop()
            coord = node[0]
            dir = node[1]
            depth = dep

            visited.append(coord)
            moves = moves[:dep - 1]
            moves.append(dir)

            #print("Visiting", coord, dep)

            if (not problem.isGoalState(coord)):
                #print(problem.getSuccessors(coord))
                for c in problem.getSuccessors(coord):
                    if c[0] not in visited:
                        stack.push((c, depth + 1))
                        
            else:
                print("found goal")
                break

        print(moves)
        return moves


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    visited = [problem.getStartState()]

    if (problem.isGoalState(problem.getStartState())):
        return []
    
    queue = util.Queue()
    for n in problem.getSuccessors(problem.getStartState()):
        queue.push([n])
    while (not queue.isEmpty()):
        path = queue.pop()
        last = path[-1][0]

        if (problem.isGoalState(last)):
            moves = []
            for n in path:
                print(n[0])
                moves.append(n[1])
            print(moves)
            return moves
        elif (last not in visited):
            for n in problem.getSuccessors(last):
                if (n[0] not in visited):
                    p = list(path)
                    p.append(n)
                    queue.push(p)
            visited.append(last)


def uniformCostSearch(problem, heuristic=None):
    """Search the node of least total cost first."""

    def getCost(path):
        cost = 0
        for i in path:
            cost += i[2]
        return cost

    start = problem.getStartState()
    pr_Q = util.PriorityQueue()
    visited = [start]
    moves = []

    print("Start:", start)

    for n in problem.getSuccessors(start):
        print(n)
        pr_Q.push([n], n[2])

    while (not pr_Q.isEmpty()):
        path = pr_Q.pop()
        node = path[-1][0]

        if (node not in visited):
            visited.append(node)

            if (problem.isGoalState(node)):
                for n in path:
                    moves.append(n[1])
                print(moves)
                return moves

            for n in problem.getSuccessors(node):
                if (n[0] not in visited):
                    new_path = list(path)
                    new_path.append(n)
                    pr_Q.update(new_path, getCost(new_path))

    return moves




def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    def getCost(path):
        cost = 0
        for i in path:
            cost += i[2]
        return cost

    start = problem.getStartState()
    pr_Q = util.PriorityQueue()
    visited = [start]
    moves = []

    print("Start:", start)

    for n in problem.getSuccessors(start):
        print(n)
        pr_Q.push([n], n[2] + heuristic(n[0], problem))

    while (not pr_Q.isEmpty()):
        path = pr_Q.pop()
        node = path[-1][0]

        if (node not in visited):
            visited.append(node)

            if (problem.isGoalState(node)):
                for n in path:
                    moves.append(n[1])
                print(moves)
                return moves

            for n in problem.getSuccessors(node):
                if (n[0] not in visited):
                    new_path = list(path)
                    new_path.append(n)
                    pr_Q.update(new_path, getCost(new_path) + heuristic(n[0], problem))

    return moves


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
