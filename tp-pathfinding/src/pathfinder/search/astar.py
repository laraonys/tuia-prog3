from ..models.grid import Grid
from ..models.frontier import PriorityQueueFrontier
from ..models.solution import NoSolution, Solution
from ..models.node import Node


class AStarSearch:
    @staticmethod
    def search(grid: Grid) -> Solution:
        """Find path between two points in a grid using A* Search

        Args:
            grid (Grid): Grid of points

        Returns:
            Solution: Solution found
        """
        # Initialize root node
        root = Node("", state=grid.initial, cost=0, parent=None, action=None)

        # Initialize reached with the initial state
        reached = {}
        reached[root.state] = root.cost

        frontier = PriorityQueueFrontier()
        

        h = abs(root.state[0] - grid.end[0]) + abs(root.state[1] - grid.end[1])

        priority = root.cost + h
        frontier.add(root, priority)

        # Main loop
        while not frontier.is_empty():
            
            node = frontier.pop()

            # Check if we reached the goal
            if grid.objective_test(node.state):
                return Solution(node, reached)

            # Expand current node
            for action in grid.actions(node.state):
                successor = grid.result(node.state, action)
                
                # Calculate new cost
                new_cost = node.cost + grid.individual_cost(node.state, action)

                if successor not in reached or new_cost < reached[successor]:
                    reached[successor] = new_cost
                    
                    # Create successor node
                    son = Node(
                        "",
                        successor,
                        cost=new_cost,
                        parent=node,
                        action=action,
                    )
                    
                    # Calculate heuristic for successor
                    h = abs(successor[0] - grid.end[0]) + abs(successor[1] - grid.end[1])
                    priority = new_cost + h
                    frontier.add(son, priority)

        return NoSolution(reached)