from grid import Node, NodeGrid
from math import inf
from grid import CostMap
import heapq


class PathPlanner(object):
    """
    Represents a path planner, which may use Dijkstra, Greedy Search or A* to plan a path.
    """
    def __init__(self, cost_map):
        """
        Creates a new path planner for a given cost map.

        :param cost_map: cost used in this path planner.
        :type cost_map: CostMap.
        """
        self.cost_map = cost_map
        self.node_grid = NodeGrid(cost_map)

    @staticmethod
    def construct_path(goal_node):
        """
        Extracts the path after a planning was executed.

        :param goal_node: node of the grid where the goal was found.
        :type goal_node: Node.
        :return: the path as a sequence of (x, y) positions: [(x1,y1),(x2,y2),(x3,y3),...,(xn,yn)].
        :rtype: list of tuples.
        """
        node = goal_node
        # Since we are going from the goal node to the start node following the parents, we
        # are transversing the path in reverse
        reversed_path = []
        while node is not None:
            reversed_path.append(node.get_position())
            node = node.parent
        return reversed_path[::-1]  # This syntax creates the reverse list

    def dijkstra(self, start_position, goal_position):
        """
        Plans a path using the Dijkstra algorithm.

        :param start_position: position where the planning stars as a tuple (x, y).
        :type start_position: tuple.
        :param goal_position: goal position of the planning as a tuple (x, y).
        :type goal_position: tuple.
        :return: the path as a sequence of positions and the path cost.
        :rtype: list of tuples and float.
        """
        # Inicialização
        self.node_grid.reset()
        start_node = self.node_grid.get_node(start_position[0], start_position[1])
        goal_node = self.node_grid.get_node(goal_position[0], goal_position[1])
        start_node.g = 0

        # Fila de prioridade (min-heap)
        open_list = [(0, start_node)]  # (custo g, nó)
        heapq.heapify(open_list)

        while open_list:
            # Pega o nó com menor custo g
            current_g, current_node = heapq.heappop(open_list)

            # Verifica se o nó atual é o objetivo
            if current_node == goal_node:
                path = self.construct_path(goal_node)
                return path, current_node.g

            # Se o nó já foi fechado, ignora
            if current_node.closed:
                continue

            # Marca o nó atual como fechado
            current_node.closed = True

            # Explora os vizinhos
            successors = self.node_grid.get_successors(current_node.i, current_node.j)
            for successor_pos in successors:
                successor = self.node_grid.get_node(successor_pos[0], successor_pos[1])
                if successor.closed:
                    continue

                # Calcula custo da aresta
                edge_cost = self.cost_map.get_edge_cost(
                    (current_node.i, current_node.j),
                    (successor.i, successor.j)
                )
                new_g = current_node.g + edge_cost

                # Se um caminho melhor for encontrado, atualiza o nó
                if new_g < successor.g:
                    successor.g = new_g
                    successor.parent = current_node
                    heapq.heappush(open_list, (new_g, successor))

        # The first return is the path as sequence of tuples (as returned by the method construct_path())
        # The second return is the cost of the path
        return [], inf

    def greedy(self, start_position, goal_position):
        """
        Plans a path using greedy search.

        :param start_position: position where the planning stars as a tuple (x, y).
        :type start_position: tuple.
        :param goal_position: goal position of the planning as a tuple (x, y).
        :type goal_position: tuple.
        :return: the path as a sequence of positions and the path cost.
        :rtype: list of tuples and float.
        """
		# Todo: implement the Greedy Search algorithm

        # Inicialização
        self.node_grid.reset()
        start_node = self.node_grid.get_node(start_position[0], start_position[1])
        goal_node = self.node_grid.get_node(goal_position[0], goal_position[1])
        start_node.g = 0

        # Fila de prioridade baseada na heurística - distância euclidiana até o objetivo
        open_list = [(start_node.distance_to(goal_node.i, goal_node.j), start_node)] #(distância até objetivo , nó)
        heapq.heapify(open_list)

        while open_list:
            # Pega o nó com menor heurística (distância ao objetivo)
            current_h, current_node = heapq.heappop(open_list)

            if current_node == goal_node:
                path = self.construct_path(goal_node)
                return path, current_node.g

            if current_node.closed:
                continue

            current_node.closed = True

            # Explora os vizinhos
            successors = self.node_grid.get_successors(current_node.i, current_node.j)
            for successor_pos in successors:
                successor = self.node_grid.get_node(successor_pos[0], successor_pos[1])
                if successor.closed:
                    continue

                # Calcula custo real do caminho (para retorno)
                edge_cost = self.cost_map.get_edge_cost(
                    (current_node.i, current_node.j),
                    (successor.i, successor.j)
                )
                new_g = current_node.g + edge_cost

                if new_g < successor.g:
                    successor.g = new_g
                    successor.parent = current_node
                    # Usa apenas a heurística para a fila de prioridade
                    h = successor.distance_to(goal_node.i, goal_node.j)
                    heapq.heappush(open_list, (h, successor))

		# The first return is the path as sequence of tuples (as returned by the method construct_path())
		# The second return is the cost of the path

        return [], inf

    def a_star(self, start_position, goal_position):
        """
        Plans a path using A*.

        :param start_position: position where the planning stars as a tuple (x, y).
        :type start_position: tuple.
        :param goal_position: goal position of the planning as a tuple (x, y).
        :type goal_position: tuple.
        :return: the path as a sequence of positions and the path cost.
        :rtype: list of tuples and float.
        """
		# Todo: implement the A* algorithm
        self.node_grid.reset()
        start_node = self.node_grid.get_node(start_position[0], start_position[1])
        goal_node = self.node_grid.get_node(goal_position[0], goal_position[1])
        start_node.g = 0
        start_node.f = start_node.distance_to(goal_node.i, goal_node.j)

        # Fila de prioridade baseada em f = g + h
        open_list = [(start_node.f, start_node)]
        heapq.heapify(open_list)

        while open_list:
            # Pega o nó com menor f
            current_f, current_node = heapq.heappop(open_list)

            if current_node == goal_node:
                path = self.construct_path(goal_node)
                return path, current_node.g

            if current_node.closed:
                continue

            current_node.closed = True

            # Explora os vizinhos
            successors = self.node_grid.get_successors(current_node.i, current_node.j)
            for successor_pos in successors:
                successor = self.node_grid.get_node(successor_pos[0], successor_pos[1])
                if successor.closed:
                    continue

                # Calcula custo da aresta
                edge_cost = self.cost_map.get_edge_cost(
                    (current_node.i, current_node.j),
                    (successor.i, successor.j)
                )
                new_g = current_node.g + edge_cost

                if new_g < successor.g:
                    successor.g = new_g
                    successor.parent = current_node
                    h = successor.distance_to(goal_node.i, goal_node.j)
                    successor.f = new_g + h
                    heapq.heappush(open_list, (successor.f, successor))

		# The first return is the path as sequence of tuples (as returned by the method construct_path())
		# The second return is the cost of the path

        return [], inf
