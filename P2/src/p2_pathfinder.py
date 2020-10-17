from collections import deque
from heapq import heappop, heappush
from math import sqrt


# TODO: Fix decide how and implement fix of boxes having xy flip

def find_path (source_point, destination_point, mesh):

    """
    Searches for a path from source_point to destination_point through the mesh

    Args:
    source_point: starting point of the pathfinder
    destination_point: the ultimate goal the pathfinder must reach
    mesh: pathway constraints the path adheres to

    Returns:

        A path (list of points) from source_point to destination_point if exists
        A list of boxes explored by the algorithm
        """

    path = []
    boxes = {}

    box_source = None
    box_dest   = None

    # Simple individual search, can be combined into search for both
    box_source = find_box(source_point, mesh['boxes'])
    box_dest   = find_box(destination_point, mesh['boxes'])

    # print(box_source, box_dest)
    # path.append(box_dest)
    # path.append(box_source)

    # path = bfs(box_source, box_dest, mesh['adj'])
    path = dsp(box_source, box_dest, mesh['adj'], get_box_costs)
    print('\n')
    # print(path)


    return [source_point, destination_point], path
    #return path, boxes.keys()


def find_box(point, boxes):
    """

    Searches a list of 4-tuples for the box that contains the point

    Args:
    point - a co-ordinate tuple of non-negative integers
    boxes - a list of boxes, where a box is (y1, y2, x1, x2) such that
    (x1, y1) is the point closest to the origin and (x2, y2) is the opposite point

    Returns:
    If the point is contained within a box in the list, the box is returned
    Else if the point is invalid, or not in any box in the list None is returned

    """

    x, y = point

    # Check that the point has non-negative values
    if x < 0 or y < 0:
        return None

    # assume boxes are valid 4-tuples
    for box in boxes:
        x1, x2, y1, y2 = box

        if x1 <= x and x <= x2:
            if y1 <= y and y <= y2:

                return box
    return None


# SEARCHES
def bfs(source_node, dest_node, graph):
    """
    Arguments:
    source_node - starting node of the search
    dest_node   - when node is reached, search is complete
    graph       - a dictionary of adjacent nodes
                    { n1: [ n2, n3, ...], n2: [n1, n4, ...], ... }

    Returns:

        A list of nodes that form a path from source to destination,
        if there is no path then an empty list is returned

    """

    visited = {}
    path    = []

    # Elements are (node, parent)
    queue = deque()
    queue.append((source_node, None))

    while queue:
        node, parent = queue.popleft()
        # Skip nodes that have been previously explored
        if node in visited:
            continue

        visited[node] = parent

        # Search complete
        if node == dest_node:
            path.append(node)

            while parent:
                node = parent
                parent = visited.get(node)
                path.append(node)

            # nodes are added from tail to head
            path.reverse()
            return path


        # Add all adjacent nodes to the queue
        queue.extend([ (n, node) for n in graph[node] ])

    return path

def dsp(initial_position, destination, graph, adj):
    """ Searches for a minimal cost path through a graph using Dijkstra's algorithm.

    Args:
        initial_position: The initial cell from which the path extends.
        destination: The end location for the path.
        graph: A loaded level, containing walls, spaces, and waypoints.
        adj: An adjacency function returning cells adjacent to a given cell as well as their respective edge costs.

    Returns:
        If a path exits, return a list containing all cells from initial_position to destination.
        Otherwise, return None.

    """
    # The priority queue
    queue = [(0, initial_position)]

    # The dictionary that will be returned with the costs
    distances = {}
    distances[initial_position] = 0

    # The dictionary that will store the backpointers
    backpointers = {}
    backpointers[initial_position] = None

    while queue:
        current_dist, current_node = heappop(queue)

        # Check if current node is the destination
        if current_node == destination:

            # List containing all cells from initial_position to destination
            path = [current_node]

            # Go backwards from destination until the source using backpointers
            # and add all the nodes in the shortest path into a list
            current_back_node = backpointers[current_node]
            while current_back_node is not None:
                path.append(current_back_node)
                current_back_node = backpointers[current_back_node]

            return path[::-1]

        # Calculate cost from current note to all the adjacent ones
        for adj_node, adj_node_cost in adj(graph, current_node):
            pathcost = current_dist + adj_node_cost

            ## TODO: use shared edge detection for something
            adj_edge = get_detail_range(current_node, adj_node)
            print(current_node, 'to', adj_node)
            print(adj_edge, '\n')

            # If the cost is new
            if adj_node not in distances or pathcost < distances[adj_node]:
                distances[adj_node] = pathcost
                backpointers[adj_node] = current_node
                heappush(queue, (pathcost, adj_node))

    return None

def box_mid(box_s):
    x1, x2, y1, y2 = box_s
    return (x2-x1, y2-y1)

def dist_linear(point_s, point_d):
    x1, y1 = point_s
    x2, y2 = point_d

    dist = sqrt(abs(x1-x2) + abs(y1-y2))

    return dist

def get_box_costs(graph, box_source, cost_function=dist_linear):
    """

    Returns:
        A list of tuples, (destination, distance_from_source)
    """

    source_mid = box_mid(box_source)

    adj_boxes = graph[box_source]
    adj_costs = []

    for adj_box in adj_boxes:
        adj_mid = box_mid(adj_box)

        adj_costs.append(cost_function(source_mid, adj_mid))

    return list(zip(adj_boxes, adj_costs))


def get_detail_range(box_source, box_dest):
    """

    Returns a 'box' tuple with the ranges of valid co-ordinates
    (x_min, x_max, y_min, y_max). Either the x or y pair will be equal
    """

    ys1, ys2, xs1, xs2 = box_source
    yd1, yd2, xd1, xd2 = box_dest

    x_min = max(xs1, xd1)
    x_max = min(xs2, xd2)
    y_min = max(ys1, yd1)
    y_max = min(ys2, yd2)

    return (x_min, x_max, y_min, y_max)
