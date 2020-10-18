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

    #print("Source:", source_point)
    #print("Source box:", box_source)
    #print("Destination:", destination_point)

    # print(box_source, box_dest)
    # path.append(box_dest)
    # path.append(box_source)
    if(box_source and box_dest):
        #path = bfs(box_source, box_dest, mesh['adj'])
        path, points_path = dsp(source_point, destination_point, mesh, get_box_costs)
        print('\n')

    # print(path)

    # Print there is no path if path is empty
    if not path or not box_source or not box_dest:
        print("No Path!!")

    return points_path, path
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

    y, x = point

    # Check that the point has non-negative values
    if y < 0 or x < 0:
        return None

    # assume boxes are valid 4-tuples
    for box in boxes:
        y1, y2, x1, x2 = box

        if y1 <= y and y <= y2:
            if x1 <= x and x <= x2:

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
        initial_position: The initial box from which the path extends. 4-tuple: (y1, y2, x1, x2)
        destination: The end box for the path. 4-tuple: (y1, y2, x1, x2)
        graph: Adj dict of neighboring boxes to a box
        adj: Closest box function returning box list of boxes and their cost to reach from intitial position.

    Returns:
        If a path exits, return a list containing all cells from initial_position to destination.
        Otherwise, return None.

    """
    box_source = find_box(initial_position, graph['boxes'])
    box_dest   = find_box(destination, graph['boxes'])

    # The priority queue
    queue = [(0, box_source, initial_position)]

    # The dictionary that will be returned with the costs
    distances = {}
    distances[box_source] = 0

    # The dictionary that will store the backpointers
    backpointers = {}
    backpointers[box_source] = None

    # The dictionary that will store the detail points of the path key: (y,x); value:(y,x)
    detail_points = {}
    detail_points[initial_position] = None

    while queue:
        current_dist, current_box, current_point = heappop(queue)

        # Check if current node is the destination
        if current_box == box_dest:
            # List containing all cells from initial_position to destination
            path = [current_box]

            # Go backwards from destination until the source using backpointers
            # and add all the nodes in the shortest path into a list
            current_back_box = backpointers[current_box]
            while current_back_box is not None:
                path.append(current_back_box)
                current_back_box = backpointers[current_back_box]

            # build detail_points_path
            detail_points_path = [destination, current_point]
            
            current_parent_point = detail_points[current_point]
            i = 0
            while current_parent_point is not None and i < 1000:
                print(current_parent_point)
                i += 1
                detail_points_path.append(current_parent_point)
                current_parent_point = detail_points[current_parent_point]

            print('initial:', initial_position, 'goal:', destination)
            print('point path:', detail_points_path)
            return path[::-1], detail_points_path[::-1]

        # Calculate cost from current note to all the adjacent ones
        for adj_box, adj_box_cost in adj(graph["adj"], current_box):
            pathcost = current_dist + adj_box_cost

            adj_edge = get_detail_range(current_box, adj_box)
            adj_point = get_closest_detail_point(current_point, adj_edge)
            print('point:', adj_point)
            # print(current_node, 'to', adj_node)
            # print(adj_edge, '\n')

            # If the cost is new
            if adj_box not in distances or pathcost < distances[adj_box]:
                distances[adj_box] = pathcost
                backpointers[adj_box] = current_box
                detail_points[adj_point] = current_point
                heappush(queue, (pathcost, adj_box, adj_point))

    return None

def box_mid(box_s):
    y1, y2, x1, x2 = box_s
    return ((y2+y1)/2, (x2+x1)/2)

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

    return (y_min, y_max, x_min, x_max)

def get_closest_detail_point(source_point, detail_range):

    yr1, yr2, xr1, xr2 = detail_range
    ys, xs = source_point

    # Find which axis the range is constant for
    if yr1 == yr2:
        line = [xs, xr1, xr2]
        line.sort()

        return (yr1, line[1])

    # xr1 == xr2
    else:
        line = [ys, yr1, yr2]
        line.sort()

        return (line[1], xr1)
