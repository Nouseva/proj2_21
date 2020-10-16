from collections import deque


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

    path = bfs(box_source, box_dest, mesh['adj'])
    # print(path)


    return [source_point, destination_point], path
    #return path, boxes.keys()


def find_box(point, boxes):
    """

    Searches a list of 4-tuples for the box that contains the point

    Args:
    point - a co-ordinate tuple of non-negative integers
    boxes - a list of boxes, where a box is (x1, x2, y1, y2) such that
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
