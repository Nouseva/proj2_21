
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
    path.append(box_dest)
    path.append(box_source)


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
