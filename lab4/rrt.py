import math

# Return Euclidean distance between two States
def distance(p0, p1):
    return math.sqrt((p0.x - p1.x)**2 + (p0.y - p1.y)**2)

# Given a set of points V in C-space and single other target point,
# determine which points in V are closest to target
def nearestNeighbors(V, target):
    res = []
    min_distance = None

    # Each point is a State object
    for point in V:
        curr_distance = distance(point, target)
        if min_distance is None:
            res.append(point)
            min_distance = curr_distance
        elif curr_distance < min_distance:
            res = [point]
            min_distance = curr_distance
        # Can return multiple points if necessary
        elif curr_distance == min_distance:
            res.append(point)

    return res
