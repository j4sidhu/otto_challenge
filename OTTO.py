"""
This module calculates the shortest time needed for OTTO (the robot) to navigate
through all of the waypoints in a course in accordance with the
Clearpath robotics coding challenge guidelines
"""

import time

FACTORY_LENGTH = 100  # in meters
FACTORY_WIDTH = 100  # in meters

START_POINT = (0, 0, 0)
# Starting point for Otto. Format: (X, Y, penalty)

END_POINT = (100, 100, 0)  # Ending point

SPEED = 2.0  # in m/s
TIME_AT_WAYPOINTS = 10  # in seconds

computed_waypoints = {}


def calc_time(x, y):
    '''
    The function returns the time taken to travel from point a to b using
    at Otto's speed

    :param x (tuple): Tuple with 2 elements representing the x,y coordinates
                        of point A
    :param y (tuple):  Tuple with 2 elements representing the x,y coordinates
                        of point B
    :return float: Time taken to travel from A to B
    '''

    dist = ((y[1] - x[1]) ** 2 + (y[0] - x[0]) ** 2) ** 0.5
    # Calculate the distance between 2 points in meters
    time_taken = dist / SPEED
    # Time taken to travel from x to y in seconds for OTTO

    return time_taken


def minimum_dist(waypoints, time):
    '''
    Compares the times of all the waypoints and returns the shortest time

    :param waypoints (list of tuples): List representing waypoints with three
                                        elements x, y, penalty
    :param time (dict): dctionary where waypoints and shortest time to that
                        waypoint are the key value pair
    :return (tuple): Tuple that has the shortest time
    '''

    shortest = (waypoints[0], time[waypoints[0]])
    # Assume first waypoint has the shortest time
    for waypoint in waypoints:
        if time[waypoint] < shortest[1]:
            shortest = (waypoint, time[waypoint])

    return shortest[0]


def reconstruct_path_for_dijk(cameFrom, current):
    '''
    This function determines the optimal route from the current position
    to the start of the course
    :param cameFrom (dict): Dictionary containing the information on the optimal
                            previous node for each node
    :param current (tuple): Location of the current waypoint
    :return (list): Optimal route through the course
    '''

    total_path = []
    while cameFrom[current] is not None:
        total_path.insert(0, current)
        current = cameFrom[current]
    total_path.insert(0, current)

    return total_path


def reconstruct_path_for_AStar(cameFrom, current):
    '''
    This function determines the optimal route from the current position
    to the start of the course
    :param cameFrom (dict): Dictionary containing the information on the optimal
                            previous node for each node
    :param current (tuple): Location of the current waypoint
    :return (list): Optimal route through the course
    '''
    total_path = [current]
    while current in cameFrom.keys():
        current = cameFrom[current]
        total_path.append(current)
    return total_path


def find_shortest_time_recursive(waypoints):
    '''
    Calcualte the shortest time for the course using recursion.
    Slowest of the 3 methods.

    :param waypoints (tuple): Tuple of format (x, y, penalty) representing
                                all the waypoints of a course
                                (including start point and end point) in sequential
                                order i.e waypoint 0, 1, 2...
    :return float: Shortest time taken to complete the course
    '''

    computed_waypoints = {}
    if waypoints in computed_waypoints.keys():
        return computed_waypoints[waypoints]
        # If we have already seen this set of waypoints before
        # No need to calculate the shortest time again
        # Just return the calculations from the first time
        # to optimize performance

    if len(waypoints) == 2:  # We have reached the end
        shortest_time = calc_time(waypoints[0][:2], waypoints[1][:2]) + TIME_AT_WAYPOINTS
        computed_waypoints[waypoints] = shortest_time
        return shortest_time

    # Option1: Choose to go to next waypoint
    time_option1 = calc_time(waypoints[0][:2], waypoints[1][:2]) + \
                   TIME_AT_WAYPOINTS + \
                   find_shortest_time_recursive(waypoints[1:])
                   # Since we have decided to go to the first waypoint,
                   # we take out the 0th waypoint

    # Option2: Choose not to go to next waypoint
    penalty = waypoints[1][2]
    time_option2 = penalty + find_shortest_time_recursive(waypoints[:1] + waypoints[2:])

    shortest_time = min(time_option1, time_option2)
    # Pick the shortest time between the two options

    computed_waypoints[waypoints] = shortest_time

    return shortest_time


def find_shortest_time_dijk(w):
    '''
    Calcualte the shortest time with the least number of waypoints using Dijkstra's algorithm.
    For some reason, it works for 8 out of the 9 courses but not for the
    last course. It is off by 2.7 seconds for that and I ran out of time to debug

    :param w (tuple of tuples): Waypoints representing a course
    :return (float): Shortest time possible through the course
    '''

    start_point = w[0]
    end_point = w[-1]
    unvisited_waypoints = []  # Waypoints that have not been examined yet
    total_time = {}  # Distance from start to waypoint
    prev = {}  # Optimal prev waypoint

    for waypoint in w:
        total_time[waypoint] = float("inf")
        prev[waypoint] = None
        unvisited_waypoints.append(waypoint)  # All waypoints are unvisited initially

    total_time[start_point] = 0  # Distance from start to start is zero

    while unvisited_waypoints:
        point = minimum_dist(unvisited_waypoints, total_time)
        # From the remaining waypoints, the one with the
        # minimum distance will be picked next
        if point == end_point:
            shortest_time = total_time[point]
            #print(reconstruct_path_for_dijk(prev, point))
            # Uncomment the line above if optimal path needs to be printed
            return shortest_time

        unvisited_waypoints.remove(point)
        # Remove the waypoint since it has been visited

        penalty = 0
        for waypoint in w[w.index(point) + 1:]:
        # Only the waypoints coming ahead sequentially are neighbours
        # since we can not visit the waypoints already skipped
            new_time = total_time[point] + calc_time(point[:2], waypoint[:2]) + TIME_AT_WAYPOINTS + penalty
            if new_time < total_time[waypoint]:  # Shorter path has been found
                total_time[waypoint] = new_time
                prev[waypoint] = point

            penalty += waypoint[2]
            # In the next iteration, we will be skipping this
            # waypoint so add its penalty

def find_shortest_time_AStar(w):
    '''
    Calcualte the shortest time with the least number of waypoints using A* algorithm.
    (Variant of Dijkstra algorithm above)
    For some reason, it works for 8 out of the 9 courses but not for the
    last course. It is off by 2.7 seconds for that and I ran out of time to debug

    :param w (tuple of tuples): Waypoints representing a course
    :return (float): Shortest time possible through the course
    '''

    start_point = w[0]
    end_point = w[-1]
    closedSet = []  # Set of waypointa already evaluated
    openSet = [start_point]  # Set of nodes not evaluated yet
    gScore = {}
    # For each waypoint, shortest distance from the start to that waypoint
    fScore = {}
    # For each waypoint, shortest distance to go from start to finish
    # through the waypoint. First part of start to waypoint is known
    # and second part of waypoint to finish is heuristic

    # Heuristic function: Direct line from waypoint to end + waiting for
    # a certain time length at the end

    prev = {}

    for waypoint in w:  # initializing
        gScore[waypoint] = float("inf")
        fScore[waypoint] = float("inf")

    gScore[start_point] = 0  # Cost of going from start to start is zero
    fScore[start_point] = calc_time(start_point[:2], end_point[:2]) + TIME_AT_WAYPOINTS

    while openSet:
        point = minimum_dist(openSet, fScore)
        if point == end_point:
            #(reconstruct_path_for_AStar(prev, point))
            # Uncomment the line above if optimal path needs to be printed
            return gScore[point]

        openSet.remove(point)  # waypoint has been visited
        closedSet.append(point)

        penalty = 0
        for waypoint in w[w.index(point) + 1:]:
            # Only the waypoints coming ahead sequentially are neighbours
            # since we can not visit the waypoints already skipped

            penalty += waypoint[2]
            if waypoint in closedSet:
                continue  # Ignore the neighbour which is already seen before

            new_time = gScore[point] + calc_time(point[:2], waypoint[:2]) + TIME_AT_WAYPOINTS + (penalty - waypoint[2])
            # Time from start to the waypoint

            if waypoint not in openSet:
                openSet.append(waypoint)  # New waypoint never seen before
            elif new_time >= gScore[waypoint]:
                continue  # Not an optimal path

            # Optimal path
            prev[waypoint] = point
            gScore[waypoint] = new_time
            fScore[waypoint] = gScore[waypoint] + calc_time(waypoint[:2], end_point[:2]) + TIME_AT_WAYPOINTS


def main(filename=None):
    '''
    Read the course either from the user directly or through the optional parameter.
    Prints the shortest time through the course to 3 decimals

    :param filename(string): Optional parameter. Name of the file to read the course from
    :return: None
    '''

    if filename is None:
        filename = input('File containing the waypoints for the course:')

    with open(filename, 'r') as inputfile:
        content = inputfile.readlines()
    content = [line.strip() for line in content]
    
    file_counter = 0  # Counter that keeps tracks of the lines in the input file
    while file_counter < len(content) - 1:
        num_waypoints = int(content[file_counter])

        waypoints = (START_POINT,)
        for index in range(num_waypoints):
            waypoint_info = content[file_counter + index + 1].split()
            waypoints += ((int(waypoint_info[0]), int(waypoint_info[1]), int(waypoint_info[2])), )
            # Load all the waypoints

        waypoints += (END_POINT,)
        print(round(find_shortest_time_dijk(waypoints), 3))
        # Change the above line to point to any of
        # find_shortest_time_recursive, find_shortest_time_dijk or
        # find_shortest_time_AStar
        file_counter = file_counter + num_waypoints + 1


def calc_avg_time():
    '''
    Just for my curiosity. It runs the code 50 times on a course
    and prints the average time it took to run. I knew the recursion
    would be the slowest but Dijkstra's or A* runtimes were up for
    debate

    Results
    Note: Please keep in mind that these were on my home laptop

    Small course
    Average run time of recursion: 0.001 seconds
    Average run time of Dijkstra: 0.002 seconds
    Average run time of A*: 0.002 seconds

    Medium course
    Average run time of recursion: Too long
    Average run time of Dijkstra: 0.0416 seconds
    Average run time of A*: 0.0589 seconds

    Large course
    Average run time of recursion: I am not even gonna try
    Average run time of Dijkstra: 2.291 seconds
    Average run time of A*: 12.768 seconds
    '''

    times = []
    for c in range(0, 50):  # run the code 50 times
        start_time = time.time()
        main('sample_input_large.txt')
        total_time = time.time() - start_time
        times.append(total_time)  # record the time

    average = sum(times)/float(len(times))
    print("On average, it took %s seconds to run---" % average)

if __name__ == '__main__':
    main()
    # calc_avg_time()
