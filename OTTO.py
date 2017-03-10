factory_length = 100  # in meters
factory_width = 100  # in meters

robot_start_point = (0, 0)
robot_end_point = (100, 100)

robot_speed = 2.0  # in m/s
time_at_waypoints = 10  # in seconds

computed_waypoints = {}


def calc_time(x, y):
    dist = ((y[1] - x[1]) ** 2 + (y[0] - x[0]) ** 2) ** 0.5  # Calculate the distance between 2 points in meters
    time = dist / robot_speed  # Time taken to travel from x to y in seconds for OTTO

    return time


def find_shortest_time(waypoints):
    if waypoints in computed_waypoints.keys():
        return computed_waypoints[waypoints]
        # If we have already seen this set of waypoints before
        # No need to calculate the shortest time again
        # Just return the calculations from the first time
        # to optimize

    if len(waypoints) == 2:  # We have reached the end
        shortest_time = calc_time(waypoints[0][:2], waypoints[1][:2]) + time_at_waypoints
        computed_waypoints[waypoints] = shortest_time
        return shortest_time

    # Option1: Choose to go to next waypoint
    time_option1 = calc_time(waypoints[0][:2], waypoints[1][:2]) + \
                   time_at_waypoints + \
                   find_shortest_time(waypoints[1:])

    # Option2: Choose not to go to next waypoint
    penalty = waypoints[1][-1]
    time_option2 = penalty + find_shortest_time(waypoints[:1] + waypoints[2:])
    shortest_time = min(time_option1, time_option2)
    computed_waypoints[waypoints] = shortest_time
    return shortest_time


def moves_three(n, a=0, b=1, c=2):
    # Helpful function to convert recursion into iteration
    first_entry = True
    stack = [(first_entry, n, a, b, c)]
    output = []
    while stack:
        first_entry, n1, a1, b1, c1 = stack.pop()
        if n1 == 1:
            output.append((a1, c1))
        elif first_entry:
            stack.append((False, n1, a1, b1, c1))
            stack.append((True, n1-1, a1, c1, b1))
        else:
            output.append((a1, c1))
            stack.append((True, n1-1, b1, a1, c1))
    return tuple(output)


def main(filename):
    with open(filename, 'r') as inputfile:
        content = inputfile.readlines()
    content = [line.strip() for line in content]
    
    i = 0
    while i < len(content) - 1:
        num_waypoints = int(content[i])

        waypoints = (robot_start_point + (0, ), )
        for index in range(num_waypoints):
            waypoint_info = content[i + index + 1].split()
            waypoints += ((int(waypoint_info[0]), int(waypoint_info[1]), int(waypoint_info[2])), )

        waypoints += (robot_end_point + (0, ), )

        import time
        start_time = time.time()
        print(round(find_shortest_time(waypoints), 3))
        print("--- %s seconds ---" % (time.time() - start_time))
        i = i + num_waypoints + 1
    
if __name__ == '__main__':
    main('sample_input_large.txt')