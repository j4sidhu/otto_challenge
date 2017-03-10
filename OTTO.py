factory_length = 100  # in meters
factory_width = 100

robot_start_point = (0, 0)
robot_end_point = (100, 100)

robot_speed = 2.0  # in m/s
time_at_waypoints = 10  # in seconds


class course:
    def __init__(self, num_waypoints, waypoints):
        self.num_waypoints = num_waypoints
        self.waypoints = waypoints
        self.current_pos = robot_start_point
        self.total_time = 0
    
    def run_the_course(self):
        index = 0
        while index < self.num_waypoints:
            next_waypoint = self.waypoints[index]
            following_waypoint = self.waypoints[index + 1]  # This will never be out of bounds
            
            time_to_next_waypoint = self.calc_time(self.current_pos, next_waypoint['position'])
            time_to_following_waypoint = (next_waypoint['penalty'] + 
                                            self.calc_time(self.current_pos, following_waypoint['position']))
            if time_to_next_waypoint < time_to_following_waypoint:
                self.total_time += time_to_next_waypoint + time_at_waypoints
                self.current_pos = next_waypoint['position']
                index += 1
            else:
                self.total_time += time_to_following_waypoint + time_at_waypoints
                self.current_pos = following_waypoint['position']
                index += 2

        if self.current_pos != robot_end_point:
            time_to_end_point = self.calc_time(self.current_pos, robot_end_point)
            self.total_time += time_to_end_point
            self.current_pos = robot_end_point

        return self.total_time
    
    def calc_time(self, x, y):
        dist = ((y[1] - x[1]) ** 2 + (y[0] - x[0]) ** 2) ** 0.5  # Calculate the distance between 2 points in meters
        time = dist / robot_speed  # Time taken to travel from x to y in seconds for OTTO
         
        return time


def calc_time(x, y):
    dist = ((y[1] - x[1]) ** 2 + (y[0] - x[0]) ** 2) ** 0.5  # Calculate the distance between 2 points in meters
    time = dist / robot_speed  # Time taken to travel from x to y in seconds for OTTO

    return time


def find_shortest_time(index, waypoints):
    if index < len(waypoints) * -1:
        return waypoints

    shortest_time = []
    for i in range((index + 1) * -1):
        penalities =  0
        for x in range((index + 1 + i) * -1):
            pass
            # penalities +=
        shortest_time.append(calc_time(waypoints[index]['position'],
                                       waypoints[index + 1 + i]['position'])
                             + time_at_waypoints + penalities)
    waypoints[-2]['shortest_time'] = min(shortest_time)

    return find_shortest_time(index - 1, waypoints)

computed_waypoints = {}


def find_shortest_time1(coords, waypoints):
    if coords in computed_waypoints.keys():
        return computed_waypoints[coords]
        # If we have already seen this set of waypoints before
        # No need to calculate the shortest time again
        # Just return the calculations from the first time
        # to optimiza the process better

    if len(waypoints) == 2:  # We have reached the end
        shortest_time = calc_time(waypoints[0]['position'], waypoints[1]['position'])
        computed_waypoints[coords] = shortest_time
        return shortest_time

    # Option1: Choose to go to next waypoint
    time_option1 = calc_time(waypoints[0]['position'], waypoints[1]['position']) + \
                   time_at_waypoints + \
                   find_shortest_time1(coords[1:], waypoints[1:])

    # Option2: Choose not to go to next waypoint
    penalty = waypoints[1]['penalty']
    # del waypoints[1]
    time_option2 = penalty + find_shortest_time1(coords[:1] + coords[2:],
                                                 waypoints[:1] + waypoints[2:])
    shortest_time = min(time_option1, time_option2)
    computed_waypoints[coords] = shortest_time
    return shortest_time


def find_shortest_time2(coords):
    if coords in computed_waypoints.keys():
        return computed_waypoints[coords]
        # If we have already seen this set of waypoints before
        # No need to calculate the shortest time again
        # Just return the calculations from the first time
        # to optimize the process better

    if len(coords) == 2:  # We have reached the end
        shortest_time = calc_time(coords[0][:2], coords[1][:2]) + time_at_waypoints
        computed_waypoints[coords] = shortest_time
        return shortest_time

    # Option1: Choose to go to next waypoint
    time_option1 = calc_time(coords[0][:2], coords[1][:2]) + \
                   time_at_waypoints + \
                   find_shortest_time2(coords[1:])

    # Option2: Choose not to go to next waypoint
    penalty = coords[1][-1]
    # del waypoints[1]
    time_option2 = penalty + find_shortest_time2(coords[:1] + coords[2:])
    shortest_time = min(time_option1, time_option2)
    computed_waypoints[coords] = shortest_time
    return shortest_time


def find_shortest_route(curr_pos, remaining_waypoints):
    shortest_time = 0
    if len(remaining_waypoints) == 0:
        return shortest_time

    #for waypoint in remaining_waypoints:
    # I have 2 options. Move to that waypoint or don't move to that waypoint

    # Move to the waypoint
    shortest_time += calc_time(curr_pos, remaining_waypoints[0]['position']) + \
                     time_at_waypoints + find_shortest_route(remaining_waypoints[0]['position'], remaining_waypoints[1:])

    # Don't move there
    shortest_time += remaining_waypoints[0]['penalty'] + find_shortest_route(curr_pos, remaining_waypoints[1:])


def read_in_the_course(filename):
    with open(filename, 'r') as inputfile:
        content = inputfile.readlines()
    content = [x.strip() for x in content]
    
    i = 0
    while i < len(content) - 1:
        num_waypoints = int(content[i])
        waypoints = {}
        list_of_waypoints = [{'position': robot_start_point, 'penalty': None,
                              'shortest_time': None}]
        tuple_of_waypoints = ({'position': robot_start_point, 'penalty': None,
                              'shortest_time': None}, )
        coords = (robot_start_point + (0, ), )
        for index in range(num_waypoints):
            waypoint_info = content[i + index + 1].split()
            waypoints[index] = {'position': (int(waypoint_info[0]), int(waypoint_info[1])),
                                'penalty': int(waypoint_info[2]),
                                'shortest_time': None}
            list_of_waypoints.append({'position': (int(waypoint_info[0]), int(waypoint_info[1])),
                                    'penalty': int(waypoint_info[2]),
                                     'shortest_time': None})
            tuple_of_waypoints += ({'position': (int(waypoint_info[0]), int(waypoint_info[1])),
                                    'penalty': int(waypoint_info[2]),
                                     'shortest_time': None}, )
            coords += ((int(waypoint_info[0]), int(waypoint_info[1]), int(waypoint_info[2])), )
        
        waypoints[max(waypoints.keys()) + 1] = {'position': robot_end_point, 'penalty': None}
        # Last waypoint is the ending pos
        list_of_waypoints.append({'position': robot_end_point, 'penalty': None,
                                  'shortest_time': 0})

        tuple_of_waypoints += ({'position': robot_end_point, 'penalty': None,
                                  'shortest_time': 0}, )
        coords += (robot_end_point + (0, ), )

        #otto_course = course(num_waypoints, waypoints)
        #print round(otto_course.run_the_course(), 3)
        #find_shortest_route(robot_start_point, find_shortest_time(list_of_waypoints))
        #find_shortest_time(-2, list_of_waypoints)
        # print(find_shortest_time1(coords, tuple_of_waypoints))
        import time
        start_time = time.time()
        print (round(find_shortest_time2(coords), 3))
        print("--- %s seconds ---" % (time.time() - start_time))
        i = i + num_waypoints + 1
    
if __name__ == '__main__':
    read_in_the_course('sample_input_large.txt')