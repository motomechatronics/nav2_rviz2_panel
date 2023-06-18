import yaml   

with open('/home/user/ros2_ws/src/rooms_pkg/config/office_points.yaml', 'r') as file:
            rooms = yaml.safe_load(file)['rooms']

print('points = ',rooms['points'])
print('no. points = ',len(rooms['points']))
print('point_position(x,y) = ',rooms['points'][1]['position']['x'],rooms['points'][1]['position']['y'])

print('------------------------------------------')

def get_waypoint_position(rooms,point_number):
    x = rooms['points'][point_number]['position']['x']
    y = rooms['points'][point_number]['position']['y']
    return x,y

def get_waypoint_orientation(rooms,point_number):
    z = rooms['points'][point_number]['orientation']['z']
    w = rooms['points'][point_number]['orientation']['w']
    return z,w

# get name, number of waypoints and phase coordinates
def get_routine_data(rooms,route_id):
    # rooms is the data rooms dictionary (waypoint, nav_routes, etc.)
    # route is the id that identifies the route in the nav_route dictionary

    # get route i.e. the dictionary of all couples: waypoint (key) and routine (value)
    route = rooms['nav_routes'][route_id]['points']

    # get the name of the route
    route_name = rooms['nav_routes'][route_id]['route_name']   

    # get the list of the waypoints reached in the route
    route_waypoints = list(route.keys())
    route_waypoints_number = len(rooms['nav_routes'][route_id]['points'])
    
    # for every waypoint, it extracts from route the routine. 
    # phases_number is the number of phase inside a routine.
    routine =  {}
    phases_number = []
    for waypoint in route:
        values = route[waypoint]
        routine[waypoint] = values  
        phases_number.append(len(routine[waypoint]))
    return route_name,route_waypoints,routine,phases_number


x,y = get_waypoint_position(rooms,1)
print(x,y)
z,w = get_waypoint_orientation(rooms,1)
print(x,y,z,w)

id = 1
frame_name = "waypoint_" + str(id)
print(frame_name)

nav_routes = 0
route_name,route_waypoints,routine,phases_number = get_routine_data(rooms,nav_routes)
print("---------------------")
print('nav_routes: ',nav_routes)
for wayoint in routine:
    print(wayoint,routine[wayoint])
    print("---------------------")



#print(route_waypoints[nav_routes])
#print(phase[nav_routes])
#print("---------------------")
#print(phases_number)
#print(len(phases_number))
for j in range(phases_number[2]):
    print("---------------------")
    print(phases_number[j])

print('***************')
for j in phases_number:
    print(j)

print('***************')
print(rooms['nav_routes'][0]['points'][1])

print('***************')
phases_number = len(rooms['nav_routes'][0]['points'][2])
print(phases_number)

print('xxxxxxxxxxxxxx')
for phase in rooms['nav_routes'][0]['points'][3]:
    print("phase")

print('xxxxxxxxxxxxxx')
route_id = 0
wp = 1
phase = 1
angle = rooms['nav_routes'][route_id]['points'][wp][phase][0]
time = rooms['nav_routes'][route_id]['points'][wp][phase][1]
print(angle,time)


points = [[0.0,0.0],[1.0,2.0],[3.0,4.0]]
for i in range(len(points) - 1):
    print('ccccccccccccccc')
    print(i)
    print(points[i])
    print(i+1)
    print(points[i+1])
    print('ccccccccccccccc')
    middle_point = (points[i]+points[i+1])
    print(middle_point)


    print("YYYYYYYYYYYYYYYYYYY")
    d = rooms['room_name']
    print(d)