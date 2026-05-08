start_point = 13

list = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42]
if len (list) <= start_point+1:
    global_waypoints = list[-1]
elif (len(list)-1) % start_point == 0:
    global_waypoints = list[start_point::start_point]
    #global_waypoints.append(response.global_path.poses[-1])
elif (len(list[start_point:])-1) % start_point != 0:
    if (len(list[start_point:])-2) % start_point == 0 or (len(list[start_point:])-3) % start_point == 0:
        global_waypoints = list[start_point:-start_point:start_point]
        global_waypoints.append(list[-1])
    else:
        global_waypoints = list[start_point::start_point]
        global_waypoints.append(list[-1])

print(global_waypoints)