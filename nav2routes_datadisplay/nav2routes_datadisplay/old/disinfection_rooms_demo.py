room = {

    "id": 1,
    "id_name": 'office',
    "name": 'Akara office',
    "initialpose": {
        "position": {
            "x": 0,
            "y": 0},
        "orientation": {
            "z": 0,
            "w": 1,
        }},
    "waypoints":{
        "1": {
            "id":10,
            "position": {
                "x": 0.45,
                "y": -1.67},
            "orientation": {
                "z": 0,
                "w": 1,
            "note": "first wp"}},
        "2": {
            "id":20,
            "position": {
                "x": 3.45,
                "y": -7.67},
            "orientation": {
                "z": 0,
                "w": 1,
            "note": "second wp"}},
        "3": {
            "id":30,
            "position": {
                "x": 1.45,
                "y": 2.67},
            "orientation": {
                "z": 0,
                "w": 1,
            "note": "third wp"}},
    },
    "routes": {
        "0": {
            "name": "bed",
            "waypoints": {
                "1": {
                    "id":30,
                    "position": {
                        "x": 0.45,
                        "y": -1.67},
                    "orientation": {
                        "z": 0,
                        "w": 1,
                    "note": "first wp"},
                    "phases":[(90,2),(45,3),(15,5)]},
                "2": {
                    "id":10,
                    "position": {
                        "x": 0.45,
                        "y": -1.67},
                    "orientation": {
                        "z": 0,
                        "w": 1,
                    "note": "first wp"},
                    "phases":[(90,2),(45,3),(15,1)]},
                "3": {
                    "id":20,
                    "position": {
                        "x": 0.45,
                        "y": -1.67},
                    "orientation": {
                        "z": 0,
                        "w": 1,
                    "note": "first wp"},
                    "phases":[(90,2),(45,3),(15,7)]},                
                             },
    
            },
        "1": {
            "name": "chair",
            "waypoints": {
                "1": {
                    "id":20,
                    "position": {
                        "x": 0.45,
                        "y": -1.67},
                    "orientation": {
                        "z": 0,
                        "w": 1,
                    "note": "first wp"},
                    "phases":[(90,2),(45,3)]},
                "2": {
                    "id":10,
                    "position": {
                        "x": 0.45,
                        "y": -1.67},
                    "orientation": {
                        "z": 0,
                        "w": 1,
                    "note": "first wp"},
                    "phases":[(90,2),(45,3),(15,7)]},
              
                             },
    
            },
    
            },
    }



print(room["id"])
print(room["id_name"])
print(room["name"])
print(room["initialpose"])
print(room["initialpose"]["position"])
print(room["initialpose"]["orientation"])
print(room["initialpose"]["position"]['x'])
print(room["waypoints"])
print(room["waypoints"]["1"])
print("routes")
print(room["routes"])
print("routes - chair")
print(room["routes"]['1'])
print("routes - chair - waypoint 1 - phases")
print(room["routes"]['1']['waypoints']['1']['phases'])
print("routes - chair - waypoint 1 - phase (angle,time))")
print(room["routes"]['1']['waypoints']['1']['phases'][0])
print("routes - chair - waypoint 1 - phase 0 angle")
print(room["routes"]['1']['waypoints']['1']['phases'][0][0])
print("routes - chair - waypoint 1 - phase 0 time")
print(room["routes"]['1']['waypoints']['1']['phases'][0][1])
print("routes - bed - waypoint 1 - phase 0 time")
print(room["routes"]['0']['waypoints']['2']['phases'][2][1])