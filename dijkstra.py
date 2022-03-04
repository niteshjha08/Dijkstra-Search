#!/usr/bin/python3
import numpy as np
from generate_map import define_obstacle_space
import cv2
# map_arr,curr_node_location,curr_node_values,goal_location,moveBindings,visited,closed,associations
class DijkstraSearch:
    def __init__(self,map_arr,start_location,goal_location, moveBindings):
        self.map_arr = map_arr
        self.goal_location = goal_location
        self.moveBindings = moveBindings
        self.start_location = start_location
        self.node_count = 1
        self.search_order = ['e','n','w','s','ne','nw','sw','se']

        # Format: {(location):[cost,node_idx,parent_idx]}
        self.visited = {start_location:[0,0,None]}
        self.closed = {}

        # Format: {node_idx:[parent_node,(location)]}
        self.associations = {}

def assert_search_valid(search_state):
    print("checking...")
    # print(map_arr[start], map_arr[end])
    if(search_state.map_arr[search_state.start_location]!=255 or search_state.map_arr[search_state.goal_location]!=255):
        print("Start or End location is inside an obstacle!")
        return False
    else:
        return True

def backtrack(search_state):
    print("Backtracking now!")
    end_goal_location = list(search_state.closed.keys())[-1]
    search_state.map_arr[end_goal_location[0], end_goal_location[1]] = [255,0,0]
    end_goal_values = search_state.closed[end_goal_location]
    parent = end_goal_values[2]
    optimal_path = [end_goal_location]
    while(parent!=None):
        parent_location = search_state.associations[parent][1]
        optimal_path.append(parent_location)
        search_state.map_arr[parent_location[0], parent_location[1]] = [255,0,0]
        parent = search_state.associations[parent][0]

    print("optimal path length:",len(optimal_path))
    cv2.imshow('map_arr',search_state.map_arr)


# Function which checks if a node is visited, if it has, returns the current cost, else returns -1
def check_visited(visited,location):
    try:
        cost = visited[location]
    except:
        cost = None
    return cost

def check_closed(closed,location):
    try:
        _ = closed[location]
        return True
    except:
        return False

def check_direction(search_state,curr_node_location,curr_node_values,direction):
    # check if location is an obstacle or not
    check_location = tuple([sum(x) for x in zip(search_state.moveBindings[direction][0],curr_node_location)])
    if(search_state.map_arr[check_location]) == 0:
        # print('obstacle found, returning')
        return

    # if the location is never visited, append location and cost
    check_res = check_visited(search_state.visited,check_location)
    if not check_closed(search_state.closed, check_location):
        if check_res is None:
            search_state.map_arr[check_location[0],check_location[1]] = [0,255,0]
            search_state.visited[check_location] = [curr_node_values[0] + search_state.moveBindings[direction][1], search_state.node_count, curr_node_values[1]]
            search_state.node_count+=1
        # if location has been visited, check if new cost is lower. If so, modify the node.
        else:
            if(check_res > curr_node_values[0] + search_state.moveBindings[direction][1]):
                search_state.visited[check_location][0] = curr_node_values[0] + search_state.moveBindings[direction][1]
             

def dijkstra_search(search_state,visualize_search):
    curr_node_location = search_state.start_location
    curr_node_values = search_state.visited[curr_node_location]

    while(curr_node_location!=search_state.goal_location):     

        # Search all directions, if not visited, append, else modify only if lower cost is found
        for direction in search_state.search_order:
            check_direction(search_state,curr_node_location,curr_node_values,direction)

        # Visualization
        if visualize_search:
            cv2.imshow('map_arr',search_state.map_arr)
            cv2.waitKey(1)
        # Mark curr_location to closed nodes
        search_state.closed[curr_node_location] = curr_node_values
        # Save Parent-child relationship
        search_state.associations[curr_node_values[1]] = [curr_node_values[2],curr_node_location]

        # Pop first visited element and sort for next iteration
        search_state.visited.pop(next(iter(search_state.visited)))        
        search_state.visited = {k: v for k, v in sorted(search_state.visited.items(), key=lambda item: item[1])}  # Sort visited based on cost
        
         # Set first node of visited as curr_node_location
        curr_node_location = next(iter(search_state.visited))
        curr_node_values = search_state.visited[curr_node_location]
        
    print("Reached goal")
    
    # Save goal locations and parent-child associations
    search_state.visited[curr_node_location]=curr_node_values
    search_state.closed[curr_node_location]=curr_node_values
    search_state.associations[curr_node_values[1]] = [curr_node_values[2],curr_node_location]

    # Traverse backwards till start location
    backtrack(search_state)
    cv2.waitKey(0)

def main():

    map_arr = define_obstacle_space()
    start_location = (2,2,0)
    goal_location = (200,230,0)
    visualize_search = False
    
    # Directions : {associated movement delta, costs}
    moveBindings = {'e':[(0,1,0),1],'n':[(-1,0,0),1],'w':[(0,-1,0),1],'s':[(1,0,0),1],'ne':[(-1,1,0),1.4],\
                    'nw':[(-1,-1,0),1.4],'sw':[(1,-1,0),1.4],'se':[(1,1,0),1.4]}

    search_state = DijkstraSearch(map_arr,start_location,goal_location, moveBindings)

    if not(assert_search_valid(search_state)):
        return
    
    dijkstra_search(search_state,visualize_search)


if __name__=="__main__":
    main()
