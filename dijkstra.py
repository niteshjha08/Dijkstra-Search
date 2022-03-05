#!/usr/bin/python3
import numpy as np
from generate_map import define_obstacle_space
import cv2
import random

class DijkstraSearch:
    def __init__(self,map_arr,start_location,goal_location, moveBindings,search_clearance):
        self.map_arr = map_arr
        self.goal_location = goal_location
        self.moveBindings = moveBindings
        self.start_location = start_location
        self.node_count = 1
        self.search_order = ['e','n','w','s','ne','nw','sw','se']
        self.clearance = search_clearance

        # Format: {(location):[cost,node_idx,parent_idx]}
        self.visited = {start_location:[0,0,None]}
        self.closed = {}

        # Format: {node_idx:[parent_node,(location)]}
        self.associations = {}

def assert_search_valid(search_state):
    print("checking search validity...")
    try:
        if(min(np.min(search_state.map_arr[search_state.start_location[0]-search_state.clearance:search_state.start_location[0]+search_state.clearance+1,search_state.start_location[1]-search_state.clearance:search_state.start_location[1]+search_state.clearance+1]),\
            np.min(search_state.map_arr[search_state.goal_location[0]-search_state.clearance:search_state.goal_location[0]+search_state.clearance+1,search_state.goal_location[1]-search_state.clearance:search_state.goal_location[1]+search_state.clearance+1])))== 0 :
            print("Obstacle/boundaries near start or end locations")
            return False
        else:
            return True
    except Exception as e:
        # print(e)
        print("Obstacle/boundaries near start or end locations OR not in map_range")
        return False

def visualize(search_state,frame_skip):
    frame = 0
    for location in search_state.closed:
        frame +=1
        search_state.map_arr[location[0],location[1]] = [0,255,0]
        if frame % frame_skip ==0:
            cv2.imshow('map_arr',search_state.map_arr)
            cv2.waitKey(1)
    backtrack(search_state)
    cv2.waitKey(0)

def backtrack(search_state):
    print("Backtracking now!")
    end_goal_location = list(search_state.closed.keys())[-1]
    cv2.circle(search_state.map_arr,(end_goal_location[1],end_goal_location[0]),5,(0,0,255),-1)
    search_state.map_arr[end_goal_location[0], end_goal_location[1]] = [0,0,255]
    end_goal_values = search_state.closed[end_goal_location]
    parent = end_goal_values[2]
    optimal_path = [end_goal_location]
    while(parent!=None):
        parent_location = search_state.associations[parent][1]
        optimal_path.append(parent_location)
        search_state.map_arr[parent_location[0], parent_location[1]] = [0,0,255]
        parent = search_state.associations[parent][0]

    cv2.circle(search_state.map_arr,(parent_location[1],parent_location[0]),5,(255,0,0),-1)
    print("optimal path length:",len(optimal_path))
    cv2.imshow('map_arr',search_state.map_arr)


# Function which checks if a node is visited, if it has, returns the current cost, else returns -1
def check_visited(visited,location):
    try:
        cost = visited[location][0]
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

    if(np.min(search_state.map_arr[check_location[0]-search_state.clearance:check_location[0]+search_state.clearance+1,check_location[1]-search_state.clearance:check_location[1]+search_state.clearance+1]))== 0 :
        return
    
    # if the location is never visited, append location and cost
    check_res = check_visited(search_state.visited,check_location)

    # if not check_closed(search_state.closed, check_location):
    if check_closed(search_state.closed,check_location):
        return
    
    if check_res is None:
        search_state.visited[check_location] = [curr_node_values[0] + search_state.moveBindings[direction][1], search_state.node_count, curr_node_values[1]]
        search_state.node_count+=1
    # if location has been visited, check if new cost is lower. If so, modify the node.
    elif check_res is not None:
        if(check_res > curr_node_values[0] + search_state.moveBindings[direction][1]):
            search_state.visited[check_location][0] = curr_node_values[0] + search_state.moveBindings[direction][1]
            

def dijkstra_search(search_state,visualize_search):
    curr_node_location = search_state.start_location
    curr_node_values = search_state.visited[curr_node_location]
    
    while(curr_node_location!=search_state.goal_location):     
        # Search all directions, if not visited, append, else modify only if lower cost is found
        for direction in search_state.search_order:
            check_direction(search_state,curr_node_location,curr_node_values,direction)
       
        # Mark curr_location to closed nodes
        search_state.closed[curr_node_location] = curr_node_values
        # Save Parent-child relationship
        search_state.associations[curr_node_values[1]] = [curr_node_values[2],curr_node_location]
        # Pop first visited element and sort for next iteration
        search_state.visited.pop(next(iter(search_state.visited)))        
        search_state.visited = {k: v for k, v in sorted(search_state.visited.items(), key=lambda item: item[1])}  # Sort visited based on cost
        
         # Set first node of visited as curr_Reached goal
        curr_node_location = next(iter(search_state.visited))
        curr_node_values = search_state.visited[curr_node_location]
    
    # Save goal locations and parent-child associations
    search_state.visited[curr_node_location]=curr_node_values
    search_state.closed[curr_node_location]=curr_node_values
    search_state.associations[curr_node_values[1]] = [curr_node_values[2],curr_node_location]

    # Visualization
    if visualize_search:
        visualize(search_state,frame_skip=50)

def main():
    map_arr = define_obstacle_space()
    start_location = (random.randint(0,250)+1,random.randint(1,400)+1,0)
    goal_location = (random.randint(0,250)+1,random.randint(1,400)+1,0)
    search_clearance = 5
    # start_location = (6,6,0)
    # goal_location = (192,189,0)
    print("start:",start_location)
    print("goal:",goal_location)
    
    visualize_search = True
    
    # Format: {Directions : [associated movement delta, costs]}
    moveBindings = {'e':[(0,1,0),1],'n':[(-1,0,0),1],'w':[(0,-1,0),1],'s':[(1,0,0),1],'ne':[(-1,1,0),1.4],\
                    'nw':[(-1,-1,0),1.4],'sw':[(1,-1,0),1.4],'se':[(1,1,0),1.4]}

    search_state = DijkstraSearch(map_arr,start_location,goal_location, moveBindings,search_clearance)

    if not(assert_search_valid(search_state)):
        cv2.circle(search_state.map_arr,(goal_location[1],goal_location[0]),search_clearance,(0,0,255),-1)
        cv2.circle(search_state.map_arr,(start_location[1],start_location[0]),search_clearance,(255,0,0),-1)
        cv2.imshow('map_arr',search_state.map_arr)
        cv2.waitKey(0)
        return
    
    dijkstra_search(search_state,visualize_search)


if __name__=="__main__":
    # for testcase in range(50):
    #     cv2.destroyAllWindows()
    #     main()
    main()