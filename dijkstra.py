#!/usr/bin/python3
import numpy as np
from generate_map import define_obstacle_space
import cv2
# import sys
# sys.setrecursionlimit(50000)
# import enum
class Node:
    def __init__(self,location,cost=None):
        self.location = location
        self.cost = cost
    
    

# Directions : {associated movement delta, costs}
moveBindings = {'e':[(0,1,0),1],'n':[(-1,0,0),1],'w':[(0,-1,0),1],'s':[(1,0,0),1],'ne':[(-1,1,0),1.4],\
                'nw':[(-1,-1,0),1.4],'sw':[(1,-1,0),1.4],'se':[(1,1,0),1.4]}

visited = []
closed = []
search_order = ['e','n','w','s','ne','nw','sw','se']

#
def assert_search_valid(map_arr,start,end):
    print("checking...")
    # print(map_arr[start], map_arr[end])
    if(map_arr[start]!=255 or map_arr[end]!=255):
        print("Start or End location is inside an obstacle!")
        exit
    

# Function which checks if a node is visited, if it has, returns the current cost, else returns -1
def check_visited(location):
    # print(len(visited))
    # print(len(closed))
    for idx,node in enumerate(visited):

        # print("comparing: ",node.location," and ",location)
        if(node.location == location):
            return node.cost,idx
        # else:
    return -1,None

def check_closed(location):
    for node in closed:
        if(node.location[0]==location[0] and node.location[1]==location[1]):
            return True  
    return False

def check_direction(map_arr,node,direction):

    # check if location is an obstacle or not
    check_location = tuple([sum(x) for x in zip(moveBindings[direction][0],node.location)])
    # print("check_location:",check_location)
    if(map_arr[check_location]) == 0:
        print('obstacle found, returning')
        return
    # if the location is never visited, append location and cost
    check_res,idx = check_visited(check_location)
    # print("check_res for loc:",check_res)
    if not check_closed(check_location):
        if(check_res) == -1:
            new_node = Node(check_location,node.cost + moveBindings[direction][1])
            map_arr[new_node.location[0],new_node.location[1]] = [0,255,0]
            visited.append(new_node)
        # if location has been visited, check if new cost is lower. If so, modify the node.
        else:
            if(check_res > node.cost + moveBindings[direction][1]):
                visited[idx].cost = node.cost + moveBindings[direction][1]


def dijkstra_search(map_arr,curr_node,goal_location):
    count=0
    while(curr_node.location!=goal_location):
        print(curr_node.location)
        count+=1
        print(count)
        if(curr_node.location==goal_location):
            print("Reached goal")
            break
        for direction in search_order:
            check_direction(map_arr,curr_node,direction)
        cv2.imshow('map_arr',map_arr)
        cv2.waitKey(1)
        closed.append(curr_node)
        visited.pop(0)
        # visited.sort(key = lambda c: c.cost)
        curr_node = visited[0]
    print("Reached goal") 
      
    
        
def main():
    map_arr = define_obstacle_space()
    
    start_location = (65,350,0)
    goal_location = (150,50,0)
    # goal_location = (70,345,0)
    # cv2.circle(map_arr,(start_location[1],start_location[0]),4,(0,0,255),5)
    # cv2.circle(map_arr,(goal_location[1],goal_location[0]),4,(0,0,255),5)
    # cv2.imshow('map',map_arr)
    # cv2.waitKey()

    assert_search_valid(map_arr,start_location,goal_location)

    curr_node = Node(start_location,0)
    visited.append(curr_node)
    # print(moveBindings['e'][0] + start_location)
    # print(map_arr[start_location])
    dijkstra_search(map_arr,curr_node,goal_location)

if __name__=="__main__":
    main()
