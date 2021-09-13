#!/usr/bin/python3
# -*- coding: utf-8 -*-
from os import read
from time import sleep
from typing import Coroutine
import numpy as np
class Algorithm(): 
    def __init__(self):
        # self.N, self.MaxTips, self.Cost_c, self.city_map, self.T, self.D = self.get_data()
        self.file = '02'
        self.N, self.MaxTips, self.Cost_c, self.city_map, self.T, self.D = self.get_data_from_file(self.file)
        self.orders = []
        self.robot_coordinates = []
        self.order_iter = 0
        self.simulation()
        
    def simulation(self):
        #print R - count of robots
        #print coordinates of spawn robots
        coord = self.check_spawn_point()
        self.robot_coordinates.append(coord)
        self.order_iter = self.N+2
        # x_r = self.robot_coordinates[0][0]
        # y_r = self.robot_coordinates[0][1]
        # print(self.city_map[x_r][y_r])
        self.get_new_orders_from_file(self.file)
        print(self.robot_coordinates[0])
        print(self.orders[0])
        path = astar(self.city_map, self.robot_coordinates[0], (self.orders[0][0],self.orders[0][1]))
        print(path)
        # for i in range(0, self.T):
        
            
            #analize orders and do the job
            # a = 1

    def get_new_orders(self):
        s = input()
        k = int(s)
        for j in range(0, k):
            s = input()
            f = s.split(" ")
            s_row = int(f[0])
            s_col = int(f[1])
            f_row = int(f[2])
            f_col = int(f[3])
            self.orders.append([s_row, s_col, f_row, f_col])
            
    def get_new_orders_from_file(self, filename):
        with open(filename, 'r') as reader:
            data = reader.readlines()
            order_count = int(data[self.order_iter])
            for i in range(1, order_count+1):
                coord = data[self.order_iter+i]
                f = coord.split(" ")
                s_row = int(f[0])
                s_col = int(f[1])
                f_row = int(f[2])
                f_col = int(f[3])
                self.orders.append([s_row, s_col, f_row, f_col])
            self.order_iter+=order_count+1
    
    def check_spawn_point(self):
        middle = int(self.N/2)
        for i in range(0, middle):
            for j in range(0, middle):
                if self.city_map[middle-i][middle-j] == 1:
                    return middle-i, middle-j 

    def get_data_from_file(self, filename):
        with open(filename, 'r') as reader:
            data = reader.readlines()
            f = data[0].split(" ")
            N = int(f[0])
            MaxTips = int(f[1])
            Cost_c = int(f[2])
            city_map = np.zeros((N,N), int)
            for i in range(0,N):
                s = data[i+1]
                j = 0
                for sym in s:
                    if sym == '.':
                        city_map[i][j] = 1
                    j+=1
            s = data[N+1]
            f = s.split(" ")
            T = int(f[0])
            D = int(f[1])
            return N, MaxTips, Cost_c, city_map, T, D

    def get_data(self):
        s = input()
        f = s.split(" ")
        N = int(f[0])
        MaxTips = int(f[1])
        Cost_c = int(f[2])
        city_map = np.zeros((N,N), int)
        for i in range(0,N):
            s = input()
            j = 0
            for sym in s:
                if sym == '.':
                    city_map[i][j] = 1
                j+=1
        s = input()
        f = s.split(" ")
        T = int(f[0])
        D = int(f[1])
        return N, MaxTips, Cost_c, city_map, T, D

class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position


def astar(maze, start, end):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""

    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Add the start node
    open_list.append(start_node)

    # Loop until you find the end
    while len(open_list) > 0:

        # Get the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)

        # Found the goal
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1] # Return reversed path

        # Generate children
        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]: # Adjacent squares

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range
            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1]) -1) or node_position[1] < 0:
                continue

            # Make sure walkable terrain
            if maze[node_position[0]][node_position[1]] != 1:
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:

            # Child is on the closed list
            for closed_child in closed_list:
                if child == closed_child:
                    continue

            # Create the f, g, and h values
            child.g = current_node.g + 1
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h

            # Child is already in the open list
            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    continue

            # Add the child to the open list
            open_list.append(child)


def main(args=None):
    alg = Algorithm()
if __name__ == '__main__':
    main()