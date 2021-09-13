#!/usr/bin/python3
# -*- coding: utf-8 -*-
from os import path, read
from time import sleep
from typing import Coroutine
import numpy as np
from math import sqrt
from matplotlib import pyplot as plt
from warnings import warn
import heapq
class Robot():
    def __init__(self, position):
        self.position = position
        self.current_path = None
        self.go_to_target = False
        self.take_order = False
        self.order_deliverd = False
        self.orders_done = 0
    
    def compute_path(self, city_map, goal):
        self.current_path = np.asarray(astar(city_map, self.position, goal))

    def clean_part_of_path(self, part):
        self.current_path = self.current_path[part:self.current_path.shape[0]]

class Algorithm(): 
    def __init__(self):
        # self.N, self.MaxTips, self.Cost_c, self.city_map, self.T, self.D = self.get_data()
        self.file = '01'
        self.N, self.MaxTips, self.Cost_c, self.city_map, self.T, self.D = self.get_data_from_file(self.file)
        self.orders = []
        self.order_iter = 0
        self.count_of_orders = 0
        self.simulation()
        # self.show_city()
        
    def simulation(self):
        #print R - count of robots
        #print coordinates of spawn robots
        coord = self.check_spawn_point()
        self.order_iter = self.N+2
        robot = Robot(coord)
        for i in range(0, 4):
            self.get_new_orders_from_file(self.file)
            if robot.take_order is False: #go to the order
                if robot.go_to_target is False:
                    goal = (self.orders[0][0]-1, self.orders[0][1]-1)
                    robot.compute_path(self.city_map, goal)
                robot.go_to_target = True
                if robot.current_path.shape[0] > 60:
                    path_str = self.generate_str_path(robot.current_path[:60])
                    print(path_str)
                    robot.clean_part_of_path(60)
                else:
                    path_str = self.generate_str_path(robot.current_path)
                    print(path_str)
                    robot.position = goal 
                    robot.take_order = True
                    robot.go_to_target = False
            else: #go to the end point
                if robot.go_to_target is False:
                    goal = (self.orders[0][2]-1, self.orders[0][3]-1)
                    robot.compute_path(self.city_map, goal)
                robot.go_to_target = True
                if robot.current_path.shape[0] > 60:
                    path_str = self.generate_str_path(robot.current_path[:60])
                    print(path_str)
                    robot.clean_part_of_path(60)
                else:
                    path_str = self.generate_str_path(robot.current_path)
                    print(path_str)
                    robot.position = goal
                    robot.order_deliverd = True
                    robot.go_to_target = False
                    #generate string with path
            if robot.order_deliverd is True: #order deliverd
                robot.take_order = False
                robot.order_deliverd = False
                robot.busy = False
                robot.orders_done += 1
                self.orders.pop(0)
        print(robot.orders_done)
        print(self.count_of_orders)
                # print(len(path))
        # print(self.orders)
        #analize orders and do the job
        # a = 1
    def generate_str_path(self, path):
        path_str = ''
        for i in range(path.shape[0]-1):
            dif = path[i+1] - path[i]
            if dif[0] == 1:
                path_str+='U' 
            if dif[0] == -1:
                path_str+='D'
            if dif[1] == 1:
                path_str+='R'
            if dif[1] == -1:
                path_str+='L'
        return path_str
    def show_city(self):
        binary = self.city_map > 0
        plt.imshow(binary)
        plt.show()

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
                self.count_of_orders+=1
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


class Node:
    """
    A node class for A* Pathfinding
    """

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position
    
    def __repr__(self):
      return f"{self.position} - g: {self.g} h: {self.h} f: {self.f}"

    # defining less than for purposes of heap queue
    def __lt__(self, other):
      return self.f < other.f
    
    # defining greater than for purposes of heap queue
    def __gt__(self, other):
      return self.f > other.f

def return_path(current_node):
    path = []
    current = current_node
    while current is not None:
        path.append(current.position)
        current = current.parent
    return path[::-1]  # Return reversed path


def astar(maze, start, end, allow_diagonal_movement = False):
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0
    open_list = []
    closed_list = []
    heapq.heapify(open_list) 
    heapq.heappush(open_list, start_node)
    outer_iterations = 0
    max_iterations = (len(maze[0]) * len(maze) // 2)
    adjacent_squares = ((0, -1), (0, 1), (-1, 0), (1, 0),)
    if allow_diagonal_movement:
        adjacent_squares = ((0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1),)
    while len(open_list) > 0:
        outer_iterations += 1
        if outer_iterations > max_iterations:
          warn("giving up on pathfinding too many iterations")
          return return_path(current_node)       
        
        current_node = heapq.heappop(open_list)
        closed_list.append(current_node)
        if current_node == end_node:
            return return_path(current_node)
        children = []
        for new_position in adjacent_squares: # Adjacent squares
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])
            if node_position[0] > (maze.shape[0] - 1) or node_position[0] < 0 or node_position[1] > (maze.shape[1] -1) or node_position[1] < 0:
                continue
            if maze[node_position[0]][node_position[1]] != 1:
                continue
            new_node = Node(current_node, node_position)
            children.append(new_node)
        for child in children:
            if len([closed_child for closed_child in closed_list if closed_child == child]) > 0:
                continue
            child.g = current_node.g + 1
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h
            if len([open_node for open_node in open_list if child.position == open_node.position and child.g > open_node.g]) > 0:
                continue
            heapq.heappush(open_list, child)
    warn("Couldn't get a path to destination")
    return None

def main(args=None):
    alg = Algorithm()

if __name__ == '__main__':
    main()