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
        x_r = self.robot_coordinates[0][0]
        y_r = self.robot_coordinates[0][1]
        print(self.city_map[x_r][y_r])
        for i in range(0, self.T):
            self.get_new_orders_from_file(self.file)
            
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
            print(self.order_iter)
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

def main(args=None):
    alg = Algorithm()
if __name__ == '__main__':
    main()