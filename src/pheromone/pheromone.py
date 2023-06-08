#! /usr/bin/env python3

import numpy as np
import time
from pathlib import Path


class Pheromone():

    def __init__(self, grid_map_size, resolution, evaporation=0.0, diffusion=0.0):
        # グリッド地図の生成
        # map size = 1 m * size
        self.grid_map_size = grid_map_size
        # grid cell size = 1 m / resolution
        self.resolution = resolution
        # 1辺におけるグリッドセルの合計個数
        self.num_cell = self.resolution * self.grid_map_size + 1
        # 例外処理 : 一辺におけるグリッドセルの合計個数を必ず奇数でなければならない
        if self.num_cell % 2 == 0:
            raise Exception(
                "Number of cell is even. It needs to be an odd number")
        self.grid = np.zeros((self.num_cell, self.num_cell))
        self.grid_copy = np.zeros((self.num_cell, self.num_cell))

        # 蒸発パラメータの設定
        self.evaporation = evaporation
        if self.evaporation == 0.0:
            self.isEvaporation = False
        else:
            self.isEvaporation = True

        # 拡散パラメータの設定
        self.diffusion = diffusion
        if self.diffusion == 0.0:
            self.isDiffusion = False
        else:
            self.isDiffusion = True

        # Timers
        self.update_timer = time.process_time()
        self.step_timer = time.process_time()
        self.injection_timer = time.process_time()
        self.save_timer = time.process_time()
        self.reset_timer = time.process_time()

    # 指定した座標(x, y)からフェロモンを取得
    def getPheromone(self, x, y):
        return self.grid[x, y]

    # 指定した座標(x, y)へフェロモンを配置
    def setPheromone(self, x, y, value):
        self.grid[x, y] = value

    # 正方形の形でフェロモンの射出する
    def injection(self, x, y, pheromone_value, injection_size,
                  max_pheromone_value):
        # 例外処理 : フェロモンを射出するサイズはかならず奇数
        if injection_size % 2 == 0:
            raise Exception("Pheromone injection size must be an odd number.")
        # 現在時刻を取得
        current_time = time.process_time()
        # フェロモンを射出する間隔を0.1s間隔に設定
        if current_time - self.injection_timer > 0.1:
            for i in range(injection_size):
                for j in range(injection_size):
                    # 指定した範囲のセルにフェロモンを配置
                    self.grid[x-(injection_size-1)/2+i,
                              y-(injection_size-1)/2+j] += pheromone_value
                    # 配置するフェロモン値がフェロモンの最大値より大きければカット
                    if self.grid[x-(injection_size-1)/2+i,
                                 y-(injection_size-1)/2+j] >=\
                            max_pheromone_value:
                        self.grid[x-(injection_size-1)/2+i,
                                  y-(injection_size-1)/2+j] =\
                            max_pheromone_value
            # フェロモンを射出した時間を記録
            self.injection_timer = current_time

    def update(self, min_pheromone_value, max_pheromone_value):
        current_time = time.process_time()
        time_elapsed = current_time - self.update_timer
        self.update_timer = current_time

        # 拡散を行うかどうかの判定
        if self.isDiffusion is True:
            # Diffusion
            for i in range(self.num_cell):
                for j in range(self.num_cell):
                    self.grid_copy[i, j] += 0.9*self.grid[i, j]
                    if i >= 1:
                        self.grid_copy[i-1, j] += 0.025*self.grid[i, j]
                    if j >= 1:
                        self.grid_copy[i, j-1] += 0.025*self.grid[i, j]
                    if i < self.num_cell-1:
                        self.grid_copy[i+1, j] += 0.025*self.grid[i, j]
                    if j < self.num_cell-1:
                        self.grid_copy[i, j+1] += 0.025*self.grid[i, j]
            # 最大と最小を丸める
            for i in range(self.num_cell):
                for j in range(self.num_cell):
                    if self.grid_copy[i, j] < min_pheromone_value:
                        self.grid_copy[i, j] = min_pheromone_value
                    elif self.grid_copy[i, j] > max_pheromone_value:
                        self.grid_copy[i, j] = max_pheromone_value
            # グリッドセルを更新
            self.grid = np.copy(self.grid_copy)
            # 複製したグリッドセルを初期化
            self.grid_copy = np.zeros((self.num_cell, self.num_cell))

        # 蒸発を行うかどうかの判定
        if self.isEvaporation is True:
            # Evaporation
            decay = 2**(-time_elapsed/self.evaporation)
            for i in range(self.num_cell):
                for j in range(self.num_cell):
                    self.grid[i, j] = decay * self.grid[i, j]

    def save(self, file_name):
        parent = Path(__file__).resolve().parent
        with open(parent.joinpath('pheromone_saved/' +
                                  file_name + '.pheromone'), 'wb') as f:
            np.save(f, self.grid)
        print("The pheromone matrix {} is successfully saved".
              format(file_name))

    def load(self, file_name):
        parent = Path(__file__).resolve().parent
        with open(parent.joinpath('pheromone_saved/' +
                                  file_name + '.pheromone'), 'rb') as f:
            self.grid = np.load(f)
        print("The pheromone matrix {} is successfully loaded".
              format(file_name))
