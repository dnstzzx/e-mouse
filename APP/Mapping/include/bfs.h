#pragma once

#ifndef BFS_FILE_H
#define BFS_FILE_H


#include "stdio.h"
#include "stdbool.h"
#include "stdint.h"
#include "base_old.h"
#include "stdlib.h"
#include "string.h"

#define LEN 150
#define MAX_DISTANCE 254

// 坐标结构体
typedef struct {
    int8_t x;
    int8_t y;
} Coordinate;

// 队列节点
typedef struct {
    Coordinate coord;
    uint8_t distance;
} QueueNode;

// 路径
typedef struct {
    uint8_t len;
    Coordinate coords[LEN];
} PATH;

// 进行BFS搜索，用于建图过程中，搜索最近发现，但没走过的路
bool bfsShortestPath(Coordinate start, PATH *path);

// 打印搜索出来的路径
void printPath(PATH *path);

// 标准化坐标
static inline Coordinate standardize_cord(int8_t x, int8_t y){
    x = x<0?(x + MAX_ROWS) % MAX_ROWS : x % MAX_ROWS;
    y = y<0?(y + MAX_COLS) % MAX_COLS : y % MAX_COLS; 
    return (Coordinate){x, y};
}

bool Coords_eq(Coordinate *a, Coordinate *b);

#endif