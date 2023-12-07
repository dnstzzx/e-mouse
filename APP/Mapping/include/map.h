#pragma once
#include "bsp_5883.h"
#include "direction.h"
#include "vector.h"

// ------------------ 地图定义 ------------------

#define MAX_ROWS 6
#define MAX_COLS 6

#define GRID_SIZE 400

typedef struct cordinate{
    uint8_t x;
    uint8_t y;
} cordinate_t;

typedef struct{
    direction_t direction;
    cordinate_t cordinate;
} pose_t;

typedef struct {
    unsigned borders_marked:4;  // 0-3位 1已探索 
    unsigned borders_marks:4;    // 0-3位 1有墙
    bool visited;
    bool has_calibrator;
    float origin;
    bsp_5883_calibrator_t calibrator;
} cell_t;

extern cell_t map[MAX_COLS][MAX_ROWS];

// ------------------ 路径定义 ------------------

typedef struct{
    uint8_t length;
    pose_t poses[];
} path_t;

path_t *path_from_pose_vector(vector_t *poses);
void path_free(path_t *path);

// ------------------ 辅助常量 ------------------

const static cordinate_t direction_offsets[4] = {
    {0, 1}, {-1, 0}, {0, -1}, {1, 0}
};

// ------------------ 坐标操作 ------------------

static inline cordinate_t cordinate_neighbor(cordinate_t cordinate, direction_t direction){
    return (cordinate_t){cordinate.x + direction_offsets[direction].x, cordinate.y + direction_offsets[direction].y};
}

static inline bool cordinate_equal(cordinate_t a, cordinate_t b){
    return a.x == b.x && a.y == b.y;
}

static inline bool pose_equal(pose_t a, pose_t b){
    return cordinate_equal(a.cordinate, b.cordinate) && a.direction == b.direction;
}

// ------------------ 地图操作 ------------------

void map_init();

static inline bool map_valid_cordinate(cordinate_t cordinate){
    return cordinate.x >= 0 && cordinate.y >= 0 && cordinate.x < MAX_COLS && cordinate.y < MAX_ROWS;
}

static inline cell_t *map_cell_of(cordinate_t cordinate){
    return &map[cordinate.x][cordinate.y];
}

static inline bool cell_border_is_barrier(cell_t *cell, direction_t direction){
    return cell->borders_marks >> direction & 1;
}

static inline bool cell_border_is_marked(cell_t *cell, direction_t direction){
    return cell->borders_marked >> direction & 1;
}

static inline bool map_border_is_barrier(cordinate_t cordinate, direction_t direction){
    return cell_border_is_barrier(&map[cordinate.x][cordinate.y], direction);
}

static inline bool map_border_is_marked(cordinate_t cordinate, direction_t direction){
    return cell_border_is_marked(&map[cordinate.x][cordinate.y], direction);
}

static inline void cell_mark_border(cell_t *cell, direction_t direction, bool is_barrier){
    cell->borders_marked |= (1 << direction);
    if(is_barrier)
        cell->borders_marks |= (1 << direction);
    else
        cell->borders_marks &= ~(1 << direction);
}

static inline void map_mark_border(cordinate_t cordinate, direction_t direction, bool has_border){
    cell_mark_border(&map[cordinate.x][cordinate.y], direction, has_border);
    cordinate_t neighbor = cordinate_neighbor(cordinate, direction);
    if(map_valid_cordinate(neighbor))
        cell_mark_border(map_cell_of(neighbor), direction_reverse(direction), has_border);
}

// ------------------ 路径生成 ------------------

path_t *map_calc_path(pose_t start, pose_t end);


// ------------------ 地图生成 ------------------

void start_build_map();
void rush_map();