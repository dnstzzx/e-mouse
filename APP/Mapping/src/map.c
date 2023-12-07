#include <stdlib.h>
#include <string.h>
#include "map.h"

cell_t map[MAX_COLS][MAX_ROWS];

void map_init(){
    for(int i = 0; i < MAX_COLS; i++){
        for(int j = 0; j < MAX_ROWS; j++){
            map[i][j].borders_marks = 0xF;
            map[i][j].borders_marked = 0x0;
            map[i][j].visited = false;
            map[i][j].has_calibrator = false;
            map[i][j].origin = -1;
        }
    }
    
}

path_t *path_from_pose_vector(vector_t *poses){
    path_t *path = malloc(sizeof(path_t) + poses->size * sizeof(pose_t));
    path->length = poses->size;
    for(int i = 0; i < poses->size; i++){
        path->poses[i] = vector_at(poses, pose_t, i);
    }
    return path;
}

void path_free(path_t *path){
    free(path);
}

typedef struct{
    int16_t cost;   // -1表示未加入队列
    pose_t from;
} bfs_record_t;



#define record_of(pose) bfs_records[pose.cordinate.x][pose.cordinate.y][pose.direction]
static bfs_record_t bfs_records[MAX_COLS][MAX_ROWS][4];
#define process_next(next) \
    bfs_record_t *next_record = &record_of(next);   \
    if(next_record->cost == -1 || next_record->cost > new_cost){   \
        next_record->cost = new_cost;   \
        next_record->from = current;   \
        vector_push_back(poses, next);   \
    }

path_t *map_calc_path(pose_t start, pose_t end){
    vector_t *poses = vector_new(pose_t);
    memset(bfs_records, 0xFF, sizeof(bfs_records)); // 把cost设置为 -1

    record_of(start).cost = 0;
    record_of(start).from = start;
    vector_push_back(poses, start);

    while(!vector_empty(poses)){
        pose_t current = vector_pop_front(poses, pose_t);
        if(pose_equal(current, end)){
            break;
        }
        int16_t new_cost = record_of(current).cost + 1;

        // 直行
        if(!map_border_is_barrier(current.cordinate, current.direction)){
            pose_t next = {
                .direction = current.direction,
                .cordinate = cordinate_neighbor(current.cordinate, current.direction)
            };
            if(map_valid_cordinate(next.cordinate)){
                process_next(next);
            }
        }

        // 转弯
        for(direction_t d = 0; d < 4; d++){
            if(d == DIRECTION_F)    continue;
            direction_t abs = direction_relative_apply(current.direction, d);
            if(!map_border_is_barrier(current.cordinate, abs)){
                pose_t next = {
                    .direction = abs,
                    .cordinate = current.cordinate
                };
                process_next(next);
            }
        }
    }
    vector_delete(poses);
    if(record_of(end).cost == -1){
        printf("ERR: no path found from (%d, %d, %d) to (%d, %d, %d)\n", start.cordinate.x, start.cordinate.y, start.direction, end.cordinate.x, end.cordinate.y, end.direction);
        return NULL;
    }else{
        // 回溯
        vector_t *steps = vector_new(pose_t);
        pose_t current = end;
        while(!pose_equal(current, start)){
            vector_push_back(steps, current);
            current = record_of(current).from;
        }
        vector_reverse(steps);
        path_t *rst = path_from_pose_vector(steps);
        vector_delete(steps);
        return rst;
    }
}