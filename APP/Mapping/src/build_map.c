#include <stdlib.h>
#include "cmsis_os.h"
#include "bsp_motor.h"
#include "bsp_5883.h"
#include "map.h"
#include "vector.h"
#include "action.h"
#include "sensor.h"
#include "build_map.h"

direction_t explorer_order[] = {DIRECTION_F, DIRECTION_L, DIRECTION_R};
pose_t build_map_task_start = {.cordinate = {0, 0}, .direction = DIRECTION_F};
pose_t build_map_task_end = {.cordinate = {0, 5}, .direction = DIRECTION_R};

#define DEBUG_EXEC_PATH 1
void exec_path(path_t *path, pose_t current){
    float old_origin = origin_azimuth;
    bsp_5883_calibrator_t old_calibrator;
    bsp_5883_get_calibrator(&old_calibrator);

    bool last_is_straight = false;
    for(int i = 0; i < path->length; i++){
        if(map_cell_of(current.cordinate)->has_calibrator){
            bsp_5883_set_calibrator(&(map_cell_of(current.cordinate)->calibrator));
        }
        if(map_cell_of(current.cordinate)->origin != -1){
            origin_azimuth = map_cell_of(current.cordinate)->origin;
        }
        pose_t next = path->poses[i];
        if(DEBUG_EXEC_PATH) printf("next: (%d, %d, %d)\n", next.cordinate.x, next.cordinate.y, next.direction);
        if(cordinate_equal(next.cordinate, current.cordinate)){
            action_stop();
            osDelay(300);
            //action_turn(direction_relative_get(current.direction, next.direction));
            action_pad_to_direction(next.direction);
            action_stop();
            last_is_straight = false;
        }else{
            action_keep_forward_cell(1);
            last_is_straight = true;
        }
        current = next;
    }
    if(last_is_straight){
        action_stop();
        osDelay(300);
        action_pad();
        action_stop();
        osDelay(300);
    }
    origin_azimuth = old_origin;
    bsp_5883_set_calibrator(&old_calibrator);
    osDelay(100);
}

#define for_each_explorer_direction(direction) \
    for(int _i=2;_i>=0;_i--)\
        for(;;) \
        for(direction_t direction = explorer_order[_i];;({break;}))

void start_build_map(){
    map_init();
    action_switch_motor_mode(BSP_MOTOR_CTRL_MODE_SPEED);
    action_make_origin();
    osDelay(200);

    pose_t current;
    vector_t *to_explore = vector_new(pose_t);

    // todo : 从外部进入
    current = (pose_t){.direction = DIRECTION_F, .cordinate = {0, 0}};
    vector_push_back(to_explore, current);
    while (vector_size(to_explore) > 0){
        // 前往下一个目标
        pose_t target = vector_pop_back(to_explore, pose_t);
        if(map_cell_of(target.cordinate)->visited) continue;
        map_cell_of(target.cordinate)->visited = true;
        
        printf("target: (%d, %d, %d)\n", target.cordinate.x, target.cordinate.y, target.direction);
        path_t *path = map_calc_path(current, target);
        if(!path){
            printf("no path to target(%d, %d, %d), skipped\n", target.cordinate.x, target.cordinate.y, target.direction);
            continue;
        }
        exec_path(path, current);
        path_free(path);
        current = target;

        // 探索
        osDelay(33);
        update_sensor_data();
        for_each_explorer_direction(dir){
            direction_t abs = direction_relative_apply(current.direction, dir);
            if(!map_valid_cordinate(cordinate_neighbor(current.cordinate, abs))) continue;
            if(!map_border_is_marked(current.cordinate, abs)){
                bool has_barrier = sensor_data_valid(sensor_distances[dir]) && sensor_distances[dir] < (GRID_SIZE * 1.25);
                map_mark_border(current.cordinate, abs, has_barrier);
                if(!has_barrier){
                    pose_t next = {
                        .direction = abs, 
                        .cordinate = cordinate_neighbor(current.cordinate, abs)
                    };
                    vector_push_back(to_explore, next);
                    printf("add target: (%d, %d, %d)\n", next.cordinate.x, next.cordinate.y, next.direction);
                }
            }
        }

        // 记录校准器
        action_calibrate_and_turn(DIRECTION_F);
        action_stop();
        bsp_5883_get_calibrator(&(map_cell_of(current.cordinate)->calibrator));
        map_cell_of(current.cordinate)->has_calibrator = true;
        
        // 方向基校准
        uint8_t wall_mask = 0;
        for(direction_t i=0;i<4;i++){
            if(map_border_is_barrier(current.cordinate, direction_relative_apply(current.direction, i))){
                wall_mask |= 1 << i;
            }
        }
        printf("wall mask: %d\n", wall_mask);
        
        if(wall_mask != 0){
            float old_origin = action_wall_pad_and_fix_origin(wall_mask);
            map_cell_of(current.cordinate)->origin = origin_azimuth;
            printf("old origin: %f, new origin: %f\n", old_origin, origin_azimuth);
        }
    };
    
    vector_delete(to_explore);

    // 返回起点
    path_t *path = map_calc_path(current, (pose_t){.direction = DIRECTION_F, .cordinate = {0, 0}});
    exec_path(path, current);
    path_free(path);
}

void rush_map(){
    path_t *path = map_calc_path(build_map_task_start, build_map_task_end);
    exec_path(path, build_map_task_start);
    path_free(path);
}