#include "stdint.h"
#include "stdbool.h"
#include "stdio.h"
#include "string.h"
#include "vl53l0x.h"
#include "bsp_motor.h"
#include "cmsis_os.h"
#include "bsp_vl53.h"
#include "stm32f4xx_hal.h"

#define LEN 150
#define MAX_ROWS 10
#define MAX_COLS 5
#define MAX_DISTANCE 180

#define False false
#define True true
// 直行速度，接口速度是多少圈每秒，理论每圈10.6cm，浮点型
#define FORWARD_SPEED 1.0f

uint8_t visited[MAX_ROWS][MAX_COLS];

bool down_data[][5] = {{False, True, False, False, True}, {True, False, False, True, False}, {True, False, True, False, True}, {True, True, False, True, False}, {True, False, True, True, False}, {True, False, False, True, False}, {True, False, False, False, True}, {False, True, False, True, False}, {False, False, True, False, False}, {True, True, True, True, True}};
bool right_data[][5] = {{False, False, True, False, False}, {False, True, True, False, True}, {False, True, True, False, True}, {True, False, False, True, True}, {False, False, False, False, True}, {False, True, False, True, True}, {False, True, True, False, True}, {True, False, False, True, True}, {True, False, True, True, True}, {False, False, False, False, True}};

// 回溯路径，目前所在索引
uint8_t path_index = 0;

// 马达状态
enum MOTOR_STATE{STOPPING, FORWARD, TURN_LEFT, TURN_RIGHT, TURN_BACK, WAITING};

// 马达状态
uint8_t motor_state = STOPPING;

// 机器人状态
enum ROBOT_ACTION{FINISH, BACK_TO_BIF, MAPPING};

// 机器人状态
uint8_t robot_action = MAPPING;

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

// 队列
QueueNode queue[LEN];

 // 机器人姿态，启动瞬间，正前方为X轴正方向，正左方为Y轴正方向; 初始坐标为 (0, 0); direction : 0 X轴正方向，1 Y轴正方向， 2 X轴负方向， 3 Y轴负方向
typedef struct{
    Coordinate cord;
    uint8_t direction;
}POSE;

POSE pose;

typedef struct {
    uint8_t len;
    Coordinate coords[LEN];
} PATH;

PATH path;

float left, right;

bool bfsShortestPath(Coordinate start, Coordinate end, PATH *path) {
    memset(visited, (uint8_t)MAX_DISTANCE, sizeof(visited)); // 初始化
    printf("%d %d\n", visited[0][1], visited[1][1]);

    printf("start from (%d, %d)\n", start.x, start.y);
    int8_t dx[] = {-1, 0, 1, 0}; // 上、右、下、左四个方向的x偏移量
    int8_t dy[] = {0, 1, 0, -1}; // 上、右、下、左四个方向的y偏移量

    uint8_t front = 0, rear = 0; // 队列的前后指针

    // 将起始节点加入队列
    queue[rear++] = (QueueNode){start, 0};
    visited[start.x][start.y] = 0;

    // 开始BFS
    while (front != rear) {
        QueueNode currentNode = queue[front];
        front = (front + 1) % LEN;

        Coordinate currentCoord = currentNode.coord;
        uint8_t currentDistance = currentNode.distance;

        printf("current (%d,%d), current distance is %d\n", currentCoord.x, currentCoord.y, currentDistance);

        // 如果找到目标节点，构造路径并返回true
        if (currentCoord.x == end.x && currentCoord.y == end.y) {
            printf("founded (%d, %d)\n", end.x, end.y);
            path->len = currentDistance + 1;
            path->coords[path->len-1] = currentCoord;
            uint8_t now = path->len - 2;
            path->coords[0] = start;

            while(currentCoord.x != start.x || currentCoord.y != start.y){
                for (uint8_t i = 0; i < 4; i++) {
                    Coordinate new_cord = (Coordinate){currentCoord.x + dx[i], currentCoord.y + dy[i]};
                    if(new_cord.x < 0 || new_cord.y < 0 || new_cord.x > MAX_ROWS - 1 || new_cord.y > MAX_COLS - 1)
                        continue;
                    if (visited[new_cord.x][new_cord.y] == currentDistance-1) {
                        bool add_flag = false;
                        switch (i){
                        case 0:
                            if(!down_data[new_cord.x][new_cord.y])
                                add_flag = true;
                            break;
                        case 1:
                            if(!right_data[currentCoord.x][currentCoord.y])
                                add_flag = true;
                            break;
                        case 2:
                            if(!down_data[currentCoord.x][currentCoord.y])
                                add_flag = true;
                            break;
                        case 3:
                            //printf("(%d,%d) right data is %d\n", new_cord.x, new_cord.y, right_data[new_cord.x][new_cord.y]);
                            if(!right_data[new_cord.x][new_cord.y])
                                add_flag = true;
                            break;
                        }

                        if(add_flag){
                            path->coords[now--] = new_cord;
                            currentCoord = new_cord;
                            currentDistance -= 1;
                            break;
                        }
                    }
                }
            }

            return true;
        }

        // 否则，继续搜索四个方向
        for (uint8_t i = 0; i < 4; i++) {
            Coordinate new_cord = (Coordinate){currentCoord.x + dx[i], currentCoord.y + dy[i]};
            if(new_cord.x < 0 || new_cord.y < 0 || new_cord.x > MAX_ROWS - 1 || new_cord.y > MAX_COLS - 1)
                continue;

            printf("try (%d,%d), index is %d, visited is %d\n", new_cord.x, new_cord.y, i, visited[new_cord.x][new_cord.y]);

            if (currentDistance < visited[new_cord.x][new_cord.y]) {
                
                bool add_flag = false;
                switch (i){
                case 0:
                    if(!down_data[new_cord.x][new_cord.y])
                        add_flag = true;
                    break;
                case 1:
                    if(!right_data[currentCoord.x][currentCoord.y])
                        add_flag = true;
                    break;
                case 2:
                    if(!down_data[currentCoord.x][currentCoord.y])
                        add_flag = true;
                    break;
                case 3:
                    //printf("(%d,%d) right data is %d\n", new_cord.x, new_cord.y, right_data[new_cord.x][new_cord.y]);
                    if(!right_data[new_cord.x][new_cord.y])
                        add_flag = true;
                    break;
                }

                if(add_flag){
                    // 将新的可达节点加入队列
                    queue[rear] = (QueueNode){{new_cord.x, new_cord.y}, currentDistance + 1};
                    rear = (rear + 1) % LEN;
                    visited[new_cord.x][new_cord.y] = currentDistance + 1;
                }
            }
        }
    }

    // 如果无法找到路径，返回false
    return false;
}

void printPath(PATH *path){
    printf("len %d\n", path->len);
    for(uint8_t i = 0; i < path->len; i++){
        printf("(%d,%d)\n", path->coords[i].x, path->coords[i].y);
    }
}

const int8_t direction_k[4][2] = {{1,0}, {0,1},{-1,0},{0,-1}};

// 联合机器人方向计算坐标
static inline Coordinate cal_cord_with_direction(uint8_t rel_direction, uint8_t offset, POSE *pose){
    Coordinate new_cord = {direction_k[(pose->direction + rel_direction) % 4][0]*offset + pose->cord.x, direction_k[(pose->direction + rel_direction) % 4][1]*offset + pose->cord.y};
    return new_cord;
}

void cal_turn(){
    Coordinate forward_pos = cal_cord_with_direction(0, 1, &pose);
    Coordinate left_pos = cal_cord_with_direction(1, 1, &pose);
    Coordinate back_pos = cal_cord_with_direction(2, 1, &pose);
    Coordinate right_pos = cal_cord_with_direction(3, 1, &pose);

    if(Coords_eq(&path.coords[path_index+1], &forward_pos))
        change_state(FORWARD);
    else if(Coords_eq(&path.coords[path_index+1], &left_pos))
        change_state(TURN_LEFT);
    else if(Coords_eq(&path.coords[path_index+1], &back_pos))
        change_state(TURN_BACK);
    else if(Coords_eq(&path.coords[path_index+1], &right_pos))
        change_state(TURN_RIGHT);
}

#define GRID_POS 3.773584

// 累加值
int32_t encoder_accu = 0;
int32_t _encoder_accu_start = 0;

void get_encoder_val(){
    encoder_accu = BSP_MOTOR_L->current_pos - _encoder_accu_start;
}

void mem_encoder_val(){
    _encoder_accu_start = BSP_MOTOR_L->current_pos;
    encoder_accu = 0;
}

void change_state(uint8_t state){
    motor_state = state;
    mem_encoder_val();
}

float get_abs_val(){
    return BSP_MOTOR_L->current_pos / 2800;
}


void set_motor(){

    if(motor_state == FORWARD){            
        set_flag = true;
        pos[0] += GRID_POS;
        pos[1] += GRID_POS;
    }

    else if(motor_state == TURN_LEFT){
        set_flag = true;
        pos[0] -= GRID_POS;
        pos[1] += GRID_POS;
    }

    else if(motor_state == TURN_RIGHT || motor_state == TURN_BACK){
        set_flag = true;
        pos[0] += GRID_POS;
        pos[1] -= GRID_POS;
    }

    if(set_flag){
        for(int i = 0 ; i < 2; i++){
            bsp_motor_set_pos(&bsp_motors[i], pos[i]);
        }
    }

}

void init(){
    // init map
    pose.cord = (Coordinate){9,4};
    pose.direction = 3;

    // init motor
    bsp_motor_init();    
    for(int i = 0 ; i < 2; i++){
        bsp_motor_set_ctrl_mode(&bsp_motors[i], BSP_MOTOR_CTRL_MODE_SPEED);
    }

    // init state
    change_state(WAITING);
    mem_encoder_val();
}

void p2p(Coordinate start, Coordinate end){
    if(bfsShortestPath(pose.cord, end, &path)){
        path_index = 1;
    }
    else{
        printf("error\n");
    }
}

void print_pose(){
    printf("\tcoordinate:(%d,%d), direction:%d\n", pose.cord.x, pose.cord.y, pose.direction);
}

#define task_len 2
Coordinate tasks[task_len] = {{7,4}, {8,3}};
uint8_t task_cnt = 0;


// 周期，与频率互为倒数
#define CYCLE 0.05

// 周期，毫秒
#define CYCLE_MS 50


void update_motor_state(){
    get_encoder_val();

    if(motor_state == STOPPING){
        if(task_cnt == task_len - 1){
            robot_action = FINISH;
            motor_state = STOPPING;
        }
        else{
            p2p(pose.cord, tasks[task_cnt++]);
            robot_action = MAPPING;
        }
    }

    else if(motor_state == WAITING){
        if(path_index == path.len - 1){
            motor_state = STOPPING;
        }
        else
            cal_turn(); // state have been change here
    }

    else if(motor_state == FORWARD){
        if (abs(encoder_accu) > TURN_90_ENCODER_VAL - encoder_time_error){ // 判断是否已经转了90度
            change_state(WAITING);
        }
    }
    else if(motor_state == TURN_LEFT){
        if (abs(encoder_accu) > TURN_90_ENCODER_VAL - encoder_time_error){ // 判断是否已经转了90度
            change_state(WAITING);
            pose.direction = (pose.direction + 1) % 4;
        }
    }
    else if(motor_state == TURN_RIGHT){
        if (abs(encoder_accu) > TURN_90_ENCODER_VAL - encoder_time_error){ // 判断是否已经转了90度
            change_state(WAITING);
            pose.direction = (pose.direction + 3) % 4;
        }
    }
    else if(motor_state == TURN_BACK){
        if (abs(encoder_accu) > TURN_180_ENCODER_VAL - encoder_time_error){ // 判断是否已经转了90度
            change_state(WAITING);
            pose.direction = (pose.direction + 2) % 4;
        }
    }
}



void main_open(){
    init();
    uint32_t main_cnt = 0;

    while(1){
        update_motor_state();
        set_motor();
        if(main_cnt % 30 == 0){
            printf("iter %d\n", main_cnt);
            print_pose();
        }
        main_cnt++;
    }

    count_up_forever();
}
