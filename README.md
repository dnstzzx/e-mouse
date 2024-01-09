# 智能电脑鼠高级迷宫小车
![8e82b34e52f0d6c31afdf8772360ef7.jpg](https://image.lceda.cn/pullimage/OdnvftMGMK6XyXdUTu8d0OMZwZmMyBJ8NYdVKrsl.jpeg)

# 需求描述

小车在寻找终点和遍历的过程中, 使用迷宫数组记录迷宫格子信息, 使用回溯算法返回到上一个节点或者岔路口。遍历完成后, 使用深度优先法则, 创建等高表, 根据等高表将最短路径存储进入迷宫数组, 最后根据路径进行冲刺,完成所有任务。

## 设计需求

* 指定使用主控：立创开发板设计；
* 迷宫电老鼠在迷宫中能够正常修正, 转弯, 掉头, 自由行走，完成对迷宫的扫描；
* 从终点开始遍历回到起点, 中间需要判断路径是否被走过来选择未遍历过的路径以及回溯路径；
* 返回到起点后, 根据遍历得到的迷宫地图使用广度优先算法BFS计算出等高表, 再根据等高表计算出最优路径,存储迷宫数组的高四位；
* 根据最后产生的迷宫数组的高四位, 最后以此进行冲刺；
* 智能电脑数整体尺寸不得大于10x10cm；
* 可适应不同的迷宫地图；
* 使用嘉立创EDA专业版设计；
* 不能使用模块搭建整体小车，特殊模块除外；

# 整体方案

* 主控： 立创开发板
* 电源： 3S航模电池
* 底盘： 2带编码器的N20电机+2被动轮
* 传感器：使用地磁传感器感知方向，使用TOF距离传感器感知地图。
* 开发框架：STM32 HAL
* 工具链：CubeMx + GNU/Make + GCC
* 任务流程：
![image.png](https://image.lceda.cn/pullimage/OSfZD3LVY3CiZ3bWHDfodL1TVIXcXPu0a2rnvjOh.png)

# 算法设计

## 地图数据结构

``` c
// ------------------ 地图定义 ------------------
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

```

## 建图算法

建图使用DFS(深度优先搜索)，根据前、左、右的探索顺序先走到某个尽头后，回溯到最近的一个分叉点。重复这个过程直至所有坐标都完成搜索。

![image.png](https://image.lceda.cn/pullimage/Dllrdo59oJqKyvQjUNUtm7GOFIjsncWzmB77uUqU.png)

``` C
void start_build_map(){	
    // ...
	pose_t current;
    vector_t *to_explore = vector_new(pose_t);

    current = (pose_t){.direction = DIRECTION_F, .cordinate = {0, 0}};
    vector_push_back(to_explore, current);
    while (vector_size(to_explore) > 0){
        // 前往下一个目标
        pose_t target = vector_pop_back(to_explore, pose_t);
        if(map_cell_of(target.cordinate)->visited) continue;
        map_cell_of(target.cordinate)->visited = true;

        // 计算路径并执行
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
        for_each_explorer_direction(dir){	// 前左右
            direction_t abs = direction_relative_apply(current.direction, dir);
            if(!map_valid_cordinate(cordinate_neighbor(current.cordinate, abs))) continue;	// 边界检查
            if(!map_border_is_marked(current.cordinate, abs)){	
                bool has_barrier = sensor_data_valid(sensor_distances[dir]) && sensor_distances[dir] < (GRID_SIZE * 1.25);
                map_mark_border(current.cordinate, abs, has_barrier);	// 记录边界信息
                if(!has_barrier){
                    pose_t next = {
                        .direction = abs,
                        .cordinate = cordinate_neighbor(current.cordinate, abs)
                    };
                    vector_push_back(to_explore, next);	// 将邻接节点加入栈
                    printf("add target: (%d, %d, %d)\n", next.cordinate.x, next.cordinate.y, next.direction);
                }
            }
        }

        // ...
    };

    vector_delete(to_explore);

    // ...
}

```

## 寻路算法

寻路使用带权的BFS(广度优先搜索)算法，可以在有环/无环带权图中计算最短路径(你可以通过调整直行和转弯的权重来生成更平直的路径)。将队列改为优先队列即为dijkstra算法。
![image.png](https://image.lceda.cn/pullimage/XArRXCYz7pFQ0EFjWZ4zNWHa4AuUflEWDznkGrxQ.png)

``` c
typedef struct{
    int16_t cost;   // -1表示未加入队列
    pose_t from;	// 回溯信息
} bfs_record_t;

static bfs_record_t bfs_records[MAX_COLS][MAX_ROWS][4];		// 状态空间: pose_t

#define record_of(pose) bfs_records[pose.cordinate.x][pose.cordinate.y][pose.direction]
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
        int16_t new_cost = record_of(current).cost + 1; // 直行和转弯的代价视为相同，如果要不同代价可在下面分别设置

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
    if(record_of(end).cost == -1){	// 终点不可达
        printf("ERR: no path found from (%d, %d, %d) to (%d, %d, %d)\n", start.cordinate.x, start.cordinate.y, start.direction, end.cordinate.x, end.cordinate.y, end.direction);
        return NULL;
    }else{
        // 回溯路径
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

```

## 静态修正算法

静态修正算法是小车通过定点旋转，动态扫描周围墙壁的距离信息，计算得到道路前进方向的算法。
![ca910e67-7ca4-4a78-8290-014078e5041a.gif](https://image.lceda.cn/pullimage/Xg5xcqJbL3XyHnejBlosPww1T4W7BH0UYLAHMJlt.gif)

首先我们需要获取周围有墙壁的方向，然后小范围定点旋转，在旋转过程中使用TOF传感器持续检测这些参考方向上墙壁的距离。

当这些参考方向上的距离和最小时，说明小车与周围墙壁方向保持一致~~前提是迷宫被假定为直角迷宫~~。

``` c
float action_wall_pad(uint8_t walls_mask){ // walls_mask 用于参考的墙壁掩码，低四位分别表示前右后左是否有参考墙
    float origin_az = get_current_azimuth();
    float range = WALL_PAD_RANGE;
    float points[3] = {origin_az - range, origin_az + range , origin_az};

    uint32_t min_sum = 99999;
    float min_sum_az = 0;
    float seg_start_az = origin_az;
    vector_t *az_history = vector_new(float);	// 延迟修正队列
    for(int i=0;i<3;i++){	// 需要完成三段旋转
        points[i] = fmodf(points[i], 360.0f);
        float speed = 0.7;
        bool turn_right = azimuth_delta(seg_start_az ,points[i]) > 0;
        if(turn_right) speed *= -1;
        vector_clear(az_history);

        if(DEBUG_WALL_PAD) printf("\n\nturn %s\n", turn_right ? "right" : "left");
        if(DEBUG_WALL_PAD) printf("\nstart az: %f, target az: %f\n\n", seg_start_az, points[i]);
        action_set_speed(-speed, speed);

        uint32_t last_t = HAL_GetTick();
        float delta;
        do{	// 持续扫描墙壁距离
            update_sensor_data();
            float current_az = get_current_azimuth();
            vector_push_back(az_history, current_az);
            if(az_history->size > 2)
                _vector_pop_front(az_history, NULL);
            uint32_t sum = 0;
            for(int d=0;d<4;d++){
                uint16_t dis = filtered_sensor_distances[d];
                if(walls_mask >> d & 1){
                    if(!sensor_data_valid(dis)){
                        printf("wall pad failed: sensor data invalid\n");
                        // action_stop();
                        // while(1) osDelay(1);
                    }
                    sum +=  dis;
                }
            }
            if(sum < min_sum){
                min_sum = sum;
                min_sum_az = vector_front(az_history, float);
            }
            osDelayUntil(last_t + 33);
            last_t = HAL_GetTick();
            delta = azimuth_delta(get_current_azimuth(), points[i]);
            if(DEBUG_WALL_PAD) printf("az: %f, delta: %f, sum: %d\n", get_current_azimuth(), delta, sum);
        }while(turn_right ^ (delta < 0) );
        seg_start_az = points[i];
        action_stop();
    }
    vector_delete(az_history);
    return min_sum_az;
}

```

## 动态修正算法(直行算法)

为了让小车能持续在格子中间走直线，我们需要保证：

* 小车的方向朝前，我们可以用地磁传感器获取小车的实时朝向并和静态修正算法得到的墙壁角度比较进行修正。
* 小车的位置在格子中线附近，我们可以用左边和右边的TOF传感器距离值来感知自身在格子中的横向位置进行修正。

因此我们可以将上面两个参考项进行PID修正

``` c
// 直行一帧，非阻塞
#define DEBUG_KEEP_FORWARD 0
void action_keep_forward(){
    int delta = 0;
    float spdl=0, spdr=0;

    if(DIS_FRONT < 50){
        action_stop();
        printf("keep forward failed: met barrier\n");
        while(1) osDelay(1);
    }

    if(!sensor_data_valid(DIS_LEFT) && !sensor_data_valid(DIS_RIGHT)){
        printf("keep forward bias failed\n");
    }else if(sensor_data_valid(DIS_LEFT) && DIS_LEFT_FILTERED <= DIS_RIGHT_FILTERED){
        delta = KEEP_FORWARD_SIDE_DIS_LEFT - (DIS_LEFT_FILTERED % GRID_SIZE);
    }else{
        delta = (DIS_RIGHT_FILTERED % GRID_SIZE) - KEEP_FORWARD_SIDE_DIS_RIGHT;
    }

    float fix_delta =  azimuth_delta(keep_forward_keeping_azimuth, get_current_azimuth());
    float bias_fdbk = pid_update(&keep_forward_bias_pid, 0.0f, (float)delta);
    float fix_fdbk = pid_update(&keep_forward_fix_pid, 0, fix_delta);


    spdl = 1.0f - bias_fdbk + fix_fdbk;
    spdr = 1.0 + bias_fdbk - fix_fdbk;
    spdl *= speed_rate;
    spdr *= speed_rate;

    action_set_speed(spdl, spdr);

    if(DEBUG_KEEP_FORWARD){
        //printf("\tleft:%d,  right:%d\n", DIS_LEFT, DIS_RIGHT);
        printf("delta: %d, bias_fdbk: %f\n",  delta, bias_fdbk);
        printf("fix: %f,%f\n",  fix_delta, fix_fdbk);
    }
}

```

## 地磁计校准算法

由于对于地球上某一点，地磁场的方向可视为恒定不变的，因此我们可以通过测量三个正交基的地磁场分量计算出自身的姿态角。然而，地磁场的强度很弱，因此地磁计测得的数据需要排除周围的磁场干扰。而在室内，由于空间中金属非常多且分布复杂，会导致空间磁场畸变非常严重。为了能使用地磁计指导小车直行与转向，我们需要在每一个坐标处进行地磁校准并记录下来(相当于记录地图各处的磁场信息)以便在冲刺时直接使用。

让我们先来看一下我们如何通过地磁计获得姿态角，为了方便描述，下面的计算均以x-y二维平面为例。

在正常情况下，我们以分别以X/Y轴磁感应强度作为X/Y轴，将传感器旋转一周得到的数据画在图上会得到下面a图所示圆形，数据点方位角arctan(y/x) 即为该数据点处传感器的方位角。

![1701285008941.jpg](https://image.lceda.cn/pullimage/CMbx2PmtBapsGNfIvxlPPlEb7F8naiHqFSlZUQiU.png)
图源： 参考文献1， 下二同
然而，空间中会有各种其他磁场会使该圆移动或者扭曲。这些干扰分为：

* 硬磁干扰： 方向与传感器角度无关的磁场，在各角度施加的干扰是相同的，表现为圆(球)心偏离原点
* 软磁干扰： 方向与传感器角度有关的磁场，在各角度施加的干扰是不同的，表现为圆(球)会扭曲为椭圆(球)

![1701285026186.jpg](https://image.lceda.cn/pullimage/znzM8fOf9X29SEt4zVJDPCT4wl6L8uoy03sZPCKW.png)
![1701285057384.jpg](https://image.lceda.cn/pullimage/jId6dCkoxX6HRElXpbzReZdkWGUxc4aiLrhdJEJI.png)

比较有效的方法是在原地旋转传感器一周(每个轴)，得到完整的测量曲线，并对其进行椭圆(球)拟合得到被干扰后的椭圆(球)方程，然后对其进行修正。

然而各种椭圆拟合算法较为复杂，不适合在嵌入式平台进行计算。因此我们将椭圆简化为矩形进行修正：检测各轴测量曲线的x、y最小值和最大值，得到一个矩形包络框，然后计算将该矩形变换为中心在原点的正方形。

``` c
typedef struct{
    bool enable_hard_calibrate;
    bool enable_soft_calibrate;
    int16_t center[3];
    float scale[3];
} bsp_5883_calibrator_t;

void bsp_5883_calibrate_data(const int16_t raw_data[3], int16_t output[3]){
    for(int i=0;i<3;i++){
        output[i] = raw_data[i];
        if(calibrator.enable_hard_calibrate){
            output[i] -= calibrator.center[i];
        }
        if(calibrator.enable_soft_calibrate){
            output[i] *= calibrator.scale[i];
        }
    }
}

static bool calibrating = false;
static bool calibrate_first = true;
static int16_t min_data[3], max_data[3];

void bsp_5883_updt_callback(const int16_t data[3]){
    if(!calibrating) return;
    if(calibrate_first){
        min_data[0] = max_data[0] = data[0];
        min_data[1] = max_data[1] = data[1];
        min_data[2] = max_data[2] = data[2];
        calibrate_first = false;
    }
    for(int i=0;i<3;i++){
        if(data[i] > max_data[i])
            max_data[i] = data[i];
        if(data[i] < min_data[i])
            min_data[i] = data[i];
    }
    //printf("%d,%d\n", data[0], data[1]);
}

void bsp_5883_start_calibrate(){
    memset(min_data, 0, sizeof(min_data));
    memset(max_data, 0, sizeof(max_data));
    calibrating = true;
    calibrate_first = true;
}

void bsp_5883_stop_calibrate(){
    calibrator.enable_hard_calibrate = true;
    calibrator.enable_soft_calibrate = true;
    int16_t delta[3], max_delta = 0;
    for(int i=0;i<3;i++){
        calibrator.center[i] = (min_data[i] + max_data[i]) / 2;
        delta[i] = max_data[i] - min_data[i];
        if(delta[i] > max_delta)
            max_delta = delta[i];
    }

    for(int i=0;i<3;i++){
        calibrator.scale[i] = (float)max_delta / delta[i];
    }
}

```

# 软件设计

软件部分基于STM32 HAL框架，目标平台为STM32F427，该平台与立创开发板所使用的GD32F450/GD32F470基本兼容,构建系统为GNU/Make。

代码开源地址： [https://github.com/dnstzzx/e-mouse](https://github.com/dnstzzx/e-mouse)

``` makefile
BSP_C_SOURCES = \
bsp_uart.c \
bsp_tasks.c  \
bsp_i2c.c \
bsp_motor.c \
bsp_vl53.c \
bsp_led.c \
bsp_5883.c \
bsp_int.c

APP_C_SOURCES = \
app_entry.c \
Mapping/src/odometer.c  \
Mapping/src/sensor.c  \
Mapping/src/action.c  \
Mapping/src/ropen.c \
Mapping/src/map.c \
Mapping/src/build_map.c

Components_C_SOURCES = \
algs/fifo.c \
algs/filters.c \
algs/vector.c \
vl53l0x/simple/vl53l0x.c \
algs/pid.c \
openocd_rtos_helper/FreeRTOS-openocd.c \
hmc5883l/hmc5883l.c

```

代码总体分为：

* Core主要以CubeMX生成代码为主
* BSP为板级功能代码
* APP为与任务相关的高层API及业务代码
* Components为算法及部分外部驱动代码

# 硬件设计

小车硬件由三层组成，第一层用于底盘控制，第二层用于传感器连接，第三层为立创梁山派主控板。

## 第一层

第一层为底盘层，主要包括电池接口、降压电路、电机驱动电路、电机孔位。
![image.png](https://image.lceda.cn/pullimage/6Gl7fDMR6aQ8cGDqJkVLtGlciOktir5HPPJDh2Mn.png)
![image.png](https://image.lceda.cn/pullimage/c7uZPEkPxJjjEJ44iolAbOgW6lexxSkCcNb4z2jB.png)

### 电机

本设计使用2主动轮+2被动轮设计。主动轮轴心与小车几何中心重合可使小车转向更为灵活。
主动轮使用带编码器的N20电机，可以使用电机编码器对小车移动速度和行程进行闭环控制。
电机驱动芯片为双半桥直流电机驱动芯片DRV8870

![image.png](https://image.lceda.cn/pullimage/CFGfuQRjYu0f8S6qi0f1fpW77vHWnjwr4DGXQtpW.png)

### DCDC电路

本设计使用3S锂电池进行供电，输入电压约12V。本层PCB板载一个12V-5V DCDC电路，通过CN2为梁山派供电。
![image.png](https://image.lceda.cn/pullimage/tt0wDByh4GLObaEl3YX0iOTFjkedBzwsawztGJJv.png)

## 第二层

第二层包括以下部分： 立创开发板接口、姿态传感器mpu6050、4 * TOF距离传感器vl53l0x、地磁传感器hmc5883L
![8de6ac633216623a63c1a2dd5efbc42.jpg](https://image.lceda.cn/pullimage/V9GWbw6KCuPPtVoPND0v01UUuOTBvGTrui9Vm6YC.jpeg)
![92d317808f738dc55940ef68770068f.jpg](https://image.lceda.cn/pullimage/BVGfBP28LKNyVTZ5ECIJxNKIquwQdj6xprek5rz2.jpeg)

## VL53L0X矩阵

上层版四边的中间各装有一个VL53L0x,它不断发射红外信号并接收反射回来的信号，根据发射与接收的时间差(TOF)可以计算出小车与四个方向的障碍物的距离。出于传感器的一致性考量，避免手焊带来的各种差异问题，各个VL53L0x为同一型号的模组并通过2.54mm弯排针接入上层版。

VL53L0x通过I2C总线与主控进行通信，所有的Vl53L0x传感器都在I2C0总线上。由于VL53L0x没有地址记忆或硬件选择功能，需要每次上电后重新设置地址。因此当多个VL53L0x在同一I2C总线时，需要为每个传感器加入一路片选信号，即SHUT1-SHUT4。
![image.png](https://image.lceda.cn/pullimage/Ndi4UY74LKGmD1EbpvkDttYbMapCiZgNRt3I8Lsj.png)![image.png](https://image.lceda.cn/pullimage/Itbl758XegKDrLX3sQyhAoGk2BOmgnbGVQRZv6qg.png)
同时，由于I2C为开漏输出协议，需要加入额外上拉电阻(各模块的上拉电阻已被拆除)。
![image.png](https://image.lceda.cn/pullimage/gWDUs8TMCApFd0hSKq5gZn3NyLiKx9YSxIH3mvCt.png)

## MPU6060与HMC5883

上层版上有一个用于测量运动状态的加速度计+陀螺仪MPU6050和一个用于测量姿态角的磁力计HMC5883。二者均使用I2C进行通信并共用I2C1总线。

在实际任务中，MPU6050并未被使用，因此没有焊接。

![image.png](https://image.lceda.cn/pullimage/zQVdtVAHBcPgnEwH8JcpgCv0GHuMyhD0FVR49VJM.png)![image.png](https://image.lceda.cn/pullimage/jD425EcFiLNevMJjmAwxBjKGRPEiDBCw4QDz3UJc.png)![image.png](https://image.lceda.cn/pullimage/cRSTvisMZGYQRYmtGwVcxjRBL9UaUocCl0mSdDp3.png)

## BOM

### 底板BOM

| 物料 | 数量 | 位号 | 封装 | 链接 |
| --- | --- | --- | --- | --- |
| 54.9K 1% | 1 | R2 | R0603 | 标准贴片元件 |
| 10K 1% | 1 | R3 | R0603 | 标准贴片元件 |
| 10K | 1 | R1 | R0603 | 标准贴片元件 |
| 100nF | 7 | C2,C4,C7,C9,C10,C13,C14 | C0603 | 标准贴片元件 |
| 22uF | 2 | C5,C6 | C0805 | 标准贴片元件 |
| 330pF | 1 | C8 | C0603 | 标准贴片元件 |
| 10uF | 2 | C11,C12 | C0603 | 标准贴片元件 |
| 47uF | 2 | C1,C3 | CAP-TH_BD5.0-P2.00-D0.8-FD | C88724 |
| DRV8870DDAR | 2 | U1,U2 | HSOP-8_L5.0-W4.0-P1.27-LS6.2-BL-EP | C86590 |
| HDGC1501WR-S-6P | 2 | U3,U4 | CONN\-SMD\_6P\-P1\.50\_HDGC1501WR\-S\-6P | C2936220 |
| TPS563201DDCR | 1 | U5 | SOT-23-6_L2.9-W1.6-P0.95-LS2.8-BL | C116592 |
| XT60PW-M | 1 | CN1 | CONN-TH_XT60PW-M | C98732 |
| HX25003-2AWB | 1 | CN2 | CONN\-SMD\_2P\-P2\.50\_HX25003\-2AWB | C442361 |
| HX25003-8AB | 1 | CN3 | CONN\-SMD\_8P\-P2\.50\_HX25003\-8AB | C442360 |
| 3.3uH | 1 | L1 | IND-SMD_L4.4-W4.2 | C408336 |

### 上版BOM

| 物料 | 数量 | 位号 | 封装 | 链接 |
| --- | --- | --- | --- | --- |
| 0Ω | 1 | R1 | R0603 | 标准贴片元件 |
| 4.7K | 4 | R3,R4,R5,R6 | R0603 | 标准贴片元件 |
| 2.2nF | 1 | C6 | C0603 | 标准贴片元件 |
| 100nF | 4 | C7,C8,C9,C10 | C0603 | 标准贴片元件 |
| 220nF | 1 | C11 | C0603 | 标准贴片元件 |
| 4.7uF | 1 | C12 | C0603 | 标准贴片元件 |
| 100nF | 2 | C13,C16 | C0603 | 标准贴片元件 |
| 10uF | 2 | C14,C15 | C0603 | 标准贴片元件 |
| HX25003-8AB | 1 | CN1 | CONN\-SMD\_8P\-P2\.50\_HX25003\-8AB | C442360 |
| HX25003-2AWB | 1 | CN2 | CONN\-SMD\_2P\-P2\.50\_HX25003\-2AWB | C442361 |
| HDR\-F\_2\.54\_2x4 | 1 | H8 | HDR-TH_8P-P2.54-V-F-R2-C4-S2.54-1 | C92271 |
| 贴片排母2x20 | 2 | P1,P2 | 贴片排母2x20 | C3975165 |
| MPU-6050 | 1 | U2 | QFN-24_L4.0-W4.0-P0.50-BL-EP2.7 | C24112 |
| HMC5883L | 1 | U3 | LPCC-16_L3.0-W3.0-P0.50-BL | C2859540 |

### 非焊接BOM

| 物料 | 数量 | 位置 | 备注 | 链接 |
| --- | --- | --- | --- | --- |
| 带编码器的N20减速电机 | 2 | 下板下方 | 12V 减速比100 | [GA12-N20减速马达编码器 N20微型直流减速电机 霍尔编码器减速-淘宝网 \(taobao.com\)](https://item.taobao.com/item.htm?id=565304239241) |
| N20轮子 | 2 | 电机轴处 | 34mm轮子 | [D字轴橡胶轮胎34/43MM机器人配件循迹小车模型车轮 配N20减速电机-tmall.com天猫](https://detail.tmall.com/item.htm?id=655446533609&skuId=4733598053237) |
| 被动轮 | 2 | 下板下方 | CY-12D 主体尼龙 | [带杆万向不锈钢滚珠轴承牛眼轮尼龙球滚轮万向轮输送机器传送机床-tmall.com天猫](https://detail.tmall.com/item.htm?id=589837992993&skuId=4042469705121) |
| M5垫片 | 4 | 下板与被动轮间 |  | 标准元件 |
| M7垫片 | 2 | 下板与被动轮间 |  | 标准元件 |
| M5螺母 | 2 | 下板上方，用于固定被动轮 |  | 标准元件 |
| VL53L0x模组 | 4 | 上板H1-H4 | 单孔蓝色版 | [GY-530 VL53L0X 激光测距传感器 ToF测距 飞行时间测距传感器模块-淘宝网 \(taobao.com\)](https://item.taobao.com/item.htm?id=551955373190) |
| XH2.54 2pin端子线 | 1 | 连接上板CN2与下板CN2 | 同序连接 | 标准元件 |
| XH2.54 8pin端子线 | 1 | 连接上板CN1与下板CN3 | 反序连接 | 标准元件 |
| 立创开发板 | 1 | 插在上板P1与P2上 | 注意方向，两块板子的丝印方向一致，立创开发板的调试接口应与上板CN2在同一侧 |  |
| 3S电池 | 1 | 下板 CN1 | XT60接口 | [格氏电池格式航模电池3S 2S4S高倍率动力锂电池12V需配专用充电器-淘宝网 \(taobao.com\)](https://item.taobao.com/item.htm?id=41728701920) |

## 实物图
![8e82b34e52f0d6c31afdf8772360ef7.jpg](https://image.lceda.cn/pullimage/OdnvftMGMK6XyXdUTu8d0OMZwZmMyBJ8NYdVKrsl.jpeg)

# 参考文献
1. [https://zhuanlan.zhihu.com/p/98325286](https://zhuanlan.zhihu.com/p/98325286)
