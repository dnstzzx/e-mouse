#include "app_entry.h"
#include "stdio.h"
#include "stdbool.h"
#include "vl53l0x.h"
#include "cmsis_os.h"
#include "bsp_uart.h"
#include "build_map.h"

void app_idle_task();
void count_up_forever(){
    size_t aaa = 0;
    while(1){
        printf("%d\n", aaa);
        osDelay(1000);
        aaa ++;
    }
}

//#include "app_entry_motor.c"
#include "app_entry_build_map.c"
//#include "app_entry_path_calc.c"
//#include "app_entry_vl53.c"
//#include "app_entry_5883.c"
//#include "app_entry_i2c.c"
//#include "app_entry_echo.c"