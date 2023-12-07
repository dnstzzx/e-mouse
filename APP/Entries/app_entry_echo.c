#include "app_entry.h"
#include "stdio.h"
#include "bsp_uart.h"
#include "cmsis_os.h"
#include "bsp_task.h"

void app_idle_task(){
    static char a[1024];
    bsp_task_create("spd rpt", count_up_forever, NULL, osPriorityBelowNormal1, 128);
    
    while(1){
        /*
        uint32_t count = 0;
        do{
            osDelay(1);
            count = bsp_uart1_received_data_count();
        }while(count==0);
        uint32_t len = bsp_uart1_read_data(a, 1024);
        bsp_uart1_send_data(a, len);
        */
       
        /* bsp_uart1_read_line(a, 1024);
        printf("%s\n", a); */

        
        bsp_uart1_read_line(a, 1024);
        int d;
        sscanf(a, "%d", &d);
        printf("%d\n", d);
        osDelay(100);
        
    }

    
}