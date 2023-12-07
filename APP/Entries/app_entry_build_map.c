void ropen_main();

void app_idle_task(){
    //bsp_uart1_read_char();
    //while(bsp_uart1_received_data_count())  bsp_uart1_read_char();
    //main_mapping();
    ropen_main();
}