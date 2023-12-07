// #include "base.h"
// #include "build_map.h"
// #include "bfs.h"

// PATH path;

// void test_init(){
//     printf("allow_parsing is %d\n", is_diff);
//     printMaze();
//     for(int i = 0; i<4; i++){
//         for(int j = 0; j < 2; j++){
//             printf("%d ", sensor_index[i][j]);
//         }
//     }
//     printf("\n");
//     for(int i = 0; i<4; i++){
//         for(int j = 0; j < 2; j++){
//             printf("%d ", direction_k[i][j]);
//         }
//     }
//     printf("\n");
// }

// void test_robot(){
//     memset(maze, '$', sizeof(maze));
//     maze[0][0] = '+';
//     Coordinate left = cal_cord_with_direction(1, 1, &pose);
//     Coordinate forward = cal_cord_with_direction(0, 2, &pose);
//     Coordinate right = cal_cord_with_direction(3, 3, &pose);
//     printf("(%d,%d) (%d,%d) (%d,%d)\n", left.x, left.y, forward.x, forward.y, right.x, right.y);
// }


// void test_bfs(){
//     memset(maze, '$', sizeof(maze));
//     printMaze();
//     setPath((Coordinate){0,0});
//     printMaze();
//     setPath((Coordinate){0,1});
//     setPath((Coordinate){0,2});
//     memPath((Coordinate){0,0});
//     memPath((Coordinate){0,1});
//     memPath((Coordinate){0,2});
//     setObstacle((Coordinate){1,0});
//     setObstacle((Coordinate){1,1});
//     setPath((Coordinate){0,3});
//     setPath((Coordinate){0,4});
//     setPath((Coordinate){0,5});
//     memPath((Coordinate){0,3});
//     memPath((Coordinate){0,4});
//     memPath((Coordinate){0,5});
//     setPath((Coordinate){1,5});
//     setPath((Coordinate){2,2});
//     printPath(&path);
//     if(bfsShortestPath((Coordinate){0, 0}, &path)){
//         printf("founded\n");
//     }
//     printPath(&path);
//     printMaze();
// }

// int test_main(){

//     //test_init();
//     //test_bfs();
//     test_robot();
//     return 1;
// }