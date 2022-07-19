/*
 * File:          map_generator.c
 * Date:          2/23/2021
 * Description:   Generates map of accessible pathways on a single file
 * Author:        Seyed Amirhosein Mohaddesi
 * Modifications: 
 */


#include <webots/distance_sensor.h>
#include <webots/compass.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>

// Definitions for Webots environment
#define TIME_STEP 64
#define PLATFORM_X 64
#define PLATFORM_Z 64

int find_index(double x){
    if(floor(2*x+32)<0)
      return 0;
    if(floor(2*x+32)>63)
      return 63;
    return floor(2*x+32);
} 

int main(int argc, char **argv) {
    wb_robot_init();
    
    // do this once only
    WbNodeRef robot_node = wb_supervisor_node_get_from_def("PR2");
    WbFieldRef trans_field = wb_supervisor_node_get_field(robot_node, "translation");
    // WbFieldRef rot_field = wb_supervisor_node_get_field(robot_node, "rotation");
    if (robot_node == NULL) {
      fprintf(stderr, "No DEF MY_ROBOT node found in the current world file\n");
      exit(1);
    }
    
    // Set the initial position of the robot using the supervisor
    // wb_supervisor_field_set_sf_vec3f(trans_field, startingTranslation[0]);
    // wb_supervisor_field_set_sf_rotation(rot_field, startingRotation[0]);
    
    printf("Morris Water Maze using Nex Fire Bird 6 robot\n");
    
    int i,k;
    int map[PLATFORM_X][PLATFORM_Z];
    for(i=0;i<PLATFORM_X;i++){
        for(k=0;k<PLATFORM_Z;k++){
            map[i][k]=0;
        }
    }
    
    FILE *fmap;
    
    while ((wb_robot_step(TIME_STEP) != -1)) {
        
        const double *trans_value = wb_supervisor_field_get_sf_vec3f(trans_field);
        printf("%d,%d\n",find_index(trans_value[0]),find_index(trans_value[2]));
        map[find_index(trans_value[0])][find_index(trans_value[2])] = 1;
        
    };
    // fclose(fpLatency);
        
    fmap = fopen("map.txt", "w");
    for (int i = 0; i < PLATFORM_X; i++) {
        for (int k = 0; k < PLATFORM_Z; k++) {
            fprintf(fmap,"%d\t", map[i][k]);
        }
        fprintf(fmap, "\n");
    }
    fclose(fmap);
  
    wb_robot_cleanup();
    
    return 0;
}
