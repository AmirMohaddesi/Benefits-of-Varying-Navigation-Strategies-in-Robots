/*
 * Copyright 1996-2020 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <webots/camera.h>
#include <webots/device.h>
#include <webots/inertial_unit.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>
#include <webots/touch_sensor.h>
#include <webots/lidar.h>
#include <webots/supervisor.h>

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <vector>

using namespace std;

#define TIME_STEP 16
#define BASE_SPEED 2.5
#define LIDAR_THRESHOLD 0.8

#define MAX_WHEEL_SPEED 3.0        // maximum velocity for the wheels [rad / s]
#define WHEELS_DISTANCE 0.4492     // distance between 2 caster wheels (the four wheels are located in square) [m]
#define SUB_WHEELS_DISTANCE 0.098  // distance between 2 sub wheels of a caster wheel [m]
#define WHEEL_RADIUS 0.08          // wheel radius

// function to check if a double is almost equal to another
#define TOLERANCE 0.05  // arbitrary value
#define ALMOST_EQUAL(a, b) ((a < b + TOLERANCE) && (a > b - TOLERANCE))

// gaussian function
double gaussian(double x, double mu, double sigma) {
  return (1.0 / (sigma * sqrt(2.0 * M_PI))) * exp(-((x - mu) * (x - mu)) / (2 * sigma * sigma));
}

enum { FLL_WHEEL, FLR_WHEEL, FRL_WHEEL, FRR_WHEEL, BLL_WHEEL, BLR_WHEEL, BRL_WHEEL, BRR_WHEEL };
enum { FL_ROTATION, FR_ROTATION, BL_ROTATION, BR_ROTATION };

static WbDeviceTag wheel_motors[8];
static WbDeviceTag wheel_sensors[8];
static WbDeviceTag rotation_motors[4];
static WbDeviceTag rotation_sensors[4];
static WbDeviceTag laser_tilt;

// Simpler step function
static void step() {
  if (wb_robot_step(TIME_STEP) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

// Retrieve all the pointers to the PR2 devices
static void initialize_devices() {
  int i;
  wheel_motors[FLL_WHEEL] = wb_robot_get_device("fl_caster_l_wheel_joint");
  wheel_motors[FLR_WHEEL] = wb_robot_get_device("fl_caster_r_wheel_joint");
  wheel_motors[FRL_WHEEL] = wb_robot_get_device("fr_caster_l_wheel_joint");
  wheel_motors[FRR_WHEEL] = wb_robot_get_device("fr_caster_r_wheel_joint");
  wheel_motors[BLL_WHEEL] = wb_robot_get_device("bl_caster_l_wheel_joint");
  wheel_motors[BLR_WHEEL] = wb_robot_get_device("bl_caster_r_wheel_joint");
  wheel_motors[BRL_WHEEL] = wb_robot_get_device("br_caster_l_wheel_joint");
  wheel_motors[BRR_WHEEL] = wb_robot_get_device("br_caster_r_wheel_joint");
  for (i = FLL_WHEEL; i <= BRR_WHEEL; ++i)
    wheel_sensors[i] = wb_motor_get_position_sensor(wheel_motors[i]);

  rotation_motors[FL_ROTATION] = wb_robot_get_device("fl_caster_rotation_joint");
  rotation_motors[FR_ROTATION] = wb_robot_get_device("fr_caster_rotation_joint");
  rotation_motors[BL_ROTATION] = wb_robot_get_device("bl_caster_rotation_joint");
  rotation_motors[BR_ROTATION] = wb_robot_get_device("br_caster_rotation_joint");
  for (i = FL_ROTATION; i <= BR_ROTATION; ++i)
    rotation_sensors[i] = wb_motor_get_position_sensor(rotation_motors[i]);

  laser_tilt = wb_robot_get_device("laser_tilt");
  //base_laser = wb_robot_get_device("base_laser");
}

// enable the robot devices
static void enable_devices() {
  int i = 0;
  for (i = 0; i < 8; ++i) {
    wb_position_sensor_enable(wheel_sensors[i], TIME_STEP);
    // init the motors for speed control
    wb_motor_set_position(wheel_motors[i], INFINITY);
    wb_motor_set_velocity(wheel_motors[i], 0.0);
  }

  for (i = 0; i < 4; ++i)
    wb_position_sensor_enable(rotation_sensors[i], TIME_STEP);

  wb_lidar_enable(laser_tilt, TIME_STEP);
  //wb_lidar_enable(base_laser, TIME_STEP);
}

// set the speeds of the robot wheels
static void set_wheels_speeds(double fll, double flr, double frl, double frr, double bll, double blr, double brl, double brr) {
  wb_motor_set_velocity(wheel_motors[FLL_WHEEL], fll);
  wb_motor_set_velocity(wheel_motors[FLR_WHEEL], flr);
  wb_motor_set_velocity(wheel_motors[FRL_WHEEL], frl);
  wb_motor_set_velocity(wheel_motors[FRR_WHEEL], frr);
  wb_motor_set_velocity(wheel_motors[BLL_WHEEL], bll);
  wb_motor_set_velocity(wheel_motors[BLR_WHEEL], blr);
  wb_motor_set_velocity(wheel_motors[BRL_WHEEL], brl);
  wb_motor_set_velocity(wheel_motors[BRR_WHEEL], brr);
}

static void set_wheels_speed(double speed) {
  set_wheels_speeds(speed, speed, speed, speed, speed, speed, speed, speed);
}

static void stop_wheels() {
  set_wheels_speeds(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}

// Set the rotation wheels angles.
// If wait_on_feedback is true, the function is left when the rotational motors have reached their target positions.
static void set_rotation_wheels_angles(double fl, double fr, double bl, double br, bool wait_on_feedback) {
  if (wait_on_feedback) {
    stop_wheels();
  }

  wb_motor_set_position(rotation_motors[FL_ROTATION], fl);
  wb_motor_set_position(rotation_motors[FR_ROTATION], fr);
  wb_motor_set_position(rotation_motors[BL_ROTATION], bl);
  wb_motor_set_position(rotation_motors[BR_ROTATION], br);

  if (wait_on_feedback) {
    double target[4] = {fl, fr, bl, br};

    while (true) {
      bool all_reached = true;
      int i;
      for (i = 0; i < 4; ++i) {
        double current_position = wb_position_sensor_get_value(rotation_sensors[i]);
        if (!ALMOST_EQUAL(current_position, target[i])) {
          all_reached = false;
          break;
        }
      }

      if (all_reached)
        break;
      else
        step();
    }

  }
}

// High level function to rotate the robot around itself of a given angle [rad]
// Note: the angle can be negative
static void robot_rotate(double angle) {
  stop_wheels();

  set_rotation_wheels_angles(3.0 * M_PI_4, M_PI_4, -3.0 * M_PI_4, -M_PI_4, true);
  const double max_wheel_speed = angle > 0 ? MAX_WHEEL_SPEED : -MAX_WHEEL_SPEED;
  set_wheels_speed(max_wheel_speed);

  double initial_wheel0_position = wb_position_sensor_get_value(wheel_sensors[FLL_WHEEL]);
  // expected travel distance done by the wheel
  double expected_travel_distance = fabs(angle * 0.5 * (WHEELS_DISTANCE + SUB_WHEELS_DISTANCE));

  while (true) {
    double wheel0_position = wb_position_sensor_get_value(wheel_sensors[FLL_WHEEL]);
    // travel distance done by the wheel
    double wheel0_travel_distance = fabs(WHEEL_RADIUS * (wheel0_position - initial_wheel0_position));

    if (wheel0_travel_distance > expected_travel_distance)
      break;

    // reduce the speed before reaching the target
    if (expected_travel_distance - wheel0_travel_distance < 0.025)
      set_wheels_speed(0.1 * max_wheel_speed);

    step();
  }

  // reset wheels
  set_rotation_wheels_angles(0.0, 0.0, 0.0, 0.0, true);
  stop_wheels();
}

// High level function to go forward for a given distance [m]
// Note: the distance can be negative
static void robot_go_forward(double distance) {
  double max_wheel_speed = distance > 0 ? MAX_WHEEL_SPEED : -MAX_WHEEL_SPEED;
  set_wheels_speed(max_wheel_speed);

  double initial_wheel0_position = wb_position_sensor_get_value(wheel_sensors[FLL_WHEEL]);

  while (true) {
    double wheel0_position = wb_position_sensor_get_value(wheel_sensors[FLL_WHEEL]);
    // travel distance done by the wheel
    double wheel0_travel_distance = fabs(WHEEL_RADIUS * (wheel0_position - initial_wheel0_position));

    if (wheel0_travel_distance > fabs(distance))
      break;

    // reduce the speed before reaching the target
    if (fabs(distance) - wheel0_travel_distance < 0.025)
      set_wheels_speed(0.1 * max_wheel_speed);

    step();
  }

  stop_wheels();
}

// //Convenient initial position
// static void set_initial_position() {
  // set_left_arm_position(0.0, 1.35, 0.0, -2.2, 0.0, false);
  // set_right_arm_position(0.0, 1.35, 0.0, -2.2, 0.0, false);

  // set_gripper(false, true, 0.0, false);
  // set_gripper(true, true, 0.0, false);

  // set_torso_height(0.2, true);
// }

int find_index(double x){
    if(floor(2*x+32)<0)
      return 0;
    if(floor(2*x+32)>63)
      return 63;
    return floor(2*x+32);
} 

int** find(float** v, int spike,int sizeX, int sizeY, int gle){
    int i,j;
    int *fExcX;
    int *fExcY;
    int count=0;
    for(i=0;i<sizeX;i++){
        for(j=0;j<sizeY;j++){ 
            if(gle == 1 && v[i][j]>=spike){
                count++;
            if(gle == 0 && v[i][j]==spike){
                count++;
            if(gle == -1 && v[i][j]<=spike){
                count++;
        }
    }
    fExcX = (int*) malloc((count+1)*sizeof(int));
    fExcY = (int*) malloc((count+1)*sizeof(int));
    fExcX[count] = -1;
    fExcY[count] = -1;
    count=0;
    for(i=0;i<sizeX;i++){
        for(j=0;j<sizeY;j++){
            if(gle == 1 && v[i][j]>=spike){
                fExcX[count]=i;
                fExcY[count]=j;
                count++;
            if(gle == 0 && v[i][j]==spike){
                fExcX[count]=i;
                fExcY[count]=j;
                count++;
            if(gle == -1 && v[i][j]<=spike){
                fExcX[count]=i;
                fExcY[count]=j;
                count++;
                
            }
        }
    }
    int** fExc;
    fExc = (int**) malloc(2*sizeof(int*));
    fExc[0]=fExcX;
    fExc[1]=fExcX;
    return fExc;
}

int** find4D(float**** v, int k, int l ,int spike,int sizeX, int sizeY, int gle){
    int i,j;
    int *fExcX;
    int *fExcY;
    int count=0;
    for(i=0;i<sizeX;i++){
        for(j=0;j<sizeY;j++){ 
            if(gle == 1 && v[i][j][k][l]>=spike){
                count++;
            if(gle == 0 && v[i][j][k][l]==spike){
                count++;
            if(gle == -1 && v[i][j][k][l]<=spike){
                count++;
        }
    }
    fExcX = (int*) malloc((count+1)*sizeof(int));
    fExcY = (int*) malloc((count+1)*sizeof(int));
    fExcX[count] = -1;
    fExcY[count] = -1;
    count=0;
    for(i=0;i<sizeX;i++){
        for(j=0;j<sizeY;j++){
            if(gle == 1 && v[i][j][k][l]>=spike){
                fExcX[count]=i;
                fExcY[count]=j;
                count++;
            if(gle == 0 && v[i][j][k][l]==spike){
                fExcX[count]=i;
                fExcY[count]=j;
                count++;
            if(gle == -1 && v[i][j][k][l]<=spike){
                fExcX[count]=i;
                fExcY[count]=j;
                count++;
            }
        }
    }
    int** fExc;
    fExc = (int**) malloc(2*sizeof(int*));
    fExc[0]=fExcX;
    fExc[1]=fExcX;
    return fExc;
}

void spikeWave(int xStart, int zStart, int xGoal,int zGoal, int** map, const int xMapSize, const int zMapSize){

    const int Ne1 = xMapSize;
    const int Ne2 = zMapSize;
    int SPIKE = 1;
    int REFRACTORY = -5;
    int W_INIT = 5;
    float LEARNING_RATE = 1.0;
    
    // Each neuron connects to its 8 neighbors
    float wgt[Ne1][Ne2][Ne1][Ne2];
    for(int i = 0; i<Ne1; i++){
        for(int j = 0; j<Ne2; j++){
            for(int m = -1;m<=1;m++){
                for(int m = -1;m<=1;m++){
    //                 if i+m > 0 && i+m <= size(wgt,1) && j+n > 0 && j+n <= size(wgt,2) && (m ~= 0 || n ~= 0)
                    if (m == 0 || n == 0) && m != n && i+m >= 0 && i+m < Ne1 && j+n >= 0 && j+n < Ne2
                        wgt[i][j][i+m][j+n] = W_INIT;
                }
            }
        }
    }

    int delayBuffer[Ne1][Ne2][Ne1][Ne2] = {0};
    float v[Ne1][Ne2]={0};  // Initial values of v voltage
    float u[Ne1][Ne2]={0};  // Initial values of u recovery

    // the spike wave is initiated from the starting location
    v[startX][startY] = SPIKE;
    
    bool foundGoal = false;
    int timeSteps = 0;
    vector<vector<int>> aer;
    
    while (~foundGoal){
        timeSteps = timeSteps + 1;
        int ** fExc = find(v , SPIKE, Ne1, Ne2, 1);// indices of spikes
        int* fExcX=fExc[0];
        int* fExcY=fExc[1];
        for(int i=0;fExcX[i]!=-1;i++){
            vector<int> temp;
            temp.push_back(timeSteps);
            temp.push_back(fExcX[i]);
            temp.push_back(fExcY[i]);
            aer.push_back(temp); // keep spike information in an addressable event representation (spikeID and timeStep)
        }
        
        // Neurons that spike send their spike to their post-synaptic targets.
        // The weights are updated and the spike goes in a delay buffer to
        // targets. The neuron's recovery variable is set to its refractory value.
        if (fExcX[0] != -1){
            for (int i = 0; fExcX[i] != -1; i++){
                u[fExcX[i]][fExcY[i]] = REFRACTORY;
                delayRule(&wgt,fExcX[i],fExcY[i],Ne1,Ne2,map[fExcX[i]][fExcY[i]], LEARNING_RATE);
                for (int j=0, j<Ne2, j++){
                    for (int k=0, k<Ne1, j++){
                        delayBuffer[fExcX[i]][fExcY[i]][k][j] = round(wgt[fExcX[i]][fExcY[i]][k][j]);
                    }
                }
                if (fExcX[i] == endX && fExcY[i] == endY)
                    foundGoal = true;   // neuron at goal location spiked.
            }
        }
        free(fExcX)
        free(fExcY)
        free(fExc)
        float Iexc[Ne1][Ne2];
        // if the spike wave is still propagating, get the synaptic input for
        // all neurons. Synaptic input is based on recovery variable, and spikes
        // that are arriving to the neuron at this time step.
        if (~foundGoal){
            for (int j=0, j<Ne2, j++){
                for (int k=0, k<Ne1, j++){
                    Iexc[k][j] = u[k][j];
                }
            }
            for (int i=0, i<Ne1, j++){
                for (int j=0, j<Ne2, j++){
                    fExc=find4D(delayBuffer,i,j, 1, Ne1, Ne2, 0);
                    fExcX=fExc[0];
                    fExcY=fExc[1];
                    if (fExcX[0]==-1){//is not empty
                        for(int k=0; fExcX[k]!=-1; k++){
                            Iexc[i][j] = Iexc[i][j] + (wgt[fExcX[k]][fExcY[k]][i][j] > 0);
                        }
                    }
                }
            }
            // Update membrane potential (v) and recovery variable (u)
            for (int j=0, j<Ne2, j++){
                for (int k=0, k<Ne1, j++){
                    v[k][j] = v[k][j] + Iexc[k][j];
                }
            }
            for (int j=0, j<Ne2, j++){
                for (int k=0, k<Ne1, j++){
                    u[k][j] = min(u[k][j] + 1, 0);
                }
            }
        }
        
    // %     if dispWave
    // %         inx = find(v >= SPIKE);
    // %         dispMap = map;
    // %         dispMap(inx) = 20;
    // %         imagesc(dispMap);
    // %         axis square;
    // %         axis off;
    // %         title(['S(', num2str(startX), ',', num2str(startY), ') E(', num2str(endX), ',', num2str(endY), ')'])
    // %         drawnow
    // %     end
        for (int i=0, i<Ne1, j++){
            for (int j=0, j<Ne2, j++){
                for (int k=0, k<Ne1, k++){
                    for (int l=0, l<Ne2, l++){
                        delayBuffer[i][j][k][l] = max(0, delayBuffer[i][j][k][l] - 1);  // Update the delays of the scheduled spikes.
                    }
                }
            }
        }
    
        free(fExcX)
        free(fExcY)
        free(fExc)
    
    }


path = getPath(aer, map, [startX,startY], [endX,endY]); % Get the path from the AER table. //todo

// if dispWave
    // pathLen = size(path,2);
    // map(path(1).x,path(1).y) = 75;
    // for i = 2:pathLen
        // map(path(i).x,path(i).y) = 20;
    // end
    // map(path(end).x,path(end).y) = 50;
    // imagesc(map);
    // axis square;
    // axis off;
    // title(['Survey S(', num2str(startX), ',', num2str(startY), ') E(', num2str(endX), ',', num2str(endY), ')'])
// end

}


// delayRule - Calculates a delta function for the weights. The weights hold
//   the axonal delay between neurons.
//
// @param wBefore - weight value before applying learning rule.
// @param value - value from the map.
// @param learnRate - learning rate.
// @return - weight value after applying learning rule.
void delayRule(float**** wBefore, int fExcXi , int fExcYi , int Ne1, int Ne2, int value, float learnRate){

    float valMat[Ne1][Ne2];
    for(int i=0; i < Ne1; i++){
        for(int j=0; j <Ne2; j++){
            valMat[i][j] = learnRate * (value - wBefore[fExcXi][fExcYi][i][j]);
            wBefore[fExcXi][fExcYi][i][j] = wBefore[fExcXi][fExcYi][i][j] + (wBefore[fExcXi][fExcYi][i][j] > 0) * valMat[i][j];
        }
    }
    return;
}

//  getPath - Generates the path based on the AER spike table. The spike
//    table is ordered from earliest spike to latest. The algorithm starts at
//    the end of the table and finds the most recent spike from a neighbor.
//    This generates a near shortest path from start to end.
//  
//  @param spks - AER  table containing the spike time and ID of each neuron.
//  @param map - map of the environment.
//  @param s - start location.
//  @param e - end (goal) location.
//  @return - path from start to end.
int** getPath(vector < vector < int > > spks, int** map, int sx, int sy, int ex, int ey ){

    // global START;

    int pinx = 0;
    int pathx[1000] //some arbitrary value that we assume the path lenght wouldn't exceed it
    int pathy[1000]

    pathx[pinx] = ex;
    pathy[pinx] = ey;

    while (sqrt(pow(pathx[pinx]-sx,2)+pow(pathy[pinx]-sy,2) > 1.0){
        pinx = 0;
        pathx[pinx] = ex;
        pathy[pinx] = ey;
    
        // work from most recent to oldest in the spike table
        for (vector<vector <int> >::reverse_iterator it = my_vector.rbegin(); it != my_vector.rend(); ++it ){
            int i = *it[0];
            vector<vector <int> > inx;
            while(*it[0] == i){
                inx.push_back(*it);
                it++;
            }
            bool found = false;
            int k = 0;
            int lst[20][2];
            // find the last spike from a neighboring neuron.
            for(int j=0;j<inx.size();j++){
                if(sqrt(pow(pathx[pinx]-inx[j][1],2)+pow(pathy[pinx]-inx[j][2],2)) < 1.5){
                    k = k + 1;
                    lst[k][0] = inx[j][1];
                    lst[k][1] = inx[j][2];
                    found = true;
                }
            }
            
            // if there is more then one spike, find the one with the lowest
            // cost and closest to starting location.
            
            if(found){
                int minx;
                cost = INT_MAX;
                for(int m =0; m<k; m++){
                    if (map[lst[m][0]][lst[m][1]] < cost){
                        cost = map[lst[m][0]][lst[m][1]];
                        minx = m;
                        dist = sqrt(pow((xStart - lst[m][0]),2)+pow((yStart-lst[m][1]),2));
                    }
                    else if (map[lst[m][0]][lst[m][1]] == cost && sqrt(pow((xStart - lst[m][0]),2)+pow((yStart-lst[m][1]),2)) < dist){
                        minx = m;
                        dist = sqrt(pow((xStart - lst[m][0]),2)+pow((yStart-lst[m][1]),2));
                    }
                }
                
                // add the neuron to the path.
                pinx++;
                pathx(pinx) = lst[minx][0];
                pathy(pinx) = lst[minx][1];
            }
        }
    }
}



int main(int argc, char **argv) {
    wb_robot_init();
    
    initialize_devices();
    enable_devices();
    //set_initial_position();
  
    // //go to the initial position
    // set_left_arm_position(0.0, 0.5, 0.0, -0.5, 0.0, true);
    // set_right_arm_position(0.0, 0.5, 0.0, -0.5, 0.0, true);
    
    wb_lidar_enable_point_cloud(laser_tilt);
  
    const int lidar_width = wb_lidar_get_horizontal_resolution(laser_tilt);
    const double lidar_max_range = wb_lidar_get_max_range(laser_tilt);
    
    WbNodeRef robot_node = wb_supervisor_node_get_from_def("PR2");
    if (robot_node == NULL) {
        fprintf(stderr, "No DEF MY_ROBOT node found in the current world file\n");
        exit(1);
    }
    WbFieldRef trans_field = wb_supervisor_node_get_field(robot_node, "translation");
    const double *trans_value = wb_supervisor_field_get_sf_vec3f(trans_field);
    
    int xStart = find_index(trans_value[0]);
    int zStart = find_index(trans_value[2]);
    printf("%d %d, source index\n", xSrcIdx, zSrcIdx);
    // init braitenberg coefficient
    double *braitenberg_coefficients = (double *)malloc(sizeof(double) * lidar_width);
    int i;
    for (i = 0; i < lidar_width; i++){
        braitenberg_coefficients[i] = 6 * gaussian(i, lidar_width / 4, lidar_width / 12);
        // printf("bc: %g\n", braitenberg_coefficients[i]);
    }
    int xGoal=0;
    int zGoal=0;
    while (wb_robot_step(TIME_STEP) != -1) {
        bool obstacle = false;
        double left_speed =  MAX_WHEEL_SPEED, right_speed =  MAX_WHEEL_SPEED;
        
        // get lidar values
        const float *lidar_values = wb_lidar_get_range_image(laser_tilt);
        
        // apply the braitenberg coefficients on the resulted values of the lidar
        for (i = 0.25 * lidar_width; i < 0.5 * lidar_width; i++) {
            const int j = lidar_width - i - 1;
            const int k = i - 0.25 * lidar_width;
            float lidari = lidar_values[i];
            float lidarj = lidar_values[j];
            if (isinf(lidar_values[i]))
                lidari = lidar_max_range;
            if (isinf(lidar_values[j]))
                lidarj = lidar_max_range;
            left_speed +=
              braitenberg_coefficients[k] * ((1.0 - lidari / lidar_max_range) - (1.0 - lidarj / lidar_max_range));
            // printf("bc[%d]: %g, Lidar valuesi[%d]: %g, Lidar valuesj[%d]: %g\n", k, braitenberg_coefficients[k],i,lidar_values[i],j,lidar_values[j]);
            
            right_speed +=
              braitenberg_coefficients[k] * ((1.0 - lidarj / lidar_max_range) - (1.0 - lidari / lidar_max_range));
            // printf("bc[%d]: %g, Lidar valuesj[%d]: %g, Lidar valuesi[%d]: %g\n", k, braitenberg_coefficients[k],i,lidar_values[i],j,lidar_values[j]);
          
        }
        if(lidar_values[(int)floor(0.5 * lidar_width)]<lidar_max_range/5)
            obstacle = true;
        if (obstacle)
            robot_rotate(M_PI/16);
        else
            if(left_speed < right_speed - LIDAR_THRESHOLD)
                robot_rotate(M_PI/32);
            else if(right_speed < left_speed - LIDAR_THRESHOLD)
                robot_rotate(-M_PI/32);
            else
                robot_go_forward(1);
        // //apply computed velocities
        printf("left speed: %g\n", left_speed);
        printf("right speed: %g\n", right_speed);
        printf("%g\n",lidar_values[(int)floor(0.5 * lidar_width)]);
      
    };
  
    free(braitenberg_coefficients);
    wb_robot_cleanup();
  
    return 0;
}
