/********************************************************************************
* Copyright (c) 2021 Seyed Amirhosein Mohaddesi
* Email: smohadde@uci.edu
* 
* Permission is hereby granted, free of charge, to any person obtaining a copy of
* this software and associated documentation files (the "Software"), to deal in
* the Software without restriction, including without limitation the rights to
* use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
* of the Software, and to permit persons to whom the Software is furnished to do
* so, subject to the following conditions:
* 
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
********************************************************************************/

#include <webots/Camera.hpp>
#include <webots/Device.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>
#include <webots/TouchSensor.hpp> 
#include <webots/Lidar.hpp>
#include <webots/Supervisor.hpp>


#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <sstream>
#include <time.h>
#include <algorithm>
// #include <cfloat>
// #include <cmath>

using namespace webots;
using namespace std;

#define TIME_STEP 32
#define BASE_SPEED 2.5
#define LIDAR_THRESHOLD 2
#define DELAY 4.0

#define MAX_WHEEL_SPEED 3.0        // maximum velocity for the wheels [rad / s]
#define WHEELS_DISTANCE 0.4492     // distance between 2 caster wheels (the four wheels are located in square) [m]
#define SUB_WHEELS_DISTANCE 0.098  // distance between 2 sub wheels of a caster wheel [m]
#define WHEEL_RADIUS 0.08          // wheel radius

#define PLATFORM_X 64
#define PLATFORM_Z 64

#define N_SOURCES 11
#define N_GOALS 14

// function to check if a double is almost equal to another
#define TOLERANCE 0.05  // arbitrary value
#define ALMOST_EQUAL(a, b) ((a < b + TOLERANCE) && (a > b - TOLERANCE))

void delayRule(vector< vector < vector < vector <float> > > > &wgt, int fExcXi , int fExcYi , int Ne1, int Ne2, int value, float learnRate);
bool getPath(vector < vector < int > > spks, int** map, int sx, int sy, int ex, int ey );
void calculateDirection();
static void robotGoForward(Supervisor * robot, double distance);
static void robotRotate(Supervisor* robot, double angle);
// gaussian function
double gaussian(double x, double mu, double sigma) {
  return (1.0 / (sigma * sqrt(2.0 * M_PI))) * exp(-((x - mu) * (x - mu)) / (2 * sigma * sigma));
}

enum { FLL_WHEEL, FLR_WHEEL, FRL_WHEEL, FRR_WHEEL, BLL_WHEEL, BLR_WHEEL, BRL_WHEEL, BRR_WHEEL };
enum { FL_ROTATION, FR_ROTATION, BL_ROTATION, BR_ROTATION };
enum { SHOULDER_ROLL, SHOULDER_LIFT, UPPER_ARM_ROLL, ELBOW_LIFT, WRIST_ROLL };
enum { NORTH, NORTHWEST, WEST, SOUTHWEST, SOUTH, SOUTHEAST, EAST, NORTHEAST};

Motor * wheelMotors[8];
PositionSensor * wheelSensors[8];
Motor * rotationMotors[4];
PositionSensor * rotationSensors[4];
Motor * leftArmMotors[5];
PositionSensor * leftArmSensors[5];
Motor * rightArmMotors[5];
PositionSensor * rightArmSensors[5];
Lidar * laserTilt;
Lidar * baseLaser;
InertialUnit * imuSensor;

Supervisor * robot;
Node ** robotNodes;
Field ** transFields;
Field ** customDataFields;
const double **transValues;
string thisNode;

int X_GOAL[N_GOALS];
int Z_GOAL[N_GOALS];
int X_SOURCE[N_GOALS];
int Z_SOURCE[N_GOALS];
int xGoal;
int zGoal;
int path[2][1000];//some arbitrary value that we assume the path lenght wouldn't exceed it
int *xCur;
int *zCur;
int xBlock;
int zBlock;
int cGoal;
int goalList[2000];
double headDirection = NORTH;
double yaw;
double *braitenbergCoefficients;
bool obstacle = false;
bool startingState = false;
int lidarWidth;
double lidarMaxRange;
double leftSpeed; 
double rightSpeed;
int pathTurnsIdx[2][1000];
int endIdx;
int numberOfRobots;
double* distances;
int argTRRef;
int argCFRobot = -1; //Arg number of the Robot in conflict with this robot
bool foundCollision;
// bool visitedGoals[N_GOALS] = { false };
int areaOccupancy[PLATFORM_X][PLATFORM_Z] = { 0 };
int nRobotObs = 0;    //Number of Robot obstacles
int nWallObs = 0;     //Number of Wall obstacles
int robotNumbers[5];
// Simpler step function
static void step(Robot* robot) {
  if (robot->step(TIME_STEP) == -1) {
    exit(EXIT_SUCCESS);
  }
}

// Retrieve all the pointers to the PR2 devices
static void initializeDevices(Robot * robot) {
  wheelMotors[FLL_WHEEL] = robot->getMotor("fl_caster_l_wheel_joint");
  wheelMotors[FLR_WHEEL] = robot->getMotor("fl_caster_r_wheel_joint");
  wheelMotors[FRL_WHEEL] = robot->getMotor("fr_caster_l_wheel_joint");
  wheelMotors[FRR_WHEEL] = robot->getMotor("fr_caster_r_wheel_joint");
  wheelMotors[BLL_WHEEL] = robot->getMotor("bl_caster_l_wheel_joint");
  wheelMotors[BLR_WHEEL] = robot->getMotor("bl_caster_r_wheel_joint");
  wheelMotors[BRL_WHEEL] = robot->getMotor("br_caster_l_wheel_joint");
  wheelMotors[BRR_WHEEL] = robot->getMotor("br_caster_r_wheel_joint");
  for (int i = FLL_WHEEL; i <= BRR_WHEEL; ++i)
    wheelSensors[i] = wheelMotors[i]->getPositionSensor();

  rotationMotors[FL_ROTATION] = robot->getMotor("fl_caster_rotation_joint");
  rotationMotors[FR_ROTATION] = robot->getMotor("fr_caster_rotation_joint");
  rotationMotors[BL_ROTATION] = robot->getMotor("bl_caster_rotation_joint");
  rotationMotors[BR_ROTATION] = robot->getMotor("br_caster_rotation_joint");
  for (int i = FL_ROTATION; i <= BR_ROTATION; ++i)
    rotationSensors[i] = rotationMotors[i]->getPositionSensor();

  leftArmMotors[SHOULDER_ROLL] = robot->getMotor("l_shoulder_pan_joint");
  leftArmMotors[SHOULDER_LIFT] = robot->getMotor("l_shoulder_lift_joint");
  leftArmMotors[UPPER_ARM_ROLL] = robot->getMotor("l_upper_arm_roll_joint");
  leftArmMotors[ELBOW_LIFT] = robot->getMotor("l_elbow_flex_joint");
  leftArmMotors[WRIST_ROLL] = robot->getMotor("l_wrist_roll_joint");
  for (int i = SHOULDER_ROLL; i <= WRIST_ROLL; ++i)
    leftArmSensors[i] = leftArmMotors[i]->getPositionSensor();

  rightArmMotors[SHOULDER_ROLL] = robot->getMotor("r_shoulder_pan_joint");
  rightArmMotors[SHOULDER_LIFT] = robot->getMotor("r_shoulder_lift_joint");
  rightArmMotors[UPPER_ARM_ROLL] = robot->getMotor("r_upper_arm_roll_joint");
  rightArmMotors[ELBOW_LIFT] = robot->getMotor("r_elbow_flex_joint");
  rightArmMotors[WRIST_ROLL] = robot->getMotor("r_wrist_roll_joint");
  for (int i = SHOULDER_ROLL; i <= WRIST_ROLL; ++i)
    rightArmSensors[i] = rightArmMotors[i]->getPositionSensor();

  laserTilt = robot->getLidar("laser_tilt");  //todo
  baseLaser = robot->getLidar("base_laser");
  
  imuSensor = robot->getInertialUnit("imu_sensor");
}

// enable the robot devices
static void enableDevices() {
  for (int i = 0; i < 8; ++i) {
    wheelSensors[i]->enable(TIME_STEP);
    // init the motors for speed control
    wheelMotors[i]->setPosition(INFINITY);
    wheelMotors[i]->setVelocity(0.0);
  }

  for (int i = 0; i < 4; ++i)
    rotationSensors[i]->enable(TIME_STEP);
  
  for (int i = 0; i < 5; ++i) {
      leftArmSensors[i]->enable(TIME_STEP);
      rightArmSensors[i]->enable(TIME_STEP);
  }
  
  laserTilt->enable(TIME_STEP);
  baseLaser->enable(TIME_STEP);
  
  imuSensor->enable(TIME_STEP);
}

// set the speeds of the robot wheels
static void setWheelsSpeeds(double fll, double flr, double frl, double frr, double bll, double blr, double brl, double brr) {
  wheelMotors[FLL_WHEEL]->setVelocity(fll);
  wheelMotors[FLR_WHEEL]->setVelocity(flr);
  wheelMotors[FRL_WHEEL]->setVelocity(frl);
  wheelMotors[FRR_WHEEL]->setVelocity(frr);
  wheelMotors[BLL_WHEEL]->setVelocity(bll);
  wheelMotors[BLR_WHEEL]->setVelocity(blr);
  wheelMotors[BRL_WHEEL]->setVelocity(brl);
  wheelMotors[BRR_WHEEL]->setVelocity(brr);
}

static void setWheelsSpeed(double speed) {
    setWheelsSpeeds(speed, speed, speed, speed, speed, speed, speed, speed);
}

static void stopWheels() {
    setWheelsSpeeds(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}

// Set the rotation wheels angles.
// If wait_on_feedback is true, the function is left when the rotational motors have reached their target positions.
static void setRotationWheelsAngles(Robot* robot,double fl, double fr, double bl, double br, bool waitOnFeedback) {
    if (waitOnFeedback) {
        stopWheels();
    }
  
    rotationMotors[FL_ROTATION]->setPosition(fl);
    rotationMotors[FR_ROTATION]->setPosition(fr);
    rotationMotors[BL_ROTATION]->setPosition(bl);
    rotationMotors[BR_ROTATION]->setPosition(br);
  
    if (waitOnFeedback) {
        double target[4] = {fl, fr, bl, br};
    
        while (true) {
            bool allReached = true;
            
            for (int i = 0; i < 4; ++i) {
                double currentPosition = rotationSensors[i]->getValue();
                if (!ALMOST_EQUAL(currentPosition, target[i])) {
                  allReached = false;
                  break;
                }
            }
      
            if (allReached)
                break;
            else
                step(robot);
        }
  
    }
}



// Set the right arm position (forward kinematics)
// If wait_on_feedback is enabled, the function is left when the target is reached.
static void setRightArmPosition(double shoulder_roll, double shoulder_lift, double upper_arm_roll, double elbow_lift,
                                   double wrist_roll, bool wait_on_feedback) {
  rightArmMotors[SHOULDER_ROLL]->setPosition(shoulder_roll);
  rightArmMotors[SHOULDER_LIFT]->setPosition(shoulder_lift);
  rightArmMotors[UPPER_ARM_ROLL]->setPosition(upper_arm_roll);
  rightArmMotors[ELBOW_LIFT]->setPosition(elbow_lift);
  rightArmMotors[WRIST_ROLL]->setPosition(wrist_roll);

  // if (wait_on_feedback) {
    // while (!ALMOST_EQUAL(wb_position_sensor_get_value(right_arm_sensors[SHOULDER_ROLL]), shoulder_roll) ||
           // !ALMOST_EQUAL(wb_position_sensor_get_value(right_arm_sensors[SHOULDER_LIFT]), shoulder_lift) ||
           // !ALMOST_EQUAL(wb_position_sensor_get_value(right_arm_sensors[UPPER_ARM_ROLL]), upper_arm_roll) ||
           // !ALMOST_EQUAL(wb_position_sensor_get_value(right_arm_sensors[ELBOW_LIFT]), elbow_lift) ||
           // !ALMOST_EQUAL(wb_position_sensor_get_value(right_arm_sensors[WRIST_ROLL]), wrist_roll)) {
      // step();
    // }
  // }
}

// Idem for the left arm
static void setLeftArmPosition(double shoulder_roll, double shoulder_lift, double upper_arm_roll, double elbow_lift,
                                  double wrist_roll, bool wait_on_feedback) {
  leftArmMotors[SHOULDER_ROLL]->setPosition(shoulder_roll);
  leftArmMotors[SHOULDER_LIFT]->setPosition(shoulder_lift);
  leftArmMotors[UPPER_ARM_ROLL]->setPosition(upper_arm_roll);
  leftArmMotors[ELBOW_LIFT]->setPosition(elbow_lift);
  leftArmMotors[WRIST_ROLL]->setPosition(wrist_roll);

  // if (wait_on_feedback) {
    // while (!ALMOST_EQUAL(wb_position_sensor_get_value(left_arm_sensors[SHOULDER_ROLL]), shoulder_roll) ||
           // !ALMOST_EQUAL(wb_position_sensor_get_value(left_arm_sensors[SHOULDER_LIFT]), shoulder_lift) ||
           // !ALMOST_EQUAL(wb_position_sensor_get_value(left_arm_sensors[UPPER_ARM_ROLL]), upper_arm_roll) ||
           // !ALMOST_EQUAL(wb_position_sensor_get_value(left_arm_sensors[ELBOW_LIFT]), elbow_lift) ||
           // !ALMOST_EQUAL(wb_position_sensor_get_value(left_arm_sensors[WRIST_ROLL]), wrist_roll)) {
      // step();
    // }
  // }
}

//Convenient initial position
static void setInitialPosition(Robot * robot) {
  setLeftArmPosition(0.0, 1.35, 0.0, -2.32, 0.0, false);
  setRightArmPosition(0.0, 1.35, 0.0, -2.32, 0.0, false);

  // setGripper(false, true, 0.0, false);
  // setGripper(true, true, 0.0, false);

  // setTorsoHeight(0.2, true);
}

void calculateDirection(){
    double yaw = imuSensor->getRollPitchYaw()[2];
    // cout<<"Yaw is: "<<yaw<<endl;
    headDirection = yaw;
    // cout<<"HeadDirection is :"<<headDirection*180/M_PI<< endl;
    // if(yaw>-M_PI/8 && yaw<=M_PI/8){
        // headDirection = NORTH;
        // cout<<"HeadDirection is : NORTH"<< endl;
    // }
    // if(yaw>M_PI/4-M_PI/8 && yaw<=M_PI/4+M_PI/8){
        // headDirection = NORTHWEST;
        // cout<<"HeadDirection is : NORTHWEST"<< endl;
    // }
    // if(yaw>M_PI/2-M_PI/8 && yaw<=M_PI/2+M_PI/8){
        // headDirection = WEST;
        // cout<<"HeadDirection is : WEST"<< endl;
    // }
    // if(yaw>3*M_PI/4-M_PI/8 && yaw<=3*M_PI/4+M_PI/8){
        // headDirection = SOUTHWEST;
        // cout<<"HeadDirection is : SOUTHWEST"<< endl;
    // }
    // if(yaw>M_PI-M_PI/8 && yaw<=-M_PI+M_PI/8){
        // headDirection = SOUTH;
        // cout<<"HeadDirection is : SOUTH"<< endl;
    // }
    // if(yaw>-3*M_PI/4-M_PI/8 && yaw<=-3*M_PI/4+M_PI/8){
        // headDirection = SOUTHEAST;
        // cout<<"HeadDirection is : SOUTHEAST"<< endl;
    // }
    // if(yaw>-M_PI/2-M_PI/8 && yaw<=-M_PI/2+M_PI/8){
        // headDirection = EAST;
        // cout<<"HeadDirection is : EAST"<< endl;
    // }
    // if(yaw>-M_PI/4-M_PI/8 && yaw<=-M_PI/4+M_PI/8){
        // headDirection = NORTHEAST;
        // cout<<"HeadDirection is : NORTHEAST"<< endl;
    // }
    
}

int findIndex(double x){
    /*if(floor(2*x+32)<0)
      return 0;
    if(floor(2*x+32)>63)
      return 63;*/
    return floor(x+32);
} 

double findPosition(int x){
    return (double)(x-31.5);
}

void blockMap(int **map, int xBlock, int zBlock, int size, int xUBlock, int zUBlock, int sizeU){  //size is asssumed to be an odd number
    for(int i = -size/2;i<=size/2;i++){
        for(int j = -size/2;j<=size/2;j++){
            if( (xBlock+i) < PLATFORM_X && (xBlock+i) >= 0 && (zBlock+j) < PLATFORM_Z && (zBlock+j) >= 0 )
                map[xBlock+i][zBlock+j] += 500;
        }
    }
    for(int i = -sizeU/2;i<=sizeU/2;i++){
        for(int j = -sizeU/2;j<=sizeU/2;j++){
            if( (xUBlock+i) < PLATFORM_X && (xUBlock+i) >= 0 && (zUBlock+j) < PLATFORM_Z && (zUBlock+j) >= 0 && map[xUBlock+i][zUBlock+j] > 500)
                map[xUBlock+i][zUBlock+j] -= 500;
        }
    }
    ofstream of("BlockedMap.tsv", ofstream::out);
    cout<<"valhallllaaaaaaaaaa "<< map[xUBlock][zUBlock]<<endl;
    for (int i = 0; i < PLATFORM_X; i++){
        for(int j = 0; j < PLATFORM_Z; j++){
            of << map[i][j] << "\t";
        }
        of <<endl;        
    }
    of.close();
    return;
}
// A function to change the a block to its original state
void unBlockMap(int** map, int xBlock, int zBlock, int size, int xUBlock, int zUBlock, int sizeU){  //size is asssumed to be an odd number
    for(int i = -size/2;i<=size/2;i++){
        for(int j = -size/2;j<=size/2;j++){
            if( (xBlock+i) < PLATFORM_X && (xBlock+i) >= 0 && (zBlock+j) < PLATFORM_Z && (zBlock+j) >= 0 && map[xBlock+i][zBlock+j] > 500)
                map[xBlock+i][zBlock+j] -= 500;
        }
    }
    return;
}

//Find in a 2d vector 
void find2D(vector <int> &fExc2Dx, vector <int> &fExc2Dz, vector < vector <float> > &v, int spike,int sizeX, int sizeY, int gle){
    

    int count=0;
    for(int i=0;i<sizeX;i++){
        for(int j=0;j<sizeY;j++){
            if(gle == 1 && v[i][j]>=spike){
                fExc2Dx[count]=i;
                fExc2Dz[count]=j;
                count++;
            }
            if(gle == 0 && v[i][j]==spike){
                fExc2Dx[count]=i;
                fExc2Dz[count]=j;
                count++;
            }
            if(gle == -1 && v[i][j]<=spike){
                fExc2Dx[count]=i;
                fExc2Dz[count]=j;
                count++;    
            }
        }
    }
    
    fExc2Dx[count] = -1;
    fExc2Dz[count] = -1;
}

void find4D(vector <int> &fExc4Dx, vector <int> &fExc4Dz, vector< vector < vector < vector <int> > > > & delayBuffer,int k, int l ,int spike,int sizeX, int sizeY, int gle){
    
    int count=0;
    for(int i=0;i<sizeX;i++){
        for(int j=0;j<sizeY;j++){
            if(gle == 1 && delayBuffer[i][j][k][l]>=spike){
                fExc4Dx[count]=i;
                fExc4Dz[count]=j;
                count++;
            }
            if(gle == 0 && delayBuffer[i][j][k][l]==spike){
                fExc4Dx[count]=i;
                fExc4Dz[count]=j;
                count++;
            }
            if(gle == -1 && delayBuffer[i][j][k][l]<=spike){
                fExc4Dx[count]=i;
                fExc4Dz[count]=j;
                count++;
            }
        }
    }
    fExc4Dx[count] = -1;
    fExc4Dz[count] = -1;
}

//The Spikewave algorithm used to propagate signals through the grid map from start to goal
bool spikeWave(int xStart, int zStart, int xGoal,int zGoal, int** map, const int xMapSize, const int zMapSize){

    const int Ne1 = xMapSize;
    const int Ne2 = zMapSize;
    int SPIKE = 1;
    int REFRACTORY = -5;
    int W_INIT = 5;
    float LEARNING_RATE = 1.0;
    
    //Each neuron connects to its 8 neighbors
    vector< vector < vector < vector <float> > > > wgt(Ne1 , vector < vector < vector <float> > > (Ne2, vector < vector <float> > (Ne1, vector <float> (Ne2 , 0.0) ) ) );
        
    for(int i = 0; i<Ne1; i++){
        for(int j = 0; j<Ne2; j++){
            for(int m = -1; m<=1; m++){
                for(int n = -1; n<=1; n++){
                    //if i+m > 0 && i+m <= size(wgt,1) && j+n > 0 && j+n <= size(wgt,2) && (m ~= 0 || n ~= 0)
                    if ((m == 0 || n == 0) && m != n && i+m >= 0 && i+m < Ne1 && j+n >= 0 && j+n < Ne2){
                        wgt[i][j][i+m][j+n] = W_INIT;
                    }
                }
            }
        }
    }
    
    vector < vector <float> > v (Ne1 , vector <float> (Ne2 , 0.0));
    vector < vector <float> > u (Ne1 , vector <float> (Ne2 , 0.0));

         
    vector< vector < vector < vector <int> > > > delayBuffer(Ne1 , vector < vector < vector <int> > > (Ne2, vector < vector <int> > (Ne1, vector <int> (Ne2 , 0) ) ) );
      
    //the spike wave is initiated from the starting location
    v[xStart][zStart] = SPIKE;
      // cout<<xStart<<zStart<<endl;
    
    bool foundGoal = false;
    int timeSteps = 0;
    vector < vector <int > > aer;
    
    while (!foundGoal){
        
        // for(vector<vector <int> >::iterator jt = aer.begin(); jt != aer.end(); ++jt ){
        // }
        timeSteps = timeSteps + 1;
        vector <int> fExc2Dx (Ne1*Ne2, 0);
        vector <int> fExc2Dz (Ne1*Ne2, 0);
        find2D(fExc2Dx, fExc2Dz, v, SPIKE, Ne1, Ne2, 1);// indices of spikes in v
        for(int i=0;fExc2Dx[i]!=-1;i++){
            vector<int> temp;
            temp.push_back(timeSteps);
            temp.push_back(fExc2Dx[i]);
            temp.push_back(fExc2Dz[i]);
            aer.push_back(temp); // keep spike information in an addressable event representation (spikeID and timeStep)
        }
        //Neurons that spike send their spike to their post-synaptic targets.
        //The weights are updated and the spike goes in a delay buffer to
        //targets. The neuron's recovery variable is set to its refractory value.
        if (fExc2Dx[0] != -1){
            for (int i = 0; fExc2Dx[i] != -1; i++){
                u[fExc2Dx[i]][fExc2Dz[i]] = REFRACTORY;
                delayRule(wgt,fExc2Dx[i],fExc2Dz[i],Ne1,Ne2,map[fExc2Dx[i]][fExc2Dz[i]], LEARNING_RATE);
                for (int j=0; j<Ne2; j++){
                    for (int k=0; k<Ne1; k++){
                        delayBuffer[fExc2Dx[i]][fExc2Dz[i]][k][j] = round(wgt[fExc2Dx[i]][fExc2Dz[i]][k][j]);
                    }
                }
                
                if (fExc2Dx[i] == xGoal && fExc2Dz[i] == zGoal){
                    foundGoal = true;   // neuron at goal location spiked.
                }
            }
        }
        
        float Iexc[Ne1][Ne2];
        
        //if the spike wave is still propagating, get the synaptic input for
        //all neurons. Synaptic input is based on recovery variable, and spikes
        //that are arriving to the neuron at this time step.
        vector <int> fExc4Dx (Ne1*Ne2, 0);
        vector <int> fExc4Dz (Ne1*Ne2, 0);
        if (!foundGoal){
            for (int j=0; j<Ne2; j++){
                for (int k=0; k<Ne1; k++){
                    Iexc[k][j] = u[k][j];
                }
            }
            for (int i=0; i<Ne1; i++){
                for (int j=0; j<Ne2; j++){
                    find4D( fExc4Dx, fExc4Dz, delayBuffer,i,j, 1, Ne1, Ne2, 0); // find indecies in delayBuffer
                    // if(v[k][j]>=SPIKE)
                    if (fExc4Dx[0]!=-1){//is not empty
                        for(int k=0; fExc4Dx[k]!=-1; k++){
                            // if(v[k][j]>=SPIKE)
                            Iexc[i][j] = Iexc[i][j] + (wgt[fExc4Dx[k]][fExc4Dz[k]][i][j] > 0);
                        }
                    }
                }
            }
            //Update membrane potential (v) and recovery variable (u)
            for (int j=0; j<Ne2; j++){
                for (int k=0; k<Ne1; k++){
                    // if(v[k][j]>=SPIKE)
                    v[k][j] = v[k][j] + Iexc[k][j];
                }
            }
            for (int j=0; j<Ne2; j++){
                for (int k=0; k<Ne1; k++){
                    u[k][j] = min(u[k][j] + 1, (float)0.0);
                }
            }
        }
   
        for (int i=0; i<Ne1; i++){
            for (int j=0; j<Ne2; j++){
                for (int k=0; k<Ne1; k++){
                    for (int l=0; l<Ne2; l++){
                        delayBuffer[i][j][k][l] = max(0, delayBuffer[i][j][k][l] - 1);  // Update the delays of the scheduled spikes.
                    }
                }
            }
        }
        
    }
    
    return getPath(aer, map, xStart, zStart, xGoal, zGoal); // Get the path from the AER table. 
        
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
  // the axonal delay between neurons.
// //
// @param wBefore - weight value before applying learning rule.
// @param value - value from the map.
// @param learnRate - learning rate.
// @return - weight value after applying learning rule.
void delayRule(vector< vector < vector < vector <float> > > > &wgt, int fExcXi , int fExcYi , int Ne1, int Ne2, int value, float learnRate){

    float valMat[Ne1][Ne2];
    for(int i=0; i < Ne1; i++){
        for(int j=0; j <Ne2; j++){
            valMat[i][j] = learnRate * (value - wgt[fExcXi][fExcYi][i][j]);
            wgt[fExcXi][fExcYi][i][j] = wgt[fExcXi][fExcYi][i][j] + (wgt[fExcXi][fExcYi][i][j] > 0) * valMat[i][j];
        }
    }
    return;
}

 // getPath - Generates the path based on the AER spike table. The spike
   // table is ordered from earliest spike to latest. The algorithm starts at
   // the end of the table and finds the most recent spike from a neighbor.
   // This generates a near shortest path from start to end.
 
 // @param spks - AER  table containing the spike time and ID of each neuron.
 // @param map - map of the environment.
 // @param s - start location.
 // @param e - end (goal) location.
 // @return - path from start to end.
bool getPath(vector < vector < int > > spks, int** map, int sx, int sy, int ex, int ey ){

    // global START;
    
    int pinx = 0;
    // for(vector<vector <int> >::iterator jt = spks.begin(); jt != aer.end(); ++jt ){
    // }
    path[0][pinx] = ex;
    path[1][pinx] = ey;
    
    // for (vector<vector <int> >::reverse_iterator it = spks.rbegin(); it != spks.rend(); ++it ){
        // int i = (*it)[0];
        // while((*it)[0] == i){
            // it++;
            // cout<<"okokokok "<<(*it)[0]<<" "<<(*it)[1]<<" "<<(*it)[2]<<endl;
        // }
    // }
    int count = -1;
    while (sqrt(pow(path[0][pinx]-sx,2)+pow(path[1][pinx]-sy,2) > 1.0)){
        count++;
        cout<<"In spikewave debugging count : pinx "<<count<<" "<<pinx<<endl;
        if (count >= 600 || pinx>=1000-1){
            return false;
        }
        //work from most recent to oldest in the spike table
        for (vector<vector <int> >::reverse_iterator it = spks.rbegin(); it != spks.rend(); ++it ){
            int i = (*it)[0];
            vector<vector <int> > inx;
            while((*it)[0] == i){
                inx.push_back(*it);
                it++;
            }
            bool found = false;
            int k = 0;
            int lst[20][2];
            //find the last spike from a neighboring neuron.
            for(vector<vector <int> >::iterator jt = inx.begin(); jt != inx.end(); ++jt ){
                
                if(sqrt(pow(path[0][pinx]-(*jt)[1],2)+pow(path[1][pinx]-(*jt)[2],2)) < 1.5 && sqrt(pow(path[0][pinx]-(*jt)[1],2)+pow(path[1][pinx]-(*jt)[2],2))>0.5){
                    
                    lst[k][0] = (*jt)[1];
                    lst[k][1] = (*jt)[2];
                    k = k + 1;
                    found = true;
                }
            }
            
            //if there is more then one spike, find the one with the lowest
            //cost and closest to starting location.
            
            if(found){
                int minx = 0;
                int cost = INT_MAX;
                float dist = (float) INT_MAX;
                for(int m =0; m<k; m++){
                    if (map[lst[m][0]][lst[m][1]] < cost){
                        cost = map[lst[m][0]][lst[m][1]];
                        minx = m;
                        dist = sqrt(pow((sx - lst[m][0]),2)+pow((sy-lst[m][1]),2));
                    }
                    else if (map[lst[m][0]][lst[m][1]] == cost && sqrt(pow((sx - lst[m][0]),2)+pow((sy-lst[m][1]),2)) < dist){
                        minx = m;
                        dist = sqrt(pow((sx - lst[m][0]),2)+pow((sy-lst[m][1]),2));
                    }
                }
                //add the neuron to the path.
                pinx++;
                path[0][pinx] = lst[minx][0];
                path[1][pinx] = lst[minx][1];
            }
        }
    }
    pinx++;
    path[0][pinx] = -1;
    path[1][pinx] = -1;
    return true;
}

void goForRInThisDirection(Supervisor * robot, double R, double angle){
    if(abs(angle-headDirection)>M_PI){
        if((angle-headDirection)>0)
            robotRotate(robot,(angle-headDirection)-2*M_PI);
        else
            robotRotate(robot,(angle-headDirection)+2*M_PI);
    }else{
        robotRotate(robot,angle-headDirection);
    }
    robotGoForward(robot,R);
}


// High level function to rotate the robot around itself of a given angle [rad]
// Note: the angle can be negative
static void robotRotate(Supervisor* robot, double angle) {
    stopWheels();
    
    setRotationWheelsAngles(robot,3.0 * M_PI_4, M_PI_4, -3.0 * M_PI_4, -M_PI_4, true);
    const double maxWheelSpeed = angle > 0 ? MAX_WHEEL_SPEED : -MAX_WHEEL_SPEED;
    setWheelsSpeed(maxWheelSpeed);
  
    double initialWheel0Position = wheelSensors[FLL_WHEEL]->getValue();
    // expected travel distance done by the wheel
    double expectedTravelDistance = fabs(angle * 0.5 * (WHEELS_DISTANCE + SUB_WHEELS_DISTANCE));
  
    while (true) {
        double wheel0Position = wheelSensors[FLL_WHEEL]->getValue();
        // travel distance done by the wheel
        double wheel0TravelDistance = fabs(WHEEL_RADIUS * (wheel0Position - initialWheel0Position));
    
        if (wheel0TravelDistance > expectedTravelDistance)
            break;
    
        // reduce the speed before reaching the target
        if (expectedTravelDistance - wheel0TravelDistance < 0.025)
            setWheelsSpeed(0.1 * maxWheelSpeed);
    
        step(robot);
  }
  calculateDirection();
  // reset wheels
  setRotationWheelsAngles(robot,0.0, 0.0, 0.0, 0.0, true);
  stopWheels();
}

float minMidLidar(const float * lidarValues, int lidarWidth, int range){
    float total = 0;
    for (int i=(int)floor(0.5 * lidarWidth)-range/2;i<=(int)floor(0.5 * lidarWidth)+range/2;i++){
        // lidarValues[i]<min ? min=lidarValues[i] : min=min;
        total+= lidarValues[i];
    }
    return total/range;
}

int checkLidarForObs(const float * lidarValues, int lidarWidth, float thresh){
    for (int i=1;i<lidarWidth-1;i++){
        if(lidarValues[i-1] < thresh && lidarValues[i] < thresh && lidarValues[i+1] < thresh)
            return i;
    }
    return -1;
}

// High level function to go forward for a given distance [m]
// Note: the distance can be negative
static void robotGoForward(Supervisor * robot, double distance) {
    double maxWheelSpeed = distance > 0 ? MAX_WHEEL_SPEED : -MAX_WHEEL_SPEED;
    setWheelsSpeed(maxWheelSpeed);
    
    double initialWheel0Position = wheelSensors[FLL_WHEEL]->getValue();
    while (true) {
        // leftSpeed =  MAX_WHEEL_SPEED; 
        // rightSpeed =  MAX_WHEEL_SPEED;
        
        // get lidar values
        const float *lidarValues = laserTilt->getRangeImage();
        
        //apply the braitenberg coefficients on the resulted values of the lidar
        /*for (int i= 0.25 * lidarWidth; i < 0.5 * lidarWidth; i++) {
            const int j = lidarWidth - i - 1;
            const int k = i - 0.25 * lidarWidth;
            float lidari = lidarValues[i];
            float lidarj = lidarValues[j];
            if (isinf(lidarValues[i]))
                lidari = lidarMaxRange;
            if (isinf(lidarValues[j]))
                lidarj = lidarMaxRange;
            leftSpeed +=
              braitenbergCoefficients[k] * ((1.0 - lidari / lidarMaxRange) - (1.0 - lidarj / lidarMaxRange));
            //printf("bc[%d]: %g, Lidar valuesi[%d]: %g, Lidar valuesj[%d]: %g\n", k, braitenberg_coefficients[k],i,lidar_values[i],j,lidar_values[j]);
            
            rightSpeed +=
              braitenbergCoefficients[k] * ((1.0 - lidarj / lidarMaxRange) - (1.0 - lidari / lidarMaxRange));
            //printf("bc[%d]: %g, Lidar valuesj[%d]: %g, Lidar valuesi[%d]: %g\n", k, braitenberg_coefficients[k],i,lidar_values[i],j,lidar_values[j]);
          
        }*/
        
        for(int i=0;i<numberOfRobots;i++){
            transFields[i] = robotNodes[i]->getField("translation");
            transValues[i] = transFields[i]->getSFVec3f();
            
            xCur[i] = findIndex(transValues[i][0]);
            zCur[i] = findIndex(transValues[i][2]);
            
            distances[i] = (double)sqrt(pow((xCur[i]-xCur[0]),2)+pow((zCur[i]-zCur[0]),2));
        }
        areaOccupancy[xCur[0]][zCur[0]]+=1;
        double minDis = 10000;
        int argMinDis = -1;
        for(int i=1;i<numberOfRobots;i++){
            if(minDis>distances[i]){
                minDis = distances[i];
                argMinDis = i;
            }
        }
        if((minMidLidar(lidarValues,lidarWidth,20)<(float)(lidarMaxRange/35)) && distance > 0){
            // customDataFields[0]->setSFString("OBSTACLE"); 
            cout<<"Obstacle: a wall or something in front of robot"<<endl;
            nWallObs++;
            // xBlock = -1;
            // zBlock = -1;
            obstacle = true;
            robotGoForward(robot,-0.5);
            setWheelsSpeed(0);
            break;
        }
        int l = checkLidarForObs(lidarValues,lidarWidth,(lidarMaxRange/20));
        if(l != -1 && distance > 0){
            // customDataFields[0]->setSFString("OBSTACLE"); 
            cout<<"Obstacle: a wall or something besides the robot"<<endl;
            nWallObs++;
            // xBlock = -1;
            // zBlock = -1;
            obstacle = true;
            if(l<lidarWidth/2){
            robotGoForward(robot,-0.5);
                robotRotate(robot,M_PI/2);
            }else{
                robotRotate(robot,-M_PI/2);
            }
            robotGoForward(robot,-0.5);
            setWheelsSpeed(0);
            break;
        }
        
        // if(customDataFields[0]->getSFString().compare("EXPLORE")==0 && argTRRef != -1 && distance > 0){  //third robot point of referrence to let go of third robot status
            // if(customDataFields[argTRRef]->getSFString().compare("NULL")==0 ){
                // customDataFields[0]->setSFString("NULL");
                // argTRRef = -1;
                // argCFRobot = -1;
            // }
        // }
        
        if(customDataFields[0]->getSFString().compare("NULL")==0 && !foundCollision){
            for(int i=1;i<numberOfRobots;i++){
                if((customDataFields[i]->getSFString().compare("EXPLORE")==0
                    || customDataFields[i]->getSFString().compare("WAITING")==0)
                    && distance > 0 && distances[i]<25 ){
                    
                    xBlock = xCur[i];
                    zBlock = zCur[i];
                    cout<<"balahablahablha "<<xBlock<<" "<<zBlock<<endl;
                    obstacle = true;
                    foundCollision = true;
                    break;
                }
            }
        }
        
        if(customDataFields[0]->getSFString().compare("NULL")==0 && argCFRobot!=-1){
            argCFRobot=-1;
            setWheelsSpeed(maxWheelSpeed);
        }
        
        
        if(distance > 0){
            if(minDis<7 && argMinDis != argCFRobot){
                cout<<"Obstacle Detected "<<thisNode<<endl;
                nRobotObs++;
                
                
                transFields[argMinDis] = robotNodes[argMinDis]->getField("translation");
                transValues[argMinDis] = transFields[argMinDis]->getSFVec3f();
                
                xCur[argMinDis] = findIndex(transValues[argMinDis][0]);
                zCur[argMinDis] = findIndex(transValues[argMinDis][2]);
                
                xBlock = xCur[argMinDis];
                zBlock = zCur[argMinDis];
                
                
                if((customDataFields[0]->getSFString().compare("NULL")==0 && 
                customDataFields[argMinDis]->getSFString().compare("NULL")==0) || 
                (customDataFields[0]->getSFString().compare("EXPLORE")==0 && 
                customDataFields[argMinDis]->getSFString().compare("EXPLORE")==0)){  
                    
                    // int randomNumber = rand();
                    // cout<<"Robot "<<thisNode<<" here 1 "<<to_string(randomNumber)<<endl;
                    // customDataFields[argMinDis]->setSFString(to_string(randomNumber));
                    
                    // while(customDataFields[0]->getSFString().compare("NULL")==0||customDataFields[0]->getSFString().compare("EXPLORE")==0){ //one thing that might happen is that one robot get stuck in this loop while the other one is somewhere else
                        // robot->step(TIME_STEP);
                    // }
                    // cout<<"Robot "<<thisNode<<" here 2 "<<customDataFields[0]->getSFString()<<endl;
                    // if(randomNumber>stoi(customDataFields[0]->getSFString(),nullptr,10)){
                    if(robotNumbers[0]<robotNumbers[argMinDis]){ //the Robot with grater number would explore
                        setWheelsSpeed(0);
                        while(customDataFields[argMinDis]->getSFString().compare("NULL")==0){ //one thing that might happen is that one robot get stuck in this loop while the other one is somewhere else
                            robot->step(TIME_STEP);
                        }
                        customDataFields[0]->setSFString("WAITING");
                        
                        /*for(int i=1;i<numberOfRobots;i++){
                            if(i == argMinDis)
                              continue;
                            if(distances[i]<25 && customDataFields[i]->getSFString().compare("NULL")==0){
                                customDataFields[i]->setSFString("BLOCK AT "+to_string(xCur[0])+" "+to_string(zCur[0])); 
                            }
                        }*/
                        while(minDis<16){  
                            
                            for(int i=0;i<numberOfRobots;i++){
                                transFields[i] = robotNodes[i]->getField("translation");
                                transValues[i] = transFields[i]->getSFVec3f();
                                
                                xCur[i] = findIndex(transValues[i][0]);
                                zCur[i] = findIndex(transValues[i][2]);
                                
                                distances[i] = (double)sqrt(pow((xCur[i]-xCur[0]),2)+pow((zCur[i]-zCur[0]),2));
                            }
                            minDis = 10000;
                            int argNMinDis = -1;
                            for(int i=1;i<numberOfRobots;i++){
                                if(minDis>distances[i]){
                                    minDis = distances[i];
                                    argNMinDis = i;
                                }
                            }
                            if(argMinDis != argNMinDis){
                                if(customDataFields[argMinDis]->getSFString().compare("WAITING")!=0)
                                    customDataFields[argMinDis]->setSFString("NULL");
                                argMinDis = argNMinDis;
                            }
                            if(customDataFields[0]->getSFString().compare("WAITING")==0 && customDataFields[argMinDis]->getSFString().compare("WAITING")==0 ){ //Explore-Waiting
                                obstacle = true;
                                break;
                            }
                            robot->step(TIME_STEP);
                        }
                        customDataFields[0]->setSFString("NULL");
                        if(customDataFields[argMinDis]->getSFString().compare("WAITING")!=0)
                            customDataFields[argMinDis]->setSFString("NULL");
                        argCFRobot = -1;
                        setWheelsSpeed(maxWheelSpeed);
                        break;
                    }else{
                        customDataFields[0]->setSFString("EXPLORE"); 
                        argCFRobot = argMinDis;
                        obstacle = true;
                        break;
                    }
                }else if(customDataFields[argMinDis]->getSFString().compare("NULL")==0){ //Explore-Null or Waiting-Null
                    cout<<"Robot "<<thisNode<<" (EXPLORE_null)(WAITING_null)"<<endl;
                    customDataFields[0]->setSFString("WAITING");
                    setWheelsSpeed(0);
                    while(minDis<16){  
                            
                            for(int i=0;i<numberOfRobots;i++){
                                transFields[i] = robotNodes[i]->getField("translation");
                                transValues[i] = transFields[i]->getSFVec3f();
                                
                                xCur[i] = findIndex(transValues[i][0]);
                                zCur[i] = findIndex(transValues[i][2]);
                                
                                distances[i] = (double)sqrt(pow((xCur[i]-xCur[0]),2)+pow((zCur[i]-zCur[0]),2));
                            }
                            minDis = 10000;
                            int argNMinDis = -1;
                            for(int i=1;i<numberOfRobots;i++){
                                if(minDis>distances[i]){
                                    minDis = distances[i];
                                    argNMinDis = i;
                                }
                            }
                            if(argMinDis != argNMinDis){
                                if(customDataFields[argMinDis]->getSFString().compare("WAITING")!=0)
                                    customDataFields[argMinDis]->setSFString("NULL");
                                argMinDis = argNMinDis;
                            }
                            if(customDataFields[0]->getSFString().compare("WAITING")==0 && customDataFields[argMinDis]->getSFString().compare("WAITING")==0 ){ //Explore-Waiting
                                obstacle = true;
                                break;
                            }
                            robot->step(TIME_STEP);
                        }
                        customDataFields[0]->setSFString("NULL");
                        if(customDataFields[argMinDis]->getSFString().compare("WAITING")!=0)
                            customDataFields[argMinDis]->setSFString("NULL");
                        argCFRobot = -1;
                        setWheelsSpeed(maxWheelSpeed);
                        break;
                }else if(customDataFields[0]->getSFString().compare("NULL")==0 ){ //Null-Explore or Null-Waiting
                    cout<<"Robot "<< thisNode<<" (NULL_explore)(NULL_waiting)"<<endl;
                    customDataFields[0]->setSFString("EXPLORE");
                    argTRRef = argMinDis;
                    argCFRobot = argMinDis;
                    obstacle = true;
                    break;
                }else if(customDataFields[0]->getSFString().compare("EXPLORE")==0 && customDataFields[argMinDis]->getSFString().compare("WAITING")==0 ){ //Explore-Waiting
                    cout<<"Robot "<< thisNode<<" (EXPLORE_waiting)"<<endl;
                    customDataFields[0]->setSFString("EXPLORE");
                    argTRRef = argMinDis;
                    argCFRobot = argMinDis;
                    obstacle = true;
                    break;
                }
            }
            // else if(customDataFields[0]->getSFString().compare("OBSTACLE")==0){
                // obstacle = true;
                // customDataFields[0]->setSFString("NULL"); 
                // break;
            // }
            // else if(leftSpeed < rightSpeed - LIDAR_THRESHOLD){
                // cout<<"LeftSpeed&rightSpeed1 "<<leftSpeed<<" "<<rightSpeed<<endl;
                // robotRotate(robot,M_PI/2);
                // robotGoForward(robot,0.5);
                // robotRotate(robot,-M_PI/2);
                // setWheelsSpeed(maxWheelSpeed);
            // }
            // else if(rightSpeed < leftSpeed - LIDAR_THRESHOLD){
                // cout<<"LeftSpeed&rightSpeed2 "<<leftSpeed<<" "<<rightSpeed<<endl;
                // robotRotate(robot,-M_PI/2);
                // robotGoForward(robot,0.5);
                // robotRotate(robot,M_PI/2);
                // setWheelsSpeed(maxWheelSpeed);
            // }
        }
        double wheel0Position = wheelSensors[FLL_WHEEL]->getValue();
        // travel distance done by the wheel
        double wheel0TravelDistance = fabs(WHEEL_RADIUS * (wheel0Position - initialWheel0Position));
        
        if (wheel0TravelDistance > fabs(distance))
            break;
    
        // reduce the speed before reaching the target
        if (fabs(distance) - wheel0TravelDistance < 0.025)
            setWheelsSpeed(0.1 * maxWheelSpeed);
    
        step(robot);
    }
    stopWheels();
}

void findTurnsInPath(int m){ // m is the index of the end node in the path 
    int j=0;
    int xDiff,zDiff;
    int xBase=xCur[0];
    int zBase=zCur[0];
    for(int i=m;i>=0;i--) {
        int xNext = path[0][i];
        int zNext = path[1][i];
        xDiff = xNext-xBase;
        zDiff = zNext-zBase;
        do{
            xBase = xNext;
            zBase = zNext;
            i--;
            xNext = path[0][i];
            zNext = path[1][i];
        }while((xDiff==(xNext-xBase))&&(zDiff==(zNext-zBase)));
        i++;
        pathTurnsIdx[0][j] = xBase;
        pathTurnsIdx[1][j] = zBase;
        
        cout<<"xBase: "<<xBase<<" zBase: "<<zBase<<endl;
        
        j++;
    }
    return;
}


int main(int argc, char **argv) {
    
    startingState = true;
    robot = new Supervisor();
    ifstream ft("../RobotList.txt");
    if (! ft) {
        cout << "Error, file couldn't be opened" << endl;
    }
    
    string r;
    thisNode = argv[1];
    cout << "Allthough " <<thisNode<< endl;
    vector<string> robotNames;
    numberOfRobots = 0;
    argTRRef = -1;
    foundCollision = false;
    while(ft>>r){
        numberOfRobots++;
        robotNames.push_back(r);
    }  
    ifstream ft1("../Sources.txt");
    if (! ft1) {
        cout << "Error, file couldn't be opened" << endl;
    }
    for(int i=0; i<N_SOURCES; i++){
        ft1 >> X_SOURCE[i];
        ft1 >> Z_SOURCE[i];
        cout<<"i: "<< i << " X_SOURCE: "<<X_SOURCE[i]<<" Z_SOURCE: "<<Z_SOURCE[i]<<endl;
    }
    ifstream ft2("../Goals.txt");
    if (! ft2) {
        cout << "Error, file couldn't be opened" << endl;
    }
    for(int i=0; i<N_GOALS; i++){
        ft2 >> X_GOAL[i];
        ft2 >> Z_GOAL[i];
        cout<<"i: "<< i << " X_GOAL: "<<X_GOAL[i]<<" Z_GOAL: "<<Z_GOAL[i]<<endl;
    }
    robotNodes = new Node*[numberOfRobots];
    transFields = new Field*[numberOfRobots];
    customDataFields = new Field*[numberOfRobots];
    transValues = new const double*[numberOfRobots];
    xCur = new int[numberOfRobots];
    zCur = new int[numberOfRobots];
    distances = new double[numberOfRobots];
     
    int flag = 0;
    srand (time(NULL)+(intptr_t)(&flag));
    for(int i=0; i<numberOfRobots; i++){
        cout<<"sadsad "<<(N_SOURCES/numberOfRobots)<< " "<<"PR2"+robotNames[i]<<endl;
        if (robotNames[i].compare(thisNode)==0){
            robotNodes[0] = robot->getFromDef("PR2"+robotNames[i]);
            robotNumbers[0] = stoi(robotNames[i],nullptr,10);
            customDataFields[0] = robotNodes[0]->getField("customData");
            flag++;
            int cSource = (rand() % (N_SOURCES/numberOfRobots))+(N_SOURCES/numberOfRobots)*(stoi(thisNode,nullptr,10)-1);//stoi(thisNode,nullptr,10);
            transFields[0] = robotNodes[0]->getField("translation");
            const double initialPos[3] = { findPosition(X_SOURCE[cSource]) , 0.0 , findPosition(Z_SOURCE[cSource]) };
            transFields[0]->setSFVec3f(initialPos);
        }else{
            std::vector<string>::iterator it;
            it = find (robotNames.begin(), robotNames.end(), thisNode);
            if (it != robotNames.end()){
                robotNodes[i-flag+1]=robot->getFromDef("PR2"+robotNames[i]);
                robotNumbers[i-flag+1] = stoi(robotNames[i],nullptr,10);
                customDataFields[i-flag+1] = robotNodes[i-flag+1]->getField("customData");
                cerr<<"thatnode"<<endl;
            }else{
                return 0;
            }
            
        }
    }
    ifstream fGoal("../AssignedGoals"+thisNode+".txt");
    if(fGoal.is_open())
    {   
        for(int i = 0; i < 2000; ++i){
            if(!fGoal.eof()){
            fGoal >> goalList[i];
            }else{
              goalList[i] = -1;
              break;
            }
        }
    }else{
        cerr<<"fGoal is not open"<<endl;
    }
    
    if (robotNodes[0] == NULL) {
        fprintf(stderr, "No DEF MY_ROBOT node found in the current world file\n");
        exit(1);
    }
    
    
    initializeDevices(robot);
    enableDevices();
    setInitialPosition(robot);
    customDataFields[0]->setSFString("NULL");
    // //go to the initial position
    // setLeftArmPosition(0.0, 0.5, 0.0, -0.5, 0.0, true);
    // setRightArmPosition(0.0, 0.5, 0.0, -0.5, 0.0, true);
    
    // laserTilt->enablePointCloud();
    // baseLaser->enablePointCloud();
    
    lidarWidth = laserTilt->getHorizontalResolution();
    lidarMaxRange = laserTilt->getMaxRange();

    
    // init braitenberg coefficient
    braitenbergCoefficients = (double *)malloc(sizeof(double) * lidarWidth);
    
    for (int i = 0; i < lidarWidth; i++){
        braitenbergCoefficients[i] = 6 * gaussian(i, lidarWidth / 4, lidarWidth / 12);
    }
    
    
    int* map[PLATFORM_X];
    for (int i=0; i<PLATFORM_X; i++)
         map[i] = (int *)malloc(PLATFORM_Z * sizeof(int));
    ifstream fp("edited map2.txt");
    if (! fp) {
        cout << "Error, file couldn't be opened" << endl; 
    }    
    for(int row = 0; row < PLATFORM_X; row++) {  // stop loops if nothing to read
        for(int column = 0; column < PLATFORM_Z; column++){
            fp >> map[row][column];
            if ( ! fp ) {
                cout << "Error reading file for element " << row << "," << column << endl; 
            }
        }
    }
    int fCCounter = 0; //found collision counter
    int j=0;
    cGoal = 0;
    ofstream ofp("outputLogFor"+thisNode+"with"+to_string(numberOfRobots)+"Robots"+".txt");
    if (!ofp) {
        cout << "Error, output file couldn't be opened" << endl; 
    }   
    ofstream areafp("outputLogFor"+thisNode+"with"+to_string(numberOfRobots)+"Robots"+"AreaOccupancy.txt");
    if (!areafp) {
        cout << "Error, output file couldn't be opened" << endl; 
    }
    while (robot->step(TIME_STEP) != -1) {
        
        fCCounter++;
        if(fCCounter>6000){  // reset the check for collision detector to 0 after certain amount of time
            foundCollision = false;
            fCCounter = 0;
        }
        
        transFields[0] = robotNodes[0]->getField("translation");
        transValues[0] = transFields[0]->getSFVec3f();
        
        xCur[0] = findIndex(transValues[0][0]);
        zCur[0] = findIndex(transValues[0][2]);    
        bool result;
        if(obstacle || startingState){
            
            cout<<"Obstacle or starting state "<<thisNode<<" "<<obstacle<<" "<<startingState<<endl;
            if(startingState){
                // cGoal = rand() % N_GOALS;
                int g = goalList[cGoal++];
                // if(stoi(thisNode,nullptr,10)==1){
                    // cGoal = 2;
                // }else{
                    // cGoal = 1;
                // }
                if(g>13||g<-1){
                  cerr<<"Error: out of boundry goal exists"<<endl;
                  return 0;
                }
                if(g!=-1){
                    xGoal = X_GOAL[g];
                    zGoal = Z_GOAL[g];
                }else{
                    const double initialPos[3] = {(double)(-90-5*stoi(thisNode,nullptr,10))  , 0.0 , (double)(-90-2*stoi(thisNode,nullptr,10)) };
                    transFields[0]->setSFVec3f(initialPos);
                    cout<<"Robot "<<thisNode<<" has visited all its goals"<<endl;
                    ofp<<"time to visit all goals: "<<robot->getTime()<<endl;
                    ofp<<"Number of Wall Obstacles "<<nWallObs<<endl;
                    ofp<<"Number of Robot Obstacles "<<nRobotObs<<endl;
                    for (int i = 0; i < PLATFORM_X; ++i){
                        for (int j = 0; j < PLATFORM_Z; ++j){
                            areafp << areaOccupancy[i][j]<<" ";
                        }
                        areafp<<"\n";
                    }
                    ofp.close();
                    areafp.close();
                    free(braitenbergCoefficients);
                    
                    while (robot->step(TIME_STEP) != -1);
                }    
            }
            int xStart = xCur[0];
            int zStart = zCur[0];
            printf("%d %d, source index\n", xStart, zStart);
            printf("%d %d %d, sink index\n", cGoal, xGoal, zGoal);
            if(obstacle && xBlock!=-1){
                blockMap(map, xBlock, zBlock, 3, xCur[0], zCur[0], 3);
                result = spikeWave(xStart, zStart, xGoal, zGoal, map, PLATFORM_X, PLATFORM_Z);
                unBlockMap(map, xBlock, zBlock, 3, xCur[0], zCur[0], 3);
            }else{
                result = spikeWave(xStart, zStart, xGoal, zGoal, map, PLATFORM_X, PLATFORM_Z);
            }
            cout<<"Robot "<<thisNode<< " The result "<< result <<endl;
            if(!result){
                obstacle = false;
                startingState = true;
                j=0;
                continue;
            }
            cout<<"Robot "<<thisNode<<" The Chosen Path is: "<<endl;
            endIdx=0;
            for(int i = 0; path[0][i]!=-1 ; i++) {  // stop loops if nothing to read
                cout << path[0][i]<< " " << path[1][i] << endl;
                endIdx =i;
            }
            if(endIdx==1){
                cerr<<"Robot "<<thisNode<<" path length is 1, Enter a further destination "<<endl;
            }
            findTurnsInPath(endIdx);
            obstacle = false;
            startingState = false;
            j=0;
        }
        
        int xNext;
        int zNext;
      
        xNext = pathTurnsIdx[0][j];
        zNext = pathTurnsIdx[1][j];
        j++;
        
        double angle;
        double distance = sqrt(pow((xNext-xCur[0]),2)+pow((zNext-zCur[0]),2));
        
        // cout<<"Hello50 "<<xCur[0]<<" "<<zCur[0]<<endl;
        // cout<<"Hello51 "<<xNext<<" "<<zNext<<endl;
        if((xNext-xCur[0]) != 0){
            angle = -atan2((zNext-zCur[0]),(xNext-xCur[0]));
            // cout<<"Hello22 "<<-angle<<endl;
        }
        else if((zNext-zCur[0])>0){
            // cout<<"Hello40 "<<xCur[0]<<" "<<zCur[0]<<endl;
            // cout<<"Hello41 "<<xNext<<" "<<zNext<<endl;
            angle = -M_PI/2;
        }else{
            // cout<<"Hello40 "<<xCur[0]<<" "<<zCur[0]<<endl;
            // cout<<"Hello41 "<<xNext<<" "<<zNext<<endl;
            angle = M_PI/2;
        }
        // cout<<"Robot "<<thisNode<<" Hello23 "<<(angle)<<" "<<headDirection<<endl;
        goForRInThisDirection(robot,distance,angle);
        endIdx--;
        if(xGoal==xCur[0]&&zGoal==zCur[0]){
            cout<<"Robot "<<thisNode<<" Robot Reached the Goal"<<endl;
            // visitedGoals[cGoal] = true; 
            startingState = true;
            if(customDataFields[0]->getSFString().compare("EXPLORE")==0)
                obstacle = true;
        }  
        // for(int g=0; g<N_GOALS;g++){
            // if(!visitedGoals[g])
                // break;
            // if(g==N_GOALS-1){
                // const double initialPos[3] = { -17 , 0.0 , (double)(-21-2*stoi(thisNode,nullptr,10)) };
                // transFields[0]->setSFVec3f(initialPos);
                // cout<<"Robot "<<thisNode<<" has visited all its goals"<<endl;
                // ofp<<"time to visit all goals: "<<robot->getTime()<<endl;
                // ofp<<"Number of Wall Obstacles "<<nWallObs<<endl;
                // ofp<<"Number of Robot Obstacles "<<nRobotObs<<endl;
                // free(braitenbergCoefficients);
                // delete robot;
                // return 0;
            // }
        // }
    }
}
