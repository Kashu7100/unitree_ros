/************************************************************************
using position_mode.cpp as an example, implement some logic to test feedback
and control 
************************************************************************/

#include <ros/ros.h>
#include <string>
#include <pthread.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "aliengo_sdk/aliengo_sdk.hpp"
#include "convert.h"

// data structure to store feedback data and command data
#include "AOneData.hpp"

using namespace UNITREE_LEGGED_SDK;

template<typename TLCM>
void* update_loop(void* param)
{
    TLCM *data = (TLCM *)param;
    while(ros::ok){
        data->Recv();
        usleep(2000);
    }
}

double jointLinearInterpolation(double initPos, double targetPos, double rate)
{
    double p;
    rate = std::min(std::max(rate, 0.0), 1.0);
    p = initPos*(1-rate) + targetPos*rate;
    return p;
}

template<typename TCmd, typename TState, typename TLCM>
int mainHelper(int argc, char *argv[], TLCM &roslcm)
{
    std::cout << "WARNING: Control level is set to LOW-level." << std::endl
              << "Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    ros::NodeHandle n;
    ros::Rate loop_rate(500);
    ros::Publisher pub = n.advertise<unitree_legged_msgs::LowState>("unitree/fbk_msgs", 5);

    long motiontime = 0;
    int rate_count = 0;
    int sin_count = 0;
    float Kp[3] = {0};  
    float Kd[3] = {0};
    AOneData init_fbk_data, // record initial joint angles
             init_cmd_data, // stores stance configuration
             fbk_data,      // the up-to-date feedback
             cmd_data;      // the command sents to the robot
    // set initial stance configuration
    init_cmd_data.setAng(FR_, -0.7, 1.2, -2.0); 
    init_cmd_data.setAng(FL_, 0.7, 1.2, -2.0);
    init_cmd_data.setAng(RR_, -0.7, 1.2, -2.0);
    init_cmd_data.setAng(RL_, 0.7, 1.2, -2.0);

    TCmd SendLowLCM = {0};
    TState RecvLowLCM = {0};
    unitree_legged_msgs::LowCmd SendLowROS;
    unitree_legged_msgs::LowState RecvLowROS;

    bool initiated_flag = false;  // initiate need time
    int count = 0;

    roslcm.SubscribeState();

    pthread_t tid;
    pthread_create(&tid, NULL, update_loop<TLCM>, &roslcm);

    SendLowROS.levelFlag = LOWLEVEL;
    for(int i = 0; i<12; i++){
        SendLowROS.motorCmd[i].mode = 0x0A;   // motor switch to servo (PMSM) mode
        SendLowROS.motorCmd[i].q = PosStopF;        // disable position control loop
        SendLowROS.motorCmd[i].Kp = 0;
        SendLowROS.motorCmd[i].dq = VelStopF;       // disable veloicity control loop
        SendLowROS.motorCmd[i].Kd = 0;
        SendLowROS.motorCmd[i].tau = 0;
    }

    while (ros::ok()){
        roslcm.Get(RecvLowLCM);
        RecvLowROS = ToRos(RecvLowLCM);

        // publish the received data structure 
        pub.publish(RecvLowROS);

        // always refresh the fbk_data
        for (int i = 0; i < 12; i++) {
            fbk_data.setAng(i, RecvLowROS.motorState[i].q);
        }

        if(initiated_flag == true){
            motiontime++;

            // print feedback angles
            printf("FL %d %f %f %f\n", FR_0, RecvLowROS.motorState[FL_0].q, RecvLowROS.motorState[FL_1].q, RecvLowROS.motorState[FL_2].q);
            printf("FR %d %f %f %f\n", FR_0, RecvLowROS.motorState[FR_0].q, RecvLowROS.motorState[FR_1].q, RecvLowROS.motorState[FR_2].q);
            printf("RL %d %f %f %f\n", FR_0, RecvLowROS.motorState[RL_0].q, RecvLowROS.motorState[RL_1].q, RecvLowROS.motorState[RL_2].q);
            printf("RR %d %f %f %f\n", FR_0, RecvLowROS.motorState[RR_0].q, RecvLowROS.motorState[RR_1].q, RecvLowROS.motorState[RR_2].q);
            // printf("%f %f \n",  RecvLowROS.motorState[FR_0].mode, RecvLowROS.motorState[FR_1].mode);
            // printf("foot force %d %d %d %d \n", RecvLowROS.footForce[0],
            //                                     RecvLowROS.footForce[1],
            //                                     RecvLowROS.footForce[2],
            //                                     RecvLowROS.footForce[3]);
            // printf("foot force est %d %d %d %d \n", RecvLowROS.footForceEst[0],
            //                                     RecvLowROS.footForceEst[1],
            //                                     RecvLowROS.footForceEst[2],
            //                                     RecvLowROS.footForceEst[3]);
            
            if( motiontime >= 0){
                // first, get record initial position
                if( motiontime >= 0 && motiontime < 10){
                    for (int i = 0; i < 12; i++) {
                        init_fbk_data.setAng(i, RecvLowROS.motorState[i].q);
                    }
                }
                // then run a stand up logic
                if( motiontime >= 10 && motiontime < 400){
                    // printf("%f %f %f\n", );
                    rate_count++;
                    double rate = rate_count/380.0;             
                    Kp[0] = 5.0; Kp[1] = 5.0; Kp[2] = 5.0;
                    Kd[0] = 1.0; Kd[1] = 1.0; Kd[2] = 1.0;
                    
                    for (int i = 0; i < 12; i++) {
                        cmd_data.setAng(i, jointLinearInterpolation(init_fbk_data.getMotorAng(i), init_cmd_data.getMotorAng(i), rate));
                    }
                }
                double sin_joint1, sin_joint2;
                // last, do sine wave
                if( motiontime >= 1700){
                    sin_count++;
                    sin_joint1 = 0.6 * sin(3*M_PI*sin_count/500.0);
                    sin_joint2 = -0.6 * sin(1.8*M_PI*sin_count/500.0);

                    cmd_data.setAng(FR_, init_cmd_data.getMotorAng(FR_,0),
                                         init_cmd_data.getMotorAng(FR_,1)+sin_joint1,
                                         init_cmd_data.getMotorAng(FR_,2)+sin_joint2);
                    cmd_data.setAng(FL_, init_cmd_data.getMotorAng(FL_,0),
                                         init_cmd_data.getMotorAng(FL_,1)-sin_joint1,
                                         init_cmd_data.getMotorAng(FL_,2)-sin_joint2);
                    cmd_data.setAng(RR_, init_cmd_data.getMotorAng(RR_,0),
                                         init_cmd_data.getMotorAng(RR_,1)-sin_joint1,
                                         init_cmd_data.getMotorAng(RR_,2)-sin_joint2);
                    cmd_data.setAng(RL_, init_cmd_data.getMotorAng(RL_,0),
                                         init_cmd_data.getMotorAng(RL_,1)+sin_joint1,
                                         init_cmd_data.getMotorAng(RL_,2)+sin_joint2);
                }
                cmd_data.printAng();

                // fill the send data structure
                for (int i = 0; i < 12; i++) {
                    SendLowROS.motorCmd[i].q = cmd_data.getMotorAng(i);
                    SendLowROS.motorCmd[i].dq = 0;
                    SendLowROS.motorCmd[i].Kp = Kp[i % 3];
                    SendLowROS.motorCmd[i].Kd = Kd[i % 3];                    
                }
                // sort of doing gravity compensation 
                SendLowROS.motorCmd[FR_0].tau = -0.65f;
                SendLowROS.motorCmd[FL_0].tau = 0.65f;
                SendLowROS.motorCmd[RR_0].tau = -0.65f;
                SendLowROS.motorCmd[RL_0].tau = 0.65f;

            }

        }

        SendLowLCM = ToLcm(SendLowROS, SendLowLCM);
        roslcm.Send(SendLowLCM);
        ros::spinOnce();
        loop_rate.sleep();

        count++;
        if(count > 10){
            count = 10;
            initiated_flag = true;
        }
    }
    return 0;
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "position_ros_mode");
    std::string firmwork;
    ros::param::get("/firmwork", firmwork);

    if(firmwork == "3_1"){
        aliengo::Control control(aliengo::LOWLEVEL);
        aliengo::LCM roslcm;
        mainHelper<aliengo::LowCmd, aliengo::LowState, aliengo::LCM>(argc, argv, roslcm);
    }
    else if(firmwork == "3_2"){
        std::string robot_name;
        UNITREE_LEGGED_SDK::LeggedType rname;
        ros::param::get("/robot_name", robot_name);
        if(strcasecmp(robot_name.c_str(), "A1") == 0)
            rname = UNITREE_LEGGED_SDK::LeggedType::A1;
        else if(strcasecmp(robot_name.c_str(), "Aliengo") == 0)
            rname = UNITREE_LEGGED_SDK::LeggedType::Aliengo;
            
        // UNITREE_LEGGED_SDK::Control control(rname, UNITREE_LEGGED_SDK::LOWLEVEL);
        UNITREE_LEGGED_SDK::InitEnvironment();
        UNITREE_LEGGED_SDK::LCM roslcm(LOWLEVEL);
        mainHelper<UNITREE_LEGGED_SDK::LowCmd, UNITREE_LEGGED_SDK::LowState, UNITREE_LEGGED_SDK::LCM>(argc, argv, roslcm);
    }
}