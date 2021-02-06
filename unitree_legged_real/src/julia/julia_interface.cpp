
#include <thread>
#include <array>
#include <iostream>
#include <map>
#include <cmath>
#include <string>
#include <memory>

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "jlcxx/jlcxx.hpp"
#include "jlcxx/functions.hpp"
#include "jlcxx/stl.hpp"
using namespace UNITREE_LEGGED_SDK;

struct RobotInterface
{
public:
    RobotInterface() : safe(LeggedType::A1), udp(UNITREE_LEGGED_SDK::LOWLEVEL) {
    // RobotInterface()  {
        // InitEnvironment();
        thread_ = std::thread(&RobotInterface::ReceiveObservation, this);
        udp.InitCmdData(cmd);
    }
    ~RobotInterface() {
        thread_.join(); 
    }
    void ReceiveObservation();
    // void SendCommand(std::array<double,60> motorcmd);

    // function for Julia
    int16_t getFootForce(int idx){
        return state.footForce[idx];
    }

    MotorState getMotorState(int idx) {
        return state.motorState[idx];
    }

    UDP udp;
    Safety safe;
    LowState state = {0};
    LowCmd cmd = {0};
    std::thread thread_;
};

void RobotInterface::ReceiveObservation() {
    while(1) {
        sleep(0.01);
        // std::cout << udp.targetIP << std::endl;
        udp.Recv();
        // std::cout << "receive" << std::endl;
        udp.GetRecv(state);
        // std::cout << state.footForce[0] << std::endl;
        // std::cout << state.imu.accelerometer[0] << std::endl;
    };
}

// void RobotInterface::SendCommand(std::array<double,60> motorcmd) {
//     cmd.levelFlag = 0xff;
//     for (int motor_id = 0; motor_id < 12; motor_id++) {
//         cmd.motorCmd[motor_id].mode = 0x0A;
//         cmd.motorCmd[motor_id].q = motorcmd[motor_id * 5];
//         cmd.motorCmd[motor_id].Kp = motorcmd[motor_id * 5 + 1];
//         cmd.motorCmd[motor_id].dq = motorcmd[motor_id * 5 + 2];
//         cmd.motorCmd[motor_id].Kd = motorcmd[motor_id * 5 + 3];
//         cmd.motorCmd[motor_id].tau = motorcmd[motor_id * 5 + 4];
//     }
//     safe.PositionLimit(cmd);
//     udp.SetSend(cmd);
//     udp.Send();
// }




std::string doc()
{
   InitEnvironment();
   return "A1 Robot Julia Bindings";
}

// struct MutableBits
// {
//   double a;
//   double b;
// };


// struct World
// {
//   World(const std::string& message = "default hello") : msg(message){}
//   void set(const std::string& msg) { this->msg = msg; }
//   std::string greet() { return msg; }
//   std::string msg;
//   ~World() { std::cout << "Destroying World with message " << msg << std::endl; }
// };

namespace jlcxx
{
  template<> struct IsMirroredType<IMU> : std::true_type { };
  template<> struct IsMirroredType<MotorState> : std::true_type { };
  template<> struct IsMirroredType<MotorCmd> : std::true_type { };
//   template<> struct IsMirroredType<LowState> : std::false_type { };
//   template<> struct IsMirroredType<LowCmd> : std::true_type { };
}

JLCXX_MODULE define_julia_module(jlcxx::Module& mod)
{
    struct Array { Array() {} double a;};
    mod.method("doc", &doc);
    // map several types
    // mod.map_type<MutableBits>("MutableBits");
    mod.map_type<Cartesian>("Cartesian");
    mod.map_type<IMU>("IMU");
    mod.map_type<LED>("LED");
    mod.map_type<MotorState>("MotorState");
    mod.map_type<MotorCmd>("MotorCmd");
    // mod.map_type<LowState>("LowState");
    // mod.map_type<LowCmd>("LowCmd");


    mod.add_type<RobotInterface>("RobotInterface")
    .method("getFootForce", &RobotInterface::getFootForce)
    .method("getMotorState", &RobotInterface::getMotorState);
    // .method("ReceiveObservation", &RobotInterface::ReceiveObservation);
    // .method("ReceiveObservation", &RobotInterface::ReceiveObservation);
    // .method("SendCommand", &RobotInterface::SendCommand);
    // mod.add_type<World>("World")
    // .constructor<const std::string&>()
    // .method("set", &World::set)
    // .method("greet", &World::greet);
}
