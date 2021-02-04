#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "jlcxx/jlcxx.hpp"
#include "jlcxx/functions.hpp"
#include "jlcxx/stl.hpp"
using namespace UNITREE_LEGGED_SDK;

struct RobotInterface
{
public:
    RobotInterface() {
        // InitEnvironment();
    }
    void ReceiveObservation(){return;};
    // // LowState ReceiveObservation();
    void SendCommand(){return;};
    // void Initialize();

    // UDP udp;
    // Safety safe;
    // LowState state = {0};
    // LowCmd cmd = {0};
};

// void RobotInterface::ReceiveObservation() {
// // LowState RobotInterface::ReceiveObservation() {
//     // udp.Recv();
//     // udp.GetRecv(state);
//     // return state;
// }

// void RobotInterface::SendCommand() {
//     // cmd.levelFlag = 0xff;
//     // for (int motor_id = 0; motor_id < 12; motor_id++) {
//     //     cmd.motorCmd[motor_id].mode = 0x0A;
//     //     cmd.motorCmd[motor_id].q = motorcmd[motor_id * 5];
//     //     cmd.motorCmd[motor_id].Kp = motorcmd[motor_id * 5 + 1];
//     //     cmd.motorCmd[motor_id].dq = motorcmd[motor_id * 5 + 2];
//     //     cmd.motorCmd[motor_id].Kd = motorcmd[motor_id * 5 + 3];
//     //     cmd.motorCmd[motor_id].tau = motorcmd[motor_id * 5 + 4];
//     // }
//     // safe.PositionLimit(cmd);
//     // udp.SetSend(cmd);
//     // udp.Send();
// }




std::string doc()
{
   return "A1 Robot Julia Bindings";
}

struct MutableBits
{
  double a;
  double b;
};


struct World
{
  World(const std::string& message = "default hello") : msg(message){}
  void set(const std::string& msg) { this->msg = msg; }
  std::string greet() { return msg; }
  std::string msg;
  ~World() { std::cout << "Destroying World with message " << msg << std::endl; }
};

JLCXX_MODULE define_julia_module(jlcxx::Module& mod)
{
    struct Array { Array() {} double a;};
    mod.method("doc", &doc);
    // mod.map_type<MutableBits>("MutableBits");
    // mod.add_type<Array>("Array");
    mod.add_type<RobotInterface>("RobotInterface")
    .method("ReceiveObservation", &RobotInterface::ReceiveObservation)
    .method("SendCommand", &RobotInterface::SendCommand);
    mod.add_type<World>("World")
    .constructor<const std::string&>()
    .method("set", &World::set)
    .method("greet", &World::greet);
}
