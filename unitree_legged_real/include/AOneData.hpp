#ifndef AONEDATA_H
#define AONEDATA_H

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>


class AOneData {
    public:
        AOneData() {
            ang.resize(LEG_NUM); vel.resize(LEG_NUM); tau.resize(LEG_NUM);
            for (int i = 0; i < LEG_NUM; i++) {
                ang[i].setZero(); vel[i].setZero(); tau[i].setZero();
            }
        }
        void setAng(int leg_idx, double q1, double q2, double q3) {ang[leg_idx](0) = q1;ang[leg_idx](1) = q2;ang[leg_idx](2) = q3;}
        void setVel(int leg_idx, double q1, double q2, double q3) {vel[leg_idx](0) = q1;vel[leg_idx](1) = q2;vel[leg_idx](2) = q3;}
        void setTau(int leg_idx, double q1, double q2, double q3) {tau[leg_idx](0) = q1;tau[leg_idx](1) = q2;tau[leg_idx](2) = q3;}
                
        void setAng(int motor_idx, double q) {int i = motor_idx / MTR_NUM; int j = motor_idx % MTR_NUM; ang[i](j) = q;}
        void setVel(int motor_idx, double q) {int i = motor_idx / MTR_NUM; int j = motor_idx % MTR_NUM; vel[i](j) = q;}
        void setTau(int motor_idx, double q) {int i = motor_idx / MTR_NUM; int j = motor_idx % MTR_NUM; tau[i](j) = q;}
     
        Eigen::Vector3d getLegAng(int leg_idx) {return ang[leg_idx];}
        Eigen::Vector3d getLegVel(int leg_idx) {return vel[leg_idx];}
        Eigen::Vector3d getLegTau(int leg_idx) {return tau[leg_idx];}        
        double getMotorAng(int leg_idx, int motor_idx) {return ang[leg_idx](motor_idx);}
        double getMotorVel(int leg_idx, int motor_idx) {return vel[leg_idx](motor_idx);}
        double getMotorTau(int leg_idx, int motor_idx) {return tau[leg_idx](motor_idx);}

        double getMotorAng(int motor_idx) {int i = motor_idx / MTR_NUM; int j = motor_idx % MTR_NUM; return ang[i](j);}
        double getMotorVel(int motor_idx) {int i = motor_idx / MTR_NUM; int j = motor_idx % MTR_NUM; return vel[i](j);}
        double getMotorTau(int motor_idx) {int i = motor_idx / MTR_NUM; int j = motor_idx % MTR_NUM; return tau[i](j);}

        void printAng() {Eigen::MatrixXd p(MTR_NUM, LEG_NUM); p << ang[0], ang[1], ang[2], ang[3]; std::cout << p << std::endl;}
     private:
        const int LEG_NUM = 4;
        const int MTR_NUM = 3;
        std::vector<Eigen::Vector3d> ang;
        std::vector<Eigen::Vector3d> vel;
        std::vector<Eigen::Vector3d> tau;
};

// class AOneData {
    
//     public:
//         AOneData() {
//             fbk_ang.resize(LEG_NUM); fbk_vel.resize(LEG_NUM); fbk_tau.resize(LEG_NUM);
//             cmd_ang.resize(LEG_NUM); cmd_vel.resize(LEG_NUM); cmd_tau.resize(LEG_NUM);
//             for (int i = 0; i < LEG_NUM; i++) {
//                 fbk_ang[i].setZero(); fbk_vel[i].setZero(); fbk_tau[i].setZero();
//                 cmd_ang[i].setZero(); cmd_vel[i].setZero(); cmd_tau[i].setZero();
//             }
//         }
//         void setFbkAng(int leg_idx, double q1, double q2, double q3) {fbk_ang[leg_idx](0) = q1;fbk_ang[leg_idx](1) = q2;fbk_ang[leg_idx](2) = q3;}
//         void setFbkVel(int leg_idx, double q1, double q2, double q3) {fbk_vel[leg_idx](0) = q1;fbk_vel[leg_idx](1) = q2;fbk_vel[leg_idx](2) = q3;}
//         void setFbkTau(int leg_idx, double q1, double q2, double q3) {fbk_tau[leg_idx](0) = q1;fbk_tau[leg_idx](1) = q2;fbk_tau[leg_idx](2) = q3;}
//         void setCmdAng(int leg_idx, double q1, double q2, double q3) {cmd_ang[leg_idx](0) = q1;cmd_ang[leg_idx](1) = q2;cmd_ang[leg_idx](2) = q3;}
//         void setCmdVel(int leg_idx, double q1, double q2, double q3) {cmd_vel[leg_idx](0) = q1;cmd_vel[leg_idx](1) = q2;cmd_vel[leg_idx](2) = q3;}
//         void setCmdTau(int leg_idx, double q1, double q2, double q3) {cmd_tau[leg_idx](0) = q1;cmd_tau[leg_idx](1) = q2;cmd_tau[leg_idx](2) = q3;}
        
//         void setFbkAng(int motor_idx, double q) {int i = motor_idx / MTR_NUM; int j = motor_idx % MTR_NUM; fbk_ang[i](j) = q;}
//         void setFbkVel(int motor_idx, double q) {int i = motor_idx / MTR_NUM; int j = motor_idx % MTR_NUM; fbk_vel[i](j) = q;}
//         void setFbkTau(int motor_idx, double q) {int i = motor_idx / MTR_NUM; int j = motor_idx % MTR_NUM; fbk_tau[i](j) = q;}
//         void setCmdAng(int motor_idx, double q) {int i = motor_idx / MTR_NUM; int j = motor_idx % MTR_NUM; cmd_ang[i](j) = q;}
//         void setCmdVel(int motor_idx, double q) {int i = motor_idx / MTR_NUM; int j = motor_idx % MTR_NUM; cmd_vel[i](j) = q;}
//         void setCmdTau(int motor_idx, double q) {int i = motor_idx / MTR_NUM; int j = motor_idx % MTR_NUM; cmd_tau[i](j) = q;}
//     private:
//         const int LEG_NUM = 4;
//         const int MTR_NUM = 3;
//         std::vector<Eigen::Vector3d> fbk_ang;
//         std::vector<Eigen::Vector3d> fbk_vel;
//         std::vector<Eigen::Vector3d> fbk_tau;

//         std::vector<Eigen::Vector3d> cmd_ang;
//         std::vector<Eigen::Vector3d> cmd_vel;
//         std::vector<Eigen::Vector3d> cmd_tau;
// };

#endif