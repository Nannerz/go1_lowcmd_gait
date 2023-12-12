/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "unitree_legged_sdk/go1_const.h"
#include <math.h>
#include <iostream>
#include <sstream>

using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:
    Custom(uint8_t level) : safe(LeggedType::Go1),
                            udp(level, 8090, "192.168.123.10", 8007)
    {
        udp.InitCmdData(cmd);
    }
    void UDPRecv();
    void UDPSend();
    void RobotControl();
    std::vector<float> swingLegs();
    void checkPowerProtect();
    void getCurrentPosition();
    void setStandingPosition();
    void getFootForces(double* footForces);
    std::vector<double> ik(double x, double y);
    std::vector<double> fk(double theta_thigh, double theta_calf);
    void genSwingTraj(double startx, double goalx, double starty, double height, std::vector<double>& calfAngles, std::vector<double>& thighAngles);
    void getLinearTraj(double startx, double goalx, double starty, double height, std::vector<double>& calfAngles, std::vector<double>& thighAngles);

    Safety safe;
    UDP udp;
    LowCmd cmd = {0};
    LowState state = {0};
    int Tpi = 0;
    int motiontime = 0;
    float dt = 0.002; // 0.001~0.01
    float swing_length = 250;

    double time_consume = 0;
    int rate_count = 0;
    int swing_count = 0;
    int swing_leg = 0;
    enum legStates
    {
        STANDING,
        SWINGING,
        DRAGGING,
        BALANCING
    };

    int all_legs[12] = {FR_0, RR_0, FL_0, RL_0, FR_1, RR_1, FL_1, RL_1, FR_2, RR_2, FL_2, RL_2};
    int hips[4] = {FR_0, FL_0, RR_0, RL_0};
    // 0 = FR, 1 = FL, 2 = RR, 3 = RL
    int thighs[4] = {FR_1, FL_1, RR_1, RL_1};
    int calves[4] = {FR_2, FL_2, RR_2, RL_2};
    double leg_max[3] = {go1_Hip_max - 0.05, go1_Thigh_max - 0.05, go1_Calf_max - 0.05};
    double leg_min[3] = {go1_Hip_min + 0.05, go1_Thigh_min + 0.05, go1_Calf_min + 0.05};

    float qInit[12] = {0};
    float qNext[12] = {0};
    float qCurrent[12] = {0};
    float qDes[12] = {0};
    float qStanding[12] = {0};
    float Kp[3] = {0};
    float Kd[3] = {0};
    const double LEG_LENGTH = 0.2;

    std::vector<std::vector<double>> thighTraj = std::vector<std::vector<double>>(4, std::vector<double>(swing_length, 0));
    std::vector<std::vector<double>> calfTraj = std::vector<std::vector<double>>(4, std::vector<double>(swing_length, 0));
    std::vector<std::vector<double>> prev_pos = std::vector<std::vector<double>>(4, std::vector<double>(2, 0));
    std::vector<std::vector<double>> pos_standing = std::vector<std::vector<double>>(4, std::vector<double>(2, 0));
    bool didonce = false;
    int states[4] = {0, 0, 0, 0};
};

void Custom::UDPRecv()
{
    udp.Recv();
}

void Custom::UDPSend()
{
    udp.Send();
}

double jointLinearInterpolation(double initPos, double targetPos, double rate)
{
    double p;
    rate = std::min(std::max(rate, 0.0), 1.0);
    p = initPos * (1 - rate) + targetPos * rate;
    return p;
}

void Custom::RobotControl()
{
    motiontime++;
    udp.GetRecv(state);

    setStandingPosition();

    if (motiontime >= 0)
    {
        // first, get record initial position
        // if( motiontime >= 100 && motiontime < 500){
        if (motiontime >= 0 && motiontime < 10)
        {
            // getCurrentPosition();
            for (int i=0; i<12; i++)
            {
                qInit[i] = qCurrent[i];
            }
        }
        // second, move to the origin point of a sine movement with Kp Kd
        if (motiontime >= 10 && motiontime < 1010)
        {
            rate_count++;
            double rate = rate_count / 1000.0; // needs count to 200

            Kp[0] = 5;
            Kp[1] = 36;
            Kp[2] = 59;
            Kd[0] = 2;
            Kd[1] = 2;
            Kd[2] = 2;

            for (int i : all_legs)
            {
                qNext[i] = jointLinearInterpolation(qInit[i], qStanding[i], rate);
            }
            for (int i=4; i<4; i++)
            {
                prev_pos[i] = fk(qNext[thighs[i]], qNext[calves[i]]);
            }
        }
        else if (motiontime == 1050)
        {
            for (int i=0; i<4; i++)
            {
                pos_standing[i] = fk(qNext[thighs[i]], qNext[calves[i]]);
            }
        }
        else if (motiontime > 1100)
        {
            Kp[1] = 90;
            Kp[0] = 90;
            Kp[2] = 33;
            Kd[0] = 4;
            Kd[1] = 4;
            Kd[2] = 4;

            // Set next join pos
            if (swing_count > 1 && int(fmod(swing_count,swing_length)) == 0)
                swing_leg = (swing_leg < 3) ? swing_leg + 1 : 0;

            swing_count++;

            std::vector<float> getSwing = swingLegs();
            for(int i=0; i<12; i++)
                qNext[i] = getSwing[i]; 
        }

        // Format command to send to robot
        for (int i : hips)
        {       
            cmd.motorCmd[i].q = qNext[i];
            cmd.motorCmd[i].dq = 0;
            cmd.motorCmd[i].Kp = Kp[0];
            cmd.motorCmd[i].Kd = Kd[0];
            // Opposite gravity for right and left hips
            if (i == FL_0 || i == RL_0)
                cmd.motorCmd[i].tau = -0.63f;
            else
                cmd.motorCmd[i].tau = 0.63f;
        }
        for (int i : thighs)
        {
            cmd.motorCmd[i].q = qNext[i];
            cmd.motorCmd[i].dq = 0;
            cmd.motorCmd[i].Kp = Kp[1];
            cmd.motorCmd[i].Kd = Kd[1];
            cmd.motorCmd[i].tau = 0;
        }
        for (int i : calves)
        {
            cmd.motorCmd[i].q = qNext[i];
            cmd.motorCmd[i].dq = 0;
            cmd.motorCmd[i].Kp = Kp[2];
            cmd.motorCmd[i].Kd = Kd[2];
            cmd.motorCmd[i].tau = 0;

            if (swing_leg == calves[i])
            {
                cmd.motorCmd[i].Kp = 8;
                cmd.motorCmd[i].Kd = 1;
            }
        }
    }

    checkPowerProtect();
    udp.SetSend(cmd);

    getCurrentPosition();
}

void Custom::checkPowerProtect()
{
    if (motiontime > 10)
    {
        // int res1 = safe.PowerProtect(cmd, state, 4);
        int res1 = safe.PowerProtect(cmd, state, 2);
        // You can uncomment it for position protection
        // int res2 = safe.PositionProtect(cmd, state, 10);
        if (res1 < 0)
            exit(-1);
    }
}

void Custom::getCurrentPosition()
{
    for (int i : all_legs)
    {
        qCurrent[i] = state.motorState[i].q;
    }
}

void Custom::setStandingPosition()
{
    for (int i : hips)
    {
        if (i == FL_1 || i == RL_1)
            qStanding[i] = 0;
        else
            qStanding[i] = 0;
    }
    for (int i : thighs)
    {
        qStanding[i] = 0.65;
        // increase = backwards, towards butt
    }
    for (int i : calves)
    {
        qStanding[i] = -1.15;
        // toward 0 = open leg
    }
}

// Ikine and Fkine thanks to:
// https://github.com/katie-hughes/UnitreeGoGait/blob/main/gaitlib/src/gaits.cpp
std::vector<double> Custom::ik(double x, double y)
{
    double alpha = acos(sqrt(x * x + y * y) / (2 * LEG_LENGTH));
    double gamma = atan(x / y);
    double theta_thigh = gamma + alpha;
    double theta_calf = asin(-x / LEG_LENGTH - sin(theta_thigh)) - theta_thigh;

    std::vector<double> angles(2, 0);
    angles[0] = theta_calf;
    angles[1] = theta_thigh;

    return angles;
}

std::vector<double> Custom::fk(double theta_thigh, double theta_calf)
{
    const double fx = -LEG_LENGTH*(sin(theta_thigh) + sin(theta_calf + theta_thigh));
    const double fy = -LEG_LENGTH*(cos(theta_thigh) + cos(theta_calf + theta_thigh));
    std::vector<double> pos{fx, fy};
    return pos;
}

void Custom::genSwingTraj(double startx, double endx, double starty, double height, std::vector<double>& calfAngles, std::vector<double>& thighAngles)
{   
    std::vector<double> x(swing_length, 0);
    std::vector<double> y(swing_length, 0);
    std::vector<double> calf_angles(swing_length, 0);
    std::vector<double> thigh_angles(swing_length, 0);

    for (int i=0; i<2*swing_length/3; i++)
        x[i] = startx;
    
    for (int i=2*swing_length/3; i<swing_length; i++)
        x[i] = startx + (endx - startx)*i/swing_length;

    for (int i=0; i<swing_length/2; i++)
        y[i] = starty + (height*2*i /swing_length);

    for (int i=swing_length/2; i<swing_length; i++)
        y[i] = starty + height - (height*2*(i-swing_length/2) /swing_length); 
        
    for (int i=0; i<swing_length; i++)
    {
        std::vector<double> getIK = ik(x[i], y[i]);
        calf_angles[i] = getIK[0];
        thigh_angles[i] = getIK[1];
    }
    
    calfAngles = calf_angles;
    thighAngles = thigh_angles;
}

void Custom::getLinearTraj(double startx, double endx, double starty, double height, std::vector<double>& calfAngles, std::vector<double>& thighAngles)
{   
    std::vector<double> x(swing_length, 0);
    std::vector<double> y(swing_length, 0);
    std::vector<double> calf_angles(swing_length, 0);
    std::vector<double> thigh_angles(swing_length, 0);

    for (int i=0; i<swing_length; i++)
        x[i] = startx - abs((startx - endx))*i/swing_length;

    for (int i=0; i<swing_length; i++)
    {
        y[i] = starty - (height*i /swing_length);
    }
    
    y[swing_length - 1] = starty;
        
    for (int i=0; i<swing_length; i++)
    {
        std::vector<double> getIK  = ik(x[i], y[i]);
        calf_angles[i] = getIK[0];
        thigh_angles[i] = getIK[1];
    }
    
    calfAngles = calf_angles;
    thighAngles = thigh_angles;
}

std::vector<float> Custom::swingLegs()
{
    std::vector<float> nextPos(12, 0);
    double xgoal = 0.08;
    double gait_height = 0.13;
    int timestep = ceil(fmod(swing_count,swing_length));

    for (int i=0; i<4; i++)
    {
        if (timestep <= 1)
        {   
            std::vector<double> cur_pos = fk(qNext[thighs[i]], qNext[calves[i]]);
            if (swing_leg == i)
            {
                states[i] = 0;
                if (FR_1 == thighs[i] || FL_1 == thighs[i])
                    genSwingTraj(cur_pos[0], pos_standing[i][0] + xgoal, pos_standing[i][1], gait_height, calfTraj[i], thighTraj[i]);
                if (RR_1 == thighs[i] || RL_1 == thighs[i])
                    genSwingTraj(cur_pos[0], pos_standing[i][0], pos_standing[i][1], gait_height, calfTraj[i], thighTraj[i]);
                
            }
            else
            {
                states[i] += 1;
                if (FR_1 == thighs[i] || FL_1 == thighs[i])
                    getLinearTraj(cur_pos[0], pos_standing[i][0] + xgoal - states[i]*xgoal/3, cur_pos[1], 0, calfTraj[i], thighTraj[i]);
                if (RR_1 == thighs[i] || RL_1 == thighs[i])
                    getLinearTraj(cur_pos[0], pos_standing[i][0] - states[i]*xgoal/3, cur_pos[1], 0, calfTraj[i], thighTraj[i]);
            }
            prev_pos[i] = cur_pos;
        }
    }

    for (int i=0; i<4; i++)
    {
        nextPos[hips[i]] = qStanding[hips[i]];
        nextPos[calves[i]] = calfTraj[i][timestep];
        nextPos[thighs[i]] = thighTraj[i][timestep];
    }
    return nextPos;
}

int main(void)
{
    std::cout << "Communication level is set to LOW-level." << std::endl
              << "WARNING: Make sure the robot is hung up." << std::endl
              << "NOTE: The robot also needs to be set to LOW-level mode, otherwise it will make strange noises and this example will not run successfully! " << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    Custom custom(LOWLEVEL);
    LoopFunc loop_control("control_loop", custom.dt, boost::bind(&Custom::RobotControl, &custom));
    LoopFunc loop_udpSend("udp_send", custom.dt, 3, boost::bind(&Custom::UDPSend, &custom));
    LoopFunc loop_udpRecv("udp_recv", custom.dt, 3, boost::bind(&Custom::UDPRecv, &custom));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    while (1)
    {
        sleep(10);
    };

    return 0;
}
