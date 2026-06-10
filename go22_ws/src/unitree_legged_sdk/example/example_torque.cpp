/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <unistd.h>

using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:
  Custom(uint8_t level) : safe(LeggedType::Go1),
                          udp(level, 8090, "192.168.123.10", 8007)
  {
    udp.InitCmdData(cmd);
  }
  void UDPSend();
  void UDPRecv();
  void RobotControl();

  Safety safe;
  UDP udp;
  LowCmd cmd = {0};
  LowState state = {0};
  int motiontime = 0;
  float dt = 0.002; // 0.001~0.01
};

void Custom::UDPRecv()
{
  udp.Recv();
}

void Custom::UDPSend()
{
  udp.Send();
}

void Custom::RobotControl()
{
  motiontime++;
  udp.GetRecv(state);
  // gravity compensation
  cmd.motorCmd[FR_0].tau = -0.65f;//设置正前右（Front Right）电机的力矩为-0.65。
  cmd.motorCmd[FL_0].tau = +0.65f;//设置正前左（Front Left）电机的力矩为+0.65。
  cmd.motorCmd[RR_0].tau = -0.65f;//设置正后右（Rear Right）电机的力矩为-0.65。
  cmd.motorCmd[RL_0].tau = +0.65f;//设置正后左（Rear Left）电机的力矩为+0.65。

  if (motiontime >= 500)
  {
    //计算计算需要施加的力矩:根据 电机角度和速度 计算需要施加的 力矩。
    float torque = (0 - state.motorState[FR_1].q) * 10.0f + (0 - state.motorState[FR_1].dq) * 1.0f;
    //对力矩进行限制：如果力矩大于5.0，则将其限制为5.0。如果力矩小于-5.0，则将其限制为-5.0。
    if (torque > 5.0f)
      torque = 5.0f;
    if (torque < -5.0f)
      torque = -5.0f;

    cmd.motorCmd[FR_1].q = PosStopF;//设置正前右（Front Right）电机的角度指令为停止位置。
    cmd.motorCmd[FR_1].dq = VelStopF;//设置正前右（Front Right）电机的速度指令为停止速度。
    cmd.motorCmd[FR_1].Kp = 0;//设置正前右（Front Right）电机的位置环 控制增益为0。
    cmd.motorCmd[FR_1].Kd = 0;//设置正前右（Front Right）电机的速度环 控制增益为0。
    cmd.motorCmd[FR_1].tau = torque;//设置正前右（Front Right）电机的力矩指令为之前计算出的力矩值。
  }
  //进行安全保护处理：使用 ​safe.PowerProtect​函数对命令和状态进行安全保护处理，并将结果存储在 ​res​变量中。
  int res = safe.PowerProtect(cmd, state, 1);
  if (res < 0)
    //如果安全保护处理的结果小于0，表示出现错误，直接退出程序
    exit(-1);

  udp.SetSend(cmd);
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
