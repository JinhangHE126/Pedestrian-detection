/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>

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

  Safety safe;
  UDP udp;
  LowCmd cmd = {0};
  LowState state = {0};
  int Tpi = 0;
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
  cmd.motorCmd[FR_0].tau = -0.65f;
  cmd.motorCmd[FL_0].tau = +0.65f;
  cmd.motorCmd[RR_0].tau = -0.65f;
  cmd.motorCmd[RL_0].tau = +0.65f;

  if (motiontime >= 500)
  {
    float speed = 2 * sin(3 * M_PI * Tpi / 1500.0);

    cmd.motorCmd[FR_1].q = PosStopF;//设置电机的角度指令为停止位置。这意味着机器人的正前右电机将停止旋转，并保持当前位置。
    cmd.motorCmd[FR_1].dq = speed;//设置电机的速度指令为之前计算得到的 ​speed​值。这意味着机器人的正前右电机将以指定的速度进行旋转。
    cmd.motorCmd[FR_1].Kp = 0;//设置位置环控制增益为0。位置环控制增益用于调节电机的位置控制稳定性，将其设置为0表示不对电机的位置进行控制。
    cmd.motorCmd[FR_1].Kd = 4;//设置速度环控制增益为4。速度环控制增益用于调节电机的速度控制响应和稳定性，将其设置为4表示对电机的速度进行较强的控制。
    cmd.motorCmd[FR_1].tau = 0.0f;//设置力矩指令为0。将力矩指令设为0表示不施加任何额外的力矩。
    Tpi++;
  }

  if (motiontime > 10)
  {
    int res1 = safe.PowerProtect(cmd, state, 1);
    // You can uncomment it for position protection
    // int res2 = safe.PositionProtect(cmd, state, 10);
    if (res1 < 0)
      exit(-1);
  }

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
