import time
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
# from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
# from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.utils.crc import CRC
# import config
# if config.ROBOT=="g1":
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_ as LowState_default
# else:
#     from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
#     from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
#     from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_ as LowState_default

def HighStateHandler(msg: SportModeState_):
    print("Position: ", msg.position)
    #print("Velocity: ", msg.velocity)


def LowStateHandler(msg: LowState_):
    print("IMU state: ", msg.imu_state)
    # print("motor[0] state: ", msg.motor_state[0])
Kp = [
    60, 60, 60, 100, 100, 40,      # legs
    60, 60, 60, 100, 100, 40,      # legs
    60, 40, 40,                   # waist
    40, 40, 40, 40,  40, 40, 40,  # arms
    40, 40, 40, 40,  40, 40, 40   # arms
]
Kd = [
    1, 1, 1, 2, 1, 1,     # legs
    1, 1, 1, 2, 1, 1,     # legs
    1, 1, 1,              # waist
    1, 1, 1, 1, 1, 1, 1,  # arms
    1, 1, 1, 1, 1, 1, 1   # arms 
]

if __name__ == "__main__":
    ChannelFactoryInitialize(0)
    # hight_state_suber = ChannelSubscriber("rt/sportmodestate", SportModeState_)
    # low_state_suber = ChannelSubscriber("rt/lowstate", LowState_)

    # hight_state_suber.Init(HighStaeteHandler, 10)
    # low_state_suber.Init(LowStateHandler, 10)

    low_cmd_puber = ChannelPublisher("rt/lowcmd", LowCmd_)
    low_cmd_puber.Init()
    crc = CRC()

    cmd = unitree_hg_msg_dds__LowCmd_()
    cmd.mode_pr = 1
    cmd.mode_machine = 1

    # cmd.head[0]=0xFE
    # cmd.head[1]=0xEF
    # cmd.level_flag = 0xFF
    # cmd.gpio = 0
    for i in range(29):
        cmd.motor_cmd[i].mode = 1 # (PMSM) mode
        cmd.motor_cmd[i].q= 0.0
        cmd.motor_cmd[i].kp = 0.0
        cmd.motor_cmd[i].dq = 0.0
        cmd.motor_cmd[i].kd = 0.0
        cmd.motor_cmd[i].tau = 0.0
    direction = 1.0
    cnt = 0
    while True:

        for i in range(29):
            if i == 22:

                cmd.motor_cmd[i].q = 0.6 * direction
                cmd.motor_cmd[i].kp = Kp[i] / 10
                cmd.motor_cmd[i].dq = 0.0 
                cmd.motor_cmd[i].kd = Kd[i]
                cmd.motor_cmd[i].tau = 0.0
            # elif i == 24:
            #     cmd.motor_cmd[i].q = 0.75
            #     cmd.motor_cmd[i].kp = Kp[i] / 5
            #     cmd.motor_cmd[i].dq = 0.0 
            #     cmd.motor_cmd[i].kd = Kd[i]
            #     cmd.motor_cmd[i].tau = 0.0
            elif i == 15:
                cmd.motor_cmd[i].q = -0.6 *direction
                cmd.motor_cmd[i].kp = Kp[i] / 10
                cmd.motor_cmd[i].dq = 0.0 
                cmd.motor_cmd[i].kd = Kd[i]
                cmd.motor_cmd[i].tau = 0.0


            else:
                cmd.motor_cmd[i].q = 0.0 
                cmd.motor_cmd[i].kp = Kp[i]
                cmd.motor_cmd[i].dq = 0.0 
                cmd.motor_cmd[i].kd = Kd[i]
                cmd.motor_cmd[i].tau = 0.0 

        
        cmd.crc = crc.Crc(cmd)

        #Publish message
        low_cmd_puber.Write(cmd)
        if cnt == 200:
            direction *= -1
            print("cmd msg log ", cmd)
            cnt = 0 
        cnt += 1
        time.sleep(0.002)
