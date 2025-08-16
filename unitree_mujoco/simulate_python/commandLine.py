import time
import numpy as np

from threading import Thread
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher
from unitree_sdk2py_bridge import UnitreeSdk2Bridge
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.utils.crc import CRC


class UnitreeCLI:
    def __init__(self, default_state="stand"):
        self.Kp = [
            60, 60, 60, 100, 100, 40,      # legs
            60, 60, 60, 100, 100, 40,      # legs
            60, 40, 40,                   # waist
            40, 40, 40, 40,  40, 40, 40,  # arms
            40, 40, 40, 40,  40, 40, 40   # arms
        ]

        self.Kd = [
                1, 1, 1, 2, 1, 1,     # legs
                1, 1, 1, 2, 1, 1,     # legs
                1, 1, 1,              # waist
                1, 1, 1, 1, 1, 1, 1,  # arms
                1, 1, 1, 1, 1, 1, 1   # arms 
        ]
        
        
        self.cmd = unitree_hg_msg_dds__LowCmd_()
        self.cmd.mode_pr = 1
        self.mode_machine = 1
        
        for i in range(29):
            self.cmd.motor_cmd[i].mode = 1 # (PMSM) mode
            self.cmd.motor_cmd[i].q= 0.0
            self.cmd.motor_cmd[i].kp = 0.0
            self.cmd.motor_cmd[i].dq = 0.0
            self.cmd.motor_cmd[i].kd = 0.0
            self.cmd.motor_cmd[i].tau = 0.0

        self.default_state = default_state
        self.current_state = default_state

        self.low_cmd_pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.low_cmd_pub.Init()
        self.crc = CRC()
        self.direction = 1.0
        self.cnt = 0

    def stand(self):
        for i in range(29):
            self.cmd.motor_cmd[i].q = 0.0 
            self.cmd.motor_cmd[i].kp = self.Kp[i]
            self.cmd.motor_cmd[i].dq = 0.0 
            self.cmd.motor_cmd[i].kd = self.Kd[i]
            self.cmd.motor_cmd[i].tau = 0.0 
        self.cmd.crc = self.crc.Crc(self.cmd)
        

    def shake(self):
        for i in range(29):
            if i == 23:

                self.cmd.motor_cmd[i].q = 0.5 * self.direction
                self.cmd.motor_cmd[i].kp = 10
                self.cmd.motor_cmd[i].dq = 0.0 
                self.cmd.motor_cmd[i].kd = self.Kd[i]
                self.cmd.motor_cmd[i].tau = 0.0
            else:
                self.cmd.motor_cmd[i].q = 0.0 
                self.cmd.motor_cmd[i].kp = self.Kp[i]
                self.cmd.motor_cmd[i].dq = 0.0 
                self.cmd.motor_cmd[i].kd = self.Kd[i]
                self.cmd.motor_cmd[i].tau = 0.0 
        self.cmd.crc = self.crc.Crc(self.cmd)
        if self.cnt == 200:
            self.direction *= -1
            self.cnt = 0
        self.cnt += 1

    def clap(self):
        for i in range(29):
            if i == 22:

                self.cmd.motor_cmd[i].q = 0.6 * self.direction
                self.cmd.motor_cmd[i].kp = self.Kp[i] / 10
                self.cmd.motor_cmd[i].dq = 0.0 
                self.cmd.motor_cmd[i].kd = self.Kd[i]
                self.cmd.motor_cmd[i].tau = 0.0
            # elif i == 24:
            #     cmd.motor_cmd[i].q = 0.75
            #     cmd.motor_cmd[i].kp = Kp[i] / 5
            #     cmd.motor_cmd[i].dq = 0.0 
            #     cmd.motor_cmd[i].kd = Kd[i]
            #     cmd.motor_cmd[i].tau = 0.0
            elif i == 15:
                self.cmd.motor_cmd[i].q = -0.6 *self.direction
                self.cmd.motor_cmd[i].kp = self.Kp[i] / 10
                self.cmd.motor_cmd[i].dq = 0.0 
                self.cmd.motor_cmd[i].kd = self.Kd[i]
                self.cmd.motor_cmd[i].tau = 0.0
            else:
                self.cmd.motor_cmd[i].q = 0.0 
                self.cmd.motor_cmd[i].kp = self.Kp[i]
                self.cmd.motor_cmd[i].dq = 0.0 
                self.cmd.motor_cmd[i].kd = self.Kd[i]
                self.cmd.motor_cmd[i].tau = 0.0 
        self.cmd.crc = self.crc.Crc(self.cmd)
        if self.cnt == 200:
            self.direction *= -1
            self.cnt = 0
        self.cnt += 1

    def wave(self):
        
        for i in range(29):
            if i == 21:

                self.cmd.motor_cmd[i].q = -3.0
                self.cmd.motor_cmd[i].kp = self.Kp[i] / 5
                self.cmd.motor_cmd[i].dq = 0.0 
                self.cmd.motor_cmd[i].kd = self.Kd[i]
                self.cmd.motor_cmd[i].tau = 0.0
            elif i == 24:
                self.cmd.motor_cmd[i].q = 0.25
                self.cmd.motor_cmd[i].kp = self.Kp[i] / 5
                self.cmd.motor_cmd[i].dq = 0.0 
                self.cmd.motor_cmd[i].kd = self.Kd[i]
                self.cmd.motor_cmd[i].tau = 0.0
            elif i == 20:
                self.cmd.motor_cmd[i].q = -1.5   
                self.cmd.motor_cmd[i].kp = self.Kp[i] / 10
                self.cmd.motor_cmd[i].dq = 0.0 
                self.cmd.motor_cmd[i].kd = self.Kd[i]
                self.cmd.motor_cmd[i].tau = 0.0
            elif i == 23:
                self.cmd.motor_cmd[i].q = 0.5 * self.direction
                self.cmd.motor_cmd[i].kp = 10
                self.cmd.motor_cmd[i].dq = 0.0 
                self.cmd.motor_cmd[i].kd = self.Kd[i]
                self.cmd.motor_cmd[i].tau = 0.0
            else:
                self.cmd.motor_cmd[i].q = 0.0 
                self.cmd.motor_cmd[i].kp = self.Kp[i]
                self.cmd.motor_cmd[i].dq = 0.0 
                self.cmd.motor_cmd[i].kd = self.Kd[i]
                self.cmd.motor_cmd[i].tau = 0.0 
        self.cmd.crc = self.crc.Crc(self.cmd)
        if self.cnt == 200:
            self.direction *= -1
            self.cnt = 0
        self.cnt += 1

    # -----------------
    # State loop
    # -----------------
    def run(self):
        while 1:
            # Always run the current state
            state_fn = getattr(self, self.current_state)
            state_fn()
            self.low_cmd_pub.Write(self.cmd)
            time.sleep(0.002)


    def menu_loop(self):
        """Blocking user menu for choosing states."""
        while 1:
            print("\n--- State Menu ---")
            print("1: Stand")
            print("2: Shake")
            print("3: Clap")
            print("4: Wave")
            print("q: Quit current state -> return to default")

            choice = input(f"Current [{self.current_state}] >> ").strip()

            if choice == "1":
                self.current_state = "stand"
                self.direction = 1.0
                self.cnt = 0
            elif choice == "2":
                self.current_state = "shake"
                self.direction = 1.0
                self.cnt = 0
            elif choice == "3":
                self.current_state = "clap"
                self.direction = 1.0
                self.cnt = 0
            elif choice == "4":
                self.current_state = "wave"
                self.direction = 1.0
                self.cnt = 0
            elif choice == "q":
                self.current_state = self.default_state
                self.direction = 1.0
                self.cnt = 0


def main():
    ChannelFactoryInitialize(0)
    # Ask user for settings
    
    default_state = "stand"

    cli = UnitreeCLI(default_state=default_state)

    control_thread = Thread(target= cli.run, daemon=True)
    control_thread.start()

    cli.menu_loop()

if __name__ == "__main__":
    main()
