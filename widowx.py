# WidowX.cpp - Library to control WidowX Robotic Arm of Trossen Robotics
# Created by Lenin Silva, June, 2020
#  
#  MIT License
# 
# Copyright (c) 2021 LeninSG21, heavily modified by downingbots
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

#########################################################
import math
import time
import copy
import DXL_servo

class WidowX(object):

    def __init__(self):

         self.DXL = DXL_servo.DXL_Servos()
         # constant Limits 
         limPi_2 = 181 * math.pi / 360.0
         lim5Pi_6 = 5 * math.pi / 6
         self.q2Lim = [-limPi_2, limPi_2]
         self.q3Lim = [-limPi_2, lim5Pi_6]
         self.q4Lim = [-11 * math.pi / 18, limPi_2]

         # The mounting of Q1 is off by approx 1/16 rotation 
         # when converting from REPLAB to BRIDGE

         # CONSTANT DEFINES
         self.MX_MAX_POSITION_VALUE = 4095
         self.Q1_ADJUST = (1 * (self.MX_MAX_POSITION_VALUE+1) / 32 + 28)
         self.AX12_MAX = 1023
         self.IDX_GAMMA   = 3
         self.IDX_ROT     = 4
         self.IDX_GRIPPER = 5
         self.GRIPPER_OPEN_POS = 512
         self.GRIPPER_OPEN = 1
         self.GRIPPER_CLOSED = 2
         self.GRIPPER_DOWN_ANGLE = (-(self.MX_MAX_POSITION_VALUE - 1) / self.MX_MAX_POSITION_VALUE) * (math.pi / 2.0)
         self.GRIPPER_DOWN_POSITION = self.angleToPosition(self.IDX_GAMMA, self.GRIPPER_DOWN_ANGLE)

         self.SERVO_DELTA = self.DXL.DXL_MOVING_STATUS_THRESHOLD
         self.SERVOCOUNT = 6 

         # Poses
         self.Center = [2048, 2048, 2048, 2048, 512, 512]
         self.Home   = [2033, 1698, 1448, 2336, 512, 512]
         # Rest      = [2048, 2048, 2048, 1024, 512, 512]
         self.Rest   = [2048, 1020, 1030, 2048, 512, 512]
         self.PickNDropHome    = [2048, 2048, 2048, 1024, 512, 512]
         self.PickNDropTrash   = [3072, 2048, 2048, 1024, 512, 512]


         # Dimensions based on Interbotix urdf measurements 
         self.L0 = 12.5
         self.L1 = 14.203
         self.L2 =  4.825 
         self.L3 = 14.203
         # self.L4 = 16      #  to end of gripper
         self.L4 = 14      #  to middle of gripper

         # calculations based on measurements
         self.D  = math.sqrt(math.pow(self.L1, 2) + math.pow(self.L2, 2))
         self.alpha = math.atan2(self.L1, self.L2) 
         self.xy_lim = 43.0
         self.z_lim_up = 52.0
         self.z_lim_down = 0.0
         self.xy_min = 10.5
         self.isRelaxed = 0  
         self.DEFAULT_TIME = 2000
         self.gamma_lim = math.pi / 2.0
         self.Kp = 60.0 / 127000 
         self.Kg = math.pi / 2 / 255000
         self.Ks = 1024.0 / 255000
         self.W  = [[],[],[],[],[],[]]
         self.max_delta = math.pi / 10

         # initialize global state
         # servo state:
         self.id = []
         self.current_angle = []
         self.current_position = []
         self.desired_position = []
         self.desired_angle = []
         self.next_position = []
         # x,y,z state:
         self.point = [None, None, None]
         self.speed_points = [None, None, None]
         self.IKdist = []  

         self.state = {}      # shared with other classes
         self.fully_open_closed = 0  # gripper position
         print("Rest:", self.Rest)
         for i in range(self.SERVOCOUNT):
           self.id.append(i + 1)
           if (i == 0):
             # self.current_position.append(self.Rest[i]-self.Q1_ADJUST)
             self.current_position.append(int(self.Rest[i]-self.Q1_ADJUST))
             # self.current_position.append(self.Rest[i])
           elif (i in [4,5]):
             self.current_position.append(self.Rest[i])
           else:
             self.current_position.append(self.Rest[i])
           self.desired_position.append(self.current_position[i])
           self.next_position.append(self.current_position[i])
           self.current_angle.append(self.positionToAngle(i, self.current_position[i]))
           self.desired_angle.append(self.current_angle[i])
         self.torqueServos()
         print("setInitPos:", self.current_position)
         keep_trying = True
         while keep_trying:
           self.setDesiredPosition()
           self.delay(300)
           self.getCurrentPosition()
           keep_trying = False
           for i in range(self.SERVOCOUNT):
             if (abs(self.desired_position[i] - self.current_position[i]) > self.DXL.DXL_MOVING_STATUS_THRESHOLD):
               keep_trying = True
           print("Interim CP:", self.current_position, self.desired_position)
         print("Final CP:", self.current_position, self.desired_position)
         self.updatePoint()
         self.init_arm()

#########################################################
#    *** PUBLIC FUNCTIONS ***


    #  This is NOT a required function in order to work properly. It sends the arm 
    #  to rest position. It also checks the voltage with the function checkVoltage(). 
    #  If relax != 0, then the torque is disabled after it reaches the rest position.
    def delay(self, tm):
        time.sleep(0.001 * tm)

    def ping_all(self):
        for i in range(self.SERVOCOUNT):
            self.DXL.DXL_Ping(self.id[i])

    def init_arm(self, relax=False):
        self.ping_all()
        self.checkVoltage()
        # self.getCurrentPosition(self.SERVOCOUNT)
        self.moveRest()
        self.openCloseGrip(self.GRIPPER_OPEN)
        self.updatePoint()
        time.sleep(1)
        if (relax):
            self.relaxServos()

    # ID HANDLERS
    #  Sets the id of the specified motor by idx
    def setId(self, idx, newID):
        if (idx < 0 or idx >= self.SERVOCOUNT):
            return
        self.id[idx] = newID

    #  Gets the id of the specified motor by idx.
    def getId(self, idx):
        return self.id[idx]

    #  Preloaded Poses:
    #  Moves to the arm to the center of all motors, forming an upside L. As seen from 
    #  the kinematic analysis done for this library, it would be equal to setting all
    #  articular values to 0 deg.  Reads the pose from PROGMEM. Updates point once it is done
    def moveCenter(self):
        print("CENTER")
        self.getCurrentPosition(self.SERVOCOUNT)
        self.interpolateFromPose(self.Center, self.DEFAULT_TIME)
        self.delay(300)
        self.updatePoint()

    #  Moves to the arm to the home position as defined by the bioloid controller. 
    #  Reads the pose from PROGMEM. Updates point once it is done
    def moveHome(self):
        print("HOME")
        self.getCurrentPosition(self.SERVOCOUNT)
        self.interpolateFromPose(self.Home, self.DEFAULT_TIME)
        self.delay(300)
        self.updatePoint()

    #  Moves to the arm to the rest position, which is when the arm is resting over itself. 
    #  Reads the pose from PROGMEM. Updates point once it is done
    def moveRest(self):
        print("REST")
        self.getCurrentPosition(self.SERVOCOUNT)
        self.interpolateFromPose(self.Rest, self.DEFAULT_TIME)
        self.delay(300)
        self.updatePoint()

    def moveToPose(self,pose):
        self.getCurrentPosition(self.SERVOCOUNT)
        self.interpolateFromPose(pose, self.DEFAULT_TIME)
        self.delay(300)
        self.updatePoint()

    # Get Information
    # 
    #  Checks that the voltage values are adequate for the robotic arm. If it is 
    #  below 10V, it remains in a loop until the voltage increases. This is to prevent
    #  damage to the arm. Also, it sends through the serial port of the ArbotiX some 
    #  information about the voltage, which can be then seen by the serial monitor or
    #   received with another interface connected to the ArbotiX serial
    def checkVoltage(self):
        voltage = []
        #  wait, then check the voltage (LiPO safety)
        min_v = 10000
        for i in range(self.SERVOCOUNT):
          voltage.append((self.DXL.DXL_GetVoltage(self.id[i])) / 10.0)
          if min_v > voltage[i]:
            min_v = voltage[i]
          # self.delay(3)
        print("Volts:", voltage)
        # print("Min Volts:", min_v)
        while (voltage[1] <= 10.0):
          print("LOW VOLTAGE WARNING!")
          voltage[1] = (self.DXL.DXL_GetVoltage(self.id[i]) / 10.0)


    def setDesiredPosition(self, until_idx=None):
        if until_idx is None:
          until_index = self.SERVOCOUNT
        else:
          until_index = until_idx
        if until_index >= 4:
          self.DXL.DXL_SyncSetPosition(4, self.desired_position)
          if until_index >= 5:
            self.SetPosition2(5, self.desired_position[4])
          if until_index >= 6:
            self.SetPosition2(6, self.desired_position[5])
        else:
          for i in range(until_index):
            id = i + 1
            self.SetPosition2(id, self.desired_position[i])

    #  This function calls getServoPosition for each of the motors in the arm
    def getCurrentPosition(self, until_idx=None):
        if until_idx is None:
          until_index = self.SERVOCOUNT
        else:
          until_index = until_idx
        if until_index >= 4:
          curpos = self.DXL.DXL_BulkGetPosition(4)
          if until_index >= 5:
            self.getServoPosition(4)  # idx 4 / id 5
          if until_index >= 6:
            self.getServoPosition(5)  # idx 5 / id 6
        else:
          for i in range(until_index):
            self.getServoPosition(i)

    # ALWAYS USE to SetPosition as a layer on top of ax12 SetPosition library.
    # The Position to servo id [0] is a lie.  Under the covers, it transparently adjusts 
    # the 0 position to reflect Arm mounting position errors/adjustments.
    # This way, the logic for angles is automatically taken into account and
    # servo id 0 can be treated like other servos.
    def SetPosition2(self, servo_id, pos):
        if (servo_id == self.id[0]):
          if (pos-self.Q1_ADJUST < 0 or pos-self.Q1_ADJUST > self.MX_MAX_POSITION_VALUE):
            #  Q1 is an MX-28
            print("Err: Bad Pos Q1")
            self.checkVoltage()
          else:
            self.DXL.DXL_SetPosition(servo_id, pos-self.Q1_ADJUST)
        elif (servo_id == 5 or servo_id == 6):
          #  Servos 5 & 6 are AX-12 with max pos (0-1023)
          self.DXL.DXL_SetPosition(servo_id, int(pos))
        else:
          self.DXL.DXL_SetPosition(servo_id, pos)
        # self.delay(9)

    #  This function calls the GetPosition function from the ax12.h library. However, 
    #  it was seen that in some cases the value returned was -1. Hence, it made the 
    #  arm to move drastically, which, represents a hazard to those around and the arm 
    #  itself. That is why this function checks if the returned value is -1. If it is, 
    #  reads the current position of the specified motor until the value is not -1. Also, 
    #  for every iteration, the delay between lectures varies from 5 to 45ms, to give time 
    #  to the motor to respond appropriately. Due to this check conditions, this function 
    #  is preferred over the readPose() function from the BioloidController.h library. 
    #  The problem is that it might cause a significant delay if the motors keep returning -1. 
    #  Nonetheless, it is better to have this delay than to risk the arm’s integrity. 
    def getServoPosition(self, idx):
        i = 0
        # self.delay(20)
        cp = self.DXL.DXL_GetPosition(self.id[idx])
        while ((cp == -1 or cp == 65535) and i <= 10):
          j = (i % 10) + 1
          i = i + 1
          self.delay(5 * j)
          cp = self.DXL.DXL_GetPosition(self.id[idx])
          if (cp == -1 or cp == 65535):
            print("Err: srvo, pos ", idx, cp)
            self.checkVoltage()
            return self.current_position[idx]  
        if (idx == 0):
          self.current_position[idx] = cp + self.Q1_ADJUST
        else:
          self.current_position[idx] = cp
        self.current_angle[idx] = self.positionToAngle(idx, self.current_position[idx])
        return self.current_position[idx]

    #  This function returns the current angle of the specified motor.
    #  It uses getServoPosition() to prevent failure from reading -1 in the current
    #  position
    def getServoAngle(self, idx):
        self.getServoPosition(idx)
        return self.current_angle[idx]
    
    #  Calls the private function updatePoint to load the current point and copies
    #  the values into the provided pointer
    def getPoint(self, get_state=False):
        self.updatePoint(get_state=False)
        return self.point
  
    
    # Torque
    #  This function disables the torque of all the servos and sets the global flag isRelaxed to true.
    #  WARNING: when using it be careful of the arm's position; otherwise it can be damaged if it 
    #  impacts too hard on the ground or with another object
    def relaxServos(self):
        for i in range(self.SERVOCOUNT):
            self.DXL.DXL_Relax(self.id[i])
            # self.delay(10)
        self.isRelaxed = 1
    
    #  This function enables the torque of every motor. It does not alter their current positions.
    def torqueServos(self):
        for i in range(self.SERVOCOUNT):
            self.DXL.DXL_TorqueOn(self.id[i])
            # self.delay(10)
        self.isRelaxed = 0
    
    # Move Servo
    #  Moves the specified motor (by its idx) to the desired angle in radians. It is important to understand
    #  the directions of turn of every motor as described in the documentation in order to select the
    #  appropriate angle
    def moveServo2Angle(self, idx, angle):
        if (idx < 0 or idx >= self.SERVOCOUNT):
            return
        pos = self.angleToPosition(idx, angle)
        self.moveServo2Position(idx, pos)
    
    #  Moves the specified motor (by its idx) to the desired position. It does not validate if the position is in the appropriate
    #  ranges, so be careful
    def moveServo2Position(self, idx, pos):
        if (idx < 0 or idx >= self.SERVOCOUNT):
            return
    
        curr = self.getServoPosition(idx)
        if (curr < pos):
            while (curr < pos):
                if (pos - curr >= 10):
                  curr += 10
                else:
                  curr += 1
                self.SetPosition2(self.id[idx], curr)
                # self.delay(5)
        else:
            while (curr > pos):
                if (curr - pos >= 10):
                  curr -= 10
                else:
                  curr -= 1
                self.SetPosition2(self.id[idx], curr)
                # self.delay(5)
        # self.delay(9)
    
    #  Closes or opens the gripper (Q6 | idx = 5): 
    #      close = 0 --> open, close = 1 --> close in steps of 10
    #  Use it inside a loop with a delay to control the smoothness of the turn. Ideal for
    #  movement with control or key that is being sent as long as it is pressed
    def moveGrip(self, open_close):
        posQ6 = self.getServoPosition(self.IDX_GRIPPER)
        if (open_close == self.GRIPPER_CLOSED):
            if (posQ6 > self.GRIPPER_OPEN_POS):
                posQ6 = (self.AX12_MAX - self.GRIPPER_OPEN_POS)
            if (posQ6 > 10):
                posQ6 -= 10
            else:
                posQ6 = 0
        elif (open_close == self.GRIPPER_OPEN):
            if (posQ6 > self.GRIPPER_OPEN_POS):
                posQ6 -= 10
            elif (posQ6 < self.GRIPPER_OPEN_POS):
                posQ6 += 10
                print("posQ6", posQ6, self.GRIPPER_OPEN_POS)
            else:
                posQ6 = self.GRIPPER_OPEN_POS

        # ARD: TODO: check temperature/voltage if closing
        # ARD: change to 0/1 if all closed or all open
        # ARD: return 0/0 otherwise
        self.SetPosition2(self.id[self.IDX_GRIPPER], posQ6)
        # self.delay(9)
    
        # int load = GetLoad(self.id[self.IDX_GRIPPER])
        # int voltage = GetVoltage(self.id[self.IDX_GRIPPER])
        # int temperature = GetTemperature(self.id[self.IDX_GRIPPER])
        # int ax12err = ax12GetLastError(self.id[self.IDX_GRIPPER])
    
        if (posQ6 >= self.GRIPPER_OPEN_POS-self.SERVO_DELTA):
          self.fully_open_closed = self.GRIPPER_OPEN
        elif (posQ6 <= self.SERVO_DELTA):
          self.fully_open_closed = self.GRIPPER_CLOSED
        else:
          self.fully_open_closed = 0
        return self.fully_open_closed
    
    def openCloseGrip(self, open_close):
        if (open_close != self.GRIPPER_OPEN and open_close != self.GRIPPER_CLOSED):
          return open_close
        while (self.fully_open_closed == 0):
          self.fully_open_closed = self.moveGrip(open_close)
          self.delay(5)
        # self.delay(300)
        return self.fully_open_closed
    
    #  Sets the specified servo to the given position without a smooth tansition.
    #  Designed to be used with a controller.
    def setServo2Position(self, idx, position):
        self.SetPosition2(self.id[idx], position)
    
    def millis(self):
        return round(time.time() * 1000)

    def moveServoWithSpeed(self, idx, speed, initial_time):
        tf = self.millis() - initial_time
        lim_up = self.AX12_MAX
        if (idx < 4): # MX-28 | MX_64
            lim_up = self.MX_MAX_POSITION_VALUE
        self.current_position[idx] = max(0, min(lim_up, self.current_position[idx] + speed * self.Ks * tf))
        self.SetPosition2(self.id[idx], round(self.current_position[idx]))
    
    # Move Arm
    #   This function is designed to work with a joystick or controller. It moves the origin 
    #   of the grippers coordinate system. To move the arm with speed control, use this 
    #   function inside a loop. At the beginning of the loop, the initial_time is 
    #   set—for example, with the function millis() in Arduino. 
    #   The function receives four different velocities:
    #   •        vx: velocity to move the x-coordinate as seen from the base of the robot
    #   •        vy:  velocity to move the y-coordinate as seen from the base of the robot
    #   •        vz:  velocity to move the z-coordinate as seen from the base of the robot
    #   •        vg:  velocity to move the angle of the gripper as seen from the base of the robot
    #   This function was designed considering that speed values range from [-127,127] for 
    #   vx, vy and vz and [-255,255] for vg. Higher values are mathematically possible, 
    #   but will make the arm move faster, so be cautious. This function offers a 
    #   better controlling experience for machines.
    def movePointWithSpeed(self, vx, vy, vz, vg, initial_time):
        tf = self.millis() - initial_time
        self.speed_points[0] = max(-self.xy_lim, min(self.xy_lim, self.speed_points[0] + float(vx) * self.Kp * tf))
        self.speed_points[1] = max(-self.xy_lim, min(self.xy_lim, self.speed_points[1] + float(vy) * self.Kp * tf))
        self.speed_points[2] = max(self.z_lim_down, min(self.z_lim_up, self.speed_points[2] + float(vz) * self.Kp * tf))
        self.global_gamma = max(-self.gamma_lim, min(self.gamma_lim, self.global_gamma + float(vg) * self.Kg * tf))
        self.setArmGamma(self.speed_points[0], self.speed_points[1], self.speed_points[2], self.global_gamma)
    
    #  interpolation currently used with Controller or Joystick.
    def movePointWithSpeed(self, vx, vy, vz, vg, initial_time):
        tf = self.millis() - initial_time
        too_close = 0   # too close to the base
        
        if (math.sqrt(math.pow(self.speed_points[0],2) + math.pow(self.speed_points[1],2)) < self.xy_min):
          too_close = 1    
        if ((too_close == 0) or (too_close == 1 and vx * self.speed_points[0] > 0)):
          # if not too close to base 
          # or (too close and trying to move away from base)
          self.speed_points[0] = max(-self.xy_lim, min(self.xy_lim, self.speed_points[0] + vx))
        if ((too_close == 0) or (too_close == 1 and vy * self.speed_points[1] > 0)):
          # vy and speedpoints both pos or both neg
          self.speed_points[1] = max(-self.xy_lim, min(self.xy_lim, self.speed_points[1] + vy ))
        self.speed_points[2] = max(self.z_lim_down, min(self.z_lim_up, self.speed_points[2] + vz))
    
        self.global_gamma = max(-self.gamma_lim, min(self.gamma_lim, self.global_gamma + vg * self.Kg * tf))
        self.max_delta = abs(vx) + abs(vy) + abs(vz) + abs(vg)
        self.moveArmGammaController(self.speed_points[0], self.speed_points[1], self.speed_points[2], -254.0)
    
    #   This function also moves the arm when controlled with a joystick or controller, 
    #   just as movePointWithSpeed(). However, while the other function moves according 
    #   to the desired translation and rotation of the origin of the grippers coordinate 
    #   system, this function moves the arm to the front or back with vx, it turns it 
    #   around with vy and with vy it goes up and down. This function was designed 
    #   considering that speed values range from [-127,127] for vx, vy and vz and 
    #   [-255,255] for vg. Higher values are mathematically possible, but will make 
    #   the arm move faster, so be cautious. This function offers a better controlling 
    #   experience for human operators.
    def moveArmWithSpeed(self, vx, vy, vz, vg, initial_time):
        tf = self.millis() - initial_time
    
        theta_0 = math.atan2(self.speed_points[1], self.speed_points[0])
        delta_theta = 4 * vy * self.Kg * tf
    
        magnitude_U0 = math.sqrt(math.pow(self.speed_points[0], 2) + math.pow(self.speed_points[1], 2))
        deltaU = vx * self.Kp * tf
    
        magnitude_Uf = magnitude_U0 + deltaU
        theta_f = theta_0 + delta_theta
        self.speed_points[0] = magnitude_Uf * math.cos(theta_f)
        self.speed_points[1] = magnitude_Uf * math.sin(theta_f)
        self.speed_points[2] = max(self.z_lim_down, min(self.z_lim_up, self.speed_points[2] + vz * self.Kp * tf))
        self.global_gamma = max(-self.gamma_lim, min(self.gamma_lim, self.global_gamma + vg * self.Kg * tf))
        setArmGamma(self.speed_points[0], self.speed_points[1], self.speed_points[2], self.global_gamma)
    
    #  Moves the center of the gripper to the specified coordinates Px, Py and Pz, as seen from the base of the robot, with the 
    #  desired angle gamma of the gripper. For example, gamma = pi/2 will make the arm a Pick N Drop since the gripper will be heading
    #  to the floor. It uses getIK_Gamma. This function only affects Q1, Q2, Q3, and Q4. 
    #  It interpolates the step using a cubic interpolation with the default time.
    #  If there is no solution for the IK, the arm does not move and a message is printed into the serial monitor.
    def moveArmPick(self, q1Angle = 0):
        print("PICK")
        if (self.isRelaxed):
            self.torqueServos()
        self.getCurrentPosition()
        self.moveCenter()
        # self.delay(1000)
        self.getCurrentPosition()
    
        # point gripper down
        # gamma = (self.AX12_MAX + math.pow(-1,g%2) * float(g))
        gamma = -(self.MX_MAX_POSITION_VALUE - 1)
        wristAngle = (gamma / self.MX_MAX_POSITION_VALUE) * (math.pi / 2.0)
        self.moveServo2Angle(3, wristAngle)
    
        # swivel base
        self.moveServo2Angle(0, q1Angle)
        self.updatePoint()
        return
    
    def moveArmGammaController(self, Px, Py, Pz, gamma):
        # check if nothing to do 
        if (Px == self.state['X'] and Py == self.state['Y'] and
            Pz == self.state['Z'] and gamma == self.state['Gamma']):
            return # Do nothing!
        # check for simple swivel solution 
        elif (Px != self.state['X'] and Py != self.state['Y'] and
            Pz == self.state['Z'] and gamma == self.state['Gamma']):
            old_radius = math.sqrt(math.pow(self.state['X'],2) + math.pow(self.state['Y'],2))
            new_radius = math.sqrt(math.pow(Px,2) + math.pow(Py,2))
            if (old_radius - new_radius < 30) :
              q1 = math.atan2(Py,Px)
              pos = self.angleToPosition(0, q1)
              self.SetPosition2(self.id[0], pos)
              # self.delay(300)
              return
             
        bestGammaIKdist  = [100000000]
        closestIKdist  = [100000000]
        prevIKdist  = [100000000]
        firstIKdist = [100000000]
        self.IKdist = [100000000]
        found = False
        for g in range(360):
            # -1.919 is gamma's lower limit
            gm2 = (self.MX_MAX_POSITION_VALUE + math.pow(-1,g%2) * float(g))
            wristAngle = -1 * (float(gm2) / self.MX_MAX_POSITION_VALUE) * math.pi / 2.0
            failure = self.getIK_Gamma_Controller(Px, Py, Pz, wristAngle)
            if (failure):
              if self.IKdist[0] < closestIKdist[0]:
                prevIKdist = copy.deepcopy(closestIKdist)
                closestIKdist = copy.deepcopy(self.IKdist)
                closestIKdist.append(g)
              if len(self.IKdist) > 6 and self.IKdist[6] - math.pi/2 < bestGammaIKdist[0] - math.pi/2:
                bestGammaIKdist = copy.deepcopy(self.IKdist)
                bestGammaIKdist.append(g)
              if firstIKdist[0] == 100000000:
                firstIKdist = copy.deepcopy(self.IKdist)
                firstIKdist.append(g)
              if g > 55:
                print("getIK_Gamma_Controller fails: %d %f" % (g, wristAngle))
            else:
              found = True
              self.interpolate(self.DEFAULT_TIME)  # sets the desired pos
              # self.delay(5000)
              self.updatePoint()
              break
        if not found and closestIKdist[0] != 100000000:
          bestIKdist = copy.deepcopy(bestGammaIKdist)
          print("getIK_Gamma_Controller: best IKdist   :", bestIKdist)
          print("getIK_Gamma_Controller: closest IKdist:", closestIKdist)
          print("getIK_Gamma_Controller: prev IKdist   :", prevIKdist)
          print("getIK_Gamma_Controller: 1rst IKdist   :", firstIKdist)
          print("getIK_Gamma_Controller: Gamma IKdist  :", bestGammaIKdist)
          self.desired_angle[0] = bestIKdist[1]
          self.desired_angle[1] = bestIKdist[2]
          self.desired_angle[2] = bestIKdist[3]
          self.desired_angle[3] = bestIKdist[4]
          self.interpolate(self.DEFAULT_TIME)
          self.updatePoint()
        return

    # ////////////////////////////////////////////////////////////////////////////////////
    #   *** PRIVATE FUNCTIONS ***
    
    def getState(self, s1="State:"):
        pt = self.getPoint()
        wrist_angle = self.getServoAngle(self.IDX_GAMMA)
        wrist_rot   = self.getServoAngle(self.IDX_ROT)
        posQ6       = self.getServoPosition(self.IDX_GRIPPER)
        if (posQ6 >= self.GRIPPER_OPEN_POS-self.SERVO_DELTA):
          self.fully_open_closed = self.GRIPPER_OPEN
        elif (posQ6 <= self.SERVO_DELTA):
          self.fully_open_closed = self.GRIPPER_CLOSED
        else:
          self.fully_open_closed = 0
        grip = self.fully_open_closed
    
        # self.state['seqnm'] = seqnm   # no longer required without arbotix
        self.state['X'] = pt[0]
        self.state['Y'] = pt[1]
        self.state['Z'] = pt[2]
        self.state['Gamma']   = wrist_angle
        self.state['Rot']     = wrist_rot
        self.state['Gripper'] = grip
    
        # self.delay(100)  #  must empty the serial buffer
        print(s1, self.state)
    
    # Conversions
    def positionToAngle(self, idx, position):
        if (idx == 0 or idx == 3 or idx == 2): # MX-28 or MX-64
              # 0 - 4095, 0.088°
              # 0° - 360°
              return 0.00153435538637 * (position - 2047.5)
        elif (idx == 1):
              # 0 - 4095, 0.088°
              # 0° - 360°
              return -0.00153435538637 * (position - 2047.5)
        else:  #AX-12
              # 0-1023
              # 0° - 300°
              # return 0.00511826979472 * (position - 511.5)
              # print("IDX, pos, angle", idx, position, (0.00511826979472 * (position - self.AX12_MAX)))
              return 0.00511826979472 * (position - (self.AX12_MAX/2))
    
    def angleToPosition(self, idx, angle):
        if (abs(angle) > math.pi):
            if (angle > 0):
                angle -= 2 * math.pi
            else:
                angle += 2 * math.pi
        if (idx == 0 or idx == 3 or idx == 2): # MX-28 or MX-64
            # 0 - 4095, 0.088°
            #  0° - 360°
            #  return (int)651.74 * angle + 2048
            return round(651.739492 * angle + 2047.5)
        elif (idx == 1): # MX-64
            # 0 - 4095, 0.088°
            #  0° - 360°
            return round(-651.739492 * angle + 2047.5)
        else: # AX-12
            # 0-1023
            # 0° - 300°
            return round(195.378524405 * angle + (self.AX12_MAX/2))
    
    # Poses and interpolation
    def updatePoint(self, get_state=True):
        q1 = self.getServoAngle(0)
        q2 = self.getServoAngle(1)
        q3 = self.getServoAngle(2)
        q4 = self.getServoAngle(3)
    
        #  phi2 projects the end-point down from X-Z space to X-Y space length
        phi2 = self.D * math.cos(self.alpha + q2) + self.L3 * math.cos(q2 + q3) + self.L4 * math.cos(q2 + q3 + q4)
        self.global_gamma = -q2 - q3 - q4
    
        self.point = [0.0,0.0,0.0]
        self.point[0] = math.cos(q1) * phi2
        self.point[1] = math.sin(q1) * phi2
        self.point[2] = self.L0 + self.D * math.sin(self.alpha + q2) + self.L3 * math.sin(q2 + q3) + self.L4 * math.sin(q2 + q3 + q4)
    
        #  sprintf(line,"pnts*100: %d %d %d", int(round(point[0]*100)), int(round(point[1]*100)), int(round(point[2]*100)))
        #  Serial.println(line)
        #  self.delay(5)
        self.speed_points[0] = self.point[0]
        self.speed_points[1] = self.point[1]
        self.speed_points[2] = self.point[2]
        if get_state:
          self.getState()
    
    #  For this cube interpolation, the initial time and both the initial and final
    #  speed are assumed to always be 0. Thus, when solving the inverse matrix and
    #  multiplying the matrix and the desired vector, a simple generalization is 
    #  obtained, where a0 = q0, a1 = 0, a2 = (qf-q0)*(3/tf^2), and a3 = (q0-qf)*(2/tf^3).
    #  This saves time by reducing the amount of operations needed. However, it won't
    #  work when the start and end velocities and the initial time are not zero. 
    #  For those scenarios, a function like 
    #  cubeInterpolation(float q0, float qf, float v0, float vf, int t0, int tf)
    #  should be made.
    def cubeInterpolation(self, q0, qf, i, time):
        tf_2_3 = 2 / math.pow(time, 3)
        tf_3_2 = 3 / math.pow(time, 2)
        self.W[i] = [0.0,0.0,0.0,0.0]
        self.W[i][0] = q0
        self.W[i][1] = 0
        self.W[i][2] = tf_3_2 * (qf - q0)
        self.W[i][3] = tf_2_3 * (q0 - qf)
    
    def interpolate(self, remTime):
        for i in range(self.SERVOCOUNT):
            self.desired_position[i] = self.angleToPosition(i, self.desired_angle[i])
            self.cubeInterpolation(self.current_position[i], self.desired_position[i], i, remTime)
    
        t0 = self.millis()
        currentTime = self.millis() - t0
    
        while (currentTime < remTime):
            curr_2 = math.pow(currentTime, 2)
            curr_3 = math.pow(currentTime, 3)
            for i in range(self.SERVOCOUNT-1):
                self.next_position[i] = round(self.W[i][0] + self.W[i][1] * currentTime + self.W[i][2] * curr_2 + self.W[i][3] * curr_3)
                self.SetPosition2(self.id[i], self.next_position[i])
                # self.delay(5)
            # self.delay(10)
            currentTime = self.millis() - t0
    
        cur_pos = 0
        for i in range(self.SERVOCOUNT-1):
          for numtry in range(3):
            cur_pos = self.getServoPosition(i)
            if (abs(int(self.desired_position[i]) - cur_pos) > self.SERVO_DELTA):
              self.SetPosition2(self.id[i], self.desired_position[i])
              # self.delay(5)
              # if (numtry > 0):
              #   print("i,des_pos,cur_pos: %d %d %d",i, desired_position[i], cur_pos)
            else:
              break
    
    def interpolateFromPose(self, pose, remTime):
        for i in range(self.SERVOCOUNT):
            # self.desired_position[i] = pgm_read_word_near(pose + i)
            self.desired_position[i] = pose[i]
            self.cubeInterpolation(self.current_position[i], self.desired_position[i], i, remTime)
    
        t0 = self.millis()
        currentTime = self.millis() - t0
        self.next_position = copy.deepcopy(self.current_position)
        while (currentTime < remTime):
            curr_2 = math.pow(currentTime, 2)
            curr_3 = math.pow(currentTime, 3)
            for i in range(self.SERVOCOUNT-1):
                self.next_position[i] = round(self.W[i][0] + self.W[i][1] * currentTime + self.W[i][2] * curr_2 + self.W[i][3] * curr_3)
                self.SetPosition2(self.id[i], self.next_position[i])
            # self.delay(10)
            currentTime = self.millis() - t0
        cur_pos = 0
        #  avoid resetting to same position b/c actually causes slippage due to weight.
        for i in range(self.SERVOCOUNT - 1):
          for numtry in range(3):
            cur_pos = self.getServoPosition(i)
            if (abs(int(self.desired_position[i]) - cur_pos) > self.SERVO_DELTA):
              self.SetPosition2(self.id[i], self.desired_position[i])
              # self.delay(9)
              # if (numtry > 0):
                # print("i,des_pos,cur_pos: ",i, self.desired_position[i], cur_pos)
            else:
              break
    
    #  Moves the center of the gripper to the specified coordinates Px, Py and Pz,
    #  as seen from the base of the robot, with the 
    #  desired angle gamma of the gripper. For example, gamma = pi/2 will make the 
    #  arm a Pick N Drop since the gripper will be heading
    #  to the floor. It uses getIK_Gamma. This function only affects Q1, Q2, Q3, and Q4. 
    #  It does not interpolate the step, since it is designed to be used by a controller 
    #  that will move the arm smoothly. If there is no solution for the IK, the arm does not move.
    def setArmGamma(self, Px, Py, Pz, gamma):
        if (self.isRelaxed):
            self.torqueServos()
        if (self.getIK_Gamma_Controller(Px, Py, Pz, gamma)):
            print("getIK_Gamma_Controller failed")
            self.updatePoint()
            return
        self.syncWrite(4)
    
    def syncWrite(self, numServos):
        dp = self.desired_position
        dp[0] -= self.Q1_ADJUST
        self.DXL.DXL_BulkSetPosition(4, dp)
    
    #  Inverse Kinematics:
    #  Obtains the IK with a desired angle gamma for the gripper. 
    #  Returns 0 if succeeds, returns 1 if fails
    def getIK_Gamma_Controller(self, Px, Py, Pz, gamma):
        # Calculate sine and cosine of gamma
        sg = math.sin(gamma) 
        cg = math.cos(gamma)
        # Obtain the desired point as seen from {1}
        X = math.sqrt(math.pow(Px, 2) + math.pow(Py, 2)) - self.L4 * cg
        # Z = Pz - self.L0 + self.L4 * sg
        Z = Pz - self.L0 - self.L4 * sg
        print("sp", self.speed_points, self.state, Px, Py, Pz, gamma)
    
        # Obtain q1
        q1 = math.atan2(Py, Px)
    
        # calculate condition for q3
        cq3 = (math.pow(X, 2) + math.pow(Z, 2) - math.pow(self.D, 2) - math.pow(self.L3, 2)) / (2 * self.D * self.L3)
        if (abs(cq3) > 1):
            print("cq3, X,Z:", cq3,X,Z)
            return 1
    
        q2a = [0.0,0.0]
        q3a = [0.0,0.0]
        q4a = [0.0,0.0]
        xa  = [0.0,0.0]
        ya  = [0.0,0.0]
        za  = [0.0,0.0]
        ga  = [0.0,0.0]
        da  = [0.0,0.0]
        for i in range(2):
            if (i == 0):
              q3a[i] = self.alpha + math.acos(cq3)
            else:
              q3a[i] = self.alpha - math.acos(cq3)
            q3a[i] = math.atan2(math.sin(q3a[i]), math.cos(q3a[i]))
            # Check q3 limits
            if (q3a[i] < self.q3Lim[0] or q3a[i] > self.q3Lim[1]):
              print("q3a", q3a[i], self.q3Lim)
              za[i] = -10000000
              continue
            c3 = math.cos(q3a[i])
            s3 = math.sin(q3a[i])
            a = self.D * math.cos(self.alpha) + self.L3 * c3
            b = self.D * math.sin(self.alpha) + self.L3 * s3
            print("x,z,q3,c3,s3,a,b", X,Z,q3a[i],c3,s3,a,b)
            q2a[i] = math.atan2(a * Z - b * X, a * X + b * Z)
            if (q2a[i] < self.q2Lim[0] or q2a[i] > self.q2Lim[1]):
              print("q2a", q2a[i], self.q2Lim)
              print("angles", self.current_angle)
              za[i] = -10000000
              continue
    
            # q4a[i] = -gamma - q2a[i] - q3a[i]
            q4a[i] = gamma - q2a[i] - q3a[i]
            if (q4a[i] < self.q4Lim[0] or q4a[i] > self.q4Lim[1]):
              print("cur", self.current_angle)
              print("q4a", q4a[i], self.q4Lim, gamma, q2a[i], q3a[i])
              za[i] = -10000000
              continue
    
            # Possible value for q2 and q4 exists. Check if close to requested values
            phi2 = self.D * math.cos(self.alpha + q2a[i]) + self.L3 * math.cos(q2a[i] + q3a[i]) + self.L4 * math.cos(q2a[i] + q3a[i] + q4a[i])
            #  phi2 = self.D * math.cos(self.alpha + q2a[i]) + self.L3 * math.cos(self.alpha + q2a[i] + q3a[i]) + self.L4 * math.cos(self.alpha + q2a[i] + q3a[i] + q4a[i])
            xa[i] = math.cos(q1) * phi2
            ya[i] = math.sin(q1) * phi2
            za[i] = self.L0 + self.D * math.sin(self.alpha + q2a[i]) + self.L3 * math.sin(q2a[i] + q3a[i]) + self.L4 * math.sin(q2a[i] + q3a[i] + q4a[i])
            ga[i] = q4a[i] - q2a[i] - q3a[i]
            # da[i] = math.sqrt(math.pow((xa[i]-Px),2) + math.pow((ya[i]-Py),2) + math.pow((za[i]-Pz),2) + math.pow((ga[i]-gamma),2))
            da[i] = math.sqrt(math.pow((xa[i]-Px),2) + math.pow((ya[i]-Py),2) + math.pow((za[i]-Pz),2)) 
    
        print("cur angle:", self.current_angle)
        print("new angle:", q1, q2a, q3a, q4a)
        print("new xyzgd:", xa, ya, za, ga, da)
        # Save articular values into the array that will set the next positions
        if (za[0] != -10000000 and (za[1] == -10000000 or (da[0] < da[1]))):
          i = 0
        elif (za[1] != -10000000):
          i = 1
        else:
          print("za:", za, da)
          return 1
        if (round(da[i]*1000) > round(self.max_delta*999)):
          print("da:", da, self.max_delta, i)
          self.IKdist = [da[i], q1, q2a[i], q3a[i], q4a[i], gamma, ga[i]]
          return 1
        self.desired_angle[0] = q1
        self.desired_angle[1] = q2a[i]
        self.desired_angle[2] = q3a[i]
        self.desired_angle[3] = q4a[i]
        return 0     # Returns with success
    
    def setLED(self, id, on_off):
        self.DXL.DXL_SetLED(id, on_off)
        self.delay(1000)
   
    #################################################################
