import rospy
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode, ParamSet
from geometry_msgs.msg import PoseStamped, Pose, Twist, Point
from std_msgs.msg import String, Bool, Int16
from pyquaternion import Quaternion
import sys, time
# from warehouse_control import warehouse_open
from filter import FirstOrderFilter
import random

rospy.init_node("control_position_node")
rate = rospy.Rate(30)

class Control:

    def __init__(self):
        
        self.abs_position = {'take_off_position':[0,0],'qr':[0,5.08],'land_position_right':[1.6,0],'land_position_left':[-1.6,0],\
                             'r1':[1.6+random.random()*0.5, 1.98], 'r2':[1.6+random.random()*0.5,3.92+random.random()*0.5],'r3':[1.6+random.random()*0.5,5.88+random.random()*0.5],\
                             'l1':[-1.6+random.random()*0.5,1.98],'l2':[-1.6+random.random()*0.5,3.92+random.random()*0.5],'l3':[-1.6+random.random()*0.5,5.88+random.random()*0.5]}
        self.neirong_abs_position = {'qr_info':[1,3,8],'qr_down_dir':"l",'r1':8,'r2':7,'r3':1,'l1':5,'l2':3,'l3':2}
        
        self.flight_state = "init"
        self.last_flight_state = "init"
        self.target_motion = Pose()  # 目标时刻位置
        self.current_position = Point()
        self.flag = 1  # 标志位，让发给fastplanner的位置指令只发一次
        self.last_flag = 1
        self.start_time = 0
        self.end_time = 0
        self.flag_time = 1
        self.dx = 0
        self.dy = 0
        self.go_flag = "r3"
        self.last_go_flag = "init"
        self.vehicle_type = "iris"
        self.vehicle_id = "0"
        self.coordinate_frame = 1
        self.motion_type = 0
        self.hight = 1
        self.i = 0
        self.qr_data = PoseStamped()
        self.begin_scan_num = Bool()#开始检测数字的标志
        self.num_data = Int16()
        self.circle_centen_delta = Point()
        self.circle_count = 0
        self.circle_last = Point()
        self.vel_cmd = Twist()
        self.aim_flag1 = 1
        self.aim_flag2 = 2
        self.vel_cmd_z = Twist()
        self.test1 = 1
        self.last_cmd = String()
        self.pid_count = 0
        self.pid_finish = False
        self.final_flag = 'land_position_left'
        self.count = 0
        self.filter_x = FirstOrderFilter(0.15)
        self.filter_y = FirstOrderFilter(0.15)
        self.max_vel_x = 0.3
        self.max_vel_y = 0.3

        #PID参数
        kp,ki,kd =0.003,0,0.0001
        self.KP = kp
        self.KI = ki
        self.KD = kd
        self.err_x = 0
        self.err_y = 0
        self.l_err_x = 0
        self.l_err_y = 0
        self.sum_err_x = 0
        self.sum_err_y = 0
        self.fix_vel_x = 0
        self.fix_vel_y = 0  
            
        '''
        ros 订阅话题消息
        '''
        self.state_sub = rospy.Subscriber("/iris_0/mavros/state", State, self.state_callback, queue_size=1) # /iris_0
        self.local_pose_sub = rospy.Subscriber("/iris_0/mavros/local_position/pose", PoseStamped, self.local_pose_callback,queue_size=1)
        self.qr_data_sub = rospy.Subscriber("/qrcode", PoseStamped , self.qr_data_callback, queue_size=1)
        self.circle_centen_sub = rospy.Subscriber("/circle/point", Point, self.circle_centen_callback, queue_size=1)
        # self.numb_sub = rospy.Subscriber("/mnist/num", Int16, self.numb_sub_callback, queue_size=1)
        ''' 
        ros 发布话题消息
        '''
        self.last_state_cmd = rospy.Publisher("/xtdrone/"+self.vehicle_type+'_'+self.vehicle_id+"/cmd", String, queue_size=1)
        #self.target_motion_pub = rospy.Publisher(self.vehicle_type+'_'+self.vehicle_id+"/mavros/setpoint_raw/local", PositionTarget, queue_size=1)
        self.cmd_pose_enu_pub = rospy.Publisher("/xtdrone/"+self.vehicle_type+'_'+self.vehicle_id+"/cmd_pose_enu", Pose, queue_size=1)
        self.cmd_vel_enu_pub = rospy.Publisher("/xtdrone/"+self.vehicle_type+'_'+self.vehicle_id+"/cmd_vel_enu", Twist, queue_size=1)
        self.begin_scan_num_pub = rospy.Publisher("/num/activate", Bool,queue_size=1)
        #self.point_pub = rospy.Publisher("/my_simple/goal", PoseStamped, queue_size=1)
        #self.target_motion_pub.publish(self.target_motion)
        '''
        ros 服务通信
        '''
        #self.armService = rospy.ServiceProxy("iris_0"+"/mavros/cmd/arming", CommandBool)
        #self.flightModeService = rospy.ServiceProxy("iris_0"+"/mavros/set_mode", SetMode)

        print("话题、服务初始化")

    def start(self):
        '''
        主 ROS 线程
        '''
        while not rospy.is_shutdown():
            self.flight()
            rate.sleep()
            
    def circle_centen_callback(self, msg):
        self.circle_centen_delta.x = self.filter_x.get_value(msg.x - 320)
        self.circle_centen_delta.y = self.filter_y.get_value(240 - msg.y) 
            
    
    def qr_data_callback(self,msg):
        print('获取二维码成功！！！！！！！！！！！！！！！！')
        self.neirong_abs_position['qr_info'][0] = msg.pose.orientation.x
        self.neirong_abs_position['qr_info'][1] = msg.pose.orientation.y
        self.neirong_abs_position['qr_info'][2] = msg.pose.orientation.z
        if msg.pose.orientation.w == 1:
            self.neirong_abs_position['qr_down_dir'] = "l"
        else:  
            self.neirong_abs_position['qr_down_dir'] = "r"
            
    # def numb_sub_callback(self,msg):
    #     self.neirong_abs_position[self.last_go_flag] = msg.data
        
    
    # 状态回调函数，用以读取当前状态值，并初始化无人机当前状态信息        
    def state_callback(self, msg):
        if msg.mode == "OFFBOARD":
            if self.flight_state == "init":
                
                self.dx = self.current_position.x
                self.dy = self.current_position.y
                #self.last_flight_state = self.flight_state
                self.flight_state = "take_off"
                print(self.flight_state)
                
                
    #位置回调函数，用以读取当前的位置信息
    def local_pose_callback(self, msg):
        self.current_position.x = msg.pose.position.x
        self.current_position.y = msg.pose.position.y
        self.current_position.z = msg.pose.position.z
        
    
    #检查函数，用以检查当前是否到达了目标点
    def judge_position(self,d=0.2):
        if abs(self.current_position.x - self.target_motion.position.x) < d \
            and abs(self.current_position.y - self.target_motion.position.y) < d \
            and abs(self.current_position.z - self.target_motion.position.z) < d and self.flag_time:
            self.count += 1
            if self.count == 100:
                self.count = 0
                return True

    #控制指令发布函数，用以构建指令信息等待发出
    def construct_target(self, x=0, y=0, z=0):
        target_raw_pose = Pose()
        target_raw_pose.position.x = x
        target_raw_pose.position.y = y
        target_raw_pose.position.z = z

        return target_raw_pose 

    def flight(self):
        if self.flight_state == "take_off":
            if self.flag == 1:
                self.target_motion = self.construct_target(x=0 + self.dx,y= 0 + self.dy,z= 0.5)
                self.cmd_pose_enu_pub.publish(self.target_motion)
                if self.judge_position():
                    self.flag=2
            elif self.flag == 2:
                self.target_motion = self.construct_target(x=0 + self.dx,y= 0 + self.dy,z= self.hight)
                self.cmd_pose_enu_pub.publish(self.target_motion)
                if self.judge_position():
                    self.last_flag = self.flag
                    self.flag=1
                    self.last_flight_state = self.flight_state
                    self.flight_state = "to_qr"
                    
        elif self.flight_state == "to_qr":
            print(self.flight_state)
            if self.flag == 1:
                self.target_motion = self.construct_target(x=1.5 + self.dx,y= 2 + self.dy,z= self.hight)
                self.cmd_pose_enu_pub.publish(self.target_motion)
                if self.judge_position():
                    self.flag=2
            elif self.flag == 2:
                self.target_motion = self.construct_target(x=1.5 + self.dx,y=4.6  + self.dy,z= self.hight)
                self.cmd_pose_enu_pub.publish(self.target_motion)
                if self.judge_position():
                    self.flag=3
            elif self.flag == 3:
                self.target_motion = self.construct_target(x=self.abs_position['qr'][0] + self.dx,y= self.abs_position['qr'][1] + self.dy,z= self.hight)
                self.cmd_pose_enu_pub.publish(self.target_motion)
                if self.judge_position():
                    self.flag=1
                    self.last_flight_state = self.flight_state
                    self.flight_state = "go_around"
                    
        elif self.flight_state == "go_around":
            print(self.go_flag, end=' ')
            print(self.flag, end=' ')
            if self.go_flag == "r1":
                print(self.flight_state)
                if self.flag == 1:
                    self.target_motion = self.construct_target(x=self.abs_position['r1'][0] + self.dx,y= self.abs_position['r1'][1] + self.dy,z= self.hight)
                    self.cmd_pose_enu_pub.publish(self.target_motion)
                    if self.judge_position():
                        self.last_go_flag = self.go_flag
                        self.go_flag = "l1"
                        self.last_flag = self.flag
                        self.flag=1
                        if self.aim_flag1%2:
                            self.pid_finish = False
                            self.flight_state = "PID_control"
                        if self.pid_finish == True:
                            self.find_munber()

            elif self.go_flag == "r2":
                print(self.flight_state)
                if self.flag == 1:
                    self.target_motion = self.construct_target(x=self.abs_position['r2'][0] + self.dx,y= self.abs_position['r2'][1] + self.dy,z= self.hight)
                    self.cmd_pose_enu_pub.publish(self.target_motion)
                    if self.judge_position():
                        self.last_flag = self.flag
                        self.flag=1
                        self.last_go_flag = self.go_flag
                        self.go_flag = "r1"
                        if self.aim_flag2%2:
                            self.pid_finish = False
                            self.flight_state = "PID_control"
                        if self.pid_finish == True:
                            self.find_munber()
                        
            elif self.go_flag == "r3":
                print(self.flight_state)
                if self.flag == 1:
                    self.target_motion = self.construct_target(x=self.abs_position['r3'][0] + self.dx,y= self.abs_position['r3'][1] + self.dy,z= self.hight)
                    self.cmd_pose_enu_pub.publish(self.target_motion)
                    if self.judge_position():
                        self.last_flag = self.flag
                        self.flag=1
                        self.last_go_flag = self.go_flag
                        self.go_flag = "r2"
                        if self.aim_flag1%2:
                            self.pid_finish = False
                            self.flight_state = "PID_control"
                        if self.pid_finish == True:
                            self.find_munber()
                        
            elif self.go_flag == "l3":
                print(self.flight_state)
                if self.flag == 1:
                    self.target_motion = self.construct_target(x=self.abs_position['l3'][0] + self.dx,y= self.abs_position['l3'][1] + self.dy,z= self.hight)
                    self.cmd_pose_enu_pub.publish(self.target_motion)
                    if self.judge_position():
                        self.last_flag = self.flag
                        self.flag=1
                        self.last_go_flag = self.go_flag
                        self.go_flag = "r3"
                        if self.aim_flag2%2:
                            self.pid_finish = False
                            self.flight_state = "PID_control"
                        if self.pid_finish == True:
                            self.find_munber()
                
            elif self.go_flag == "l2":
                print(self.flight_state)
                if self.flag == 1:
                    self.target_motion = self.construct_target(x=self.abs_position['l2'][0] + self.dx,y= self.abs_position['l2'][1] + self.dy,z= self.hight)
                    self.cmd_pose_enu_pub.publish(self.target_motion)
                    if self.judge_position():
                        self.last_flag = self.flag
                        self.flag=1
                        self.last_go_flag = self.go_flag
                        self.go_flag = "l3"
                        if self.aim_flag1%2:
                            self.pid_finish = False
                            self.flight_state = "PID_control"
                        if self.pid_finish == True:
                            self.find_munber()
                
            elif self.go_flag == "l1":
                print(self.flight_state)
                if self.flag == 1:
                    self.target_motion = self.construct_target(x=0.6 + self.dx,y= 1.5 + self.dy,z= self.hight)
                    self.cmd_pose_enu_pub.publish(self.target_motion)
                    if self.judge_position():
                        self.flag=2
                elif self.flag == 2:
                    self.target_motion = self.construct_target(x=self.abs_position['l1'][0] + self.dx,y= self.abs_position['l1'][1] + self.dy,z= self.hight)
                    self.cmd_pose_enu_pub.publish(self.target_motion)
                    if self.judge_position():
                        self.last_flag = self.flag
                        self.flag=1
                        self.last_go_flag = self.go_flag
                        self.go_flag = "l2"
                        if self.aim_flag2%2:
                            self.pid_finish = False
                            self.flight_state = "PID_control"
                        if self.pid_finish == True:
                            self.find_munber()
        
        elif self.flight_state == "throw":
            # time.sleep(2)
            # warehouse_open(self.i)
            # warehouse_open(self.i)
            # time.sleep(2)
            # warehouse_open(self.i)
            print(self.flight_state)
            if self.i < 3:
                self.flight_state = "go_around"
                self.go_flag = self.last_go_flag
                print("投放成功")
                if self.go_flag == "l1":
                    self.flag = 2
            else:
                self.flight_state = "land"
                self.flag = 1
                print("返航")

        elif self.flight_state == "PID_control":
            print(self.circle_centen_delta.x, self.circle_centen_delta.y )
            print("开始PID调试")
            if (abs(self.circle_centen_delta.x) < 10 and abs(self.circle_centen_delta.y) < 10) and self.pid_count > 60:
                print("PID调试完成")
                self.pid_count = 0
                self.pid_finish = True
                self.dx = self.current_position.x - self.abs_position[self.last_go_flag][0]
                self.dy = self.current_position.y - self.abs_position[self.last_go_flag][1]
                print("dx, dy: ", end='')
                print(self.dx, self.dy)
                self.aim_flag1 = self.aim_flag1+1
                self.aim_flag2 = self.aim_flag2+1
                self.flight_state = "go_around"
                self.go_flag = self.last_go_flag 
                if self.go_flag == "l1":
                        self.flag = 2
                print("即将返回go_around状态")
            else:
                self.reset_pid()
                self.pid_count += 1
                print(self.circle_centen_delta.x, self.circle_centen_delta.y)
                self.cmd_pid()
                self.construct_vel(self.fix_vel_x, self.fix_vel_y)
                self.cmd_vel_enu_pub.publish(self.vel_cmd)

            
                    
        elif self.flight_state == "land":
            print(self.flight_state)
            #右边位置在右边降落
            if self.last_go_flag in "r1r2r3" and self.neirong_abs_position['qr_down_dir'] == 'r':
                if self.flag == 1:
                    self.target_motion = self.construct_target(x = self.abs_position['land_position_right'][0]+ self.dx,y= self.abs_position['land_position_right'][1]+ self.dy,z= self.hight)
                    self.cmd_pose_enu_pub.publish(self.target_motion)
                    if self.judge_position():                        
                        self.pid_fun()
                        self.flag=2
                elif self.flag == 2:
                    self.target_motion = self.construct_target(x = self.abs_position['land_position_right'][0]+ self.dx,y= self.abs_position['land_position_right'][1]+ self.dy,z= 0)
                    self.cmd_pose_enu_pub.publish(self.target_motion)
                    self.construct_z_vel()
                    self.cmd_vel_enu_pub.publish(self.vel_cmd_z)
                    self.last_cmd ='AUTO.LAND'
                    self.last_state_cmd.publish(self.last_cmd)             
            #左边位置在左边降落
            elif self.last_go_flag in "l1l2l3" and self.neirong_abs_position['qr_down_dir'] == 'l':
                if self.flag == 1:
                    self.target_motion = self.construct_target(x = self.abs_position['land_position_left'][0]+ self.dx,y= self.abs_position['land_position_left'][1]+ self.dy,z= self.hight)
                    self.cmd_pose_enu_pub.publish(self.target_motion)
                    if self.judge_position():
                        self.flag=2
                        self.pid_fun()
                elif self.flag == 2:
                    self.target_motion = self.construct_target(x = self.abs_position['land_position_left'][0]+ self.dx,y= self.abs_position['land_position_left'][1]+ self.dy,z= 0)
                    self.cmd_pose_enu_pub.publish(self.target_motion)
                    self.construct_z_vel()
                    self.cmd_vel_enu_pub.publish(self.vel_cmd_z)
                    self.last_cmd ='AUTO.LAND'
                    self.last_state_cmd.publish(self.last_cmd) 
            #右边位置在左边降落
            elif self.last_go_flag in "r1r2r3" and self.neirong_abs_position['qr_down_dir'] == 'l':
                if self.flag == 1:
                    self.target_motion = self.construct_target(x = self.abs_position['land_position_right'][0]+ self.dx,y= self.abs_position['land_position_right'][1]+ self.dy,z= self.hight)
                    self.cmd_pose_enu_pub.publish(self.target_motion)
                    if self.judge_position():
                        time.sleep(2)
                        self.flag=2
                elif self.flag == 2:
                    self.target_motion = self.construct_target(x = self.abs_position['land_position_left'][0]+ self.dx,y= self.abs_position['land_position_left'][1]+ self.dy,z= self.hight)
                    self.cmd_pose_enu_pub.publish(self.target_motion)
                    if self.judge_position():
                        self.flag=3
                        self.pid_fun()
                elif self.flag == 3:
                    self.target_motion = self.construct_target(x = self.abs_position['land_position_left'][0]+ self.dx,y= self.abs_position['land_position_left'][1]+ self.dy,z= 0)
                    self.cmd_pose_enu_pub.publish(self.target_motion)
                    self.construct_z_vel()
                    self.cmd_vel_enu_pub.publish(self.vel_cmd_z)
                    self.last_cmd ='AUTO.LAND'
                    self.last_state_cmd.publish(self.last_cmd) 
            #左边位置右边降落
            elif self.last_go_flag in "l1l2l3" and self.neirong_abs_position['qr_down_dir'] == 'r':
                if self.flag == 1:
                    self.target_motion = self.construct_target(x = self.abs_position['land_position_left'][0]+ self.dx,y= self.abs_position['land_position_left'][1]+ self.dy,z= self.hight)
                    self.cmd_pose_enu_pub.publish(self.target_motion)
                    if self.judge_position():
                        self.flag=2
                        time.sleep(2)
                elif self.flag == 2:
                    self.target_motion = self.construct_target(x = self.abs_position['land_position_right'][0]+ self.dx,y= self.abs_position['land_position_right'][1]+ self.dy,z= self.hight)
                    self.cmd_pose_enu_pub.publish(self.target_motion)
                    if self.judge_position():
                        self.flag=3                        
                        self.pid_fun()
                elif self.flag == 3:
                    self.target_motion = self.construct_target(x = self.abs_position['land_position_right'][0]+ self.dx,y= self.abs_position['land_position_right'][1]+ self.dy,z= 0)
                    self.cmd_pose_enu_pub.publish(self.target_motion)
                    self.construct_z_vel()
                    self.cmd_vel_enu_pub.publish(self.vel_cmd_z)
                    self.last_cmd ='AUTO.LAND'
                    self.last_state_cmd.publish(self.last_cmd) 
       
    def begin_scan(self,state_begin = 'n'):
        if state_begin == 'y':
            self.begin_scan_num = True
        else:
            self.begin_scan_num = False
        self.begin_scan_num_pub.publish(self.begin_scan_num)
         
    def find_munber(self):
        time.sleep(2)
        # self.begin_scan(state_begin = 'y') 
        # self.begin_scan()
        print("开始检察数字，当前字典内容为：", end='')
        print(self.neirong_abs_position)
        if(self.test1==1) and (self.neirong_abs_position[self.last_go_flag] == self.neirong_abs_position["qr_info"][self.i]):
            print("发现正确数字")
            print(self.neirong_abs_position["qr_info"][self.i])
            self.i = self.i + 1 
            self.target_motion = self.construct_target(x = self.abs_position[self.last_go_flag][0]+ self.dx,y= self.abs_position[self.last_go_flag][1] + self.dy,z= 0.6)
            self.cmd_pose_enu_pub.publish(self.target_motion)
            time.sleep(2)
            print("开始降落")
            while not self.judge_position():
                self.target_motion = self.construct_target(x = self.abs_position[self.last_go_flag][0]+ self.dx,y= self.abs_position[self.last_go_flag][1] + self.dy,z= 0.6)
                self.cmd_pose_enu_pub.publish(self.target_motion)
            self.last_flight_state = self.flight_state
            print("开始投放物块")
            self.flight_state = "throw"  
            
    #PID调用函数
    def pid_fun(self):
        time.sleep(2)
        print(self.circle_centen_delta.x, self.circle_centen_delta.y )
        while (not (abs(self.circle_centen_delta.x) < 30 and abs(self.circle_centen_delta.y) < 30)) and self.pid_count < 50:
            print("开始PID调试")
            time.sleep(0.03)
            self.pid_count += 1
            print(self.circle_centen_delta.x, self.circle_centen_delta.y)
            self.cmd_pid()
            self.construct_vel(self.fix_vel_x, self.fix_vel_y)
            self.cmd_vel_enu_pub.publish(self.vel_cmd)
        print("PID调试完成")
        self.pid_count = 0
        if self.neirong_abs_position['qr_down_dir'] == 'r':
            self.final_flag = 'land_position_left'
        else:
            self.final_flag = 'land_position_right'
        self.dx = self.current_position.x - self.abs_position[self.final_flag][0]
        self.dy = self.current_position.y - self.abs_position[self.final_flag][1]
        self.reset_pid()

    def construct_vel(self, vx=0, vy=0):    
        self.vel_cmd.linear.x = vx
        self.vel_cmd.linear.y = vy

    def construct_z_vel(self, x=0,y=0,z=-0.2):
        self.vel_cmd_z.linear.x = x
        self.vel_cmd_z.linear.y = y
        self.vel_cmd_z.linear.z = z

    
    def cmd_pid(self):
        d_x = self.circle_centen_delta.x
        d_y = self.circle_centen_delta.y
        self.l_err_x = self.err_x
        self.l_err_y = self.err_y
        self.err_x = d_x
        self.err_y = d_y
        self.sum_err_x += self.err_x
        self.sum_err_y += self.err_y       
        self.fix_vel_x = self.KP * self.err_x + \
            self.KI * self.sum_err_x + self.KD * (self.err_x - self.l_err_x)
        self.fix_vel_y = self.KP * self.err_y + \
            self.KI * self.sum_err_y + self.KD * (self.err_y - self.l_err_y)
        if self.fix_vel_x > self.max_vel_x:
            self.fix_vel_x = self.max_vel_x
        if self.fix_vel_x < -self.max_vel_x:
            self.fix_vel_x = -self.max_vel_x
        if self.fix_vel_y > self.max_vel_y:
            self.fix_vel_y = self.max_vel_y
        if self.fix_vel_y < -self.max_vel_y:
            self.fix_vel_y = -self.max_vel_y
        
    
    #清除误差数据
    def reset_pid(self):
        self.err_x = 0
        self.err_y = 0
        self.l_err_x = 0
        self.l_err_y = 0
        self.sum_err_x = 0
        self.sum_err_y = 0
        self.fix_vel_x = 0
        self.fix_vel_y = 0
        self.filter_x.reset()
        self.filter_y.reset()
        

if __name__ == '__main__':
    communication = Control()
    print("初始化成功，即将开启程序")
    communication.start()
