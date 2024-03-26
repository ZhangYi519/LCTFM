from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import subprocess
import random
import traci
import numpy as np
import random
import copy
import json

# we need to import python modules from the $SUMO_HOME/tools directory
try:
    sys.path.append(os.path.join(os.path.dirname(
        __file__), '..', '..', '..', '..', "tools"))  # tutorial in tests
    sys.path.append(os.path.join(os.environ.get("SUMO_HOME", os.path.join(
        os.path.dirname(__file__), "..", "..", "..")), "tools"))  # tutorial in docs
    from sumolib import checkBinary
except ImportError:
    sys.exit(
        "please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")

# the port used for communicating with your sumo instance
PORT = 8873


def generate_routefile():
    #random.seed(42)  # make tests reproducible
    #N = 3600  # number of time steps
    # demand per second from different directions
    #pCHV = 1. / 10
    #pCAV = 1. / 11
    #pUHV= 1. / 30
    with open("0322/0322.rou.xml", "w") as routes:
        print("""<routes>
        <vType id="HV" accel="5" decel="8" sigma="0.5" length="5" minGap="2.5" maxSpeed="30" 
        guiShape="passenger" color="Yellow" />
        <vType id="CAV" accel="5" decel="8" sigma="0.5" length="5" minGap="2.5" maxSpeed="30" 
        guiShape="passenger" color="red" />
        <vType id="Pseudo_HV" accel="5" decel="8" sigma="0.5" length="5" minGap="2.5" maxSpeed="30" 
        guiShape="passenger" color="Blue" RealVeh="Default" />

        <route id="right" edges="E3 E0 E1 E2" />

        """, file=routes)

        print("</routes>", file=routes)
def AddVehicle(VehicleKey,name):
    #This function is to add vehicle in to the system
    # VehicleKey is the dic of this vehicle
    # name is vehicle's name f'Vehicle{j}'
    vehtype=VehType[ str(VehicleKey['vehicle_type'][-1])]
    speed=VehicleKey['Speed'][-1]
    pos=VehicleKey['Position'][-1]
    targetln=VehicleKey['Lane_No'][-1]-1
    traci.vehicle.add(name, "right", typeID=vehtype, depart='now', departLane=targetln, departPos=pos, departSpeed=speed)

def RemoveVehicle(VehicleKey):
    # Remove this vehicle from SUMO
    traci.vehicle.remove(VehicleKey)

def StateUpdate(VehicleKey,name):
    speed = VehicleKey['Speed'][-1]
    traci.vehicle.setSpeed(name, speed)
    targetln = VehicleKey['Lane_No'][-1]-1
    traci.vehicle.changeLane(name, targetln, duration=0.1)
def LaneChange(VehicleKey,name):
    targetln = VehicleKey['Lane_No'][-1]
    traci.vehicle.changeLane(name, targetln, duration=1.5)

lc_decisionrate = 0.01

LaneSet={    "Lane1": 1,    "Lane2": 2,    "Lane3": 3}
VSL_All = {        "E0": None,        ":J1_0":None,        "E1": 13.9,        ":J3_0":None,        "E2": None}
VehType={"0":"HV", "1":"CAV", "2":"HV"}
ln_num=3 #   total lane number
t_sim=1#simulation interval is 1s
t_inflow_upd=5# inflow update per 5 min
T=2; s0=3; s_min=3;tau=5; gamma=0.6;b_safe=2;acc_delta=5;acc_bias=0;a_max=5;b_max=-5;b_sharp=-8
x_min=0;x_max=5000
lc_decisionrate=[0.01,0.01,0.01]
rou_set=[10, 20, 10] # traffic density rou_set for each lane: vehicle per kilometer
speed_set_CAV=[22.2,22.2,22.2]#   mean speed set for each lane: meter per sec
speed_set_HV=[22.2,22.2,22.2]
speed_var_CAV=[1,1,1]#   speed variance for each lane: meter per sec
speed_var_HV=[1,1,1]
flow=[200,200,200]#   traffic flow for each lane: vehicle per hour
#gen_number=[20,30,20]# generate how many vehicles in total each lane
gen_number=[int(item*(t_inflow_upd/60) )for item in flow]#flow*(t_inflow_upd/60)
P_cav=[0.5,0.5,0.5]# penatration rate of CAV
veh_length=[4.5,4.5,4.5]# average vehicle length on each lane
t_runtotal=3600# system total simulation time step number
#send_in_speed=13# the speed to put vehicles into the system

control_position=[2000,5000]# in meter
control_speed=[13.9,15]# in meter/s
free_speed=[22.2,22.2,22.2]# in meter/s
#Vehicle Generate

def TypeGenerator(numb_total,numb_cav):# Generate vehicle type HV0 CAV1
    vehicle_type=np.zeros(numb_total)
    vehicle_type=[int(a) for a in vehicle_type]
    # generate the cav sequence
    index_CAV=random.sample(range(numb_total),int(numb_cav))
    for i in index_CAV: vehicle_type[i]=1
    return vehicle_type,index_CAV
def SpeedGenerator(numb_total,index_CAV,speed_HV,speed_CAV):
    # Generate initial speed for each vehicle
    vehicle_meanspeed=speed_HV*np.ones(numb_total)
    for i in index_CAV: vehicle_meanspeed[i]=speed_CAV
    return vehicle_meanspeed
def SpeedVarGenerator(numb_total,index_CAV,speedvar_HV,speedvar_CAV):
    #Generate speed variance for each vehicle
    vehicle_speedvar = np.ones(numb_total)*speedvar_HV*random.uniform(-1,1)
    for i in index_CAV: vehicle_speedvar[i] = speedvar_CAV*random.uniform(-1,1)
    return vehicle_speedvar
def PosGenerator(numb_total,dist):
    # Generate position for each vehicle
    vehicle_position=np.arange(-10,-(numb_total*dist)-10,-(dist))
    return vehicle_position
def VehicleGenerator(ln_num,rou_set,gen_number,P_cav,speed_set_CAV,speed_set_HV,speed_var_HV,speed_var_CAV):
    init_vehicle_type = []# initial matrix of vehicle type in lane-order
    init_index_CAV = []# initial matrix of CAV index in lane-order
    init_vehicle_speed = []# initial matrix of vehicle speed in lane-order
    init_vehicle_position = []# initial matrix of vehicle position in lane-order
    send_in_speed=[]
    # calculated input
    dis_set = 1000 / np.array(rou_set)  # initial CF distance based on the density
    # number of each kind of vehicles in each lane
    numb_CAV = np.round(np.array(gen_number) * np.array(P_cav))
    #numb_HV = np.array(gen_number) - np.array(numb_CAV)
    for i in range(ln_num):
        vehicle_type, index_CAV = TypeGenerator(gen_number[i], numb_CAV[i])
        vehicle_speed = SpeedGenerator(gen_number[i], index_CAV, speed_set_HV[i], speed_set_CAV[i]) + SpeedVarGenerator(
            gen_number[i], index_CAV, speed_var_HV[i], speed_var_CAV[i])
        vehicle_position = PosGenerator(gen_number[i], dis_set[i])
        init_vehicle_type.append(vehicle_type)
        init_index_CAV.append(index_CAV)
        init_vehicle_speed.append(vehicle_speed)
        init_vehicle_position.append(vehicle_position)
        send_in_speed.append(-min(vehicle_position)/(60*t_inflow_upd))

    return init_vehicle_type,init_index_CAV,init_vehicle_speed,init_vehicle_position,send_in_speed
def Vehicle_initializer(init_vehicle_type, init_vehicle_speed, init_vehicle_position,ite,Vehicle_set):
    # Generate the total vehicle dic, transfer the matrix into dict
    #total_veh_number = sum([len(init_vehicle_type[i]) for i in range(len(init_vehicle_type))])
    #total_veh_number=sum(gen_number) # replace calcu
    # Generate the Initial Vehicle Set
    Vehicle_set2 ={}
    count=len(Vehicle_set)
    pre = list(np.zeros(ite - 1))
    for i in range(len(init_vehicle_type)):
        for j in range(len(init_vehicle_type[i])):
            count=count+1
            if init_vehicle_type[i][j]==0: # This is a HV
                Vehicle_detail = {f'Vehicle{count}': {
                    'vehicle_No': [count],  # attach to specific vehicle, no change, no reuse
                    'vehicle_type': pre+[init_vehicle_type[i][j]],  # 0HV  1CAV 2LCHV 3PSEUDOHV
                    'Position': pre+[init_vehicle_position[i][j]],
                    'Speed': pre+[init_vehicle_speed[i][j]],
                    'Acceleration': pre+[0],
                    'DesireSpeed': pre+[free_speed[i]],
                    'Lane_No':pre+ [i + 1], 'LC_status': [[0, 0]], 'LC_Status_Count': [[0, 0]],
                    'Distance_CF': pre+[0],
                    'Distance_ln+1': [[0, 0, 0]],
                    # car-following distance on lane n+1 the first one is the distance between leading vehicle and subject, the second one is the distance between the following vehicle and this vehicle
                    'Distance_ln-1': [[0, 0, 0]],  # car-following distance on lane n-1
                    'Speed_CF': pre+[0],  # speed of the vehicle before this vehicle
                    'Speed_ln+1': [[0, 0, 0]],  # speed of the n+1 lane for LC judgement
                    'Speed_ln-1': [[0, 0, 0]],  # speed of the n-1 lane for LC judgement
                    'Pseudoveh_No': [[0, -1]],
                    # if there is a pseudo v activated, it is the number, if there is no, it is -1
                    'Pseudoveh_lnNo': [[0, -1]],
                    # if there is a pseudo v activated, it is the number, if there is no, it is -1
                    'Pseudoveh_distance': [[0, -1]],  # the distance for the leading vehicle before the pseudo vehicle
                    'Pseudoveh_speed': [[0, -1]],  # the speed of the leading vehicle before the pseudo vehicle
                    'in_system': pre+[0]  # in system or not, for the HV and CAV, if it is in the area, it is in the system;
                    # for pseudo vehicle, if it is activated, it is in the system, if it is deleted, it is out.
                }}
            elif init_vehicle_type[i][j]==1:# this is a CAV
                Vehicle_detail = {f'Vehicle{count}': {
                    'vehicle_No': [count],  # attach to specific vehicle, no change, no reuse
                    'vehicle_type': [init_vehicle_type[i][j]],  # 0HV  1CAV 2LCHV 3PSEUDOHV
                    'Position': pre+[init_vehicle_position[i][j]],
                    'Speed': pre+[init_vehicle_speed[i][j]],
                    'Acceleration': pre+[0],
                    'DesireSpeed': pre+[free_speed[i]],
                    'Lane_No': [i + 1], 'LC_status': [[0, 0]], 'LC_Status_Count': [[0, 0]],
                    'Distance_CF': pre+[0],
                    'Distance_ln+1': [[0, 0, 0]],
                    # car-following distance on lane n+1 the first one is the distance between leading vehicle and subject, the second one is the distance between the following vehicle and this vehicle
                    'Distance_ln-1': [[0, 0, 0]],  # car-following distance on lane n-1
                    'Speed_CF': pre+[0],  # speed of the vehicle before this vehicle
                    'Speed_ln+1': [[0, 0, 0]],  # speed of the n+1 lane for LC judgement
                    'Speed_ln-1': [[0, 0, 0]],  # speed of the n-1 lane for LC judgement
                    'Pseudoveh_No': [[0, -1]],
                    # if there is a pseudo v activated, it is the number, if there is no, it is -1
                    'Pseudoveh_lnNo': [[0, -1]],
                    # if there is a pseudo v activated, it is the number, if there is no, it is -1
                    'Pseudoveh_distance': [[0, -1]],  # the distance for the leading vehicle before the pseudo vehicle
                    'Pseudoveh_speed': [[0, -1]],  # the speed of the leading vehicle before the pseudo vehicle
                    'in_system': pre+[0]  # in system or not, for the HV and CAV, if it is in the area, it is in the system;
                    # for pseudo vehicle, if it is activated, it is in the system, if it is deleted, it is out.
                }}
            else:
                print('Vehicle_initializer')
            Vehicle_set2.update(Vehicle_detail)
    return Vehicle_set2

# Functions
def InsystemFilter(Fil):
    Insystem_vehicleset={}
    for i in range(1,len(Fil)+1):
        if Fil[f'Vehicle{i}']['in_system'][-1]==1:
            in_veh={f'Vehicle{i}':Fil[f'Vehicle{i}']}
            Insystem_vehicleset.update(in_veh)
    return Insystem_vehicleset
def AdjacentVehicleSort(Vehiclekey, Fil2 ,veh_length,judge_ln): # Sort ADjacent vehicles
    #Vehicle_detail=Vehicle_set[Vehiclekey]
    #Vehicle_detail['Lane_No'][-1]=int(judge_ln)
    #Vehicle_set.update(Vehicle_detail)

    Fil2[Vehiclekey]['Lane_No'][-1]=int(judge_ln)
    in_Vehicle_set = InsystemFilter(Fil2)
    sort_veh_ln_posi = sorted(in_Vehicle_set.items(), key=lambda x: (x[1]['Lane_No'][-1], -x[1]['Position'][-1]))
    # sort vehicles with the lane and position in each lane
    for k in range(len(in_Vehicle_set)):
        if (sort_veh_ln_posi[k][0] == Vehiclekey):# & (sort_veh_ln_posi[k][1]['Lane_No'][-1] == judge_ln):
            currentveh_index = k  # get the index of current HV in adjacent lane
        # preveh_index=k-1
    # Judge if this vehicle is the first vehicle
    if (currentveh_index == 0) | (sort_veh_ln_posi[currentveh_index - 1][1]['Lane_No'][-1] != \
            sort_veh_ln_posi[currentveh_index][1]['Lane_No'][-1]):
        v_lead = 9999
        d_lead = 9999
        if currentveh_index + 1==len(in_Vehicle_set):
            v_follow = 0
            d_follow = 9999
        else:
            #print(len(sort_veh_ln_posi),len(in_Vehicle_set),currentveh_index)
            v_follow = sort_veh_ln_posi[currentveh_index + 1][1]['Speed'][-1]
            d_follow = sort_veh_ln_posi[currentveh_index][1]['Position'][-1] - sort_veh_ln_posi[currentveh_index + 1][1]['Position'][-1] - veh_length#[judge_ln-1]
    # Judge if this vehicle is the last vehicle
    elif currentveh_index ==(len(in_Vehicle_set)-1):
        v_lead = sort_veh_ln_posi[currentveh_index - 1][1]['Speed'][-1]
        d_lead = sort_veh_ln_posi[currentveh_index - 1][1]['Position'][-1] - \
                 sort_veh_ln_posi[currentveh_index][1]['Position'][-1] - veh_length#[judge_ln-1]
        v_follow=0
        d_follow=9999
    elif sort_veh_ln_posi[currentveh_index + 1][1]['Lane_No'][-1] != sort_veh_ln_posi[currentveh_index][1]['Lane_No'][-1]:
        v_lead = sort_veh_ln_posi[currentveh_index - 1][1]['Speed'][-1]
        d_lead = sort_veh_ln_posi[currentveh_index - 1][1]['Position'][-1] - \
                 sort_veh_ln_posi[currentveh_index][1]['Position'][-1] - veh_length#[judge_ln - 1]
        v_follow = 0
        d_follow = 9999
    else:
        v_lead = sort_veh_ln_posi[currentveh_index - 1][1]['Speed'][-1]
        v_follow = sort_veh_ln_posi[currentveh_index + 1][1]['Speed'][-1]
        # x_lead = sort_veh_ln_posi[currentveh_index - 1][1]['Position'][-1]
        d_lead = sort_veh_ln_posi[currentveh_index - 1][1]['Position'][-1] - \
                    sort_veh_ln_posi[currentveh_index][1]['Position'][-1] - veh_length#[judge_ln-1]
        d_follow = sort_veh_ln_posi[currentveh_index][1]['Position'][-1] - sort_veh_ln_posi[currentveh_index + 1][1]['Position'][-1] - veh_length#[judge_ln-1]
    #print(v_lead, d_lead, v_follow, d_follow)
    return v_lead, d_lead, v_follow, d_follow

# vehicle class
class Vehicle(object):
    class_name='General Vehicles'
    def __init__(self,  speed, position):
        self.speed = speed# current speed
        self.position = position# current position
        self.acceleration = 0  # acceleration for next step
        #self.vehicle_type=vehicle_type # vehicle type
        self.speed_next = 0# speed of next steep
        self.position_next = 0# position for next step

    def vehicle_speed(self, t):
        # v_current speed of current time interval; t time gap for experiment; a acceleration
        v_current = self.speed
        a = self.acceleration
        v_next=v_current + a * t
        if v_next < 0:
            v_next = 0
        self.speed_next=v_next
        return v_next

    def vehicle_trajectory(self, t):
        v = self.speed_next
        x_current = self.position
        x_next = x_current + v * t
        self.position_next=x_next
        return x_next

    def InsystermJudge(self,x_min,x_max): #ONLY used after update
        if (self.position_next>=x_min) & (self.position<=x_max):
            in_system=1
        else:
            in_system=0
        return in_system
class CAV(Vehicle):
    class_name = 'Connected Automated Vehicle'

    def __init__(self, speed, position,speed_desired):
        super(CAV, self).__init__(speed, position)
        #self.lane_number = lane_number#lane number of CAV
        self.distance=0# car-following distance of CAV
        self.speed_lead=0# speed of the leading vehicle
        self.v_desired = speed_desired#Control speed of CAV

    def FVDM_CAV(self, T, s0, tau, gamma,a_max,b_max,b_sharp):
        # v current speed; T time gap for vopt; s car-following distance, s0 minimum distance gap; tau adaption time;
        # gamma speed difference sensitivity; v_lead speed of previous vehicle; v_desired desire speed of current vehicle
        #v = self.speed
        #s = self.distance
        #v_lead=self.speed_lead
        #v_exp=self.v_desired
        # NO-CF
        if (self.distance>(T*self.v_desired+s0))|(self.distance>120):
            gamma = 0

        # NORMAL CF
        v_op = max(0, min(self.v_desired , (self.distance - s0) / T))
        a = ((v_op - self.speed) / tau) + (gamma*(self.speed_lead - self.speed))
        if (a>0) & (a>a_max):
            a=a_max
        elif (a<0) & (a<b_max):
            a=b_max

        # CLOSE CF
        if (self.speed - self.speed_lead) > 0:
            if (self.distance / (self.speed - self.speed_lead)) <= 4:
                a = b_sharp
        elif self.distance<4*s0:
            a=b_sharp

        return a
class HV(Vehicle):
    class_name = 'Human Driven Vehicle'

    def __init__(self, speed, position,v_desired,lc_status_count, pseu_veh_no,pseu_ln_numb):
        super(HV, self).__init__( speed, position)

        self.v_desired = v_desired  # expected speed for the HV
        #self.lane_number = lane_number  # current lane number for HV
        self.distance = 0  # car-following distance for HV
        self.speed_lead = 0  # speed of vehicle before HV
        #self.lc_status = lc_status# lane-changing status for HV 1 in 0 no
        self.lc_status_count=lc_status_count# lane changing time count, how long in the LC status
        self.pseu_veh_no = pseu_veh_no# the pseudo vehicle number
        self.pseu_ln_numb = pseu_ln_numb# HV will get into this lane
        self.distance_pseuv=0#ar-following distance for pseu_HV
        self.v_pseuvlead=0#speed of vehicle before pseu_HV
        #self.dele_pseu=0 # if 1 or not 0 excute delete the pseudo vehicle after this time step
    def FVDM_HV(self, v_lead,s, T, s0, tau, gamma,a_max,b_max,b_sharp):
        # v current speed; T time gap for vopt; s car-following distance, s0 minimum distance gap; tau adaption time;
        # gamma speed difference sensitivity; v_lead speed of previous vehicle; v_desired desire speed of current vehicle
        v = self.speed
        v_desired=self.v_desired

        # NO CF
        if (s>(T*v_desired+s0))|(s>120):
            gamma=0

        # NORMAL CF
        v_op = max(0, min(v_desired, (s - s0) / T))
        a = ((v_op - v) / tau) + (gamma*(v_lead - v))
        if (a>0) & (a>a_max):
            a=a_max
        elif (a<0) & (a<b_max):
            a=b_max

        # TOO CLOSE CF
        if (v-v_lead)>0:
            if (s/(v-v_lead))<=4:
                a = b_sharp
        elif s<4*s0:
            a=b_sharp
        return a
    def FVDM_safe(self, s0, T, v_nf, tau, b_safe, gamma):
        # s0 minimum distance gap, T time gap for vopt, v_nf speed of following vehicle on target lane,
        # tau adaption time, b_safe safe deceleration, gamma speed difference sensitivity, v current speed
        v = self.speed
        s_safe = s0 + T * (v_nf + tau * b_safe + gamma * tau * (v_nf - v))
        return s_safe
    def FVDM_adv(self, s, T, tau, acc_delta, acc_bias, gamma, v_l, v_nl):
        # s current car-following distance,T time gap for vopt, tau adaption time, acc_delta gain acceleration,
        # acc_bias lane-change preference on adjacent lane, gamma speed difference sensitivity,
        # v_l speed of current leading vehicle, v_nl speed of leading vehicle on the adjacent lane
        #s = self.distance
        s_adv = s + T * tau * (acc_delta + acc_bias + gamma*(v_l - v_nl))
        return s_adv
    def FVDM_LC_judge(self,s,s_min,Vehiclekey,Fil3,judge_ln):  # judge if or not lane-changing
        # if the distance between the LC vehicle and following vehicle on adjacent lane is larger than the safe distance
        # and if the distance between the LC Vehicle and the leading vehicle on the adjacent lane is larger than the adv distance
        # x_nf: the position of new following vehicle's head
        # x_nl: the position of new leading vehicle's head
        # x_lcv: the position of lc vehicle's head
        #x_lcv = self.position
        #s_nf = x_lcv - x_nf - l_veh; s_nl = x_nl - x_lcv - l_veh
        veh_len=veh_length[Fil3[Vehiclekey]['Lane_No'][-1] - 1]
        v_nl, s_nl, v_nf, s_nf = AdjacentVehicleSort(Vehiclekey, Fil3, veh_len, judge_ln)
        s_safe = self.FVDM_safe(s0, T, v_nf, tau, b_safe, gamma)
        s_adv = self.FVDM_adv(s,T, tau, acc_delta, acc_bias, gamma, self.speed_lead, v_nl)

        # Too close no LC
        if (s_nf<=s_min)|(s_nl<=s_min):
            ln_judge = 0
        elif (s_nf >= s_safe) & (s_nl >= s_adv):  # satisfy the lc conditions both
            ln_judge = 1
        else:
            ln_judge = 0
        return ln_judge,v_nl, s_nl, v_nf, s_nf

    def HV_LC_Lane(self,s,s_min,ln_num_total, Vehiclekey,lane_number,Fil4):
        # get the target lane number
        # ln_num_total=3 #   total lane number
        lc_lane=99999
        if lane_number == 1:
            ln_judge,v_nl, s_nl, v_nf, s_nf=self.FVDM_LC_judge(s,s_min,Vehiclekey,Fil4,2)
            Distance_lnup=[s_nl,s_nf]
            Distance_lndn=[0,0]
            Speed_lnup=[v_nl,v_nf]
            Speed_lndn=[0,0]
            if ln_judge==1:
                lc_lane=2
        elif lane_number == ln_num_total:
            ln_judge,v_nl, s_nl, v_nf, s_nf=self.FVDM_LC_judge(s,s_min,Vehiclekey,Fil4,ln_num_total-1)
            Distance_lndn = [s_nl, s_nf]
            Distance_lnup= [0, 0]
            Speed_lndn = [v_nl, v_nf]
            Speed_lnup = [0, 0]
            if ln_judge == 1:
                lc_lane = ln_num_total-1
        else:
            # Do LC judge on both side
            ln_judge1,v_nl1, s_nl1, v_nf1, s_nf1 = self.FVDM_LC_judge(s,s_min,Vehiclekey, Fil4, lane_number - 1)
            ln_judge2,v_nl2, s_nl2, v_nf2, s_nf2 = self.FVDM_LC_judge(s,s_min,Vehiclekey, Fil4, lane_number + 1)
            Distance_lndn = [s_nl1, s_nf1]
            Distance_lnup = [s_nl2, s_nf2]
            Speed_lndn = [v_nl1, v_nf1]
            Speed_lnup = [v_nl2, v_nf2]
            if ln_judge1+ln_judge2==2:
                j=[random.random(), random.random(), lane_number - 1, lane_number + 1]
                lc_lane=j[j.index(max(j[0],j[1]))+2]
            elif ln_judge1+ln_judge2==1:
                j = [ln_judge1, ln_judge2, lane_number - 1, lane_number + 1]
                lc_lane = j[j.index(max(j[0], j[1])) + 2]
            elif ln_judge1 + ln_judge2 == 0:# LC not satisfied
                lc_lane = 99999
            else:
                print('LC ERROR INNER LANE')
        return lc_lane,Distance_lnup,Distance_lndn,Speed_lnup,Speed_lndn

# Vehicle Run
def SpeedControl(Vehicle_set,control_position,control_speed):
    for j in range(1, len(Vehicle_set) + 1):# for all vehicles
        if Vehicle_set[f'Vehicle{j}']['in_system'][-1] == 1:# if the vehicle in the system
            for i in range(0,len(control_position)-1):# for each control segment
                if (Vehicle_set[f'Vehicle{j}']['Position'][-1]>=control_position[i]) & (Vehicle_set[f'Vehicle{j}']['Position'][-1]<=control_position[i+1]):
                    if (Vehicle_set[f'Vehicle{j}']['vehicle_type'][-1] == 1)|\
                            (Vehicle_set[f'Vehicle{j}']['vehicle_type'][-1] == 0):# if it is HV or CAV
                        Vehicle_set[f'Vehicle{j}']['DesireSpeed'].append(control_speed[i])
                    elif(Vehicle_set[f'Vehicle{j}']['vehicle_type'][-1] == 2)|\
                            (Vehicle_set[f'Vehicle{j}']['vehicle_type'][-1] == 3):# if it is an LCHV
                        Vehicle_set[f'Vehicle{j}']['DesireSpeed'].append(free_speed[Vehicle_set[f'Vehicle{j}']['Lane_No'][-1]-1])
    return Vehicle_set

def run():
    traci.init(PORT)
    Vehicle_set={}
    for i in range(1,t_runtotal+1):# for the whole simulation period
        print(i)
        traci.simulationStep()
        if i % (60*5) == 1:  # call the generator per 5 min
            init_vehicle_type, init_index_CAV, init_vehicle_speed, init_vehicle_position, send_in_speed = VehicleGenerator(
                ln_num, rou_set, gen_number, P_cav, speed_set_CAV, speed_set_HV, speed_var_HV, speed_var_CAV)
            Vehicle_set1 = Vehicle_initializer(init_vehicle_type, init_vehicle_speed, init_vehicle_position,i,Vehicle_set)
            Vehicle_set.update(Vehicle_set1)
        # Control with time
        Vehicle_set=SpeedControl(Vehicle_set,control_position,control_speed)
        input_vehset = copy.deepcopy(Vehicle_set)
        #input_vehset=Vehicle_set
        for j in range(1,len(input_vehset)+1):# for all vehilces

            #  TYPE 1: currently, the vehicle is out system, and haven't been into system
            # then, only push them into the system
            if (input_vehset[f'Vehicle{j}']['in_system'][-1] == 0) & (input_vehset[f'Vehicle{j}']['Position'][
                                                                            -1] < x_min):
                new_posi = input_vehset[f'Vehicle{j}']['Position'][-1] + send_in_speed[Vehicle_set[f'Vehicle{j}']['Lane_No'][-1]-1] * t_sim
                Vehicle_set[f'Vehicle{j}']['Position'].append(new_posi)
                # Insystem Judgement
                if new_posi >= x_min:  # next step get into the system
                    Vehicle_set[f'Vehicle{j}']['in_system'].append(1)
                    Vehicle_set[f'Vehicle{j}']['Distance_CF'].append(9999)
                else:
                    Vehicle_set[f'Vehicle{j}']['in_system'].append(0)
                    Vehicle_set[f'Vehicle{j}']['Distance_CF'].append(Vehicle_set[f'Vehicle{j}']['Distance_CF'][-1])
                # Other Values
                Vehicle_set[f'Vehicle{j}']['Speed'].append(Vehicle_set[f'Vehicle{j}']['Speed'][-1])
                Vehicle_set[f'Vehicle{j}']['Acceleration'].append(Vehicle_set[f'Vehicle{j}']['Acceleration'][-1])
                Vehicle_set[f'Vehicle{j}']['DesireSpeed'].append(Vehicle_set[f'Vehicle{j}']['DesireSpeed'][-1])
                Vehicle_set[f'Vehicle{j}']['Speed_CF'].append(Vehicle_set[f'Vehicle{j}']['Speed_CF'][-1])
                if input_vehset[f'Vehicle{j}']['vehicle_type'][-1]==0: # the vehicle is a HV
                    # HV has not got into the system this time step, no pseudo vehicles, no LC judge
                    # Only update the full-time-rank parameters
                    Vehicle_set[f'Vehicle{j}']['vehicle_type'].append(Vehicle_set[f'Vehicle{j}']['vehicle_type'][-1])
                    Vehicle_set[f'Vehicle{j}']['Lane_No'].append(Vehicle_set[f'Vehicle{j}']['Lane_No'][-1])
                # ADD New-in Vehicle
                if new_posi >= x_min:
                    AddVehicle(input_vehset[f'Vehicle{j}'], f'Vehicle{j}')

            # IN_SYSTEM VEHICLES
            elif input_vehset[f'Vehicle{j}']['in_system'][-1] == 1:
                # TYPE 2-1 CAV
                if input_vehset[f'Vehicle{j}']['vehicle_type'][-1] == 1:
                    Veh_CAV=CAV(input_vehset[f'Vehicle{j}']['Speed'][-1],
                            input_vehset[f'Vehicle{j}']['Position'][-1],
                            input_vehset[f'Vehicle{j}']['DesireSpeed'][-1])# single vehicle generate
                    Veh_CAV.speed_lead,Veh_CAV.distance, _, _ =AdjacentVehicleSort(f'Vehicle{j}', input_vehset, veh_length[input_vehset[f'Vehicle{j}']['Lane_No'][-1]-1], input_vehset[f'Vehicle{j}']['Lane_No'][-1])
                    # CF
                    Veh_CAV.acceleration = Veh_CAV.FVDM_CAV(T, s0, tau, gamma, a_max, b_max,b_sharp)
                    Veh_CAV.speed_next = Veh_CAV.vehicle_speed(t_sim)
                    Veh_CAV.position_next = Veh_CAV.vehicle_trajectory(t_sim)
                    in_system = Veh_CAV.InsystermJudge(x_min, x_max)
                    # UPDATE
                    Vehicle_set[f'Vehicle{j}']['Acceleration'].append(Veh_CAV.acceleration)
                    Vehicle_set[f'Vehicle{j}']['Speed'].append(Veh_CAV.speed_next)
                    Vehicle_set[f'Vehicle{j}']['Position'].append(Veh_CAV.position_next)
                    Vehicle_set[f'Vehicle{j}']['in_system'].append(in_system)
                    Vehicle_set[f'Vehicle{j}']['Speed_CF'].append(Vehicle_set[f'Vehicle{j}']['Speed_CF'][-1])
                    #Vehicle_set[f'Vehicle{j}']['DesireSpeed'].append(Vehicle_set[f'Vehicle{j}']['DesireSpeed'][-1])


                # TYPE 2-2 LCHV
                elif input_vehset[f'Vehicle{j}']['vehicle_type'][-1] == 2:
                    # single vehicle generate
                    Veh_LCHV=HV(input_vehset[f'Vehicle{j}']['Speed'][-1],#speed,
                            input_vehset[f'Vehicle{j}']['Position'][-1],#position,
                            input_vehset[f'Vehicle{j}']['DesireSpeed'][-1],#v_desired,
                            input_vehset[f'Vehicle{j}']['LC_Status_Count'][-1][-1],#lc_status_count,
                            input_vehset[f'Vehicle{j}']['Pseudoveh_No'][-1][-1],#pseu_veh_no,
                            input_vehset[f'Vehicle{j}']['Pseudoveh_lnNo'][-1][-1])#pseu_ln_numb)
                    if Veh_LCHV.lc_status_count == 15: # TYPE 2-2-1 finished LC HV
                    # lc status update(for next time step)
                        Vehicle_set[f'Vehicle{j}']['LC_status'].append([i, 0])  # lc_status = 0
                        Vehicle_set[f'Vehicle{j}']['LC_Status_Count'].append([i, 0])  # lc_status_count = 0
                        Vehicle_set[f'Vehicle{j}']['Lane_No'].append(Veh_LCHV.pseu_ln_numb)  # ln_numb = LCHVsample.pseu_ln_numb
                        Vehicle_set[f'Vehicle{j}']['vehicle_type'].append(0)  # veh_type = 0  # LCHV finished the LC and turns to CFHV type=0
                    # Update the basic-3 status
                        Veh_LCHV.v_pseuvlead, Veh_LCHV.distance_pseuv, _, _  = AdjacentVehicleSort(f'Vehicle{Veh_LCHV.pseu_veh_no}',
                                                                                        input_vehset ,
                                                                                        veh_length[Veh_LCHV.pseu_ln_numb-1],
                                                                                        Veh_LCHV.pseu_ln_numb)

                    # v_lead=LCHVsample.v_pseuvlead;        s=LCHVsample.distance_pseuv
                        Veh_LCHV.acceleration = Veh_LCHV.FVDM_HV(Veh_LCHV.v_pseuvlead, Veh_LCHV.distance_pseuv, T,
                                                                 s0, tau, gamma, a_max, b_max,b_sharp)
                        Veh_LCHV.speed_next = Veh_LCHV.vehicle_speed(t_sim)
                        Veh_LCHV.position_next = Veh_LCHV.vehicle_trajectory(t_sim)
                    # Update the dic
                        Vehicle_set[f'Vehicle{j}']['Acceleration'].append(Veh_LCHV.acceleration)
                        Vehicle_set[f'Vehicle{j}']['Speed'].append(Veh_LCHV.speed_next)
                        Vehicle_set[f'Vehicle{j}']['Position'].append(Veh_LCHV.position_next)
                        Vehicle_set[f'Vehicle{j}']['Speed_CF'].append(Veh_LCHV.v_pseuvlead)
                        Vehicle_set[f'Vehicle{j}']['Distance_CF'].append(Veh_LCHV.distance_pseuv)
                    # delete pseudo vehicle in current LCHV
                        Vehicle_set[f'Vehicle{j}']['Pseudoveh_No'].append([i, -1])
                        Vehicle_set[f'Vehicle{j}']['Pseudoveh_lnNo'].append([i, -1])
                        Vehicle_set[f'Vehicle{j}']['Pseudoveh_distance'].append([i, -1])
                        Vehicle_set[f'Vehicle{j}']['Pseudoveh_speed'].append([i, -1])
                    # in system judge
                        in_system = Veh_LCHV.InsystermJudge(x_min, x_max)
                        Vehicle_set[f'Vehicle{j}']['in_system'].append(in_system)

                    # Delete pseudo vehicle itself, only change the in-system status
                        Vehicle_set[f'Vehicle{Veh_LCHV.pseu_veh_no}']['in_system'].append(-1)
                    elif (Veh_LCHV.lc_status_count < 15) & (Veh_LCHV.lc_status_count > 0): # TYPE 2-2-2 CHANGING LC HV
                    # vehicle status update acc, speed, position
                        Veh_LCHV.speed_next = Veh_LCHV.vehicle_speed(t_sim)
                        Veh_LCHV.position_next = Veh_LCHV.vehicle_trajectory(t_sim)
                        Vehicle_set[f'Vehicle{j}']['Acceleration'].append(Veh_LCHV.acceleration)
                        Vehicle_set[f'Vehicle{j}']['Speed'].append(Veh_LCHV.speed_next)
                        Vehicle_set[f'Vehicle{j}']['Position'].append(Veh_LCHV.position_next)

                        Veh_LCHV.v_pseuvlead, Veh_LCHV.distance_pseuv, _, _  = AdjacentVehicleSort(f'Vehicle{Veh_LCHV.pseu_veh_no}',
                                                                                        input_vehset,
                                                                                        veh_length[Veh_LCHV.pseu_ln_numb - 1],
                                                                                        Veh_LCHV.pseu_ln_numb)
                        Veh_LCHV.speed_lead, Veh_LCHV.distance, _, _ =AdjacentVehicleSort(f'Vehicle{j}',
                                                                                        input_vehset,
                                                                                        veh_length[input_vehset[f'Vehicle{j}']['Lane_No'][-1] - 1],
                                                                                        input_vehset[f'Vehicle{j}']['Lane_No'][-1])
                        Vehicle_set[f'Vehicle{j}']['Distance_CF'].append(Veh_LCHV.distance)
                        Vehicle_set[f'Vehicle{j}']['Speed_CF'].append(Veh_LCHV.speed_lead)
                    # lane changing update
                        Vehicle_set[f'Vehicle{j}']['LC_status'].append([i, 1])  # lc_status = 1
                        Vehicle_set[f'Vehicle{j}']['LC_Status_Count'].append([i, Veh_LCHV.lc_status_count + 1])  # lc_status_count = lc_status_count + 1

                        Vehicle_set[f'Vehicle{j}']['vehicle_type'].append(Vehicle_set[f'Vehicle{j}']['vehicle_type'][-1])  # vehicle type no change
                        Vehicle_set[f'Vehicle{j}']['Lane_No'].append(Vehicle_set[f'Vehicle{j}']['Lane_No'][-1])  # ln_numb no change

                    # Pseudo veh for current LCHV
                        Vehicle_set[f'Vehicle{j}']['Pseudoveh_No'].append([i, Veh_LCHV.pseu_veh_no])
                        Vehicle_set[f'Vehicle{j}']['Pseudoveh_lnNo'].append([i, Veh_LCHV.pseu_ln_numb])
                    # Vehicle_set[f'Vehicle{j}']['Pseudoveh_distance'].append([i,LCHVsample.distance_pseuv])
                        Vehicle_set[f'Vehicle{j}']['Pseudoveh_speed'].append([i, Veh_LCHV.speed_next])

                    # Pseudo vehicle status update
                        Vehicle_set[f'Vehicle{Veh_LCHV.pseu_veh_no}']['Position'].append(
                            Veh_LCHV.position_next)  # pseudo_posi = LCHVsample.position_next
                        Vehicle_set[f'Vehicle{Veh_LCHV.pseu_veh_no}']['Speed'].append(
                            Veh_LCHV.speed_next)  # pseudo_speed = LCHVsample.speed_next
                    # Vehicle_set[f'Vehicle{LCHVsample.pseu_veh_no}']['Acceleration'].append([i,0])#pseudo_acc = LCHVsample.acceleration# ALL 0, NO NEED
                    # Vehicle_set[f'Vehicle{LCHVsample.pseu_veh_no}']['DesireSpeed'].append([i,])

                        Vehicle_set[f'Vehicle{Veh_LCHV.pseu_veh_no}']['Lane_No'].append(
                            Veh_LCHV.pseu_ln_numb)  # pseudo_ln_numb = LCHVsample.pseu_ln_numb
                        Vehicle_set[f'Vehicle{Veh_LCHV.pseu_veh_no}']['LC_status'].append([i, 1])  # pseudo_lc_status = 1
                        Vehicle_set[f'Vehicle{Veh_LCHV.pseu_veh_no}']['LC_Status_Count'].append(
                            [i, Veh_LCHV.lc_status_count + 1])

                        Vehicle_set[f'Vehicle{Veh_LCHV.pseu_veh_no}']['Distance_CF'].append(
                            [i, Veh_LCHV.distance_pseuv])
                        Vehicle_set[f'Vehicle{Veh_LCHV.pseu_veh_no}']['Speed_CF'].append([i, Veh_LCHV.v_pseuvlead])

                    # In-system judge
                        in_system = Veh_LCHV.InsystermJudge(x_min, x_max)
                        Vehicle_set[f'Vehicle{j}']['in_system'].append(in_system)
                        Vehicle_set[f'Vehicle{Veh_LCHV.pseu_veh_no}']['in_system'].append(in_system)
                    else:  # something goes wrong
                        print('Wrong LC Status')

                # TYPE 2-3 HV
                elif input_vehset[f'Vehicle{j}']['vehicle_type'][-1] == 0:
                # single vehicle generate
                    Veh_HV = HV(input_vehset[f'Vehicle{j}']['Speed'][-1],  # speed,
                            input_vehset[f'Vehicle{j}']['Position'][-1],  # position,
                            input_vehset[f'Vehicle{j}']['DesireSpeed'][-1],  # v_desired,
                            0,  # lc_status_count,
                            0,  # pseu_veh_no,
                            0)  # pseu_ln_numb)
                # GET THE CF status of HV
                    Veh_HV.speed_lead, Veh_HV.distance, _, _  = AdjacentVehicleSort(f'Vehicle{j}', input_vehset,
                                                                         veh_length[input_vehset[f'Vehicle{j}']['Lane_No'][-1] - 1],
                                                                         input_vehset[f'Vehicle{j}']['Lane_No'][-1])
                # LC JUDGEMENT -1 the first vehicle does not change lane
                    if (Veh_HV.speed_lead==9999) & (Veh_HV.distance==9999):
                        new_ln_numb = 99999
                        Distance_lnup = [0, 0]
                        Distance_lndn = [0, 0]
                        Speed_lnup = [0, 0]
                        Speed_lndn = [0, 0]
                    else: # Other vehicle do judgement and see which lane to move in
                        new_ln_numb,Distance_lnup,Distance_lndn,Speed_lnup,Speed_lndn=Veh_HV.HV_LC_Lane(Veh_HV.distance, s_min,#s_min,
                                                                                                    ln_num, f'Vehicle{j}',
                                                                                                    input_vehset[f'Vehicle{j}']['Lane_No'][-1],
                                                                                                    input_vehset)

                # do the lc rate
                    if random.random() >= lc_decisionrate[input_vehset[f'Vehicle{j}']['Lane_No'][-1] - 1]:
                        new_ln_numb = 99999
                # TYPE 2-3-1 NO-LCHV
                    if new_ln_numb == 99999:  # no lc for next step
                        # lc status update
                        # Vehicle_set[f'Vehicle{j}']['LC_status'].append([i,0])#lc_status = 0
                        Vehicle_set[f'Vehicle{j}']['vehicle_type'].append(0)
                        Vehicle_set[f'Vehicle{j}']['Lane_No'].append(
                            Vehicle_set[f'Vehicle{j}']['Lane_No'][-1])  # ln_numb = ln_numb
                    # Update the basic-3 status
                        Veh_HV.acceleration = Veh_HV.FVDM_HV(Veh_HV.speed_lead, Veh_HV.distance, T,
                                                             s0, tau, gamma, a_max,b_max,b_sharp)  # HVsample.acceleration=FVDM_HV
                        Veh_HV.speed_next = Veh_HV.vehicle_speed(t_sim)  # speed = speed + acc * sim_t
                        Veh_HV.position_next = Veh_HV.vehicle_trajectory(t_sim)  # position = position + speed * sim_t
                        Vehicle_set[f'Vehicle{j}']['Acceleration'].append(Veh_HV.acceleration)
                        Vehicle_set[f'Vehicle{j}']['Speed'].append(Veh_HV.speed_next)
                        Vehicle_set[f'Vehicle{j}']['Position'].append(Veh_HV.position_next)
                    # Other Update
                        Vehicle_set[f'Vehicle{j}']['Distance_CF'].append(Veh_HV.distance)
                        Vehicle_set[f'Vehicle{j}']['Distance_ln+1'].append([i] + Distance_lnup)
                        Vehicle_set[f'Vehicle{j}']['Distance_ln-1'].append([i] + Distance_lndn)
                        Vehicle_set[f'Vehicle{j}']['Speed_CF'].append(Veh_HV.speed_lead)
                        Vehicle_set[f'Vehicle{j}']['Speed_ln+1'].append([i] + Speed_lnup)
                        Vehicle_set[f'Vehicle{j}']['Speed_ln-1'].append([i] + Speed_lndn)
                    # in system judge
                        in_system = Veh_HV.InsystermJudge(x_min, x_max)
                        Vehicle_set[f'Vehicle{j}']['in_system'].append(in_system)
                    else: # lc in next step
                    # lc status update
                        Vehicle_set[f'Vehicle{j}']['vehicle_type'].append(2)
                        Vehicle_set[f'Vehicle{j}']['LC_status'].append([i, 1])  # lc_status = 1
                        Vehicle_set[f'Vehicle{j}']['LC_Status_Count'].append([i, 1])
                        Vehicle_set[f'Vehicle{j}']['Lane_No'].append(
                            Vehicle_set[f'Vehicle{j}']['Lane_No'][-1])  # lane number no change yet
                    # Update the basic-3 status
                    # acc = 0
                        Veh_HV.speed_next = Veh_HV.vehicle_speed(t_sim)  # speed = speed
                        Veh_HV.position_next = Veh_HV.vehicle_trajectory(t_sim)  # position = position + speed * sim_t
                        Vehicle_set[f'Vehicle{j}']['Acceleration'].append(Veh_HV.acceleration)
                        Vehicle_set[f'Vehicle{j}']['Speed'].append(Veh_HV.speed_next)
                        Vehicle_set[f'Vehicle{j}']['Position'].append(Veh_HV.position_next)
                    # Other Update
                        Vehicle_set[f'Vehicle{j}']['Distance_CF'].append(Veh_HV.distance)
                        Vehicle_set[f'Vehicle{j}']['Distance_ln+1'].append([i] + Distance_lnup)
                        Vehicle_set[f'Vehicle{j}']['Distance_ln-1'].append([i] + Distance_lndn)
                        Vehicle_set[f'Vehicle{j}']['Speed_CF'].append(Veh_HV.speed_lead)
                        Vehicle_set[f'Vehicle{j}']['Speed_ln+1'].append([i] + Speed_lnup)
                        Vehicle_set[f'Vehicle{j}']['Speed_ln-1'].append([i] + Speed_lndn)
                    # pseudo vehicle generate in current vehicle
                        Vehicle_set[f'Vehicle{j}']['Pseudoveh_No'].append(
                            [i, len(Vehicle_set) + 1])  # Pseudoveh_No=len(Vehicle_set)+1
                        Vehicle_set[f'Vehicle{j}']['Pseudoveh_lnNo'].append(
                            [i, new_ln_numb])  # pseudo_ln_numb = new_ln_numb
                        Vehicle_set[f'Vehicle{j}']['Pseudoveh_distance'].append([i, 0])
                        Vehicle_set[f'Vehicle{j}']['Pseudoveh_speed'].append([i, Veh_HV.speed_next])
                    # in system judge
                        in_system = Veh_HV.InsystermJudge(x_min, x_max)
                        Vehicle_set[f'Vehicle{j}']['in_system'].append(in_system)
                        if in_system == 1:
                        # psuedo vehicle generate in vehicle set
                            Vehicle_newpseudo = {f'Vehicle{len(Vehicle_set)+1}': {
                            'vehicle_No': [len(Vehicle_set)+1],
                            'vehicle_type': [3],  # 0HV  1CAV 2LCHV 3PSEUDOHV
                            'Position': [Veh_HV.position_next],
                            'Speed': [Veh_HV.speed_next],
                            'Acceleration': [0],
                            'DesireSpeed': [[i,15]],
                            'Lane_No': [new_ln_numb], 'LC_status': [[i,1]], 'LC_Status_Count': [[i,1]],
                            'Distance_CF': [[i,-1]],
                            'Distance_ln+1': [],
                        # car-following distance on lane n+1 the first one is the distance between leading vehicle and subject, the second one is the distance between the following vehicle and this vehicle
                            'Distance_ln-1': [],  # car-following distance on lane n-1
                            'Speed_CF': [[i,-1]],  # speed of the vehicle before this vehicle
                            'Speed_ln+1': [],  # speed of the n+1 lane for LC judgement
                            'Speed_ln-1': [],  # speed of the n-1 lane for LC judgement
                            'Pseudoveh_No': [],
                        # if there is a pseudo v activated, it is the number, if there is no, it is -1
                            'Pseudoveh_lnNo': [],
                        # if there is a pseudo v activated, it is the number, if there is no, it is -1
                            'Pseudoveh_distance': [],  # the distance for the leading vehicle before the pseudo vehicle
                            'Pseudoveh_speed': [],  # the speed of the leading vehicle before the pseudo vehicle
                            'in_system': [1]
                        # in system or not, for the HV and CAV, if it is in the area, it is in the system;
                        # for pseudo vehicle, if it is activated, it is in the system, if it is deleted, it is out.
                        }}
                            Vehicle_set.update(Vehicle_newpseudo)
                            input_vehset.update(Vehicle_newpseudo)

                # TraCI Update or Remove, NOT for PseudoHV
                if input_vehset[f'Vehicle{j}']['vehicle_type'][-1] !=3:
                    if input_vehset[f'Vehicle{j}']['in_system'][-1]==1:
                        # Status Update
                        #print(input_vehset[f'Vehicle{j}'])
                        StateUpdate(input_vehset[f'Vehicle{j}'], f'Vehicle{j}')
                    elif input_vehset[f'Vehicle{j}']['in_system'][-1]==0:
                        # Remove this vehicle
                        RemoveVehicle(f'Vehicle{j}')

    # Close connection to SUMO
    traci.close()
    sys.stdout.flush()






def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options


# this is the main entry point of this script
if __name__ == "__main__":
    options = get_options()

    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    # first, generate the route file for this simulation
    generate_routefile()

    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    sumoProcess = subprocess.Popen([sumoBinary, "-c", "0322/3lanemixDemo.sumocfg", "--tripinfo-output",
                                    "PlayDemo/tripinfo.xml", "--remote-port", str(PORT)], stdout=sys.stdout,
                                   stderr=sys.stderr)
    run()
    sumoProcess.wait()