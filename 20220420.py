
from functools import wraps
import time

import numpy
import os

def test_time(function):
    @wraps(function)
    def func_time(*args, **kwargs):
        t0 = time.perf_counter()
        result = function(*args, **kwargs)
        t1 = time.perf_counter()
        print("Total running time: %s s" % (str(t1 - t0)))
        return result

    return func_time


def Plane_Location(time, m, n, H,  v = 5, d_intraOrbit = 90, d_interOrbit = 80):
    Plane_x_t = v * time + d_intraOrbit * m
    Plane_y_t = d_interOrbit * n
    Plane_z_t = H
    return Plane_x_t, Plane_y_t, Plane_z_t

def distance(x, y, z, x_0, y_0, z_0 = 0):
    x = x_0 - x
    y = y_0 - y
    z = z_0 - z
    return (x**2 + y**2 + z**2)**0.5

def CommunicationTime(S, t_f = 0.1):
    return t_f + S/10000

def Initial(time, m, n, x_0,  y_0, time_id = None, H=10, d=125, D=70,  h=0):
    time = time
    m = m
    n = n
    time_id = time_id
    Plane_Distribution = []
    In_Communicate_scale = []
    index_i = []
    index_j = []
    time_decay = []
    time_now = []
    Initial_Distance_to_BaseStation = numpy.empty(shape = (m,n))
    for i in range(m):
        for j in range(n):
            Plane_x_t, Plane_y_t, Plane_z_t =Plane_Location(time=time, m=i, n=j, H=H)
            Location = [Plane_x_t, Plane_y_t, Plane_z_t]  #t = 某時刻的無人機位置
            Plane_Distribution.append(Location)
            Initial_Distance_to_BaseStation[i][j] = distance(Plane_x_t,Plane_y_t, Plane_z_t, x_0=x_0, y_0=y_0, z_0=h)  
            #t = 0時刻無人機到0電臺的距離
           
            if Initial_Distance_to_BaseStation[i][j] <= D :
                
                t_communication =  CommunicationTime( Initial_Distance_to_BaseStation[i][j] )#通訊時間
                Plane_x_t, Plane_y_t, Plane_z_t =Plane_Location(time=time+t_communication, m=i, n=j,H=H)
                FinalDistance = distance(Plane_x_t,Plane_y_t, Plane_z_t, x_0=x_0, y_0=y_0, z_0=h)
                if FinalDistance <= D :
                    id_ok = time_id
                    In_Communicate_scale.append(Location) #可完成通訊的無人機

                    index_i.append(i)
                    index_j.append(j)
                    time_now.append(t_communication + time) #當前時間
                    time_decay.append(t_communication) #成立情況下的不同路徑的延時時間
                    
    
    return Plane_Distribution, Initial_Distance_to_BaseStation, In_Communicate_scale, index_i, index_j, time_now, time_decay,id_ok

def Drone_to_Drone(Point_A, In_Communicate_scale_B, i,
                                            D=70,d_intraOrbit = 90, d_interOrbit = 80):
    Time = []
    Index = []
    j = 0
    PointA = Point_A

    for PointB in In_Communicate_scale_B:
        PointA = numpy.array(PointA)
        PointB = numpy.array(PointB)
        x_num = (PointA[0]-PointB[0])/d_intraOrbit
        y_num = (PointA[1]-PointB[1])/d_interOrbit
        min_num = min(abs(x_num),abs(y_num))
        max_num = max(abs(x_num),abs(y_num))
        if x_num <= y_num:
            time = min_num * CommunicationTime(S=(d_intraOrbit**2+d_interOrbit**2)**0.5) + (max_num-min_num) * CommunicationTime(S=d_interOrbit)
        else:
            time = min_num * CommunicationTime(S=(d_intraOrbit**2+d_interOrbit**2)**0.5) + (max_num-min_num)* CommunicationTime(S=d_intraOrbit)
        Time.append(time)
        index = (i, j)
        Index.append(index)
        j=j+1


    return Time, Index
      
def Total_Decay(t, BaseStation_B, Decay, BaseStation_A= 0):
    initial_time_config = (t, BaseStation_A, BaseStation_B, round(Decay,4)) #初始時刻 基站A 基站B 總延時

    return initial_time_config

def Position_Path(time, final_time,First, End,Path_to_Fianl_BaseStation,d_intraOrbit = 90, d_interOrbit = 80, 
                                        Station_1 = True):
    FinalDrone = End
    FirstDrone = First
    t = time
    final_time = final_time

    Path_to_Fianl_BaseStation.append((round(time,4),FirstDrone[0],FirstDrone[1])) #基站A -> 無人機
    t = 0
    a = abs(FirstDrone[0]-FinalDrone[0]) #行
    b = abs(FirstDrone[1]-FinalDrone[1]) #列
    if Station_1:
        flag = 0
    else:
        flag = 1

    m = FirstDrone[0]
    n = FirstDrone[1]
    for i in range(min(a,b)):  #无人机到无人机
        m =  m  +(-1)**flag
        n =  n +1
        t = t + CommunicationTime(S= (d_intraOrbit**2 + d_interOrbit**2)**0.5)
        Path_to_Fianl_BaseStation.append((round(t,4),m, n))

    if a > b: #行数大于列数
        for j in range(a-b):
            m = m + (-1)**flag
            n= n
            t = t + CommunicationTime(S=d_intraOrbit)
            Path_to_Fianl_BaseStation.append((round(t,4),m, n))

    if a < b :
        for j in range(b-a):
            m = m
            n = n + 1 
            t = t + CommunicationTime(S=d_interOrbit)
            Path_to_Fianl_BaseStation.append((round(t,4),m, n))

    return Path_to_Fianl_BaseStation

def File(result):
    if not os.path.isfile('./result.txt'):
        os.mknod('result.txt')
    file_result = open('./result.txt', mode='r+')
    file_result.read()
    file_result.write(result)
    file_result.close()

def Time_begin():
    time_config =[
        0, 4.7, 16.4
    ] 
    return time_config

def Station_begin():
    station_config =[
        1, 2
    ] 
    return station_config

@test_time
def test(m = 150,
                    n = 150,
                    Base_D = 70,
                    Drone_h = 10,
                    Base_h = 0,
                    x_00 = 1080,  #12
                    y_0=45.26,
                    y_1=700,
                    y_2=1100):

    x_0=45.73 + x_00
    x_1=1200 +x_00
    x_2= x_00 -940
    time_config = Time_begin()
    station_config = Station_begin()

    for time_i in range(3):
        time = time_config[time_i]
        for station_j in range (2):
            use_BaseStation_1 = True if station_config[station_j] == 1 else False #選擇基站

            #初始化位置
            _,_, In_Communicate_scale_toX0,index_i0,index_j0, time_now_0, time_decay_0,_ = Initial(time=time, m=m, 
                                                                                                                                        n=n, x_0= x_0, y_0=y_0)

            Drone_to_Drone_communication = []
            Drone_AB =[]
            Index_AB_extend = []
            index_i1_extend = []
            index_j1_extend = []
            DroneTime = []
            conmunication_time = []
            Final_Decay = []
            time_0_extend = []

            if use_BaseStation_1 :
                i =0
                for time_now in time_now_0: #在與基站0通訊後還在基站1通訊範圍的無人機

                    _,_, In_Communicate_scale_toX1,index_i1,index_j1, time_now_1,time_decay_1,_ = Initial(time=time_now, 
                                                                                                                                                m=m, n=n, x_0= x_1, y_0=y_1)
                    Drone_to_Drone_time, Index_AB = Drone_to_Drone(In_Communicate_scale_toX0[i],
                                                                                                                                In_Communicate_scale_toX1,i=i )
                    Drone_to_Drone_communication.extend(Drone_to_Drone_time)
                    i = i+1
                    Index_AB_extend.extend(Index_AB)
                    index_i1_extend.extend(index_i1)
                    index_j1_extend.extend(index_j1)
                    time_0_extend.extend([time_now])
                

                id = 0
                for Drone_to_Drone_communication in Drone_to_Drone_communication:
                    Drone_x_t, Drone_y_t, Drone_z_t = Plane_Location(time=time_now+Drone_to_Drone_communication,
                                                                                                    m=index_i1_extend[id], n=index_j1_extend[id], H=Drone_h)
                    Distance_to_end = distance(x=Drone_x_t, y =Drone_y_t, z =Drone_z_t,x_0 =x_1,
                                                                            y_0=y_1,z_0=Base_h )
                    
                    if Distance_to_end  <= Base_D:
                        Drone_AB.append(Index_AB_extend[id])
                        DroneTime.append(Drone_to_Drone_communication)
                        conmunication_time.append(CommunicationTime(S=Distance_to_end))
                    id = id+1

            else:
                i =0
                for time_now in time_now_0: #在與基站0通訊後還在基站2通訊範圍的無人機

                    _,_, In_Communicate_scale_toX2,index_i1,index_j1, time_now_1,time_decay_1,_ = Initial(time=time_now, 
                                                                                                                                                m=m, n=n, x_0= x_2, y_0=y_2)
                    Drone_to_Drone_time, Index_AB = Drone_to_Drone(In_Communicate_scale_toX0[i],
                                                                                                                                In_Communicate_scale_toX2,i=i )
                    Drone_to_Drone_communication.extend(Drone_to_Drone_time)
                    i = i+1
                    Index_AB_extend.extend(Index_AB)
                    index_i1_extend.extend(index_i1)
                    index_j1_extend.extend(index_j1)
                    time_0_extend.extend([time_now])
                
                id = 0
                for Drone_to_Drone_communication in Drone_to_Drone_communication:
                    Drone_x_t, Drone_y_t, Drone_z_t = Plane_Location(time=time_now+Drone_to_Drone_communication,
                                                                                                    m=index_i1_extend[id], n=index_j1_extend[id], H=Drone_h)
                    Distance_to_end = distance(x=Drone_x_t, y =Drone_y_t, z =Drone_z_t,x_0 =x_2,
                                                                            y_0=y_2,z_0=Base_h )
                    
                    if Distance_to_end  <= Base_D:
                        Drone_AB.append(Index_AB_extend[id])
                        DroneTime.append(Drone_to_Drone_communication)
                        conmunication_time.append(CommunicationTime(S=Distance_to_end))
                    id = id+1

            for a,b in zip(DroneTime,conmunication_time):
                summ = a+b
                Final_Decay.append(summ)

            Final_Decay_extend = []
            for c in range(len(Drone_AB)):
                summ = Final_Decay[c] + time_0_extend[Drone_AB[c][0]]
                Final_Decay_extend.append(summ)

            final_time = min(Final_Decay_extend)
            final_id = Final_Decay_extend.index(final_time)
            time_0 = final_time - Final_Decay[final_id]

        ###################################################
            print("index_i0:", index_i0)
            print("index_j0:", index_j0)
            print("index_i1:", index_i1)
            print("index_j1:", index_j1)
            print("Index_AB_extend:", Index_AB_extend)

            print("Drone_AB", Drone_AB)  #有效的AB
            print("DroneTime", DroneTime) #无人机延时
            print("conmunication_time", conmunication_time) #接受延时
            print("time_0_extend",time_0_extend) #输出延时
            print("Final_Decay_extend",Final_Decay_extend) #所有可能延时
            print("final_time",final_time) #最小延时
            print("final_id",final_id) #最小延时Drone A B配置
            First_Drone =  (index_i0[Drone_AB[final_id][0]] - 12, index_j0[Drone_AB[final_id][0]])
            Final_Drone = (index_i1[Drone_AB[final_id][1]] - 12, index_j1[Drone_AB[final_id][1]])
            print("First Drone:",First_Drone)
            print("Final Drone:", Final_Drone)

            Path_to_Fianl_BaseStation = []
            if use_BaseStation_1:
                begin = Total_Decay(t=time, BaseStation_B=1,Decay=final_time-time)
                print("Total_Decay_config:",begin )
                Station_1 = True

            else:
                begin = Total_Decay(t=time, BaseStation_B=2,Decay=final_time-time)
                print("Total_Decay_config:", begin)
                Station_1 = False
            
            File(str(begin) + '\n')
            
            result = Position_Path(time=time+time_0,final_time=final_time, First=First_Drone,End=Final_Drone, 
                                                        Path_to_Fianl_BaseStation=Path_to_Fianl_BaseStation,Station_1 = Station_1)
            for r in result:
                File(str(r))
            File('\n')


if __name__ == '__main__':
    m = 150
    n = 150
    Base_D = 70
    Drone_h = 10
    Base_h = 0
    x_00 = 1080  #12
    x_0=45.73 + x_00
    y_0=45.26 
    x_1=1200 +x_00
    y_1=700
    x_2= x_00 -940 
    y_2=1100

    test(m= m, n = n, Base_D= Base_D, Drone_h= Drone_h, Base_h= Base_h, x_00= x_00, 
                    y_0= y_0, y_1= y_1, y_2= y_2)

    # #from config  import Time_begin, Station_begin
    # time_config = Time_begin()
    # station_config = Station_begin()

    # for time_i in range(3):
    #     time = time_config[time_i]
    #     for station_j in range (2):
    #         use_BaseStation_1 = True if station_config[station_j] == 1 else False #選擇基站

    #         #初始化位置
    #         _,_, In_Communicate_scale_toX0,index_i0,index_j0, time_now_0, time_decay_0,_ = Initial(time=time, m=m, 
    #                                                                                                                                     n=n, x_0= x_0, y_0=y_0)

    #         Drone_to_Drone_communication = []
    #         Drone_AB =[]
    #         Index_AB_extend = []
    #         index_i1_extend = []
    #         index_j1_extend = []
    #         DroneTime = []
    #         conmunication_time = []
    #         Final_Decay = []
    #         time_0_extend = []

    #         if use_BaseStation_1 :
    #             i =0
    #             for time_now in time_now_0: #在與基站0通訊後還在基站1通訊範圍的無人機

    #                 _,_, In_Communicate_scale_toX1,index_i1,index_j1, time_now_1,time_decay_1,_ = Initial(time=time_now, 
    #                                                                                                                                             m=m, n=n, x_0= x_1, y_0=y_1)
    #                 Drone_to_Drone_time, Index_AB = Drone_to_Drone(In_Communicate_scale_toX0[i],
    #                                                                                                                             In_Communicate_scale_toX1,i=i )
    #                 Drone_to_Drone_communication.extend(Drone_to_Drone_time)
    #                 i = i+1
    #                 Index_AB_extend.extend(Index_AB)
    #                 index_i1_extend.extend(index_i1)
    #                 index_j1_extend.extend(index_j1)
    #                 time_0_extend.extend([time_now])
                

    #             id = 0
    #             for Drone_to_Drone_communication in Drone_to_Drone_communication:
    #                 Drone_x_t, Drone_y_t, Drone_z_t = Plane_Location(time=time_now+Drone_to_Drone_communication,
    #                                                                                                 m=index_i1_extend[id], n=index_j1_extend[id], H=Drone_h)
    #                 Distance_to_end = distance(x=Drone_x_t, y =Drone_y_t, z =Drone_z_t,x_0 =x_1,
    #                                                                         y_0=y_1,z_0=Base_h )
                    
    #                 if Distance_to_end  <= Base_D:
    #                     Drone_AB.append(Index_AB_extend[id])
    #                     DroneTime.append(Drone_to_Drone_communication)
    #                     conmunication_time.append(CommunicationTime(S=Distance_to_end))
    #                 id = id+1

    #         else:
    #             i =0
    #             for time_now in time_now_0: #在與基站0通訊後還在基站2通訊範圍的無人機

    #                 _,_, In_Communicate_scale_toX2,index_i1,index_j1, time_now_1,time_decay_1,_ = Initial(time=time_now, 
    #                                                                                                                                             m=m, n=n, x_0= x_2, y_0=y_2)
    #                 Drone_to_Drone_time, Index_AB = Drone_to_Drone(In_Communicate_scale_toX0[i],
    #                                                                                                                             In_Communicate_scale_toX2,i=i )
    #                 Drone_to_Drone_communication.extend(Drone_to_Drone_time)
    #                 i = i+1
    #                 Index_AB_extend.extend(Index_AB)
    #                 index_i1_extend.extend(index_i1)
    #                 index_j1_extend.extend(index_j1)
    #                 time_0_extend.extend([time_now])
                
    #             id = 0
    #             for Drone_to_Drone_communication in Drone_to_Drone_communication:
    #                 Drone_x_t, Drone_y_t, Drone_z_t = Plane_Location(time=time_now+Drone_to_Drone_communication,
    #                                                                                                 m=index_i1_extend[id], n=index_j1_extend[id], H=Drone_h)
    #                 Distance_to_end = distance(x=Drone_x_t, y =Drone_y_t, z =Drone_z_t,x_0 =x_2,
    #                                                                         y_0=y_2,z_0=Base_h )
                    
    #                 if Distance_to_end  <= Base_D:
    #                     Drone_AB.append(Index_AB_extend[id])
    #                     DroneTime.append(Drone_to_Drone_communication)
    #                     conmunication_time.append(CommunicationTime(S=Distance_to_end))
    #                 id = id+1

    #         for a,b in zip(DroneTime,conmunication_time):
    #             summ = a+b
    #             Final_Decay.append(summ)

    #         Final_Decay_extend = []
    #         for c in range(len(Drone_AB)):
    #             summ = Final_Decay[c] + time_0_extend[Drone_AB[c][0]]
    #             Final_Decay_extend.append(summ)

    #         final_time = min(Final_Decay_extend)
    #         final_id = Final_Decay_extend.index(final_time)
    #         time_0 = final_time - Final_Decay[final_id]

    #     ###################################################
    #         print("index_i0:", index_i0)
    #         print("index_j0:", index_j0)
    #         print("index_i1:", index_i1)
    #         print("index_j1:", index_j1)
    #         print("Index_AB_extend:", Index_AB_extend)

    #         print("Drone_AB", Drone_AB)  #有效的AB
    #         print("DroneTime", DroneTime) #无人机延时
    #         print("conmunication_time", conmunication_time) #接受延时
    #         print("time_0_extend",time_0_extend) #输出延时
    #         print("Final_Decay_extend",Final_Decay_extend) #所有可能延时
    #         print("final_time",final_time) #最小延时
    #         print("final_id",final_id) #最小延时Drone A B配置
    #         First_Drone =  (index_i0[Drone_AB[final_id][0]] - 12, index_j0[Drone_AB[final_id][0]])
    #         Final_Drone = (index_i1[Drone_AB[final_id][1]] - 12, index_j1[Drone_AB[final_id][1]])
    #         print("First Drone:",First_Drone)
    #         print("Final Drone:", Final_Drone)

    #         Path_to_Fianl_BaseStation = []
    #         if use_BaseStation_1:
    #             begin = Total_Decay(t=time, BaseStation_B=1,Decay=final_time-time)
    #             print("Total_Decay_config:",begin )
    #             Station_1 = True

    #         else:
    #             begin = Total_Decay(t=time, BaseStation_B=2,Decay=final_time-time)
    #             print("Total_Decay_config:", begin)
    #             Station_1 = False
            
    #         File(str(begin) + '\n')
            
    #         result = Position_Path(time=time+time_0,final_time=final_time, First=First_Drone,End=Final_Drone, 
    #                                                     Path_to_Fianl_BaseStation=Path_to_Fianl_BaseStation,Station_1 = Station_1)
    #         for r in result:
    #             File(str(r))
    #         File('\n')



    
 

