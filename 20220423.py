#from functools import wraps
#import time

# ######################
# def test_time(function):
#     @wraps(function)
#     def func_time(*args, **kwargs):
#         t0 = time.perf_counter()
#         result = function(*args, **kwargs)
#         t1 = time.perf_counter()
#         print("Total running time: %s s" % (str(t1 - t0)))
#         return result

#     return func_time
class Drone():
    def __init__(self,begin_m,begin_n,end_m,end_n, time_begin, time_drone, time_end):
        self.begin_m = begin_m
        self.begin_n =begin_n
        self.end_m  = end_m
        self.end_n = end_n
        self.time_begin = time_begin
        self.time_drone = time_drone
        self.time_end = time_end


def Plane_Location(time, m, n, H,  v = 5, d_intraOrbit = 90, d_interOrbit = 80):
    Plane_x_t = v * time + d_intraOrbit * m
    Plane_y_t = d_interOrbit * n
    Plane_z_t = H
    return Plane_x_t, Plane_y_t, Plane_z_t

def distance(x, y, z, x_0, y_0, z_0 = 0):
    x = x_0 - x
    y = y_0 - y
    z = z_0 - z
    dd = pow(x,2)+pow(y,2)+pow(z,2)
    return pow(dd,0.5)

def CommunicationTime(S, t_f = 0.1):
    t = t_f + S/10000
    return float(format(t,'.4f'))

def Initial(time, m, n, x_0,  y_0, time_id = None, H=10, d=125, D=70,  h=0):
    time = float(format(time,'.4f'))
    m = m
    n = n
    time_id = time_id
    Plane_Distribution = []
    In_Communicate_scale = []
    index_i = []
    index_j = []
    time_decay = []
    time_now = []
    Initial_Distance_to_BaseStation = [[0] * m for i in range(n)]
    for i in range(m):
        for j in range(n):
            Plane_x_t, Plane_y_t, Plane_z_t =Plane_Location(time=time, m=i, n=j, H=H)
            Location = [Plane_x_t, Plane_y_t, Plane_z_t]  #t = 某時刻的無人機位置
            Plane_Distribution.append(Location)
            Initial_Distance_to_BaseStation[i][j] = distance(Plane_x_t,Plane_y_t, Plane_z_t, x_0=x_0, y_0=y_0, z_0=h)  
            #t = 0時刻無人機到0電臺的距離
           
            if Initial_Distance_to_BaseStation[i][j] < D :
                
                t_communication =  float(format(CommunicationTime( Initial_Distance_to_BaseStation[i][j] ),'.4f'))#通訊時間
                Plane_x_t, Plane_y_t, Plane_z_t =Plane_Location(time=float(format(time+t_communication,'.4f')), m=i, n=j,H=H)
                FinalDistance = distance(Plane_x_t,Plane_y_t, Plane_z_t, x_0=x_0, y_0=y_0, z_0=h)
                if FinalDistance < D :
                    id_ok = time_id

                    In_Communicate_scale.append(Location) #可完成通訊的無人機

                    index_i.append(i)
                    index_j.append(j)
                    time_now_0 = float(format(time+t_communication,'.4f'))
                    time_now.append(time_now_0) #當前時間
                    time_decay.append(t_communication) #成立情況下的不同路徑的延時時間
                    
    
    return Plane_Distribution, Initial_Distance_to_BaseStation, In_Communicate_scale, index_i, index_j, time_now, time_decay,id_ok

def Drone_to_Drone(Point_A, In_Communicate_scale_B, i,num_x,num_y, x0,y0,
                                            D=70,d_intraOrbit = 90, d_interOrbit = 80):
    Time = []
    Index = []
    PointA = Point_A

    for j in range(len(In_Communicate_scale_B)):
        x1 = num_x[j]
        y1 = num_y[j]
        x_num = abs(x0-x1)
        y_num = abs(y0-y1)

        min_num = min(abs(x_num),abs(y_num))
        max_num = max(abs(x_num),abs(y_num))

        b = pow(d_intraOrbit,2)+pow(d_interOrbit,2)
        a = pow(b,0.5)

        if x_num <= y_num:
            time = float(format(min_num * CommunicationTime(S=a) + (max_num-min_num) * CommunicationTime(S=d_interOrbit),'.4f'))
        else:
            time = float(format(min_num * CommunicationTime(S=a) + (max_num-min_num)* CommunicationTime(S=d_intraOrbit),'.4f'))
       
        Time.append(time)
        index = (i, j)
        Index.append(index)



    return Time, Index
      
def Total_Decay(t, BaseStation_B, Decay, BaseStation_A= 0):
    initial_time_config = (float(format(t,'.4f')), BaseStation_A, BaseStation_B, float(format(Decay,'.4f'))) #初始時刻 基站A 基站B 總延時

    return initial_time_config

def Position_Path(time_0,First_m,First_n,End_m,End_n,Path_to_Fianl_BaseStation,
                                        d_intraOrbit = 90, d_interOrbit = 80, Station_1 = True):
    
    FirstDrone_m = First_m
    FirstDrone_n = First_n
    EndDrone_m = End_m
    EndtDrone_n = End_n
    t = float(format(time_0,'.4f'))

    Path_to_Fianl_BaseStation.append((float(format(t,'.4f')), FirstDrone_m,FirstDrone_n)) #基站A -> 無人機
    #t = time_0
    a = abs(FirstDrone_m-EndDrone_m) #行差
    b = abs(FirstDrone_n-EndtDrone_n) #列差
    if Station_1:
        flag = 0
    else:
        flag = 1

    m = FirstDrone_m 
    n = FirstDrone_n
    d = pow(d_intraOrbit,2)+pow(d_interOrbit,2)
    for i in range(min(a,b)):  #无人机到无人机
        m =  m  +(-1)**flag
        n =  n +1
        t = t+ CommunicationTime(S= pow(d,0.5))
        Path_to_Fianl_BaseStation.append((float(format(t,'.4f')),m, n))

    if a > b: #行数大于列数
        for j in range(a-b):
            m = m + (-1)**flag
            n= n
            t = t + CommunicationTime(S=d_intraOrbit)
            Path_to_Fianl_BaseStation.append((float(format(t,'.4f')),m, n))

    if a < b :
        for j in range(b-a):
            m = m
            n = n + 1 
            t = t +CommunicationTime(S=d_interOrbit)
            Path_to_Fianl_BaseStation.append((float(format(t,'.4f')),m, n))

    return Path_to_Fianl_BaseStation

#############
def File(result):
    import os
    if not os.path.isfile('./result.txt'):
        os.mknod('result.txt')
    file_result = open('./result.txt', mode='r+')
    file_result.read()
    file_result.write(result)
    file_result.close()

def Time_begin():
    time_config =[
        0.0000, 4.7000, 16.4000
    ] 
    return time_config

def Station_begin():
    station_config =[
        1, 2
    ] 
    return station_config

#@test_time
def test(m = 150,
                    n = 150,
                    Base_D = 70,
                    Drone_h = 10,
                    Base_h = 0,
                    x_00 = 1080,  #12 坐标轴移动
                    y_0=45.26,
                    y_1=700,
                    y_2=1100):

    x_0=45.73 + x_00
    x_1=1200 +x_00
    x_2= x_00 -940
    time_config = Time_begin()
    station_config = Station_begin()

    for time_i in range(len(time_config)):  #遍历每一个时间点
        time = time_config[time_i]
        for station_j in range (len(station_config)):
            print("========================================================================")
            use_BaseStation_1 = True if station_config[station_j] == 1 else False #選擇基站

            #初始化位置
            _,_, In_Communicate_scale_toX0,index_i0,index_j0, time_now_0, _,_ = Initial(time=time, m=m, 
                                                                                                                                        n=n, x_0= x_0, y_0=y_0, D=70)

            Drone_to_Drone_communication = []
            Drone_AB =[]
            Index_AB_extend = []
            index_i1_extend = []
            index_j1_extend = []
            DroneTime = []
            conmunication_time = []
            Final_Decay = []
            time_0_extend = []
            time0_real = []
            Matched_Drone = []

            if use_BaseStation_1 :

                for i in range(len(time_now_0)):
                    #for time_now in time_now_0: #在與基站0通訊後還在基站1通訊範圍的無人機

                    _,_, In_Communicate_scale_toX1,index_i1,index_j1, _,_,_ = Initial(time=time_now_0[i], 
                                                                                                                                                m=m, n=n, x_0= x_1, y_0=y_1, D=140)
                    
                    Drone_to_Drone_time, Index_AB = Drone_to_Drone(In_Communicate_scale_toX0[i],
                                                                                                                                In_Communicate_scale_toX1,i=i,
                                                                                                                                num_x= index_i1,
                                                                                                                                num_y=index_j1,
                                                                                                                                x0= index_i0[i],
                                                                                                                                y0=index_j0[i])
                                                                                                   
                    Drone_to_Drone_communication.extend(Drone_to_Drone_time) #begin - end  配对完成
                    Index_AB_extend.extend(Index_AB)
                    index_i1_extend.extend(index_i1)  # 123 123 123 123
                    index_j1_extend.extend(index_j1)
                    time_0_extend.append(time_now_0[i])
                print("time0_class:",time_0_extend)    

                time0 = []

                for z in range(len(In_Communicate_scale_toX0)):
                    for zz in range(len(In_Communicate_scale_toX1)):
                        time0.append(time_0_extend[z])
                            
                for k in range(len(In_Communicate_scale_toX0)):
                    for n in range(len(In_Communicate_scale_toX1)):
                        time0_Drone = time_0_extend[k]
                        Drone_communication = Drone_to_Drone_communication[k+n]
                        Drone_x_t, Drone_y_t, Drone_z_t = Plane_Location(time=float(format(time0_Drone+Drone_communication,'.4f')),
                                                                                                    m=index_i1[n], n=index_j1[n], H=Drone_h)
                        Distance_to_end = distance(x=Drone_x_t, y =Drone_y_t, z =Drone_z_t,x_0 =x_1,
                                                                            y_0=y_1,z_0=Base_h )
                        
                        if Distance_to_end  < Base_D:
                            Communication = CommunicationTime(S=Distance_to_end)
                            Drone_x_t, Drone_y_t, Drone_z_t = Plane_Location(
                                                                                                time=float(format(time0_Drone+Drone_communication + Communication,'.4f')),
                                                                                                m=index_i1[n], n=index_j1[n], H=Drone_h)
                            Distance_final = distance(x=Drone_x_t, y =Drone_y_t, z =Drone_z_t,x_0 =x_1,
                                                                            y_0=y_1,z_0=Base_h )
                            
                            if Distance_final < Base_D:
                                drone = Drone(begin_m=index_i0[k],begin_n=index_j0[k],end_m=index_i1[n],end_n=index_j1[n],
                                                                time_begin=time0_Drone, time_drone=Drone_communication, time_end=Communication)
                                Matched_Drone.append(drone)

                for j in range(len(Matched_Drone)):
                    print("begin(m,n):", (Matched_Drone[j].begin_m, Matched_Drone[j].begin_n))
                    print("end(m,n):", (Matched_Drone[j].end_m, Matched_Drone[j].end_n))
                    print("Drone_time", Matched_Drone[j].time_drone)
                    print("timebegin_Drone", Matched_Drone[j].time_begin)
                    print("timeDrone_end:",Matched_Drone[j].time_end)
                    print("11111111111111111111111111111111")


            else:
                for i in range(len(time_now_0)):

                    _,_, In_Communicate_scale_toX1,index_i1,index_j1, _,_,_ = Initial(time=time_now_0[i], 
                                                                                                                                                m=m, n=n, x_0= x_2, y_0=y_2, D=140)
                    
                    Drone_to_Drone_time, Index_AB = Drone_to_Drone(In_Communicate_scale_toX0[i],
                                                                                                                                In_Communicate_scale_toX1,i=i,
                                                                                                                                num_x= index_i1,
                                                                                                                                num_y=index_j1,
                                                                                                                                x0= index_i0[i],
                                                                                                                                y0=index_j0[i])
                                                                                                   
                    Drone_to_Drone_communication.extend(Drone_to_Drone_time) #begin - end  配对完成
                    Index_AB_extend.extend(Index_AB)
                    index_i1_extend.extend(index_i1)  # 123 123 123 123
                    index_j1_extend.extend(index_j1)
                    time_0_extend.append(time_now_0[i])
                print("time0_class:",time_0_extend)    

                time0 = []

                for z in range(len(In_Communicate_scale_toX0)):
                    for zz in range(len(In_Communicate_scale_toX1)):
                        time0.append(time_0_extend[z])
                            
                for k in range(len(In_Communicate_scale_toX0)):
                    for n in range(len(In_Communicate_scale_toX1)):
                        time0_Drone = time_0_extend[k]
                        Drone_communication = Drone_to_Drone_communication[k+n]
                        Drone_x_t, Drone_y_t, Drone_z_t = Plane_Location(time=float(format(time0_Drone+Drone_communication,'.4f')),
                                                                                                    m=index_i1[n], n=index_j1[n], H=Drone_h)
                        Distance_to_end = distance(x=Drone_x_t, y =Drone_y_t, z =Drone_z_t,x_0 =x_2,
                                                                            y_0=y_2,z_0=Base_h )
                        
                        if Distance_to_end  < Base_D:
                            Communication = CommunicationTime(S=Distance_to_end)
                            Drone_x_t, Drone_y_t, Drone_z_t = Plane_Location(
                                                                                                time=float(format(time0_Drone+Drone_communication + Communication,'.4f')),
                                                                                                m=index_i1[n], n=index_j1[n], H=Drone_h)
                            Distance_final = distance(x=Drone_x_t, y =Drone_y_t, z =Drone_z_t,x_0 =x_2,
                                                                            y_0=y_2,z_0=Base_h )
                            
                            if Distance_final < Base_D:
                                drone = Drone(begin_m=index_i0[k],begin_n=index_j0[k],end_m=index_i1[n],end_n=index_j1[n],
                                                                time_begin=time0_Drone, time_drone=Drone_communication, time_end=Communication)
                                Matched_Drone.append(drone)

                for j in range(len(Matched_Drone)):
                    print("begin(m,n):", (Matched_Drone[j].begin_m, Matched_Drone[j].begin_n))
                    print("end(m,n):", (Matched_Drone[j].end_m, Matched_Drone[j].end_n))
                    print("Drone_time", Matched_Drone[j].time_drone)
                    print("timebegin_Drone", Matched_Drone[j].time_begin)
                    print("timeDrone_end:",Matched_Drone[j].time_end)
                    print("11111111111111111111111111")

            min_decay = 100000
            Drone_id = 100000
            for b in range(len(Matched_Drone)):
                decay = Matched_Drone[b].time_begin + Matched_Drone[b].time_drone + Matched_Drone[b].time_end
                print("decay",decay)
                if decay < min_decay:
                    min_decay = decay
                    Drone_id = b

            print("final_time",min_decay)
            print("final_id",Drone_id)

            print("First Drone:",(Matched_Drone[Drone_id].begin_m-12,Matched_Drone[Drone_id].begin_n))
            print("Final Drone:", (Matched_Drone[Drone_id].end_m-12,Matched_Drone[Drone_id].end_n))

            Path_to_Fianl_BaseStation = [] #路径
            if use_BaseStation_1:
                begin = Total_Decay(t=float(format(time,'.4f')), BaseStation_B=1,Decay=float(format(min_decay-time,'.4f')))
                for i in range(len(begin)-1):
                    print(begin[i],end=",")
                print(begin[i+1])
                Station_1 = True

            else:
                begin = Total_Decay(t=float(format(time,'.4f')), BaseStation_B=2,Decay=float(format(min_decay-time,'.4f')))
                for i in range(len(begin)-1):
                    print(begin[i],end=",")
                print(begin[i+1])
                Station_1 = False
            
################################
            for i in range(len(begin)-1):
                File(str(begin[i]) + ",")
            File(str(begin[i+1]))
            File('\n')
            
            result = Position_Path(time_0= float(format(Matched_Drone[Drone_id].time_begin,'.4f')),
                                                        First_m=Matched_Drone[Drone_id].begin_m,First_n=Matched_Drone[Drone_id].begin_n,
                                                        End_m=Matched_Drone[Drone_id].end_m,End_n=Matched_Drone[Drone_id].end_n,
                                                        Path_to_Fianl_BaseStation=Path_to_Fianl_BaseStation,Station_1 = Station_1)
            
################################
            for i in range(len(result)-1):
                File(str(result[i]) + ",")
            File(str(result[i+1]))
            File('\n')

            for r in range(len(result)-1):
                print(result[r], end = ",")

            print(result[r+1])


if __name__ == '__main__':
    m = 200
    n = 200
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
