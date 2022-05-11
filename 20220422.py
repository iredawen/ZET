
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
    
    def __init__(self,m,n, time_1=None,time_0=None,send_time=None):
        """
        m, n 计算位置的点数
        time_0 基站与Drone连接的时间
        send_time Drone to Drone
        time_1

        """

        self.m  = m
        self.n = n
        self.time_1 = time_1
        self.time_0 = time_0
        self.send_time = send_time
        self.send_time_list = []
        

    def get_send_time(self):
        return self.send_time_list.append(self.send_time)
    def get_send_time_list(self):
        return self.send_time_list



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
    return t

def Initial(time, m, n, x_0,  y_0, time_id = None, H=10, d=125, D=70,  h=0):
    time = time
    m = m
    n = n
    time_id = time_id
    Plane_Distribution = []
    Drone_Communicate_scale = []
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
                
                t_communication =  CommunicationTime( Initial_Distance_to_BaseStation[i][j] )#通訊時間
                Plane_x_t, Plane_y_t, Plane_z_t =Plane_Location(time=time+t_communication, m=i, n=j,H=H)
                FinalDistance = distance(Plane_x_t,Plane_y_t, Plane_z_t, x_0=x_0, y_0=y_0, z_0=h)
                if FinalDistance < D :
                    id_ok = time_id
                    drone = Drone(m=i,n=j,time_0=t_communication)
                    Drone_Communicate_scale.append(drone) #可完成通訊的無人機
                    index_i.append(i)
                    index_j.append(j)
                    #print(t_communication + time)
                    time_now.append(t_communication + time) #當前時間
                    time_decay.append(t_communication) #成立情況下的不同路徑的延時時間
                    
    
    return Plane_Distribution, Initial_Distance_to_BaseStation, Drone_Communicate_scale, index_i, index_j, time_now, time_decay,id_ok

def Drone_to_Drone(Point_A, In_Communicate_scale_B, i,num_x,num_y, x0,y0,
                                            D=70,d_intraOrbit = 90, d_interOrbit = 80):
    Time = []
    Index = []

    for j in range(len(In_Communicate_scale_B)): #遍历终点附近所有无人机
        Drone =In_Communicate_scale_B[j]
        x1 = Drone.m
        y1 = Drone.n
        x0 = Point_A.m
        y0 = Point_A.n
        # x1 = num_x[j]
        # y1 = num_y[j]
        x_num = abs(x0-x1)
        y_num = abs(y0-y1)

        min_num = min(abs(x_num),abs(y_num))
        max_num = max(abs(x_num),abs(y_num))

        b = pow(d_intraOrbit,2)+pow(d_interOrbit,2)
        a = pow(b,0.5)
        if x_num <= y_num:
            time = min_num * CommunicationTime(S=a) + (max_num-min_num) * CommunicationTime(S=d_interOrbit)
            Point_A.send_time = time
            Point_A.get_send_time()

        else:
            time = min_num * CommunicationTime(S=a) + (max_num-min_num)* CommunicationTime(S=d_intraOrbit)
            Point_A.send_time = time
            Point_A.get_send_time()

        Time.append(time)
        index = (i, j)
        Index.append(index)
        j=j+1


    return Time, Index
      
def Total_Decay(t, BaseStation_B, Decay, BaseStation_A= 0):
    initial_time_config = (t, BaseStation_A, BaseStation_B, float(format(Decay,'.4f'))) #初始時刻 基站A 基站B 總延時

    return initial_time_config

def Position_Path(time, time_0,final_time,First, End,Path_to_Fianl_BaseStation,
                                        d_intraOrbit = 90, d_interOrbit = 80, Station_1 = True):
    FinalDrone = End
    FirstDrone = First
    t = time
    final_time = final_time

    Path_to_Fianl_BaseStation.append((float(format(time_0,'.4f')), FirstDrone[0],FirstDrone[1])) #基站A -> 無人機
    t = time_0
    a = abs(FirstDrone[0]-FinalDrone[0]) #行
    b = abs(FirstDrone[1]-FinalDrone[1]) #列
    if Station_1:
        flag = 0
    else:
        flag = 1

    m = FirstDrone[0]
    n = FirstDrone[1]
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
            t = t + CommunicationTime(S=d_interOrbit)
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
        1
    ] 
    return station_config

#@test_time
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
            print("========================================================================")
            use_BaseStation_1 = True if station_config[station_j] == 1 else False #選擇基站

            #初始化位置
            _,_, Drone_Communicate_scale_X0,index_i0,index_j0, time_now_0, _,_ = Initial(time=time, m=m, 
                                                                                                                                        n=n, x_0= x_0, y_0=y_0, D=70)
#Drone_Communicate_scale_X0 在范围内的无人机列表，m n time_0
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

            if use_BaseStation_1 :
                #i =0
                #for time_now in time_now_0: #在與基站0通訊後還在基站1通訊範圍的無人機
                for z in range(len(time_now_0)): 

                    _,_, Drone_Communicate_scale_X1,index_i1,index_j1, _,_,_ = Initial(time=time_now_0[z], 
                                                                                                                                                m=m, n=n, x_0= x_1, y_0=y_1, D=140)
                    
                    Drone_to_Drone_time, Index_AB = Drone_to_Drone(Drone_Communicate_scale_X0[z],
                                                                                                                                Drone_Communicate_scale_X1,i=z,
                                                                                                                                num_x= index_i1,
                                                                                                                                num_y=index_j1,
                                                                                                                                x0= index_i0[z],
                                                                                                                                y0=index_j0[z])
                    Drone_to_Drone_communication.extend(Drone_to_Drone_time)
                    #i = i+1
                    Index_AB_extend.extend(Index_AB)   #A ：1 2 3 4
                    index_i1_extend.extend(index_i1)                            #B: 1 2 3 
                    index_j1_extend.extend(index_j1)
                    time_0_extend.extend(time_now[z])
                print("time0_class:",time_0_extend)    

                time0 = []

                for z in range(len(Drone_Communicate_scale_X0)):
                    for zz in range(len(Drone_Communicate_scale_X1)):
                        time0.append(time_0_extend[z])
                #time0 = time_0_extend*len(Drone_Communicate_scale_X1)
                             
                Drone_0 = []
                for i in range(len(Drone_Communicate_scale_X0)):
                    for j in range(len(Drone_Communicate_scale_X1)):
                        time0_Drone = Drone_Communicate_scale_X0[i].time_0 #输出时间
                        send_time_list = Drone_Communicate_scale_X0[i].get_send_time_list()
                        Drone_communication =  send_time_list[j] #传输时间
                        Drone_x_t, Drone_y_t, Drone_z_t = Plane_Location(time=time0_Drone+Drone_communication,
                                                                                                     m=Drone_Communicate_scale_X1[j].m,
                                                                                                     n=Drone_Communicate_scale_X1[j].n, H=Drone_h)
                        Distance_to_end = distance(x=Drone_x_t, y =Drone_y_t, z =Drone_z_t,x_0 =x_1,
                                                                             y_0=y_1,z_0=Base_h )
                        if Distance_to_end  < Base_D: #无人机互传后在范围内
                            Communication = CommunicationTime(S=Distance_to_end)
                            Drone_x_t, Drone_y_t, Drone_z_t = Plane_Location(
                                                                                                    time=time0_Drone+Drone_communication + Communication,
                                                                                                    m=Drone_Communicate_scale_X1[j].m,
                                                                                                    n=Drone_Communicate_scale_X1[j].n, H=Drone_h)
                            Distance_final = distance(x=Drone_x_t, y =Drone_y_t, z =Drone_z_t,x_0 =x_1,
                                                                                y_0=y_1,z_0=Base_h )
                            if Distance_final < Base_D:
                                Drone_Communicate_scale_X1[j].time_1 = Communication
                                Drone_AB.append(Index_AB_extend[j])
                                DroneTime.append(Drone_communication)
                                conmunication_time.append(Communication) #Drone - b1
                                time0_real.append(time0_Drone) #8
                                Drone_0.append(Drone_Communicate_scale_X0[i])
                                                                            
                """
                # id = 0 
                # for j in range(len(Drone_to_Drone_communication)):
                #     time0_Drone = time0[j]
                #     Drone_communication = Drone_to_Drone_communication[j]
                #     Drone_x_t, Drone_y_t, Drone_z_t = Plane_Location(time=time0_Drone+Drone_communication,
                #                                                                                     m=index_i1_extend[j], n=index_j1_extend[j], H=Drone_h)
                #     Distance_to_end = distance(x=Drone_x_t, y =Drone_y_t, z =Drone_z_t,x_0 =x_1,
                #                                                             y_0=y_1,z_0=Base_h )
                    
                #     if Distance_to_end  < Base_D: #无人机互传后在范围内
                #         Communication = CommunicationTime(S=Distance_to_end)
                #         Drone_x_t, Drone_y_t, Drone_z_t = Plane_Location(
                #                                                                                 time=time0_Drone+Drone_communication + Communication,
                #                                                                                 m=index_i1_extend[j], n=index_j1_extend[j], H=Drone_h)
                #         Distance_final = distance(x=Drone_x_t, y =Drone_y_t, z =Drone_z_t,x_0 =x_1,
                #                                                             y_0=y_1,z_0=Base_h )
                #         if Distance_final < Base_D:
                #             Drone_AB.append(Index_AB_extend[j])
                #             DroneTime.append(Drone_communication)
                #             conmunication_time.append(Communication) #Drone - b1
                #             time0_real.append(time0_Drone)#8
                
                # print("id:", (Index_AB_extend))
                # print("Drone_time", len(Drone_to_Drone_communication))
                # print("time0_Drone", len(time0))
                # print("time0_real:",time0_real)
                """

            else:
                i =0
                for time_now in time_now_0: #在與基站0通訊後還在基站2通訊範圍的無人機

                    _,_, In_Communicate_scale_toX2,index_i1,index_j1, time_now_1,time_decay_1,_ = Initial(time=time_now, 
                                                                                                                                                m=m, n=n, x_0= x_2, y_0=y_2)
                    Drone_to_Drone_time, Index_AB = Drone_to_Drone(In_Communicate_scale_toX0[i],
                                                                                                                                In_Communicate_scale_toX2,i=i,
                                                                                                                                num_x= index_i1,
                                                                                                                                num_y=index_j1,
                                                                                                                                x0= index_i0[i],
                                                                                                                                y0=index_j0[i] )
                    Drone_to_Drone_communication.extend(Drone_to_Drone_time)
                    i = i+1
                    Index_AB_extend.extend(Index_AB)
                    index_i1_extend.extend(index_i1)
                    index_j1_extend.extend(index_j1)
                    time_0_extend.extend([time_now])
                time0 = time_0_extend*len(In_Communicate_scale_toX2)
                print("time_0_extend",time_0_extend)
                
                id = 0
                for Drone_to_Drone_communication in Drone_to_Drone_communication:
                    Drone_x_t, Drone_y_t, Drone_z_t = Plane_Location(time=time0[id]+Drone_to_Drone_communication,
                                                                                                    m=index_i1_extend[id], n=index_j1_extend[id], H=Drone_h)
                    Distance_to_end = distance(x=Drone_x_t, y =Drone_y_t, z =Drone_z_t,x_0 =x_2,
                                                                            y_0=y_2,z_0=Base_h )
                    
                    if Distance_to_end  < Base_D:
                        Communication = CommunicationTime(S=Distance_to_end)
                        Drone_x_t, Drone_y_t, Drone_z_t = Plane_Location(time=time0[id]+Drone_to_Drone_communication + Communication,
                                                                                                    m=index_i1_extend[id], n=index_j1_extend[id], H=Drone_h)
                        Distance_final = distance(x=Drone_x_t, y =Drone_y_t, z =Drone_z_t,x_0 =x_2,
                                                                            y_0=y_2,z_0=Base_h )
                        if Distance_final < Base_D:
                            Drone_AB.append(Index_AB_extend[id])
                            DroneTime.append(Drone_to_Drone_communication)
                            conmunication_time.append(Communication)
                            time0_real.append(time0[id])
                    id = id+1

            # decay_class = []
            # for zz in range(len(Drone_AB)):
            #     time_0 = Drone_Communicate_scale_X0[Drone_AB[zz][0]].time_0
            #     time_drone = Drone_Communicate_scale_X0[Drone_AB[zz][0]].get_send_time_list()[Drone_AB[zz][1]]
            #     time_1 = Drone_Communicate_scale_X0[Drone_AB[zz][1]].time_1
            #     decay = time_0 + time_drone + time_1
            #     decay_class.append(decay)

            for a,b in zip(DroneTime,conmunication_time): #D-D D- B1
                summ = a+b
                Final_Decay.append(summ)

            #print(len(Final_Decay)) #8
            Final_Decay_extend = []

            for a,b in zip(Final_Decay,time0_real):  #B0 - D   D-D   D-B1 8
                summ = a+b
                Final_Decay_extend.append(summ)

            final_time = min(Final_Decay_extend)
            final_id = Final_Decay_extend.index(final_time) #8个中的下标

    
            time_0 = Drone_0[Drone_AB[final_id][0]].time_0 #在8个中选择
            print("final_time",Final_Decay_extend)
            print(final_id)

            First_Drone =  (index_i0[Drone_AB[final_id][0]] - 12, index_j0[Drone_AB[final_id][0]])
            Final_Drone = (index_i1[Drone_AB[final_id][1]] - 12, index_j1[Drone_AB[final_id][1]])
        
        # ##############################
            print("First Drone:",First_Drone)
            print("Final Drone:", Final_Drone)
            print("Drone_AB:", Drone_AB)
            print("index_j0",index_j0)
            print("index_i0",index_i0)

            Path_to_Fianl_BaseStation = [] #路径
            if use_BaseStation_1:
                begin = Total_Decay(t=time, BaseStation_B=1,Decay=final_time-time)
                for i in range(len(begin)-1):
                    print(begin[i],end=",")
                print(begin[i+1])
                Station_1 = True

            else:
                begin = Total_Decay(t=time, BaseStation_B=2,Decay=final_time-time)
                for i in range(len(begin)-1):
                    print(begin[i],end=",")
                print(begin[i+1])
                Station_1 = False
            
################################
            for i in range(len(begin)-1):
                File(str(begin[i]) + ",")
            File(str(begin[i+1]))
            File('\n')
            
            result = Position_Path(time=time,time_0= time_0,final_time=final_time, First=First_Drone,End=Final_Drone, 
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
