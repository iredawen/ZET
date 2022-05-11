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
"""""
一条路径复用
一种行走方式（全行-列） --- 518.84,,
两种行走方式（先列-行，剩下的行-列 )  ---   518.45 
两种行走方式（先行-列，剩下的列-行 )  ---   518.49
"""
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

    for PointB in range(len(In_Communicate_scale_B)): 
        x1 = num_x[PointB] #终点坐标
        y1 = num_y[PointB]
        x_num = abs(x0-x1) #行差
        y_num = abs(y0-y1) #列差
        time = float(format(x_num * CommunicationTime(S=d_intraOrbit) + y_num * CommunicationTime(S=d_interOrbit),'.4f'))
  
        Time.append(time)
        index = (i, PointB)
        Index.append(index)

    return Time, Index
      
def Total_Decay(t, BaseStation_B, Decay, BaseStation_A,j):
    if j <3:
        initial_time_config = (float(format(t,'.4f')),BaseStation_A,BaseStation_B,float(format(Decay,'.4f')),3) #初始時刻 基站A 基站B 總延時
    else:
        initial_time_config = (float(format(t,'.4f')),BaseStation_A,BaseStation_B,float(format(Decay,'.4f')),1)
    return initial_time_config

def Position_Path(id_list, index,time_0,First, End,Path_to_Fianl_BaseStation,
                                        d_intraOrbit = 90, d_interOrbit = 80, Station_x = True, Station_y = True,
                                        result_copy=None,final=None):
    FinalDrone = End
    FirstDrone = First

    index = index #信号批次
    t = float(format(time_0,'.4f'))
    #
    # print("result_copy",result_copy)

    Path_to_Fianl_BaseStation.append((float(format(t,'.4f')),FirstDrone[0],FirstDrone[1])) #初始点

    a = abs(FirstDrone[0]-FinalDrone[0]) #行差
    b = abs(FirstDrone[1]-FinalDrone[1]) #列差
    if Station_x:
        flag_x = 0
    else:
        flag_x = 1
    if Station_y:
        flag_y = 0
    else:
        flag_y = 1

    m = FirstDrone[0] 
    n = FirstDrone[1]


    if index == 0:  #第1批次
        for i in range(b):#先列后行
            m = m
            n = n + (-1)**flag_y
            t = t + CommunicationTime(S=d_interOrbit)
            Path_to_Fianl_BaseStation.append((float(format(t,'.4f')),m,n))
        for j in range(a): 
                m = m + (-1)**flag_x
                n= n
                t = t + CommunicationTime(S=d_intraOrbit)
                Path_to_Fianl_BaseStation.append((float(format(t,'.4f')),m,n))
        

    elif index == 1 or index == 3: #两种行走方式
    #else: #内收仅一种行走方式
        result_copy =result_copy
        for j in range(a):    #先行后列
            m = m + (-1)**flag_x
            n= n
            t1 = CommunicationTime(S=d_intraOrbit)
            t = t + t1
            t_last = Path_to_Fianl_BaseStation[-1][0]
            m_last = Path_to_Fianl_BaseStation[-1][1]
            n_last = Path_to_Fianl_BaseStation[-1][2]
            for re in range(len(result_copy)):
                if m==result_copy[re][1] and n == result_copy[re][2] and\
                                                    m_last== result_copy[re -1][1] and n_last == result_copy[re -1][2] and t_last < result_copy[re][0]: #当前位置重合
                    Path_to_Fianl_BaseStation[-1] = (float(format(result_copy[re][0] ,'.4f')),m_last,n_last)
                    t = result_copy[re][0]+t1 #等待后到达目标点的时间
                else:
                    t =t
            Path_to_Fianl_BaseStation.append((float(format(t,'.4f')),m,n))
        
        for i in range(b):
            m = m
            n = n + (-1)**flag_y
            t1 = CommunicationTime(S=d_interOrbit)
            t = t + t1
            t_last = Path_to_Fianl_BaseStation[-1][0]
            m_last = Path_to_Fianl_BaseStation[-1][1]
            n_last = Path_to_Fianl_BaseStation[-1][2]
            for re in range(len(result_copy)):
                if m==result_copy[re][1] and n == result_copy[re][2] and\
                                                        m_last== result_copy[re -1][1] and n_last == result_copy[re -1][2] and t_last < result_copy[re][0]:
                    Path_to_Fianl_BaseStation[-1] = (float(format(result_copy[re][0] ,'.4f')),m_last,n_last)
                    t = result_copy[re][0]+t1
                else:
                    t =t
            Path_to_Fianl_BaseStation.append((float(format(t,'.4f')),m,n))
        
        if Path_to_Fianl_BaseStation[-1][0] < result_copy[-1][0]:
            Path_to_Fianl_BaseStation[-1] = (float(format(result_copy[-1][0] ,'.4f')),m,n)


    else: #index == 2
        if index==2 : #第2条
            result_copy =result_copy
            for i in range(b):
                m = m
                n = n + (-1)**flag_y
                t1 = CommunicationTime(S=d_interOrbit)
                t = t + t1
                t_last = Path_to_Fianl_BaseStation[-1][0]
                m_last = Path_to_Fianl_BaseStation[-1][1]
                n_last = Path_to_Fianl_BaseStation[-1][2]
                for re in range(len(result_copy)):
                    if m==result_copy[re][1] and n == result_copy[re][2] and\
                                                                    m_last== result_copy[re -1][1] and n_last == result_copy[re -1][2] and t_last < result_copy[re][0]: #当前位置重合
                        Path_to_Fianl_BaseStation[-1] = (float(format(result_copy[re][0] ,'.4f')),m_last,n_last)
                        t = result_copy[re][0]+t1 #等待后到达目标点的时间
                    else:
                        t =t
                Path_to_Fianl_BaseStation.append((float(format(t,'.4f')),m,n))

            for j in range(a): #先列后行
                m = m + (-1)**flag_x
                n= n
                t1 = CommunicationTime(S=d_intraOrbit)
                t = t + t1
                t_last = Path_to_Fianl_BaseStation[-1][0]
                m_last = Path_to_Fianl_BaseStation[-1][1]
                n_last = Path_to_Fianl_BaseStation[-1][2]
                for re in range(len(result_copy)):
                    if m==result_copy[re][1] and n == result_copy[re][2] and\
                                                                    m_last== result_copy[re -1][1] and n_last == result_copy[re -1][2] and t_last < result_copy[re][0]: #当前位置重合
                        Path_to_Fianl_BaseStation[-1] = (float(format(result_copy[re][0] ,'.4f')),m_last,n_last)
                        t = result_copy[re][0]+t1 #等待后到达目标点的时间
                        
                    else:
                        t =t
                        
                Path_to_Fianl_BaseStation.append((float(format(t,'.4f')),m,n))

    return Path_to_Fianl_BaseStation, Path_to_Fianl_BaseStation[-1][0]

def elemet_list(Final_Decay_extend,num=4):
    final_time_list = []
    time_list_save = Final_Decay_extend
    for jj in range(num):
        final_time_list.append(min(time_list_save))
        time_list_save.remove(min(time_list_save))
    return final_time_list

def find_id(save_list,fianl_time_list):
    id_list = []
    for i in range(len(fianl_time_list)):
        final_id = save_list.index(fianl_time_list[i])
        id_list.append(final_id)
    return id_list

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
    # station_config =[
    #    [0,1,True, True], [0,2,False, True]
    # ] 
    station_config =[
       [0,1,True, True], [0,2,False, True],
       [1, 0, False, False], [1, 2, False, True],
       [2, 0, True, False], [2,1, True, False]
    ] 
    return station_config

def If_in_scale(Path_to_Fianl_BaseStation,result_copy,drone_ab,final,x_0,y_0):
    Path_to_Fianl_BaseStation = Path_to_Fianl_BaseStation
    result_copy = result_copy
    final = final
    if len(result_copy) == 0:
        pass
    else:
        if Path_to_Fianl_BaseStation[-1][0] < result_copy[-1][0]:
            final = float(format(result_copy[-1][0] ,'.4f'))
        else:
            final = final
    loc_x, loc_y, loc_z = Plane_Location(time=final, m=drone_ab.end_m, n=drone_ab.end_n, H=10)
    distance_before = distance(x=loc_x,y=loc_y,z=loc_z,x_0=x_0,y_0=y_0)
    Before = True if distance_before < 70 else False
    communite_time = CommunicationTime(S=distance_before)
    final_time = float(format(final + communite_time,'.4f'))
    loc_x, loc_y, loc_z = Plane_Location(time=final_time, m=drone_ab.end_m, n=drone_ab.end_n, H=10)
    distance_after = distance(x=loc_x,y=loc_y,z=loc_z,x_0=x_0,y_0=y_0)
    After = True if distance_after < 70 else False
    
    return final_time, Before, After


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
    Ponit = [
        [x_0,y_0], #0
        [x_1, y_1], #1
        [x_2, y_2] #2
    ]
    time_config = Time_begin()
    station_config = Station_begin()
    
    Total = 0
    for time_i in range(len(time_config)):  #遍历每一个时间点 3
        time = time_config[time_i]
        Total_2 = 0
        for station_j in range (len(station_config)): # 6 

            #use_BaseStation_1 = True if station_config[station_j][1] == 1 else False #選擇基站

            #初始化位置
            _,_, In_Communicate_scale_toX0,index_i0,index_j0, time_now_0, _,_ = Initial(time=time, m=m, n=n, 
                                                                                                                                                                x_0= Ponit[station_config[station_j][0]][0],   #x_0 = x_0
                                                                                                                                                                y_0= Ponit[station_config[station_j][0]][1], D=70) # y_0 = y_0

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

            i =0
            for time_now in time_now_0: #在與基站0通訊後還在基站1通訊範圍的無人機

                _,_, In_Communicate_scale_toX1,index_i1,index_j1, _,_,_ = Initial(time=time_now,m=m, n=n, 
                                                                                                                                            x_0= Ponit[station_config[station_j][1]][0], #x_0 = x_1
                                                                                                                                            y_0= Ponit[station_config[station_j][1]][1], D=140) #x_0 = y_1
                
                Drone_to_Drone_time, Index_AB = Drone_to_Drone(In_Communicate_scale_toX0[i],
                                                                                                                            In_Communicate_scale_toX1,i=i,
                                                                                                                            num_x= index_i1,
                                                                                                                            num_y=index_j1,
                                                                                                                            x0= index_i0[i],
                                                                                                                            y0=index_j0[i])
                i = i+1                                                                                                            
                Drone_to_Drone_communication.extend(Drone_to_Drone_time)
                Index_AB_extend.extend(Index_AB)
                index_i1_extend.extend(index_i1)
                index_j1_extend.extend(index_j1)
                time_0_extend.append(time_now)


            time0 = []
            Match_Drone = []

            for z in range(len(In_Communicate_scale_toX0)):
                for zz in range(len(In_Communicate_scale_toX1)):
                    time0.append(time_0_extend[z])
                        

            for j in range(len(Drone_to_Drone_communication)): 
                time0_Drone = time0[j]
                Drone_communication = Drone_to_Drone_communication[j]
                Drone_x_t, Drone_y_t, Drone_z_t = Plane_Location(time=float(format(time0_Drone+Drone_communication,'.4f')),
                                                                                                m=index_i1_extend[j], n=index_j1_extend[j], H=Drone_h)
                
                Distance_to_end = distance(x=Drone_x_t, y =Drone_y_t, z =Drone_z_t,
                                                                        x_0 =Ponit[station_config[station_j][1]][0],  #x_0 = x_1
                                                                        y_0=Ponit[station_config[station_j][1]][1], z_0=Base_h ) #y_0 = y_1
                
                if Distance_to_end  < Base_D: #无人机互传后在范围内
                    Communication = CommunicationTime(S=Distance_to_end)
                    Drone_x_t, Drone_y_t, Drone_z_t = Plane_Location(
                                                                                            time=float(format(time0_Drone+Drone_communication + Communication,'.4f')),
                                                                                            m=index_i1_extend[j], n=index_j1_extend[j], H=Drone_h)
                    Distance_final = distance(x=Drone_x_t, y =Drone_y_t, z =Drone_z_t,x_0 =Ponit[station_config[station_j][1]][0],
                                                                        y_0=Ponit[station_config[station_j][1]][1],z_0=Base_h ) #x_0 = x_1  y_0 = y_1
                    if Distance_final < Base_D:
                        drone = Drone(begin_m=index_i0[Index_AB_extend[j][0]],begin_n=index_j0[Index_AB_extend[j][0]],
                                                        end_m=index_i1[Index_AB_extend[j][1]], end_n=index_j1[Index_AB_extend[j][1]],
                                                        time_begin=time0_Drone,time_drone=Drone_communication,time_end=Communication)
                        Match_Drone.append(drone)
                        Drone_AB.append(Index_AB_extend[j])
                        DroneTime.append(Drone_communication)
                        conmunication_time.append(Communication) #Drone - b1
                        time0_real.append(time0_Drone)#8
            

            
            for a,b in zip(DroneTime,conmunication_time): #D-D D- B1
                summ = a+b
                Final_Decay.append(summ)
            #print(len(Final_Decay)) #8
            Final_Decay_extend = []
            for a,b in zip(Final_Decay,time0_real):  #B0 - D   D-D   D-B1 8
                summ = a+b
                Final_Decay_extend.append(summ)
            save_list = Final_Decay_extend.copy()
            final_time_list  = elemet_list(save_list,num=4)
            timeid_list = find_id(Final_Decay_extend,final_time_list)
            drone_ab_list = []
            for j in range(4):
                drone_ab = Match_Drone[timeid_list[j]]
                drone_ab_list.append((drone_ab.begin_m,drone_ab.begin_n,drone_ab.end_m,drone_ab.end_n))


            path_list_1 = []
            path_list = []
            path_list.append(Match_Drone[timeid_list[0]])
            path_id = []
            path_id_1 = []
            path_id.append(timeid_list[0])

            for i in range(3):
                if (Match_Drone[timeid_list[0]].begin_m != Match_Drone[timeid_list[i+1]].begin_m or Match_Drone[timeid_list[0]].begin_n != Match_Drone[timeid_list[i+1]].begin_n):
                    path_list_1.append(Match_Drone[timeid_list[i+1]])
                    path_id_1.append(timeid_list[i+1])

            
            if len(path_list_1) > 1:
                path_list.append(path_list_1[0])
                path_id.append(path_id_1[0])
                for i in range(len(path_list_1)-1):
                    if path_list_1[0].begin_m != path_list_1[i+1].begin_m or path_list_1[0].begin_n != path_list_1[i+1].begin_n:
                        path_list.append(path_list_1[i+1])
                        path_id.append(path_id_1[i+1])
            else:
                path_list = path_list+path_list_1
                path_id = path_id + path_id_1
            

            
            #进行路径计算和规划。

            """
            11 2 3 和 11 22复用,path_id 是推荐路径的下标,在Match_Drone列表中取对应的Drone。path_list是Drone对的列表
            """
            Station_x = station_config[station_j][2]
            Station_y = station_config[station_j][3]
                            
            jj = 0
            Total_1 = 0
            result_copy=[]
            for j in range(4):   
                for ij in range(1): #1111
                    drone_ab = path_list[0]
                    First_Drone =  (drone_ab.begin_m - 12, drone_ab.begin_n)
                    Final_Drone = (drone_ab.end_m - 12, drone_ab.end_n)
                    decay_0 = drone_ab.time_begin-time
                    Path_to_Fianl_BaseStation = [] #路径

                    result, final = Position_Path(id_list=path_list,index=jj,time_0= float(format(drone_ab.time_begin+j*decay_0,'.4f')),
                                                                    First=First_Drone,End=Final_Drone, Path_to_Fianl_BaseStation=Path_to_Fianl_BaseStation,
                                                                    Station_x = Station_x, Station_y = Station_y,result_copy=result_copy)
                    
                    #因为可能有延时，所以输出时间可能改变，甚至无人机飞出去
                    end_time, Before, After = If_in_scale(Path_to_Fianl_BaseStation=result,
                                                                                            result_copy=result_copy,
                                                                                            drone_ab=drone_ab,final=final,
                                                                                            x_0=Ponit[station_config[station_j][1]][0],
                                                                                            y_0=Ponit[station_config[station_j][1]][1])

                    begin = Total_Decay(t=float(format(time,'.4f')), BaseStation_B=station_config[station_j][1],
                                                                    Decay=float(format(end_time-time,'.4f')),
                                                                    BaseStation_A=station_config[station_j][0],j=jj)
                    
                    if j < 3:
                        total = 3*float(format(end_time-time,'.4f'))
                    else:
                        total = float(format(end_time-time,'.4f'))
                    for i in range(len(begin)-1):
                        print(begin[i],end=",")
                    print(begin[i+1])                    
                    for r in range(len(result)-1):
                        print(result[r], end = ",")
                    print(result[r+1])
                    jj = jj+1

                    for result_1 in result:
                        result_copy.append(result_1)
                    result_copy.append((end_time,None,None))

                    for i in range(len(begin)-1):
                        File(str(begin[i]) + ",")
                    File(str(begin[i+1]))
                    File('\n')
                    for i in range(len(result)-1):
                        File(str(result[i]) + ",")
                    File(str(result[i+1]))
                    File('\n')

                    Total_1 = Total_1+total      
            Total_2 = Total_2+Total_1
        Total = Total_2+Total
    print("Total:               ", Total)



if __name__ == '__main__':
    m = 150
    n = 150
    Base_D = 70
    Drone_h = 10
    Base_h = 0
    x_00 = 1080  #12
   #x_0=45.73 + x_00
    y_0=45.26 
    #x_1=1200 +x_00
    y_1=700
    #x_2= x_00 -940 
    y_2=1100

    test(m= m, n = n, Base_D= Base_D, Drone_h= Drone_h, Base_h= Base_h, x_00= x_00, 
                    y_0= y_0, y_1= y_1, y_2= y_2)
    
