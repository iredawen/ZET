from re import I
import numpy

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
      
def Time_decay(time_A, time_drone, time_B):
    return time_A + time_drone + time_B

def Total_Decay(t, BaseStation_B, Decay, BaseStation_A= 0):
    initial_time_config = [t, BaseStation_A, BaseStation_B, Decay] #初始時刻 基站A 基站B 總延時

    return initial_time_config

def Position_Path(time, index_i, index_j, Decay, Path_to_Fianl_BaseStation):

    Path_to_Fianl_BaseStation.append([time + Decay,index_i, index_j]) #基站A -> 無人機

    return Path_to_Fianl_BaseStation



if __name__ == '__main__':
    m = 150
    n = 150
    Base_D = 70
    Drone_h = 10
    Base_h = 0
    time = 0
    x_00 = 1080  #12
    x_0=45.73 + x_00
    y_0=45.26 
    x_1=1200 +x_00
    y_1=700
    x_2= x_00 -940 
    y_2=1100
    use_BaseStation_1 = True #選擇基站

    #初始化位置
    _,_, In_Communicate_scale_toX0,index_i0,index_j0, time_now_0, time_decay_0,_ = Initial(time=time, m=m, 
                                                                                                                                n=n, x_0= x_0, y_0=y_0)

  
    FirstDrone = []
    indexi = []
    indexj = []
    indexi_total = []
    indexj_total = []
    Decay_end = []
    Drone_to_Drone_communication = []
    Total_time = []
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
        
            

##############################################
            time_id = 0
            for time_decay in Drone_to_Drone_time:
                time_index = Index_AB[time_id]
                print("+++++")
                time_id = time_id +1
                _, _, inscal, index_i,index_j, tt, decay, FirstDrone_id = Initial(time=time_now + time_decay,
                                                                                                                    m=m, n=n,x_0= x_1, y_0=y_1, time_id=time_index)
                
                Total_time.extend(tt)
                FirstDrone.append(FirstDrone_id)
                indexi.append(index_i)
                indexj.append(index_j)
                Decay_end.append(decay)
                print("tollll",Total_time)
            print("++++++++++++++")
################################################

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
                print("*****************")
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

            
    else:
        for decay_0 in time_decay_0:
            _,_, In_Communicate_scale_toX2,index_i1,index_j1, time_now_1,time_decay_1,_= Initial(time=time+decay_0,
                                                                                                                                    m=m, n=n, x_0= x_2, y_0=y_2)
            Drone_to_Drone_time , Index_AB= Drone_to_Drone(In_Communicate_scale_toX0,
                                                                                                                            In_Communicate_scale_toX2 )
            for time_decay in Drone_to_Drone_time:
                time_index = Index_AB[time_id]
                time_id = time_id + 1
                _, _, inscal, index_i,index_j, _, decay, FirstDrone_id = Initial(time=time + time_decay+decay_0, m=m, n=n, 
                                                                                                                            x_0= x_2, y_0=y_2,time_id=time_index)
                FirstDrone.append(FirstDrone_id)
                indexi.append(index_i)
                indexj.append(index_j)
                Decay_end.append(decay)

###################################################
    print("Drone_AB", Drone_AB)  #有效的AB
    print("DroneTime", DroneTime) #无人机延时
    print("conmunication_time", conmunication_time) #接受延时
    print("time_0_extend",time_0_extend) #输出延时
    print("Final_Decay_extend",Final_Decay_extend) #所有可能延时
    print("final_time",final_time) #最小延时
    print("final_id",final_id) #最小延时Drone A B配置
 
 
    Decay = min(Decay_end)
    min_time_idx = Decay_end.index(Decay)
    Decay_min = min(Decay)
    min_idx = Decay.index(Decay_min)
    # 接入 + 輸出 + Drone
    #Decay_total = Decay_min + time_decay_0[FirstDrone[min_time_idx][0]] + Drone_to_Drone_communication[min_time_idx]
    
    TimeNowmin = min(tt)
    min_time_idx_now = tt.index(TimeNowmin)
    Decay_total = TimeNowmin - time  

    
    Path_to_Fianl_BaseStation = []

    if use_BaseStation_1:
        TotalDecay = Total_Decay(t = time, BaseStation_B = 1, Decay = Decay_total)
        #path = Position_Path()

    else:
        TotalDecay = Total_Decay(t = time, BaseStation_B = 2, Decay = Decay_total)
        #path = Position_Path()

   
    print("BaseStation m_0:", index_i0)
    print("BaseStation n_0:", index_j0)
    # print("Time now:", time_now_0)
    # print("Time Decay:", time_decay_0)

    print("BaseStation m_1:", index_i1)
    print("BaseStation n_1:", index_j1)
    # print("Time now:", time_now_1)
    # print("Time Decay:", time_decay_1)

    # print("Drone to Drone AB:", Index_AB)
    print("Drone to Drone Time:", len(Drone_to_Drone_time))

    # print("BaseStation m_2:", index_i2)
    # print("BaseStation n_2:", index_j2)
    # print(In_Communicate_scale_toX2)

    # print("after_BaseStation m_1:", indexi)
    # print("after_BaseStation n_1:",indexj)

    # print("Decay;", Decay)
    # print("time_id;", min_time_idx)
    # print("end_id;", min_idx)
    #print(Decay_end)
    print("the end Drone:", (indexi[min_time_idx][min_idx], indexj[min_time_idx][min_idx])) 
    #print(FirstDrone)
    print("FirstDrone:",(index_i0[FirstDrone[min_time_idx][0]], index_j0[FirstDrone[min_time_idx][0]]))
       
    print("The total Decay:", TotalDecay)
    #print("The Path to Final Station:", path)
