from turtle import end_fill
import matplotlib.pyplot as plt
import os
import time
import numpy as np


def gauge_time(args):
    start_time = time.time()
    if type(args) is str:
        os.system(args)
    else: 
        os.system(' '.join(args))
    end_time = time.time()
    print(f'compile and run time: {(end_time - start_time):.4f}')

x0 = [45.73, 1200, -940]
y0 = [45.26, 700, 1100]

x1 = [-614, -934, 1073, 715,186,-923,833,-63]
y1 = [1059, 715, 291, 129,432,632,187,363]

def circle():
    x1 = [-614, -934, 1073, 715,186,-923,833,-63]
    y1 = [1059, 715, 291, 129,432,632,187,363]
    r = 115
    for i in range(len(x1)):
        theta = np.arange(0, 2*np.pi, 0.001)
        x = x1[i] + r * np.cos(theta)
        y = y1[i] + r + np.sin(theta)
        fig = plt.figure()
        axes = fig.add_subplot(111)
        axes.plot(x,y)
        axes.axis("equal")




def get_output_data():
    
    fname ='result.txt'
    # fname ='ljy.txt'
    with open(fname) as f:
        data = f.read().splitlines()
   
    ls=[]
    num=0
    tmp_list=[]
    m = []
    n = []
    start_id=0
    end_id=0

    for each in data[:]:
        num+=1
        
        if(num%2)==1:
            m=[]
            n=[]
            tmp_list=[]
            each=each.split(',')
            if len(each)==5:
                # print(len(each))
                id=int(each[1])
                # print(x0[id],y0[id])
                end_id=int(each[2])
                m.append(x0[id])
                n.append(y0[id])
        if(num%2)==0:
            
            each=each[1:-1] 

            d=each.split('),(')
            for tmp in d:         
                
                e=tmp.split(',')
                if len(e)==3:
                    x=float(e[0])*5+90*int(e[1])
                    y=80* int(e[2])  
                else:
                    x=x1[int(e[1])]
                    y=y1[int(e[1])]
                # print(x,y)     
                # ls[0].append(x)
                # ls[1].append(y)      
            
                m.append(x)
                n.append(y)
            m.append(x0[end_id])
            n.append(y0[end_id])
            tmp_list.append(m)
            tmp_list.append(n)
            
            ls.append(tmp_list)

        
       
    return ls
if __name__ == '__main__':
    #gauge_time('sh build_and_run.sh')

    ls=get_output_data()
    # plt.plot(ls[0],ls[1],"r")
    # plt.savefig("path.png")
    # plt.close()

    i=0
    for e in ls:
       
        i+=1
        color_tpye=i%4
        if color_tpye==0:
            plt.plot(e[0],e[1],color='r',linestyle="-")
            plt.scatter(e[0],e[1], marker = 'o', color = 'c', s = 20)
        elif color_tpye==1:
            plt.plot(e[0],e[1],"g",linestyle="-")
            plt.scatter(e[0],e[1], marker = 'o', color = 'g', s = 20)
        elif color_tpye==2:
            plt.plot(e[0],e[1],"b",linestyle="-")
            plt.scatter(e[0],e[1], marker = 'o', color = 'b', s = 20)
        elif color_tpye==3:
            plt.plot(e[0],e[1],"y",linestyle="-")
            plt.scatter(e[0],e[1], marker = 'o', color = 'y', s = 20)

        if i%4==0:
            plt.grid()
            plt.scatter(e[0][0], e[1][0], marker = 'o', color = 'red', s = 60, label = 'base_station')
            plt.scatter(e[0][-1], e[1][-1], marker = 'x', color = 'b', s = 60, label = 'base_station')
            # plt.scatter(x0, y0, marker = 'x', color = 'red', s = 40, label = 'base_station')
            plt.scatter(x1, y1, marker = 'o', color = 'green', s = 40, label = 'high_platform')
            plt.title('path{id}'.format(id=int(i/4)))
            plt.xlabel('x')
            plt.ylabel('y')
            plt.savefig('{id}.png'.format(id=int(i/4)))
            plt.close()
        # plt.grid()
        # plt.title('path{id}'.format(id=i))
        # plt.savefig('{id}.png'.format(id=i))
        # plt.close()
    print("finish!")

    