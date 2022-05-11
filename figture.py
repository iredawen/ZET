import pandas as pd
import matplotlib.pyplot as plt
import time
import numpy as np

def main():
    path = "./location.csv"
    ydata = []
    xdata = []
    zdata = []
    data = pd.read_csv(path, encoding='gbk')
    ydata = data.loc[:,'y']
    xdata = data.loc[:,'x']

    plt.figure(1)
    plt.scatter(xdata, ydata, s=5,c='r',marker='o')
    plt.title(u'location', size = 10)
    plt.legend()
    plt.xlabel(u'x', size = 10)
    plt.ylabel(u'y', size = 10)
    plt.savefig('./location.jpg')
    plt.show()
    print("ok")

#     #############3D
#     import pandas as pd
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
# import numpy as np

# def main():
#     path = "./location.csv"
#     ydata = []
#     xdata = []
#     zdata = []
#     data = pd.read_csv(path, encoding='gbk')
#     ydata = data.loc[:,'y']
#     xdata = data.loc[:,'x']
#     zdata = data.loc[:,'z']

#     fig = plt.figure()
#     ax = fig.add_subplot(111, projection = '3d')
#     ax.scatter(xdata, ydata,zdata, s=5,c='r',marker='o')
#     plt.title(u'location', size = 10)
#     plt.legend()
#     ax.set_xlabel(u'x', size = 10)
#     ax.set_ylabel(u'y', size = 10)
#     ax.set_zlabel(u'z', size = 10)
#     plt.savefig('./location.jpg')
#     plt.show()
#     print("ok")

if __name__ == "__main__":
    main()