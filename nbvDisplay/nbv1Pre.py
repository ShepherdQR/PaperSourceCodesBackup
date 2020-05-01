'''
@Author: Shepherd Qirong
@Date: 2020-04-27 12:04:17
@Github: https://github.com/ShepherdQR
@LastEditors: Shepherd Qirong
@LastEditTime: 2020-04-27 13:28:23
@Copyright (c) 2019--20xx Shepherd Qirong. All rights reserved.
'''

import numpy as np
import time
timeCur = time.strftime("%Y%m%d%H%M%S", time.localtime())
time_4=int(timeCur[-4:])


nameOut = "planViews" + timeCur + ".txt"


       
with open('plan.dat') as nbvdata:
    print(nbvdata.name)
    fileList = nbvdata.readlines()
    nRows = len(fileList)
    print( nRows )

nRowsOut = int((nRows - 4)/8)##here is [0,13]
for i in range(nRowsOut+1):
    outRow = fileList[4+i*8-1]
    with open(nameOut, 'a') as f:
        f.write(outRow)


# for i in range(nRows):
#     curRow = fileList[i]
#     curRow = curRow.strip('\n').split(' ')
#     if(i==107):
#         print(curRow)


matrix = np.loadtxt(nameOut, usecols=range(6))
print(matrix)






# import os
# import numpy as np
# from functools import reduce

# def str2float(s):
#     def fn(x,y):
#         return x*10+y

#     if(s.find('.')== -1):
#         s1=list(map(int,[x for x in s]))
#         return reduce(fn,s1)
#     else:
#         n=s.index('.')
#         s1=list(map(int,[x for x in s[:n]]))
#         s2=list(map(int,[x for x in s[n+1:]]))
#         return reduce(fn,s1)+reduce(fn,s2)/(10**len(s2))#乘幂
        
# with open('plan.dat') as nbvdata:
#     print(nbvdata.name)
#     fileList = nbvdata.readlines()
#     nRows = len(fileList)
#     print( nRows )

# oriData = np.zeros((nRows, 666), dtype = float)

# for i in range(nRows):
#     curRow = fileList[i]
#     curRow = curRow.strip('\n').split(' ')
#     if(i==107):
#         print(curRow)
#     for j in range(len(curRow)):
#         oriData[i][j] = str2float(curRow[j])

# print(oriData)