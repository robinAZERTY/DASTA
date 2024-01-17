# to use an extended Kalman filter, we can use

import myKalmanFilter as kf
import numpy as np

'''
____________STATE ESTIMATE VECTOR______________

        State                   unit        index
postion(xyz)                    m           1:3
velocity(xyz)                   m/s         4:6
orientation(quaternion)                     7:10



___________COMMANDS VECTOR_____________________

        State                   unit        index
gyroscopes(xyz)                 rad/s       1:3
accelerometers(xyz)             m/s²        4:6


___________SENSORS VECTOR_____________________

        State                   unit       index
beac1_cam1(uv)                  pixel       1:2
beac2_cam1(uv)                  pixel       3:4
beac1_cam2(uv)                  pixel       5:6
beac2_cam2(uv)                  pixel       7:8

___________PARAMS VECTOR_____________________

        State                   unit       index
cam1_pos(xyz)                   m           1:3
cam1_ori(quaternion)                        4:7
cam1_k                          pixel        8
cam2_pos(xyz)                   m           9:11
cam2_ori(quaternion)                        12:15
cam2_k                          pixel        16
dt                              s            17
g                               m/s²         18
led1_pos(xyz)                   m           19:21
led2_pos(xyz)                   m           22:24
'''