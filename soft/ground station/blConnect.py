# import bluetooth #pip install git+https://github.com/pybluez/pybluez.git
# import time
# import sys
# import os
# import numpy as np
# import matplotlib.pyplot as plt



# connect_to = "C0:49:EF:CD:A7:D6"
# #the connection don't use a secure protocol, so the password is not used

# print("Connecting to " + connect_to)

# sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
# #Create a bluetooth socket
# connect = False

# while  not connect:
#     try:
#         sock.connect((connect_to, 1))
#         connect = True
#     except:
#         print("Can't connect to " + connect_to)
        


# print("Connected to " + connect_to)

# def test_ping():
#     #test the ping between the computer and the raspberry pi
#     #send a message and measure the time it took to get the answer
#     start = time.time()
#     sock.send("pyucfuyfvuyfuyfouyfyufufuyfuyfyu")
#     data = sock.recv(1024)
#     end = time.time()
#     return end - start
    
# # Average ping time: 0.026480768275260927s
# # Standard deviation: 0.026622161634210904s
# # Maximum ping time: 0.2710251808166504s
# # Minimum ping time: 0.0s

# # Average ping time: 0.010245248079299926s
# # Standard deviation: 0.01644696737827921s
# # Maximum ping time: 0.21167731285095215s
# # Minimum ping time: 0.0s

# # test_length = 10000
# # test_time = np.zeros(test_length)
# # for i in range(test_length):
# #     print("Test " + str(i+1) + "/" + str(test_length))
# #     test_time[i] = test_ping()
    
    
# # print("Average ping time: " + str(np.mean(test_time)) + "s")
# # print("Standard deviation: " + str(np.std(test_time)) + "s")
# # print("Maximum ping time: " + str(np.max(test_time)) + "s")
# # print("Minimum ping time: " + str(np.min(test_time)) + "s")

# # #plot as a histogram
# # plt.hist(test_time, bins=50,density=True)
# # plt.title("Ping time")
# # plt.xlabel("Time (s)")
# # plt.ylabel("Number of tests")
# # plt.show()
    

# #show the data received 
# END_LINE = "end_line\n"

# DESCRIPTION_KEY="description:"


# while True:
#     #read as string until the end of the line
#     data = ""
#     while not data.endswith(END_LINE):
#         data += sock.recv(1024).decode("utf-8")
        
#     if data.startswith(DESCRIPTION_KEY):
#         print(data[len(DESCRIPTION_KEY):])


#pair the device if not already paired

import socket

s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
s.connect(('C0:49:EF:CD:A7:D6', 1))
print("Connected")