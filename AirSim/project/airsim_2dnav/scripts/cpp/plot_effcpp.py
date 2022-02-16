#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray, Int8
from matplotlib import pyplot as plt

def getTArr(arr):
    t_array = arr.data;

    #plt.figure(figsize=(8, 10))
    fig, ax = plt.subplots(figsize=(8,10))

    plt.subplot(5,1,1)
    plt.plot(t_array,v_array,'-r')
    plt.title("Velocity vs Time")

    plt.subplot(5,1,2)
    plt.plot(t_array,a_array,'-r')
    plt.title("Acceleration vs Time")

    plt.subplot(5,1,3)
    plt.plot(t_array,e_array,'-r')
    plt.title("Energy vs Time")

    plt.subplot(5,1,4)
    plt.plot(t_array,d_array,'-r')
    plt.title("Drag Force vs Time")

    plt.subplot(5,1,5)
    plt.plot(t_array,ma_array,'-r')
    plt.title("Inertial Force vs Time")

    plt.tight_layout()

    for p in fig.axes:
        pos = p.get_position()
        i = 0

        for t in goaltime:
            p.axvline(t,ls ='--',color = 'g')
            p.text(t,pos.x1,i)
            i += 1
    plt.show()

def getVArr(arr):
    global v_array
    v_array = arr.data;

def getAArr(arr):
    global a_array
    a_array = arr.data;

def getDArr(arr):
    global d_array
    d_array = arr.data;

def getEArr(arr):
    global e_array
    e_array = arr.data;

def getMaArr(arr):
    global ma_array
    ma_array = arr.data;

def getGT(arr):
    global goaltime
    goaltime = arr.data;

def listener():
    rospy.init_node('efficiency_calc')
    rospy.Subscriber("/velocity", Float64MultiArray, getVArr)
    rospy.Subscriber("/acceleration", Float64MultiArray, getAArr)
    rospy.Subscriber("/energy", Float64MultiArray, getEArr)
    rospy.Subscriber("/dragF", Float64MultiArray, getDArr)
    rospy.Subscriber("/inertialF", Float64MultiArray, getMaArr)
    rospy.Subscriber("/goalTime", Float64MultiArray, getGT)
    rospy.Subscriber("/time", Float64MultiArray, getTArr)
    rospy.spin()


if __name__ == '__main__':
    v_array = []
    a_array = []
    e_array = []
    d_array = []
    ma_array = []
    t_array = []
    goaltime = []
    passed = 0
    listener()
