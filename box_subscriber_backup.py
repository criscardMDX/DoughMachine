#!/usr/bin/env pyton
import rospy
import roslaunch
import yaml
import serial
from yaml.loader import SafeLoader
import os
import time
import datetime
from pathlib import Path
from std_msgs.msg import Float32
from std_msgs.msg import Int32MultiArray

#these are all globa variables, used to take sensors' values
BoxTemp =0.0
DistanceCm=0.0
Humidity=0.0
Ethylene_ppm=0.0
CO2_ppm=0.0
PEl1=0.0
PEl2=0.0
PEl3=0.0
PEl4=0.0
launchpath = '/catkin_ws/src/dough_master/dough_launch/'
parampath = '/catkin_ws/src/dough_master/dough_parameters/'
launchfile='run_dough.launch'
paramfile='dough_parameters.yaml'
originalpath=''
starttime = datetime.datetime.now()
 
def callback1(data):
    # print the actual message in its raw format
    #rospy.loginfo("The Box temperature in Celsius is Value is: %s", data.data)
    global BoxTemp
    BoxTemp=data.data
def callback2(data):
    # print the actual message in its raw format
    #rospy.loginfo("The distance in cms is: %s", data.data)
    global DistanceCm
    DistanceCm=data.data
def callback3(data):
    # print the actual message in its raw format
    #rospy.loginfo("The Humidity is: %s", data.data)
    global Humidity
    Humidity=data.data
def callback4(data):
    # print the actual message in its raw format
    #rospy.loginfo("Ethylene ppm is: %s", data.data)
    global Ethylene_ppm
    Ethylene_ppm=data.data
def callback5(data):
    # print the actual message in its raw format
    #rospy.loginfo("CO2 ppm is: %s", data.data)
    global CO2_ppm
    CO2_ppm=data.data
def callback6(data):
    # print the actual message in its raw format
    #rospy.loginfo("PEl1 Temperature in Celsius is: %s", data.data)
    global PEl1
    PEl1=data.data
def callback7(data):
    # print the actual message in its raw format
    #rospy.loginfo("PEl2 Temperature in Celsius is: %s", data.data)
    global PEl2
    PEl2=data.data 
def callback8(data):
    # print the actual message in its raw format
    #rospy.loginfo("PEl3 Temperature in Celsius is: %s", data.data)
    global PEl3
    PEl3=data.data
def callback9(data):
    # print the actual message in its raw format
    #rospy.loginfo("PEl4 Temperature in Celsius is: %s", data.data)
    global PEl4
    PEl4=data.data    
       
def main():
    VoltPolarity=[0,1,0] #Initialise the message array
    if not rospy.is_shutdown(): os.system("killall -9 rosmaster") #Clean way to kill roscore if it exists
    # Identify the location of the launch file from the current working directory
    homepath=str(Path.home())
    originalpath=homepath+launchpath+launchfile
    launchfilelocation=str(originalpath)
    # Open the YAML file containing the dough raising parameters and load it
    originalparampath=homepath+parampath+paramfile
    paramfilelocation=str(originalparampath)
    with open(paramfilelocation) as f:
        paramdata = yaml.load(f, Loader=SafeLoader)
        print(paramdata) #this is used to test that I am loading the YAML File
        #print(paramdata["Process"][0]["name"]) #Here I take the name of the process 
        #print(paramdata["Process"][0]["nocycles"]) #Here I take the maximum amount of cycles for the "for" cycle to regulate temperature.
    ProcessName=paramdata["Process"][0]["name"]
    MaxNoOfCycles=paramdata["Process"][0]["nocycles"]    

    # This code comes from https://answers.ros.org/question/215600/how-can-i-run-roscore-from-python/
    # I want to run roscore straight from the Python script and shut it down once the machine finishes her cycle
    # This script also offer the chance to include the launch file. The only issue I need to solve is in case the port changes.
    #during debug, from the terminal window kill roscore master with[killall -9 rosmaster].
    #MEMO!! For now I kill the roscore launch, but I may want to look at a cleaner killing process, including subprocesses.
    uuid = roslaunch.rlutil.get_or_generate_uuid(options_runid=None, options_wait_for_master=False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_files=[launchfilelocation], is_core=True)
    launch.start()
    
    #Initialise the node that will communicate voltage and polarity back to Arduino    
    pubvoltagepolarity = rospy.Publisher('/voltageAndPolarityInput', Int32MultiArray, queue_size=10)
    data_to_send = Int32MultiArray()
    data_to_send.data = VoltPolarity
  
    # initialize a node by the name 'Dough_Machine_Input_Manager'.
    # instead of spin, that has its own cycle time, I would rather keep this into a state of constant monitoring
    while not rospy.is_shutdown():
        rospy.init_node('Dough_Machine_Input_Manager', anonymous=True)
        pubvoltagepolarity.publish(data_to_send) 
        rospy.Subscriber("/box_temp_Celsius", Float32, callback1)
        rospy.Subscriber("/Distance_Cms", Float32, callback2)  
        rospy.Subscriber("/Humidity", Float32, callback3)
        rospy.Subscriber("/Ethylene_ppm", Float32, callback4)
        rospy.Subscriber("/CO2_ppm", Float32, callback5)
        rospy.Subscriber("/PEl_1_single", Float32, callback6)
        rospy.Subscriber("/PEls_controller", Float32, callback7)
        rospy.Subscriber("/PEl_2_couple", Float32, callback8)
        rospy.Subscriber("/PEl_3_couple", Float32, callback9)
        MeasurementArray=[BoxTemp,DistanceCm,Humidity,Ethylene_ppm,CO2_ppm,PEl1,PEl2,PEl3,PEl4]
        print (MeasurementArray)
        rospy.sleep(0.2)
        
        while BoxTemp!=0:
            # spin() simply keeps python from exiting until this node is stopped, but it creates issues with timing.
            # I would rather collect every ping from the dough machine.
            #rospy.spin()
            
            # Algorythm to control the temperature in the box. The additional information is in the notes to this experiment (Notes: 10th December)
            timenow=datetime.datetime.now()
            timediff=timenow-starttime
            minsdiff=round((timediff.total_seconds()/60),0)
            for cycleindicator in range(1,MaxNoOfCycles+1):
                minstart=paramdata["Process"][0]["routines"][cycleindicator-1]["minstart"]
                minend=paramdata["Process"][0]["routines"][cycleindicator-1]["minend"]
                if minend >= minsdiff >= minstart :
                    cycleNo=paramdata["Process"][0]["routines"][cycleindicator-1]["cycleNo"]
                    changedegree=paramdata["Process"][0]["routines"][cycleindicator-1]["changedegree"]
                    changeintervalminutes=paramdata["Process"][0]["routines"][cycleindicator-1]["changeintervalminutes"]
                    targettemperature=paramdata["Process"][0]["routines"][cycleindicator-1]["temperature"]
                    if (changedegree==0 and changeintervalminutes==0):
                        while BoxTemp!=targettemperature:
                            rospy.Subscriber("/box_temp_Celsius", Float32, callback1)
                            gradientvstarget=targettemperature-BoxTemp
                            if (gradientvstarget<0): 
                                polarity=0 #In the motor controller meaning, "L" means backward, hence inverse polarity. By convention R=1, L=0, so I can transfer bot Integer values through my publisher.
                                voltage=int((1-(targettemperature/BoxTemp))*255) #value goes from 0 to 255, but I do not want to boost the Peltier to the max if the gradient is small.
                            elif(gradientvstarget>0):
                                polarity=1 #In the motor controller meaning, "R" means forward, hence DC as per original polarity. By convention R=1, L=0, so I can transfer bot Integer values through my publisher.
                                voltage=int((1-(BoxTemp/targettemperature))*255) #value goes from 0 to 255, but I do not want to boost the Peltier to the max if the gradient is small.
                            else:
                                polarity=1 #By convention R=1, L=0, so I can transfer bot Integer values through my publisher.
                                voltage=0
                            if voltage<150: 
                                    voltage=150
                            else:   voltage=250
                            #send both voltage and polarity back to Arduino
                            VoltPolarity=[voltage, polarity,cycleNo]
                            data_to_send.data = VoltPolarity
                            #rospy.Publisher("/voltageAndPolarityCommand",Int32MultiArray, callback10)
                            pubvoltagepolarity.publish(data_to_send) 
                            #rospy.loginfo(VoltPolarity)
                            #pub.publish(VoltPolarity)
                            #rate.sleep()
                            time.sleep(0.5)
                            #read again the temperature after 30 seconds
                            rospy.Subscriber("/box_temp_Celsius", Float32, callback1)                        
                    elif (changedegree!=0 and changeintervalminutes!=0):    #here the temperature in the Yaml file is the final temperature from the previous stage. 
                        starttemperature=targettemperature
                        timenowIFgradient=datetime.datetime.now()             #I wanted to make sure not to take the Box temperature as starting value to calculate the gradient, as this may generate errors.
                        while timenowIFgradient<=minend:
                            timenowIFgradient=datetime.datetime.now()  
                            timestack=timenowIFgradient+changeintervalminutes
                            temperaturestack=starttemperature+changedegree
                            timestackNo+=1
                            while timenowIFgradient<=timestack:
                                while BoxTemp != temperaturestack:
                                    voltage=200
                                    if BoxTemp<temperaturestack: polarity="1" #this is the case of a positive positive for a certain an amount of time change interval minutes        
                                    elif BoxTemp>temperaturestack: polarity="0" #this is the case of a negative difference for a certain an amount of time change interval minutes
                                    else: 
                                        voltage=0
                                        polarity=1
                                #send both voltage and polarity back to Arduino
                                VoltPolarity=[voltage, polarity,cycleNo]
                                data_to_send.data = VoltPolarity
                                pubvoltagepolarity.publish(data_to_send) 
                                time.sleep(0.5)
                                #read again the temperature after 30 seconds
                                rospy.Subscriber("/box_temp_Celsius", Float32, callback1)         
                                timenowIFgradient=datetime.datetime.now()
  
    #After the machine has finished its cycle, it should kill roscore and its sub processes
    launch.shutdown()
  
  
if __name__ == '__main__':
    
    # you could name this function
    try:
        main()
    except rospy.ROSInterruptException:
        pass
