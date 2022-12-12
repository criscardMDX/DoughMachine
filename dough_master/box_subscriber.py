#!/usr/bin/env pyton
import rospy
import roslaunch
import yaml
from yaml.loader import SafeLoader
#import os
import time
import datetime
from pathlib import Path
from std_msgs.msg import Float32


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

    #This was used to debug correct file location syntaxis 
    #for f in os.listdir(Path(originalpath)):
    #    print(f)

    # This code comes from https://answers.ros.org/question/215600/how-can-i-run-roscore-from-python/
    # I want to run roscore straight from the Python script and shut it down once the machine finishes her cycle
    # This script also offer the chance to include the launch file. The only issue I need to solve is in case the port changes.
    #during debug, from the terminal window kill roscore master with[killall -9 rosmaster].
    #MEMO!! For now I kill the roscore launch, but I may want to look at a cleaner killing process, including subprocesses.
    uuid = roslaunch.rlutil.get_or_generate_uuid(options_runid=None, options_wait_for_master=False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_files=[launchfilelocation], is_core=True)
    launch.start()
  
    # initialize a node by the name 'Dough_Machine_Input_Listener'.
    # instead of spin, that has its own cycle time, I would rather keep this into a state of constant monitoring
    while not rospy.is_shutdown():
        rospy.init_node('Dough_Machine_Input_Listener', anonymous=True)  
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
        rospy.sleep(1)    
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
                        gradientvstarget=targettemperature-BoxTemp
                        if (gradientvstarget<0): 
                            polarity="L" #In the motor controller meaning, "L" means backward, hence inverse polarity.
                            voltage=int((1-(targettemperature/BoxTemp))*255) #value goes from 0 to 255, but I do not want to boost the Peltier to the max if the gradient is small.
                        elif(gradientvstarget>0):
                            polarity="R" #In the motor controller meaning, "R" means forward, hence DC as per original polarity.
                            voltage=int((1-(BoxTemp/targettemperature))*255) #value goes from 0 to 255, but I do not want to boost the Peltier to the max if the gradient is small.
                        else:
                            polarity="R"
                            voltage=0 
                elif (changedegree!=0 and changeintervalminutes!=0):
                    timenowgradient=datetime.datetime.now()
                    targettemperature=BoxTemp+changedegree
                    while timenowgradient<timenowgradient+changeintervalminutes:
                        while BoxTemp<targettemperature:
                            if changedegree<0:
                                polarity="L"
                                voltage=int(abs(changedegree)/BoxTemp*255)
                        
                    
            
            

    
    
    

    #After the machine has finished its cycle, it should kill roscore and its sub processes
    launch.shutdown()
  
  
if __name__ == '__main__':
    
    # you could name this function
    try:
        main()
    except rospy.ROSInterruptException:
        pass
