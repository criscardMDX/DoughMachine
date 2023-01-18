#!/usr/bin/env pyton

##Initialise libraries##
import rospy
import roslaunch
import yaml
import serial
from yaml.loader import SafeLoader
import os
import time
import datetime
from datetime import datetime
from datetime import timedelta
import logging
from pathlib import Path
from std_msgs.msg import Float32
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String

#Initialise GLOBAL VARIABLES, used to take sensors' values and to transfer data between main and sub functions
global BoxTemp
global DistanceCm
global Humidity
global Ethylene_ppm
global CO2_ppm
global PEl1Heat
global PEl3Chill
global PEl4Chill
global LM35Sensor
global DHTSensor
global Sleeprate
global AveragingTempCycles
global voltage
global polarity
global cycleNo
global coolingonoff
global pubvoltagepolarity
global ONOFFchilldata_to_send
global data_to_send
global startstopchillingunit
Sleeprate=0.2
AveragingTempCycles=10
polarity=1
voltage=35
maxtempallowed=42
safeinterrupttime=15
coolingonoff=0
BoxTemp =0.0
DistanceCm=0.0
Humidity=0.0
Ethylene_ppm=0.0
CO2_ppm=0.0
PEl3Chill=0.0
PEl4Chill=0.0
LM35Sensor=0.0
DHTSensor=0.0
PEl1Heat=0.0
launchpath = '/catkin_ws/src/dough_master/dough_launch/'
parampath = '/catkin_ws/src/dough_master/dough_parameters/'
logpath='/catkin_ws/src/dough_master/dough_logfiles/'                          # The correct address should be: '/catkin_ws/src/dough_master/dough_logfiles'#
launchfile='run_dough.launch'
paramfile='dough_parameters.yaml'
originalpath=''
starttime = datetime.now()
logfileName="LogFile"
logdata="no message"
homePath = str(Path.home())
PolVoltOutput=[]
MeasurementArray=[]

##0001 This Function will save into the log all values sent to the dough machine
def createlogfile(logdata):
    #entries = os.listdir(homePath)
    #for entry in entries:
    #    print(entry)
    with open(homePath+"/"+logpath+logfileName+str(starttime)+".txt", 'a') as LogFile:
        LogFile.write(logdata[0]+"\n")

##0002 CallBack This function takes the third set of values from the Box read 
##      I have created a separate callback for each value read from the dough machine
def callback1(data):
    # print the actual message in its raw format
    #rospy.loginfo("The Box temperature in Celsius is Value is: %s", data.data)
    global BoxTemp
    BoxTemp=round(data.data,2)
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
    #rospy.loginfo("PEl3Chill Temperature in Celsius is: %s", data.data)
    global PEl3Chill
    PEl3Chill=data.data
def callback7(data):
    # print the actual message in its raw format
    #rospy.loginfo("PEl4Chill Temperature in Celsius is: %s", data.data)
    global PEl4Chill
    PEl4Chill=data.data 
def callback8(data):
    # print the actual message in its raw format
    #rospy.loginfo("LM35Sensor Temperature in Celsius is: %s", data.data)
    global LM35Sensor
    LM35Sensor=data.data
def callback9(data):
    # print the actual message in its raw format
    #rospy.loginfo("DHTSensor Temperature in Celsius is: %s", data.data)
    global DHTSensor
    DHTSensor=data.data
def callback10(data):
    # print the actual message in its raw format
    #rospy.loginfo("PEl1Heat Temperature in Celsius is: %s", data.data)
    global PEl1Heat
    PEl1Heat=data.data

##0003 subscriber and publisher built into one function, so that I do not have to write
##       the same code multiple times.  
def spinNode():
        rospy.Subscriber("/box_temp_Celsius", Float32, callback1)
        rospy.Subscriber("/Distance_Cms", Float32, callback2)  
        rospy.Subscriber("/Humidity", Float32, callback3)
        rospy.Subscriber("/Ethylene_ppm", Float32, callback4)
        rospy.Subscriber("/CO2_ppm", Float32, callback5)
        rospy.Subscriber("/PEl_3_chill", Float32, callback6)
        rospy.Subscriber("/PEl_4_chill", Float32, callback7)
        rospy.Subscriber("/LM35_sensor", Float32, callback8)
        rospy.Subscriber("/DHT_sensor", Float32, callback9)
        rospy.Subscriber("/PEl_1_heat", Float32, callback10)
        rospy.sleep(Sleeprate)
        rospy.spin

##0004 Sensors are volatile, I want to take an average of 10 
##      readings before I can tell the dough machine what temperature to adopt
def avgBoxSensorReadings():
    global AveragingTempCycles
    global BoxTemp
    global DistanceCm
    global Humidity
    global Ethylene_ppm
    global CO2_ppm
    global PEl1Heat
    global PEl3Chill
    global PEl4Chill
    global LM35Sensor
    global DHTSensor
    avgindex=0
    BoxTempAvgArray=[]
    BoxCM2AvgArray=[]
    BoxAvgHumidityArray=[]
    BoxAvgEthyleneArray=[]
    BoxAvgCO2Array=[]
    BoxAvgPEl3ChillArray=[]
    BoxAvgPELContrArray=[]
    BoxAvgLM35SensorArray=[]
    BoxAvgDHTSensorArray=[]
    BoxAvgPEl1HeatArray=[]
    while avgindex<AveragingTempCycles:
        rospy.Subscriber("/box_temp_Celsius", Float32, callback1)
        rospy.Subscriber("/Distance_Cms", Float32, callback2)  
        rospy.Subscriber("/Humidity", Float32, callback3)
        rospy.Subscriber("/Ethylene_ppm", Float32, callback4)
        rospy.Subscriber("/CO2_ppm", Float32, callback5)
        rospy.Subscriber("/PEl_3_chill", Float32, callback6)
        rospy.Subscriber("/PEl_4_chill", Float32, callback7)
        rospy.Subscriber("/LM35_sensor", Float32, callback8)
        rospy.Subscriber("/DHT_sensor", Float32, callback9)
        rospy.Subscriber("/PEl_1_heat", Float32, callback10)
        rospy.sleep(Sleeprate)
        rospy.spin
        BoxTempAvgArray.insert(avgindex,BoxTemp)
        BoxCM2AvgArray.insert(avgindex,DistanceCm)
        BoxAvgHumidityArray.insert(avgindex,Humidity)
        BoxAvgEthyleneArray.insert(avgindex,Ethylene_ppm)
        BoxAvgCO2Array.insert(avgindex,CO2_ppm)
        BoxAvgPEl3ChillArray.insert(avgindex,PEl3Chill)
        BoxAvgPELContrArray.insert(avgindex,PEl4Chill)
        BoxAvgLM35SensorArray.insert(avgindex,LM35Sensor)
        BoxAvgDHTSensorArray.insert(avgindex,DHTSensor)
        BoxAvgPEl1HeatArray.insert(avgindex,PEl1Heat)
        avgindex =avgindex+1
    BoxTemp=round(sum(BoxTempAvgArray) / len(BoxTempAvgArray),2)
    DistanceCm=round(sum(BoxCM2AvgArray) / len(BoxCM2AvgArray),2) 
    Humidity=round(sum(BoxAvgHumidityArray) / len(BoxAvgHumidityArray),2)
    Ethylene_ppm=round(sum(BoxAvgEthyleneArray) / len(BoxAvgEthyleneArray),2)
    CO2_ppm= round(sum(BoxAvgCO2Array) / len(BoxAvgCO2Array),2)
    PEl3Chill=round(sum(BoxAvgPEl3ChillArray) / len(BoxAvgPEl3ChillArray),2)
    PEl4Chill=round(sum(BoxAvgPELContrArray) / len(BoxAvgPELContrArray),2)
    LM35Sensor=round(sum(BoxAvgLM35SensorArray) / len(BoxAvgLM35SensorArray),2)
    DHTSensor=round(sum(BoxAvgDHTSensorArray) / len(BoxAvgDHTSensorArray),2)
    PEl1Heat=round(sum(BoxAvgPEl1HeatArray) / len(BoxAvgPEl1HeatArray),2)
    
##0005 Kill all ROS applications if they exist
def KillAllROSApps():
    if not rospy.is_shutdown(): 
        os.system("killall -9 rosmaster") #Clean way to kill roscore if it exists
        rospy.sleep(2)

##0006 Start a new instance of ROS; Code source: https://answers.ros.org/question/215600/how-can-i-run-roscore-from-python/
    # I want to run roscore straight from the Python script and shut it down once the machine finishes her cycle
    # This script also offer the chance to include the launch file. The only issue I need to solve is in case the port changes.
    #during debug, from the terminal window kill roscore master with[killall -9 rosmaster].
def StartNewROSInstance(homepath,launchpath,launchfile):
    originalpath=homepath+launchpath+launchfile
    launchfilelocation=str(originalpath)
    uuid = roslaunch.rlutil.get_or_generate_uuid(options_runid=None, options_wait_for_master=False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_files=[launchfilelocation], is_core=True)
    launch.start()

##0007 This function is deciding if the motor controller will have to cool or heat the dough master box.
def DecidePolarityVoltageToAdopt(gradientvstarget):
    global voltage
    global polarity
    global coolingonoff
    ##gradientvstarget=targettemperature-BoxTemp, hence if gradient<0, means I need to chill
    if (gradientvstarget<-0.5):
        # I will activate the chilling unit and Put the motor control in state L=heating, but at voltage of 35, only to maintain the heat shield.
        coolingonoff=1
        polarity=1      
        voltage=35
    elif(gradientvstarget>0.5):
        ##gradientvstarget=targettemperature-BoxTemp, hence if gradient>0, means I need to heat
        # I will activate the motor control and push the heat at 70% of the maximum Wattage: 180/255*12V=8.5 Volts, at 5 Amps = 42.5Watts out of Max 60
        # I also switch off hte cooling unit. Here I have no issue with the thermal gradient, as I am heating the box.
        coolingonoff=0
        polarity=0     
        voltage=255      
    else:
        coolingonoff=0
        polarity=0      
        voltage=35      #Thermal shield

##0008 Fubnction for safety, if the box reaches the max temperature allowed in the settings (42 degrees Celsius), then it should switch off
##      then wait at threshold voltage for an amount of seconds (15 seconds), then start a cooling process to bring the machine to a safe temperature of (36 degrees Celsius).
##  This function must also have an interrupt
def SecurityCoolOff(cycleNo,BoxTemp,maxtempallowed,safeinterrupttime):
    global voltage
    global polarity
    global coolingonoff
    while BoxTemp>maxtempallowed-3:
        SecCoolOffProcessTimeInttStart=datetime.now()
        SecCoolOffProcessTimeInt=datetime.now()
        SecCoolOffProcessTimeIntTarget=SecCoolOffProcessTimeInttStart+timedelta(seconds = safeinterrupttime)
        voltage=35
        VoltPolarity=[voltage,polarity,cycleNo]
        data_to_send.data = VoltPolarity
        pubvoltagepolarity.publish(data_to_send)
        #Switch on cooling unit, full blast
        coolingonoff=1
        chilOnOff=[coolingonoff]
        ONOFFchilldata_to_send.data = chilOnOff
        startstopchillingunit.publish(ONOFFchilldata_to_send)  
        print("Heat data sent back"+ str(data_to_send.data))
        print("Chill data sent back"+ str(ONOFFchilldata_to_send.data))
        while SecCoolOffProcessTimeInt<SecCoolOffProcessTimeIntTarget:
            print('Machine cooling down')
            SecCoolOffProcessTimeInt=datetime.now()
        #while(BoxTemp<safetemp):
        #    polarity=0          # For the motor controller, "R"=1=hot=forward and L=0=cold=backward, as the peltier was installed with the hot side up.
        #                        # In the arduino R=1 (forward), L=0 (backward/reverse polarity), so I can transfer both Integer values through my publisher and Arduino will assign "R" and "L".
        #    voltage=200
        #    VoltPolarity=[voltage,polarity,cycleNo]
        #    data_to_send.data = VoltPolarity
        #    pubvoltagepolarity.publish(data_to_send)
        #    print("data sent back from Temperature Security Function"+ str(data_to_send))

def main():
    global voltage
    global polarity
    global cycleNo
    global VoltPolarity
    global chilOnOff
    global coolingonoff
    print(os.path.expanduser('~'))
    VoltPolarity=[35,1,0] #Initialise the message array
    chilOnOff=[0]
    SampleNo=0
    StatusFUp_msg=[0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00]
    StatusFollowUp='no message'
    PeltierInputstabilisetime=20
    homepath=str(Path.home())
    KillAllROSApps()                #0005   Kill ROS Apps if they are already running
    StartNewROSInstance(homepath,launchpath,launchfile) #0006   Start a new instance of ROS using the launch file.
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
    #Initialise the node that will communicate voltage and polarity back to Arduino    
    pubvoltagepolarity = rospy.Publisher('/voltageAndPolarityInput', Int32MultiArray, queue_size=10)
    data_to_send = Int32MultiArray()
    data_to_send.data = VoltPolarity
    #Initialise the Node that will communicate the status of the process, to be displayed on the Arduino's LCD
    pubprocessstatus=rospy.Publisher('/ProcessStatus', Float32MultiArray, queue_size=10)
    SFUp_msg = Float32MultiArray()
    SFUp_msg.data = StatusFUp_msg
    #Initialise the Node that will start or stop the chilling unit in Arduino
    startstopchillingunit=rospy.Publisher('/CoolingOn', Int32MultiArray, queue_size=10)
    ONOFFchilldata_to_send = Int32MultiArray()
    ONOFFchilldata_to_send.data = chilOnOff    
    # initialize a node by the name 'Dough_Machine_Input_Manager'.
    # instead of spin, that has its own cycle time, I would rather keep this into a state of constant monitoring
    while not rospy.is_shutdown():
        rospy.init_node('Dough_Machine_Input_Manager', anonymous=True)
        pubvoltagepolarity.publish(data_to_send)
        pubprocessstatus.publish(SFUp_msg)
        startstopchillingunit.publish(ONOFFchilldata_to_send)
        #Subscriber subfunction callout
        #spinNode()
        avgBoxSensorReadings()
        SampleNo+=1
        MeasurementArray=[BoxTemp,DistanceCm,Humidity,Ethylene_ppm,CO2_ppm,PEl1Heat,PEl3Chill,PEl4Chill,LM35Sensor,DHTSensor,SampleNo]
        print (MeasurementArray)
        #while (SampleNo>2 and BoxTemp==0): Sleeprate=0
        rospy.sleep(Sleeprate)
        while BoxTemp==0:
            avgBoxSensorReadings()
            MeasurementArray=[BoxTemp,DistanceCm,Humidity,Ethylene_ppm,CO2_ppm,PEl1Heat,PEl3Chill,PEl4Chill,LM35Sensor,DHTSensor,SampleNo,datetime.now()]
            print (MeasurementArray)
            print ("Connecting")
            SampleNo+=1
        # Algorythm to control the temperature in the box. The additional information is in the notes to this experiment (Notes: 10th December)
        timenow=datetime.now()
        timediff=timenow-starttime
        minsdiff=round((timediff.total_seconds()/60),0)
        for cycleindicator in range(1,MaxNoOfCycles+1):
            minstart=paramdata["Process"][0]["routines"][cycleindicator-1]["minstart"]
            minend=paramdata["Process"][0]["routines"][cycleindicator-1]["minend"]
            if minend >= minsdiff >= minstart :
                cycleNo=paramdata["Process"][0] ["routines"][cycleindicator-1]["cycleNo"]
                changedegree=paramdata["Process"][0]["routines"][cycleindicator-1]["changedegree"]
                changeintervalminutes=paramdata["Process"][0]["routines"][cycleindicator-1]["changeintervalminutes"]
                targettemperature=paramdata["Process"][0]["routines"][cycleindicator-1]["temperature"]
                # routine activated when the Yaml targets a specific temperature and not a gradient over time. E.g. "set the machine at 30 degree Celsius"
                if (changedegree==0 and changeintervalminutes==0):
                        #The routine will be activated until the time specified in the Yaml file.
                        while minsdiff<minend:
                            #The routine will be activated until the target temperature is reached
                            while BoxTemp!=targettemperature:
                                avgBoxSensorReadings()
                                #I set a safety threshold as I do not want the temperature to ever surpass a certain value, 
                                # as otherwise it will kill the yeast. MaxTemp is set at 43 degree Celsius. 
                                gradientvstarget=targettemperature-BoxTemp
                                DecidePolarityVoltageToAdopt(gradientvstarget)
                                VoltPolarity=[voltage,polarity,cycleNo]
                                chilOnOff=[coolingonoff]
                                data_to_send.data = VoltPolarity
                                pubvoltagepolarity.publish(data_to_send) ##Send data to the dough machine
                                ONOFFchilldata_to_send.data = chilOnOff
                                startstopchillingunit.publish(ONOFFchilldata_to_send)                               
                                print("Heat data sent back"+ str(data_to_send.data)) #This is a matrix, not an array!!
                                print("Chill data sent back"+ str(ONOFFchilldata_to_send.data)) #This is a matrix, not an array!!
                                MeasurementArray=[BoxTemp,DistanceCm,Humidity,Ethylene_ppm,CO2_ppm,PEl1Heat, PEl3Chill,PEl4Chill,LM35Sensor,DHTSensor,SampleNo]                       
                                timenow=datetime.now()
                                timediff=timenow-starttime
                                minsdiff=round((timediff.total_seconds()/60),0)
                                #Publish status on Computer and stored log file
                                StatusFollowUp="TargetT: "+str(targettemperature)+" ; AvgBoxT: "+str(BoxTemp)+" ; Time: "+str(timenow)+" ; Volts: "+ str(voltage)+" ; Polarity: "+str(polarity)+" ; Cooling: "+str(coolingonoff)+" ; CycleNo: "+str(cycleNo)+" ; MinsDiff: "+str(minsdiff)+" ; MinEnd: "+str(minend)
                                print(StatusFollowUp)
                                MeasurementArray=[BoxTemp,DistanceCm,Humidity,Ethylene_ppm,CO2_ppm,PEl1Heat,PEl3Chill,PEl4Chill,LM35Sensor,DHTSensor,SampleNo,datetime.now()]
                                print (MeasurementArray)
                                createlogfile(StatusFollowUp)  
                                #Publish status on ROS
                                StatusFUp_msg=StatusFUp_msg=[targettemperature, BoxTemp, voltage, polarity,coolingonoff,cycleNo, minsdiff, minend]
                                for item in range(len(StatusFUp_msg)):
                                    SFUp_msg.data[item] = StatusFUp_msg[item]
                                pubprocessstatus.publish(SFUp_msg)
                                #After I send the data, I wait the time from the stabilisertimevariable
                                ProcessTimeInterruptStart=datetime.now()
                                ProcessTimeInterrupt=datetime.now()
                                ProcessTimeInterruptTarget=ProcessTimeInterruptStart+timedelta(seconds = PeltierInputstabilisetime)
                                while ProcessTimeInterrupt<ProcessTimeInterruptTarget:                                    
                                    #print (MeasurementArray) ENABLE FOR TESTING
                                    #Subscriber subfunction callout
                                    #spinNode()
                                    SecurityCoolOff(cycleNo,BoxTemp,maxtempallowed,safeinterrupttime)
                                    avgBoxSensorReadings()
                                    ProcessTimeInterrupt=datetime.now()
                                time.sleep(Sleeprate)
                                #Publish status on Computer and stored log file
                                StatusFollowUp="TargetT: "+str(targettemperature)+" ; AvgBoxT: "+str(BoxTemp)+" ; Time: "+str(timenow)+" ; Volts: "+ str(voltage)+" ; Polarity: "+str(polarity)+" ; Cooling: "+str(coolingonoff)+" ; CycleNo: "+str(cycleNo)+" ; MinsDiff: "+str(minsdiff)+" ; MinEnd: "+str(minend)
                                print(StatusFollowUp)
                                createlogfile(StatusFollowUp)  
                                #Publish status on ROS
                                StatusFUp_msg=[targettemperature, BoxTemp, voltage, polarity, cycleNo, minsdiff, minend]
                                for item in range(len(StatusFUp_msg)):
                                    SFUp_msg.data[item] = StatusFUp_msg[item]
                                pubprocessstatus.publish(SFUp_msg)
                                MeasurementArray=[BoxTemp,DistanceCm,Humidity,Ethylene_ppm,CO2_ppm,PEl1Heat,PEl3Chill,PEl4Chill,LM35Sensor,DHTSensor,SampleNo]
                                #print (MeasurementArray) ENABLE FOR TESTING
                                    
                            # when the temperature aligns with target, switching off completely the Peltier would cause heat or cold to move to the 
                            # opposite surface. As the Peltier works like a resistor, I will run it at 2V to maintain the thermal shield, while
                            #the fans should eliminate the excess gradient. Source (waineh): https://forum.allaboutcircuits.com/threads/running-peltier-thermoelectric-cooler-at-low-standby-voltage.159249/
                            # From the spec: https://robu.in/product/tec1-12706-thermoelectric-peltier-cooler-12-volt-92-watt/ and 
                            # https://peltiermodules.com/peltier.datasheet/TEC1-12706.pdf I take 15V as maximum current, which is equal to 
                            # 255 passed on to the motor controller. 2V base current needed for the thermal shield is therefore 35.
                            # 35 is therefore equal to 0 thermal input, with thermal shield on. Also, I will not change the polarity towards the last cycle.
                            voltage=35
                            polarity=0
                            coolingonoff=0
                            VoltPolarity=[voltage, polarity,cycleNo]
                            data_to_send.data = VoltPolarity
                            pubvoltagepolarity.publish(data_to_send)
                            ONOFFchilldata_to_send.data = chilOnOff
                            startstopchillingunit.publish(ONOFFchilldata_to_send)    
                            print("data sent back"+ str(data_to_send))
                            print("Chill data sent back"+ str(ONOFFchilldata_to_send.data)) #This is a matrix, not an array!!
                            ProcessTimeInterruptStart=datetime.now()
                            ProcessTimeInterrupt=datetime.now()
                            ProcessTimeInterruptTarget=ProcessTimeInterruptStart+timedelta(seconds = PeltierInputstabilisetime)
                            while ProcessTimeInterrupt<ProcessTimeInterruptTarget:                                    
                                #print (MeasurementArray) ENABLE FOR TESTING
                                #Subscriber subfunction callout
                                #spinNode()
                                SecurityCoolOff(cycleNo,BoxTemp,maxtempallowed,safeinterrupttime)
                                avgBoxSensorReadings()
                                ProcessTimeInterrupt=datetime.now()
                            #read again the temperature after 30 seconds
                            spinNode()        
                            timenow=datetime.datetime.now()
                            timediff=timenow-starttime
                            minsdiff=round((timediff.total_seconds()/60),0)
                            #Publish status on Computer and stored log file
                            StatusFollowUp="TargetT: "+str(targettemperature)+" ; AvgBoxT: "+str(BoxTemp)+" ; Time: "+str(timenow)+" ; Volts: "+ str(voltage)+" ; Polarity: "+str(polarity)+" ; Cooling: "+str(coolingonoff)+" ; CycleNo: "+str(cycleNo)+" ; MinsDiff: "+str(minsdiff)+" ; MinEnd: "+str(minend)
                            print(StatusFollowUp)
                            createlogfile(StatusFollowUp)  
                            #Publish status on ROS
                            StatusFUp_msg=[targettemperature, BoxTemp, voltage, polarity,coolingonoff,cycleNo, minsdiff, minend]
                            for item in range(len(StatusFUp_msg)):
                                SFUp_msg.data[item] = StatusFUp_msg[item]
                            pubprocessstatus.publish(SFUp_msg)
                            MeasurementArray=[BoxTemp,DistanceCm,Humidity,Ethylene_ppm,CO2_ppm,PEl1Heat,PEl3Chill,PEl4Chill,LM35Sensor,DHTSensor,SampleNo]
                            print (MeasurementArray)
                
                    
                elif (changedegree!=0 and changeintervalminutes!=0):        #here the temperature in the Yaml file is the final temperature from the previous stage. 
                    starttemperature=targettemperature
                    timenowIFgradient=datetime.datetime.now()               #I wanted to make sure not to take the Box temperature as starting value to calculate the gradient, as this may generate errors.
                    while timenowIFgradient<=minend:
                        timenowIFgradient=datetime.datetime.now()  
                        timestack=timenowIFgradient+changeintervalminutes
                        temperaturestack=starttemperature+changedegree
                        timestackNo+=1
                        while timenowIFgradient<=timestack:
                            while BoxTemp != temperaturestack:
                                avgBoxSensorReadings()
                                #I set a safety threshold as I do not want the temperature to ever surpass a certain value, 
                                # as otherwise it will kill the yeast. MaxTemp is set at 43 degree Celsius. 
                                gradientvstarget=targettemperature-BoxTemp
                                DecidePolarityVoltageToAdopt(gradientvstarget)
                                #send both voltage and polarity back to Arduino
                                VoltPolarity=[voltage, polarity,cycleNo]
                                data_to_send.data = VoltPolarity
                                pubvoltagepolarity.publish(data_to_send)
                                chilOnOff=[coolingonoff]
                                ONOFFchilldata_to_send.data = chilOnOff
                                startstopchillingunit.publish(ONOFFchilldata_to_send)   
                                print("Heat data sent back"+ str(data_to_send.data)) #This is a matrix, not an array!!
                                print("Chill data sent back"+ str(ONOFFchilldata_to_send.data)) #This is a matrix, not an array!!
                                MeasurementArray=[BoxTemp,DistanceCm,Humidity,Ethylene_ppm,CO2_ppm,PEl1Heat, PEl3Chill,PEl4Chill,LM35Sensor,DHTSensor,SampleNo]           
                                SFUp_msg.data = StatusFUp_msg
                                rospy.loginfo(SFUp_msg)
                                pubprocessstatus.publish(SFUp_msg)
                                print(StatusFollowUp)
                                createlogfile(StatusFollowUp)
                                time.sleep(Sleeprate)        
                                timenowIFgradient=datetime.datetime.now()
                                #Publish status on Computer and stored log file
                                StatusFollowUp="TargetT: "+str(targettemperature)+" ; AvgBoxT: "+str(BoxTemp)+" ; Time: "+str(timenow)+" ; Volts: "+ str(voltage)+" ; Polarity: "+str(polarity)+" ; Cooling: "+str(coolingonoff)+" ; CycleNo: "+str(cycleNo)+" ; MinsDiff: "+str(minsdiff)+" ; MinEnd: "+str(minend)
                                print(StatusFollowUp)
                                createlogfile(StatusFollowUp)
                                #Publish status on ROS
                                StatusFUp_msg=[targettemperature, BoxTemp, voltage, polarity,coolingonoff,cycleNo, minsdiff, minend]
                                for item in range(len(StatusFUp_msg)):
                                    SFUp_msg.data[item] = StatusFUp_msg[item]
                                pubprocessstatus.publish(SFUp_msg)
                                time.sleep(Sleeprate)
                            voltage=35
                            polarity=0
                            coolingonoff=0
                            VoltPolarity=[voltage, polarity,cycleNo]
                            data_to_send.data = VoltPolarity
                            pubvoltagepolarity.publish(data_to_send)
                            ONOFFchilldata_to_send.data = chilOnOff
                            startstopchillingunit.publish(ONOFFchilldata_to_send)    
                            print("data sent back"+ str(data_to_send))
                            print("Chill data sent back"+ str(ONOFFchilldata_to_send.data)) #This is a matrix, not an array!!
                            ProcessTimeInterruptStart=datetime.now()
                            ProcessTimeInterrupt=datetime.now()
                            ProcessTimeInterruptTarget=ProcessTimeInterruptStart+timedelta(seconds = PeltierInputstabilisetime)
                            while ProcessTimeInterrupt<ProcessTimeInterruptTarget:                                    
                                #print (MeasurementArray) ENABLE FOR TESTING
                                #Subscriber subfunction callout
                                #spinNode()
                                SecurityCoolOff(cycleNo,BoxTemp,maxtempallowed,safeinterrupttime)
                                avgBoxSensorReadings()
                                ProcessTimeInterrupt=datetime.now()
                            #read again the temperature after 30 seconds
                            spinNode()        
                            timenow=datetime.datetime.now()
                            timediff=timenow-starttime
                            minsdiff=round((timediff.total_seconds()/60),0)
                            #Publish status on Computer and stored log file
                            StatusFollowUp="TargetT: "+str(targettemperature)+" ; AvgBoxT: "+str(BoxTemp)+" ; Time: "+str(timenow)+" ; Volts: "+ str(voltage)+" ; Polarity: "+str(polarity)+" ; Cooling: "+str(coolingonoff)+" ; CycleNo: "+str(cycleNo)+" ; MinsDiff: "+str(minsdiff)+" ; MinEnd: "+str(minend)
                            print(StatusFollowUp)
                            createlogfile(StatusFollowUp)  
                            #Publish status on ROS
                            StatusFUp_msg=[targettemperature, BoxTemp, voltage, polarity,coolingonoff,cycleNo, minsdiff, minend]
                            for item in range(len(StatusFUp_msg)):
                                SFUp_msg.data[item] = StatusFUp_msg[item]
                            pubprocessstatus.publish(SFUp_msg)
                            time.sleep(Sleeprate)
                            MeasurementArray=[BoxTemp,DistanceCm,Humidity,Ethylene_ppm,CO2_ppm,PEl1Heat,PEl3Chill,PEl4Chill,LM35Sensor,DHTSensor,SampleNo]
                            print (MeasurementArray)                      
        #Publish status on Computer and stored log file
        StatusFollowUp="TargetT: "+str(targettemperature)+" ; AvgBoxT: "+str(BoxTemp)+" ; Time: "+str(timenow)+" ; Volts: "+ str(voltage)+" ; Polarity: "+str(polarity)+" ; CycleNo: "+str(cycleNo)+" ; MinsDiff: "+str(minsdiff)+" ; MinEnd: "+str(minend)
        print(StatusFollowUp)
        createlogfile(StatusFollowUp)  
        #Publish status on ROS
        StatusFUp_msg=[targettemperature, BoxTemp, voltage, polarity,coolingonoff,cycleNo, minsdiff, minend]
        for item in range(len(StatusFUp_msg)):
            SFUp_msg.data[item] = StatusFUp_msg[item]
        pubprocessstatus.publish(SFUp_msg)
        #After the machine has finished its cycle, it should kill roscore and its sub processes
        rospy.launch.shutdown()
  
  
if __name__ == '__main__':
    
    # you could name this function
    try:
        main()
    except rospy.ROSInterruptException:
        pass
