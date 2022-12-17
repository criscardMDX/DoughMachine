/* Carbon Dioxide Parts Per Million Meter: CO2PPM
 *
 * Notes, CO2 Sensor:
 * Atmospheric CO2 Level..............400ppm
 * Average indoor co2.............350-450ppm
 * Maxiumum acceptable co2...........1000ppm
 * Dangerous co2 levels.............>2000ppm
 */
/* Ethylene readings:
 * In the absence of alcohol (around 120ppm)
 * In the presence of alcohol (around 500ppm)
 */

/*duty cycle in arduino is represented by a 8 bit value: 0 - 255
0 --> 0%
127 -->50%
255 --> 100%
*/


//---------------------------------------------------------------------------------------------------------------
//                                                   DEFINES
//---------------------------------------------------------------------------------------------------------------
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h> /* Variables are declared for storing the ROS data types; this is for integers  */
#include <rosserial_arduino/Adc.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32MultiArray.h>

ros::NodeHandle nh;
std_msgs::String my_string_data;  /*just a reminder, not needed now. */

/* PUBLISHER BLOCK: One topic for each measurement that I will transfer to ROS */
std_msgs::Float32 my_temp_float_data; /*Average box temperature in centigrades */
ros::Publisher boxtemp_pub_ardu("/box_temp_Celsius", &my_temp_float_data  ); /*publisher for the same */

std_msgs::Float32 my_expansion_data; /*Distance between the dough and the ultrasound sensor */
ros::Publisher expansion_pub_ardu("/Distance_Cms", &my_expansion_data  ); /*publisher for the same */

std_msgs::Float32 my_humidity_float_data; /*Humidity inside the box */
ros::Publisher humidity_pub_ardu("/Humidity", &my_humidity_float_data  ); /*publisher for the same */

std_msgs::Float32 my_ethy_float_data; /*Ethylene levels inside the box */
ros::Publisher ethy_pub_ardu("/Ethylene_ppm", &my_ethy_float_data  ); /*publisher for the same */

std_msgs::Float32 my_CO2ppm_float_data; /*Ethylene levels inside the box */
ros::Publisher CO2ppm_pub_ardu("/CO2_ppm", &my_CO2ppm_float_data  ); /*publisher for the same */

std_msgs::Float32 my_tempc1_float_data; /*Peltier 1, temperature differential dissipation heath sink */
ros::Publisher tempc1_pub_ardu("/PEl_1_single", &my_tempc1_float_data  ); /*publisher for the same */

std_msgs::Float32 my_tempc2_float_data; /*Peltier system control unit, temperature differential dissipation heath sink */
ros::Publisher tempc2_pub_ardu("/PEls_controller", &my_tempc2_float_data  ); /*publisher for the same */

std_msgs::Float32 my_tempc3_float_data; /*Peltier 2 (couple), temperature differential dissipation heath sink */
ros::Publisher tempc3_pub_ardu("/PEl_2_couple", &my_tempc3_float_data  ); /*publisher for the same */

std_msgs::Float32 my_tempc4_float_data; /*Peltier 3 (couple), temperature differential dissipation heath sink */
ros::Publisher tempc4_pub_ardu("/PEl_3_couple", &my_tempc4_float_data  ); /*publisher for the same */


//---------------------------------------------------------------------------------------------------------------
//                                          SUBFUNCTION PELTIER CONTROL
//---------------------------------------------------------------------------------------------------------------

void mgmtinput(const std_msgs::Int32MultiArray& voltageprovided){
int PEl_voltage=voltageprovided.data[0];
int PEl_polarity=voltageprovided.data[1];
int CycleNumber=voltageprovided.data[2];
int PEl_PolCharPrev='R';
int PEl_PolChar='R';
Serial.println("PEl voltage is: "+String(PEl_voltage));
  if (PEl_polarity==0){ 
     int PEl_PolChar='L';
     if (PEl_PolChar!=PEl_PolCharPrev){
        closeMotor(PEl_PolCharPrev);
        delay(100);
     }
  }
  if (PEl_polarity==1){ 
    int PEl_PolChar='R';
    if (PEl_PolChar!=PEl_PolCharPrev){
        closeMotor(PEl_PolCharPrev);
        delay(100);
     }
  }
  if (PEl_voltage!=0) {
    setMotor(PEl_PolChar,PEl_voltage);
    Serial.println("Voltage rate: " + String(PEl_voltage));
    Serial.println("Polarity: " + PEl_PolChar);
  }
   else{
    closeMotor(PEl_PolChar);
    }   
}
ros::Subscriber<std_msgs::Int32MultiArray> subvolpol("/voltageAndPolarityInput", &mgmtinput);

long publisher_timer;

#include "DHT.h"
#define DHTPIN 12                     /* Digital pin connected to the DHT sensor*/
#define DHTTYPE DHT11                 /* DHT 11*/
#include "DHT.h"
#define sensorPintmpLM35 A8 /*Temperature Sensor Pin */
DHT dht(DHTPIN, DHTTYPE);

#define co2Zero 55                    /*calibrated CO2 0 level*/
const int npnpin = 4;                 /*assigning Arduino pins for the giving signal to MOSFET (it was 6 before, but I swapped to make room for the PEl controller*/
float EthysensorValue;                /*variable to store sensor value Ethylene sensor */
float tempc1;                         /*variable to store temperature in degree Celsius - Fan1*/
float vout1;                          /*temporary variable to hold sensor reading - Fan1*/
float tempc2;                         /*variable to store temperature in degree Celsius - Fan2*/
float vout2;                          /*temporary variable to hold sensor reading - Fan2*/
float tempc3;                         /*variable to store temperature in degree Celsius - Fan3*/
float vout3;                          /*temporary variable to hold sensor reading - Fan3*/
float tempc4;                         /*variable to store temperature in degree Celsius - Fan4*/
float vout4;                          /*temporary variable to hold sensor reading - Fan4*/
float tempAverage;                    /*variable to store temperature in degree Celsius - Fan1*/
float tempMax = 0;                    /*variable to store temperature in degree Celsius - Fan1*/

unsigned long interruptcounter = 0;   /* loop counter */
unsigned long previousMillis = 0;     /* to store previous time */ 
long duration, inches, cm;
const long interruptinterval = 2000;  /* every measurement is taken after 2 seconds */
const int pingPin = 5; // Trigger Pin of Ultrasonic Sensor
const int echoPin = 2; // Echo Pin of Ultrasonic Sensor


/* code for BTS7960 Motor driver,on timers 1 and 3, inspired by Mohannad Rawashdeh */
int RPWM=3;
int LPWM=11;
int L_EN=7;
int R_EN=8;
void setPWMfrequency(int freq){
    TCCR1B = TCCR2B & 0b11111000 | freq ;
    TCCR3B = TCCR2B & 0b11111000 | freq ;
}
void MotorActiveStatus(char Side,boolean s){
 boolean state=s;
   if(Side=='R'){
   digitalWrite(R_EN,s);
   }
   if(Side=='L'){
   digitalWrite(L_EN,s);
   }    
}
void setMotor(char side,byte pwm){
   if(side=='R'){
    analogWrite(RPWM,pwm);
   }
   if(side=='L'){
   analogWrite(LPWM,pwm);
   }
}
void closeMotor(char side){
   if(side=='R'){
   digitalWrite(RPWM,LOW);
   }
   if(side=='L'){
   digitalWrite(LPWM,LOW);
   }
}


/* Here it is important that every time that there is an inversion in polarity, the machine receives a 
 * voltage 0, before reversing, to switch on the polarity on the controller 

 void subscriberCallback(const std_msgs::Int32MultiArray &voltageprovided) {
         } 
*/

/* Old example of motor control, used to test the controller
  for(int i=0;i<256;i++){
    setMotor('R',i);
    delay(50);
  }
  delay(500);
  closeMotor('R');
  delay(1000);
  for(int i=0;i<256;i++){
    setMotor('L',i);
    delay(50);
  }
  delay(500);
  closeMotor('L');
  delay(1000);
  */

//---------------------------------------------------------------------------------------------------------------
//                                                  SETUP
//---------------------------------------------------------------------------------------------------------------
void setup() {

nh.initNode();
nh.subscribe(subvolpol);
nh.advertise(boxtemp_pub_ardu);
nh.advertise(expansion_pub_ardu);
nh.advertise(humidity_pub_ardu);
nh.advertise(ethy_pub_ardu);
nh.advertise(CO2ppm_pub_ardu);
nh.advertise(tempc1_pub_ardu);
nh.advertise(tempc2_pub_ardu);
nh.advertise(tempc3_pub_ardu);
nh.advertise(tempc4_pub_ardu);

pinMode(npnpin,OUTPUT);/* assigning the transistor  pin as an output of Arduino*/
/*safety speed reset of the motor*/
analogWrite(npnpin,0);
setPWMfrequency(0x02);// timer 2 , 3.92KHz
dht.begin();
pinMode(3,OUTPUT);
pinMode(11,OUTPUT);
pinMode(7,OUTPUT);
pinMode(8,OUTPUT);
digitalWrite(3,LOW);
digitalWrite(11,LOW);
digitalWrite(7,LOW);
digitalWrite(8,LOW);
delay(1000);
MotorActiveStatus('R',true);
MotorActiveStatus('L',true);
Serial.begin(57600);
}
//---------------------------------------------------------------------------------------------------------------
//                                               MAIN LOOP
//---------------------------------------------------------------------------------------------------------------


void loop() {
float val = 1023;
float duty = map(val,0,1023,0,255);
analogWrite(npnpin, duty); //here's how to generate PWM signal from Digital arduino pin

//int PEl_voltage=voltageprovided.data[0];
//int PEl_polarity=voltageprovided.data[1];
//int CycleNumber=voltageprovided.data[2];


/*In this block I use an interrupt to establish the frequency of all sensors' measurements.
I do not want to measure too often as this process does not need constant monitoring */

unsigned long currentMillis= millis(); /* and compare with previous time taken */
if (currentMillis - previousMillis >= interruptinterval) {
  previousMillis = currentMillis; // save current time as prev
  
/*CO2 sensor reading stack */  
  int co2now[10];                               //int array for co2 readings
  int co2raw = 0;                               //int for raw value of co2
  int co2comp = 0;                              //int for compensated co2 
  int co2ppm = 0;                               //int for calculated ppm
  int co2sampling = 0;                                  //int for averaging
  int grafX = 0;                                //int for x value of graph
  for (int x = 0;x<10;x++){                   //samplpe co2 10x over 2 seconds
    co2now[x]=analogRead(A7);
    //Serial.println(co2now[x]);
   }

  for (int x = 0;x<10;x++){                     //add samples together
    co2sampling=co2sampling + co2now[x];
   }
  co2raw = co2sampling/10;                    //divide samples by 10
  co2comp = co2raw - co2Zero;                 //get compensated value
  co2ppm = map(co2comp,0,1023,50,2000);      //map value for atmospheric levels

/* Ethylene Sensor reading stack */
  EthysensorValue = analogRead(A6); // read analog input pin A6 for the Ethylene sensor
/* Humidity andf Temperature sensor reading stack */
  float DHTtemp= dht.readTemperature();
  float DHThumidity= dht.readHumidity();
/* LM35 Temperature sensor stack */
  int readingLM35 = analogRead(sensorPintmpLM35);
  float voltageLM35 = readingLM35 * (5.0 / 1024.0);   /* Convert that reading into voltage */
  float LM35temperatureC = voltageLM35 * 100;     /* Convert the voltage into the temperature in Celsius */
  float AvgBoxTemperature = (LM35temperatureC+DHTtemp)/2;

  vout1=analogRead(A0); //Reading the value from sensor
  vout2=analogRead(A1); //Reading the value from sensor
  vout3=analogRead(A2); //Reading the value from sensor
  vout4=analogRead(A3); //Reading the value from sensor
  tempc1=(vout1*500)/1023;
  tempc2=(vout2*500)/1023;
  tempc3=(vout3*500)/1023;
  tempc4=(vout4*500)/1023;
  Serial.println("Temperature for Fan Peltier1 (Single) is: " +String(tempc1));
  Serial.println("Temperature for Fan Peltier Control (2) is: " +String(tempc2));
  Serial.println("Temperature for Fan Peltier2 (Parallel) is: " +String(tempc3));
  Serial.println("Temperature for Fan Peltier3 (Parallel) is: " +String(tempc4));
  Serial.println("The fan duty rate 0:255 is: " +String(duty));
  Serial.println ("CO2 ppm is: " + String(co2ppm));
  Serial.println ("Ethylene ppm is: " + String(EthysensorValue));
  Serial.println ("Box temperature Sensor1 is: " + String(DHTtemp));
  Serial.println ("Humidity: " + String(DHThumidity));
  Serial.print("LM35 Temperature: ");
  Serial.println(LM35temperatureC);
  Serial.print("Box Average Temperature: ");
  Serial.print(AvgBoxTemperature);  
  Serial.print("\xC2\xB0"); // shows degree symbol
  Serial.println("C  |  ");
  Serial.print(inches);
  Serial.print("in, ");
  Serial.print(cm);
  Serial.println("cm");
  Serial.println();
/*After prinitng on the serial, I transfer the same data from Arduino to ROS */
  my_CO2ppm_float_data.data = co2ppm;
  CO2ppm_pub_ardu.publish(&my_CO2ppm_float_data);
  my_temp_float_data.data = AvgBoxTemperature;
  boxtemp_pub_ardu.publish(&my_temp_float_data);
  my_expansion_data.data = cm;
  expansion_pub_ardu.publish(&my_expansion_data);
  my_humidity_float_data.data = DHThumidity;
  humidity_pub_ardu.publish(&my_humidity_float_data);
  my_ethy_float_data.data = EthysensorValue;
  ethy_pub_ardu.publish(&my_ethy_float_data); 
  my_tempc1_float_data.data = tempc1;
  tempc1_pub_ardu.publish(&my_tempc1_float_data);
  my_tempc2_float_data.data = tempc2;
  tempc2_pub_ardu.publish(&my_tempc2_float_data);
  my_tempc3_float_data.data = tempc3;
  tempc3_pub_ardu.publish(&my_tempc3_float_data);
  my_tempc4_float_data.data = tempc4;
  tempc4_pub_ardu.publish(&my_tempc4_float_data);
  
  }
nh.spinOnce();

/*Here I set the distance measurement from the ultrasonic sensor*/
pinMode(pingPin, OUTPUT);
digitalWrite(pingPin, LOW);
delayMicroseconds(2);
digitalWrite(pingPin, HIGH);
delayMicroseconds(10);
digitalWrite(pingPin, LOW);
pinMode(echoPin, INPUT);
duration = pulseIn(echoPin, HIGH);
inches = microsecondsToInches(duration);
cm = microsecondsToCentimeters(duration);
}

long microsecondsToInches(long microseconds) {
   return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds) {
   return microseconds / 29 / 2;
}
