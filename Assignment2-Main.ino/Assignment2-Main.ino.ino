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
//                                  INITIALISATION OF LED INDICATORS
//---------------------------------------------------------------------------------------------------------------
const int BluePin = 43;               // Chilling process ON
const int AmberPin = 45;              // Heating process ON
const int GreenPin = 49;              // Machine Process Finished
const int RedPin = 47;                // Machine Still Processing
/*Const int uses less memory */

/* I need booleans to turn on or off within the various functions*/
bool BlueSwitch =0;
bool AmberSwitch =0;
bool GreenSwitch =0;
bool RedSwitch =0;

//---------------------------------------------------------------------------------------------------------------
//                                  INITIALISATION OF THE COOLING UNIT
//---------------------------------------------------------------------------------------------------------------

const int measurePin0 = A0;           /* Fan 4 Chilling Unit */
const int measurePin1 = A1;           /* Fan 3 Chilling Unit */
const int measurePin2 = A2;           /* Fan 1 Heating Unit */
const int RelayPin = 7;               /* The relay switches the cooling unit ON or OFF */
const int coolPin0 = 5;               /* This Pin controls Fan 4, MOSFET at Pin 5 */
const int coolPin1 = 6;               /* This Pin controls Fan 3, MOSFET at Pin 6 */
const long switchTime = 10000;        /*time between switches in ms */
float temperature0 = 22.0;            /*Starting temperature for the average at Fan 4 */
float temperature1 = 22.0;            /*Starting temperature for the average at Fan 3 */
float temperature2 = 22.0;            /*Starting temperature for the average at Fan 1 */
float temperature3 = 22.0;            /*Starting temperature for the LM35 Sensor inside the box */
long coolTimer0=0;                    /* Interrupt cooling Fan 4 */
long coolTimer1=0;                    /* Interrupt cooling Fan 3 */
int isCooling0 = LOW;                 /* Initial state Cooler 4 - OFF */
int isCooling1 = LOW;                 /* Initial state Cooler 3 - OFF */
float CmdPublished[7];

//---------------------------------------------------------------------------------------------------------------
//                                  INITIALISATION OF ALL SENSORS
//---------------------------------------------------------------------------------------------------------------

/* Definitions related to the use of sensors */
#include "DHT.h"                      /*This is the library for the humidity and temperature sensor DHT */
#define DHTPIN 12                     /* Digital pin connected to the DHT sensor*/
#define DHTTYPE DHT11                 /* DHT 11*/
#define sensorPintmpLM35 A8           /* LM35 Sensor, placed inside the short side of the box, close to the ultrasound sensor*/
DHT dht(DHTPIN, DHTTYPE);
#define co2Zero 55                    /*calibrated CO2 0 level*/
const long interruptinterval = 2000;  /* every measurement is taken after 2 seconds */
const int pingPin = 23;               /* Trigger Pin of Ultrasonic Sensor */
const int echoPin = 22;               /* Echo Pin of Ultrasonic Sensor */
int CmdPublishedLen=0;
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
long publisher_timer;


//---------------------------------------------------------------------------------------------------------------
//                                  DEFINITIONS FOR ROS STANDARD MESSAGE PUBLISHERS
//---------------------------------------------------------------------------------------------------------------
#include <LiquidCrystal_I2C.h>        /*Library for the LCD screen*/
#include <RunningMedian.h>            /* Library to calculate the median, used to derive a correct temperature sensor read */
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>          /* Variables are declared for storing the ROS data types; this is for integers  */
#include <rosserial_arduino/Adc.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>

ros::NodeHandle nh;

/* PUBLISHER BLOCK: One topic for each measurement that I will transfer to ROS */
std_msgs::Float32 AVG_temp_float_data; /*Average box temperature in centigrades */
ros::Publisher boxtemp_pub_ardu("/box_temp_Celsius", &AVG_temp_float_data  ); /*publisher for the same */

std_msgs::Float32 my_expansion_data; /*Distance between the dough and the ultrasound sensor */
ros::Publisher expansion_pub_ardu("/Distance_Cms", &my_expansion_data  ); /*publisher for the same */

std_msgs::Float32 my_humidity_float_data; /*Humidity inside the box */
ros::Publisher humidity_pub_ardu("/Humidity", &my_humidity_float_data  ); /*publisher for the same */

std_msgs::Float32 my_ethy_float_data; /*Ethylene levels inside the box */
ros::Publisher ethy_pub_ardu("/Ethylene_ppm", &my_ethy_float_data  ); /*publisher for the same */

std_msgs::Float32 my_CO2ppm_float_data; /* CO2 levels inside the box */
ros::Publisher CO2ppm_pub_ardu("/CO2_ppm", &my_CO2ppm_float_data  ); /*publisher for the same */

std_msgs::Float32 PEl3_temp_float_data; /* PEl 3, chilling unit, temperature taken on the fan at the base of the box */
ros::Publisher PEl3temp_pub_ardu("/PEl_3_chill", &PEl3_temp_float_data  ); /*publisher for the same */

std_msgs::Float32 PEl4_temp_float_data; /* PEl 4, chilling unit, temperature taken on the fan at the base of the box */
ros::Publisher PEl4temp_pub_ardu("/PEl_4_chill", &PEl4_temp_float_data  ); /*publisher for the same */

std_msgs::Float32 PEl1_temp_float_data; /*PEl 1, heating unit, temperature taken on the fan at the base of the box  */
ros::Publisher PEl1temp_pub_ardu("/PEl_1_heat", &PEl1_temp_float_data  ); /*publisher for the same */

std_msgs::Float32 LM35_temp_float_data; /* LM35 Temperature sensor, top side of the box */
ros::Publisher LM35temp_pub_ardu("/LM35_sensor", &LM35_temp_float_data  ); /*publisher for the same */

std_msgs::Float32 DHT_temp_float_data; /* DHT Temperature sensor, top side of the box */
ros::Publisher DHTtemp_pub_ardu("/DHT_sensor", &DHT_temp_float_data  ); /*publisher for the same */

/* Here I built a diagnostic, to check if the measurements' message from ROS is received correctly
std_msgs::Float32MultiArray DiagnosticprocessStatePublisher; 
ros::Publisher Diag1Proc_pub_ardu("/DiagnosticProcessStat", &DiagnosticprocessStatePublisher  ); /*publisher for the same */

//---------------------------------------------------------------------------------------------------------------
//                                          SUBFUNCTION PELTIER MOTOR CONTROL
//---------------------------------------------------------------------------------------------------------------

/* I kept the motor as to maintain the possibility of changing polarity. 
 * In fact, however, I keep the motor only because 1) It's part of the assignment scoring and 2) I will use one Peltier for heating and two for cooling.
 * Here I monted the PEl back with the cooling position up, ro the R means cooling and the L means heating.
 * Polarity therefore will be: 0=L=warm and 1=R=chill
 */ 

void mgmtinput(const std_msgs::Int32MultiArray& voltageprovided){
int PEl_voltage=voltageprovided.data[0];
int PEl_polarity=voltageprovided.data[1];
int CycleNumber=voltageprovided.data[2];
int PEl_PolCharPrev='R'; /*Default on Chill */
int PEl_PolChar='R';

//Serial.println("PEl voltage is: "+String(PEl_voltage)); L= Heat, R=Chill
  if (PEl_polarity==0){ 
     int PEl_PolChar='L';
     if (PEl_PolChar!=PEl_PolCharPrev){
        /*Machine is ON RedLED Goes ON */
        digitalWrite(RedPin, HIGH);
        digitalWrite(GreenPin, LOW);
        digitalWrite(BluePin, LOW);
        digitalWrite(AmberPin, HIGH);
        BlueSwitch =0;
        AmberSwitch =1;
        GreenSwitch =0;
        RedSwitch =1;

        closeMotor(PEl_PolCharPrev);
        delay(100);
     }
  }
  if (PEl_polarity==1){ 
    int PEl_PolChar='R';
    if (PEl_PolChar!=PEl_PolCharPrev){
        /*Machine is ON RedLED Goes ON */
        digitalWrite(RedPin, HIGH);
        digitalWrite(BluePin, HIGH);
        digitalWrite(GreenPin, LOW);
        digitalWrite(AmberPin, LOW);
        BlueSwitch =1;
        AmberSwitch =0;
        GreenSwitch =0;
        RedSwitch =1;
        closeMotor(PEl_PolCharPrev);
        delay(100);
     }
  }
  if (PEl_voltage!=0) {
    setMotor(PEl_PolChar,PEl_voltage);
    //Serial.println("Voltage rate: " + String(PEl_voltage));
    //Serial.println("Polarity: " + PEl_PolChar);
  }
   else{
    closeMotor(PEl_PolChar);
        digitalWrite(BluePin, LOW);
        digitalWrite(AmberPin, LOW);
    
    }   
}

//---------------------------------------------------------------------------------------------------------------
//                                          SUBFUNCTION CHILLING UNIT
//---------------------------------------------------------------------------------------------------------------

/* As I separated heated from chilling, due to issues with heat accumulation on the PEls, I created a separate message to activate the chilling unit 
 *  and I will decide whether the motor or the unit get activated from ROS. 1 means "SWITCH ON" while 0 "SWITCH OFF".
 *  I used an array, as it is easier in case I have to add instructions.
 */

void cmdchill(const std_msgs::Int32MultiArray& coolingmsg){
  int CoolingOnOff=coolingmsg.data[0];
  if (CoolingOnOff==1)
    { 
      if (  (isCooling0 == LOW) & (isCooling1 == LOW) & ((coolTimer0 - millis()) > switchTime) )
        {
            if (RelayPin==LOW){
            // Let's turn the relay ON
                digitalWrite(RelayPin, HIGH);
            }
        isCooling0 = HIGH;
        isCooling1 = HIGH;
        coolTimer0 = millis();
        digitalWrite(RedPin, HIGH);
        digitalWrite(BluePin, HIGH);
        digitalWrite(GreenPin, LOW);
        digitalWrite(AmberPin, LOW);
        RedSwitch =1;
        BlueSwitch =1;
        AmberSwitch =0;
        GreenSwitch =0;
        }
      else 
      {
        if ( (isCooling0 == HIGH) & (isCooling1 == HIGH) & ((coolTimer0 - millis()) > switchTime) )
          {
            isCooling0 = LOW;
            isCooling1 = LOW;
            digitalWrite(RedPin, HIGH);
            digitalWrite(BluePin, LOW);
            digitalWrite(GreenPin, LOW);
            digitalWrite(AmberPin, LOW);
            RedSwitch =1;
            BlueSwitch =0;
            AmberSwitch =0;
            GreenSwitch =0;
            coolTimer0 = millis();
          }
      }
    }
  else 
    {
      if (RelayPin==HIGH){
            // Let's turn the relay OFF
                digitalWrite(RelayPin, LOW);
                digitalWrite(RedPin, LOW);
                digitalWrite(BluePin, LOW);
                digitalWrite(GreenPin, LOW);
                digitalWrite(AmberPin, LOW);
                RedSwitch =0;
                BlueSwitch =0;
                AmberSwitch =0;
                GreenSwitch =0;
            }
    }
  digitalWrite(coolPin0,isCooling0);
  digitalWrite(coolPin1,isCooling1);
  delay(50);
}

//---------------------------------------------------------------------------------------------------------------
//                                          SUBFUNCTION LCD DASHBOARD
//---------------------------------------------------------------------------------------------------------------
/*This function obtains a multiarray of float data from ROS node. These data includ target and current temperature, 
 * for display on the LCD screen. This function should contain all needed to create a dashboard, placed on top of the
 * doughmaster Lid. Note for Future, I need to learn how to extract the data from the function. I tried "return" but 
 * it did not work.
 */
 
void cmdoutcome(const std_msgs::Float32MultiArray& subprocstatus){
  #define ARRAYSIZE 10
  String Label [ARRAYSIZE]= {"Target Temp","Current Temp", "Voltage", "Polarity", "Cycle Nr", "Mins Past", "Mins End"};
  float CmdPublished;
  String LabelStart="Connected";
  unsigned long previousMillis = 0;
  const long TimeInterval = 1200;
  LiquidCrystal_I2C lcd(0x27,16,2);
  lcd.init();
  lcd.clear();         
  lcd.backlight();      // Make sure backlight is on
  lcd.setCursor(2,0);
  lcd.print("Connection OK");
    for (int countL = 0;countL<7;countL++){
      LabelStart=Label[countL];
      unsigned long TimeStart= millis(); /* take the time now, start of the process */
      unsigned long TimeEnd= TimeStart+TimeInterval; /* take the time now, start of the process */
      unsigned long TimeNow= millis(); /* take the time now, for the Milliseconds' counter */
      while (TimeNow<=TimeEnd) {
        //if (Label[countL]=!LabelStart){
          lcd.clear();
          lcd.setCursor(2,0);
          lcd.print(Label[countL]);
          lcd.setCursor(2,1);
          CmdPublished=subprocstatus.data[countL];
          lcd.print(CmdPublished);  
          //}
          TimeNow= millis();
      }  
    }
  //Serial.println(CmdPublished);
}

//---------------------------------------------------------------------------------------------------------------
//                                          SUBSCRIBER FUNCTIONS
//---------------------------------------------------------------------------------------------------------------

//This subscriber receives the instructions for the heating and cooling units to operate. These instructions are: Voltage and Polarity(heat/chill).
ros::Subscriber<std_msgs::Int32MultiArray> subvolpol("/voltageAndPolarityInput", &mgmtinput);

//This subscriber reads the diagnostics from ROS, to be launched on the LCD display.
ros::Subscriber<std_msgs::Float32MultiArray> subcmdmsg("/ProcessStatus", &cmdoutcome);

//This subscriber activates and manages the Cooling Unit
ros::Subscriber<std_msgs::Int32MultiArray> activatechill("/CoolingOn", &cmdchill);

//---------------------------------------------------------------------------------------------------------------
//              INITIALIZATION AND RUNNING FUNCTION FOR THE PELTIER HEATING MOTOR CONTROL
//---------------------------------------------------------------------------------------------------------------

RunningMedian temperatureArray = RunningMedian(30);
/* code for BTS7960 Motor driver,on timers 1 and 3, inspired by Mohannad Rawashdeh */
int RPWM=8;
int LPWM=11;
int L_EN=9;
int R_EN=10;
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


//---------------------------------------------------------------------------------------------------------------
//                                                  LCD SETUP
//---------------------------------------------------------------------------------------------------------------


LiquidCrystal_I2C lcd(0x27,16,2);  /* set the LCD address to 0x27 for a 16 chars and 2 line display (to determine address, 
                                     use scan sketch at: https://lastminuteengineers.com/i2c-lcd-arduino-tutorial/  */

//---------------------------------------------------------------------------------------------------------------
//                                                  SETUP
//---------------------------------------------------------------------------------------------------------------
void setup() {
  
  // Initialisation of LCD Screen//
  lcd.init();
  lcd.clear();         
  lcd.backlight();      // Make sure backlight is on
  // Print a message on both lines of the LCD.
  lcd.setCursor(2,0);   //Set cursor to character 2 on line 0
  lcd.print("Dough Master");
  lcd.setCursor(2,1);   //Move cursor to character 2 on line 1
  lcd.print("Ready for use");
  // end of LCD Initialisation
  
  //Initialisation of LEDs
  pinMode(BluePin, OUTPUT);
  pinMode(GreenPin, OUTPUT);
  pinMode(RedPin, OUTPUT);
  pinMode(AmberPin, OUTPUT);
  digitalWrite(BluePin, LOW);
  digitalWrite(GreenPin, LOW);
  digitalWrite(RedPin, LOW);
  digitalWrite(AmberPin, LOW);

  // Set RelayPin as an output pin
  pinMode(RelayPin, OUTPUT);
  
  //analogReference(EXTERNAL );
  pinMode(coolPin0,OUTPUT);
  pinMode(coolPin1,OUTPUT);
  digitalWrite(coolPin0,isCooling0);
  digitalWrite(coolPin1,isCooling1);
  
  // Let's turn on the relay OFF
  digitalWrite(RelayPin, LOW);
  
  // Initialisation of ROS Node and Arduino data publishers
  nh.initNode();
  nh.subscribe(subvolpol);
  nh.subscribe(subcmdmsg);            /*Subscriber to process measurements */
  nh.subscribe(activatechill);
  nh.advertise(boxtemp_pub_ardu);
  nh.advertise(expansion_pub_ardu);
  nh.advertise(humidity_pub_ardu);
  nh.advertise(ethy_pub_ardu);
  nh.advertise(CO2ppm_pub_ardu);
  nh.advertise(PEl3temp_pub_ardu);
  nh.advertise(PEl4temp_pub_ardu);
  nh.advertise(PEl1temp_pub_ardu);
  nh.advertise(LM35temp_pub_ardu);
  nh.advertise(DHTtemp_pub_ardu);
  /*nh.advertise(Diag1Proc_pub_ardu);   /* Diagnostic Process Status. Activate only for diagnostics */
  
  // Initialisation of Motor Controls //
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
  /*Machine is ON RedLED Goes ON */
  RedSwitch =1;
  
  if(RedSwitch=1){
    digitalWrite(RedPin, HIGH);
  }
  else {
    digitalWrite(RedPin, LOW);
  }
  
  if(AmberSwitch=1){
    digitalWrite(AmberPin, HIGH);
  }
  else {
    digitalWrite(AmberPin, LOW);
  }
  
  if(BlueSwitch=1){
    digitalWrite(BluePin, HIGH);
  }
  else {
    digitalWrite(BluePin, LOW);
  }
  
  if(GreenSwitch=1){
    digitalWrite(GreenPin, HIGH);
  }
  else {
    digitalWrite(GreenPin, LOW);
  }
  
  digitalWrite(RedPin, HIGH);
  digitalWrite(BluePin, LOW);
  digitalWrite(GreenPin, LOW);
  digitalWrite(AmberPin, LOW);
  
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
  int co2sampling = 0;                          //int for averaging
  int grafX = 0;                                //int for x value of graph
  for (int x = 0;x<10;x++){                     //samplpe co2 10x over 2 seconds
    co2now[x]=analogRead(A7);
    //Serial.println(co2now[x]);
   }
  for (int x = 0;x<10;x++){                     //add samples together
    co2sampling=co2sampling + co2now[x];
   }
  co2raw = co2sampling/10;                      //divide samples by 10
  co2comp = co2raw - co2Zero;                   //get compensated value
  co2ppm = map(co2comp,0,1023,50,2000);         //map value for atmospheric levels

  /* Temperature measured at the base of the box Fans 3 and 4, Cooling Unit */
  temperature0 = (0.8 * temperature0) + (0.2 * temperatureC(measurePin0));
  temperature1 = (0.8 * temperature1 )+ (0.2 * temperatureC(measurePin1));

  /* Temperature measured at the base of the box Fan 1, Heating or Cooling Unit */
  temperature2 = (0.8 * temperature2 )+ (0.2 * temperatureC(measurePin2));
  
  /* Temperature measured by the LM35 sensor inside the box */
  temperature3 = (0.8 * temperature3 )+ (0.2 * temperatureC(sensorPintmpLM35));
  float LM35temperatureC = temperature3;     /* Convert the voltage into the temperature in Celsius */
  
  /* Ethylene Sensor reading stack */
  EthysensorValue = analogRead(A6); // read analog input pin A6 for the Ethylene sensor

  /* Humidity and Temperature sensor reading stack */
  float DHTtemp= dht.readTemperature();
  float DHThumidity= dht.readHumidity();
  /* Average Box Temperature */
  float AvgBoxTemperature = (LM35temperatureC+DHTtemp+temperature0+temperature1+temperature2)/5;

  /* Serial Monitor Printing for diagnostics 
  Serial.println("Temperature for Fan 4 Chilling Unit is: " +String(temperature0));
  Serial.println("Temperature for Fan 3 Chilling Unit is: " +String(temperature1));
  Serial.println("Temperature for Fan 1 Heating Unit is: " +String(temperature2));
  Serial.println ("Box temperature DHT Sensor is: " + String(DHTtemp));
  Serial.print("Box temperature LM35 Sensor: ");
  Serial.println(LM35temperatureC);
  Serial.print("Box Average Temperature: ");
  Serial.print(AvgBoxTemperature);
  Serial.print("\xC2\xB0"); // shows degree symbol
  Serial.println("C  |  ");
  Serial.println ("CO2 ppm is: " + String(co2ppm));
  Serial.println ("Ethylene ppm is: " + String(EthysensorValue));
  Serial.println ("Humidity: " + String(DHThumidity));
  Serial.print(inches);
  Serial.print("in, ");
  Serial.print(cm);
  Serial.println("cm");
  Serial.println();
  */
  
/*I transfer the same data from Arduino to ROS */
  my_CO2ppm_float_data.data = co2ppm;
  CO2ppm_pub_ardu.publish(&my_CO2ppm_float_data);
  my_expansion_data.data = cm;
  expansion_pub_ardu.publish(&my_expansion_data);
  my_humidity_float_data.data = DHThumidity;
  humidity_pub_ardu.publish(&my_humidity_float_data);
  my_ethy_float_data.data = EthysensorValue;
  ethy_pub_ardu.publish(&my_ethy_float_data); 
  PEl3_temp_float_data.data = temperature0;
  PEl3temp_pub_ardu.publish(&PEl3_temp_float_data);
  PEl4_temp_float_data.data = temperature1;
  PEl4temp_pub_ardu.publish(&PEl4_temp_float_data);
  PEl1_temp_float_data.data = temperature2;
  PEl1temp_pub_ardu.publish(&PEl1_temp_float_data);
  LM35_temp_float_data.data = LM35temperatureC;
  LM35temp_pub_ardu.publish(&LM35_temp_float_data);
  DHT_temp_float_data.data = DHTtemp;
  DHTtemp_pub_ardu.publish(&DHT_temp_float_data);
  AVG_temp_float_data.data = AvgBoxTemperature;
  boxtemp_pub_ardu.publish(&AVG_temp_float_data);
  /*Diagnostic - Publish what you get from ROS
  Diag1Proc_pub_ardu.publish(&DiagnosticprocessStatePublisher);*/
  }
nh.spinOnce();
delay(500);


}

//---------------------------------------------------------------------------------------------------------------
//                                               SUPPORTING FUNCTIONS
//---------------------------------------------------------------------------------------------------------------

// Calculate the Median of the temperature (most obtained value for the sensors)
float temperatureC(int sensorPin){   
 analogRead(sensorPin);
 delay(100);
 for (int n=0;n<30;n++){
  temperatureArray.add(analogRead(sensorPin));
  delay(5);
 }
 float reading = temperatureArray.getMedian();
 // converting that reading to voltage, for 11v arduino use 11
 float voltage = (reading * 467)/1023; //measured with the voltimeter, Arduino's Volts are not exactly 5
 // print out the voltage
 //Serial.print(voltage); Serial.println(" volts");
 // now print out the temperature
 float temperatureC = voltage ;  //converting from 10 mv per degree wit 500 mV offset to degrees ((voltage - 500mV) times 100)
 return temperatureC; 
}


long microsecondsToInches(long microseconds) {
   return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds) {
   return microseconds / 29 / 2;
}
