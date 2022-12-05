/*duty cycle in arduino is represented by a 8 bit value: 0 - 255
0 --> 0%
127 -->50%
255 --> 100%
*/

int npnpin = 4;/*assigning Arduino pins for the giving signal to MOSFET (it was 6 before, but I swapped to make room for the PEl controller*/
float tempc1; /*variable to store temperature in degree Celsius - Fan1*/
float vout1; /*temporary variable to hold sensor reading - Fan1*/
float tempc2; /*variable to store temperature in degree Celsius - Fan2*/
float vout2; /*temporary variable to hold sensor reading - Fan2*/
float tempc3; /*variable to store temperature in degree Celsius - Fan3*/
float vout3; /*temporary variable to hold sensor reading - Fan3*/
float tempc4; /*variable to store temperature in degree Celsius - Fan4*/
float vout4; /*temporary variable to hold sensor reading - Fan4*/
float tempAverage; /*variable to store temperature in degree Celsius - Fan1*/
float tempMax = 0; /*variable to store temperature in degree Celsius - Fan1*/
unsigned long interruptcounter = 0; // loop counter 
unsigned long previousMillis = 0; // to store previous time 
const long interruptinterval = 3000;

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

void setup() {
pinMode(npnpin,OUTPUT);/* assigning the transistor  pin as an output of Arduino*/

/*safety speed reset of the motor*/
analogWrite(npnpin,0);

setPWMfrequency(0x02);// timer 2 , 3.92KHz
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
Serial.begin(9600);
}


void loop() {
float val = 1023;
float duty = map(val,0,1023,0,255);
analogWrite(npnpin, duty); //here's how to generate PWM signal from Digital arduino pin

/*In this block I use an interrupt to establish the frequency of all sensors' measurements.
I do not want to measure too often as this process does not need constant monitoring */

unsigned long currentMillis= millis(); /* and compare with previous time taken */
if (currentMillis - previousMillis >= interruptinterval) {
  previousMillis = currentMillis; // save current time as prev
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
  }

/* In this block I set up how the peltier will work, through the motor control */
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
}
