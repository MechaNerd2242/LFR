#include <LiquidCrystal.h> //library for LCD
// initialize the library with the numbers of the interface pins
 LiquidCrystal lcd(8, 9, 10, 11, 12, 13);
// Voltage Sensor
int analogInput = A1;     //voltage sensor
float vout = 0.0;
float vin = 0.0;
float R1 = 30000.0; //30k
float R2 = 7500.0; //7500 ohm resistor, I tweaked this
int value = 0;
int sensor = A2; 
unsigned long start_time = 0;
unsigned long end_time = 0;
int steps=0;
float steps_old=0;
float temp=0;
float rps=0;
//Measuring Current Using ACS712
const int currentPin = A0; // current sensor is attached
int sensitivity = 66;  // 66-185 mv/A
int adcValue= 0; // 
int offsetVoltage = 2500; // 0.25 mv 0v ki value ko remove karne k liye 
double adcVoltage = 0;
double currentValue = 0;
//variables for battery level indicator
int f=2, e=3,d=4, c=5, b=6,a=7;
void setup() {
  //baud rate
  Serial.begin(9600);//baud rate at which arduino communicates with Laptop/PC
  // set up the LCD's number of columns and rows
  lcd.begin(16, 4);  //LCD order
pinMode(sensor,INPUT_PULLUP);
//  lcd.setCursor(0,0);
//  lcd.print(" STEPS - 0");
//  lcd.setCursor(0,1);
//  lcd.print(" RPS   - 0.00");
pinMode(a,OUTPUT);
pinMode(b,OUTPUT);
pinMode(c,OUTPUT);
pinMode(d,OUTPUT);
pinMode(e,OUTPUT);
pinMode(f,OUTPUT);
digitalWrite(f,HIGH);
delay(500);
digitalWrite(e,HIGH);
delay(500);
digitalWrite(d,HIGH);
delay(500);
digitalWrite(c,HIGH);
delay(500);
digitalWrite(b,HIGH);
delay(500);
digitalWrite(a,HIGH);
delay(500);
digitalWrite(a,LOW);
delay(500);
digitalWrite(b,LOW);
delay(500);
digitalWrite(c,LOW);
delay(500);
digitalWrite(d,LOW);
delay(500);
digitalWrite(e,LOW);
delay(500);
digitalWrite(f,LOW);
delay(500);
  lcd.setCursor(0,0);//Setting cursor on LCD
  lcd.print("CHEAP THRILLS");//Prints on the LCD
  delay(1000);
  lcd.setCursor(1,1);
  lcd.print("FAIZAAN-221708");
  lcd.setCursor(1,2);
  lcd.print("NOUMAN-221708");
  lcd.setCursor(1,3);
  delay(3000);//time delay for 3 sec
  lcd.clear();//clearing the LCD display
  lcd.display();//Turning on the display again
  lcd.setCursor(1,0);//setting LCD cursor
  lcd.print("Values from");//prints on LCD
  lcd.setCursor(1,1);
  lcd.print("Current Sensor");
  lcd.setCursor(2,2);
  lcd.print("ACS712 and");
  lcd.setCursor(1,4);
  lcd.print("Voltage sensor");
  lcd.clear();
  delay(1000);//delay for 1 sec}
void loop() //run the source code repeatedly
{start_time=millis();
 end_time=start_time+1000;
 while(millis()<end_time)
 {if(digitalRead(sensor))
   { steps=steps+1; 
    while(digitalRead(sensor));}
   lcd.setCursor(9,2);
   lcd.print(steps);
   lcd.print("   ");}
// voltage
   value = analogRead(analogInput);
   vout = (value * 5.0) / 1024.0;
   vin = vout / (R2/(R1+R2));
if(vin>11.88) 
{ digitalWrite(a,HIGH);
   digitalWrite(b,HIGH);
    digitalWrite(c,HIGH);
     digitalWrite(d,HIGH);
      digitalWrite(e,HIGH);
       digitalWrite(f,HIGH);}
if (vin<=11.46 && vin>11.28)
{ digitalWrite(a,LOW);
   digitalWrite(b,HIGH);
    digitalWrite(c,HIGH);
     digitalWrite(d,HIGH);
      digitalWrite(e,HIGH);
       digitalWrite(f,HIGH); }
if (vin<=11.12 && vin>10.98)
{digitalWrite(a,LOW);
   digitalWrite(b,LOW);
    digitalWrite(c,HIGH);
     digitalWrite(d,HIGH);
      digitalWrite(e,HIGH);
       digitalWrite(f,HIGH);}
if (vin<=10.90 && vin>10.79)
{ digitalWrite(a,LOW);
   digitalWrite(b,LOW);
    digitalWrite(c,LOW);
     digitalWrite(d,HIGH);
      digitalWrite(e,HIGH);
       digitalWrite(f,HIGH);}
if (vin<=10.60 && vin>10.53)
{digitalWrite(a,LOW);
   digitalWrite(b,LOW);
    digitalWrite(c,LOW);
     digitalWrite(d,LOW);
      digitalWrite(e,HIGH);
       digitalWrite(f,HIGH);}
if  (vin<=10.53)
{ digitalWrite(a,LOW);
   digitalWrite(b,LOW);
    digitalWrite(c,LOW);
     digitalWrite(d,LOW);
      digitalWrite(e,LOW);
       digitalWrite(f,HIGH);}
// current
  adcValue = analogRead(currentPin);
  adcVoltage = (adcValue / 1024.0) * 5000;
  currentValue = ((adcVoltage - offsetVoltage) / sensitivity);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Current =       ");
  lcd.setCursor(10,0);
  lcd.print(currentValue,2); // upto two decimal points
  lcd.setCursor(14,0);
  lcd.print("A");
   lcd.setCursor(0,1);  
 lcd.print("Voltage=");
 lcd.setCursor(8,1);  
 lcd.print(vin,2);
 lcd.setCursor(12,1);  
 lcd.print("V");
lcd.setCursor(0,2);
  lcd.print(" STEPS = 0");
  lcd.setCursor(0,3);
  lcd.print(" RPS   = 0.00");
      temp=steps-steps_old;
    steps_old=steps;
    rps=(temp/20);
    lcd.setCursor(9,3);
    lcd.print(rps);
    lcd.print("   ");}
#include <SPI.h>
#include <SD.h>
#include <Servo.h> //Servo motor library. This is standard library
#include <NewPing.h> //Ultrasonic sensor function library. 
File myFile;
//sensor pins 
#define trig_pin A4 //analog output 1
#define echo_pin A5 //analog input 2
#define maximum_distance 200
boolean goesForward = false;
int distance = 100;
NewPing sonar(trig_pin, echo_pin, maximum_distance); //sensor function
Servo servo_motor; //our servo name
void setup() 
{
Serial.begin(9600);
SD.begin(10);
  Serial.println("initialization done.");
 servo_motor.attach(8); //our servo pin
 servo_motor.write(115);
 delay(100);
 distance = readPing();
 pinMode(2, INPUT);//left sensor 1 
 pinMode(4, INPUT); //left sensor 2
 pinMode(5, INPUT); //middle sensor3
 pinMode(6, INPUT); //right sensor 4
 pinMode(7, INPUT); //right sensor5
 pinMode(3, OUTPUT);  // pwm
  pinMode(8, OUTPUT);
 pinMode(A0, OUTPUT); //LEFT MOTOR.
 pinMode(A1, OUTPUT);
 pinMode(A2, OUTPUT); //RIGHT MOTOR.
 pinMode(A3, OUTPUT);} 
 // put your setup code here, to run once
void loop(){ 
 int distanceRight = 0;
 int distanceLeft = 0;
 delay(50);
 if (distance <= 25){
 distanceRight = lookRight();
 delay(0);
 distanceLeft = lookLeft();
 delay(0);}
 distance = readPing();
if (distance<=25){
STOP();}
else{
 int a = digitalRead(2);
 int b = digitalRead(4);
int c = digitalRead(5);
 int d = digitalRead(6); 
 int e = digitalRead(7);
if ((a == LOW && b == LOW && c == HIGH && d==LOW && e== LOW)||(a == LOW && b == HIGH && c == HIGH && 
d==HIGH && e== LOW)||(a == HIGH && b == HIGH && c == HIGH && d==HIGH && e== HIGH))//logic for FORWARD
 { analogWrite(A0, 0);
 analogWrite(A1, 255);
 analogWrite(A2, 0);
 analogWrite(A3, 255);
 analogWrite(3, 100);
f(); }
if ((a == LOW && b == HIGH && c == LOW && d==LOW && e==LOW)||(a==HIGH && b== HIGH && c==LOW &&d== 
LOW &&e==LOW)||(a==HIGH && b== HIGH && c==HIGH &&d== LOW &&e==LOW)||(a==HIGH && b== LOW && 
c==LOW &&d== LOW &&e==LOW)||(a==LOW && b== HIGH && c==HIGH &&d== LOW &&e==LOW)||(a==HIGH && 
b== HIGH && c==HIGH && d==HIGH && e==LOW)) //logic for LEFT TURN
 {
 analogWrite(A0, 0);
 analogWrite(A1, 255);
 analogWrite(A2, 0);
 analogWrite(A3, 0);
 analogWrite(3, 100);
l(); } 
if ((a == LOW && b == LOW && c == LOW && d==HIGH && e==LOW)||(a==LOW && b== LOW && c==LOW &&d== 
HIGH &&e==HIGH)||(a==LOW && b== LOW && c==HIGH &&d== HIGH &&e==HIGH)||(a==LOW && b== LOW && 
c==LOW &&d== LOW &&e==HIGH)||(a==LOW && b== HIGH && c==HIGH &&d== HIGH &&e==HIGH)||(a==LOW && 
b== LOW && c==HIGH &&d== HIGH &&e==LOW) ) //logic for RIGHT TURN
 {
 analogWrite(A0, 0);
 analogWrite(A1, 0);
 analogWrite(A2, 0);
 analogWrite(A3, 255);
 analogWrite(3, 100);
 r(); }
if (a == LOW && b == LOW && c == LOW && d==LOW && e== LOW) //logic for  BACK {
 analogWrite(A0, 0);
 analogWrite(A1, 255);
 analogWrite(A2, 255);
 analogWrite(A3, 0); 
 analogWrite(3, 100);
bs(); }}}
int lookRight(){
 servo_motor.write(50);
 delay(100);
 int distance = readPing();
 delay(100);
 servo_motor.write(115);
 return distance;}
int lookLeft(){
 servo_motor.write(170);
 delay(100);
 int distance = readPing();
 delay(100);
 servo_motor.write(115);
 return distance;
 delay(100);}
int readPing(){
 delay(70);
 int cm = sonar.ping_cm();
 if (cm==0)
 { cm=250;  }
 return cm; }
void STOP(){
 analogWrite(A0, 0);
 analogWrite(A1, 0);
 analogWrite(A2, 0);
 analogWrite(A3, 0);
 analogWrite(3, 100);
 obs();
 }
 void f(){
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile = SD.open("test.txt", FILE_WRITE);
  // re-open the file for reading:
  myFile = SD.open("test.txt");
  Serial.println("forward.txt:"); 
    Serial.println("forward.txt:");
    myFile.close(); }
void r(){
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile = SD.open("test.txt", FILE_WRITE);
  // re-open the file for reading:
  myFile = SD.open("test.txt");
  Serial.println("right.txt:"); 
    Serial.println("right.txt:");
    myFile.close(); }
  void l(){
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile = SD.open("test.txt", FILE_WRITE);
  // re-open the file for reading:
  myFile = SD.open("test.txt");
  Serial.println("left.txt:"); 
    Serial.println("left.txt:");
    myFile.close();  }
  void bs(){
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile = SD.open("test.txt", FILE_WRITE)
  // re-open the file for reading:
  myFile = SD.open("test.txt");
  Serial.println("back.txt:"); 
    Serial.println("back.txt:");
    myFile.close(); }
 void obs(){
  // open the file. note that only one file can be open at a time,
  myFile = SD.open("test.txt", FILE_WRITE)
  // re-open the file for reading:
  myFile = SD.open("test.txt");
  Serial.println("Obstacle.txt:"); 
    Serial.println("Obstacle.txt:");  
    myFile.close();
  }
