void back2zero();
void motor_stop();


#include <Servo.h>
#include <SPI.h>
#include <SD.h>
File myFile;

Servo myservo;

//PID parameters
float Kp=2.5;
float Ki=0.6;
float Kd=0.01;

//PID parameters
float Kpfree=1;
float Kifree=0;
float Kdfree=0;

//PID for feeding
float fKp=1.1;
float fKi=.00075;
float fKd=0.015;
int start_mode = 0;
//Encoder pot pins
#define inputCLK 31
#define inputDT 30

#define lms_1 20
#define hall 30

#define servo_pin 45

#define feed_button1 40
#define feed_button2 41
#define start_button 21


//rotation axis pins
#define rdirPin 8
#define rstepPin 9

#define potPin A0

//bending axis pins
int motor_pin1 = 6;
int motor_pin2 = 5;
int encoder1=3;
int encoder2=2;
volatile int count = 0;

//feeding axis pins
int feed_motor_pin1 = 10;
int feed_motor_pin2 = 11;

int feed_encoder1=18;
int feed_encoder2=19;
volatile long int feed_count = 0;

String buffer;


int encoder_count = 6400;
int value1 = 0;
int value2 = 0;
int mode = 0;
int initial_bend;
int counter_feed_initial = 0; 
int currentStateCLK;
int previousStateCLK; 
float motor_angle = 0;
float f_motor_angle = 0;
int mode_bend = 0;

int val = digitalRead(hall);
void setup() {
  // put your setup code here, to run once:
  //define the pin modes
  pinMode(rstepPin, OUTPUT);
  pinMode(rdirPin, OUTPUT);
  pinMode(motor_pin1, OUTPUT);
  pinMode(motor_pin2, OUTPUT);
  pinMode(encoder1, INPUT);
  pinMode(encoder2, INPUT);
  pinMode(feed_motor_pin1, OUTPUT);
  pinMode(feed_motor_pin2, OUTPUT);
  pinMode(feed_encoder1, INPUT);
  pinMode(feed_encoder2, INPUT);
  pinMode(hall, INPUT);
  pinMode(lms_1, INPUT);
  pinMode (inputCLK,INPUT);
  pinMode (inputDT,INPUT);
  pinMode (feed_button1,INPUT);
  pinMode (feed_button2,INPUT);
  pinMode (start_button,INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder1), count_1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2), count_2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(feed_encoder1), feed_count_1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(feed_encoder2), feed_count_2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(start_button), start_count, RISING);
  attachInterrupt(digitalPinToInterrupt(lms_1), Limit, FALLING);
  previousStateCLK = digitalRead(inputCLK);
  myservo.attach(servo_pin);
  Serial.begin(9600);


  //delay(1000);
  //read_SD();
  //Serial.println("SA");
  //start();
  //Serial.println("çıktım");
  initial_bending();
  //bend_tube(165);
  //back2zero();
  
}

//The codes in the main loop are for trial purposes

void loop() {

  //rotate_a(-90);
  //rotate_a(-90);
  start();
  
}

void start_count(){
  if(digitalRead(start_button) == 1) start_mode++;
  delay(200);
  //Serial.println(start_mode);
}

void start(){
  delay(100);
  Serial.println(start_mode%2);
  if(start_mode%2 == 0){
    feed_initial();
    //Serial.println("feed_mode");
  }
  else{
    read_SD();
    delay(1000);
    start_mode++;
    //Serial.println("SD");
  }

}
void Limit(){
  
  while(digitalRead(lms_1) == 0){
    analogWrite(feed_motor_pin1, 0);
    analogWrite(feed_motor_pin2, 0);
    delay(100);
  }
  
}

void initial_bending(){
  val = digitalRead(hall);
  //Serial.println(val);
  while(val == HIGH && mode_bend == 0){
    val = digitalRead(hall);
    //Serial.println(val);
    analogWrite(motor_pin1, 0);
    analogWrite(motor_pin2, 50);
    delay(10);
    
  }
  mode_bend = 1;
  motor_stop();
  
}
int last_count=0;
void feed_initial(){
  //Serial.println(digitalRead(feed_button1));
  while(digitalRead(feed_button1) == 1){
    analogWrite(feed_motor_pin1, 0);
    analogWrite(feed_motor_pin2, 255);
    delay(100);
    
  }
  while(digitalRead(feed_button2) == 1){
    analogWrite(feed_motor_pin1, 255);
    analogWrite(feed_motor_pin2, 0);
    delay(100);
  }
  while(digitalRead(feed_button1) == 0 && digitalRead(feed_button2) == 0){
    delay(100);
    analogWrite(feed_motor_pin1, 0);
    analogWrite(feed_motor_pin2, 0);
    
  }
  

}

//bending axis encoder countes
void count_1(){
  if(digitalRead(encoder1)==digitalRead(encoder2)){
    count++;
  }
  else{
    count--;
  }
  //Serial.println(count);
}

void count_2(){
  if(digitalRead(encoder1)==digitalRead(encoder2)){
    count--;
  }
  else{
    count++;
  }
  //Serial.println(count);
}

//feeding axis encoder countes
void feed_count_1(){
  if(digitalRead(feed_encoder1)==digitalRead(feed_encoder2)){
    feed_count++;
  }
  else{
    feed_count--;
  }
}

void feed_count_2(){
  if(digitalRead(feed_encoder1)==digitalRead(feed_encoder2)){
    feed_count--;
  }
  else{
    feed_count++;
  }
}

void bend_tube(float angle){
  angle = angle*1.141+7.2189;
  /*if (angle<7) angle = angle+3.5;
  else if(angle<20) angle = angle+2.5;
  else if(angle<30) angle = angle;
  else if(angle<40) angle = angle;
  else if(angle<50) angle = angle;
  else if(angle<60) angle = angle;
  else angle = angle;*/
  //Serial.print("given:  ");
  //Serial.println(angle);
  int i_motor_voltage = 60;
  angle = angle + 14.06;
  //Serial.println(angle);
  if(angle<20){
    Kp=0.15;
    Ki=0.4;
    Kd=0.05;
    i_motor_voltage = 0;
   // angle= angle+5;
    
  }
  else if(angle>=20 && angle<30){
    Kp=0.03;
    Ki=0.2;
    Kd=0;
    i_motor_voltage = 0;
    //angle= (angle-14.06)*1+5+14.06;
  }
  else if(angle>=30 && angle<40){
    Kp=0.01 ;
    Ki=0.15;
    Kd=0.05;
    i_motor_voltage = 0;
    //angle= angle*1.2;
   // angle = 1.2857*(angle-14.06)-2.1429+14.06;
    
  }
  else if (angle>=40 && angle<50){
    Kp=0.0007;
    Ki=0.08;
    Kd=0.001;
    i_motor_voltage = 0;
    //angle = 0.5*(angle-17.65)+17.5+17.5;
  }
  else if(angle>=50 && angle<60){
    Kp=0.0007;
    Ki=0.05;
    Kd=0.0001;
    i_motor_voltage = 0;
  }
  else{
    Kp=0.005;
    Ki=0.025;
    Kd=0.001;
    i_motor_voltage = 0;
  }
  
  motor_angle = count*360.0/(69/34)/encoder_count;
  float error = abs(angle-motor_angle);
  float motor_voltage;
  float last_error = error;
  float sum = 0;
  if(angle<0){
  while(angle<motor_angle){
    
    motor_angle=count*360.0/(69/34)/encoder_count; 
    error = abs(angle-motor_angle);
    analogWrite(motor_pin1, 0);
    //if(sum>7000) sum = sum/2;
    motor_voltage = i_motor_voltage+Kp*error+Kd*(last_error-error)+Ki*sum;
    if(motor_voltage>255) motor_voltage = 255;
    analogWrite(motor_pin2, motor_voltage); 
    delay(100); 
    last_error = error;
    sum = sum+error;
  }
  }
  else{
    while(angle>motor_angle){
    motor_angle=count*180.0/encoder_count;
    error = abs(angle-motor_angle);
    //if(sum>7000) sum = sum/2;
    motor_voltage = i_motor_voltage+Kp*error+Kd*(last_error-error)+Ki*sum;
    if(motor_voltage>255) motor_voltage = 255;
    analogWrite(motor_pin1, motor_voltage);
    analogWrite(motor_pin2, 0); 
    delay(100); 
    last_error = error;
    sum = sum+error;
  }
}
motor_stop();
//Serial.println(motor_angle-14.06);
}

void bend_free(float angle){
  motor_angle = count*180.0/encoder_count;
  float error = abs(angle-motor_angle);
  float motor_voltage_free;
  float last_error = error;
  float sum = 0;
  if(angle<motor_angle){
  while(angle<motor_angle){
    motor_angle=count*180.0/encoder_count;
    error = abs(angle-motor_angle);
    analogWrite(motor_pin1, 0);
    motor_voltage_free = 60+Kpfree*error+Kdfree*(last_error-error)+Kifree*sum;
    if(motor_voltage_free>255) motor_voltage_free = 255;
    analogWrite(motor_pin2, motor_voltage_free); 
    delay(100);
    last_error = error; 
    sum = sum+error;
  }
  }
  else{
    while(angle>motor_angle){
      motor_angle=count*180.0/encoder_count;
      error = abs(angle-motor_angle);
      motor_voltage_free = 60+Kpfree*error+Kdfree*(last_error-error)+Kifree*sum;
      if(motor_voltage_free>255) motor_voltage_free = 255;
      analogWrite(motor_pin1, motor_voltage_free);
      analogWrite(motor_pin2, 0); 
      delay(100); 
      last_error = error;
      sum = sum+error;
  }
}
motor_stop();
//Serial.println(motor_angle);
}


void feed(float f){
  int feed_mode = 0;
    //Serial.println("feed");
  f=-f;
  f_motor_angle=feed_count*360.0/encoder_count;
  float desired_angle = f*360.0/4;
  float error = desired_angle-f_motor_angle;
  float last_error = error;
  float fmotor_voltage;
  float sum = 0;
  
  while(abs(error)>20 && feed_mode == 0)
  {
    if(sum>90) sum = 0;
    error = desired_angle-f_motor_angle;
    f_motor_angle=feed_count*360.0/encoder_count; 
    fmotor_voltage = fKp*abs(error)+fKd*abs(last_error-error)+fKi*sum;
    if (fmotor_voltage>255) fmotor_voltage = 255;
    if(error<10){
      //Serial.println(-1*f_motor_angle*4.0/360);
      //Serial.println(fmotor_voltage);
      analogWrite(feed_motor_pin1, 0);
      analogWrite(feed_motor_pin2, fmotor_voltage);
    }
    else{
      //Serial.println(-1*f_motor_angle*4.0/360);
      //Serial.println(fmotor_voltage);
      analogWrite(feed_motor_pin2, 0);
      analogWrite(feed_motor_pin1, fmotor_voltage);
    }
    last_error = error;
    sum = sum + abs(error);
    delay(100);
    
  
  }


feed_motor_stop();
feed_count = 0;
feed_mode = 1;
//Serial.println(-1*f_motor_angle*4.0/360);
}


void rotate_a(float r){
 int step = r/1.8*16;
 //Serial.println(step);
  if(step>0){
    digitalWrite(rdirPin, HIGH);
    for (int i = 0; i < step; i++) {
      digitalWrite(rstepPin, HIGH);
      delayMicroseconds(500);
      digitalWrite(rstepPin, LOW);
      delayMicroseconds(500);
  }
  delay(100);
  }
  else{
    digitalWrite(rdirPin, LOW);
    for (int i = 0; i > step; i--) {
      digitalWrite(rstepPin, HIGH);
      delayMicroseconds(500);
      digitalWrite(rstepPin, LOW);
      delayMicroseconds(500);
  }
  delay(100);
}
  delay(500);
  //Serial.println(r);
}

void back2zero(){
 mode_bend = 0;
 initial_bending();
}

void motor_stop(){
  digitalWrite(motor_pin1, 0);
  digitalWrite(motor_pin2, 0);
  delay(500);
}

void feed_motor_stop(){
  digitalWrite(feed_motor_pin1, 0);
  digitalWrite(feed_motor_pin2, 0);
  delay(500);
}

void read_SD(){

  
  while (!Serial) {
  ; // wait for serial port to connect. Needed for native USB port only
  }
  
  Serial.print("Initializing SD card...");
  
  if (!SD.begin(53)) {
  Serial.println("initialization failed!");
  while (1);
  }

  
  Serial.println("initialization done.");
  // open the file for reading:

  myFile = SD.open("gcode.txt");
  
  while (myFile.available()) {

      String strs[10];
      int StringCount = 0;
  
    buffer = myFile.readStringUntil('\n');
    Serial.println(buffer); //Printing for debugging purpose
    while (buffer.length() > 0)
    {
      int index = buffer.indexOf(' ');
      if (index == -1) // No space found
      {
        strs[StringCount++] = buffer;
        break;
      }
      else
      {
        strs[StringCount++] = buffer.substring(0, index);
        buffer = buffer.substring(index+1);
      }
    }

    float var = strs[1].toFloat();
    if (strs[0] == "G01Y"){
      
      //Serial.println("feed");
      feed(var);
    }
    else if (strs[0] == "G01C"){
      //Serial.println("bend");
      //var = var*1.141+7.2189;
      //bend_tube(1.155*var);
      bend_tube(var);
      back2zero();
    }
    else if (strs[0] == "G01B"){
      //Serial.print(var);
      //Serial.println("rotate");
      
      rotate_a(1*var);
    }
    else{

      Serial.println("command not found");
    }
    delay(100);

}
  // close the file:
  myFile.close();
}
