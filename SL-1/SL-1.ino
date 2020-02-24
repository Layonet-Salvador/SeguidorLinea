#include <QTRSensors.h>
#define NUM_SENSORS   8     
#define TIMEOUT       2000  
#define EMITTER_PIN   6     
#define led1          13 
#define led2          4  
#define mot_i         7
#define mot_d         8
#define sensores      6
#define boton_1       2  
#define pin_pwm_i     9
#define pin_pwm_d     10
 
QTRSensorsRC qtrrc((unsigned char[]) {19, 18, 17, 16,15,14,11,12}
,NUM_SENSORS, TIMEOUT, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];
unsigned int position=0;
int  derivativo=0, proporcional=0, integral=0; 
int  salida_pwm=0, proporcional_pasado=0;
int velocidad=120;           
float Kp=0.18, Kd=4, Ki=0.001;  
int linea=0;            
void setup()
{
 delay(800);
 pinMode(mot_i, OUTPUT);
 pinMode(mot_d, OUTPUT);
 pinMode(led1, OUTPUT); 
 pinMode(led2, OUTPUT); 
 pinMode(boton_1, INPUT);   
 for (int i = 0; i <40; i++) 
 {                                
  digitalWrite(led1, HIGH); 
  delay(20);
  qtrrc.calibrate();  
  digitalWrite(led1, LOW);  
  delay(20);
 }
digitalWrite(led1, LOW);
delay(400); 
digitalWrite(led2,HIGH);
while(true)
{
    int x=digitalRead(boton_1);
    delay(100);
    if(x==0)
    {
        digitalWrite(led2,LOW);
        digitalWrite(led1,HIGH); 
        delay(100);
        break; 
    }
}
}       
void loop()
{
  pid(linea,velocidad,Kp,Ki,Kd);
  frenos_contorno(linea,700);
}
 void pid(int linea, int velocidad, float Kp, float Ki, float Kd)
{
  position = qtrrc.readLine(sensorValues, QTR_EMITTERS_ON, linea); 
  proporcional = (position) - 3500;
  integral=integral + proporcional_pasado; 
  derivativo = (proporcional - proporcional_pasado); 
  if (integral>1000) integral=1000; 
  if (integral<-1000) integral=-1000;
  salida_pwm =( proporcional * Kp ) + ( derivativo * Kd )+(integral*Ki);
  if (  salida_pwm > velocidad )  salida_pwm = velocidad;
  if ( salida_pwm < -velocidad )  salida_pwm = -velocidad;   
  if (salida_pwm < 0)
 {
  motores(velocidad+salida_pwm, velocidad);
 }
 if (salida_pwm >0)
 {
  motores(velocidad, velocidad-salida_pwm);
 }
 
 proporcional_pasado = proporcional;  
}
void motores(int motor_izq, int motor_der)
{
  if ( motor_izq >= 0 )
 {
  digitalWrite(mot_i,HIGH); 
  analogWrite(pin_pwm_i,255-motor_izq);
 }
 else
 {
  digitalWrite(mot_i,LOW);
  motor_izq = motor_izq*(-1);
  analogWrite(pin_pwm_i,motor_izq); 
 }
 
  if ( motor_der >= 0 )
 {
  digitalWrite(mot_d,HIGH);
  analogWrite(pin_pwm_d,255-motor_der);
 }
 else
 {
  digitalWrite(mot_d,LOW);
  motor_der= motor_der*(-1);
  analogWrite(pin_pwm_d,motor_der);
 }
}
 
void frenos_contorno(int tipo,int flanco_comparacion)
{
   
if(tipo==0)
{
  if(position<=50)
 {
  motores(-80,90);
  while(true)  
  {
   qtrrc.read(sensorValues);
   if( sensorValues[0]>flanco_comparacion || sensorValues[1]>flanco_comparacion ) 
   //asegurar que esta en linea
   {
    break;
   } 
  }
 }
 
 if (position>=6550)
 { 
  motores(90,-80);
  while(true)
  {
   qtrrc.read(sensorValues);
   if(sensorValues[7]>flanco_comparacion || sensorValues[6]>flanco_comparacion )
   {
    break;
   }  
  }
 }
}
 
if(tipo==1)
{
 if(position<=50)
 {
  motores(-80,90);
  while(true)  
  {
   qtrrc.read(sensorValues);
   if(sensorValues[0]<flanco_comparacion || sensorValues[1]<flanco_comparacion )
   {
    break;
   }
  }
 }
 
 if(position>=6550) 
 { 
  motores(90,-80);
  while(true)
  {
   qtrrc.read(sensorValues);
   if(sensorValues[7]<flanco_comparacion || sensorValues[6]<flanco_comparacion)
   {
    break;
   }  
  }
 }
}
}
