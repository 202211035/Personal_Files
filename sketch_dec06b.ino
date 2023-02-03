#include <NewPing.h>
#define SONAR_NUM 3
#define MAX_DIStANCE 200

#define MOTOR_R_PWM 6
#define MOTOR_R_IN1 7
#define MOTOR_R_IN2 4

#define MOTOR_L_IN3 3
#define MOTOR_L_IN4 2
#define MOTOR_L_PWM 5

//NewPing sonar[SONAR_NUM]= {
//  NewPing(4,5, MAX_DIStANCE),
//  NewPing(6,7, MAX_DIStANCE),
//  NewPing(8,9, MAX_DIStANCE)
//  };


void go_forward(){
  digitalWrite(MOTOR_R_IN1,HIGH);
  digitalWrite(MOTOR_R_IN2,LOW);
  analogWrite(MOTOR_R_PWM, 80);

  digitalWrite(MOTOR_L_IN3,HIGH);
  digitalWrite(MOTOR_L_IN4,LOW);
  analogWrite(MOTOR_L_PWM, 80);
}

int line_sensor[5] = {0,0,0,0,0};
  
int read_line_sensor()
  {
    int line_index;
    int i;
    int sum =0;
    for (i-0; i<5;i++)
    {
      line_sensor[i] = 1-digitalRead(A0+i); //A0 아날로그 신호
      sum+=line_sensor[i];
      Serial.print(line_sensor[i]);
      Serial.print("  ");
    }
    Serial.print("");

  if(sum==1) 
    {
      if(line_sensor[0]==1) line_index = -4;
      if(line_sensor[1]==1) line_index = -2;
      if(line_sensor[2]==1) line_index = 0;
      if(line_sensor[3]==1) line_index = 2;
      if(line_sensor[4]==1) line_index = 4;
  }
  if(sum==2)
    {
      if((line_sensor[0]==1)  && (line_sensor[1]==1))  line_index = -3;
      if((line_sensor[1]==1)  && (line_sensor[2]==1))  line_index = -1;
      if((line_sensor[2]==1)  && (line_sensor[3]==1))  line_index = 1;
      if((line_sensor[3]==1)  && (line_sensor[4]==1))  line_index = 3;
  }
  if(sum==5)
  { 
    line_index = -10;
  }
  Serial.print(line_index);
  Serial.println("");
  return line_index;
  }
  
int motor_Control(int speed_1,int speed_2, int dir_1)
{
  if (dir_1 == 1) //전진
{  
  digitalWrite(MOTOR_L_IN3,HIGH);
  digitalWrite(MOTOR_L_IN4,LOW);
  digitalWrite(MOTOR_R_IN1,HIGH);
  digitalWrite(MOTOR_R_IN2,LOW);
  analogWrite(MOTOR_L_PWM,speed_1);
  analogWrite(MOTOR_R_PWM,speed_2); 
}
  else if (dir_1 == -1) //후진
{ 
  digitalWrite(MOTOR_L_IN3,LOW);
  digitalWrite(MOTOR_L_IN4,HIGH);
  digitalWrite(MOTOR_R_IN1,LOW);
  digitalWrite(MOTOR_R_IN2,HIGH);
  analogWrite(MOTOR_L_PWM,speed_1);
  analogWrite(MOTOR_R_PWM,speed_2);
  
}
 else       //정지
{ 
  digitalWrite(MOTOR_L_IN3,LOW);
  digitalWrite(MOTOR_L_IN4,LOW);
  digitalWrite(MOTOR_R_IN1,LOW);
  digitalWrite(MOTOR_R_IN2,LOW);
  analogWrite(MOTOR_L_PWM,speed_1);
  analogWrite(MOTOR_R_PWM,speed_2);
 }
}

  void setup(){
    Serial.begin(9600);
//    pinMode(MOTOR_R_ENA,OUTPUT);
//    pinMode(MOTOR_R_ENB,OUTPUT);
//    pinMode(MOTOR_L_ENA,OUTPUT);
//    pinMode(MOTOR_L_ENB,OUTPUT);
//    pinMode(MOTOR_R_PWM,OUTPUT);
//   pinMode(MOTOR_L_PWM,OUTPUT);
  }

  void loop()
  {
    
    int index;  
    index = read_line_sensor();
  
switch(index)
{
      case -10 : //정지
      motor_Control(0,0,1);
      break;
      
      case 0 : //전진
      motor_Control(100,100,1);
      break;
//좌회전
      case 1 : 
      motor_Control(20,100,1);
      break;

      case 2 : 
      motor_Control(50,100,1);
      break;

      case 3 : 
      motor_Control(30,100,1);
      break;

      case 4 : 
      motor_Control(0,50,1);
      break;
//우회전
      case -1 : 
      motor_Control(100,70,1);
      break;

      case -2 : 
      motor_Control(100,50,1);
      break;

      case -3 : 
      motor_Control(100,30,1);
      break;

      case -4 : 
      motor_Control(50,0,1);
      break;
}
  }
