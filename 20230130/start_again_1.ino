/////////////////////////////////////////Motor Control Setup///////////////////////////////////////////
#define IN1 2
#define IN2 3
#define ENA 5

#define IN3 4
#define IN4 7
#define ENB 6

int mode = -1;

float error_old = 0;

void motor_control_l(int direction, int speed)
{
  switch(direction)
  {
    case 1:  digitalWrite(IN1, HIGH);
             digitalWrite(IN2, LOW);
             analogWrite(ENA, speed);//0~255까지 입력
             break;

    case 0:  digitalWrite(IN1, LOW);
             digitalWrite(IN2, LOW);
             analogWrite(ENA, speed);
             break;

    case -1: digitalWrite(IN1, LOW);
             digitalWrite(IN2, HIGH);
             analogWrite(ENA, speed);
             break;
  }
}

void motor_control_r(int direction, int speed)
{
  switch(direction)
  {
    case 1:  digitalWrite(IN3, LOW);
             digitalWrite(IN4, HIGH);
             analogWrite(ENB, speed);//0~255까지 입력
             break;

    case 0:  digitalWrite(IN3, LOW);
             digitalWrite(IN4, LOW);
             analogWrite(ENB, speed);
             break;

    case -1: digitalWrite(IN3, HIGH);
             digitalWrite(IN4, LOW);
             analogWrite(ENB, speed);
             break;
  }
}
/////////////////////////////////////////Sonar Sensor Setup///////////////////////////////////////////
#include <NewPing.h>

#define SONAR_NUM 4
#define MAX_DISTANCE 150 //cm

float UltrasonicSensorData[SONAR_NUM]; //배열

NewPing sonar[SONAR_NUM] = 
{
  NewPing(8, 8, MAX_DISTANCE),
  NewPing(9, 9, MAX_DISTANCE),
  NewPing(10, 10, MAX_DISTANCE),
  NewPing(11, 11, MAX_DISTANCE)
};

void read_ultrasonic_sensor(void)
{
  UltrasonicSensorData[0] = sonar[0].ping_cm();//Front
  UltrasonicSensorData[1] = sonar[1].ping_cm();//Rear
  UltrasonicSensorData[2] = sonar[2].ping_cm();//Right
  UltrasonicSensorData[3] = sonar[3].ping_cm();//Left
}

void Sonar_Data_Display(int flag)
{
  char Sonar_data_display[40];
  if(flag == 0) return;
  else
  {
    Serial.print("F:");
    Serial.print(UltrasonicSensorData[0]);
    Serial.print("B:");
    Serial.print(UltrasonicSensorData[1]);
    Serial.print("R:");
    Serial.print(UltrasonicSensorData[2]);
    Serial.print("L:");
    Serial.println(UltrasonicSensorData[3]);
  }
}

void Robot_Mode_Define(void)
{
  mode = -1;
  read_ultrasonic_sensor();

  for(int i = 0; i < 4; i++)
  {
    if(UltrasonicSensorData[i] == 0)
    {
      UltrasonicSensorData[i] = MAX_DISTANCE;
    }
  }
  
  Sonar_Data_Display(1);
  
  //////////////////////////// mode = 0 //////////////////////////////////////
  if(  (UltrasonicSensorData[2] >= 80) && (UltrasonicSensorData[3] >= 80)  )
  {
    mode = 0;
  }
  
  //////////////////////////// mode = 1 //////////////////////////////////////
  if(  (UltrasonicSensorData[2] <= 80) && (UltrasonicSensorData[3] <= 80)  )
  {
    mode = 1;
  }
  
  //////////////////////////// mode = 2 //////////////////////////////////////왼쪽 벽이 가까운 경우
  if(  UltrasonicSensorData[3] < UltrasonicSensorData[2]  )
  {
    mode = 2;
  }
  
  //////////////////////////// mode = 3 //////////////////////////////////////오른 쪽 벽이 가까운 경우
  if(  UltrasonicSensorData[2] < UltrasonicSensorData[3]   )
  {
    mode = 3;
  }
}

void robot_stop()
{
  motor_control_r(1, 0);
  motor_control_l(1, 0);
}

void robot_90_right_turn()
{
  motor_control_r(1, 0);
  motor_control_l(1, 0);

  delay(500);
  
  motor_control_r(-1, 150);
  motor_control_l(1, 150);
  
  delay(800); // 90도를 돌 때까지 시간 조정
  
  motor_control_r(1, 0);
  motor_control_l(1, 0);
}

void robot_90_left_turn()
{
  motor_control_r(1, 0);
  motor_control_l(1, 0);

  delay(500);
  
  motor_control_r(1, 150);
  motor_control_l(-1, 150);
  
  delay(800); // 90도를 돌 때까지 시간 조정
  
  motor_control_r(1, 0);
  motor_control_l(1, 0);
}

void setup()
{
  int i;
  
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  Serial.begin(9600);
}

void loop()
{
  int i;

  Serial.println(mode);
  Robot_Mode_Define();
  
  if(mode == 1)
  {
    if(UltrasonicSensorData[0] > 40)
    {
      motor_control_l(1, 110);
      motor_control_r(1, 150);
    }
    else
    {
      robot_stop();
      delay(500);
    }
  }
  else if(mode == 3)
  {
    if(UltrasonicSensorData[0] > 40)
    {
      motor_control_l(1, 110);
      motor_control_r(1, 170);
    }
    else
    {
      robot_stop();
      delay(500);
      robot_90_left_turn();
      delay(300);
    }
  }
  else if(mode == 2)
  {
    if(UltrasonicSensorData[0] > 40)
    {
      motor_control_l(1, 130);
      motor_control_r(1, 150);
    }
    else
    {
      robot_stop();
      delay(500);
      robot_90_right_turn();
      delay(300);
    }
  }
  else
  {
    robot_stop();
  }
}
