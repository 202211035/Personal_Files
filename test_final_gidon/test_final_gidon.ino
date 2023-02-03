#include <NewPing.h>
#define SONAR_NUM 4
#define MAX_DISTANCE 150 //cm
/////////////////////////////////////////Motor Control Setup///////////////////////////////////////////
#define IN1 2
#define IN2 3
#define ENA 5

#define IN3 4
#define IN4 7
#define ENB 6

int mode = -1;

float error_old = 0;

void motor_control_l(int direction, int speed) //왼쪽 모터 제어
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

void motor_control_r(int direction, int speed) //오른쪽 모터 제어
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

float UltrasonicSensorData[SONAR_NUM]; //배열

NewPing sonar[SONAR_NUM] = 
{
  NewPing(8, 8, MAX_DISTANCE),
  NewPing(9, 9, MAX_DISTANCE),
  NewPing(10, 10, MAX_DISTANCE),
  NewPing(11, 11, MAX_DISTANCE)
};

void read_ultrasonic_sensor(void) //초음파로 거리 측정(앞, 뒤, 오른쪽, 왼쪽
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
/////////////////////////////////////////Motor Control Setup///////////////////////////////////////////
#define DEBUG 1

/////////////////////////////////////////Line Sensor Setup///////////////////////////////////////////////////////
#define LINE_DETECT_WHITE 1

int LineSensorPinData[5] = {0, };
int LineType = -1;
/////////////////////////////////////////Line Sensor Setup///////////////////////////////////////////////////////
int sum = 0;

int read_digital_line_sensor(void) //색상인식센서
{
  int i;
  sum = 0;
  int data[5] = {0, };

  LineSensorPinData[0] = analogRead(A0);
  LineSensorPinData[1] = analogRead(A1);
  LineSensorPinData[2] = analogRead(A2);
  LineSensorPinData[3] = analogRead(A3);
  LineSensorPinData[4] = analogRead(A4);

  for(i = 0; i < 5; i++)
  {
    if(LineSensorPinData[i] > 900) LineSensorPinData[i] = 1; //라인이 있을경우 1, 라인이 없을 경우 0
    else LineSensorPinData[i] = 0;
  }

  for(i = 0; i < 5; i++)
  {
    if(LINE_DETECT_WHITE == 0)
    {
      LineSensorPinData[i] = 1 - LineSensorPinData[i];
    }
    sum += LineSensorPinData[i];
  }

  if(sum == 5)
  {
    return sum;
  }
  else if(sum == 2)
  {
    if(  (LineSensorPinData[3] == 1) && (LineSensorPinData[4] == 1)  ) return 3;
    if(  (LineSensorPinData[2] == 1) && (LineSensorPinData[3] == 1)  ) return 1;
    if(  (LineSensorPinData[1] == 1) && (LineSensorPinData[2] == 1)  ) return -1;
    if(  (LineSensorPinData[0] == 1) && (LineSensorPinData[1] == 1)  ) return -3;
  }
  else if(sum == 1)
  {
    if(LineSensorPinData[0] == 1) return -4;
    if(LineSensorPinData[1] == 1) return -2;
    if(LineSensorPinData[2] == 1) return 0;
    if(LineSensorPinData[3] == 1) return 2;
    if(LineSensorPinData[4] == 1) return 4;
  }
  else if(sum == 3)
  {
    return -10;
  }
  else
  {
    return -5;
  }
  Serial.println(sum);
}

void send_serial_data(void)
{
  for(int i = 0; i < 5; i++)
  {
    Serial.print(LineSensorPinData[i]); //검은색 15~20 노란색 550이상
    Serial.print("  ");
  }
  
  Serial.println(LineType);
  Serial.print("  ");

  for(int i = 0; i < 4; i++)
  {
    Serial.print(UltrasonicSensorData[i]);
    Serial.print("  ");
  }
  Serial.println("  ");
}

void robot_line_trace(void) //라인 따라가기
{
  switch(LineType)
  {
    case -4:
            motor_control_r(1, 100); //속도 제어할 것
            motor_control_l(-1, 100);
            delay(300);
            break;
    
    case -3:
            motor_control_r(1, 100);
            motor_control_l(-1, 100);
            delay(100);
            break;
    
    case -2:
            motor_control_r(1, 100);
            motor_control_l(-1, 100);
            break;
    
    case -1:
            motor_control_r(1, 100);
            motor_control_l(-1, 100);
            break;

    case 0:
            motor_control_r(1, 100);
            motor_control_l(1, 100);
            break;

    case 1:
            motor_control_r(-1, 100);
            motor_control_l(1, 100);
            break;
    
    case 2:
            motor_control_r(-1, 100);
            motor_control_l(1, 100);
            break;
    
    case 3:
            motor_control_r(-1, 100);
            motor_control_l(1, 100);
            delay(200);
            break;
    
    case 4:
            motor_control_r(-1, 100);
            motor_control_l(1, 100);
            delay(300);
            break;

    case 5:
            motor_control_r(-1, 10);
            motor_control_l(-1, 10); //역주행으로 정지
            delay(500);
            motor_control_r(1, 0);
            motor_control_l(1, 0); //정지
            delay(1000);
            break;
  }
}

void Robot_Mode_Define(void) //미로 속에서 움직임 제어
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

void robot_stop() //로봇 움직임 정지
{
  motor_control_r(1, 0);
  motor_control_l(1, 0);
}

void robot_90_right_turn() //로봇 움직임 오른쪽으로 90도 회전
{
  motor_control_r(1, 0);
  motor_control_l(1, 0);

  delay(500);
  
  motor_control_r(-1, 150);
  motor_control_l(1, 150);
  
  delay(680); // 90도를 돌 때까지 시간 조정
  
  motor_control_r(1, 0);
  motor_control_l(1, 0);
}

void robot_90_left_turn() //로봇 움직임 왼쪽으로 90도 회전
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

void setup() //세팅
{
  int i;
  
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  Serial.begin(9600);
}

void loop() // 반복문(메인문)
{
  int i;

    Serial.println(mode);
    Robot_Mode_Define();
    LineType = read_digital_line_sensor();
    read_ultrasonic_sensor();
    robot_line_trace();
    //미로 속에서의 움직임
    if(  (  (UltrasonicSensorData[2] + UltrasonicSensorData[3]) < 130 ) && (  (UltrasonicSensorData[2] * UltrasonicSensorData[3]) > 0 ) && (LineType == -5)  )
    {
      
      //Robot_Mode_Define();
      Serial.println(mode);
      if(mode == 1)
      {
        if(UltrasonicSensorData[0] > 60)
        {
          motor_control_l(1, 150);
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
        if(UltrasonicSensorData[0] > 60)
        {
          motor_control_l(1, 150);
          motor_control_r(1, 200);
        }
        else
        {
          robot_stop();
          delay(500);
          robot_90_right_turn();
          delay(300);
        }
      }
      else if(mode == 2)
      {
        if(UltrasonicSensorData[0] > 60)
        {
          motor_control_l(1, 200);
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
    //라인안에서의 움직임
    else
    {
        //LineType = read_digital_line_sensor();
        //read_ultrasonic_sensor();
        //robot_line_trace();
        
        if(DEBUG == 1) send_serial_data();
      
        if(  (UltrasonicSensorData[0] <= 20) && (UltrasonicSensorData[0] > 0) && (  (UltrasonicSensorData[2] + UltrasonicSensorData[3]) > 140 ) && (LineType != -5)  )
        {
          motor_control_r(1, 0);
          motor_control_l(1, 0);
          delay(1000);
        }
        else
        {
          motor_control_r(1, 100);
          motor_control_l(1, 120); //좌우 속도 맞추기
        }
     }
}
