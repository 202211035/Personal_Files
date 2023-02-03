/////////////////////////////////////////Motor Control Setup///////////////////////////////////////////
#define DEBUG 1

/////////////////////////////////////////Motor Control Setup///////////////////////////////////////////
#define IN1 2
#define IN2 3
#define ENA 5

#define IN3 4
#define IN4 7
#define ENB 6

int mode = -1;

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
/////////////////////////////////////////Line Sensor Setup///////////////////////////////////////////////////////
#define LINE_DETECT_WHITE 1

int LineSensorPinData[5] = {0, };
int LineType = -1;
/////////////////////////////////////////Line Sensor Setup///////////////////////////////////////////////////////

int read_digital_line_sensor(void)
{
  int i;
  int sum = 0;
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

void robot_line_trace(void)
{
  switch(LineType)
  {
    case -4:
            motor_control_r(1, 100); //속도 제어할 것
            motor_control_l(-1, 100);
            delay(200);
            break;
    
    case -3:
            motor_control_r(1, 100);
            motor_control_l(-1, 100);
            delay(100);
            break;
    
    case -2:
            motor_control_r(1, 100);
            motor_control_l(-1, 70);
            break;
    
    case -1:
            motor_control_r(1, 100);
            motor_control_l(-1, 70);
            break;

    case 0:
            motor_control_r(1, 100);
            motor_control_l(1, 100);
            break;

    case 1:
            motor_control_r(-1, 70);
            motor_control_l(1, 100);
            break;
    
    case 2:
            motor_control_r(-1, 70);
            motor_control_l(1, 100);
            break;
    
    case 3:
            motor_control_r(-1, 100);
            motor_control_l(1, 100);
            delay(100);
            break;
    
    case 4:
            motor_control_r(-1, 100);
            motor_control_l(1, 100);
            delay(200);
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



void setup()
{
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  Serial.begin(9600);
}

void loop()
{
  LineType = read_digital_line_sensor();
  read_ultrasonic_sensor();

  robot_line_trace();
  
  if(DEBUG == 1) send_serial_data();

  if(  (UltrasonicSensorData[0] <= 20) && (UltrasonicSensorData[0] > 0)  )
  {
    motor_control_r(1, 0);
    motor_control_l(1, 0);
  }
  else
  {
    motor_control_r(1, 100);
    motor_control_l(1, 120); //좌우 속도 맞추기
  }
}
