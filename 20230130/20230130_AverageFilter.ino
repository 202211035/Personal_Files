const int numReadings = 10; // 이동평균을 구할 갯수를 지정. 숫자가 변하지 않게 const int 로 지정​ 
int readings[numReadings];  // 신호값을 읽는 배열을 지정. 배열의 크기는 위의 값으로 정함
int readIndex = 0;          // 몇번째 신호인지를 표시하는 인덱스 변수 
int total = 0;              // 합계 변수 
int average = 0;            // 평균값 변수 
int inputPin[4] = {0, };          // 입력핀 정의 

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
  
  for(int i = 0; i < 4; i++)
  {
    inputPin[i] = UltrasonicSensorData[i];
  }
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

void setup()
{ 
  Serial.begin(9600);       // 시리얼 통신 속도를 9600으로 통신 시작 

  for (int thisReading = 0; thisReading < numReadings; thisReading++)
  { 
      // 현재값을 읽는 변수를 0으로 초기화
      readings[thisReading] = 0;
  }
}

void loop()
{ 
  int i;
  int rawInput[i];
  for(i = 0; i < 4; i++)
  {
    rawInput[i] = analogRead(inputPin[i]);     // 센서입력값을 읽어온다

    total = total - readings[readIndex];     // 가장 오래된 data를 합계에서 빼서 없앤다
    readings[readIndex] = rawInput[i];          // 센서입력값을 배열에 저장
    total = total + readings[readIndex];     // 읽은 값을 합계에 더한다
    readIndex = readIndex + 1;               // 신호값을 읽은 인덱스를 1 증가 시킨다.
  
    if (readIndex >= numReadings)
    {     // 만약 신호를 읽는 인덱스의 값이 평균갯수보다 커지면
      readIndex = 0; // 0으로 만들어 처음부터 다시 시작한다
    }
    average = total / numReadings; // 평균값을 구한다
    
    Serial.println(average); // 평균값을 시리얼로 출력
    delay(10); // 안정되게 신호를 보낼수있도록 시간 지연을 조금 준다. 
  }
}
