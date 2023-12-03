#define ss1 A0
#define ss2 A1
#define ss3 A2
#define ss4 A3
#define ss5 A4

#define sstrai 12
#define ssphai 13

#define trig 11
#define echo 10
unsigned long thoigian; 
int khoangcach; 

const int EnA = 5;
const int EnB = 6;
const int inA1 = 2;
const int inA2 = 3;  
const int inB1 = 4;  
const int inB2 = 7;  

bool phai;
bool trai;

unsigned long startTime;
float P;
float I, D;
float error;
float previous_error = 0;
float PID_value;
int sensor[5];
int initial_motor_speed = 80;
int *ptr = &initial_motor_speed;

void setup()
{
  pinMode(ss1, INPUT);
  pinMode(ss2, INPUT);
  pinMode(ss3, INPUT);
  pinMode(ss4, INPUT);
  pinMode(ss5, INPUT);
  
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  
  pinMode(ssphai, INPUT);
  pinMode(sstrai, INPUT);

  pinMode(inA1, OUTPUT);
  pinMode(inA2, OUTPUT);
  pinMode(inB1, OUTPUT);
  pinMode(inB2, OUTPUT);
  pinMode(EnA, OUTPUT);
  pinMode(EnB, OUTPUT);
  Serial.begin(9600);
  I = 0;
  D = 0;
  P = 0;
  error = 0;
}

void loop()
{
  check_barrier();
  if (khoangcach <= 15) {
    digitalWrite(inA1, LOW);
    digitalWrite(inA2, LOW);
    digitalWrite(inB1, LOW);
    digitalWrite(inB2, LOW);
  } else {
  unsigned long currentTime = millis();
  read_sensor_value();

  if (currentTime - startTime < 1000) {
    calculate_pid();
    motor_control();
    if((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1))
    {
      tien();
    }
    if((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0))
    {
      tien();
    }
  } else {
    if (phai == 1 && trai == 0)
    {
      re_trai();
      delay(250);
    }
    if (phai == 0 && trai == 1)
    {
      re_phai();
      delay(250);
    }
  calculate_pid();
  motor_control();
   }
 }
}

void re_phai()
{
  int cua = *ptr;
  analogWrite(EnA, cua + 30);
  analogWrite(EnB, cua + 30);

  digitalWrite(inA1, HIGH);
  digitalWrite(inA2, LOW);
  digitalWrite(inB1, LOW);
  digitalWrite(inB2, HIGH);

}

void re_trai()
{
  int cua = *ptr;
  analogWrite(EnA, cua + 30);
  analogWrite(EnB, cua + 30);

  digitalWrite(inA1, LOW);
  digitalWrite(inA2, HIGH);
  digitalWrite(inB1, HIGH); 
  digitalWrite(inB2, LOW);
}

void dung()
{
  digitalWrite(inA1, LOW);
  digitalWrite(inA2, LOW);
  digitalWrite(inB1, LOW);
  digitalWrite(inB2, LOW);

  analogWrite(EnA, 0);
  analogWrite(EnB, 0);
}

void read_sensor_value()
{
  sensor[0] = digitalRead(ss1);
  sensor[1] = digitalRead(ss2);
  sensor[2] = digitalRead(ss3);
  sensor[3] = digitalRead(ss4);
  sensor[4] = digitalRead(ss5);
  phai = digitalRead(ssphai);
  trai = digitalRead(sstrai);

  // Giữ nguyên cách đọc giá trị từ cảm biến

  if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 1))
    error = 4;
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 1) && (sensor[4] == 1))
    error = 3;
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1))
    error = 2.5;
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 1) && (sensor[4] == 0))
    error = 2;
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 0))
    error = 1;
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0))
    error = 0;
  else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0))
    error = -1;
  else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0))
    error = -2;
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0))
    error = -2.5;
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0))
    error = -3;
  else if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0))
    error = -4;
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0))
  {
    error = 0;
    analogWrite(EnA, *ptr);
    analogWrite(EnB, *ptr);
    digitalWrite(inA1, HIGH);
    digitalWrite(inA2, LOW);
    digitalWrite(inB1, HIGH);
    digitalWrite(inB2, LOW);
    }
  
  else
    error = 0;
    
  if(sensor[2] == 1 )
   *ptr = 70  ;
    else
  {
    *ptr = 70;
  }
}

void calculate_pid()
{
  float Kp = 20.5, Ki = 0.07, Kd = 2;  // Corrected variable name
  float P, I, D;
  float SamplingTime = 0.01;
  error = 0 - error;
  P = error * Kp;
  I += Ki * error * SamplingTime;
  D = (Kd * (error - previous_error)) / SamplingTime;
  PID_value = P + I + D;
  previous_error = error;
}

void motor_control()
{
  read_sensor_value();

  int left_motor_speed = initial_motor_speed - PID_value;
  int right_motor_speed = initial_motor_speed + PID_value;
  
  PID_value;

  if (left_motor_speed > 255)
    left_motor_speed = 255;
  else if (left_motor_speed < 0)
    left_motor_speed = 0;  

  if (right_motor_speed > 255)
    right_motor_speed = 255;
  else if (right_motor_speed < 0)
    right_motor_speed = 0;  

  analogWrite(EnA, left_motor_speed);
  analogWrite(EnB, right_motor_speed);

  digitalWrite(inA1, HIGH);
  digitalWrite(inA2, LOW);
  digitalWrite(inB1, HIGH);
  digitalWrite(inB2, LOW);
}

void tien()
{
  int  cua = *ptr;

  analogWrite(EnA, cua + 30);
  analogWrite(EnB, cua + 30);

  digitalWrite(inA1, HIGH);
  digitalWrite(inA2, LOW);
  digitalWrite(inB1, HIGH);
  digitalWrite(inB2, LOW);
}

void tien_cham()
{
  analogWrite(EnA, 60);
  analogWrite(EnB, 60);

  digitalWrite(inA1, HIGH);
  digitalWrite(inA2, LOW);
  digitalWrite(inB1, HIGH);
  digitalWrite(inB2, LOW);
}

void check_barrier() {
  digitalWrite(trig,0); //Tắt chân trig
  delayMicroseconds(2); 
  digitalWrite(trig,1); //bật chân trig để phát xung
  delayMicroseconds(10); //Xung có độ rộng là 10 microsecond
  digitalWrite(trig,0);
  //Chân echo sẽ nhận xung phản xạ lại
  //Và đo độ rộng xung cao ở chân echo
  thoigian = pulseIn (echo, HIGH);
  // Sound speed = 340 m/s => 29.412 microSeconds/cm 
  khoangcach = int (thoigian / 2 / 29.412); 
  Serial.print("Khoảng cách: ");
  Serial.println(khoangcach);
}
