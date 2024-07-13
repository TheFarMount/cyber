
#include <Servo.h>
#include <AFMotor.h>
#include <DHT.h>

//robot modes
#define ERRORMODE   0   //Initialization mode, which represents an abnormal exit
#define SHAKEHANDS  1   //readiness mode before shake hands
#define CRUISE      2   //ruising mode(moniter TH, moniter noise, )
#define SEARCHBOOK  3   //Wait user to input book number;
#define LEADWAY     4   //Lead the way
#define ADMINISTER  5   //Find people who is making the noise


/*************************************************************************************************/
/*                                   All functions' declarations                                 */
/*************************************************************************************************/
void touchISR();
void sonarISR();
void sonarAvoid();
char readKeypad();
void point_book(double book_height);

int shakeHands();
int robotCruise();
int searchBook();
int leadWay();
int stopNoise();
/*************************************************************************************************/
/*                                           全局变量                                             */
/*************************************************************************************************/

/*********/
/*引脚设定*/
/*********/

//keypad引脚
static int col_pins[4] = {44, 42, 40, 38};    // 定义行引脚
static int row_pins[4] = {52, 50, 48, 46};    // 定义列引脚
//舵机引脚
const int bookpointer_servoPin1 = 38; // 上(下)舵机
const int bookpointer_servoPin2 = 40; // 上(下)舵机
//触摸传感器引脚
int touch_DOpin = 27;  // define Metal Touch Sensor Interface
int touch_AOpin = A13; //
//声纳引脚
const int trigPin = 12;
const int echoPin = 2; // Using pin 2 for interrupt
//DHT引脚
#define DHTPIN A12
#define DHTTYPE DHT11
//LED引脚
const int led_DHTB = 22;        //温度异常
const int led_DHTR = 23;        //湿度异常
const int led_DHTG = 24;        //温湿度正常
const int led_barrier = 25;     //有障碍物

const int led_mode[6] = {1,2,3,4,5,6};   //表示所处模式


/*********/
/*标志信息*/
/*********/
int mode = SHAKEHANDS;         //总模式标志
bool touch_tag = 0;            //是否被触摸
bool barrier_tag = 0;         //是否有障碍物

/*********/
/*马达信息*/
/*********/
//马达对象绑定
const int motor_fl_pin = 2; AF_DCMotor motor_fl(motor_fl_pin);
const int motor_fr_pin = 1; AF_DCMotor motor_fr(motor_fr_pin);
const int motor_bl_pin = 3; AF_DCMotor motor_bl(motor_bl_pin);
const int motor_br_pin = 4; AF_DCMotor motor_br(motor_br_pin);
int max_speed = 255;


/*********/
/*书籍信息*/
/*********/

const int book_amount = 3;    //书籍总数据量
int target_book = 0;          //待取书籍
int book_code[book_amount] =  //书记编码
  {0000, 0001, 0010};
int book_locate[3][book_amount]=  //书籍坐标
{
  {   1,    1,    1},          //小车目标横坐标
  {   2,    2,    2},          //小车目标列坐标
  {   3,    3,    3}           //小车目标高度
};

/**********/
/*声纳测距*/
/**********/

const float minDistance = 20.0; //极限距离(cm)
volatile unsigned long pulseStart;     
volatile unsigned long pulseEnd;
volatile bool newMeasurement = false;


/*************************************************************************************************/
/*                               Arduino functions                                               */
/*************************************************************************************************/

void setup()
{
  Serial.begin(9600);
  /*--------------------------------引脚设置-------------------------------------------------*/

  //LED引脚
  for(int i=0; i<6; i++)
  {
    pinMode(led_mode[i], OUTPUT);
  }
  pinMode(led_DHTR, OUTPUT);
  pinMode(led_DHTG, OUTPUT);
  pinMode(led_DHTB, OUTPUT);
  pinMode(led_barrier, OUTPUT);

  //keypad引脚
  for (int i = 0; i < 4; i++)
  {
    pinMode(row_pins[i], INPUT_PULLUP);  //  行引脚设置为输入
  }
  for (int i = 0; i < 4; i++)
  {
    pinMode(col_pins[i], OUTPUT);  // 列引脚设置为输出
    digitalWrite(col_pins[i], HIGH); // 初始化为高电平
  }
  // 触摸传感器引脚
  pinMode(touch_DOpin, INPUT);
  pinMode(touch_AOpin, INPUT);
  //声纳引脚
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  /*--------------------------------中断设置-------------------------------------------------*/
  //中断设置
  attachInterrupt(digitalPinToInterrupt(touch_DOpin), touchISR, CHANGE);   
  attachInterrupt(digitalPinToInterrupt(echoPin), sonarISR, CHANGE);
}

void loop()
{
  switch(mode)
  {
    case ERRORMODE:
      digitalWrite(led_mode[0], HIGH);
      Serial.println("ERROR: Abnormal exit");
      delay(1000);
      break;
    case SHAKEHANDS:
      digitalWrite(led_mode[1], HIGH);
      Serial.println("INFO: Shaking hands");
      mode = shakeHands();
      break;
    case CRUISE:
      digitalWrite(led_mode[2], HIGH);
      Serial.println("INFO: Cruising");
      mode = robotCruise();
      break;
    case SEARCHBOOK:
      digitalWrite(led_mode[3], HIGH);
      mode = searchBook();
      break;
    case LEADWAY:
      digitalWrite(led_mode[4], HIGH);
      //Serial.println("INFO: Leading");
      mode = leadWay();
      break;
    case ADMINISTER:
      digitalWrite(led_mode[5], HIGH);
      stopNoise();
      break;
  }
  for(int i=0; i<6; i++)
  {
    digitalWrite(led_mode[i],LOW);
  }
}


/*************************************************************************************/
/*                          interrupt functions                                          */
/*************************************************************************************/
void touchISR()
{
  int data = 0;
  int val = 0;
  data = analogRead(touch_AOpin);
  val = digitalRead (touch_DOpin) ; // digital interface will be assigned a value of 3 to read val
  if (val == HIGH) // When the metal touch sensor detects a signal, LED flashesanalogRead(AOpin);
  {
    touch_tag = 1;         //被触摸
  }
  else
  {
    touch_tag = 0;
  }
}

void sonarISR() 
{
  if (digitalRead(echoPin) == HIGH) 
  {
    // Rising edge: record the start time
    pulseStart = micros();
  } else {
    // Falling edge: record the end time and set flag
    pulseEnd = micros();
    newMeasurement = true;
  }
}

/*************************************************************************************/
/*                          moving functions                                        */
/*************************************************************************************/


/***********************/
/*功能:声呐避障         */
/*返回值:是否有障碍物    */
/***********************/

void sonarAvoid()
{
  static unsigned long lastTriggerTime = 0;

  // Trigger the ultrasonic sensor every 100ms
  if (millis() - lastTriggerTime >= 100) {
    // Trigger the ultrasonic sensor
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    lastTriggerTime = millis(); // Reset the timer
  }
  if (newMeasurement) 
  {
    newMeasurement = false;
    unsigned long duration = pulseEnd - pulseStart;
    float distance = (duration * 0.0343) / 2;

    // Print the distance to the Serial Monitor
    //Serial.print("Distance: ");
    //Serial.println(distance);

    // Update the LED state based on the measured distance
    if (distance < minDistance) 
    {
      barrier_tag = 1;
      digitalWrite(led_barrier, HIGH);
    }
    else 
    {
      barrier_tag = 0;
      digitalWrite(led_barrier, LOW);
    }
  }
}

void motorRun()
{
  if (Serial.available())
  {
    int msg = Serial.read();
    if (msg == 87 && !barrier_tag)
    {
      moveBackward();
      delay(200);
    }
    else if (msg == 83)
    {
      moveForward();
      delay(200);
    }
    else if (msg == 65)
    {
      turnLeft();
      delay(200);
    }
    else if (msg == 68)
    {
      turnRight();
      delay(200);
    }
  }
  motor_fl.run(RELEASE);
  motor_bl.run(RELEASE);
  motor_fr.run(RELEASE);
  motor_br.run(RELEASE);
}

// 左转
void turnLeft()
{
  int speed = max_speed;
  motor_fl.setSpeed(speed);
  motor_bl.setSpeed(speed);
  motor_fr.setSpeed(speed);
  motor_br.setSpeed(speed);

  motor_fl.run(BACKWARD);
  motor_bl.run(BACKWARD);
  motor_fr.run(FORWARD);
  motor_br.run(BACKWARD);
}

// 右转
void turnRight()
{
  int speed = max_speed;
  motor_fr.setSpeed(speed);
  motor_br.setSpeed(speed);
  motor_fl.setSpeed(speed);
  motor_bl.setSpeed(speed);

  motor_fr.run(BACKWARD);
  motor_br.run(FORWARD);
  motor_fl.run(FORWARD);
  motor_bl.run(FORWARD);
}

// 前进
void moveForward()
{
  int speed = max_speed;
  motor_fl.setSpeed(speed);
  motor_bl.setSpeed(speed);
  motor_fr.setSpeed(speed);
  motor_br.setSpeed(speed);

  motor_fl.run(FORWARD);
  motor_bl.run(FORWARD);
  motor_fr.run(FORWARD);
  motor_br.run(BACKWARD);
}

// 后退
void moveBackward()
{
  int speed = max_speed;
  motor_fl.setSpeed(speed);
  motor_bl.setSpeed(speed);
  motor_fr.setSpeed(speed);
  motor_br.setSpeed(speed);

  motor_fl.run(BACKWARD);
  motor_bl.run(BACKWARD);
  motor_fr.run(BACKWARD);
  motor_br.run(FORWARD);
}


/*************************************************************************************/
/*                        shake hands' functions                                     */
/*************************************************************************************/

int shakeHands()
{
  int next_mode = 0;
  next_mode = CRUISE;
  return next_mode;
}

/*************************************************************************************/
/*                          crusing functions                                        */
/*************************************************************************************/


/********************/
/*功能: 监控温湿度    */
/********************/
void testDHT()
{
  const static int temp_max = 30;
  const static int temp_min = 0;
  const static int hum_max = 50;
  const static int hum_min = 20;

  static int begin_tag = 1;
  static DHT dht(DHTPIN, DHTTYPE);
  if(begin_tag)
  {
    dht.begin();
    begin_tag = 0;
  }
  // 读取湿度
  float humid = dht.readHumidity();
  // 读取温度
  float temp = dht.readTemperature();
  //温度监测
  if(!(temp_min<temp && temp<temp_max)){
    digitalWrite(led_DHTB,HIGH);
  }
  else{
    digitalWrite(led_DHTB,LOW);
  }
  //湿度监测
  if(!(hum_min<humid && humid<hum_max)){
    digitalWrite(led_DHTR,HIGH);
  }
  else{
    digitalWrite(led_DHTR,LOW);
  }
  //温湿度正常
  if((temp_min<temp && temp<temp_max)&&(hum_min<humid && humid<hum_max)){
    digitalWrite(led_DHTG,HIGH);
  }
  else{
    digitalWrite(led_DHTG,LOW);
  }

}

/**********************/
/*功能: 巡航状态主函数  */
/*返回值: 下一循环状态  */
/**********************/
int robotCruise()
{
  int next_mode = 0;
  testDHT();
  //若被触摸进入领路模式
  if(touch_tag == 1)
  {
    next_mode = SEARCHBOOK;
  }
  else            //根据python指令移动
  {
    motorRun();
  }
  return next_mode;
}

/*************************************************************************************/
/*                leading way functions                                              */
/*************************************************************************************/

/************************/
/*功能：读取keypad输入   */
/*返回值：keypad输入     */
/************************/
char readKeypad()
{
  // 定义键盘按键布局
  char keys[4][4] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}
  };
  // 按下的按键
  char key = 0;

  // 行列扫描法
  for (int j=0;j<4;j++) {
    // 将当前列设置为低电平
    digitalWrite(col_pins[j], LOW);
    for (int i=0;i<4;i++) {
      // 检测行输入引脚状态, 检测到低电平,说明按键被按下,则返回该按键值
      if (!digitalRead(row_pins[i])) {
        // 将该行恢复为高电平
        digitalWrite(col_pins[j], HIGH);
        key = keys[i][j];
      }
    }
    // 将该行恢复为高电平
    digitalWrite(col_pins[j], HIGH);
  }
  
  return key;
}

/******************************/
/*功能：通过客户输入的编码寻书  */
/*符号说明：A代表删除上次输入   */
/*         *代表输入清零       */
/*         #代表确定输入       */
/*返回值：下次loop()进入的模式  */
/******************************/
int searchBook()
{
  static const int max_wait_time =  1000;  //max wait time(ms)
  static int begin_tag = 1; //weather just enter the mode
  static int start_time = 0;
  static bool  release_tag = 1;       //两次读数中间必松手一次，防止重复读入
  int present_time;
  static int input_num = 0;    //num has been input
  int next_mode = 0;
  int key;
  
  //转入寻书模式时重置等待时间，重置输入编码
  if(begin_tag)
  {
    start_time = millis();
    input_num = 0;
    begin_tag = 0;
  }
  present_time = millis();
  //超时检测
  if(present_time - start_time > max_wait_time)
  {
    begin_tag = 1;
    Serial.println("ERROR: Time out");
    next_mode = CRUISE;
  }
  //等待keypad输入
  else
  {
    key = readKeypad();
    if(key == 0)
    {
      next_mode = SEARCHBOOK;
      release_tag = 1;
    }
    else if(release_tag)
    {
      release_tag = 0;
      start_time = millis();             //重置等待时间
      switch(key)
      {
        case '0':
          input_num = input_num*10 + 0;
          break;
        case '1':
          input_num = input_num*10 + 1;
          break;
        case '2':
          input_num = input_num*10 + 2;
          break;
        case '3':
          input_num = input_num*10 + 3;
          break;
        case '4':
          input_num = input_num*10 + 4;
          break;
        case '5':
          input_num = input_num*10 + 5;
          break;
        case '6':
          input_num = input_num*10 + 6;
          break;
        case '7':
          input_num = input_num*10 + 7;
          break;
        case '8':
          input_num = input_num*10 + 8;
          break;
        case '9':
          input_num = input_num*10 + 9;
          break;
        case 'A':
          input_num = input_num/10;
          break;
        case 'B':
          break;
        case 'C':
          break;
        case 'D':
          break;
        case '*':
          input_num = 0;
          break;
        case '#':
          int temp_tag = 1;       //标记是否找到书
          for(int i=0; i<book_amount; i++)
          {
            if(book_code[i] == input_num)
            {
              temp_tag = 0;
              target_book = i;
            }
            if(temp_tag)
              next_mode = CRUISE;        //未找到书
            else
              next_mode = LEADWAY;
          }
          break;
      }
      next_mode = SEARCHBOOK;
      delay(200);//防止重复读入
    }
  }
  
  return next_mode;
}

/***********************************/
/* 功能: 使用机械臂取书              */
/* 参数: 书籍所在高度                 */
/************************************/
void point_book(double book_height)
{
  // 舵机对象
  const static Servo bookpointer_servo1;
  const static Servo bookpointer_servo2;
  const static double car_book_d = 8;        // 底部舵机-书水平距离(认为定值)
  const static double arm_length;            // 机械臂长度
  static bool begin_tag = 1;

  if(begin_tag)
  {
    // 舵机设置Attaches the pin to the servo object
    bookpointer_servo1.attach(bookpointer_servoPin1); 
    bookpointer_servo2.attach(bookpointer_servoPin2); 
    begin_tag = 0;
  }

  double base_length = sqrt(car_book_d * car_book_d + book_height * book_height);
  // theta均为弧度
  double theta1_1 = atan(book_height / car_book_d);
  double theta1_2 = acos(base_length / (2 * arm_length));
  double theta1 = theta1_1 + theta1_2;
  double theta2 = 2 * asin(base_length / (2 * arm_length));

  bookpointer_servo1.write(180-(theta1 * 180 / PI)-35);
  bookpointer_servo2.write(180-(theta2 * 180 / PI));
}


/*************************************/
/*功能：根据全局变量移动到书籍对应位置  */
/*返回值：下次loop()进入的模式         */
/*************************************/
int leadWay()
{
  int next_mode = 0;
  static const int max_wait_time =  1000;  //max wait time(ms)
  static int start_time = 0;
  int present_time;
  static int step_tag = 0;

  if(step_tag == 0)    //移动阶段----------------------------------------------待完成
  {
    next_mode = LEADWAY;                          
  }
  else if(step_tag == 1)
  {
    point_book(book_locate[3][target_book]);
    step_tag = 2;
    start_time = millis();
    next_mode = LEADWAY;
  }
  else if(step_tag == 2)
  {
    present_time = millis();
    //超时检测
    if(present_time - start_time > max_wait_time || touch_tag)  //超时或被触摸进入巡航模式
    {
      Serial.println("INFO: Survice done");
      step_tag = 0;
      next_mode = CRUISE;
    }
    else
    {
      next_mode = LEADWAY;
    }
  }

  return next_mode;
}


/******************************************************************************************************/
/******************************************************************************************************/

//stoping noise functions
int stopNoise()
{
  Serial.println(4);
}

