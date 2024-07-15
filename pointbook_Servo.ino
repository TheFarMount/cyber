#include <Servo.h>
int a = 0;

#define bookpointer_servoPin1 40
#define bookpointer_servoPin2 38

Servo bookpointer_servo1;
Servo bookpointer_servo2;

void setup() {
  Serial.begin(9600);
  delay(1000);
  bookpointer_servo1.attach(bookpointer_servoPin1); // Attaches the pin to the servo object
  bookpointer_servo2.attach(bookpointer_servoPin2);
  // put your setup code here, to run once:
  bookpointer_servo1.write(0);
  bookpointer_servo2.write(180);
  delay(100);
  a=1;
  Serial.println(0);
}

void loop() {
  
  double book_height=12.0;
  double car_book_d=6.0; // 底部舵机-书水平距离（认为定制）
  double base_length = sqrt(car_book_d * car_book_d + book_height * book_height);
  double theta1_1 = atan(book_height / car_book_d);

  double arm_length=10.5; // 机械臂长度
  // theta均为弧度
  double theta1_2 = acos(base_length / (2 * arm_length));
  double theta1 = theta1_1 + theta1_2;
  double theta2 = 2 * asin(base_length / (2 * arm_length));

  if(a==1)
  {
    bookpointer_servo1.write(180-(theta1 * 180 / PI)-35);
    bookpointer_servo2.write(180-(theta2 * 180 / PI));
    Serial.println(1);
    delay(500);

  }


  
  // put your main code here, to run repeatedly:
  

}
