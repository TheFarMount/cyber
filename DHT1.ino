#include "DHT.h"

#define DHTPIN A12
#define DHTTYPE DHT11
 
 
// 初始化 DHT 对象
DHT dht(DHTPIN, DHTTYPE);
 
void setup() {
  Serial.begin(9600);
  Serial.println(F("DHTxx test!"));
 
  dht.begin();
}
 
void loop() {
  // 读取湿度
  float humid = dht.readHumidity();
  // 读取温度
  float temp = dht.readTemperature();

  // 显示内容
  Serial.print("湿度: ");
  Serial.print(humid);
  Serial.print("% 温度: ");
  Serial.print(temp);
  Serial.println("°C ");
  delay(2000);
}