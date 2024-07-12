int Led = 23 ; // define LED Interface
int DOpin = 27; // define Metal Touch Sensor Interface
int AOpin = A13; //
int val ; // define numeric variables val
int data;
void setup ()
{
  Serial.begin(9600);
  pinMode (Led, OUTPUT) ; // define LED as output interface
  pinMode (DOpin, INPUT) ; // define metal touch sensor output interface
}
void loop ()
{
  data = analogRead(AOpin);
  Serial.println(data);
  delay(500);
  val = digitalRead (DOpin) ; // digital interface will be assigned a value of 3 to read val
  if (val == HIGH) // When the metal touch sensor detects a signal, LED flashesanalogRead(AOpin);
  {
    digitalWrite (Led, HIGH);
  }
  else
  {
    digitalWrite (Led, LOW);
  }
}
