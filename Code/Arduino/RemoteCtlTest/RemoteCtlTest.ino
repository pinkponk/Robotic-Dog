

const int sensorPinX = A14;    // select the input pin for the potentiometer
const int sensorPinY = A15;    // select the input pin for the potentiometer
const int buttonPin = 53;    // select the input pin for the potentiometer
const int VpwrPin1 = 51;    // select the input pin for the potentiometer
const int VpwrPin2 = 50;    // select the input pin for the potentiometer

int sensorValueX = 0;  // variable to store the value coming from the sensor
int sensorValueY = 0;  // variable to store the value coming from the sensor
int buttonState = 0;
int timeBetweenPoints;
void setup() {
  // declare the ledPin as an OUTPUT:
  pinMode(51, INPUT_PULLUP);
  //pinMode(VpwrPin1, OUTPUT);
  pinMode(VpwrPin2, OUTPUT);
  //digitalWrite(VpwrPin1, HIGH);
  digitalWrite(VpwrPin2, HIGH);
  Serial.begin(19200);
}

void loop() {
  // read the value from the sensor:
  sensorValueX = analogRead(sensorPinX);
  sensorValueY = analogRead(sensorPinY);
  buttonState = digitalRead(51);
  
  Serial.write(27);       // ESC command
  Serial.print("[2J");    // clear screen command
  Serial.write(27);
  Serial.print("[H");     // cursor to home command
  timeBetweenPoints = map(sensorValueX, 0, 1023, 0, 200);

  Serial.print("sensorValueX=");Serial.println(timeBetweenPoints);
  Serial.print("sensorValueY=");Serial.println(sensorValueY);
  Serial.print("buttonState=");Serial.println(buttonState);
}
