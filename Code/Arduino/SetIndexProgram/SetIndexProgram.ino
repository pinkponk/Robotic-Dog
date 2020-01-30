
int DEPin = 2; // The pin to be used for enable/disable signal
byte servoID= 0xFE;




void setup() {
  Serial2.begin(500000);
  Serial.begin(4800);
  pinMode(13, OUTPUT);
  pinMode(DEPin, OUTPUT);
  digitalWrite(DEPin,HIGH);
  
  int newID = 4;
  lightServo(newID);
  int free = 0;
  
  if (free==1){
  freeRun(newID,0x06,0);
  freeRun(newID,0x07,0);
  freeRun(newID,0x08,0);
  freeRun(newID,0x09,0);}
  else{
  freeRun(newID,0x06,0);
  freeRun(newID,0x07,0);
  freeRun(newID,0x08,0xFF);
  freeRun(newID,0x09,0x03);}


}

void loop() {
  delay(1000);
  int newID = 4;
  int free = 1;
  setID(newID);
  //Testing the new address by turning on the LED
  lightServo (newID);
if (free==1){
  torque(newID,1);}
  else{
    
    torque(newID,0);
    }
}


void freeRun(int ID,byte address,byte value){

   int checksum_ACK;
   byte notchecksum;
   byte startAddress = address;     // Turning on led

    byte buf[] = {ID,0x04,0x03,startAddress,value};
    notchecksum=calcNotChecksum(buf,5);
 
    digitalWrite(DEPin,HIGH);     // Notify max485 transciever to accept tx 
    delayMicroseconds(50);
    Serial2.write(0xFF);
    Serial2.write(0xFF);
    Serial2.write(buf, sizeof(buf));
    Serial2.write(notchecksum);
    delayMicroseconds(50);
}

void torque(int ID,byte value){

   int checksum_ACK;
   byte notchecksum;
   byte startAddress = 0X20;     // Turning on led

    byte buf[] = {ID,0x05,0x03,startAddress,value,value};
    notchecksum=calcNotChecksum(buf,6);
 
    digitalWrite(DEPin,HIGH);     // Notify max485 transciever to accept tx 
    delayMicroseconds(50);
    Serial2.write(0xFF);
    Serial2.write(0xFF);
    Serial2.write(buf, sizeof(buf));
    Serial2.write(notchecksum);
    delayMicroseconds(50);
}


//Skapa ett kalebreingsprogram som sätter rätt nolla, angle limits och id





int setID(int newID){
 //Sets the Servo whos ID is currently 1 to newID 
  if (checkIsBetweenValue(newID,0,200)){
    digitalWrite(DEPin,HIGH);          // Notify SN75176BP transciever to accept tx 
    delayMicroseconds(50);              // Allow this to take effect, the SN75176 needs about 20 nanosec

    byte buf[] = {servoID,0x04,0x03,0x03,byte (newID)};
    byte notCheckSum = calcNotChecksum(buf,5);
    Serial2.write(0xFF);
    Serial2.write(0xFF);
    Serial2.write(buf, sizeof(buf));
    Serial2.write(notCheckSum);
    delayMicroseconds(50);
    delay(1000);
    delay(1000);

    digitalWrite(DEPin,LOW);
        delay(1000);
    delay(1000);
    return 1;
  }else{
    return 0;
  }
}

void lightServo (int ID){
   int checksum_ACK;
   byte notchecksum;
   byte startAddress = 0x19;     // Turning on led

    byte buf[] = {ID,0x04,0x03,startAddress,0x01};
    notchecksum=calcNotChecksum(buf,5);
 
    digitalWrite(DEPin,HIGH);     // Notify max485 transciever to accept tx 
    delayMicroseconds(50);
    Serial2.write(0xFF);
    Serial2.write(0xFF);
    Serial2.write(buf, sizeof(buf));
    Serial2.write(notchecksum);
    delayMicroseconds(50);
}


boolean checkIsBetweenValue(int value, int minimum, int maximum){
  if (value > minimum && value < maximum){
    return  true;
  }
 return false;
}

byte calcNotChecksum(byte* data, int length){
  byte sum = 0;
  for (int i = 0; i < length; i++){
    sum += data[i];
  }
  return (~sum);
}
