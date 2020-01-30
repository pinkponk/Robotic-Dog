
int state = 1;
int CCW_IDP_button = -1;
int CW_IDM_button = -1;
int freerunbutton = -1;
int resetAnglebutton = -1;
int movePotbutton = -1;
int currID = 1;
int valPot = 512;
const int CCW_IDP_buttonPin = 3;
const int CW_IDM_buttonPin = 4;
const int freerunbuttonPin = 5;
const int resetAnglebuttonPin = 6;
const int movePotbuttonPin = 7;
const int PotPin = 0;
const int DEPin = 2; // The pin to be used for enable/disable signal

byte sendingData[10];
byte returningData[20];

void setup() {
   pinMode(CCW_IDP_buttonPin, INPUT);
   pinMode(CW_IDM_buttonPin, INPUT);
   pinMode(freerunbuttonPin, INPUT);
   pinMode(resetAnglebuttonPin, INPUT);
   pinMode(movePotbuttonPin, INPUT);
   pinMode(DEPin, OUTPUT);
   Serial2.begin(500000);
   Serial.begin(19200);
   delay(1000);

   lightServo(0xFE,1);
   delay(1000);
   limitReturnPackage(0xFE,1);
//   delay(1000);
   setDelayTime(0xFE,10);      //A value of 100 sets the delay time to 200microsek
//   delay(1000);
//   freeRun(false);
//   delay(1000);
   

}

void loop() {
  
  CCW_IDP_button = digitalRead(CCW_IDP_buttonPin);
  CW_IDM_button = digitalRead(CW_IDM_buttonPin);
  freerunbutton = digitalRead(freerunbuttonPin);
  resetAnglebutton = digitalRead(resetAnglebuttonPin);
  movePotbutton = digitalRead(movePotbuttonPin);
  valPot = analogRead(PotPin);    // read the input pin
  
  switch(state){
    case 1:
    //Move with Pot
      delay(10);
      moveServo(currID,valPot);
      
      if (resetAnglebutton==1){
        state = 2;
        
      }else if(freerunbutton==1){
        state = 3;
        
      }else if(CCW_IDP_button==1 && CW_IDM_button==1){
        setCCWLimit(1023);
        setCWLimit(0);
        delay(300);
    
      }else if(CCW_IDP_button==1){
        //setCCWLimit(getCurrPos());
        //setEnableTorque(1);
       // setTorqueLimit(10);
        delay(300);
    
      }else if(CW_IDM_button==1){
        //setCWLimit(getCurrPos());
        //setEnableTorque(0);
        delay(300);
      } 
      
      break;
    case 2:
    //Go to middle, 150 degrees, change currID, reset ID currID to all servos connected
    delay(10);
    moveServo(currID,512);

    if (movePotbutton==1){
      state = 1;
      
    }else if(freerunbutton==1){
      state = 3;
      
    }else if(CCW_IDP_button==1 && CW_IDM_button==1){
      //Dont fuck up
      setIDtoAll(currID);
      lightServo(currID,0);
      delay(100);
      lightServo(currID,1);
      delay(100);
      lightServo(currID,0);
      delay(100);
      lightServo(currID,1);
      delay(100);

  
    }else if(CCW_IDP_button==1){
      lightServo(currID,0);
      currID++;
      lightServo(currID,1);
      delay(300);
  
    }else if(CW_IDM_button==1){
      lightServo(currID,0);
      currID--;
      lightServo(currID,1);
      delay(300);
    } 
    
    break;
    case 3:
    //Freerun CW and CCW
    delay(1);
    
    if (movePotbutton==1){
      state = 1;
      
    }else if(resetAnglebutton==1){
      state = 2;
      
    }else if(CCW_IDP_button==1){

//      freeRun(true);
//      delay(2);
//      freeRunActivate(1);
//      delay(2);
  
    }else if(CW_IDM_button==1){

//      freeRun(true);
//      delay(2);
//      freeRunActivate(0);
//      delay(2);
     
      
    }else{
      
//      freeRun(false);
//      delay(2);
//
//      setSpeed1(0,0);
//      delay(2);
      }
    
    
    break;
     
  }
      printAllInfo2Serial();
      delay(1);
}














void printAllInfo2Serial(){
    

    Serial.write(27);       // ESC command
    Serial.print("[2J");    // clear screen command
    Serial.write(27);
    Serial.print("[H");     // cursor to home command
  
    Serial.print("Servo ID=");
    Serial.print(currID);
    Serial.print("     State=");
    Serial.println(state);
    Serial.print("AnalogPot =");
    Serial.println(valPot);
    Serial.print("Position =");
    Serial.println(getCurrPos());
    int Speed = getCurrSpeed();
    Serial.print("Speed= ");
    Serial.print(Speed&1023);  //Keeps the first 10 bits by using and with binary number 1111111111 (1023)
    Serial.print("    Dir=");
    Serial.print(Speed>>10);
    int Speed2 = getSetSpeed();
    Serial.print("  ||  Target Speed= ");
    Serial.print(Speed2&1023);  //Keeps the first 10 bits by using and with binary number 1111111111 (1023)
    Serial.print("    Dir=");
    Serial.println(Speed2>>10);
    Serial.print("IsMoving=");
    Serial.println(getIsMoving());
    Serial.print("Temp=");
    Serial.println(getTemp());
    Serial.print("Voltage=");
    Serial.println(getVoltage());
    Serial.print("CCWLimit=");
    Serial.print(getCCWLimit());
    Serial.print("  ||  CWLimit=");
    Serial.println(getCWLimit());
    
    Serial.print("Load=");
    int load = getLoad();
    Serial.print(load&1023);  //Load does not read right
    Serial.print("    Dir=");
    Serial.println(load>>10);
    Serial.print("Return Delay Time=");
    Serial.println(getDelayTime());
    Serial.print("Torque Enabled=");
    Serial.println(getEnableTorque());
    Serial.print("Status LVL=");
    Serial.println(getStatusLvl());
    Serial.print("ping error=");
    Serial.println(ping(currID));
    Serial.print("TorqueLimit=");
    Serial.println(getTorqueLimit());
    
    
   Serial.flush();            //Needed otherwise the sending on serial1 will effect reading info on serial2, flush makes the contorller pause until its done sending
   delay(100);
  
  
  }
  
void reset(byte ID){
  byte buf[] = {ID,0x02,0x06};
  memcpy ( sendingData, buf, sizeof(buf) );  //sizeof(buf)
  sendInformation(sendingData, sizeof(buf));
  }
  
int ping(byte ID){
  byte buf[] = {ID,0x02,0x01};
  memcpy ( sendingData, buf, sizeof(buf) );  //sizeof(buf)
  sendInformation(sendingData, sizeof(buf));
  getInfo();
  return (returningData[4]);
  }
  
void setIDtoAll(int value){
  //if(checkIsBetweenValue(value,0, 1)){
  byte buf[] = {0xFE,0x04,0x03,0x03,value};
  memcpy ( sendingData, buf, sizeof(buf) );  //sizeof(buf)
  sendInformation(sendingData, sizeof(buf));
  //}
}


void limitReturnPackage(byte ID, int value){
  byte buf[] = {ID,0x04,0x03,0x10,value};
  memcpy ( sendingData, buf, sizeof(buf) );  //sizeof(buf)
  sendInformation(sendingData, sizeof(buf));
  }


int getCurrPos(){
  byte buf[] = {currID,0x04,0x02,0x24,0x02};
  memcpy ( sendingData, buf, sizeof(buf) );  //sizeof(buf)
  sendInformation(sendingData, sizeof(buf));
  getInfo();
  return (returningData[5]+returningData[6]*255);
  }


int getCurrSpeed(){
  byte buf[] = {currID,0x04,0x02,0x26,0x02};
  memcpy ( sendingData, buf, sizeof(buf) );  //sizeof(buf)
  sendInformation(sendingData, sizeof(buf));
  getInfo();
  return ((returningData[5]+returningData[6]*255));
  }
  
int getSetSpeed(){
  byte buf[] = {currID,0x04,0x02,0x20,0x02};
  memcpy ( sendingData, buf, sizeof(buf) );  //sizeof(buf)
  sendInformation(sendingData, sizeof(buf));
  getInfo();
  return ((returningData[5]+returningData[6]*255));
  }
  
int getIsMoving(){
  byte buf[] = {currID,0x04,0x02,0x2E,0x01};
  memcpy ( sendingData, buf, sizeof(buf) );  //sizeof(buf)
  sendInformation(sendingData, sizeof(buf));
  getInfo();
  return (returningData[5]);
  }
  
int getEnableTorque(){
  byte buf[] = {currID,0x04,0x02,0x18,0x01};
  memcpy ( sendingData, buf, sizeof(buf) );  //sizeof(buf)
  sendInformation(sendingData, sizeof(buf));
  getInfo();
  return (returningData[5]);
  }

void setEnableTorque(int value){
  //if(checkIsBetweenValue(value,0, 1)){
  byte buf[] = {currID,0x04,0x03,0x18,value};
  memcpy ( sendingData, buf, sizeof(buf) );  //sizeof(buf)
  sendInformation(sendingData, sizeof(buf));
  //}
}



int getDelayTime(){
  byte buf[] = {currID,0x04,0x02,0x05,0x01};
  memcpy ( sendingData, buf, sizeof(buf) );  //sizeof(buf)
  sendInformation(sendingData, sizeof(buf));
  getInfo();
  return (returningData[5]);
  }
  
void setDelayTime(int ID, int value){
  if(checkIsBetweenValue(value,0, 250)){
  byte buf[] = {ID,0x04,0x03,0x05,value};
  memcpy ( sendingData, buf, sizeof(buf) );  //sizeof(buf)
  sendInformation(sendingData, sizeof(buf));
  }
}
  
int getVoltage(){
  byte buf[] = {currID,0x04,0x02,0x2A,0x01};
  memcpy ( sendingData, buf, sizeof(buf) );  //sizeof(buf)
  sendInformation(sendingData, sizeof(buf));
  getInfo();
  return (returningData[5]);
  }
  
int getTemp(){
  byte buf[] = {currID,0x04,0x02,0x2B,0x01};
  memcpy ( sendingData, buf, sizeof(buf) );  //sizeof(buf)
  sendInformation(sendingData, sizeof(buf));
  getInfo();
  return (returningData[5]);
  }
  
int getStatusLvl(){
  byte buf[] = {currID,0x04,0x02,0x10,0x01};
  memcpy ( sendingData, buf, sizeof(buf) );  //sizeof(buf)
  sendInformation(sendingData, sizeof(buf));
  getInfo();
  return (returningData[5]);
}
  
  
int getCCWLimit(){
  byte buf[] = {currID,0x04,0x02,0x08,0x02};
  memcpy ( sendingData, buf, sizeof(buf) );  //sizeof(buf)
  sendInformation(sendingData, sizeof(buf));
  getInfo();
  return ((returningData[5]+returningData[6]*255));
}

int getCWLimit(){
  byte buf[] = {currID,0x04,0x02,0x06,0x02};
  memcpy ( sendingData, buf, sizeof(buf) );  //sizeof(buf)
  sendInformation(sendingData, sizeof(buf));
  getInfo();
  return ((returningData[5]+returningData[6]*255));
}

void setCCWLimit(int value){
  if(checkIsBetweenValue(value,0, 1023)){
  byte buf[] = {currID,0x05,0x03,0x08,lowByte(value),highByte(value)};
  memcpy ( sendingData, buf, sizeof(buf) );  //sizeof(buf)
  sendInformation(sendingData, sizeof(buf));
  }
}

void setCWLimit(int value){
  if(checkIsBetweenValue(value,0, 1023)){
  byte buf[] = {currID,0x05,0x03,0x06,lowByte(value),highByte(value)};
  memcpy ( sendingData, buf, sizeof(buf) );  //sizeof(buf)
  sendInformation(sendingData, sizeof(buf));
  }
}
  
int getLoad(){
  byte buf[] = {currID,0x04,0x02,0x28,0x02};
  memcpy ( sendingData, buf, sizeof(buf) );  //sizeof(buf)
  sendInformation(sendingData, sizeof(buf));
  getInfo();
  return ((returningData[5]+returningData[6]*255));
  }
  
void setTorqueLimit(int value){
  if(checkIsBetweenValue(value,0, 1023)){
  byte buf[] = {currID,0x05,0x03,0x22,lowByte(value),highByte(value)};
  memcpy ( sendingData, buf, sizeof(buf) );  //sizeof(buf)
  sendInformation(sendingData, sizeof(buf));
  }
}
  
  
int getTorqueLimit(){
  byte buf[] = {currID,0x04,0x02,0x22,0x02};
  memcpy ( sendingData, buf, sizeof(buf) );  //sizeof(buf)
  sendInformation(sendingData, sizeof(buf));
  getInfo();
  return ((returningData[5]+returningData[6]*255));
}

void getInfo(){
  digitalWrite(DEPin,LOW);
  delayMicroseconds(1);              // Allow this to take effect, the SN75176 needs about 20 nanosec
  long startTime_timeOut = micros();
  long startTime2_EndOfMes = micros();
  memset(returningData, 0, sizeof(returningData));  
  int i = 0;
  boolean timedOut = true;
  boolean recivedSomething = false;
  
  //To send one byte with baud rate 500K a minimum time of 20 microsec is needed
  // We wait 1ms before calling time out, all data recived will be saved in buf and loop will break if data stops coming
  //Defualt return delay time for servos is 500microsec not sure anymore?!
  //If 100millisec between bytes servo will disregard and wait for next packet
  while ((micros()-startTime_timeOut)<3000){ 
     if (Serial2.available()){
       recivedSomething = true;
       startTime2_EndOfMes = micros();
       returningData[i++] = Serial2.read();
       
     }
     else if((recivedSomething==true) && ((micros()-startTime2_EndOfMes)>500)){ 
       timedOut = false;
       break;
     }
  }

  if(timedOut || returningData[0]!=0xFF || returningData[1]!=0xFF){
    memset(returningData, 0, sizeof(returningData));
    Serial.print("...............ERROR IN GETINFO  timed out= ");Serial.println(timedOut);
    }
}

void getInfoWithError(){
  
  digitalWrite(DEPin,LOW);
  delayMicroseconds(1);              // Allow this to take effect, the SN75176 needs about 20 nanosec
  long startTime_timeOut = micros();
  long startTime2_EndOfMes = micros();
  memset(returningData, 0, sizeof(returningData));  
  int i = 0;
  boolean timedOut = true;
  boolean recivedSomething = false;
  
  //To send one byte with baud rate 500K a minimum time of 20 microsec is needed
  // We wait 1ms before calling time out, all data recived will be saved in buf and loop will break if data stops coming
  //Defualt return delay time for servos is 500microsec not sure anymore?!
  //If 100millisec between bytes servo will disregard and wait for next packet
  while ((micros()-startTime_timeOut)<3000){ 
     if (Serial2.available()){
       recivedSomething = true;
       startTime2_EndOfMes = micros();
       returningData[i++] = Serial2.read();
       
     }
     else if((recivedSomething==true) && ((micros()-startTime2_EndOfMes)>500)){ 
       timedOut = false;
       break;
     }
  }
  
  
  
  if(timedOut || returningData[0]!=0xFF || returningData[1]!=0xFF){
    memset(returningData, 0, sizeof(returningData));
    }
  Serial.println("==============");
    Serial.print("TOut=");
    Serial.print(timedOut);
    Serial.print(", Mes=");
    for (int ii = 0; ii < i; ii++){
      Serial.print(returningData[ii],HEX);
      Serial.print(" ");
    }
    Serial.println(" ");
    Serial.println("================");
    delay(100);
}




void freeRun(boolean shouldFreerun){
  
//  byte buf[] = {currID,0x05,0x03,0x06,0,0};
//  memcpy ( sendingData, buf, sizeof(buf) );
//  sendInformation(sendingData, sizeof(buf));
    
  if (shouldFreerun==true){
    
    
    byte buf2[] = {currID,0x05,0x03,0x08,0,0};
    memcpy ( sendingData, buf2, sizeof(buf2) );
    sendInformation(sendingData, sizeof(buf2));

  }else{


    
    byte buf2[] = {currID,0x05,0x03,0x08,0xFF,0x03};
    memcpy ( sendingData, buf2, sizeof(buf2) );
    sendInformation(sendingData, sizeof(buf2));

    }
  
  }
  
 
void setSpeed1(byte LB,byte HB){
  
      byte buf[] = {currID,0x05,0x03,0x20,LB,HB};
      memcpy ( sendingData, buf, sizeof(buf) );
      sendInformation(sendingData, sizeof(buf));
  
  }

void freeRunActivate(int angledirection){
      int data  = 700 |(angledirection<<10);
      byte LB = lowByte(data);
      byte HB = highByte(data);
      
      
      delay(20);      //Needed otherwise speed is not set...
      setSpeed1(LB,HB);
  
  }



void sendInformation(byte* buf, int bufLength){
    setTrRxControlpin(HIGH);
    byte notCheckSum = calcNotChecksum(buf,bufLength);
    Serial2.write(0xFF);
    Serial2.write(0xFF);
    Serial2.write(buf, bufLength);
    Serial2.write(notCheckSum);
    Serial2.flush();
    setTrRxControlpin(LOW);
    }

void setTrRxControlpin(boolean setPin){
  digitalWrite(DEPin,setPin);         // Notify SN75176BP transciever to accept tx(HIGH) or rx(LOW)
  delayMicroseconds(50);              // Allow this to take effect, the SN75176 needs about 20 nanosec
  }


int moveServo (byte ID, int pos){
  if (checkIsBetweenValue(pos,0,1023)){
  
    byte LB = lowByte(pos);
    byte HB = highByte(pos);

    byte buf[] = {ID,0x05,0x03,0x1E,LB,HB};
    memcpy ( sendingData, buf, sizeof(buf) );
    sendInformation(sendingData, sizeof(buf));

    return 1;
  }else{
    return 0;
  }
}


void lightServo (int ID,int onOff){
  
    byte buf[] = {ID,0x04,0x03,0x19,onOff};
    memcpy ( sendingData, buf, sizeof(buf) );
    sendInformation(sendingData, sizeof(buf));
  
}

void blinkSingnal(int nrOfTimes){
  pinMode(13, OUTPUT);
  digitalWrite(13,LOW);
  for (int i = 0; i < nrOfTimes; i++){
    digitalWrite(13,HIGH);
    delay(300);
    digitalWrite(13,LOW);
    delay(300);
  }
}

boolean checkIsBetweenValue(int value, int minimum, int maximum){
  if (value >= minimum && value <= maximum){
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





