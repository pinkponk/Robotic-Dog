int DEPin = 2; // The pin to be used for enable/disable signal
const int PotPin = 0;
byte leg1_1 = 0x01;
byte leg1_2 = 0x02;
byte leg1_3 = 0x03;
byte IDAll = 0xFE;

int Vspeeds_1[3];
float angleCoord_1[3];
double cartCoord_1[3];

byte sendingData[10];
byte returningData[20];

int Ly = 55;  //55,169 mm
int Lz = 45;  //44,856 mm
int L2 = 123; //123,028 mm
int L3 = 67;  //67 mm    94,011 mm with heel

float k = 1.0;

const int nrOfPoints = 30;
int Xtransport[nrOfPoints] = {0};
int Ytransport[nrOfPoints] = {0};
int Ztransport[nrOfPoints] = {0};

int Xsupport[nrOfPoints] = {0};
int Ysupport[nrOfPoints] = {0};
int Zsupport[nrOfPoints] = {0};

int state = 3;
int currPointIndex = 0;
unsigned long programStartTime = 0;
unsigned long timeBetweenPoints = 500;
int minimumCycleTime = 20; //millisec

void setup() {
  Serial2.begin(500000);
  Serial.begin(19200);
  pinMode(13, OUTPUT); //THE LED
  pinMode(DEPin, OUTPUT);
  digitalWrite(DEPin,LOW);
  
  makeCurves();
  
  limitReturnPackage(IDAll,1);
  setDelayTime(IDAll,10);      //A value of 100 sets the delay time to 200microsek
  
  
}

void loop() {
  int valPot;
  int xtarget;
  int ytarget;
  int ztarget;
  
  switch(state){
    case 1:
  
  
      timeSyncDelay(currPointIndex);
      
      valPot = analogRead(PotPin);    // read the input pin
      k = map(valPot, 0, 1023, 0, 200)/100.0;
      
      xtarget = Xtransport[currPointIndex];
      ytarget = Ytransport[currPointIndex];
      ztarget = Ztransport[currPointIndex];
      currPointIndex++;
      
      
      calcNewSpeeds(xtarget,ytarget,ztarget,leg1_1,leg1_2,leg1_3);
      prepareSpeed(leg1_1, Vspeeds_1[0]);
      prepareSpeed(leg1_2, Vspeeds_1[1]);
      prepareSpeed(leg1_3, Vspeeds_1[2]);
      activateSpeeds();
  
      break;
   case 2:
      //Serial.println("state=2");
      //timeSyncDelay(currPointIndex);
      delay(50);
      xtarget = Xtransport[currPointIndex];
      Serial.print("xtarget=");Serial.println(xtarget);        //FEL I UTRÄKNADE KORDINATERNA
      ytarget = Ytransport[currPointIndex];
     // Serial.print("ytarget=");Serial.println(ytarget);       //FEL I UTRÄKNADE KORDINATERNA
      ztarget = Ztransport[currPointIndex];  
     // Serial.print("ztarget=");Serial.println(ztarget);        //FEL I UTRÄKNADE KORDINATERNA
      currPointIndex++;  
     // Serial.print("currPointIndex=");Serial.println(currPointIndex);
      
      InverseKinematics(xtarget, ytarget, ztarget,angleCoord_1);
     // Serial.print("angleCoord_1=");Serial.println(((int)angleCoord_1[0]*1000));
     // Serial.print("angleCoord_2=");Serial.println(((int)angleCoord_1[1]*1000));
     // Serial.print("angleCoord_3=");Serial.println(((int)angleCoord_1[2]*1000));
      
//      Serial.print("v1Move=");Serial.println(convertToServoAngle(angleCoord_1[0]));
//      Serial.print("v2Move=");Serial.println(convertToServoAngle(angleCoord_1[1]));
//      Serial.print("v3Move=");Serial.println(convertToServoAngle(angleCoord_1[2]));
      moveServo (leg1_3, convertToServoAngle(angleCoord_1[0]));
      moveServo (leg1_2, convertToServoAngle(angleCoord_1[1]));
      moveServo (leg1_1, convertToServoAngle(angleCoord_1[2]));
   
     if (currPointIndex==1){
       delay(300);  //start time to get to in position
       }
     if (currPointIndex>=nrOfPoints){
       //state = 3;
       currPointIndex = 0;
       delay(1);
       }
   
   
   
   
   
   break;
    case 3:
      delay(500);
      Serial.print("state=3");
      
      if (digitalRead(6)){
        state=2;
        delay(500);
        programStartTime = millis();
        }
      break;
    }
  
}


void timeSyncDelay(int PointIndex){
  int timeDelay = (timeBetweenPoints*PointIndex-(millis()-programStartTime));
  if (sign(timeDelay)==1){
      delay(timeDelay);
    }else if (sign(timeDelay)==-1){
      Serial.println("Going to FAST!");
      delay(1);
   }
}

void printInfo(int leg){
  if (digitalRead(7)){
  
  Serial.print("showing info for leg=");
  Serial.println(leg);
  for (int i=1; i <= 3; i++){
  int ID = i+(leg-1)*3;
  Serial.println("=================");
  int pos = getCurrPos(ID);
  int Speed = getCurrSpeed(ID);
  int SpeedValue = Speed&1023;
  int SpeedDir = Speed>>10;
  Serial.print("Pos=");Serial.println(pos);
  Serial.print("Speed=");Serial.print(SpeedValue);Serial.print("      SpeedDir=");Serial.println(SpeedDir);
  Serial.print("Voltage=");Serial.println(getVoltage(ID));
  Serial.print("CCWLimit=");Serial.print(getCCWLimit(ID)); Serial.print("      CWLimit=");Serial.println(getCWLimit(ID));
  Serial.print("K*100 Value=");Serial.println(k*100);  
    
  
  }
  }
  }


int convertToServoAngle(float radAngle){
 return map(radAngle*1000, -150*PI/180*1000, 150*PI/180*1000, 0, 1023);
  }

void calcNewSpeeds(int xtarget,int ytarget,int ztarget,int IDv1,int IDv2,int IDv3){
  
  float v1Curr = getCurrPos(IDv1);
  float v2Curr = getCurrPos(IDv2);
  float v3Curr = getCurrPos(IDv3);
  
  
  InverseKinematics(xtarget, ytarget, ztarget, angleCoord_1);
  float dv1 = angleCoord_1[0]-v1Curr;    //Still in Radians
  float dv2 = angleCoord_1[1]-v2Curr;
  float dv3 = angleCoord_1[2]-v3Curr;
  
  int v1Speed = map(dv1, -PI/4, PI/4, -1023, 1023)*k;    //Value can get higher than 1023 if the angle difference is bugger than pi/4
  int v2Speed = map(dv2, -PI/4, PI/4, -1023, 1023)*k;    //Value can get higher than 1023 if the angle difference is bugger than pi/4
  int v3Speed = map(dv3, -PI/4, PI/4, -1023, 1023)*k;    //Value can get higher than 1023 if the angle difference is bugger than pi/4


//  int v1Speed = dv1*k;
//  int v2Speed = dv2*k;
//  int v3Speed = dv3*k;
  
  Vspeeds_1[0] = v1Speed;Vspeeds_1[1] = v2Speed;Vspeeds_1[2] = v3Speed;
  }







void makeCurves(){

  int startXtransport = -100;
  int endXtransport = 100;
  float stepSize = (endXtransport-startXtransport)/((float) nrOfPoints-1);
  
   for (int i=0; i <= nrOfPoints-1; i++){
    Xtransport[i]=startXtransport+stepSize*i;
    Ytransport[i]=60+Xtransport[i]/100;
    Ztransport[i]=-150-pow((Xtransport[i]/15.0),2);
  }
  
  
}







void InverseKinematics(double x, double y, double z, float* angleCoords){
  
  float v1 = (atan(y/abs(z))+acos(Ly/sqrt(pow(z,2)+pow(y,2))))-PI/2;

  double x1 = 0;
  double y1 = Ly*cos(v1)+Lz*sin(v1);
  double z1 = Ly*sin(v1)-Lz*cos(v1);
  
  double r = sqrt(pow((x1-x),2)+pow((y1-y),2)+pow((z1-z),2));
  float v3 = PI-acos((pow(L2,2)+pow(L3,2)-pow(r,2))/(2*L2*L3));
  
  double B = abs(z-z1);
  float beta = atan(x/B/cos(v1));
  float v2 = acos((pow(r,2)+pow(L2,2)-pow(L3,2))/(2*r*L2))+beta;
  
  angleCoords[0]=v1;angleCoords[1]=v2;angleCoords[2]=v3;
  }


void Forwardkinematics(float v1,float v2,float v3, double* cartCoords){
  //Angles in Radians!
  
//  int x1 = 0;
//  int y1 = Ly*cosd(v1);
//  int z1 = Ly*sind(v1);
//  
//  int x2 = 0;
//  int y2 = y1+Lz*sind(v1);
//  int z2 = z1-Lz*cosd(v1);
//  
//  int x3 = L2*sind(v2);
//  int y3 = y2+L2*cosd(v2)*sind(v1);
//  int z3 = z2-L2*cosd(v2)*cosd(v1);
// 
//  int x4 = x3+L3*sind(v2-v3);
//  int y4 = y3+L3*cosd(v2-v3)*sind(v1);
//  int z4 = z3-L3*cosd(v2-v3)*cosd(v1);
  
  double x = double (L2*sin(v2)+L3*sin(v2-v3));
  double y = double (Ly*cos(v1)+Lz*sin(v1)+L2*cos(v2)*sin(v1)+L3*cos(v2-v3)*sin(v1));
  double z = double (Ly*sin(v1)-Lz*cos(v1)-L2*cos(v2)*cos(v1)-L3*cos(v2-v3)*cos(v1));

  cartCoords[0]=x;cartCoords[1]=y;cartCoords[2]=z;
 }


//==================================
//==================================
//==================================
//           GETS and SETS

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


int getCurrPos(byte ID){
  byte buf[] = {ID,0x04,0x02,0x24,0x02};
  memcpy ( sendingData, buf, sizeof(buf) );  //sizeof(buf)
  sendInformation(sendingData, sizeof(buf));
  getInfo();
  return (returningData[5]+returningData[6]*255);
  }


int getCurrSpeed(byte ID){
  byte buf[] = {ID,0x04,0x02,0x26,0x02};
  memcpy ( sendingData, buf, sizeof(buf) );  //sizeof(buf)
  sendInformation(sendingData, sizeof(buf));
  getInfo();
  return ((returningData[5]+returningData[6]*255));
  }
  
int getSetSpeed(byte ID){
  byte buf[] = {ID,0x04,0x02,0x20,0x02};
  memcpy ( sendingData, buf, sizeof(buf) );  //sizeof(buf)
  sendInformation(sendingData, sizeof(buf));
  getInfo();
  return ((returningData[5]+returningData[6]*255));
  }
  
int getIsMoving(byte ID){
  byte buf[] = {ID,0x04,0x02,0x2E,0x01};
  memcpy ( sendingData, buf, sizeof(buf) );  //sizeof(buf)
  sendInformation(sendingData, sizeof(buf));
  getInfo();
  return (returningData[5]);
  }
  
int getEnableTorque(byte ID){
  byte buf[] = {ID,0x04,0x02,0x18,0x01};
  memcpy ( sendingData, buf, sizeof(buf) );  //sizeof(buf)
  sendInformation(sendingData, sizeof(buf));
  getInfo();
  return (returningData[5]);
  }

void setEnableTorque(byte ID,int value){
  //if(checkIsBetweenValue(value,0, 1)){
  byte buf[] = {ID,0x04,0x03,0x18,value};
  memcpy ( sendingData, buf, sizeof(buf) );  //sizeof(buf)
  sendInformation(sendingData, sizeof(buf));
  //}
}


int getDelayTime(byte ID){
  byte buf[] = {ID,0x04,0x02,0x05,0x01};
  memcpy ( sendingData, buf, sizeof(buf) );  //sizeof(buf)
  sendInformation(sendingData, sizeof(buf));
  getInfo();
  return (returningData[5]);
  }
  
void setDelayTime(byte ID,int value){
  if(checkIsBetweenValue(value,0, 250)){
  byte buf[] = {ID,0x04,0x03,0x05,value};
  memcpy ( sendingData, buf, sizeof(buf) );  //sizeof(buf)
  sendInformation(sendingData, sizeof(buf));
  }
}
  
int getVoltage(byte ID){
  byte buf[] = {ID,0x04,0x02,0x2A,0x01};
  memcpy ( sendingData, buf, sizeof(buf) );  //sizeof(buf)
  sendInformation(sendingData, sizeof(buf));
  getInfo();
  return (returningData[5]);
  }
  
int getTemp(byte ID){
  byte buf[] = {ID,0x04,0x02,0x2B,0x01};
  memcpy ( sendingData, buf, sizeof(buf) );  //sizeof(buf)
  sendInformation(sendingData, sizeof(buf));
  getInfo();
  return (returningData[5]);
  }
  
int getStatusLvl(byte ID){
  byte buf[] = {ID,0x04,0x02,0x10,0x01};
  memcpy ( sendingData, buf, sizeof(buf) );  //sizeof(buf)
  sendInformation(sendingData, sizeof(buf));
  getInfo();
  return (returningData[5]);
}
  
  
int getCCWLimit(byte ID){
  byte buf[] = {ID,0x04,0x02,0x08,0x02};
  memcpy ( sendingData, buf, sizeof(buf) );  //sizeof(buf)
  sendInformation(sendingData, sizeof(buf));
  getInfo();
  return ((returningData[5]+returningData[6]*255));
}

int getCWLimit(byte ID){
  byte buf[] = {ID,0x04,0x02,0x06,0x02};
  memcpy ( sendingData, buf, sizeof(buf) );  //sizeof(buf)
  sendInformation(sendingData, sizeof(buf));
  getInfo();
  return ((returningData[5]+returningData[6]*255));
}

void setCCWLimit(byte ID,int value){
  if(checkIsBetweenValue(value,0, 1023)){
  byte buf[] = {ID,0x05,0x03,0x08,lowByte(value),highByte(value)};
  memcpy ( sendingData, buf, sizeof(buf) );  //sizeof(buf)
  sendInformation(sendingData, sizeof(buf));
  }
}

void setCWLimit(byte ID,int value){
  if(checkIsBetweenValue(value,0, 1023)){
  byte buf[] = {ID,0x05,0x03,0x06,lowByte(value),highByte(value)};
  memcpy ( sendingData, buf, sizeof(buf) );  //sizeof(buf)
  sendInformation(sendingData, sizeof(buf));
  }
}
  
int getLoad(byte ID){
  byte buf[] = {ID,0x04,0x02,0x28,0x02};
  memcpy ( sendingData, buf, sizeof(buf) );  //sizeof(buf)
  sendInformation(sendingData, sizeof(buf));
  getInfo();
  return ((returningData[5]+returningData[6]*255));
  }



//===================================
//===================================
//===================================

void prepareSpeed(int ID, int vspeed){
     int targetAngle = 0;
     if (sign(vspeed)){
        targetAngle = 1023;
     }
     vspeed = abs(vspeed);    
     moveServoRegWrite (ID, targetAngle, vspeed); 
  }
  
void activateSpeeds(){
  
    byte buf[] = {0xFE,0x02,0x05};    //Broadcasting ID, exicute all saved commands
    memcpy ( sendingData, buf, sizeof(buf) );
    sendInformation(sendingData, sizeof(buf));
  }
  


int moveServoRegWrite (byte ID, int pos, int Speed){
  if (checkIsBetweenValue(pos,0,1023)){
    if (checkIsBetweenValue(Speed,0,1023)){
      byte LBpos = lowByte(pos);
      byte HBpos = highByte(pos);
      byte LBspeed = lowByte(Speed);
      byte HBspeed = highByte(Speed);
  
  
      byte buf[] = {ID,0x07,0x04,0x1E,LBpos,HBpos,LBspeed,HBspeed};
      memcpy ( sendingData, buf, sizeof(buf) );
      sendInformation(sendingData, sizeof(buf));
  
      return 1;
    }else{
      return 0;
    }
  }
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


void getInfo(){
  setTrRxControlpin(LOW);
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

int sign(int number){
  if (number>0) {return 1;}
  else if (number<0) {return -1;}
  else {return 0;}
  }
