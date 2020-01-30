//==== Pins   ============   ============
int DEPin = 2; // The pin to be used for enable/disable signal
const int PotPin = 0;
const int sensorPinX = A14;    // select the input pin for the potentiometer
const int sensorPinY = A15;    // select the input pin for the potentiometer
const int buttonPin = 51;      //pin 53 does not work
const int VpwrPin2 = 50;    // select the input pin for the potentiometer

//==== Constants   ============   ============
byte IDAll = 0xFE;

int Ly = 55;  //55,169 mm
int Lz = 45;  //44,856 mm
int L2 = 123; //123,028 mm
int L3 = 67+20;  //67 mm    94,011 mm with heel

float k;
int k_multi = 2;
int state = 3;
int DoF3 = 1;
unsigned long lastPointIncrement;
unsigned long previousSwitchTime;
int timeBetweenPoints; //Millisec
int MaxTimeBetweenPoints = 400;//400; //Millisec
const int nrOfPointsT = 8;
const int nrOfPointsS = nrOfPointsT*3;
const int nrOfPoints = nrOfPointsT+nrOfPointsS;


//===== Input and Control Variables   ============
int buttonState;
int sensorValueX = 0;  // variable to store the value coming from the sensor
int sensorValueY = 0;  // variable to store the value coming from the sensor

//=====   LEG 1   ============ Right Front Leg
byte leg1_3 = 0x01; int leg1_3_dir=1; int leg1_3_offset=-20;
byte leg1_2 = 0x02; int leg1_2_dir=1; int leg1_2_offset=60;
byte leg1_1 = 0x03; int leg1_1_dir=1; int leg1_1_offset=0;
float phaseShift_1 = 0.25;
int currPointIndex_1 = nrOfPoints*phaseShift_1;
int Curve_1[nrOfPoints][3];

int Vspeeds_1[3];
float angleCoord_1[3];
double cartCoord_1[3];

//=====   LEG 2   ============ Left Front Leg
byte leg2_3 = 0x04; int leg2_3_dir=1; int leg2_3_offset=20;
byte leg2_2 = 0x05; int leg2_2_dir=1; int leg2_2_offset=-35;
byte leg2_1 = 0x06; int leg2_1_dir=1; int leg2_1_offset=-30;
float phaseShift_2 = 0.75;
int currPointIndex_2 = nrOfPoints*phaseShift_2;
int Curve_2[nrOfPoints][3] = {0};

int Vspeeds_2[3];
float angleCoord_2[3];
double cartCoord_2[3];

//=====   LEG 3   ============ Right Back Leg
byte leg3_3 = 0x07; int leg3_3_dir=1; int leg3_3_offset=30;
byte leg3_2 = 0x08; int leg3_2_dir=1; int leg3_2_offset=-30;
byte leg3_1 = 0x09; int leg3_1_dir=1; int leg3_1_offset=40;
float phaseShift_3 = 0;
int currPointIndex_3 = nrOfPoints*phaseShift_3;
int Curve_3[nrOfPoints][3] = {0};

int Vspeeds_3[3];
float angleCoord_3[3];
double cartCoord_3[3];

//=====   LEG 4   ============ Left Back Leg
byte leg4_3 = 10; int leg4_3_dir=1; int leg4_3_offset=0;
byte leg4_2 = 11; int leg4_2_dir=1; int leg4_2_offset=-15;
byte leg4_1 = 12; int leg4_1_dir=1; int leg4_1_offset=-15;
float phaseShift_4 = 0.5;
int currPointIndex_4 = nrOfPoints*phaseShift_4;
int Curve_4[nrOfPoints][3] = {0};

int Vspeeds_4[3];
float angleCoord_4[3];
double cartCoord_4[3];


//==== Temp and Log Variables   ============   ============
int i=0;
int powerConsumpiton[10] ={0};
byte sendingData[10];
byte returningData[20];
int corupptReturnData=0;



void setup() {
  Serial2.begin(500000);
  Serial.begin(19200);
  pinMode(VpwrPin2, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);    //Pull up
  digitalWrite(VpwrPin2, HIGH);
  
  pinMode(DEPin, OUTPUT);
  digitalWrite(DEPin,LOW);
  
  //makeCurve(Curve_1, 1, -1, -1, 1);
  //makeCurve(Curve_2);
  limitReturnPackage(IDAll,1);
  setDelayTime(IDAll,10);      //A value of 100 sets the delay time to 200microsek
  setMaxTorque(IDAll,1023);      //To try to solve the pwr problem
  delay(8);              //PWR PROBLEM Need this in order for all the servos to actually move to 512, don't not yet why but could have to do with power to the servos or the command above
   moveServo (IDAll, 512);
   delay(80);

  
  previousSwitchTime=millis();
  lastPointIncrement = millis();
}

void loop() {
  int valPot;
  int xtarget_1;
  int ytarget_1;
  int ztarget_1;
  
  int xtarget_2;
  int ytarget_2;
  int ztarget_2;
  
  int xtarget_3;
  int ytarget_3;
  int ztarget_3;
  
  int xtarget_4;
  int ytarget_4;
  int ztarget_4;
  float turn;
  

  switch(state){
    case 1:
    delay(10);
      buttonState = digitalRead(buttonPin);
      valPot = analogRead(PotPin);    // read the input pin
      k = map(valPot, 0, 1023, 0, 200*k_multi)/100.0;
      sensorValueX = analogRead(sensorPinX);
      sensorValueY = analogRead(sensorPinY);
      timeBetweenPoints = map(sensorValueX, 0, 1023, 0, MaxTimeBetweenPoints);
      turn = map(sensorValueY, 0, 1023, -100, 100)/100.0;
      
       if (buttonState==LOW && (millis()-previousSwitchTime)>100){
        DoF3 = (DoF3+1)%2;
        previousSwitchTime = millis();
       }
      
      makeCurve(Curve_1, abs(turn), sign(((int)(turn*100))), 1,DoF3);
      makeCurve(Curve_2, abs(turn), sign(((int)(turn*100))), -1,DoF3);
      
      makeCurve(Curve_3, abs(turn), sign(((int)(turn*100))), 1,DoF3);
      makeCurve(Curve_4, abs(turn), sign(((int)(turn*100))), -1,DoF3);
      
      timeSyncNextPoint();

//        Serial.write(27);       // ESC command
//        Serial.print("[2J");    // clear screen command
//        Serial.write(27);
//        Serial.print("[H");     // cursor to home command
//      Serial.print("CurrPoint=");Serial.println(currPointIndex_1);
//      Serial.print("sensorValueY=");Serial.println(sensorValueY);
//      Serial.print("timeBetweenPoints=");Serial.println(timeBetweenPoints);
//      Serial.print("Load ID 2=");Serial.println(getLoad(2));
//      printInfo(1);
      
      //========LEG 1=========
      xtarget_1 = Curve_1[currPointIndex_1%nrOfPoints][0];
      ytarget_1 = Curve_1[currPointIndex_1%nrOfPoints][1];
      ztarget_1 = Curve_1[currPointIndex_1%nrOfPoints][2];
      
//      InverseKinematics(xtarget_1, ytarget_1, ztarget_1,angleCoord_1);
//      moveServo (leg1_1, convertToServoAngle(angleCoord_1[0])+leg1_1_offset);
//      moveServo (leg1_2, convertToServoAngle(angleCoord_1[1])+leg1_2_offset);
//      moveServo (leg1_3, convertToServoAngle(angleCoord_1[2])+leg1_3_offset);

      
      calcNewSpeeds(xtarget_1,ytarget_1,ztarget_1,leg1_1,leg1_2,leg1_3,Vspeeds_1, angleCoord_1,1,  leg1_1_offset, leg1_2_offset, leg1_3_offset);
      prepareSpeed(leg1_1, Vspeeds_1[0]*leg1_1_dir);
      prepareSpeed(leg1_2, Vspeeds_1[1]*leg1_2_dir);
      prepareSpeed(leg1_3, Vspeeds_1[2]*leg1_3_dir);

      
      //========LEG 2=========      
      xtarget_2 = Curve_2[currPointIndex_2%nrOfPoints][0];
      ytarget_2 = Curve_2[currPointIndex_2%nrOfPoints][1];
      ztarget_2 = Curve_2[currPointIndex_2%nrOfPoints][2];
      
//      InverseKinematics(xtarget_2, ytarget_2, ztarget_2,angleCoord_2);
//      moveServo (leg2_1, convertToServoAngle(angleCoord_2[0])+leg2_1_offset);
//      moveServo (leg2_2, convertToServoAngle(angleCoord_2[1])+leg2_2_offset);
//      moveServo (leg2_3, convertToServoAngle(angleCoord_2[2])+leg2_3_offset);
      
      calcNewSpeeds(xtarget_2,ytarget_2,ztarget_2,leg2_1,leg2_2,leg2_3,Vspeeds_2, angleCoord_2,-1,  leg2_1_offset, leg2_2_offset, leg2_3_offset);
      prepareSpeed(leg2_1, Vspeeds_2[0]*leg2_1_dir);
      prepareSpeed(leg2_2, Vspeeds_2[1]*leg2_2_dir);
      prepareSpeed(leg2_3, Vspeeds_2[2]*leg2_3_dir);
      
      //========LEG 3=========      
      xtarget_3 = Curve_3[currPointIndex_3%nrOfPoints][0];
      ytarget_3 = Curve_3[currPointIndex_3%nrOfPoints][1];
      ztarget_3 = Curve_3[currPointIndex_3%nrOfPoints][2];
      
//      InverseKinematics(xtarget_3, ytarget_3, ztarget_3,angleCoord_3);
//      moveServo (leg3_1, convertToServoAngle(angleCoord_3[0])+leg3_1_offset);
//      moveServo (leg3_2, convertToServoAngle(angleCoord_3[1])+leg3_2_offset);
//      moveServo (leg3_3, convertToServoAngle(angleCoord_3[2])+leg3_3_offset);
      
      calcNewSpeeds(xtarget_3,ytarget_3,ztarget_3,leg3_1,leg3_2,leg3_3,Vspeeds_3, angleCoord_3,1,  leg3_1_offset, leg3_2_offset, leg3_3_offset);
      prepareSpeed(leg3_1, Vspeeds_3[0]*leg3_1_dir);
      prepareSpeed(leg3_2, Vspeeds_3[1]*leg3_2_dir);
      prepareSpeed(leg3_3, Vspeeds_3[2]*leg3_3_dir);
      
      //========LEG 4=========      
      xtarget_4 = Curve_4[currPointIndex_4%nrOfPoints][0];
      ytarget_4 = Curve_4[currPointIndex_4%nrOfPoints][1];
      ztarget_4 = Curve_4[currPointIndex_4%nrOfPoints][2];
      
//      InverseKinematics(xtarget_4, ytarget_4, ztarget_4,angleCoord_4);
//      moveServo (leg4_1, convertToServoAngle(angleCoord_4[0])+leg4_1_offset);
//      moveServo (leg4_2, convertToServoAngle(angleCoord_4[1])+leg4_2_offset);
//      moveServo (leg4_3, convertToServoAngle(angleCoord_4[2])+leg4_3_offset);
      
      calcNewSpeeds(xtarget_4,ytarget_4,ztarget_4,leg4_1,leg4_2,leg4_3,Vspeeds_4, angleCoord_4,-1,  leg4_1_offset, leg4_2_offset, leg4_3_offset);
      prepareSpeed(leg4_1, Vspeeds_4[0]*leg4_1_dir);
      prepareSpeed(leg4_2, Vspeeds_4[1]*leg4_2_dir);
      prepareSpeed(leg4_3, Vspeeds_4[2]*leg4_3_dir);
      
      //====ACTIVATE====
      activateSpeeds();
      
      
      
//      Serial.println("%%%%%%%%%%%%%%%%%%%%%%");
////      printInfo(1);
//      Serial.print("valPot=");Serial.print(valPot);
//      Serial.print("  k=");printFloat(k, 3);
//      Serial.print("  currPointIndex=");Serial.println(currPointIndex_1);
//
//      
//                  Serial.print("   xtarget=");Serial.print(xtarget_1);
//                  Serial.print("         ytarget=");Serial.print(ytarget_1);
//                  Serial.print("            ztarget=");Serial.println(ztarget_1);        //FEL I UTRÄKNADE KORDINATERNA <-vet inte längre
////            
////
////      
////      
////
//                  Serial.print("v1Speed=");Serial.println(Vspeeds_1[0]);
//                  Serial.print("v2Speed=");Serial.println(Vspeeds_1[1]);
//                  Serial.print("v3Speed=");Serial.println(Vspeeds_1[2]);
      
  
  
      break;
    
    case 3:
        delay(50);
        goToZero();
      if (digitalRead(6)){
        state=1;
        lastPointIncrement = millis();
        }
      break;
    }
  
}

void goToZero(){
  

  moveServo(leg1_1, 512+leg1_1_offset);
  moveServo(leg1_2, 512+leg1_2_offset);
  moveServo(leg1_3, 512+leg1_3_offset);
  
  moveServo(leg2_1, 512+leg2_1_offset);
  moveServo(leg2_2, 512+leg2_2_offset);
  moveServo(leg2_3, 512+leg2_3_offset);
  
  moveServo(leg3_1, 512+leg3_1_offset);
  moveServo(leg3_2, 512+leg3_2_offset);
  moveServo(leg3_3, 512+leg3_3_offset);
  
  moveServo(leg4_1, 512+leg4_1_offset);
  moveServo(leg4_2, 512+leg4_2_offset);
  moveServo(leg4_3, 512+leg4_3_offset);
  
  
  
  }

void timeSyncNextPoint(){
  int timeDelay = (timeBetweenPoints-(millis()-lastPointIncrement));
  if (timeDelay<=0){
      currPointIndex_1++;
      currPointIndex_2++;
      currPointIndex_3++;
      currPointIndex_4++;
      //(*PointIndexPointer)++;    //(*PointIndexPointer)++; The parantasis are most vital otherwise the pointer will be ++ first
      //getLoad(1);
      
//      powerConsumpiton[i%10]=0;
//      for (int ii = 1; ii < 5; ii++) {
//      powerConsumpiton[i%10] = powerConsumpiton[i%10]+getLoad(ii)%1023;
//       delay(1);     
//    }
//      int meanPwr=0;
//      for (int ii = 0; ii < 10; ii++) {
//       meanPwr = meanPwr+powerConsumpiton[ii]/5;
//       } 
//      Serial.print("pwrcons=");Serial.println(meanPwr/10);
//      i++;
      
      lastPointIncrement=millis();
   }
}

void printInfo(int leg){
  
  
  Serial.print("showing info for leg=");
  Serial.println(leg);
  for (int i=1; i <= 3; i++){
  int ID = i+(leg-1)*3;
  Serial.println("=================");
  int pos = getCurrPos(ID);
  int Speed = getCurrSpeed(ID);
  int SpeedValue = Speed&1023;
  int SpeedDir = Speed>>10;
  Serial.print("ID=");Serial.print(ID);
  Serial.print("  Pos=");Serial.print(pos);
  Serial.print("  Speed=");Serial.print(SpeedValue);Serial.print("  SpeedDir=");Serial.print(SpeedDir);
  Serial.print("  Voltage=");Serial.print(getVoltage(ID));
  Serial.print("  CCWLimit=");Serial.print(getCCWLimit(ID)); Serial.print("  CWLimit=");Serial.println(getCWLimit(ID));    
  
  }
  Serial.println("=================");
  }


int convertToServoAngle(float radAngle){
  //WORKS
 return map(radAngle*1000, -2.618*1000, 2.618*1000, 0, 1023);    //2.618=150*PI/180
  }


float convertFromServoAngle(int bitAngle){
  //WORKS
 return map(bitAngle, 0, 1023, -2.618*1000, 2.618*1000)/1000.0;  //2.618=150*PI/180
  }


void calcNewSpeeds(int xtarget,int ytarget,int ztarget,int IDv1,int IDv2,int IDv3, int Vspeeds[3], float* angleCoord, int leftOrRight, int leg_1_offset, int leg_2_offset, int leg_3_offset){
  //leftOrRight [-1 | 1], -1=left, 1=right
  
  
  int v1Pos = getCurrPos(IDv1)-leg_1_offset;if(corupptReturnData==1){return;}
  int v2Pos = getCurrPos(IDv2)-leg_2_offset;if(corupptReturnData==1){return;}
  int v3Pos = getCurrPos(IDv3)-leg_3_offset;if(corupptReturnData==1){return;}
  
  float v1Curr = convertFromServoAngle(v1Pos)*leftOrRight;
  float v2Curr = convertFromServoAngle(v2Pos)*leftOrRight;
  float v3Curr = convertFromServoAngle(v3Pos)*leftOrRight;
//  Serial.print("deg v1Curr=");printFloat(v1Curr*180/PI, 3);
//  Serial.print("    deg v2Curr=");printFloat(v2Curr*180/PI, 3);
//  Serial.print("    deg v3Curr=");printFloat(v3Curr*180/PI, 3);Serial.println();  
  
  InverseKinematics(xtarget, ytarget, ztarget, angleCoord);
  float dv1 = angleCoord[0]-v1Curr;    //Still in Radians
  float dv2 = angleCoord[1]-v2Curr;
  float dv3 = angleCoord[2]-v3Curr;
//  Serial.print("deg target v1=");printFloat(convertToServoAngle(angleCoord[0]), 3);
//  Serial.print("    deg target v2=");printFloat(convertToServoAngle(angleCoord[1]), 3);
//  Serial.print("    deg target v3=");printFloat(convertToServoAngle(angleCoord[2]), 3);Serial.println();
//  
//  Serial.print("   deg dv1=");printFloat(dv1*180/PI, 3);
//  Serial.print("        deg dv2=");printFloat(dv2*180/PI, 3);
//  Serial.print("        deg dv3=");printFloat(dv3*180/PI, 3);Serial.println();

  
  int v1Speed = map( dv1*1000,(-PI/2.0)*1000,(PI/2.0)*1000, -1023, 1023)*k*leftOrRight;    //Value can get higher than 1023 if the angle difference is bugger than pi/4
  int v2Speed = map( dv2*1000,(-PI/2.0)*1000,(PI/2.0)*1000, -1023, 1023)*k*leftOrRight;    //Value can get higher than 1023 if the angle difference is bugger than pi/4
  int v3Speed = map( dv3*1000,(-PI/2.0)*1000,(PI/2.0)*1000, -1023, 1023)*k*leftOrRight;    //Value can get higher than 1023 if the angle difference is bugger than pi/4

//  Serial.print("   v1Speed=");printFloat(v1Speed, 3);
//  Serial.print("        v1Speed=");printFloat(v2Speed, 3);
//  Serial.print("        v1Speed=");printFloat(v3Speed, 3);Serial.println();


//  int v1Speed = dv1*k;
//  int v2Speed = dv2*k;
//  int v3Speed = dv3*k;
  
  Vspeeds[0] = v1Speed;Vspeeds[1] = v2Speed;Vspeeds[2] = v3Speed;
  }



void makeCurve(int curve[][3], float turn, int turningLeftOrRight, int leftOrRightLeg, int DoF3){
  //turn = [0->1], how much you want to turn
  //turningLeftOrRight = [-1 | 1], -1 = turn left, 1 = turn right
  //leftOrRightLeg = [-1 | 1], -1 = left leg, 1 = right leg
  
  
  if (DoF3 == 1){
    
  //3 DOF LEG MOTION
//  int startXtransport = -110;
//  int endXtransport = 110;
  int startXtransport = -60;
  int endXtransport = 60;
  int yOffsetTurn = 60*turn*turningLeftOrRight*leftOrRightLeg;
  float stepSizeX = (endXtransport-startXtransport)/(nrOfPoints/4.0-1);
  float stepSizeY = (yOffsetTurn-0)/(nrOfPoints/4.0-1);
  
   for (int i=0; i < (nrOfPoints/4); i++){
    curve[i][0]=startXtransport+stepSizeX*i;
//    curve[i][1]=60-pow((curve[i][0]/40),2)+8+stepSizeY*i-yOffsetTurn/2;    //added this remember that, 21:34
//    curve[i][2]=-145-pow((curve[i][0]/20.0),2);
    curve[i][1]=0+stepSizeY*i-yOffsetTurn/2;    //added this remember that, 21:34 -34 = very close feet
    curve[i][2]=-115-70-0-pow(((curve[i][0]-0)/8.0),2);
  }
  
  stepSizeX = (startXtransport-endXtransport)/(nrOfPoints/(3/4.0)-1);
  stepSizeY = (curve[0][1]-curve[nrOfPoints/4-1][1])/(nrOfPoints/(3/4.0)-1);
  int z = curve[0][2];
  
    for (int i=nrOfPoints/4; i < (nrOfPoints); i++){
    curve[i][0]=curve[nrOfPoints/4-1][0]+stepSizeX*(i-nrOfPoints/4);
    curve[i][1]=curve[nrOfPoints/4-1][1]+stepSizeY*(i-nrOfPoints/4);
    curve[i][2]=z;
  }
  }else{
    
    
  //2 DOF LEG MOTION
  //int xOffsetTurn = turn*turningLeftOrRight*leftOrRightLeg;        //Turning is done by offsetting the x so that an offset in y is created
  int startXtransport = -60;//-xOffsetTurn*10;
  int endXtransport = 60;//-xOffsetTurn*10;
  float stepSizeX = (endXtransport-startXtransport)/(nrOfPoints/2.0-1);
  
   for (int i=0; i < (nrOfPoints/2); i++){
    curve[i][0]=startXtransport+stepSizeX*i;    //x
    curve[i][2]=-218-pow((curve[i][0]/10.0),2);  //z
    curve[i][1]=sqrt(pow(Ly,2)+pow((sqrt(pow((L2+L3),2)-pow(curve[i][0],2))+Lz),2)-pow(curve[i][2],2))-1;    //The one is for not getting imaginary when inverse kinematics

  }
  
  stepSizeX = (startXtransport-endXtransport)/(nrOfPoints/2.0-1);
  int z = curve[0][2];
  
    for (int i=nrOfPoints/2; i < (nrOfPoints); i++){
    curve[i][0]=curve[nrOfPoints/2-1][0]+stepSizeX*(i-nrOfPoints/2);
    curve[i][1]=-(sqrt(pow(Ly,2)+pow((sqrt(pow((L2+L3),2)-pow(curve[i][0],2))+Lz),2)-pow(curve[i][2],2))-1);    //The one is for not getting imaginary when inverse kinematics
    //curve[i][2]=z;
    curve[i][2]=-253;
  }   
    
    
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

void setMaxTorque(byte ID,int value){
  if(checkIsBetweenValue(value,0, 1023)){
  byte buf[] = {ID,0x05,0x03,0x0E,lowByte(value),highByte(value)};
  memcpy ( sendingData, buf, sizeof(buf) );  //sizeof(buf)
  sendInformation(sendingData, sizeof(buf));
  }else{
    Serial.print("Error in setMaxTorque");}
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
  
int getLoad(byte ID){  //To get the load takes a bit of time for the servo I think
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
     if (vspeed!=0){
       
     int targetAngle = 0;      
     if (sign(vspeed)==1){      //NEED TO EXPLICITY SAY ==1 and not just if(x)
        targetAngle = 1023;
     }
     if (abs(vspeed)>1023){
       vspeed = 1023*sign(vspeed);
       }
       
     vspeed = abs(vspeed);    
     moveServoRegWrite (ID, targetAngle, vspeed); 
     }
  }
  
void activateSpeeds(){
  
    byte buf[] = {0xFE,0x02,0x05};    //Broadcasting ID, exicute all saved commands
    memcpy ( sendingData, buf, sizeof(buf) );
    sendInformation(sendingData, sizeof(buf));
  }
  


void moveServoRegWrite (byte ID, int pos, int Speed){
  if (checkIsBetweenValue(pos,0,1023)){
    if (checkIsBetweenValue(Speed,0,1023)){
      byte LBpos = lowByte(pos);
      byte HBpos = highByte(pos);
      byte LBspeed = lowByte(Speed);
      byte HBspeed = highByte(Speed);
  
  
      byte buf[] = {ID,0x07,0x04,0x1E,LBpos,HBpos,LBspeed,HBspeed};
      memcpy ( sendingData, buf, sizeof(buf) );
      sendInformation(sendingData, sizeof(buf));
  
    }else{
      Serial.println("moveServoRegWrite error1");
    }
  }else{
    Serial.println("moveServoRegWrite error2");
    }
}

int moveServo (byte ID, int pos){ 
    byte LB = lowByte(pos);
    byte HB = highByte(pos);

    byte buf[] = {ID,0x05,0x03,0x1E,LB,HB};
    memcpy ( sendingData, buf, sizeof(buf) );
    sendInformation(sendingData, sizeof(buf));
}


void getInfo(){
  setTrRxControlpin(LOW);
  long startTime_timeOut = micros();
  long startTime2_EndOfMes = micros();
  corupptReturnData = 0;
  memset(returningData, 0, sizeof(returningData));  
  int i = 0;
  boolean timedOut = true;
  boolean recivedSomething = false;
  
  //To send one byte with baud rate 500K a minimum time of 20 microsec is needed
  // We wait 1ms before calling time out, all data recived will be saved in buf and loop will break if data stops coming
  //Defualt return delay time for servos is 500microsec not sure anymore?!
  //If 100millisec between bytes servo will disregard and wait for next packet
  while ((micros()-startTime_timeOut)<1000){ 
     if (Serial2.available()){
       recivedSomething = true;
       startTime2_EndOfMes = micros();
       returningData[i++] = Serial2.read();
       
     }
     else if((recivedSomething==true) && ((micros()-startTime2_EndOfMes)>200)){ //Kan bli mycket snabbare 500 för stort
       timedOut = false;
       break;
     }
  }

  if(timedOut || returningData[0]!=0xFF || returningData[1]!=0xFF){
    memset(returningData, 0, sizeof(returningData));
    corupptReturnData = 1;
    Serial.print("ERROR IN GETINFO  timed out= ");Serial.println(timedOut);
    delay(1);    //wait for trash data
    while(Serial.available()){
      Serial.read();}
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
  
  
  
  
  // printFloat prints out the float 'value' rounded to 'places' places after the decimal point
void printFloat(float value, int places) {
  int digit;
  float tens = 0.1;
  int tenscount = 0;
  int i;
  float tempfloat = value;
  float d = 0.5;
  if (value < 0)
    d *= -1.0;
  for (i = 0; i < places; i++)
    d/= 10.0;    
  tempfloat +=  d;
  if (value < 0)
    tempfloat *= -1.0;
  while ((tens * 10.0) <= tempfloat) {
    tens *= 10.0;
    tenscount += 1;
  }
  if (value < 0)
    Serial.print('-');

  if (tenscount == 0)
    Serial.print(0, DEC);
  for (i=0; i< tenscount; i++) {
    digit = (int) (tempfloat/tens);
    Serial.print(digit, DEC);
    tempfloat = tempfloat - ((float)digit * tens);
    tens /= 10.0;
  }
  if (places <= 0)
    return;
  Serial.print('.');  
  for (i = 0; i < places; i++) {
    tempfloat *= 10.0; 
    digit = (int) tempfloat;
    Serial.print(digit,DEC);  
    // once written, subtract off that digit
    tempfloat = tempfloat - (float) digit; 
  }
}
