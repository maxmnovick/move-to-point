/*  Move to Point
 *  Input (x, y) coordinates to move to that location. 
 */

#include "TM1651.h"

//======Advisor======
const int numEncoders = 2;

int eStop, // global flag to halt robot
  setPosActive,
  setVelActive;

bool bDsplyOn;

volatile bool newPress = 0,
  validPress,
  bDsplyEnabled;
  
volatile bool newPulses[] = { 0, 0 },
  validPulses[numEncoders];

//======Organizer======
int numWpts = 5,
  btnBounceTime;

int wp[][3] = 
{
  { 0, 0, 0 },
  { 1000, 0, 135 },
  { 1000, 1000, 225 },
  { 0, 1000, 315 },
  { 0, 0, 0 }
};

long vMax = 7500, // [mV]
  vMin, // [mV]
  blinkV;

unsigned long prevPressTime;

//======Interface======
volatile String inputString = "";

volatile boolean stringComplete = 0;  // whether the string is complete

const byte numBLevels = 5, // no. discrete batt levels
  // Pins: 
  bDsplyDIOPin = 10,
  bDsplyClkPin = 11,
  batPin = 15,
  btnPin = 21;

byte bLevel; // [0-5]

long vInput;

TM1651 batteryDisplay(bDsplyClkPin, bDsplyDIOPin);
  
//======Encoders======
const byte esPins[] = 
{
  2, // encoder signal 1 pin
  3 // encoder signal 2 pin
};

const byte pulsesPerRev = 20;

double minAngularRes; // [deg/pulse]

volatile int angPos[numEncoders],
  prevAngPos[numEncoders],
  setPos[numEncoders];

volatile long pulseCounts[numEncoders];

//======Motor Driver======
const byte mSigPins[] = { 8, 9 },
  mEnablePins[] = { 6, 7 };

//======Mobile Platform======
int wheelDiam = 64, // [mm]
  botDiam = 138; // [mm] (i.e. wheel base or distance between wheel centers

//======Circle======
float piApprox = 3.14159,
  degsPerRad = 57.2958; // radians to deg conversion

//======Controller======
int sensorsTmrCtr,
  maxOutVal,
  pubVelRate,
  posDeadZone,
  navDeadZone,
  minStaticDutyCycle,
  minStaticMtrCmd,
  minKineticDutyCycle,
  minKineticMtrCmd,
  fwdPosErr, // [mm]
  setFwdPos, // [mm]
  fwdPos, // [mm]
  yawPosErr,
  setYawPos, // [deg]
  yawPos, // [deg]
  setRadius, // [mm]
  setX, // [mm]
  measDX,
  measX,
  xErr,
  setY,
  measDY,
  measY,
  yErr,
  wpNum;

unsigned long prevSenseTime;

unsigned long curTimes[numEncoders];

long prevPulseTimes[numEncoders];

float minLinearRes;

bool newPos,
  newVel = 0,
  turned = 0,
  drove = 0,
  manual;

volatile bool decelerating[] = { 0, 0 };

volatile float kp[numEncoders], ki[numEncoders], kd[numEncoders];

volatile int topAngVel, // [deg/s]
  setFwdVel, // [mm/s]
  setYawVel, // [deg/s]
  cmdFwdVel,
  cmdYawVel;

volatile int prevAngVels[numEncoders],
  angVels[numEncoders],
  setAngVels[numEncoders], // [deg/s]
  cmdAngVels[numEncoders], // [deg/s]
  cmdPrcntMtrVltges[numEncoders],
  prevCmdPMV[numEncoders],
  pubMtrCmds[numEncoders],
  signs[numEncoders],
  //setPrcntMtrVltge,
  pulses[numEncoders],
  prevPulses[numEncoders],
  prevPosErrs[numEncoders],
  posErrs[numEncoders],
  prevVelErrs[numEncoders],
  velErrs[numEncoders],
  bounceTimes[numEncoders];

volatile long instance;

volatile long samples[numEncoders],
  prevSamples[numEncoders];

//======Main======
void setup() 
{
  initSystem();

  initBehaviors();

  //initSensorsTimer(); // CHANGE to scheduler(); ADD: publish rotational (and later translational) angVel
}

int initSystem()
{
  initNode("RobotControlTest8");

  initSubscribers();

  initPublishers();
  
  return 0;
}

void initNode(String id)
{
  Serial.begin(115200);

  while(!Serial);
  
  Serial.print("Starting ");
  Serial.print(id);
  Serial.println(".ino\n");

  displayCommands();

  Serial.println("Inst\tSetFP\tActFP\tSetX\tMeasX\tXErr\tSetY\tMeasY\tYErr\tFPErr\tSetYP\tActYP\tYPErr\tTime1\tSamp1\tPuls1\tSetV1\tCmdV1\tActV1\tVErr1\tCmdPMV1\tMtrCmd1\tTime2\tSamp2\tPuls2\tSetV2\tCmdV2\tActV2\tVErr2\tCmdPMV2\tMtrCmd2");
}

void displayCommands()
{
  Serial.println("======User Commands======");
  Serial.println("s: stop moving (i.e. emergency stop robot)");
  Serial.println("g: start moving to next target");
  Serial.println("fdx: move forward x mm");
  Serial.println("tdx: turn x deg");
  Serial.println("fvx: move forward x mm/s");
  Serial.println("tvx: turn x deg/s");
  Serial.println("xx: set x coordinate to x mm");
  Serial.println("yx: set y coordinate to x mm");
  Serial.println("px: set target position to point x");
  Serial.println("h: home to coordinates of previous target");
  Serial.println("b: battery display on/off");
//  Serial.println("ha: home to angle facing target");
//  Serial.println("hax: home to angle x");
  Serial.println();
}

void initSubscribers()
{
  // pulse count
  /* Interrupt flags can be set before you attach the 
   * interrupt handler. To avoid this, manually clear 
   * the flag. 
   */
  //EIFR = bit (INTF4); // clear flag for interrupt 4
  //EIFR = bit (INTF5); // clear flag for interrupt 5
  
  pinMode(btnPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(btnPin), btnPressed, FALLING);
  
  attachInterrupt(digitalPinToInterrupt(esPins[0]), encoder1Callback, CHANGE);
  attachInterrupt(digitalPinToInterrupt(esPins[1]), encoder2Callback, CHANGE);

  //Serial.println("Subscribers Inititialized");
}

void initPublishers()
{
  /* Start Motor Channel */
  for(int i=0; i < numEncoders; i++)
  {
    pinMode(mEnablePins[i], OUTPUT);
    pinMode(mSigPins[i], OUTPUT);
  }

  /* Start Battery Capacity Display */
  batteryDisplay.init();
//  Serial.println("Battery Display Inititialized");

  batteryDisplay.set(BRIGHTEST);//BRIGHT_TYPICAL = 2,BRIGHT_DARKEST = 0,BRIGHTEST = 7;

//  Serial.println("Publishers Inititialized");
}

void initBehaviors()
{
  initVars();

  setParams();

  //Serial.println("Behaviors Inititialized");
}

void initVars()
{ 
  //======Interface======
  btnBounceTime = 250; // [ms]
  
  //===Controller===
  topAngVel = 720; // [deg/s]; 720 deg/s = highest value encountered in tests
  instance = 0;
  prevSenseTime = 0;
  fwdPos = 0;
  setFwdPos = 0;
  fwdPosErr = 0;
  yawPos = 0;
  setYawPos = 0;
  yawPosErr = 0;
  
  wpNum = 0;

  setX = 0; // [mm]
  measX = 0; // [mm]
  measDX = 0;
  xErr = 0;
  
  setY = 0; // [mm]
  measY = 0; // [mm]
  measDY = 0;
  yErr = 0;
  
  for(int i=0; i < numEncoders; i++)
  {
    //===Encoder===
    pulseCounts[i] = 0;
    
    bounceTimes[i] = 50; // [ms]
    validPulses[i] = true;
  
    prevAngPos[i] = 0;
    angPos[i] = 0;
  
    prevAngVels[i] = 0;
    angVels[i] = 0;

    prevPulseTimes[i] = 0; // [ms]
    curTimes[i] = 0;

    //===Controller===
    setAngVels[i] = 0; // [deg/s]
    cmdAngVels[i] = 0; // [deg/s]
    prevVelErrs[i] = 0;
    velErrs[i] = 0;
  
    setPos[i] = 0;
    prevPosErrs[i] = 0;
    posErrs[i] = 0;
  
    cmdPrcntMtrVltges[i] = 0;
    prevCmdPMV[i] = 0;
    pubMtrCmds[i] = 0;
    
    signs[i] = 1;
  
    prevSamples[i] = 0;
    samples[i] = 0;
    pulses[i] = 0;
    prevPulses[i] = 0;
  }

  //Serial.println("Variables Inititialized");
}

void setParams()
{
  float setKp[] = { 0.050, 0.050 },
    setKi[] = { 0.000, 0.000 },
    setKd[] = { 0.000, 0.000 };

  setM1PIDGains(setKp[0], setKi[0], setKd[0]); // m1
  setM2PIDGains(setKp[1], setKi[1], setKd[1]); // m2
  //Serial.println("PID Gains Set");
  
  maxOutVal = 100 * 256; // max. output value in fixed point integer
  
  pubVelRate = 10; // [Hz]

  minAngularRes = 360.0 / pulsesPerRev;

  minLinearRes = (int) round( piApprox * wheelDiam / pulsesPerRev ); // r_{min} [mm/pulse]

  newPos = 1; // should be 1 default b/c when starting assume from rest and new pos is condition if starting from rest
  newVel = 0;

  turned = 0;
  drove = 0;
  
  eStop = 1;

  manual = 1;

  setPosActive = 0;
  setVelActive = 0;

  posDeadZone = 9; // [deg], CHANGE: tune based on resolution

  minStaticDutyCycle = 35; // [%] for M2 Decrementing by 1 from 35, 35% for M2 incrementing from 0 by 1; min duty cycle determined by OpenLoopControl.ino
  minStaticMtrCmd = (int) round( 255 * minStaticDutyCycle / 100.0 );
  minKineticDutyCycle = 14; // [%] for M2 Decrementing by 1 from 35, 35% for M2 incrementing from 0 by 1; min duty cycle determined by OpenLoopControl.ino
  minKineticMtrCmd = (int) round( 255 * minKineticDutyCycle / 100.0 );

  // diff drive:
  setFwdVel = 0;
  setYawVel = 0;
  cmdFwdVel = 0;
  cmdYawVel = 0;
  
  setRadius = 30; // [mm]
  navDeadZone = 9; // [deg], maybe = minAngularRes or what that amounts to for heading using minAngularRes of both wheels

  bDsplyEnabled = 0;
  
  vMax = 7500; // [mV]
  vMin = (long) round(5 * vMax / 6.0);
  blinkV = (long) round(51 / 60.0 * vMax);
  
  //Serial.println("Parameters Set");
}

void setM1PIDGains(float pg, float ig, float dg)
{
  if(pg < 0 || ig < 0 || dg < 0) return;

  kp[0] = pg;
  
  ki[0] = ig;

  kd[0] = dg;
}

void setM2PIDGains(float pg, float ig, float dg)
{
  if(pg < 0 || ig < 0 || dg < 0) return;

  kp[1] = pg;
  
  ki[1] = ig;

  kd[1] = dg;
}

void loop() 
{
  int i;

  unsigned long curPressTime = millis();

  if(curPressTime - prevPressTime > btnBounceTime) // if the noise is gone
  {
    noInterrupts();
      
    validPress = 1;

    if(newPress)
    {
      prevPressTime = curPressTime; // record the time of the first pulse in bounce group
      
      newPress = 0;
    }

    interrupts();
  }
  
  for(i=0; i < numEncoders; i++)
  { 
    if(decelerating[i] || pulses[i] == 0 || prevPulses[i] == 0)
      bounceTimes[i] = 50; // [ms]
    else
      bounceTimes[i] = 35; // [ms]
      
    curTimes[i] = millis();
    
    if(curTimes[i] - prevPulseTimes[i] > bounceTimes[i]) // if the noise is gone
    {
      noInterrupts();
      
      validPulses[i] = true;
      
      if(newPulses[i])
      {
        //Serial.println("===New Valid Pulse Detected===");
  
        prevPulseTimes[i] = curTimes[i]; // record the time of the first pulse in bounce group
        
        newPulses[i] = false;
      }

      interrupts();
    }
  }

  Serial.print(instance);
  Serial.print("\t");
  Serial.print(setFwdPos);
  Serial.print("\t");
  Serial.print(fwdPos);
  Serial.print("\t");
  Serial.print(setX);
  Serial.print("\t");
  Serial.print(measX);
  Serial.print("\t");
  Serial.print(xErr);
  Serial.print("\t");
  Serial.print(setY);
  Serial.print("\t");
  Serial.print(measY);
  Serial.print("\t");
  Serial.print(yErr);
  Serial.print("\t");
  Serial.print(fwdPosErr);
  Serial.print("\t");
  Serial.print(setYawPos);
  Serial.print("\t");
  Serial.print(yawPos);
  Serial.print("\t");
  Serial.print(yawPosErr);
  Serial.print("\t");
  
  for(i=0; i < numEncoders; i++)
  {
    Serial.print(curTimes[i]);
    Serial.print("\t");
    Serial.print(samples[i]);
    Serial.print("\t");
    Serial.print(pulses[i]);
    Serial.print("\t");
    Serial.print(setAngVels[i]);
    Serial.print("\t");
    Serial.print(cmdAngVels[i]);
    Serial.print("\t");
    Serial.print(angVels[i]);
    Serial.print("\t");
    Serial.print(velErrs[i]);
    Serial.print("\t");
    Serial.print(cmdPrcntMtrVltges[i]);
    Serial.print("\t");
    Serial.print(pubMtrCmds[i]);
    Serial.print("\t");
  }
  Serial.println();

  if(millis() - prevSenseTime > 100)
  {
    userTask(); 
    
    sensorsTask();
  
    mtrCmd();
  
    instance++;

    prevSenseTime = millis();
  }
}

void userTask()
{
  inputTask();

  outputTask();
}

void inputTask()
{
  if(stringComplete)
  {
    //Serial.print("inputString: ");

    // receive command from user
    if(inputString.substring(0,1) == "s")
    {
      //Serial.println("stop");

      eStop = 1;
    }
    else if(inputString.substring(0,1) == "g")
    {
      //Serial.println("go");

      wpNum++;

      wpNum %= numWpts;

      manual = 0;
      
      newPos = 1;
      newVel = 1;

      setPosActive = 1;
      setVelActive = 1;

      eStop = 0;
    }
    else if(inputString.substring(0,2) == "fd") // given in mm
    { 
      int dSetFwdPos = inputString.substring(2, inputString.length()).toInt(); // get string after 'fd'

      setFwdPos = fwdPos + dSetFwdPos;

      setYawPos = yawPos;
      
//      Serial.print("move forward ");
//      Serial.print(dSetFwdPos);
//      Serial.println(" mm\n");

      manual = 1;
      
      newPos = 1;
      newVel = 1;
      
      setPosActive = 1;
      setVelActive = 1;
      
      eStop = 0;
    }
    else if(inputString.substring(0,2) == "td") // given in deg
    { 
      int dSetYawPos = inputString.substring(2, inputString.length()).toInt(); // get string after 'td'

      setYawPos = ( yawPos + dSetYawPos ) % 360; // keep setYawPos w/in 360 (eg if dSetYawPos=540 and yawPos=180, then setYawPos=0)

      setFwdPos = fwdPos;
      
//      Serial.print("turn ");
//      Serial.print(dSetYawPos);
//      Serial.println(" deg\n");

      manual = 1;
      
      newPos = 1;
      newVel = 1;
      
      setPosActive = 1;
      setVelActive = 1;
      
      eStop = 0;
    }
    else if(inputString.substring(0,2) == "fv") // given in mm/s
    { 
      setFwdVel = inputString.substring(2, inputString.length()).toInt(); // get string after 'fv'
      
//      Serial.print("move forward ");
//      Serial.print(setFwdVel);
//      Serial.println(" mm/s\n");

      if(setYawVel == 0) setYawVel = 180;
      else if(setYawVel > 0 && setYawVel < 180) setYawVel = 180;
      else if(setYawVel < 0 && setYawVel > -180) setYawVel = -180;

      newVel = 1;
      
      //setVelActive = 1;
      
      eStop = 0;
    }
    else if(inputString.substring(0,2) == "tv") // given in deg/s
    { 
      setYawVel = inputString.substring(2, inputString.length()).toInt(); // get string after 'tv'
      
//      Serial.print("turn ");
//      Serial.print(setYawVel);
//      Serial.println(" deg/s\n");

      if(setYawVel > 0 && setYawVel < 180) setYawVel = 180;
      else if(setYawVel < 0 && setYawVel > -180) setYawVel = -180;

      newVel = 1;
      
      //setVelActive = 1;
      
      eStop = 0;
    }
    else if(inputString.substring(0,1) == "x") // given in mm
    { 
      setX = inputString.substring(1, inputString.length()).toInt(); // get string after 'x'
      
//      Serial.print("go  ");
//      Serial.print(setX);
//      Serial.println(" mm right of origin\n");

      manual = 0;
      
      newPos = 1;
      newVel = 1;
      
      setPosActive = 1;
      setVelActive = 1;

      eStop = 0;
    }
    else if(inputString.substring(0,1) == "y") // given in mm
    { 
      setY = inputString.substring(1, inputString.length()).toInt(); // get string after 'y'
      
//      Serial.print("go  ");
//      Serial.print(setY);
//      Serial.println(" mm above origin\n");

      manual = 0;
      
      newPos = 1;
      newVel = 1;
      
      setPosActive = 1;
      setVelActive = 1;

      eStop = 0;
    }
    else if(inputString.substring(0,1) == "p") // given number from table
    { 
      wpNum = inputString.substring(1, inputString.length()).toInt() % numWpts; // get string after 'p'
      
//      Serial.print("go to waypoint number ");
//      Serial.println(wpNum);

      //setX = wp[wpNum][0];
    
//      Serial.print("go  ");
//      Serial.print(setX);
//      Serial.println(" mm right of origin\n");
      
      //setY = wp[wpNum][1];
    
//      Serial.print("go  ");
//      Serial.print(setY);
//      Serial.println(" mm above origin\n");

      manual = 0;
      
      newPos = 1;
      newVel = 1;
      
      setPosActive = 1;
      setVelActive = 1;

      eStop = 0;
    }
    else if(inputString.substring(0,1) == "h")
    { 
      measX = wp[wpNum][0];
    
//      Serial.print("you are  ");
//      Serial.print(measX);
//      Serial.println(" mm right of origin\n");
      
      measY = wp[wpNum][1];
    
//      Serial.print("you are  ");
//      Serial.print(measY);
//      Serial.println(" mm above origin\n");

      yawPos = wp[wpNum][2];
    }
    else if(inputString.substring(0,1) == "b")
    { 
      bDsplyEnabled = !bDsplyEnabled;
    }
    
    // clear string:
    inputString = ""; //note: in code below, inputString will not become blank, inputString is blank until '\n' is received

    stringComplete = false;
  }

  if(Serial.available())
    serialEvent();
}

void serialEvent()
{
  while (Serial.available()) 
  {
    // get the new byte:
    char inChar = (char) Serial.read();
    
    // add it to the inputString:
    inputString += inChar;
    
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n')
      stringComplete = true;
  }
}

void outputTask()
{
  if(bDsplyEnabled)
  {
    readBatV(); // read battery voltage
    
    if(vInput < blinkV)
        blinkBDsply();
        
    else
    {
      bLevel = (byte) round(map(vInput, vMin, vMax, 0, numBLevels * 100) / 100.0);
  
      batteryDisplay.displayLevel(bLevel);
      batteryDisplay.frame(FRAME_ON);
    }
  }
  else
  {
    batteryDisplay.displayLevel(0);
    batteryDisplay.frame(FRAME_OFF);
  }
}

void blinkBDsply()
{
  bDsplyOn = !bDsplyOn;
  
  if(bDsplyOn)
  {
    batteryDisplay.displayLevel(0);
    batteryDisplay.frame(FRAME_OFF);
  }
  else
  {
    batteryDisplay.displayLevel(1);
    batteryDisplay.frame(FRAME_ON);
  }
}

void readBatV()
{
  const byte numSamples = 10; //number of analog readings taken to determine more accurate supply voltage
  
  int analogVal, //supply voltage converted to analog value by Arduino
    sum; //sum of sampled voltage readings
    
  float divFactor = 1.97; 
    
  for (int i=0; i < numSamples; i++) //take 10 analog samples:
  {
    analogVal = analogRead(batPin); //read value on analog pin
    
    sum += analogVal; //add values until all samples taken (sum averaged later)
    
    delayMicroseconds(3); //avoid potentially jumbled readings
  }
  
  vInput = (long) round( 625 * divFactor * sum / ( 128 * numSamples ) ); // [mV], 5 * divFactor * sum * 1000 / ( 1024 * numSamples )

  vInput = clip(vInput, vMax, vMin);
  
  sum = 0;
}

void sensorsTask()
{
  //===Search Task===
  int i;
  
  for(i=0; i < numEncoders; i++)
    observeSample(i);
      
  if(setPosActive == 1)
  {
    setX = wp[wpNum][0];
    setY = wp[wpNum][1];
    
    readCoordinates();
    computeCoordErr();
    computeYawPosErr();
    computeFwdPosErr();

    navDeadZone = (manual || newPos) ? 9 : 6;
    setRadius = manual ? 18 : 30;

    if(manual && abs(yawPosErr) > navDeadZone)
    {
      if(newVel || drove)
      {
        cmdFwdVel = 0; // pivot
        
        if(yawPosErr > 0)
          cmdYawVel = setYawVel; // +CCW
        else
          cmdYawVel = -setYawVel; // +CCW
  
        computeWheelVels();
  
        newVel = 0;
        drove = 0;
      }
      else
      {
        if(drove == 1 && newPos == 0)
        {
          for(i=0; i < numEncoders; i++)
          {
            if(setVelActive == 0 && decelerating[i] && pulses[i] != 0) // if mtr not fully at rest yet
              cmdAngVels[i] = 0; // ensure mtr does not move until 0 vel detected after decelerating
            else if(setVelActive == 0 && decelerating[i] && pulses[i] == 0 && prevPulses[i] != 0)
              cmdAngVels[i] = 0; // ensure mtr does not move until 0 vel detected after decelerating
            else
              cmdAngVels[i] = setAngVels[i];
            
            computeMtrCtrlSgnl(i); // assume set vel active b/c vel ctrl test
          }
        }
      }
      
      turned = 1;
    }
    else if(abs(yawPosErr) > navDeadZone && abs(fwdPosErr) > setRadius) // Check abs(dispErr)>setRadius so robot doesn't spin after reaching target even if it slightly overshot target
    {
//      Serial.print(abs(yawErr));
//      Serial.println(" deg > navDeadZone");
//      Serial.print("===Pivot to ");
//      Serial.print(setYawPos);
//      Serial.println(" deg===");

      if(newVel || drove)
      {
        if(newPos)
          cmdFwdVel = 0; // pivot
        else
        {
          if(fwdPosErr > 0)
            cmdFwdVel = setFwdVel; // +Fwd
          else
            cmdFwdVel = -setFwdVel; // +Fwd
        }
        
        if(yawPosErr > 0)
          cmdYawVel = setYawVel; // +CCW
        else
          cmdYawVel = -setYawVel; // +CCW
  
        computeWheelVels();

        if(drove == 1 && newPos == 0)
        {
          for(i=0; i < numEncoders; i++)
          {
            if(setVelActive == 0 && decelerating[i] && pulses[i] != 0) // if mtr not fully at rest yet
              cmdAngVels[i] = 0; // ensure mtr does not move until 0 vel detected after decelerating
            else if(setVelActive == 0 && decelerating[i] && pulses[i] == 0 && prevPulses[i] != 0)
              cmdAngVels[i] = 0; // ensure mtr does not move until 0 vel detected after decelerating
            else
              cmdAngVels[i] = setAngVels[i];
            
            computeMtrCtrlSgnl(i); // assume set vel active b/c vel ctrl test
          }
        }
  
        newVel = 0;
        drove = 0;
      }
      else
      {
        for(i=0; i < numEncoders; i++)
        {
          if(setVelActive == 0 && decelerating[i] && pulses[i] != 0) // if mtr not fully at rest yet
            cmdAngVels[i] = 0; // ensure mtr does not move until 0 vel detected after decelerating
          else if(setVelActive == 0 && decelerating[i] && pulses[i] == 0 && prevPulses[i] != 0)
            cmdAngVels[i] = 0; // ensure mtr does not move until 0 vel detected after decelerating
          else
            cmdAngVels[i] = setAngVels[i];
          
          computeMtrCtrlSgnl(i); // assume set vel active b/c vel ctrl test
        }
      }
      
      turned = 1;

      //newVel = 0;
      //drove = 0;
    }
    else if(abs(fwdPosErr) > setRadius)
    {
//      Serial.print(abs(fwdPosErr));
//      Serial.println(" mm > setRadius");
//      Serial.print("===Move Straight ");
//      Serial.print(abs(fwdPosErr));
//      Serial.println(" mm===");

      if(newVel || turned)
      {
        if(fwdPosErr > 0)
          cmdFwdVel = setFwdVel; // +Fwd
        else
          cmdFwdVel = -setFwdVel; // +Fwd
          
        cmdYawVel = 0; // drive straight
  
        computeWheelVels();
  
        newVel = 0;
        turned = 0;
      }
      else
      {
        for(i=0; i < numEncoders; i++)
        {
          if(setVelActive == 0 && decelerating[i] && pulses[i] != 0) // if mtr not fully at rest yet
            cmdAngVels[i] = 0; // ensure mtr does not move until 0 vel detected after decelerating
          else if(setVelActive == 0 && decelerating[i] && pulses[i] == 0 && prevPulses[i] != 0)
            cmdAngVels[i] = 0; // ensure mtr does not move until 0 vel detected after decelerating
          else
            cmdAngVels[i] = setAngVels[i];
          
          computeMtrCtrlSgnl(i); // assume set vel active b/c vel ctrl test
        }
      }

      drove = 1;

      //newVel = 0;
      //turned = 0;
      
      newPos = 0;
    }
    else // found target
    {
      for(i=0; i < numEncoders; i++)
      {
        cmdAngVels[i] = 0;
        cmdPrcntMtrVltges[i] = 0; // may not need to explicitly set to 0 if cmdAngVel auto-changes this quickly enough 
        pubMtrCmds[i] = 0; // may not need to explicitly set to 0 if cmdAngVel auto-changes this quickly enough 
      }

      if(pulses[0] == 0 && prevPulses[0] == 0 && pulses[1] == 0 && prevPulses[1] == 0)
      {
          manual = 0;
          
          setPosActive = 0;
          setVelActive = 0;
      }
    }
  }
  else if(setVelActive == 1)
  {
    if(newVel)
    {
      cmdFwdVel = setFwdVel;
      cmdYawVel = setYawVel; // +CW
      
      computeWheelVels();

      newVel = 0;
    }
    else
    {
      for(i=0; i < numEncoders; i++)
      {
        cmdAngVels[i] = setAngVels[i];
        
        computeMtrCtrlSgnl(i); // assume set vel active b/c vel ctrl test
      }
    }

    //newVel = 0;
  }

  // ADD rc task for radio/remote ctrl
}
  
// get displacement of each wheel to get this instant's
// linear and angular displacements
/* Read and zero pulses.
 * Copy and accumulate counts from pulses
 * to the ang. vel. variable and
 * then reset pulses to zero.
 */
void observeSample(int mid)
{
  //Serial.println("======Observe Sample======");

  noInterrupts();
  
  samples[mid] = pulseCounts[mid];

  interrupts();

//  Serial.print("samples[");
//  Serial.print(mid);
//  Serial.print("] = pulseCounts[");
//  Serial.print(mid);
//  Serial.print("] = ");
//  Serial.print(samples[mid]);
//  Serial.println(" pulses");
  
  //  Serial.print("samples[0]: ");
  //  Serial.println(samples[0]);
  
  angPos[mid] = (int) round( minAngularRes * ( samples[mid] % pulsesPerRev ) ); // [deg]
  
  prevPulses[mid] = pulses[mid];
  pulses[mid] = samples[mid] - prevSamples[mid]; // P pulses

  prevAngVels[mid] = angVels[mid]; // maintain history of previous measured rotVel
  angVels[mid] = (int) round( minAngularRes * pulses[mid] * pubVelRate); // [deg/s]
    //    Serial.print("angVels[");
  //    Serial.print(i);
  //    Serial.print("] = minAngularRes * pulses[");
  //    Serial.print(i);
  //    Serial.print("] * signs[");
  //    Serial.print(i);
  //    Serial.print("] * pubVelRate = ");
  //    Serial.print(minAngularRes);
  //    Serial.print(" deg/pulse * ");
  //    Serial.print(pulses[i]);
  //    Serial.print(" pulses * ");
  //    Serial.print(signs[i]);
  //    Serial.print(" * ");
  //    Serial.print(pubVelRate);
  //    Serial.print(" Hz = ");
  //    Serial.print(angVels[i]);
  //    Serial.println(" deg/s");

  if(pulses[mid] != 0 && prevPulses[mid] != 0) // check that mtr is moving before checking if decelerating
  {
      // assume decelerating if going at slower speed than before:
      if(abs(angVels[mid]) < abs(prevAngVels[mid])) // could compare pulses[] and prevPulses[]
        decelerating[mid] = true;
      else
        decelerating[mid] = false;
  }
  else if(pulses[mid] == 0 && prevPulses[mid] == 0)
    decelerating[mid] = false;
    
  prevSamples[mid] = samples[mid];
}

void readCoordinates()
{
  int yawPosScale = 2, // determined by testing how far off yaw pos calc is
    
    yawDisp = (int) round( ( 360 * minLinearRes 
                            * ( pulses[1] - pulses[0] )
                            / ( piApprox * botDiam ) ) / yawPosScale ), // [deg]
    
    relativePositionVector 
            = (int) round( minLinearRes * ( pulses[0] + pulses[1] ) / 2.0 ); // [mm]

  fwdPos = (int) round( minLinearRes * ( samples[0] + samples[1] ) / 2.0 );

  yawPos = ( yawPos + yawDisp ) % 360; // [deg]
  
  if(yawPos < 0) yawPos += 360;
  
  if(yawPos > 0 && yawPos < 90) // if traveling up positive slope
  {
    measDX = (int) round ( relativePositionVector * cos( yawPos / degsPerRad ) );

    measDY = (int) round ( relativePositionVector * sin( yawPos / degsPerRad ) );
  }
  else if(yawPos > 90 && yawPos < 180) // if traveling up negative slope
  {
    measDX = (int) round ( -relativePositionVector * cos( ( 180 - yawPos ) / degsPerRad ) );

    measDY = (int) round ( relativePositionVector * sin( ( 180 - yawPos ) / degsPerRad ) );
  }
  else if(yawPos > 180 && yawPos < 270) // if traveling down positive slope
  {
    measDX = (int) round ( -relativePositionVector * cos( ( yawPos - 180 ) / degsPerRad ) );

    measDY = (int) round ( -relativePositionVector * sin( ( yawPos - 180 ) / degsPerRad ) );
  }
  else if(yawPos > 270 && yawPos < 360) // if traveling down negative slope
  {
    measDX = (int) round ( relativePositionVector * cos( ( 360 - yawPos ) / degsPerRad ) );

    measDY = (int) round ( -relativePositionVector * sin( ( 360 - yawPos ) / degsPerRad ) );
  }
  else if(yawPos == 0)
  {
    measDX = relativePositionVector;
    measDY = 0;
  }
  else if(yawPos == 90)
  {
    measDX = 0;
    measDY = relativePositionVector;
  }
  else if(yawPos == 180)
  {
    measDX = -relativePositionVector;
    measDY = 0;
  }
  else if(yawPos == 270)
  {
    measDX = 0;
    measDY = -relativePositionVector;
  }

  measX += measDX;
  measY += measDY;
}

void computeCoordErr()
{
  xErr = setX - measX;
  yErr = setY - measY;
}

void computeYawPosErr()
{
  int ye1, ye2; // heading errors

  if(!manual)
  {
    if(abs(xErr) > 0)
      setYawPos = atan2(yErr, xErr) * degsPerRad;
    else if(xErr == 0 && yErr == 0) // note: dx == 0 is redundant b/c it must be 0 to reach this point
      setYawPos = yawPos;
    else if(xErr == 0)
    {
      if(yErr > 0)
        setYawPos = 90;
      else if(yErr < 0)
        setYawPos = 270; // or -90 deg
    }
    else if(yErr == 0)
    {
      if(xErr > 0)
        setYawPos = 0;
      else if(xErr < 0)
        setYawPos = 180;
    }
  }
  
  if(setYawPos < 0)
    setYawPos += 360; 

  ye1 = setYawPos - yawPos; // [deg]

  ye2 = ye1 > 0 ? ye1 - 360 : ye1 + 360; // [deg]

  yawPosErr = abs(ye1) > abs(ye2) ? ye2 : ye1; // [deg]
}

void computeFwdPosErr()
{
  if(manual)// if no coordinates
    fwdPosErr = setFwdPos - fwdPos; // [mm]

  else
    fwdPosErr = (int) round( sqrt( pow(xErr, 2) + pow(yErr, 2) ) );
}

void computeWheelVels()
{
  int minRes = 180,
    lowerBound,
    upperBound;

  setAngVels[0] = (int) round( ( cmdFwdVel / minLinearRes - cmdYawVel / minAngularRes ) * minAngularRes ); // [deg/s]
  setAngVels[1] = (int) round( ( cmdFwdVel / minLinearRes + cmdYawVel / minAngularRes ) * minAngularRes ); // [deg/s]

  for(int i=0; i < numEncoders; i++)
  {
    if(setAngVels[i] != 0)
    {
      setAngVels[i] = clip(setAngVels[i], topAngVel, -topAngVel);
    
      if(abs(setAngVels[i]) < minRes) 
        setAngVels[i] = setAngVels[i] < 0 ? -minRes : minRes;
  
      else
      {
        lowerBound = minRes * floor( setAngVels[i] / minRes );
        upperBound = lowerBound + minRes;
  
        setAngVels[i] = abs(lowerBound - setAngVels[i]) 
                      < abs(setAngVels[i] - upperBound)
                      ? lowerBound : upperBound;
      }
    }

    if(newVel || (drove && newPos) || turned)
      cmdPrcntMtrVltges[i] = map(setAngVels[i], -topAngVel, topAngVel, -100, 100); // convert to % mtr voltage

//    else // set ang vel = 0
//    {
//      if(newVel || (drove && newPos) || turned)
//        cmdPrcntMtrVltges[i] = 0;
//    }
  }  
}

void computeMtrCtrlSgnl(int mid)
{
  long errs[numEncoders], 
    dAngVels[numEncoders], 
    P[numEncoders], 
    I[numEncoders], 
    D[numEncoders];

//  Serial.print("======Locate Target (");
//  Serial.print(setAngVel);
//  Serial.println(" deg/s)======");

//  if(newVel || (drove && newPos) || turned)
//    cmdPrcntMtrVltges[mid] = map(setAngVels[mid], -topAngVel, topAngVel, -100, 100); // convert to % mtr voltage
//
//  else
//  {

  velErrs[mid] = cmdAngVels[mid] - angVels[mid];
    
  errs[mid] = (long) round( velErrs[mid] / degsPerRad * 256 ); // [rad/s]*256, generate error signal based on difference b/t measured rotVel and requested rotVel for the wheel. Note: multiply by 256 and later divide by 256 b/c operation in fixed point integer
  //  Serial.print("errs[");
  //  Serial.print(i);
  //  Serial.print("] = ( b * setAngVels[");
  //  Serial.print(i);
  //  Serial.print("] - angVels[");
  //  Serial.print(i);
  //  Serial.print("] ) / degsPerRad = ( ");
  //  Serial.print(b);
  //  Serial.print(" * ");
  //  Serial.print(setAngVels[i]);
  //  Serial.print(" deg/s - ");
  //  Serial.print(angVels[i]);
  //  Serial.print(" deg/s ) / ");
  //  Serial.print(degsPerRad);
  //  Serial.print(" deg/rad = ");
  //  Serial.print(b * setAngVels[i] - angVels[i]);
  //  Serial.print(" deg/s / ");
  //  Serial.print(degsPerRad);
  //  Serial.print(" deg/rad = ");
  //  Serial.print(errs[i] / 256.0);
  //  Serial.println(" rad/s");

  float velScaleFactor = cmdAngVels[mid] / 360.0; // should =1 when set fwd vel=200mm/s or cmd ang vel=360deg/s b/c results in smooth motion
  
  P[mid] = (long) round( kp[mid] * errs[mid] * pubVelRate * velScaleFactor ); // P(t_k) = K(by_{sp}(t_k) - y(t_k))
  //  Serial.print("P[");
  //  Serial.print(i);
  //  Serial.print("] = kp[1][");
  //  Serial.print(i);
  //  Serial.print("] * errs[");
  //  Serial.print(i);
  //  Serial.print("] = ");
  //  Serial.print(kp[0][i]);
  //  Serial.print(" % * ");
  //  Serial.print(errs[i] / 256.0);
  //  Serial.print(" rad/s = ");
  //  Serial.print(P[i] / 256.0);
  //  Serial.println(" %/s");
  
  //    if(abs(errs[i]) < threshIntegral)
  //    {
  //      I[i] += (long) round( ki * errs[i] );
  //
  //      I[i] = clip(I[i], maxOutVal, -maxOutVal);
  //    }
  //    else
  //      I[i] = 0;
  //
  dAngVels[mid] = (long) round( ( angVels[mid] - prevAngVels[mid] ) / degsPerRad * 256 );
  //  Serial.print("dAngVels[");
  //  Serial.print(i);
  //  Serial.print("] = ( angVels[");
  //  Serial.print(i);
  //  Serial.print("] - prevAngVels[");
  //  Serial.print(i);
  //  Serial.print("] ) / degsPerRad = ( ");
  //  Serial.print(angVels[i]);
  //  Serial.print(" deg/s - ");
  //  Serial.print(prevAngVels[i]);
  //  Serial.print(" deg/s ) / ");
  //  Serial.print(degsPerRad);
  //  Serial.print(" deg/rad = ");
  //  Serial.print(dAngVels[i] / 256.0);
  //  Serial.println(" rad/s");
    
  D[mid] = (long) round( kd[mid] * dAngVels[mid] * pubVelRate ); // large when amount of change requested by PID controller is large, and small as signal approaches zero
  //  Serial.print("D[");
  //  Serial.print(i);
  //  Serial.print("] = kd[1][");
  //  Serial.print(i);
  //  Serial.print("] * dAngVels[");
  //  Serial.print(i);
  //  Serial.print("] = ");
  //  Serial.print(kd[1][i]);
  //  Serial.print(" %.s * ");
  //  Serial.print(dAngVels[i]);
  //  Serial.print(" rad/s / 1 s = ");
  //  Serial.print(D[i] / 256.0);
  //  Serial.println(" %/s");
  
  //  Serial.print("mtrOut[");
  //  Serial.print(i);
  //  Serial.print("] = prevMtrOut[");
  //  Serial.print(i);
  //  Serial.print("] + P[");
  //  Serial.print(i);
  //  Serial.print("] + D[");
  //  Serial.print(i);
  //  Serial.print("] = ");
  //  Serial.print((int) round( mtrOut[i] / 256.0 ));
  //  Serial.print(" %/s + ");
      
  cmdPrcntMtrVltges[mid] += (int) round( ( P[mid] + D[mid] ) / 256.0 ); // [%/s], + I[i]; // Increase or decrease speed of motor to force measured to equal requested rotVel
  //  Serial.print(P[i] / 256.0);
  //  Serial.print(" %/s + ");
  //  Serial.print(D[i] / 256.0);
  //  Serial.print(" %/s = ");
  //  Serial.print((int) round( mtrOut[i] / 256.0 ));
  //  Serial.println(" %/s");
  
  //}
  
  cmdPrcntMtrVltges[mid] = clip(cmdPrcntMtrVltges[mid], 100, -100); // [%/s]
  //  Serial.print("mtrOut[");
  //  Serial.print(i);
  //  Serial.print("] = ");
  //  Serial.print((int) round( mtrOut[i] / 256.0 ));
  //  Serial.println(" %/s");
}

void locateTarget()
{
  long errs[numEncoders], 
    dAngPos[numEncoders], 
    P[numEncoders], 
    I[numEncoders], 
    D[numEncoders];

//  Serial.print("======Locate Target (");
//  Serial.print(setPos);
//  Serial.println(" deg)======");

  computePosErrs();

  for(int i=0; i < numEncoders; i++)
  {
    errs[i] = (long) round( posErrs[i] / degsPerRad * 256 ); // [rad]*256, generate error signal based on difference b/t measured rotVel and requested rotVel for the wheel. Note: multiply by 256 and later divide by 256 b/c operation in fixed point integer
  //  Serial.print("errs[");
  //  Serial.print(i);
  //  Serial.print("] = -posErrs[");
  //  Serial.print(i);
  //  Serial.print("] / degsPerRad = ");
  //  Serial.print(-posErrs[i]);
  //  Serial.print(" deg / ");
  //  Serial.print(degsPerRad);
  //  Serial.print(" deg/rad = ");
  //  Serial.print(errs[i] / 256.0);
  //  Serial.println(" rad");
    
    P[i] = (long) round( kp[i] * errs[i] * pubVelRate ); // P(t_k) = K(by_{sp}(t_k) - y(t_k))
  //  Serial.print("P[");
  //  Serial.print(i);
  //  Serial.print("] = kp[0][");
  //  Serial.print(i);
  //  Serial.print("] * errs[");
  //  Serial.print(i);
  //  Serial.print("] * pubVelRate = ");
  //  Serial.print(kp[0][i]);
  //  Serial.print(" rad/s * ");
  //  Serial.print(errs[i] / 256.0);
  //  Serial.print(" rad * ");
  //  Serial.print(pubVelRate);
  //  Serial.print(" Hz = ");
  //  Serial.print(P[i] / 256.0);
  //  Serial.println(" rad/s");
  
  //    if(abs(errs[i]) < threshIntegral)
  //    {
  //      I[i] += (long) round( ki * errs[i] );
  //
  //      I[i] = clip(I[i], maxOutVal, -maxOutVal);
  //    }
  //    else
  //      I[i] = 0;
  //
    dAngPos[i] = (long) round( ( angPos[i] - prevAngPos[i] ) / degsPerRad * 256 );
  //  Serial.print("dAngPos[");
  //  Serial.print(i);
  //  Serial.print("] = ( angPos[");
  //  Serial.print(i);
  //  Serial.print("] - prevAngPos[");
  //  Serial.print(i);
  //  Serial.print("] ) / degsPerRad = ( ");
  //  Serial.print(angPos[i]);
  //  Serial.print(" deg - ");
  //  Serial.print(prevAngPos[i]);
  //  Serial.print(" deg ) / ");
  //  Serial.print(degsPerRad);
  //  Serial.print(" deg/rad = ");
  //  Serial.print(dAngPos[i] / 256.0);
  //  Serial.println(" rad");
    
    D[i] = (long) round( kd[i] * dAngPos[i] * pubVelRate ); // large when amount of change requested by PID controller is large, and small as signal approaches zero
  //  Serial.print("D[");
  //  Serial.print(i);
  //  Serial.print("] = kd[0][");
  //  Serial.print(i);
  //  Serial.print("] * dAngPos[");
  //  Serial.print(i);
  //  Serial.print("] * pubVelRate = ");
  //  Serial.print(kd[0][i]);
  //  Serial.print(" rad * ");
  //  Serial.print(dAngPos[i] / 256.0);
  //  Serial.print(" rad * ");
  //  Serial.print(pubVelRate);
  //  Serial.print(" Hz = ");
  //  Serial.print(D[i] / 256.0);
  //  Serial.println(" rad/s");
  
    // ADD: if vel control, output setAngVel to vel controller
    setAngVels[i] = (int) round( ( P[i] + D[i] ) * degsPerRad  / 256.0 ); // [deg/s], + I[i]; // Increase or decrease speed of motor to force measured to equal requested rotVel
  //  Serial.print("setAngVels[");
  //  Serial.print(i);
  //  Serial.print("] = ( P[");
  //  Serial.print(i);
  //  Serial.print("] + D[");
  //  Serial.print(i);
  //  Serial.print("] ) * degsPerRad / 256.0 = ( ");
  //  Serial.print(P[i] / 256.0);
  //  Serial.print(" rad/s + ");
  //  Serial.print(D[i] / 256.0);
  //  Serial.print(" rad/s ) * ");
  //  Serial.print(degsPerRad);
  //  Serial.print(" deg/rad = ");
  //  Serial.print(setAngVels[i]);
  //  Serial.println(" deg/s");
    
    prevAngPos[i] = angPos[i]; // maintain history of previous measured rotVel
  
    setAngVels[i] = clip(setAngVels[i], topAngVel, -topAngVel); // [deg/s]
  //  Serial.print("setAngVels[");
  //  Serial.print(i);
  //  Serial.print("] = ");
  //  Serial.print(setAngVels[i]);
  //  Serial.println(" deg/s");
  
    cmdPrcntMtrVltges[i] = map(setAngVels[i], -topAngVel, topAngVel, -100, 100);
  }
}

void computePosErrs()
{
  //Serial.println("===Compute Position Error===");

  int pe1, pe2, // pos errors
    b = 1; // set point weight
    
  float H = 1.0; // / (1 + timeConst); // sensor
    
  // ADD for indep drive:
//  for(int i=0; i < numEncoders; i++)
//  {
//    pe1 = (int) round( b * H * setPos[i] - angPos[i] );
//  //  Serial.print("pe1 = angPos[1] - setPos[1] = ");
//  //  Serial.print(angPos[1]);
//  //  Serial.print(" deg - ");
//  //  Serial.print(setPos[1]);
//  //  Serial.print(" deg = ");
//  //  Serial.print(pe1);
//  //  Serial.println(" deg");
//  
//    //Serial.print("pe2 = pe1 ");
//    if(pe1 > 0)
//    {
//      pe2 = pe1 - 360; // [deg]
//      //Serial.print("- ");
//    }
//    else
//    {
//      pe2 = pe1 + 360;
//      //Serial.print("+ ");
//    }
//  
//  //  Serial.print("360 deg = ");
//  //  Serial.print(pe2);
//  //  Serial.println(" deg");
//  
//    posErrs[i] = abs(pe1) > abs(pe2) ? pe2 : pe1;
//  //  if(abs(pe1) > abs(pe2))
//  //    posErr = pe2;
//  //  else
//  //    posErr = pe1;
//  
//  //  Serial.print("posErrs[1] = ");
//  //  Serial.print(posErrs[1]);
//  //  Serial.println(" deg");
//  
//    if(pulses[i] != 0 && prevPulses[i] != 0) // check that mtr is moving before checking if decelerating
//    {
//      // assume decelerating if passed target:
//      if(prevPosErrs[i] <= 0 && posErrs[i] > 0) 
//      {
//        signs[i] = -1; // continue in same direc until at rest
//        
//        decelerating[i] = true;
//      }
//      else if(prevPosErrs[i] >= 0 && posErrs[i] < 0)
//      {
//        signs[i] = 1; // continue in same direc until at rest
//        
//        decelerating[i] = true;
//      }
//    }
//  
//    prevPosErrs[i] = posErrs[i];
//  }
}

void mtrCmd()
{  
  if(eStop) emergencyStop();
  
  else 
  {
    // if only pos ctrl:
    //cmdPrcntMtrVltge = map(cmdAngVel, -topAngVel, topAngVel, -100, 100);
  
    modulatePulseWidths(cmdPrcntMtrVltges);
  }
}

/* The accumulator motorOutAccum must be clipped 
 * at some positive and negative value
 * to keep from overflowing the fixed point arithmetic.
 */
int clip(int a, int maximum, int minimum)
{
  //Serial.print("Computed val: ");
  //Serial.print(a);
    
  if(a > maximum) 
    a = maximum;
  else if(a < minimum) 
    a = minimum;

  //Serial.print(", Clipped val: ");
  //Serial.println(a);

  return a;
}

void emergencyStop()
{
  for(int i=0; i < numEncoders; i++)
    digitalWrite(mEnablePins[i], LOW);
}

/* The PWM code drives the hardware H-bridge, 
 * which actually control the motor.
 * This routine takes a signed value, 
 * -100 < signedVal < 100,
 * sets the sign variable used by the speedometer code,
 * sets the forward/backward (i.e. direct/reverse) bits 
 * on the H-bridge, and
 * uses abs(signedVal) as an index into a 100 entry table 
 * of linear PWM values.
 * This function uses a timer interrupt to generate 
 * a x Hz (maybe 120 Hz) variable pulse-width output.
 */
void modulatePulseWidths(int signedVals[]) // take signed value, b/t -100 and 100
{
  for(int i=0; i < numEncoders; i++)
  {
    setSpeedometerSign(i, signedVals[i]); // set sign variable used by speedometer code
  
    setHBridgeDirectionBit(i, signedVals[i]);
    
    pubMtrCmds[i] = getPWMValueFromEntryTable(i, abs(signedVals[i])); // use abs(signedVal) as an index into a 100 entry table of linear PWM values
  
    analogWrite(mEnablePins[i], pubMtrCmds[i]); // generate variable pulse-width output
  } 
}

/* The sign variable represents the direction of rotation
 * of the motor (1 for forward and -1 for backward).
 * With more expensive quadrature encoders this info is
 * read directly from the encoders.
 * In this implementation I have only simple encoders so
 * the direction of rotation is taken from the sign of the
 * most recent command issued by the PID to the PWM code.
 */
void setSpeedometerSign(int mid, int signedVal) // signedVal should be most recent cmd issued by PID to PWM code
{
  if(!decelerating[mid])
  {
    noInterrupts();
    
    // assume sign unchanged if signedVal=cmdPrcntMtrVltge=0
    if(signedVal < 0) // {motor direction of rotation} = backward
      signs[mid] = -1;
    else if(signedVal > 0)
      signs[mid] = 1; // {motor direction of rotation} = {forward | resting}
    //else
      //Serial.println("Invalid command issued to PWM code!\n");

    interrupts();
  }
//  Serial.print("M");
//  Serial.print(mid + 1);
//  Serial.print(" speedometer sign: ");
//  Serial.println(signs[mid]);
}

void setHBridgeDirectionBit(int mid, int signedVal)
{
  // definitely works for indep mode but may need to change for diff mode
  if(signedVal < 0) // {motor direction of rotation} = CW
  {
    if(mid == 0) digitalWrite(mSigPins[mid], HIGH);
    else if(mid == 1) digitalWrite(mSigPins[mid], LOW);
  }
  else if(signedVal >= 0) // {motor direction of rotation} = {CCW | resting}
  {
    if(mid == 0) digitalWrite(mSigPins[mid], LOW);
    else if(mid == 1) digitalWrite(mSigPins[mid], HIGH);
  }
  //else
    //Serial.println("Invalid command issued to PWM code!\n");
}

// use magnitude as an index into a 100 entry table of linear PWM values
int getPWMValueFromEntryTable(int mid, int magnitude)
{
  if(magnitude == 0)
    return magnitude;
    
  else if(setPosActive && (pulses[mid] == 0 || prevPulses[mid] == 0))
    return map(magnitude, 0, 100, minStaticMtrCmd, 255);
    
  else if(pulses[mid] == 0 && prevPulses[mid] == 0)
    return map(magnitude, 0, 100, minStaticMtrCmd, 255);
    
  else
    return map(magnitude, 0, 100, minKineticMtrCmd, 255); // cruise outputs
}

//======Interrupt Service Routines======
void encoder1Callback()
{
  if(validPulses[0])
  {
    if(signs[0] == 1)
      pulseCounts[0]++;
    else
      pulseCounts[0]--;

    validPulses[0] = false;
  }

  newPulses[0] = true;
}

void encoder2Callback()
{
  if(validPulses[1])
  {
    if(signs[1] == 1)
      pulseCounts[1]++;
    else
      pulseCounts[1]--;

    validPulses[1] = false;
  }

  newPulses[1] = true;
}

void btnPressed()
{
  if(validPress)
  {
    bDsplyEnabled = !bDsplyEnabled;

    validPress = 0;
  }

  newPress = 1;
}
