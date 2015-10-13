/*  MonsterMoto Shield Example Sketch
  date: 5/24/11
  code by: Jim Lindblom
  hardware by: Nate Bernstein
  SparkFun Electronics
 
 License: CC-SA 3.0, feel free to use this code however you'd like.
 Please improve upon it! Let me know how you've made it better.
 
 This is really simple example code to get you some basic
 functionality with the MonsterMoto Shield. The MonsterMote uses
 two VNH2SP30 high-current full-bridge motor drivers.
 
 Use the motorGo(uint8_t motor, uint8_t direct, uint8_t pwm) 
 function to get motors going in either CW, CCW, BRAKEVCC, or 
 BRAKEGND. Use motorOff(int motor) to turn a specific motor off.
 
 The motor variable in each function should be either a 0 or a 1.
 pwm in the motorGo function should be a value between 0 and 255.
 */
#define BRAKEVCC 0
#define CW   1
#define CCW  2
#define BRAKEGND 3
#define CS_THRESHOLD 100

const int Switch = 1;
const int Joystick = 2;

/*  VNH2SP30 pin definitions
 xxx[0] controls '1' outputs
 xxx[1] controls '2' outputs */
int inApin[2] = {7, 4};  // INA: Clockwise input
int inBpin[2] = {8, 9}; // INB: Counter-clockwise input
int pwmpin[2] = {5, 6}; // PWM input
int cspin[2] = {2, 3}; // CS: Current sense ANALOG input
int enpin[2] = {0, 1}; // EN: Status of switches output (Analog pin)




int statpin = 13;
const int FwButton = 2;
const int RevButton = 3;
const int RightButton = 10;
const int LeftButton = 11;

const int FwRevAxis = A5;
const int RlAxis = A10;

int InputType = 0;

void setup()
{
  Serial.begin(9600);
  
  pinMode(statpin, OUTPUT);

  // Initialize digital pins as outputs
  for (int i=0; i<2; i++)
  {
    pinMode(inApin[i], OUTPUT);
    pinMode(inBpin[i], OUTPUT);
    pinMode(pwmpin[i], OUTPUT);
  }
  // Initialize braked
  for (int i=0; i<2; i++)
  {
    digitalWrite(inApin[i], LOW);
    digitalWrite(inBpin[i], HIGH);
  }

  InputType = getInputType();

if (Switch == InputType)
{
  pinMode(FwButton, INPUT_PULLUP);
  pinMode(RevButton, INPUT_PULLUP);
  pinMode(RightButton, INPUT_PULLUP);
  pinMode(LeftButton, INPUT_PULLUP);
}
if (Joystick == InputType)
{
  pinMode(FwRevAxis, INPUT);
  pinMode(RlAxis, INPUT);
}


}

const int onFull = 110;
const int offFull = 0;
int leftDrive = 0;

// positive is CW, negative is CCW;
int MotorTarget[2] = {0};
int MotorCurrent[2] = {0};
const int StepSize = 1;

void loop()
{
  if (InputType == Switch)
  {
    updateTargets_Switches();
  }
  else if (InputType == Joystick)
  {
    updateTargets_Joystick();
//    for (int i=0; i < 2; i++)
//    {    
//      Serial.print(" i=");
//      Serial.print(
//      smartMotorGo(i, MotorCurrent[i]);
//    }
//    delay(3);
//    return;
  }
  
//  Serial.print("Target[0] = ");
//  Serial.print(MotorTarget[0]);
//  Serial.print("  Target[1] = ");
//  Serial.print(MotorTarget[1]);

//  Serial.print("  Current[0] = ");
//  Serial.print(MotorCurrent[0]);
//  Serial.print("  Current[1] = ");
//  Serial.print(MotorCurrent[1]);
//  Serial.println();
  
  bool changed = false;
  
  for (int i=0; i < 2; i++)
  {
    if (MotorTarget[i] > MotorCurrent[i])
    {
      MotorCurrent[i] += StepSize;
      if (MotorTarget[i] < MotorCurrent[i])
      {
        MotorCurrent[i] = MotorTarget[i];
      }      
      changed = true;
    }
    else if (MotorTarget[i] < MotorCurrent[i])
    {
      MotorCurrent[i] -= StepSize;
      if (MotorTarget[i] > MotorCurrent[i])
      {
        MotorCurrent[i] = MotorTarget[i];
      }      
      changed = true;
    }
    if (changed)
    {
      smartMotorGo(i, MotorCurrent[i]);
    }

  }
  
  delay(3);
  
}

int getInputType()
{
//  while(1)
//  {

  pinMode(FwButton, INPUT_PULLUP);
  pinMode(RevButton, INPUT_PULLUP);
  pinMode(RightButton, INPUT_PULLUP);
  pinMode(LeftButton, INPUT_PULLUP);
  pinMode(FwRevAxis, INPUT_PULLUP);
  pinMode(RlAxis, INPUT_PULLUP);

  //delay(50);
  
  int FwRev = analogRead(FwRevAxis);
  int Rl = analogRead(RlAxis);

  Serial.print("x=");
  Serial.print(Rl);
  Serial.print("  y=");
  Serial.println(FwRev);
  
  if ((FwRev > 300 && FwRev < 700) &&
      (Rl > 300 && Rl < 700))
  {
//    for (int i=0; i < 20; i++)
//    {
      Serial.println("Found Joystick!");
//      Serial.print("x=");
//      Serial.print(Rl);
//      Serial.print("  y=");
//      Serial.println(FwRev);                
//      delay(5000);
//   FwRev = analogRead(FwRevAxis);
//   Rl = analogRead(RlAxis);
      
//    }
    return Joystick;
  }
  
  Serial.println("Found Switches!");

  
  return Switch;
//  }
  
}

void updateTargets_Joystick()
{
  Serial.println("Reading Joystick");
  pinMode(FwRevAxis, INPUT_PULLUP);
  pinMode(RlAxis, INPUT_PULLUP);

  int FwRev = analogRead(FwRevAxis);
  int Rl = analogRead(RlAxis);

  if (FwRev < 300 || Rl < 300 || FwRev > 700 || Rl > 700)
  {
    // Joystick was disconnected or has failed
    MotorTarget[0] = 0;
    MotorTarget[1] = 0;        
    return;
  } 
  pinMode(FwRevAxis, INPUT);
  pinMode(RlAxis, INPUT);
  
  delay(1); // allow the new values to stabilize
  
  FwRev = analogRead(FwRevAxis);
  Rl = analogRead(RlAxis);

  float fFw = FwRev;
  fFw -= 512;
  fFw = fFw / 150;
  if (fFw > 1) fFw = 1;
  if (fFw < -1) fFw = -1;
  
  Serial.print("FwRev=");
  Serial.print(FwRev);
  Serial.print("  Rl=");
  Serial.println(Rl);
  
  float fRl = Rl;
  fRl -= 512;
  fRl = fRl / 150;
  if (fRl > 1) fRl = 1;
  if (fRl < -1) fRl = -1;
  
//  Serial.print("fFw=");
//  Serial.println(fFw);
  
  // if you're in the center, zero it all out so we don't drift.
  if ((fFw < 0.05) && (fRl < 0.05) &&
      (fFw > -0.05) && (fRl > -0.05))
  {
    fFw = 0; 
    fRl = 0;
  }
  
  float fR=fFw, fL=fFw;
  
//  Serial.print("  fR=");
//  Serial.print(fR);
  
  fR -= fRl;
  fL += fRl;
  
  if (fR > 1) fR = 1;
  if (fR < -1) fR = -1;

  if (fL > 1) fL = 1;
  if (fL < -1) fL = -1;
  
//  Serial.print("  fR=");
//  Serial.println(fR);
  
  MotorTarget[0] = ((float)onFull) * fR;
  MotorTarget[1] = ((float)onFull) * -fL;    
  
}

void updateTargets_Switches()
{
  Serial.println("Reading Switches");
  
  if (digitalRead(FwButton) == LOW)
  {
//    Serial.println("FW");
    MotorTarget[0] = onFull * 1;
    MotorTarget[1] = onFull * -1;    
  }
  else if (digitalRead(RevButton) == LOW)
  {
//    Serial.println("REV");
    MotorTarget[0] = onFull * -1;
    MotorTarget[1] = onFull * 1;    
  }
  else if (digitalRead(RightButton) == LOW)
  {
//    Serial.println("RT");
    MotorTarget[0] = onFull/2 * 1;
    MotorTarget[1] = onFull/2 * 1;    
  }
  else if (digitalRead(LeftButton) == LOW)
  {
//    Serial.println("LT");
    MotorTarget[0] = onFull/2 * -1;
    MotorTarget[1] = onFull/2 * -1;    
  }
  else
  {
//    Serial.println("STOP");
    MotorTarget[0] = 0;
    MotorTarget[1] = 0;    
  }
  
  delay(3);
  
 return;
}
  
void motorOff(int motor)
{
  // Initialize braked
  for (int i=0; i<2; i++)
  {
    digitalWrite(inApin[i], LOW);
    digitalWrite(inBpin[i], LOW);
  }
  analogWrite(pwmpin[motor], 0);
}

// positive velocity is CW, negative is CCW, 0 is motorOff
void smartMotorGo(uint8_t motor, int velocity)
{
  if (velocity == 0)
  {
    motorOff(motor);
    return;
  }

  uint8_t direct;

  if (velocity < 0)
  {
    direct = CCW;
  }  
  else
  {
    direct = CW;
  }
  
  int theSpeed = abs(velocity);
  
  motorGo(motor, direct, theSpeed);
  
  return;
}
/* motorGo() will set a motor going in a specific direction
 the motor will continue going in that direction, at that speed
 until told to do otherwise.
 
 motor: this should be either 0 or 1, will selet which of the two
 motors to be controlled
 
 direct: Should be between 0 and 3, with the following result
 0: Brake to VCC
 1: Clockwise
 2: CounterClockwise
 3: Brake to GND
 
 pwm: should be a value between ? and 1023, higher the number, the faster
 it'll go
 */
void motorGo(uint8_t motor, uint8_t direct, uint8_t pwm)
{
  if (motor <= 1)
  {
    if (direct <=4)
    {
      // Set inA[motor]
      if (direct <=1)
        digitalWrite(inApin[motor], HIGH);
      else
        digitalWrite(inApin[motor], LOW);

      // Set inB[motor]
      if ((direct==0)||(direct==2))
        digitalWrite(inBpin[motor], HIGH);
      else
        digitalWrite(inBpin[motor], LOW);

      analogWrite(pwmpin[motor], pwm);
    }
  }
}
