// HB_Motor_PID - with PWM and ZS-X11H controller - 10/02/2024
int pb = A0;                                // Start/Stop pgm button
int DIR_PIN_BT = A2;                        // Direction button 
int DIR_PIN_OUT = 10;                       // Direction Output
int HALL_PIN = 2;                           // interrupt feedback pin 
int PWM_PIN_OUT = 3;                        // 490Hz PWM Output


float pot;

bool dir=0;                                 // direction, 0=clockwise, 1=counterclockwise
unsigned long now, prvTime, dt;

float P=0.00, I=0.00, D=0.00, error=0.00, errDiff=0.00, prevErr=0.00, maxSum=200, errSum=0.00, pid=0.00;

float kp=0.2, ki=0.2, kd=0.1;

unsigned long tic=0,tac=0;

long bum_delta = 0;
long target=0;



void setup() {
  Serial.begin(115200);
Serial.println();Serial.println();Serial.println();Serial.println();Serial.println();Serial.println();Serial.println();Serial.println();Serial.println();
  pinMode(pb,INPUT_PULLUP);                                               // Start pgm button
  
  /* autopilot button to ground */
  pinMode(A1, INPUT_PULLUP);
  
  /* speed pot */
  pinMode(A7, INPUT);
  
  pinMode(DIR_PIN_BT,INPUT_PULLUP);                                       // Direction button
  pinMode(DIR_PIN_OUT, OUTPUT);                                           // Direction Output
  pinMode(PWM_PIN_OUT, OUTPUT);                                           // 490Hz PWM Output
  attachInterrupt(digitalPinToInterrupt(HALL_PIN), intrupt, CHANGE);      // Attach Interrupt for motor Hall sensors

}

int state = 2;

void next_state()
{
  switch(state)
  {
    case 0:
    {    
      if (pot <= 500)
        target = map(pot, 0, 500, 300000, 100000);
      else
        target = map(pot, 500, 1023, 100000, 0);

      target = constrain(target, 0, 300000);

      if (digitalRead(A1) == 0)
      {
        target = bum_delta;
        state = 1;
      }
    }
    break;
    case 1:
    
    if (digitalRead(A1) == 1)
    {
      target = 300000;
      state = 2;
    }
    break;
    case 2:
    if (pot == 0)
    {
      state = 0;
    }
    if (digitalRead(A1) == 0)
    {
      target = bum_delta;
      state = 1;
    }
    break;
  }
}

void loop() {
  pot = analogRead(A7);

  next_state();
  
  now = micros();
  
  dt = (now - prvTime); 
  prvTime = now;

noInterrupts();
  bum_delta = max(tic-tac, now - tic);
interrupts();
  bum_delta = constrain(bum_delta, 0, 300000);
   
  pid = PID();                                        
  
  analogWrite(PWM_PIN_OUT, round(constrain(pid,0,255)) );
  Plotter();                                             
  //Trace();                                             
}

void intrupt(){
  tac = tic;
  tic = micros();
}

float PID(){

  error = (bum_delta - target)/1000;

  P = kp * error;

  errSum = errSum + (error * dt);
  errSum = constrain( errSum, -maxSum, maxSum );

  I = ki * errSum; 
  D = kd * (error - prevErr) / (dt);  prevErr = error;

  return P + I + D;
}



void Plotter(){
  Serial.print("nula:"); Serial.print(0);       
  //Serial.print(" tic:"); Serial.print(tic);
  //Serial.print(" tac:"); Serial.print(tac);
  //Serial.print(" pot:"); Serial.print(pot);
  Serial.print(" error:"); Serial.print(error);
  Serial.print(" target:"); Serial.print(target/1000);
  Serial.print(" bum_delta:"); Serial.print(bum_delta/1000);
  Serial.print(" pid:"); Serial.print(pid);
  Serial.print(" state:"); Serial.print(state * 100);
  //Serial.print(500);
  Serial.println();
}

void Trace(){
  Serial.print(String() + "\n" 
//                        + "now: "    + now
//                        + " \tlbrt: "    + last_bum_reset_time
//                        + " \tbum:  "    + bum
                        + " \tpot: "    + pot
                        + " \ttarget: "    + target
                        + " \tbd: "  + String(bum_delta)
//                        + " \tspeed2: "  + String(speed2)
                        + " \tpid: "       + String(pid)
                        + " \terror: "     + String(error)
                        + " \tprevErr: "   + String(prevErr)
                        + " \terrSum: "    + String(errSum)
                        + " P: "         + String(P)
                        + " I: "         + String(I)
                        + " D: "         + String(D)
//                        + "  dir: "       + dir
                        + " dt: "        + String(dt)
                        );
}
