// HB_Motor_PID - with PWM and ZS-X11H controller - 10/02/2024
int pb = A0;                                // Start/Stop pgm button
int DIR_PIN_BT = A2;                        // Direction button 
int DIR_PIN_OUT = 10;                       // Direction Output
int HALL_PIN = 2;                           // interrupt feedback pin 
int PWM_PIN_OUT = 3;                        // 490Hz PWM Output


float speed=100.00;
float speed2;
bool dir=0;                                 // direction, 0=clockwise, 1=counterclockwise
unsigned long now, prvTime, dt;
float P=0.00, I=0.00, D=0.00, error=0.00, errDiff=0.00, prevErr=0.00, maxSum=50, errSum=0.00, pid=0.00;  // PID variables

float kp=2, ki=0.4, kd=0.5;

float target=0;

float pot;
int bum = 0;
unsigned long last_bum_reset_time=0;
float trgt_min=12, trgt_max=25, fb_min=104, fb_max=46; // variables to be modified

void setup() {
  Serial.begin(115200);
  pinMode(pb,INPUT_PULLUP);                                               // Start pgm button
  
  /* autopilot button to ground */
  pinMode(A1, INPUT_PULLUP);
  
  /* speed pot */
  pinMode(A7, INPUT);
  
  pinMode(DIR_PIN_BT,INPUT_PULLUP);                                       // Direction button
  pinMode(DIR_PIN_OUT, OUTPUT);                                           // Direction Output
  pinMode(PWM_PIN_OUT, OUTPUT);                                           // 490Hz PWM Output
  attachInterrupt(digitalPinToInterrupt(HALL_PIN), intrupt, CHANGE);      // Attach Interrupt for motor Hall sensors
  last_bum_reset_time = micros();
  //Serial.print("\n\t To start pgm, click on the Start button"); while(digitalRead(pb)); delay(400); Serial.print("\n\t Started"); // start button
}

int state = 2;

void next_state()
{
  switch(state)
  {
    case 0:
    {
      pot = analogRead(A7);
      if (pot <= 500)
        target = map(pot, 0, 500, 0, 15);
      else
        target = map(pot, 500, 1023, 16, 255);

      target = constrain(target, 0, 255);


      if (digitalRead(A1) == 0)
      {
        target = speed;
        state = 1;
      }
    }
    break;
    case 1:
    
    if (digitalRead(A1) == 1)
    {
      target = 0;
      state = 2;
    }
    break;
    case 2:
    if (analogRead(A7) == 0)
    {
      state = 0;
    }
    if (digitalRead(A1) == 0)
    {
      target = speed;
      state = 1;
    }
    break;
  }
}

void loop() {

  next_state();
  
  now = micros();
  
  dt = (now - prvTime); 
  prvTime = now;


noInterrupts();
  unsigned long bum_delta = now - last_bum_reset_time;
  if (bum_delta > 30000)
  {
    speed = (250000*bum) / bum_delta;
    //unsigned long bum_micros = bum_delta / bum;
    last_bum_reset_time = last_bum_reset_time + 30000;
    bum = 0;
  }
interrupts();
   
  pid = PID();                                        
  pid = target + pid;
  
  analogWrite(PWM_PIN_OUT, round(constrain(pid,0,255)) ); // output PWM PID - constrain speed for security
  Plotter();                                                                // values for the plotter
  //Trace();                                                                 // print variables
}

void intrupt(){
  bum++;
}

float PID(){
  error = target - speed;
  P = kp * error;

  errSum = errSum + (error * dt);
  errSum = constrain( errSum, -maxSum, maxSum );

/*  if (speed < 15)
  {
    I = ki * errSum; 
    D = kd * (error - prevErr) / (dt);  prevErr = error;
  }
  else
  {
    I = 0;
    D = 0;
  }
  */

  //I = constrain((target/speed)*15, 0, 255);
  
  if (error/speed > 10)
  {
    if (speed < 3)
      P += 200;
    if (speed < 15)
      P += 50;
    /*
    if (speed < 15)
      P += 100;
    if (speed < 20)
      P += 80;
    if (speed < 25)
      P += 60;*/
    /*if (speed < 30)
     P += 20;
    if (speed < 35)
      P += 10;
    if (speed < 40)
      P += 10;
    if (speed < 45)
      P += 10;*/
  }
    
    
  
  return P + I + D;
}

void Plotter(){
  Serial.print(0);       Serial.print("  ");                         // to limit plotter scale
  Serial.print(target);  Serial.print("  ");
  Serial.print(speed);   Serial.print("  ");
  //Serial.print(pid);     Serial.print("  ");
  //Serial.print(state * 100);  Serial.print("  ");
  //Serial.print(500);
  Serial.println();
}

void Trace(){
  Serial.print(String() + "\n" 
//                        + "now: "    + now
//                        + " \tlbrt: "    + last_bum_reset_time
                        + " \tbum:  "    + bum
                        + " \tpot: "    + pot
                        + " \ttarget: "    + target
                        + " \tspeed: "  + String(speed)
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
