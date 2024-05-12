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

//float kp=0.1, ki=0.1, kd=0.5;

unsigned long tic=0,tac=0;

long bum_delta = 0;
long target = 0;
long speed = 0;


void setup() {
  Serial.begin(115200);
  
  for (long i = 0; i < 1000; i++)
    Serial.println("!start");

  Legend();
  
  /**/
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

int state = 0;

void next_state()
{
  switch(state)
  {
    case -3:
    {    
      target = 0;
      state = -2;
    }
    break;
    case -2:
    {    
      target += dt / 100;
      if (target >= 1000)
      {
        state = -1;
      }
    }
    break;
    case -1:
    {
      target -= dt / 100;
      if (target <= 0)
      {
        state = -2;
      }
    }  
    break;  
    case 0:
    {    
      if (pot <= 500)
        target = map(pot, 0, 500, 0, 100);
      else
        target = map(pot, 500, 1023, 100, 1000);

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

  pot = constrain(analogRead(A7)-10,0, 1023);
  now = micros();
  dt = (now - prvTime); 
  prvTime = now;

  next_state();


noInterrupts();
  bum_delta = max(tic-tac, now - tic);
interrupts();
  speed = 10000000 / constrain(bum_delta, 0, 500000);
   
  pid = PID();                                        
  
  analogWrite(PWM_PIN_OUT, constrain(pid,0,255)) ;
  Plotter();                                             
  //Trace();                                             
}

void intrupt(){
  tac = tic;
  tic = micros();
}

float PID(){

  error = speed - target;

  P = -0.1 * error;

  errSum = errSum * 0.95 + (error * dt/10000);
  errSum = constrain( errSum, -maxSum, maxSum );

  I = 0.1 * errSum; 
  D = 0.1 * ((error - prevErr) * 100000 / dt);
  prevErr = error;
  
  return P + I + D;
  
}






void Legend()
{
  Serial.println("#pot,target,bum_delta,error,errSum,P,I,D,pid,state");
}

int legend_intersperser = 0;
void Plotter(){
  
  if (legend_intersperser++ % 1000 == 0)
    Legend();
   
  //Serial.print(tic);
  //Serial.print(tac);
  Serial.print(pot);  Serial.print(',');
  Serial.print(target/1000);Serial.print(',');
  Serial.print(bum_delta/1000);Serial.print(',');
  Serial.print(error);  Serial.print(',');
  Serial.print(errSum);  Serial.print(',');
  Serial.print(P);  Serial.print(',');
  Serial.print(I);  Serial.print(',');
  Serial.print(D);  Serial.print(',');
  Serial.print(pid);  Serial.print(',');
  Serial.print(state * 100);  Serial.print(',');
  Serial.println();
}
