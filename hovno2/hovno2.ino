#include <movingAvg.h>                  // https://github.com/JChristensen/movingAvg



int DIR_PIN_BT = A2;                        // Direction button 
int DIR_PIN_OUT = 10;                       // Direction Output
int HALL_PIN = 2;                           // interrupt feedback pin 
int PWM_PIN_OUT = 3;                        // 490Hz PWM Output


movingAvg speed_mvavg(10);                // define the moving average object
movingAvg pot_mvavg(10);                // define the moving average object

int pot;

unsigned long now, prvTime, dt;

float P=0.00, I=0.00, D=0.00, error=0.00, errDiff=0.00, prevErr=0.00, maxSum=255, errSum=0.00, pid=0.00;

unsigned long tic=0,tac=0;

long bum_delta = 0;
float output = 0;
long target = 0;
int speed = 0;
long last_dir_change = 0;

int last_hals = 0;
int dir = 0;



void setup() {
  Serial.begin(115200);
  
  for (long i = 0; i < 1000; i++)
    Serial.println("!start");

  Legend();


  /*haly*/
  pinMode(7,INPUT);
  pinMode(6,INPUT);                                               
  pinMode(5,INPUT);                                               

  pinMode(A1, INPUT);
  /* speed pot */
  pinMode(A7, INPUT);
  
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
        target = map(pot, 100, 500, 0, 1000);
      else
        target = map(pot, 500, 1023, 1000, 37000);

      target = constrain(target, 0, 37000);

      /*if (digitalRead(A1) == 0)
      {
        target = 0;
        state = 1;
      }*/
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

  pot = pot_mvavg.reading(constrain(analogRead(A7)-10,0, 1023));
  now = micros();
  dt = (now - prvTime); 
  prvTime = now;

  next_state();

noInterrupts();
  unsigned long tic2 = tic;
  unsigned long tac2 = tac;
interrupts();

  bum_delta = max(tic2-tac2, now - tic2);
  bum_delta = constrain(bum_delta, 1, 500000);
  speed = 100000000 / bum_delta - 200; /* cca 200 - 20000 bpk */
  speed = speed_mvavg.reading(constrain(speed * dir, 0, 37000));
  

  /* ignore oscillations when wheel is stuck */
  if (now - last_dir_change < 1000 * 50)
    speed = 0;
   
  pid = PID();                                        
  
  analogWrite(PWM_PIN_OUT, constrain(pid,0,255)) ;
  Plotter();                                             
}


int hals;


void intrupt(){
  tac = tic;
  tic = micros();

  int new_dir = get_dir();
  if (new_dir != dir)
  {
    last_dir_change = tic;
    dir = new_dir;
  }
  
  last_hals = hals;
}




/*
The bldc motor with 3 hall sensors sends out a pulse every time one of the hall sensors output flips, and we have an interrupt handler for that. In addition, each of the 3 hall sensor outputs is connected to a digital input pin. In the interrupt handler code, we store the current state of hall sensors in a variable, compare to the previous state, and decide what direction the motor is spinning. */

int hal_magic = 0;

int get_dir()
{
//  Serial.print("#get_dir: ");

  int d7 = digitalRead(7);
  int d6 = digitalRead(6);
  int d5 = digitalRead(5);
  
  //Serial.print(d7); Serial.print(d6); Serial.print(d5); Serial.println(',');
  
  hals = d7 << 2 | d6 << 1 | d5;
  
  if(hals == last_hals)
    return 0;
  
  hal_magic = hals | (last_hals << 3);

  switch(hal_magic)
  {

  /* left to right */
   /* no tail */
    case 0b100010:
    case 0b010001:
    case 0b001100:
   /* new state has tail */
    case 0b100110:
    case 0b010011:
    case 0b001101:
   /* tail catches up */
    case 0b101100:
    case 0b110010:
    case 0b011001:
  /* both tails */
    case 0b101110:
    case 0b110011:
    case 0b011101:
      return 1;
  /* right to left */
  /* no tail */
    case 0b100001:
    case 0b010100:
    case 0b001010:
  /* new state has tail */
    case 0b100101:
    case 0b010110:
    case 0b001011:
   /* tail catches up */
    case 0b110100:
    case 0b011010:
    case 0b101001:
  /* both tails */
    case 0b110101:
    case 0b011110:
    case 0b101011:
      return -1;

  }

  Serial.print("#Unknown stateb: ");
  Serial.println(hal_magic);

  return 0;  
}


float PID(){

  error = target - speed;

  P = 0;//0.1 * error;


  errSum = errSum + (error * dt/1000000);
  errSum = constrain( errSum, 0, 255 );
  I = errSum;


  D = 1 * ((error - prevErr) * 10000 / dt);
  prevErr = error;
  
  return I;
  
}




void Legend()
{
//  Serial.println("#pot,target,bum_delta,speed,dir,error,errSum,P,I,D,pid,state");
  Serial.println("##last_dir_change,target,speed,dir,error,errSum,P,I,D,pid,state,hal_magic");
}

int legend_intersperser = 0;

void Plotter(){
  
  if (legend_intersperser++ % 1000 == 0)
    Legend();
   
  //Serial.print(tic);
  //Serial.print(tac);
  //Serial.print(pot);  Serial.print(',');
  Serial.print(last_dir_change);Serial.print(',');
  
  Serial.print(target);Serial.print(',');
  //Serial.print(bum_delta);Serial.print(',');
  Serial.print(speed);Serial.print(',');
  Serial.print(dir);Serial.print(',');
  Serial.print(error);  Serial.print(',');
  Serial.print(errSum);  Serial.print(',');
  Serial.print(P);  Serial.print(',');
  Serial.print(I);  Serial.print(',');
  Serial.print(D);  Serial.print(',');
  Serial.print(pid);  Serial.print(',');
  Serial.print(state * 100);  Serial.print(',');
  Serial.print(hal_magic);  Serial.print(',');
  Serial.println();
}
