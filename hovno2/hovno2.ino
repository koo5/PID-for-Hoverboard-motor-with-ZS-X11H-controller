
int DIR_PIN_BT = A2;                        // Direction button 
int DIR_PIN_OUT = 10;                       // Direction Output
int HALL_PIN = 2;                           // interrupt feedback pin 
int PWM_PIN_OUT = 3;                        // 490Hz PWM Output


float pot;

unsigned long now, prvTime, dt;

float P=0.00, I=0.00, D=0.00, error=0.00, errDiff=0.00, prevErr=0.00, maxSum=255, errSum=0.00, pid=0.00;

unsigned long tic=0,tac=0;

long bum_delta = 0;
float output = 0;
long target = 0;
long speed = 0;


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
        target = map(pot, 0, 500, 0, 1000);
      else
        target = map(pot, 500, 1023, 1000, 20000);

      target = constrain(target, 0, 20000);

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

  pot = 0.9*pot + 0.1*constrain(analogRead(A7)-10,0, 1023);
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
  speed = constrain(speed * dir, 0, 20000);
   
  pid = PID();                                        
  
  analogWrite(PWM_PIN_OUT, constrain(pid,0,255)) ;
  Plotter();                                             
}


int hals;


void intrupt(){
  tac = tic;
  tic = micros();

  int new_dir = get_dir();
  if (new_dir != 0)
    dir = new_dir;
  last_hals = hals;
}

int get_dir()
{
  int d7 = digitalRead(7);
  int d6 = digitalRead(6);
  int d5 = digitalRead(5);
  
  //Serial.print(d7); Serial.print(d6); Serial.print(d5); Serial.println(',');
  
  hals = d7 << 2 | d6 << 1 | d5;
  
  if(hals == last_hals)
    return 0;
  
  switch(hals | (last_hals << 3))
  {
    case 0b100110:
      return 1;
    case 0b100010:
      return 1;
    case 0b100001:
      return -1;
    case 0b100011:
      return -1;


    case 0b010110:
      return 1;
    case 0b010010:
      return 1;
    case 0b010001:
      return -1;
    case 0b010011:
      return -1;


      
    case 0b110010:
      return 1;
    case 0b110001:
      return -1;
    case 0b010001:
      return -1;
    case 0b010100:
      return 1;
      
    case 0b010011:
      return 1;
    case 0b010110:
      return -1;
      
    case 0b001011:
      return -1;
    case 0b001100:
      return 1;
    case 0b001110:
      return 1;
  }
  return 0;  
}


float PID(){

  error = target - speed;

  P = 0;//0.1 * error;


  errSum = errSum + (error * dt/100000000);
  errSum = constrain( errSum, 0, 255 );
  I = errSum;


  D = 1 * ((error - prevErr) * 10000 / dt);
  prevErr = error;
  
  return I;
  
}






void Legend()
{
//  Serial.println("#pot,target,bum_delta,speed,dir,error,errSum,P,I,D,pid,state");
  Serial.println("#target,speed,dir,error,errSum,I,pid,state");
}

int legend_intersperser = 0;
void Plotter(){
  
  if (legend_intersperser++ % 1000 == 0)
    Legend();
   
  //Serial.print(tic);
  //Serial.print(tac);
  //Serial.print(pot);  Serial.print(',');
  Serial.print(target);Serial.print(',');
  //Serial.print(bum_delta);Serial.print(',');
  Serial.print(speed);Serial.print(',');
  Serial.print(dir);Serial.print(',');
  Serial.print(error);  Serial.print(',');
  Serial.print(errSum);  Serial.print(',');
  //Serial.print(P);  Serial.print(',');
  Serial.print(I);  Serial.print(',');
  //Serial.print(D);  Serial.print(',');
  Serial.print(pid);  Serial.print(',');
  Serial.print(state * 100);  Serial.print(',');
  Serial.println();
}
