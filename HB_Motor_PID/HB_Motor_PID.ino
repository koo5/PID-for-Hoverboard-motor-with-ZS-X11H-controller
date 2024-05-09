// HB_Motor_PID - with PWM and ZS-X11H controller - 10/02/2024
int pb = A0;                                // Start/Stop pgm button
int DIR_PIN_BT = A2;                        // Direction button 
int DIR_PIN_OUT = 10;                       // Direction Output
int HALL_PIN = 2;                           // interrupt feedback pin 
int PWM_PIN_OUT = 3;                        // 490Hz PWM Output

volatile float tic, tac;
float delta=100.00;
float delta2, delta3;
bool dir=0;                                 // direction, 0=clockwise, 1=counterclockwise
float now, prvTime, dt;                     // time variables
float P=0.00, I=0.00, D=0.00, error=0.00, errDiff=0.00, prevErr=0.00, maxSum=50, errSum=0.00, pid=0.00;  // PID variables

float kp=0.4, ki=0.9, kd=0.001, target=15, trgt_min=12, trgt_max=25, fb_min=104, fb_max=46; // variables to be modified

void setup() {
  Serial.begin(115200);
  pinMode(pb,INPUT_PULLUP);                                               // Start pgm button
  pinMode(DIR_PIN_BT,INPUT_PULLUP);                                       // Direction button
  pinMode(DIR_PIN_OUT, OUTPUT);                                           // Direction Output
  pinMode(PWM_PIN_OUT, OUTPUT);                                           // 490Hz PWM Output
  attachInterrupt(digitalPinToInterrupt(HALL_PIN), intrupt, RISING);      // Attach Interrupt for motor Hall sensors
  //Serial.print("\n\t To start pgm, click on the Start button"); while(digitalRead(pb)); delay(400); Serial.print("\n\t Started"); // start button
  tic = tac = millis();
}

void loop() {
  now = millis(); dt = (now - prvTime) /1000.00; prvTime = now;             // time between two loops
  if( ! digitalRead(DIR_PIN_BT)){ dir = dir ^ 1; delay(300); analogWrite(PWM_PIN_OUT,0); digitalWrite(DIR_PIN_OUT, dir); } // change direction when button pushed

noInterrupts();
  if (tic < millis() - 300)
    delta = 300;
  else
    delta = tic - tac;
interrupts();
    
  delta2 = map( delta, 300, 0, 0, 100);
  delta3 = constrain(delta2, 0, 255);
  
  
  pid = PID();                                        
  pid = target + error;
  
  analogWrite(PWM_PIN_OUT, round(constrain(pid,0,255)) ); // output PWM PID - constrain speed for security
  //Plotter();                                                                // values for the plotter
  Trace();                                                                  // print variables
}

void intrupt(){
  
  tac = tic;
  tic = millis();

}

float PID(){
  error = target - delta3;
  P = kp * error;
  I = ki * (errSum = errSum + (error * dt)); errSum = constrain( errSum, -maxSum, maxSum );
  D = kd * (error - prevErr) / dt;  prevErr = error;
  return P + I + D;
}

void Plotter(){
  Serial.print(0);            Serial.print("  ");                         // to limit plotter scale
  Serial.print(delta3,3);   Serial.print("  ");
  Serial.print(pid,3);        Serial.print("  ");
  Serial.println(trgt_max,0);                                             // to limit plotter scale
}

void Trace(){
  Serial.print(String() + "\n" 
                        + "  target: "    + target
                        + "  delta: "  + String(delta,3)
                        + "  delta2: "  + String(delta2,3)
                        + "  delta3: "  + String(delta3,3)
                        + "  pid: "       + String(pid,3)
                        + "  error: "     + String(error,3)
                        + "  prevErr: "   + String(prevErr,3)
                        + "  errSum: "    + String(errSum,3)
                        + "  P: "         + String(P,3)
                        + "  I: "         + String(I,3)
                        + "  D: "         + String(D,3)
                        + "  dir: "       + dir
                        + "  dt: "        + String(dt,3)
                        );
}
