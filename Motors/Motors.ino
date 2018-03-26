typedef enum
{
 STOP=0,
 FORWARD=1,
 BACKWARD=2,
 LEFT=3,
 RIGHT=4
} TDirection;
volatile TDirection dir = STOP;

/*
 * Vincent's configuration constants
 */
//ALL MEASUREMENTS ARE IN "CM"
//#define PI 3.141592654
#define VINCENT_LENGTH 17.5 
#define VINCENT_BREADTH 12.5 
 
#define COUNTS_PER_REV      192
#define WHEEL_CIRC          20.42 

//VINCENT diagonal
float vincentDiagonal = 0.0;
//VINCENT turning circumference
float vincentCirc = 0.0;

/*
 *    Vincent's State Variables
 */
//variable for moving forward, backward
volatile unsigned long leftForwardTicks; 
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks; 
volatile unsigned long rightReverseTicks;

//Distance moved
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

//To keep track of whether target distance achieved
unsigned long deltaDist;
unsigned long newDist;


//Turning variables
volatile unsigned long leftForwardTicksTurns; 
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns; 
volatile unsigned long rightReverseTicksTurns;

//To keep track of whether target angle achieved
volatile unsigned long deltaTicks;
volatile unsigned long targetTicks;

//To compute angle
volatile unsigned long leftRevs;
volatile unsigned long rightRevs;


/*
 * Setup and start codes for external interrupts and 
 * pullup resistors.
 * 
 */

void setupSerial()
{
  // To replace later with bare-metal.
  Serial.begin(9600);
  Serial.print("Test");
}

void enablePullups()
{
  DDRD &= 0b11110011; //Set PD2 and PD3 to inputs
  PORTD |= 0b00001100; //enable pullup PD2 and PD3
}

// Functions to be called by INT0 and INT1 ISRs.
void leftISR()
{
  if (dir == FORWARD){
    leftForwardTicks++;
    forwardDist = (unsigned long) ((float) leftForwardTicks / COUNTS_PER_REV * WHEEL_CIRC);
  }
  if (dir == BACKWARD){
    leftReverseTicks++;
    reverseDist = (unsigned long) ((float) leftReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);
  }
  if (dir == LEFT){
    leftReverseTicksTurns++;
  }
  if (dir == RIGHT){
    leftForwardTicksTurns++;
  }
 
  //Serial.print("LEFT FORWARD: ");
  //Serial.println(leftForwardTicks);
}

void rightISR()
{
  if (dir == FORWARD){
    rightForwardTicks++;
  }
  if (dir == BACKWARD){
    rightReverseTicks++;
  }
  if (dir == LEFT){
    rightForwardTicksTurns++;
  }
  if (dir == RIGHT){
    rightReverseTicksTurns++;
  }
  //Serial.print("RIGHT FORWARD: ");
  //Serial.println(rightForwardTicks);
}

// Set up the external interrupt pins INT0 and INT1
void setupEINT()
{
  EICRA |= 0b00001010; //configure pins 2 and 3 to be falling edge triggered.
  EIMSK |= 0b00000011; //Enable the INT0 and INT1 interrupts.
}

// Implement the external interrupt ISRs below.
ISR (INT0_vect)
{
  leftISR(); // INT0 ISR should call leftISR
}

ISR (INT1_vect)
{
  rightISR(); //INT1 ISR should call rightISR.
}

/*
 * Vincent's motor drivers.
 * 
 */

//Set up the PWMs to drive the motors.
void setupMotors()
{
  DDRD |= 0b01100000;
  DDRB |= 0b00000110;

  TCNT0 = 0;
  TCNT1H = 0;
  TCNT1L = 0;
  
  OCR0A = 0;
  OCR0B = 0;
  OCR1AH = 0;
  OCR1AL = 0;
  OCR1BH = 0;
  OCR1BL = 0; 
  
  TIMSK0 |= 0b00000000;
  TIMSK1 |= 0b00000000;
  /* Our motor set up is:  
   *    A1IN - Pin 5, PD5, OC0B
   *    A2IN - Pin 6, PD6, OC0A
   *    B1IN - Pin 9, PB1, OC1A
   *    B2In - pIN 10, PB2, OC1B
   */
}

// Start the PWM for Vincent's motors.
void startMotors()
{
  TCCR0B = 0b00000011; //Prescale 64, WGM02 = 0
  TCCR1B = 0b00000011; //Prescale 64, WGM13,12=0
  //sei();
 }

// Convert percentages to PWM values
int pwmVal(float speed)
{
  if(speed < 0.0)
    speed = 0;

  if(speed > 100.0)
    speed = 100.0;

  return (int) ((speed / 100.0) * 255.0);
}

// Move Vincent forward "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// move forward at half speed.
// Specifying a distance of 0 means Vincent will
// continue moving forward indefinitely.
void forward(float dist, float speed)
{
  dir = FORWARD;
  
  if(dist > 0)
    deltaDist = dist;
  else
    deltaDist=9999999;
  newDist=forwardDist + deltaDist;
  
  OCR0A = pwmVal(speed); //LF PWM Val
  OCR1AH = 0;            //RF PWM Val
  OCR1AL = pwmVal(speed);
   
  // For now we will ignore dist and move
  // forward indefinitely. We will fix this
  // in Week 9.

  TCCR0A = 0b10000001; // OC0A PWM LF
  TCCR1A = 0b10000001; // OC1A PWM RF
  PORTD &= 0b11011111; // Clear PD5 LR
  PORTB &= 0b11111011; // Clear PB2 RR
}

// Reverse Vincent "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// reverse at half speed.
// Specifying a distance of 0 means Vincent will
// continue reversing indefinitely.
void reverse(float dist, float speed)
{
  dir = BACKWARD;
   
   if(dist > 0)
    deltaDist = dist;
  else
    deltaDist=9999999;
  newDist = reverseDist + deltaDist;  
  
  OCR0B = pwmVal(speed); //LR PWM Val
  OCR1BH = 0;            //RR PWM Val
  OCR1BL = pwmVal(speed);
  
  // For now we will ignore dist and 
  // reverse indefinitely. We will fix this
  // in Week 9.

  TCCR0A = 0b00100001; // OC0B PWM LR
  TCCR1A = 0b00100001; // OC1B PWM RR
  PORTD &= 0b10111111; // Clear PD6 LF
  PORTB &= 0b11111101; // Clear PB1 RF
}

unsigned long computeDeltaTicks(float ang) 
{
  //ang dist move = dist move in 1 wheel
  unsigned long ticks = (unsigned long) ((ang * vincentCirc * COUNTS_PER_REV)/(360.0 * WHEEL_CIRC));

  return ticks;
}


// Turn Vincent left "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Vincent to
// turn left indefinitely.
void left(float ang, float speed)
{
  dir = LEFT;
 
  if(ang == 0)
    deltaTicks=99999999;
  else
    deltaTicks=computeDeltaTicks(ang);
  
  targetTicks = leftReverseTicksTurns + deltaTicks;
  
  OCR0B = pwmVal(speed); //LR PWM Val
  OCR1AH = 0;            //RF PWM Val
  OCR1AL = pwmVal(speed);
  
  // For now we will ignore ang. We will fix this in Week 9.
  // To turn left we reverse the left wheel and move
  // the right wheel forward.
  
  TCCR0A = 0b00100001; // OC0B PWM LR
  TCCR1A = 0b10000001; // OC1A PWM RF
  PORTD &= 0b10111111; // Clear PD6 LF
  PORTB &= 0b11111011; // Clear PB2 RR
}

// Turn Vincent right "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Vincent to
// turn right indefinitely.
void right(float ang, float speed)
{
  dir = RIGHT;
    
  if(ang == 0)
    deltaTicks=99999999;
  else
    deltaTicks=computeDeltaTicks(ang);
  targetTicks = rightReverseTicksTurns + deltaTicks;
  
  OCR0A = pwmVal(speed); //LF PWM Val
  OCR1BH = 0;            //RR PWM Val
  OCR1BL = pwmVal(speed);
  
  // For now we will ignore ang. We will fix this in Week 9.
  // To turn right we reverse the right wheel and move
  // the left wheel forward.
  
  TCCR0A = 0b10000001; // OC0A PWM LF
  TCCR1A = 0b00100001; // OC1B PWM RR
  PORTD &= 0b11011111; // Clear PD5 LR
  PORTB &= 0b11111101; // Clear PB1 RF
}

// Stop Vincent.
void stop()
{
  dir = STOP;

  PORTD &= 0b10111111; // Clear PD6 LF
  PORTB &= 0b11111101; // Clear PB1 RF
  PORTD &= 0b11011111; // Clear PD5 LR
  PORTB &= 0b11111011; // Clear PB2 RR
}

/*
 * Vincent's setup and run codes
 * 
 */

// Clears all our counters
void clearCounters()
{
  leftForwardTicks=0;
  rightForwardTicks=0;
  leftReverseTicks=0;
  rightReverseTicks=0;
  
  leftForwardTicksTurns=0;
  rightForwardTicksTurns=0;
  leftReverseTicksTurns=0;
  rightReverseTicksTurns=0;
  
  leftRevs=0;
  rightRevs=0;
  
  forwardDist=0;
  reverseDist=0;

  deltaTicks=0;
  targetTicks=0;

  newDist=0;
  deltaDist=0;
}

// Clears one particular counter
void clearOneCounter(int which)
{
  clearCounters();
 /* switch(which)
  {
    case 0:
      clearCounters();
      break;

    case 1:
      leftTicks=0;
      break;

    case 2:
      rightTicks=0;
      break;

    case 3:
      leftRevs=0;
      break;

    case 4:
      rightRevs=0;
      break;

    case 5:
      forwardDist=0;
      break;

    case 6:
      reverseDist=0;
      break;*/
}

// Intialize Vincent's internal states

void initializeState()
{
  clearCounters();
}

void setup() {
  // put your setup code here, to run once:

  cli();
  setupEINT();
  setupSerial();
  setupMotors();
  startMotors();
  enablePullups();
  initializeState();
  sei();

  //compute vincent diagonal
  vincentDiagonal = sqrt((VINCENT_LENGTH * VINCENT_LENGTH) + (VINCENT_BREADTH *VINCENT_BREADTH));
  vincentCirc = PI * vincentDiagonal;
  
}

void loop() {
  
  //forward(0, 100);

//FORWARD, BACKWARD MOVING DISTANCE CHECKING
  if(deltaDist > 0) 
  {
    if(dir==FORWARD) 
    {
      if(forwardDist > newDist) 
      {
        deltaDist=0;
        newDist=0;
        stop();
      }
    }
  else
    if(dir == BACKWARD) 
    {
      if(reverseDist > newDist) 
      {
        deltaDist=0;
        newDist=0;
        stop();
      }
    }
  else
    if(dir == STOP) 
    {
      deltaDist=0;
      newDist=0;
      stop();
    }
  }
  //TURNING LEFT, RIGHT ANGULAR CHECKING
  if(deltaTicks > 0)  
  {
    if(dir==LEFT) 
    {
       if(leftReverseTicksTurns >= targetTicks)
       {
        deltaTicks=0;
        targetTicks=0;
        stop();
       }
    }
  else
    if(dir==RIGHT)
    {
      if(rightReverseTicksTurns >= targetTicks) 
      {
        deltaTicks=0;
        targetTicks=0;
        stop();
      }
    }
  else
    if(dir == STOP) 
    {
      deltaTicks=0;
      targetTicks=0;
      stop();
    }
  }
}
