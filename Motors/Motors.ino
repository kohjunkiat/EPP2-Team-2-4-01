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
 
#define COUNTS_PER_REV      192
#define WHEEL_CIRC          20.42

/*
 *    Vincent's State Variables
 */

volatile unsigned long leftForwardTicks; 
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks; 
volatile unsigned long rightReverseTicks;

volatile unsigned long leftForwardTicksTurns; 
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns; 
volatile unsigned long rightReverseTicksTurns;

volatile unsigned long leftRevs;
volatile unsigned long rightRevs;

volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

/*
 * Setup and start codes for external interrupts and 
 * pullup resistors.
 * 
 */
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
    forwardDist = (unsinged long) ((float) leftForwardTicks / COUNTS_PER_REV * WHEEL_CIRC);
  }
  if (dir == BACKWARD){
    leftReverseTicks++;
    reverseDist = (unsinged long) ((float) leftReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);
  }
  if (dir == LEFT){
    leftReverseTicksTurn++;
  }
  if (dir == RIGHT){
    leftForwardTicksTurn++;
  }
  Serial.print("LEFT: ");
  Serial.println(leftTicks);
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
    rightForwardTicksTurn++;
  }
  if (dir == RIGHT){
    rightReverseTicksTurn++;
  }
  Serial.print("RIGHT: ");
  Serial.println(rightTicks);
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
  
  TIMSK0 |= 0b00000110;
  TIMSK1 |= 0b00000110;
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
  sei();
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

// Turn Vincent left "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Vincent to
// turn left indefinitely.
void left(float ang, float speed)
{
  dir = LEFT;
  
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
}
// Intialize Vincet's internal states

void initializeState()
{
  clearCounters();
}

void setup() {
  // put your setup code here, to run once:

  cli();
  setupEINT();
  setupMotors();
  startMotors();
  enablePullups();
  initializeState();
  sei();
}

void loop() {
/*
//Testing
  forward(0, 100);
  delay(1000);
  reverse(0, 100);
  delay(1000);
  left(0, 100);
  delay(1000);
  right(0, 100);
  delay(1000);
  stop();
  delay(1000);
//End of test */
}
