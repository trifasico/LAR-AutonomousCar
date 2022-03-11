#include <Servo.h>
#include <HCSR04.h>

const int BUFFER_SIZE = 20;
char Received[BUFFER_SIZE];
float Ki = 3;
float Kp = 2;
float Kd = 0.15;

// defines pins numbers
#define brake_position 0
#define unbrake_position 180

const byte interruptPin = 21;
const int brakePin = 2;
const int in1Pin = 5;
const int in2Pin = 3;
const int pwmPin = 4;
const int directionPin = 6;
const int buzzerPin = 7;
const int RPin = 42;
const int GPin = 44;
const int BPin = 40;
const int BatteryPin = A9;
byte triggerPin = 50;
byte echoLPin = 30;
byte echoMPin = 48;
byte echoRPin = 52;

// defines variables Movement
Servo S_direction; // Direction Servo
Servo S_brake;     // Brake Servo

volatile int Velocity;
float Velocity_error;
float Velocity_last_error = 0;
float Velocity_Ki_error;
float Velocity_rate_error;
float Velocity_out;
int Direction;

// defines variables UltraSound
long duration;
int distance;
float Battery;
float Low_Battery = 10.0;

// defines variables Sound
const int Sound_up[6][2] = {{200, 3}, {300, 3}, {400, 3}, {0, 0}, {0, 0}, {0, 0}};
const int Sound_down[6][2] = {{400, 3}, {300, 3}, {200, 3}, {0, 0}, {0, 0}, {0, 0}};
const int Sound_emergency[6][2] = {{400, 3}, {300, 3}, {400, 3}, {300, 3}, {400, 3}, {300, 3}};
int Sound_to_play[6][2];
volatile int notes_counter = 0, time_counter = 0;
volatile int encoder_counter = 0, last_encoder_counter = 0;

volatile int R_value = 200;
volatile int M_value = 200;
volatile int L_value = 200;
volatile int R_last_value = 200;
volatile int M_last_value = 200;
volatile int L_last_value = 200;
byte *echoPins = new byte[3]{echoLPin, echoMPin, echoRPin};

long last_read_time = 20000; // wait 20 Seconds to boot

int Buffer_counter = 0;
int PIDEnable = 1;

void Change_Warnings(int state)
{
  switch (state)
  {
  case 0:
    memcpy(Sound_to_play, Sound_up, sizeof(Sound_up));
    digitalWrite(RPin, 1); // GREEN
    digitalWrite(GPin, 0);
    digitalWrite(BPin, 1);
    time_counter = 0;
    notes_counter = 0;
    break;
  case 1:
    memcpy(Sound_to_play, Sound_down, sizeof(Sound_down));
    digitalWrite(RPin, 0); // YELLOW
    digitalWrite(GPin, 0);
    digitalWrite(BPin, 1);
    time_counter = 0;
    notes_counter = 0;
    break;
  case 2:

    memcpy(Sound_to_play, Sound_emergency, sizeof(Sound_emergency));
    digitalWrite(RPin, 0); // RED
    digitalWrite(GPin, 1);
    digitalWrite(BPin, 1);
    time_counter = 0;
    notes_counter = 0;
    break;
  }
}
bool STOP = false;
void Brake(int state)
{
  if (state == 0)
  {
    S_brake.write(unbrake_position);
    Velocity_Ki_error = 0;
    Velocity_rate_error = 0;
    STOP = false;
    digitalWrite(in1Pin, 1);
    digitalWrite(in2Pin, 0);
  }
  else
  {
    STOP = true;
    digitalWrite(in1Pin, 1);
    digitalWrite(in2Pin, 1);
    analogWrite(pwmPin, 0);
    S_brake.write(brake_position);
  }
}

void Play_sound()
{
  if (notes_counter < 6)
  {
    if (Sound_to_play[notes_counter][0] != 0)
      tone(buzzerPin, Sound_to_play[notes_counter][0]);

    ++time_counter;
    if (time_counter > Sound_to_play[notes_counter][1])
    {
      ++notes_counter;
      time_counter = 0;
    }
  }
  else
    noTone(buzzerPin);
}

void enc_increment()
{
  ++encoder_counter;
}
void setup()
{
  // === Init Serial Port ===
  Serial.begin(9600);
  while (!Serial)
    ;

  // === Init Pinout ===
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), enc_increment, RISING);
  pinMode(pwmPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(RPin, OUTPUT);
  pinMode(GPin, OUTPUT);
  pinMode(BPin, OUTPUT);
  pinMode(BPin, OUTPUT);
  S_direction.attach(directionPin);
  S_brake.attach(brakePin);
  S_direction.write(90);
  S_brake.write(unbrake_position);

  // === Initialisation class HCSR04 === (trig pin , echo pin, number of sensor)
  HCSR04.begin(triggerPin, echoPins, 3);
  delay(3000);

  // === Timers ===
  cli();
  // set timer4 interrupt at 10Hz
  TCCR4A = 0; // set entire TCCR1A register to 0
  TCCR4B = 0; // same for TCCR1B
  TCNT4 = 0;  // initialize counter value to 0
  // set compare match register for 1hz increments
  OCR4A = 5562 / 1; // = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR4B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR4B |= (1 << CS12) | (1 << CS10);
  // enable timer compare interrupt
  TIMSK4 |= (1 << OCIE4A);
  // TIMSK4 |= (1 << TOIE4);   // enable timer overflow interrupt

  // Clear registers
  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3 = 0;
  OCR3A = 6249;            // 10 Hz (16000000/((6249+1)*256))
  TCCR3B |= (1 << WGM32);  // CTC
  TCCR3B |= (1 << CS32);   // Prescaler 256
  TIMSK3 |= (1 << OCIE3A); // Output Compare Match A Interrupt Enable

  sei(); // allow interrupts
}

void loop()
{

  // === Serial Read ===
  while (Serial.available() > 0)
  {
    last_read_time = millis();
    char Byte = Serial.read();
    // Serial.println(Byte);
    if (Byte != '\r')
    {
      Received[Buffer_counter] = Byte;
      Buffer_counter++;
    }
    else
    {
      Buffer_counter = 0;
      char event = 0;
      int arg1 = 0, arg2 = 0;
      int n = sscanf(Received, "%c.%d", &event, &arg1);
      // Serial.println(event);
      switch (event)
      {
      case 'V': // === Velocity ===
        if (n == 2)
        { // Check received arguments
          if (arg1 > 100)
          {
            arg1 = 100;
          }
          else if (arg1 < -100)
          {
            arg1 = -100;
          }
          if (arg1 < 0)
          {
            Velocity = -arg1;
            digitalWrite(in1Pin, 0);
            digitalWrite(in2Pin, 1);
            // Serial.print(Velocity);
            // analogWrite(pwmPin,Velocity);
          }
          else
          {
            Velocity = arg1;
            digitalWrite(in1Pin, 1);
            digitalWrite(in2Pin, 0);
            // analogWrite(pwmPin,Velocity);
          }
        }
        break;

      case 'D': // === Direction ===
        if (arg1 > 100)
        {
          arg1 = 100;
        }
        else if (arg1 < -100)
        {
          arg1 = -100;
        }

        int f_Direction = map(arg1, -100, 100, 65, 115); // scale it for use with the servo (value between 65º and 115º)
        // Serial.println(arg1);
        // Serial.println(f_Direction);
        S_direction.write(f_Direction); // sets the servo position according to the scaled value
        break;

      case 'B': // === Brake ===
        if (n == 2)
          Brake(arg1);
        break;

      case 'S': // === State ===
        if (n == 2)
        {
          Change_Warnings(arg1);
          // Serial.println("STATE");
        }
        break;

      case 'L': // === Low Battery ===  (0 - 255) == (0V - 25,5V) ===
        if (n == 2)
        {
          Low_Battery = (float)arg1 / 10;
        }
        break;

      case 'T': // === Update ===
        Serial.print("T.");
        Serial.println(Battery);
        break;

      case 'U': // === Ultrasounds ===  // Tested
        Serial.print("U.");
        Serial.print(L_last_value);
        Serial.print(".");
        Serial.print(M_last_value);
        Serial.print(".");
        Serial.println(R_last_value);
        break;
        
      case 'P': // === Ultrasounds ===  // Teste
        PIDEnable = arg1;
        break;
      }

      memset(Received, 0, sizeof(Received));
    }
  }
  double *distances = HCSR04.measureDistanceCm();
  L_last_value = (distances[0] <= 0) ? L_last_value : distances[0];
  M_last_value = (distances[1] <= 0) ? M_last_value : distances[1];
  R_last_value = (distances[2] <= 0) ? R_last_value : distances[2];

  // === TIMEOUT === 1 Second // Tested
  /*
  if(((long)millis()-last_read_time)>2000){
    Brake(1);
    Serial.println("O");
    last_read_time=millis();}
  */
  //
  //Serial.println();
}

// =========== ISR FUNCTIONS ===============

int mili_counter = 0;
ISR(TIMER4_COMPA_vect)
{
  Play_sound();
  Battery = (float)map(analogRead(BatteryPin), 0, 300, 0, 1000) / 100;
  if (Low_Battery > Battery)
  {
    digitalWrite(RPin, 1); // Blue
    digitalWrite(GPin, 1);
    digitalWrite(BPin, 0);
  }
}

ISR(TIMER3_COMPA_vect)
{ // timer1 interrupt 1Hz toggles pin 13 (LED)
  // generates pulse wave of frequency 1Hz/2 = 0.5kHz (takes two cycles for full wave- toggle high then toggle low)

  if (Velocity == 0 || STOP)
  {
    // Serial.println(encoder_counter);
    analogWrite(pwmPin, 0);
    encoder_counter = 0;
  }
  else
  {
    if(PIDEnable == 1){
      Velocity_error = (Velocity - (encoder_counter / 1.8441));
      encoder_counter = 0;
      Velocity_Ki_error += Velocity_error * 0.1;
      Velocity_rate_error = -(Velocity_error - Velocity_last_error) / 0.1;

      Velocity_out = Kp * Velocity_error + Ki * Velocity_Ki_error + Kd * Velocity_rate_error;
    } else {
      Velocity_out = Velocity;
    }
    if (Velocity_out > 255)
    {
      Velocity_out = 255;
    }
    else if (Velocity_out < 0)
    {
      Velocity_out = 0;
    }
    // Serial.print("Kp*Velocity_error");
    // Serial.print(Kp*Velocity_error);
    // Serial.print(" Ki:");
    // Serial.print(Ki*Velocity_Ki_error);
    // Serial.print(" Kd:");
    // Serial.println(Kd*Velocity_rate_error);
    // Serial.print(" Out:");
    analogWrite(pwmPin, Velocity_out);
    // Serial.println(Velocity_out);

    Velocity_last_error = Velocity_error;
  }
  // analogWrite(pwmPin,(int)Ki*(Velocity-(encoder_counter*0.542)));

  // Serial.print(Battery);
  // Serial.print("   ");
  // Serial.println((float)map(Battery,0,300,0,1000)/100);
}
