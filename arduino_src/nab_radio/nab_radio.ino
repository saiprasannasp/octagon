#define MAX_PWM 243
#define MID_PWM 143
#define MIN_PWM 43

#define MAX_MAP 100
#define MIN_MAP -100

#define LEFT_WHEEL_PIN 4
#define RIGHT_WHEEL_PIN 5

#define CH1_PIN 7
int ch1; 
#define CH1_PIN_HIGH 1904
#define CH1_PIN_LOW 1058

#define CH2_PIN 6
int ch2; 
#define CH2_PIN_HIGH 1903
#define CH2_PIN_LOW 1049

#define CH3_PIN 8
int ch3; 
#define CH3_PIN_HIGH 1903
#define CH3_PIN_LOW 1048

#define LIGHT_PIN 13

#include <nabSerialCom.h>

int throttle;
int turn;
bool disabled = false;
extern byte bkp_TCCR1B, bkp_TCCR0B, bkp_TCCR2B;

int pwm_left = 3;
int pwm_right = 11;

//int dir_a = 12;  //direction control for motor outputs
//int dir_b = 13;  //direction control for motor outputs
int prev_ch1=0;
int prev_ch2=0;
int avg_throttle[5] = {0, 0, 0, 0, 0};
int avg_turn[5] = {0, 0, 0, 0, 0};
int countl=0;

int count_blink=0;
bool on = true;

void setup() {

  pinMode(LEFT_WHEEL_PIN, OUTPUT);
  pinMode(RIGHT_WHEEL_PIN, OUTPUT);
  
  pinMode(CH1_PIN, INPUT);
  pinMode(CH2_PIN, INPUT);
  pinMode(CH3_PIN, INPUT);
  
  Serial.begin(9600);

  pinMode(pwm_left, OUTPUT);  
  pinMode(pwm_right, OUTPUT);
  
  setPwmFrequency(pwm_left, 128); // adjusts the frequency of the pwm sent to the b-bridge
  setPwmFrequency(pwm_right, 128); // adjusts the frequency of the pwm sent to the b-bridge
  
  analogWrite(pwm_left, MID_PWM);  
  analogWrite(pwm_right, MID_PWM);
  
  pinMode(LIGHT_PIN, OUTPUT);
}

void loop() {
  
  //unsigned long st = millis();
  ch3 = pulseIn(CH3_PIN, HIGH); 
  if(ch3>CH3_PIN_HIGH)
   ch3 = CH3_PIN_HIGH;
  else if (ch3 < CH3_PIN_LOW)
   ch3 = CH3_PIN_LOW;
  
  //it auto is enabled then donot send any pwm value.
  if(ch3 >= 1500) {
    disabled = true;
    TCCR1B = bkp_TCCR1B;
    TCCR1B = bkp_TCCR0B; 
    TCCR2B = bkp_TCCR2B;    
    pinMode(pwm_left, INPUT);  
    pinMode(pwm_right, INPUT);
    digitalWrite(LEFT_WHEEL_PIN, HIGH);
    digitalWrite(RIGHT_WHEEL_PIN, HIGH);
    //Serial.println("HI");
    
    count_blink++;
    if(count_blink>50  && on)
    {
      digitalWrite(LIGHT_PIN, LOW);
      on = false;
    }
    
    if(count_blink>100)
    {
      count_blink = 0;
      on = true;
      digitalWrite(LIGHT_PIN, HIGH);
    }
    
    return;
  }
  
  if(disabled)
  {
    disabled = false;
    memset(avg_throttle, 0, sizeof(int)*5);
    memset(avg_turn, 0, sizeof(int)*5); 
    digitalWrite(LEFT_WHEEL_PIN, LOW);
    digitalWrite(RIGHT_WHEEL_PIN, LOW);
    pinMode(pwm_left, OUTPUT);  
    pinMode(pwm_right, OUTPUT);
    setPwmFrequency(pwm_left, 128);
    setPwmFrequency(pwm_left, 128);
    
    digitalWrite(LIGHT_PIN, HIGH);
  }
  
  ch1 = pulseIn(CH1_PIN, HIGH);
  if(ch1>CH1_PIN_HIGH)
   ch1 = CH1_PIN_HIGH;
  else if (ch1 < CH1_PIN_LOW)
   ch1 = CH1_PIN_LOW;
  throttle = map(ch1, CH1_PIN_LOW, CH1_PIN_HIGH, MIN_MAP, MAX_MAP);
  throttle = constrain(throttle, MIN_MAP, MAX_MAP);
  
  /*Here we're determining whether a left or a right turn is being 
  executed*/
  ch2 = pulseIn(CH2_PIN, HIGH); 
  if(ch2>CH2_PIN_HIGH)
   ch2 = CH2_PIN_HIGH;
  else if (ch2 < CH2_PIN_LOW)
   ch2 = CH2_PIN_LOW;
  turn = map(ch2,CH2_PIN_LOW,CH2_PIN_HIGH, MIN_MAP, MAX_MAP);
  turn = constrain(turn, MIN_MAP, MAX_MAP);
  
  int loc = countl%5;
  avg_throttle[loc]=throttle;
  avg_turn[loc]=turn;
  countl++;
  int throttle_sum=0;
  int turn_sum=0;
  for(int i=0; i<5; i++)
  {
    throttle_sum += avg_throttle[i];
    turn_sum += avg_turn[i];
    throttle = throttle_sum/5;
    turn = turn_sum/5;
  }
  
  //unsigned long ed = millis();
 
  //if(ch1<CH1_PIN_LOW || ch1>CH1_PIN_HIGH)
    //ch1 =  prev_ch1;
  //if(ch2<CH2_PIN_LOW || ch2>CH2_PIN_HIGH)
    //ch2 =  prev_ch2;
  
  //prev_ch1 = ch1;
  //prev_ch2 = ch2;  
  /*This is where we do some mixing, by subtracting our "turn" 
  variable from the appropriate motor's speed we can execute
  a turn in either direction*/
  int left = 0;
  int right = 0;
  
  left = MID_PWM-throttle+turn;
  right = MID_PWM-throttle-turn;
  
  if(left>MID_PWM-5 && left<MID_PWM+5) {
    left=MID_PWM;
  } else if(left<MIN_PWM){
    left=MIN_PWM;
  } else if (left > MAX_PWM) {
    left = MAX_PWM;
  }
  
  if(right>MID_PWM-5 && right<MID_PWM+5) {
    right=MID_PWM;
  } if(right<MIN_PWM){
    right=MIN_PWM;
  } else if (right > MAX_PWM) {
    right = MAX_PWM;
  }
  
    
  analogWrite(pwm_left, left); 
  analogWrite(pwm_right, right);
 
  //Serial.print("1:");
  //Serial.println(ch1);
  //Serial.print("move:"); //Serial debugging stuff
  //Serial.println(move);
  
  //Serial.print("turn:"); //Serial debugging stuff
  //Serial.println(turn);
  
  //Serial.print("move-turn:"); //Serial debugging stuff
  //Serial.println(move-turn);
  
  Serial.print(left); //Serial debugging stuff
  Serial.print("  ");
  Serial.println(right);

  //delay(100);

}

