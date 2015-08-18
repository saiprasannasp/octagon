#define CH1_PIN 7
int ch1, max_ch1, min_ch1;

#define CH2_PIN 6
int ch2, max_ch2, min_ch2;


#define CH3_PIN 8
int ch3, max_ch3, min_ch3;

bool change = false;

void setup() {
  // put your setup code here, to run once:
  max_ch1=max_ch2=max_ch3=0;
  min_ch1=min_ch2=min_ch3=5000;
  
  pinMode(CH1_PIN, INPUT);
  pinMode(CH2_PIN, INPUT);
  pinMode(CH3_PIN, INPUT);

  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  ch3 = pulseIn(CH3_PIN, HIGH); 
  ch1 = pulseIn(CH1_PIN, HIGH);
  ch2 = pulseIn(CH2_PIN, HIGH);
   
  if(max_ch1<ch1){
    max_ch1=ch1;
    change = true;
  }
  if(min_ch1>ch1){
    min_ch1=ch1;
    change = true;  
  }

  if(max_ch2<ch2) {
    max_ch2=ch2;
    change = true;
  }
  if(min_ch2>ch2) {
    min_ch2=ch2;
    change = true;
  }
    
  if(max_ch3<ch3) {
    max_ch3=ch3;
    change = true;
  }
  if(min_ch3>ch3) {
    min_ch3=ch3;
    change = true;
  }
  
  if(change) {
    Serial.print("max : ");
    Serial.print(max_ch1);
    Serial.print("  ");
    Serial.print(max_ch2);
    Serial.print("  ");
    Serial.println(max_ch3);
   
    Serial.print("min : ");
    Serial.print(min_ch1);
    Serial.print("  ");
    Serial.print(min_ch2);
    Serial.print("  ");
    Serial.println(min_ch3); 
    
    change=false;
  }
}
