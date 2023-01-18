
#define DIR 7
#define EN  2
#define SA  3
#define SB  9
#define POT A0
#define BTN2  22
#define BTN3  23

#define DELAY 20  
#define INCREMENT 5   

#define HISTORY_SIZE 32   
                          
#define TIMEOUT 1000 

#include <Adafruit_MPL3115A2.h>
#include <avr/interrupt.h>

Adafruit_MPL3115A2 baro;


bool changeDir();
bool setDir(bool newDir);
bool setSpd(int newSpeed);

bool currentDir = 0;
int valuePWM = 0;
bool motorDir = 0;

bool encoder2Value = 0;


//Target values
int targetPosition=0; //159=53*3 one turn
int temperature=0;
bool firstMeasure = true;

//Measured values
volatile float motorPosition = 0;
float previousMotorPosition = -1;                       


// PID parameters
float Kp=5;
float Ki=1.5;
float Kd=3;
float u= 0 ;


//PID related
float previousTime=0;  //for calculating delta t
float previousError=0; // for calculating the derivative
float errorIntegral=0; // integral error
float currentTime=0; //time in the moment of calculation
float deltaTime=0; //time difference
float errorValue=0; //error
float edot=0; //error derivative (de/dt)

int counter=0;

void setup() {
  pinMode(DIR, OUTPUT);
  pinMode(EN, OUTPUT);
  pinMode(SA, INPUT);
  pinMode(SB, INPUT);
  pinMode(POT, INPUT);
  pinMode(BTN2, INPUT);
  pinMode(BTN3, INPUT);

  //Initialize Sensor
  Serial.begin(500000); 
  while(!Serial);
  Serial.println("Adafruit_MPL3115A2 test!");  

  if (!baro.begin()) {
    Serial.println("Could not find sensor. Check wiring.");
    while(1);
  }

  baro.setSeaPressure(986);

  // Initialize our direction and speed to 0
  analogWrite(EN, valuePWM);
  digitalWrite(DIR, currentDir);
  delay(1);   // Wait for things to settle
  TCCR1A = 0;
  TCCR1B = 1<<WGM12 | 1<<CS12 | 0<<CS11 | 1<<CS10;
  TCNT1 = 0;          // reset Timer 1 counter
  // OCR1A = ((F_clock / prescaler) / Fs) - 1 = 2499
  OCR1A = 7812*2;       // Set sampling frequency Fs = 100 Hz
  TIMSK1 = 1<<OCIE1A; // Enable Timer 1 interrupt
  attachInterrupt(digitalPinToInterrupt(SA),checkEncoder,RISING);

  setSpd(120); 
  setDir(1);
}



void loop() {
  // put your main code here, to run repeaedly:
  
  defineTarget();

  calculatePID();

  driveMotor();

  // if(counter==(53*6)*3){     //159 pulses
  //   setSpd(0);
  // }

  //printValues();

}

ISR(TIMER1_COMPA_vect) {
  getTemp();
}

void getTemp(){
  //Serial.println("ISR");
  baro.startOneShot();
  if (!firstMeasure) {
    temperature=baro.getLastConversionResults(MPL3115A2_TEMPERATURE);
    firstMeasure = false;
  }
  //temperature = baro.getTemperature();
  Serial.print("temperature = "); Serial.print(temperature); Serial.println(" C");  
  // if (lastMeasureTime==0) {
  //   baro.startOneShot();
  //   lastMeasureTime = millis();
  //   return;
  // }
  // if (millis()-lastMeasureTime > 500) {
  //   lastMeasureTime = millis();
  //   baro.startOneShot();
  //   temperature=baro.getLastConversionResults(MPL3115A2_TEMPERATURE);
  //   Serial.print("temperature = "); Serial.print(temperature); Serial.println(" C");
  // }
}

void defineTarget(){
    targetPosition=analogRead(A0)/6.289;
};

void driveMotor(){
  //Direction
  if(u<0){
    motorDir=0;
  }
  else if(u>0){
    motorDir=1;    
  }

  //Speed
  valuePWM = (int)fabs(u);


  if(valuePWM!=0){
    //Set Direction
    if(currentDir!=motorDir){
      setDir(motorDir);
    }    

    //Set Speed
    setSpd(valuePWM);
  }
  else{
    //Set Direction
    if(currentDir!=motorDir){
      setDir(motorDir);
    }    
    //Set Speed
    setSpd(0);    
  }   
}


void calculatePID(){
  currentTime= micros();
  deltaTime = (currentTime-previousTime) / 1000000.0;
  previousTime = currentTime;

  errorValue = motorPosition - targetPosition;

  edot= (errorValue - previousError)/deltaTime;

  errorIntegral = errorIntegral + errorValue*deltaTime;

  u = (Kp*errorValue) + (Kd*edot) + (Ki*errorIntegral);

  previousError = errorValue;
}

void checkEncoder(){
    encoder2Value=digitalRead(SB);

    if(encoder2Value==1){
      motorPosition--;
    }
    else {
      motorPosition++;      
    }  
    counter++;
}

void printValues(){
  Serial.print(targetPosition);
  Serial.print(" ");  
  Serial.print(errorValue);
  Serial.print(" ");
  Serial.print(u);
  Serial.print(" ");  
  Serial.print(valuePWM);
  Serial.print(" ");
  Serial.print(motorDir);
  Serial.print(" ");
  Serial.println(motorPosition);
}

bool setSpd(int newSpeed){
  // Ensure that new speed is within bounds
  if (newSpeed > 255) {
    newSpeed = 255;
  }
  else if(newSpeed<=255 && newSpeed>=40){
    newSpeed=newSpeed;
  }
  else if (newSpeed < 45  && errorValue != 0){
     newSpeed = 45;
     //Serial.println("Entre");
  }
  else {
    newSpeed=0;
  } 

  valuePWM = newSpeed;
  analogWrite(EN, valuePWM);
}

bool changeDir(){
  // Stop our motor so we can change direction
  analogWrite(EN, 0);
  delay(10);  // Wait a bit for things to settle

  // Change our direction
  currentDir = !currentDir;
  digitalWrite(DIR, currentDir);
  delay(10);  // Wait a bit for things to settle

  // Start our motor back up again
  analogWrite(EN, valuePWM);
}

bool setDir(bool newDir){
  // Stop our motor so we can change direction
  analogWrite(EN, 0);
  delay(10);  // Wait a bit for things to settle

  // Change our direction
  currentDir = newDir;
  digitalWrite(DIR, currentDir);
  delay(10);  // Wait a bit for things to settle

  // Start our motor back up again
  analogWrite(EN, valuePWM);
}
