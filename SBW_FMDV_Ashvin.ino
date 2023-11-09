// Constants

float pi = 3.1415;

int RxD_Port=0;       // Pin that gets data from Predictive Stanley
//int TxD_Port=1;       // Pin that send data to the Ebike Motor
float max_Steer_Stanley = pi/4;        // Max Steer as per Predictive Stanley
float min_Steer_Stanley = -pi/4;       // Min Steer as per Predictive Stanley
float max_Steer_SW_Voltage = 3.3f;       // Max Steer for Steering Wheel
float min_Steer_SW_Voltage = 0;       // Min Steer for Steering Wheel
float steer = 0.00f;        // Initializing the Steer. This is the feed data recieved by the RxD_Port.
bool newDataRecieved = false;       // Checks if this is the incoming data is a fresh data or a previously used data.
float sensorValue = 0;
float Volt_Desired = 0;
int PM_Pin_1 = A0;      
float PGain = 0.2f;
float sample_time = 1/50;
/*
int PWM_Pin_2 = A1;       
int PWM_Pin_3 = A2;
int PWM_Pin_4 = A3;
int PWM_Pin_5 = A4;
*/

// Pins 9 and 10 have PWM functionalities
int enA = 9;
int enB = 10;

// Motor A input
int inA1 = 8;
int inA2 = 7;

// Motor B inputs
int inB1 = 4;
int inB2 = 2;


void showNewData() {
  /*
   Shows the Recieved Data
  */

 if (newDataRecieved == true) {
    Serial.print("Input Recieved : ");
    Serial.println(steer);
    delay(100);
    newDataRecieved = false; 
 }
}

void recieveSteer(){
  //steer = analogRead(RxD_Port);
  while (Serial.available() <= 0){}       //Wait for user input
  if (Serial.available() > 0){
    /* 
      Note : If input is directly read as Integer/Float , it is not read as intended(Garbage values are given even before user-input).
      Thus it is being recieved as string and then converted to a Float.
    */

    // User-Input
    String inp = Serial.readString(); 
    steer = inp.toFloat();

    // Debugging Statements for User-Input
    
    //mySerial.print("Float representation : ");
    //mySerial.println(accel);

    delay(200);
  }

  if (steer > max_Steer_Stanley){
    steer = max_Steer_Stanley;
  }
  else if(steer < min_Steer_Stanley){
    steer = min_Steer_Stanley;  
  }
  newDataRecieved = true;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  /*
  pinMode(PWM_Pin_2 , OUTPUT);
  */
  // Potentiometer pin
  pinMode(PM_Pin_1 , INPUT);
  // UART Pins
  pinMode(RxD_Port,INPUT);

  // Set All motor A pins to Output mode
  pinMode(enA , OUTPUT);
  pinMode(inA1 , OUTPUT);
  pinMode(inA2 , OUTPUT);

  // Set All motor B pins to Output mode
  pinMode(enB , OUTPUT);
  pinMode(inB1 , OUTPUT);
  pinMode(inB2 , OUTPUT);
  delay(100);
}

void readPotentiometerValue(){
  // read the input on analog pin 0:
  sensorValue = analogRead(PM_Pin_1);
  // print out the value you read:
  Serial.println("Potentiometer Value : ");
  Serial.println(sensorValue);
  delay(1000);  // delay in between reads for stability
}

void directionControl(){
  // Set Motor A to max speed
  analogWrite(enA , 255);

  if(steer < 0) {
    // Change the Motor A's Direction
    digitalWrite(inA1,LOW);
    digitalWrite(inA2,HIGH);
    delay(2000);
  }
  else{
    //Turn on Motor A in opposite direction
    digitalWrite(inA1,HIGH);
    digitalWrite(inA2,LOW);
    delay(2000);
  }
  // // Turn Off the Motor
  // digitalWrite(in1,LOW);
  // digitalWrite(in2,LOW);
}

// Controlling the speed of the Motor
void SpeedControl(){
  directionControl();

  // Accelerate from 0 to max_speed
  for(int i = 0;i<256;i++){
    analogWrite(enA,i);
    delay(20);
  }

  // Decelerate from 0 to max_speed
  for(int i = 255;i<=0;i--){
    analogWrite(enA,i);
    delay(20);
  }
  // // Turn Off the Motor
  // digitalWrite(in1,LOW);
  // digitalWrite(in2,LOW);
}

float PControl(float volt_curr , float volt_desired){
  float pControl = PGain * (volt_desired - volt_curr);
  Serial.print("Volt Diff : "); 
  Serial.println(volt_desired - volt_curr);
  return pControl;
}

float mod(float x){
  if (x > 0){
    return x;
  }
  else{
    return (-x);
  }
}

void loop() {
  // put your main code here, to run repeatedly:

  recieveSteer();
  showNewData();
  readPotentiometerValue();
  delay(500);

  float Volt_Desired = map(steer , min_Steer_Stanley , max_Steer_Stanley , min_Steer_SW_Voltage , max_Steer_SW_Voltage);
  Serial.print("Desired Voltage : ");
  Serial.println(Volt_Desired);
  delay(500);
  while(mod(Volt_per_Sec) != 0.1){
    float Volt_per_Sec = PControl(sensorValue , Volt_Desired); 
    delay(500);
    Serial.print("PID Out: ");
    float output_voltage = sensorValue + Volt_per_Sec * sample_time;
    if output_voltage > max_Steer_SW_Voltage {
      output = max_Steer_SW_Voltage;
    }
    if output_voltage < 0{
      output = 0;
    }
    int output = (int)(output_voltage/max_Steer_SW_Voltage * 255);
    analogWrite(enA , output);
    Serial.print("Volt Out : ");
    Serial.println(output);
    delay(500);
  }
}
