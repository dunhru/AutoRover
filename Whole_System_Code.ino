#define echoPin1 8 //obstacle ultrasonic echo 1
#define trigPin1 9 //obstacle ultrasonic trigger 1
#define echoPin2 10 //obstacle ultrasonic echo 2
#define trigPin2 11 //obstacle ultrasonic trigger 2
#define echoPin3 12 //obstacle ultrasonic echo 3
#define trigPin3 13 //obstacle ultrasonic trigger 3

#define echoa 4 //automatic ultrasonic echo
#define triga 5 //automatic ultrasonic trigger

#define IR1 2 //IR sensor 1
#define IR2 3 //IR sensor 2
#define LED1 4 //IR1 led
#define LED2 5 //IR2 led

#define joyX A0 //joystick X
#define joyY A1 //joystick Y
#define joySw 1 //joystick switch

#define motor1_1 31 //motor 1_1
#define motor1_2 33 //motor 1_2
#define motor1_en 9 //motor 1_en
#define motor2_1 37 //motor 2_1
#define motor2_2 39 //motor 2_2
#define motor2_en 10 //motor 2_en


//VARIABLES

//OBSTACLE DETECTION
int frontLimit = 12; 
int bottomLimit = 4;
int backLimit = 12;

//MANUAL FOLLOWING
int xPosJoy = 0;
int yPosJoy = 0;
int swStateJoy = 0;
int mapX = 0;
int mapY = 0;

//AUTOMATIC FOLLOWING
int statusSensorLeft;
int statusSensorRight;
float autoDistance;
float distances[10];

void setup()
{
  // Turn off motors - Initial state
  motors_stop();
  motors_speed(0);
   
  //OBSTACLE DETECTION
  pinMode(echoPin1, INPUT); //obstacle ultrasonic echo 1
  pinMode(trigPin1, OUTPUT); //obstacle ultrasonic trigger 1
  
  pinMode(echoPin2, INPUT); //obstacle ultrasonic echo 2
  pinMode(trigPin2, OUTPUT); //obstacle ultrasonic trigger 2
  
  pinMode(echoPin3, INPUT); //obstacle ultrasonic echo 3
  pinMode(trigPin3, OUTPUT); //obstacle ultrasonic trigger 3
  
  //AUTOMATIC FOLLOWING
  pinMode(echoa, INPUT); //automatic ultrasonic echo
  pinMode(triga, OUTPUT); //automatic ultrasonic trigger
  
  pinMode (IR1, INPUT); //IR sensor 1
  pinMode (IR2, INPUT); //IR sensor 2
  pinMode (LED1, OUTPUT); 
  pinMode (LED2, OUTPUT);
  
  //MANUAL FOLLOWING
  pinMode (joyX, INPUT); //joystick X
  pinMode (joyY, INPUT); //joystick Y
  pinMode (joySw, INPUT_PULLUP); //joystick switch
  
  //MOTOR CONTROL   
  pinMode(motor1_1, OUTPUT); //motor 1_1  
  pinMode(motor1_2, OUTPUT); //motor 1_2
  pinMode(motor1_en, OUTPUT); //motor1_en
  pinMode(motor2_1, OUTPUT); //motor 2_1
  pinMode(motor2_2, OUTPUT); //motor 2_2 
  pinMode(motor2_en, OUTPUT); //motor2_en 

  motors_speed(0); //start motors at 0 speed
  
  digitalWrite(motor1_1, LOW);
  digitalWrite(motor1_2, LOW);
  digitalWrite(motor2_1, LOW);
  digitalWrite(motor2_2, LOW);

  Serial.begin(9600); //start serial communication at 9600 baudrate speed
}

void loop()
{
  //HIERARCHY
    //1) obstacle detection
    //2) manual control
    //3) automatic control


    //???? NEEDS TO BE IMPLEMENTED ????


  
  //OBSTACLE DETECTION
  if (obstacle_front()) { //obstacle_front is true of there is an obstacle
    Serial.print("\n");
    Serial.print("WARNING: FRONT OBSTACLE DETECTED");

    avoid_obstacle();
  }
  if (obstacle_back()) { //obstacle_front is true of there is an obstacle
    Serial.print("\n");
    Serial.print("WARNING: BACK OBSTACLE DETECTED");

    avoid_obstacle();
  }
  else {
    Serial.print("\n");
    Serial.print("no obstacles detected");
  }
  
  //AUTOMATIC FOLLOWING
  
  //MANUAL FOLLOWING
  xPosJoy = analogRead(joyX);
  yPosJoy = analogRead(joyY);
  swStateJoy = digitalRead(joySw);
  mapX = map(xPosJoy, 0, 1023, -512, 512);
  mapY = map(yPosJoy, 0, 1023, -512, 512);

  //mapY > 150 => joystick right
  //mapy < -150 => joystick left
  //mapX > 150 => joystick up
  //mapX < -150 => joystick down

  //check that joystick is being moved
  if(mapX > 150 || mapX < -150 || mapY > 150 || mapY < -150){
    
    int x_offset = map(mapX, 0, 1023, -70, 70); 
    int y_speed = map(mapY, 0, 1023, -255, 255); 
    
    //set motor speeds
    int left_speed = y_speed + x_offset;
    int right_speed = y_speed - x_offset;

    motors_speed_individual(left_speed, right_speed);
    
    //MAP mapX TO A VAR TO BE AN OFFSET
    //MAP mapY TO SPEED FORWARD OR BACKWORDS
  }
}




//----------FUNCTIONS----------



//OBSTACLE DETECTION

bool obstacle_back() { //if no obstacle, return false
  long duration3;
  int distance3;

  //clear trigPins to low
  digitalWrite(trigPin3, LOW);
  
  //clear trigPins to low
  digitalWrite(trigPin3, LOW);
  delayMicroseconds(2); //delay, required as per arduino example code
  
  //set trigPins high for 10 microseconds
  digitalWrite(trigPin3, HIGH);
  delayMicroseconds(10);
  
  //clear trigPins again
  digitalWrite(trigPin3, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration3 = pulseIn(echoPin3, HIGH);

  // Calculating the distance
  distance3 = duration3 / 72 / 2; ////sound travels at 72 microseconds per inch, and then divide by 2 because the sound traveled there and back
  
  //serial prints for trouble shooting
  //Serial.print("\n");
  //Serial.print(distance3);
  //Serial.print(" inches");
  
  if (distance3 > backLimit) { //limit set in inches
    return false; //no obstacle if sensor reads further than limit
  }
  else {
    return true;
  }
}

bool obstacle_front() { //if no obstacle, return false
  //create variables
  long duration1;
  long duration2;
  int distance1;
  int distance2;
  
  //clear trigPins to low
  digitalWrite(trigPin1, LOW);
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2); //delay, required as per arduino example code
  
  //set trigPins high for 10 microseconds
  digitalWrite(trigPin1, HIGH);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  
  //clear trigPins again
  digitalWrite(trigPin1, LOW);
  digitalWrite(trigPin2, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration1 = pulseIn(echoPin1, HIGH);
  duration2 = pulseIn(echoPin2, HIGH);

  // Calculating the distance
  distance1 = duration1 / 72/ 2; ////sound travels at 72 microseconds per inch, and then divide by 2 because the sound traveled there and back
  distance2 = duration2 / 72 / 2; 

  //serial prints for testing
  //Serial.print("\n");
  //Serial.print(distance2);
  //Serial.print(" inches");
  
  if ((distance1 > frontLimit) and (distance2 < bottomLimit)) { //obstacle distance limits in inches
    return false; //no obstacles detected ahead or below, the function returns false
  }
  else {
    return true;
  }
}

void avoid_obstacle(){ //turn in place to avoid obstacle
    motors_stop(); //stop movement
    //turn until safe
    int count = 0;
    while(obstacle_back()){
      if(count<30){
        stationary_turn_right(30); //turn right with movement speed 30
      }
      else{
        stationary_turn_left(50); //turn left with movement speed 50
      }
      count++; //if cannot avoid obstacle for 30 cycles in one direction, turn in other direction
    }
}


//MOTOR CONTROL

void motor_startup() { //quick pulse to turn on motors
  analogWrite(motor1_en, 200);
  analogWrite(motor2_en, 200);

  Serial.println("Motor startup");
  
  digitalWrite(motor1_1, LOW);
  digitalWrite(motor1_2, HIGH);
  digitalWrite(motor2_1, LOW);
  digitalWrite(motor2_2, HIGH);
  
  delay(20);
  
  digitalWrite(motor1_1, LOW);
  digitalWrite(motor1_2, LOW);
  digitalWrite(motor2_1, LOW);
  digitalWrite(motor2_2, LOW);
}

void motors_stop() { //stop the motors
  digitalWrite(motor1_1, LOW);
  digitalWrite(motor1_2, LOW);
  digitalWrite(motor2_1, LOW);
  digitalWrite(motor2_2, LOW);
}

void motors_set_backwards() { //switch motor direction to reverse
  digitalWrite(motor1_1, HIGH);
  digitalWrite(motor1_2, LOW);
  digitalWrite(motor2_1, HIGH);
  digitalWrite(motor2_2, LOW);
}
  
void motors_set_forwards() { //switch motor direction to forward
  digitalWrite(motor1_1, LOW);
  digitalWrite(motor1_2, HIGH);
  digitalWrite(motor2_1, LOW);
  digitalWrite(motor2_2, HIGH);
}

void motors_speed(int motors_speed) { //input movement speed
  //prevent value from exceed 255
  if(motors_speed>255){
    motors_speed = 255;
  }
  else if(motors_speed<0){
    motors_speed = 0;
  }
  analogWrite(motor1_en, motors_speed);
  analogWrite(motor2_en, motors_speed);
}

void motors_speed_individual(int motor1_speed, int motor2_speed) { //input movement speed
  //prevent value from exceed 255
  if(motor1_speed>255){
    motor1_speed = 255;
  }
  else if(motor1_speed<0){
    motor1_speed = 0;
  }
  if(motor2_speed>255){
    motor2_speed = 255;
  }
  else if(motor2_speed<0){
    motor2_speed = 0;
  }
  analogWrite(motor1_en, motor1_speed);
  analogWrite(motor2_en, motor2_speed);
}

void moving_turn_right(int motors_speed) { //turn right while moving forward
  //prevent value from exceed 255
  if(motors_speed>55){
    motors_speed = 55;
  }
  else if(motors_speed<0){
    motors_speed = 0;
  }
  
  int motor1_speed = 200 - motors_speed;
  int motor2_speed = 200 + motors_speed;
  analogWrite(motor1_en, motor1_speed);
  analogWrite(motor2_en, motor2_speed);
}

void moving_turn_left(int motors_speed) { //turn left while moving forward
  //prevent value from exceed 255
  if(motors_speed>55){
    motors_speed = 55;
  }
  else if(motors_speed<0){
    motors_speed = 0;
  }
  
  int motor1_speed = 200 + motors_speed;
  int motor2_speed = 200 - motors_speed;
  analogWrite(motor1_en, motor1_speed);
  analogWrite(motor2_en, motor2_speed);
}


void stationary_turn_right(int motors_speed) { //turn right in place
  //prevent value from exceed 255
  if(motors_speed>255){
    motors_speed = 255;
  }
  else if(motors_speed<0){
    motors_speed = 0;
  }
  
  int motor1_speed = 0 - motors_speed;
  int motor2_speed = motors_speed;
  analogWrite(motor1_en, motor1_speed);
  analogWrite(motor2_en, motor2_speed);
}

void stationary_turn_left(int motors_speed) { //turn left in place
  //prevent value from exceed 255
  if(motors_speed>255){
    motors_speed = 255;
  }
  else if(motors_speed<0){
    motors_speed = 0;
  }
  
  int motor1_speed = motors_speed;
  int motor2_speed = 0 - motors_speed;
  analogWrite(motor1_en, motor1_speed);
  analogWrite(motor2_en, motor2_speed);
}


//AUTOMATIC

void read_IR() {
  statusSensorRight = digitalRead (IR1);
  statusSensorLeft = digitalRead (IR2);
}

void read_auto_ultra() {

  long duration;

  digitalWrite(triga, LOW);
  delayMicroseconds(2); //delay, required as per arduino example code

  //set trigPins high for 10 microseconds
  digitalWrite(triga, HIGH);
  delayMicroseconds(10);

  digitalWrite(triga, LOW);

  duration = pulseIn(echoa, HIGH);
  autoDistance = duration / 72 / 2;
}

void auto_drive() {
  float mean;
  float devs[10];
  float sdev;
  int pwm_in;
  float kp = 0; // proportional control constant
  read_IR(); //read directional IR values
  for (int i = 0; i < 10; i++) {
    read_auto_ultra();
    distances[i] = autoDistance; //read 10 utrasonic values and input into size 10 array
    delay(10);
  }

  mean = sumArray(distances) / 10; //find mean of distances
  for(int i=0; i<10; i++){
    devs[i] = distances[i] - mean; // calculate deviations from the mean
  }
  sdev = sumArray(devs) / 10; //calculate standard deviation from this mean

  pwm_in = int(sdev * kp) + 50;

  if (statusSensorRight == 1 && statusSensorLeft == 0) {
    //case: something is detected on the left
    analogWrite(motor1_en, pwm_in);
    analogWrite(motor2_en, pwm_in / 2);
  }
  else if (statusSensorRight == 0 && statusSensorLeft == 1) {
    //case: something is detected on the right
    analogWrite(motor1_en, pwm_in / 2);
    analogWrite(motor2_en, pwm_in);
  }
  else if (statusSensorRight == 1 && statusSensorLeft == 1) {
    //case: something is detected on both sides
    analogWrite(motor1_en, pwm_in);
    analogWrite(motor2_en, pwm_in);
  }
  else {
    motors_stop();
  }
}

float sumArray(float d[10]){
  float sum = 0;
  for(int i; i < 10; i++){
    sum = sum + d[i];
  }
  return sum;
}
