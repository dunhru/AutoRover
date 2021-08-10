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
#define joySw 52 //joystick switch - controls attachInterrupt                                                                             // ????? NEWLY UPDATED ?????

#define motor1_1 31 //motor 1_1
#define motor1_2 33 //motor 1_2
#define motor1_en 6 //motor 1_en
#define motor2_1 37 //motor 2_1
#define motor2_2 39 //motor 2_2
#define motor2_en 7 //motor 2_en


//VARIABLES

//OBSTACLE DETECTION
int frontLimit = 12; //distance limit in inches
int bottomLimit = 4; //distance limit in inches
int backLimit = 12;  //distance limit in inches

//MANUAL FOLLOWING
int xPosJoy = 0;
int yPosJoy = 0;
int swStateJoy = 0;
int mapX = 0;
int mapY = 0;

bool manual_stop = false; //use joystick switch to toggle emergency stop                                                                   // ????? NEWLY ADDED ?????

//AUTOMATIC FOLLOWING
int statusSensorLeft;
int statusSensorRight;
float autoDistance;
float distances[10];

void setup()
{
  // Turn off motors - Initial state
  motors_stop();
  motors_speed_func(0); //start motors at 0 speed

  //MANUAL SHUTOFF
  pinMode(joySw, INPUT_PULLUP);                                                                                                                // ????? NEWLY ADDED ?????
  attachInterrupt(joySw, manual_shutoff, CHANGE);
   
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
  
  digitalWrite(motor1_1, LOW);
  digitalWrite(motor1_2, LOW);
  digitalWrite(motor2_1, LOW);
  digitalWrite(motor2_2, LOW);

  Serial.begin(9600); //start serial communication at 9600 baudrate speed
}

void loop() //controls which type of system is being run
{

  Serial.println("Manual stop state: " + manual_stop);
  
  //IF IN MANUAL STOP MODE, DO NOT RUN CODE                                                                                                 // ????? NEWLY ADDED ?????
  if(manual_stop == false){


    //HIERARCHY
      //1) obstacle detection
      //2) manual control
      //3) automatic control

    xPosJoy = analogRead(joyX);
    yPosJoy = analogRead(joyY);
    mapX = map(xPosJoy, 0, 1023, -512, 512);
    mapY = map(yPosJoy, 0, 1023, -512, 512);
    Serial.println("\n\nX_pos: " + xPosJoy);
    Serial.println("\n\nY_pos: " + yPosJoy);
    Serial.println("\n\nX: " + mapX);
    Serial.println("\n\nY: " + mapY);

  
    if(mapX > 150 || mapX < -150 || mapY > 150 || mapY < -150){
      //----MANUAL CONTROL OVERRIDE----
      
      if(obstacle_front() || obstacle_back()){  //need to avoid any obstacles before manual following can be implemented
        //----OBSTACLE DETECTION OVERRIDE----
        Serial.println("\nObstacle detection - 1");
        obstacle_detection_full(); //complete obstacle detection  
      }
      
      manual_following_full(); //run joystick control
        Serial.println("\nManual control");
    }
    else if(obstacle_front() || obstacle_back()){
      //----OBSTACLE DETECTION OVERRIDE----
      Serial.println("\nObstacle detection - 1");
      obstacle_detection_full(); //complete obstacle detection
    }
    else{
      //----AUTOMATIC CONTROL BASELINE----
      Serial.println("\nAutomatic control");
      
      auto_drive(); //run automatic detection otherwise
    } 

    
  }
  else{   //STOP MOTORS IF MANUAL STOP ENACTED
    motors_stop();
    motors_speed_func(0); //stop motors from running
  }
}




//----------IMPLEMENTATIONS----------



//OBSTACLE DETECTION

void obstacle_detection_full(){
  if (obstacle_front()) { //obstacle_front is true of there is an obstacle
    Serial.print("\n");
    Serial.println("WARNING: FRONT OBSTACLE DETECTED");

    avoid_obstacle();
  }
  if (obstacle_back()) { //obstacle_front is true of there is an obstacle
    Serial.print("\n");
    Serial.println("WARNING: BACK OBSTACLE DETECTED");

    avoid_obstacle();
  }
  else {
    Serial.print("\n");
    Serial.println("no obstacles detected");
  }
}

//MANUAL FOLLOWING

void manual_shutoff(){                                                                                                                    // ????? NEWLY ADDED ?????
  Serial.println("Manual shutoff - state change");
  swStateJoy = digitalRead(joySw); //0 = unclicked, 1 = clicked                                                     // ????? removed from full manual function ?????

  //update manual stop state
  if(manual_stop == false){
    manual_stop = true;
  }
  else{
    manual_stop = false;
  }
}

void manual_following_full(){
  Serial.println("Manual following function");
  
  xPosJoy = analogRead(joyX);
  yPosJoy = analogRead(joyY);
  mapX = map(xPosJoy, 0, 1023, -512, 512);
  mapY = map(yPosJoy, 0, 1023, -512, 512);

  //mapY > 150 => joystick right
  //mapy < -150 => joystick left
  //mapX > 150 => joystick up
  //mapX < -150 => joystick down

  //check that joystick is being moved
  if(mapX > 150 || mapX < -150 || mapY > 150 || mapY < -150){
    
    int x_speed = map(mapX, 0, 1023, -100, 100); //moves back and forth slowly
    int y_offset = map(mapY, 0, 1023, -155, 155); //turn speed mapping
    
    //set motor speeds
    int left_speed = x_speed + y_offset;
    int right_speed = x_speed - y_offset;

    motors_speed_individual(left_speed, right_speed);
  }
}

//AUTOMATIC FOLLOWING

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
    Serial.println("Avoiding obstacle");
  
    motors_stop(); //stop movement
    //turn until safe
    int count = 0;

    motor_startup();
    
    while(obstacle_back() || obstacle_front()){
      if(count<30){
        stationary_turn_right(30); //turn right with movement speed 30
        Serial.println("RIGHT TEST");
      }
      else{
        stationary_turn_left(50); //turn left with movement speed 50
        Serial.println("LEFT TEST");
      }
      count++; //if cannot avoid obstacle for 30 cycles in one direction, turn in other direction
      delay(10);
      Serial.println("Count: " + count);
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
  
  analogWrite(motor1_en, 0);
  analogWrite(motor2_en, 0);
}

void motors_stop() { //stop the motors
  Serial.println("Motors stop");
  
  digitalWrite(motor1_1, LOW);
  digitalWrite(motor1_2, LOW);
  digitalWrite(motor2_1, LOW);
  digitalWrite(motor2_2, LOW);
}

void motors_set_backwards() { //switch motor direction to reverse
  Serial.println("Motors set backwards");
  
  digitalWrite(motor1_1, HIGH);
  digitalWrite(motor1_2, LOW);
  digitalWrite(motor2_1, HIGH);
  digitalWrite(motor2_2, LOW);
}
  
void motors_set_forwards() { //switch motor direction to forward
  Serial.println("Motors set forwards");
  
  digitalWrite(motor1_1, LOW);
  digitalWrite(motor1_2, HIGH);
  digitalWrite(motor2_1, LOW);
  digitalWrite(motor2_2, HIGH);
}

void motors_speed_func(int motors_speed) { //input movement speed
  Serial.println("Motors speed: " + motors_speed);
  
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
  Serial.println("Motor speeds: " + motor1_speed + ' ' + motor2_speed);
  
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
  Serial.println("Moving, turn right - " + motors_speed);
  
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
  Serial.println("Moving, turn left - " + motors_speed);
  
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
  Serial.println("Stationary, turn right - " + motors_speed);
  
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
  Serial.println("Stationary, turn left - " + motors_speed);
  
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

float sumArray(float d[10]){
  float sum = 0;
  for(int i; i < 10; i++){
    sum = sum + d[i];
  }
  return sum;
}
