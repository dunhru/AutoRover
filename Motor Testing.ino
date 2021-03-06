//add switch statements
//use flag for start stop emergency button
//use interrupt on button pin for emergency button

#define motor1_1 31 //motor 1_1
#define motor1_2 33 //motor 1_2
#define motor1_en 9 //motor 1_en
#define motor2_1 37 //motor 2_1
#define motor2_2 39 //motor 2_2
#define motor2_en 10 //motor 2_en

void setup() {
  // put your setup code here, to run once:
  pinMode(motor1_1, OUTPUT); //motor 1_1  
  pinMode(motor1_2, OUTPUT); //motor 1_2
  pinMode(motor1_en, OUTPUT); //motor1_en
  pinMode(motor2_1, OUTPUT); //motor 2_1
  pinMode(motor2_2, OUTPUT); //motor 2_2 
  pinMode(motor2_en, OUTPUT); //motor2_en 

  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  // Set motors to maximum speed
  // For PWM maximum possible values are 0 to 255
  motor_startup();
  
  analogWrite(motor1_en, 255);
  analogWrite(motor2_en, 255);
  
  Serial.println("Full PWM, spin one direction");

  // Turn on motor A & B
  digitalWrite(motor1_1, HIGH);
  digitalWrite(motor1_2, LOW);
  digitalWrite(motor2_1, HIGH);
  digitalWrite(motor2_2, LOW);
  delay(3500);

  Serial.println("Full PWM, spin other direction");
  
  // Now change motor directions
  digitalWrite(motor1_1, LOW);
  digitalWrite(motor1_2, HIGH);
  digitalWrite(motor2_1, LOW);
  digitalWrite(motor2_2, HIGH);
  delay(3500);

  
  // Decelerate from maximum speed to zero

  Serial.println("Decelerate");
  
  for (int i = 255; i >= 50; --i) {
    analogWrite(motor1_en, i);
    analogWrite(motor2_en, i);

    if(i%10==0){
      Serial.println(i);
    }
    
    delay(20);
  }

  delay(1000);

  // Accelerate from zero to maximum speed

  Serial.println("Accelerate");
  
  for (int i = 50; i < 255; i++) {
    analogWrite(motor1_en, i);
    analogWrite(motor2_en, i);
    
    if(i%10==0){
      Serial.println(i);
    }
    
    delay(20);
  }

  Serial.println("Motors off");

  motors_stop();



  
  //Functions Test
  analogWrite(motor1_en, 255);
  analogWrite(motor2_en, 255);
  int var_time = 3500;

  Serial.println("\n\n FUNCTION TEST \n\n");

  Serial.println("Startup");
  motor_startup();

  Serial.println("Backwards");
  motors_set_backwards();
  delay(var_time);
  Serial.println("Forwards");
  motors_set_forwards();
  delay(var_time);

  Serial.println("Speed - 255");
  both_motor_speed(255);
  delay(var_time);
  Serial.println("Speed - 150");
  both_motor_speed(150);
  delay(var_time);
  Serial.println("Speed - 90");
  both_motor_speed(50);
  delay(var_time);
  
  Serial.println("Stationary - right");
  stationary_turn_right(150);
  delay(var_time);
  Serial.println("Stationary - left");
  stationary_turn_left(150);
  delay(var_time);
  Serial.println("Moving - right");
  moving_turn_right(50);
  delay(var_time);
  Serial.println("Moving - left");
  moving_turn_left(50);
  delay(var_time);

  Serial.println("Stop");
  motors_stop();
}



//FUNCTIONS

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

void both_motor_speed(int motors_speed) { //input movement speed
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
