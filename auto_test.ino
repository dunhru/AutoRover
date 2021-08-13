#define echoPin 8 //obstacle ultrasonic echo 1
#define trigPin 9 //obstacle ultrasonic trigger 1

#define IRL 2 //
#define IRR 53 //

#define motor1_1 31 //motor 1_1
#define motor1_2 33 //motor 1_2
#define motor1_en 6 //motor 1_en
#define motor2_1 37 //motor 2_1
#define motor2_2 39 //motor 2_2
#define motor2_en 7 //motor 2_en

int statusSensorLeft;
int statusSensorRight;
float autoDistance;

void setup() {
  pinMode(echoPin, INPUT); //automatic ultrasonic echo
  pinMode(trigPin, OUTPUT); //automatic ultrasonic trigger

  pinMode (IRR, INPUT); //IR sensor 1
  pinMode (IRL, INPUT); //IR sensor 2

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
  
  Serial.begin(9600);
}



void loop() {
  auto_drive();
}

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

void motors_speed_individual(int motor1_speed, int motor2_speed) { //input movement speed

  check_direction(motor1_speed, motor2_speed);
  motor1_speed = abs(motor1_speed);
  motor2_speed = abs(motor2_speed);


  //prevent value from exceed 255
  if (motor1_speed > 255) {
    motor1_speed = 255;
  }
  else if (motor1_speed < 0) {
    motor1_speed = 0;
  }
  if (motor2_speed > 255) {
    motor2_speed = 255;
  }
  else if (motor2_speed < 0) {
    motor2_speed = 0;
  }

  Serial.print("Motor speeds: ");
  Serial.print(motor1_speed);
  Serial.print(' ');
  Serial.println(motor2_speed);

  motor_startup();
  motors_set_forwards();

  analogWrite(motor1_en, motor1_speed);
  analogWrite(motor2_en, motor2_speed);
}

void motors_set_forwards() { //switch motor direction to forward
  Serial.println("Motors set forwards");

  digitalWrite(motor1_1, HIGH);
  digitalWrite(motor1_2, LOW);
  digitalWrite(motor2_1, HIGH);
  digitalWrite(motor2_2, LOW);
}

void check_direction(int motor1, int motor2) {
  if (motor1 < 0) {
    digitalWrite(motor1_1, HIGH);
    digitalWrite(motor1_2, LOW);
  }
  else if (motor1 >= 0) {
    digitalWrite(motor1_1, LOW);
    digitalWrite(motor1_2, HIGH);
  }

  if (motor2 < 0) {
    digitalWrite(motor2_1, HIGH);
    digitalWrite(motor2_2, LOW);
  }
  else if (motor2 >= 0) {
    digitalWrite(motor2_1, LOW);
    digitalWrite(motor2_2, HIGH);
  }
  motors_set_forwards();
}

float calculateSD(float data[])
{
    float sum = 0.0, mean, standardDeviation = 0.0;

    int i;

    for(i = 0; i < 10; ++i)
    {
        sum += data[i];
    }

    mean = sum/10;

    for(i = 0; i < 10; ++i)
        standardDeviation += pow(data[i] - mean, 2);

    return sqrt(standardDeviation / 10);
}

void read_IR() {
  statusSensorRight = digitalRead (IRR);
  statusSensorLeft = digitalRead (IRL);
}

void read_auto_ultra() {

  long duration;

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2); //delay, required as per arduino example code

  //set trigPins high for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  autoDistance = duration / 72 / 2;
}

void auto_drive() {
  float mean;
  float devs[10];
  float sdev;
  float distances[10];
  int pwm_in;
  float kp = 20; // proportional control constant
  read_IR(); //read directional IR values
  for (int i = 0; i < 10; i++) {
    read_auto_ultra();
    distances[i] = autoDistance; //read 10 utrasonic values and input into size 10 array
    delay(50);
  }

  sdev = calculateSD(distances);

  pwm_in = int(sdev * kp) + 50;
  motors_speed_individual(pwm_in,pwm_in);
 /* if  (statusSensorRight == 1 && statusSensorLeft == 0) {
    //case: something is detected on the left
    Serial.println("left ");
    Serial.print("pwm: ");
    Serial.println(pwm_in);
    motors_speed_individual(pwm_in,pwm_in/2);
  }
  else if (statusSensorRight == 0 && statusSensorLeft == 1) {
    //case: something is detected on the right
    Serial.println("right ");
    Serial.print("pwn: ");
    Serial.println(pwm_in);
    motors_speed_individual(pwm_in/2,pwm_in);
  }
  else if (statusSensorRight == 1 && statusSensorLeft == 1) {
    //case: something is detected on both sides
    Serial.println("centre ");
    Serial.print("pwn: ");
    Serial.println(pwm_in);
    motors_speed_individual(pwm_in,pwm_in);
  }
  else if (statusSensorRight == 0 && statusSensorLeft == 0){
    Serial.println("none");
    Serial.print("pwm: ");
    Serial.println(pwm_in);
  }
  Serial.print("sense state: ");
  Serial.print(statusSensorRight);
  Serial.println(statusSensorLeft);
  */
}
