#define echoPin1 2 //echo for sensor 1
#define trigPin1 3 //trig for sensor 1
#define echoPin2 4 //echo for sensor 2
#define trigPin2 5 //trig for sensor 2
#define echoPin3 6 //echo for sensor 3
#define trigPin3 7 //trig for sensor 3

void setup() {
  //set all echo pins as inputs:
  pinMode (echoPin1, INPUT); 
  pinMode (echoPin2, INPUT);
  pinMode (echoPin3, INPUT);

  //set all trig pins as outputs:
  pinMode (trigPin1, OUTPUT);
  pinMode (trigPin2, OUTPUT);
  pinMode (trigPin3, OUTPUT);

  Serial.begin(9600); //start serial communication at 9600 baudrate speed

}

void loop() {


  if (obstacle_front()) { //obstacle_front is true of there is an obstacle
    Serial.print("\n");
    Serial.print("WARNING: FRONT OBSTACLE DETECTED");
  }

  if (obstacle_back()) { //obstacle_front is true of there is an obstacle
    Serial.print("\n");
    Serial.print("WARNING: BACK OBSTACLE DETECTED");
  }

  else {
    Serial.print("\n");
    Serial.print("no obstacles detected");
  }
}

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
  
  if (distance3 > 12) { //12 inches
    return false; //no obstacle if sensor reads further than 12 inches
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
  
  if ((distance1 > 12) and (distance2 < 4)) { //obstacle distance limits in inches
    return false; //no obstacles detected ahead or below, the function returns false
  }
  else {
    return true;
  }
}
  
