#define joyX A0 //joystick X
#define joyY A1 //joystick Y
#define joySw 4 //joystick switch

//MANUAL FOLLOWING
int xPosJoy = 0;
int yPosJoy = 0;
int swStateJoy = 0;
int mapX = 0;
int mapY = 0;
void setup() {
  // put your setup code here, to run once:
  //MANUAL FOLLOWING
  pinMode (joyX, INPUT); //joystick X
  pinMode (joyY, INPUT); //joystick Y
  pinMode (joySw, INPUT_PULLUP); //joystick switch
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
 //MANUAL FOLLOWING
  xPosJoy = analogRead(joyX);
  yPosJoy = analogRead(joyY);
  swStateJoy = digitalRead(joySw);
  mapX = map(xPosJoy, 0, 1023, -512, 512);
  mapY = map(yPosJoy, 0, 1023, -512, 512);

  words();
  //prints();
  }


 int prints(){
    delay(500);
  Serial.println("");
  
  Serial.print("xPosJoy: ");
  Serial.println(xPosJoy);

  Serial.print("yPosJoy: ");
  Serial.println(yPosJoy);
  
  Serial.print("mapX: ");
  Serial.println(mapX);

  Serial.print("mapY: ");
  Serial.println(mapY);

  Serial.print("swStateJoy: ");
  Serial.println(swStateJoy);
 }

 int words(){
    delay(500);
    //Serial.print("mapX: ");
    //Serial.println(mapX);

    //Serial.print("mapY: ");
    //Serial.println(mapY);
    //check that joystick is being moved
    if(mapX > 150 || mapX < -150 || mapY > 150 || mapY < -150){



    if(mapY > 150) {
      Serial.println("joystick right");
    }

     if(mapY < -150) {
      Serial.println("joystick left");
    }

    if(mapX > 150) {
      Serial.println("joystick up");
    }

    if(mapX < -150) {
      Serial.println("joystick down");
    }


  }
  else {
   Serial.println("Joystick not moving");
  }
}
