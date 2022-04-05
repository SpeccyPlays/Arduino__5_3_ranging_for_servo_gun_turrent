/*
  2.2
ranging for motion sensor with alarm led and servo sweep if object is too close
  
created on 21-09-21
by benjamin gibbs

22-09-21 updated by Benjamin Gibbs
updating code to remove delay() and replace with timer
code doesnt work correctly. servo does a half sweep then i think it tries to run too often

23-09-21 updated by Benjamin Gibbs
updating to use noDelay library for sections where a delay is needed but the rest of the program can run
change the coding for how the servo moves as previous for loops caused problems.
commented out serial outputs

26-09-21 updated by Benjamin Gibbs
added second ultrasonic. roughly positioned at 45 degrees to each other
edited to use nodelay for ping delay
edited to use nodelay for led blink
updated increment from 1 to 3 as it was moving slowly
last update trying to get ranging working unsuccesfully
takes which ever sensor has lowest figure then maps result to servo position

28-9-21 Benjamin Gibbs
v4.0
Adding use of NewPing libary as it allows better ranging
cover trigger on right sensor so it detects ping from left sensor
remove all related to previous sonar including no delay
Update getSonar to use newping

adding a maths section for calculating angles

29-9-21 Benjamin Gibbs
v4.2
improved maths section
added if/else if to main loop to try to stop each side trying to move servo at same time
split in to left and right sections. I think it would be better with a third center section
to remove the blind spots

30-9-21 Benjamin Gibbs
v4.3
RIGHT FUNCTIOMS FOR TARGING AND MATHS added
last version for now. Needs a complete rewrite really

15-10-2021 Benjamin Gibbs
adding code for tftscreen to display informatiom

18-10-2021 Benjamin Gibbs
displays both sensors on screen correctly

20-10-2021 Benjamin Gibbs
v5.2
done -change targeting function to inckude a bool showing side like screen display does
done - instead of refreshing whole screen, print rectangle over top part
done - moved sonar and range checks from loop to own functions

21-10-2021 Benjamin Gibbs
Adding blue rectangle to bottom of the screen
changed whole screen so connections on right and 0,0 is top left corner
*/
#include <Servo.h>
#include <NewPing.h>
#include <NoDelay.h>
#include <SPI.h>
#include <TFT.h>
#define trigPin 7 // define right TrigPin
#define echoPin 6 // define EchoPin.
#define trigPin2 5 // define left TrigPin
#define echoPin2 4 // define EchoPin.
#define servoPin 3
#define MAX_DISTANCE 200 // Maximum sensor distance is rated at 400-500cm.
#define cs 10
#define dc 9
#define rst 8
Servo myservo; //create servo object
//create noDelay objects
noDelay sweepTimer(15); 
noDelay ledWait(5);
noDelay sonarEvent(200); //time delay between pings
noDelay displayWait(500); //delay between showing blips on screen
noDelay blipRemoveWait(150); //delay for wiping blips from screen
//create NewPing objects
NewPing leftSonar(trigPin, echoPin, MAX_DISTANCE);
NewPing rightSonar(trigPin2, echoPin2, MAX_DISTANCE);
//set distance range for beginning and ending servo sweep
const uint8_t sweepRangeMax = 60; 
const uint8_t sweepRangeMin = 12; //set this to close to base and where ultrasonics output stops crossing

uint8_t pos; // used for servo positioning.Keep global to remember previous position
uint8_t increment = 3; //amount to increase servo position by
//used for storing echo results
uint8_t leftResult;
uint8_t rightResult;
//used for trigomentary later
const float baseWidth = 22; //distance from servo to servo
const float baseAngle = 52; //angle of ulltrasonic sensors
const float decToRad = 57.2958; // used for degree to radian conversion and vice versa
//set up display
TFT display = TFT(cs, dc, rst);
uint16_t displayXstart = 80;
uint16_t displayYstart = 78;
uint16_t displayWidth = 160;
uint16_t displayHeight = 128;

void setup() {
	// set pin modes for ultrasonics
	pinMode(trigPin,OUTPUT);
	pinMode(echoPin,INPUT);
	pinMode(trigPin2,OUTPUT);
	pinMode(echoPin2,INPUT);
	Serial.begin(9600);    // Open serial monitor at 9600 baud to see ping results.
	myservo.attach(servoPin); //connect servo to pin
	// initialise screen
	display.begin();
	display.background(0, 0, 0);
}

void loop() {
displayFirstRun();
checkSonar();
checkRanges();

if (displayWait.update() == true){
		displayBlip(leftResult, true);
		displayBlip(rightResult, false);
		displayWait.start();
		blipRemoveWait.start();
}
if (blipRemoveWait.update() == true){
		display.fillRect(0, 0, displayWidth, 98, 0);
//		display.fillScreen(0);
	}
}
void displayFirstRun(){
	display.setCursor(48, 98);
	display.setTextColor(65535, 0);
	display.fillRoundRect(-10, 98, 54, 128, 15, 63488);
	display.fillRoundRect(116, 98, 54, 128, 15, 63488);
	display.fillRect(0, 108, 160, 128, 63488);
	display.setCursor(85, 98);
	display.print("R:");
	display.print(rightResult);
	display.setCursor(45, 98);
	display.print("L:");
	display.print(leftResult);
		
}

void checkSonar(){
if (sonarEvent.update() == true){ //uses nodelay update to check if pingwait time amount has passed. returns true if it has
	getSonar();
//output results to serial
	Serial.print(" leftPing: ");
	Serial.println(leftResult);
	Serial.print(" rightPing: ");
	Serial.println(rightResult);
	sonarEvent.start(); //resets nodelay timer object
  }	
}
//gets distances from ultrasonic sensors
void getSonar() {
	leftResult = leftSonar.ping_cm();
	rightResult = rightSonar.ping_cm();	
}
// check if physical object is close enough to trigger sweep or targeting
void checkRanges(){
if (leftResult < sweepRangeMax && leftResult > sweepRangeMin){
	sweep();
   }
else if (rightResult < sweepRangeMax && rightResult > sweepRangeMin){
	sweep();	
   }	 
if (leftResult <= sweepRangeMin && leftResult >= 1){
	targeting(true);				
	}
else if (rightResult <= sweepRangeMin && rightResult >= 1){
	targeting(false);					
	}	
}
//sweeps servo backwards and forwards
void sweep() {
//	Serial.println("sweep start");
if (sweepTimer.update() == true){ //uses nodelay update to check if sweep1 time amount has passed. returns true if it has
	sweepTimer.start(); //reset nodelay timer object
	pos += increment; //position of servo = increment variable
	myservo.write(pos); //move servo to position
	}
if ((pos >= 180 )|| (pos <= 0)){ //if servo at end of range then change direction
	increment = -increment;
	}											
}
//positions servo based on doMaths result
void targeting(bool left) {
	uint8_t targetPos;
	if (left == true){
		targetPos = doMathsFromLeft(baseAngle, leftResult, baseWidth);
	}
	else if (left == false){
		targetPos = doMathsFromRight(baseAngle, rightResult, baseWidth);
	}
	myservo.write(targetPos);
	//makes it smoother when it goes back to sweeping	
	pos = targetPos;						
   sweepTimer.start();
}
void displayBlip(int a, bool left){
if (a < sweepRangeMax){
	uint8_t x;
	uint8_t y;
	display.fill(0, 0, 255);
	if (left == true){
		uint8_t x = map(a, 0, sweepRangeMax, displayXstart, displayWidth);
		uint8_t y = map(a, 0, sweepRangeMax, displayYstart, 0);
		display.circle(x, y, 5);
	}
	else if (left == false) {
		uint8_t x = map(a, 0, sweepRangeMax, displayXstart, 0);
		uint8_t y = map(a, 0, sweepRangeMax, displayYstart, 0);
		display.circle(x, y, 5);
	}
		}
}
// I just copied this from a maths webpage so dont understand it but it works out where to put the servo
float doMathsFromLeft(float A, float b, float c){
	float B;
	float a;
	float C;
   float degrees;
	float angle;
	A = A / decToRad;
   a = ((sq(b) + sq(c)) - ((2 * b * c) * cos(A)));
	a = sqrt(a);
	B = ((sin(A) * b )/ a);
	B = sin(B);
	degrees = B * decToRad; 	
	angle = (180 - degrees) - (A * decToRad);
	return angle;
}
float doMathsFromRight(float B, float a, float c){
	float A;
	float b;
	float C;
   float degrees;
	float angle;
	B = B / decToRad;
   b = ((sq(a) + sq(c)) - ((2 * a * c) * cos(B)));
	b = sqrt(b);
	A = ((sin(B) * a )/ b);
	A = sin(A);
	degrees = A * decToRad; 	
	angle = degrees + (A * decToRad);
	return angle;
}