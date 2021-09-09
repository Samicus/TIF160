#include <Servo.h>

#if ARDUINO >= 100
    #include "Arduino.h"
#else
    #include "WProgram.h"
#endif

// Class serial is used to communicate with the computer
class serial
{
    public:
		//open: Define the first character of a message
		//end: Define the last character of a message
		//spd: Define the speed of the serial port
        serial(char open, char end, int spd=9600); //default constuctor
        virtual ~serial(); //default virtual destructor
		void init(){Serial.begin(speed);}
		//Read the serial port and return a string
		//Return a void string if there are not info
		String read();
		//Write as array of unsigned char by the serial port
		void write(unsigned char* output, int lng);
		//Write a string by the serial port
		void writeString(String output);
		//Write an openChar by the serial port
		void writeOpen(){Serial.write(openChar);}
		//Write an endChar by the serial port
		void writeEnd(){Serial.write(endChar);}
		//Write an '\n' by the serial port
		void writeln(){Serial.write('\n');}
		
	private:
		boolean messageInit;  // whether the string is complete		
		String inputString;
		char openChar;
		char endChar;
		int cont;
		int speed;
		int leng;
};

/********************************************************************
* Main application
*********************************************************************/


int input;
int initialized = 0;

// Servos
Servo thumb;
Servo index;  
Servo middle;  
Servo ring;  
Servo pinky;  
Servo wrist;  

// Serial data
serial serialport('#', ';', 9600);
String msg = "";
String premsg = "";

void setup()
{    
    Serial.begin(9600);
}

void attachServo()
{  
    thumb.attach(2);
    index.attach(3);
    middle.attach(4);
    ring.attach(5);
    pinky.attach(6);  
    wrist.attach(7);
    initialized = 1;
}

void detachServo()
{  
    thumb.detach();
    index.detach();
    middle.detach();
    ring.detach();
    pinky.detach();  
    wrist.detach();
    initialized = 0;
}

void setAll(int angle)
{
  thumb.write(0);
  index.write(angle);
  middle.write(0);
  ring.write(0);
  pinky.write(0);  
}

void setAllMax()
{
  thumb.write(150);
  index.write(180);
  middle.write(180);
  ring.write(120);
  pinky.write(120); 
  wrist.write(90); 
}

void setServo(int servoNum, int servoAngle)
{
  if(initialized == 0)
    attachServo();
    
  switch(servoNum)
  {
    case 2:
    thumb.write(servoAngle);
    break;
    
    case 3:
    index.write(servoAngle);
    break;
    
    case 4:
    middle.write(servoAngle);
    break;
    
    case 5:
    ring.write(servoAngle);
    break;
    
    case 6:
    pinky.write(servoAngle);
    break;
    
    case 7:
    wrist.write(servoAngle);
    break;
    
    default:
    break;    
  }
}

void loop()
{      
    msg = serialport.read(); //read a message

    // If no data is available
    if(msg == "none" || msg == "") 
    {
      detachServo();
    }    
    else
    {      
      int n;
      n = atoi(msg.c_str());
      int servoNum = n/1000; // Thousands for servo number
      int servoAngle = n%1000;
      setServo(servoNum, servoAngle);
    }
}




/********************************************************************
* Class serial
*********************************************************************/

serial::serial(char open, char end, int spd)
{
	openChar = open;
	endChar = end;
	cont = 0;
	messageInit = false;
	speed = spd;
	inputString = "";
}

serial::~serial()
{
}

void serial::write(unsigned char* output, int lng)
{
    for(int c = 0; c < lng ; c++) Serial.write(output[c]);
}

void serial::writeString(String output)
{	
	unsigned char *msg;
	msg = new unsigned char [output.length()];
	strcpy( (char*) msg, output.c_str());
	serial::write(msg, output.length());
	delete[] msg;
}

String serial::read()
{ 
  if (Serial.available()) {
    char inChar = (char)Serial.read(); 
    if (inChar == endChar){
      messageInit = false;
	  return inputString;
    }
    if (messageInit){
      inputString += inChar;
    }
    if (inChar == openChar){
	  inputString = "";
      messageInit = true;
    }
  }
  return "";
}
