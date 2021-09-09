#include <iostream>
#include <cstring>
#include <cstdio>
#include <fcntl.h>      // for low-level file open()
#include <termios.h>    // for seral-port functions
#include <math.h>       // for maths and M_PI
#include <fstream>      // for file io
#include <stdio.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <string.h>
#include <iostream>     // for standard IO
#include "Leap.h"


using namespace Leap;
using namespace std;

class SampleListener : public Listener {
  public:
    virtual void onInit(const Controller&);
    virtual void onConnect(const Controller&);
    virtual void onDisconnect(const Controller&);
    virtual void onExit(const Controller&);
    virtual void onFrame(const Controller&);
    virtual void onFocusGained(const Controller&);
    virtual void onFocusLost(const Controller&);
    virtual void onDeviceChange(const Controller&);
    virtual void onServiceConnect(const Controller&);
    virtual void onServiceDisconnect(const Controller&);
    void sendAnglesArduino(int, int);
    void sendMessageArduino(string);
    float radToDeg(const float&);
    
  private:
};

class SerialPort
{        
	public:
		SerialPort();
		int init(const char device[]);  
		void close();
		int setBaud(int baud);
		int receive(int num,unsigned char* cade);
		int send(int num,unsigned char* cade);
			
	protected:

	#ifdef WIN32
		DCB dcb;
		HANDLE  idCom;
		COMMTIMEOUTS timeouts;
	#else
		int serialPort;
	#endif
};

class serial
{
    public:
		//open: Define the first character of a message
		//end: Define the last character of a message
		//device: Define the serial port ID
		//spd: Define the speed of the serial port
        serial(char open, char end, const char device[], int spd=9600); //default constuctor
        virtual ~serial(); //default virtual destructor
		//Read the serial port and return a string
		//Return a void string if there are not info
		string read();
		//Write as array of unsigned char by the serial port
		void write(unsigned char* data, int lng){serialport.send(lng,data);}
		//Write a string by the serial port
		void writeString(string output);
		
	private:
		bool messageInit;

		char openChar;
		char endChar;
		int cont;
		int speed;
		int separators;
		string inputString;
		SerialPort serialport;

};

const std::string fingerNames[] = {"Thumb", "Index", "Middle", "Ring", "Pinky", "Wrist"};
const std::string boneNames[] = {"Metacarpal", "Proximal", "Middle", "Distal"};
const std::string stateNames[] = {"STATE_INVALID", "STATE_START", "STATE_UPDATE", "STATE_END"};

const int mapping[6][4] =
{
	{2, 300, 20, 0}, // Thumb{servo number, max range servo, offset, max range reality}
	{3, 180, -20, 0}, // Index
	{4, 180, -20, 0}, // Middle
	{5, 140, -20, 0}, // Ring
	{6, 160, -20, 0}, // Pinky
	{7, 180, 0, 0}, // Wrist
};
const char* device = "/dev/ttyACM0";
serial serialport('#', ';', "/dev/ttyACM0", 9600);

void SampleListener::onInit(const Controller& controller) {
	std::cout << "Leap Initialized" << std::endl;
}

void SampleListener::onConnect(const Controller& controller) {
  std::cout << "Connected" << std::endl;
  
}

void SampleListener::onDisconnect(const Controller& controller) {
  // Note: not dispatched when running in a debugger.
  std::cout << "Disconnected" << std::endl;
}

void SampleListener::onExit(const Controller& controller) {
  std::cout << "Exited" << std::endl;
}

void SampleListener::onFrame(const Controller& controller) {

	// Get the most recent frame from leap
	const Frame frame = controller.frame();

	HandList hands = frame.hands();
	if(hands.isEmpty())
	{
		sendMessageArduino("none");
	}

	for (HandList::const_iterator hl = hands.begin(); hl != hands.end(); ++hl) 
	{
		// Get the first hand
		const Hand hand = *hl;
		// Skip if left hand
		if(hand.isLeft())
			continue;

		// Get the hand's normal vector and direction
		const Vector normal = hand.palmNormal();
		const float roll = normal.roll(); // in rad // * RAD_TO_DEG to get degree

		// Send roll to ardunio
		sendAnglesArduino(5, roll*RAD_TO_DEG + 130);
		
		const FingerList fingers = hand.fingers();
		for (FingerList::const_iterator fl = fingers.begin(); fl != fingers.end(); ++fl) 
		{
			const Finger finger = *fl;

			// Get all bones  
			Bone metacarpal = finger.bone(static_cast<Bone::Type>(0));
			Bone proximal = finger.bone(static_cast<Bone::Type>(1));
			Bone middle = finger.bone(static_cast<Bone::Type>(2));
			Bone distal = finger.bone(static_cast<Bone::Type>(3));

			// Calculate angles in radian
			float angleMetaProxi = metacarpal.direction().angleTo(proximal.direction());
			float angleProxiMid = proximal.direction().angleTo(middle.direction());
			float angleMidDist = middle.direction().angleTo(distal.direction());
			float total = angleMetaProxi + angleProxiMid + angleMidDist ;

			int angle = total * RAD_TO_DEG;

			sendAnglesArduino(finger.type(),angle);
		}
		if (!fingers.isEmpty())
				cout << endl;	

	}
}

void SampleListener::onFocusGained(const Controller& controller) {
  std::cout << "Focus Gained" << std::endl;
}

void SampleListener::onFocusLost(const Controller& controller) {
  std::cout << "Focus Lost" << std::endl;
}

void SampleListener::onDeviceChange(const Controller& controller) {
  std::cout << "Device Changed" << std::endl;
  const DeviceList devices = controller.devices();

  for (int i = 0; i < devices.count(); ++i) {
    std::cout << "id: " << devices[i].toString() << std::endl;
    std::cout << "  isStreaming: " << (devices[i].isStreaming() ? "true" : "false") << std::endl;
  }
}

void SampleListener::onServiceConnect(const Controller& controller) {
  std::cout << "Service Connected" << std::endl;
}

void SampleListener::onServiceDisconnect(const Controller& controller) {
  std::cout << "Service Disconnected" << std::endl;
}

void SampleListener::sendAnglesArduino(int finger, int angleLeap)
{    
    int servoPin = mapping[finger][0];
    
    // Offset due to leap not reading 0 when flat
    int angleServo = angleLeap + mapping[finger][2]; 
    
    // mapping due to max range of servos 
    angleServo = angleServo*mapping[finger][1]/180; 
    
    int toSend = servoPin*1000 + angleServo;  
	
	cout << fingerNames[finger] << " " << angleLeap << "(" << angleServo << ") ";			
	
	stringstream ss;
	ss << toSend;
	
	sendMessageArduino(ss.str());
}
void SampleListener::sendMessageArduino(string data)
{
    string incomingData = "#" + data + ";";
	serialport.writeString(incomingData);
}

int main(int argc, char** argv) {
  // Create a sample listener and controller
  SampleListener listener;
  Controller controller;

  // Have the sample listener receive events from the controller
  controller.addListener(listener);

  if (argc > 1 && strcmp(argv[1], "--bg") == 0)
    controller.setPolicy(Leap::Controller::POLICY_BACKGROUND_FRAMES);

  // Keep this process running until Enter is pressed
  std::cout << "Press Enter to quit..." << std::endl;
  std::cin.get();

  // Remove the sample listener when done
  controller.removeListener(listener);

  return 0;
}

/****************************************************************************/

serial::serial(char open, char end, const char device[], int spd)
{
	serialport.init(device);
	serialport.setBaud(spd);
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

void serial::writeString(string output)
{	
	unsigned char *msg;
	msg = new unsigned char [output.length()];
	strcpy( (char*) msg, output.c_str());
	serial::write(msg, output.length());
	delete[] msg;
}

string serial::read(){
	unsigned char single[1] = "";
	while (serialport.receive(0, single)>0)
	{	
		if (single[0] == endChar){
		  messageInit = false;
		  return inputString;
		}
		if (messageInit){
		  inputString += single[0];
		}
		if (single[0] == openChar){
		  inputString = "";
          messageInit = true;
		}
	}
	return "";
}


//CONFIGURATION PARAMETERS
#define PORT_COMMS_INTERVAL_TIMEOUT         150        //millisecs
#define PORT_COMMS_TOTAL_TIMEOUT_MULTIPLIER 150        //millisecs
#define PORT_COMMS_TOTAL_TIMEOUT_CONSTANT   0   //millisecs
        

//ERROR CODES
#define PORT_OK                   0
#define PORT_ERROR               -1
#define PORT_ERROR_CREATEFILE    -101
#define PORT_ERROR_SETDCB        -102
#define PORT_ERROR_WRITEFILE     -103 
#define PORT_ERROR_WRITENUM      -104 
#define PORT_ERROR_READFILE      -105
#define PORT_ERROR_READNUM       -106
#define PORT_ERROR_CLOSE_HANDLE  -107
#define PORT_ERROR_BAD_PARAMETER -108
#define PORT_ERROR_TIMEOUT       -109




SerialPort::SerialPort()
{
    #ifdef WIN32
        idCom=NULL;
    #else
        serialPort=-1;                
    #endif
}
int SerialPort::init (const char device[]) 
{
    #ifdef WIN32
        idCom=CreateFile(device,GENERIC_READ|GENERIC_WRITE,0,NULL,OPEN_EXISTING,FILE_ATTRIBUTE_NORMAL,NULL);
        if(idCom==INVALID_HANDLE_VALUE ) 
        {        
            idCom=NULL;
            printf("Error Opening port Win32\n");
            return(PORT_ERROR_CREATEFILE);
        }
        else
        {        
            GetCommState(idCom,&dcb);

            dcb.BaudRate=CBR_9600;
            dcb.fBinary=1;
            dcb.fOutxCtsFlow=0;
            dcb.fOutxDsrFlow=0;
            dcb.fDtrControl=1;////??????????
            //dcb.fDtrControl=DTR_CONTROL_DISABLE;////??????????
            dcb.fDsrSensitivity=0;
            dcb.fTXContinueOnXoff=1;
            dcb.fOutX=0;
            dcb.fInX=0;
            dcb.fErrorChar=0;
            dcb.fNull=0;
            dcb.fRtsControl=1;
            dcb.fAbortOnError=1;
            dcb.XonLim=2048;
            dcb.XoffLim=512;
            dcb.ByteSize=8;
            dcb.Parity=0;
            dcb.StopBits=0;
            //dcb.StopBits=ONESTOPBIT;
            dcb.XonChar=17;
            dcb.XoffChar=19;
            dcb.ErrorChar=0;
            dcb.EofChar=0;
            dcb.EvtChar=0;
            dcb.DCBlength=sizeof(dcb);
            if(SetCommState(idCom,&dcb)==0)
            {
                CloseHandle(idCom);
                idCom=NULL;
                printf("Unable to configure port\n");
                return(PORT_ERROR_SETDCB);
            }
            timeouts.ReadIntervalTimeout=PORT_COMMS_INTERVAL_TIMEOUT;
            timeouts.ReadTotalTimeoutMultiplier=PORT_COMMS_TOTAL_TIMEOUT_MULTIPLIER;
            timeouts.ReadTotalTimeoutConstant=PORT_COMMS_TOTAL_TIMEOUT_CONSTANT;
            timeouts.WriteTotalTimeoutMultiplier=0;
            timeouts.WriteTotalTimeoutConstant=0;
            SetCommTimeouts(idCom,&timeouts);

            return(PORT_OK);
        }
        
    #else        
        if(serialPort!=-1)
            close();
        struct termios serialSettings;
        serialPort=-1;
        serialPort = open (device, O_RDWR |O_NOCTTY);

        if(serialPort==-1) 
        {
            printf("Error al abrir el puerto\n");
            return PORT_ERROR;
        }

        serialSettings.c_cc[VMIN] = 0;
        serialSettings.c_cc[VTIME] = 1;
 
        serialSettings.c_oflag = 0;
        serialSettings.c_iflag = ( IGNPAR ) ;//DRL( IGNBRK | IGNPAR ) ;
        serialSettings.c_cflag = ( CLOCAL| CREAD | B9600 | CS8 );
        serialSettings.c_lflag = 0;   

        cfsetospeed (&serialSettings, B9600);
        cfsetispeed (&serialSettings, B9600);

        if(tcsetattr (serialPort, TCSANOW, &serialSettings)!=0)
            return PORT_ERROR;
        else return PORT_OK;

    #endif
}

void SerialPort::close () 
{
    #ifdef WIN32
        CloseHandle(idCom);        
        idCom=NULL;
    #else
        ::close(serialPort);
        serialPort=-1;
    #endif        
}
   

int SerialPort::receive ( int num,unsigned char * cad) 
{
    #ifdef WIN32
        unsigned long read=0;
        unsigned long read2=0;
        ///LEo un primer cararcter
        if(ReadFile(idCom,cad,1,&read2,NULL)==0) 
            return PORT_ERROR_READFILE;
        if(read2!=1)return PORT_ERROR_TIMEOUT;
        //SI lo he leido bien sigo leyendo los que me piden
        if(num>1)//intento leer el resto si lo hay
        {
            if(ReadFile(idCom,cad+1,num-1,&read,NULL)==0)
                return PORT_ERROR_READFILE;
        }
        return(read+1);
    #else
        char car;
        int i=0;
    //printf("SE solicitan %d\n",num);
    while(read(serialPort,&car,1)==1)
    {
    //printf("Leido 1\n");
        cad[i]=car;
        i++;
        if(i>=num)break;
    }
    //int leidos= read(serialPort,cad,num);
    int leidos=i;

    //printf("Se han leido %d\n",leidos);
        return leidos;
    #endif        
}
   
int SerialPort::send (int n,unsigned char * cad) 
{
    #ifdef WIN32
        unsigned long written=0;
        if(WriteFile(idCom,cad,n,&written,NULL)==0) return(PORT_ERROR_WRITEFILE);
        if(written!=n) 
            return(PORT_ERROR_WRITENUM);
        return(PORT_OK);
    #else
        // printf("Enviando %d\n",n);
        int num=write(serialPort, cad, n);
        if(num!=n)
            return PORT_ERROR_WRITENUM;
        return PORT_OK;
    #endif        
}


int SerialPort::setBaud (int baud) 
{        
    #ifdef WIN32
        dcb.BaudRate=baud;

        if(SetCommState(idCom,&dcb)==0)
            return(PORT_ERROR_SETDCB);
        return(PORT_OK);
    #else
        struct termios serialSettings;
        
        serialSettings.c_cc[VMIN] = 0;
        serialSettings.c_cc[VTIME] = 1;
     
        serialSettings.c_oflag = 0;
        serialSettings.c_iflag = ( IGNBRK | IGNPAR ) ;
        serialSettings.c_cflag = ( CLOCAL| CREAD | B9600 | CS8 );
        serialSettings.c_lflag = 0;   
                
        if(baud==38400)
        {
            cfsetospeed (&serialSettings, B38400);
            cfsetispeed (&serialSettings, B38400);
        }    
        else if(baud==9600)
        {
            cfsetospeed (&serialSettings, B9600);
            cfsetispeed (&serialSettings, B9600);
        }
        else if(baud==115200)
        {
            cfsetospeed (&serialSettings, B115200);
            cfsetispeed (&serialSettings, B115200);
        }
        #ifdef __LINUX__
            else if (baud==500000)
            {
                cfsetospeed (&serialSettings, B500000);
                cfsetispeed (&serialSettings, B500000);
            }
        #endif
        if(tcsetattr (serialPort, TCSANOW, &serialSettings)!=0)
            return PORT_ERROR;
        else 
            return PORT_OK;
    #endif        
}
