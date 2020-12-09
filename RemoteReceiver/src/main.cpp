#include <Arduino.h>
//#include <SPI.h>
#include "RF24.h"

#define HOVER_SERIAL_BAUD   38400       // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define SERIAL_BAUD         115200      // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define START_FRAME         0xABCD     	// [-] Start frme definition for reliable serial communication
#define TIME_SEND           50         // [ms] Sending time interval
#define TIME_RECEIVE        300         // [ms] Timeout for NRF Module

HardwareSerial HoverSerial(2);

// Global variables
uint8_t idx = 0;                        // Index for new data pointer
uint16_t bufStartFrame;                 // Buffer Start Frame
byte *p;                                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;

typedef struct{
   uint16_t	start;
   int16_t  steer;
   int16_t  speed;
   uint16_t checksum;
} SerialCommand;
SerialCommand Command;

typedef struct{
   uint16_t start;
   int16_t 	cmd1;
   int16_t 	cmd2;
   int16_t 	speedR_meas;
   int16_t 	speedL_meas;
   int16_t 	batVoltage;
   int16_t 	boardTemp;
   uint16_t cmdLed;
   uint16_t checksum;
} SerialFeedback;
SerialFeedback Feedback;
SerialFeedback NewFeedback;

unsigned long iTimeSend = 0;
unsigned long rTimeSend = 0;

RF24 radio(4, 5); // CE, CSN

const byte address[6] = "11001"; //David: 11001 Henning: 10101

int lost_connections = 0;
int steer, speed = 0;

struct Data_Package {
  uint16_t voltage;
  uint16_t x; // Left - Right
  uint16_t y; // Forward - Backward
  uint16_t z; // Twist: Left - Right
  uint8_t button;
};

Data_Package data;


void setup() {
  Serial.begin(230400);
  HoverSerial.begin(HOVER_SERIAL_BAUD);
  if(radio.begin()){
    Serial.println("NRF Ready");
  }
  else{
    Serial.println("NRF fail");
  }
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_1MBPS);
  radio.setCRCLength(RF24_CRC_8);
  radio.setAutoAck(1);
  radio.enableAckPayload();
  radio.openReadingPipe(0, address);
  radio.startListening();
  Serial.println("Receiver");
}

void Send(int16_t uSteer, int16_t uSpeed) {
  // Create command
  Command.start    = (uint16_t)START_FRAME;
  Command.steer    = (int16_t)uSteer;
  Command.speed    = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);

  // Write to Serial
  HoverSerial.write((uint8_t *) &Command, sizeof(Command));
}

void Receive() {
	// Check for new data availability in the Serial buffer
	if (HoverSerial.available()) {
		incomingByte 	  = HoverSerial.read();		                              // Read the incoming byte
		bufStartFrame	= ((uint16_t)(incomingByte) << 8) | incomingBytePrev;	  // Construct the start frame
	}
	else {
		return;
	}

  // If DEBUG_RX is defined print all incoming bytes
  #ifdef DEBUG_RX
		//Serial.print(incomingByte);
		return;
	#endif

	// Copy received data
	if (bufStartFrame == START_FRAME) {	                    // Initialize if new data is detected
		p 		= (byte *)&NewFeedback;
    *p++  = incomingBytePrev;
		*p++ 	= incomingByte;
		idx 	= 2;
	} else if (idx >= 2 && idx < sizeof(SerialFeedback)) {	// Save the new received data
		*p++ 	= incomingByte;
		idx++;
	}

	// Check if we reached the end of the package
	if (idx == sizeof(SerialFeedback)) {
		uint16_t checksum;
		checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas
					^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);

		// Check validity of the new data
		if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum) {
			// Copy the new data
			memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));

			// Print data to built-in Serial
			Serial.print("1: ");   Serial.print(Feedback.cmd1);
			Serial.print(" 2: ");  Serial.print(Feedback.cmd2);
			Serial.print(" 3: ");  Serial.print(Feedback.speedR_meas);
			Serial.print(" 4: ");  Serial.print(Feedback.speedL_meas);
			Serial.print(" 5: ");  Serial.print(Feedback.batVoltage);
			Serial.print(" 6: ");  Serial.print(Feedback.boardTemp);
			Serial.print(" 7: ");  Serial.println(Feedback.cmdLed);
		} else {
		  //Serial.println("Non-valid data skipped");
		}
		idx = 0;	// Reset the index (it prevents to enter in this if condition in the next cycle)
	}

	// Update previous states
	incomingBytePrev 	= incomingByte;
}

void loop() {

  unsigned long timeNow = millis();

  if (radio.available()) {
    radio.read(&data, sizeof(data));
    steer = map(data.y, 0, 1023, -1000, 1000);
    speed = map(data.x, 0, 1023, -1000, 1000);
    if(speed > 0){
      speed = map(speed, 60, 1000, 0, 1000);
    }
    if(speed < 0){
      speed = map(speed, -1000, -60, -1000, 0);
    }
    if(steer > 0){
      steer = map(steer, 60, 1000, 0, 1000);
    }
    if(steer < 0){
      steer = map(steer, -1000, -60, -1000, 0);
    }
    float voltage = (data.voltage*0.00488)+0.14;
    //Serial.print("Speed: ");
    //Serial.println(speed);
    //Serial.print("Steer: ");
    //Serial.println(steer);
    /*Serial.print("Button: ");
    Serial.println(data.button);
    /*Serial.print("Voltage: ");
    Serial.println(voltage);*/
    rTimeSend = timeNow + TIME_RECEIVE;
  }
  else if (rTimeSend < timeNow){
    //Serial.println("No Signal");
    steer = 0;
    speed = 0;
  }

  Receive();

  if (iTimeSend > timeNow) return;
  iTimeSend = timeNow + TIME_SEND;
  Send(steer, speed);
}
