#include <JeeLib.h>
#include <avr/sleep.h>

#define DEBUG	 1	 // set to 1 to display each loop() run and PIR trigger

// Watchdog is a timerTick on a avg 8,2 sec timebase
// SEND_MSG_EVERY=7	-> +- 1min
// SEND_MSG_EVERY=14 -> +- 2min
// SEND_MSG_EVERY=22 -> +- 3min
// SEND_MSG_EVERY=26 -> +- 4min
// SEND_MSG_EVERY=33 -> +- 5min

#define SEND_MSG_BATT_EVERY	1584	// Measure battery voltage every N ticks
																	 // MEASURE_EVERY=1584 -> +- 4 hour

#define NODE_ID				 5				 // NodeId of this JeeNode
#define GROUP_ID				5				 // GroupId of this JeeNode

volatile unsigned int sendMsgBatteryLevelTimer = SEND_MSG_BATT_EVERY - 1;

//Message max 8 bytes
struct payload {
	 byte batteryLevel;				 //getVcc 1.0V = 0, 1.8V = 40, 3.3V = 115, 5.0V = 200, 6.0V = 250
	 bool doorOpenChanged;
	 bool doorOpen;
} payload;

//Number of pulses, used to measure energy.
volatile bool sendMsg = false;
volatile bool timerTick = false;
volatile bool doorOpenChanged = false;
volatile bool adcDone = false;

//Pins JeeNode port 1
#define DOOR_INPUT	4 //Port1.DIO

ISR(WDT_vect) {
	 Sleepy::watchdogEvent();
	 if(!timerTick) { timerTick = true; }
}

ISR(ADC_vect) { adcDone = true; }

//Pin change interrupt routine for PortD0..7, only PD4 is used
ISR(PCINT2_vect) { doorOpenChanged = true; }

static byte batteryLevelRead (byte count =4) {
	 set_sleep_mode(SLEEP_MODE_ADC);
	 ADMUX = bit(REFS0) | 14; // use VCC and internal bandgap
	 bitSet(ADCSRA, ADIE);
	 while (count-- > 0) {
			adcDone = false;
			while (!adcDone) sleep_mode();
	 }
	 bitClear(ADCSRA, ADIE);
	 // convert ADC readings to fit in one byte, i.e. 20 mV steps:
	 //	1.0V = 0, 1.8V = 40, 3.3V = 115, 5.0V = 200, 6.0V = 250
	 return (55U * 1023U) / (ADC + 1) - 50;
}

void setup() {
	 rf12_initialize(NODE_ID, RF12_868MHZ, GROUP_ID);

	 //Init the door input and pin change IRQ routine on this input
	 //Bron: http://thewanderingengineer.com/2014/08/11/arduino-pin-change-interrupts/
	 pinMode(DOOR_INPUT,INPUT_PULLUP);
	 bitSet(PCICR,	PCIE2);				//Enable pin change interrupt for Port D (D0..D7)
	 bitSet(PCMSK2, DOOR_INPUT);	 //Choose which pins has to interrupt: D4

	 bitSet(EICRA, ISC11);
	 bitClear(EICRA, ISC10);

	 Sleepy::watchdogInterrupts(6); //Start the watchdog timer for time base
}

void loop() {
	if (doorOpenChanged) {
		doorOpenChanged = false;
		payload.doorOpenChanged = true;
		sendMsg = true;
	}

	if (timerTick)
	{ // There has ben a Watchdog interrupt for time measurement
		timerTick = false;

		sendMsgBatteryLevelTimer++;
		if (sendMsgBatteryLevelTimer >= SEND_MSG_BATT_EVERY) {
			sendMsgBatteryLevelTimer = 0;
			payload.batteryLevel = batteryLevelRead();
			sendMsg = true;
		}
	}

	if (sendMsg)
	{
		sendMsg = false;

		payload.doorOpen = !digitalRead(DOOR_INPUT);

		rf12_sleep(RF12_WAKEUP);
		while (!rf12_canSend())
		rf12_recvDone();
		rf12_sendStart(0, &payload, sizeof payload);
		rf12_sendWait(1);
		rf12_sleep(RF12_SLEEP);

		payload.doorOpenChanged = false;
	}
	Sleepy::watchdogInterrupts(9);		// Start the watchdog timer for timerTick 6=5,2sec 9=8,31sec
	Sleepy::powerDown();
}
