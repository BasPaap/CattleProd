/*
    Name:       Bas.CattleProd.ino
    Created:	4-12-2019 16:17:16
    Author:     DESKTOP-CHGLIEU\baspa
*/

#include "humSound.h"
#include "zapSound.h"

#define SAMPLE_RATE 8000;

const int triggerButtonPin = 7;
const int probeButtonPin = 3;
const int arcLedPin = 5;
const int statusLedPin = 4;
const int speakerPin = 11;  // Can be either 3 or 11, two PWM outputs connected to Timer 2
const int smokeRelayPin = 12;

volatile uint16_t sample;
byte lastSample;
unsigned char const* sounddata_data = 0;
int sounddata_length = 0;
int lastTriggerButtonState = HIGH;   // the previous reading from the input pin
unsigned long lastTriggerButtonDebounceTime = 0;  // the last time the output pin was toggled
int triggerButtonState = HIGH;             // the current reading from the input pin
int lastProbeButtonState = HIGH;   // the previous reading from the input pin
unsigned long lastProbeButtonDebounceTime = 0;  // the last time the output pin was toggled
int probeButtonState = HIGH;             // the current reading from the input pin
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers
bool isAudioLooping = false;
bool isTriggerButtonDown = false;
bool isProbeButtonDown = false;

bool arcLedIsHigh = false;
const int minArcLedIntensity = 0;
const int maxArcLedIntensity = 127;
const long minMillisecondsBetweenArcs = 50;
long timeSinceLastArc = 0;


// The setup() function runs once each time the micro-controller starts
void setup()
{
	Serial.begin(9600);

	pinMode(triggerButtonPin, INPUT_PULLUP);
	pinMode(probeButtonPin, INPUT_PULLUP);	
	pinMode(statusLedPin, OUTPUT);
	pinMode(smokeRelayPin, OUTPUT);

	digitalWrite(smokeRelayPin, LOW);	// Ensure the smoke machine is off.
	digitalWrite(statusLedPin, HIGH);	// Turn on the status LED to signal the cattle prod being ready.
}

// Add the main program code into the continuous loop() function
void loop()
{
	readTriggerButton();
	readProbeButton();

	if (isTriggerButtonDown)
	{
		if (millis() - timeSinceLastArc > minMillisecondsBetweenArcs)
		{
			int intensity = arcLedIsHigh ? minArcLedIntensity : maxArcLedIntensity;
			intensity += isProbeButtonDown ? 255 - maxArcLedIntensity : 0;
			arcLedIsHigh = !arcLedIsHigh;
			analogWrite(arcLedPin, intensity);
			timeSinceLastArc = millis();
		}
	}
	else
	{
		analogWrite(arcLedPin, 0);
	}
}

void readTriggerButton()
{
	// read the state of the switch into a local variable:
	int reading = digitalRead(triggerButtonPin);

	// check to see if you just pressed the button
	// (i.e. the input went from LOW to HIGH), and you've waited long enough
	// since the last press to ignore any noise:

	// If the switch changed, due to noise or pressing:
	if (reading != lastTriggerButtonState) {
		// reset the debouncing timer
		lastTriggerButtonDebounceTime = millis();
	}

	if ((millis() - lastTriggerButtonDebounceTime) > debounceDelay) {
		// whatever the reading is at, it's been there for longer than the debounce
		// delay, so take it as the actual current state:
		bool isButtonStateChanged = reading != triggerButtonState;
		triggerButtonState = reading;

		// If the button state has changed:
		if (isButtonStateChanged) {
			if (triggerButtonState == LOW)
			{
				// The button is pressed, so keep looping the loop sound while pressed
				isTriggerButtonDown = true;
				isAudioLooping = true;
				
				if (isProbeButtonDown)
				{
					startPlayback(zapSound, sizeof(zapSound));
				}
				else
				{
					startPlayback(humSound, sizeof(humSound));
				}
			}
			else
			{
				// The button has been released, so stop the sound.
				isAudioLooping = false;
				isTriggerButtonDown = false;
				stopPlayback();
			}
		}
	}

	// save the reading. Next time through the loop, it'll be the lastButtonState:
	lastTriggerButtonState = reading;
}


void readProbeButton()
{
	// read the state of the switch into a local variable:
	int reading = digitalRead(probeButtonPin);

	// check to see if you just pressed the button
	// (i.e. the input went from LOW to HIGH), and you've waited long enough
	// since the last press to ignore any noise:

	// If the switch changed, due to noise or pressing:
	if (reading != lastProbeButtonState) {
		// reset the debouncing timer
		lastProbeButtonDebounceTime = millis();
	}

	if ((millis() - lastProbeButtonDebounceTime) > debounceDelay) {
		// whatever the reading is at, it's been there for longer than the debounce
		// delay, so take it as the actual current state:
		bool isButtonStateChanged = reading != probeButtonState;
		probeButtonState = reading;

		// If the button state has changed:
		if (isButtonStateChanged) {
			if (probeButtonState == LOW)
			{
				isProbeButtonDown = true;
				
				if (isTriggerButtonDown)
				{
					digitalWrite(smokeRelayPin, HIGH); // Turn on the smoke machine
					switchToSound(zapSound, sizeof(zapSound));
				}
			}
			else
			{
				// The button has been released, so stop zapping.
				isProbeButtonDown = false;
				digitalWrite(smokeRelayPin, LOW); // Turn off the smoke machine
				switchToSound(humSound, sizeof(humSound));
			}
		}
	}

	// save the reading. Next time through the loop, it'll be the lastButtonState:
	lastProbeButtonState = reading;
}

void switchToSound(unsigned char const* data, int length)
{
	sounddata_data = data;
	sounddata_length = length;
	lastSample = pgm_read_byte(&sounddata_data[sounddata_length - 1]);
	sample = 0;
}

void stopPlayback()
{
	// Disable playback per-sample interrupt.
	TIMSK1 &= ~_BV(OCIE1A);

	// Disable the per-sample timer completely.
	TCCR1B &= ~_BV(CS10);

	// Disable the PWM timer.
	TCCR2B &= ~_BV(CS10);

	digitalWrite(speakerPin, LOW);
}

// This is called at 8000 Hz to load the next sample.
ISR(TIMER1_COMPA_vect) {

	if (isAudioLooping && sample >= sounddata_length)
	{
			sample = 0;
	}
	else
	{
		if (sample >= sounddata_length) {
			if (sample == sounddata_length + lastSample) {
				stopPlayback();
			}
			else {
				if (speakerPin == 11) {
					// Ramp down to zero to reduce the click at the end of playback.
					OCR2A = sounddata_length + lastSample - sample;
				}
				else {
					OCR2B = sounddata_length + lastSample - sample;
				}
			}
		}
		else {
			if (speakerPin == 11) {
				OCR2A = pgm_read_byte(&sounddata_data[sample]);
			}
			else {
				OCR2B = pgm_read_byte(&sounddata_data[sample]);
			}
		}

		++sample;
	}
}

void startPlayback(unsigned char const* data, int length)
{
	sounddata_data = data;
	sounddata_length = length;

	pinMode(speakerPin, OUTPUT);

	// Set up Timer 2 to do pulse width modulation on the speaker
	// pin.

	// Use internal clock (datasheet p.160)
	ASSR &= ~(_BV(EXCLK) | _BV(AS2));

	// Set fast PWM mode  (p.157)
	TCCR2A |= _BV(WGM21) | _BV(WGM20);
	TCCR2B &= ~_BV(WGM22);

	if (speakerPin == 11) {
		// Do non-inverting PWM on pin OC2A (p.155)
		// On the Arduino this is pin 11.
		TCCR2A = (TCCR2A | _BV(COM2A1)) & ~_BV(COM2A0);
		TCCR2A &= ~(_BV(COM2B1) | _BV(COM2B0));
		// No prescaler (p.158)
		TCCR2B = (TCCR2B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);

		// Set initial pulse width to the first sample.
		OCR2A = pgm_read_byte(&sounddata_data[0]);
	}
	else {
		// Do non-inverting PWM on pin OC2B (p.155)
		// On the Arduino this is pin 3.
		TCCR2A = (TCCR2A | _BV(COM2B1)) & ~_BV(COM2B0);
		TCCR2A &= ~(_BV(COM2A1) | _BV(COM2A0));
		// No prescaler (p.158)
		TCCR2B = (TCCR2B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);

		// Set initial pulse width to the first sample.
		OCR2B = pgm_read_byte(&sounddata_data[0]);
	}

	// Set up Timer 1 to send a sample every interrupt.
	cli();

	// Set CTC mode (Clear Timer on Compare Match) (p.133)
	// Have to set OCR1A *after*, otherwise it gets reset to 0!
	TCCR1B = (TCCR1B & ~_BV(WGM13)) | _BV(WGM12);
	TCCR1A = TCCR1A & ~(_BV(WGM11) | _BV(WGM10));

	// No prescaler (p.134)
	TCCR1B = (TCCR1B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);

	// Set the compare register (OCR1A).
	// OCR1A is a 16-bit register, so we have to do this with
	// interrupts disabled to be safe.
	OCR1A = F_CPU / SAMPLE_RATE;    // 16e6 / 8000 = 2000

	// Enable interrupt when TCNT1 == OCR1A (p.136)
	TIMSK1 |= _BV(OCIE1A);

	lastSample = pgm_read_byte(&sounddata_data[sounddata_length - 1]);
	sample = 0;
	sei();
}