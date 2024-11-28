#include <avr/io.h>
#include <avr/interrupt.h>

bool audioEnabled = false; // Global flag to control audio output
const int buttonPin = 2;   // Pin connected to button


uint16_t syncPhaseAcc;
uint16_t syncPhaseInc;

//map analog input
#define EOG_SYNC_CONTROL  (1)
#define PIN 3 //PWM to buzzer or audiojack
const int EOGpin = A1; //anaolog pin for EOG


#if defined(__AVR_ATmega8__)
//
// On old ATmega8 boards.
//    Output is on pin 11
//
#define LED_PIN       13
#define LED_PORT      PORTB
#define LED_BIT       5
#define PWM_PIN       11
#define PWM_VALUE     OCR2
#define PWM_INTERRUPT TIMER2_OVF_vect
#elif defined(__AVR_ATmega1280__)
//
// On the Arduino Mega
//    Output is on pin 3
//
#define LED_PIN       13
#define LED_PORT      PORTB
#define LED_BIT       7
#define PWM_PIN       3
#define PWM_VALUE     OCR3C
#define PWM_INTERRUPT TIMER3_OVF_vect
#else
//
// For modern ATmega168 and ATmega328 boards
//    Output is on pin 3
//
#define PWM_PIN       3
#define PWM_VALUE     OCR2B
#define LED_PIN       13
#define LED_PORT      PORTB
#define LED_BIT       5
#define PWM_INTERRUPT TIMER2_OVF_vect
#endif

// Stepped Pentatonic mapping
// 
/*
uint16_t pentatonicTable[54] = {
  0,19,22,26,29,32,38,43,51,58,65,77,86,103,115,129,154,173,206,231,259,308,346,
  411,461,518,616,691,822,923,1036,1232,1383,1644,1845,2071,2463,2765,3288,
  3691,4143,4927,5530,6577,7382,8286,9854,11060,13153,14764,16572,19708,22121,26306
};*/

//3x octave
uint16_t pentatonicTable[54] = {
  0, 57, 66, 78, 87, 96, 114, 129, 153, 174, 195, 231, 258, 309, 345, 387, 462, 519, 615, 693, 
  777, 924, 1038, 1232, 1384, 1554, 1848, 2071, 2485, 2765, 3288, 3691, 4143, 4927, 5530, 6577, 
  7382, 8286, 9854, 11060, 13153, 14764, 16572, 19708, 22121, 26306, 31612, 36918
};

uint16_t mapPentatonic(uint16_t input) {
  uint8_t value = (1023-input) / (1024/26);
  //uint8_t value = (1023-input) / (1024/53);
  //uint8_t value = log(1023 - input + 1) / log(1023); // Using log for sensitivity tuning
  //uint8_t value = sqrt(1023 - input);  // Adjust with sqrt or pow for non-linear sensitivity
  return (pentatonicTable[value]);
}

void audioOn() {
#if defined(__AVR_ATmega8__)
  // ATmega8 has different registers
  TCCR2 = _BV(WGM20) | _BV(COM21) | _BV(CS20);
  TIMSK = _BV(TOIE2);
#elif defined(__AVR_ATmega1280__)
  TCCR3A = _BV(COM3C1) | _BV(WGM30);
  TCCR3B = _BV(CS30);
  TIMSK3 = _BV(TOIE3);
#else
  // Set up PWM to 31.25kHz, phase accurate
  TCCR2A = _BV(COM2B1) | _BV(WGM20);
  TCCR2B = _BV(CS20);
  TIMSK2 = _BV(TOIE2);
#endif
}


void setup() {
  // put your setup code here, to run once:
  pinMode(buttonPin, INPUT_PULLUP);
  Serial.begin(115200);
  pinMode(EOGpin, INPUT);
  pinMode(PWM_PIN, OUTPUT);

  audioOn();

}

void loop() {
  int EOG_Reading = analogRead(EOGpin);
  audioEnabled = (digitalRead(buttonPin) == LOW); // Assuming LOW when button pressed
  // put your main code here, to run repeatedly:
  if(audioEnabled){
    //Serial.println("Button pressed: Audio ON");
    Serial.println(EOG_Reading);
    uint16_t rawValue = analogRead(EOGpin);//

    syncPhaseInc = mapPentatonic(rawValue);
    Serial.print("Raw: ");
    Serial.print(rawValue);
    Serial.print(" -> Mapped: ");
    Serial.println(syncPhaseInc);
  }else{
    syncPhaseInc = 0;
    //Serial.println("Button released: Audio OFF");

  }
  delay(10);


}

SIGNAL(TIMER2_OVF_vect){
  if(audioEnabled){
    uint8_t value;
    uint16_t output;
    syncPhaseAcc+=syncPhaseInc;
    value = (syncPhaseAcc >> 7) & 0xff; //convert to triangle
    if (syncPhaseAcc & 0x8000) value = ~value; // Invert value for triangle wave

    // Scale output to the available range, clipping if necessary
    output = value;
    if (output > 255) output = 255;
    PWM_VALUE = output;
  }else{
    PWM_VALUE = 0;
  }
}
