/** 
 *  LED Strip Controller
 *  Built for Sunshine Hospital Radio. SMD5050 LED strips used in studio to 
 *  display the mic live and phone status.
 *  Author: Matt Ribbins (matt@mattyribbo.co.uk)
 *  Date: May 2018
 */

/** 
  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:
  1. Redistributions of source code must retain the above copyright notice, this
     list of conditions and the following disclaimer.
  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation
     and/or other materials provided with the distribution.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
  ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

/*** 
 *  User configuration values
 *  
 *  Set the pins used on the Arduino controller for the LED strip, and input relays.
 *  LED pins must be PWM capable pins. Pin 9-11 is recommended.
 */
#define LED_RED     11
#define LED_GREEN   10
#define LED_BLUE    9 
#define MIC_INPUT   7
#define TBU1_INPUT  5
#define TBU2_INPUT  6

#define LOOP_DELAY  20
#define FADE_SPEED  0.2
#define FLASH_SPEED 100
#define XFADE_HOLD  0
#define DEBOUNCE_UP_DELAY 100
#define DEBOUNCE_DOWN_DELAY 100
#define DEBOUNCE_HOLD_THRESHOLD 3000

/***
 * No more user editable values.
 */
 
// Uncomment if debug required. 
// COM port 9600bps
//#define DEBUG_MODE 
//#define DEBUG_MODE_EXTENDED

// Standard Colours
#define YELLOW 255,180,0
#define RED 255,0,0
#define GREEN 0,255,0
#define BLUE 0,0,255
#define WHITE 255,255,255
#define PURPLE 153,0,153

// Number definitions
#define MODE_STATIC 0
#define MODE_FADE 1
#define MODE_FLASH 2

#define FLASH_FORWARD 0
#define FLASH_REVERSE 1

// LED status
long rgb[3]           = { 0, 0, 0 }; // Current 'live; value
long prevRgb[3]       = { 0, 0, 0 }; // Previous colour
long newRgb[3]        = { 0, 0, 0 }; // Future colour (what we 'set')
long flashStartRgb[3] = { 0, 0, 0 }; // Flash start colour
long flashEndRgb[3]   = { 0, 0, 0 }; // Flash end colour
long bright[3] = { 255, 255, 255 }; // Brightness settings
int displayMode = MODE_STATIC;
long flashCounter = 0;
int flashDirection = 0;


// Relay status
bool micLive = LOW;
bool tbu1On = LOW;
bool tbu2On = LOW;
bool statusChange = false;
int tbu1DebounceTime = 0;

// Debug mode
#ifdef DEBUG_MODE
#define DEBUG_LOOP_COUNT 60
#endif

static int rgbpins[3] = { LED_RED, LED_GREEN, LED_BLUE }; // Used in loops


// Function Prototypes 
void setLedColour(int, int, int);
void changeLedColour(void);

/**
 * Controller setup
 */
void setup () {
  int k;

  // Set up pins
  // INPUT if standard input, i.e. relay
  // INPUT_PULLUP if open connector output from remote
  randomSeed(analogRead(4));
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(MIC_INPUT, INPUT_PULLUP); 
  pinMode(TBU1_INPUT, INPUT_PULLUP);
  pinMode(TBU2_INPUT, INPUT_PULLUP);

  // Initialise outputs
  for (k=0; k<3; k++) {
    analogWrite(rgbpins[k], rgb[k] * bright[k]/256);
  }
  setLedColour(YELLOW);

  tbu1DebounceTime = millis();

  #ifdef DEBUG_MODE
  Serial.begin(9600);
  #endif
}

/** 
 * The Loop (TM)
 */
void loop() {  
  // Check for mic live
  micLive = !digitalRead(MIC_INPUT);

  // Check TBU with Comrex Mic Light
  // Uses Pin 14 Hybrid 1 On Indication ** - NC connection.
  // Pulse when incoming call
  // High when call connected.
  tbu2On = !digitalRead(TBU2_INPUT);
  tbu2On = LOW;
  
  // Debouncer for TBU1 input
  if(tbu1On != (!digitalRead(TBU1_INPUT))) {
    if(tbu1On) {
      if ( (millis() - tbu1DebounceTime) > DEBOUNCE_UP_DELAY) {
        tbu1On = !digitalRead(TBU1_INPUT);
        tbu1DebounceTime = millis(); 
      }
    } else {
      if ( (millis() - tbu1DebounceTime) > DEBOUNCE_DOWN_DELAY) {
        tbu1On = !digitalRead(TBU1_INPUT);
        tbu1DebounceTime = millis(); 
      }
    }
  }

  // Lights logic starts here

  // Do we have a TBU call incoming?
  if((tbu1On == HIGH) || (tbu2On == HIGH)) {
    if(micLive == HIGH) {
      // Flash RED to PURPLE
      if((millis() - tbu1DebounceTime) > DEBOUNCE_HOLD_THRESHOLD) {
        setLedColour(RED);
        setLedMode(MODE_FLASH);
      } else {
        setLedColour(PURPLE);
        setLedMode(MODE_STATIC);
      }
    } else {
      // Flash YELLOW to PURPLE
      setLedColour(PURPLE);
      changeFlashEndColour(YELLOW);
      setLedMode(MODE_STATIC);
    }
  } else {
    if(micLive == HIGH) {
      // Static RED
      setLedColour(RED);
      setLedMode(MODE_FADE);
    } else {
      // Static YELLOW
      setLedColour(YELLOW);
      setLedMode(MODE_FADE);
    }
  }

  if(statusChange) {
    changeLedColour();
  }
  if(displayMode == MODE_FLASH) {
    flashLoop();
  }
  delay(LOOP_DELAY);
}

/***
 * Set the LED colour that we will change to.
 */
void setLedColour(int red, int green, int blue) { 
  if((red != rgb[0]) || (green != rgb[1]) || (blue != rgb[2])) { 
    newRgb[0] = red;
    newRgb[1] = green;
    newRgb[2] = blue;
    statusChange = true;
    
    #ifdef DEBUG_MODE
    Serial.print("SetLedColour | ");
    Serial.print(rgb[0]);
    Serial.print(" / ");
    Serial.print(rgb[1]);
    Serial.print(" / ");  
    Serial.println(rgb[2]); 
    #endif
  }
}

/**
 * Set the LED mode we will change to. Perform any tasks associated.
 */
void setLedMode(int mode) {
  if(displayMode == mode) return;
  
  #ifdef DEBUG
  Serial.print("SetLedMode | ");
  Serial.println(mode); 
  #endif
  
  switch(mode) {
    case MODE_STATIC:
      displayMode = MODE_STATIC;
      break;
    case MODE_FADE:
      displayMode = MODE_FADE;
      break;
    case MODE_FLASH:
      displayMode = MODE_FLASH;
      break;
    default:
      displayMode = MODE_STATIC; 
      break;
  }

  #ifdef DEBUG

  if (mode == MODE_FLASH) {
    Serial.print("SetLedMode | Flash Start: ");
    Serial.print(flashStartRgb[0]);
    Serial.print(" / ");
    Serial.print(flashStartRgb[1]);
    Serial.print(" / ");  
    Serial.print(flashStartRgb[2]); 
    Serial.print(" | End: ");
    Serial.print(flashEndRgb[0]);
    Serial.print(" / ");
    Serial.print(flashEndRgb[1]);
    Serial.print(" / ");  
    Serial.println(flashEndRgb[2]); 
  }
  #endif
}

/**
 * Make the change to the LED colour.
 */
void changeLedColour() {
  switch(displayMode) {
    case MODE_STATIC:
      changeImmediateLedColour();
      break;
    case MODE_FADE:
      fadeLedColour();
      break;
    case MODE_FLASH:
      startFlashLoop();
      break;
    default:
      break;
  }
  statusChange = false;
  
  #ifdef DEBUG
  Serial.print("changeLedColour | ");
  Serial.println(displayMode); 
  if (mode == MODE_FLASH) {

  }
  #endif
}

/**
 * Immediately change the LED colour
 */
void changeImmediateLedColour() {
  analogWrite(LED_RED, newRgb[0]);
  analogWrite(LED_GREEN, newRgb[1]);
  analogWrite(LED_BLUE, newRgb[2]);
  rgb[0] = newRgb[0];
  rgb[1] = newRgb[1];
  rgb[2] = newRgb[2];

  #ifdef DEBUG_MODE
  Serial.print("changeImmediateLedColour | ");
  Serial.print(rgb[0]);
  Serial.print(" / ");
  Serial.print(rgb[1]);
  Serial.print(" / ");  
  Serial.println(rgb[2]); 
  #endif
}

void startFlashLoop() {
  flashStartRgb[0] = newRgb[0]; 
  flashStartRgb[1] = newRgb[1]; 
  flashStartRgb[2] = newRgb[2];
  flashEndRgb[0] = rgb[0]; 
  flashEndRgb[1] = rgb[1]; 
  flashEndRgb[2] = rgb[2];
  statusChange = false;
  #ifdef DEBUG
  Serial.print("startFlashLoop | Flash Start: ");
  Serial.print(flashStartRgb[0]);
  Serial.print(" / ");
  Serial.print(flashStartRgb[1]);
  Serial.print(" / ");  
  Serial.print(flashStartRgb[2]); 
  Serial.print(" | End: ");
  Serial.print(flashStartRgb[0]);
  Serial.print(" / ");
  Serial.print(flashStartRgb[1]);
  Serial.print(" / ");  
  Serial.println(flashStartRgb[2]); 
  #endif
}

void changeFlashEndColour(int red, int green, int blue) {
  if((red != flashEndRgb[0]) || (green != flashEndRgb[1]) || (blue != flashEndRgb[2])) { 
    rgb[0] = red;
    rgb[1] = green;
    rgb[2] = blue;
    statusChange = true;
    
    #ifdef DEBUG_MODE
    Serial.print("changeFlashStartColour | ");
    Serial.print(rgb[0]);
    Serial.print(" / ");
    Serial.print(rgb[1]);
    Serial.print(" / ");  
    Serial.println(rgb[2]); 
    #endif
  }
}
/***
 * Loop for flash mode.
 */
void flashLoop() {
  // Do we flash old -> new or new -> old?
  if(flashCounter == 0) {
    newRgb[0] = flashStartRgb[0];
    newRgb[1] = flashStartRgb[1];
    newRgb[2] = flashStartRgb[2];
    #ifdef DEBUG_MODE_EXTENDED
    Serial.print("flashLoop | ");
    Serial.print(newRgb[0]);
    Serial.print(" / ");
    Serial.print(newRgb[1]);
    Serial.print(" / ");  
    Serial.println(newRgb[2]); 
    #endif
    changeImmediateLedColour();
    flashDirection = FLASH_FORWARD;
  }
  if(flashCounter >= FLASH_SPEED) {
    newRgb[0] = flashEndRgb[0];
    newRgb[1] = flashEndRgb[1];
    newRgb[2] = flashEndRgb[2];
    #ifdef DEBUG_MODE_EXTENDED
    Serial.print("flashLoop | ");
    Serial.print(newRgb[0]);
    Serial.print(" / ");
    Serial.print(newRgb[1]);
    Serial.print(" / ");  
    Serial.println(newRgb[2]); 
    #endif
    changeImmediateLedColour();
    flashDirection = FLASH_REVERSE;
  }

  if(flashDirection == 0) {
    flashCounter++;
  } else {
    flashCounter--;
  }
  #ifdef DEBUG_MODE_EXTENDED
  Serial.print("flashLoop | ");
  Serial.print(flashDirection);
  Serial.print(" - ");
  Serial.println(flashCounter);
  #endif
}

void fadeLedColour() {
  #ifdef DEBUG_MODE
    Serial.print("fadeLedColour | ");
    Serial.print(newRgb[0]);
    Serial.print(" / ");
    Serial.print(newRgb[1]);
    Serial.print(" / ");  
    Serial.println(newRgb[2]); 
    #endif
  crossFade(newRgb);   
}


/**
 * Colour Crossfade - Calculate change steps
 * Reference: https://www.arduino.cc/en/Tutorial/ColorCrossfader
 */
int calculateStep(int prevValue, int endValue) {
  int step = endValue - prevValue; // What's the overall gap?
  if (step) {                      // If its non-zero, 
    step = 1020/step;              //   divide by 1020
  } 
  return step;
}

/**
 * Colour Crossfade - Calculate values
 * Reference: https://www.arduino.cc/en/Tutorial/ColorCrossfader
 */
int calculateVal(int step, int val, int i) {

  if ((step) && i % step == 0) { // If step is non-zero and its time to change a value,
    if (step > 0) {              //   increment the value if step is positive...
      val += 1;           
    } 
    else if (step < 0) {         //   ...or decrement it if step is negative
      val -= 1;
    } 
  }
  // Defensive driving: make sure val stays in the range 0-255
  if (val > 255) {
    val = 255;
  } 
  else if (val < 0) {
    val = 0;
  }
  return val;
}

/** 
 * Colour crossfade
 * Reference: https://www.arduino.cc/en/Tutorial/ColorCrossfader 
 */
void crossFade(long color[3]) {
  int stepR = calculateStep(prevRgb[0], color[0]);
  int stepG = calculateStep(prevRgb[1], color[1]); 
  int stepB = calculateStep(prevRgb[2], color[2]);

  for (int i = 0; i <= 960; i++) {
    rgb[0] = calculateVal(stepR, rgb[0], i);
    rgb[1] = calculateVal(stepG, rgb[1], i);
    rgb[2] = calculateVal(stepB, rgb[2], i);

    analogWrite(LED_RED, rgb[0]);   // Write current values to LED pins
    analogWrite(LED_GREEN, rgb[1]);      
    analogWrite(LED_BLUE, rgb[2]); 

    delay(FADE_SPEED); // Pause for 'wait' milliseconds before resuming the loop

    #ifdef DEBUG_MODE_EXTENDED
    
    if (i == 0 or i % DEBUG_LOOP_COUNT == 0) { // beginning, and every loopCount times
      Serial.print("Loop/RGB: #");
      Serial.print(i);
      Serial.print(" | ");
      Serial.print(rgb[0]);
      Serial.print(" / ");
      Serial.print(rgb[1]);
      Serial.print(" / ");  
      Serial.println(rgb[2]); 
    } 
    #endif
  }
  // Update current values for next loop
  prevRgb[0] = rgb[0]; 
  prevRgb[1] = rgb[1]; 
  prevRgb[2] = rgb[2];
  delay(XFADE_HOLD); // Pause for optional 'wait' milliseconds before resuming the loop
}


