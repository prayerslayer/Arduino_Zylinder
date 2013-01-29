#include "math.h"

// PINS
const int RED_PIN = 9;    // which pin has cable for red?
const int GREEN_PIN = 10; // which pin has cable for green?
const int BLUE_PIN = 11;  // which pin has cable for blue?

// CONSTANTS
const int LED_COUNT = 2;              // how many leds are there?
const int DEFAULT_COLOR_DELAY = 10;   // 10 ms
const int DEFAULT_PCNTG_DELAY = 50;   // 50 ms
const int DEFAULT_COLOR_STEP = 5;     // 5 gray values
const int DEFAULT_PCNTG_STEP = 51;    // 20 % brightness
const int LED_MAXVALUE = 255;
const int LED_MINVALUE = 0;

// GLOBAL VARS
int ledIntensity[ LED_COUNT ];  // contains brightness of each led
int ledPin[ LED_COUNT ];        // contains pin for each led

// color codes
int COLOR_NOPE[3] = { 0, 0, 0 };
int COLOR_GREEN[3] = { 0, 255, 0 };
int COLOR_RED[3] = { 255, 0, 0 };
int COLOR_BLUE[3] = { 0, 0, 255 };
int currentColor[3] = { 0, 0, 0 };

//Sets brightness of LED
void setLedBrightness( int led, int intensity ) {
  ledIntensity[ led ] = intensity;
  analogWrite( ledPin[ led ], intensity );
}

// Turns led group on or off
void powerLed( int led, boolean on ) {
  if ( on ) {
    setLedBrightness( led, LED_MAXVALUE );
  } else {
    setLedBrightness( led, LED_MINVALUE );
  }
}

// Sets green value of displayed color
void setColorGreen( int green ) {
  currentColor[ 1 ] = green;
  analogWrite( GREEN_PIN, 255 - green );
}

// Sets red value of displayed color
void setColorRed( int red ) {
  currentColor[ 0 ] = red;
  analogWrite( RED_PIN, 255 - red );
}

// Sets blue value of displayed color
void setColorBlue( int blue ) {
  currentColor[ 2 ] = blue;
  analogWrite( BLUE_PIN, 255 - blue );
}

// Convenience method
void setColor( int red, int green, int blue ) {
  setColorRed( red );
  setColorGreen( green );
  setColorBlue( blue );
}

//Sets color for all LEDs
void setColor( int rgb[ 3 ] ) {
  for ( int i = 0; i < 3; i++) {
    currentColor[ i ] = rgb[ i ];
  } 
  analogWrite( RED_PIN, LED_MAXVALUE - rgb[ 0 ] );
  analogWrite( GREEN_PIN, LED_MAXVALUE - rgb[ 1 ] );
  analogWrite( BLUE_PIN, LED_MAXVALUE - rgb[ 2 ] );
}

// init stuff
void setup()  { 
  // serial monitor
  Serial.begin( 9600 );
  // led pins
  ledPin[ 0 ] = 5;
  ledPin[ 1 ] = 6;
  // turn leds on
  setColor( COLOR_BLUE );
  powerLed( 0, HIGH );
  powerLed( 1, HIGH );
}

// Fades a LED brightness to target value
void fadeLed( int led, int targetval ) {
  int currentval = ledIntensity[ led ];
  int stepp = currentval < targetval ? DEFAULT_PCNTG_STEP : DEFAULT_PCNTG_STEP * (-1);
  while ( currentval != targetval ) {
    currentval += stepp;
    setLedBrightness( led, currentval );
    delay( DEFAULT_PCNTG_DELAY );
  }
}

// Counts enlighted ( brightness > 0 ) LEDs
int getActiveLeds() {
  int activeLeds = 0;
  for ( int i = LED_COUNT - 1; i >= 0; i-- ) {
    activeLeds += ledIntensity[ i ] > 0 ? 1 : 0;
  }
  return activeLeds;
}

// Turns percentage of LEDs on/off and fades in between
void fadeToPercentage( float percentage ) {
  int necessaryLeds = round( percentage * LED_COUNT );
  int activeLeds = getActiveLeds();
  if ( activeLeds == necessaryLeds )  // no need to do anything then
    return;
  int increment = necessaryLeds > activeLeds ? 1 : -1;  // order is important!
  int target = increment > 0 ? LED_MAXVALUE : LED_MINVALUE; // target brightness of LEDs
  int indexmodifier = increment > 0 ? 0 : -1; // if we start at LED 2 (index 1) and want to go up, start index is 1, else it's 0.
  do {
    fadeLed( activeLeds + indexmodifier, target );
    activeLeds += increment;
  } while ( necessaryLeds != activeLeds );
}

// Fades LEDs to given color
void fadeToColor( int color[ 3 ] ) {
   fadeToColor( color, DEFAULT_COLOR_DELAY );
}

// Fades LEDs to given color
void fadeToColor( int color[ 3 ], int stepdelay ) {
  int diffRed = color[ 0 ] - currentColor[ 0 ];   //possibly negative!
  int diffGreen = color[ 1 ] - currentColor[ 1 ]; //possibly negative!
  int diffBlue = color[ 2 ] - currentColor[ 2 ];  //possibly negative!
  
  // different steps, because we can fade red up and blue down at the same time
  int stepRed = diffRed < 0 ? DEFAULT_COLOR_STEP * (-1) : DEFAULT_COLOR_STEP;
  int stepGreen = diffGreen < 0 ? DEFAULT_COLOR_STEP * (-1) : DEFAULT_COLOR_STEP;
  int stepBlue = diffBlue < 0 ? DEFAULT_COLOR_STEP * (-1) : DEFAULT_COLOR_STEP;

  // temporary values
  int tempRed = 0;
  int tempGreen = 0;
  int tempBlue = 0;
  
  // is there a reason to fade colors?
  boolean fadeRed = diffRed != 0;
  boolean fadeGreen = diffGreen != 0;
  boolean fadeBlue = diffBlue != 0;

  // while there is a reason to fade, fade
  while ( fadeRed || fadeGreen || fadeBlue ) {
    // fade red
    if ( fadeRed ) {
      tempRed = currentColor[ 0 ] + stepRed;
      setColorRed( tempRed );
      fadeRed = diffRed < 0 ? tempRed > color[ 0 ] : tempRed < color[ 0 ]; // means basically tempRed != target value
    }

    // fade green
    if ( fadeGreen ) {
      tempGreen = currentColor[ 1 ] + stepGreen;
      setColorGreen( tempGreen );
      fadeGreen = diffGreen < 0 ? tempGreen > color[ 1 ] : tempGreen < color[ 1 ];
    }

    // fade blue
    if ( fadeBlue ) {
      tempBlue = currentColor[ 2 ] + stepBlue;
      setColorBlue( tempBlue );
      fadeBlue = diffBlue < 0 ? tempBlue > color[ 2 ] : tempBlue < color[ 2 ];
    }
    delay( stepdelay );
  }
}

// Reads voltage from A0 and returns it
float getVoltage() {
  float volt = analogRead( A0 ) * ( 5.0 / 1023.0 );
  return volt;
}

// Converts voltage to color code and writes it in color
void setColorFromVoltage( float voltage, int *color ) {
  Serial.println( voltage );
  int volt = (int) ( voltage * 50 );  // == 250 values ~ 2^8
  int colorCode = floor( volt/32 ); // 2^8 / 2^5 = 8 different color codes
  
  if ( colorCode == 0 ) {
    // no color
    Serial.println( "BLACK" );
    color[ 0 ] = 0;
    color[ 1 ] = 0;
    color[ 2 ] = 0;
  }
  if ( colorCode == 1 ) {
    //red
    Serial.println( "RED" );
    color[ 0 ] = 255;
    color[ 1 ] = 0;
    color[ 2 ] = 0;
  } else if ( colorCode == 2 ) {
    //green
    Serial.println( "GREEN" );
    color[ 0 ] = 0;
    color[ 1 ] = 255;
    color[ 2 ] = 0;
  } else if ( colorCode == 3 ) {
    //blue
    Serial.println( "BLUE" );
    color[ 0 ] = 0;
    color[ 1 ] = 0;
    color[ 2 ] = 255;
  } else if ( colorCode == 4 ) {
    // red + green
    Serial.println( "LIME" );
    color[ 0 ] = 255;
    color[ 1 ] = 255;
    color[ 2 ] = 0;
  } else if ( colorCode == 5 ) {
    //red + blue
    Serial.println( "PURPLE" );
    color[ 0 ] = 255;
    color[ 1 ] = 0;
    color[ 2 ] = 255;
  } else if ( colorCode == 6 ) {
    // blue + green
    Serial.println( "CYAN" );
    color[ 0 ] = 0;
    color[ 1 ] = 255;
    color[ 2 ] = 255;
  } else if ( colorCode == 7 ) {
    // red + green + blue
    Serial.println( "WHITE" );
    color[ 0 ] = 255;
    color[ 1 ] = 255;
    color[ 2 ] = 255;
  }
}

void loop() {
  int color[3];
  setColorFromVoltage( getVoltage(), &color[0] ) ;
  fadeToPercentage( getVoltage() / 5 );
  fadeToColor( color );
}