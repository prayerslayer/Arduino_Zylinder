#include "math.h"

// PINS
const int RED_PIN = 9;
const int GREEN_PIN = 10;
const int BLUE_PIN = 11;

// CONSTANTS
const int LED_COUNT = 2;
const int DEFAULT_STEP_DELAY = 10;  // 10 ms
const int DEFAULT_STEP = 5;         // 5 gray values

// GLOBAL VARS
boolean ledState[ LED_COUNT ];
int ledIntensity[ LED_COUNT ];
int ledPin[ LED_COUNT ];

// colors
int COLOR_NOPE[3] = { 0, 0, 0 };
int COLOR_GREEN[3] = { 0, 255, 0 };
int COLOR_RED[3] = { 255, 0, 0 };
int COLOR_BLUE[3] = { 0, 0, 255 };
int currentColor[3] = { 0, 0, 0 };

//Sets brightness of LED
void setLedBrightness( int led, float intensity ) {
  int rounded = floor( intensity * 255 );
  ledIntensity[ led ] = rounded;
  analogWrite( ledPin[ led ], rounded );
}

// Turns led group on or off
void powerLed( int led, boolean on ) {
  if ( on ) {
    setLedBrightness( led, 1.0 );
  } else {
    setLedBrightness( led, 0.0 );
  }
}

void setColorGreen( int green ) {
  currentColor[ 1 ] = green;
  analogWrite( GREEN_PIN, 255 - green );
}

void setColorRed( int red ) {
  currentColor[ 0 ] = red;
  analogWrite( RED_PIN, 255 - red );
}

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
  analogWrite( RED_PIN, 255 - rgb[ 0 ] );
  analogWrite( GREEN_PIN, 255 - rgb[ 1 ] );
  analogWrite( BLUE_PIN, 255 - rgb[ 2 ] );
}

void setup()  { 
  Serial.begin( 9600 );
  //init LEDs
  ledPin[ 0 ] = 5;
  ledPin[ 1 ] = 6;
  setColor( COLOR_BLUE );
  powerLed( 0, HIGH );
  powerLed( 1, HIGH );
}


// Fades LEDs to given color
void fadeToColor( int color[ 3 ] ) {
   fadeToColor( color, DEFAULT_STEP_DELAY );
}

// Fades LEDs to given color
void fadeToColor( int color[ 3 ], int stepdelay ) {
  int diffRed = color[ 0 ] - currentColor[ 0 ];   //possibly negative!
  int diffGreen = color[ 1 ] - currentColor[ 1 ]; //possibly negative!
  int diffBlue = color[ 2 ] - currentColor[ 2 ];  //possibly negative!
  
  int stepRed = diffRed < 0 ? DEFAULT_STEP * (-1) : DEFAULT_STEP;
  int stepGreen = diffGreen < 0 ? DEFAULT_STEP * (-1) : DEFAULT_STEP;
  int stepBlue = diffBlue < 0 ? DEFAULT_STEP * (-1) : DEFAULT_STEP;

  int tempRed = 0;
  int tempGreen = 0;
  int tempBlue = 0;
  
  boolean fadeRed = diffRed != 0;
  boolean fadeGreen = diffGreen != 0;
  boolean fadeBlue = diffBlue != 0;

  while ( fadeRed || fadeGreen || fadeBlue ) {
    // fade red
    if ( fadeRed ) {
      tempRed = currentColor[ 0 ] + stepRed;
      setColorRed( tempRed );
      fadeRed = diffRed < 0 ? tempRed > color[ 0 ] : tempRed < color[ 0 ]; // tempRed != target value
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
  int code = floor( volt/32 ); // 8 different codes --> colors
  
  if ( code == 0 ) {
    // no color
    Serial.println( "BLACK" );
    color[ 0 ] = 0;
    color[ 1 ] = 0;
    color[ 2 ] = 0;
  }
  if ( code == 1 ) {
    //red
    Serial.println( "RED" );
    color[ 0 ] = 255;
    color[ 1 ] = 0;
    color[ 2 ] = 0;
  } else if ( code == 2 ) {
    //green
    Serial.println( "GREEN" );
    color[ 0 ] = 0;
    color[ 1 ] = 255;
    color[ 2 ] = 0;
  } else if ( code == 3 ) {
    //blue
    Serial.println( "BLUE" );
    color[ 0 ] = 0;
    color[ 1 ] = 0;
    color[ 2 ] = 255;
  } else if ( code == 4 ) {
    // red + green
    Serial.println( "LIME" );
    color[ 0 ] = 255;
    color[ 1 ] = 255;
    color[ 2 ] = 0;
  } else if ( code == 5 ) {
    //red + blue
    Serial.println( "PURPLE" );
    color[ 0 ] = 255;
    color[ 1 ] = 0;
    color[ 2 ] = 255;
  } else if ( code == 6 ) {
    // blue + green
    Serial.println( "CYAN" );
    color[ 0 ] = 0;
    color[ 1 ] = 255;
    color[ 2 ] = 255;
  } else if ( code == 7 ) {
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

