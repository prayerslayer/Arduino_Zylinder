#include "math.h"

/*
  TODO: Methode um mehere LED Gruppe einer Prozentzahl zuzuordnen
  TODO: LED Farbe mit Potentiometer steuern
*/


// PINS
const int RED_PIN = 9;
const int GREEN_PIN = 10;
const int BLUE_PIN = 11;

// CONSTANTS
const int LED_COUNT = 2;
const int DEFAULT_DURATION = 200; // 200 ms
const int DEFAULT_STEP_TIME = 10; // 10 ms

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
  setColor( COLOR_NOPE );
  setLedBrightness( 0, 1 );
  setLedBrightness( 1, 1 );
}

void fadeToColor( int color[ 3 ] ) {
   fadeToColor( color, DEFAULT_DURATION, DEFAULT_STEP_TIME );
}

void fadeToColor( int color[ 3 ], int duration ) {
  fadeToColor( color, duration, DEFAULT_STEP_TIME );
}


//TODO remove time constraint
void fadeToColor( int color[ 3 ], int duration, int steptime ) {
  int stepcount = round( duration / steptime );
  int diffRed = color[ 0 ] - currentColor[ 0 ];   //possibly negative!
  int diffGreen = color[ 1 ] - currentColor[ 1 ]; //possibly negative!
  int diffBlue = color[ 2 ] - currentColor[ 2 ];  //possibly negative!
  int stepRed = round( diffRed / stepcount );
  int stepGreen = round( diffGreen / stepcount );
  int stepBlue = round( diffBlue / stepcount );
  int newRed = 0;
  int newGreen = 0;
  int newBlue = 0;

  int tempRed = 0;
  int tempGreen = 0;
  int tempBlue = 0;
  
  boolean fadeRed = diffRed != 0;
  boolean fadeGreen = diffGreen != 0;
  boolean fadeBlue = diffBlue != 0;

  boolean canFadeRed = false;
  boolean canFadeGreen = false;
  boolean canFadeBlue = false;
  
  for( int i = 1; i <= stepcount; i++ ) {
    Serial.print( "=======");
    Serial.print( i );
    Serial.println( "======");
    tempRed = currentColor[ 0 ] + stepRed;
    canFadeRed = i < stepcount & ( diffRed < 0 ? tempRed > color[ 0 ] : tempRed < color[ 0 ] ) & ( tempRed >= 0 & tempRed < 256 );
    Serial.print( "Can fade red? ");
    Serial.println( canFadeRed );
    if ( canFadeRed && fadeRed ) {
      newRed = tempRed;
      setColorRed( newRed );
      Serial.print( "R " );
      Serial.println( newRed );
    } else if ( fadeRed) {
      newRed = color[ 0 ];
      setColorRed( newRed );
      Serial.print( "R " );
      Serial.println( newRed );
      fadeRed = false;
    }
    tempGreen = currentColor[ 1 ] + stepGreen;
    canFadeGreen = i < stepcount & ( diffGreen < 0 ? tempGreen > color[ 1 ] : tempGreen < color[ 1 ] ) & ( tempGreen >= 0 & tempGreen < 256 );
    Serial.print( "Can fade green? ");
    Serial.println( canFadeGreen );
    if ( canFadeGreen && fadeGreen ) {
      newGreen = tempGreen;
      setColorGreen( newGreen );
      Serial.print( "G ");
      Serial.println( newGreen );
    } else if ( fadeGreen ) {
      newGreen = color[ 1 ];
      setColorGreen( newGreen );
      Serial.print( "G ");
      Serial.println( newGreen );
      fadeGreen = false;
    }
    tempBlue = currentColor[ 2 ] + stepBlue;
    canFadeBlue = i < stepcount &  ( diffBlue < 0 ? tempBlue > color[ 2 ] : tempBlue < color [ 2 ] ) & ( tempBlue >= 0 & tempBlue < 256 );
    Serial.print( "Can fade blue? ");
    Serial.println( canFadeBlue );
    if( canFadeBlue && fadeBlue ) {
      newBlue = tempBlue;
      setColorBlue( newBlue );
      Serial.print( "B " );
      Serial.println( newBlue );
    } else if ( fadeBlue ) {
      newBlue = color[ 2 ];
      setColorBlue( newBlue );
      Serial.print( "B " );
      Serial.println( newBlue );
      fadeBlue = false;
    }
    delay( steptime );
  }
}


void loop() 
{
  setLedBrightness( 0, random( 2 ) );
  setLedBrightness( 1, random( 2 ) );
  fadeToColor( COLOR_BLUE, 500 );
  delay( 2000 );
  fadeToColor( COLOR_GREEN, 500 );
  delay( 2000 );
  fadeToColor( COLOR_NOPE, 500 );
  delay( 2000 );
}

