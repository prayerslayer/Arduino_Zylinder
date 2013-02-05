#include "math.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// PINS
const int BRIDGE_PIN = 7; // which pin has cable for reed sensor?
const int RED_PIN = 9;    // which pin has cable for red?
const int GREEN_PIN = 10; // which pin has cable for green?
const int BLUE_PIN = 11;  // which pin has cable for blue?

/*
  OTHER USED PINS:
    DIGITAL 2:  Gyro Interrupt
    ANALOG 4, ANALOG 5: Gyro I2C
*/


/*
  TODO: berechnung von stehend/liegend mit baseAcceleration
  TODO: berechnung von weg durch änderung in accelerometer
*/

// CONSTANTS
const int LED_COUNT = 2;              // how many leds are there?
const int DEFAULT_COLOR_DELAY = 10;   // 10 ms
const int DEFAULT_PCNTG_DELAY = 50;  // 50 ms
const int DEFAULT_COLOR_STEP = 5;     // 5 gray values, 51 steps, 500 ms default duration
const int DEFAULT_PCNTG_STEP = 51;    // 20 % brightness, 5 steps, 250 ms default duration
const int LED_MAXVALUE = 255;
const int LED_MINVALUE = 0;
const int ACCEL_SENSITIVITY_SCALE = 8192;

// CHIP SPECIFIC STUFF
const int CHIP_PERSON = 5;    // color and identifier for persona chip
const int CHIP_FUEL = 4;      // color and identifier for fuel chip
const int CHIP_TRUNK = 7;     // color and identifier for trunk chip
const float RADIUS = 4.0;    // radius of base station and chips - entspricht dem deckel eines kleinen nutellaglases
// fuel chip
float fuelFill = 0.6;         // amount of tankfüllung
int fuelScale = 2;            // how many km equal 1 cm rolling?
int fuelRange = fuelFill * 35/4.5 * 100;  // based on toyota aigo
// trunk chip
float trunkMetering = 0;
int trunkCapacity = 140;      // liters
int trunkScale = 5;

// GLOBAL VARS
int ledIntensity[ LED_COUNT ];  // contains brightness of each led
int ledPin[ LED_COUNT ];        // contains pin for each led
int defaultChip = CHIP_PERSON;  // which color to show when there is no chip on base station
int lastChip = CHIP_PERSON;     // to check whether the chip was changed or not
boolean standing = true;        // if base station is standing for sure
boolean lying = false;          // if base station is lying for sure
float cumulativeRevolutions = 0.0;  // how many revolutions since last standing
boolean initialized = false;
boolean fifoOverflow = false;  // if there was a FIFO overflow in last gyro processing
int octant = 0;  // current angle of base statoin
int lastOctant = 0;

// GYRO SENSOR
MPU6050 mpu;            // gyro sensor
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements 
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
float euler[3];         // [psi, theta, phi]    Euler angle container
float acceleration[ 3 ];               // current acceleration values
float baseAcceleration[ 3 ];   // acceleration values at initialization

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

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

// Set Gyro ready
void dmpDataReady() {
  mpuInterrupt = true;
}

// Basic setup, actual initialization is done in initialize()
void setup()  { 
  // serial monitor
  Serial.begin( 9600 );
  
  // Gyro stuff
  Wire.begin();
  Serial.println( "Init I2C...");
  mpu.initialize();
  Serial.println( "Test gyro connection...");
  Serial.println( mpu.testConnection() ? "Successful." : "Failed.");
  Serial.println( "Init DMP...");
  devStatus = mpu.dmpInitialize();
  if ( devStatus == 0 ) {
    // success
    Serial.println( "Success.");
    mpu.setDMPEnabled( true );
    //enable interrupt detection
    attachInterrupt( 0, dmpDataReady, RISING );
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // failed
    Serial.println( "Failed.");
  }
  Serial.println( "Gyro initialization passed.");
}

void initialize() {

  // save base acceleration values
  baseAcceleration[ 0 ] = acceleration[ 0 ];
  baseAcceleration[ 1 ] = acceleration[ 1 ];
  baseAcceleration[ 2 ] = acceleration[ 2 ];

  // bridge pin
  pinMode( BRIDGE_PIN, INPUT );
  // led pins
  ledPin[ 0 ] = 5;
  ledPin[ 1 ] = 6;
  // turn leds on
  setColor( COLOR_BLUE );
  powerLed( 0, HIGH );
  powerLed( 1, HIGH );

  delay( 1000 );

  initialized = true;
}

// Check if there is a chip attached to base station
boolean isBridged( ) {
  boolean bridged = digitalRead( BRIDGE_PIN );
  //Serial.println( bridged );
  return bridged;
}

// Fades a LED brightness to target value
void fadeLed( int led, int targetval ) {
  int currentval = ledIntensity[ led ];
  int stepp = currentval < targetval ? DEFAULT_PCNTG_STEP : DEFAULT_PCNTG_STEP * (-1);
  while ( currentval != targetval ) {
    //Serial.println( "=== fading LED ===");
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
  //Serial.println( necessaryLeds );
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
  /*Serial.println( "-- start color");
  Serial.println( currentColor[0]);
  Serial.println( currentColor[1]);
  Serial.println( currentColor[2]);
  Serial.println( "-- target color");
  Serial.println( color[0]);
  Serial.println( color[1]);
  Serial.println( color[2]);*/
  // is there a reason to fade colors?
  boolean fadeRed = diffRed != 0;
  boolean fadeGreen = diffGreen != 0;
  boolean fadeBlue = diffBlue != 0;
  
  // check if there's a LED on or any difference in color
  if ( getActiveLeds() == 0 || ( !fadeRed && !fadeGreen && !fadeBlue ) ) {
    return; //break as soon as possible
  }
  
  // different steps, because we can fade red up and blue down at the same time
  int stepRed = diffRed < 0 ? DEFAULT_COLOR_STEP * (-1) : DEFAULT_COLOR_STEP;
  int stepGreen = diffGreen < 0 ? DEFAULT_COLOR_STEP * (-1) : DEFAULT_COLOR_STEP;
  int stepBlue = diffBlue < 0 ? DEFAULT_COLOR_STEP * (-1) : DEFAULT_COLOR_STEP;

  // temporary values
  int tempRed = 0;
  int tempGreen = 0;
  int tempBlue = 0;

  // while there is a reason to fade, fade
  while ( fadeRed || fadeGreen || fadeBlue ) {
    //Serial.println( "=== fading color ===");
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
  //Serial.print( volt );
  //Serial.println( " V");
  return volt;
}

// Reads voltage and transforms it into chip code
int getChipCode() {
  float volt = getVoltage();
  int chip = floor( ( (int) volt * 50 ) / 32 ); // 0-5 * 50 = 0-250 ~ 2^8 / 2^5 = 2^3 = 8 chips
  return min( chip, 7 ); // falls doch mal mehr als 5V reingehen sollten
}


// Converts chip code to percentage
float setPercentageFromChipCode( int chip ) {
  switch ( chip ) {
    case CHIP_PERSON: return 1.0;
    case CHIP_FUEL: return 0.4;
    default: return 0.0;
  }
}

// Converts voltage to color code and writes it in color
void setColorFromChipCode( int chip, int *color ) {
  
  if ( chip == 0 ) {
    // no color
    Serial.println( "BLACK" );
    color[ 0 ] = 0;
    color[ 1 ] = 0;
    color[ 2 ] = 0;
  }
  if ( chip == 1 ) {
    //red
    Serial.println( "RED" );
    color[ 0 ] = LED_MAXVALUE;
    color[ 1 ] = LED_MINVALUE;
    color[ 2 ] = LED_MINVALUE;
  } else if ( chip == 2 ) {
    //green
    Serial.println( "GREEN" );
    color[ 0 ] = LED_MINVALUE;
    color[ 1 ] = LED_MAXVALUE;
    color[ 2 ] = LED_MINVALUE;
  } else if ( chip == 3 ) {
    //blue
    Serial.println( "BLUE" );
    color[ 0 ] = LED_MINVALUE;
    color[ 1 ] = LED_MINVALUE;
    color[ 2 ] = LED_MAXVALUE;
  } else if ( chip == 4 ) {
    // red + green
    Serial.println( "LIME" );
    color[ 0 ] = LED_MAXVALUE;
    color[ 1 ] = LED_MAXVALUE;
    color[ 2 ] = LED_MINVALUE;
  } else if ( chip == 5 ) {
    //red + blue
    Serial.println( "PURPLE" );
    color[ 0 ] = LED_MAXVALUE;
    color[ 1 ] = LED_MINVALUE;
    color[ 2 ] = LED_MAXVALUE;
  } else if ( chip == 6 ) {
    // blue + green
    Serial.println( "CYAN" );
    color[ 0 ] = LED_MINVALUE;
    color[ 1 ] = LED_MAXVALUE;
    color[ 2 ] = LED_MAXVALUE;
  } else {
    // red + green + blue
    Serial.println( "WHITE" );
    color[ 0 ] = LED_MAXVALUE;
    color[ 1 ] = LED_MAXVALUE;
    color[ 2 ] = LED_MAXVALUE;
  }
  
}

void processGyro() {
  // break if there is no gyro
  if ( !dmpReady )
    return;
  
  // wait for data
  while (!mpuInterrupt && fifoCount < packetSize) {
        
  }
    
  // reset flags and get fifo count
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  
  // if there was FIFO overflow before, return because values are probably shit anyway
  if ( fifoOverflow ) {
    fifoOverflow = false;
    return;
  }
  
  // check for overflow
  if ( ( mpuIntStatus & 0x10 ) || fifoCount == 1024 ) {
    fifoOverflow = true;
    mpu.resetFIFO();
    Serial.println( "FIFO overflow");
  // else check for DMP data ready
  } else if ( mpuIntStatus & 0x02 ) {
    // wait for correct data length
    while ( fifoCount < packetSize )
      fifoCount = mpu.getFIFOCount();
      
    // read a packet
    mpu.getFIFOBytes( fifoBuffer, packetSize );
    
    fifoCount -= packetSize;

    // output as ypr
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetEuler( euler, &q );
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld( &aaWorld, &aaReal, &q );
    // set values in degrees 
    ypr[ 0 ] = ypr[0] * 180/M_PI;
    ypr[ 1 ] = ypr[1] * 180/M_PI;
    ypr[ 2 ] = ypr[2] * 180/M_PI;
    

    // scale accelerometer sensitivity
    acceleration[ 0 ] = ( float ) aaReal.x / ACCEL_SENSITIVITY_SCALE - abs( baseAcceleration[ 0 ] );
    acceleration[ 1 ] = ( float ) aaReal.y / ACCEL_SENSITIVITY_SCALE - abs( baseAcceleration[ 1 ] );
    acceleration[ 2 ] = ( float ) aaReal.z / ACCEL_SENSITIVITY_SCALE - abs( baseAcceleration[ 2 ] );

    Serial.print("ypr\t");
    Serial.print(ypr[0]);
    Serial.print("\t");
    Serial.print(ypr[1]);
    Serial.print("\t");
    Serial.println(ypr[2]);
    
/*    
    Serial.print("accel\t");
    Serial.print(aaReal.x);
    Serial.print("\t");
    Serial.print(aaReal.y);
    Serial.print("\t");
    Serial.println(aaReal.z);
*/    
    Serial.print("accel\t");
    Serial.print(acceleration[ 0 ]);
    Serial.print("\t");
    Serial.print(acceleration[ 1 ]);
    Serial.print("\t");
    Serial.println(acceleration[ 2 ]);
  }
}

// Checks whether base station is standing or not
void updateStanding() {
 
  // Standing means near initial values
  standing = abs( acceleration[ 2 ] ) < 0.1 && abs( acceleration[ 0 ] ) < 0.1 && abs( acceleration[ 1 ] ) < 0.1;
  // Acceleraton on X/Y axis means lying
  float xy =  abs( acceleration[ 0 ] ) + abs( acceleration[ 1 ] );
  lying = xy > 0.4;
  
  /*
  Serial.print( "pitched?\t");
  Serial.println( pitched );
  Serial.print( "!pitched?\t");
  Serial.println( not_pitched );
  Serial.print( "rolled?\t");
  Serial.println( rolled );
  Serial.print( "!rolled?\t");
  Serial.println( rolled );
  
  Serial.print( "standing?\t");
  Serial.println( standing );
  Serial.print( "lying?\t");
  Serial.println( lying );
  */
}

void updateRevolutions() {
  // check octants
  float x = acceleration[ 0 ];
  float y = acceleration[ 1 ];

  // pluck values into interval 0.5..-0.5
  x = min( max( x, -0.5 ), 0.5 );
  y = min( max( y, -0.5 ), 0.5 );
  
  if ( x > 0 && y > 0 ) {
    octant = 1;
    if ( x > 0.25 )
      octant = 2;
  } else if ( x < 0 && y > 0 ) {
    octant = 3;
    if ( y < 0.25 )
      octant = 4;
  } else if ( x < 0 && y < 0 ) {
    octant = 5;
    if ( x < 0.25 )
      octant = 6;
  } else if ( x > 0 && y < 0 ) {
    octant = 7;
    if ( y > 0.25 )
      octant = 8;
  }
  
  if ( lastOctant == 0 ) {
    lastOctant = octant;
    return;
  }

  int diff = octant - lastOctant;
 /*
  Serial.print( "quad:\t");
  Serial.println( octant );
  Serial.print( "lastq:\t");
  Serial.println( lastOctant);
 */
  if ( abs( diff ) >= 1 ) {
    
    if ( abs( diff ) == 7 ) {
      if ( diff < 0 )
        diff = 1;
      if ( diff > 0 )
        diff = -1;
    }
  
    cumulativeRevolutions += diff * PI / 4;
    lastOctant = octant;
  }
}

float convertRevolutionsToPath() {
  // convert revolutions to radians: revolutions * 2 PI
  // calculate arc length: radians * radius
  float path = abs( cumulativeRevolutions ) * RADIUS;
  Serial.print( "DISTANCE\t");
  Serial.print( path );
  Serial.println( " cm ");
  return path;
}

void resetRevolutions() {
  cumulativeRevolutions = 0.0;
  octant = 0;
  lastOctant = 0;
}

void processChip( chip, standing ) {

  if ( lastChip != chip ) {
      // chip is fresh!
      if ( lastChip == CHIP_TRUNK ) {
        // close trunk
        // send group|2#trunk|0 to unity    
      }
      if ( chip == CHIP_TRUNK ) {
        // open trunk
        // send group|2#trunk|1 to unity    
      }
      
    }

  if ( standing ) {
    // standing
    if ( chip == CHIP_PERSON ) {
      powerLed( 0, HIGH );
      powerLed( 1, LOW );
      // send group|2#person|0#present|1#color|xyz
      // send group|2#person|1#present|0#color|xyz
    } else if ( chip == CHIP_FUEL ) {
      // show amount of fuel left
      fadeToPercentage( fuelFill );
    } else if ( chip == CHIP_TRUNK ) {
      // show amount of trunk space
      fadeToPercentage( 1.0 );
    }
  } else {
    // lying
    if ( chip == CHIP_PERSON ) {
      // do nothing
    } else if ( chip == CHIP_FUEL ) {
      // set percentage from distance/maxdistance
      float fuelRangeAvailable = max( fuelFill, convertRevolutionsToPath() * fuelScale / fuelRange );
      fadeToPercentage( 1 - fuelRangeAvailable );
      // no unity here
    } else if ( chip == CHIP_TRUNK ) {
      // set percentage from distance/trunk
      float trunkFree = max( 1, convertRevolutionsToPath()*trunkScale / trunkCapacity );
      fadeToPercentage( 1 - trunkFree );
      // send group|2#trunkspace|trunkFree
    }
  }
  lastChip = chip;
}

void loop() {
  
  int chip = 0;
  int color[3];

  // get ypr values
  processGyro();

  // wait to get stable values
  if ( millis() < 2000 )
    return;

  // initialize LEDs and stuff
  if ( ! initialized ) 
    initialize();

  // check if there is chip on base station
  if ( isBridged() ) { 
    // if yes, identify it
    chip = getChipCode();
  } else {
    // else use default chip (second-top)
    chip = defaultChip;
  }
  // set LED colors to chip
  setColorFromChipCode( chip , &color[0] ) ;
  fadeToColor( color );
  // check if standing
  updateStanding();

  if ( standing == lying ) {
    // indefinite state! return!
    return;
  }

  // base is standing
  if ( standing && !lying ) {
    resetRevolutions();
    processChip( chip, true );
    return;
  }
  // base is lying
  if ( !standing && lying ) {
    // set percentage based on path travelled
    updateRevolutions();
    processChip( chip, false );
  }
  
}
