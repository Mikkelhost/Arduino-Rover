float heading = 0;
float headingDegrees = 0;
int currentHeading = 0;
//float bearing = 0;
//float distance = 0;

struct compassCalibrationData {
  int lower[3];
  int upper[3];
  int center[3];
  float scale[3];
};

compassCalibrationData compCalib;

int compassNow[3];
    
void compass_setup() {
  Serial3.print("Mag... ");
  // instruct chip we want to read it continuously (TODO: REPLACE WITH BETTER POWER SAVING MODE)
  Wire.beginTransmission(0x1e); // transmit to device 0x1e
  Wire.write(0x02); // sends one byte (select register to write to - MODE)
  Wire.write(0x00); // sends one byte (configure mode - continuous)
  Wire.endTransmission();
  Serial3.println("done.");
}

void displayCompassInfo()
{
  static unsigned long lastCompassDisplay = 0;
  unsigned long now = millis();
  if ( now - lastCompassDisplay > 200 ) {
    lastCompassDisplay = now;
    Serial3.print("Heading: ");
    Serial3.println(currentHeading);
 /*   Serial.print("  Bearing: ");
    Serial.print(bearing);
    Serial.print("  Distance: ");
    Serial.print(distance);
    Serial.println();*/
  }
}

bool getCompassRaw()
{
  Wire.beginTransmission(0x1e);
  Wire.write(0x03); //select register 3, X MSB register - to start read from
  Wire.endTransmission();

  Wire.requestFrom(0x1e, 6);
  if (6 <= Wire.available()) {
    
    //Read data from each axis, 2 registers per axis
    for (int i = 0; i < 3; i++) {
      compassNow[i] = Wire.read()<<8; //X msb
      compassNow[i] |= Wire.read(); //X lsb
    }
    
    return true;
  }
  
  return false;
}

void printCompassCalibrationValues() {
  Serial3.println("Compass mins ");
  for (int i = 0; i < 3; i++) {
    Serial3.println(compCalib.lower[i]);
  }
  Serial3.println("Compass maxs ");
  for (int i = 0; i < 3; i++) {
    Serial3.println(compCalib.upper[i]);
  }  
  Serial3.println("Compass avgs ");
  for (int i = 0; i < 3; i++) {
    Serial3.println(compCalib.center[i]);
  }
  //Serial.println("Average span ");
  //Serial.println(averageSpan);  
  Serial3.println("Compass scale ");
  for (int i = 0; i < 3; i++) {
    Serial3.println(compCalib.scale[i]);
  }
}

void calibrateCompass() 
{
  Serial3.print("Calibrating compass... ");
  
  memset( &compCalib, 0, sizeof(compCalib) );
  
  unsigned long startTime = millis();

  while ( millis() - startTime < 30000 ) {
    if ( getCompassRaw() ) {
      for (int i = 0; i < 3; i++) {
        compCalib.lower[i] = min( compCalib.lower[i], compassNow[i] );
        compCalib.upper[i] = max( compCalib.upper[i], compassNow[i] );
      }
    }
    delay(1);
  }  
  
  for (int i = 0; i < 3; i++) {
    compCalib.center[i] = 0.5 * compCalib.lower[i] + 0.5 * compCalib.upper[i];
  }
  for (int i = 0; i < 3; i++) {
    compCalib.scale[i] = (compCalib.upper[i] - compCalib.lower[i]) * 0.5;
  }
  float averageSpan = 0.3333 * compCalib.scale[0] + 0.3333 * compCalib.scale[1] + 0.3333 * compCalib.scale[2];
  for (int i = 0; i < 3; i++) {
    compCalib.scale[i] = averageSpan / compCalib.scale[i];
  }

  printCompassCalibrationValues();
  
  for ( int i = 0; i < (int)sizeof(compCalib); i++)
    EEPROM.write( i, ((byte*)&compCalib)[i] );
    
  Serial3.println("done.");
}

void loadCompass() {
  
  Serial3.println("Loading compass calibration...");
  
  for ( int i = 0; i < (int)sizeof(compCalib); i++)
    ((byte*)&compCalib)[i] = EEPROM.read( i );

  printCompassCalibrationValues();
}

void adjustCompassNow() 
{
  for (int i = 0; i < 3; i++) {
    compassNow[i] -= compCalib.center[i];
    compassNow[i] *= compCalib.scale[i];
  }
}
#define RADTODEG 57.29577951308230876f
void compass_loop() {
  static unsigned long lastCompassRead = 0;
  unsigned long now = millis();
  if ( now - lastCompassRead > 50 ) {
    lastCompassRead = now;

    if ( getCompassRaw() ) {

      adjustCompassNow();
      
      heading = atan2(-compassNow[0], compassNow[2]);
      heading += 0.044;

      if(heading < 0)
        heading += 2*PI;

      if(heading > 2*PI)
        heading -= 2*PI;

      headingDegrees = heading * 180/M_PI;

      currentHeading = round(headingDegrees);

      displayCompassInfo();
    }
  }
}

