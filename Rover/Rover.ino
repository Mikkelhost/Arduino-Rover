// Wire.h er det vi bruger når man skal bruge i2c
#include <Wire.h>
// Refererer til kompas bibleoteket
#include <HMC5883L.h>

/* EEPROM står for Electrically Erasable Programmable Read-Only Memory
 *  og er en slags hukommelse som ikke bliver rydet hvis man slukker arduinoen.
 *  Her kan vi gemme vores kalibration af kompasset derved sparer vi de 20 sek.
 *  det nu tager. Har man kalibreret kompasset kan man bruge loadCompass() i stedet
 *  for. Så læser den de x og y offsets som er lagret i EEPROM
 */
#include <EEPROM.h>

/* Compass.h er et library som vi har taget fra en youtuber,
 * han har lavet nogenlunde det samme, Normalt ville vi have brugt 
 * sample koden som kom fra adafruit, dog er data fra et ukalibreret
 * HMC5883L modul ikke det bedste. Det er det ikke fordi at plotter man de 
 * x- og y-værdier man får fra kompasset ind i et koordinatsystem,
 * er de ikke centreret omkring origo. Dette er der en funktion 
 * calibrateCompass(); der kan gøre fra Compass.h.
 */
#include "Compass.h"


#define trigPin1 47
#define echoPin1 46
#define trigPin2 39
#define echoPin2 38
#define trigPin3 31
#define echoPin3 30
#define trigPin4 49
#define echoPin4 48
#define trigPin5 27
#define echoPin5 26
#define M1 40 // RØD
#define M2 41 // BLÅ
#define PWM1 44 // BRUN
#define PWM2 45 // GUL

// Denne funktion sørger for at ligemeget hvor stor en værdi der er udregnet,
// bliver den aldrig mindre en 0 eller større end 255
int pwmlimit(int value)
{
   if(value < 0)
     value = 0;

    if (value > 255)
       value = 255;
    return value;  
}

// Disse værdier indeholder al ultralydssensorenes data
long duration, distance, HoejreSensor, VenstreSensor, FrontSensor, HoejreSide, VenstreSide;
// Store our compass as a variable.
// Record any errors that may occur in the compass.
int error = 0;

// HeadingDegree er vores kurs
int HeadingDegree = 0;

// Distance, indeholder distancen til destinationen
float Distance;

// Denne boolean er falsk når bilen står stille og sand når den kører
bool drive = false;

// Bilen kan køre baglæns så er denne sand kører bilen ligeudbaglæns
bool backWards = false;

// denne værdi siger noget om hvornår bilen skal kører efter sensorer eller gps
int SensorsMax = 50;

// Bilen starter op med disse koordinater lagret
float HomeLat = 56.654251, HomeLon = 9.774503, latDecimal, lonDecimal;

void setup()
{
  // alle de pins vi bruger på arduinoen bliver defineret som IN- eller OUTPUT
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);
  pinMode(trigPin4, OUTPUT);
  pinMode(echoPin4, INPUT);
  pinMode(trigPin5, OUTPUT);
  pinMode(echoPin5, INPUT);
  
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  
  // Starter den serielle kommunikation
  Serial.begin(9600);
  Serial2.begin(38400);
  Serial3.begin(38400);

  // I setuppet står motoren stille
  digitalWrite(M1, HIGH);
  digitalWrite(M2, HIGH);
  analogWrite(PWM1, 0);
  analogWrite(PWM2, 0);
  
  // Starter i2c
  Wire.begin(); 
  Serial3.print("Starting");

  // En funktion fra Compass.h, denne sætter kompasset til at tage kontinuerte målinger
  compass_setup();

  /* Man kan tage en solid ledning og holde pin 7 høj for at starte calibration
  *  er den ikke høj loader man sidste kalibrering
  */
  if(digitalRead(7) == HIGH)
  {
    calibrateCompass();
  }
  else
  {
    loadCompass();
  }
 
}





// Our main program loop.
void loop()
{
  /* Serial3 er der hvor bluetooth modulet er sat til
   *  vi styre ikke ligefrem bilen herfra, men vi kan sætte den 
   *  til at køre efter gps og stå stille samt køre ligeud baglæns
   *  Udover det har vi sagt at den skal skifte til nogle andre koordinater 
   *  hvis man tryder på 'q', 'w' og 'e' 
   *  Trykker man på space tager den 20 målinger fra gpsmodulet og laver
   *  dem til hjemmedestinationen
   */
  if(Serial3.available())
  {
    switch(Serial3.read())
    {
      case '1': 
        drive = true; 
        backWards = false;
      break;
      case '0': 
        drive = false; 
        backWards = false;
      break;
      case ' ':
        HomeLat = 0;
        HomeLon = 0;
        for(int i = 0; i < 20; i++)
        {
          GPSvalues();
          HomeLat += latDecimal;
          HomeLon += lonDecimal;
        }
        HomeLat = HomeLat/20;
        HomeLon = HomeLon/20;
        Serial3.print("HomeLat: "); Serial3.print(HomeLat); Serial3.print(" HomeLon: "); Serial3.println(HomeLon);
      break;  
      case 'q':
        HomeLat = 56.633598;
        HomeLon = 9.813211;
      break;
      case 'w':
        HomeLat = 56.633619;
        HomeLon = 9.813399;
      break;
      case 'e':
        HomeLat = 56.633632;
        HomeLon = 9.813620;
      break;
      case '2':
        backWards = true; 
        drive = false;   
      break;  
    }
  }
  
  /* Serial2 er der hvor vores gps modul er sat i og har gps'en lavet
   *  en måling, beregner den lændge og bearing for at nå til hjemme destinationen
   *  herefter tager kompasset en måling og laver denne måling om til den retning
   *  som bilen har hvor den står.
   */
  if(Serial2.available())
  {
    GPSvalues();
    //compassHeading();
    compass_loop();
  }

  // drive er sand
  if(drive)
  { 
    // Læser de fem sensorer der sidder på bilen
    Obstacles(trigPin1, echoPin1);
    HoejreSensor = distance;
    if(distance == 0)
    {
      HoejreSensor = 100;
    }
    Obstacles(trigPin2, echoPin2);
    FrontSensor = distance;
    if(distance == 0)
    {
      FrontSensor = 100;
    }
    Obstacles(trigPin3, echoPin3);
    VenstreSensor = distance;
    if(distance == 0)
    {
      VenstreSensor = 100;
    }
    Obstacles(trigPin5, echoPin5);
    VenstreSide = distance;
    if(distance == 0)
    {
      VenstreSide = 100;
    }
    Obstacles(trigPin4, echoPin4);
    HoejreSide = distance;
    if(distance == 0)
    {
      HoejreSide = 100;
    }
    
    Serial.println(VenstreSide);
    Serial.println(HoejreSide);
    
    // autopiloten kører 
    autoPilot();
  }
  else if(backWards)
  { 
    // bilen kører baglæns hvis backWards er sand
    digitalWrite(M1, LOW);
    digitalWrite(M2, LOW);
    analogWrite(PWM1, 255);
    analogWrite(PWM2, 255);
  }
  else
  {
    // hvis bilen hverken kører frem eller tilbage står den selvfølgelig stille
    digitalWrite(M1, HIGH);
    digitalWrite(M2, HIGH);
    analogWrite(PWM1, 0);
    analogWrite(PWM2, 0);
  }      
}

/* autoPilot() er det stykke i programmet der får bilen til at kører automatisk */
void autoPilot()
{
 int PWM1_1 = 150; // 150 er hvad de 2 PWM værdier ca står på når bilen kører ligeud
 int PWM2_2 = 150;
 int distPwm1; // Disse 2 værdier stiger jo tætter på destinationen  bilen kommer
 int distPwm2; 
 float pk = 1.42; // en konstant vi ganger på når vi regner hvilken vej den skal køre
 int kursfejl = HeadingDegree-currentHeading; // Kursfejlen svinger mellem 180 og -180
 
 /* Da der på et kompas er 360 grader vil kursfejlen blive over 180 hvis man for
  *  eksempel skulle kører mod 2 grader men kører 358, vil kursfejlen ikke være 4,
  *  men -356 og det kompenserer vi for nedenunder i dette if statement
  */
 if(kursfejl > 180 || kursfejl < -180)
 {
    if(kursfejl < 180)
    {
      kursfejl = HeadingDegree-currentHeading+360;
    }
    else
    {
      kursfejl = HeadingDegree-currentHeading-360;
    }
    Serial3.print("kursfejl1: "); Serial3.println(kursfejl);
 }
 else
 {
    kursfejl = HeadingDegree-currentHeading;
    Serial3.print("kursfejl2: "); Serial3.println(kursfejl);
 }
  
  /* kommer bilen inden for 2 meter begynder de to værdier at falde mod -255,
   *  indtil at bilen står helt stille
   */
  if(Distance <= 2)
  {
    distPwm1 = map(Distance, 0, 2, -255, 0);
    distPwm2 = map(Distance, 0, 2, -255, 0);
  }
  else
  {
    distPwm1 = 0;
    distPwm2 = 0;
  } 

  /* Da vores hjul har for godt grip, har vi sagt at bliver kursfejlen større end 5
   *  eller mindre end -5 skal den dreje med fuld kraft
   */
  if(kursfejl > 5)
  {
    PWM1_1 = 255;
    PWM2_2 = 0;
  }
  else if(kursfejl < -5)
  {
    PWM1_1 = 0;
    PWM2_2 = 255;
  }
  // er kursfejlen inden for vores parametre styres de to PWM værdier så vi kommer så tæt på 0 i kursfejl som vi kan
  PWM1_1 = pwmlimit( (PWM1_1+kursfejl*pk)+distPwm1 );
  PWM2_2 = pwmlimit( (PWM2_2-kursfejl*pk)+distPwm2 );

  /* der sidder to sensorer med en ret vinkel til fronten, dette gør at vi kan
   *  køre langs vægge osv. Og møder en af disse sensorer noget på vejen, ignorerer 
   *  den de to PWM værdier og kører langs objektet den har mødt indtil objektet ikke længere 
   *  er til stede
   */
  if(HoejreSide > 40 && VenstreSide > 40|| Distance < 5)
  {
    // bilen kører efter gps så længe at ingen af sensorerne er er under deres Max
    if(FrontSensor > SensorsMax && HoejreSensor > SensorsMax && VenstreSensor > SensorsMax)
    { 
      digitalWrite(M1, HIGH);
      digitalWrite(M2, HIGH);
      analogWrite(PWM1, PWM1_1);// Change to PWM1_1 to turn on navigation
      analogWrite(PWM2, PWM2_2);// Change to PWM2_2 to turn on Navigation      
    }
    else if(HoejreSensor <= SensorsMax && VenstreSensor <= SensorsMax && FrontSensor > SensorsMax && Distance > 5)
    {
      digitalWrite(M1, HIGH);
      digitalWrite(M2, HIGH);
      analogWrite(PWM1, 180);
      analogWrite(PWM2, 180);
    }
    // påvirkes nogle af de tre foreste sensorer hopper programmet ind i dette stykke kode
    else if(FrontSensor <= SensorsMax || HoejreSensor <= SensorsMax || VenstreSensor <= SensorsMax && Distance > 5)
    {
      if(HoejreSensor <= SensorsMax)
      {
        digitalWrite(M1, LOW);
        digitalWrite(M2, HIGH);
        analogWrite(PWM1, 255);
        analogWrite(PWM2, 255);      
      }
      else if(VenstreSensor <= SensorsMax)
      {
        digitalWrite(M1, HIGH);
        digitalWrite(M2, LOW);
        analogWrite(PWM1, 255);
        analogWrite(PWM2, 255);
        
      }
      else
      {
        digitalWrite(M1, HIGH);
        digitalWrite(M2, LOW);
        analogWrite(PWM1, 255);
        analogWrite(PWM2, 255);
      }
    }
  }
  else if(HoejreSide <= 40)
  {
    if(FrontSensor < 50)
    {
      digitalWrite(M1, LOW);
      digitalWrite(M2, HIGH);
      analogWrite(PWM1, 255);
      analogWrite(PWM2, 255);
      delay(100);
    }
    else
    {
      // jo tættere på vægen bilen kommer jo hurtigere kører de inderste hjul
      PWM1_1 = 200;
      PWM2_2 = 200;
      PWM1_1 = pwmlimit(map(HoejreSide, 0, 40, 0, 255));
      PWM2_2 = pwmlimit(map(HoejreSide, 0, 40, 255, 0));
      digitalWrite(M1, HIGH);
      digitalWrite(M2, HIGH);
      analogWrite(PWM1, PWM1_1);
      analogWrite(PWM2, PWM2_2); 
      Serial.print("pwm1=");
      Serial.print(PWM1_1);
      Serial.print(" pwm2=");
      Serial.println(PWM2_2);
    }
  }
  else
  {
    if(FrontSensor < 50)
    {
      digitalWrite(M1, HIGH);
      digitalWrite(M2, LOW);
      analogWrite(PWM1, 255);
      analogWrite(PWM2, 255);  
      delay(100);         
    }
    else
    {
      // jo tættere på vægen bilen kommer jo hurtigere kører de inderste hjul
      PWM1_1 = 200;
      PWM2_2 = 200;
      PWM1_1 = pwmlimit(map(VenstreSide, 0, 40, 255, 0));
      PWM2_2 = pwmlimit(map(VenstreSide, 0, 40, 0, 255));
      digitalWrite(M1, HIGH);
      digitalWrite(M2, HIGH);
      analogWrite(PWM1, PWM1_1);
      analogWrite(PWM2, PWM2_2); 
      Serial.println(PWM1_1);
      Serial.println(PWM2_2);
    }
  }
}

/* for at spare plads har vi lavet dette som en funktion
 *  denne funktion er forklaret på dirverse forums og er ikke
 *  vores egen
 */
void Obstacles(int trigPin,int echoPin)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH, 7000);
  distance = (duration / 2) / 29.1;
}
  
//******************************************************//
//                    Funktion: GPSvalues()             //
// Den følgende funktion læser værdier fra vores U-blox //
// Neo-6M modul Herefter splitter man den string som    //  
// er i form af NMEA. Dette gøres ved splitString som   //
// fordybes længere nede i programmet. Herudaf får man  //
// latitude og longtitude i form af grader og minutter  //
// f.eks. kunne latituden være 5639.24 her vil de 56    //
// være graderne og de 39.24 være minutterne.           //
// For at man kan få et decimal koordinat, skal man     //
// dividere minutterne med 60 og ligge dem til graderne //
// for at seperere grader fra minutter bruges substring.//
// disse substrings konverteres til float værdier.      // 
// til sidst har man to float værdier indeholdene       //
// latituden og longtituden i decimal output.           //                           
//******************************************************//
void GPSvalues()
{
  if(Serial2.find("$GPGLL"))
  {
    String GPSWhole = Serial2.readStringUntil('\n'); // hele strengen læses indtil der ikke er mere
    String lat = splitString(GPSWhole, ',',1);// andvender splitString til at klippe latitude og longtitude ud
    String lon = splitString(GPSWhole, ',',3);
    /* Her udnyttes substring til at seperere grader fra
     *  minutter */ 
    String latDegree = lat.substring(0,2);
    String latMinutes = lat.substring(2);
    String lonDegree = lon.substring(0,3);
    String lonMinutes = lon.substring(3);
    
    /* graderne ligges sammen med minutterne som er blevet
       divideret med 60 */
    latDecimal = latDegree.toFloat() + latMinutes.toFloat() / 60;
    lonDecimal = lonDegree.toFloat() + lonMinutes.toFloat() / 60;

    
    Serial3.print("Latitude: "); Serial3.println(latDecimal,7);
    Serial3.print("Longtitude: "); Serial3.println(lonDecimal,7);

    // Beregning af heading
    /* Formlen for at beregne Heading/Bearing kan findes på
     * http://www.igismap.com/formula-to-find-bearing-or-heading-angle-between-two-points-latitude-longitude/ */ 
    float Xdec, Ydec, HeadingRadian, HomeLatRad, HomeLonRad, latRad, lonRad;
    
    // Konversion til Radianer
    HomeLatRad = HomeLat * (PI / 180);
    HomeLonRad = HomeLon * (PI / 180);
    latRad = latDecimal * (PI / 180);
    lonRad = lonDecimal * (PI / 180);

    Xdec = cos(HomeLatRad) * sin(HomeLonRad - lonRad);
    Ydec = cos(latRad) * sin(HomeLatRad) - sin(latRad) * cos(HomeLatRad) * cos(HomeLonRad - lonRad);
    
    HeadingRadian = atan2(Xdec,Ydec);
    HeadingDegree = round(HeadingRadian * (180 / PI));
    if(HomeLon-lonDecimal < 0) 
    {
      HeadingDegree += 360;
    }
    

    // Beregning af distance
    /* Formlen for at beregne distance kan findes på
     * http://www.igismap.com/haversine-formula-calculate-geographic-distance-earth/ */ 
    float a, c;
   
    a = pow(sin((HomeLatRad-latRad)/2),2) + cos(latRad) * cos(HomeLatRad) * pow(sin((HomeLonRad-lonRad)/2),2);
    c = 2 * atan2(sqrt(a),sqrt(1-a));
    Distance = 6371000 * c;    
  
    Serial3.print("Heading to reach destination: "); Serial3.println(HeadingDegree);
    Serial3.print("Distance to reach destination: "); Serial3.print(Distance); Serial3.println(" Meters");
  }
  else
  {
    Serial3.println("No data was found");
  }
}

//*********************************************************************//
//       Funktion: splitString(String s, char parser, int index)       //
// Funktionen er fra nettet, vi bruger den fordi den gør det meget     //
// mere overskueligt at se på.                                         //
// Denne funktion er nævnt tidligere. Den bliver brugt i forbindelse   //
// med at udlede latitude og lontitude fra en NMEA string. Den tager   //
// en string s og leder derefter en parser som i vores tidligere       //
// tilfælde var et komma. så kigger den på indexet. Indexet siger      //
// noget om hvilket komma funktionen skal begynde at læse fra. Når den //
// så har læst fra indexet til den næste parser stopper den. Man har   //
// nu en del af den større string som man så kan gøre andet ved.       //
//*********************************************************************//
String splitString(String s, char parser,int index){
  String rs=String('\0');
  int parserIndex = index;
  int parserCnt=0;
  int rFromIndex=0, rToIndex=-1;

  while(index>=parserCnt){
    rFromIndex = rToIndex+1;
    rToIndex = s.indexOf(parser,rFromIndex);

    if(index == parserCnt){
      if(rToIndex == 0 || rToIndex == -1){
        return String('\0');
      }
      return s.substring(rFromIndex,rToIndex);
    }
    else{
      parserCnt++;
    }

  }
  return rs;
}



