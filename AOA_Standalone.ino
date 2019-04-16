// AOA Standalone Unit
// Differential pressure sensor:
//     MPXV7002 - 2 kPa 
//     good for up to 110Kts if used as an airspeed sensor
//     With 60deg betwwen pitot and AOA ports it is good up to 220Kts
// or  MPXV5010 - 10 kPa (~248 Knots)
//     This one might be a bit too rough for the AoA sensor but good for Airspeed sensor
//
// A bit of theory (magic)
// Assume
// z - Angle of attack
// y - angle between pitot and AoA port
// V - airspeed
// D = differential AoA airspeed (I know this one is weird, but bear with me)
// D = V*(cos(z) - cos(y-z))
// Here where we cheat: (cos(z) - cos(y-z)) non-linear, but in the range of z [-5..+15] it is practically linear so can be converted to (a*z+b)
// a = (cos(15)-cos(y-15))/15 - linear approximation between 0deg and 15deg
// b = cos(y)
// so D = V(az+b)
// Now, D is measured by the AoA sensor
// V is measured by the Airspeed sensor. At this stage we assume V doesnt change with the change of AoA. In reality there is 3% variation on the range of z [-5..+15]
// Finally z = (D/V-b)/a
// Linear approximation parameters a and b will be calculated at startup based on Y and stall AoA
// However, D/V is equvalent to SQRT(Paoa/Pias) because of the formula V=SQRT(2*Pias/R) where R is air density
// So finally z = (SQRT(Paoa/Pias)-b)/a

int i = 0; // counter
unsigned long currentMillis;
float AOA = 0;
int AOAmin = -2;
int AOAmax = 16;   //Stall AoA
float a_lap, b_lap; // linear approximation params 

int topLED = 2;

unsigned int SensorAngle = 62; //Angle between the pittot port and the AOA port (degrees)
float Pmin = 100; // Minimum differeretial pressure to activate the unit. 
                  // Corresponds to about 25 knots

// AoA differential sensor
unsigned int AOArefMin = 5;    // Minimum reading from the sensor. For some reason it is never 0
unsigned int AOArefHalf = 534; // Sensor reading when AOA is half the angle between pitot and AOA port
unsigned int AOArefMax = 1015; // Maximum reading from the sensor. For some reason it is never 1023
unsigned int AOAMaxPressure = 2000; // max differential pressure for the installed sensor
                                    // for MPXV7002 - 2000 Pa (~110 Knots)
                                    // for MPXV5010 - 10000 pa (~248 Knots)
// IAS differential sensor
unsigned int IASrefMin = 6;    // Minimum reading from the sensor. For some reason it is never 0
unsigned int IASrefHalf = 537; // Sensor reading when AOA is half the angle between pitot and AOA port
unsigned int IASrefMax = 1015; // Maximum reading from the sensor. For some reason it is never 1023
unsigned int IASMaxPressure = 2000; // max differential pressure for the installed sensor
                                    // for MPXV7002 - 2000 Pa (~110 Knots)
                                    // for MPXV5010 - 10000 pa (~248 Knots)

unsigned int AOA_RAW_Input = 0;
unsigned int IAS_RAW_Input = 0;
float Pias = 0, Paoa = 0; // Differential pressure from the sensors

// two arrays for moving avarage
#define ArraySize 8 // play with the value - greater value display is less sensitive to change, smaller value - display becomes jumpy
float PiasArray[ArraySize];
float PiasArraySum =0;
float PaoaArray[ArraySize];
float PaoaArraySum =0;
int ArrayIndex = 0;

void setup() 
{
   Serial.begin(115200);    
// init the pins 2-11
  for (int i = 2; i <= 11; i++) {
    pinMode(i, OUTPUT);  
    delay(10);
  }   

  delay(300);

  for (int i = 2; i <= 11; i++) {
    digitalWrite(i, HIGH); 
    delay(200);
  }   

  // calculate linear approximation params
  a_lap = (cos((float)AOAmax*PI/180)-cos((float)(SensorAngle-AOAmax)*PI/180))/(float)AOAmax;
  b_lap = cos((float)SensorAngle*PI/180);

  Serial.print("a_lap = ");
  Serial.println(a_lap);
  Serial.print("b_lap = ");
  Serial.println(b_lap);
}

void loop()
{

 AOA_RAW_Input = analogRead(A1); 
 Serial.print("AOA_RAW_Input = ");
 Serial.println(AOA_RAW_Input);

 IAS_RAW_Input = analogRead(A7); 
 Serial.print("IAS_RAW_Input = ");
 Serial.println(IAS_RAW_Input);

 // the "if" below is to compensate for non-linearity and disbalance of the pressure sensor
 if (AOA_RAW_Input > AOArefHalf) {
    Paoa = (float)AOAMaxPressure * ((float)AOA_RAW_Input - (float)AOArefHalf) / ((float)AOArefMax - (float)AOArefHalf);
 } else {
    Paoa = (float)AOAMaxPressure * ((float)AOA_RAW_Input - (float)AOArefHalf) / ((float)AOArefHalf - (float)AOArefMin);
 }

 if (IAS_RAW_Input > IASrefHalf) {
    Pias = (float)IASMaxPressure * ((float)IAS_RAW_Input - (float)IASrefHalf) / ((float)IASrefMax - (float)IASrefHalf);
 } else {
    Pias = (float)IASMaxPressure * ((float)IAS_RAW_Input - (float)IASrefHalf) / ((float)IASrefHalf - (float)IASrefMin);
 }

  PiasArraySum -= PiasArray[ArrayIndex];
  PiasArray[ArrayIndex] = Pias;
  PiasArraySum += PiasArray[ArrayIndex];
  Pias = PiasArraySum / ArraySize;

 Serial.print("Paoa = ");
 Serial.println(Paoa);
 Serial.print("Pias = ");
 Serial.println(Pias);

  PaoaArraySum -= PaoaArray[ArrayIndex];
  PaoaArray[ArrayIndex] = Paoa;
  PaoaArraySum += PaoaArray[ArrayIndex];
  Paoa = PaoaArraySum / ArraySize;

  ArrayIndex +=1; // shift the index for next time
  if (ArrayIndex == ArraySize) {  // if we reached the top of the array
    ArrayIndex = 0;                       //go to the start of the array
  }

 
 Serial.print("Paoa = ");
 Serial.println(Paoa);
 Serial.print("Pias = ");
 Serial.println(Pias);
 Serial.print("IAS = ");
 Serial.println(3600.0*sqrt(2*Pias/1.225)/1852.0);
 Serial.println();

 if (Pias > Pmin and Paoa > 0) {
    AOA = (sqrt(Paoa/Pias)-b_lap)/a_lap;
 } else {
    AOA = 0;
 }
 
 Serial.print("AOA = ");
 Serial.println(AOA);
 Serial.println();

  topLED = map((int)AOA, AOAmin, AOAmax, 11, 2); // map doesnt work properly

  for (int i = topLED; i <= 11; i++) {
    digitalWrite(i, LOW); 
  }
  
  for (int i = 2; i < topLED; i++) {
    digitalWrite(i, HIGH); 
  }
  
  delay(100);
 
}
