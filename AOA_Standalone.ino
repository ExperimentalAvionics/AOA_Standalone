// AOA Standalone Unit
// Differential pressure sensor:
//     MPXV7002 - 2 kPa 
//     good for up to 110Kts if used as an airspeed sensor
//     With 60deg betwwen pitot and AOA ports it is good up to 220Kts
// or  MPXV5010 - 10 kPa (~248 Knots)
//     This one might be a bit too rough for the AoA sensor but good for Airspeed sensor
//
// For details visit: http://experimentalavionics.com/angle-of-attack-standalone-unit/

// V=SQRT(2*P/R) where P - dinamic pressure, R is air density (1.225kg/m3)

int i = 0; // counter
unsigned long currentMillis;
unsigned long LastOutputTS;
int Fault = 0; // Error code
                // 1 - blocked port (one of them)

// flaps change airfoil so the calibration data will be different for each flaps position
#define FlapsArraySize 4      // Number of flaps positions (eg 0, 5, 10, 30deg)
float Xzero[FlapsArraySize];  // Pias/Paoa at Zero AoA for each flaps position. Xzero[0] - no flaps
float Xstall[FlapsArraySize]; // Pias/Paoa at Stall AoA for each flaps position. Xstall[0] - no flaps
int FlapsPosition = 0;        // Flaps position 0 - no flaps, 1 - first stage, etc

float AOA = 0;
float Xaoa = 0;    // Temporary variable for Pias/Paoa
float AOAmin = -1;   // Min AoA (Zero Lift for NACA 23012) (for display - turns blue and green LEDs on at Zero AoA)
float AOAmax = 15;   // Stall AoA
float AOAbg = 4.8;     // Best Glide AoA (Top Green LED is on)
float A, B;        // linear funclion coefficients for calculating AoA = A*Xaoa + B

int topLED = 2;

float Pmin = 15; // Minimum differeretial pressure to activate the unit. 

// AoA differential sensor
unsigned int AOArefMin = 5;    // Minimum reading from the sensor. For some reason it is never 0
unsigned int AOArefHalf = 534; // Sensor reading at Zero airspeed
unsigned int AOArefMax = 1015; // Maximum reading from the sensor. For some reason it is never 1023
unsigned int AOAMaxPressure = 2000; // max differential pressure for the installed sensor
                                    // for MPXV7002 - 2000 Pa (~110 Knots)
                                    // for MPXV5010 - 10000 pa (~248 Knots)
// IAS differential sensor
unsigned int IASrefMin = 6;    // Minimum reading from the sensor. For some reason it is never 0
unsigned int IASrefHalf = 535; // Sensor reading at Zero airspeed
unsigned int IASrefMax = 1015; // Maximum reading from the sensor. For some reason it is never 1023
unsigned int IASMaxPressure = 2000; // max differential pressure for the installed sensor
                                    // for MPXV7002 - 2000 Pa (~110 Knots)
                                    // for MPXV5010 - 10000 pa (~248 Knots)

unsigned int AOA_RAW_Input = 0;
unsigned int IAS_RAW_Input = 0;
float Pias = 0, Paoa = 0; // Differential pressure from the sensors

// two arrays for moving avarage
#define ArraySize 300 // play with the value - greater value display is less sensitive to change, smaller value - display becomes jumpy
unsigned int RAWiasArray[ArraySize];
unsigned long RAWiasArraySum =0;
unsigned int RAWaoaArray[ArraySize];
unsigned long RAWaoaArraySum =0;
unsigned int ArrayIndex = 0;

void setup() 
{
   Serial.begin(115200);

// pins for detecting flaps position
// Assumption is that flaps position sensed by groubding some pins with a a few vane tipe swithes
// Alternatively the A0, A4, or A5 can be easily configured to sense voltage from a resistive positioner.
  pinMode(14, INPUT_PULLUP); // A0 - D-SUB - 2
  pinMode(18, INPUT_PULLUP); // A4 - D-SUB - 3
  pinMode(19, INPUT_PULLUP); // A5 - D-SUB - 7

// init the pins 2-11
  for (int i = 2; i <= 12; i++) {
    pinMode(i, OUTPUT);  
    delay(10);
  }   

  delay(300);
  
  for (int i = 2; i <= 12; i++) {
    digitalWrite(i, HIGH); 
    delay(200);
  }   

  digitalWrite(12, LOW); 

// Read the calibrations values from EEPROM and calculate the linear function coefficients
// hardcoded values for now
   Xzero[0] = 1.6752;
   Xstall[0] = 1.967;

   Xzero[1] = 1.6752;
   Xstall[1] = 1.967;

   Xzero[2] = 1.6752;
   Xstall[2] = 1.967;

   Xzero[3] = 1.6752;
   Xstall[3] = 1.967;

   Serial.print("AOA_RAW");
   Serial.print("\t");
   Serial.print("IAS_RAW");
   Serial.print("\t");
   Serial.print("Paoa");
   Serial.print("\t");
   Serial.print("Pias");
   Serial.print("\t");
   Serial.print("IAS");
   Serial.print("\t");
   Serial.println("AOA");

}

void loop()
{
 
// Read Flaps position  
 FlapsPosition = 0;

 if (digitalRead(14) == LOW) {
  FlapsPosition = 1;
 }
 if (digitalRead(18) == LOW) {
  FlapsPosition = 2;
 }
 if (digitalRead(19) == LOW) {
  FlapsPosition = 3;
 }

 A = (AOAmax - AOAmin) / (Xstall[FlapsPosition] - Xzero[FlapsPosition]);
 B = AOAmin - (A * Xzero[FlapsPosition]);

// Read sensors
  IAS_RAW_Input = analogRead(A7); 

  RAWiasArraySum -= RAWiasArray[ArrayIndex];
  RAWiasArray[ArrayIndex] = IAS_RAW_Input;
  RAWiasArraySum += RAWiasArray[ArrayIndex];
  IAS_RAW_Input = RAWiasArraySum / ArraySize;

  AOA_RAW_Input = analogRead(A1); 

  RAWaoaArraySum -= RAWaoaArray[ArrayIndex];
  RAWaoaArray[ArrayIndex] = AOA_RAW_Input;
  RAWaoaArraySum += RAWaoaArray[ArrayIndex];
  AOA_RAW_Input = RAWaoaArraySum / ArraySize;

  ArrayIndex++; // shift the index for next time
  if (ArrayIndex == ArraySize) {  // if we reached the top of the array
    ArrayIndex = 0;                       //go to the start of the array
  }

  
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

 Paoa=(-1)*Paoa;  // either this or swap around the ports on the sensor


 if (Pias > Pmin) {
    Xaoa = Pias/Paoa;
    if (abs(Xaoa)>15) {  // large value indicates blocked port somewhere
      AOA = AOAmax;
      Fault = 1;
    } else {
      Fault = 0;
      AOA = round(A*Xaoa + B);
    }
 } else {
    Fault = 0;
    Xaoa = 0;
    AOA = 0;
 }

  if (AOA < AOAmin) {
    AOA = AOAmin;
  }
  
  if (AOA > AOAmax) {
    AOA = AOAmax;
  }

  if (AOA > AOAbg) {
    topLED = map(AOA,AOAbg, AOAmax, 6, 2); // map function is weird sometimes. Watch out!
  } else {
    topLED = map(AOA, AOAmin, AOAbg, 11, 7); // map function is weird sometimes. Watch out!
  }

  
  for (int i = topLED; i <= 11; i++) {
    digitalWrite(i, LOW); 
  }
  
  for (int i = 2; i < topLED; i++) {
    digitalWrite(i, HIGH); 
  }
  
// serial output every xxx ms

  if (millis() - LastOutputTS > 1000) {

      Serial.print(AOA_RAW_Input);
      Serial.print("\t");
      Serial.print(IAS_RAW_Input);
      Serial.print("\t");
      Serial.print(Paoa);
      Serial.print("\t");
      Serial.print(Pias);
      Serial.print("\t");
      Serial.print(3600.0*sqrt(2*Pias/1.225)/1852.0);
      Serial.print("\t");
      Serial.println(AOA);

      LastOutputTS = millis();

  }

 
}
