//#include <dht.h>
#define analogPinForRV    1   
#define analogPinForTMP   0
const float zeroWindAdjustment =  .42; 

int TMP_Therm_ADunits;  //temp termistor value from wind sensor
float RV_Wind_ADunits;    //RV output from wind sensor 
float RV_Wind_Volts;
unsigned long lastMillis;
int TempCtimes100;
float zeroWind_ADunits;
float zeroWind_volts;
float WindSpeed_MPH;

void setup()
{
  Serial1.begin(115200);   // faster printing to get a bit better throughput on extended info
  // remember to change your serial monitor

  Serial1.println("start");
  // put your setup code here, to run once:

  //   Uncomment the three lines below to reset the analog pins A2 & A3
  //   This is code from the Modern Device temp sensor (not required)
  pinMode(A2, INPUT);        // GND pin      
  pinMode(A3, INPUT);        // VCC pin
  digitalWrite(A3, LOW);     // turn off pullups
}

void loop()
{

//    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
//  delay(1000);                       // wait for a second
//  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
//  delay(1000); 

  //Sensor - Humidity and Temp
//  DHT.read11(dht_dpin);
  
  if (millis() - lastMillis > 200){      // read every 200 ms - printing slows this down further
    
    TMP_Therm_ADunits = analogRead(analogPinForTMP);
    RV_Wind_ADunits = analogRead(analogPinForRV);
    RV_Wind_Volts = (RV_Wind_ADunits *  0.0048828125);

    // these are all derived from regressions from raw data as such they depend on a lot of experimental factors
    // such as accuracy of temp sensors, and voltage at the actual wind sensor, (wire losses) which were unaccouted for.
    TempCtimes100 = (0.005 *((float)TMP_Therm_ADunits * (float)TMP_Therm_ADunits)) - (16.862 * (float)TMP_Therm_ADunits) + 9075.4;  

    zeroWind_ADunits = -0.0006*((float)TMP_Therm_ADunits * (float)TMP_Therm_ADunits) + 1.0727 * (float)TMP_Therm_ADunits + 47.172;  //  13.0C  553  482.39

    zeroWind_volts = (zeroWind_ADunits * 0.0048828125) - zeroWindAdjustment;  

    // This from a regression from data in the form of 
    // Vraw = V0 + b * WindSpeed ^ c
    // V0 is zero wind at a particular temperature
    // The constants b and c were determined by some Excel wrangling with the solver.
    
   WindSpeed_MPH =  pow(((RV_Wind_Volts - zeroWind_volts) /.2300) , 2.7265);   
   
    Serial1.print("  TMP volts ");
    Serial1.print(TMP_Therm_ADunits * 0.0048828125);
    
    Serial1.print(" RV volts ");
    Serial1.print((float)RV_Wind_Volts);

    Serial1.print("\t  TempC*100 ");
    Serial1.print(TempCtimes100 );

    Serial1.print("   ZeroWind volts ");
    Serial1.print(zeroWind_volts);

    Serial1.print("   WindSpeed MPH ");
    Serial1.println((float)WindSpeed_MPH);
    Serial.print("   WindSpeed MPH ");
    Serial.println((float)WindSpeed_MPH);

    lastMillis = millis();    
  } 

  // arduino reads Serial
   if (Serial1.available() > 0) {
     int incomingByte = Serial1.read();

     Serial.print(incomingByte);

     if (incomingByte == 00) { // 0x01 = char 1
       digitalWrite(LED_BUILTIN, HIGH);
     } else if (incomingByte == 0) { // 0x00 = char 0
       digitalWrite(LED_BUILTIN, LOW);
     }
   }
}
