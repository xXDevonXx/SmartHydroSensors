  #include "DFRobot_EC10.h"
  #include <EEPROM.h>
  
  #define EC_PIN A2
  float voltage,ecValue,temperature = 25;
  DFRobot_EC10 ec;
  
  #include "DFRobot_PH.h"
  #include <EEPROM.h>
  
  #define PH_PIN A1
  float p_voltage,p_phValue,p_temperature = 25;
  DFRobot_PH ph;
  
  #include <dht.h>
  
  dht DHT;
  #define DHT22_PIN 7
  
  void setup() {
    Serial.begin(115200);  
    ec.begin();
    
    //Serial.begin(115200);  
    //ph.begin();
  }
  
  void loop() {
    //ec sensor loop
    static unsigned long timepoint = millis();
      if(millis()-timepoint>2000U)  //time interval: 1s
      {
        timepoint = millis();
        Serial.println();
        EC_Sensor();
        PH_Sensor();     
        TempHum_Sensor();
      }
      ec.calibration(voltage,temperature);  // calibration process by Serail CMD
      ph.calibration(p_voltage,p_temperature); // calibration process by Serail CMD
  }
  
  void EC_Sensor() {
        voltage = analogRead(EC_PIN)/1024.0*5000;  // read the voltage
        Serial.print("voltage: ");
        Serial.print(voltage);
        //temperature = readTemperature();  // read your temperature sensor to execute temperature compensation
        ecValue =  ec.readEC(voltage,temperature);  // convert voltage to EC with temperature compensation
        Serial.print("  temperature: ");
        Serial.print(temperature,1);
        Serial.print("^C  EC: ");
        Serial.print(ecValue,1);
        Serial.println("ms/cm");
  }
  
  void PH_Sensor() {
        p_voltage = analogRead(PH_PIN)/1024.0*5000;  // read the voltage
        p_phValue = ph.readPH(p_voltage,p_temperature);  // convert voltage to pH with temperature compensation
        Serial.print("temperature: ");
        Serial.print(p_temperature,1);
        Serial.print("^C  pH: ");
        Serial.println(p_phValue,2);
  }

  void TempHum_Sensor() {
      DHT.read22(DHT22_PIN);
      Serial.print("humidity(%): ");
      Serial.print(DHT.humidity, 1);
      Serial.print("  temperature: ");
      Serial.println(DHT.temperature, 1);
  }