
#include "Particle.h"
#define LENG 100

int lmp91000_1_en = B0;     //enable line for the lmp91000 AFE chip for measuring CO
int lmp91000_2_en = B2;
int fiveVolt_en = D5;
int plantower_en = B4;
int power_led_en = D6;
int kill_power = WKP;
int esp_wroom_en = D7;
int blower_en = D2;
int co2_en = C5;        //enables the CO2 sensor power

char buf[LENG]; //Serial buffer sent from PMS1003 Particulate Matter sensor
char incomingByte;  //serial connection from user
PMIC pmic;
int currentValues[] = { 100, 150, 500, 900, 1200, 1500, 2000, 3000 };
SYSTEM_MODE(MANUAL);

void writeRegister(uint8_t reg, uint8_t value);
void setupPMIC(void);
void serialMenu(void);
void goToSleep(void);

void setup()
{
    pinMode(lmp91000_1_en, OUTPUT);
    pinMode(lmp91000_2_en, OUTPUT);
    pinMode(fiveVolt_en, OUTPUT);
    pinMode(plantower_en, OUTPUT);
    pinMode(power_led_en, OUTPUT);
    pinMode(esp_wroom_en, OUTPUT);
    pinMode(blower_en, OUTPUT);
    pinMode(D4, INPUT);
    pinMode(co2_en, OUTPUT);
    //if user presses power button during operation, reset and it will go to low power mode
    //attachInterrupt(D4, System.reset, RISING);



    digitalWrite(lmp91000_1_en, HIGH);
    digitalWrite(lmp91000_2_en, HIGH);
    digitalWrite(power_led_en, HIGH);
    digitalWrite(plantower_en, HIGH);
    digitalWrite(esp_wroom_en, HIGH);
    digitalWrite(blower_en, HIGH);
    digitalWrite(co2_en, HIGH);
    digitalWrite(fiveVolt_en, LOW);

    /*if(digitalRead(D4)){
      goToSleep();
  }*/
    //delay for 5 seconds to give time to programmer person for connecting to serial port for debugging
    delay(10000);
    Serial.begin(9600);

    pmic.begin();
    //pmic.setInputVoltageLimit(4600);  //  for 5Volt usb input
    //pmic.setChargeVoltage(4208);      //  Set Li-Po charge termination voltage to 4.21V,
    //pmic.setInputCurrentLimit(2000);
    pmic.setChargeCurrent(0,0,0,0,1,0);
    pmic.enableDPDM();
    pmic.enableBuck();                //  enableBuck required when using EN Pin Shutdown ??.
    pmic.enableCharging();

    Serial.print("Starting value of inputCurrentLimit: ");
    Serial.println(pmic.getInputCurrentLimit());
    Serial.println("");

    /*for (int i = 0; i < sizeof(currentValues) / sizeof(int); i++) {
        Serial.print("Calling setInputCurrentLimit with value: ");
        Serial.println(currentValues[i]);
        pmic.setInputCurrentLimit(currentValues[i]);
        delay(500);

        Serial.print("Calling getInputCurrentLimit.... Result: ");
        Serial.println(pmic.getInputCurrentLimit());
        delay(500);

        Serial.println("");
    }

    Serial.println("Finished.");*/
}


void loop()
{
    if (Serial.available() > 0) {
        // read the incoming byte:
        incomingByte = Serial.read();

            Serial.print("incomming byte:");
            Serial.println(incomingByte);


        Serial.println(incomingByte);
        if(incomingByte == 'm'){
          serialMenu();
        }
    }
    delay(500);
}

void setupPMIC(void){
    // Setup the PMIC manually (resets the BQ24195 charge controller)
    // REG00 Input Source Control Register  (disabled)
    writeRegister(0, 0b00110000);  //0x30

    // REG01 Power-On Configuration Register (charge battery, 3.5 V)
    writeRegister(1, 0b00011011);   //0x1B

    // REG02 Charge Current Control Register (1024 + 512 mA)
    writeRegister(2, 0b01100000);   //0x60

    // REG03 Pre-Charge/Termination Current Control Register (128mA pre charge, 128mA termination current)
    writeRegister(3, 0b00010001);   //0x11

    // REG04 Charge Voltage Control Register Format
    writeRegister(4, 0b10110010);   //0xB2

    // REG05 Charge Termination/Timer Control Register
    writeRegister(5, 0b10011010);   //0x9A
    // REG06 Thermal Regulation Control Register
    writeRegister(6, 0b00000011);   //0x03
    // REG07 Misc Operation Control Register Format
    writeRegister(7, 0b01001011);   //0x4B
}

void serialMenu(void){
  incomingByte = '0';
  while(incomingByte!= 'x')
  {
    Serial.print("Menu>");
    Serial.flush();
    while(!Serial.available());
    incomingByte = Serial.read();
    if(incomingByte == 'a'){
        //serialGetCo2Slope();
    }
  }
  Serial.println("Exiting serial menu...");
}

//test for setting up PMIC manually
void writeRegister(uint8_t reg, uint8_t value) {
    // This would be easier if pmic.writeRegister wasn't private
    Wire3.beginTransmission(PMIC_ADDRESS);
    Wire3.write(reg);
    Wire3.write(value);
    Wire3.endTransmission(true);

}

void goToSleep(void){
  //Serial.println("Going to sleep:)");
  digitalWrite(power_led_en, LOW);
  digitalWrite(plantower_en, LOW);
  digitalWrite(esp_wroom_en, LOW);
  digitalWrite(blower_en, LOW);
  digitalWrite(co2_en, LOW);
  digitalWrite(fiveVolt_en, LOW);
  System.sleep(D4,FALLING);
  delay(500);
  System.reset();
  //detachInterrupt(D3);
}