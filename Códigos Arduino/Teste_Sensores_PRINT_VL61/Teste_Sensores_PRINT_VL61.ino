#include <Wire.h> // For I2C
#include "LiquidCrystal_I2C.h" // Added library*
#include <EEPROM.h>
#include <Ultrasonic.h>
#include <VL6180X.h>


#define io2 A0
#define hbridgedir A1
#define pot A2
#define asensor A3
//#define SDA A4
//#define SCL A5
#define io0 0
#define io1 1
#define dsenstrig 2
#define hbridgepwm 3
#define bot0 4
#define led0 5
#define mos 6
#define bot1 7
#define bot2 8
#define led1 9
#define led2 10
#define led3 11
#define bot3 12
#define dsensecho 13

#define SCALING 3
VL6180X sensor;
LiquidCrystal_I2C lcd(0x27,2,1,0,4,5,6,7); 
Ultrasonic ultrassom(2,13);

long starttime;

int sel;
double dist; 
bool start;
void setup() {
  Serial.begin(9600);
  
  pinMode(bot0, INPUT);
  pinMode(bot1, INPUT);
  pinMode(bot2, INPUT);
  pinMode(bot3, INPUT);
  pinMode(pot, INPUT);
  pinMode(led0, OUTPUT);
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);
  pinMode(mos, OUTPUT);
  Wire.begin();
// INICIALIZAÇÃO DO LCD 20x4

  lcd.begin (20,4); 
  lcd.setBacklightPin(3,POSITIVE);
  lcd.setBacklight(HIGH);

// CONFIGURA A REFERÊNCIA DO ADC COMO EXTERNA

  //analogReference(EXTERNAL);

  sel = 0;
#define SCALING 3
  sensor.init();
  sensor.configureDefault();
  sensor.setScaling(SCALING);
  //sensor.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
  //sensor.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
  sensor.setTimeout(500);
  Serial.println("Tempo mS --- distancia mm");
  }

void loop() {
  selectsensor();
  //lersensor();
  escreverlcd();
  //testemos();
  //dist = sensor.readRangeSingleMillimeters();
  testar();
  
  }


void testar(){
  if (sel == 3){
    
    digitalWrite(mos, HIGH);
    Serial.print(millis() - starttime);
    Serial.print("     ");
    Serial.println(sensor.readRangeSingleMillimeters());
    }
  else digitalWrite(mos, LOW);
  }
void selectsensor(){
  
  if (digitalRead(bot0)) sel = 1;
  if (digitalRead(bot1)) sel = 2;
  if (digitalRead(bot2)) {sel = 3; starttime = millis();}
  }
  
void escreverlcd() {
  lcd.setCursor(0,0);
  lcd.print("Teste de Sensor    ");  
  lcd.setCursor(0,1);
  lcd.print("Selecione o Sensor");  
  lcd.setCursor(0,2);
  switch (sel) {
  case 1:
    lcd.print("Sensor Ultrassonico");
    break;
  case 2:
    lcd.print("Sensor SHARP       ");
    break;
  case 3:
    lcd.print("Sensor TOF         ");
    break;
  default:
    lcd.print("Nenhum selecionado ");
    break;
}
  lcd.setCursor(0,3);
  lcd.print("Distancia: ");
  lcd.print(dist);
  lcd.print(" cm ");
  }

void testemos(){
  if (digitalRead(bot0) && digitalRead(bot3)){
    analogWrite(mos, analogRead(pot)/4);
    }
  if (digitalRead(bot1) && digitalRead(bot2)){
    analogWrite(mos, 0);
    }
  }

  void lersensor(){
    if (sel == 0) lcd.print("Nenhum selecionado ");
    if (sel == 1) dist = ultrassom.Ranging(CM);
    if (sel == 2) {
      
                    int sensval = analogRead(asensor);
                    dist = 187754 * pow(sensval, -1.51);
                    delay(500);
      }
    if (sel == 3) dist = sensor.readRangeSingleMillimeters();
    }
