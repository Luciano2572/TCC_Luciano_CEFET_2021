#include "LiquidCrystal_I2C.h" // Added library*
#include <EEPROM.h>

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

LiquidCrystal_I2C lcd(0x27,2,1,0,4,5,6,7); 

void setup() {
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

// INICIALIZAÇÃO DO LCD 20x4

  lcd.begin (20,4); 
  lcd.setBacklightPin(3,POSITIVE);
  lcd.setBacklight(HIGH);

// CONFIGURA A REFERÊNCIA DO ADC COMO EXTERNA

//  analogReference(EXTERNAL);

  }

void loop() {
  
  escreverlcd();
  testebotoes();
  testemos();
  }

void testebotoes(){
  
  digitalWrite(led0, digitalRead(bot0));
  digitalWrite(led1, digitalRead(bot1));
  digitalWrite(led2, digitalRead(bot2));
  digitalWrite(led3, digitalRead(bot3));
  
  }
  
void escreverlcd() {
  lcd.setCursor(0,0);
  lcd.print("Primeira linha");  
  lcd.setCursor(0,1);
  lcd.print("Segunda linha");  
  lcd.setCursor(0,2);
  lcd.print("Terceira linha...");
  lcd.setCursor(0,3);
  lcd.print("Valor do pot: ");
  lcd.print(analogRead(pot));
  lcd.print("   ");
  }

void testemos(){
  if (digitalRead(bot0) && digitalRead(bot3)){
    analogWrite(mos, analogRead(pot)/4);
    }
  if (digitalRead(bot1) && digitalRead(bot2)){
    analogWrite(mos, 0);
    }
  }
