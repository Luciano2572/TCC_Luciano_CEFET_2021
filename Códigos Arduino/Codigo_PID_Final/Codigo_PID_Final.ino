#include <Wire.h> //  I2C
#include "LiquidCrystal_I2C.h" 
#include <EEPROM.h>
#include <VL6180X.h>
#include <FastPID.h>

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

#define SCALING 3 // argumento do sensor VL6180X
VL6180X sensor;
LiquidCrystal_I2C lcd(0x27,2,1,0,4,5,6,7); 

long starttime;
int var;
// VARIAVEIS PID

// Aqui se definem as variáveis do PID. Abaixo algumas variáveis que foram testadas, como comentários
// Dependendo da configuração do protótipo, bem como a ventoinha, esses valores podem mudar drasticamente
//float Kp=1.6, Ki=0.30, Kd=0.62, Hz=60; // PID OK 1 BOLA
// float Kp=0.04, Ki=0.10, Kd=0.122, Hz=20; // RAZOAVEL  PID OK 1 BOLA 2021
//float Kp=0.07, Ki=0.09, Kd=0.08, Hz=20; 
//float Kp=0.15, Ki=0.06, Kd=0.08, Hz=15; // bom



float Kp=0.13, Ki=0.06, Kd=0.08, Hz=15; // muito bom
int output;
unsigned int valorpot, dist, setpoint;
int output_bits = 8;
bool output_signed = false;
int saida;
FastPID myPID(Kp, Ki, Kd, Hz, output_bits, output_signed);
unsigned char sel;
/////////////

unsigned int offsetfan = 0;

unsigned long temposel, temposelaux, t1,t2;
int aux;

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
// descomente a linha abaixo para usar a referência externa de 5v
  //analogReference(EXTERNAL);

  sel = 0;
  sensor.init();
  sensor.configureDefault();
  sensor.setScaling(SCALING);
  sensor.setTimeout(500);
  Serial.println("Tempo(mS) Distancia Setpoint");
  // valorpot = map(304, 50, 500, 0, 1023); // utilize essa linha para fixar um setpoint inicial logo de cara de 303mm, útil para calibrar
  }

void loop() {
  
  
  if (digitalRead(bot0)) valorpot = analogRead(pot); // somente atualiza o setpoint (valorpot) se o key0 for pressionado
  escreverlcd(); // atualiza o lcd com os valores

  pid(); // existe também o onoff(); - onoffpwm(); - e proporcional(); nesse mesmo código
  Serial.print(millis());
  Serial.print(" ");
  Serial.print(sensor.readRangeSingleMillimeters());
  Serial.print(" ");
  Serial.println(setpoint);
  statusleds();
  }

void escreverlcd() {
  lcd.setCursor(0,0);
  lcd.print("Setpoint: ");  
  lcd.print(setpoint);
  lcd.print(" mm  Diff");
  lcd.setCursor(0,1);
  lcd.print("Distancia ");  
  lcd.print(dist);
  lcd.print(" mm  ");
  lcd.print(abs(setpoint-dist));
  lcd.print(" ");
  lcd.setCursor(0,2);
  lcd.print("PWM: ");
  lcd.print(var);
  lcd.print("   ");
  lcd.setCursor(0,3);
  lcd.print("TCC");
  }

  void onoff(){
    dist = sensor.readRangeSingleMillimeters();
    setpoint = map(valorpot, 0, 1023, 50, 500);

    if (dist > setpoint) if (dist - setpoint > 6) analogWrite(mos, 0);
    if (dist < setpoint) if (setpoint - dist > 6) analogWrite(mos, 255);
    
    }

  void onoffpwm(){
    dist = sensor.readRangeSingleMillimeters();
    setpoint = map(valorpot, 0, 1023, 50, 500);

    if (dist > setpoint) if (dist - setpoint > 6) analogWrite(mos, 129);
    if (dist < setpoint) if (setpoint - dist > 6) analogWrite(mos, 140);
    
    }

  void proporcional(){
    dist = sensor.readRangeSingleMillimeters();
    setpoint = map(valorpot, 0, 1023, 50, 500);
    int auxp = dist - setpoint;
    auxp = abs(auxp);
    int auxmos;
    if (dist > setpoint)  auxmos = 131 - (auxp / 7);
    if (dist < setpoint)  auxmos = 131 + (auxp / 7);
    analogWrite(mos, auxmos);
    
    }
    
  void pid(){
    sel = 0;
    //valorpot = analogRead(pot);
    dist = sensor.readRangeSingleMillimeters();
    setpoint = map(valorpot, 0, 1023, 50, 500);
    temposelaux = millis() - temposel;

    unsigned int feedback = dist;
    uint32_t before, after;
    before = micros();
    uint8_t output = myPID.step(setpoint, feedback);
    after = micros();
    unsigned int auxfan = output+offsetfan;
    if (auxfan > 250) auxfan = 250;
    var = auxfan;
    analogWrite(mos, auxfan);
    saida = output;
  }

  void statusleds(){ // funcao simples para ir acendendo mais leds conforme a bola estiver mais próxima ao setpoint
    int diff = dist - setpoint;
    if (abs(diff) < 7) digitalWrite(led0, HIGH); else digitalWrite(led0, LOW);
    if (abs(diff) < 13) digitalWrite(led1, HIGH); else digitalWrite(led1, LOW);
    if (abs(diff) < 21) digitalWrite(led2, HIGH); else digitalWrite(led2, LOW);
    if (abs(diff) < 31) digitalWrite(led3, HIGH); else digitalWrite(led3, LOW);    
    }
