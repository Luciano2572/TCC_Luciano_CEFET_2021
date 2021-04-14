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


// CONFIGURA A REFERÊNCIA DO ADC COMO EXTERNA
  // descomente a linha abaixo para utilizar a referência de tensão externa
  //analogReference(EXTERNAL);

  }

void loop() {
  
  piscaled();
  
  }

void piscaled(){
  digitalWrite(led3, LOW);
  digitalWrite(led0, HIGH);
  delay(500);
  digitalWrite(led0, LOW);
  digitalWrite(led1, HIGH);
  delay(500);
  digitalWrite(led1, LOW);
  digitalWrite(led2, HIGH);
  delay(500);
  digitalWrite(led2, LOW);
  digitalWrite(led3, HIGH);
  delay(500);  
  
  }
