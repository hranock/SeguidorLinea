#include <SoftwareSerial.h>

//Pines a usar
#define BTN   12   // Boton
#define APWM  3    // pin PWM del Motor Izquierdo
#define AIN2  4    // pin 2 de dirección del Motor Izquierdo
#define AIN1  5    // pin 1 de dirección del Motor Izquierdo
#define STB   6    // pin de standby del puente g
#define BIN1  7    // pin 1 de dirección del Motor Derecho
#define BIN2  8    // pin 2 de dirección del Motor Derecho
#define BPWM  9    // pin PWM del Motor Derecho
#define RX    11   // TX del Bluetooth
#define TX    12   // RX del Bluetooth
#define LED   13   // LED indicador

SoftwareSerial bt(TX, RX);

//Paramatros de sensores QTR
#define N_SENSORES  8     // numero de sensores
#define REFERENCIA  3500
#define UMBRAL_MIN  45

unsigned char pines[N_SENSORES] = {A7,A6,A5,A4,A3,A2,A1,A0};
unsigned long sensorValor[N_SENSORES];
unsigned int sensorValorMAX[N_SENSORES];
unsigned int sensorValorMIN[N_SENSORES];
unsigned int sensorValorPROM[N_SENSORES];
unsigned long posicionLinea;
int error[8];

//:::::::::::::::: A J U S T E S  P I D ::::::::::::::::::::::::
const int velMax = 160;

float kP = 0.0412 ; // REFERENCIA / velMax
float kD = 0.683; // kP * 10~15

// Variables para Control
int proporcional;
int derivada;
int diferenciaPwr; 

void setup() {
  pinMode(BTN, INPUT);
  pinMode(APWM, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STB, OUTPUT);
  pinMode(BPWM, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  Serial.begin(115200);
  bt.begin(115200);

  // ***************** CALIBRANDO *****************
  esperarBoton(BTN, LED);
  iniciarCalibracion(LED);

  esperarBoton(BTN, LED);
  iniciarlizarVariables();

  esperarBoton(BTN, LED);
  for(int i=10; i<velMax; i+=10) {
    setMotores(i,i);
    delay(15);
  }
}

void loop() {
    calcularPID();
    //verPosicionPwr();
    //verValores();
}

//******************* BOTON *******************
void esperarBoton(byte boton, byte led) {
  digitalWrite(led, 1);
  while(digitalRead(boton));
  delay(150);
  digitalWrite(led, 0);
}

//******************* MOTORES *******************
void setMotores(int mIzq, int mDer) {
  setMotor(mIzq, BPWM, BIN2, BIN1);
  setMotor(mDer, APWM, AIN1, AIN2);
}

void setMotor(byte v, byte pwm, byte in1, byte in2){
  if(v>0){
    digitalWrite(in1, 1);
    digitalWrite(in2, 0);
    analogWrite(pwm, v);
  }else{
    digitalWrite(in1, 0);
    digitalWrite(in2, 1);
    analogWrite(pwm, -v);
  }
}

//******************* REGLETA *******************
void leerSensores(){
   for (int i = 0; i < N_SENSORES ; i++){
      sensorValor[i] = analogRead(pines[i])/10;
   }
}

void iniciarCalibracion(byte led){
  for( int i = 0 ; i < N_SENSORES ; i++){;
    sensorValorMAX[i] = 0;
    sensorValorMIN[i] = UMBRAL_MIN;
  }
  for(int i = 0 ; i < 150 ; i++){
    digitalWrite(led, 1);
    leerSensores();
    for(int i = 0 ; i < N_SENSORES; i++){
      if(sensorValor[i] > sensorValorMAX[i]){
        sensorValorMAX[i] = sensorValor[i];
      }
      if(sensorValor[i] < sensorValorMIN[i]){
        sensorValorMIN[i] = sensorValor[i]; 
      }
    }
    delay(22);
    digitalWrite(led, 0);
    delay(22);
  }
}

int  leerLineaNegra(int lineaAnterior){
  unsigned long suma = 0;
  unsigned long divisor = 0;
  leerSensores();  
  for(int i = 0 ; i < N_SENSORES; i++){
      if(sensorValor[i] > sensorValorPROM[i]){
        suma += i*1000;
        divisor ++;
      }
  }
  if(divisor > 0)
    return (long) suma/divisor;
  else
    return  lineaAnterior; 
}

int  leerLineaBlanca(int lineaAnterior){
  unsigned long suma = 0;
  unsigned long divisor = 0;
  leerSensores();  
  for(int i = 0 ; i < N_SENSORES; i++){
      if(sensorValor[i] < sensorValorPROM[i]){
        suma += i*1000;
        divisor ++;
      }
  }
  if(divisor > 0)
    return (long) suma/divisor;
  else
    return  lineaAnterior; 
}

//     ::::::::::::: CALCULAR ERROR (PID) :::::::::::::
void calcularPID() {
  posicionLinea = leerLineaBlanca(posicionLinea);
  proporcional = posicionLinea - REFERENCIA;
    
  error[7] = error[6];
  error[6] = error[5];
  error[5] = error[4];
  error[4] = error[3];
  error[3] = error[2];
  error[2] = error[1];
  error[1] = error[0];
  error[0] = proporcional;      
  
  derivada = proporcional - error[7];
  
  diferenciaPwr = proporcional*kP + derivada*kD;

  if (diferenciaPwr > velMax) diferenciaPwr = velMax;
  if (diferenciaPwr < -velMax) diferenciaPwr = -velMax;
    
  //Control de motores
  if (diferenciaPwr > 0){
    setMotores(velMax, velMax - diferenciaPwr);
  }else{
    setMotores(velMax + diferenciaPwr, velMax);
  }
}

void iniciarlizarVariables(){
  diferenciaPwr = 0;
  derivada = 0;
  error[0] = 0;
  error[1] = 0;
  error[2] = 0;
  error[3] = 0;
  error[4] = 0;
  error[5] = 0;
  error[6] = 0;
  error[7] = 0;
  for(int i = 0 ; i<N_SENSORES ; i++){
    sensorValorPROM[i] = int (sensorValorMAX[i] + sensorValorMIN[i])/2;
  }
  for(int i = 0; i<N_SENSORES; i++){
    Serial.print(sensorValorMAX[i]);
    Serial.print(F(" - "));
    Serial.print(sensorValorMIN[i]);
    Serial.print(F(" - "));
    Serial.println(sensorValorPROM[i]);
    
    bt.print(sensorValorMAX[i]);
    bt.print(F(" - "));
    bt.print(sensorValorMIN[i]);
    bt.print(F(" - "));
    bt.println(sensorValorPROM[i]);
  }
}

//******************* REVISANDO *******************
void verValores(){
  leerSensores();
  for(int i = 0; i<N_SENSORES; i++){
    Serial.print(sensorValor[i]);
    Serial.print(F(" || "));
  }
  Serial.println(F(""));

  for(int i = 0; i<N_SENSORES; i++){
    bt.print(sensorValor[i]);
    bt.print(F(" || "));
  }
  bt.println(F(""));
}

void verPosicionPwr(){
  Serial.print(proporcional);
  Serial.print(F(" -> "));
  Serial.println(diferenciaPwr);

  bt.print(proporcional);
  bt.print(F(" -> "));
  bt.println(diferenciaPwr);
}
