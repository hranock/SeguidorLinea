#include <OrangutanMotors.h> //libreria motores

//Pines a usar
#define LON         2      // LED ON Regleta
#define TRIG        4      // IR
#define LEDD        7      // led der
#define LEDI        8      // led izq
#define BOTON       10     // Boton
#define muxS0       A1     // pin s0 canal
#define muxS1       A2     // pin s1 canal
#define muxS2       A3     // pin s2 canal
#define muxS3       A4     // pin s3 canal
#define muxSIG      A5     // pin OUT multiplexor

//Paramatros de sensores QTR
#define N_SENSORES  14     // numero de sensores
#define REFERENCIA  6500
#define UMBRAL_MIN  79

unsigned long sensorValor[N_SENSORES];
unsigned int sensorValorMAX[N_SENSORES];
unsigned int sensorValorMIN[N_SENSORES];
unsigned int sensorValorPROM[N_SENSORES];
unsigned long posicionLinea;
int error[8];

//:::::::::::::::: A J U S T E S  P I D ::::::::::::::::::::::::
const int velMax = 105; //120  // 190
const int velFrenoMax = 100;
const int velFrenoMin = 90;
int limite;

float kP = 0.0236; //.018
float kI = 0.0004; //.0007
float kD = 0.2854; //0.216


// Variables para Control
int proporcional;
long integral;
int derivada;
int diferenciaPwr; 
int toleranciaM;

OrangutanMotors motores;

void setup() {
  pinMode(TRIG, INPUT);
  pinMode(BOTON, INPUT);
  pinMode(LEDD, OUTPUT);
  pinMode(LEDI, OUTPUT);
  pinMode(LON, OUTPUT);
  pinMode(muxS0, OUTPUT);
  pinMode(muxS1, OUTPUT);
  pinMode(muxS2, OUTPUT);
  pinMode(muxS3, OUTPUT);

  Serial.begin(9600);

  //activar/desactivar motores, regleta y turbina
  motBaby(0,0);
  digitalWrite(LON,1);

  // ***************** CALIBRANDO *****************
  esperarBoton();
  iniciarCalibracion();

  esperarBoton();
  iniciarlizarVariables();

  esperarBoton();
  esperarTrigger();
  
  for(int i = 50; i<velMax; i+=15){
    motBaby(i,i);
    delay(15);
  }
}

void loop() {
  if (!digitalRead(TRIG)){ 
    motBaby(0,0);
    diferenciaPwr = 0;
  }else{
    calcularPID();
    //verPosicionPwr();
    //verValores();
  }
}

//******************* BOTON *******************
void esperarBoton() {
  digitalWrite(LEDD, HIGH);
  digitalWrite(LEDI, HIGH);
  while(digitalRead(BOTON));
  delay(100);
  digitalWrite(LEDD, LOW);
  digitalWrite(LEDI, LOW);
}

void esperarTrigger(){
  digitalWrite(LEDD, HIGH);
  digitalWrite(LEDI, HIGH);
  while(!digitalRead(TRIG));
  digitalWrite(LEDD, LOW);
  digitalWrite(LEDI, LOW);
}

//******************* MOTORES *******************
void motBaby(int motor_izq, int motor_der) {
  motores.setSpeeds(motor_izq, motor_der);
}


//******************* REGLETA *******************
void setCanal(byte canal){
   digitalWrite(muxS3, bitRead(canal, 3));
   digitalWrite(muxS2, bitRead(canal, 2));
   digitalWrite(muxS1, bitRead(canal, 1));
   digitalWrite(muxS0, bitRead(canal, 0));
}

void leerSensores(){
   int k = 0;
   for (int i = 14; i > 0 ; i--){
      setCanal(i);
      sensorValor[k] = analogRead(muxSIG)/10;
      k++;      
   }
}

int leerLineaNegra(int lineaAnterior){
  unsigned long suma = 0;
  unsigned long divisor = 0;
  leerSensores();  
  for(int i = 0 ; i < N_SENSORES; i++){
    if(sensorValor[i] > sensorValorPROM[i]){
      suma += i*1000*sensorValor[i];
      divisor += sensorValor[i];
    }
  }
  if(divisor > 0)
    return (long) suma/divisor;
  else
    return  lineaAnterior; 
}

int leerLineaBlanca(int lineaAnterior){
  unsigned long suma = 0;
  unsigned long divisor = 0;
  leerSensores();  
  for(int i = 0 ; i < N_SENSORES; i++){
    if(sensorValor[i] < sensorValorPROM[i]){
      suma += i*1000*sensorValor[i];
      divisor += sensorValor[i];
    }
  }
  if(divisor > 0)
    return (long) suma/divisor;
  else
    return  lineaAnterior; 
}

void iniciarCalibracion(){
  for( int i = 0 ; i < N_SENSORES ; i++){;
    sensorValorMAX[i] = 0;
    sensorValorMIN[i] = UMBRAL_MIN;
  }
  for(int i = 0 ; i < 75 ; i++){
    digitalWrite(LEDD, HIGH);
    digitalWrite(LEDI, HIGH);
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
    digitalWrite(LEDD, LOW);
    digitalWrite(LEDI, LOW);
    delay(22);
  }
}

//     ::::::::::::: CALCULAR ERROR (PID) :::::::::::::
void calcularPID() {
  posicionLinea=leerLineaNegra(posicionLinea);
  proporcional=posicionLinea-REFERENCIA;

  integral=error[0]+error[1]+error[2]+error[3]+error[4]+error[5]+error[6]+error[7];

  error[7]=error[6];
  error[6]=error[5];
  error[5]=error[4];
  error[4]=error[3];
  error[3]=error[2];
  error[2]=error[1];
  error[1]=error[0];
  error[0]=proporcional;      
  
  derivada=proporcional-error[7];

  if(integral>5000){
    digitalWrite(LEDD, 0);
    digitalWrite(LEDI, 1);
  }else{
    if(integral<-5000){
      digitalWrite(LEDD, 1);
      digitalWrite(LEDI, 0);
    }else{
      digitalWrite(LEDD, 0);
      digitalWrite(LEDI, 0);
    }
  } 
  diferenciaPwr=proporcional*kP+integral*kI+derivada*kD;

  if (diferenciaPwr>toleranciaM) diferenciaPwr=toleranciaM;
  if (diferenciaPwr<-toleranciaM) diferenciaPwr=-toleranciaM;
    
  //Control de motores
  if (diferenciaPwr > 0){
    motBaby(velMax, velMax-diferenciaPwr);
  }else{
    motBaby(velMax+diferenciaPwr, velMax);
  }
}

void iniciarlizarVariables(){
  limite = velMax-10;
  diferenciaPwr = 0;
  posicionLinea = 0;
  integral = 0;
  derivada = 0;
  error[0] = 0;
  error[1] = 0;
  error[2] = 0;
  error[3] = 0;
  error[4] = 0;
  error[5] = 0;
  error[6] = 0;
  error[7] = 0;
  toleranciaM = velMax;
  for(int i = 0 ; i<N_SENSORES ; i++){
    sensorValorPROM[i] = int (sensorValorMAX[i] + sensorValorMIN[i])/2;
  }
  for(int i = 0; i<N_SENSORES; i++){
    Serial.print(sensorValorMAX[i]);
    Serial.print(F(" - "));
    Serial.print(sensorValorMIN[i]);
    Serial.print(F(" - "));
    Serial.println(sensorValorPROM[i]);
  }
}

void frenosContorno(){
  if(sensorValor[0]>sensorValorPROM[0] || sensorValor[1]>sensorValorPROM[1]){
    motBaby(-velFrenoMin,velFrenoMax); 
    while(true){
      leerSensores(); //lectura en bruto de sensor   
      if(sensorValor[2]>sensorValorPROM[2] || sensorValor[3]>sensorValorPROM[3]){
        break;
      } 
    }
  }
 
  if(sensorValor[12]>sensorValorPROM[12] || sensorValor[13]>sensorValorPROM[13]){ 
    motBaby(velFrenoMax, -velFrenoMin);
    while(true){
      leerSensores();
      if(sensorValor[10]>sensorValorPROM[10] || sensorValor[11]>sensorValorPROM[11]){
        break;
      }  
    }
  }
}

//******************* REVISANDO *******************
void verPosicionPwr(){
      Serial.print(proporcional);
      Serial.print(F(" => "));
      Serial.println(diferenciaPwr);
}

void verValores(){
  leerSensores();
  for(int i = 0 ; i < N_SENSORES ; i++ ){
    Serial.print(sensorValor[i]);
    Serial.print(F(" || "));
  }
  Serial.println(F("~"));
}
