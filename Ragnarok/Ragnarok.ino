#include <OrangutanMotors.h> //libreria motores
#include <ESC.h> // libreria turbina

//Pines a usar
#define BOTON       2      // Boton
#define TRIG        4      // IR
#define LEDD        7      // led derecho
#define LEDI        8      // led izquierdo
#define TURB        9      // Turbina
#define LON         A0     // Led ON Regleta
#define muxS0       A1     // pin s0 canal
#define muxS1       A2     // pin s1 canal
#define muxS2       A3     // pin s2 canal
#define muxS3       A4     // pin s3 canal
#define muxSIG      A5     // pin OUT multiplexor

//Paramatros de sensores QTR
#define N_SENSORES  16     // numero de sensores
#define REFERENCIA  7500
#define UMBRAL_MIN  79

unsigned long sensorValor[N_SENSORES];
unsigned int sensorValorMAX[N_SENSORES];
unsigned int sensorValorMIN[N_SENSORES];
unsigned int sensorValorPROM[N_SENSORES];
unsigned long posicionLinea;
int error[8];

//:::::::::::::::: A J U S T E S  P I D ::::::::::::::::::::::::
const int velMax = 180; //100  //190     //220      //240
const int velFrenoMax = 100;
const int velFrenoMin = 90;
const int velTurbina = 1450;  


float kP = 0.0312 ;  //0.0107 //.0259  //.03195   //.03389
float kI = 0.0005; //0.00035 //.00025   //.00025 
float kD = 0.374;   //0.214    //.397    //.357    //.384

//vel 115   
//p .01534  
//i   
//d .294    


// Variables para Control
int proporcional;
long integral;
int derivada;
int diferenciaPwr; 
int toleranciaM;
boolean activo;


OrangutanMotors motores;
ESC turbina(TURB);

void setup() {
  pinMode(TRIG, INPUT);
  pinMode(BOTON, INPUT);
  pinMode(LEDI, OUTPUT);
  pinMode(LEDD, OUTPUT);
  pinMode(LON, OUTPUT);
  pinMode(muxS0, OUTPUT);
  pinMode(muxS1, OUTPUT);
  pinMode(muxS2, OUTPUT);
  pinMode(muxS3, OUTPUT);

  Serial.begin(115200);

  //activar/desactivar motores, regleta y turbina
  setMotores(0,0);
  digitalWrite(LON,1);
  activo=true;

  // ***************** CALIBRANDO *****************
  esperarBoton();
  iniciarCalibracion();

  esperarBoton();
  iniciarlizarVariables();
  
  iniciarTurbina(activo);
  delay(100);

  esperarBoton();
  velocidadTurbina(activo, velTurbina);
  esperarTrigger();

  for(int i=20; i<120; i+=10){
    setMotores(i, i);
    delay(10);
  }
}

void loop() {
  if (!digitalRead(TRIG)){ 
    setMotores(0,0);
    velocidadTurbina(true, 1000);
    diferenciaPwr = 0;
  }else{
    calcularPID();
    //frenosContorno();
    //verPosicionPwr();
    //verValores();
  }
}

//     ::::::::::::: CALCULAR ERROR (PID) :::::::::::::
void calcularPID() {
  posicionLinea = leerLineaNegra(posicionLinea);
  proporcional = posicionLinea - REFERENCIA;
  /*
  if(proporcional > 2000){
    digitalWrite(LEDD, HIGH);
  }else{
    if(proporcional < -2000){
      digitalWrite(LEDI, HIGH);
    }else{
      digitalWrite(LEDI, LOW);
      digitalWrite(LEDD, LOW);
    }
  }
  //*/

  integral = error[0] + error[1] + error[2] + error[3] + error[4] + error[5] + error[6] + error[7] ;

  error[7] = error[6];
  error[6] = error[5];  
  error[5] = error[4];
  error[4] = error[3];
  error[3] = error[2];
  error[2] = error[1];
  error[1] = error[0];
  error[0] = proporcional;      
  
  derivada = proporcional - error[5];
  /*
  if(integral>30000 || integral <-30000){
    frenosContorno();
  }else{
  */
  diferenciaPwr = proporcional*kP + integral*kI + derivada*kD;

  if (diferenciaPwr > toleranciaM) diferenciaPwr = toleranciaM;
  if (diferenciaPwr < -toleranciaM) diferenciaPwr = -toleranciaM;
    
  //Control de motores
  if (diferenciaPwr > 0){
    setMotores(velMax, velMax - diferenciaPwr);
  }else{
    setMotores(velMax + diferenciaPwr, velMax);
  }
  //}
}

//******************* BOTON *******************
void esperarBoton() {
  digitalWrite(LEDI, 1);
  digitalWrite(LEDD, 1);
  while(digitalRead(BOTON));
  delay(150);
  digitalWrite(LEDI, 0);
  digitalWrite(LEDD, 0);

}

void esperarTrigger(){
  delay(100);
  digitalWrite(LEDI, 1);
  digitalWrite(LEDD, 1);
  while(!digitalRead(TRIG));
}

//******************* MOTORES *******************
void setMotores(int motor_izq, int motor_der) {
  motores.setSpeeds(motor_izq, motor_der);
}

//******************* TURBIN> *******************
void iniciarTurbina(boolean activo){
  if(activo){
    turbina.init();
  }
}

void velocidadTurbina(boolean activar, int velocidad){
  if(activar){
    turbina.setSpeed(velocidad);
  }
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
   for (int i = 15; i > -1 ; i--){
      setCanal(i);
      sensorValor[k] = analogRead(muxSIG)/10;
      k++;      
   }
}

void iniciarCalibracion(){
  for( int i = 0 ; i < N_SENSORES ; i++){;
    sensorValorMAX[i] = 0;
    sensorValorMIN[i] = UMBRAL_MIN;
  }
  for(int i = 0 ; i < 95 ; i++){
    digitalWrite(LEDI, 1);
    digitalWrite(LEDD, 1);
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
    digitalWrite(LEDI, 0);
    digitalWrite(LEDD, 0);
    delay(22);
  }
}

int  leerLineaNegra(int lineaAnterior){
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

int  leerLineaBlanca(int lineaAnterior){
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

void iniciarlizarVariables(){
  diferenciaPwr = 0;
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
  toleranciaM = velMax * 1;
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
    setMotores(-velFrenoMin,velFrenoMax); 
    while(true){
      leerSensores(); //lectura en bruto de sensor   
      if(sensorValor[2]>sensorValorPROM[2] || sensorValor[3]>sensorValorPROM[3]){
        break;
      } 
    }
  }
 
  if(sensorValor[14]>sensorValorPROM[14] || sensorValor[15]>sensorValorPROM[15]){ 
    setMotores(velFrenoMax, -velFrenoMin);
    while(true){
      leerSensores();
      if(sensorValor[12]>sensorValorPROM[12] || sensorValor[13]>sensorValorPROM[13]){
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
