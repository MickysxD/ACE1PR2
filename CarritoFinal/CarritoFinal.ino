#include <LedControl.h>
#include <Servo.h>
#include <Servo.h>
#include <Stepper.h>
#include <NewPing.h>

//Definir sensor de sonido
  #define Trigger 3
  #define Echo 2
//Fin de sensor de sonido

//Sensor de color
  #define COLOR A0
//Fin sensor de color

//Definir Llantas
  #define LlantaIA 7
  #define LlantaIR 6
  #define LlantaDA 5
  #define LlantaDR 4
//Fin define llantas

//Definir Sensores
  #define Si 13
  #define Sc 12
  #define Sd 11
//Fin Define Sensores

//Definir variaciones
  #define V1 18
  #define V2 19
//Fin Define variaciones

//Definir stepper
  Stepper STPI(20, 47, 49, 51, 53);
  int velSTPI=256;
  Stepper STPD(20, 31, 33, 35, 37);
  int velSTPD=256;
//Fin de stepper

//Definir servo proteus/turtle
  Servo servo;
  Servo Tservo;
//Fin servo

//Sensor sonido
  NewPing sonar(Trigger, Echo, 200);
//Sensor sonido

//Variables
  int SA,SB,SC;
  int Velocidad=0;
  int Estabilizador=0;
  int Graduador,Referencia;
  unsigned long Tiempo;
  int Centraliza = 0;
  int Direccion = 0;
//Fin Variables

//Velocidad Pwm
  #define PwmI 10
  #define PwmD 9
//Fin Velocidad Pwm

//Otras variables
  int Distancia;
  int ps=0;
  long t,d;
//Fin otras variables

// LedControl(DATA, CLK, CS, NoDEVICE)
LedControl matrix = LedControl(25, 23, 27, 1);

//MATRIZ
int mat_clear[8][8] =   {
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0}
};

//Cofiguracion Iniciales 
void setup() {
  //Llamas Como Salidas
    pinMode(LlantaIA, OUTPUT);
    pinMode(LlantaIR, OUTPUT);
    pinMode(LlantaDA, OUTPUT);
    pinMode(LlantaDR, OUTPUT);
  //Fin Llantas

  //Configuracion Sensores
    pinMode(Si, INPUT);
    pinMode(Sc, INPUT);
    pinMode(Sd, INPUT);
  //Fin Configuracion Sensores

  //Pwm Configuracion
    pinMode(PwmI, OUTPUT);
    pinMode(PwmD, OUTPUT);
  //Fin configuracion pwm

  //Configuracion sensor de sonido
    pinMode(Trigger, OUTPUT);
    pinMode(Echo, INPUT);
    pinMode(COLOR, INPUT);
    digitalWrite(Trigger, LOW);
  //Fin Configuracion sensor de sonido

  //Configuracion servo
    servo.attach(43,1100,2000);
    servo.write(0);
    Tservo.attach(8);
  //Fin servo

  //Velocidad de stepper proteus
    STPI.setSpeed(300);
    STPD.setSpeed(300);


  //Velicidad Comuncacion Serial
    Serial.begin(9600);
  //Fin Velocidad Serial

  matrix.shutdown(0, false);
  matrix.setIntensity(0, 10);
  matrix.clearDisplay(0);

}

/*  COMIENZA PINTAR RECORRIDO DE CARRITO EN MATRIZ 8X8*/
/*STRUC CARRO*/
struct Carro{
    int x = 3;
    int y = 3;
};
Carro carrito;
int tab_principal[8][8];
int dir_x = 1;
int dir_y = 1;
int contador_pasos = 0;
int mov_x = 1, mov_y = 0;
int grados_m = 0;

void pintar_matriz(int caracter[][8]) {
  for (int i = 0; i < 8; i++) {
    for (int j = 0; j < 8; j++) {
      if (caracter[i][j] == 1) {
        // TODO: matrix.setLed(number, row, column, state);
        matrix.setLed(0, i, j, HIGH);
      }
    }
    delay(2);
  }
}

void actualizar_tablero(){
  for(int i = 0; i < 8; i++){
    for(int j = 0; j < 8; j++){
      tab_principal[i][j] = 0;
      if(carrito.x == j && carrito.y == i){
        tab_principal[i][j] = 1;
      }
    }
  }
  pintar_matriz(tab_principal);
}

int cont_loop = 0;
void cambiar_orientacion(){
  if(mov_x == 1){
    mov_x = 0;
    mov_y = 1;
    //rotar_y();
  }else{
    mov_x = 1;
    mov_y = 0;
    //rotar_x();
  }
      if(cont_loop < 4){
        cont_loop++;
    }else{
      matrix.clearDisplay(0);
      cont_loop = 0;
    }
  

}

void rotar_x(){
  dir_x *= -1;
}
void rotar_y(){
  dir_y *= -1;
}
void moveren_x(){
  if(carrito.x + dir_x < -1 || carrito.x + dir_x > 7){
    dir_x *= -1;
  }else{
     carrito.x += dir_x;
  }
}
void moveren_y(){
  if(carrito.y + dir_y < -1 || carrito.y + dir_y > 7){
    dir_y *= -1;
  }else{
     carrito.y+= dir_y;
  }
}
void mover_carrito(int grados){
     if(contador_pasos < 90){
        contador_pasos++;
     }else{
        if(grados > 250){
           cambiar_orientacion();
           grados_m = 0;
        }else{
           if(mov_x == 1){
              moveren_x();
           }else if(mov_y == 1){
              moveren_y();
           }
           contador_pasos = 0;
        }
     }
}

int cm=0;
void loop() {
  todoTurtle();
  //todoMotores();

  if(digitalRead(V1) == HIGH || digitalRead(V2) == HIGH){
    if(cm < 20){
      cm++;  
    }else{
      todoSonido();
      cm = 0;
    }
  }else{
    todoMotores();  
  }
  
  

/*  //movimiento servo motor proteus
  if(ps > 0){
    ps++;
    TServ.write(180);
    delay(5000);
    TServ.write(0);
    delay(5000);
  }
*/ 
}

//Sensor de sonido
void todoSonido(){
    
  digitalWrite(Trigger, HIGH);
  delayMicroseconds(10);          //Enviamos un pulso de 10s
  digitalWrite(Trigger, LOW);
  
  int uS = sonar.ping_median();
  uS = uS / US_ROUNDTRIP_CM;
  // Imprimir la distancia medida a la consola serial
  Serial.print("Distancia: ");
  // Calcular la distancia con base en una constante
  Serial.print(uS);
  Serial.println("cm"); 

  //Código para evitar obstaculo
  if(uS >0 && uS <= 5) {

    

    analogWrite(PwmI,105);
    analogWrite(PwmD,105);
    digitalWrite(LlantaIA, HIGH);
    digitalWrite(LlantaIR, LOW); 
    digitalWrite(LlantaDA, LOW);
    digitalWrite(LlantaDR, HIGH);

    delay(2000);

    analogWrite(PwmI,0);
    analogWrite(PwmD,0);
    digitalWrite(LlantaIA, LOW);
    digitalWrite(LlantaIR, LOW);
    digitalWrite(LlantaDA, LOW);
    digitalWrite(LlantaDR, LOW);

    delay(500);

    if(digitalRead(V2) == HIGH){
      Serial.print("Color: ");
      Serial.println(analogRead(COLOR));
      simuladorServo();
    }
  }
}

//Simulador de servo motor
void simuladorServo(){
  ps++;
  servo.write(180);
  delay(5000);
  servo.write(0);
  delay(5000);
}

//Movimiento motores en proteus
void todoMotores(){
  //Estabilizada
  if(SA==HIGH && SB==HIGH && SC==HIGH){
    //delay(70);
    if(Estabilizador==1){
       //Movimiento de stepper izq
       STPI.step(256);
       //Movimiento de stepper der
       STPD.step(-256);
       
       //delay(700);
    }
    if(Estabilizador==2){
       //Movimiento de stepper izq
       STPI.step(-256);
       //Movimiento de stepper der
       STPD.step(256);
       
       //delay(700);
    }
  }
  //Estabilizada

  //Sin sensor
  if(SA==LOW && SB==LOW && SC == LOW){
    if(Estabilizador==1){
      //Movimiento de stepper izq
      STPI.step(-256);
      //Movimiento de stepper der
      STPD.step(-256);
      
    }
    if(Estabilizador==2){
      //Movimiento de stepper izq
      STPI.step(256);
      //Movimiento de stepper der
      STPD.step(256);
    }
  }  
  //Sin sensor

  //SensorCentral
  if(SA==LOW && SB==HIGH && SC==LOW){
    //Movimiento de stepper izq
    STPI.step(256);
    //Movimiento de stepper der
    STPD.step(256);

    //delay(300);
  }
  //FinSensorCentral

  //SensorCentralIzquierda
  if(SA==HIGH && SB==HIGH && SC==LOW){
    //Movimiento de stepper izq
    STPI.step(128);
    //Movimiento de stepper der
    STPD.step(256);
    
    //delay(100);
    
    //Movimiento de stepper izq
    STPI.step(128);
    //Movimiento de stepper der
    STPD.step(256);
  }
  //FinSensorCentralIzquierda


  //SensorCentralDerecha
  if(SA==LOW && SB==HIGH && SC==HIGH){
    //Movimiento de stepper izq
    STPI.step(256);
    //Movimiento de stepper der
    STPD.step(128);
    
    //delay(100);
    
    //Movimiento de stepper izq
    STPI.step(256);
    //Movimiento de stepper der
    STPD.step(128);
  }
  //FinSensorCentralDerecha


  //SensorIzquierda
  if(SA==HIGH && SB==LOW && SC==LOW){
     //Movimiento de stepper izq
     STPI.step(64);
     //Movimiento de stepper der
     STPD.step(512);  
  }
  //FinSensorIzquierda

  //SensorDerecha
  else if(SA==LOW && SB==LOW && SC==HIGH){
     //Movimiento de stepper izq
     STPI.step(512);
     //Movimiento de stepper der
     STPD.step(64);
  }
  //SensorDerecha
  
}

void todoTurtle(){
  //LecturaSensores
    actualizar_tablero();
    SA = digitalRead(Si);// izquierda
    SB = digitalRead(Sc); // centro
    SC = digitalRead(Sd); // derecha
  //FinLecturaSensores
  
  //Estabilizada
  if(SA==HIGH && SB==HIGH && SC==HIGH){
    //delay(70);
    mover_carrito(0);
    if(Estabilizador==1){
       analogWrite(PwmI,80);
       analogWrite(PwmD,85);

       digitalWrite(LlantaIA, HIGH);
       digitalWrite(LlantaIR, LOW); 

       digitalWrite(LlantaDA, LOW);
       digitalWrite(LlantaDR, HIGH);
       
       Referencia = 0;
       Graduador = 0;
       delay(700);

       //Movimiento de stepper izq
       //STPI.step(256);
       //Movimiento de stepper der
       //STPD.step(-256);
    }
    if(Estabilizador==2){
       analogWrite(PwmI,85);
       analogWrite(PwmD,80);

       digitalWrite(LlantaIA, LOW);
       digitalWrite(LlantaIR, HIGH); 
  
       digitalWrite(LlantaDA, HIGH);
       digitalWrite(LlantaDR, LOW);
       
       Referencia = 0;
       Graduador = 0;
       delay(700);

       //Movimiento de stepper izq
       //STPI.step(-256);
       //Movimiento de stepper der
       //STPD.step(256);
    }
  }

  if(SA==LOW && SB==LOW && SC == LOW){/// izq = off, centro = iff, der = off
    if(Estabilizador==1){
      analogWrite(PwmI, 65);
      analogWrite(PwmD, 50);
      
      digitalWrite(LlantaIA, LOW);
      digitalWrite(LlantaIR, HIGH);

      digitalWrite(LlantaDA, LOW);
      digitalWrite(LlantaDR, HIGH);
      Referencia=0;
      Graduador =0;
      delay(15);

      //Movimiento de stepper izq
      //STPI.step(-256);
      //Movimiento de stepper der
      //STPD.step(-256);
    }
    if(Estabilizador==2){
      analogWrite(PwmI, 50);
      analogWrite(PwmD, 65);
     
      //FinVelocidad
      digitalWrite(LlantaIA, LOW);
      digitalWrite(LlantaIR, HIGH);

      digitalWrite(LlantaDA, LOW);
      digitalWrite(LlantaDR, HIGH);
      Referencia = 0;
      Graduador = 0;
      delay(15);

      //Movimiento de stepper izq
      //STPI.step(256);
      //Movimiento de stepper der
      //STPD.step(256);
    }
  }
  
  //SensorCentral
  if(SA==LOW && SB==HIGH && SC==LOW){
    //ConfiguracionVelacidad
      mover_carrito(0);

      analogWrite(PwmI,100);
      analogWrite(PwmD,100);

      digitalWrite(LlantaIA,HIGH);
      digitalWrite(LlantaIR,LOW);

      digitalWrite(LlantaDA,HIGH);
      digitalWrite(LlantaDR,LOW);

      Referencia = 0;
      Graduador = 0;
      Estabilizador = 0;
      Direccion = 0;

      //Movimiento de stepper izq
      //STPI.step(256);
      //Movimiento de stepper der
      //STPD.step(256);
  }
  //FinSensorCentral

  //SensorCentralIzquierda
  if(SA==HIGH && SB==HIGH && SC==LOW){
    mover_carrito(grados_m++);
    
    analogWrite(PwmI,30);
    analogWrite(PwmD,200+Graduador);

    digitalWrite(LlantaIA, HIGH);
    digitalWrite(LlantaIR, LOW);

    digitalWrite(LlantaDA, HIGH);
    digitalWrite(LlantaDR, LOW);
    
    delay(100);

    //Movimiento de stepper izq
    //STPI.step(128);
    //Movimiento de stepper der
    //STPD.step(256);
    
    //ConfiguracionVelocidad
    analogWrite(PwmI, 50);
    analogWrite(PwmD, 100+Graduador);

    digitalWrite(LlantaIA, HIGH);
    digitalWrite(LlantaIR, LOW);

    digitalWrite(LlantaDA, HIGH);
    digitalWrite(LlantaDR, LOW);
    
    Estabilizador = 1; 

    //Movimiento de stepper izq
    //STPI.step(128);
    //Movimiento de stepper der
    //STPD.step(256);
  }
  //FinSensorCentralIzquierda

  //SensorCentralDerecha
  if(SA==LOW && SB==HIGH && SC==HIGH){
      mover_carrito(grados_m++);
      
      analogWrite(PwmI, 200+Graduador);
      analogWrite(PwmD, 30);

      digitalWrite(LlantaIA,HIGH);
      digitalWrite(LlantaIR,LOW);

      digitalWrite(LlantaDA, HIGH);
      digitalWrite(LlantaDR, LOW);
      
      Referencia = 0;
      delay(100);

      //Movimiento de stepper izq
      //STPI.step(256);
      //Movimiento de stepper der
      //STPD.step(128);
      
      //ConfiguracionVelocidad
      analogWrite(PwmI, 100+Graduador);
      analogWrite(PwmD, 50+Graduador);

      digitalWrite(LlantaIA, HIGH);
      digitalWrite(LlantaIR, LOW);

      digitalWrite(LlantaDA, HIGH);
      digitalWrite(LlantaDR, LOW);
      
      Referencia = 0;
      Estabilizador = 2; 

      //Movimiento de stepper izq
      //STPI.step(256);
      //Movimiento de stepper der
      //STPD.step(128);
  }
  //FinSensorCentralDerecha


  //SensorIzquierda
  if(SA==HIGH && SB==LOW && SC==LOW){
      mover_carrito(grados_m++);
      //ConfiguracionVelocidad
      analogWrite(PwmI, 40);
      analogWrite(PwmD, 145+Graduador);

      digitalWrite(LlantaIA, LOW);
      digitalWrite(LlantaIR, HIGH);
      
      digitalWrite(LlantaDA,HIGH);
      digitalWrite(LlantaDR,LOW);
      
      Estabilizador = 1;

      //Movimiento de stepper izq
      //STPI.step(256);
      //Movimiento de stepper der
      //STPD.step(128);
  }
  //FinSensorIzquierda


  //SensorDerecha
  if(SA==LOW && SB==LOW && SC==HIGH){
    mover_carrito(grados_m++);
    
    analogWrite(PwmI, 145+Graduador);
    analogWrite(PwmD, 40);

    digitalWrite(LlantaIA, HIGH);
    digitalWrite(LlantaIR, LOW);
    
    digitalWrite(LlantaDA, LOW);
    digitalWrite(LlantaDR, HIGH);
    
    Estabilizador = 2; //Izuierda

    //Movimiento de stepper izq
    //STPI.step(512);
    //Movimiento de stepper der
    //STPD.step(64);
  }
}
