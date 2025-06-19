#include <Servo.h>

Servo ServoDerecha;
Servo ServoIzquierda;

//Inicializacion hardware
int servomotorDerechaPin = 10;
int servomotorIzquizerdaPin = 6;
int sensorPin = A0; 

int maxAngulo = 165;
int minAngulo = 10;

int posicionServoDerecha = minAngulo;
int posicionServoIzquierda = maxAngulo;

unsigned long contador = 0;
int velocidadD = 10;
int objetivoAnguloD = minAngulo;
int velocidadI = 10;
int objetivoAnguloI = maxAngulo;
int ultimaValor = 0;
int modo = 0;
int valorSensor = 0;
int mapSensorValor = 0;
int anguloTecho = 30; //valor minima de diferencia entre un angulo y el objetivo para que los motores funcionan

int ultimaMapValor = minAngulo;


// bloquaje
int utlimaValor = 0;
bool cambioEstado = false;
bool bloqueo = false;
int sobreLimita = 0;
int umbral1 = 610;

//Operacional task a 20 y 200 ms
int os20 = 20;
int os100 = 100;


//Control de la deteccion de señal
unsigned long T0 = 0;
unsigned long T1 = 0;
unsigned long T2 = 0;
int contraccion = 0;

// Detection de Patern et changement de mode
int calibrationDuree1 = 3000;
int calibrationDuree2 = 2000;
int stepDetection = 0;


void setup() {
  // inicializacion de los motores y del pin de entrade de la señal
  Serial.begin(115200);
  ServoDerecha.attach(servomotorDerechaPin); 
  ServoIzquierda.attach(servomotorIzquizerdaPin);
  pinMode(sensorPin,INPUT);

  // Posicion initcial de los motores a 0 grados
  ServoDerecha.write(posicionServoDerecha);
  ServoIzquierda.write(posicionServoIzquierda);
  digitalWrite(A0, LOW);

}

void loop() {

  contador = millis();
  // ##################### Tarea de operaciones #################
    // Tarea cada 20 ms
  if (contador % os20 == 0) {
    valorSensor = analogRead(sensorPin); //leemos la valor del detector
    Serial.println(valorSensor);
    assertValue(valorSensor, 0, 675, "Error Valor sensor ");
    mapSensorValor = tratamientoSenal(valorSensor); //tratamos la señal
  }

  // Tarea cada 100 ms
  if (contador % os100 == 0) {
    deteccion(); //Detectamos la paterna
    controlFuncion(valorSensor, mapSensorValor);//actualizamos los valores dados a los motores
  }
      // ###################### Pilotage Motor #######################
  if (contador % velocidadD == 0) {
   //Serial.println("DROIT --> " + String(objetivoAnguloD) + " POS " + String(posicionServoDerecha));
    motorDerecha(objetivoAnguloD);
  }
  Serial.println(objetivoAnguloD);

  if (contador % velocidadI == 0) {
    //Serial.println("GAUCHE -->" + String(objetivoAnguloI)+ " POS " + String(posicionServoIzquierda));
    motorIzquierda(objetivoAnguloI);
  }
  
 
}

int tratamientoSenal(int sV) {
  Serial.println("sV" +String(sV));

  int mapValor = map(sV, 0, 675, minAngulo, maxAngulo);
 // Serial.println("map valor " +String(mapValor) + " min angulo " +String(minAngulo) + " max angulo "+ String(maxAngulo));

  assertValue(mapValor, minAngulo, maxAngulo, "ERROR map valor ");
  if (abs(ultimaMapValor - mapValor) > anguloTecho){
    ultimaMapValor = mapValor;
    Serial.println("msv" +String(mapValor));
    return mapValor;
  }
  else{
    return ultimaMapValor;
  }
}

void motorDerecha(int ObjAngD) {
  if (posicionServoDerecha < ObjAngD) {
    Serial.println("posicion d" +String(posicionServoDerecha));
    Serial.println("angle d" + String(ObjAngD));

    posicionServoDerecha++;
    assertValue(posicionServoDerecha, minAngulo, maxAngulo, "ERROR posicion Servo Derecha ");
    ServoDerecha.write(posicionServoDerecha);
  }
  else if (posicionServoDerecha > ObjAngD) {
    Serial.println(posicionServoDerecha);
    posicionServoDerecha--;
    assertValue(posicionServoDerecha, minAngulo, maxAngulo, "ERROR posicion Servo Derecha ");
    ServoDerecha.write(posicionServoDerecha);
  }
}

void motorIzquierda(int ObjAngI) {
  if (posicionServoIzquierda < ObjAngI) {
    //Serial.println("on est rentré à gauche d'un angle inférieur");
    posicionServoIzquierda++;
    assertValue(posicionServoIzquierda, minAngulo, maxAngulo, "ERROR posicion Servo izquierda ");
    ServoIzquierda.write(posicionServoIzquierda);
  }
  else if (posicionServoIzquierda > ObjAngI) {
    //Serial.println("on est rentré à gauche d'un angle supérieur");
    posicionServoIzquierda--;
    assertValue(posicionServoIzquierda, minAngulo, maxAngulo, "ERROR posicion Servo izquierda ");
    ServoIzquierda.write(posicionServoIzquierda);
  }
}

void deteccion(){
  /*if (contador%2000 == 0){
    Serial.println("Contraccion info " +  String(contraccion)+ " Pattern detection step  --> " + String(stepDetection) + "Le mode est " + String(modo) + "valeur sensor " + String(valorSensor));
  }*/
  int ultimaContraccion = contraccion;
  boolean flancoAscendente = false;
  boolean flancoDescendente = false;

  if (valorSensor < 100){ //attribuamos 3 valores a contraccion segun la intensidad
    contraccion = 0;
  }
  else if (valorSensor > 600){
    contraccion = 2;
  }
  else {
    contraccion = 1;
  }
  
 // ######### deteccion fronts ############## 
  if (ultimaContraccion == 1 && contraccion == 2){ //si estabamos en un estado de transicion y que detectamos una contraccion superior a 600
    flancoAscendente = true; //detectamos un frente subiendo de un PMW
    //Serial.println("on a détecté un front montant");
  }
  if (ultimaContraccion == 1 && contraccion == 0){ //si estabamos en un estado de transicion y que detectamos una contraccion inferior a 100
    flancoDescendente = true; //detectamos un frente bajando de un PMW
    //Serial.println("on a détecté un front descendant");
  }
   // ######### Patern detection ##############
  if (stepDetection == 0  && flancoAscendente) { //si somos a la primera etapa de la deteccion y que detectamos un frente subiendo
    T0 = contador; //Guardamos el "tiempo" al momento que detectamos
    stepDetection = 1; //Pasamos a la segunda etapa
    Serial.println("on a le premier front montant à " + String(T0) + "contraction " + String(valorSensor) + "temps " + String(contador));
  }
  //--------------------
  if ((stepDetection == 1)  && (contador - T0 > calibrationDuree1)) { // Si estamos a la segunda etapa pero que el tiempo entre la proxima etapa y la ultima es demasiado largo
    stepDetection = 0; //Volvemos a la primera etapa
    Serial.println("on réénitialise premiere etape trop longue car " + String(contador-T0));
  }
  //--------------------
  if ((stepDetection == 1  && flancoDescendente) && (contador - T0 < calibrationDuree1)) { //Si somos a la segunda etapa de la deteccion en el buen tiempo y que detectamos un frente bajando 
    T1 = contador; //Guardamos el "tiempo" al momento que detectamos
    stepDetection = 2; //Pasamos a la tercera etapa 
    Serial.println("on a le premier front descendant à " + String(T1) + "contraction " + String(valorSensor) + "temps " + String(contador));
  }
  //--------------------
  if ((stepDetection == 2  && flancoAscendente) &&  (contador - T1 < calibrationDuree2)) { //Si somos a la tercera etapa de la deteccion en el buen tiempo y que detectamos un frente subiendo 
    T2 = contador; //Guardamos el "tiempo" al momento que detectamos
    stepDetection=3; //Pasamos a la tercera etapa
    Serial.println("on a le deuxieme front montant");
  }
  //--------------------
  if ((stepDetection == 2)  && (contador - T1 > calibrationDuree2)) {// Si estamos a la tercera etapa pero que el tiempo entre la proxima etapa y la ultima es demasiado largo
    stepDetection = 0; //Volvemos a la primera etapa
    Serial.println("on réénitialise deuxieme etape trop longue car " + String(contador-T1));
  }
  //--------------------
  if ((stepDetection == 3  && flancoDescendente) && (contador - T2 < calibrationDuree1)) { //Si somos a la quartacera etapa de la deteccion en el buen tiempo y que detectamos un frente bajando 
    stepDetection = 0; // alcanzamos la ultima etapa, inicializamos de nuevo la deteccion
    Serial.println("on a le deuxieme front descendant : contraction" + String(valorSensor) + "temps " + String(contador));
    if (modo >= 1){
      modo = 0;
    }
    else {
      modo++;
    }
    Serial.println(String(modo));
  }
  //--------------------
  if ((stepDetection == 3)  && (contador - T2 > calibrationDuree1)) {
    // Patern non reconnu -> reset
    stepDetection = 0;
    Serial.println("on réénitialise troisieme etape trop longue car" + String(contador-T2));
  }
 
}


// Función para verificar si un valor está fuera de un rango y mostrar un mensaje si es necesario
void assertValue(int X, int min, int max, String message) {
  // Comprobar si X es menor que min o mayor que max
  if ((X < min) || (X > max)) {
    Serial.println("Tempo: "+ String(contador) + "->" + message + " Valor: " + String(X) + " min: " + String(min) + " max: " + String(max));
  }
}

void controlFuncion(int sV, int mSV) {
  //Serial.println(String(sV));
  //Serial.println(String(mSV));
  if (ultimaValor - umbral1 <= 0 && sV - umbral1 >= 0)  { //detectamos cuando sobre pasamos el umbral
    cambioEstado = true; // es un cambio de estado
    sobreLimita += 1; //permite de saber si estamos bloqueado o no
  }
  else{
    cambioEstado = false;
  }
  if (cambioEstado){ // Si cambiamos de estado
    if (sobreLimita % 2 == 0){ //si es par, es que debemos desbloquear
      bloqueo = false;
      Serial.println("débloqué");
      Serial.println(String(sV));

    }
    else { //si es impar debemos bloquear
      bloqueo = true;
      Serial.println("bloqué");
      Serial.println(String(sV));

    }
  }
  ultimaValor = sV; //Guardamos la ultima valor

  if (modo == 0) { //primer modo : todos los dedos se contractan
    velocidadI = 15; 
    velocidadD = 15;
    if (bloqueo){ //si estamos en un estado bloqueado
      objetivoAnguloI = minAngulo; //Contractamos a los maximos los dedos de la parte izquierda
      objetivoAnguloD = maxAngulo; //Contractamos a los maximos los dedos de la parte derecha
    }
    else {
      objetivoAnguloI = maxAngulo - mSV;//contractamos con una valor que corresponda a la intensidad de la contractaccion
      objetivoAnguloD = mSV;//contractamos con una valor que corresponda a la intensidad de la contractaccion
    }
  }
  if (modo == 1) {
    velocidadI = 15;
    velocidadD = 15;
    if (bloqueo){//si estamos en un estado bloqueado
      objetivoAnguloI = minAngulo;//no activamos los dedos de la parte izquierda
      objetivoAnguloD = minAngulo; //Contractamos a lo maximum los dedos de la parte derecha (index y pulgar)
    }
    else {
      objetivoAnguloI = maxAngulo - mSV;//no activamos los dedos de la parte izquierda
      objetivoAnguloD = minAngulo; //contractamos con una valor que corresponda a la intensidad de la contractaccion
    }
  }

}
