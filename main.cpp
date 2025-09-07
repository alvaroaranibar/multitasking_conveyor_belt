#include <Arduino.h>
#include <ESP32Servo.h>
#include <Robojax_L298N_DC_motor.h>
#include <cstdint>

#define TASA_CAIDA_SERVO 1250   //RETARDO ENTRE OBJETOS DE 1.25 SEGUNDOS (CADA OBJETO CAE CADA 1.25 SEGUNDO)

// Constantes de color: cada numero representa un color
#define   R          1
#define   G          2
#define   B          3
#define   W          4 //Blanco es el color defectuoso

/*****************************************PINES SENSOR DE PROXIMIDAD********************************************************/
#define   PROX_COLOR    39
#define   PROX_RECHAZO  34
#define   PROX_CAIDA    35
#define   PROX_CAJA     21

/************************************************** MOTORES Y SERVOS********************************************************/

// Objetos de libreria <ESP32Servo.h>
Servo myservoR;
Servo myservoG;
Servo myservoB;
Servo myservo_rechaza;
void config_servos(){
  // Allow allocation of all timers
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
  // Periodos de todos los servos es 50 Hz
	myservoR.setPeriodHertz(50);    
  myservoG.setPeriodHertz(50);    
  myservoB.setPeriodHertz(50);    
  myservo_rechaza.setPeriodHertz(50);
  // PINES (Se usa min/max por defecto de 1000us and 2000us para MG995, SG90-360)
  myservoR.attach(32);
  myservoG.attach(33);
  myservoB.attach(25);
  myservo_rechaza.attach(26);
  // Se inicializan actuadores lineales (360°) parados
	myservoR.write(94); // Nota SG90-360: Apagado en 92°,96°
  myservoG.write(94);
  myservoB.write(94);
  // Se inicializa servo 180 en 0° (Sin rechazar)
  myservo_rechaza.write(0);
}
// PINES MOTOR DC para libreria <Robojax_L298N_DC_motor.h>
// motor 1 settings
#define CHA 14  //Selecciona canal de PWM a usar (16 disponibles). Se usan 2 ultimos para que no haya conflicto
#define ENA 19 //PWM: ENA = PWMA
#define IN1 18  // IN1 = AI1
#define IN2 17   // IN2 = AI2
// motor 2 settings
#define IN3 16  // IN3 = BI1
#define IN4 4  // IN4 = BI2
#define ENB 2 // PWM: ENB = PWMB
#define CHB 15
// Otras constantes <Robojax_L298N_DC_motor.h>
const int CCW = 2; // do not change
const int CW  = 1; // do not change
#define motor1 1 // do not change
#define motor2 2 // do not change
Robojax_L298N_DC_motor robot(IN1, IN2, ENA, CHA,  IN3, IN4, ENB, CHB); // Configura Driver TB6612FNG 


/************************************************** QUEUE FIFO********************************************************/
/*Recibe pedidos de 5 elementos (Pueden tomar 0,1,2,3) del terminal serial periodicamente y los almacena en la QUEUE 
  Si faja no esa ocupada actualiza el arreglo secuencia_actual con el siguiente pedido*/
#define ARRAY_SIZE 5 //Numero de elementos maximos por pedido

int faja_ocupado = 0;    //Se pone a 1 cada vez que la faja este despachando un producto (ocupado) (solo se despecha uno por vez)
                          //Se inicializa con 0: no esta ocupada
uint8_t secuencia_actual[5] = {0, 0, 0, 0, 0};  //Almacena la secuencia actual de productos en orden
                                                //solo se actualiza de queue cuando faja no este ocupada

int flag_arr_terminal = 0; //1 si se ingreso arreglo en terminal  y esta a la espera de ser enviado
                      //0 si el arreglo fue enviado y no hay arreglo en terminal
// Arreglo para almacenar los números ingresados
uint8_t secuencia_terminal[ARRAY_SIZE];

void readSerialTask(void *parameter) {
  char inputBuffer[ARRAY_SIZE + 1]; // Buffer para la entrada del serial, +1 para el carácter nulo
  int index = 0;

  while (1) {
    // Leer datos del serial si hay disponibles
    while (Serial.available() > 0) {
      char received = Serial.read();
      if (received == '\n' || received == '\r') {
        // Ignorar caracteres de nueva línea adicionales
        if (index == 0) {
          continue;
        }
        inputBuffer[index] = '\0'; // Terminar el string
        // Mostrar el buffer recibido para depuración
        Serial.print("Received: ");
        Serial.println(inputBuffer);

        // Verificar que la longitud del input es correcta
        if (index == ARRAY_SIZE) {
          bool validInput = true;
          for (int i = 0; i < ARRAY_SIZE; i++) {
            if (inputBuffer[i] < '0' || inputBuffer[i] > '3') {
              validInput = false;
              break;
            }
          }
          if (validInput) {
            // Convertir los caracteres a enteros y almacenarlos en el arreglo
            for (int i = 0; i < ARRAY_SIZE; i++) {
              secuencia_terminal[i] = inputBuffer[i] - '0'; // Convertir char a int
            }
            // Mostrar el nuevo arreglo
            Serial.print("New array: ");
            for (int i = 0; i < ARRAY_SIZE; i++) {
              Serial.print(secuencia_terminal[i]);
              if (i < ARRAY_SIZE - 1) {
                Serial.print(", ");
              }
            }
            Serial.println();
            flag_arr_terminal = 1;
          } else {
            Serial.println("Error: Numbers must be between 0 and 3.");
          }
        } else {
          Serial.println("Error: Please enter exactly 5 numbers.");
        }
        // Reiniciar el índice para la próxima lectura
        index = 0;
      } else {
        // Almacenar el carácter recibido en el buffer
        if (index < ARRAY_SIZE) {
          inputBuffer[index++] = received;
        }
      }
    }

    // Retardo de la tarea para no bloquear el procesador
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

TaskHandle_t task_send_Handle1 = NULL;
TaskHandle_t task_receive_Handle1 = NULL;
QueueHandle_t queue1;

void task_send1(void *arg) {
  while (1) {
    if (flag_arr_terminal == 1) {   // Si hay arreglo en terminal
      if (xQueueSend(queue1, (void *)secuencia_terminal, pdMS_TO_TICKS(100)) == pdPASS) {
        Serial.print("Enviado a queue: ");                                                               
        // Imprimimos los elementos del arreglo y los limpiamos
        for (int i = 0; i < ARRAY_SIZE; i++) {
          Serial.print(secuencia_terminal[i]);
          secuencia_terminal[i] = 0;
        }
        Serial.println();
        flag_arr_terminal = 0; // Arreglo de terminal enviado y limpiado
      } else {
        Serial.println("Error: no se pudo enviar a la cola");
      }
    }
    vTaskDelay(pdMS_TO_TICKS(300)); // Se leera terminal cada 300 ms
  }
}
uint8_t parada_por_sensar_color = 0;
uint8_t parada_por_sensar_caja = 0;

//Se lee de queue, escribe en secuencia_actual y se activan servos 360 solo si faja_ocupado==0 (faja no esta ocupada)
void task_receive1(void *arg) {
  while (1) {
    if (faja_ocupado == 0) {  
      if (xQueueReceive(queue1, &(secuencia_actual), pdMS_TO_TICKS(100)) == pdPASS) { // Se escribio exitosamente en secuencia actual?
        faja_ocupado = 1; //Cuando servos envien el pedido, faja estara ocupada
                          //Solo cuando pedidos lleguen a caja de segunda faja se desocupa
        robot.rotate(motor1, 100, CW);    //Se enciende motor 1
        int i = 0;
        while (secuencia_actual[i] != 0) {
          if (secuencia_actual[i] == R) {
            myservoR.write(180);              
            vTaskDelay(pdMS_TO_TICKS(420));                       
            myservoR.write(0);              
            vTaskDelay(pdMS_TO_TICKS(440)); // Se le da mas tiempo de regreso para compensar desplazamiento de servo (Existe tope que limita)            
            myservoR.write(94);
          } else if (secuencia_actual[i] == G) {
            myservoG.write(180);              
            vTaskDelay(pdMS_TO_TICKS(440));                       
            myservoG.write(0);              
            vTaskDelay(pdMS_TO_TICKS(470));                      
            myservoG.write(94);
          } else if (secuencia_actual[i] == B) {
            myservoB.write(180);              
            vTaskDelay(pdMS_TO_TICKS(420));                       
            myservoB.write(0);              
            vTaskDelay(pdMS_TO_TICKS(440));                      
            myservoB.write(94);
          } else { // En cualquier otro caso se paran los 3 servomotores
            myservoR.write(94);
            myservoG.write(94);
            myservoB.write(94);
          }
          vTaskDelay(pdMS_TO_TICKS(TASA_CAIDA_SERVO)); //RETARDO ENTRE OBJETOS DE 1.5 SEGUNDOS (CADA OBJETO CAE CADA 1. SEGUNDO)
          i++; // Avanza al siguiente elemento del arreglo
          while( (parada_por_sensar_caja==1)||(parada_por_sensar_color==1)){ //Si hay algun tipo de parada no entrega sgte objeto
            vTaskDelay(pdMS_TO_TICKS(100));
          }
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(300)); // Se verificara faja_ocupado cada 300ms
  }
}

//////////////////////////SENSOR COLOR/////////////////////////////
#define S0  23 
#define S1  22
#define OUT 36 
#define S2  27
#define S3  14

void config_sensor_color(){
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(OUT, INPUT);
  digitalWrite(S0, HIGH);
  digitalWrite(S1, HIGH);
  //Alternativamente: Al 20% f. scale, tambien se podria pone delay fijo en pulseIn
  //digitalWrite(PIN_S0,HIGH);
  //digitalWrite(PIN_S1,LOW);
}

u_int8_t sensa_color (){
  int countRed = 0;
  int countGreen = 0;
  int countBlue = 0;
  //Se repite la medicion 5 veces, aqui se registran las veces que se detectaron color_proximo, gana el que es >=3 veces
  int veces_color_sensado[4] = {0, 0, 0, 0}; //Cuarto elemento es para cuenta de blanco
  for (int i = 1; i <= 5; i++){
    // Mide cuenta de frecuencia de cada color
    digitalWrite(S2, LOW);
    digitalWrite(S3, LOW);
    countRed = pulseIn(OUT, digitalRead(OUT) == HIGH ? LOW : HIGH);
    digitalWrite(S2,LOW);
    digitalWrite(S3,HIGH);
    countBlue = pulseIn(OUT, digitalRead(OUT) == HIGH ? LOW : HIGH);
    digitalWrite(S2,HIGH);
    digitalWrite(S3,HIGH);
    countGreen = pulseIn(OUT, digitalRead(OUT) == HIGH ? LOW : HIGH);
    //Muestra cuenta de colores (Solo para depurar)
    Serial.print("Red: ");
    Serial.print(countRed);
    Serial.print("Green: ");
    Serial.print(countGreen);
    Serial.print("Blue: ");
    Serial.print(countBlue);
    //Notar la prioridad de negro y blanco al inicio
    if (countRed>4500 && countGreen>4500 && countBlue >4500){  //Detecta negro y no registra nada
      Serial.println(" - Black");                              //Nunca deberia pasar si se pone objeto en el sensor adecuadamente
    }
    else if (countRed<425 && countGreen<425 && countBlue<425){
      Serial.println("White");
      veces_color_sensado[W - 1]++;
    }
    else if (countRed < countBlue && countRed < countGreen){
      Serial.println(" - Red");
      veces_color_sensado[R - 1]++;
    }
    else if (countGreen < countRed && countGreen < countBlue){
      Serial.println(" - Green");
      veces_color_sensado[G - 1]++;
    }
    else if (countBlue < countRed && countBlue < countGreen){
      Serial.println(" - Blue");
      veces_color_sensado[B - 1]++;
    }
    else{
      Serial.println("No detecto"); //Solo para depuración, se quita despues, se asume que no falla
    }
    vTaskDelay(pdMS_TO_TICKS(100)); //Delay para estabilizar sensor, Se puede aumentar este delay. Notar que 5 mediciones 500ms o mas
    //Este delay actua sobre task_unamed
  }
  //Se busca el color que se registro mas de 3 veces
  for (int i = 0; i < 4; ++i) {
    if (veces_color_sensado[i] >= 3) {
      //Todos estos serial.print solo para depuracion
      if(i==0){
        Serial.print("Color proximo: Red");
      }
      else if (i==1){
        Serial.print("Color proximo: Green");
      }
      else if (i==2){
        Serial.print("Color proximo: Blue");
      }
      else if (i==3){
        Serial.print("Color proximo: White");
      }
      return i + 1; //Notar: Indice+1 = codigo de color sensado
    }
    veces_color_sensado[i] = 0; //Se limpia arreglo al final de la medicion
  }

  return 0;           //Retorna 0 para salir de la funcion pero nunca pasara si estra bien calibrado
}



///////////////////////////////////////////////////////////////////////////////////
uint8_t arr_cuenta_rechazados[4]={0,0,0,0}; //Cada fila reprsenta la cuenta de un color RGBW
uint8_t necesita_repuestos = 0;
uint8_t cuenta_correctos = 0;
uint8_t color_sensado=0;
uint8_t cuenta_color=0;
uint8_t cuenta_caidos = 0;

void task_color_y_control_faja1y2 (void *arg){    //FALTA INICIALIZAR ESTA TASK
  while(1){
    if ( (cuenta_caidos == cuenta_correctos) && (cuenta_caidos > 3) && (necesita_repuestos==0)){ //Este es prioritario sobre prox_caja
                                                                          //aqui se desplazara la caja y no espero que se ponga caja
      vTaskDelay(pdMS_TO_TICKS(200)); //Pequeño retardo para que termine de caer objeto antes de parar motor 1
      robot.brake(1); //Se volvera a reanudar cuando en recibe
      robot.rotate(motor2, 100, CCW);
      ///////////////////////////////////////////////////////se envia al sv
      for (int i = 0; i < 5;i++){ //Se limpia secuencia_actual
        secuencia_actual[i] = 0;
      }
      cuenta_color = 0;
      color_sensado = 0;
      cuenta_correctos = 0;
      cuenta_caidos = 0;
      vTaskDelay(pdMS_TO_TICKS(2000));//Se espera 2 seg para que se despache caja 
      robot.brake(2);
      faja_ocupado = 0;
    }
    //aqui abajo, ya no se usa && (faja_ocupado==1) &&?
    else if( (digitalRead(PROX_CAJA)==1) && (digitalRead(PROX_COLOR)==0)){    //Que no haya caja es prioritario sobre el color, si no hay caja, se para motor
                                      //y no se sensa color
      parada_por_sensar_caja = 1;

      Serial.print("HOLAHOLA HOLA");
      robot.brake(1);
      //Serial.println("parada de faja1");
      while (digitalRead(PROX_CAJA)==1){ //BUCLE DE ESPERA hasta que haya caja, solo para esta task, tareas corriendo: task_cuenta_caidos, send y desactiva servo_rechazo
        vTaskDelay(pdMS_TO_TICKS(80));    //Permite ir a otras tareas
      }
      parada_por_sensar_caja = 0;
      Serial.print("Se puso a 0 parada");
      
      robot.rotate(motor1, 100, CW);    //Se reanuda motor 1 si hay caja
    }
    
    else if(digitalRead(PROX_COLOR)==0){ 
      //se parar todas task (excepto registro de pedidos) creo que no, intentare hacerlo secuencial
      vTaskDelay(pdMS_TO_TICKS(100)); //Retardo para que se centre la caja (calibrar)
      parada_por_sensar_color = 1;
      robot.brake(1);
      color_sensado = sensa_color();//Funcion que tiene vtaskdelay()
      if (color_sensado != secuencia_actual[cuenta_color]){
        secuencia_actual[cuenta_color] = W;   ///modifica arreglo para indicar que se tiene un rechazado
        arr_cuenta_rechazados[color_sensado-1]++;
        necesita_repuestos = 1;      
        myservo_rechaza.write(90); //Se activa servo de rechazo
      }
      else // si color sensado es el esperado
      {
        cuenta_correctos++;
        Serial.print("cuenta_correctos: ");
        Serial.println(cuenta_correctos);
      }
      cuenta_color++;
      
      robot.rotate(motor1, 100, CW);    //Se reanuda motor cuando se termine de sensar color
      parada_por_sensar_color = 0;
      vTaskDelay((pdMS_TO_TICKS(400)));  //Delay para que bloque salga de la caja hasta hacer sgte lectura
    }
    vTaskDelay(pdMS_TO_TICKS(40)); // Se sensa PROX_COLOR y PROX_CAJA cada 80 ms (calibrar)
  }
}

void task_cuenta_caidos(void *arg){
  while(1){
    if ((digitalRead(PROX_CAIDA) == 0) && (parada_por_sensar_color == 0) && (parada_por_sensar_caja == 0))
    {
      cuenta_caidos++;
      Serial.print("cuenta_caidos: ");
      Serial.println(cuenta_caidos);
    }
    vTaskDelay(pdMS_TO_TICKS(210));  //NORTA DEBE SER MENOR A 300
  }
}

void task_desactiva_rechazo(void *arg){
  while(1){
    if(digitalRead(PROX_RECHAZO)==0){
      myservo_rechaza.write(0);   //Se desactiva servo de rechazo cada vez que caiga caja de rechazo
    }
    vTaskDelay(pdMS_TO_TICKS(80)); // Se sensa cada 80 ms (calibrar)
  }
}
////////////////////////////////////////////////////////////////////////////////



/*
void task_prueba (void *arg){
  while (1){
    Serial.print("Spaces available: ");
    Serial.println(uxQueueSpacesAvailable(queue1));
    vTaskDelay(pdMS_TO_TICKS(20000)); //20 segundo para que se envie pedido
    Serial.print("Spaces available: ");
    Serial.println(uxQueueSpacesAvailable(queue1));
    faja_ocupado = 0;
    vTaskDelay(pdMS_TO_TICKS(20000)); //20 segundos para que se envie segundo pedido
    Serial.print("Spaces available: ");
    Serial.println(uxQueueSpacesAvailable(queue1));
  }
}
*/

void setup() {
  Serial.begin(115200);
  config_servos(); // Configura Driver TB6612FNG 
  robot.begin(); // Configura Driver TB6612FNG
  robot.brake(1); //Se inicializa a ambos motores parados
  robot.brake(2);
  config_sensor_color();  //Configura sensor de color

  pinMode(PROX_COLOR, INPUT); //Configura sensor de proximidad color
  pinMode(PROX_RECHAZO, INPUT); //Configura sensor de proximidad rechazo
  pinMode(PROX_CAIDA, INPUT); //Configura sensor de proximidad caida
  pinMode(PROX_CAJA, INPUT); //Configura sensor de proximidad caja

  //Configura queue 1
  queue1 = xQueueCreate(30, sizeof(secuencia_terminal)); //Crea queue de 30 slots, 5 bytes cada slot
  xTaskCreate(task_send1, "task_send1", 4096, NULL, 3, &task_send_Handle1);
  xTaskCreate(task_receive1, "task_receive1", 4096, NULL, 3, &task_receive_Handle1);
  xTaskCreate(readSerialTask, "ReadSerial", 1024, NULL, 2, NULL);
  xTaskCreate(task_color_y_control_faja1y2, "controlfajas", 4096, NULL, 1, NULL);
  xTaskCreate(task_cuenta_caidos, "cuenta_caidos", 1024, NULL, 1, NULL);
  xTaskCreate(task_desactiva_rechazo, "desactiva_rechazo", 1024, NULL, 3, NULL);
}


void loop() {
  
}
