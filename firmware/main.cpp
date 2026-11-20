/*
 * Multitasking dual conveyor belt and rack system.
 * ESP32 + FreeRTOS: order queue, servo dispensers, color verification,
 * rejection of defective cubes and box dispatch on a second conveyor.
 */

#include <Arduino.h>
#include <ESP32Servo.h>
#include <Robojax_L298N_DC_motor.h>
#include <cstdint>

#define TASA_CAIDA_SERVO 1250   // Delay between dispensed cubes [ms]

// Color codes
#define   R          1
#define   G          2
#define   B          3
#define   W          4 // White = defective cube

/***************************************** PROXIMITY SENSOR PINS ************************************************/
#define   PROX_COLOR    39
#define   PROX_RECHAZO  34
#define   PROX_CAIDA    35
#define   PROX_CAJA     21

/************************************************** MOTORS AND SERVOS ********************************************/

// <ESP32Servo.h> objects
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
  // All servos run at 50 Hz
	myservoR.setPeriodHertz(50);
  myservoG.setPeriodHertz(50);
  myservoB.setPeriodHertz(50);
  myservo_rechaza.setPeriodHertz(50);
  // Pins (default min/max of 1000 us and 2000 us for MG995 and SG90-360)
  myservoR.attach(32);
  myservoG.attach(33);
  myservoB.attach(25);
  myservo_rechaza.attach(26);
  // 360° linear actuators start stopped
	myservoR.write(94); // SG90-360: stopped between 92° and 96°
  myservoG.write(94);
  myservoB.write(94);
  // 180° rejection servo starts at 0° (not rejecting)
  myservo_rechaza.write(0);
}
// DC motor pins for <Robojax_L298N_DC_motor.h>
// Motor 1
#define CHA 14  // PWM channel (16 available); the last two are used to avoid conflicts
#define ENA 19  // PWM: ENA = PWMA
#define IN1 18  // IN1 = AI1
#define IN2 17  // IN2 = AI2
// Motor 2
#define IN3 16  // IN3 = BI1
#define IN4 4   // IN4 = BI2
#define ENB 2   // PWM: ENB = PWMB
#define CHB 15
// Other <Robojax_L298N_DC_motor.h> constants
const int CCW = 2; // do not change
const int CW  = 1; // do not change
#define motor1 1 // do not change
#define motor2 2 // do not change
Robojax_L298N_DC_motor robot(IN1, IN2, ENA, CHA,  IN3, IN4, ENB, CHB); // TB6612FNG driver


/************************************************** FIFO QUEUE ***************************************************/
/* Orders of 5 elements (values 0-3) are read periodically from the serial terminal and stored in the queue.
   When the conveyor is free, secuencia_actual is updated with the next order. */
#define ARRAY_SIZE 5 // Maximum number of elements per order

int faja_ocupado = 0;    // 1 while the conveyor is dispatching an order (one order at a time)
                         // 0 = conveyor free
uint8_t secuencia_actual[5] = {0, 0, 0, 0, 0};  // Current order, in sequence
                                                // only updated from the queue when the conveyor is free

int flag_arr_terminal = 0; // 1 = an order was entered in the terminal and is waiting to be queued
                           // 0 = no pending order in the terminal
// Order entered in the terminal
uint8_t secuencia_terminal[ARRAY_SIZE];

void readSerialTask(void *parameter) {
  char inputBuffer[ARRAY_SIZE + 1]; // Serial input buffer, +1 for the null terminator
  int index = 0;

  while (1) {
    while (Serial.available() > 0) {
      char received = Serial.read();
      if (received == '\n' || received == '\r') {
        // Ignore extra newline characters
        if (index == 0) {
          continue;
        }
        inputBuffer[index] = '\0';
        Serial.print("Received: ");
        Serial.println(inputBuffer);

        // Check input length
        if (index == ARRAY_SIZE) {
          bool validInput = true;
          for (int i = 0; i < ARRAY_SIZE; i++) {
            if (inputBuffer[i] < '0' || inputBuffer[i] > '3') {
              validInput = false;
              break;
            }
          }
          if (validInput) {
            // Convert characters to integers
            for (int i = 0; i < ARRAY_SIZE; i++) {
              secuencia_terminal[i] = inputBuffer[i] - '0';
            }
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
        index = 0;
      } else {
        if (index < ARRAY_SIZE) {
          inputBuffer[index++] = received;
        }
      }
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

TaskHandle_t task_send_Handle1 = NULL;
TaskHandle_t task_receive_Handle1 = NULL;
QueueHandle_t queue1;

// Sends the order entered in the terminal to the queue
void task_send1(void *arg) {
  while (1) {
    if (flag_arr_terminal == 1) {
      if (xQueueSend(queue1, (void *)secuencia_terminal, pdMS_TO_TICKS(100)) == pdPASS) {
        Serial.print("Sent to queue: ");
        // Print and clear the order
        for (int i = 0; i < ARRAY_SIZE; i++) {
          Serial.print(secuencia_terminal[i]);
          secuencia_terminal[i] = 0;
        }
        Serial.println();
        flag_arr_terminal = 0;
      } else {
        Serial.println("Error: could not send to queue");
      }
    }
    vTaskDelay(pdMS_TO_TICKS(300)); // Check the terminal every 300 ms
  }
}
uint8_t parada_por_sensar_color = 0;
uint8_t parada_por_sensar_caja = 0;

// Reads the next order from the queue and drives the 360° servos, only when the conveyor is free
void task_receive1(void *arg) {
  while (1) {
    if (faja_ocupado == 0) {
      if (xQueueReceive(queue1, &(secuencia_actual), pdMS_TO_TICKS(100)) == pdPASS) {
        faja_ocupado = 1; // Busy until the order reaches the box on the second conveyor
        robot.rotate(motor1, 100, CW);    // Start conveyor 1
        int i = 0;
        while (secuencia_actual[i] != 0) {
          if (secuencia_actual[i] == R) {
            myservoR.write(180);
            vTaskDelay(pdMS_TO_TICKS(420));
            myservoR.write(0);
            vTaskDelay(pdMS_TO_TICKS(440)); // Longer return time to compensate servo drift (mechanical stop)
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
          } else { // Any other value stops the three servos
            myservoR.write(94);
            myservoG.write(94);
            myservoB.write(94);
          }
          vTaskDelay(pdMS_TO_TICKS(TASA_CAIDA_SERVO)); // Delay between cubes
          i++;
          while( (parada_por_sensar_caja==1)||(parada_por_sensar_color==1)){ // Hold the next cube while the conveyor is stopped
            vTaskDelay(pdMS_TO_TICKS(100));
          }
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(300)); // Check faja_ocupado every 300 ms
  }
}

/************************************************** COLOR SENSOR (TCS230) ****************************************/
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
  // 100% frequency scaling
  digitalWrite(S0, HIGH);
  digitalWrite(S1, HIGH);
}

// Measures the color 5 times; the color detected at least 3 times wins
u_int8_t sensa_color (){
  int countRed = 0;
  int countGreen = 0;
  int countBlue = 0;
  int veces_color_sensado[4] = {0, 0, 0, 0}; // R, G, B, W counts
  for (int i = 1; i <= 5; i++){
    // Measure the output pulse width for each color filter
    digitalWrite(S2, LOW);
    digitalWrite(S3, LOW);
    countRed = pulseIn(OUT, digitalRead(OUT) == HIGH ? LOW : HIGH);
    digitalWrite(S2,LOW);
    digitalWrite(S3,HIGH);
    countBlue = pulseIn(OUT, digitalRead(OUT) == HIGH ? LOW : HIGH);
    digitalWrite(S2,HIGH);
    digitalWrite(S3,HIGH);
    countGreen = pulseIn(OUT, digitalRead(OUT) == HIGH ? LOW : HIGH);
    Serial.print("Red: ");
    Serial.print(countRed);
    Serial.print("Green: ");
    Serial.print(countGreen);
    Serial.print("Blue: ");
    Serial.print(countBlue);
    // Black and white are checked first
    if (countRed>4500 && countGreen>4500 && countBlue >4500){  // Black: nothing is counted
      Serial.println(" - Black");                              // Should not happen if the cube is placed correctly
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
      Serial.println("Not detected");
    }
    vTaskDelay(pdMS_TO_TICKS(100)); // Sensor settling time (5 readings take at least 500 ms)
  }
  // Pick the color detected at least 3 times
  for (int i = 0; i < 4; ++i) {
    if (veces_color_sensado[i] >= 3) {
      if(i==0){
        Serial.print("Detected color: Red");
      }
      else if (i==1){
        Serial.print("Detected color: Green");
      }
      else if (i==2){
        Serial.print("Detected color: Blue");
      }
      else if (i==3){
        Serial.print("Detected color: White");
      }
      return i + 1; // Index + 1 = color code
    }
    veces_color_sensado[i] = 0;
  }

  return 0; // No majority (should not happen with a calibrated sensor)
}


/************************************************** CONVEYOR CONTROL *********************************************/
uint8_t arr_cuenta_rechazados[4]={0,0,0,0}; // Rejected count per color (R, G, B, W)
uint8_t necesita_repuestos = 0;
uint8_t cuenta_correctos = 0;
uint8_t color_sensado=0;
uint8_t cuenta_color=0;
uint8_t cuenta_caidos = 0;

void task_color_y_control_faja1y2 (void *arg){
  while(1){
    // Order complete: has priority over PROX_CAJA, the box is moved out without waiting for a new one
    if ( (cuenta_caidos == cuenta_correctos) && (cuenta_caidos > 3) && (necesita_repuestos==0)){
      vTaskDelay(pdMS_TO_TICKS(200)); // Let the last cube fall before stopping motor 1
      robot.brake(1); // Restarted by task_receive1
      robot.rotate(motor2, 100, CCW);
      for (int i = 0; i < 5;i++){ // Clear the current order
        secuencia_actual[i] = 0;
      }
      cuenta_color = 0;
      color_sensado = 0;
      cuenta_correctos = 0;
      cuenta_caidos = 0;
      vTaskDelay(pdMS_TO_TICKS(2000)); // 2 s to dispatch the box
      robot.brake(2);
      faja_ocupado = 0;
    }
    // No box: has priority over color sensing; conveyor 1 stops until a box is present
    else if( (digitalRead(PROX_CAJA)==1) && (digitalRead(PROX_COLOR)==0)){
      parada_por_sensar_caja = 1;

      Serial.print("No box: conveyor 1 stopped");
      robot.brake(1);
      while (digitalRead(PROX_CAJA)==1){ // Wait for a box; the other tasks keep running
        vTaskDelay(pdMS_TO_TICKS(80));
      }
      parada_por_sensar_caja = 0;
      Serial.print("Box detected: resuming");

      robot.rotate(motor1, 100, CW);    // Resume motor 1
    }

    else if(digitalRead(PROX_COLOR)==0){
      vTaskDelay(pdMS_TO_TICKS(100)); // Let the cube center under the sensor (calibrate)
      parada_por_sensar_color = 1;
      robot.brake(1);
      color_sensado = sensa_color();
      if (color_sensado != secuencia_actual[cuenta_color]){
        secuencia_actual[cuenta_color] = W;   // Mark the position as rejected
        arr_cuenta_rechazados[color_sensado-1]++;
        necesita_repuestos = 1;
        myservo_rechaza.write(90); // Activate the rejection servo
      }
      else // Expected color
      {
        cuenta_correctos++;
        Serial.print("cuenta_correctos: ");
        Serial.println(cuenta_correctos);
      }
      cuenta_color++;

      robot.rotate(motor1, 100, CW);    // Resume motor 1 after sensing
      parada_por_sensar_color = 0;
      vTaskDelay((pdMS_TO_TICKS(400)));  // Let the cube leave the sensor before the next reading
    }
    vTaskDelay(pdMS_TO_TICKS(40)); // Poll PROX_COLOR and PROX_CAJA (calibrate)
  }
}

// Counts cubes falling into the box
void task_cuenta_caidos(void *arg){
  while(1){
    if ((digitalRead(PROX_CAIDA) == 0) && (parada_por_sensar_color == 0) && (parada_por_sensar_caja == 0))
    {
      cuenta_caidos++;
      Serial.print("cuenta_caidos: ");
      Serial.println(cuenta_caidos);
    }
    vTaskDelay(pdMS_TO_TICKS(210));  // Must be shorter than 300 ms
  }
}

// Resets the rejection servo once the rejected cube has passed
void task_desactiva_rechazo(void *arg){
  while(1){
    if(digitalRead(PROX_RECHAZO)==0){
      myservo_rechaza.write(0);
    }
    vTaskDelay(pdMS_TO_TICKS(80)); // Poll every 80 ms (calibrate)
  }
}


void setup() {
  Serial.begin(115200);
  config_servos();
  robot.begin();  // TB6612FNG driver
  robot.brake(1); // Both motors start stopped
  robot.brake(2);
  config_sensor_color();

  pinMode(PROX_COLOR, INPUT);
  pinMode(PROX_RECHAZO, INPUT);
  pinMode(PROX_CAIDA, INPUT);
  pinMode(PROX_CAJA, INPUT);

  queue1 = xQueueCreate(30, sizeof(secuencia_terminal)); // 30 slots of 5 bytes
  xTaskCreate(task_send1, "task_send1", 4096, NULL, 3, &task_send_Handle1);
  xTaskCreate(task_receive1, "task_receive1", 4096, NULL, 3, &task_receive_Handle1);
  xTaskCreate(readSerialTask, "ReadSerial", 1024, NULL, 2, NULL);
  xTaskCreate(task_color_y_control_faja1y2, "controlfajas", 4096, NULL, 1, NULL);
  xTaskCreate(task_cuenta_caidos, "cuenta_caidos", 1024, NULL, 1, NULL);
  xTaskCreate(task_desactiva_rechazo, "desactiva_rechazo", 1024, NULL, 3, NULL);
}


void loop() {

}
