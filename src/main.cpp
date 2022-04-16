#include <Arduino.h>
#include <U8g2lib.h>
#include <STM32FreeRTOS.h>
#include <ES_CAN.h>

//Constants
const uint32_t interval = 100; //Display update interval

/////////// Pin definitions ///////////
//Row select and enable
const int RA0_PIN = D3;
const int RA1_PIN = D6;
const int RA2_PIN = D12;
const int REN_PIN = A5;

//Matrix input and output
const int C0_PIN = A2;
const int C1_PIN = D9;
const int C2_PIN = A6;
const int C3_PIN = D1;
const int OUT_PIN = D11;

//Audio analogue out
const int OUTL_PIN = A4;
const int OUTR_PIN = A3;

//Joystick analogue in
const int JOYY_PIN = A0;
const int JOYX_PIN = A1;

//Output multiplexer bits
const int DEN_BIT = 3;
const int DRST_BIT = 4;
const int HKOW_BIT = 5;
const int HKOE_BIT = 6;

const int DISP_SDA = D4;
const int DISP_SCL = D5;
const int DISP_RXD = D10;
const int DISP_TXD = D2;

/*
This variable will be accessed by more than one concurrent task (thread), so it is declared with the
keyword volatile. This instructs the compiler to access the variable in memory each time it
appears in the source code. Otherwise, the compiled code may keep a copy of the variable in
a CPU register and miss updates made by other tasks.
*/
volatile uint8_t keyArray[7];
volatile int32_t currentStepSize;
String note;

uint8_t TX_Message[8] = {0};
volatile uint8_t prev = 0;

/*
The array keyArray is now a global variable
and it is accessed by both the main loop and scanKeysTask(). It cannot be accessed atomically
because it is an array of multiple bytes – it requires multiple operations to read and write. Since
all the accesses to the array are from threads, not interrupts, it can be protected by a mutex.

KeyarrayMutex is a global handle for a FreeRTOS mutex that can be used by different threads to
access the mutex object

The main difference between a semaphore and a mutex is theway they are used. 

A mutex is ‘taken’ (locked) by a thread while it accesses the protected
 data or resource, then ‘given’ (unlocked) when it is finished. All threads that access the
resource take then give the mutex in the same way.

A semaphore is used asymmetrically – one thread tries to take the semaphore, but it is
blocked until a different thread gives it. A single thread will only take or give a particular
semaphore, never both.
*/
SemaphoreHandle_t keyArrayMutex; 

//Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

//Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value) {
      digitalWrite(REN_PIN,LOW);
      digitalWrite(RA0_PIN, bitIdx & 0x01);
      digitalWrite(RA1_PIN, bitIdx & 0x02);
      digitalWrite(RA2_PIN, bitIdx & 0x04);
      digitalWrite(OUT_PIN,value);
      digitalWrite(REN_PIN,HIGH);
      delayMicroseconds(2);
      digitalWrite(REN_PIN,LOW);
}

uint8_t readCols() {
  // Make RA0, RA1, RA2 Low, Row select enable (REN) high.
  // digitalWrite(RA0_PIN, LOW);
  // digitalWrite(RA1_PIN, LOW);
  // digitalWrite(RA2_PIN, LOW);
  // digitalWrite(REN_PIN, HIGH);

  // Read values: Pressing each of the four left-most keys of the keyboard should change the number that is displayed on the screen
  uint8_t C0 = digitalRead(C0_PIN);
  uint8_t C1 = digitalRead(C1_PIN);
  uint8_t C2 = digitalRead(C2_PIN);
  uint8_t C3 = digitalRead(C3_PIN);

  // String dataString = "C0:" + String(C0) + ", C1:" + String(C1) + ", C2:"  + String(C2) + ", C3:"  + String(C3);
  // Serial.println(dataString);

  return (C0 << 3) | (C1 << 2) | (C2 << 1) | (C3); 
}

void setRow(uint8_t rowIdx){
  // Disable (set low) the row select enable first
  digitalWrite(REN_PIN, LOW);

  bool RA0 = rowIdx & B1;
  bool RA1 = (rowIdx >> 1) & B1;
  bool RA2 = (rowIdx >> 2) & B1;

  digitalWrite(RA0_PIN, RA0);
  digitalWrite(RA1_PIN, RA1);
  digitalWrite(RA2_PIN, RA2);

  // Enable (set high) the row select enable after changes
  digitalWrite(REN_PIN, HIGH);
}

int rotation(uint8_t prev, uint8_t current) {
  if (prev == current) {
    return 0;
  } else if (prev) {
    
  }
}

// Array of phase step sizes required for each of the 12 notes of your keyboard
const int32_t stepSizes [] = {
  0, // no sound
  51076057,
  54113197,
  57330935,
  60740010,
  64351798,
  68178356,
  72232452,
  76527617,
  81078186,
  85899345,
  91007186,
  96418755
};

// update the phase accumulator and set the analogue output voltage at each sample interval
void sampleISR() {
  // interrupt service routine, which means that it cannot have arguments or a return value

  /*
  Define the phase accumulator as a static local variable, so that its value will be stored between
  successive calls of sampleISR()
  */
  static int32_t phaseAcc = 0;
  phaseAcc += currentStepSize;

  int32_t Vout = phaseAcc >> 24; // Range reduced to -2^7 to 2^7. Sawtooth value proportional to phase
  analogWrite(OUTR_PIN, Vout + 128); // Audio value of 0 produces 1.65V

  // Note: always define function to have midpoint of zero, add DCoffset in last steps
}

/*
• Loop through the rows of the key matrix
• Read the columns of the matrix and store the result in keyArray
• Look up the phase step size for the key that is pressed and update currentStepSize
*/
void scanKeysTask(void * pvParameters) {
    const TickType_t xFrequency = 50/portTICK_PERIOD_MS;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    /*
    xFrequency will be the initiation interval of the task. It is given in units of RTOS scheduler
    ticks and we can use the constant portTICK_PERIOD_MS to convert a time in milliseconds to
    scheduler ticks. Here we have set the initiation interval to 50ms.

    xLastWakeTime will store the time (tick count) of the last initiation. We initialise it with the
    API call xTaskGetTickCount() to get the current time. In subsequent iterations this variable
    will be updated by the call to vTaskDelayUntil().
    */

    while(1) { // independent thread, infinite loop
      int32_t localCurrentStepSize;
      uint8_t localKeyArray[7];
      vTaskDelayUntil( &xLastWakeTime, xFrequency );

      /*
      A better approach may be to create local copies of the array in each function that accesses it.
      You can use the C function memcpy() for this purpose. Then the mutex would only be locked
      while the copy is taking place, and not the processing. The downside of this method is that extra
      memory and CPU execution time are required to make the copies. It would be impractical for
      large arrays.
      */
      xSemaphoreTake(keyArrayMutex, portMAX_DELAY); //take the mutex while keyArray is accessed using the appropriate functions
      // TODO: memcpy(localKeyArray, keyArray, sizeof(localKeyArray));
            
      for (int i=0; i<=2; i++) {
        setRow(i);
        // The switch matrix columns take some time to switch from logic 0 to logic 1 when the row select changes due to parasitic capacitance.
        delayMicroseconds(3); 
        uint8_t keys = readCols();
        
        // Only doing 0 to 2 because it covers 12 music keys
        keyArray[i] = keys;
      }
      //Check state of each key in keyArray
      bool C  = (~(keyArray)[0] >> 3) & B1;
      bool Db = (~(keyArray)[0] >> 2) & B1;
      bool D  = (~(keyArray)[0] >> 1) & B1;
      bool Eb = (~(keyArray)[0] >> 0) & B1;
      bool E  = (~(keyArray)[1] >> 3) & B1;
      bool F  = (~(keyArray)[1] >> 2) & B1;
      bool Gb = (~(keyArray)[1] >> 1) & B1;
      bool G  = (~(keyArray)[1] >> 0) & B1;
      bool Ab = (~(keyArray)[2] >> 3) & B1;
      bool A  = (~(keyArray)[2] >> 2) & B1;
      bool Bb = (~(keyArray)[2] >> 1) & B1;
      bool B  = (~(keyArray)[2] >> 0) & B1;
      xSemaphoreGive(keyArrayMutex);

      uint8_t key = 0 ;
      note = "ugh";

      // TODO: Reduce accesses to currentStepSize. Use local variable for step size until all keys checked.
      if (C ) {key = 1;  note = "C ";}
      if (Db) {key = 2;  note = "C#/ Db";}
      if (D ) {key = 3;  note = "D ";}
      if (Eb) {key = 4;  note = "D#/ Eb";}
      if (E ) {key = 5;  note = "E ";}
      if (F ) {key = 6;  note = "F ";}
      if (Gb) {key = 7;  note = "F#/ Gb";}
      if (G ) {key = 8;  note = "G ";}
      if (Ab) {key = 9;  note = "G#/ Ab";}
      if (A ) {key = 10; note = "A ";}
      if (Bb) {key = 11; note = "A#/ Bb";}
      if (B ) {key = 12; note = "B ";}

      TX_Message[1] = 0; // For now there is only one keyboard
      if (prev == key) { // Holding on to key
        // pass
      } else if (key == 0 && prev != 0) { // Releasing key
        TX_Message[0] = 'R';
        TX_Message[2] = key;
      } else if (key == 0){
        TX_Message[0] = 0;
        TX_Message[1] = 0;
        TX_Message[2] = 0;
      } else if (prev != key) { // Press new key
        TX_Message[0] = 'P';
        TX_Message[2] = key;
      }

      // Send message over the bus using the CAN library
      CAN_TX(0x123, TX_Message);

      prev = key;

      localCurrentStepSize = stepSizes[key];
      __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED); // stores localCurrentStepSize in currentStepSize as an atomic operation.
    }
}

void displayUpdateTask(void * pvParameters) {
    const TickType_t xFrequency = 100/portTICK_PERIOD_MS; //100ms initiation interval
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t ID;
    uint8_t RX_Message[8]={0};

    while(1) {
      vTaskDelayUntil( &xLastWakeTime, xFrequency );

      while (CAN_CheckRXLevel()) {
        CAN_RX(ID, RX_Message);
      }

      //Update display
      u8g2.clearBuffer();         // clear the internal memory
      u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font

      u8g2.setCursor(2,10);
      u8g2.drawStr(2,10,"Not a real piano");

      u8g2.setCursor(2,20);

      xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
      u8g2.print(keyArray[0],HEX);
      u8g2.print(keyArray[1],HEX);
      u8g2.print(keyArray[2],HEX);
      xSemaphoreGive(keyArrayMutex);

      u8g2.setCursor(2,30);
      u8g2.drawStr(2,30, note.c_str());
      //String dataString = "Row0:" + String(keyArray[0]) + ", Row1:" + String(keyArray[1]) + ", Row2:"  + String(keyArray[2]);
      //Serial.println(dataString);

      u8g2.setCursor(66,30);
      u8g2.print((char) RX_Message[0]);
      u8g2.print(RX_Message[1]);
      u8g2.print(RX_Message[2]);

      u8g2.sendBuffer();          // transfer internal memory to the display
      digitalToggle(LED_BUILTIN); //Toggle LED
    }
}

void setup() {
  // put your setup code here, to run once:

  //Set pin directions
  pinMode(RA0_PIN, OUTPUT);
  pinMode(RA1_PIN, OUTPUT);
  pinMode(RA2_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT);
  pinMode(OUTL_PIN, OUTPUT);
  pinMode(OUTR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(C0_PIN, INPUT);
  pinMode(C1_PIN, INPUT);
  pinMode(C2_PIN, INPUT);
  pinMode(C3_PIN, INPUT);
  pinMode(JOYX_PIN, INPUT);
  pinMode(JOYY_PIN, INPUT);

  //Initialise display
  setOutMuxBit(DRST_BIT, LOW);  //Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);  //Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);  //Enable display power supply

  // Create a timer
  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);

  // Configure timer
  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();

  //Initialise UART
  Serial.begin(9600);
  Serial.println("Hello World");

  TaskHandle_t displayUpdateHandle = NULL;
  xTaskCreate(
    displayUpdateTask, /* Function that implements the task */
    "displayUpdate", /* Text name for the task */
    256, /* Stack size in words, not bytes (256 bytes)*/
    NULL, /* Parameter passed into the task */
    1, /* Task priority: Low */
    &displayUpdateHandle
  ); /* Pointer to store the task handle */

  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
    scanKeysTask, /* Function that implements the task */
    "scanKeys", /* Text name for the task */
    64, /* Stack size in words, not bytes (256 bytes)*/
    NULL, /* Parameter passed into the task */
    2, /* Task priority: HIGH */
    &scanKeysHandle
  ); /* Pointer to store the task handle */

  // initialise CAN module
  CAN_Init(true);
  setCANFilter(0x123,0x7ff);
  CAN_Start();
  
  keyArrayMutex = xSemaphoreCreateMutex(); // Create mutex before schedule starts
  vTaskStartScheduler(); //start the RTOS scheduler

}

void loop() {
  // put your main code here, to run repeatedly:
}