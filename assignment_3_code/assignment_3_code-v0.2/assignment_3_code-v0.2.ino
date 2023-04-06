
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

#define DIGITAL_SIGNAL_PIN 4
#define FREQ_PIN_1 5
#define FREQ_PIN_2 6
#define ANALOG_READ_PIN 0 
#define ANALOG_ERROR_PIN 1 
#define BUTTON_READ_PIN 2 ////******************test***************////
#define BUTTON_LED_PIN 3     ////******************test***************////




//variables for avgAnalogVal
uint16_t analog_val_array[4] ={0, 0, 0, 0} ;

//variables for checkButtonState
uint32_t last_button_pressed = 0;
QueueHandle_t button_pressed_queue;

SemaphoreHandle_t freqsemaphore;

// structure to log frequency data
struct{
  uint16_t last_freq_signal_1;
  uint16_t last_freq_signal_2;
} freq_log;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);                    // Initializing serial port
  pinMode(DIGITAL_SIGNAL_PIN, OUTPUT);   // making DIGITAL_SIGNAL_PIN as output to create desired WAVEFORM
  digitalWrite(DIGITAL_SIGNAL_PIN, LOW); // making DIGITAL_SIGNAL_PIN as default low

  pinMode(FREQ_PIN_1, INPUT);            // making FREQ_PIN_1 as input to measure frequency
  pinMode(FREQ_PIN_2, INPUT);            // making FREQ_PIN_2 as input to measure frequency

  pinMode(ANALOG_READ_PIN, INPUT);       // making ANALOG_READ_PIN as input to measure analog reading
  pinMode(ANALOG_ERROR_PIN, OUTPUT);     // making ANALOG_ERROR_PIN as output to show error
  digitalWrite(ANALOG_ERROR_PIN, LOW);   // making ANALOG_ERROR_PIN as default low
  
  // the input is pulled up high using internal pullup resistors so external wiring can be reduced
  pinMode(BUTTON_READ_PIN, INPUT_PULLUP);// making BUTTON_READ_PIN as input to get button state

  freqsemaphore = xSemaphoreCreateMutex();
  xSemaphoreGive(freqsemaphore);

  button_pressed_queue = xQueueCreate(3, sizeof(uint8_t));

  while(!Serial);                        // waiting for serial port to become available
  xTaskCreate(digitalSignal, "Create squarewave", 1024, NULL, 1, NULL);
  xTaskCreate(freq1, "Read frequency 1", 1024, NULL, 2, NULL);
  xTaskCreate(freq2, "Read frequency 1", 1024, NULL, 2, NULL);
  xTaskCreate(avgAnalogVal, "Read analog value", 1024, NULL, 1, NULL);
  xTaskCreate(logSerial, "Create squarewave", 1024, NULL, 3, NULL);
  xTaskCreate(checkButtonState, "Monitor button", 1024, NULL, 1, NULL);
  xTaskCreate(toggleLED, "Toggle LED", 1024, NULL, 3, NULL);
}

void loop() {
  // put your main code here, to run repeatedly:

}


// Function to create desired output signal as given in task 1  [Period = 4ms / Rate = 250Hz]
// worst case execution time = 291 micros
void digitalSignal(void *pvParameters){
  TickType_t last_wake_time = xTaskGetTickCount();
  while(true){
    digitalWrite(DIGITAL_SIGNAL_PIN, HIGH);
    delayMicroseconds(200);
    digitalWrite(DIGITAL_SIGNAL_PIN, LOW);
    delayMicroseconds(50);
    digitalWrite(DIGITAL_SIGNAL_PIN, HIGH);
    delayMicroseconds(30);
    digitalWrite(DIGITAL_SIGNAL_PIN, LOW);
    vTaskDelayUntil(&last_wake_time, 4/portTICK_PERIOD_MS);
  }
}


// Function to measure frequency of a signal as given in task 2  [Period = 20ms / Rate = 50Hz]
// worst case execution time = 3181 micros (without serial printing out of range Warning)
// worst case execution time = 3237 micros (with serial printing out of range Warning)
void freq1(void *pvParameters){  
  TickType_t last_wake_time = xTaskGetTickCount();
  while(true){
    uint16_t freq_signal_1 = freqCounter(FREQ_PIN_1, 333, 1000);
    if (xSemaphoreTake(freqsemaphore, (TickType_t)100) == pdTRUE) {
        freq_log.last_freq_signal_1 = freq_signal_1;
        xSemaphoreGive(freqsemaphore);
    }
    vTaskDelayUntil(&last_wake_time, 20/portTICK_PERIOD_MS);
  }
}

// Function to measure frequency of a signal as given in task 3  [Period = 8ms / Rate = 125Hz]
// worst case execution time = 2130 micros (without serial printing out of range Warning)
// worst case execution time = 2185 micros (with serial printing out of range Warning)
void freq2(void *pvParameters){
  TickType_t last_wake_time = xTaskGetTickCount();
  while(true){
    uint16_t freq_signal_2 = freqCounter(FREQ_PIN_2, 500, 1000);
     if (xSemaphoreTake(freqsemaphore, (TickType_t)100) == pdTRUE) {
        freq_log.last_freq_signal_2 = freq_signal_2;
        xSemaphoreGive(freqsemaphore);
     }
    vTaskDelayUntil(&last_wake_time, 8/portTICK_PERIOD_MS);
  }
}


// Function to calculate average analog value using 4 readings as given in task 4  [Period = 20ms / Rate = 50Hz]
// worst case execution time = 217 micros
void avgAnalogVal(void *pvParameters){
  TickType_t last_wake_time = xTaskGetTickCount();
  while(true){
    const uint8_t ANALOG_AVG_COUNT = 4 ;  //averaging 4 analog values
    uint32_t sum = 0;
    //removing the oldest entry from analog_val_array and moving remaining entries to the begenning of the array so that new entry can be entered
    for(uint8_t i = 0; i <= ANALOG_AVG_COUNT-2; i++){
      analog_val_array[i] = analog_val_array[i+1];
      sum += analog_val_array[i];                                       // calculating the sum of previous 3 entries
    }

    analog_val_array[ANALOG_AVG_COUNT-1] = analogRead(ANALOG_READ_PIN); // adding new value at the end of the array
    sum+= analog_val_array[ANALOG_AVG_COUNT-1];                         // adding new value to sum for averaging
    
    uint16_t avg_val = sum/ANALOG_AVG_COUNT;                            // average of 4 readings
    
    if(avg_val > 4095/2)                                                // 12 bit ADC = max 4095
      digitalWrite(ANALOG_ERROR_PIN, HIGH);
    else
      digitalWrite(ANALOG_ERROR_PIN, LOW);
    vTaskDelayUntil(&last_wake_time, 20/portTICK_PERIOD_MS);
  }
}

// Function to log data of task 2 and task 3 to Serial port as given in task 5   [Period = 100ms / Rate = 10Hz]
// worst case execution time = 135 micros
void logSerial(void *pvParameters){
  TickType_t last_wake_time = xTaskGetTickCount();
  while(true){
    if (xSemaphoreTake(freqsemaphore, (TickType_t)100) == pdTRUE) {
      //mapping the frequencies to values given in task 5
      int8_t mapped_freq_1 = map(freq_log.last_freq_signal_1, 333, 1000, 0 , 99);
      int8_t mapped_freq_2 = map(freq_log.last_freq_signal_2, 500, 1000, 0 , 99);
      xSemaphoreGive(freqsemaphore);

      // limiting the range to given range in task 5
      if(mapped_freq_1 < 0)  mapped_freq_1 = 0;
      else if(mapped_freq_1 > 99) mapped_freq_1 = 99;
      if(mapped_freq_2 < 0)  mapped_freq_2 = 0;
      else if(mapped_freq_2 > 99) mapped_freq_2 = 99;

      //printing the result in mentioned form
      Serial.printf("%d,%d\n",mapped_freq_1, mapped_freq_2);
    }
    vTaskDelayUntil(&last_wake_time, 100/portTICK_PERIOD_MS);
  }
}

// Function to measure frequency of a given wave
// max_timeperiod = 1000000*1/minfreq    time = 1/f converted into micros 
// also there is a 2.5 percent error rate allowance
// therefore max_timeperiod = 1000000*1/minfreq + 0.025*((1000000*1)/minfreq)
// max_timeperiod = 1000000/minfreq + 25000/minfreq = 1025000/minfreq
// also duty cycle is 50% so effective timeperiod required to measure frequency = timeperiod/2
// therefore max_timeperiod = (1025000/minfreq)/2 = 512500/minfreq
uint16_t freqCounter(uint8_t read_pin, uint16_t minfreq, uint16_t maxfreq){
  uint64_t max_timeperiod = ceil(512500.0/minfreq);                                                     // max timeperiod in micros for a wave with 50% percent duty cycle and 2.5% error margin
  uint64_t timeout_counter = micros();                                                           // store current time for timeout purposes
  uint8_t pin_status = digitalRead(read_pin);                                                   // store the status of pin for state change monitoring purposes
  while(digitalRead(read_pin) == pin_status && micros() - timeout_counter < max_timeperiod); // wait for the signal to change state with a timeout period
  uint64_t freq_start_time = micros();                                                                // starting measuring frequency
  timeout_counter = micros();                                                                // updating timeout_counter for next timeout call
  pin_status = digitalRead(read_pin);                                                        // updating pin_status for capturing next pin state change
  while(digitalRead(read_pin) == pin_status && micros() - timeout_counter < max_timeperiod); // wait for the signal to change state with a timeout period
  uint16_t freq = 500000/(micros() - freq_start_time);                                      // freq = 1/timeperiod changed in secs from micros and divided by 2 as we are only measuring either high or low time
  // checking range of frequency with 2.5% error margin
  if(freq < minfreq - 0.025*minfreq ){
    freq = minfreq;
  } 
  else if(freq > maxfreq + 0.025*maxfreq){                      
    freq = maxfreq;
  }
  return freq;
}

// Function to check the status of a button with debounce delay
// based on button pressed, toggles a button state variable HIGH and LOW
void checkButtonState(void *pvParameters){
  TickType_t last_wake_time = xTaskGetTickCount();
  while(true){
    if(millis() - last_button_pressed > 50 && digitalRead(BUTTON_READ_PIN) == 1){
      //button_state = !button_state;
      uint8_t tmpevent = 1;
      xQueueSend(button_pressed_queue, &tmpevent, portMAX_DELAY);
      last_button_pressed = millis();
    }
    vTaskDelayUntil(&last_wake_time, 40/portTICK_PERIOD_MS);
  }
}

// Function to control the LED 
// simply write the button state varible to LED pin
void toggleLED(void *pvParameters){
  while(true){
    uint8_t tmpevent;
    if (xQueueReceive(button_pressed_queue, &tmpevent, portMAX_DELAY) == pdTRUE) {
      if (tmpevent == 1) {
        digitalWrite(BUTTON_LED_PIN, !digitalRead(BUTTON_LED_PIN));
      }
    }  
  }
}


/* REFERENCES
  1. https://esp32tutorials.com/esp32-esp-idf-freertos-semaphore-tutorial/
  2. https://github.com/DiegoPaezA/ESP32-freeRTOS
  3. https://www.freertos.org/fr-content-src/uploads/2018/07/161204_Mastering_the_FreeRTOS_Real_Time_Kernel-A_Hands-On_Tutorial_Guide.pdf
  4. https://canvas.hw.ac.uk/courses/18948/files/2451779?module_item_id=1609173
  5. https://canvas.hw.ac.uk/courses/18948/pages/semaphores-problems-and-solutions?module_item_id=1609174
  6. https://www.youtube.com/playlist?list=PLEBQazB0HUyQ4hAPU1cJED6t3DU0h34bz
  */