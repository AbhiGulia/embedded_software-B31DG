#include <B31DGMonitor.h>
#include <Ticker.h>

#define DIGITAL_SIGNAL_PIN 4
#define FREQ_PIN_1 5
#define FREQ_PIN_2 6
#define ANALOG_READ_PIN 0 
#define ANALOG_ERROR_PIN 1 
#define ANALOG_AVG_COUNT 4   //averaging 4 analog values


B31DGCyclicExecutiveMonitor monitor;
Ticker frameticker;

//variables for avgAnalogVal
uint16_t analog_val_array[ANALOG_AVG_COUNT];
uint32_t sum = 0;

//variables for freqCounter
uint64_t max_timeperiod = 0;
uint64_t freq_start_time = 0;
uint64_t timeout_counter = 0; // variable to store time for timeout purposes
uint8_t pin_status = 0;       // variable to store the status of pin for state change monitoring purposes

//variables for freq1 and freq2
uint16_t last_freq_signal_1 = 0;
uint16_t last_freq_signal_2 = 0;

// variables for cyclic executive

uint64_t last_time_1 = 0; // Last task 1 execution time in micros
uint64_t last_time_2 = 0; // Last task 2 execution time in micros
uint64_t last_time_3 = 0; // Last task 3 execution time in micros
uint64_t last_time_4 = 0; // Last task 4 execution time in micros
uint64_t last_time_5 = 0; // Last task 5 execution time in micros

//variables for frame
uint16_t frameCount = 0;
uint16_t section = 0;

void frame(){
  switch (section) {
        case 0:   digitalSignal();              freq2();  avgAnalogVal();   logSerial(); break;
        case 1:   digitalSignal();   freq1();                                            break;
        case 2:   digitalSignal();              freq2();                                 break; 
        case 3:   digitalSignal();                                                       break;
        case 4:   digitalSignal();              freq2();                                 break; 
        case 5:   digitalSignal();                        avgAnalogVal();                break;
        case 6:   digitalSignal();              freq2();                                 break;
        case 7:   digitalSignal();   freq1();                                            break;
        case 8:   digitalSignal();              freq2();                                 break;
        case 9:   digitalSignal();                                                       break;
        case 10:  digitalSignal();              freq2();  avgAnalogVal();                break;
        case 11:  digitalSignal();   freq1();                                            break;  
        case 12:  digitalSignal();              freq2();                                 break;
        case 13:  digitalSignal();                                                       break;
        case 14:  digitalSignal();              freq2();                                 break;
        case 15:  digitalSignal();                        avgAnalogVal();                break;
        case 16:  digitalSignal();              freq2();                                 break;
        case 17:  digitalSignal();   freq1();                                            break;
        case 18:  digitalSignal();              freq2();                                 break;
        case 19:  digitalSignal();                                                       break;
        case 20:  digitalSignal();              freq2();                                 break;
        case 21:  digitalSignal();   freq1();             avgAnalogVal();                break;
        case 22:  digitalSignal();              freq2();                                 break;
        case 23:  digitalSignal();                                                       break;
        case 24:  digitalSignal();              freq2();                                 break;
        case 25:  digitalSignal();                        avgAnalogVal();   logSerial(); break;
        case 26:  digitalSignal();              freq2();                                 break;
        case 27:  digitalSignal();   freq1();                                            break;
        case 28:  digitalSignal();              freq2();                                 break;
        case 29:  digitalSignal();                                                       break;
        case 30:  digitalSignal();              freq2();  avgAnalogVal();                break;
        case 31:  digitalSignal();   freq1();                                            break;
        case 32:  digitalSignal();              freq2();                                 break;
        case 33:  digitalSignal();                                                       break;
        case 34:  digitalSignal();              freq2();                                 break;
        case 35:  digitalSignal();                        avgAnalogVal();                break;
        case 36:  digitalSignal();              freq2();                                 break;
        case 37:  digitalSignal();   freq1();                                            break;
        case 38:  digitalSignal();              freq2();                                 break;
        case 39:  digitalSignal();                                                       break;
        case 40:  digitalSignal();              freq2();  avgAnalogVal();                break;
        case 41:  digitalSignal();   freq1();                                            break;
        case 42:  digitalSignal();              freq2();                                 break;
        case 43:  digitalSignal();                                                       break;
        case 44:  digitalSignal();              freq2();                                 break;
        case 45:  digitalSignal();                        avgAnalogVal();                break;
        case 46:  digitalSignal();              freq2();                                 break;
        case 47:  digitalSignal();   freq1();                                            break;
        case 48:  digitalSignal();              freq2();                                 break;
        case 49:  digitalSignal();                                                       break;
    
  }
  frameCount ++;                             //Increments Frame counter which is later used to determined remaining number of frames
  section = frameCount % 50;                 //Determines no. of remaining frames by taking the remainder, so microcontroller determines specific frame to execute in succession
}

void setup() {
  // put your setup code here, to run once:
  pinMode(DIGITAL_SIGNAL_PIN, OUTPUT);   // making DIGITAL_SIGNAL_PIN as output to create desired WAVEFORM
  digitalWrite(DIGITAL_SIGNAL_PIN, LOW); // making DIGITAL_SIGNAL_PIN as default low

  pinMode(FREQ_PIN_1, INPUT);            // making FREQ_PIN_1 as input to measure frequency
  pinMode(FREQ_PIN_2, INPUT);            // making FREQ_PIN_2 as input to measure frequency

  pinMode(ANALOG_READ_PIN, INPUT);       // making ANALOG_READ_PIN as input to measure analog reading
  pinMode(ANALOG_ERROR_PIN, OUTPUT);     // making ANALOG_ERROR_PIN as output to show error
  digitalWrite(ANALOG_ERROR_PIN, LOW);   // making ANALOG_ERROR_PIN as default low

  //initializing analog_val_array
  for(uint8_t i = 0; i <= ANALOG_AVG_COUNT-1; i++)
    analog_val_array[i] = 0;
  Serial.begin(9600);
  while(!Serial);
  monitor.startMonitoring();
  frame();
  frameticker.attach_ms(4, frame);

}

void loop() {
  // put your main code here, to run repeatedly:

 
}

// Function to create desired output signal as given in task 1
// worst case execution time = 291 micros
void digitalSignal(){
  monitor.jobStarted(1);
  digitalWrite(DIGITAL_SIGNAL_PIN, HIGH);
  delayMicroseconds(200);
  digitalWrite(DIGITAL_SIGNAL_PIN, LOW);
  delayMicroseconds(50);
  digitalWrite(DIGITAL_SIGNAL_PIN, HIGH);
  delayMicroseconds(30);
  digitalWrite(DIGITAL_SIGNAL_PIN, LOW);
  monitor.jobEnded(1);
}

// Function to measure frequency of a signal as given in task 2
// worst case execution time = 4750 micros (without serial printing out of range Warning)
// worst case execution time = 127634 micros (with serial printing out of range Warning)
void freq1(){
  monitor.jobStarted(2);
  last_freq_signal_1 = freqCounter(FREQ_PIN_1, 333, 1000);
  //return last_freq_signal_1;
  monitor.jobEnded(2);
}

// Function to measure frequency of a signal as given in task 3
// worst case execution time = 3175 micros (without serial printing out of range Warning)
// worst case execution time = 126091 micros (with serial printing out of range Warning)
void freq2(){
  monitor.jobStarted(3);
  last_freq_signal_2 = freqCounter(FREQ_PIN_2, 500, 1000);
  monitor.jobEnded(3);
}

// Function to calculate average analog value using 4 readings as given in task 4
// worst case execution time = 217 micros
void avgAnalogVal(){
  monitor.jobStarted(4);
  sum = 0;
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
  //return avg_val;
  monitor.jobEnded(4);
}

// Function to log data of task 2 and task 3 to Serial port as given in task 5
// worst case execution time = 122878 micros
void logSerial(){
  monitor.jobStarted(5);
  //mapping the frequencies to values given in task 5
  int mapped_freq_1 = map(last_freq_signal_1, 333, 1000, 0 , 99);
  int mapped_freq_2 = map(last_freq_signal_2, 500, 1000, 0 , 99);

  // limiting the range to given range in task 5
  if(mapped_freq_1 < 0)  mapped_freq_1 = 0;
  else if(mapped_freq_1 > 99) mapped_freq_1 = 99;
  if(mapped_freq_2 < 0)  mapped_freq_2 = 0;
  else if(mapped_freq_2 > 99) mapped_freq_2 = 99;

  //printing the result in mentioned form
  Serial.printf("%d,%d\n",mapped_freq_1, mapped_freq_2);
  monitor.jobEnded(5);
}

// Function to measure frequency of a given wave
uint16_t freqCounter(uint8_t read_pin, uint16_t minfreq, uint16_t maxfreq){
  max_timeperiod = 1000000/minfreq;                                                     // max timeperiod in micros
  max_timeperiod = max_timeperiod/2 //+ max_timeperiod*0.025;                             // max timeperiod for a wave with 50% percent duty cycle and 2.5% error margin
  timeout_counter = micros();                                                           // store current time for timeout purposes
  pin_status = digitalRead(read_pin);                                                   // store the status of pin for state change monitoring purposes
  while(digitalRead(read_pin) == pin_status && micros() - timeout_counter < max_timeperiod); // wait for the signal to change state with a timeout period
  freq_start_time = micros();                                                                // starting measuring frequency
  timeout_counter = micros();                                                                // updating timeout_counter for next timeout call
  pin_status = digitalRead(read_pin);                                                        // updating pin_status for capturing next pin state change
  while(digitalRead(read_pin) == pin_status && micros() - timeout_counter < max_timeperiod); // wait for the signal to change state with a timeout period
  timeout_counter = micros();                                                                // updating timeout_counter for next timeout call
  pin_status = digitalRead(read_pin);                                                        // updating pin_status for capturing next pin state change
  while(digitalRead(read_pin) == pin_status && micros() - timeout_counter < max_timeperiod); // wait for the signal to change state with a timeout period
  uint16_t freq = 1000000/(micros() - freq_start_time);                                      // freq = 1/timeperiod changed in secs from micros
  if(freq < minfreq - 0.025*minfreq || freq > maxfreq + 0.025*maxfreq){                      // checking range of frequency with 2.5% error margin
    //Serial.print(" WARNING! frequency of signal is out of range. ");
  }
  return freq;
}

/* REFERENCES
  1. https://canvas.hw.ac.uk/courses/18948/files/2577544?module_item_id=1663100
  */