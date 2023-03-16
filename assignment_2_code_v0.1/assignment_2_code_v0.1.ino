#define DIGITAL_SIGNAL_PIN 4
#define FREQ_PIN_1 5
#define FREQ_PIN_2 6
#define ANALOG_READ_PIN 0 
#define ANALOG_AVG_COUNT 4   //averaging 4 analog values

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

void setup() {
  // put your setup code here, to run once:
  pinMode(DIGITAL_SIGNAL_PIN, OUTPUT);   // making DIGITAL_SIGNAL_PIN as output to create desired WAVEFORM
  digitalWrite(DIGITAL_SIGNAL_PIN, LOW); // making DIGITAL_SIGNAL_PIN as default low

  pinMode(FREQ_PIN_1, INPUT);            // making FREQ_PIN_1 as input to measure frequency
  pinMode(FREQ_PIN_2, INPUT);            // making FREQ_PIN_2 as input to measure frequency

  pinMode(ANALOG_READ_PIN, INPUT);       // making ANALOG_READ_PIN as input to measure analog reading

  //initializing analog_val_array
  for(uint8_t i = 0; i <= ANALOG_AVG_COUNT-1; i++)
    analog_val_array[i] = 0;
  freq1();
  freq2();
  Serial.begin(9600);
  while(!Serial);
}
uint32_t worst_time = 0;
void loop() {
  // put your main code here, to run repeatedly:
 uint64_t start_time = micros();
  //uint16_t fr = freq2();
  logSerial();
  uint64_t time_took = micros() - start_time;
  if(time_took > worst_time){
    worst_time = time_took;
  }
  //Serial.print(fr);
  Serial.print(" - ");
  Serial.println(worst_time);
}

// Function to create desired output signal as given in task 1
// worst case execution time = 291 micros
void digitalSignal(){
  digitalWrite(DIGITAL_SIGNAL_PIN, HIGH);
  delayMicroseconds(200);
  digitalWrite(DIGITAL_SIGNAL_PIN, LOW);
  delayMicroseconds(50);
  digitalWrite(DIGITAL_SIGNAL_PIN, HIGH);
  delayMicroseconds(30);
  digitalWrite(DIGITAL_SIGNAL_PIN, LOW);
}

// Function to measure frequency of a signal as given in task 2
// worst case execution time = 4750 micros (without serial printing out of range Warning)
// worst case execution time = 127634 micros (with serial printing out of range Warning)
uint16_t freq1(){
  last_freq_signal_1 = freqCounter(FREQ_PIN_1, 333, 1000);
  return last_freq_signal_1;
}

// Function to measure frequency of a signal as given in task 3
// worst case execution time = 3175 micros (without serial printing out of range Warning)
// worst case execution time = 126091 micros (with serial printing out of range Warning)
uint16_t freq2(){
  last_freq_signal_2 = freqCounter(FREQ_PIN_2, 500, 1000);
  return last_freq_signal_2;
}

// Function to calculate average analog value using 4 readings as given in task 4
// worst case execution time = 217 micros
uint16_t avgAnalogVal(){
  sum = 0;
  //removing the oldest entry from analog_val_array and moving remaining entries to the begenning of the array so that new entry can be entered
  for(uint8_t i = 0; i <= ANALOG_AVG_COUNT-2; i++){
    analog_val_array[i] = analog_val_array[i+1];
    sum += analog_val_array[i];                                       // calculating the sum of previous 3 entries
  }
  analog_val_array[ANALOG_AVG_COUNT-1] = analogRead(ANALOG_READ_PIN); // adding new value at the end of the array
  sum+= analog_val_array[ANALOG_AVG_COUNT-1];                         // adding new value to sum for averaging
  return sum/ANALOG_AVG_COUNT;                                        // returning average of 4 readings
}

// Function to log data of task 2 and task 3 to Serial port as given in task 5
// worst case execution time = 122878 micros
void logSerial(){
  //mapping the frequencies to values given in task 5
  int8_t mapped_freq_1 = map(last_freq_signal_1, 333, 1000, 0 , 99);
  int8_t mapped_freq_2 = map(last_freq_signal_2, 500, 1000, 0 , 99);

  // limiting the range to given range in task 5
  if(mapped_freq_1 < 0)  mapped_freq_1 = 0;
  else if(mapped_freq_1 > 99) mapped_freq_1 = 99;
  if(mapped_freq_2 < 0)  mapped_freq_2 = 0;
  else if(mapped_freq_2 > 99) mapped_freq_2 = 99;

  //printing the result in mentioned form
  Serial.printf("%d,%d",mapped_freq_1, mapped_freq_2);
}

// Function to measure frequency of a given wave
uint16_t freqCounter(uint8_t read_pin, uint16_t minfreq, uint16_t maxfreq){
  max_timeperiod = 1000000/minfreq;                                                     // max timeperiod in micros
  max_timeperiod = max_timeperiod/2 + max_timeperiod*0.025;                             // max timeperiod for a wave with 50% percent duty cycle and 2.5% error margin
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