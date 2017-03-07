// JSON parsing library
#include <ArduinoJson.h>

// Android send/receive buffers
const size_t INPUT_BUFFER_SIZE = 512;
char input_buffer[INPUT_BUFFER_SIZE+1];

const size_t OUTPUT_BUFFER_SIZE = 576;
char output_buffer[OUTPUT_BUFFER_SIZE+3];

//Sampler sensor index. Negative until initialised
int sampler_index = -1;

//Output pins
const int pwm1 = 3;
const int pwm2 = 6;
const int dir1 = 2;
const int dir2 = 5;

bool run_time_exceeded[4];
bool active[4];
long start_time[4];
long prev_run_time[4];

bool first_command_recvd = false;

long max_run_time = 100000;

/**
 * Wrapper for ADK send command that copies data to debug port.
 * Requires a null-terminated char* pointer.
 */
void send(char *str) 
{ 
  // Add newline termination
  // TODO: Make sure we don't buffer overflow
  size_t len = strlen(str);
  str[len++] = '\r';
  str[len++] = '\n';
  str[len] = '\0';

  Serial1.print(str);
  // Copy string to debugging console.
  //Serial.print("-> ");
  //Serial.print(str);
}


/**
 * Handler to respond to incoming JSON commands and dispatch them to
 * configurable hardware components.
 */
int handleCommand(char *buffer)
{
  char param_name, param_value;
  sscanf(buffer, "{\"%c\":%c}", &param_name, &param_value);

  if (!set(param_name, param_value)) {
    //reportError("Invalid parameter set.", buffer);
    return -1;
  }
  else if(!first_command_recvd)
  {
    first_command_recvd = true;
  }

  return 0;

}

void setup() 
{
  delay(1000);

  //Initialize tx/rx pins
   Serial1.begin(9600);
  
  // Initialize debugging serial console.
  //Serial.begin(115200);


  input_buffer[INPUT_BUFFER_SIZE] = '\0';
  output_buffer[OUTPUT_BUFFER_SIZE] = '\0';
  

  //Initialise sampler
  pinMode(pwm1, OUTPUT);
  pinMode(dir1, OUTPUT);
  pinMode(pwm2, OUTPUT);
  pinMode(dir2, OUTPUT);
  
}

void loop() 
{
  // Index to last character in debug buffer.
  static size_t input_buffer_idx = 0;
  
  // Wait until characters are received.
  while (!Serial1.available()) yield();
  
  // Put the new character into the buffer, ignore \n and \r
  char c = Serial1.read();
 
  if (c != '\n' && c != '\r'){
    input_buffer[input_buffer_idx++] = c;
  }
  
  // If it is the end of a line, or we are out of space, parse the buffer.
  if (input_buffer_idx >= INPUT_BUFFER_SIZE || c == '\n' || c == '\r') 
  {
    // Properly null-terminate the buffer.
    input_buffer[input_buffer_idx] = '\0';
    input_buffer_idx = 0;
    
    // Attempt to parse command.
    //Serial.println(String("Received ") + input_buffer);
    if(handleCommand(input_buffer) < 0)
      return;
  
    //Wait for first command to be received
    if(!first_command_recvd)
      return;
    //byte for update message
    byte sampler_status = 0;
    //Check if the run time has been exceeded for any pump
    for(int i = 0; i < 4; i++)
    {
      if(active[i] && (prev_run_time[i] + millis() - start_time[i] > max_run_time))
      {
        active[i] = false;
        run_time_exceeded[i] = true;
        disable(i);
      }
      sampler_status |= run_time_exceeded[i] << (i*2 + 1);
      sampler_status |= active[i] << (i*2); 
    }
    char output_str[128];
    snprintf(output_str, 128,
             "{"
             "\"s\":\"%d\""
             "}",
             sampler_status
            );
    send(output_str);  
  }
}


bool set(const char param, const char value)
{
  // Set winch position
  if (param == 'e')
  { 
    int pump_num = value - '0';
    
    if(active[pump_num] || run_time_exceeded[pump_num])
      return true;
    active[pump_num] = true;
    enable(pump_num);
    return true;
  }
  else if (param == 'd')
  {
    int pump_num = value - '0';
    if(!active[pump_num])
      return true;
    active[pump_num] = false;
    disable(pump_num);
    return true;
  }
  else if (param == 'q')
  {
    return true;
  }
  //Reset pump
  else if (param == 'r')
  {
    for(int i = 0; i < 4; i++)
    {
      disable(i);
      active[i] = false;
      prev_run_time[i] = 0;
    }
    return true;
  }
  //Set timeout
  else if (param == 't')
  {
    //Timeout setting is in 10's of seconds
    max_run_time = (int)value*10000;
    return true;
  }
  // Return false for unknown command.
  else
  {
    return false;
  }
}

void enable(int pump_num)
{   
  switch(pump_num)
  {
    case 0: case 1:
          digitalWrite(pwm1, HIGH); 
          digitalWrite(dir1, pump_num == 0 ? HIGH : LOW); 
          break; 
    case 2: case 3: 
          digitalWrite(pwm2, HIGH); 
          digitalWrite(dir2, pump_num == 2 ? HIGH : LOW); 
          break;
    default: break;
  };
  //Record the pump start time
  start_time[pump_num] = millis();
}

void disable(int pump_num)
{
  digitalWrite(pump_num < 2 ? pwm1 : pwm2, LOW);
  //Record the run time for the current pump
  prev_run_time[pump_num] = millis() - start_time[pump_num];
}


