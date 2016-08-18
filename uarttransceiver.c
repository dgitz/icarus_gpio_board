#include "simpletools.h"
#include "mcp3208.h"
#include "fdserial.h"
#include "serialmessage.h"
#include "Definitions.h"
#include "servo.h"

#define SYSTEM ROVER
#define SUBSYSTEM ROBOT_CONTROLLER
#define COMPONENT GPIO_NODE
#define DIAGNOSTIC_TYPE SOFTWARE
typedef struct
{
  int Pin1_Mode;
  int Pin2_Mode;
  int Pin3_Mode;
  int Pin4_Mode;
  int Pin5_Mode;
  int Pin6_Mode;
  int Pin7_Mode;
  int Pin8_Mode;
  int Pin1_Number;
  int Pin2_Number;
  int Pin3_Number;
  int Pin4_Number;
  int Pin5_Number;
  int Pin6_Number;
  int Pin7_Number;
  int Pin8_Number;
  int Pin1_Value;
  int Pin2_Value;
  int Pin3_Value;
  int Pin4_Value;
  int Pin5_Value;
  int Pin6_Value;
  int Pin7_Value;
  int Pin8_Value;
} Port_Config;

void run_veryfastrate_code(); //1000 Hz
void run_fastrate_code(); //100 Hz
void run_mediumrate_code(); //10 Hz
void run_slowrate_code(); //1 Hz
void run_read_master_cog();
int veryfastrate_counter = 0;
int fastrate_counter = 0;
int mediumrate_counter = 0;
int slowrate_counter = 0;

#define ADC_CS 18
#define ADC_CLK 16
#define ADC_DATA 17
void Initialize_Ports();
char Setup_Ports();
char Update_Ports();
Port_Config update_dio_port_config(Port_Config PC);

char gpio_board_mode = GPIOBOARD_MODE_INITIALIZING;
char gpio_board_command = GPIOBOARD_MODE_UNDEFINED;


int *read_master_cog;
volatile fdserial *fd_device;
cogmain_counter = 0;
volatile int message_type = -1;
volatile int new_message = 0;
volatile int message_buffer[64];
volatile testmessagecounter = 0;
volatile int checksum_failed_counter = 0;
volatile int checksum_passed_counter = 0;
volatile int v1,v2,v3,v4,v5,v6,v7,v8;
volatile int initialized = 0;
double message_dropped_ratio = 0.0;
volatile char System = ROVER;
volatile char SubSystem = ROBOT_CONTROLLER;
volatile char Component = GPIO_NODE;
volatile char Diagnostic_Type = SOFTWARE;


Port_Config DIO_PortA;  
Port_Config DIO_PortB; 
Port_Config ANA_PortA; 
Port_Config ANA_PortB; 
void run_veryfastrate_code() //1000 Hz
{
 if((checksum_failed_counter == 0) && (checksum_passed_counter == 0)) {message_dropped_ratio = 0.0; }
  else
  {
     message_dropped_ratio= (double)((double)checksum_failed_counter/((double)checksum_passed_counter+(double)checksum_failed_counter));
  } 
  if(new_message == 1)
  {
    new_message = 0;
    if(message_type==SERIAL_Configure_DIO_PortA_ID)
    {
    }
    if(message_type==SERIAL_GPIO_Board_Mode_ID)
    {
      if(gpio_board_command == GPIOBOARD_MODE_START)
      {
        if(Setup_Ports() == 1) { gpio_board_mode = GPIOBOARD_MODE_RUNNING; }
      }  
      if(gpio_board_command == GPIOBOARD_MODE_RESET)
      {
        gpio_board_mode = GPIOBOARD_MODE_INITIALIZING;
        Initialize_Ports();
        if(Setup_Ports() == 1) { gpio_board_mode = GPIOBOARD_MODE_RUNNING; }
      }      
    } 
    if(message_type==SERIAL_Set_DIO_PortA_ID)
    {
    }            
  } 
}  
void run_fastrate_code() //100 Hz
{
  Update_Ports();
}  
void run_mediumrate_code() //10 Hz
{
  {
    int out_buffer[12];
    int length[1];
    int status = encode_Get_ANA_PortASerial(out_buffer,length,ANA_PortA.Pin1_Value,ANA_PortA.Pin2_Value,ANA_PortA.Pin3_Value,ANA_PortA.Pin4_Value);
    for(int i = 0; i < length[0]; i++)
    {
       fdserial_txChar(fd_device,out_buffer[i]);
    }
  }
  waitcnt(2000*(CLKFREQ/1000000)+CNT);
  {
    int out_buffer[12];
    int length[1];
    int status = encode_Get_ANA_PortBSerial(out_buffer,length,ANA_PortB.Pin1_Value,ANA_PortB.Pin2_Value,ANA_PortB.Pin3_Value,ANA_PortB.Pin4_Value);
    for(int i = 0; i < length[0]; i++)
    {
       fdserial_txChar(fd_device,out_buffer[i]);
    }
  }    
}  
void run_slowrate_code() //1 Hz
{
  int out_buffer[12];
  int length[1];
  Diagnostic_Type = COMMUNICATIONS;
  int Level;
  int Message;
  if(message_dropped_ratio < 3.0)
  {
    Level = INFO;
    Message = NOERROR;
  }
  else if((message_dropped_ratio >= 3.0) && (message_dropped_ratio < 6.0))
  {
    Level = WARN;
    Message = DROPPING_PACKETS;
  }     
  else if(message_dropped_ratio >= 6.0)
  {
    Level = FATAL;
    Message = DROPPING_PACKETS;
  } 
         
  int status = encode_DiagnosticSerial(out_buffer,length,System,SubSystem,Component,Diagnostic_Type,Level,Message);
  for(int i = 0; i < length[0]; i++)
  {
     fdserial_txChar(fd_device,out_buffer[i]);
  }
}  
int main()
{
  initialized = 0;
  Initialize_Ports();
   fd_device = fdserial_open(26, 27, 0, 115200);
  read_master_cog = cog_run(run_read_master_cog,256);
  initialized = 1;
  while(1)
  { 
    veryfastrate_counter++;
    fastrate_counter++;
    mediumrate_counter++;
    slowrate_counter++;
    if(veryfastrate_counter >= 1) //1000 Hz
    {
      veryfastrate_counter = 0;
      run_veryfastrate_code();
    }  
    if(fastrate_counter >= 10) //100 Hz
    {
      fastrate_counter = 0;
      run_fastrate_code();
    }
    if(mediumrate_counter >= 100) //10 Hz
    {
      mediumrate_counter = 0;
      run_mediumrate_code();
    }
    if(slowrate_counter >= 1000) //1 Hz
    {
      slowrate_counter = 0;
      run_slowrate_code();
    }                    
   
    waitcnt(1000*(CLKFREQ/1000000)+CNT);
  }    
}
void run_read_master_cog()
{
  int message_checksum;
  int computed_checksum;
  int message_length;

  while(1)
  {
    char c;
    while(c = fdserial_rxChar(fd_device) != 0xAB);
    c = fdserial_rxChar(fd_device);
    message_length = fdserial_rxChar(fd_device);
    for(int i = 0; i < message_length; i++)
    {
      message_buffer[i] = fdserial_rxChar(fd_device);
    }   
    message_checksum = fdserial_rxChar(fd_device);
    int status =0;
    if(c == SERIAL_Configure_DIO_PortA_ID)
    {
      
      message_type = c;
      char value1,value2,value3,value4,value5,value6,value7,value8;
      status = decode_Configure_DIO_PortASerial(message_buffer,message_length,message_checksum,&value1,&value2,&value3,&value4,&value5,&value6,&value7,&value8);
      if(status == 1)
      {
        checksum_passed_counter++;
        
        DIO_PortA.Pin1_Mode = value1;
        DIO_PortA.Pin2_Mode = value2;
        DIO_PortA.Pin3_Mode = value3;
        DIO_PortA.Pin4_Mode = value4;
        DIO_PortA.Pin5_Mode = value5;
        DIO_PortA.Pin6_Mode = value6;
        DIO_PortA.Pin7_Mode = value7;
        DIO_PortA.Pin8_Mode = value8;
        new_message = 1;
      }
    }
    else if(c == SERIAL_GPIO_Board_Mode_ID)
    {
      message_type = c;
      char value1;
      status = decode_GPIO_Board_ModeSerial(message_buffer,message_length,message_checksum,&value1);
      if(status == 1)
      {
        checksum_passed_counter++;
        new_message = 1;
        gpio_board_command = value1;
      }
    }
    else if(c == SERIAL_Set_DIO_PortA_ID)
    {
      
      message_type = c;
      char value1,value2,value3,value4,value5,value6,value7,value8;
      status = decode_Set_DIO_PortASerial(message_buffer,message_length,message_checksum,&value1,&value2,&value3,&value4,&value5,&value6,&value7,&value8);
      if(status == 1)
      {
        checksum_passed_counter++;
        if((DIO_PortA.Pin1_Mode == PINMODE_DIGITAL_OUTPUT) || (DIO_PortA.Pin1_Mode == PINMODE_PWM_OUTPUT)) {DIO_PortA.Pin1_Value=value1; }
        if((DIO_PortA.Pin2_Mode == PINMODE_DIGITAL_OUTPUT) || (DIO_PortA.Pin2_Mode == PINMODE_PWM_OUTPUT)) {DIO_PortA.Pin2_Value=value2; }
        if((DIO_PortA.Pin3_Mode == PINMODE_DIGITAL_OUTPUT) || (DIO_PortA.Pin3_Mode == PINMODE_PWM_OUTPUT)) {DIO_PortA.Pin3_Value=value3; }
        if((DIO_PortA.Pin4_Mode == PINMODE_DIGITAL_OUTPUT) || (DIO_PortA.Pin4_Mode == PINMODE_PWM_OUTPUT)) {DIO_PortA.Pin4_Value=value4; }
        if((DIO_PortA.Pin5_Mode == PINMODE_DIGITAL_OUTPUT) || (DIO_PortA.Pin5_Mode == PINMODE_PWM_OUTPUT)) {DIO_PortA.Pin5_Value=value5; }
        if((DIO_PortA.Pin6_Mode == PINMODE_DIGITAL_OUTPUT) || (DIO_PortA.Pin6_Mode == PINMODE_PWM_OUTPUT)) {DIO_PortA.Pin6_Value=value6; }
        if((DIO_PortA.Pin7_Mode == PINMODE_DIGITAL_OUTPUT) || (DIO_PortA.Pin7_Mode == PINMODE_PWM_OUTPUT)) {DIO_PortA.Pin7_Value=value7; }
        if((DIO_PortA.Pin8_Mode == PINMODE_DIGITAL_OUTPUT) || (DIO_PortA.Pin8_Mode == PINMODE_PWM_OUTPUT)) {DIO_PortA.Pin8_Value=value8; }
        new_message = 1;
      }
    }
    if(status < 0)
    {
      checksum_failed_counter++;
    } 
  }
  
}          
void Initialize_Ports()
{
  DIO_PortA.Pin1_Mode = PINMODE_UNDEFINED;
  DIO_PortA.Pin2_Mode = PINMODE_UNDEFINED;
  DIO_PortA.Pin3_Mode = PINMODE_UNDEFINED;
  DIO_PortA.Pin4_Mode = PINMODE_UNDEFINED;
  DIO_PortA.Pin5_Mode = PINMODE_UNDEFINED;
  DIO_PortA.Pin6_Mode = PINMODE_UNDEFINED;
  DIO_PortA.Pin7_Mode = PINMODE_UNDEFINED;
  DIO_PortA.Pin8_Mode = PINMODE_UNDEFINED;
  DIO_PortA.Pin1_Number = 0; //P0
  DIO_PortA.Pin2_Number = 1; //P1
  DIO_PortA.Pin3_Number = 2; //P2
  DIO_PortA.Pin4_Number = 3; //P3
  DIO_PortA.Pin5_Number = 4; //P4
  DIO_PortA.Pin6_Number = 5; //P5
  DIO_PortA.Pin7_Number = 6; //P6
  DIO_PortA.Pin8_Number = 7; //P7
  DIO_PortA.Pin8_Number = 8; //P8
  DIO_PortA.Pin1_Value = 0;
  DIO_PortA.Pin2_Value = 0;
  DIO_PortA.Pin3_Value = 0;
  DIO_PortA.Pin4_Value = 0;
  DIO_PortA.Pin5_Value = 0;
  DIO_PortA.Pin6_Value = 0;
  DIO_PortA.Pin7_Value = 0;
  DIO_PortA.Pin8_Value = 0;
}                       
char Setup_Ports()
{
  //First do PortA
  if(DIO_PortA.Pin1_Mode == PINMODE_DIGITAL_OUTPUT)
  {
  }
  if(DIO_PortA.Pin2_Mode == PINMODE_DIGITAL_OUTPUT)
  {
  }
  if(DIO_PortA.Pin3_Mode == PINMODE_DIGITAL_OUTPUT)
  {
  }
  if(DIO_PortA.Pin4_Mode == PINMODE_DIGITAL_OUTPUT)
  {
  }
  if(DIO_PortA.Pin5_Mode == PINMODE_DIGITAL_OUTPUT)
  {
  }
  if(DIO_PortA.Pin6_Mode == PINMODE_DIGITAL_OUTPUT)
  {
  }
  if(DIO_PortA.Pin7_Mode == PINMODE_DIGITAL_OUTPUT)
  {
  }
  if(DIO_PortA.Pin8_Mode == PINMODE_DIGITAL_OUTPUT)
  {
  }  
  return 1;  
  
}  
char Update_Ports()
{
  //First read ANA PortA and PortB.
  ANA_PortA.Pin1_Value =  readADC(0, ADC_DATA, ADC_CLK, ADC_CS);
  ANA_PortA.Pin2_Value = readADC(1, ADC_DATA, ADC_CLK, ADC_CS);
  ANA_PortA.Pin3_Value = readADC(2, ADC_DATA, ADC_CLK, ADC_CS);
  ANA_PortA.Pin4_Value = readADC(3, ADC_DATA, ADC_CLK, ADC_CS);
  ANA_PortB.Pin1_Value = readADC(4, ADC_DATA, ADC_CLK, ADC_CS);
  ANA_PortB.Pin2_Value = readADC(5, ADC_DATA, ADC_CLK, ADC_CS);
  ANA_PortB.Pin3_Value = readADC(6, ADC_DATA, ADC_CLK, ADC_CS);
  ANA_PortB.Pin4_Value = readADC(7, ADC_DATA, ADC_CLK, ADC_CS);
  //The do DIO PortA
  DIO_PortA = update_dio_port_config(DIO_PortA);
  DIO_PortB = update_dio_port_config(DIO_PortB);  
  
  return 1;    
}  
 
Port_Config update_dio_port_config(Port_Config PC)
{
   if(PC.Pin1_Mode == PINMODE_DIGITAL_OUTPUT)
  {
    if(PC.Pin1_Value > 0) {high(PC.Pin1_Number); }
    else{low(PC.Pin1_Number); }
  }
  else if(PC.Pin1_Mode == PINMODE_PWM_OUTPUT)
  {
    servo_angle(PC.Pin1_Number,PC.Pin1_Value*1800/256);
  }
  
  if(PC.Pin2_Mode == PINMODE_DIGITAL_OUTPUT)
  {
    if(PC.Pin2_Value > 0) {high(PC.Pin2_Number); }
    else{low(PC.Pin2_Number); }
  }
  else if(PC.Pin2_Mode == PINMODE_PWM_OUTPUT)
  {
    servo_angle(PC.Pin2_Number,PC.Pin2_Value*1800/256);
  }
  
  if(PC.Pin3_Mode == PINMODE_DIGITAL_OUTPUT)
  {
    if(PC.Pin3_Value > 0) {high(PC.Pin3_Number); }
    else{low(PC.Pin3_Number); }
  }
  else if(PC.Pin3_Mode == PINMODE_PWM_OUTPUT)
  {
    servo_angle(PC.Pin3_Number,PC.Pin3_Value*1800/256);
  }
  
  if(PC.Pin4_Mode == PINMODE_DIGITAL_OUTPUT)
  {
    if(PC.Pin4_Value > 0) {high(PC.Pin4_Number); }
    else{low(PC.Pin4_Number); }
  }
  else if(PC.Pin4_Mode == PINMODE_PWM_OUTPUT)
  {
    servo_angle(PC.Pin4_Number,PC.Pin4_Value*1800/256);
  }
  
  if(PC.Pin4_Mode == PINMODE_DIGITAL_OUTPUT)
  {
    if(PC.Pin4_Value > 0) {high(PC.Pin4_Number); }
    else{low(PC.Pin4_Number); }
  }
  else if(PC.Pin4_Mode == PINMODE_PWM_OUTPUT)
  {
    servo_angle(PC.Pin4_Number,PC.Pin4_Value*1800/256);
  }
  
  if(PC.Pin5_Mode == PINMODE_DIGITAL_OUTPUT)
  {
    if(PC.Pin5_Value > 0) {high(PC.Pin5_Number); }
    else{low(PC.Pin5_Number); }
  }
  else if(PC.Pin5_Mode == PINMODE_PWM_OUTPUT)
  {
    servo_angle(PC.Pin5_Number,PC.Pin5_Value*1800/256);
  }
  
  if(PC.Pin6_Mode == PINMODE_DIGITAL_OUTPUT)
  {
    if(PC.Pin6_Value > 0) {high(PC.Pin6_Number); }
    else{low(PC.Pin6_Number); }
  }
  else if(PC.Pin6_Mode == PINMODE_PWM_OUTPUT)
  {
    servo_angle(PC.Pin6_Number,PC.Pin6_Value*1800/256);
  }
  
  if(PC.Pin7_Mode == PINMODE_DIGITAL_OUTPUT)
  {
    if(PC.Pin7_Value > 0) {high(PC.Pin7_Number); }
    else{low(PC.Pin7_Number); }
  }
  else if(PC.Pin7_Mode == PINMODE_PWM_OUTPUT)
  {
    servo_angle(PC.Pin7_Number,PC.Pin7_Value*1800/256);
  }
  
  if(PC.Pin8_Mode == PINMODE_DIGITAL_OUTPUT)
  {
    if(PC.Pin8_Value > 0) {high(PC.Pin8_Number); }
    else{low(PC.Pin8_Number); }
  }
  else if(PC.Pin8_Mode == PINMODE_PWM_OUTPUT)
  {
    servo_angle(PC.Pin8_Number,PC.Pin8_Value*1800/256);
  }
  return PC;
  
}  
 
 
 
 
 
 
 
 
 
 
