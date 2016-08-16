#include "simpletools.h"
#include "fdserial.h"
#include "serialmessage.h"
#include "Definitions.h"
#define SYSTEM ROVER
#define SUBSYSTEM ROBOT_CONTROLLER
#define COMPONENT GPIO_NODE
#define DIAGNOSTIC_TYPE SOFTWARE


void run_veryfastrate_code(); //1000 Hz
void run_fastrate_code(); //100 Hz
void run_mediumrate_code(); //10 Hz
void run_slowrate_code(); //1 Hz
void run_read_master_cog();
int veryfastrate_counter = 0;
int fastrate_counter = 0;
int mediumrate_counter = 0;
int slowrate_counter = 0;

void Initialize_Ports();
int Setup_Ports();
int Update_Ports();
int gpio_board_mode = GPIOBOARD_MODE_INITIALIZING;
int gpio_board_command = GPIOBOARD_MODE_UNDEFINED;

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
Port_Config PortA;  
void run_veryfastrate_code() //1000 Hz
{
  if((checksum_failed_counter == 0) && (checksum_passed_counter == 0)) {message_dropped_ratio = 0.0; }
  else
  {
     message_dropped_ratio= 100.0*(double)((double)checksum_failed_counter/((double)checksum_passed_counter+(double)checksum_failed_counter));
  } 
  if(new_message == 1)
  {
    printf("checksum_failed_counter: %d\n",checksum_failed_counter);
    new_message = 0;
    if(message_type==SERIAL_TestMessageCommand_ID)
    {       
    }
    if(message_type==SERIAL_Configure_GPIO_PortA_ID)
    {
      printf("Got Command: SERIAL_Configure_GPIO_PortA_ID\n");
    }
    if(message_type==SERIAL_GPIO_Board_Mode_ID)
    {
      printf("Got Command: SERIAL_GPIO_Board_Mode_ID\n");
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
    if(message_type==SERIAL_Set_GPIO_PortA_ID)
    {
      printf("Got Command to set Pin1 to: %d\n",PortA.Pin1_Value);
    }            
  }     
}  
void run_fastrate_code() //100 Hz
{
  Update_Ports();
}  
void run_mediumrate_code() //10 Hz
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
    //printf("Sending: ");
    for(int i = 0; i < length[0]; i++)
    {
        //printf("b: %d D: %d",out_buffer[i]);
       fdserial_txChar(fd_device,out_buffer[i]);
    }    
}  
void run_slowrate_code() //1 Hz
{
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
    pause(1);
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
    if(c == SERIAL_TestMessageCounter_ID)
    {
      
      new_message = 1;
      message_type = c;
    }  
    else if(c == SERIAL_TestMessageCommand_ID)
    {
      
      message_type = c;
      char value1,value2,value3,value4,value5,value6,value7,value8;
      status = decode_TestMessageCommandSerial(message_buffer,message_length,message_checksum,&value1,&value2,&value3,&value4,&value5,&value6,&value7,&value8);
      if(status == 1)
      {
        checksum_passed_counter++;
        v1 = value1;
        v2 = value2;
        v3 = value3;
        v4 = value4;
        v5 = value5;
        v6 = value6;
        v7 = value7;
        v8 = value8;
        new_message = 1;
      }
    }
    else if(c == SERIAL_Configure_GPIO_PortA_ID)
    {
      
      message_type = c;
      char value1,value2,value3,value4,value5,value6,value7,value8;
      status = decode_Configure_GPIO_PortASerial(message_buffer,message_length,message_checksum,&value1,&value2,&value3,&value4,&value5,&value6,&value7,&value8);
      if(status == 1)
      {
        checksum_passed_counter++;
        
        PortA.Pin1_Mode = value1;
        PortA.Pin2_Mode = value2;
        PortA.Pin3_Mode = value3;
        PortA.Pin4_Mode = value4;
        PortA.Pin5_Mode = value5;
        PortA.Pin6_Mode = value6;
        PortA.Pin7_Mode = value7;
        PortA.Pin8_Mode = value8;
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
    else if(c == SERIAL_Set_GPIO_PortA_ID)
    {
      
      message_type = c;
      char value1,value2,value3,value4,value5,value6,value7,value8;
      status = decode_Set_GPIO_PortASerial(message_buffer,message_length,message_checksum,&value1,&value2,&value3,&value4,&value5,&value6,&value7,&value8);
      if(status == 1)
      {
        checksum_passed_counter++;
        if(PortA.Pin1_Mode == PINMODE_DIGITAL_OUTPUT) {PortA.Pin1_Value=value1; }
        if(PortA.Pin2_Mode == PINMODE_DIGITAL_OUTPUT) {PortA.Pin2_Value=value2; }
        if(PortA.Pin3_Mode == PINMODE_DIGITAL_OUTPUT) {PortA.Pin3_Value=value3; }
        if(PortA.Pin4_Mode == PINMODE_DIGITAL_OUTPUT) {PortA.Pin4_Value=value4; }
        if(PortA.Pin5_Mode == PINMODE_DIGITAL_OUTPUT) {PortA.Pin5_Value=value5; }
        if(PortA.Pin6_Mode == PINMODE_DIGITAL_OUTPUT) {PortA.Pin6_Value=value6; }
        if(PortA.Pin7_Mode == PINMODE_DIGITAL_OUTPUT) {PortA.Pin7_Value=value7; }
        if(PortA.Pin8_Mode == PINMODE_DIGITAL_OUTPUT) {PortA.Pin8_Value=value8; }
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
  PortA.Pin1_Mode = PINMODE_UNDEFINED;
  PortA.Pin2_Mode = PINMODE_UNDEFINED;
  PortA.Pin3_Mode = PINMODE_UNDEFINED;
  PortA.Pin4_Mode = PINMODE_UNDEFINED;
  PortA.Pin5_Mode = PINMODE_UNDEFINED;
  PortA.Pin6_Mode = PINMODE_UNDEFINED;
  PortA.Pin7_Mode = PINMODE_UNDEFINED;
  PortA.Pin8_Mode = PINMODE_UNDEFINED;
  PortA.Pin1_Number = 0; //P0
  PortA.Pin2_Number = 1; //P1
  PortA.Pin3_Number = 2; //P2
  PortA.Pin4_Number = 3; //P3
  PortA.Pin5_Number = 4; //P4
  PortA.Pin6_Number = 5; //P5
  PortA.Pin7_Number = 6; //P6
  PortA.Pin8_Number = 7; //P7
  PortA.Pin8_Number = 8; //P8
  PortA.Pin1_Value = 0;
  PortA.Pin2_Value = 0;
  PortA.Pin3_Value = 0;
  PortA.Pin4_Value = 0;
  PortA.Pin5_Value = 0;
  PortA.Pin6_Value = 0;
  PortA.Pin7_Value = 0;
  PortA.Pin8_Value = 0;
}                       
int Setup_Ports()
{
  //First do PortA
  if(PortA.Pin1_Mode == PINMODE_DIGITAL_OUTPUT)
  {
  }
  if(PortA.Pin2_Mode == PINMODE_DIGITAL_OUTPUT)
  {
  }
  if(PortA.Pin3_Mode == PINMODE_DIGITAL_OUTPUT)
  {
  }
  if(PortA.Pin4_Mode == PINMODE_DIGITAL_OUTPUT)
  {
  }
  if(PortA.Pin5_Mode == PINMODE_DIGITAL_OUTPUT)
  {
  }
  if(PortA.Pin6_Mode == PINMODE_DIGITAL_OUTPUT)
  {
  }
  if(PortA.Pin7_Mode == PINMODE_DIGITAL_OUTPUT)
  {
  }
  if(PortA.Pin8_Mode == PINMODE_DIGITAL_OUTPUT)
  {
  }  
  return 1;  
  
}  
int Update_Ports()
{
  //First do PortA
  if(PortA.Pin1_Mode == PINMODE_DIGITAL_OUTPUT)
  {
    if(PortA.Pin1_Value > 0) {high(PortA.Pin1_Number); }
    else{low(PortA.Pin1_Number); }
  }
  
  return 1;    
}  
 
 
 
 
 
 
 
 
 
 
