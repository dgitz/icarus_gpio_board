#include "simpletools.h"
#include "mcp3208.h"
#include "fdserial.h"
#include "serialmessage.h"
#include "Definitions.h"
#include "servo.h"

#define FIRMWARE_MAJOR_VERSION 2
#define FIRMWARE_MINOR_VERSION 1
#define FIRMWARE_BUILD_NUMBER 1
#define SYSTEM ROVER
#define SUBSYSTEM ROBOT_CONTROLLER
#define COMPONENT GPIO_NODE
#define DIAGNOSTIC_TYPE SOFTWARE
typedef struct
{
  int Pin_Mode[8];
  int Pin_Number[8];
  int Pin_Value[8];
} Port_Config;

void run_veryfastrate_code(); //1000 Hz
void run_fastrate_code(); //100 Hz
void run_mediumrate_code(); //10 Hz
void run_slowrate_code(); //1 Hz
void run_receive_master_cog();
void run_transmit_master_cog();
int veryfastrate_counter = 0;
int fastrate_counter = 0;
int mediumrate_counter = 0;
int slowrate_counter = 0;

#define ADC_CS 18
#define ADC_CLK 16
#define ADC_DATA 17
#define STATUS_LED 25
#define ERROR_LED 24
void Initialize_Ports();
char Setup_Ports();
char Update_Ports();
void display_firmware();
Port_Config update_dio_port_config(Port_Config PC);

char board_mode = GPIO_MODE_UNDEFINED;
char prev_board_mode = GPIO_MODE_UNDEFINED;
char node_mode = GPIO_MODE_UNDEFINED;


int *receive_master_cog;
int *transmit_master_cog;
fdserial *fd_device;
volatile int Send_DIO_PortA = 0;
volatile int Send_DIO_PortB = 0;
volatile int Send_ANA_PortA = 0;
volatile int Send_ANA_PortB = 0;
volatile int Send_BoardMode = 0;
volatile int Send_Firmware = 0;
volatile int Send_Diagnostics = 0;

volatile long Send_Firmware_Counter = 0;
volatile long Send_DIO_PortA_Counter = 0;
volatile long Send_DIO_PortB_Counter = 0;
volatile long Send_ANA_PortA_Counter = 0;
volatile long Send_ANA_PortB_Counter = 0;
volatile long Send_BoardMode_Counter = 0;
volatile long Send_Diagnostics_Counter = 0;
volatile long Receive_Configure_DIO_PortA_Counter = 0;
volatile long Receive_Configure_DIO_PortB_Counter = 0;
volatile long Receive_BoardMode_Counter = 0;
volatile long Receive_Set_DIO_PortA_Counter = 0;
volatile long Receive_Set_DIO_PortB_Counter = 0;
int Diagnostic_Level = INFO;
int Diagnostic_Message = INITIALIZING;
volatile int cogmain_counter = 0;
volatile int message_type = -1;
volatile int new_message = 0;
volatile int message_buffer[64];
volatile int testmessagecounter = 0;
volatile int checksum_failed_counter = 0;
volatile int checksum_passed_counter = 0;
volatile int v1,v2,v3,v4,v5,v6,v7,v8;
volatile int initialized = 0;
double message_dropped_ratio = 0.0;
volatile char System = ROVER;
volatile char SubSystem = ROBOT_CONTROLLER;
volatile char Component = GPIO_NODE;
volatile char Diagnostic_Type = SOFTWARE;
volatile int checking_gpio_comm = 0;
volatile int received_config_DIO_PortA = 0;
volatile int received_config_DIO_PortB = 0;

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
    if(message_type != 23)
    {
      //printf("mes: %d\n",message_type);
    }      
  }     
}
void run_fastrate_code() //100 Hz
{
  Update_Ports();
  if(board_mode == GPIO_MODE_RUNNING)
  {
    Send_ANA_PortA = 1;
    Send_ANA_PortB = 1;
    Send_DIO_PortA = 1;
    Send_DIO_PortB = 1;
  }    
  if(board_mode != prev_board_mode)
  {
    Send_BoardMode = 1;
    prev_board_mode = board_mode;
  }
      
}  
void run_mediumrate_code() //10 Hz
{
  if(input(STATUS_LED) == 1){ low(STATUS_LED); }
  else { high(STATUS_LED); }
  /*printf("0: mode: %d number: %d value: %d\n",
    DIO_PortA.Pin_Mode[0],
    DIO_PortA.Pin_Number[0],
    DIO_PortA.Pin_Value[0]);
    */
  
}  
void run_slowrate_code() //1 Hz
{
  /*
  printf("Failed Checksum: %d Passed Checksum: %d\n",checksum_failed_counter,checksum_passed_counter);
  printf("Received Config DIO PortA (0xAB16) %d times.\n",Receive_Configure_DIO_PortA_Counter);
  printf("Received Config DIO PortB (0xAB21) %d times.\n",Receive_Configure_DIO_PortB_Counter);
  printf("Received Node Mode (0xAB17) %d times.\n",Receive_BoardMode_Counter);
  printf("Received Set DIO PortA (0x23) %d times.\n",Receive_Set_DIO_PortA_Counter);
  printf("Send Board Mode (0xAB17) %d times.\n",Send_BoardMode_Counter);
  */

  int out_buffer[12];
  int length[1];
  Diagnostic_Type = COMMUNICATIONS;
  if(message_dropped_ratio < 3.0)
  {
    Diagnostic_Level = INFO;
    Diagnostic_Message = NOERROR;
  }
  else if((message_dropped_ratio >= 3.0) && (message_dropped_ratio < 6.0))
  {
    high(ERROR_LED);
    Diagnostic_Level = WARN;
    Diagnostic_Message = DROPPING_PACKETS;
  }     
  else if(message_dropped_ratio >= 6.0)
  {
    high(ERROR_LED);
    Diagnostic_Level = FATAL;
    Diagnostic_Message = DROPPING_PACKETS;
  } 
  Send_Diagnostics = 1;
  Send_BoardMode = 1;
  if(board_mode == GPIO_MODE_INITIALIZED)
  {
    board_mode = GPIO_MODE_RUNNING;
  }    
 
}  
int main()
{
  printf("Mode: %d\n",board_mode);
  simpleterm_close();
  board_mode = GPIO_MODE_BOOT;
  initialized = 0;
  //display_firmware();
  high(STATUS_LED);
  high(ERROR_LED);
  Initialize_Ports();
  fd_device = fdserial_open(26, 27, 0, 115200);
  Send_BoardMode = 1;
  Send_Firmware = 1; 
  receive_master_cog = cog_run(run_receive_master_cog,512);
  transmit_master_cog = cog_run(run_transmit_master_cog,512);
  initialized = 1;
  pause(1000);
  low(ERROR_LED);
  prev_board_mode = board_mode; 
  board_mode = GPIO_MODE_INITIALIZING; 
  //printf("Mode: %d\n",board_mode);
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
      //printf("Board Mode: %d Node Mode: %d\n",board_mode,node_mode);
      mediumrate_counter = 0;
      run_mediumrate_code();
    }
    if(slowrate_counter >= 1000) //1 Hz
    {
      //printf("Executing: %d\n",slowrate_counter);
      slowrate_counter = 0;
      run_slowrate_code();
    }                    
   
    waitcnt(1000*(CLKFREQ/1000000)+CNT);
  }    
}
void run_transmit_master_cog()
{
  while(1)  
  {
    
    if(Send_Firmware == 1)
    {
      Send_Firmware = 0;  
      int out_buffer[12];
      int length[1];
      int status = encode_FirmwareVersionSerial(out_buffer,length,FIRMWARE_MAJOR_VERSION,FIRMWARE_MINOR_VERSION,FIRMWARE_BUILD_NUMBER);
      if(status == 1)
      {
        Send_Firmware_Counter++;
        for(int i = 0; i < length[0]; i++)
        {
          fdserial_txChar(fd_device,out_buffer[i]);
        } 
        waitcnt(4000*(CLKFREQ/1000000)+CNT);      
      }        
    }
    if(Send_Diagnostics == 1)
    {
      Send_Diagnostics = 0;
      int out_buffer[12];
      int length[1];
      int status = encode_DiagnosticSerial(out_buffer,length,System,SubSystem,Component,Diagnostic_Type,Diagnostic_Level,Diagnostic_Message);
      if(status == 1)
      {  
        Send_Diagnostics_Counter++;    
        for(int i = 0; i < length[0]; i++)
        {
           fdserial_txChar(fd_device,out_buffer[i]);
        }         
        waitcnt(4000*(CLKFREQ/1000000)+CNT);        
      }
    }  
     
    
    if(Send_DIO_PortA == 1)
    {
      Send_DIO_PortA = 0;
      int out_buffer[12];  
      int length[1];
      int status = encode_Get_DIO_PortASerial(out_buffer,length,
      DIO_PortA.Pin_Value[0],DIO_PortA.Pin_Value[1],DIO_PortA.Pin_Value[2],DIO_PortA.Pin_Value[3],
      DIO_PortA.Pin_Value[4],DIO_PortA.Pin_Value[5],DIO_PortA.Pin_Value[6],DIO_PortA.Pin_Value[7]);
      if(status == 1)
      {
        Send_DIO_PortA_Counter++;
        for(int i = 0; i < length[0]; i++)
        {
           fdserial_txChar(fd_device,out_buffer[i]);
        }
        waitcnt(4000*(CLKFREQ/1000000)+CNT);
      }        
    }   
    if(Send_DIO_PortB == 1)
    {
      Send_DIO_PortB = 0;
      int out_buffer[12];  
      int length[1];
      int status = encode_Get_DIO_PortBSerial(out_buffer,length,
      DIO_PortB.Pin_Value[0],DIO_PortB.Pin_Value[1],DIO_PortB.Pin_Value[2],DIO_PortB.Pin_Value[3],
      DIO_PortB.Pin_Value[4],DIO_PortB.Pin_Value[5],DIO_PortB.Pin_Value[6],DIO_PortB.Pin_Value[7]);
      if(status == 1)
      {
        Send_DIO_PortB_Counter++;
        for(int i = 0; i < length[0]; i++)
        {
          fdserial_txChar(fd_device,out_buffer[i]);
        }
        waitcnt(4000*(CLKFREQ/1000000)+CNT);      
      }        
    }     
    if(Send_ANA_PortA == 1)
    {
      Send_ANA_PortA = 0;
      int out_buffer[12];
      int length[1];
      int status = encode_Get_ANA_PortASerial(out_buffer,length,ANA_PortA.Pin_Value[0],ANA_PortA.Pin_Value[1],ANA_PortA.Pin_Value[2],ANA_PortA.Pin_Value[3]);
      if(status == 1)
      {
        Send_ANA_PortA_Counter++;
        for(int i = 0; i < length[0]; i++)
        {
           fdserial_txChar(fd_device,out_buffer[i]);
        }
        waitcnt(4000*(CLKFREQ/1000000)+CNT);
      }        
    } 
    if(Send_ANA_PortB == 1)
    {
      Send_ANA_PortB = 0;
      int out_buffer[12];
      int length[1];
      int status = encode_Get_ANA_PortBSerial(out_buffer,length,ANA_PortB.Pin_Value[0],ANA_PortB.Pin_Value[1],ANA_PortB.Pin_Value[2],ANA_PortB.Pin_Value[3]);
      if(status == 1)
      {
        Send_ANA_PortB_Counter++;
        for(int i = 0; i < length[0]; i++)
        {
           fdserial_txChar(fd_device,out_buffer[i]);
        }   
        waitcnt(4000*(CLKFREQ/1000000)+CNT);
      }        
    }         
    if(Send_BoardMode == 1)
    {
      Send_BoardMode = 0;
      int out_buffer[12];  
      int length[1];
      int status = encode_ModeSerial(out_buffer,length,board_mode);
      if(status == 1)
      {
        Send_BoardMode_Counter++;
        for(int i = 0; i < length[0]; i++)
        {
          fdserial_txChar(fd_device,out_buffer[i]);
        }
        waitcnt(4000*(CLKFREQ/1000000)+CNT);
      }        
    }  
  }
} 
void run_receive_master_cog()
{
  int message_checksum;
  int computed_checksum;
  int message_length;
  simpleterm_open();
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
     
   /* for(int i = 0; i < message_length;i++)
    {
      printf(" i: %d b: %d",i,message_buffer[i]);
    }      
    printf(" len: %d checksum: %d\n",message_length,message_checksum);
    */
    int status =0;
    if(c == SERIAL_Configure_DIO_PortA_ID)
    {
      message_type = c;
      char value1,value2,value3,value4,value5,value6,value7,value8;
      status = decode_Configure_DIO_PortASerial(message_buffer,message_length,message_checksum,&value1,&value2,&value3,&value4,&value5,&value6,&value7,&value8);
      for(int i = 0; i < message_length;i++)
      {
        printf("i: %d b: %d\n",i,message_buffer[i]);
      }  
      if(status == 1)
      {
        Receive_Configure_DIO_PortA_Counter++;
        received_config_DIO_PortA = 1;
        checksum_passed_counter++;
        DIO_PortA.Pin_Mode[0] = value1;
        DIO_PortA.Pin_Mode[1] = value2;
        DIO_PortA.Pin_Mode[2] = value3;
        DIO_PortA.Pin_Mode[3] = value4;
        DIO_PortA.Pin_Mode[4] = value5;
        DIO_PortA.Pin_Mode[5] = value6;
        DIO_PortA.Pin_Mode[6] = value7;
        DIO_PortA.Pin_Mode[7] = value8;
        new_message = 1;
      }
      if(status < 0)  
      {
        checksum_failed_counter++;
      }
    }
    else if(c == SERIAL_TestMessageCommand_ID)
    {
      message_type = c;
      char value1,value2,value3,value4,value5,value6,value7,value8;
      status = decode_TestMessageCommandSerial(message_buffer,message_length,message_checksum,&value1,&value2,&value3,&value4,&value5,&value6,&value7,&value8);
      if(status == 1)
      {
        checksum_passed_counter++;
        new_message = 1;
      }
      if(status < 0)  
      {
        checksum_failed_counter++;
      }
    }
    else if(c == SERIAL_Configure_DIO_PortB_ID)
    {
      message_type = c;
      char value1,value2,value3,value4,value5,value6,value7,value8;
      status = decode_Configure_DIO_PortBSerial(message_buffer,message_length,message_checksum,&value1,&value2,&value3,&value4,&value5,&value6,&value7,&value8);
      if(status == 1)
      {
        Receive_Configure_DIO_PortB_Counter++;
        received_config_DIO_PortB = 1;
        if(received_config_DIO_PortA == 1)
        { 
          prev_board_mode = board_mode;
          board_mode = GPIO_MODE_INITIALIZED; 
        }
        checksum_passed_counter++;
        DIO_PortB.Pin_Mode[0] = value1;
        DIO_PortB.Pin_Mode[1] = value2;
        DIO_PortB.Pin_Mode[2] = value3;
        DIO_PortB.Pin_Mode[3] = value4;
        DIO_PortB.Pin_Mode[4] = value5;
        DIO_PortB.Pin_Mode[5] = value6;
        DIO_PortB.Pin_Mode[6] = value7;
        DIO_PortB.Pin_Mode[7] = value8;
        new_message = 1;
      }
      if(status < 0)  
      {
        checksum_failed_counter++;
      }
    }
    else if(c == SERIAL_Mode_ID)
    {
      message_type = c;
      char value1;
      status = decode_ModeSerial(message_buffer,message_length,message_checksum,&value1);
      if(status == 1)
      {
        Receive_BoardMode_Counter++;
        checksum_passed_counter++;
        new_message = 1;
        node_mode = value1;
      }
      if(status < 0)  
      {
        checksum_failed_counter++;
      }
    }
    else if(c == SERIAL_Set_DIO_PortA_ID)
    {
      
      message_type = c;
      char value1,value2,value3,value4,value5,value6,value7,value8;
      status = decode_Set_DIO_PortASerial(message_buffer,message_length,message_checksum,&value1,&value2,&value3,&value4,&value5,&value6,&value7,&value8);
      if(status == 1)
      {
        printf("Set Pin 0 to: %d\n",value1);
        Receive_Set_DIO_PortA_Counter++;
        checksum_passed_counter++;
        if((DIO_PortA.Pin_Mode[0] == PINMODE_DIGITAL_OUTPUT) || (DIO_PortA.Pin_Mode[0] == PINMODE_PWM_OUTPUT)) {DIO_PortA.Pin_Value[0] = value1; }
        if((DIO_PortA.Pin_Mode[1] == PINMODE_DIGITAL_OUTPUT) || (DIO_PortA.Pin_Mode[1] == PINMODE_PWM_OUTPUT)) {DIO_PortA.Pin_Value[1] = value2; }
        if((DIO_PortA.Pin_Mode[2] == PINMODE_DIGITAL_OUTPUT) || (DIO_PortA.Pin_Mode[2] == PINMODE_PWM_OUTPUT)) {DIO_PortA.Pin_Value[2] = value3; }
        if((DIO_PortA.Pin_Mode[3] == PINMODE_DIGITAL_OUTPUT) || (DIO_PortA.Pin_Mode[3] == PINMODE_PWM_OUTPUT)) {DIO_PortA.Pin_Value[3] = value4; }
        if((DIO_PortA.Pin_Mode[4] == PINMODE_DIGITAL_OUTPUT) || (DIO_PortA.Pin_Mode[4] == PINMODE_PWM_OUTPUT)) {DIO_PortA.Pin_Value[4] = value5; }
        if((DIO_PortA.Pin_Mode[5] == PINMODE_DIGITAL_OUTPUT) || (DIO_PortA.Pin_Mode[5] == PINMODE_PWM_OUTPUT)) {DIO_PortA.Pin_Value[5] = value6; }
        if((DIO_PortA.Pin_Mode[6] == PINMODE_DIGITAL_OUTPUT) || (DIO_PortA.Pin_Mode[6] == PINMODE_PWM_OUTPUT)) {DIO_PortA.Pin_Value[6] = value7; }
        if((DIO_PortA.Pin_Mode[7] == PINMODE_DIGITAL_OUTPUT) || (DIO_PortA.Pin_Mode[7] == PINMODE_PWM_OUTPUT)) {DIO_PortA.Pin_Value[7] = value8; }
        new_message = 1;
      }
      if(status < 0)  
      {
        checksum_failed_counter++;
      }
    }
     
  }
  
}         
void Initialize_Ports()
{
  //printf("\n\n\n\n\nInitializing\n\n\n");
  for(int i = 0; i < 8; i++)
  {
    DIO_PortA.Pin_Mode[i] = PINMODE_UNDEFINED;
    DIO_PortA.Pin_Number[i] = i;
    DIO_PortA.Pin_Value[i] = 0;
    DIO_PortB.Pin_Mode[i] = PINMODE_UNDEFINED;
    DIO_PortB.Pin_Number[i] = i+8;
    DIO_PortB.Pin_Value[i] = 0;
  }    
}                       
char Setup_Ports()
{
  //First do PortA
  for(int i = 0; i < 8; i++)
  {
    if(DIO_PortA.Pin_Mode[i] == PINMODE_DIGITAL_OUTPUT)
    {
    }
    if(DIO_PortB.Pin_Mode[i] == PINMODE_DIGITAL_OUTPUT)
    {
    }      
  }    
  return 1;  
}  
char Update_Ports()
{
  //First read ANA PortA and PortB.
  for(int i = 0; i < 4; i++)
  {
    //ANA_PortA.Pin_Value[i] = readADC(i,ADC_DATA,ADC_CLK,ADC_CS);
    //ANA_PortB.Pin_Value[i] = readADC(i+4,ADC_DATA,ADC_CLK,ADC_CS);
  }    
  //The do DIO PortA and PortB
  DIO_PortA = update_dio_port_config(DIO_PortA);
  DIO_PortB = update_dio_port_config(DIO_PortB);  
  return 1;    
}  
 
Port_Config update_dio_port_config(Port_Config PC)
{
  for(int i = 0; i < 8; i++)
  {
    if(PC.Pin_Mode[i] == PINMODE_DIGITAL_OUTPUT)
    {
      if(PC.Pin_Value[i] > 0) { high(PC.Pin_Number[i]); }
      else { low(PC.Pin_Number[i]) ;}
    } 
    else if(PC.Pin_Mode[i] == PINMODE_PWM_OUTPUT)
    {
      servo_angle(PC.Pin_Number[i],PC.Pin_Value[i]*1800/256);
    }           
    else if(PC.Pin_Mode[i] == PINMODE_DIGITAL_INPUT)
    {
      PC.Pin_Value[i] = input(PC.Pin_Number[i]);
    }      
  }    
  return PC;
}  
void display_firmware()
{
  for(int i = 0; i < 1800; i++)
  {
    servo_angle(ERROR_LED,i);
    waitcnt(50*(CLKFREQ/1000000)+CNT);
  } 
  pause(100);
  for(int i = 1800; i > 0; i--)
  {
    servo_angle(ERROR_LED,i);
    waitcnt(50*(CLKFREQ/1000000)+CNT);
  } 
  pause(100);
  low(ERROR_LED);
  pause(3000);
  for(int i = 0; i < FIRMWARE_MAJOR_VERSION; i++)
  {
    high(ERROR_LED);
    pause(200);
    low(ERROR_LED);
    pause(200);
  }
  pause(2000);
  for(int i = 0; i < FIRMWARE_MINOR_VERSION; i++)
  {
    high(ERROR_LED);
    pause(200);
    low(ERROR_LED);
    pause(200);
  }
  pause(2000);
  for(int i = 0; i < FIRMWARE_BUILD_NUMBER; i++)
  {
    high(ERROR_LED);
    pause(200);
    low(ERROR_LED);
    pause(200);;
  } 
  pause(3000);    
} 
 
 
 
 
 
