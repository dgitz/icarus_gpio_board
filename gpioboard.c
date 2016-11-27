#include "simpletools.h"
//#include "mcp3208.h"
#include "fdserial.h"
#include "serialmessage.h"
#include "Definitions.h"
#include "servo.h"

#define VERBOSE 1

#define FIRMWARE_MAJOR_VERSION 3
#define FIRMWARE_MINOR_VERSION 3
#define FIRMWARE_BUILD_NUMBER 5
#define SYSTEM ROVER
#define SUBSYSTEM ROBOT_CONTROLLER
#define COMPONENT GPIO_NODE
#define DIAGNOSTIC_TYPE SOFTWARE
typedef struct
{
  int Pin_Mode[8];
  int Pin_Number[8];
  int Pin_Value[8];
  int Pin_DefaultValue[8];
} Port_Config;
typedef struct
{
  int ID[8];
  int Mode[8];
  int Input_Port[8];
  int Input_PinMode[8];
  int Input_PinNumber[8];
  int Output_Port[8];
  int Output_PinMode[8];
  int Output_PinNumber[8];
  int Proportional_Gain[8];
  int Integral_Gain[8];
  int Derivative_Gain[8]; 
  int CurrentPoint[8];
  int SetPoint[8];
  int OutPoint[8];
  int CurError[8];
  int CumError[8];
} ControlGroup_Config;  

void run_veryfastrate_code(); //1000 Hz
void run_fastrate_code(); //100 Hz
void run_mediumrate_code(); //10 Hz
void run_slowrate_code(); //1 Hz
void run_receive_master_cog();
void run_transmit_master_cog();
void run_adc_cog();
int veryfastrate_counter = 0;
int fastrate_counter = 0;
int mediumrate_counter = 0;
int slowrate_counter = 0;
volatile long time_since_last_received_message = 2000;

#define ADC_CS 18//18
#define ADC_CLK 16//16
#define ADC_DATA 17//17
#define STATUS_LED 25
#define ERROR_LED 24
#define ARM_PIN 23
void Initialize_Ports();
char Setup_Ports();
char Update_Ports();
char Update_ControlGroups();
void display_firmware();
Port_Config update_dio_port_config(Port_Config PC);
int readADCAverage(int channel, int din, int dout, int clk, int cs, int samples);
int readADC(int channel, int din, int dout, int clk, int cs);
void pinPulseHL(int pin, int d, int d1);
void pinPulseLH(int pin, int d, int d1);
char board_mode = GPIO_MODE_UNDEFINED;
char prev_board_mode = GPIO_MODE_UNDEFINED;
char node_mode = GPIO_MODE_UNDEFINED;


int *receive_master_cog;
int *transmit_master_cog;
int *adc_cog;
fdserial *fd_device;
volatile int Send_DIO_PortA = 0;
volatile int Send_DIO_PortB = 0;
volatile int Send_ANA_PortA = 0;
volatile int Send_ANA_PortB = 0;
volatile int Send_BoardMode = 0;
volatile int Send_Firmware = 0;
volatile int Send_Diagnostics = 0;
int comm_with_master = 0;

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
volatile long Receive_Setup_ControlGroup_Counter = 0;
volatile long Receive_Tune_ControlGroup_Counter = 0;
volatile long Receive_Arm_Command_Counter = 0;
volatile long Receive_Arm_State_Counter = 0;
volatile long Receive_DefaultValue_DIO_PortA_Counter = 0;
volatile long Receive_DefaultValue_DIO_PortB_Counter = 0;
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
volatile int armed_state = ARMEDSTATUS_UNDEFINED;
volatile int arm_command = ARMEDCOMMAND_UNDEFINED;
volatile int armpin_value = 0;
volatile int enable_actuators = 0;
volatile int last_enable_actuators = 0;
Port_Config DIO_PortA;  
Port_Config DIO_PortB; 
Port_Config ANA_PortA; 
Port_Config ANA_PortB;

ControlGroup_Config ControlGroupA; 
void run_veryfastrate_code() //1000 Hz
{
  armpin_value = input(ARM_PIN);
  if((armpin_value == 1) && 
     (armed_state == ARMEDSTATUS_ARMED) &&
     (arm_command == ARMEDCOMMAND_ARM))
  {
    enable_actuators = 1;
  } 
  else
  {
    enable_actuators = 0;
  }          
  time_since_last_received_message++;
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
  //printf("Armed State: %d Arm Command: %d Arm Pin: %d enable: %d\n",armed_state,arm_command,armpin_value,enable_actuators);
  if(input(STATUS_LED) == 1){ low(STATUS_LED); }
  else { high(STATUS_LED); }
  if(comm_with_master == 0)
  {
    if(input(ERROR_LED) == 1){ low(ERROR_LED); }
    else { high(ERROR_LED); }
  } 
  /*for(int i = 0; i < 4; i ++)
  {
    printf("ANA Port A Pin: %d Value: %d\n",i,ANA_PortA.Pin_Value[i]);
    printf("ANA Port B Pin: %d Value: %d\n",i,ANA_PortB.Pin_Value[i]);
  }  
  */  
  
     
}  
void run_slowrate_code() //1 Hz
{
  if(VERBOSE == 1)
  {           
    printf("Major Version: %d Minor Version: %d Build Number: %d\n",
      FIRMWARE_MAJOR_VERSION,
      FIRMWARE_MINOR_VERSION,
      FIRMWARE_BUILD_NUMBER);
    printf("Failed Checksum: %d Passed Checksum: %d\n",checksum_failed_counter,checksum_passed_counter);
    printf("Received Config DIO PortA (0xAB16) %d times.\n",Receive_Configure_DIO_PortA_Counter);
    printf("Received Node Mode (0xAB17) %d times.\n",Receive_BoardMode_Counter);
    
    printf("Send Board Mode (0xAB17) %d times.\n",Send_BoardMode_Counter);
    printf("Sent ANA Port A (0xAB19) %d times.\n",Send_ANA_PortA_Counter);
    printf("Sent ANA Port B (0xAB20) %d times.\n",Send_ANA_PortB_Counter);
    printf("Received Config DIO PortB (0xAB21) %d times.\n",Receive_Configure_DIO_PortB_Counter);
    printf("Received Set DIO PortA (0xAB23) %d times.\n",Receive_Set_DIO_PortA_Counter);
    printf("Received Set DIO PortB (0xAB22) %d times.\n",Receive_Set_DIO_PortB_Counter);
    printf("Received Setup Control Group (0xAB28) %d times.\n",Receive_Setup_ControlGroup_Counter);
    printf("Received Tune Control Group (0xAB29) %d times.\n",Receive_Tune_ControlGroup_Counter);
    printf("Received Arm Command (0xAB27) %d times.\n",Receive_Arm_Command_Counter);
    printf("Received Arm Status (0xAB30) %d times.\n",Receive_Arm_State_Counter);
    printf("Received Default Value DIO PortA (0x32) %d times.\n",Receive_DefaultValue_DIO_PortA_Counter);
    printf("Received Default Value DIO PortB (0x33) %d times.\n",Receive_DefaultValue_DIO_PortB_Counter);
         
    
  } 
  int out_buffer[12];
  int length[1];
  Diagnostic_Type = COMMUNICATIONS;
  if(time_since_last_received_message > 2000)
  { 
    comm_with_master = 0;
  } 
  if(time_since_last_received_message < 100)
  {
    comm_with_master = 1;
  }       
  if(message_dropped_ratio < 3.0)
  {
    Diagnostic_Level = INFO;
    Diagnostic_Message = NOERROR;
  }
  else if((message_dropped_ratio >= 3.0) && (message_dropped_ratio < 6.0))
  {
    if(comm_with_master == 1)
    {
      high(ERROR_LED);
    }      
    Diagnostic_Level = WARN;
    Diagnostic_Message = DROPPING_PACKETS;
  }     
  else if(message_dropped_ratio >= 6.0)
  {
    if(comm_with_master == 1)
    {
      high(ERROR_LED);
    } 
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
  printf("Major Version: %d Minor Version: %d Build Number: %d\n",
    FIRMWARE_MAJOR_VERSION,
    FIRMWARE_MINOR_VERSION,
    FIRMWARE_BUILD_NUMBER);
 // 
  board_mode = GPIO_MODE_BOOT;
  initialized = 0;
  display_firmware();
  high(STATUS_LED);
  high(ERROR_LED);
  Initialize_Ports();
  fd_device = fdserial_open(26, 27, 0, 115200);
  Send_BoardMode = 1;
  Send_Firmware = 1; 
 // simpleterm_close();
  receive_master_cog = cog_run(run_receive_master_cog,512);
 // printf("Started UART receive cog id: %d\n",cog_num(receive_master_cog));
  transmit_master_cog = cog_run(run_transmit_master_cog,256);
  //printf("Started UART transmit cog id: %d\n",cog_num(transmit_master_cog));
  adc_cog = cog_run(run_adc_cog,128);
 //printf("Started ADC cog id: %d\n",cog_num(adc_cog));
  
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
      printf("Executing: %d\n",slowrate_counter);
      slowrate_counter = 0;
      run_slowrate_code();
    }                    
   
    waitcnt(1000*(CLKFREQ/1000000)+CNT);
  }    
}
void run_adc_cog()
{
  //simpleterm_open();
  //printf("Starting.\n"):
  while(1)
  {
    //printf("ADC Cog Running.\n");
    for(int i = 0; i < 4; i++)  
    {
      ANA_PortA.Pin_Value[i] = readADCAverage(i, ADC_DATA, ADC_DATA, ADC_CLK, ADC_CS,10);
      //ANA_PortA.Pin_Value[i] = readADC(i, ADC_DATA, ADC_DATA, ADC_CLK, ADC_CS);
     // printf("ANA Port A Pin: %d Value: %d\n",i,ANA_PortA.Pin_Value[i]);
      ANA_PortB.Pin_Value[i] = readADCAverage(i+4, ADC_DATA, ADC_DATA, ADC_CLK, ADC_CS,10);
     // printf("ANA Port B Pin: %d Value: %d\n",i,ANA_PortB.Pin_Value[i]);
    }
    waitcnt(1000*(CLKFREQ/1000000)+CNT);
  }    
}  
void run_transmit_master_cog()
{
  //simpleterm_open();
  while(1)  
  {
   // printf("Transmit cog running.\n");
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
      //printf("Seing Board Mode.\n");
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
  //simpleterm_open();
  while(1)
  {
   // printf("Receive cog running.\n");
    char c;
    while(c = fdserial_rxChar(fd_device) != 0xAB);
    c = fdserial_rxChar(fd_device);
    message_length = fdserial_rxChar(fd_device);
    for(int i = 0; i < message_length; i++)
    {
      message_buffer[i] = fdserial_rxChar(fd_device);
    }   
    message_checksum = fdserial_rxChar(fd_device);
     /*
    for(int i = 0; i < message_length;i++)
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
      if(status == 1)
      {
        time_since_last_received_message = 0;
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
    else if(c == SERIAL_Set_DIO_PortA_DefaultValue_ID)
    {
      message_type = c;
      char value1,value2,value3,value4,value5,value6,value7,value8;
      status = decode_Set_DIO_PortA_DefaultValueSerial(message_buffer,message_length,message_checksum,&value1,&value2,&value3,&value4,&value5,&value6,&value7,&value8);  
      if(status == 1)
      {
        time_since_last_received_message = 0;
        Receive_DefaultValue_DIO_PortA_Counter++;
        checksum_passed_counter++;
        DIO_PortA.Pin_DefaultValue[0] = value1;
        DIO_PortA.Pin_DefaultValue[1] = value2;
        DIO_PortA.Pin_DefaultValue[2] = value3;
        DIO_PortA.Pin_DefaultValue[3] = value4;
        DIO_PortA.Pin_DefaultValue[4] = value5;
        DIO_PortA.Pin_DefaultValue[5] = value6;
        DIO_PortA.Pin_DefaultValue[6] = value7;
        DIO_PortA.Pin_DefaultValue[7] = value8;
        new_message = 1;
      }
      if(status < 0)  
      {
        checksum_failed_counter++;
      }
    }
    else if(c == SERIAL_Set_DIO_PortB_DefaultValue_ID)
    {
      message_type = c;
      char value1,value2,value3,value4,value5,value6,value7,value8;
      status = decode_Set_DIO_PortB_DefaultValueSerial(message_buffer,message_length,message_checksum,&value1,&value2,&value3,&value4,&value5,&value6,&value7,&value8);  
      if(status == 1)
      {
        time_since_last_received_message = 0;
        Receive_DefaultValue_DIO_PortB_Counter++;
        checksum_passed_counter++;
        DIO_PortB.Pin_DefaultValue[0] = value1;
        DIO_PortB.Pin_DefaultValue[1] = value2;
        DIO_PortB.Pin_DefaultValue[2] = value3;
        DIO_PortB.Pin_DefaultValue[3] = value4;
        DIO_PortB.Pin_DefaultValue[4] = value5;
        DIO_PortB.Pin_DefaultValue[5] = value6;
        DIO_PortB.Pin_DefaultValue[6] = value7;
        DIO_PortB.Pin_DefaultValue[7] = value8;
        new_message = 1;
      }
      if(status < 0)  
      {
        checksum_failed_counter++;
      }
    }
    else if(c == SERIAL_Arm_Command_ID)
    {
      message_type = c;
      char value1;
      status = decode_Arm_CommandSerial(message_buffer,message_length,message_checksum,&value1);
      if(status == 1)
      {
        arm_command = value1;
        Receive_Arm_Command_Counter++;
        time_since_last_received_message = 0;
        checksum_passed_counter++;
        new_message = 1;
      }
      if(status < 0)  
      {
        checksum_failed_counter++;
      }
    }
    else if(c == SERIAL_Arm_Status_ID)
    {
      message_type = c;
      char value1;
      status = decode_Arm_StatusSerial(message_buffer,message_length,message_checksum,&value1);
      if(status == 1)
      {
        armed_state = value1;
        Receive_Arm_State_Counter++;
        time_since_last_received_message = 0;
        checksum_passed_counter++;
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
        time_since_last_received_message = 0;
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
        time_since_last_received_message = 0;
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
        time_since_last_received_message = 0;
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
        time_since_last_received_message = 0;
        Receive_Set_DIO_PortA_Counter++;
        checksum_passed_counter++;
        if(enable_actuators == 0)
        {
          value1 = DIO_PortA.Pin_DefaultValue[0];
          value2 = DIO_PortA.Pin_DefaultValue[1];
          value3 = DIO_PortA.Pin_DefaultValue[2];
          value4 = DIO_PortA.Pin_DefaultValue[3];
          value5 = DIO_PortA.Pin_DefaultValue[4];
          value6 = DIO_PortA.Pin_DefaultValue[5];
          value7 = DIO_PortA.Pin_DefaultValue[6];
          value8 = DIO_PortA.Pin_DefaultValue[7];
        }
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
    else if(c == SERIAL_Set_DIO_PortB_ID)
    {
      message_type = c;
      char value1,value2,value3,value4,value5,value6,value7,value8;
      status = decode_Set_DIO_PortBSerial(message_buffer,message_length,message_checksum,&value1,&value2,&value3,&value4,&value5,&value6,&value7,&value8);
      if(status == 1)
      {
        time_since_last_received_message = 0;
        Receive_Set_DIO_PortB_Counter++;
        checksum_passed_counter++;
        if(enable_actuators == 0)
        {
          value1 = DIO_PortB.Pin_DefaultValue[0];
          value2 = DIO_PortB.Pin_DefaultValue[1];
          value3 = DIO_PortB.Pin_DefaultValue[2];
          value4 = DIO_PortB.Pin_DefaultValue[3];
          value5 = DIO_PortB.Pin_DefaultValue[4];
          value6 = DIO_PortB.Pin_DefaultValue[5];
          value7 = DIO_PortB.Pin_DefaultValue[6];
          value8 = DIO_PortB.Pin_DefaultValue[7];
        }
        if((DIO_PortB.Pin_Mode[0] == PINMODE_DIGITAL_OUTPUT) || (DIO_PortB.Pin_Mode[0] == PINMODE_PWM_OUTPUT)) {DIO_PortB.Pin_Value[0] = value1; }
        if((DIO_PortB.Pin_Mode[1] == PINMODE_DIGITAL_OUTPUT) || (DIO_PortB.Pin_Mode[1] == PINMODE_PWM_OUTPUT)) {DIO_PortB.Pin_Value[1] = value2; }
        if((DIO_PortB.Pin_Mode[2] == PINMODE_DIGITAL_OUTPUT) || (DIO_PortB.Pin_Mode[2] == PINMODE_PWM_OUTPUT)) {DIO_PortB.Pin_Value[2] = value3; }
        if((DIO_PortB.Pin_Mode[3] == PINMODE_DIGITAL_OUTPUT) || (DIO_PortB.Pin_Mode[3] == PINMODE_PWM_OUTPUT)) {DIO_PortB.Pin_Value[3] = value4; }
        if((DIO_PortB.Pin_Mode[4] == PINMODE_DIGITAL_OUTPUT) || (DIO_PortB.Pin_Mode[4] == PINMODE_PWM_OUTPUT)) {DIO_PortB.Pin_Value[4] = value5; }
        if((DIO_PortB.Pin_Mode[5] == PINMODE_DIGITAL_OUTPUT) || (DIO_PortB.Pin_Mode[5] == PINMODE_PWM_OUTPUT)) {DIO_PortB.Pin_Value[5] = value6; }
        if((DIO_PortB.Pin_Mode[6] == PINMODE_DIGITAL_OUTPUT) || (DIO_PortB.Pin_Mode[6] == PINMODE_PWM_OUTPUT)) {DIO_PortB.Pin_Value[6] = value7; }
        if((DIO_PortB.Pin_Mode[7] == PINMODE_DIGITAL_OUTPUT) || (DIO_PortB.Pin_Mode[7] == PINMODE_PWM_OUTPUT)) {DIO_PortB.Pin_Value[7] = value8; }
        new_message = 1;
      }
      if(status < 0)  
      {
        checksum_failed_counter++;
      }
    }
    
    else if(c == SERIAL_Setup_ControlGroup_ID)
    {
      message_type = c;
      char id,mode,input_port,input_pinmode,input_pinnumber,output_port,output_pinmode,output_pinnumber;
      status = decode_Setup_ControlGroupSerial(message_buffer,message_length,message_checksum,
      &id,&mode,
      &input_port,&input_pinmode,&input_pinnumber,
      &output_port,&output_pinmode,&output_pinnumber);
      if(status == 1)
      {
        time_since_last_received_message = 0;
        Receive_Setup_ControlGroup_Counter++;
        checksum_passed_counter++;
        int found = 0;
        int next_index = 0;
        for(int i = 0; i < 8; i++)
        {
          if(ControlGroupA.ID[i] == id)
          {
            found = 1;
            ControlGroupA.Mode[i] = mode;
            ControlGroupA.Input_Port[i] = input_port;
            ControlGroupA.Input_PinMode[i] = input_pinmode;
            ControlGroupA.Input_PinNumber[i] = input_pinnumber;
            ControlGroupA.Output_Port[i] = output_port;
            ControlGroupA.Output_PinMode[i] = output_pinmode;
            ControlGroupA.Output_PinNumber[i] = output_pinnumber;
          } 
          else if((ControlGroupA.ID[i] == -1) && (next_index == 0))
          {
            next_index = i;
          }                    
        }          
        if(found == 0)
        {
          ControlGroupA.ID[next_index] = id;
          ControlGroupA.Mode[next_index] = mode;
          ControlGroupA.Input_Port[next_index] = input_port;
          ControlGroupA.Input_PinMode[next_index] = input_pinmode;
          ControlGroupA.Input_PinNumber[next_index] = input_pinnumber;
          ControlGroupA.Output_Port[next_index] = output_port;
          ControlGroupA.Output_PinMode[next_index] = output_pinmode;
          ControlGroupA.Output_PinNumber[next_index] = output_pinnumber;
          
        }          
        new_message = 1;
      }
      if(status < 0)  
      {
        checksum_failed_counter++;
      }
    }
    else if(c == SERIAL_Tune_ControlGroup_ID)
    {
      message_type = c;
      char id,mode;
      int p_gain,i_gain,d_gain;
      status = decode_Tune_ControlGroupSerial(message_buffer,message_length,message_checksum,
      &id,&mode,&p_gain,&i_gain,&d_gain);

      if(status == 1)
      {
        time_since_last_received_message = 0;
        Receive_Tune_ControlGroup_Counter++;
        checksum_passed_counter++;
        if(id > 0)
        {
          for(int i = 0; i < 8; i++)
          {
            if(ControlGroupA.ID[i] == id)
            {
              ControlGroupA.Proportional_Gain[i] = p_gain;
              ControlGroupA.Integral_Gain[i] = i_gain;
              ControlGroupA.Derivative_Gain[i] = d_gain;
            }              
          }            
        }          
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
  for(int i = 0; i < 4; i++)
  {
    ANA_PortA.Pin_Value[i] = 0;
    ANA_PortB.Pin_Value[i] = 0;
  }    
  for(int i = 0; i < 8; i++)
  {
    ControlGroupA.ID[i] = -1;
    ControlGroupA.Mode[i] = CONTROLGROUP_MODE_UNDEFINED;
    ControlGroupA.Input_Port[i] = GPIO_PORT_UNDEFINED;
    ControlGroupA.Input_PinMode[i] = PINMODE_UNDEFINED;
    ControlGroupA.Input_PinNumber[i] = -1;
    ControlGroupA.Output_Port[i] = GPIO_PORT_UNDEFINED;
    ControlGroupA.Output_PinMode[i] = PINMODE_UNDEFINED;
    ControlGroupA.Output_PinNumber[i] = -1;
    ControlGroupA.Proportional_Gain[i] = 0;
    ControlGroupA.Integral_Gain[i] = 0;
    ControlGroupA.Derivative_Gain[i] = 0;
    ControlGroupA.CurrentPoint[i] = 0;
    ControlGroupA.SetPoint[i] = 0;
    ControlGroupA.CurError[i] = 0;
    ControlGroupA.CumError[i] = 0;
    ControlGroupA.OutPoint[i] = 0;
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
char Update_ControlGroups()
{
  
  for(int i = 0; i < 8; i++)
  {
    if(ControlGroupA.ID[i] > 0)
    {
      int last_error = ControlGroupA.CurError[i]; 
      ControlGroupA.CurError[i] = ControlGroupA.SetPoint[i]-ControlGroupA.CurrentPoint[i];
      long long cumerror = ControlGroupA.CumError[i] + ControlGroupA.CurError[i];
      if(abs(cumerror) < 32767)  
      {
        ControlGroupA.CumError[i] = cumerror;
      }   
      double p_term = (double)ControlGroupA.Proportional_Gain[i]*(double)ControlGroupA.CurError[i];
      double i_term = (double)ControlGroupA.Integral_Gain[i]*(double)ControlGroupA.CumError[i];
      double d_term = (double)ControlGroupA.Derivative_Gain[i]*(double)(ControlGroupA.CurError[i]-last_error);
      ControlGroupA.OutPoint[i] = (int)p_term + (int)i_term + (int)d_term;
      
    }      
  } 
  
  return 0;   
}  
char Update_Ports()
{
  //First read ANA PortA and PortB.
  for(int i = 0; i < 4; i++)
  {
    //ANA_PortA.Pin_Value[i] = readADCAverage(i, ADC_DATA, ADC_DATA, ADC_CLK, ADC_CS,1);
    
    //ANA_PortB.Pin_Value[i] = readADCAverage(i+4, ADC_DATA, ADC_DATA, ADC_CLK, ADC_CS,1);
   // printf("ANA Port B Pin: %d Value: %d\n",i,ANA_PortB.Pin_Value[i]);
  }    
  //The do DIO PortA and PortB
  DIO_PortA = update_dio_port_config(DIO_PortA);
  //DIO_PortB = update_dio_port_config(DIO_PortB);  
  return 1;    
}  
 
Port_Config update_dio_port_config(Port_Config PC)
{
  for(int i = 0; i < 8; i++)
  {
    if(PC.Pin_Mode[i] == PINMODE_DIGITAL_OUTPUT)
    {
      if(enable_actuators == 0)
      {
        PC.Pin_Value[i] = PC.Pin_DefaultValue[i];
      }        
      if(PC.Pin_Value[i] > 0) 
      { 
        high(PC.Pin_Number[i]); 
      }
      else 
      { 
        low(PC.Pin_Number[i]);
      }
    } 
    else if(PC.Pin_Mode[i] == PINMODE_DIGITAL_OUTPUT_NON_ACTUATOR)
    {
      if(PC.Pin_Value[i] > 0) { high(PC.Pin_Number[i]); }
      else { low(PC.Pin_Number[i]) ;}
    }      
    else if(PC.Pin_Mode[i] == PINMODE_PWM_OUTPUT)
    {
      if(enable_actuators == 0)
      {
        PC.Pin_Value[i] = PC.Pin_DefaultValue[i];
      }
      //printf("i: %d v: %d def: %d\n",PC.Pin_Number[i],PC.Pin_Value[i]*1800/256,PC.Pin_DefaultValue[i]*1800/256);
      servo_angle(PC.Pin_Number[i],PC.Pin_Value[i]*1800/256);        
    } 
    else if(PC.Pin_Mode[i] == PINMODE_PWM_OUTPUT_NON_ACTUATOR)
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
  low(ERROR_LED);
  pause(3000);    
} 

int readADCAverage(int channel, int din, int dout, int clk, int cs, int samples)
{
  int i, k = 0;
  
  for (i=0;i<samples;i++)
  {
    k = k + readADC(channel, din, dout, clk, cs);
  }    
  return k/samples;
}
int readADC(int channel, int din, int dout, int clk, int cs)
{
  int i;
  int AdcResult;
  int setup;
  
  // In case pin was already been low, we put it high
  // so we can initiate communication after setting up pins
  high(cs);
  low(din);
  low(clk);
  low(cs); // Active chip select by setting pin low
  
  // Sending configuration to device
  setup = channel | 0b11000;
  for(i=0; i < 5;i++) 
  {
    pinPulseHL(clk, 10, 0);
    if ((setup & 0b10000) == 0b10000)
    {
      high(din);
    }      
    else
    {
      low(din); // is MSB != 0
    }          
      setup <<= 1; // shift left
      usleep(10);
    } 
    pinPulseHL(clk, 10, 10); //Empty clock, for sampling
  
    pinPulseHL(clk, 10, 10); //Device returns low, NULL bit, we ignore it...
  
    // read ADC re  sult 12 bit
    AdcResult=0;
    for(i=0;i<12;i++) {
    // We are sending pulse, clock signal, to ADC, because on falling edge it will return data...
    pinPulseHL(clk, 10, 0);
    // Shifting bit to left, to make room for current one...
    AdcResult<<=1;
    AdcResult=AdcResult | (input(dout) & 0x01);
    usleep(10);
  }
  high(cs);
  return(AdcResult);
}
void pinPulseHL(int pin, int d, int d1)
{
  high(pin);
  if (d > 10)
    usleep(d);
  low(pin);
  if (d1 > 10)
    usleep(d1);
}

/**
* @brief sets pin, int pin, state to low, delays specified time, int d,
* and then puts pin high, and delays specified time, int d1
*/
void pinPulseLH(int pin, int d, int d1)
{
  high(pin);
  if (d > 10)
    usleep(d);
  low(pin);
  if (d1 > 10)
    usleep(d1);
}
 
 
