#include "simpletools.h"
#include "fdserial.h"
#include "serialmessage.h"
void run_read_master_cog();
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

int main()
{

  fd_device = fdserial_open(26, 27, 0, 115200);
  read_master_cog = cog_run(run_read_master_cog,128);
  while(1)
  {
    double dropped_ratio;
    if((checksum_failed_counter == 0) && (checksum_passed_counter == 0)) {dropped_ratio = 0.0; }
    else
    {
       dropped_ratio= 100.0*(double)((double)checksum_failed_counter/((double)checksum_passed_counter+(double)checksum_failed_counter));
    }       
    printf("Dropped Ratio: %f\n",dropped_ratio);
    if(new_message == 1)
    {
      printf("checksum_failed_counter: %d\n",checksum_failed_counter);
      new_message = 0;
      printf("Got Command: %0x v1: %d v2: %d v3: %d v4: %d v5: %d v6: %d v7: %d v8: %d\n",message_type,v1,v2,v3,v4,v5,v6,v7,v8); 
           
    }   
    cogmain_counter++;
    if(cogmain_counter > 20) { cogmain_counter = 0; }
    int out_buffer[12];
    int length[1];
    int status = encode_TestMessageCounterSerial(out_buffer,length,testmessagecounter,testmessagecounter,testmessagecounter,testmessagecounter,testmessagecounter,testmessagecounter,testmessagecounter,testmessagecounter);
    for(int i = 0; i < length[0]; i++)
    {
       fdserial_txChar(fd_device,out_buffer[i]);
    }    
 
     testmessagecounter++;
    pause(10);
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
    if(status < 0)
    {
      checksum_failed_counter++;
    } 
  }
}                               

 
 
 
 
 
 
 
 
 
 
