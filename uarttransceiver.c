#include "simpletools.h"
#include "fdserial.h"
void run_read_master_cog();
int *read_master_cog;
volatile fdserial *fd_device;
cogmain_counter = 0;
volatile int command = -1;
volatile int new_message = 0;
volatile int message_buffer[64];

volatile int checksum_failed_counter = 0;
int main()
{
  fd_device = fdserial_open(26, 27, 0, 115200);
  read_master_cog = cog_run(run_read_master_cog,128);
  while(1)
  {
    printf("checksum_failed_counter: %d\n",checksum_failed_counter);
    if(new_message == 1)
    {
      new_message = 0;
      printf("Got Command: %0x\n");      
    }   
    cogmain_counter++;
    if(cogmain_counter > 20) { cogmain_counter = 0; }
    int out_buffer[12];
    out_buffer[0] = 0xAB;
    out_buffer[1] = 0x15;  //command
    out_buffer[2] = 8; //length
    out_buffer[3] = 9;
    out_buffer[4] = 10;
     out_buffer[5] = 11;
    out_buffer[6] = 12; 
     out_buffer[7] = 13;
    out_buffer[8] = 14; 
     out_buffer[9] = 15;
    out_buffer[10] = 16; 
    int checksum = 0;
    for(int i = 3; i < 11; i++)
    {
      checksum ^= out_buffer[i];
    }
    out_buffer[11] = checksum;
    for(int i = 0; i < 12; i++)
    {
       fdserial_txChar(fd_device,out_buffer[i]);
    }      
    //dprint(fd_device,"%s",out_buffer);      
    printf("Sent: ");
    for(int i = 0; i < 12; i++)
    {
      printf("b: %d",out_buffer[i]);
    }   
    printf("\n");   
    //printf("M: %d\n",cogmain_counter);
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
    computed_checksum = 0;
    for(int i= 0; i < message_length; i++)
    {
      computed_checksum ^= message_buffer[i];
    } 
    if(computed_checksum == message_checksum)
    {         
      if(c == 0x14)
      {
        new_message = 1;
        command = c;;
      }  
      else if(c == 0x15)
      {
        new_message = 1;
        command = c;
      }    
    } 
    else
    {
      checksum_failed_counter++;
    }           
  }    
}    
 