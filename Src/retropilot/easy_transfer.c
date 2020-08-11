#include "retropilot/easy_transfer.h"
#include <string.h>
#include <stdlib.h>

uint8_t * et_address;  //et_address of struct
uint8_t et_size;       //et_size of struct
uint8_t * et_rx_buffer; //et_address for temporary storage and parsing buffer
uint8_t et_rx_array_inx;  //index for RX parsing buffer
uint8_t et_rx_len;		//RX packet length according to the packet
uint8_t et_calc_cs;	   //calculated Chacksum

uart_ring *_q;

//Captures et_address and et_size of struct
void et_begin(uint8_t * ptr, uint8_t length, uart_ring *q){
	et_address = ptr;
	et_size = length;
	_q = q;

	//dynamic creation of rx parsing buffer in RAM
	et_rx_buffer = (uint8_t*) malloc(et_size+1);
}

//Sends out struct in binary, with header, length info and checksum
void et_send_data(){
  uint8_t CS = et_size;
  serial_putc(_q, 0x06);
  serial_putc(_q, 0x85);
  serial_putc(_q, et_size);
  for(int i = 0; i<et_size; i++){
    CS^=*(et_address+i);
    serial_putc(_q, *(et_address+i));
  }
  serial_putc(_q, CS);
}


bool et_receive_data() {
  //start off by looking for the header bytes. If they were already found in a previous call, skip it.
  if(et_rx_len == 0){
    //this et_size check may be redundant due to the et_size check below, but for now I'll leave it the way it is.
    if(serial_get_avail(_q) >= 3){
      //this will block until a 0x06 is found or buffer et_size becomes less then 3.
      char elem;

      while(true){
        if (!serial_getc(_q, &elem)) {return false;}
        if (elem == 0x06) break;
        //This will trash any preamble junk in the serial buffer
        //but we need to make sure there is enough in the buffer to process while we trash the rest
        //if the buffer becomes too empty, we will escape and try again on the next call
        if(serial_get_avail(_q) < 3) {
          return false;
        }
      }
      
      if (serial_getc(_q, &elem) && elem == 0x85){
        if (!serial_getc(_q, &elem)) {return false;}
        et_rx_len = elem;
        //make sure the binary structs on both Arduinos are the same et_size.
        if(et_rx_len != et_size){
          et_rx_len = 0;
          return false;
        }
      }
    }
  }

  //we get here if we already found the header bytes, the struct et_size matched what we know, and now we are byte aligned.
  if(et_rx_len != 0){
    char elem;
    while(serial_get_avail(_q)>0 && et_rx_array_inx <= et_rx_len){
      if (!serial_getc(_q, &elem)) return false;
      et_rx_buffer[et_rx_array_inx++]=elem;
    }

    if(et_rx_len == (et_rx_array_inx-1)){
      //seem to have got whole message
      //last uint8_t is CS
      et_calc_cs = et_rx_len;
      for (int i = 0; i<et_rx_len; i++){
        et_calc_cs^=et_rx_buffer[i];
      }

      if(et_calc_cs == et_rx_buffer[et_rx_array_inx-1]){//CS good
        memcpy(et_address,et_rx_buffer,et_size);
        et_rx_len = 0;
        et_rx_array_inx = 0;
        return true;
      }
      else{
        //failed checksum, need to clear this out anyway
        et_rx_len = 0;
        et_rx_array_inx = 0;
        return false;
      }
    }
  }
  return false;
}
