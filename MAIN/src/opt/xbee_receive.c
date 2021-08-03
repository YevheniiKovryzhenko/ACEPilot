// Updated code (Feb 2019)
//
// Using data structure for packets with:
// Two start bytes:  0x81, 0xA1
// [Not included: Message ID (one byte), Message payload size (one byte) since we only have one message type]
// Message data (xbee_packet_t length)
// Fletcher-16 checksum (two bytes) computed starting with Message payload size
//
// Note:  This MBin protocol is commonly used on embedded serial devices subject to errors

#include "xbee_receive.h"
#include "state_estimator.h"

xbee_packet_t xbeeMsg; // Defined as extern in xbee_packet_t.h
int xbee_portID;  // Defined as extern in xbee_packet_t.h

// Information local to this file
void XBEE_readRingBuffer();
#define XBEE_RING_BUFSIZE 256
#define XBEE_RING_INC(a)(a<(XBEE_RING_BUFSIZE-1))?(a+1):0  // Increment ring buffer index macro
int XBEE_ring_overflow=0, XBEE_rdIndex=0, XBEE_wrIndex=0;
unsigned char XBEE_ringbuffer[XBEE_RING_BUFSIZE];

int XBEE_init(const char *xbee_port, const int baudRate)
{
  //int baudRate = 230400;  
  //int baudRate = 230400;
  xbee_portID = serial_open(xbee_port, baudRate, 0);  // Nonblocking = 0, BLOCKING = 1
  //xbee_portID = serial_open("/dev/ttyS5",baudRate,0); // Nonblocking = 0, BLOCKING = 1 
  //xbee_portID = serial_open("/dev/ttyGS0", baudRate, 0); // Nonblocking = 0, BLOCKING = 1 
  if(xbee_portID == -1)    {
    printf("Failed to open Serial Port\n");
    return -1;
  }
  return 0;
}


// Read message received from XBee; use ring buffer to assure no data loss
int XBEE_getData()
{
  int k;
  unsigned char buffer;
  
  // Populate ring buffer
  for (k=0;k<XBEE_RING_BUFSIZE;k++) {
    if (XBEE_ring_overflow) { // Overflow condition on last read attempt
      if (XBEE_rdIndex == XBEE_wrIndex) { // Indices equal:  overflow still in effect
	return -1;  // Do not read data; reading data would cause overflow
	break;
      } else
	XBEE_ring_overflow = 0;  // Reset overflow flag
    }

    if (read(xbee_portID, &buffer, 1) > 0) { // Read 1 byte from serail port into buffer: returns 0 or -1 if no data
      XBEE_ringbuffer[XBEE_wrIndex] = buffer;
	  XBEE_wrIndex = XBEE_RING_INC(XBEE_wrIndex);
      if (XBEE_wrIndex == XBEE_rdIndex) XBEE_ring_overflow = 1;
    } else
      break;
  }

  // Read and process data in ring buffer; print data
  XBEE_readRingBuffer();
  return 0;
}



// XBEE_readRingBuffer()
#define XBEE_startByte1 0x81
#define XBEE_startByte2 0xA1

uint64_t last_time;

void XBEE_readRingBuffer()
{
  static unsigned char msgState = 0, msglength = 0;
  static unsigned char msgdata[OPTI_DATA_LENGTH];
  static uint8_t ck0, ck1;
  

  while(XBEE_ring_overflow || (XBEE_rdIndex != XBEE_wrIndex)) { //Don't get ahead of the receiving data 
    if (XBEE_ring_overflow) XBEE_ring_overflow = 0; // Reset buffer overflow flag
    // Case 0:  Current character is first message header byte
    if((XBEE_ringbuffer[XBEE_rdIndex] == XBEE_startByte1) && !msgState) {
		//printf("\n Received Start byte!\n"); //test if xbee is working and receiving start byte
      msgState = 1; 
      msglength = 0;
    }
     
    // Case 1:  Current character is second message header byte
    else if (msgState == 1) {
      if (XBEE_ringbuffer[XBEE_rdIndex] == XBEE_startByte2) msgState = 2; 
      else
      {
            printf("\n WARNING: Second start byte did not match! Discarding the message... \n");
            msgState = 0;  // Bad message
      }
      ck0 = 0;  // Initialize Fletcher checksum bytes
      ck1 = 0;
      msglength = 0;  // Initialize number of message payload data bytes read thusfar
    }
       
    // Case 2:  Read data bytes into msgdata[] array
    else if (msgState == 2) {
      msgdata[msglength++] = XBEE_ringbuffer[XBEE_rdIndex];
      ck0 += XBEE_ringbuffer[XBEE_rdIndex];
      ck1 += ck0;
      if (msglength == OPTI_DATA_LENGTH) msgState = 3; // Done reading data
      //printf("\n msglength = %d, Byte = %X, ck0 = %d and ck1 = %d", msglength, XBEE_ringbuffer[XBEE_rdIndex],ck0,ck1);
    }
    
    // Case 3:  Read & check the first checksum byte
    // Throw away data if full checksum doesn't match
    else if (msgState == 3) {

        if (ck0 != XBEE_ringbuffer[XBEE_rdIndex]) msgState = 0;
        else msgState = 4;
        //printf("\n State = 3, Testing Checksum! \n ch0 = %X and message is %X \n", ck0, XBEE_ringbuffer[XBEE_rdIndex]);
    }

    // Case 4:  Read & check the second checksum byte
    else if (msgState == 4) {
        msgState = 0;  // Done reading message data
       
        //printf("\n State = 4, 1 End Byte matched! Testing next one... \n ch1 = %X and message is %X \n", ck1, XBEE_ringbuffer[XBEE_rdIndex]);
        if (ck1 == XBEE_ringbuffer[XBEE_rdIndex]) { // Valid message -- copy and print --- must figure out the checksum later (JK)
            //printf("\n It's a MATCH! Accepting message... \n");
	        
            if (settings.delay_warnings_en)
            {
                double dt_s = (rc_nanos_since_boot() - last_time) /
                              (1e9);  // calculate time since last successful reading
                if (1.0 / dt_s < 20)
                    printf("\nWARNING, Low update frequency of Xbee %f (Hz)\n",
                        1.0 / dt_s);  // check the update frequency
            }
            
  	        
            memcpy(&xbeeMsg, msgdata, OPTI_DATA_LENGTH);
            state_estimate.mocap_timestamp_ns = rc_nanos_since_boot();
	        if (settings.delay_warnings_en) last_time = rc_nanos_since_boot();
	
		    //check for xbee connection (move it out later)
            if (settings.telem_warnings_en && xbeeMsg.trackingValid == 0)
            {
				printf("\nWARNING, MOCAP LOST VISUAL\n");
	        }
	        //######################
	
	        //XBEE_printData();
        }
        else
        {
            if (settings.telem_warnings_en) printf("\n WARNING: Last checksum byte did not match! Discarding the message... \n");
        }
    }
    XBEE_rdIndex = XBEE_RING_INC(XBEE_rdIndex);
  }  
  return;
}

///////////////////////////////////////
void XBEE_printData()
{
  // Print to terminal
  printf("\r");			
  // Time
  if      (xbeeMsg.time < 1000000)   printf("   %u |",xbeeMsg.time);
  else if (xbeeMsg.time < 10000000)  printf("  %u |",xbeeMsg.time);
  else if (xbeeMsg.time < 100000000) printf(" %u |",xbeeMsg.time);
  else                         printf("%u |",xbeeMsg.time);		
  
  // XYZ
  if( xbeeMsg.x < 0)
    printf("%7.6f |",xbeeMsg.x);
  else
    printf(" %7.6f |",xbeeMsg.x);
  if( xbeeMsg.y < 0)
    printf("%7.6f |",xbeeMsg.y);
  else
    printf(" %7.6f |",xbeeMsg.y);
  if( xbeeMsg.z < 0)
    printf("%7.6f |",xbeeMsg.z);
  else
    printf(" %7.6f |",xbeeMsg.z);
  
  // Quaternion
  if( xbeeMsg.qx < 0)
    printf("%7.6f |",xbeeMsg.qx);
  else
    printf(" %7.6f |",xbeeMsg.qx);
  if( xbeeMsg.qy < 0)
    printf("%7.6f |",xbeeMsg.qy);
  else
    printf(" %7.6f |",xbeeMsg.qy);
  if( xbeeMsg.qz < 0)
    printf("%7.6f |",xbeeMsg.qz);
  else
    printf(" %7.6f |",xbeeMsg.qz);
  if( xbeeMsg.qw < 0)
    printf("%7.6f |",xbeeMsg.qw);
  else
    printf(" %7.6f |",xbeeMsg.qw);
  
  // Tracking Valid
  printf("   %u   |",xbeeMsg.trackingValid);
    
  fflush(stdout);
}	





