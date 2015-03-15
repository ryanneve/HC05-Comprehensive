#include <Arduino.h>
#ifndef _HC05C_H_
#define _HC05C_H_

#define BT_DEBUG

/* ======================================================================
 * Configuration
 * ======================================================================
 */

// -- Pairing timing
#ifndef MAX_SLAVE_TIME
#define MAX_SLAVE_TIME    60        // Wait as a slave for MAX_SLAVE_TIME seconds
#endif

#ifndef MAX_MASTER_TIME
#define MAX_MASTER_TIME   25        // Wait as a master for MAX_MASTER_TIME * 1.28s (max is 48)                                  
#endif

// -- Serial port used for hc05 communication
#ifndef blueToothSerial
#define blueToothSerial Serial1
#endif

 
/* ======================================================================
 * Helper and constants
 * ======================================================================
 */
#ifdef BT_DEBUG
  #define print_debug(x) Serial.print("Debug :");Serial.println(x);
  #define print_debug2(x,y) Serial.print("Debug :");Serial.print(x);Serial.println(y);
#else 
  #define print_debug(x) 
  #define print_debug2(x,y) 
#endif
#define hex2dec(x) ((x>'9')?10+x-'A':x-'0')

// -- HC05 States
#define ST_INITIALIZED 0
#define ST_PAIRED 3
#define ST_DISCONNECTED 7
#define ST_PAIRABLE 2
#define ST_INQUIERING 4
#define ST_ERROR -1
#define ST_CONNECTED 6
#define ST_NOFORCE -2
#define ST_SEARCH_FOR_PAIR  -3

// -- Internal
#define COD_FAIL  30
#define BUFSZ 50

 
class HC05c
{
	public:
		HC05c();
		//Configure the bluetooth device
		bool	setupConnection(char * devName);
		bool	setupConnection(char * devName, char * passwd);
		//Establsh connection to a device. return only when the connection
		//has been done with true
		bool	connect();
		// Receive string from bluetooth - not blocking operation
		// return number of char received
		// return -1 when the connection is broken
		int16_t	receive(char *,int);
		// Send string over bluetooth - blocking operation
		// return false if disconnected
		bool	send(char *);
   private:
		bool		_getConnection();
		void		_forceState(int16_t);
		int16_t		_getState();
		int16_t		_startInq(int16_t);
		bool		_getMrad();
		int16_t		_getADCN();
		int16_t		_sendAtCmd(char *, boolean);
		void		_getRName(char *);
		// list of devices detected
		char		detected_address[4][16];
		int16_t		detected_addressN;
		uint32_t	_baud_rates[6];
		int16_t 	_forced_state;	// Signed
		// state memory
		bool 	bootup;			// true a boot to validate / unvalidate pairing
		bool 	reqPairing;		// switch to true to execute a pairing or re-pairing search
		bool 	initSuccess;	// true if init finished in success
 };
 
#endif
