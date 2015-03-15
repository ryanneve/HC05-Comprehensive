#include <HC05c.h>

static const char DEFAULT_PASSWORD = "1234";
static const char BT_RESET[] = "RESET";
static const char BT_FSAD[] = "FSAD";
static const char BT_INIT[] = "INIT";
static const char CRLF[] = "\r\n";

HC05c::HC05c() {
   _baud_rates[0] = 38400; 
   _baud_rates[1] = 38400; 
   _baud_rates[2] = 115200;
   _baud_rates[3] = 115200;
   _baud_rates[4] = 9600;  
   _baud_rates[5] = 19200; 
   _baud_rates[6] = 57600; 
   detected_addressN=0;
   _forced_state=ST_NOFORCE;
}

/* --- Setup Bluetooth HC-05 device to be ready for inquiery
 * Set device name as BT_CAM, passwd 0000, inquiery search_iter 10s to 1 device
 */
boolean HC05c::setupConnection(char * devName){
	return setupConnection(devName, DEFAULT_PASSWORD);
}

boolean HC05c::setupConnection(char * devName, char * passwd) {
	char _buffer[BUFSZ];
	print_debug("Entering setupConnection() ");
	bootup = true;              // device is booting
	reqPairing = ( getHC05ADCN() == 0 ) ;  // if no device is already paired, request pairing.
	initSuccess = false;
	if ( _getConnection() ) {    
		if ( _getState() != 0 )  {
			_sendAtCmd(BT_RESET, true);          // If not in state initialized : reset
			delay(5000);
		}     
		if ( _getState() == 0 )  {
			_sendAtCmd(BT_INIT,true);           // Init SPP
			delay(2000);
			strncpy(_buffer,"NAME=",6);
			strncat(_buffer,devName,10);
			_sendAtCmd(_buffer,true);    // BT displayed name
			strncpy(_buffer,"PSWD=",6);
			strncat(_buffer,passwd,10);
			_sendAtCmd(_buffer,true);    // BT password
			_sendAtCmd("UART=38400,0,0",true); // serial over BT rate + Arduino rate (next restart)
			_sendAtCmd("IAC=9e8b33",true);     // use a Password for pairing
			_sendAtCmd("CMODE=1",true);        // connect to any address
			initSuccess = true;
		}     
	} else {
		print_debug("Leaving setupConnection() with error");
		return false;
	}
	print_debug("Leaving setupConnection() ");
	return true;
}


/* -------------------------------------------------------------
 * Establsh connection to a device. return only when the connection
 * has been done with true
 * --------------------------------------------------------------
 */
bool HC05c::connect() {
	char buf[BUFSZ];
	int bufd;
	char recvChar,lrecvChar;
	int state;
	uint16_t search_iter=0;
	if ( ! initSuccess ) return false;
	while (true) {
		state=_getState();
		print_debug2("HC05 State : ",state);
		switch ( state ) {
			case ST_INITIALIZED:
				// On startup, try to connect to existing device, then go for pairing if failed
				if ( reqPairing ) {
					reqPairing = false;
					// Start a pairing process
					_forceState(ST_SEARCH_FOR_PAIR);
					search_iter=0;
				}
				else {
					if ( getHC05ADCN() > 0 ) _forceState(ST_PAIRED);
					// mode sleep à travailler
					delay(5000); 
				}
				break;   
			// Not a real existing case, just to simplify algorithm
			case ST_SEARCH_FOR_PAIR:    
				if ( search_iter == 0 ) {
					// for the first minute we can start to INQUIRE if someone wants to pair with us as a salve
					_sendAtCmd("ROLE=0",true);       // slave mode
					_sendAtCmd("INQ",true);          // Start INQUIERING => change state to pairable  
					print_debug("HC05 waiting for pairing as a slave");
					bootup=false;			      // to not re-enter ...
				} 
				if ( search_iter < MAX_SLAVE_TIME ) {
					print_debug2("HC05 is inquiring as slave - ",search_iter);
					// Start inquiring ... this will change the status for PAIRABLE but we will proceed here
					// At this point if OK is read it means that we are connected with the device ... Inquiry success and finished with +DISC
					// already set : blueToothSerial.setTimeout(200);
					bufd = blueToothSerial.readBytes(buf,BUFSZ);
					if (bufd >= 2) && buf[0] == 'O' && buf[1] == 'K' ) _forceState(ST_PAIRED);
					delay(1000);
					search_iter++;      
				}
				else {
					print_debug("HC05 pairing as master");
					// At the end of the inquiring delay, start to inquirer in master mode ... 
					_forceState(ST_NOFORCE);               // Stop Slave inquiring ... reinit
					_sendAtCmd(BT_RESET, true);          // Reset
					delay(5000);
					_sendAtCmd(BT_INIT,true);            // Init SPP
					delay(2000);
					// Change mode to connect as a master
					_sendAtCmd("ROLE=1",true);       // act as master
					_sendAtCmd("CLASS=0",true);      // search for everything (use 200 for a smartphone)
					_startInq(MAX_MASTER_TIME); 
					// Did we found something ?
					if (  detected_addressN > 0 ) {
						// First try to connected to already known devices if we have
						for ( uint8_t k = 0 ; k < detected_addressN ; k++ ) {
							strncpy(buf,BT_FSAD,BUFSZ);
							if (_sendAtCmd(strncat(buf,detected_address[k],BUFSZ - 6),true) < 0 ) {
								// this address is already known ... linking
								strncpy(buf,"LINK=",BUFSZ);
								if (_sendAtCmd(strncat(buf,detected_address[k],BUFSZ - 6),false) < 0 ) break;     //link sucess
							}
						}
						// Then try to pair to listed devices
						for ( uint8_t k = 0 ; k < detected_addressN ; k++ ) {
							strncpy(buf,BT_FSAD,BUFSZ);
							if ( _sendAtCmd(strncat(buf,detected_address[k],BUFSZ - 6),true) == COD_FAIL ) {
							strncpy(buf,"PAIR=",BUFSZ);
							strnncat(buf,detected_address[k],BUFSZ - 6);
							strncat(buf,",20",BUFSZ - 14 - 6);
							if (_sendAtCmd(buf,false) < 0 ) {
									// pairing  sucess
									strncpy(buf,"LINK=",BUFSZ);
									if (_sendAtCmd(strncat(buf,detected_address[k],BUFSZ - 6),false) < 0 ) {
									_forceState(ST_CONNECTED);
									break;     //link sucess
									}
								}
							}
						}
						}
						_forceState(ST_NOFORCE);
				}
				break;
			case ST_PAIRABLE:
				break;
			case ST_INQUIERING:
				print_debug("HC05 is inquiering - invalid state");
				// reset
				_forceState(ST_NOFORCE);               // Stop Slave inquiering ... reinit
				_sendAtCmd(BT_RESET, true);          // Reset
				delay(5000);
				_sendAtCmd(BT_INIT,true);            // Init SPP
				delay(2000);
				break;
			case ST_PAIRED:
				print_debug("HC05 is paired to a device");
				if ( getHC05Mrad() ) {
					// Connect to the last device if possible
					strncpy(buf,"LINK=",BUFSZ);
					if (_sendAtCmd(strncat(buf,detected_address[0],BUFSZ - 6),false) < 0 ) {
					 _forceState(ST_CONNECTED);
					}
					else {
						// If connection not succeed, back to standard process ...
						_sendAtCmd(" ",true); // it seems that after a AT+LINK the next AT command is not concidered
						// When connection fail, if at bootup, we start a new pairing process
						if ( bootup ) {
							reqPairing = true;                  
							_forceState(ST_NOFORCE); 
						}
					}
				}
				bootup = false; // bootup period is finished after first pairing try
				break;
			case ST_DISCONNECTED:
				print_debug("Disconnection detected");
				_sendAtCmd(BT_RESET, true);          // If not in state initialized : reset
				delay(5000);
				_sendAtCmd(BT_INIT,true);            // Init SPP
				delay(2000);
				break;
			case ST_CONNECTED:
				print_debug("Device Connected");
				return true;
			default:
				delay(1000);
				break;
		}
	}
}

/* ---------------------------------------------------------------
 * Receive string from bluetooth - not blocking operation
 * return number of char received
 * return -1 when the connection is broken
 * ---------------------------------------------------------------
 */
int HC05c::receive(char * buf, int maxsz) {
	int16_t bufd;
	if ( _getState() != ST_CONNECTED ) return -1;
	// We are waiting for +DISC command now on blueToothSerial to quit the connection mode
	bufd = blueToothSerial.readBytes(buf,BUFSZ);
	if (bufd > 0 ) {
		if (bufd > 5) {
			if ( buf[0] == '+' && buf[1] == 'D' && buf[2] == 'I' && buf[3] == 'S' && buf[4] == 'C' ) {
				// detect disconnection
				_forceState(ST_NOFORCE);
				return -1;
			} 
		}
		buf[bufd]=0;
	}
	return bufd;
}

/* ---------------------------------------------------------------
 * Send string over bluetooth - blocking operation
 * return false if disconnected
 * ---------------------------------------------------------------
 */
boolean HC05c::send(char * buf) {
	if ( _getState() != ST_CONNECTED ) return false;
	blueToothSerial.print(buf);
	return true;
}



/* ======================================================================
 * HC-05 Subroutine
 * ======================================================================
 */

/* --- Search for HC05 device and baudrate 
 * (peace of code from https://github.com/jdunmire/HC05 project)
 * send AT at different baudrate and expect OK
 * it seems that testing multiple search_iter the same rate ensure to have a better detection
 * return true is found, false otherwise.
 */
boolean HC05c::_getConnection() {
	uint16_t numRates = sizeof(_baud_rates)/sizeof(unsigned long);
	uint16_t recvd = 0;
	char _buffer[128];
	uint8_t _bufsize = sizeof(_buffer)/sizeof(char);
	print_debug("Entering _getConnection()");
	for(uint16_t rn = 0; rn < numRates; rn++) {
		print_debug2(" * Trying new rate : ",_baud_rates[rn]);
		blueToothSerial.begin(_baud_rates[rn]);
		blueToothSerial.setTimeout(200);
		blueToothSerial.write("AT");
		blueToothSerial.write(CRLF);
		delay(200);
		recvd = blueToothSerial.readBytes(_buffer,_bufsize);
		if (recvd > 0) {
			print_debug("Leaving _getConnection() - Found card ");
			return true;
		} 
	}
	print_debug("Leaving _getConnection() - Not Found ");
	return false;
}

/* --- Force a specific state as the HC05 does not commute as I could expect
 * Once the state is forced, the device is not anymore Inquiery for State until
 * this forced state is back to ST_ERROR
 */

void HC05c::_forceState(int state) {
	_forced_state=state;
}

/* --- Get STATE return state code, according to:
 * -1 : ERROR            -2 : NOFORCE    -3 : SEARCH_FOR_PAIR
 *  0 : INITIALIZED      1 : READY        2 : PAIRABLE    3 : PAIRED
 *  4 : INQUIRING        5 : CONNECTING   6 : CONNECTED   7 : DISCONNECTED
 *  8 : NUKNOW
 */
int HC05c::_getState() {
	uint16_t recvd = 0;
	char _buffer[128];
	uint8_t _bufsize = sizeof(_buffer)/sizeof(char);
	int16_t ret = ST_ERROR;
	print_debug("Entering _getState()");
	if (_forced_state == ST_NOFORCE ) {
		blueToothSerial.print("AT+STATE?\r\n"); 
		delay(200);
		recvd = blueToothSerial.readBytes(_buffer,_bufsize);
		if (recvd > 2 ) {
			uint8_t i = 0;
			while ( i < _bufsize-1 ) {
				if ( _buffer[i] == '\n' ) { _buffer[i] = 0 ; i = _bufsize; }
				else i++;
			}
			print_debug2(" * State Received :",_buffer);
			if ( recvd > 12 ) {
				switch (_buffer[7]) {
					case 'R' : ret= 1; break; 
					case 'D' : ret= 7; break;
					case 'I' : ret= (_buffer[9]=='I')?0:4; break;
					case 'P' : ret= (_buffer[11]=='A')?2:3; break;
					case 'C' : if ( recvd > 16 ) ret = ( _buffer[14] == 'I')?5:6; else ret= ST_ERROR; break;    
					default : ret = ST_ERROR; break;
				}
			}
		} else ret= ST_ERROR;
	} else	ret = _forced_state;
	// Other cases : consider as valid command
	print_debug2("Leaving _getState() with ret code : ",ret);
	return ret;   
} 


/* --- Start Enquiring
 * start inquiring, results are read from the serial line
 * timeout is set return number of results
 */
int HC05c::_startInq(int timeout ) {
	uint8_t recvd = 0;
	char _buffer[512];
	uint8_t _bufsize = sizeof(_buffer)/sizeof(char);
	print_debug("Entering _startInq()");
	sprintf(_buffer,"INQM=1,4,%d",timeout); // mode rssi, 4 device max, timeout*1.28s max
	_sendAtCmd(_buffer,true);
	blueToothSerial.print("AT+INQ");           // start INQ    
	blueToothSerial.print(CRLF);           // start INQ                     
	delay(1300*timeout);
	recvd = blueToothSerial.readBytes(_buffer,_bufsize);
	_sendAtCmd("INQC",true);                  // Retour etat INITIALIZED
	blueToothSerial.flush();
	// Search for addresses in format : +INQ:aaaa:aa:aaaa,ttttt,ppppp 
	detected_addressN = 0;
	uint8_t i = 0, v;
	while ( i < recvd ) {
		if ( recvd - i > 15 ) { // do not spend search_iter looking for address is less than 15 byte
			if ( _buffer[i] == '+'  && _buffer[i+1] == 'I' && _buffer[i+2] == 'N' && _buffer[i+3] == 'Q' && _buffer[i+4] == ':' ) {
				// we have detected a response, search for separator (,)
				v=i+5;
				while ( v < recvd && _buffer[v] != ',' ) v++;
				if ( v < recvd ) { 
					memcpy(detected_address[detected_addressN],&_buffer[i+5], v-(i+5));  // copy address bloc
					detected_address[detected_addressN][v-(i+5)+1]=0;                    // add end of line
					
					for (int k=0 ; detected_address[detected_addressN][k] != 0 ; k++ ) {	// Change ':' separator by ',' as it will be this syntaxe latter used
						if ( detected_address[detected_addressN][k] == ':' ) detected_address[detected_addressN][k] = ',';
					}
					int c=0;                                                             // we should count already existing occurence of this address
					for ( int j=0 ; j < detected_addressN ; j++ ) {
						if ( strcmp(detected_address[j],detected_address[detected_addressN]) == 0 ) c++;
					}
					if ( c == 0 ) {
						print_debug2("Address found : ",detected_address[detected_addressN]);
						getHC05RName(detected_address[detected_addressN]);
						detected_addressN++;
						if (detected_addressN == 4 ) return 4;                            // No place left on table, skip
					}
					i=v;                                                                 // goto next
				}
			}
		}
		i++;
	}
	print_debug2("Leaving _startInq() with ret code : ",detected_addressN);
	return detected_addressN;   
 } 

/* --- Get MRAD - Most Recent Used Address
 * return it in detected_Address[0], empty this table if not failed
 */
 boolean HC05c::getHC05Mrad() {
	uint8_t recvd = 0;
	char _buffer[128];
	uint8_t _bufsize = sizeof(_buffer)/sizeof(char);
	print_debug("Entering getHC05Mrad()");
	blueToothSerial.print("AT+MRAD?\r\n");
	delay(200);
	recvd = blueToothSerial.readBytes(_buffer,_bufsize);
	if (recvd > 6 ) {
		uint8_t i = 0;
		while ( i < _bufsize-1 ) {
			if ( _buffer[i] == '\n' ) { _buffer[i] = 0 ; i = _bufsize; }
			else i++;
		}
		print_debug2(" * Received :",_buffer);
		if ( _buffer[0]=='+' && _buffer[1] == 'M' ) {
			uint8_t v=6;
			while ( v < recvd && _buffer[v] != 0 ) v++;
			if ( v < recvd ) {
				memcpy(detected_address[0],&_buffer[6], v-6);	// copy address bloc
				detected_address[0][v-6+1]=0;					// add end of line
				for (uint8_t k=0 ; detected_address[0][k] != 0 ; k++ )   // Change ':' separator by ',' as it will be this syntaxe latter used
				if ( detected_address[0][k] == ':' ) detected_address[0][k] = ',';
				detected_addressN = 1;
				print_debug("Leaving getHC05Mrad() - true");
				return true;
			}        
		}
	}
	// Other cases : consider as valid command
	print_debug("Leaving getHC05Mrad() - false");
	return false;
}
    
 
/* --- Get ADCN value
 * ADCN is the number of paired devices stored in the memory
 * return the number from response format : +ADCN:X where X is the value on 1 or 2 digit
 * return -1 when error
 */
int HC05c::getHC05ADCN(){
	uint8_t recvd = 0;
	char _buffer[128];
	uint8_t _bufsize = sizeof(_buffer)/sizeof(char);
	int16_t ret = -1;
	print_debug("Entering getHC05ADCN()");
	blueToothSerial.print("AT+ADCN?\r\n"); 
	delay(200);
	recvd = blueToothSerial.readBytes(_buffer,_bufsize);
	if (recvd > 7 ) { // test valid for 2 digit or 1 digit + \n
		if ( _buffer[0] == '+' ) {
			ret = _buffer[6] - '0'; 
			if ( _buffer[7] >= '0' && _buffer[7] <= '9' ) ret = 10 * ret + _buffer[7] - '0';
		}
		print_debug2(" * ADCN :",ret);
	}
	else ret = -1;
	// Other cases : consider as valid command
	print_debug2("Leaving getHC05ADCN() with ret code : ",ret);
	return ret;   
}
 
/* --- Send AT Command and get Error code back
 * imediate : when true directly read result, do not wait them to come
 *            make sense when you are not waiting for a remote device answer
 *
 * Send AT command over blueToothSerial then return -1 id OK error code otherwise
 * According to Doc
 * 0 - AT CMD Error           1 - Default result      2 - PSKEY write error      3 - Too Long
 * 4 - No Dev Name            5 - Bt NAP too long     6 - BT UAP too long        7 - BT LAP too long
 * 8 - No PIO num mask        9 - No PIO Num          10 - No BT device         11 - Too lenght of device
 * 12 - No Inquire ac        13 - Too long inq ac     14 - Invalid inq ac code  15 - Passkay Lenght is 0
 * 16 - Passkey len too long 17 - Invalide modul role 18 - Invalid baud rate    19 - Invalid strop bit
 * 20 - Invalid parity bit   21 - auth dev not pair   22 - SPP not init         23 - SPP has been init
 * 24 - Invalid inq mode     25 - Too long inq search_iter   26 - No BT addresse       27 - Invalid safe mode
 * 28 - Invalid encryptmode  30 - FAIL
 */
int HC05c::_sendAtCmd(char * atcmdstr, boolean imediate) {
	uint8_t recvd = 0;
	char _buffer[128];
	uint8_t _bufsize = sizeof(_buffer)/sizeof(char);
	int16_t ret = -1;
	print_debug("Entering _sendAtCmd()");
	print_debug2(" * Send : ",atcmdstr);
	blueToothSerial.print("AT+");
	blueToothSerial.print(atcmdstr);
	blueToothSerial.print(CRLF); 
	delay(200);
	while ( ( recvd = blueToothSerial.readBytes(_buffer,_bufsize) ) == 0 && (! imediate) ) delay(100);
#ifdef BT_DEBUG
	if (recvd > 0 ) {
		uint8_t i = 0;
		while ( i < _bufsize-1 ) {
			if ( _buffer[i] == '\n' ) { _buffer[i] = 0 ; i = _bufsize; }
			else i++;
		}
		print_debug2(" * Received :",_buffer);
	}
#endif
	if (recvd >= 2) {
		if ( _buffer[0] == 'O' && _buffer[1] == 'K' ) ret = -1;     
		else if ( _buffer[0] == 'E' ) {
			// Get Error number if _buffer[8] = ')' then 1 digit error code, else 2 digit
			if ( recvd >= 9 ) {
				ret = ( _buffer[8] != ')' )?16*(hex2dec(_buffer[7]))+(hex2dec(_buffer[8])):hex2dec(_buffer[7]);
				print_debug2(" * Error code : ",ret);
			}
		} else if ( _buffer[0] == 'F' ) {
			print_debug(" * Error code : FAIL");
			ret=COD_FAIL;
		}
	}
	// Other cases : consider as valid command
	print_debug("Leaving _sendAtCmd()");
	return ret;  
}


/* --- Get RName - get remote device name (mostly for debugging purpose in my case
 * Print name on debugging flow
 * Remote address is transmitted with ':' separator, it is converted to ',' in the function
 * AT+RNAME?34C0,59,F191D5
 */
void HC05c::getHC05RName(char * raddr) {
#ifdef BT_DEBUG
	uint8_t recvd = 0;
	char _buffer[128];
	uint8_t _bufsize = sizeof(_buffer)/sizeof(char);
	int16_t ret = -1;
	print_debug("Entering getHC05RName()");
	print_debug2(" * Request For : ",raddr);
	blueToothSerial.print("AT+RNAME?");
	blueToothSerial.print(raddr); // why?
	blueToothSerial.print(CRLF); 
	delay(5000);
	recvd = blueToothSerial.readBytes(_buffer,_bufsize);
	if (recvd > 0 ) {
		int i = 0;
		while ( i < _bufsize-1 ) {
			if ( _buffer[i] == '\n' ) { _buffer[i] = 0 ; i = _bufsize; }
			else i++;
		}
		print_debug2(" * Received :",_buffer);
	}
	// Other cases : consider as valid command
	print_debug("Leaving getHC05RName()");
#endif
}


