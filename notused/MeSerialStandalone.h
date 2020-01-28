/**
 * \par Copyright (C), 2012-2016, MakeBlock
 * \class MeSerialStandalone
 * \brief   Driver for serial.
 * @file    MeSerialStandalone.h
 * @author  MakeBlock
 * @version V1.0.1
 * @date    2015/01/20
 * @brief   Header for for MeSerialStandalone.cpp module
 *
 * \par Copyright
 * This software is Copyright (C), 2012-2016, MakeBlock. Use is subject to license \n
 * conditions. The main licensing options available are GPL V2 or Commercial: \n
 *
 * \par Open Source Licensing GPL V2
 * This is the appropriate option if you want to share the source code of your \n
 * application with everyone you distribute it to, and you also want to give them \n
 * the right to share who uses it. If you wish to use this software under Open \n
 * Source Licensing, you must contribute all your source code to the open source \n
 * community in accordance with the GPL Version 2 when your application is \n
 * distributed. See http://www.gnu.org/copyleft/gpl.html
 *
 * \par Description
 * This file is a drive for serial, It support hardware and software serial
 *
 * \par Method List:
 *
 *    1. void MeSerialStandalone::setHardware(bool mode)
 *    2. void MeSerialStandalone::begin(long baudrate)
 *    3. void MeSerialStandalone::end(void)
 *    4. size_t MeSerialStandalone::write(uint8_t byte)
 *    5. int16_t MeSerialStandalone::read(void)
 *    6. int16_t MeSerialStandalone::available(void)
 *    7. bool MeSerialStandalone::listen(void)
 *    8. bool MeSerialStandalone::isListening(void)
 *    9. int16_t MeSerialStandalone::poll(void)
 *
 * \par History:
 * <pre>
 * `<Author>`         `<Time>`        `<Version>`        `<Descr>`
 * Mark Yan         2015/09/08     1.0.0            Rebuild the old lib.
 * Mark Yan         2016/01/20     1.0.1            support arduino pin-setting.
 * </pre>
 */
#ifndef MeSerialStandalone_H
#define MeSerialStandalone_H

#include <stdint.h>
#include <stdbool.h>
#include <Arduino.h>
//#include "MeConfig.h"

//#ifdef ME_PORT_DEFINED
//#include "MePort.h"
//#endif // ME_PORT_DEFINED

/**
 * Class: MeSerialStandalone
 * \par Description
 * Declaration of Class MeSerialStandalone.
 */
#ifndef ME_PORT_DEFINED
class MeSerialStandalone
#else // !ME_PORT_DEFINED
class MeSerialStandalone : public MePort, public SoftwareSerial
#endif // !ME_PORT_DEFINED
{
public:
/**
 * Alternate Constructor which can call your own function to map the serial to arduino port,
 * no pins are used or initialized here. hardware serial will be used by default.
 * \param[in]
 *   None
 */
  MeSerialStandalone(void);
	
/**
 * Alternate Constructor which can call your own function to map the serial to arduino port,
 * If the hardware serial was selected, we will used the hardware serial.
 * \param[in]
 *   port - RJ25 port from PORT_1 to M2
 */
  MeSerialStandalone(uint8_t port);

/**
 * Alternate Constructor which can call your own function to map the serial to arduino port,
 * If the hardware serial was selected, we will used the hardware serial.
 * \param[in]
 *   receivePin - the rx pin of serial(arduino port)
 * \param[in]
 *   transmitPin - the tx pin of serial(arduino port)
 * \param[in]
 *   inverse_logic - Whether the Serial level need inv.
 */
  MeSerialStandalone(uint8_t receivePin, uint8_t transmitPin, bool inverse_logic = false);

/**
 * \par Function
 *   setHardware
 * \par Description
 *   if need change the hardware and software serial, this function can be used.
 * \param[in]
 *   mode - if need use hardware serial this value should set to true, or set it false.
 * \par Output
 *   None
 * \return
 *   None
 * \par Others
 *   None
 */
  void setHardware(bool mode);

/**
 * \par Function
 *   begin
 * \par Description
 *   Sets the speed (baud rate) for the serial communication. Supported baud 
 *   rates are 300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 31250, 
 *   38400, 57600, and 115200.
 * \param[in]
 *   baudrate - he baud rate (long)
 * \par Output
 *   None
 * \return
 *   None
 * \par Others
 *   None
 */
  void begin(long baudrate);

/**
 * \par Function
 *   write
 * \par Description
 *   Writes binary data to the serial port. This data is sent as a byte or series of bytes; 
 * \param[in]
 *   byte - a value to send as a single byte
 * \par Output
 *   None
 * \return
 *   it will return the number of bytes written, though reading that number is optional
 * \par Others
 *   None
 */
  size_t write(uint8_t byte);

/**
 * \par Function
 *   read
 * \par Description
 *   Return a character that was received on the RX pin of the software serial port. 
 *   Note that only one SoftwareSerial instance can receive incoming data at a time 
 *  (select which one with the listen() function).
 * \par Output
 *   None
 * \return
 *   The character read, or -1 if none is available
 * \par Others
 *   None
 */
  int read();

/**
 * \par Function
 *   available
 * \par Description
 *   Get the number of bytes (characters) available for reading from a software
 *   serial port. This is data that's already arrived and stored in the serial 
 *   receive buffer.
 * \par Output
 *   None
 * \return
 *   The number of bytes available to read
 * \par Others
 *   None
 */
  int available();

/**
 * \par Function
 *   poll
 * \par Description
 *   If we used the serial as software serial port, and set the _polling mask true.
 *   we beed use this function to read the serial data.
 * \par Output
 *   None
 * \return
 *   The character read, or -1 if none is available
 * \par Others
 *   None
 */
  int16_t poll(void);

/**
 * \par Function
 *   end
 * \par Description
 *   Stop listening and release the object
 * \par Output
 *   None
 * \return
 *   None
 * \par Others
 *   None
 */
  void end(void);

/**
 * \par Function
 *   listen
 * \par Description
 *   Enables the selected software serial port to listen, used for software serial.
 *   Only one software serial port can listen at a time; data that arrives for other 
 *   ports will be discarded. Any data already received is discarded during the call 
 *   to listen() (unless the given instance is already listening).
 * \par Output
 *   None
 * \return
 *   This function sets the current object as the "listening"
 *   one and returns true if it replaces another
 * \par Others
 *   None
 */
  bool listen(void);

/**
 * \par Function
 *   isListening
 * \par Description
 *   Tests to see if requested software serial port is actively listening.
 * \par Output
 *   None
 * \return
 *   Returns true if we were actually listening.
 * \par Others
 *   None
 */
  bool isListening(void);
	
/**
 * \par Function
 *   sendString
 * \par Description
 *   Send a string as a series of bytes, used for printf().
 * \param[in]
 *   str - A string to send as a series of bytes
 * \par Output
 *   None
 * \return
 *   None
 * \par Others
 *   None
 */
  void sendString(char *str);

/**
 * \par Function
 *   printf
 * \par Description
 *   Printf format string (of which "printf" stands for "print formatted") 
 *   refers to a control parameter used by a class of functions in the 
 *   string-processing libraries of various programming languages.
 * \param[in]
 *   fmt - A string that specifies the format of the output. The formatting 
 *   string determines what additional arguments you need to provide.
 * \par Output
 *   None
 * \return
 *   None
 * \par Others
 *   None
 */
  void printf(char *fmt,...);

  boolean dataLineAvailable(void);
  String readDataLine(void);
  String concatenateWith(String s1,String s2);
  char letterOf(int i,String s);
  int stringLength(String s);
  boolean equalString(String s1,String s2);
  float getValue(String key);

protected:
  bool _hard;
  bool _polling;
  bool _scratch;
  int16_t _bitPeriod;
  int16_t _byte;
  long _lastTime;
  char buffer[64];
  String lastLine;
  int bufferIndex;

private:
  volatile uint8_t _RxPin;
  volatile uint8_t _TxPin; 
};
#endif

