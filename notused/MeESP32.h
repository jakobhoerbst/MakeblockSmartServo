/**
 * \par Copyright (C), 2012-2016, MakeBlock
 * \brief   Driver for Shield Board.
 * \file    MeShield.h
 * @author  MakeBlock
 * @version V1.0.0
 * @date    2015/09/01
 * @brief   Driver for Shield Board.
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
 * This file is Hardware adaptation layer between Shiled board
 * and all MakeBlock drives
 *
 * \par History:
 * <pre>
 * `<Author>`         `<Time>`        `<Version>`        `<Descr>`
 * Mark Yan         2015/09/01     1.0.0            Rebuild the old lib.
 * </pre>
 */
#ifndef MeESP32_H
#define MeESP32_H

//#include <Arduino.h>
//#include "MeConfig.h"

/* Supported Modules drive needs to be added here */

//#include "MeSerial.h"



/*********************  Shield Board GPIO Map *********************************/
MePort_Sig mePort[17] =
{
  { NC, NC }, { 11, 10 }, {  9, 12 }, { 13,  8 }, { NC,  3 },
  { 16, 17 }, { NC,  2 }, { A2, A3 }, { A0, A1 }, {  5,  4 },
  {  6,  7 }, { NC, NC }, { NC, NC }, { NC, NC }, { NC, NC },
  { NC, NC }, { NC, NC },
};


#endif // MeShield_H

