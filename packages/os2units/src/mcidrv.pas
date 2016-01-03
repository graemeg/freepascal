{
    Copyright (c) 1991, 1992 by International Business Machines Corporation
    Copyright (c) 2002 by Andry Svirgunov (cool2@ngs.ru)
    Copyright (c) 2002-2003 by Yuri Prokushev (prokushev@freemail.ru)

    MCI drivers interface

    This program is free software; you can redistribute it and/or modify it
    under the terms of the GNU Library General Public License (LGPL) as
    published by the Free Software Foundation; either version 2 of the
    License, or (at your option) any later version. This program is
    distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or
    FITNESS FOR A PARTICULAR PURPOSE.

    See the GNU Library General Public License for more details. You should
    have received a copy of the GNU Library General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.

 **********************************************************************}

{
@abstract(MCI drivers interface)
@author(Andry Svirgunov (cool2@ngs.ru))
@author(Yuri Prokushev (prokushev@freemail.ru))
@created(01 Oct 2002)
@lastmod(19 Jan 2003)
MCI drivers interface
Warning: This code is alfa. Future versions of this unit will propably
not be compatible.
}
Unit mcidrv;

Interface

Type
  GID=Integer;

///**************************************************************************/
//             MCI Driver  message identifiers                              */
///**************************************************************************/
const
  MCIDRV_SAVE                   = 900;
  MCIDRV_RESTORE                = 901;
  MCIDRV_SYNC                   = 902;
  MCIDRV_CHANGERESOURCE         = 903;
  MCIDRV_CONNECT                = 904;
  MCIDRV_ASSOCIATE              = 905;
  MCIDRV_DISCONNECT             = 906;
  MCIDRV_TEST_CONNECTION        = 907;
  MCIDRV_START                  = 908;
  MCIDRV_SEEK                   = 909;
  MCIDRV_STOP                   = 910;
  MCIDRV_MODIFY_CONNECTION      = 911;
  MCIDRV_INIT_CONNECTION        = 912;
  MCIDRV_DEINIT_CONNECTION      = 913;
  MCIDRV_ENABLE_EVENT           = 915;
  MCIDRV_GETTIME                = 917;

///*******************************************************************/
// The following messages are used by the MCIDRV_CONNECT message     */
///*******************************************************************/

  MCIDRV_CONNECT_TARGET         = $00010000;
  MCIDRV_CONNECT_SOURCE         = $00020000;
  MCIDRV_SET_CONNNECTOR_TYPE    = $00040000;
  MCIDRV_CHANGE_SEM             = $00080000;

///*******************************************************************/
// The following messages are used by MCIDRV_MODIFY_CONNECTION       */
///*******************************************************************/


  MCIDRV_MODIFY_10              =$00100000;  // back level MCI driver.
  MCIDRV_MIX_STANDALONE         =$00200000;


///*******************************************************************/
// The following messages are used by MCIDRV_START                   */
///*******************************************************************/

  MCIDRV_CUE_PLAYBACK           = $00010000;
  MCIDRV_CUE_RECORD             = $00020000;
  MCIDRV_START_PLAYBACK         = $00040000;
  MCIDRV_START_RECORD           = $00080000;
  MCIDRV_START_PREROLL          = $00100000;

///*******************************************************************/
// The following messages are used by MCIDRV_ASSOCIATE               */
///*******************************************************************/

  MCIDRV_OUTPUT                 = $00010000;
  MCIDRV_INPUT                  = $00020000;

///*******************************************************************/
// The following messages are used by MCIDRV_STOP                    */
///*******************************************************************/

  MCIDRV_PAUSE_STREAM_NETWORK   = $00010000;
  MCIDRV_FLUSH_STREAM_NETWORK   = $00020000;
  MCIDRV_DISCARD_STREAM_NETWORK = $00040000;

///*******************************************************************/
// The following messages are used by MCIDRV_DEINIT_CONNECTION       */
///*******************************************************************/

  MCIDRV_MODIFY_HANDLER         =$00010000;



///*******************************************************************/
// The following range of message ID's are reserved for internal use */
//  by MCI drivers                                                   */
///*******************************************************************/
  MCI_INTERNAL_MESSAGES_START   = 1000;
  MCI_INTERNAL_MESSAGES_END     = 1999;

///*******************************************************************/
// Flags used by the string parser for command tables                */
// NOTE: All flags must have an "L" suffix in order to be parsed as  */
// DWORDs by the resource compiler                                   */
///*******************************************************************/
  MCI_COMMAND_HEAD              = $00000001;
  MCI_END_COMMAND               = $00000002;
  MCI_END_COMMAND_LIST          = $00000003;
  MCI_RETURN                    = $00000004;
  MCI_STRING                    = $00000005;
  MCI_FLAG                      = $00000006;
  MCI_INTEGER                   = $00000007;
  MCI_CONSTANT                  = $00000008;
  MCI_CONSTANT_INTEGER          = $00000009;
  MCI_CONSTANT_STRING           = $0000000A;
  MCI_END_CONSTANT              = $0000000B;
  MCI_DEFAULT_STRING            = $0000000C;
  MCI_DEFAULT_INTEGER           = $0000000D;
  MCI_RETURN_TYPE               = $0000000E;
  MCI_RETURN_TYPE_STRING        = $0000000F;
  MCI_END_RETURN_TYPE           = $00000010;
  MCI_CONSTANT_2                = $00000011;
  MCI_END_CONSTANT_2            = $00000012;
  MCI_CONSTANT_PAIR             = $00000013;
  MCI_END_CONSTANT_PAIR         = $00000014;
  MCI_CONSTANT_INTEGER_PAIR     = $00000015;
  MCI_CONSTANT_STRING_PAIR      = $00000016;
  MCI_RECTL                     = $00000017;
  MCI_CONSTANT_4                = $00000018;
  MCI_END_CONSTANT_4            = $00000019;
  MCI_FOURCC                    = $0000001A;
  MCI_OR                        = $0000001B;
  MCI_END_OR                    = $0000001C;
  MCI_STRING_LIST               = $0000001D;

///*******************************************************************/
// Return types supported by mciSendString                           */
//   Values 0x0000 -> 0xFF00  are reserved by MMPM2                  */
//   Values 0xFF01 -> 0xFFFF  are user definable                     */
///*******************************************************************/
  MCI_INTEGER_RETURNED          = $1000;
  MCI_COLONIZED2_RETURN         = $2000;
  MCI_COLONIZED3_RETURN         = $3000;
  MCI_COLONIZED4_RETURN         = $4000;
  MCI_TRUE_FALSE_RETURN         = $5000;
  MCI_ON_OFF_RETURN             = $6000;
  MCI_DEVICENAME_RETURN         = $7000;
  MCI_TIME_FORMAT_RETURN        = $8000;
  MCI_SPEED_FORMAT_RETURN       = $9000;
  MCI_MODE_RETURN               = $A000;
  MCI_MEDIA_TYPE_RETURN         = $B000;
  MCI_TRACK_TYPE_RETURN         = $C000;
  MCI_CONNECTOR_TYPE_RETURN     = $D000;
  MCI_CDXA_CHANNEL_DESTINATION_RETURN = $E000;
  MCI_PREROLL_TYPE_RETURN       = $F000;
  MCI_FORMAT_TAG_RETURN         = $F100;
  MCI_SEQ_SYNCHRONIZATION_RETURN = $F200;
  MCI_VIDEO_QUALITY_RETURN      = $F300;
  MCI_AUDIO_QUALITY_RETURN      = $F400;
  MCI_IMAGE_QUALITY_RETURN      = $F500;
  MCI_VIDEO_COMPRESSION_RETURN  = $F600;
  MCI_AUDIO_COMPRESSION_RETURN  = $F700;
  MCI_IMAGE_COMPRESSION_RETURN  = $F800;
  MCI_RECTL_RETURN              = $F900;
  MCI_FOURCC_RETURN             = $FA00;
  MCI_IMAGE_PELFORMAT_RETURN    = $FB00;
  MCI_DIRECTION_RETURN          = $FC00;
  MCI_SIGNED_INTEGER_RETURN     = $FD00;

  MCI_USER_RETURN               = $FF01;
///*******************************************************************/
// End of msg text used by mciGetErrorString                         */
///*******************************************************************/
  MCIERR_BASE                       = 5000;
  MCIERR_MSG_TABLE_END          = MCIERR_BASE + 7000;

///*******************************************************************/
//                                                                   */
//  MCI DEVICE shareability categories                               */
//                                                                   */
///*******************************************************************/
  FIXEDSINGLECONTEXT            = $0001;
  DYNAMICSINGLECONTEXT          = $0002;
  LIMITEDMULTIPLECONTEXT        = $0003;
  UNLIMITEDMULTIPLECONTEXT      = $0004;

///*******************************************************************/
// MCI driver flag for close during exit list processing             */
///*******************************************************************/
  MCI_CLOSE_EXIT                = $10000000;

///*******************************************************************/
// MCI driver specific error table resource number base              */
///*******************************************************************/
  MMERROR_TABLE_BASE            = 500;

///*******************************************************************/
//                                                                   */
//  MCIDRV_CHANGERESOURCE  message flags                             */
//                                                                   */
///*******************************************************************/

Type
  mciDrv_ChangeResource_Parms = record
    pInstance                   : Pointer;     // pointer to device instance
    usResourceUnits             : Integer;      // required resource units
    usResourceClass             : Integer;      // resource class
    usResourcePriority          : Integer;      // resource priority
  end;
  pmciDrv_ChangeResource_Parms = ^mciDrv_ChangeResource_Parms;

///*******************************************************************/
//                                                                   */
//  MCIDRV_ENABLE_EVENT message flags                                */
//                                                                   */
///*******************************************************************/

Type
  HSTREAM=LongInt;
  hID=LongInt;
  hEvent = LongInt;                // Event Handle
  phEvent = ^hEvent;             // Pointer to Event Handle
  HWND = LongInt;
  MMTIME = LongInt;
Type
  evcb = record
    ulType       : LongInt;              // Event type (input)
    ulSubType    : LongInt;              // Event SubType (input)
    ulFlags      : LongInt;              // 0 (Input), status (Output)
    hstream      : HSTREAM;            // handle to stream for this event
    hid          : hID;                // handler Id (input/output)
    ulStatus     : LongInt;              // Event status (output)
    ulEventParm1 : LongInt;              // Event parameters (input), hID if implicit event
    ulEventParm2 : LongInt;              // Event parameters (input)
    ulEventParm3 : LongInt;              // Event parameters (input)
  end;
  pevcb = ^evcb;

Type
  MCIDRV_EVENT_PARMS = record
    ulLength:LongInt;   // Length of event parms                     */
    pevcb:PEVCB;      // pointer to event control block            */
    phevent:PHEVENT;    // pointer to the event to enable in network */
  end;
  PMCIDRV_EVENT_PARMS=^MCIDRV_EVENT_PARMS;

///*******************************************************************/
//                                                                   */
//  MCIDRV_SYNC  message flags                                       */
//                                                                   */
///*******************************************************************/
Const
  MCIDRV_SYNC_ENABLE            = $00000100;
  MCIDRV_SYNC_DISABLE           = $00000200;
  MCIDRV_SYNC_REC_PULSE         = $00000400;
  MCIDRV_SYNC_MASTER            = $00000800;
  MCIDRV_SYNC_SET_MASTER        = $00001000;

Type
  sync_evcb = record
    ulType       : LongInt;             // Event_SYNC
    ulSubType    : LongInt;             // Not used
    ulSyncFlags  : LongInt;             // 0 (input),status (output)
    hstream      : HStream;           // handle to stream for this event
    hid          : hID;               // Handler id
    ulStatus     : LongInt;             // Event status (output)
    mmtimeStart  : mmTime;            // Filled in by Sync/Stream manager
                                      //  at SpiEnableSync time. (input)
    mmtimeMaster : mmTime;            // Filled in by Master SH (input)
    mmtimeSlave  : mmTime;            // Filled in by slave SH. (output)
  end;
  pSync_evcb = ^sync_Evcb;

Type
  mciDrv_Sync_Parms = record
    hStreams                    : ^hStream;    // Pointer to list of stream IDs
    ulNumStreams                : LongInt;       // number of stream IDs
    mmTime                      : mmTime;      // Sync Pulse Time
    NonStreamMaster             : Boolean;        // TRUE if this device can be master
    pevcbSyncPulse              : pSync_evcb;  // pointer to EVCB address (output)
    hidMaster                   : hID;         // Master NULL sh ID (valid only on
                                               //    MCIDRV_SYNC_MASTER
    ulReserved1                 : LongInt;       // Reserved field
    ulReserved2                 : LongInt;       // Reserved field
  end;
  pmciDrv_Sync_Parms = ^mciDrv_Sync_Parms;


//*******************************************************************/
// contains information for open message for MCI drivers            */
//*******************************************************************/
Type
  mmDrv_Open_Parms = record
    hwndCallback                : hwnd;        // call back handle
    usDeviceID                  : Integer;      // The device ID assigned to this instance
    usDeviceType                : Integer;      // The device type number
    usDeviceOrd                 : Integer;      // The device ordinal number
    pInstance                   : Pointer;     //  pointer to the instance structure allocated
                                               //  initialized by the driver. The MCI driver
                                               // will fill in this parameter.
    szDevDLLName : Array[0..259] of Char;      // Character string containing the device
                                               // specific DLL name to call for the open.
                                               // (ie. ACPA.DLL)
    pszElementName              : pChar;       //   typically a file name or NULL
    usDevParmLen                : Integer;      // Device parameters data block length.
    pDevParm                    : Pointer;     // Device parameters data block. This data
                                               // block is unique to each type of device.
                                               // (ie. LVD "COM1 9600 N 7 1").
    Reserved0                   : Pointer;
    usResourceUnitsRequired     : Integer;      // number of resource units this instance
                                               // requires.
    usResourceClass             : Integer;      // resource class this instance belongs to
    usResourcePriority          : Integer;      // resource priority for this instance
    ulParam2                    : LongInt;       // Pointer to MCI_OPEN structure
  end;
  pmmDrv_Open_Parms = ^mmDrv_Open_Parms;


///*******************************************************************/
//                                                                   */
//  MCIDRV_RESTORE message flags                                     */
//                                                                   */
///*******************************************************************/

Type
  MCIDRV_RESTORE_PARMS = record
    ulLength:longint;        // Length of restore parms               */
    ulMasterVolume:longint;  // Master volume value                   */
  end;
  PMCIDRV_RESTORE_PARMS=^MCIDRV_RESTORE_PARMS;

function mdmDriverNotify( usDeviceID: Integer; wnd: Hwnd; usMsgType: Integer;
                          usUserParm: Integer; ulMsgParm: LongInt): LongInt; cdecl;

Implementation

function mdmDriverNotify( usDeviceID: Integer; wnd: Hwnd; usMsgType: Integer;
                          usUserParm: Integer; ulMsgParm: LongInt): LongInt; cdecl; external 'MDM' index 17;

End.
