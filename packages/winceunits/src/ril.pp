{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2008 Free Pascal development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 ********************************************************************** }
//-----------------------------------------------------------------------------
//
// RIL.H - Radio Interface Layer
//
//-----------------------------------------------------------------------------

//
//  Microsoft Windows Mobile 6.0 Platform Builder.
//


unit RIL;

{$CALLING cdecl}
{$INLINE ON}

interface

uses Windows;

const
      RILDLL = 'ril.dll';

const
      RIL_DRIVER_VERSION                    = $00020000;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Error Class | Each RIL error falls into a general error class bucket
//
// @comm In RIL, the low order 16 bits are divided into an 8-bit error class and
//       an 8-bit error value.  Use the RILERRORCLASS macro to obtain the error
//       class from a RIL HRESULT.
//
// -----------------------------------------------------------------------------
const
      RIL_ERRORCLASS_NONE                   = $00; // @constdefine Misc error
      RIL_ERRORCLASS_PASSWORD               = $01; // @constdefine Unspecified phone failure
      RIL_ERRORCLASS_SIM                    = $02; // @constdefine Problem with the SIM
      RIL_ERRORCLASS_NETWORKACCESS          = $03; // @constdefine Can't access the network
      RIL_ERRORCLASS_NETWORK                = $04; // @constdefine Error in the network
      RIL_ERRORCLASS_MOBILE                 = $05; // @constdefine Error in the mobile
      RIL_ERRORCLASS_NETWORKUNSUPPORTED     = $06; // @constdefine Unsupported by the network
      RIL_ERRORCLASS_MOBILEUNSUPPORTED      = $07; // @constdefine Unsupported by the mobile
      RIL_ERRORCLASS_BADPARAM               = $08; // @constdefine An invalid parameter was supplied
      RIL_ERRORCLASS_STORAGE                = $09; // @constdefine Error relating to storage
      RIL_ERRORCLASS_SMSC                   = $0A; // @constdefine Error relates to the SMSC
      RIL_ERRORCLASS_DESTINATION            = $0B; // @constdefine Error in the destination mobile
      RIL_ERRORCLASS_DESTINATIONUNSUPPORTED = $0C; // @constdefine Unsupported by destination mobile
      RIL_ERRORCLASS_RADIOUNAVAILABLE       = $0D; // @constdefine The Radio Module is Off or a radio module may not be present
      RIL_ERRORCLASS_GPRS                   = $0E;  // @constdefine GPRS related failures

function MAKE_RILERROR(errclass:byte; code:byte):ULONG; inline;

function RILERRORCLASS(rilerror:ULONG):byte; inline;

function ISRILERROR(rilerror:ULONG):BOOL; inline;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Error | Error codes
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      FACILITY_RIL                    = $0100;

      RIL_E_PHONEFAILURE           = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_MOBILE shl 8) or $01); // @constdefine Unspecified phone failure
      RIL_E_NOCONNECTION           = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_MOBILE shl 8) or $02); // @constdefine RIL has no connection to the phone
      RIL_E_LINKRESERVED           = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_MOBILE shl 8) or $03); // @constdefine RIL's link to the phone is reserved
      RIL_E_OPNOTALLOWED           = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_MOBILEUNSUPPORTED shl 8) or $04); // @constdefine Attempted operation isn't allowed
      RIL_E_OPNOTSUPPORTED         = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_MOBILEUNSUPPORTED shl 8) or $05); // @constdefine Attempted operation isn't supported
      RIL_E_PHSIMPINREQUIRED       = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_PASSWORD shl 8) or $06); // @constdefine PH-SIM PIN is required to perform this operation
      RIL_E_PHFSIMPINREQUIRED      = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_PASSWORD shl 8) or $07); // @constdefine PH-FSIM PIN is required to perform this operation
      RIL_E_PHFSIMPUKREQUIRED      = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_PASSWORD shl 8) or $08); // @constdefine PH-FSIM PUK is required to perform this operation
      RIL_E_SIMNOTINSERTED         = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_SIM shl 8) or $09); // @constdefine SIM isn't inserted into the phone
      RIL_E_SIMPINREQUIRED         = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_PASSWORD shl 8) or $0A); // @constdefine SIM PIN is required to perform this operation
      RIL_E_SIMPUKREQUIRED         = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_PASSWORD shl 8) or $0B); // @constdefine SIM PUK is required to perform this operation
      RIL_E_SIMFAILURE             = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_SIM shl 8) or $0C); // @constdefine SIM failure was detected
      RIL_E_SIMBUSY                = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_SIM shl 8) or $0D); // @constdefine SIM is busy
      RIL_E_SIMWRONG               = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_SIM shl 8) or $0E); // @constdefine Inorrect SIM was inserted
      RIL_E_INCORRECTPASSWORD      = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_PASSWORD shl 8) or $0F); // @constdefine Incorrect password was supplied
      RIL_E_SIMPIN2REQUIRED        = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_PASSWORD shl 8) or $10); // @constdefine SIM PIN2 is required to perform this operation
      RIL_E_SIMPUK2REQUIRED        = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_PASSWORD shl 8) or $11); // @constdefine SIM PUK2 is required to perform this operation
      RIL_E_MEMORYFULL             = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_STORAGE shl 8) or $12);  // @constdefine Storage memory is full
      RIL_E_INVALIDINDEX           = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_STORAGE shl 8) or $13);  // @constdefine Invalid storage index was supplied
      RIL_E_NOTFOUND               = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_STORAGE shl 8) or $14);  // @constdefine A requested storage entry was not found
      RIL_E_MEMORYFAILURE          = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_STORAGE shl 8) or $15);  // @constdefine Storage memory failure
      RIL_E_TEXTSTRINGTOOLONG      = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_BADPARAM shl 8) or $16);  // @constdefine Supplied text string is too long
      RIL_E_INVALIDTEXTSTRING      = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_BADPARAM shl 8) or $17);  // @constdefine Supplied text string contains invalid characters
      RIL_E_DIALSTRINGTOOLONG      = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_BADPARAM shl 8) or $18);  // @constdefine Supplied dial string is too long
      RIL_E_INVALIDDIALSTRING      = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_BADPARAM shl 8) or $19);  // @constdefine Supplied dial string contains invalid characters
      RIL_E_NONETWORKSVC           = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_NETWORKACCESS shl 8) or $1A);  // @constdefine Network service isn't available
      RIL_E_NETWORKTIMEOUT         = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_NETWORK shl 8) or $1B);  // @constdefine Network operation timed out
      RIL_E_EMERGENCYONLY          = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_NETWORKACCESS shl 8) or $1C);  // @constdefine Network can only be used for emergency calls
      RIL_E_NETWKPINREQUIRED       = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_PASSWORD shl 8) or $1D);  // @constdefine Network Personalization PIN is required to perform this operation
      RIL_E_NETWKPUKREQUIRED       = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_PASSWORD shl 8) or $1E);  // @constdefine Network Personalization PUK is required to perform this operation
      RIL_E_SUBSETPINREQUIRED      = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_PASSWORD shl 8) or $1F);  // @constdefine Network Subset Personalization PIN is required to perform this operation
      RIL_E_SUBSETPUKREQUIRED      = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_PASSWORD shl 8) or $20);  // @constdefine Network Subset Personalization PUK is required to perform this operation
      RIL_E_SVCPINREQUIRED         = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_PASSWORD shl 8) or $21);  // @constdefine Service Provider Personalization PIN is required to perform this operation
      RIL_E_SVCPUKREQUIRED         = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_PASSWORD shl 8) or $22);  // @constdefine Service Provider Personalization PUK is required to perform this operation
      RIL_E_CORPPINREQUIRED        = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_PASSWORD shl 8) or $23);  // @constdefine Corporate Personalization PIN is required to perform this operation
      RIL_E_CORPPUKREQUIRED        = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_PASSWORD shl 8) or $24);  // @constdefine Corporate Personalization PUK is required to perform this operation
      RIL_E_TELEMATICIWUNSUPPORTED = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_NETWORKUNSUPPORTED shl 8) or $25);  // @constdefine Telematic interworking isn't supported
      RIL_E_SMTYPE0UNSUPPORTED     = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_SMSC shl 8) or $26);  // @constdefine Type 0 messages aren't supported
      RIL_E_CANTREPLACEMSG         = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_SMSC shl 8) or $27);  // @constdefine Existing message cannot be replaced
      RIL_E_PROTOCOLIDERROR        = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_SMSC shl 8) or $28);  // @constdefine Uspecified error related to the message Protocol ID
      RIL_E_DCSUNSUPPORTED         = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_SMSC shl 8) or $29);  // @constdefine Specified message Data Coding Scheme isn't supported
      RIL_E_MSGCLASSUNSUPPORTED    = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_SMSC shl 8) or $2A);  // @constdefine Specified message class isn't supported
      RIL_E_DCSERROR               = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_SMSC shl 8) or $2B);  // @constdefine Unspecified error related to the message Data Coding Scheme
      RIL_E_CMDCANTBEACTIONED      = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_SMSC shl 8) or $2C);  // @constdefine Specified message Command cannot be executed
      RIL_E_CMDUNSUPPORTED         = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_SMSC shl 8) or $2D);  // @constdefine Specified message Command isn't supported
      RIL_E_CMDERROR               = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_SMSC shl 8) or $2E);  // @constdefine Unspecified error related to the message Command
      RIL_E_MSGBODYHEADERERROR     = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_SMSC shl 8) or $2F);  // @constdefine Unspecified error related to the message Body or Header
      RIL_E_SCBUSY                 = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_SMSC shl 8) or $30);  // @constdefine Message Service Center is busy
      RIL_E_NOSCSUBSCRIPTION       = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_SMSC shl 8) or $31);  // @constdefine No message Service Center subscription
      RIL_E_SCSYSTEMFAILURE        = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_SMSC shl 8) or $32);  // @constdefine Message service Center system failure occurred
      RIL_E_INVALIDADDRESS         = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_SMSC shl 8) or $33);  // @constdefine Specified address is invalid
      RIL_E_DESTINATIONBARRED      = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_SMSC shl 8) or $34);  // @constdefine Message destination is barred
      RIL_E_REJECTEDDUPLICATE      = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_SMSC shl 8) or $35);  // @constdefine Duplicate message was rejected
      RIL_E_VPFUNSUPPORTED         = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_SMSC shl 8) or $36);  // @constdefine Specified message Validity Period Format isn't supported
      RIL_E_VPUNSUPPORTED          = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_SMSC shl 8) or $37);  // @constdefine Specified message Validity Period isn't supported
      RIL_E_SIMMSGSTORAGEFULL      = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_STORAGE shl 8) or $38);  // @constdefine Message storage on the SIM is full
      RIL_E_NOSIMMSGSTORAGE        = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_SIM shl 8) or $39);  // @constdefine SIM isn't capable of storing messages
      RIL_E_SIMTOOLKITBUSY         = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_SIM shl 8) or $3A);  // @constdefine SIM Application Toolkit is busy
      RIL_E_SIMDOWNLOADERROR       = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_SIM shl 8) or $3B);  // @constdefine SIM data download error
      RIL_E_MSGSVCRESERVED         = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_NETWORKUNSUPPORTED shl 8) or $3C);  // @constdefine Messaging service is reserved
      RIL_E_INVALIDMSGPARAM        = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_BADPARAM shl 8) or $3D);  // @constdefine One of the message parameters is invalid
      RIL_E_UNKNOWNSCADDRESS       = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_SMSC shl 8) or $3E);  // @constdefine Unknown message Service Center address was specified
      RIL_E_UNASSIGNEDNUMBER       = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_DESTINATION shl 8) or $3F);  // @constdefine Specified message destination address is a currently unassigned phone number
      RIL_E_MSGBARREDBYOPERATOR    = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_NETWORKACCESS shl 8) or $40);  // @constdefine Message sending was barred by an operator
      RIL_E_MSGCALLBARRED          = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_NETWORKACCESS shl 8) or $41);  // @constdefine Message sending was prevented by outgoing calls barring
      RIL_E_MSGXFERREJECTED        = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_DESTINATION shl 8) or $42);  // @constdefine Sent message has been rejected by the receiving equipment
      RIL_E_DESTINATIONOUTOFSVC    = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_DESTINATION shl 8) or $43);  // @constdefine Message could not be delivered because destination equipment is out of service
      RIL_E_UNIDENTIFIEDSUBCRIBER  = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_NETWORKACCESS shl 8) or $44);  // @constdefine Sender's mobile ID isn't registered
      RIL_E_SVCUNSUPPORTED         = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_NETWORKUNSUPPORTED shl 8) or $45);  // @constdefine Requested messaging service isn't supported
      RIL_E_UNKNOWNSUBSCRIBER      = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_NETWORKACCESS shl 8) or $46);  // @constdefine Sender isn't recognized by the network
      RIL_E_NETWKOUTOFORDER        = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_NETWORK shl 8) or $47);  // @constdefine Long-term network failure
      RIL_E_NETWKTEMPFAILURE       = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_NETWORK shl 8) or $48);  // @constdefine Short-term network failure
      RIL_E_CONGESTION             = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_NETWORK shl 8) or $49);  // @constdefine Operation failed because of the high network traffic
      RIL_E_RESOURCESUNAVAILABLE   = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_NONE shl 8) or $4A);  // @constdefine Unspecified resources weren't available
      RIL_E_SVCNOTSUBSCRIBED       = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_NETWORKUNSUPPORTED shl 8) or $4B);  // @constdefine Sender isn't subscribed for the requested messaging service
      RIL_E_SVCNOTIMPLEMENTED      = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_NETWORKUNSUPPORTED shl 8) or $4C);  // @constdefine Requested messaging service isn't implemented on the network
      RIL_E_INVALIDMSGREFERENCE    = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_BADPARAM shl 8) or $4D);  // @constdefine Imvalid message reference value was used
      RIL_E_INVALIDMSG             = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_BADPARAM shl 8) or $4E);  // @constdefine Message was determined to be invalid for unspecified reasons
      RIL_E_INVALIDMANDATORYINFO   = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_BADPARAM shl 8) or $4F);  // @constdefine Mandatory message information is invalid or missing
      RIL_E_MSGTYPEUNSUPPORTED     = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_NETWORKUNSUPPORTED shl 8) or $50);  // @constdefine The message type is unsupported
      RIL_E_ICOMPATIBLEMSG         = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_NETWORKUNSUPPORTED shl 8) or $51);  // @constdefine Sent message isn't compatible with the network
      RIL_E_INFOELEMENTUNSUPPORTED = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_NETWORKUNSUPPORTED shl 8) or $52);  // @constdefine An information element specified in the message isn't supported
      RIL_E_PROTOCOLERROR          = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_NETWORK shl 8) or $53);  // @constdefine Unspefied protocol error
      RIL_E_NETWORKERROR           = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_NETWORK shl 8) or $54);  // @constdefine Unspecified network error
      RIL_E_MESSAGINGERROR         = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_NETWORK shl 8) or $55);  // @constdefine Unspecified messaging error
      RIL_E_NOTREADY               = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_NONE shl 8) or $56);  // @constdefine RIL isn't yet ready to perform the requested operation
      RIL_E_TIMEDOUT               = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_NONE shl 8) or $57);  // @constdefine Operation timed out
      RIL_E_CANCELLED              = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_NONE shl 8) or $58);  // @constdefine Operation was cancelled
      RIL_E_NONOTIFYCALLBACK       = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_NONE shl 8) or $59);  // @constdefine Requested operation requires an RIL notification callback, which wasn't provided
      RIL_E_OPFMTUNAVAILABLE       = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_NETWORKUNSUPPORTED shl 8) or $5A);  // @constdefine Operator format isn't available
      RIL_E_NORESPONSETODIAL       = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_NETWORKACCESS shl 8) or $5B);  // @constdefine Dial operation hasn't received a response for a long time
      RIL_E_SECURITYFAILURE        = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_NONE shl 8) or $5C);  // @constdefine Security failure
      RIL_E_RADIOFAILEDINIT        = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_NONE shl 8) or $5D);  // @constdefine Radio failed to initialize correctly
      RIL_E_DRIVERINITFAILED       = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_RADIOUNAVAILABLE shl 8) or $5E);  // @constdefine There was a problem initializing the radio driver
      RIL_E_RADIONOTPRESENT        = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_RADIOUNAVAILABLE shl 8) or $5F);  // @constdefine The Radio is not present
      RIL_E_RADIOOFF               = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_RADIOUNAVAILABLE shl 8) or $60);  // @constdefine The Radio is in Off mode
      RIL_E_ILLEGALMS              = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_GPRS shl 8) or $61);  // @constdefine Illegal MS
      RIL_E_ILLEGALME              = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_GPRS shl 8) or $62);  // @constdefine Illegal ME
      RIL_E_GPRSSERVICENOTALLOWED  = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_GPRS shl 8) or $63);  // @constdefine GPRS Service not allowed
      RIL_E_PLMNNOTALLOWED         = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_GPRS shl 8) or $64);  // @constdefine PLMN not allowed
      RIL_E_LOCATIONAREANOTALLOWED = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_GPRS shl 8) or $65);  // @constdefine Location area not allowed
      RIL_E_ROAMINGNOTALLOWEDINTHISLOCATIONAREA = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_GPRS shl 8) or $66); // @constdefine Roaming not allowed in this location area
      RIL_E_SERVICEOPTIONNOTSUPPORTED           = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_GPRS shl 8) or $67); // @constdefine Service option not supported
      RIL_E_REQUESTEDSERVICEOPTIONNOTSUBSCRIBED = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_GPRS shl 8) or $68); // @constdefine Requested service option not subscribed
      RIL_E_SERVICEOPTIONTEMPORARILYOUTOFORDER  = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_GPRS shl 8) or $69); // @constdefine Service option temporarily out of order
      RIL_E_PDPAUTHENTICATIONFAILURE            = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_GPRS shl 8) or $6A); // @constdefine PDP authentication failure
      RIL_E_INVALIDMOBILECLASS                  = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_GPRS shl 8) or $6B); // @constdefine invalid mobile class
      RIL_E_UNSPECIFIEDGPRSERROR                = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_GPRS shl 8) or $6C); // @constdefine unspecific GPRS error
      RIL_E_RADIOREBOOTED                = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_NONE shl 8) or $6D); // @constdefine the command failed because the radio reset itself unexpectedly
      RIL_E_INVALIDCONTEXTSTATE          = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_NONE shl 8) or $6E); // @constdefine the command failed because the requested context state is invalid
      RIL_E_MAXCONTEXTS                  = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_NONE shl 8) or $6F); // @constdefine the command failed because there are no more radio contexts.
      RIL_E_SYNCHRONOUS_DATA_UNAVAILABLE = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_NONE shl 8) or $70); // @constdefine the cached notification data is not present
      RIL_E_INVALIDASYNCCOMMANDRESPONSE  = (SEVERITY_ERROR shl 31) or (FACILITY_RIL shl 16) or ((RIL_ERRORCLASS_NONE shl 8) or $71); // @constdefine The RIL driver has issued an invalid asynchronous command response (hr == 0)


// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Notification Class | Notification classes
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_NCLASS_FUNCRESULT  = $00000000;      // @constdefine API call results
      RIL_NCLASS_CALLCTRL    = $00010000;      // @constdefine Call control notifications
      RIL_NCLASS_MESSAGE     = $00020000;      // @constdefine Messaging notifications
      RIL_NCLASS_NETWORK     = $00040000;      // @constdefine Network-related notifications
      RIL_NCLASS_SUPSERVICE  = $00080000;      // @constdefine Supplementary service notifications
      RIL_NCLASS_PHONEBOOK   = $00100000;      // @constdefine Phonebook notifications
      RIL_NCLASS_SIMTOOLKIT  = $00200000;      // @constdefine SIM Toolkit notifications
      RIL_NCLASS_MISC        = $00400000;      // @constdefine Miscellaneous notifications
      RIL_NCLASS_RADIOSTATE  = $00800000;      // @constdefine Notifications Pertaining to changes in Radio State
      RIL_NCLASS_POLLING     = $01000000;      // @constdefine polling related APIs
      RIL_NCLASS_NDIS        = $40000000;      // @constdefine Nofitifcations that won't be picked up by all.
      RIL_NCLASS_DEVSPECIFIC = $80000000;      // @constdefine Reserved for device specific notifications
      RIL_NCLASS_ALL         = $01FF0000;      // @constdefine All notification classes (except DevSpecifc)

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants API Result | API call results (RIL_NCLASS_FUNCRESULT)
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_RESULT_OK          = $00000001 or RIL_NCLASS_FUNCRESULT;  // @constdefine RIL API call succeded; lpData is NULL
      RIL_RESULT_NOCARRIER   = $00000002 or RIL_NCLASS_FUNCRESULT;  // @constdefine RIL API failed because no carrier was detected; lpData is NULL
      RIL_RESULT_ERROR       = $00000003 or RIL_NCLASS_FUNCRESULT;  // @constdefine RIL API failed; lpData points to RIL_E_* constant
      RIL_RESULT_NODIALTONE  = $00000004 or RIL_NCLASS_FUNCRESULT;  // @constdefine RIL API failed because no dialtone was detected; lpData is NULL
      RIL_RESULT_BUSY        = $00000005 or RIL_NCLASS_FUNCRESULT;  // @constdefine RIL API failed because the line was busy; lpData is NULL
      RIL_RESULT_NOANSWER    = $00000006 or RIL_NCLASS_FUNCRESULT;  // @constdefine RIL API failed because of the lack of answer; lpData is NULL
      RIL_RESULT_CALLABORTED = $00000007 or RIL_NCLASS_FUNCRESULT;  // @constdefine RIL API failed because it was cancelled prior to completion; lpData is NULL
      RIL_RESULT_CALLDROPPED = $00000008 or RIL_NCLASS_FUNCRESULT;  // @constdefine RIL API failed because the network dropped the call; lpData is NULL
      RIL_RESULT_RADIOOFF    = $00000009 or RIL_NCLASS_FUNCRESULT;  // @constdefine RIL API failed because the radio was shut offl; lpData is NULL

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Notification Call Control | Call control notifications (RIL_NCLASS_CALLCTRL)
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_NOTIFY_RING                  = $00000001 or RIL_NCLASS_CALLCTRL;  // @constdefine Incoming call; lpData points to RILRINGINFO
      RIL_NOTIFY_CONNECT               = $00000002 or RIL_NCLASS_CALLCTRL;  // @constdefine Data/voice connection has been established; lpData points to RILCONNECTINFO
      RIL_NOTIFY_DISCONNECT            = $00000003 or RIL_NCLASS_CALLCTRL;  // @constdefine Data/voice connection has been terminated; lpData points to RIL_DISCINIT_* constant
      RIL_NOTIFY_DATASVCNEGOTIATED     = $00000004 or RIL_NCLASS_CALLCTRL;  // @constdefine Data connection service has been negotiated; lpData points to RILSERVICEINFO
      RIL_NOTIFY_CALLSTATECHANGED      = $00000005 or RIL_NCLASS_CALLCTRL;  // @constdefine RIL has performed an operation that may have changed state of existing calls; lpData is NULL
      RIL_NOTIFY_EMERGENCYMODEENTERED  = $00000006 or RIL_NCLASS_CALLCTRL;  // @constdefine RIL has enetered emergency mode; lpData is NULL
      RIL_NOTIFY_EMERGENCYMODEEXITED   = $00000007 or RIL_NCLASS_CALLCTRL;  // @constdefine RIL has exited emergency mode; lpData is NULL
      RIL_NOTIFY_EMERGENCYHANGUP       = $00000008 or RIL_NCLASS_CALLCTRL;  // @constdefine Existsing calls (if any) were hung up in RIL emergency mode; lpData is NULL
      RIL_NOTIFY_HSCSDPARAMSNEGOTIATED = $00000009 or RIL_NCLASS_CALLCTRL;  // @constdefine HSCSD parameters for a call has been negotiated; lpData points to RILCALLHSCSDINFO
      RIL_NOTIFY_DIAL                  = $0000000A or RIL_NCLASS_CALLCTRL;  // @constdefine Outgoing call; lpData points to RILDIALINFO
      RIL_NOTIFY_CALLPROGRESSINFO      = $0000000B or RIL_NCLASS_CALLCTRL;  // @constdefine CPI notification; lpData points to RILCALLINFO
      RIL_NOTIFY_CURRENTLINECHANGED    = $0000000C or RIL_NCLASS_CALLCTRL;  // @constdefine Current line has changed notification; lpData points to DWORD with new current address id
      RIL_NOTIFY_GPRS_DISCONNECT       = $0000000D or RIL_NCLASS_CALLCTRL;  // @constdefine GPRS connection has been terminated; lpData points to RILGPRSCONTEXTACTIVATED sturct

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Notification Messaging | Messaging notifications (RIL_MCLASS_MESSAGE)
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_NOTIFY_MESSAGE              = $00000001 or RIL_NCLASS_MESSAGE;  // @constdefine Incoming message; lpData points to RILMESSAGE
      RIL_NOTIFY_BCMESSAGE            = $00000002 or RIL_NCLASS_MESSAGE;  // @constdefine Incoming broadcast message; lpData points to RILMESSAGE
      RIL_NOTIFY_STATUSMESSAGE        = $00000003 or RIL_NCLASS_MESSAGE;  // @constdefine Incoming status-report message; lpData points to RILMESSAGE
      RIL_NOTIFY_MSGSTORED            = $00000004 or RIL_NCLASS_MESSAGE;  // @constdefine A message has been added to storage; lpData points to the storage index assigned to the new message
      RIL_NOTIFY_MSGDELETED           = $00000005 or RIL_NCLASS_MESSAGE;  // @constdefine A message has been deleted from storage; lpData points to the storage index occupied by the deleted message
      RIL_NOTIFY_MSGSTORAGECHANGED    = $00000006 or RIL_NCLASS_MESSAGE;  // @constdefine One of the message storage locations has been changed; lpData points to RILMSGSTORAGEINFO
      RIL_NOTIFY_MESSAGE_IN_SIM       = $00000007 or RIL_NCLASS_MESSAGE;  // @constdefine Incoming message stored to SIM; lpData points to the storage RILMESSAGE_IN_SIM
      RIL_NOTIFY_BCMESSAGE_IN_SIM     = $00000008 or RIL_NCLASS_MESSAGE;  // @constdefine Incoming broadcast message stored to SIM; lpData points to RILMESSAGE_IN_SIM
      RIL_NOTIFY_STATUSMESSAGE_IN_SIM = $00000009 or RIL_NCLASS_MESSAGE;  // @constdefine Incoming status-report message stored to SIM; lpData points to RILMESSAGE_IN_SIM

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Notification Network | Network-related notifications (RIL_NCLASS_NETWORK)
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_NOTIFY_REGSTATUSCHANGED     = $00000001 or RIL_NCLASS_NETWORK;  // @constdefine Network registration status has changed; lpData points to the new status (RIL_REGSTAT_* constant)
      RIL_NOTIFY_CALLMETER            = $00000002 or RIL_NCLASS_NETWORK;  // @constdefine Call meter has changed; lpData points to a DWORD containing new current call meter value
      RIL_NOTIFY_CALLMETERMAXREACHED  = $00000003 or RIL_NCLASS_NETWORK;  // @constdefine Call meter maximum has been reached; lpData is NULL
      RIL_NOTIFY_GPRSREGSTATUSCHANGED = $00000004 or RIL_NCLASS_NETWORK;  // @constdefine Network registration status has changed; lpData points to the new status (RIL_REGSTAT_* constant)
      RIL_NOTIFY_SYSTEMCHANGED        = $00000005 or RIL_NCLASS_NETWORK;  // @constdefine This indicates that the type of coverage which is available has changed.  Typically one would expect IS-95A or 1xRTT, however CDMA does allow overlay systems; lpData is <t DWORD> of type RIL_SYSTEMTYPE_ flags
      RIL_NOTIFY_GPRSCONNECTIONSTATUS = $00000006 or RIL_NCLASS_NETWORK;  // @constdefine This indicates the pdp context state has changed. lpData points to RILGPRSCONTEXTACTIVATED
      RIL_NOTIFY_SYSTEMCAPSCHANGED    = $00000007 or RIL_NCLASS_NETWORK;  // @constdefine This indicates the system capability has changed. lpData points to the new system capability (RIL_SYSTEMCAPS_* constant)
      RIL_NOTIFY_LOCATIONUPDATE       = $00000008 or RIL_NCLASS_NETWORK;  // @constdefine This indicates the location data has changed. lpData points to RILLOCATIONINFO

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Notification Supplementary Service | Supplementary service notifications (RIL_NCLASS_SUPSERVICE)
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_NOTIFY_CALLERID       = $00000001 or RIL_NCLASS_SUPSERVICE;  // @constdefine Incoming call CallerID information; lpData points to RILREMOTEPARTYINFO
      RIL_NOTIFY_DIALEDID       = $00000002 or RIL_NCLASS_SUPSERVICE;  // @constdefine Initiated call DialedID information; lpData points to RILREMOTEPARTYINFO
      RIL_NOTIFY_CALLWAITING    = $00000003 or RIL_NCLASS_SUPSERVICE;  // @constdefine Call Waiting information; lpData points to RILCALLWAITINGINFO
      RIL_NOTIFY_SUPSERVICEDATA = $00000004 or RIL_NCLASS_SUPSERVICE;  // @constdefine Ustructured supplementary service data; lpData points to RILSUPSERVICEDATA
      RIL_NOTIFY_INTERMEDIATESS = $00000005 or RIL_NCLASS_SUPSERVICE;  // @constdefine Ustructured supplementary service data; lpData points to RILINTERMEDIATESSINFO
      RIL_NOTIFY_UNSOLICITEDSS  = $00000006 or RIL_NCLASS_SUPSERVICE;  // @constdefine Ustructured supplementary service data; lpData points to RILUNSOLICITEDSSINFO

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Notification Phonebook | Phonebook notifications (RIL_NCLASS_PHONEBOOK)
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_NOTIFY_PHONEBOOKENTRYSTORED    = $00000001 or RIL_NCLASS_PHONEBOOK;  // @constdefine A phonebook entry has been added to storage; lpData points to the storage
                                                                               // index assigned to the new entry (ifdwIndex is RIL_PBINDEX_FIRSTAVAILABLE, the new entry was stored in the first available location)
      RIL_NOTIFY_PHONEBOOKENTRYDELETED   = $00000002 or RIL_NCLASS_PHONEBOOK;  // @constdefine A phonebook entry has been deleted from storage; lpData points to the storage index occupied by the deleted entry
      RIL_NOTIFY_PHONEBOOKSTORAGECHANGED = $00000003 or RIL_NCLASS_PHONEBOOK;  // @constdefine Phonebook storage location has been changed; lpData points to RIL_PBLOC_* constant

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Notification Toolkit | SIM Toolkit notifications (RIL_NCLASS_SIMTOOLKIT)
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_NOTIFY_SIMTOOLKITCMD        = $00000001 or RIL_NCLASS_SIMTOOLKIT;  // @constdefine A SIM Toolkit command was not handled by the radio; lpData points to array of bytes containing the command
      RIL_NOTIFY_SIMTOOLKITCALLSETUP  = $00000002 or RIL_NCLASS_SIMTOOLKIT;  // @constdefine SIM Toolkit is trying to set up a call and call conditions were successfully checked by the radio;
                                                                             // lpData points to a DWORD containing the redial timeout for the call (in milliseconds)
      RIL_NOTIFY_SIMTOOLKITEVENT      = $00000003 or RIL_NCLASS_SIMTOOLKIT;  // @constdefine A SIM Toolkit command was handled by the radio or the radio sent a SIm Toolkit command response to the SIM;
                                                                             // lpData points to array of bytes containing the command or response sent
      RIL_NOTIFY_SIMTOOLKITSESSIONEND = $00000004 or RIL_NCLASS_SIMTOOLKIT;  // @constdefine A SIM Toolkit command session is ending

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Notification Radio State Change | Radio State Change notifications (RIL_NCLASS_RADIOSTATE)
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_NOTIFY_RADIOEQUIPMENTSTATECHANGED = $00000001 or RIL_NCLASS_RADIOSTATE;  // @constdefine Carries a STRUCT (RILEQUIPMENTSTATE) stating The Radio equiptmentstate has changed, also notifies a driver defined Radio ON or OFF state
      RIL_NOTIFY_RADIOPRESENCECHANGED       = $00000002 or RIL_NCLASS_RADIOSTATE;  // @constdefine Carries a dword (RIL_RADIOPRESENCE_*) stating that a Radio Module/Driver has been changed (removed, inserted, etc)
      RIL_NOTIFY_RADIORESET                 = $00000003 or RIL_NCLASS_RADIOSTATE;  // @constdefine The driver has detected that the radio reset itself. lpData points to NULL

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Notification Misc | Miscellaneous notifications (RIL_NCLASS_MISC)
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_NOTIFY_SIMNOTACCESSIBLE         = $00000001 or RIL_NCLASS_MISC;  // @constdefine SIM card has been removed or has failed to respond; lpData is NULL
      RIL_NOTIFY_DTMFSIGNAL               = $00000002 or RIL_NCLASS_MISC;  // @constdefine A DTMF signal has been detected; lpData points to char
      RIL_NOTIFY_GPRSCLASS_NETWORKCHANGED = $00000003 or RIL_NCLASS_MISC;  // @constdefine Network has indicated a change in GPRS class
                                                                           // lpData points to a DWORD containing the new RIL_GPRSCLASS_* value
      RIL_NOTIFY_GPRSCLASS_RADIOCHANGED   = $00000004 or RIL_NCLASS_MISC;  // @constdefine The radio has indicated a change in GPRS class
                                                                           // lpData points to a DWORD containing the new RIL_GPRSCLASS_* value
      RIL_NOTIFY_SIGNALQUALITY            = $00000005 or RIL_NCLASS_MISC;  // @constdefine Signal Quality Notification
                                                                           // lpData points to a RILSIGNALQUALITY structure
      RIL_NOTIFY_MAINTREQUIRED            = $00000006 or RIL_NCLASS_MISC;  // @constdefine BS notification that MS requires servicing; lpdata is NULL
      RIL_NOTIFY_PRIVACYCHANGED           = $00000007 or RIL_NCLASS_MISC;  // @constdefine Call Privacy Status; lpData points to DWORD of value RIL_CALLPRIVACY_*
      RIL_NOTIFY_SIM_DATACHANGE           = $00000008 or RIL_NCLASS_MISC;  // @constdefine data change notification; lpData points to DWORD of value RIL_SIMDATACHANGE_*
      RIL_NOTIFY_ATLOGGING                = $00000009 or RIL_NCLASS_MISC;  // @constdefine at command log data present
      RIL_NOTIFY_SIMSTATUSCHANGED         = $0000000A or RIL_NCLASS_MISC;  // @constdefine SIM card state has changed. Carries a DWORD (RIL_SIMSTATUSCHANGED_*) with the current state.
                                                                           // Notification is sent only when encountering error conditions from the radio.
      RIL_NOTIFY_EONS                     = $0000000B or RIL_NCLASS_MISC;  // @constdefine EONS information ready or updated; lpData is NULL
      RIL_NOTIFY_SIMSECURITYSTATUS        = $0000000C or RIL_NCLASS_MISC;  // @constdefine SIM security status change; lpData points to LPRILSIMSECURITYSTATUS
      RIL_NOTIFY_LINESTATE                = $0000000D or RIL_NCLASS_MISC;  // @constdefine line state; lpData points to a DWORD of value RIL_LINESTAT_*
      RIL_NOTIFY_BEARERSVCINFO            = $0000000E or RIL_NCLASS_MISC;  // @constdefine bearer service information; lpData points to LPRILBEARERSVCINFO
      RIL_NOTIFY_DATACOMPINFO             = $0000000F or RIL_NCLASS_MISC;  // @constdefine data compression information; lpData points to LPRILDATACOMPINFO
      RIL_NOTIFY_EQUIPMENTINFO            = $00000010 or RIL_NCLASS_MISC;  // @constdefine equipment information; lpData points to LPRILEQUIPMENTINFO
      RIL_NOTIFY_ERRORCORRECTIONINFO      = $00000011 or RIL_NCLASS_MISC;  // @constdefine error correction information; lpData points to LPRILERRORCORRECTIONINFO
      RIL_NOTIFY_GPRSADDRESS              = $00000012 or RIL_NCLASS_MISC;  // @constdefine GPRS address; lpData points to an array of WCHAR values that indicate the address
      RIL_NOTIFY_GPRSATTACHED             = $00000013 or RIL_NCLASS_MISC;  // @constdefine GPRS attach state; lpData points to a BOOL that indicates attach state
      RIL_NOTIFY_GPRSCONTEXT              = $00000014 or RIL_NCLASS_MISC;  // @constdefine GPRS context list; lpData points to LPRILGPRSCONTEXT
      RIL_NOTIFY_GPRSCONTEXTACTIVATED     = $00000015 or RIL_NCLASS_MISC;  // @constdefine GPRS context activated list; lpData points to LPRILGPRSCONTEXTACTIVATED
      RIL_NOTIFY_QOSMIN                   = $00000016 or RIL_NCLASS_MISC;  // @constdefine minimum quality of service profile ; lpData points to LPRILGPRSQOSPROFILE
      RIL_NOTIFY_QOSREQ                   = $00000017 or RIL_NCLASS_MISC;  // @constdefine requested quality of service profile ; lpData points to LPRILGPRSQOSPROFILE
      RIL_NOTIFY_RLPOPTIONS               = $00000018 or RIL_NCLASS_MISC;  // @constdefine requested quality of service profile ; lpData points to LPRILRLPINFO
      RIL_NOTIFY_NITZ                     = $00000019 or RIL_NCLASS_MISC;  // @constdefine NITZ Date/Time notification. lpData points to a RILNITZINFO structure.

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Notification Device Specific | Device Specific notifications (RIL_NCLASS_DEVSPECIFIC)
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_NOTIFY_LOCATION   = $00008000 or RIL_NCLASS_DEVSPECIFIC; // @constdefine Location Services; lpData points to DWORD of value RIL_LOCATION_*
      RIL_NOTIFY_ROAMSTATUS = $00008001 or RIL_NCLASS_DEVSPECIFIC; // @constdefine Roaming Status; lpData points to DWORD of value RIL_ROAMSTATUS_*
                                                                   // lpData ponts to DWORD of [ RIL_NDIS_XON |RIL_NDIS_XOFF ]

//
// Macro to extract notification class from notification code
//
function NCLASS_FROM_NOTIFICATION(code:ULONG):ULONG; inline;


//
// Structure parameter flags
//

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILNDISIPCONFIG
//
// @comm None
//
// -----------------------------------------------------------------------------
//
const
      RIL_PARAM_NDISIPCONFIG_PROTOCOL_IPV4 = $00000001;  // @paramdefine
      RIL_PARAM_NDISIPCONFIG_PROTOCOL_IPV6 = $00000002;  // @paramdefine
//
// ipv4 defines
      RIL_PARAM_NDISIPCONFIG_IPADDR         = $00000001;  // @paramdefine
      RIL_PARAM_NDISIPCONFIG_PRIMARYDNS     = $00000002;  // @paramdefine
      RIL_PARAM_NDISIPCONFIG_SECONDARYDNS   = $00000004;  // @paramdefine
      RIL_PARAM_NDISIPCONFIG_DEFAULTGATEWAY = $00000008;  // @paramdefine
      RIL_PARAM_NDISIPCONFIG_SUBNETMASK     = $00000010;  // @paramdefine
      RIL_PARAM_NDISIPCONFIG_ALL            = $0000001f;  // @paramdefine
//
// ipv6 defines
      RIL_PARAM_NDISIPCONFIG_IPV6_IPADDR         = $00000001;  // @paramdefine
      RIL_PARAM_NDISIPCONFIG_IPV6_PRIMARYDNS     = $00000002;  // @paramdefine
      RIL_PARAM_NDISIPCONFIG_IPV6_SECONDARYDNS   = $00000004;  // @paramdefine
      RIL_PARAM_NDISIPCONFIG_IPV6_DEFAULTGATEWAY = $00000008;  // @paramdefine
      RIL_PARAM_NDISIPCONFIG_IPV6_SUBNETMASK     = $00000010;  // @paramdefine
      RIL_PARAM_NDISIPCONFIG_IPV6_FLOWINFO       = $00000020;  // @paramdefine
      RIL_PARAM_NDISIPCONFIG_IPV6_SCOPEID        = $00000040;  // @paramdefine
      RIL_PARAM_NDISIPCONFIG_IPV6_ALL            = $0000007f;  // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILNDISGPRSCONTEXT
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_RILNDISGPRSCONTEXT_USERNAME = $00000001;  // @paramdefine
      RIL_PARAM_RILNDISGPRSCONTEXT_PASSWORD = $00000002;  // @paramdefine
      RIL_PARAM_RILNDISGPRSCONTEXT_DNS1     = $00000004;  // @paramdefine
      RIL_PARAM_RILNDISGPRSCONTEXT_DNS2     = $00000008;  // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILNDISGPRSCONTEXTRESPONSE
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_RILNDISGPRSCONTEXTRESPONSE_IPCONFIG  = $00000001;  // @paramdefine
      RIL_PARAM_RILNDISGPRSCONTEXTRESPONSE_FUNCTIONS = $00000002;  // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILNDISSTATUS
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_RILNDISSTATUS_IPCONFIG    = $00000001;  // @paramdefine
      RIL_PARAM_RILNDISSTATUS_FLOWCONTROL = $00000002;  // @paramdefine


// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILADDRESS
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_A_TYPE                          = $00000001; // @paramdefine
      RIL_PARAM_A_NUMPLAN                       = $00000002; // @paramdefine
      RIL_PARAM_A_ADDRESS                       = $00000004; // @paramdefine
      RIL_PARAM_A_ALL                           = $00000007; // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILSUBADDRESS
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_SA_TYPE                         = $00000001; // @paramdefine
      RIL_PARAM_SA_SUBADDRESS                   = $00000002; // @paramdefine
      RIL_PARAM_SA_ALL                          = $00000003; // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILSERIALPORTSTATS
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_SPS_READBITSPERSECOND           = $00000001; // @paramdefine
      RIL_PARAM_SPS_WRITTENBITSPERSECOND        = $00000002; // @paramdefine
      RIL_PARAM_SPS_ALL                         = $00000003; // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILSUBSCRIBERINFO
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_SI_ADDRESS                      = $00000001; // @paramdefine
      RIL_PARAM_SI_DESCRIPTION                  = $00000002; // @paramdefine
      RIL_PARAM_SI_SPEED                        = $00000004; // @paramdefine
      RIL_PARAM_SI_SERVICE                      = $00000008; // @paramdefine
      RIL_PARAM_SI_ITC                          = $00000010; // @paramdefine
      RIL_PARAM_SI_ADDRESSID                    = $00000020; // @paramdefine
      RIL_PARAM_SI_ALL                          = $0000003f; // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILOPERATORNAMES
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_ON_LONGNAME                     = $00000001; // @paramdefine
      RIL_PARAM_ON_SHORTNAME                    = $00000002; // @paramdefine
      RIL_PARAM_ON_NUMNAME                      = $00000004; // @paramdefine
      RIL_PARAM_ON_COUNTRY_CODE                 = $00000008; // @paramdefine
      RIL_PARAM_ON_GSM_ACT                      = $00000010; // @paramdefine
      RIL_PARAM_ON_GSMCOMPACT_ACT               = $00000020; // @paramdefine
      RIL_PARAM_ON_UMTS_ACT                     = $00000040; // @paramdefine
      RIL_PARAM_ON_ALL                          = $0000007F; // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILOPERATORINFO
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_OI_INDEX                        = $00000001; // @paramdefine
      RIL_PARAM_OI_STATUS                       = $00000002; // @paramdefine
      RIL_PARAM_OI_NAMES                        = $00000004; // @paramdefine
      RIL_PARAM_OI_ALL                          = $00000007; // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILCALLERIDSETTINGS
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_CIDS_PROVISIONING               = $00000001; // @paramdefine
      RIL_PARAM_CIDS_STATUS                     = $00000002; // @paramdefine
      RIL_PARAM_CIDS_ALL                        = $00000003; // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILHIDEIDSETTINGS
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_HIDS_STATUS                     = $00000001; // @paramdefine
      RIL_PARAM_HIDS_PROVISIONING               = $00000002; // @paramdefine
      RIL_PARAM_HIDS_ALL                        = $00000003; // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILDIALEDIDSETTINGS
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_DIDS_PROVISIONING               = $00000001; // @paramdefine
      RIL_PARAM_DIDS_STATUS                     = $00000002; // @paramdefine
      RIL_PARAM_DIDS_ALL                        = $00000003; // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILHIDECONNECTEDIDSETTINGS
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_HCIDS_PROVISIONING              = $00000001; // @paramdefine
      RIL_PARAM_HCIDS_STATUS                    = $00000002; // @paramdefine
      RIL_PARAM_HCIDS_ALL                       = $00000003; // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILCLOSEDGROUPSETTINGS
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_CGS_STATUS                      = $00000001; // @paramdefine
      RIL_PARAM_CGS_INDEX                       = $00000002; // @paramdefine
      RIL_PARAM_CGS_INFO                        = $00000004; // @paramdefine
      RIL_PARAM_CGS_ALL                         = $00000007; // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILCALLFORWARDINGSETTINGS
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_CFS_STATUS                      = $00000001; // @paramdefine
      RIL_PARAM_CFS_INFOCLASSES                 = $00000002; // @paramdefine
      RIL_PARAM_CFS_ADDRESS                     = $00000004; // @paramdefine
      RIL_PARAM_CFS_SUBADDRESS                  = $00000008; // @paramdefine
      RIL_PARAM_CFS_DELAYTIME                   = $00000010; // @paramdefine
      RIL_PARAM_CFS_ALL                         = $0000001f; // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILCALLINFO
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_CI_ID                           = $00000001; // @paramdefine
      RIL_PARAM_CI_DIRECTION                    = $00000002; // @paramdefine
      RIL_PARAM_CI_STATUS                       = $00000004; // @paramdefine
      RIL_PARAM_CI_TYPE                         = $00000008; // @paramdefine
      RIL_PARAM_CI_MULTIPARTY                   = $00000010; // @paramdefine
      RIL_PARAM_CI_ADDRESS                      = $00000020; // @paramdefine
      RIL_PARAM_CI_DESCRIPTION                  = $00000040; // @paramdefine
      RIL_PARAM_CI_CPISTATUS                    = $00000080; // @paramdefine
      RIL_PARAM_CI_DISCONNECTCODE               = $00000100; // @paramdefine
//Note: RIL_PARAM_CI_STATUS and RIL_PARAM_CI_CPISTATUS are mutually exclusive
// parameters because they define how the dwStatus variable is used.
// Therefore, there is no RIL_PARAM_CI_ALL to avoid any ambiguity.

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILGAININFO
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_GI_TXGAIN                       = $00000001; // @paramdefine
      RIL_PARAM_GI_RXGAIN                       = $00000002; // @paramdefine
      RIL_PARAM_GI_ALL                          = $00000003; // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILAUDIODEVICEINFO
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_ADI_TXDEVICE                    = $00000001; // @paramdefine
      RIL_PARAM_ADI_RXDEVICE                    = $00000002; // @paramdefine
      RIL_PARAM_ADI_ALL                         = $00000003; // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILHSCSDINFO
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_HSCSDI_TRANSPRXTIMESLOTS        = $00000001; // @paramdefine
      RIL_PARAM_HSCSDI_TRANSPCHANNELCODINGS     = $00000002; // @paramdefine
      RIL_PARAM_HSCSDI_NONTRANSPRXTIMESLOTS     = $00000004; // @paramdefine
      RIL_PARAM_HSCSDI_NONTRANSPCHANNELCODINGS  = $00000008; // @paramdefine
      RIL_PARAM_HSCSDI_AIRINTERFACEUSERRATE     = $00000010; // @paramdefine
      RIL_PARAM_HSCSDI_RXTIMESLOTSLIMIT         = $00000020; // @paramdefine
      RIL_PARAM_HSCSDI_AUTOSVCLEVELUPGRADING    = $00000040; // @paramdefine
      RIL_PARAM_HSCSDI_ALL                      = $0000007f; // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILCALLHSCSDINFO
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_CHSCSDI_RXTIMESLOTS             = $00000001; // @paramdefine
      RIL_PARAM_CHSCSDI_TXTIMESLOTS             = $00000002; // @paramdefine
      RIL_PARAM_CHSCSDI_AIRINTERFACEUSERRATE    = $00000004; // @paramdefine
      RIL_PARAM_CHSCSDI_CHANNELCODING           = $00000008; // @paramdefine
      RIL_PARAM_CHSCSDI_ALL                     = $0000000f; // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILDATACOMPINFO
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_DCI_DIRECTION                   = $00000001; // @paramdefine
      RIL_PARAM_DCI_NEGOTIATION                 = $00000002; // @paramdefine
      RIL_PARAM_DCI_MAXDICTENTRIES              = $00000004; // @paramdefine
      RIL_PARAM_DCI_MAXSTRING                   = $00000008; // @paramdefine
      RIL_PARAM_DCI_ALL                         = $0000000f; // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILERRORCORRECTIONINFO
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_ECI_ORIGINALREQUEST             = $00000001; // @paramdefine
      RIL_PARAM_ECI_ORIGINALFALLBACK            = $00000002; // @paramdefine
      RIL_PARAM_ECI_ANSWERERFALLBACK            = $00000004; // @paramdefine
      RIL_PARAM_ECI_ALL                         = $00000007; // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILBEARERSVCINFO
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_BSI_SPEED                       = $00000001; // @paramdefine
      RIL_PARAM_BSI_SERVICENAME                 = $00000002; // @paramdefine
      RIL_PARAM_BSI_CONNECTIONELEMENT           = $00000004; // @paramdefine
      RIL_PARAM_BSI_ALL                         = $00000007; // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILRLPINFO
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_RLPI_IWS                        = $00000001; // @paramdefine
      RIL_PARAM_RLPI_MWS                        = $00000002; // @paramdefine
      RIL_PARAM_RLPI_ACKTIMER                   = $00000004; // @paramdefine
      RIL_PARAM_RLPI_RETRANSMISSIONATTEMPTS     = $00000008; // @paramdefine
      RIL_PARAM_RLPI_VERSION                    = $00000010; // @paramdefine
      RIL_PARAM_RPLI_RESEQUENCINGPERIOD         = $00000020; // @paramdefine
      RIL_PARAM_RPLI_ALL                        = $0000003f; // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILMSGSERVICEINFO
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_MSI_SERVICE                     = $00000001; // @paramdefine
      RIL_PARAM_MSI_MSGCLASSES                  = $00000002; // @paramdefine
      RIL_PARAM_MSI_READLOCATION                = $00000004; // @paramdefine
      RIL_PARAM_MSI_READUSED                    = $00000008; // @paramdefine
      RIL_PARAM_MSI_READTOTAL                   = $00000010; // @paramdefine
      RIL_PARAM_MSI_WRITELOCATION               = $00000020; // @paramdefine
      RIL_PARAM_MSI_WRITEUSED                   = $00000040; // @paramdefine
      RIL_PARAM_MSI_WRITETOTAL                  = $00000080; // @paramdefine
      RIL_PARAM_MSI_STORELOCATION               = $00000100; // @paramdefine
      RIL_PARAM_MSI_STOREUSED                   = $00000200; // @paramdefine
      RIL_PARAM_MSI_STORETOTAL                  = $00000400; // @paramdefine
      RIL_PARAM_MSI_ALL                         = $000007ff; // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILMSGDCS
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_MDCS_TYPE                       = $00000001; // @paramdefine
      RIL_PARAM_MDCS_FLAGS                      = $00000002; // @paramdefine
      RIL_PARAM_MDCS_MSGCLASS                   = $00000004; // @paramdefine
      RIL_PARAM_MDCS_ALPHABET                   = $00000008; // @paramdefine
      RIL_PARAM_MDCS_INDICATION                 = $00000010; // @paramdefine
      RIL_PARAM_MDCS_LANGUAGE                   = $00000020; // @paramdefine
      RIL_PARAM_MDCS_ALL                        = $0000003f; // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILMSGCONFIG
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_MC_SVCCTRADDRESS                = $00000001; // @paramdefine
      RIL_PARAM_MC_ALL                          = $00000001; // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILCBMSGCONFIG
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_CBMC_BROADCASTMSGIDS            = $00000001; // @paramdefine
      RIL_PARAM_CBMC_BROADCASTMSGLANGS          = $00000002; // @paramdefine
      RIL_PARAM_CBMC_ACCEPTIDS                  = $00000004; // @paramdefine
      RIL_PARAM_CBMC_ALL                        = $00000007; // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILMESSAGE
//
// @comm None
//
// -----------------------------------------------------------------------------
{ List of Unions Labeled
GSM
ID=RIL_MSGTYPE_IN_DELIVER
ISt=RIL_MSGTYPE_IN_STATUS
OS=RIL_MSGTYPE_OUT_SUBMIT
OC=RIL_MSGTYPE_OUT_COMMAND
OR=RIL_MSGTYPE_OUT_RAW
BC=RIL_MSGTYPE_BC_GENERAL

CDMA
ID=RIL_MSGTYPE_IN_IS637DELIVER
ISt=RIL_MSGTYPE_IN_IS637STATUS
OS=RIL_MSGTYPE_OUT_IS637SUBMIT
OSt=RIL_MSGTYPE_OUT_IS637STATUS
}
// -------This block is the GSM Params for RILMESSAGE (These values may have been recycled;
//        U = This value for the field has been reused in CDMA, and if the RILMESSAGE structure
//            is expanded, developer must careful not to use two recycled fields in the same union.)
const
      RIL_PARAM_M_SVCCTRADDRESS                 = $00000001; // @paramdefine GSM=[ID,ISt,OS,OC,OR,BC] CDMA=[ID,ISt,OS,OSt]
      RIL_PARAM_M_TYPE                          = $00000002; // @paramdefine GSM=[ID,ISt,OS,OC,OR,BC] CDMA=[ID,ISt,OS,OSt]
      RIL_PARAM_M_FLAGS                         = $00000004; // @paramdefine GSM=[ID,ISt,OS,OC,OR,BC] CDMA=[ID,ISt,OS,OSt]
      RIL_PARAM_M_ORIGADDRESS                   = $00000008; // @paramdefine GSM=[ID] CDMA=[ID,ISt]
      RIL_PARAM_M_TGTRECIPADDRESS               = $00000010; // @paramdefine U GSM=[ISt]
      RIL_PARAM_M_DESTADDRESS                   = $00000020; // @paramdefine GSM=[OS,OC] CDMA=[OSt,OS]
      RIL_PARAM_M_SCRECEIVETIME                 = $00000040; // @paramdefine GSM=[ID] CDMA=[ID,Ist]
      RIL_PARAM_M_TGTSCRECEIVETIME              = $00000080; // @paramdefine U GSM=[ISt]
      RIL_PARAM_M_TGTDISCHARGETIME              = $00000100; // @paramdefine U GSM=[ISt]
      RIL_PARAM_M_PROTOCOLID                    = $00000200; // @paramdefine U GSM=[ISt]
      RIL_PARAM_M_DATACODING                    = $00000800; // @paramdefine U GSM=[ID,ISt,OS,BC]
      RIL_PARAM_M_TGTDLVSTATUS                  = $00001000; // @paramdefine U GSM=[ISt]
      RIL_PARAM_M_TGTMSGREFERENCE               = $00002000; // @paramdefine U GSM=[OC]
      RIL_PARAM_M_VPFORMAT                      = $00004000; // @paramdefine U GSM=[OS]
      RIL_PARAM_M_VP                            = $00008000; // @paramdefine U GSM=[OS]
      RIL_PARAM_M_COMMANDTYPE                   = $00010000; // @paramdefine U GSM=[OC]
      RIL_PARAM_M_GEOSCOPE                      = $00020000; // @paramdefine U GSM=[BC]
      RIL_PARAM_M_MSGCODE                       = $00040000; // @paramdefine U GSM=[BC]
      RIL_PARAM_M_UPDATENUMBER                  = $00080000; // @paramdefine U GSM=[BC]
      RIL_PARAM_M_ID                            = $00100000; // @paramdefine U GSM=[BC]
      RIL_PARAM_M_TOTALPAGES                    = $00200000; // @paramdefine U GSM=[BC]
      RIL_PARAM_M_PAGENUMBER                    = $00400000; // @paramdefine U GSM=[BC]
      RIL_PARAM_M_HDRLENGTH                     = $00800000; // @paramdefine U GSM=[ID,ISt,OS]
      RIL_PARAM_M_MSGLENGTH                     = $01000000; // @paramdefine GSM=[ID,ISt,OS,OR,BC] CDMA=[ID,ISt,OS,OSt]
      RIL_PARAM_M_CMDLENGTH                     = $02000000; // @paramdefine GSM=[OC]
      RIL_PARAM_M_HDR                           = $04000000; // @paramdefine GSM=[ID,ISt,OS]
      RIL_PARAM_M_MSG                           = $08000000; // @paramdefine GSM=[ID,ISt,OS,OR,BC] CDMA=[ID,ISt,OS,OSt]
      RIL_PARAM_M_CMD                           = $10000000; // @paramdefine U GSM=[OC]

// CDMA Message parameter definitions
const
      RIL_PARAM_M_MSGID                         = $20000000; // @paramdefine CDMA=[ID,ISt,OS,OSt]

      RIL_PARAM_M_ORIGSUBADDRESS                = $40000000; // @paramdefine CDMA=[ID,ISt]
      RIL_PARAM_M_DESTSUBADDRESS                = $80000000; // @paramdefine CDMA=[OS,OSt]
      RIL_PARAM_M_DIGIT                         = $00010000; // @paramdefine CDMA=[OS,OSt]

      RIL_PARAM_M_PRIVACY                       = $00000100; // @paramdefine CDMA=[ID,OS]
      RIL_PARAM_M_PRIORITY                      = $00000200; // @paramdefine CDMA=[ID,OS]
      RIL_PARAM_M_TELESERVICE                   = $00000400; // @paramdefine CDMA=[ID,OS]
      RIL_PARAM_M_LANG                          = $00000800; // @paramdefine CDMA=[ID,ISt,OS,OSt]

      RIL_PARAM_M_VALIDITYPERIODABS             = $00001000; // @paramdefine CDMA=[ID,OS]
      RIL_PARAM_M_VALIDITYPERIODREL             = $00002000; // @paramdefine CDMA=[OS]
      RIL_PARAM_M_DEFERREDDELTIMEABS            = $00004000; // @paramdefine CDMA=[OS]
      RIL_PARAM_M_DEFERREDDELTIMEREL            = $00008000; // @paramdefine CDMA=[OS]

      RIL_PARAM_M_ENCODING                      = $00020000; // @paramdefine CDMA=[ID,ISt,OS,OSt]
      RIL_PARAM_M_USERRESPONSECODE              = $00040000; // @paramdefine CDMA=[ISt,OSt]
      RIL_PARAM_M_DISPLAYMODE                   = $00080000; // @paramdefine CDMA=[ID,OS]

      RIL_PARAM_M_CALLBACKNUM                   = $00000010; // @paramdefine CDMA=[ID,OS]
      RIL_PARAM_M_NUMMSGS                       = $00000080; // @paramdefine CDMA=[ID]

      RIL_PARAM_M_CAUSECODE                     = $00100000; // @paramdefine CDMA=[ISt]
      RIL_PARAM_M_REPLYSEQNUMBER                = $00200000; // @paramdefine CDMA=[ISt,OSt]

      RIL_PARAM_M_BEARERREPLYACK                = $00200000; // @paramdefine CDMA=[OS]
      RIL_PARAM_M_USERACK                       = $00400000; // @paramdefine CDMA=[ID,OS]
      RIL_PARAM_M_DELIVERYACK                   = $00800000; // @paramdefine CDMA=[OS]
      RIL_PARAM_M_MSGSTATUSTYPE                 = $10000000; // @paramdefine CDMA=[ISt]

      RIL_PARAM_M_ALL_IN_DELIVER                = RIL_PARAM_M_TYPE or
                                                  RIL_PARAM_M_FLAGS or
                                                  RIL_PARAM_M_ORIGADDRESS or
                                                  RIL_PARAM_M_PROTOCOLID or
                                                  RIL_PARAM_M_DATACODING or
                                                  RIL_PARAM_M_SCRECEIVETIME or
                                                  RIL_PARAM_M_HDRLENGTH or
                                                  RIL_PARAM_M_MSGLENGTH or
                                                  RIL_PARAM_M_HDR or
                                                  RIL_PARAM_M_MSG;            // @paramdefine

      RIL_PARAM_M_ALL_IN_STATUS                 = RIL_PARAM_M_TYPE or
                                                  RIL_PARAM_M_FLAGS or
                                                  RIL_PARAM_M_TGTMSGREFERENCE or
                                                  RIL_PARAM_M_TGTRECIPADDRESS or
                                                  RIL_PARAM_M_TGTSCRECEIVETIME or
                                                  RIL_PARAM_M_TGTDISCHARGETIME or
                                                  RIL_PARAM_M_TGTDLVSTATUS or
                                                  RIL_PARAM_M_PROTOCOLID or
                                                  RIL_PARAM_M_DATACODING or
                                                  RIL_PARAM_M_HDRLENGTH or
                                                  RIL_PARAM_M_MSGLENGTH or
                                                  RIL_PARAM_M_HDR or
                                                  RIL_PARAM_M_MSG;            // @paramdefine

      RIL_PARAM_M_ALL_OUT_SUBMIT                = RIL_PARAM_M_TYPE or
                                                  RIL_PARAM_M_FLAGS or
                                                  RIL_PARAM_M_DESTADDRESS or
                                                  RIL_PARAM_M_PROTOCOLID or
                                                  RIL_PARAM_M_DATACODING or
                                                  RIL_PARAM_M_VPFORMAT or
                                                  RIL_PARAM_M_VP or
                                                  RIL_PARAM_M_HDRLENGTH or
                                                  RIL_PARAM_M_MSGLENGTH or
                                                  RIL_PARAM_M_HDR or
                                                  RIL_PARAM_M_MSG;             // @paramdefine

      RIL_PARAM_M_ALL_OUT_COMMAND               = RIL_PARAM_M_TYPE or
                                                  RIL_PARAM_M_FLAGS or
                                                  RIL_PARAM_M_PROTOCOLID or
                                                  RIL_PARAM_M_COMMANDTYPE or
                                                  RIL_PARAM_M_TGTMSGREFERENCE or
                                                  RIL_PARAM_M_DESTADDRESS or
                                                  RIL_PARAM_M_CMDLENGTH or
                                                  RIL_PARAM_M_CMD;  // @paramdefine

      RIL_PARAM_M_ALL_BC_GENERAL                = RIL_PARAM_M_TYPE or
                                                  RIL_PARAM_M_GEOSCOPE or
                                                  RIL_PARAM_M_MSGCODE or
                                                  RIL_PARAM_M_UPDATENUMBER or
                                                  RIL_PARAM_M_ID or
                                                  RIL_PARAM_M_DATACODING or
                                                  RIL_PARAM_M_TOTALPAGES or
                                                  RIL_PARAM_M_PAGENUMBER or
                                                  RIL_PARAM_M_MSGLENGTH or
                                                  RIL_PARAM_M_MSG;             // @paramdefine

      RIL_PARAM_M_ALL_OUT_RAW                   = RIL_PARAM_M_TYPE or
                                                  RIL_PARAM_M_FLAGS or
                                                  RIL_PARAM_M_MSGLENGTH or
                                                  RIL_PARAM_M_MSG;             // @paramdefine

      RIL_PARAM_M_ALL_IN_IS637DELIVER           = RIL_PARAM_M_TYPE or
                                                  RIL_PARAM_M_MSGID or
                                                  RIL_PARAM_M_TELESERVICE or
                                                  RIL_PARAM_M_DISPLAYMODE or
                                                  RIL_PARAM_M_USERACK or
                                                  RIL_PARAM_M_ORIGADDRESS or
                                                  RIL_PARAM_M_ORIGSUBADDRESS or
                                                  RIL_PARAM_M_SCRECEIVETIME or
                                                  RIL_PARAM_M_PRIORITY or
                                                  RIL_PARAM_M_PRIVACY or
                                                  RIL_PARAM_M_CALLBACKNUM or
                                                  RIL_PARAM_M_NUMMSGS or
                                                  RIL_PARAM_M_VALIDITYPERIODABS or
                                                  RIL_PARAM_M_LANG or
                                                  RIL_PARAM_M_ENCODING or
                                                  RIL_PARAM_M_MSGLENGTH or
                                                  RIL_PARAM_M_MSG;             // @paramdefine

      RIL_PARAM_M_ALL_OUT_IS637SUBMIT           = RIL_PARAM_M_TYPE or
                                                  RIL_PARAM_M_MSGID or
                                                  RIL_PARAM_M_TELESERVICE or
                                                  RIL_PARAM_M_DISPLAYMODE or
                                                  RIL_PARAM_M_DESTADDRESS or
                                                  RIL_PARAM_M_DESTSUBADDRESS or
                                                  RIL_PARAM_M_DIGIT or
                                                  RIL_PARAM_M_BEARERREPLYACK or
                                                  RIL_PARAM_M_PRIORITY or
                                                  RIL_PARAM_M_PRIVACY or
                                                  RIL_PARAM_M_CALLBACKNUM or
                                                  RIL_PARAM_M_USERACK or
                                                  RIL_PARAM_M_DELIVERYACK or
                                                  RIL_PARAM_M_VALIDITYPERIODABS or
                                                  RIL_PARAM_M_VALIDITYPERIODREL or
                                                  RIL_PARAM_M_DEFERREDDELTIMEABS or
                                                  RIL_PARAM_M_DEFERREDDELTIMEREL or
                                                  RIL_PARAM_M_LANG or
                                                  RIL_PARAM_M_ENCODING or
                                                  RIL_PARAM_M_MSGLENGTH or
                                                  RIL_PARAM_M_MSG;              // @paramdefine

      RIL_PARAM_M_ALL_IN_IS637STATUS            = RIL_PARAM_M_TYPE or
                                                  RIL_PARAM_M_MSGID or
                                                  RIL_PARAM_M_ORIGADDRESS or
                                                  RIL_PARAM_M_ORIGSUBADDRESS or
                                                  RIL_PARAM_M_SCRECEIVETIME or
                                                  RIL_PARAM_M_CAUSECODE or
                                                  RIL_PARAM_M_REPLYSEQNUMBER or
                                                  RIL_PARAM_M_LANG or
                                                  RIL_PARAM_M_ENCODING or
                                                  RIL_PARAM_M_MSGLENGTH or
                                                  RIL_PARAM_M_MSG or
                                                  RIL_PARAM_M_USERRESPONSECODE or
                                                  RIL_PARAM_M_MSGSTATUSTYPE;    // @paramdefine

      RIL_PARAM_M_ALL_OUT_IS637STATUS           = RIL_PARAM_M_TYPE or
                                                  RIL_PARAM_M_MSGID or
                                                  RIL_PARAM_M_DESTADDRESS or
                                                  RIL_PARAM_M_DESTSUBADDRESS or
                                                  RIL_PARAM_M_REPLYSEQNUMBER or
                                                  RIL_PARAM_M_LANG or
                                                  RIL_PARAM_M_ENCODING or
                                                  RIL_PARAM_M_MSGLENGTH or
                                                  RIL_PARAM_M_MSG or
                                                  RIL_PARAM_M_USERRESPONSECODE or
                                                  RIL_PARAM_M_DIGIT;            // @paramdefine



// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILMESSAGE_IN_SIM
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_MIS_LOCATION                      = $00000001; // @paramdefine
      RIL_PARAM_MIS_INDEX                         = $00000002; // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILMESSAGEINFO
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_MI_INDEX                          = $00000001;  // @paramdefine
      RIL_PARAM_MI_STATUS                         = $00000002;  // @paramdefine
      RIL_PARAM_MI_MESSAGE                        = $00000004;  // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILEQUIPMENTINFO
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_EI_MANUFACTURER                   = $00000001;  // @paramdefine
      RIL_PARAM_EI_MODEL                          = $00000002;  // @paramdefine
      RIL_PARAM_EI_REVISION                       = $00000004;  // @paramdefine
      RIL_PARAM_EI_SERIALNUMBER                   = $00000008;  // @paramdefine
      RIL_PARAM_EI_ALL                            = $0000000f;  // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILPHONEBOOKINFO
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_PBI_STORELOCATION                 = $00000001;  // @paramdefine
      RIL_PARAM_PBI_USED                          = $00000002;  // @paramdefine
      RIL_PARAM_PBI_TOTAL                         = $00000004;  // @paramdefine
      RIL_PARAM_PBI_ALL                           = $00000007;  // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILPHONEBOOKENTRY
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_PBE_INDEX                         = $00000001;  // @paramdefine
      RIL_PARAM_PBE_ADDRESS                       = $00000002;  // @paramdefine
      RIL_PARAM_PBE_TEXT                          = $00000004;  // @paramdefine
      RIL_PARAM_PBE_ALL                           = $00000007;  // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILATRINFO
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_ATR_PHASE                         = $00000001;  // @paramdefine
      RIL_PARAM_ATR_SIZE                          = $00000002;  // @paramdefine
      RIL_PARAM_ATR_ATR                           = $00000004;  // @paramdefine
      RIL_PARAM_ATR_ALL                           = $00000007;  // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILSIMTOOLKITNOTIFYCAPS
//
// @comm Parameters for LPRILSIMTOOLKITNOTIFYCAPS -> dwParams
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_SIMTKN_REFRESH                    = $00000001; // @paramdefine
      RIL_PARAM_SIMTKN_MORETIME                   = $00000002; // @paramdefine
      RIL_PARAM_SIMTKN_POLLINTERVAL               = $00000004; // @paramdefine
      RIL_PARAM_SIMTKN_POLLINGOFF                 = $00000008; // @paramdefine
      RIL_PARAM_SIMTKN_SETUPCALL                  = $00000010; // @paramdefine
      RIL_PARAM_SIMTKN_SENDSS                     = $00000020; // @paramdefine
      RIL_PARAM_SIMTKN_SENDSMS                    = $00000040; // @paramdefine
      RIL_PARAM_SIMTKN_PLAYTONE                   = $00000080; // @paramdefine
      RIL_PARAM_SIMTKN_DISPLAYTEXT                = $00000100; // @paramdefine
      RIL_PARAM_SIMTKN_GETINKEY                   = $00000200; // @paramdefine
      RIL_PARAM_SIMTKN_GETINPUT                   = $00000400; // @paramdefine
      RIL_PARAM_SIMTKN_SELECTITEM                 = $00000800; // @paramdefine
      RIL_PARAM_SIMTKN_SETUPMENU                  = $00001000; // @paramdefine
      RIL_PARAM_SIMTKN_LOCALINFO                  = $00002000; // @paramdefine
      RIL_PARAM_SIMTKN_NOTIFYFLAGS                = $00004000; // @paramdefine
      RIL_PARAM_SIMTKN_SENDUSSD                   = $00008000; // @paramdefine
      RIL_PARAM_SIMTKN_SETUPIDLEMODETEXT          = $00010000; // @paramdefine
      RIL_PARAM_SIMTKN_SETUPEVENTLIST             = $00020000; // @paramdefine
      RIL_PARAM_SIMTKN_SENDDTMF                   = $00040000; // @paramdefine
      RIL_PARAM_SIMTKN_LAUNCHBROWSER              = $00080000; // @paramdefine
      RIL_PARAM_SIMTKN_OPENCHANNEL                = $00100000; // @paramdefine
      RIL_PARAM_SIMTKN_CLOSECHANNEL               = $00200000; // @paramdefine
      RIL_PARAM_SIMTKN_RECEIVEDATA                = $00400000; // @paramdefine
      RIL_PARAM_SIMTKN_SENDDATA                   = $00800000; // @paramdefine
      RIL_PARAM_SIMTKN_TIMERMANAGEMENT            = $01000000; // @paramdefine
      RIL_PARAM_SIMTKN_EVENTS                     = $02000000; // @paramdefine
      RIL_PARAM_SIMTKN_RUNATCMD                   = $04000000; // @paramdefine
      RIL_PARAM_SIMTKN_ALL                        = $07ffffff; // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILSIMTOOLKITEVENTCAPS
//
// @comm Parameters for LPRILSIMTOOLKITEVENTCAPS -> dwParams
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_SIMTKE_MTCALL                     = $00000001; // @paramdefine
      RIL_PARAM_SIMTKE_CALLCONNECTED              = $00000002; // @paramdefine
      RIL_PARAM_SIMTKE_CALLDISCONNECTED           = $00000004; // @paramdefine
      RIL_PARAM_SIMTKE_LOCATIONSTATUS             = $00000008; // @paramdefine
      RIL_PARAM_SIMTKE_USERACTIVITY               = $00000010; // @paramdefine
      RIL_PARAM_SIMTKE_IDLESCREEN                 = $00000020; // @paramdefine
      RIL_PARAM_SIMTKE_LANGUAGESELECTION          = $00000040; // @paramdefine
      RIL_PARAM_SIMTKE_BROWSERTERMINATION         = $00000080; // @paramdefine
      RIL_PARAM_SIMTKE_DATAAVAILABLE              = $00000100; // @paramdefine
      RIL_PARAM_SIMTKE_CHANNELSTATUS              = $00000200; // @paramdefine
      RIL_PARAM_SIMTKE_DISPLAYCHANGE              = $00000400; // @paramdefine
      RIL_PARAM_SIMTKE_ALL                        = $000007FF; // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILSIMTOOLKITCMD
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_SIMTKIT_CMD_ID                    = $00000001; // @paramdefine
      RIL_PARAM_SIMTKIT_CMD_TAG                   = $00000002; // @paramdefine
      RIL_PARAM_SIMTKIT_CMD_TYPE                  = $00000004; // @paramdefine
      RIL_PARAM_SIMTKIT_CMD_QUALIFIER             = $00000008; // @paramdefine
      RIL_PARAM_SIMTKIT_CMD_ERROR                 = $00000010; // @paramdefine
      RIL_PARAM_SIMTKIT_CMD_DETAILS_OFFSET        = $00000020; // @paramdefine
      RIL_PARAM_SIMTKIT_CMD_DETAILS_SIZE          = $00000040; // @paramdefine
      RIL_PARAM_SIMTKIT_CMD_ALL                   = $0000007F; // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILSIMTOOLKITRSP
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_SIMTKIT_RSP_ID                    = $00000001; // @paramdefine
      RIL_PARAM_SIMTKIT_RSP_TAG                   = $00000002; // @paramdefine
      RIL_PARAM_SIMTKIT_RSP_TYPE                  = $00000004; // @paramdefine
      RIL_PARAM_SIMTKIT_RSP_QUALIFIER             = $00000008; // @paramdefine
      RIL_PARAM_SIMTKIT_RSP_RESPONSE              = $00000010; // @paramdefine
      RIL_PARAM_SIMTKIT_RSP_ADDITIONALINFO        = $00000020; // @paramdefine
      RIL_PARAM_SIMTKIT_RSP_ALL                   = $0000003F; // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILSIMCMDPARAMETERS
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_SCP_FILEID                        = $00000001; // @paramdefine
      RIL_PARAM_SCP_PARAM1                        = $00000002; // @paramdefine
      RIL_PARAM_SCP_PARAM2                        = $00000004; // @paramdefine
      RIL_PARAM_SCP_PARAM3                        = $00000008; // @paramdefine
      RIL_PARAM_SCP_ALL                           = $0000000f; // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILSIMRESPONSE
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_SR_STATUSWORD1                    = $00000001; // @paramdefine
      RIL_PARAM_SR_STATUSWORD2                    = $00000002; // @paramdefine
      RIL_PARAM_SR_RESPONSE                       = $00000004; // @paramdefine
      RIL_PARAM_SR_ALL                            = $00000007; // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILSIMRECORDSTATUS
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_SRS_RECORDTYPE                    = $00000001;     // @paramdefine
      RIL_PARAM_SRS_ITEMCOUNT                     = $00000002;     // @paramdefine
      RIL_PARAM_SRS_SIZE                          = $00000004;     // @paramdefine
      RIL_PARAM_SRS_ALL                           = $00000007;     // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILCOSTINFO
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_CSTI_CCM                          = $00000001; // @paramdefine
      RIL_PARAM_CSTI_ACM                          = $00000002; // @paramdefine
      RIL_PARAM_CSTI_MAXACM                       = $00000004; // @paramdefine
      RIL_PARAM_CSTI_COSTPERUNIT                  = $00000008; // @paramdefine
      RIL_PARAM_CSTI_CURRENCY                     = $00000010; // @paramdefine
      RIL_PARAM_CSTI_ALL                          = $0000001f; // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILSIGNALQUALITY
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_SQ_SIGNALSTRENGTH                 = $00000001; // @paramdefine
      RIL_PARAM_SQ_MINSIGNALSTRENGTH              = $00000002; // @paramdefine
      RIL_PARAM_SQ_MAXSIGNALSTRENGTH              = $00000004; // @paramdefine
      RIL_PARAM_SQ_BITERRORRATE                   = $00000008; // @paramdefine
      RIL_PARAM_SQ_LOWSIGNALSTRENGTH              = $00000010; // @paramdefine
      RIL_PARAM_SQ_HIGHSIGNALSTRENGTH             = $00000020; // @paramdefine
      RIL_PARAM_SQ_ALL                            = $0000003f; // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILCELLTOWERINFO
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_CTI_MOBILECOUNTRYCODE             = $00000001; // @paramdefine
      RIL_PARAM_CTI_MOBILENETWORKCODE             = $00000002; // @paramdefine
      RIL_PARAM_CTI_LOCATIONAREACODE              = $00000004; // @paramdefine
      RIL_PARAM_CTI_CELLID                        = $00000008; // @paramdefine
      RIL_PARAM_CTI_BASESTATIONID                 = $00000010; // @paramdefine
      RIL_PARAM_CTI_BROADCASTCONTROLCHANNEL       = $00000020; // @paramdefine
      RIL_PARAM_CTI_RXLEVEL                       = $00000040; // @paramdefine
      RIL_PARAM_CTI_RXLEVELFULL                   = $00000080; // @paramdefine
      RIL_PARAM_CTI_RXLEVELSUB                    = $00000100; // @paramdefine
      RIL_PARAM_CTI_RXQUALITY                     = $00000200; // @paramdefine
      RIL_PARAM_CTI_RXQUALITYFULL                 = $00000400; // @paramdefine
      RIL_PARAM_CTI_RXQUALITYSUB                  = $00000800; // @paramdefine
      RIL_PARAM_CTI_IDLETIMESLOT                  = $00001000; // @paramdefine
      RIL_PARAM_CTI_TIMINGADVANCE                 = $00002000; // @paramdefine
      RIL_PARAM_CTI_GPRSCELLID                    = $00004000; // @paramdefine
      RIL_PARAM_CTI_GPRSBASESTATIONID             = $00008000; // @paramdefine
      RIL_PARAM_CTI_NUMBCCH                       = $00010000; // @paramdefine
      RIL_PARAM_CTI_NMR                           = $00020000; // @paramdefine
      RIL_PARAM_CTI_BCCH                          = $00040000; // @paramdefine
      RIL_PARAM_CTI_ALL                           = $0007ffff; // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILEQUIPMENTSTATE
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_EQUIPMENTSTATE_RADIOSUPPORT        = $00000001;  // @paramdefine
      RIL_PARAM_EQUIPMENTSTATE_EQSTATE             = $00000002;  // @paramdefine
      RIL_PARAM_EQUIPMENTSTATE_READYSTATE          = $00000004;  // @paramdefine
      RIL_PARAM_EQUIPMENTSTATE_ALL                 = $00000007;  // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILREMOTEPARTYINFO
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_RPI_ADDRESS                       = $00000001;  // @paramdefine
      RIL_PARAM_RPI_SUBADDRESS                    = $00000002;  // @paramdefine
      RIL_PARAM_RPI_DESCRIPTION                   = $00000004;  // @paramdefine
      RIL_PARAM_RPI_VALIDITY                      = $00000008;  // @paramdefine
      RIL_PARAM_RPI_ALL                           = $0000000f;  // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILCALLWAITINGINFO
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_CWI_CALLTYPE                      = $00000001;  // @paramdefine
      RIL_PARAM_CWI_CALLERINFO                    = $00000002;  // @paramdefine
      RIL_PARAM_CWI_ADDRESSID                     = $00000004;  // @paramdefine
      RIL_PARAM_CWI_ALL                           = $00000007;  // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILINTERMEDIATESSINFO
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_INTSS_NOTIFICATIONCODE            = $00000001; // @paramdefine
      RIL_PARAM_INTSS_CUGINDEX                    = $00000002; // @paramdefine
      RIL_PARAM_INTSS_ALL                         = $00000003; // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILUNSOLICITEDSSINFO
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_UNSSS_NOTIFICATIONCODE            = $00000001; // @paramdefine
      RIL_PARAM_UNSSS_CUGINDEX                    = $00000002; // @paramdefine
      RIL_PARAM_UNSSS_ADDRESS                     = $00000004; // @paramdefine
      RIL_PARAM_UNSSS_SUBADDR                     = $00000008; // @paramdefine
      RIL_PARAM_UNSSS_ALL                         = $0000000f; // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILRINGINFO
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_RI_CALLTYPE                       = $00000001;  // @paramdefine
      RIL_PARAM_RI_SERVICEINFO                    = $00000002;  // @paramdefine
      RIL_PARAM_RI_ADDRESSID                      = $00000004;  // @paramdefine
      RIL_PARAM_RI_ALL                            = $00000007;  // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILDIALINFO
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_DI_CMDID                          = $00000001;  // @paramdefine
      RIL_PARAM_DI_CALLID                         = $00000002;  // @paramdefine
      RIL_PARAM_DI_ALL                            = $00000003;  // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILCONNECTINFO
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_CNI_CALLTYPE                      = $00000001;  // @paramdefine
      RIL_PARAM_CNI_BAUDRATE                      = $00000002;  // @paramdefine
      RIL_PARAM_CNI_ALL                           = $00000003;  // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILSERVICEINFO
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_SVCI_SYNCHRONOUS                  = $00000001;  // @paramdefine
      RIL_PARAM_SVCI_TRANSPARENT                  = $00000002;  // @paramdefine
      RIL_PARAM_SVCI_ALL                          = $00000003;  // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILMSGSTORAGEINFO
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_MSTI_READLOCATION                 = $00000001;  // @paramdefine
      RIL_PARAM_MSTI_WRITELOCATION                = $00000002;  // @paramdefine
      RIL_PARAM_MSTI_STORELOCATION                = $00000004;  // @paramdefine
      RIL_PARAM_MSTI_ALL                          = $00000007;  // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILSUPSERVICEDATA
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_SSDI_STATUS                       = $00000001;  // @paramdefine
      RIL_PARAM_SSDI_DATA                         = $00000002;  // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILCAPSDIAL
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_CD_CALLTYPES                      = $00000001;  // @paramdefine
      RIL_PARAM_CD_OPTIONS                        = $00000002;  // @paramdefine
      RIL_PARAM_CD_ALL                            = $00000003;  // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILCAPSBEARERSVC
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_CBS_SPEEDS1                       = $00000001;  // @paramdefine
      RIL_PARAM_CBS_SPEEDS2                       = $00000002;  // @paramdefine
      RIL_PARAM_CBS_SERVICENAMES                  = $00000004;  // @paramdefine
      RIL_PARAM_CBS_CONNECTIONELEMENTS            = $00000008;  // @paramdefine
      RIL_PARAM_CBS_ALL                           = $0000000f;  // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILCAPSRLP
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_CRLP_VERSION                      = $00000001;  // @paramdefine
      RIL_PARAM_CRLP_IWSRANGE                     = $00000002;  // @paramdefine
      RIL_PARAM_CRLP_MWSRANGE                     = $00000004;  // @paramdefine
      RIL_PARAM_CRLP_ACKTIMERRANGE                = $00000008;  // @paramdefine
      RIL_PARAM_CRLP_RETRANSMISSIONATTSRANGE      = $00000010;  // @paramdefine
      RIL_PARAM_CRLP_RESEQPERIODRANGE             = $00000020;  // @paramdefine
      RIL_PARAM_CRLP_ALL                          = $0000003f;  // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILCAPSMSGMEMORYLOCATIONS
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_CMML_READLOCATIONS                = $00000001;  // @paramdefine
      RIL_PARAM_CMML_WRITELOCATIONS               = $00000002;  // @paramdefine
      RIL_PARAM_CMML_STORELOCATIONS               = $00000004;  // @paramdefine
      RIL_PARAM_CMML_ALL                          = $00000007;  // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILCAPSLOCKINGPWDLENGTH
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_CLPL_FACILITY                     = $00000001;  // @paramdefine
      RIL_PARAM_CLPL_PASSWORDLENGTH               = $00000002;  // @paramdefine
      RIL_PARAM_CLPL_ALL                          = $00000003;  // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILCAPSBARRINGPWDLENGTH
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_CBPL_TYPE                         = $00000001;  // @paramdefine
      RIL_PARAM_CBPL_PASSWORDLENGTH               = $00000002;  // @paramdefine
      RIL_PARAM_CBPL_ALL                          = $00000003;  // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILCAPSDATACOMPRESSION
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_CDC_DIRECTION                     = $00000001;  // @paramdefine
      RIL_PARAM_CDC_NEGOTIATION                   = $00000002;  // @paramdefine
      RIL_PARAM_CDC_MAXDICT                       = $00000004;  // @paramdefine
      RIL_PARAM_CDC_MAXSTRING                     = $00000008;  // @paramdefine
      RIL_PARAM_CDC_ALL                           = $0000000f;  // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILCAPSERRORCORRECTION
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_CEC_ORIGINALREQUEST               = $00000001;  // @paramdefine
      RIL_PARAM_CEC_ORIGINALFALLBACK              = $00000002;  // @paramdefine
      RIL_PARAM_CEC_ANSWERERFALLBACK              = $00000004;  // @paramdefine
      RIL_PARAM_CEC_ALL                           = $00000007;  // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILCAPSHSCSD
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_CHSCSD_MULTISLOTCLASS             = $00000001;  // @paramdefine
      RIL_PARAM_CHSCSD_MAXRXTIMESLOTS             = $00000002;  // @paramdefine
      RIL_PARAM_CHSCSD_MAXTXTIMESLOTS             = $00000004;  // @paramdefine
      RIL_PARAM_CHSCSD_MAXTOTALTIMESLOTS          = $00000008;  // @paramdefine
      RIL_PARAM_CHSCSD_CHANNELCODINGS             = $00000010;  // @paramdefine
      RIL_PARAM_CHSCSD_AIRINTERFACEUSERRATES      = $00000020;  // @paramdefine
      RIL_PARAM_CHSCSD_TOPRXTIMESLOTRANGE         = $00000040;  // @paramdefine
      RIL_PARAM_CHSCSD_ALL                        = $0000007f;  // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILCAPSPBENTRYLENGTH
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_CPBEL_MAXADDRESSLENGTH            = $00000001;  // @paramdefine
      RIL_PARAM_CPBEL_MAXTEXTLENGTH               = $00000002;  // @paramdefine
      RIL_PARAM_CPBEL_ALL                         = $00000003;  // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILGPRSCONTEXT
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_GCONT_CONTEXTID                   = $00000001;  // @paramdefine
      RIL_PARAM_GCONT_PROTOCOLTYPE                = $00000002;  // @paramdefine
      RIL_PARAM_GCONT_ACCESSPOINTNAME             = $00000004;  // @paramdefine
      RIL_PARAM_GCONT_ADDRESS                     = $00000008;  // @paramdefine
      RIL_PARAM_GCONT_DATACOMPRESSION             = $00000010;  // @paramdefine
      RIL_PARAM_GCONT_HEADERCOMPRESSION           = $00000020;  // @paramdefine
      RIL_PARAM_GCONT_PARAMETERLENGTH             = $00000040;  // @paramdefine
      RIL_PARAM_GCONT_PARAMETERS                  = $00000080;  // @paramdefine
      RIL_PARAM_GCONT_ALL                         = $000000ff;  // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILGPRSQOSPROFILE
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_GQOSP_CONTEXTID                   = $00000001;  // @paramdefine
      RIL_PARAM_GQOSP_PRECEDENCECLASS             = $00000002;  // @paramdefine
      RIL_PARAM_GQOSP_DELAYCLASS                  = $00000004;  // @paramdefine
      RIL_PARAM_GQOSP_RELIABILITYCLASS            = $00000008;  // @paramdefine
      RIL_PARAM_GQOSP_PEAKTHRUCLASS               = $00000010;  // @paramdefine
      RIL_PARAM_GQOSP_MEANTHRUCLASS               = $00000020;  // @paramdefine
      RIL_PARAM_GQOSP_ALL                         = $0000003F;  // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILNITZINFO
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_NITZ_SYSTEMTIME                    = $00000001; // @paramdefine
      RIL_PARAM_NITZ_TIMEZONEOFFSET                = $00000002; // @paramdefine
      RIL_PARAM_NITZ_DAYLIGHTSAVINGOFFSET          = $00000004; // @paramdefine

//
// Other constants
//

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants RILNDISSTATUS
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_NDIS_XON  = BOOL(true);
      RIL_NDIS_XOFF = BOOL(false);

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants RILGPRSCONTEXTACTIVATED
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_RILGPRSCONTEXTACTIVATED_NWDEACT           = $00000001;  // @constdefine
      RIL_RILGPRSCONTEXTACTIVATED_NWDETACH          = $00000002;  // @constdefine
      RIL_RILGPRSCONTEXTACTIVATED_MEDEACT           = $00000003;  // @constdefine
      RIL_RILGPRSCONTEXTACTIVATED_MEDETACH          = $00000004;  // @constdefine
      RIL_RILGPRSCONTEXTACTIVATED_MEACT             = $00000005;  // @constdefine
      RIL_RILGPRSCONTEXTACTIVATED_RADIOOFF          = $00000006;  // @constdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Address Type | Different phone number representations
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_ADDRTYPE_UNKNOWN                        = $00000000;  // @constdefine Unknown type
      RIL_ADDRTYPE_INTERNATIONAL                  = $00000001;  // @constdefine International number
      RIL_ADDRTYPE_NATIONAL                       = $00000002;  // @constdefine National number
      RIL_ADDRTYPE_NETWKSPECIFIC                  = $00000003;  // @constdefine Network specific number
      RIL_ADDRTYPE_SUBSCRIBER                     = $00000004;  // @constdefine Subscriber number (protocol-specific)
      RIL_ADDRTYPE_ALPHANUM                       = $00000005;  // @constdefine Alphanumeric address
      RIL_ADDRTYPE_ABBREV                         = $00000006;  // @constdefine Abbreviated number
// additional CDMA ADDRTYPE definitions
// See IS-2000.5-A-1 page 509 table 2.7.1.3.2.4-2
      RIL_ADDRTYPE_IP                             = $00000007;      // @constdefine IP Address (RFC 791)
      RIL_ADDRTYPE_EMAIL                          = $00000008;      // @constdefine Internet Email addresss (RFC 822)

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Numbering Plan | Different numbering shcemes
//
// @comm Used for <def RIL_ADDRTYPE_UNKNOWN>, <def RIL_ADDRTYPE_INTERNATIONAL>,
//       and <def RIL_ADDRTYPE_NATIONAL>
//
// -----------------------------------------------------------------------------
const
      RIL_NUMPLAN_UNKNOWN                         = $00000000;  // @constdefine Unknown numbering plan
      RIL_NUMPLAN_TELEPHONE                       = $00000001;  // @constdefine ISDN/telephone numbering plan (E.164/E.163)
      RIL_NUMPLAN_DATA                            = $00000002;  // @constdefine Data numbering plan (X.121)
      RIL_NUMPLAN_TELEX                           = $00000003;  // @constdefine Telex numbering plan
      RIL_NUMPLAN_NATIONAL                        = $00000004;  // @constdefine National numbering plan
      RIL_NUMPLAN_PRIVATE                         = $00000005;  // @constdefine Private numbering plan
      RIL_NUMPLAN_ERMES                           = $00000006;  // @constdefine ERMES numbering plan (ETSI DE/PS 3 01-3)

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Subaddress Type | Different subaddress types
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_SUBADDRTYPE_NSAP                        = $00000001;  // @constdefine NSAP subaddress (CCITT Recommendation X.213 or ISO 8348 AD2)
      RIL_SUBADDRTYPE_USER                        = $00000002;  // @constdefine User defined subaddress

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Data Rate | Defines different protocol dependant data rates
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_SPEED_UNKNOWN                           = $00000000;  // @constdefine Unknown speed
      RIL_SPEED_AUTO                              = $00000001;  // @constdefine Automatic selection of speed
      RIL_SPEED_300_V21                           = $00000002;  // @constdefine 300 bps (V.21)
      RIL_SPEED_300_V110                          = $00000003;  // @constdefine 300 bps (V.100)
      RIL_SPEED_1200_V22                          = $00000004;  // @constdefine 1200 bps (V.22)
      RIL_SPEED_1200_75_V23                       = $00000005;  // @constdefine 1200/75 bps (V.23)
      RIL_SPEED_1200_V110                         = $00000006;  // @constdefine 1200 bps (V.100)
      RIL_SPEED_1200_V120                         = $00000007;  // @constdefine 1200 bps (V.120)
      RIL_SPEED_2400_V22BIS                       = $00000008;  // @constdefine 2400 bps (V.22bis)
      RIL_SPEED_2400_V26TER                       = $00000009;  // @constdefine 2400 bps (V.26ter)
      RIL_SPEED_2400_V110                         = $0000000a;  // @constdefine 2400 bps (V.110 or X.31 flag stuffing)
      RIL_SPEED_2400_V120                         = $0000000b;  // @constdefine 2400 bps (V.120)
      RIL_SPEED_4800_V32                          = $0000000c;  // @constdefine 4800 bps (V.32)
      RIL_SPEED_4800_V110                         = $0000000d;  // @constdefine 4800 bps (V.110 or X.31 flag stuffing)
      RIL_SPEED_4800_V120                         = $0000000e;  // @constdefine 4800 bps (V.120)
      RIL_SPEED_9600_V32                          = $0000000f;  // @constdefine 9600 bps (V.32)
      RIL_SPEED_9600_V34                          = $00000010;  // @constdefine 9600 bps (V.34)
      RIL_SPEED_9600_V110                         = $00000011;  // @constdefine 9600 bps (V.110 or X.31 flag stuffing)
      RIL_SPEED_9600_V120                         = $00000012;  // @constdefine 9600 bps (V.120)
      RIL_SPEED_14400_V34                         = $00000013;  // @constdefine 14400 bps (V.34)
      RIL_SPEED_14400_V110                        = $00000014;  // @constdefine 14400 bps (V.100 or X.31 flag stuffing)
      RIL_SPEED_14400_V120                        = $00000015;  // @constdefine 14400 bps (V.120)
      RIL_SPEED_19200_V34                         = $00000016;  // @constdefine 19200 bps (V.34)
      RIL_SPEED_19200_V110                        = $00000017;  // @constdefine 19200 bps (V.110 or X.31 flag stuffing)
      RIL_SPEED_19200_V120                        = $00000018;  // @constdefine 19200 bps (V.120)
      RIL_SPEED_28800_V34                         = $00000019;  // @constdefine 28800 bps (V.34)
      RIL_SPEED_28800_V110                        = $0000001a;  // @constdefine 28800 bps (V.110 or X.31 flag stuffing)
      RIL_SPEED_28800_V120                        = $0000001b;  // @constdefine 28800 bps (V.120)
      RIL_SPEED_38400_V110                        = $0000001c;  // @constdefine 38400 bps (V.110 or X.31 flag stuffing)
      RIL_SPEED_38400_V120                        = $0000001d;  // @constdefine 38400 bps (V.120)
      RIL_SPEED_48000_V110                        = $0000001e;  // @constdefine 48000 bps (V.110 or X.31 flag stuffing)
      RIL_SPEED_48000_V120                        = $0000001f;  // @constdefine 48000 bps (V.120)
      RIL_SPEED_56000_V110                        = $00000020;  // @constdefine 56000 bps (V.110 or X.31 flag stuffing)
      RIL_SPEED_56000_V120                        = $00000021;  // @constdefine 56000 bps (V.120)
      RIL_SPEED_56000_TRANSP                      = $00000022;  // @constdefine 56000 bps (bit transparent)
      RIL_SPEED_64000_TRANSP                      = $00000023;  // @constdefine 64000 bps (bit transparent)
      RIL_SPEED_32000_PIAFS32K                    = $00000024;      // @constdefine 32000 bps (PIAFS32k;
      RIL_SPEED_64000_PIAFS64K                    = $00000025;      // @constdefine 64000 bps (PIAFS64k;
      RIL_SPEED_28800_MULTIMEDIA                  = $00000026;      // @constdefine 28800 bps (MultiMedia;
      RIL_SPEED_32000_MULTIMEDIA                  = $00000027;      // @constdefine 32000 bps (MultiMedia;
      RIL_SPEED_33600_MULTIMEDIA                  = $00000028;      // @constdefine 33600 bps (MultiMedia;
      RIL_SPEED_56000_MULTIMEDIA                  = $00000029;      // @constdefine 56000 bps (MultiMedia;
      RIL_SPEED_64000_MULTIMEDIA                  = $0000002a;      // @constdefine 64000 bps (MultiMedia;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Telephony Service | Telephony service types
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_SERVICE_UNKNOWN                         = $00000000;  // @constdefine Unknown service
      RIL_SERVICE_MODEM_ASYNC                     = $00000001;  // @constdefine Asynchronous modem
      RIL_SERVICE_MODEM_SYNC                      = $00000002;  // @constdefine Synchronous modem
      RIL_SERVICE_PADACCESS_ASYNC                 = $00000003;  // @constdefine PAD Access (asynchronous)
      RIL_SERVICE_PACKETACCESS_SYNC               = $00000004;  // @constdefine Packet Access (synchronous)
      RIL_SERVICE_VOICE                           = $00000005;  // @constdefine Voice
      RIL_SERVICE_FAX                             = $00000006;  // @constdefine Fax

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants ITC | Information trasnfer capability types
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_ITC_31KHZ                               = $00000001;  // @constdefine 3.1 kHz
      RIL_ITC_UDI                                 = $00000002;  // @constdefine Unrestricted Digital Information

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Operator Name | Operator name formats
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_OPFORMAT_LONG                           = $00000001;  // @constdefine Long alphanumeric name
      RIL_OPFORMAT_SHORT                          = $00000002;  // @constdefine Short alphanumeric name
      RIL_OPFORMAT_NUM                            = $00000003;  // @constdefine Numeric name

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Operator Status | Operator status values
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_OPSTATUS_UNKNOWN                        = $00000000;  // @constdefine Unknown status
      RIL_OPSTATUS_AVAILABLE                      = $00000001;  // @constdefine Operator is available
      RIL_OPSTATUS_CURRENT                        = $00000002;  // @constdefine Operator is current
      RIL_OPSTATUS_FORBIDDEN                      = $00000003;  // @constdefine Operator is forbidden

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Operator Selection | Operator selection modes
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_OPSELMODE_AUTOMATIC                     = $00000001;  // @constdefine Automatic operator selection
      RIL_OPSELMODE_MANUAL                        = $00000002;  // @constdefine Manual operator selection
      RIL_OPSELMODE_MANUALAUTOMATIC               = $00000003;  // @constdefine Manual/automatic operator selection
                                                                // (if manual selection fails, automatic selection mode is entered)

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Operator Special | Special preferred operator index value
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PREFOPINDEX_FIRSTAVAILABLE              = $ffffffff;  // @constdefine Used to specify that a preferred operator is
                                                                // to be stored at the first available index

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Information Class | Telephony information classes
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_INFOCLASS_NONE                          = $00000000;  // @constdefine None
      RIL_INFOCLASS_VOICE                         = $00000001;  // @constdefine Voice
      RIL_INFOCLASS_DATA                          = $00000002;  // @constdefine Data
      RIL_INFOCLASS_FAX                           = $00000004;  // @constdefine Fax
      RIL_INFOCLASS_SMS                           = $00000008;  // @constdefine SMS
      RIL_INFOCLASS_DATACIRCUITSYNC               = $00000010;  // @constdefine Data Circuit synchronous
      RIL_INFOCLASS_DATACIRCUITASYNC              = $00000020;  // @constdefine Data Circuit asynchronous
      RIL_INFOCLASS_PACKETACCESS                  = $00000040;  // @constdefine Dedicated Packet Access
      RIL_INFOCLASS_PADACCESS                     = $00000080;  // @constdefine Dedicated PAD Access
      RIL_INFOCLASS_ALL                           = $000000ff;  // @constdefine All information classes

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Supplemental Activation | Supplementary service status values
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_SVCSTAT_UNKNOWN                         = $00000000;  // @constdefine Unknown status
      RIL_SVCSTAT_DISABLED                        = $00000001;  // @constdefine Service is disabled
      RIL_SVCSTAT_ENABLED                         = $00000002;  // @constdefine Service is enabled
      RIL_SVCSTAT_DEFAULT                         = $00000003;  // @constdefine Default status

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Supplementary Service Provisioning | Supplementary service provisioning values
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_SVCPROV_UNKNOWN                         = $00000000;  // @constdefine Unknown provisioning
      RIL_SVCPROV_NOTPROVISIONED                  = $00000001;  // @constdefine Service isn't provisioned
      RIL_SVCPROV_PROVISIONED                     = $00000002;  // @constdefine Service is provisioned
      RIL_SVCPROV_TEMPMODERESTRICTED              = $00000003;  // @constdefine Service temporary mode is restricted
      RIL_SVCPROV_TEMPMODEALLOWED                 = $00000004;  // @constdefine Service temporary mode is allowed

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants CUG Special | Closed User Group special index value
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_CUGINDEX_NONE                           = $ffffffff;  // @constdefine Used to identify the absence of CUG index

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants CUG Info Level | Closed User Group information levels
//
// @comm This feature is not used and is untested.
//
// -----------------------------------------------------------------------------
const
      RIL_CUGINFO_NONE                            = $00000000;  // @constdefine TBD
      RIL_CUGINFO_SUPPRESSOA                      = $00000001;  // @constdefine TBD
      RIL_CUGINFO_SUPRESSPREF                     = $00000002;  // @constdefine TBD
      RIL_CUGINFO_SUPPRESSOAANDPREF               = $00000003;  // @constdefine TBD

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Forwarding Reason | Forwarding reasons
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_FWDREASON_UNCONDITIONAL                 = $00000001;  // @constdefine Always forward
      RIL_FWDREASON_MOBILEBUSY                    = $00000002;  // @constdefine Forward when device busy
      RIL_FWDREASON_NOREPLY                       = $00000003;  // @constdefine Forward when no answer
      RIL_FWDREASON_UNREACHABLE                   = $00000004;  // @constdefine Forward device out of service
      RIL_FWDREASON_ALLFORWARDING                 = $00000005;  // @constdefine TBD
      RIL_FWDREASON_ALLCONDITIONAL                = $00000006;  // @constdefine TBD

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Call Type | Call types
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_CALLTYPE_UNKNOWN                        = $00000000;  // @constdefine Unknown
      RIL_CALLTYPE_VOICE                          = $00000001;  // @constdefine Voice call
      RIL_CALLTYPE_DATA                           = $00000002;  // @constdefine Data call
      RIL_CALLTYPE_FAX                            = $00000003;  // @constdefine Fax call
      RIL_CALLTYPE_PTT                            = $00000004;  // @constdefine Push-To-Talk call
      RIL_CALLTYPE_VT                             = $00000005;  // @constdefine Video Telephony call
      RIL_CALLTYPE_LAST                           = RIL_CALLTYPE_VT; // @constdefine last valid call

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Dialing Option | Dialing options
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_DIALOPT_NONE                            = $00000000;  // @constdefine No options
      RIL_DIALOPT_RESTRICTID                      = $00000001;  // @constdefine Restrict CLI presentation
      RIL_DIALOPT_PRESENTID                       = $00000002;  // @constdefine Allow CLI presentation
      RIL_DIALOPT_CLOSEDGROUP                     = $00000004;  // @constdefine Closed User Group dialing
      RIL_DIALOPT_ALL                             = $00000007;  // @constdefine All options

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Call Option | Call options defaults
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_DIALTONEWAIT_DEFAULT                    = $00000000;  // @constdefine TBD
      RIL_DIALTIMEOUT_DEFAULT                     = $00000000;  // @constdefine TBD
      RIL_COMMAPAUSE_DEFAULT                      = $00000000;  // @constdefine TBD
      RIL_DISCONNECTTIMEOUT_DEFAULT               = $00000000;  // @constdefine TBD

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants DTMF Duration | DTMF tone duration default
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_DTMFDURATION_DEFAULT                    = $00000000;  // @constdefine TBD

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Number of Calls to Track | Number of Calls to Track
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_MAX_TRACKED_CALL_ID             = 10;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Call Direction | Call direction
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_CALLDIR_INCOMING                        = $00000001;  // @constdefine Incoming call
      RIL_CALLDIR_OUTGOING                        = $00000002;  // @constdefine Outgoing call

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Call Status | Call status values
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_CALLSTAT_ACTIVE                         = $00000001;  // @constdefine Active call
      RIL_CALLSTAT_ONHOLD                         = $00000002;  // @constdefine Call on hold
      RIL_CALLSTAT_DIALING                        = $00000003;  // @constdefine In the process of dialing
      RIL_CALLSTAT_ALERTING                       = $00000004;  // @constdefine In the process of ringing
      RIL_CALLSTAT_INCOMING                       = $00000005;  // @constdefine Incoming (unanswered) call
      RIL_CALLSTAT_WAITING                        = $00000006;  // @constdefine Incoming call waiting call

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants CPI Status | CPI status values
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_CPISTAT_UNKNOWN                         = $00000000;  // @constdefine
      RIL_CPISTAT_NEW_OUTGOING                    = $00000001;  // @constdefine
      RIL_CPISTAT_NEW_INCOMING                    = $00000002;  // @constdefine
      RIL_CPISTAT_CONNECTED                       = $00000003;  // @constdefine
      RIL_CPISTAT_DISCONNECTED                    = $00000004;  // @constdefine
      RIL_CPISTAT_ONHOLD                          = $00000005;  // @constdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Intermediate Supplementary Service | Intermediate Supplementary Service Codes
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_INTSSCODE_UNCONDITIONALCFACTIVE         = $00000000;      // @constdefine Unconditional call forwarding is active
      RIL_INTSSCODE_SOMECONDITIONALCFACTIVE       = $00000001;      // @constdefine Some of the conditional call forwarding settings are active
      RIL_INTSSCODE_CALLWASFORWARDED              = $00000002;      // @constdefine Call has been forwarded
      RIL_INTSSCODE_CALLISWAITING                 = $00000003;      // @constdefine Call is waiting
      RIL_INTSSCODE_CUGCALL                       = $00000004;      // @constdefine This is a CUG call (also <index> present;
      RIL_INTSSCODE_OUTGOINGCALLSBARRED           = $00000005;      // @constdefine Outgoing calls are barred
      RIL_INTSSCODE_INCOMINGCALLSBARRED           = $00000006;      // @constdefine Incoming calls are barred
      RIL_INTSSCODE_CLIRSUPPRESSREJECT            = $00000007;      // @constdefine CLIR suppression rejected
      RIL_INTSSCODE_CALLWASDEFLECTED              = $00000008;      // @constdefine Call has been deflected

      RIL_INTSSCODE_MAX                           = RIL_INTSSCODE_CALLWASDEFLECTED;    // @constdefine Maximum valid value

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Unsolicited Supplementary Service | Unsolicited Supplementary Service Codes
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_UNSSSCODE_FORWARDEDCALL                 = $00000000;      // @constdefine This is a forwarded call (MT call setup;
      RIL_UNSSSCODE_CUGCALL                       = $00000001;      // @constdefine This is a CUG call (also <index> present; (MT call setup;
      RIL_UNSSSCODE_CALLPUTONHOLD                 = $00000002;      // @constdefine Call has been put on hold (during a voice call;
      RIL_UNSSSCODE_CALLRETRIEVED                 = $00000003;      // @constdefine Call has been retrieved (during a voice call;
      RIL_UNSSSCODE_ENTEREDMULTIPARTY             = $00000004;      // @constdefine Multiparty call entered (during a voice call;
      RIL_UNSSSCODE_HELDCALLRELEASED              = $00000005;      // @constdefine Call on hold has been released (this is not a SS notification; (during a voice call;
      RIL_UNSSSCODE_FORWARDCHECKSS                = $00000006;      // @constdefine Forward check SS message received (can be received whenever;
      RIL_UNSSSCODE_ALERTINGEXPLICITCALLXFER      = $00000007;      // @constdefine Call is being connected (alerting; with the remote party in alerting state in explicit call transfer operation (during a voice call;
      RIL_UNSSSCODE_CONNECTEDEXPLICITCALLXFER     = $00000008;      // @constdefine Call has been connected with the other remote party in explicit call transfer operation (also number and subaddress parameters may be present; (during a voice call or MT call setup;
      RIL_UNSSSCODE_DEFLECTEDCALL                 = $00000009;      // @constdefine This is a deflected call (MT call setup;
      RIL_UNSSSCODE_ADDITIONALINCOMINGCF          = $0000000a;      // @constdefine Additional incoming call forwarded

      RIL_UNSSSCODE_MAX                           = RIL_UNSSSCODE_ADDITIONALINCOMINGCF; // @constdefine Maximum valid value

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Call Multiparty | Call multiparty status values
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_CALL_SINGLEPARTY                        = $00000000;  // @constdefine Not in a conference
      RIL_CALL_MULTIPARTY                         = $00000001;  // @constdefine Participating in a conference

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Call Management | Call management commands
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_CALLCMD_RELEASEHELD                     = $00000001;  // @constdefine Release all held calls, send "busy" to waiting call
      RIL_CALLCMD_RELEASEACTIVE_ACCEPTHELD        = $00000002;  // @constdefine Release all active calls, accept waiting/held call
      RIL_CALLCMD_RELEASECALL                     = $00000003;  // @constdefine Release the specified call
      RIL_CALLCMD_HOLDACTIVE_ACCEPTHELD           = $00000004;  // @constdefine Hold all active calls, accept waiting/held call
      RIL_CALLCMD_HOLDALLBUTONE                   = $00000005;  // @constdefine Hold all active calls, except for the specified call
      RIL_CALLCMD_ADDHELDTOCONF                   = $00000006;  // @constdefine Add all held calls to a conference
      RIL_CALLCMD_ADDHELDTOCONF_DISCONNECT        = $00000007;  // @constdefine Connect held calls to a conference, disconnect the user
      RIL_CALLCMD_INVOKECCBS                      = $00000008;  // @constdefine Invokes completion of calls to busy subscribers

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Line Status | Line status values
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_LINESTAT_UNKNOWN                        = $00000000;  // @constdefine Unknown
      RIL_LINESTAT_READY                          = $00000001;  // @constdefine Line is ready
      RIL_LINESTAT_UNAVAILABLE                    = $00000002;  // @constdefine Line is unavailable
      RIL_LINESTAT_RINGING                        = $00000003;  // @constdefine Incoming call on the line
      RIL_LINESTAT_CALLINPROGRESS                 = $00000004;  // @constdefine Call in progress
      RIL_LINESTAT_ASLEEP                         = $00000005;  // @constdefine Line is asleep
      RIL_LINESTAT_CONNECTING                     = $00000006;  // @constdefine The phone is connecting to a call, but the call is not in progress yet

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Line Registration | Line registration status values
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_REGSTAT_UNKNOWN                         = $00000000;  // @constdefine Registration unknown
      RIL_REGSTAT_UNREGISTERED                    = $00000001;  // @constdefine Unregistered
      RIL_REGSTAT_HOME                            = $00000002;  // @constdefine Registered on home network
      RIL_REGSTAT_ATTEMPTING                      = $00000003;  // @constdefine Attempting to register
      RIL_REGSTAT_DENIED                          = $00000004;  // @constdefine Registration denied
      RIL_REGSTAT_ROAMING                         = $00000005;  // @constdefine Registered on roaming network

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Audio Device | Audio devices
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_AUDIO_NONE                              = $00000000;  // @constdefine No audio devices
      RIL_AUDIO_HANDSET                           = $00000001;  // @constdefine Handset
      RIL_AUDIO_SPEAKERPHONE                      = $00000002;  // @constdefine Speakerphone
      RIL_AUDIO_HEADSET                           = $00000003;  // @constdefine Headset
      RIL_AUDIO_CARKIT                            = $00000004;  // @constdefine Carkit

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants HSCSD Traffic Channel | HSCSD traffic channel codings
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_HSCSDCODING_UNKNOWN                     = $00000000;  // @constdefine Unknown channel coding
      RIL_HSCSDCODING_4800_FULLRATE               = $00000001;  // @constdefine 4800 bits per second
      RIL_HSCSDCODING_9600_FULLRATE               = $00000002;  // @constdefine 9600 bits per second
      RIL_HSCSDCODING_14400_FULLRATE              = $00000004;  // @constdefine 14400 bits per second
      RIL_HSCSDCODING_ALL                         = $00000007;  // @constdefine All channel codings valid

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants HSCSD Air Interface | HSCSD air interface user rates
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_HSCSDAIURATE_UNKNOWN                    = $00000000;  // @constdefine Air interface rate
      RIL_HSCSDAIURATE_9600                       = $00000001;  // @constdefine 9600 bits per second
      RIL_HSCSDAIURATE_14400                      = $00000002;  // @constdefine 14400 bits per second
      RIL_HSCSDAIURATE_19200                      = $00000003;  // @constdefine 19200 bits per second
      RIL_HSCSDAIURATE_28800                      = $00000004;  // @constdefine 28800 bits per second
      RIL_HSCSDAIURATE_38400                      = $00000005;  // @constdefine 38400 bits per second
      RIL_HSCSDAIURATE_43200                      = $00000006;  // @constdefine 43200 bits per second
      RIL_HSCSDAIURATE_57600                      = $00000007;  // @constdefine 57600 bits per second
      RIL_HSCSDAIURATE_DEFAULT                    = $ffffffff;  // @constdefine A special value that indicates the radio stack
                                                                    //    should calculate the appropriate number of
                                                                    //    receive timeslots based on other paramaters

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants HSCSD Special | Special HSCSD receive timeslots value
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_HSCSDTIMESLOTS_DEFAULT                  = $00000000;  // @constdefine Indicates that the radio stack should
                                                                // calculate apropriate number of timeslots
      RIL_HSCSDTIMESLOTSLIMIT_NONE                = $00000000;  // @constdefine Indicates that number of receive numeslots will not
                                                                //    be altered during the next non-transparent HSCSD call

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Data Compression | Data compression directions
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_DATACOMPDIR_NONE                        = $00000001;  // @constdefine No data compression
      RIL_DATACOMPDIR_TRANSMIT                    = $00000002;  // @constdefine Data compession when sending
      RIL_DATACOMPDIR_RECEIVE                     = $00000004;  // @constdefine Data compession when receiving
      RIL_DATACOMPDIR_BOTH                        = $00000008;  // @constdefine Bi-directional data compession

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Data Compression Negotiation | Data compression negotiation options
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_DATACOMP_OPTIONAL                       = $00000001;  // @constdefine Data compression optional
      RIL_DATACOMP_REQUIRED                       = $00000002;  // @constdefine Terminal will disconnect if no negotiation

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Error Correction | Error correction modes
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_ECMODE_UNKNOWN                          = $00000000;  // @constdefine TBD
      RIL_ECMODE_DIRECT                           = $00000001;  // @constdefine TBD
      RIL_ECMODE_BUFFERED                         = $00000002;  // @constdefine TBD
      RIL_ECMODE_NODETECT                         = $00000004;  // @constdefine TBD
      RIL_ECMODE_DETECT                           = $00000008;  // @constdefine TBD
      RIL_ECMODE_ALTERNATIVE                      = $00000010;  // @constdefine TBD
      RIL_ECMODE_OPTIONAL_USEBUFFERED             = $00000020;  // @constdefine TBD
      RIL_ECMODE_OPTIONAL_USEDIRECT               = $00000040;  // @constdefine TBD
      RIL_ECMODE_REQUIRED                         = $00000080;  // @constdefine TBD
      RIL_ECMODE_REQUIRED_LAPMONLY                = $00000100;  // @constdefine TBD
      RIL_ECMODE_REQUIRED_ALTERNATIVEONLY         = $00000200;  // @constdefine TBD

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Bearer Service | Bearer service names
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_BSVCNAME_UNKNOWN                        = $00000000;  // @constdefine TBD
      RIL_BSVCNAME_DATACIRCUIT_ASYNC_UDI_MODEM    = $00000001;  // @constdefine TBD
      RIL_BSVCNAME_DATACIRCUIT_SYNC_UDI_MODEM     = $00000002;  // @constdefine TBD
      RIL_BSVCNAME_PADACCESS_ASYNC_UDI            = $00000003;  // @constdefine TBD
      RIL_BSVCNAME_PACKETACCESS_SYNC_UDI          = $00000004;  // @constdefine TBD
      RIL_BSVCNAME_DATACIRCUIT_ASYNC_RDI          = $00000005;  // @constdefine TBD
      RIL_BSVCNAME_DATACIRCUIT_SYNC_RDI           = $00000006;  // @constdefine TBD
      RIL_BSVCNAME_PADACCESS_ASYNC_RDI            = $00000007;  // @constdefine TBD
      RIL_BSVCNAME_PACKETACCESS_SYNC_RDI          = $00000008;  // @constdefine TBD

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Bearer Service CE | Bearer service connection elements
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_BSVCCE_UNKNOWN                          = $00000000;  // @constdefine Bearer service unknown
      RIL_BSVCCE_TRANSPARENT                      = $00000001;  // @constdefine Link layer correction enabled
      RIL_BSVCCE_NONTRANSPARENT                   = $00000002;  // @constdefine No link layer correction present
      RIL_BSVCCE_BOTH_TRANSPARENT                 = $00000003;  // @constdefine Both available, transparent preferred
      RIL_BSVCCE_BOTH_NONTRANSPARENT              = $00000004;  // @constdefine Both available, non-transparent preferred

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Message Service | Messaging service types
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_MSGSVCTYPE_UNKNOWN                      = $00000000;  // @constdefine Unknown
      RIL_MSGSVCTYPE_PHASE2                       = $00000001;  // @constdefine GSM 07.05 Phase 2 ver. 4.7.0 messaging service
      RIL_MSGSVCTYPE_PHASE2PLUS                   = $00000002;  // @constdefine GSM 07.05 Pahse 2+ messaging service

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Message Storage | Message storage locations
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_MSGLOC_UNKNOWN                          = $00000000;  // @constdefine Unknown
      RIL_MSGLOC_BROADCAST                        = $00000001;  // @constdefine Broadcast message storage location
      RIL_MSGLOC_SIM                              = $00000002;  // @constdefine SIM storage location
      RIL_MSGLOC_STATUSREPORT                     = $00000003;  // @constdefine Status report storage location

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants IS637 Teleservices | Message Teleservice types
//
// @comm TIA/EIA-41-D Supported Teleservices
//
// -----------------------------------------------------------------------------
const
      RIL_MSGTELESERVICE_PAGING                   = $00000001;    // @constdefine Wireless Paging Teleservice      CPT-95  //@ Only callback number
      RIL_MSGTELESERVICE_MESSAGING                = $00000002;    // @constdefine Wireless Messaging Teleservice   CMT-95  //@ Text Message
      RIL_MSGTELESERVICE_VOICEMAIL                = $00000003;    // @constdefine Voice Mail Notification          VMN-95  //@ Voice Mail
      RIL_MSGTELESERVICE_WAP                      = $00000004;    // @constdefine Wireless Application Protocol    WAP     //@ To be investigated (Test message??)

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Message Privacy Class | Message Privacy types
//
// @comm IS-637 Message Privacy Classes
//
// -----------------------------------------------------------------------------
const
      RIL_MSGPRIVACYCLASS_NOTRESTRICTED           = $00000001; // @constdefine Not restricted (Level 0)
      RIL_MSGPRIVACYCLASS_RESTRICTED              = $00000002; // @constdefine Restricted (Level 1)
      RIL_MSGPRIVACYCLASS_CONFIDENTIAL            = $00000003; // @constdefine Confidential (Level 2)
      RIL_MSGPRIVACYCLASS_SECRET                  = $00000004; // @constdefine Secret (Level 3)

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Message Priority Class | Message Priority types
//
// @comm IS-637 Message Priority Classes
//
// -----------------------------------------------------------------------------
const
      RIL_MSGPRIORITY_NORMAL                      = $00000001;    // @constdefine Message Urgency Normal
      RIL_MSGPRIORITY_HIGH                        = $00000002;    // @constdefine Message Urgency Interactive (S N/A)
      RIL_MSGPRIORITY_URGENT                      = $00000003;    // @constdefine Message Urgency Urgent
      RIL_MSGPRIORITY_EMERGENCY                   = $00000004;    // @constdefine Message Urgency Emergency (S N/A)

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Status Message Class | Message Statustypes
//
// @comm IS-637 Message Priority Classes
//
// -----------------------------------------------------------------------------
const
      RIL_MSGSTATUSTYPE_BEARERACK                 = $00000001;    // @constdefine The Acknowledgement Message is a Bearer Ack
      RIL_MSGSTATUSTYPE_DELIVERYACK               = $00000002;    // @constdefine The Acknowledgement Message is a Delivery Ack
      RIL_MSGSTATUSTYPE_USERACK                   = $00000003;    // @constdefine The Acknowledgement Message is a User Ack

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Message Display Modes | Message Display Modes
//
// @comm Message Display Modes - Determines if the message is shown immediately or in the inbox (Ask Carrier if this feature is implemented)
//
// -----------------------------------------------------------------------------
const
      RIL_MSGDISPLAYMODE_IMMEDIATE                = $00000001;    // @constdefine The message must be show immediately.
//In the UI, Mobile Default and User Default should be treated as the same.
      RIL_MSGDISPLAYMODE_MOBILEDEFAULT            = $00000002;    // @constdefine The message is to be displayed depending on a predefined mobile setting.
      RIL_MSGDISPLAYMODE_USERDEFAULT              = $00000003;    // @constdefine The message is to be displayed depending on the user's mode.

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Message Encoding | Message Encoding types
//
// @comm Message Encoding - Determines the format of the incoming message body
//
// -----------------------------------------------------------------------------
const
//Analog Only -      RIL_MSGCODING_IS91EPP                       = $00000001;    // @constdefine IS-91 Character Format
      RIL_MSGCODING_7BITASCII                     = $00000002;    // @constdefine This the the verizon default
      RIL_MSGCODING_UNICODE                       = $00000003;    // @constdefine Unicode (double byte) format
      RIL_MSGCODING_7BITGSM                       = $00000004;    // @constdefine 7-bit GSM Alphabet
      RIL_MSGCODING_8BITGSM                       = $00000005;    // @constdefine 8-bit GSM Alphabet

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Message DCS | Message data coding scheme types
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_DCSTYPE_GENERAL                         = $00000001;  // @constdefine TBD
      RIL_DCSTYPE_MSGWAIT                         = $00000002;  // @constdefine TBD
      RIL_DCSTYPE_MSGCLASS                        = $00000003;  // @constdefine TBD
      RIL_DCSTYPE_LANGUAGE                        = $00000004;  // @constdefine TBD

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Message DCS Flags | Message data coding scheme flags
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_DCSFLAG_NONE                            = $00000000;  // @constdefine TBD
      RIL_DCSFLAG_COMPRESSED                      = $00000001;  // @constdefine TBD
      RIL_DCSFLAG_INDICATIONACTIVE                = $00000002;  // @constdefine TBD
      RIL_DCSFLAG_DISCARD                         = $00000004;  // @constdefine Only for RIL_DCSTYPE_MSGWAIT
      RIL_DCSFLAG_ALL                             = $00000007;  // @constdefine TBD

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Message DCS Classes | Message data coding scheme message classes
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_DCSMSGCLASS_0                           = $00000001;  // @constdefine TBD
      RIL_DCSMSGCLASS_1                           = $00000002;  // @constdefine TBD
      RIL_DCSMSGCLASS_2                           = $00000003;  // @constdefine TBD
      RIL_DCSMSGCLASS_3                           = $00000004;  // @constdefine TBD

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Message DCS Alphabets | Message data coding scheme alphabets
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_DCSALPHABET_DEFAULT                     = $00000001;  // @constdefine TBD
      RIL_DCSALPHABET_8BIT                        = $00000002;  // @constdefine TBD
      RIL_DCSALPHABET_UCS2                        = $00000003;  // @constdefine TBD

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Message DCS Indication | Message data coding scheme indication types
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_DCSINDICATION_VOICEMAIL                 = $00000001;  // @constdefine Voicemail indication
      RIL_DCSINDICATION_FAX                       = $00000002;  // @constdefine Fax indication
      RIL_DCSINDICATION_EMAIL                     = $00000003;  // @constdefine E-Mail indication
      RIL_DCSINDICATION_OTHER                     = $00000004;  // @constdefine Other indication

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Message DCS Broadcast| Message broadcast data coding scheme languages
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_DCSLANG_UNKNOWN                         = $00000001;  // @constdefine TBD
      RIL_DCSLANG_GERMAN                          = $00000002;  // @constdefine TBD
      RIL_DCSLANG_ENGLISH                         = $00000004;  // @constdefine TBD
      RIL_DCSLANG_ITALIAN                         = $00000008;  // @constdefine TBD
      RIL_DCSLANG_FRENCH                          = $00000010;  // @constdefine TBD
      RIL_DCSLANG_SPANISH                         = $00000020;  // @constdefine TBD
      RIL_DCSLANG_DUTCH                           = $00000040;  // @constdefine TBD
      RIL_DCSLANG_SWEDISH                         = $00000080;  // @constdefine TBD
      RIL_DCSLANG_DANISH                          = $00000100;  // @constdefine TBD
      RIL_DCSLANG_PORTUGUESE                      = $00000200;  // @constdefine TBD
      RIL_DCSLANG_FINNISH                         = $00000400;  // @constdefine TBD
      RIL_DCSLANG_NORWEGIAN                       = $00000800;  // @constdefine TBD
      RIL_DCSLANG_GREEK                           = $00001000;  // @constdefine TBD
      RIL_DCSLANG_TURKISH                         = $00002000;  // @constdefine TBD
      RIL_DCSLANG_HUNGARIAN                       = $00004000;  // @constdefine TBD
      RIL_DCSLANG_POLISH                          = $00008000;  // @constdefine TBD
      RIL_DCSLANG_CZECH                           = $00010000;  // @constdefine TBD
      RIL_DCSLANG_HEBREW                          = $00020000;  // @constdefine TBD
      RIL_DCSLANG_ARABIC                          = $00040000;  // @constdefine TBD
      RIL_DCSLANG_RUSSIAN                         = $00080000;  // @constdefine TBD
      RIL_DCSLANG_ICELANDIC                       = $00100000;  // @constdefine TBD
      RIL_DCSLANG_ALL                             = $001fffff;  // @constdefine TBD

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Message Class | Message classes
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_MSGCLASS_NONE                           = $00000000;  // @constdefine TBD
      RIL_MSGCLASS_INCOMING                       = $00010000;  // @constdefine TBD
      RIL_MSGCLASS_OUTGOING                       = $00020000;  // @constdefine TBD
      RIL_MSGCLASS_BROADCAST                      = $00040000;  // @constdefine TBD
      RIL_MSGCLASS_ALL                            = $00070000;  // @constdefine TBD

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Message Type | Message types
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_MSGTYPE_IN_DELIVER                      = $00000001 or RIL_MSGCLASS_INCOMING;  // @constdefine Incoming message
      RIL_MSGTYPE_IN_STATUS                       = $00000002 or RIL_MSGCLASS_INCOMING;  // @constdefine Incoming status message
      RIL_MSGTYPE_OUT_SUBMIT                      = $00000001 or RIL_MSGCLASS_OUTGOING;  // @constdefine Outgoing message
      RIL_MSGTYPE_OUT_COMMAND                     = $00000002 or RIL_MSGCLASS_OUTGOING;  // @constdefine Outgoing command message
      RIL_MSGTYPE_OUT_RAW                         = $00000004 or RIL_MSGCLASS_OUTGOING;
      RIL_MSGTYPE_BC_GENERAL                      = $00000001 or RIL_MSGCLASS_BROADCAST;  // @constdefine Broadcast message (incoming only)

// Macro to extract message class from message type
function MSGCLASS_FROM_MSGTYPE(itype:ULONG):ULONG; inline;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Message Flag | Message flags
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_MSGFLAG_NONE                            = $00000000;  // @constdefine None
      RIL_MSGFLAG_MORETOSEND                      = $00000001;  // @constdefine More messages to send (valid for <def RIL_MSGTYPE_IN_DELIVER> and <def RIL_MSGTYPE_IN_STATUS>)
      RIL_MSGFLAG_REPLYPATH                       = $00000002;  // @constdefine Message contains a reply path  (valid for <def RIL_MSGTYPE_IN_DELIVER> and <def RIL_MSGTYPE_OUT_SUBMIT>)
      RIL_MSGFLAG_HEADER                          = $00000004;  // @constdefine TBD (valid for <def RIL_MSGTYPE_IN_DELIVER>, <def RIL_MSGTYPE_OUT_SUBMIT>,
                                                                      //    <def RIL_MSGTYPE_IN_STATUS>, and <def RIL_MSGTYPE_OUT_COMMAND>)
      RIL_MSGFLAG_REJECTDUPS                      = $00000008;  // @constdefine TBD (valid for <def RIL_MSGTYPE_OUT_SUBMIT> only)
      RIL_MSGFLAG_STATUSREPORTRETURNED            = $00000010;  // @constdefine (valid for <def RIL_MSGTYPE_IN_DELIVER> only)
      RIL_MSGFLAG_STATUSREPORTREQUESTED           = $00000020;  // @constdefine (valid for <def RIL_MSGTYPE_OUT_SUBMIT> and <def RIL_MSGTYPE_OUT_COMMAND>)
      RIL_MSGFLAG_CAUSEDBYCOMMAND                 = $00000040;  // @constdefine (valid for <def RIL_MSGTYPE_IN_STATUS> only)
      RIL_MSGFLAG_ALL                             = $0000007f;  // @constdefine All flags are on

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Message Protocol | Message protocol IDs
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_MSGPROTOCOL_UNKNOWN                     = $00000000;  // @constdefine TBD
      RIL_MSGPROTOCOL_SMETOSME                    = $00000001;  // @constdefine TBD
      RIL_MSGPROTOCOL_IMPLICIT                    = $00000002;  // @constdefine TBD
      RIL_MSGPROTOCOL_TELEX                       = $00000003;  // @constdefine TBD
      RIL_MSGPROTOCOL_TELEFAX_GROUP3              = $00000004;  // @constdefine TBD
      RIL_MSGPROTOCOL_TELEFAX_GROUP4              = $00000005;  // @constdefine TBD
      RIL_MSGPROTOCOL_VOICEPHONE                  = $00000006;  // @constdefine TBD
      RIL_MSGPROTOCOL_ERMES                       = $00000007;  // @constdefine TBD
      RIL_MSGPROTOCOL_PAGING                      = $00000008;  // @constdefine TBD
      RIL_MSGPROTOCOL_VIDEOTEX                    = $00000009;  // @constdefine TBD
      RIL_MSGPROTOCOL_TELETEX                     = $0000000a;  // @constdefine TBD
      RIL_MSGPROTOCOL_TELETEX_PSPDN               = $0000000b;  // @constdefine TBD
      RIL_MSGPROTOCOL_TELETEX_CSPDN               = $0000000c;  // @constdefine TBD
      RIL_MSGPROTOCOL_TELETEX_PSTN                = $0000000d;  // @constdefine TBD
      RIL_MSGPROTOCOL_TELETEX_ISDN                = $0000000e;  // @constdefine TBD
      RIL_MSGPROTOCOL_UCI                         = $0000000f;  // @constdefine TBD
      RIL_MSGPROTOCOL_MSGHANDLING                 = $00000010;  // @constdefine TBD
      RIL_MSGPROTOCOL_X400                        = $00000011;  // @constdefine TBD
      RIL_MSGPROTOCOL_EMAIL                       = $00000012;  // @constdefine TBD
      RIL_MSGPROTOCOL_SCSPECIFIC1                 = $00000013;     // @constdefine TBD
      RIL_MSGPROTOCOL_SCSPECIFIC2                 = $00000014;     // @constdefine TBD
      RIL_MSGPROTOCOL_SCSPECIFIC3                 = $00000015;     // @constdefine TBD
      RIL_MSGPROTOCOL_SCSPECIFIC4                 = $00000016;     // @constdefine TBD
      RIL_MSGPROTOCOL_SCSPECIFIC5                 = $00000017;     // @constdefine TBD
      RIL_MSGPROTOCOL_SCSPECIFIC6                 = $00000018;     // @constdefine TBD
      RIL_MSGPROTOCOL_SCSPECIFIC7                 = $00000019;     // @constdefine TBD
      RIL_MSGPROTOCOL_GSMSTATION                  = $0000001a;      // @constdefine TBD
      RIL_MSGPROTOCOL_SM_TYPE0                    = $0000001b;      // @constdefine TBD
      RIL_MSGPROTOCOL_RSM_TYPE1                   = $0000001c;      // @constdefine TBD
      RIL_MSGPROTOCOL_RSM_TYPE2                   = $0000001d;      // @constdefine TBD
      RIL_MSGPROTOCOL_RSM_TYPE3                   = $0000001e;      // @constdefine TBD
      RIL_MSGPROTOCOL_RSM_TYPE4                   = $0000001f;      // @constdefine TBD
      RIL_MSGPROTOCOL_RSM_TYPE5                   = $00000020;      // @constdefine TBD
      RIL_MSGPROTOCOL_RSM_TYPE6                   = $00000021;      // @constdefine TBD
      RIL_MSGPROTOCOL_RSM_TYPE7                   = $00000022;      // @constdefine TBD
      RIL_MSGPROTOCOL_RETURNCALL                  = $00000023;      // @constdefine TBD
      RIL_MSGPROTOCOL_ME_DOWNLOAD                 = $00000024;      // @constdefine TBD
      RIL_MSGPROTOCOL_DEPERSONALIZATION           = $00000025;      // @constdefine TBD
      RIL_MSGPROTOCOL_SIM_DOWNLOAD                = $00000026;      // @constdefine TBD

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Message Delivery | Message delivery status values
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_MSGDLVSTATUS_RECEIVEDBYSME              = $00000001;  // @constdefine TBD
      RIL_MSGDLVSTATUS_FORWARDEDTOSME             = $00000002;  // @constdefine TBD
      RIL_MSGDLVSTATUS_REPLACEDBYSC               = $00000003;  // @constdefine TBD
      RIL_MSGDLVSTATUS_CONGESTION_TRYING          = $00000004;  // @constdefine TBD
      RIL_MSGDLVSTATUS_SMEBUSY_TRYING             = $00000005;  // @constdefine TBD
      RIL_MSGDLVSTATUS_SMENOTRESPONDING_TRYING    = $00000006;  // @constdefine TBD
      RIL_MSGDLVSTATUS_SVCREJECTED_TRYING         = $00000007;  // @constdefine TBD
      RIL_MSGDLVSTATUS_QUALITYUNAVAIL_TRYING      = $00000008;  // @constdefine TBD
      RIL_MSGDLVSTATUS_SMEERROR_TRYING            = $00000009;  // @constdefine TBD
      RIL_MSGDLVSTATUS_CONGESTION                 = $0000000a;  // @constdefine TBD
      RIL_MSGDLVSTATUS_SMEBUSY                    = $0000000b;  // @constdefine TBD
      RIL_MSGDLVSTATUS_SMENOTRESPONDING           = $0000000c;  // @constdefine TBD
      RIL_MSGDLVSTATUS_SVCREJECTED                = $0000000d;  // @constdefine TBD
      RIL_MSGDLVSTATUS_QUALITYUNAVAIL_TEMP        = $0000000e;  // @constdefine TBD
      RIL_MSGDLVSTATUS_SMEERROR                   = $0000000f;  // @constdefine TBD
      RIL_MSGDLVSTATUS_REMOTEPROCERROR            = $00000010;  // @constdefine TBD
      RIL_MSGDLVSTATUS_INCOMPATIBLEDEST           = $00000011;  // @constdefine TBD
      RIL_MSGDLVSTATUS_CONNECTIONREJECTED         = $00000012;  // @constdefine TBD
      RIL_MSGDLVSTATUS_NOTOBTAINABLE              = $00000013;  // @constdefine TBD
      RIL_MSGDLVSTATUS_NOINTERNETWORKING          = $00000014;  // @constdefine TBD
      RIL_MSGDLVSTATUS_VPEXPIRED                  = $00000015;  // @constdefine TBD
      RIL_MSGDLVSTATUS_DELETEDBYORIGSME           = $00000016;  // @constdefine TBD
      RIL_MSGDLVSTATUS_DELETEDBYSC                = $00000017;  // @constdefine TBD
      RIL_MSGDLVSTATUS_NOLONGEREXISTS             = $00000018;  // @constdefine TBD
      RIL_MSGDLVSTATUS_QUALITYUNAVAIL             = $00000019;  // @constdefine TBD
      RIL_MSGDLVSTATUS_RESERVED_COMPLETED         = $0000001a;  // @constdefine TBD
      RIL_MSGDLVSTATUS_RESERVED_TRYING            = $0000001b;  // @constdefine TBD
      RIL_MSGDLVSTATUS_RESERVED_ERROR             = $0000001c;  // @constdefine TBD
      RIL_MSGDLVSTATUS_RESERVED_TMPERROR          = $0000001d;  // @constdefine TBD
      RIL_MSGDLVSTATUS_SCSPECIFIC_COMPLETED       = $0000001e;  // @constdefine TBD
      RIL_MSGDLVSTATUS_SCSPECIFIC_TRYING          = $0000001f;  // @constdefine TBD
      RIL_MSGDLVSTATUS_SCSPECIFIC_ERROR           = $00000020;  // @constdefine TBD
      RIL_MSGDLVSTATUS_SCSPECIFIC_TMPERROR        = $00000021;  // @constdefine TBD



// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Message Validity | Message validity period formats
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_MSGVP_NONE                              = $00000000;  // @constdefine TBD
      RIL_MSGVP_RELATIVE                          = $00000001;  // @constdefine TBD
      RIL_MSGVP_ENHANCED                          = $00000002;  // @constdefine TBD
      RIL_MSGVP_ABSOLUTE                          = $00000003;  // @constdefine TBD

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Message Command | Message command types
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_MSGCMDTYPE_STATUSREQ                    = $00000001;  // @constdefine TBD
      RIL_MSGCMDTYPE_CANCELSTATUSREQ              = $00000002;  // @constdefine TBD
      RIL_MSGCMDTYPE_DELETEMESSAGE                = $00000003;  // @constdefine TBD
      RIL_MSGCMDTYPE_ENABLESTATUSREQ              = $00000004;  // @constdefine TBD

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Message Geographic | Message geographic scopes
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_GEOSCOPE_CELL_IMMEDIATE                 = $00000001;  // @constdefine TBD
      RIL_GEOSCOPE_CELL                           = $00000002;  // @constdefine TBD
      RIL_GEOSCOPE_PLMN                           = $00000003;  // @constdefine TBD
      RIL_GEOSCOPE_LOCATIONAREA                   = $00000004;  // @constdefine TBD

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Message Status | Message status values
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_MSGSTATUS_UNKNOWN                       = $00000000;  // @constdefine TBD
      RIL_MSGSTATUS_RECUNREAD                     = $00000001;  // @constdefine TBD
      RIL_MSGSTATUS_RECREAD                       = $00000002;  // @constdefine TBD
      RIL_MSGSTATUS_STOUNSENT                     = $00000003;  // @constdefine TBD
      RIL_MSGSTATUS_STOSENT                       = $00000004;  // @constdefine TBD

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Message Send | Send message options
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_SENDOPT_NONE                            = $00000000;  // @constdefine TBD
      RIL_SENDOPT_PERSISTLINK                     = $00000001;  // @constdefine TBD

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Phone Locked | Phone locked states
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_LOCKEDSTATE_UNKNOWN                     = $00000000;  // @constdefine Locking state unknown
      RIL_LOCKEDSTATE_READY                       = $00000001;  // @constdefine ME not locked
      RIL_LOCKEDSTATE_SIM_PIN                     = $00000002;  // @constdefine ME awaiting PIN
      RIL_LOCKEDSTATE_SIM_PUK                     = $00000003;  // @constdefine ME awaiting PUK
      RIL_LOCKEDSTATE_PH_SIM_PIN                  = $00000004;  // @constdefine ME awaiting phone-to-sim password
      RIL_LOCKEDSTATE_PH_FSIM_PIN                 = $00000005;  // @constdefine ME awaiting phone-to-first-sim password
      RIL_LOCKEDSTATE_PH_FSIM_PUK                 = $00000006;  // @constdefine ME awaiting phone-to-first-sim PUK
      RIL_LOCKEDSTATE_SIM_PIN2                    = $00000007;  // @constdefine ME awaiting PIN2/CHV2
      RIL_LOCKEDSTATE_SIM_PUK2                    = $00000008;  // @constdefine ME awaiting PUK2
      RIL_LOCKEDSTATE_PH_NET_PIN                  = $00000009;  // @constdefine ME awaiting network personilzation PIN
      RIL_LOCKEDSTATE_PH_NET_PUK                  = $0000000a;  // @constdefine ME awaiting network personilzation PUK
      RIL_LOCKEDSTATE_PH_NETSUB_PIN               = $0000000b;  // @constdefine ME awaiting network subset personilzation PIN
      RIL_LOCKEDSTATE_PH_NETSUB_PUK               = $0000000c;  // @constdefine ME awaiting network subset personilzation PUK
      RIL_LOCKEDSTATE_PH_SP_PIN                   = $0000000d;  // @constdefine ME awaiting service provider PIN
      RIL_LOCKEDSTATE_PH_SP_PUK                   = $0000000e;  // @constdefine ME awaiting service provider PUK
      RIL_LOCKEDSTATE_PH_CORP_PIN                 = $0000000f;  // @constdefine ME awaiting corporate personilzation PIN
      RIL_LOCKEDSTATE_PH_CORP_PUK                 = $00000010;  // @constdefine ME awaiting corporate personilzation PUK

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Lock Facility | Facilities for phone locking
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_LOCKFACILITY_CNTRL                      = $00000001;  // @constdefine Lock control curface
      RIL_LOCKFACILITY_PH_SIM                     = $00000002;  // @constdefine Lock phone to SIM card
      RIL_LOCKFACILITY_PH_FSIM                    = $00000003;  // @constdefine Lock phone to first SIM card
      RIL_LOCKFACILITY_SIM                        = $00000004;  // @constdefine Lock SIM card
      RIL_LOCKFACILITY_SIM_PIN2                   = $00000005;  // @constdefine SIM PIN2 (only for RIL_ChangeLockingPassword())
      RIL_LOCKFACILITY_SIM_FIXEDIALING            = $00000006;  // @constdefine SIM fixed dialing memory
      RIL_LOCKFACILITY_NETWORKPERS                = $00000007;  // @constdefine Network personalization
      RIL_LOCKFACILITY_NETWORKSUBPERS             = $00000008;  // @constdefine Network subset personalization
      RIL_LOCKFACILITY_SERVICEPROVPERS            = $00000009;  // @constdefine Service provider personalization
      RIL_LOCKFACILITY_CORPPERS                   = $0000000a;  // @constdefine Corporate personalization

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Lock Status | Locking status values
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_LOCKINGSTATUS_DISABLED                  = $00000001;  // @constdefine Disable
      RIL_LOCKINGSTATUS_ENABLED                   = $00000002;  // @constdefine Enabled

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants SIM Security | SIM Security states
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_SIMSECURITYSTATE_UNKNOWN                = $00000000;      // @constdefine SIM security state unknown
      RIL_SIMSECURITYSTATE_PINREQUESTED           = $00000001;      // @constdefine SIM security state requested PIN
      RIL_SIMSECURITYSTATE_PINRECEIVED            = $00000002;      // @constdefine SIM security state received PIN

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Call Barr Facility | Types of call barring
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_BARRTYPE_ALLOUTGOING                    = $00000001;  // @constdefine Barr all outgoing calls
      RIL_BARRTYPE_OUTGOINGINT                    = $00000002;  // @constdefine Barr outgoing international calls
      RIL_BARRTYPE_OUTGOINGINTEXTOHOME            = $00000003;  // @constdefine Barr outgoing international calls except to home country
      RIL_BARRTYPE_ALLINCOMING                    = $00000004;  // @constdefine Barr all incoming calls
      RIL_BARRTYPE_INCOMINGROAMING                = $00000005;  // @constdefine Barr incoming calls when roaming outside of home country
      RIL_BARRTYPE_INCOMINGNOTINSIM               = $00000006;  // @constdefine Barr incoming calls from numbers not stored to SIM memory
      RIL_BARRTYPE_ALLBARRING                     = $00000007;  // @constdefine All barring services
      RIL_BARRTYPE_ALLOUTGOINGBARRING             = $00000008;  // @constdefine All outgoing barring services
      RIL_BARRTYPE_ALLINCOMINGBARRING             = $00000009;  // @constdefine All incoming barring services

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Call Barr Status | Status values for call barring
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_BARRINGSTATUS_DISABLED                  = $00000001;  // @constdefine Disable
      RIL_BARRINGSTATUS_ENABLED                   = $00000002;  // @constdefine Disable

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Equipment State | Equipment states
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_EQSTATE_UNKNOWN                         = $00000000;  // @constdefine Unknown
      RIL_EQSTATE_MINIMUM                         = $00000001;  // @constdefine Minimum power state
      RIL_EQSTATE_FULL                            = $00000002;  // @constdefine Full functionality
      RIL_EQSTATE_DISABLETX                       = $00000003;  // @constdefine Transmitter disabled
      RIL_EQSTATE_DISABLERX                       = $00000004;  // @constdefine Receiver disabled
      RIL_EQSTATE_DISABLETXANDRX                  = $00000005;  // @constdefine Transmitter & receiver disabled

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Radio Presence States | Radio Presence States
//
// @comm These states are determined by whether the driver is loaded or not
//
// -----------------------------------------------------------------------------
const
      RIL_RADIOPRESENCE_NOTPRESENT                = $00000000;  // @constdefine There is not radio module present in the device
      RIL_RADIOPRESENCE_PRESENT                   = $00000001;  // @constdefine There is a radio module present that RIL can use

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Driver defined Radio ON vs OFF State | Radio ON/OFF states
//
// @comm These values normally depend on the Equiptment state
//
// -----------------------------------------------------------------------------
const
      RIL_RADIOSUPPORT_UNKNOWN                    = $00000000;  // @constdefine The Radio Functionality is in an intermediate state
      RIL_RADIOSUPPORT_OFF                        = $00000001;  // @constdefine The Radio Functionality is OFF (DOES NOT Neccessarily mean safe for flight)
      RIL_RADIOSUPPORT_ON                         = $00000002;  // @constdefine The Radio Functionality is ON

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Various components of the Radio are ready for external usage
//
// @comm This will be a mask of the below values
//
// -----------------------------------------------------------------------------
const
      RIL_READYSTATE_NONE                         = $00000000;  // @constdefine Nothing is ready yet
      RIL_READYSTATE_INITIALIZED                  = $00000001;  // @constdefine The Radio has been initialized (but may not be ready)
      RIL_READYSTATE_SIM                          = $00000002;  // @constdefine The Radio is ready for SIM Access
      RIL_READYSTATE_SMS                          = $00000004;  // @constdefine The Radio is ready for SMS messages
      RIL_READYSTATE_UNLOCKED                     = $00000008;  // @constdefine The SIM is unlocked
      RIL_READYSTATE_SIM_PB                       = $00000010;      // @constdefine The SIM PB has been fully copied to volatile memory and is ready for access

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Various SIM card states.
//
// @comm NONE
//
// -----------------------------------------------------------------------------
const
      RIL_SIMSTATUSCHANGED_NONE                   = $00000000;      // @constdefine No status yet
      RIL_SIMSTATUSCHANGED_FULL                   = $00000001;      // @constdefine SIM card memory is full
      RIL_SIMSTATUSCHANGED_NO_SIM                 = $00000002;      // @constdefine No SIM card available
      RIL_SIMSTATUSCHANGED_INVALID                = $00000004;      // @constdefine SIM card is invalid
      RIL_SIMSTATUSCHANGED_BLOCKED                = $00000008;      // @constdefine SIM card is blocked

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Phonebook Storage | Phonebook storage locations
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PBLOC_UNKNOWN                           = $00000000;  // @constdefine Unknown
      RIL_PBLOC_SIMEMERGENCY                      = $00000001;  // @constdefine Emergency numbers
      RIL_PBLOC_SIMFIXDIALING                     = $00000002;  // @constdefine Fixed dialing
      RIL_PBLOC_SIMLASTDIALING                    = $00000003;  // @constdefine Recent calls list
      RIL_PBLOC_OWNNUMBERS                        = $00000004;  // @constdefine TBD
      RIL_PBLOC_SIMPHONEBOOK                      = $00000005;  // @constdefine SIM phonebook

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Special Phonebook | Special phonebook index value
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PBINDEX_FIRSTAVAILABLE                  = $ffffffff;  // @constdefine User first available entry

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants SIM Command | SIM commands
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_SIMCMD_READBINARY                       = $00000001;  // @constdefine Read a binary
      RIL_SIMCMD_READRECORD                       = $00000002;  // @constdefine Read contents of a record
      RIL_SIMCMD_GETRESPONSE                      = $00000003;  // @constdefine Required to get output data for some commands
      RIL_SIMCMD_UPDATEBINARY                     = $00000004;  // @constdefine Update a transparent file
      RIL_SIMCMD_UPDATERECORD                     = $00000005;  // @constdefine Update a linear fixed or cyclic file
      RIL_SIMCMD_STATUS                           = $00000006;  // @constdefine Get status on a file

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants SIM Record | Different SIM file types
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_SIMRECORDTYPE_UNKNOWN          = $00000000;  // @constdefine An unknown file type
      RIL_SIMRECORDTYPE_TRANSPARENT      = $00000001;  // @constdefine A single veriable lengthed record
      RIL_SIMRECORDTYPE_CYCLIC           = $00000002;  // @constdefine A cyclic set of records, each of the same length
      RIL_SIMRECORDTYPE_LINEAR           = $00000003;  // @constdefine A linear set of records, each of the same length
      RIL_SIMRECORDTYPE_MASTER           = $00000004;  // @constdefine Every SIM has a single master record, effectively the head node
      RIL_SIMRECORDTYPE_DEDICATED        = $00000005;  // @constdefine Effectively a "directory" file which is a parent of other records

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants SIM Toolkit Terminate | SIM Toolkit session termination causes
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_SIMTKITTERMCAUSE_USERSTOPPEDREDIAL      = $00000001;  // @constdefine User stopped redial attempts
      RIL_SIMTKITTERMCAUSE_ENDOFREDIAL            = $00000002;  // @constdefine End of redial period
      RIL_SIMTKITTERMCAUSE_USERENDEDSESSION       = $00000003;  // @constdefine Session terminated by user


// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILSIMTOOLKITNOTIFYCAPS
//
// @constants Unavailable | Detailed reason for support of toolkit functions
//
// @comm Values that variables information variables in RILSIMTOOLKITNOTIFYCAPS can take on
//
// -----------------------------------------------------------------------------
const
      RIL_SIMTKN_MEIMPLEMENTS                     = $00000001;  // @constdefine The ME must implement this notification
      RIL_SIMTKN_RADIOIMPLEMENTS_NONOTIFICATION   = $00000002;  // @constdefine The radio will implement and not give a notification to the ME
      RIL_SIMTKN_RADIOIMPLEMENTS_NOTIFICATION     = $00000003;  // @constdefine The radio will implement and give a notification to the ME that it was done
      RIL_SIMTKN_RADIOIMPLEMENTS_REQUESTMEINPUT   = $00000004;  // @constdefine The radio will implement, but requests information from the ME first
      RIL_SIMTKN_NOSUPPORT                        = $FFFFFFFF;  // @constdefine RIL knows of this type of command but does not support.

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Signal Strength | Special signal strength value
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_SIGNALSTRENGTH_UNKNOWN                  = $ffffffff;  // @constdefine Unknown signal strength

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Bit Error Rate | Special bit error rate value
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_BITERRORRATE_UNKNOWN                    = $ffffffff;  // @constdefine Unknown signal strength

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Remote Party | Remote party information validity types
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_REMOTEPARTYINFO_VALID                   = $00000001;  // @constdefine Information valid
      RIL_REMOTEPARTYINFO_WITHHELD                = $00000002;  // @constdefine Information withheld by other user
      RIL_REMOTEPARTYINFO_UNAVAILABLE             = $00000003;  // @constdefine Network unable to send info

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Disconnect Initiation | Disconnect initiation values
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_DISCINIT_NULL                           = $00000000;  // @constdefine Nothing
      RIL_DISCINIT_LOCAL                          = $00000001;  // @constdefine Local party initiated
      RIL_DISCINIT_REMOTE                         = $00000002;  // @constdefine Remote party initiated
      RIL_DISCINIT_NETWORKERROR                   = $00000003;  // @constdefine The call was disconnected due to a network error condition
      RIL_DISCINIT_BUSY                           = $00000004;  // @constdefine Busy

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Subaddress Type | Supplementary service data status values
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_SUPSVCDATASTATUS_NOINFOREQUIRED         = $00000001;  // @constdefine TBD
      RIL_SUPSVCDATASTATUS_FURTHERINFOREQUIRED    = $00000002;  // @constdefine TBD
      RIL_SUPSVCDATASTATUS_TERMINATED             = $00000003;  // @constdefine TBD
      RIL_SUPSVCDATASTATUS_OTHERCLIENTRESPONDED   = $00000004;  // @constdefine TBD
      RIL_SUPSVCDATASTATUS_UNSUPPORTED            = $00000005;  // @constdefine TBD
      RIL_SUPSVCDATASTATUS_TIMEOUT                = $00000006;  // @constdefine TBD
      RIL_SUPSVCDATASTATUS_ERROR                  = $00000007;  // @constdefine TBD

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants GPRS Protocol | GPRS Packet Protocols
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_GPRSPROTOCOL_UNKNOWN                    = $00000000;  // @constdefine Unknown
      RIL_GPRSPROTOCOL_X25                        = $00000001;  // @constdefine ITU-T/CCITT X.25 Layer 4
      RIL_GPRSPROTOCOL_IP                         = $00000002;  // @constdefine Internet Protocol (IETF STD 5)
      RIL_GPRSPROTOCOL_IHOSP                      = $00000004;  // @constdefine Internet Hosted Octet Stream Protocol
      RIL_GPRSPROTOCOL_PPP                        = $00000008;  // @constdefine Point to Point Protocol
      RIL_GPRSPROTOCOL_ALL                        = $0000000f;
      
// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants GPRS Protocol | GPRS L2 Protocols
//
// @comm None
//
// See GSM 07.07 10.1.6 for definitions
// -----------------------------------------------------------------------------
const
      RIL_GPRSL2PROTOCOL_UNKNOWN                  = $00000000;  // @constdefine
      RIL_GPRSL2PROTOCOL_NULL                     = $00000001;  // @constdefine none, for PDP type OSP:IHOSS
      RIL_GPRSL2PROTOCOL_PPP                      = $00000002;  // @constdefine Point-to-point protocol for a PDP such as IP
      RIL_GPRSL2PROTOCOL_PAD                      = $00000004;  // @constdefine character stream for X.25 character (triple X PAD) mode
      RIL_GPRSL2PROTOCOL_X25                      = $00000008;  // @constdefine X.25 L2 (LAPB) for X.25 packet mode
      RIL_GPRSL2PROTOCOL_ALL                      = $0000000f;


// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants GPRS Data Comp | GPRS Data Compression
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_GPRSDATACOMP_OFF                        = $00000001;  // @constdefine compression off
      RIL_GPRSDATACOMP_ON                         = $00000002;  // @constdefine compression off
      RIL_GPRSDATACOMP_ALL                        = $00000003;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants GPRS Header Comp | GPRS Header Compression
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_GPRSHEADERCOMP_OFF                      = $00000001;  // @constdefine compression off
      RIL_GPRSHEADERCOMP_ON                       = $00000002;  // @constdefine compression off
      RIL_GPRSHEADERCOMP_ALL                      = $00000003;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants GPRS Precedence Class | GPRS Precedence Class
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_GPRSPRECEDENCECLASS_SUBSCRIBED          = $00000001;  // @constdefine subscribed value stored in network
      RIL_GPRSPRECEDENCECLASS_HIGH                = $00000002;  // @constdefine high priority
      RIL_GPRSPRECEDENCECLASS_NORMAL              = $00000004;  // @constdefine normal priority
      RIL_GPRSPRECEDENCECLASS_LOW                 = $00000008;  // @constdefine low priority
      RIL_GPRSPRECEDENCECLASS_ALL                 = $0000000f;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants GPRS Delay Class | GPRS Delay Class
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_GPRSDELAYCLASS_SUBSCRIBED              = $00000001;  // @constdefine subscribed value stored in network
      RIL_GPRSDELAYCLASS_PREDICTIVE1             = $00000002;  // @constdefine see gsm 02.60
      RIL_GPRSDELAYCLASS_PREDICTIVE2             = $00000004;  // @constdefine see gsm 02.60
      RIL_GPRSDELAYCLASS_PREDICTIVE3             = $00000008;  // @constdefine see gsm 02.60
      RIL_GPRSDELAYCLASS_BESTEFFORT              = $00000010;  // @constdefine see gsm 02.60
      RIL_GPRSDELAYCLASS_ALL                     = $0000001f;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants GPRS Reliability Class | GPRS Reliability Class
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_GPRSRELIABILITYCLASS_SUBSCRIBED        = $00000001;  // @constdefine subscribed value stored in network
      RIL_GPRSRELIABILITYCLASS_1                 = $00000002;  // @constdefine see gsm 03.60
      RIL_GPRSRELIABILITYCLASS_2                 = $00000004;  // @constdefine see gsm 03.60
      RIL_GPRSRELIABILITYCLASS_3                 = $00000008;  // @constdefine see gsm 03.60
      RIL_GPRSRELIABILITYCLASS_4                 = $00000010;  // @constdefine see gsm 03.60
      RIL_GPRSRELIABILITYCLASS_5                 = $00000020;  // @constdefine see gsm 03.60
      RIL_GPRSRELIABILITYCLASS_ALL               = $0000003f;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants GPRS Class | GPRS Class
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_GPRSCLASS_UNKNOWN                       = $00000000;  // @constdefine GPRS class unknown
      RIL_GPRSCLASS_GSMANDGPRS                    = $00000001;  // @constdefine Simultaneous voice and GPRS data
      RIL_GPRSCLASS_GSMORGPRS                     = $00000002;  // @constdefine Simultaneous voice and GPRS traffic channel, one or other data
      RIL_GPRSCLASS_GSMORGPRS_EXCLUSIVE           = $00000004;  // @constdefine Either all voice or all GPRS, both traffic channels unmonitored
      RIL_GPRSCLASS_GPRSONLY                      = $00000008;  // @constdefine Only GPRS
      RIL_GPRSCLASS_GSMONLY                       = $00000010;  // @constdefine Only circuit switched voice and data
      RIL_GPRSCLASS_ALL                           = $0000001f;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants GPRS Peak Throughput Class | GPRS Peak Throughput Class
//
// @comm Constants represent bits per second
//
// -----------------------------------------------------------------------------
const
      RIL_PEAKTHRUCLASS_SUBSCRIBED               = $00000001;  // @constdefine subscribed value stored in network
      RIL_PEAKTHRUCLASS_8000                     = $00000002;  // @constdefine bits per second
      RIL_PEAKTHRUCLASS_16000                    = $00000004;  // @constdefine bits per second
      RIL_PEAKTHRUCLASS_32000                    = $00000008;  // @constdefine bits per second
      RIL_PEAKTHRUCLASS_64000                    = $00000010;  // @constdefine bits per second
      RIL_PEAKTHRUCLASS_128000                   = $00000020;  // @constdefine bits per second
      RIL_PEAKTHRUCLASS_256000                   = $00000040;  // @constdefine bits per second
      RIL_PEAKTHRUCLASS_512000                   = $00000080;  // @constdefine bits per second
      RIL_PEAKTHRUCLASS_1024000                  = $00000100;  // @constdefine bits per second
      RIL_PEAKTHRUCLASS_2048000                  = $00000200;  // @constdefine bits per second
      RIL_PEAKTHRUCLASS_ALL                      = $000003ff;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants GPRS Mean Throughput Class | GPRS Mean Throughput Class
//
// @comm Constants represent octets per hour
//
// -----------------------------------------------------------------------------
const
      RIL_MEANTHRUCLASS_SUBSCRIBED               = $00000001;  // @constdefine subscribed value stored in network
      RIL_MEANTHRUCLASS_100                      = $00000002;  // @constdefine 0.22 bits/second
      RIL_MEANTHRUCLASS_200                      = $00000004;  // @constdefine 0.44 bits/second
      RIL_MEANTHRUCLASS_500                      = $00000008;  // @constdefine 1.11 bits/second
      RIL_MEANTHRUCLASS_1000                     = $00000010;  // @constdefine 2.2 bits/second
      RIL_MEANTHRUCLASS_2000                     = $00000020;  // @constdefine 4.4 bits/second
      RIL_MEANTHRUCLASS_5000                     = $00000040;  // @constdefine 11.1 bits/second
      RIL_MEANTHRUCLASS_10000                    = $00000080;  // @constdefine 22 bits/second
      RIL_MEANTHRUCLASS_20000                    = $00000100;  // @constdefine 44 bits/second
      RIL_MEANTHRUCLASS_50000                    = $00000200;  // @constdefine 111 bits/second
      RIL_MEANTHRUCLASS_100000                   = $00000400;  // @constdefine 220 bits/second
      RIL_MEANTHRUCLASS_200000                   = $00000800;  // @constdefine 440 bits/second
      RIL_MEANTHRUCLASS_500000                   = $00001000;  // @constdefine 1,110 bits/second
      RIL_MEANTHRUCLASS_1000000                  = $00002000;  // @constdefine 2,200 bits/second
      RIL_MEANTHRUCLASS_2000000                  = $00004000;  // @constdefine 4,400 bits/second
      RIL_MEANTHRUCLASS_5000000                  = $00008000;  // @constdefine 11,100 bits/second
      RIL_MEANTHRUCLASS_10000000                 = $00010000;  // @constdefine 22,000 bits/second
      RIL_MEANTHRUCLASS_20000000                 = $00020000;  // @constdefine 44,000 bits/second
      RIL_MEANTHRUCLASS_50000000                 = $00040000;  // @constdefine 111,000 bits/second
      RIL_MEANTHRUCLASS_DONTCARE                 = $80000000;  // @constdefine best effort
      RIL_MEANTHRUCLASS_ALL                      = $8007ffff;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Complete Call Busy | Special value for all CCBS
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_CCBS_ALL                                  = $ffffffff;  // @constdefine All CCBS

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants GPRS SMS | Mobile Originated SMS Service Constants
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_MOSMSSERVICE_CIRCUIT                      = $00000001;  // @constdefine circuit switched
      RIL_MOSMSSERVICE_GPRS                         = $00000002;  // @constdefine GPRS
      RIL_MOSMSSERVICE_CIRCUITPREFERRED             = $00000004;  // @constdefine use both, circuit switched preferred
      RIL_MOSMSSERVICE_GPRSPREFERRED                = $00000008;  // @constdefine use both, GPRS preferred
      RIL_MOSMSSERVICE_ALL                          = $0000000f;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Password type | PIN or PUK password
//
// @comm Used to distiguish between a password that is a PIN vs PUK for RIL_ChangeLockingPassword
//
// -----------------------------------------------------------------------------
const
      RIL_PASSWORDTYPE_PIN                      = $00000001;  //@ The password type is a SIM PIN (editable password)
      RIL_PASSWORDTYPE_PUK                      = $00000002;  //@ The password type is a SIM PUK (Non-user editable)

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants System Capabilities
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_SYSTEMCAPS_NONE                         = $00000000; // @constdefine The system does not support any special capabilities.
      RIL_SYSTEMCAPS_VOICEDATA                    = $00000001; // @constdefine The system supports simultaneous voice+Data
      RIL_SYSTEMCAPS_ALL                          = $00000001; // @constdefine The system supports all special capabilities.

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Caps Type | Capability types
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_CAPSTYPE_DIAL                           = $00000001;  // @constdefine TBD
      RIL_CAPSTYPE_DTMFDURATIONRANGE              = $00000002;  // @constdefine TBD
      RIL_CAPSTYPE_CALLMGTCMDS                    = $00000003;  // @constdefine TBD
      RIL_CAPSTYPE_BEARERSERVICE                  = $00000004;  // @constdefine TBD
      RIL_CAPSTYPE_RLP                            = $00000005;  // @constdefine TBD
      RIL_CAPSTYPE_EQUIPMENTSTATES                = $00000006;  // @constdefine TBD
      RIL_CAPSTYPE_PBSTORELOCATIONS               = $00000007;  // @constdefine TBD
      RIL_CAPSTYPE_PBINDEXRANGE                   = $00000008;  // @constdefine TBD
      RIL_CAPSTYPE_PBENTRYLENGTH                  = $00000009;  // @constdefine TBD
      RIL_CAPSTYPE_MSGSERVICETYPES                = $0000000a;  // @constdefine TBD
      RIL_CAPSTYPE_MSGMEMORYLOCATIONS             = $0000000b;  // @constdefine TBD
      RIL_CAPSTYPE_BROADCASTMSGLANGS              = $0000000c;  // @constdefine TBD
      RIL_CAPSTYPE_MSGCONFIGINDEXRANGE            = $0000000d;  // @constdefine TBD
      RIL_CAPSTYPE_MSGSTATUSVALUES                = $0000000e;  // @constdefine TBD
      RIL_CAPSTYPE_PREFOPINDEXRANGE               = $0000000f;  // @constdefine TBD
      RIL_CAPSTYPE_LOCKFACILITIES                 = $00000010;  // @constdefine TBD
      RIL_CAPSTYPE_LOCKINGPWDLENGTHS              = $00000011;  // @constdefine TBD
      RIL_CAPSTYPE_BARRTYPES                      = $00000012;  // @constdefine TBD
      RIL_CAPSTYPE_BARRINGPWDLENGTHS              = $00000013;  // @constdefine TBD
      RIL_CAPSTYPE_FORWARDINGREASONS              = $00000014;  // @constdefine TBD
      RIL_CAPSTYPE_INFOCLASSES                    = $00000015;  // @constdefine TBD
      RIL_CAPSTYPE_HSCSD                          = $00000016;  // @constdefine TBD
      RIL_CAPSTYPE_SIMTOOLKITNOTIFICATIONS        = $00000017;  // @constdefine TBD
      RIL_CAPSTYPE_GPRSCLASS                      = $00000018;  // @constdefine TBD
      RIL_CAPSTYPE_GPRSCONTEXT                    = $00000019;  // @constdefine TBD
      RIL_CAPSTYPE_GPRSQOS                        = $0000001a;  // @constdefine TBD
      RIL_CAPSTYPE_GPRSQOSMIN                     = $0000001b;  // @constdefine TBD
      RIL_CAPSTYPE_GPRSMOSMS                      = $0000001c;  // @constdefine TBD
      RIL_CAPSTYPE_DATACOMPRESSION                = $0000001d;  // @constdefine TBD
      RIL_CAPSTYPE_ERRORCORRECTION                = $0000001e;  // @constdefine TBD
      RIL_CAPSTYPE_SIGNALQUALITYIMPLEMENTATION    = $0000001f;  // @constdefine TBD

      RIL_CAPSTYPE_SIMSUPPORT                     = $00000020;  // @constdefine TBD
      RIL_CAPSTYPE_CALLPROGRESSNOTIFICATION       = $00000021;  // @constdefine TBD
      RIL_CAPSTYPE_NOTIFICATIONIMPLEMENTATION     = $00000022;  // @constdefine TBD
      RIL_CAPSTYPE_NITZNOTIFICATION               = $00000023;  // @constdefine TBD

      RIL_CAPSTYPE_ARG_SMALLEST                   = RIL_CAPSTYPE_DIAL;
      RIL_CAPSTYPE_ARG_LARGEST                    = RIL_CAPSTYPE_NITZNOTIFICATION;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Caps Call Type | NITZ type capabilities
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_CAPS_NITZ_DISABLED                 = $00000001;      // @constdefine TBD
      RIL_CAPS_NITZ_ENABLED                  = $00000002;      // @constdefine TBD

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Caps Call Type | Call type capabilities
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_CAPS_CALLTYPE_VOICE                     = $00000001;  // @constdefine TBD
      RIL_CAPS_CALLTYPE_DATA                      = $00000002;  // @constdefine TBD
      RIL_CAPS_CALLTYPE_FAX                       = $00000004;  // @constdefine TBD
      RIL_CAPS_CALLTYPE_PTT                       = $00000008;  // @constdefine TBD
      RIL_CAPS_CALLTYPE_VT                        = $00000010;  // @constdefine TBD

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Caps Dialing Option | Dialing options capabilities
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_CAPS_DIALOPT_RESTRICTID                 = RIL_DIALOPT_RESTRICTID;    // @constdefine TBD
      RIL_CAPS_DIALOPT_PRESENTID                  = RIL_DIALOPT_PRESENTID;     // @constdefine TBD
      RIL_CAPS_DIALOPT_CLOSEDGROUP                = RIL_DIALOPT_CLOSEDGROUP;   // @constdefine TBD

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Caps Call Mgmt | Call management command capabilities
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_CAPS_CALLCMD_RELEASEHELD                = $00000001;  // @constdefine TBD
      RIL_CAPS_CALLCMD_RELEASEACTIVE_ACCEPTHELD   = $00000002;  // @constdefine TBD
      RIL_CAPS_CALLCMD_RELEASECALL                = $00000004;  // @constdefine TBD
      RIL_CAPS_CALLCMD_HOLDACTIVE_ACCEPTHELD      = $00000008;  // @constdefine TBD
      RIL_CAPS_CALLCMD_HOLDALLBUTONE              = $00000010;  // @constdefine TBD
      RIL_CAPS_CALLCMD_ADDHELDTOCONF              = $00000020;  // @constdefine TBD
      RIL_CAPS_CALLCMD_ADDHELDTOCONF_DISCONNECT   = $00000040;  // @constdefine TBD
      RIL_CAPS_CALLCMD_INVOKECCBS                 = $00000080;  // @constdefine TBD

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Caps Bearer Speed1 | Bearer service speed capabilities (first set)
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_CAPS_SPEED1_AUTO                        = $00000001;  // @constdefine TBD
      RIL_CAPS_SPEED1_300_V21                     = $00000002;  // @constdefine TBD
      RIL_CAPS_SPEED1_300_V110                    = $00000004;  // @constdefine TBD
      RIL_CAPS_SPEED1_1200_V22                    = $00000008;  // @constdefine TBD
      RIL_CAPS_SPEED1_1200_75_V23                 = $00000010;  // @constdefine TBD
      RIL_CAPS_SPEED1_1200_V110                   = $00000020;  // @constdefine TBD
      RIL_CAPS_SPEED1_1200_V120                   = $00000040;  // @constdefine TBD
      RIL_CAPS_SPEED1_2400_V22BIS                 = $00000080;  // @constdefine TBD
      RIL_CAPS_SPEED1_2400_V26TER                 = $00000100;  // @constdefine TBD
      RIL_CAPS_SPEED1_2400_V110                   = $00000200;  // @constdefine TBD
      RIL_CAPS_SPEED1_2400_V120                   = $00000400;  // @constdefine TBD
      RIL_CAPS_SPEED1_4800_V32                    = $00000800;  // @constdefine TBD
      RIL_CAPS_SPEED1_4800_V110                   = $00001000;  // @constdefine TBD
      RIL_CAPS_SPEED1_4800_V120                   = $00002000;  // @constdefine TBD
      RIL_CAPS_SPEED1_9600_V32                    = $00004000;  // @constdefine TBD
      RIL_CAPS_SPEED1_9600_V34                    = $00008000;  // @constdefine TBD
      RIL_CAPS_SPEED1_9600_V110                   = $00010000;  // @constdefine TBD
      RIL_CAPS_SPEED1_9600_V120                   = $00020000;  // @constdefine TBD
      RIL_CAPS_SPEED1_14400_V34                   = $00040000;  // @constdefine TBD
      RIL_CAPS_SPEED1_14400_V110                  = $00080000;  // @constdefine TBD
      RIL_CAPS_SPEED1_14400_V120                  = $00100000;  // @constdefine TBD
      RIL_CAPS_SPEED1_19200_V34                   = $00200000;  // @constdefine TBD
      RIL_CAPS_SPEED1_19200_V110                  = $00400000;  // @constdefine TBD
      RIL_CAPS_SPEED1_19200_V120                  = $00800000;  // @constdefine TBD
      RIL_CAPS_SPEED1_28800_V34                   = $01000000;  // @constdefine TBD
      RIL_CAPS_SPEED1_28800_V110                  = $02000000;  // @constdefine TBD
      RIL_CAPS_SPEED1_28800_V120                  = $04000000;  // @constdefine TBD
      RIL_CAPS_SPEED1_38400_V110                  = $08000000;  // @constdefine TBD
      RIL_CAPS_SPEED1_38400_V120                  = $10000000;  // @constdefine TBD
      RIL_CAPS_SPEED1_48000_V110                  = $20000000;  // @constdefine TBD
      RIL_CAPS_SPEED1_48000_V120                  = $40000000;  // @constdefine TBD
      RIL_CAPS_SPEED1_56000_V110                  = $80000000;  // @constdefine TBD

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Caps Bearer Speed2 | Bearer service speed capabilities (second set)
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_CAPS_SPEED2_56000_V120                  = $00000001;  // @constdefine TBD
      RIL_CAPS_SPEED2_56000_TRANSP                = $00000002;  // @constdefine TBD
      RIL_CAPS_SPEED2_64000_TRANSP                = $00000004;  // @constdefine TBD
      RIL_CAPS_SPEED2_32000_PIAFS32K              = $00000008;  // @constdefine TBD
      RIL_CAPS_SPEED2_64000_PIAFS64K              = $00000010;  // @constdefine TBD
      RIL_CAPS_SPEED2_28800_MULTIMEDIA            = $00000020;  // @constdefine TBD
      RIL_CAPS_SPEED2_32000_MULTIMEDIA            = $00000040;  // @constdefine TBD
      RIL_CAPS_SPEED2_33600_MULTIMEDIA            = $00000080;  // @constdefine TBD
      RIL_CAPS_SPEED2_56000_MULTIMEDIA            = $00000100;  // @constdefine TBD
      RIL_CAPS_SPEED2_64000_MULTIMEDIA            = $00000200;  // @constdefine TBD

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Caps Bearer Name | Bearer service name capabilities
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_CAPS_BSVCNAME_DATACIRCUIT_ASYNC_UDI_MODEM   = $00000001;  // @constdefine TBD
      RIL_CAPS_BSVCNAME_DATACIRCUIT_SYNC_UDI_MODEM    = $00000002;  // @constdefine TBD
      RIL_CAPS_BSVCNAME_PADACCESS_ASYNC_UDI           = $00000004;  // @constdefine TBD
      RIL_CAPS_BSVCNAME_PACKETACCESS_SYNC_UDI         = $00000008;  // @constdefine TBD
      RIL_CAPS_BSVCNAME_DATACIRCUIT_ASYNC_RDI         = $00000010;  // @constdefine TBD
      RIL_CAPS_BSVCNAME_DATACIRCUIT_SYNC_RDI          = $00000020;  // @constdefine TBD
      RIL_CAPS_BSVCNAME_PADACCESS_ASYNC_RDI           = $00000040;  // @constdefine TBD
      RIL_CAPS_BSVCNAME_PACKETACCESS_SYNC_RDI         = $00000080;  // @constdefine TBD

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Bearer CE | Bearer service connection element capabilities
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_CAPS_BSVCCE_TRANSPARENT                 = $00000001;  // @constdefine TBD
      RIL_CAPS_BSVCCE_NONTRANSPARENT              = $00000002;  // @constdefine TBD
      RIL_CAPS_BSVCCE_BOTH_TRANSPARENT            = $00000004;  // @constdefine TBD
      RIL_CAPS_BSVCCE_BOTH_NONTRANSPARENT         = $00000008;  // @constdefine TBD

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Caps Equipment | Equipment state capabilities
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_CAPS_EQSTATE_MINIMUM                    = $00000001;  // @constdefine TBD
      RIL_CAPS_EQSTATE_FULL                       = $00000002;  // @constdefine TBD
      RIL_CAPS_EQSTATE_DISABLETX                  = $00000004;  // @constdefine TBD
      RIL_CAPS_EQSTATE_DISABLERX                  = $00000008;  // @constdefine TBD
      RIL_CAPS_EQSTATE_DISABLETXANDRX             = $00000010;  // @constdefine TBD

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Caps Phonebook | Phonebook storage location capabilities
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_CAPS_PBLOC_SIMEMERGENCY                 = $00000001;  // @constdefine TBD
      RIL_CAPS_PBLOC_SIMFIXDIALING                = $00000002;  // @constdefine TBD
      RIL_CAPS_PBLOC_SIMLASTDIALING               = $00000004;  // @constdefine TBD
      RIL_CAPS_PBLOC_OWNNUMBERS                   = $00000008;  // @constdefine TBD
      RIL_CAPS_PBLOC_SIMPHONEBOOK                 = $00000010;  // @constdefine TBD

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Caps Message Service | Message service type capabilities
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_CAPS_MSGSVCTYPE_PHASE2                  = $00000001;  // @constdefine TBD
      RIL_CAPS_MSGSVCTYPE_PHASE2PLUS              = $00000002;  // @constdefine TBD

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Caps Message Storage | Message storage location capabilities
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_CAPS_MSGLOC_BROADCAST                   = $00000001;  // @constdefine TBD
      RIL_CAPS_MSGLOC_SIM                         = $00000002;  // @constdefine TBD
      RIL_CAPS_MSGLOC_STATUSREPORT                = $00000004;  // @constdefine TBD

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Caps DCS Language | Message broadcast data coding scheme language capabilities
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_CAPS_DCSLANG_GERMAN                     = RIL_DCSLANG_GERMAN;     // @constdefine TBD
      RIL_CAPS_DCSLANG_ENGLISH                    = RIL_DCSLANG_ENGLISH;    // @constdefine TBD
      RIL_CAPS_DCSLANG_ITALIAN                    = RIL_DCSLANG_ITALIAN;    // @constdefine TBD
      RIL_CAPS_DCSLANG_FRENCH                     = RIL_DCSLANG_FRENCH;     // @constdefine TBD
      RIL_CAPS_DCSLANG_SPANISH                    = RIL_DCSLANG_SPANISH;    // @constdefine TBD
      RIL_CAPS_DCSLANG_DUTCH                      = RIL_DCSLANG_DUTCH;      // @constdefine TBD
      RIL_CAPS_DCSLANG_SWEDISH                    = RIL_DCSLANG_SWEDISH;    // @constdefine TBD
      RIL_CAPS_DCSLANG_DANISH                     = RIL_DCSLANG_DANISH;     // @constdefine TBD
      RIL_CAPS_DCSLANG_PORTUGUESE                 = RIL_DCSLANG_PORTUGUESE; // @constdefine TBD
      RIL_CAPS_DCSLANG_FINNISH                    = RIL_DCSLANG_FINNISH;    // @constdefine TBD
      RIL_CAPS_DCSLANG_NORWEGIAN                  = RIL_DCSLANG_NORWEGIAN;  // @constdefine TBD
      RIL_CAPS_DCSLANG_GREEK                      = RIL_DCSLANG_GREEK;      // @constdefine TBD
      RIL_CAPS_DCSLANG_TURKISH                    = RIL_DCSLANG_TURKISH;    // @constdefine TBD
      RIL_CAPS_DCSLANG_HUNGARIAN                  = RIL_DCSLANG_HUNGARIAN;  // @constdefine TBD
      RIL_CAPS_DCSLANG_POLISH                     = RIL_DCSLANG_POLISH;     // @constdefine TBD
      RIL_CAPS_DCSLANG_CZECH                      = RIL_DCSLANG_CZECH;      // @constdefine TBD

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Caps Message Status | Message status capabilities
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_CAPS_MSGSTATUS_RECUNREAD                = $00000001;  // @constdefine TBD
      RIL_CAPS_MSGSTATUS_RECREAD                  = $00000002;  // @constdefine TBD
      RIL_CAPS_MSGSTATUS_STOUNSENT                = $00000004;  // @constdefine TBD
      RIL_CAPS_MSGSTATUS_STOSENT                  = $00000008;  // @constdefine TBD

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Caps SIM | SIM capabilities
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_CAPS_SIM_NONE                  = $00000000;      // @constdefine TBD
      RIL_CAPS_SIM_BASIC                 = $00000001;      // @constdefine TBD

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Caps Phone Lock | Locking faciliy capabilities
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_CAPS_LOCKFACILITY_NONE                  = $00000000;  // @constdefine TBD
      RIL_CAPS_LOCKFACILITY_CNTRL                 = $00000001;  // @constdefine TBD
      RIL_CAPS_LOCKFACILITY_PH_SIM                = $00000002;  // @constdefine TBD
      RIL_CAPS_LOCKFACILITY_PH_FSIM               = $00000004;  // @constdefine TBD
      RIL_CAPS_LOCKFACILITY_SIM                   = $00000008;  // @constdefine TBD
      RIL_CAPS_LOCKFACILITY_SIM_PIN2              = $00000010;  // @constdefine TBD
      RIL_CAPS_LOCKFACILITY_SIM_FIXEDIALING       = $00000020;  // @constdefine TBD
      RIL_CAPS_LOCKFACILITY_NETWORKPERS           = $00000040;  // @constdefine TBD
      RIL_CAPS_LOCKFACILITY_NETWORKSUBPERS        = $00000080;  // @constdefine TBD
      RIL_CAPS_LOCKFACILITY_SERVICEPROVPERS       = $00000100;  // @constdefine TBD
      RIL_CAPS_LOCKFACILITY_CORPPERS              = $00000200;  // @constdefine TBD

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Caps Call Barr | Call barring capabilities
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_CAPS_BARRTYPE_ALLOUTGOING               = $00000001;  // @constdefine TBD
      RIL_CAPS_BARRTYPE_OUTGOINGINT               = $00000002;  // @constdefine TBD
      RIL_CAPS_BARRTYPE_OUTGOINGINTEXTOHOME       = $00000004;  // @constdefine TBD
      RIL_CAPS_BARRTYPE_ALLINCOMING               = $00000008;  // @constdefine TBD
      RIL_CAPS_BARRTYPE_INCOMINGROAMING           = $00000010;  // @constdefine TBD
      RIL_CAPS_BARRTYPE_INCOMINGNOTINSIM          = $00000020;  // @constdefine TBD
      RIL_CAPS_BARRTYPE_ALLBARRING                = $00000040;  // @constdefine TBD
      RIL_CAPS_BARRTYPE_ALLOUTGOINGBARRING        = $00000080;  // @constdefine TBD
      RIL_CAPS_BARRTYPE_ALLINCOMINGBARRING        = $00000100;  // @constdefine TBD

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Caps Forwarding | Forwarding reason capabilities
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_CAPS_FWDREASON_UNCONDITIONAL            = $00000001;  // @constdefine TBD
      RIL_CAPS_FWDREASON_MOBILEBUSY               = $00000002;  // @constdefine TBD
      RIL_CAPS_FWDREASON_NOREPLY                  = $00000004;  // @constdefine TBD
      RIL_CAPS_FWDREASON_UNREACHABLE              = $00000008;  // @constdefine TBD
      RIL_CAPS_FWDREASON_ALLFORWARDING            = $00000010;  // @constdefine TBD
      RIL_CAPS_FWDREASON_ALLCONDITIONAL           = $00000020;  // @constdefine TBD

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Caps Info Class | Telephony information class capabilities
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_CAPS_INFOCLASS_VOICE                    = RIL_INFOCLASS_VOICE;     // @constdefine TBD
      RIL_CAPS_INFOCLASS_DATA                     = RIL_INFOCLASS_DATA;     // @constdefine TBD
      RIL_CAPS_INFOCLASS_FAX                      = RIL_INFOCLASS_FAX;     // @constdefine TBD
      RIL_CAPS_INFOCLASS_SMS                      = RIL_INFOCLASS_SMS;     // @constdefine TBD
      RIL_CAPS_INFOCLASS_DATACIRCUITSYNC          = RIL_INFOCLASS_DATACIRCUITSYNC;     // @constdefine TBD
      RIL_CAPS_INFOCLASS_DATACIRCUITASYNC         = RIL_INFOCLASS_DATACIRCUITASYNC;     // @constdefine TBD
      RIL_CAPS_INFOCLASS_PACKETACCESS             = RIL_INFOCLASS_PACKETACCESS;     // @constdefine TBD
      RIL_CAPS_INFOCLASS_PADACCESS                = RIL_INFOCLASS_PADACCESS;     // @constdefine TBD

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Caps HSCSD Traffic Channel | HSCSD traffic channel coding capabilities
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_CAPS_HSCSDCODING_4800_FULLRATE          = RIL_HSCSDCODING_4800_FULLRATE;     // @constdefine TBD
      RIL_CAPS_HSCSDCODING_9600_FULLRATE          = RIL_HSCSDCODING_9600_FULLRATE;     // @constdefine TBD
      RIL_CAPS_HSCSDCODING_14400_FULLRATE         = RIL_HSCSDCODING_14400_FULLRATE;    // @constdefine TBD

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Caps HSCSD Air Interface | HSCSD air interface user rate capabilities
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_CAPS_HSCSDAIURATE_9600                  = $00000001;  // @constdefine TBD
      RIL_CAPS_HSCSDAIURATE_14400                 = $00000002;  // @constdefine TBD
      RIL_CAPS_HSCSDAIURATE_19200                 = $00000004;  // @constdefine TBD
      RIL_CAPS_HSCSDAIURATE_28800                 = $00000008;  // @constdefine TBD
      RIL_CAPS_HSCSDAIURATE_38400                 = $00000010;  // @constdefine TBD
      RIL_CAPS_HSCSDAIURATE_43200                 = $00000020;  // @constdefine TBD
      RIL_CAPS_HSCSDAIURATE_57600                 = $00000040;  // @constdefine TBD

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Caps SIM Toolkit | SIM Toolkit notification capabilities
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_CAPS_NOTIFY_SIMTOOLKITCMD               = $00000001;  // @constdefine TBD
      RIL_CAPS_NOTIFY_SIMTOOLKITCALLSETUP         = $00000002;  // @constdefine TBD
      RIL_CAPS_NOTIFY_SIMTOOLKITEVENT             = $00000004;  // @constdefine TBD

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Caps Signal Implemetation Quality | Signal Quality Implemetation Capabilities
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_CAPS_SIGNALQUALITY_NOTIFICATION         = $00000001;  // @constdefine The Radio Module can deliver unsolicited Signal Quality Notifications
      RIL_CAPS_SIGNALQUALITY_POLLING              = $00000002;  // @constdefine The Higher layers can poll the radio module in order to get the Signal Quality

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Caps Radio Notificaiton Implemetation | Radio Notificaiton Implemetation Capabilities
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_CAPS_NOTIFICATION_CALLLIST              = $00000001;      // @constdefine The Radio Module can deliver unsolicited notifications of call list information
      RIL_CAPS_NOTIFICATION_LINESTATUS            = $00000002;      // @constdefine The Radio Module can deliver unsolicited notifications of line status
      RIL_CAPS_NOTIFICATION_SIGNALQUALITY         = $00000004;      // @constdefine The Radio Module can deliver unsolicited notifications of signal quality
      RIL_CAPS_NOTIFICATION_BEARERSERVICEOPTIONS  = $00000008;      // @constdefine The Radio Module can deliver unsolicited notifications of bearer service options
      RIL_CAPS_NOTIFICATION_DATACOMPRESSION       = $00000010;      // @constdefine The Radio Module can deliver unsolicited notifications of data compression
      RIL_CAPS_NOTIFICATION_EQUIPMENTSTATE        = $00000020;      // @constdefine The Radio Module can deliver unsolicited notifications of equipment state
      RIL_CAPS_NOTIFICATION_ERRORCORRECTION       = $00000040;      // @constdefine The Radio Module can deliver unsolicited notifications of error correction
      RIL_CAPS_NOTIFICATION_GPRSADDRESS           = $00000080;      // @constdefine The Radio Module can deliver unsolicited notifications of GPRS address
      RIL_CAPS_NOTIFICATION_GPRSATTACHED          = $00000100;      // @constdefine The Radio Module can deliver unsolicited notifications of GPRS attached status
      RIL_CAPS_NOTIFICATION_GPRSCONTEXTLIST       = $00000200;      // @constdefine The Radio Module can deliver unsolicited notifications of GPRS context list
      RIL_CAPS_NOTIFICATION_GPRSCONTEXTACTIVATEDLIST      = $00000400;     // @constdefine The Radio Module can deliver unsolicited notifications of GPRS context activated list
      RIL_CAPS_NOTIFICATION_GPRSREGISTRATIONSTATUS        = $00000800;     // @constdefine The Radio Module can deliver unsolicited notifications of GPRS registration status
      RIL_CAPS_NOTIFICATION_MINIMUMQUALITYOFSERVICELIST   = $00001000;     // @constdefine The Radio Module can deliver unsolicited notifications of minimum QoS list
      RIL_CAPS_NOTIFICATION_REGISTRATIONSTATUS            = $00002000;     // @constdefine The Radio Module can deliver unsolicited notifications of registration status
      RIL_CAPS_NOTIFICATION_REQUESTEDQUALITYOFSERVICELIST = $00004000;     // @constdefine The Radio Module can deliver unsolicited notifications of requested QoS list
      RIL_CAPS_NOTIFICATION_RLPOPTIONS                    = $00008000;     // @constdefine The Radio Module can deliver unsolicited notifications of RLP options


// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Caps Radio Notificaiton Implemetation Registry Key | Radio Notificaiton Implemetation Capabilities Registry Key
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_CAPS_NOTIFICATION_KEY                    = 'Comm\Cellular\RIL';
      RIL_CAPS_NOTIFICATION_IMPL_PARAMS            = 'CapsNotificationImplParams';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Maximum size | Maximum size for a call list returned from radio
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_CALLLIST_MAXSIZE                         = 10;


// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Maximum size | Maximum size for a GPRS context list returned from radio
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_GPRSCONTEXTLIST_MAXSIZE                  = 10;


// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Maximum size | Maximum size for a GPRS context activated list returned from radio
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_GPRSCONTEXTACTIVATEDLIST_MAXSIZE         = 10;


// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Maximum size | Maximum size for a QoS list returned from radio
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_QOSLIST_MAXSIZE                          = 10;



// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants Maximum lengths | Maximum lengths for string parameters
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      MAXLENGTH_ADDRESS                           = 256;     // @constdefine 256
      MAXLENGTH_SUBADDR                           = 256;     // @constdefine 256
      MAXLENGTH_DESCRIPTION                       = 256;     // @constdefine 256
      MAXLENGTH_OPERATOR                          = 32;      // @constdefine 32
      MAXLENGTH_OPERATOR_LONG                     = 32;      // @constdefine 32
      MAXLENGTH_OPERATOR_SHORT                    = 16;      // @constdefine 16
      MAXLENGTH_OPERATOR_NUMERIC                  = 16;      // @constdefine 16
      MAXLENGTH_OPERATOR_COUNTRY_CODE             = 8;       // @constdefine 8
      MAXLENGTH_SERVCTR                           = 256;     // @constdefine 256
      MAXLENGTH_PASSWORD                          = 256;     // @constdefine 256
      MAXLENGTH_ERRSHORT                          = 256;     // @constdefine 256
      MAXLENGTH_ERRLONG                           = 256;     // @constdefine 256
      MAXLENGTH_EQUIPINFO                         = 128;     // @constdefine 128
      MAXLENGTH_PHONEBOOKADDR                     = 256;     // @constdefine 256
      MAXLENGTH_PHONEBOOKTEXT                     = 256;     // @constdefine 256
      MAXLENGTH_CURRENCY                          = 256;     // @constdefine 256
      MAXLENGTH_AREAID                            = 256;     // @constdefine 256
      MAXLENGTH_CELLID                            = 256;     // @constdefine 256
      MAXLENGTH_HDR                               = 256;     // @constdefine 256
      MAXLENGTH_MSG                               = 256;     // @constdefine 256
      MAXLENGTH_CMD                               = 256;     // @constdefine 256
      MAXLENGTH_MSGIDS                            = 256;     // @constdefine 256
      MAXLENGTH_USERID                            = 256;     // @constdefine 256
      MAXLENGTH_DTMF                              = 256;     // @constdefine 256
      MAXLENGTH_GPRSADDRESS                       = 64;      // @constdefine 64
      MAXLENGTH_GPRSACCESSPOINTNAME               = 64;      // @constdefine 64
      MAXLENGTH_BCCH                              = 48;      // @constdefine 48
      MAXLENGTH_NMR                               = 16;      // @constdefine 16
      MAXLENGTH_ATR                               = 33;      // @constdefine 33
      MAXLENGTH_RADIOLOG                          = 128;     // @constdefine 128
      MAXLENGTH_CALLTYPEKEY                       = 128;     // @constdefine 128
      MAXLENGTH_MMISTRING                         = 256;     // @constdefine 256

//
// Registry path and variable defintions
//
const
      RIL_REGISTRY_ROOT = HKEY_LOCAL_MACHINE;
      RIL_SECURE_REGISTRY_KEY = 'Comm\Cellular\Ril';
      RIL_REGISTRY_VALUE_PDPCONTEXTS = 'Contexts';
      RIL_REGISTRY_VALUE_PACKETIO = 'Packet';
      RIL_REGISTRY_VALUE_WAITFORGPRSDEACTRESPONSE = 'WaitForDeactRsp';
      RIL_REGISTRY_VALUE_EONSENABLED = 'EONSEnabled';

//
// Event logging registry definitions
//
const
      CELLULAR_REGISTRYKEY_EVENTLOGGING = 'Comm\Cellular\EventLogging';
      CELLULAR_REGISTRYVALUE_EVENTLOGGING_ENABLED = 'Enabled';
      CELLULAR_REGISTRYVALUE_EVENTLOGGING_SOURCENAME = 'Source';

//
// Data types
//

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILADDRESS | Represents a phone number
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     riladdress_tag = record
       cbSize:DWORD;                           // @field structure size in bytes
       dwParams:DWORD;                         // @field indicates valid parameters
       dwType:DWORD;                           // @field type of number
       dwNumPlan:DWORD;                        // @field numbering plan
       wszAddress:array[0..MAXLENGTH_ADDRESS-1] of WCHAR;    // @field address (min 3, max 43)
     end;
     RILADDRESS = riladdress_tag;
     LPRILADDRESS = ^riladdress_tag;



// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILSUBADDRESS | The subaddress of a called party
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rilsubaddress_tag = record
       cbSize:DWORD;                           // @field structure size in bytes
       dwParams:DWORD;                         // @field indicates valid parameters
       dwType:DWORD;                           // @field type of subaddress
       wszSubAddress:array[0..MAXLENGTH_SUBADDR-1] of WCHAR; // @field subaddress (min 2, max 23)
     end;
     RILSUBADDRESS = rilsubaddress_tag;
     LPRILSUBADDRESS = ^rilsubaddress_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILSERIALPORTSTATS | Statistics of the virtual serial port
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rilserialportstats_tag = record
       cbSize:DWORD;                           // @field structure size in bytes
       dwParams:DWORD;                         // @field indicates valid parameters
       dwReadBitsPerSecond:DWORD;              // @field bit rate for reading data
       dwWrittenBitsPerSecond:DWORD;           // @field bit rate for writing data
     end;
     RILSERIALPORTSTATS = rilserialportstats_tag;
     LPRILSERIALPORTSTATS = ^rilserialportstats_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILSUBSCRIBERINFO | A phone number assigned to the user
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rilsubscriberinfo_tag = record
       cbSize:DWORD;                           // @field structure size in bytes
       dwParams:DWORD;                         // @field indicates valid parameters
       raAddress:RILADDRESS;                   // @field the assigned address
       wszDescription:array[0..MAXLENGTH_DESCRIPTION-1] of WCHAR; // @field text relating to this subscriber
       dwSpeed:DWORD;                          // @field data rate related to this number
       dwService:DWORD;                        // @field the service related to this number
       dwITC:DWORD;                            // @field information transfer capability
       dwAddressId:DWORD;                      // @field the address ID of this number
     end;
     RILSUBSCRIBERINFO = rilsubscriberinfo_tag;
     LPRILSUBSCRIBERINFO = ^rilsubscriberinfo_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILOPERATORNAMES | The different representations of an operator
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     riloperatornames_tag = record
       cbSize:DWORD;                           // @field structure size in bytes
       dwParams:DWORD;                         // @field indicates valid parameters
       szLongName:array[0..MAXLENGTH_OPERATOR_LONG-1] of AnsiChar;   // @field long representation (max 16 characters)
       szShortName:array[0..MAXLENGTH_OPERATOR_SHORT-1] of AnsiChar; // @field short representation (max 8 characters)
       szNumName:array[0..MAXLENGTH_OPERATOR_NUMERIC-1] of AnsiChar; // @field numeric representation (3 digit country code & 2 digit network code)
       szCountryCode:array[0..MAXLENGTH_OPERATOR_COUNTRY_CODE-1] of AnsiChar; // @field 2 character ISO 3166 country repesentation of the MCC
     end;
     RILOPERATORNAMES = riloperatornames_tag;
     LPRILOPERATORNAMES = ^riloperatornames_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILOPERATORINFO | Indicates status of a particular operator
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     riloperatorinfo_tag = record
       cbSize:DWORD;                           // @field structure size in bytes
       dwParams:DWORD;                         // @field indicates valid parameters
       dwIndex:DWORD;                          // @field index, if applicable
       dwStatus:DWORD;                         // @field registration status, if applicable
       ronNames:RILOPERATORNAMES;              // @field representations of an operator
     end;
     RILOPERATORINFO = riloperatorinfo_tag;
     LPRILOPERATORINFO = ^riloperatorinfo_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILCALLERIDSETTINGS | Caller ID settings
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rilcalleridsettings_tag = record
       cbSize:DWORD;                           // @field structure size in bytes
       dwParams:DWORD;                         // @field indicates valid parameters
       dwStatus:DWORD;                         // @field activation status
       dwProvisioning:DWORD;                   // @field network provisioning status
     end;
     RILCALLERIDSETTINGS = rilcalleridsettings_tag;
     LPRILCALLERIDSETTINGS = ^rilcalleridsettings_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILHIDEIDSETTINGS | Hide ID settings
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rilhideidsettings_tag = record
       cbSize:DWORD;                           // @field structure size in bytes
       dwParams:DWORD;                         // @field indicates valid parameters
       dwStatus:DWORD;                         // @field activation status
       dwProvisioning:DWORD;                   // @field network provisioning status
     end;
     RILHIDEIDSETTINGS = rilhideidsettings_tag;
     LPRILHIDEIDSETTINGS = ^rilhideidsettings_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILDIALEDIDSETTINGS | Dialed ID settings
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rildialedidsettings_tag = record
       cbSize:DWORD;                           // @field structure size in bytes
       dwParams:DWORD;                         // @field indicates valid parameters
       dwStatus:DWORD;                         // @field activation status
       dwProvisioning:DWORD;                   // @field network provisioning status
     end;
     RILDIALEDIDSETTINGS = rildialedidsettings_tag;
     LPRILDIALEDIDSETTINGS = ^rildialedidsettings_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILHIDECONNECTEDIDSETTINGS | Hide Connected ID settings
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rilhideconnectedidsettings_tag = record
       cbSize:DWORD;                           // @field structure size in bytes
       dwParams:DWORD;                         // @field indicates valid parameters
       dwStatus:DWORD;                         // @field activation status
       dwProvisioning:DWORD;                   // @field network provisioning status
     end;
     RILHIDECONNECTEDIDSETTINGS = rilhideconnectedidsettings_tag;
     LPRILHIDECONNECTEDIDSETTINGS = ^rilhideconnectedidsettings_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILCLOSEDGROUPSETTINGS | Close user group settings
//
// @comm This feature is not used and is untested.
//
// -----------------------------------------------------------------------------
type
     rilclosedgroupsettings_tag = record
       cbSize:DWORD;                           // @field structure size in bytes
       dwParams:DWORD;                         // @field indicates valid parameters
       dwStatus:DWORD;                         // @field activation status
       dwIndex:DWORD;                          // @field CUG index
       dwInfo:DWORD;                           // @field additional CUG flags
     end;
     RILCLOSEDGROUPSETTINGS = rilclosedgroupsettings_tag;
     LPRILCLOSEDGROUPSETTINGS = ^rilclosedgroupsettings_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILCALLFORWARDINGSETTING | Call forwarding service settings
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rilcallforwardingsettings_tag = record
       cbSize:DWORD;                           // @field structure size in bytes
       dwParams:DWORD;                         // @field indicates valid parameters
       dwStatus:DWORD;                         // @field activation status
       dwInfoClasses:DWORD;                    // @field indicates which classes of calls to forward
       raAddress:RILADDRESS;                   // @field forwarding address
       rsaSubAddress:RILSUBADDRESS;            // @field forwarding subaddress
       dwDelayTime:DWORD;                      // @field seconds to wait in <def RIL_FWDREASON_NOREPLY> case
     end;
     RILCALLFORWARDINGSETTINGS = rilcallforwardingsettings_tag;
     LPRILCALLFORWARDINGSETTINGS = ^rilcallforwardingsettings_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILCALLINFO | Information about a specific call
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rilcallinfo_tag = record
       cbSize:DWORD;                           // @field structure size in bytes
       dwParams:DWORD;                         // @field indicates valid parameters
       dwID:DWORD;                             // @field identifies each call
       dwDirection:DWORD;                      // @field incoming or outgoing
       dwStatus:DWORD;                         // @field properties of the call
       dwType:DWORD;                           // @field voice or data or fax
       dwMultiparty:DWORD;                     // @field conference call status
       raAddress:RILADDRESS;                   // @field call address
       wszDescription:array[0..MAXLENGTH_DESCRIPTION-1] of WCHAR; // @field any associated text
       dwDisconnectCode:DWORD;                                       // if dwStatus is disconnected - this contains the reason
     end;
     RILCALLINFO = rilcallinfo_tag;
     LPRILCALLINFO = ^rilcallinfo_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILGAININFO | Audio gain information
//
// @comm The minimum and maximum values for both dwTxGain and dwRxGain
//       are 0 and ULONG_MAX (that is, 0xFFFFFFFFUL; see limits.h).
//       Values between these extremes scale linearly.
//
//       It is the RIL Driver's responsibility to scale these values
//       to match whatever is appropriate for the corresponding radio.
//       So for example, if a radio's gain range is from 0 to 0x1F,
//       the RIL Driver should interpret 0xFFFFFFFF as 0x1F, and map
//       intermediate values proportionately.
//
// -----------------------------------------------------------------------------
type
     rilgaininfo_tag = record
       cbSize:DWORD;                           // @field structure size in bytes
       dwParams:DWORD;                         // @field indicates valid parameters
       dwTxGain:DWORD;                         // @field transmit gain level (128 nominal, 0 automatic)
       dwRxGain:DWORD;                         // @field receive gain level (128 nominal, 0 automatic)
     end;
     RILGAININFO = rilgaininfo_tag;
     LPRILGAININFO = ^rilgaininfo_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILAUDIODEVICEINFO | Audio device information
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rilaudiodeviceinfo_tag = record
       cbSize:DWORD;                           // @field structure size in bytes
       dwParams:DWORD;                         // @field indicates valid parameters
       dwTxDevice:DWORD;                       // @field transmit device
       dwRxDevice:DWORD;                       // @field receive device
     end;
     RILAUDIODEVICEINFO = rilaudiodeviceinfo_tag;
     LPRILAUDIODEVICEINFO = ^rilaudiodeviceinfo_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILHSCSDINFO | High speed circuit switched data settings
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rilhscsdinfo_tag = record
       cbSize:DWORD;                       // @field structure size in bytes
       dwParams:DWORD;                     // @field indicates valid parameters
       dwTranspRxTimeslots:DWORD;          // @field number of receive timeslots for transparent HSCSD calls
       dwTranspChannelCodings:DWORD;       // @field accepted channel codings for transparent HSCSD calls
       dwNonTranspRxTimeslots:DWORD;       // @field number of receive timeslots for non-transparent HSCSD calls
       dwNonTranspChannelCodings:DWORD;    // @field accepted channel codings for non-transparent HSCSD calls
       dwAirInterfaceUserRate:DWORD;       // @field air interface user rate for non-transparent HSCSD calls
       dwRxTimeslotsLimit:DWORD;           // @field maximum number of receive timeslots to be used during the next non-transparent HSCSD call
       fAutoSvcLevelUpgrading:BOOL;        // @field TRUE if automatic user-initiated service level upgrading for non-transparent HSCSD calls is enabled, FALSE otherwise
     end;
     RILHSCSDINFO = rilhscsdinfo_tag;
     LPRILHSCSDINFO = ^rilhscsdinfo_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILCALLHSCSDINFO | High speed circuit switched data information for the current call
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rilcallhscsdinfo_tag = record
       cbSize:DWORD;                       // @field structure size in bytes
       dwParams:DWORD;                     // @field indicates valid parameters
       dwRxTimeslots:DWORD;                // @field number of receive timeslots currently in use
       dwTxTimeslots:DWORD;                // @field number of transmit timeslots currently in use
       dwAirInterfaceUserRate:DWORD;       // @field air interface user rate currently in use
       dwChannelCoding:DWORD;              // @field current channel coding
     end;
     RILCALLHSCSDINFO = rilcallhscsdinfo_tag;
     LPRILCALLHSCSDINFO = ^rilcallhscsdinfo_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILDATACOMPINFO | Data compression settings
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rildatacompinfo_tag = record
       cbSize:DWORD;                       // @field structure size in bytes
       dwParams:DWORD;                     // @field indicates valid parameters
       dwDirection:DWORD;                  // @field compression in transmit and/or receive direcitons
       dwNegotiation:DWORD;                // @field compression is required or optional
       dwMaxDictEntries:DWORD;             // @field maximum number of dictionary entries
       dwMaxStringLength:DWORD;            // @field maximum string length
     end;
     RILDATACOMPINFO = rildatacompinfo_tag;
     LPRILDATACOMPINFO = ^rildatacompinfo_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILERRORCORRECTIONINFO | Error correction settings
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rilerrorcorrectioninfo_tag = record
       cbSize:DWORD;                       // @field structure size in bytes
       dwParams:DWORD;                     // @field indicates valid parameters
       dwOriginalRequest:DWORD;            // @field TBD
       dwOriginalFallback:DWORD;           // @field TBD
       dwAnswererFallback:DWORD;           // @field TBD
     end;
     RILERRORCORRECTIONINFO = rilerrorcorrectioninfo_tag;
     LPRILERRORCORRECTIONINFO = ^rilerrorcorrectioninfo_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILBEARERSVCINFO | Bearer service settings
//
// @comm For <def RIL_BSVCCE_BOTH_> constants, the subsequent text indicates the
//       preferred connection element.  For instance, <def RIL_BSVCCE_BOTH_TRANSPARENT>
//       means that both transparent and non transparent are supported, but transparent
//       is preferred.
//
// -----------------------------------------------------------------------------
type
     rilbearersvcinfo_tag = record
       cbSize:DWORD;                       // @field structure size in bytes
       dwParams:DWORD;                     // @field indicates valid parameters
       dwSpeed:DWORD;                      // @field offered data speed (protocol dependant)
       dwServiceName:DWORD;                // @field type of data service
       dwConnectionElement:DWORD;          // @field indicates transparent or non-transparent connection
     end;
     RILBEARERSVCINFO = rilbearersvcinfo_tag;
     LPRILBEARERSVCINFO = ^rilbearersvcinfo_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILRLPINFO | Radio link protocol settings
//
// @comm None
//
// -----------------------------------------------------------------------------
//
type
     rilrlpinfo_tag = record
       cbSize:DWORD;                       // @field structure size in bytes
       dwParams:DWORD;                     // @field indicates valid parameters
       dwIWS:DWORD;                        // @field IWF-to-MS window size
       dwMWS:DWORD;                        // @field MS-to-IWF window size
       dwAckTimer:DWORD;                   // @field acknowledgement timer in 10s of milliseconds (T1)
       dwRetransmissionAttempts:DWORD;     // @field number of retransmission attempts (N2)
       dwVersion:DWORD;                    // @field RLP version number
       dwResequencingPeriod:DWORD;         // @field resequencing period (T4)
     end;
     RILRLPINFO = rilrlpinfo_tag;
     LPRILRLPINFO = ^rilrlpinfo_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILMSGSERVICEINFO | Messaging service settings
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rilmsgserviceinfo_tag = record
       cbSize:DWORD;                       // @field structure size in bytes
       dwParams:DWORD;                     // @field indicates valid parameters
       dwService:DWORD;                    // @field supported service types
       dwMsgClasses:DWORD;                 // @field supported message classes
       dwReadLocation:DWORD;               // @field currect read location
       dwReadUsed:DWORD;                   // @field number of fields used
       dwReadTotal:DWORD;                  // @field total number of fields
       dwWriteLocation:DWORD;              // @field currect read location
       dwWriteUsed:DWORD;                  // @field number of fields used
       dwWriteTotal:DWORD;                 // @field total number of fields
       dwStoreLocation:DWORD;              // @field currect read location
       dwStoreUsed:DWORD;                  // @field number of fields used
       dwStoreTotal:DWORD;                 // @field total number of fields
     end;
     RILMSGSERVICEINFO = rilmsgserviceinfo_tag;
     LPRILMSGSERVICEINFO = ^rilmsgserviceinfo_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILMSGDCS | Message data coding scheme
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rilmsgdcs_tag = record
       cbSize:DWORD;                       // @field structure size in bytes
       dwParams:DWORD;                     // @field indicates valid parameters
       dwType:DWORD;                       // @field DCS type
       dwFlags:DWORD;                      // @field DCS flags
       dwMsgClass:DWORD;                   // @field message class (Only for RIL_DCSTYPE_GENERAL and RIL_DCSTYPE_MSGCLASS)
       dwAlphabet:DWORD;                   // @field DCS alphabet
       dwIndication:DWORD;                 // @field indication (Only for RIL_DCSTYPE_MSGWAIT)
       dwLanguage:DWORD;                   // @field indication (Only for RIL_DCSTYPE_LANGUAGE)
     end;
     RILMSGDCS = rilmsgdcs_tag;
     LPRILMSGDCS = ^rilmsgdcs_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILRANGE | Range of values
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rilrange_tag = record
       dwMinValue:DWORD;                   // @field minimum value
       dwMaxValue:DWORD;                   // @field maximum value
     end;
     RILRANGE = rilrange_tag;
     LPRILRANGE = ^rilrange_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILMSGCONFIG | Messaging configuration
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rilmsgconfig_tag = record
       cbSize:DWORD;                       // @field structure size in bytes
       dwParams:DWORD;                     // @field indicates valid parameters
       raSvcCtrAddress:RILADDRESS;         // @field service center address
     end;
     RILMSGCONFIG = rilmsgconfig_tag;
     LPRILMSGCONFIG = ^rilmsgconfig_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILCBMSGCONFIG | Cell broadcast messaging configuration
//
// @comm None
//
// -----------------------------------------------------------------------------
//#pragma warning(disable : 4200) // Disable "C4200: nonstandard extension used : zero-sized array in struct/union"
type
     rilcbmsgconfig_tag = record
       cbSize:DWORD;                       // @field structure size in bytes
       dwParams:DWORD;                     // @field indicates valid parameters
       dwBroadcastMsgLangs:DWORD;          // @field broadcast message languages
       fAccept:BOOL;                       // @field TRUE if broadcast message ranges are accepted (vs. rejected)
       rgrrBroadcastMsgIDs:array[0..0] of RILRANGE;     // @field an array of RILRANGE IDs to set, a same min/max value specifies a single ID
     end;
     RILCBMSGCONFIG = rilcbmsgconfig_tag;
     LPRILCBMSGCONFIG = ^rilcbmsgconfig_tag;
//#pragma warning(default : 4200)

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILMESSAGE | Message data
//
// @comm None
//
// -----------------------------------------------------------------------------
type
// @field RIL_MSGTYPE_IN_DELIVER
     TmsgInDeliver = record
       raOrigAddress:RILADDRESS;   // @field originating address
       dwProtocolID:DWORD;         // @field message protocol
       rmdDataCoding:RILMSGDCS;    // @field data coding scheme
       stSCReceiveTime:SYSTEMTIME; // @field receive time (UTC)
       cbHdrLength:DWORD;          // @field length of header in bytes
       cchMsgLength:DWORD;         // @field length of body in bytes
       rgbHdr:array[0..MAXLENGTH_HDR-1] of byte; // @field header buffer
       rgbMsg:array[0..MAXLENGTH_MSG-1] of byte; // @field body buffer
     end;

// @field RIL_MSGTYPE_IN_STATUS
     TmsgInStatus = record
       dwTgtMsgReference:DWORD;    // @field target message reference
       raTgtRecipAddress:RILADDRESS; // @field receipient address
       stTgtSCReceiveTime:SYSTEMTIME; // @field receipient receive time (UTC)
       stTgtDischargeTime:SYSTEMTIME; // @field receipient dischage time (UTC)
       dwTgtDlvStatus:DWORD;       // @field delivery status
       dwProtocolID:DWORD;         // @field message protocol
       rmdDataCoding:RILMSGDCS;    // @field data coding scheme
       cbHdrLength:DWORD;          // @field length of header in bytes
       cchMsgLength:DWORD;         // @field length of body in bytes
       rgbHdr:array[0..MAXLENGTH_HDR-1] of byte; // @field header buffer
       rgbMsg:array[0..MAXLENGTH_MSG-1] of byte; // @field body buffer
     end;

// @field RIL_MSGTYPE_OUT_SUBMIT
     TmsgOutSubmit = record
       raDestAddress:RILADDRESS;   // @field destination address
       dwProtocolID:DWORD;         // @field message protocol
       rmdDataCoding:RILMSGDCS;    // @field data coding scheme
       dwVPFormat:DWORD;           // @field TBD
       stVP:SYSTEMTIME;            // @field relative validity period (values are expressed relative to the current time)
       cbHdrLength:DWORD;          // @field length of header in bytes
       cchMsgLength:DWORD;         // @field length of body in bytes
       rgbHdr:array[0..MAXLENGTH_HDR-1] of byte; // @field header buffer
       rgbMsg:array[0..MAXLENGTH_MSG-1] of byte; // @field body buffer
     end;

// @field RIL_MSGTYPE_OUT_COMMAND
     TmsgOutCommand = record
       dwProtocolID:DWORD;         // @field message protocol
       dwCommandType:DWORD;        // @field command type
       dwTgtMsgReference:DWORD;    // @field target message reference
       raDestAddress:RILADDRESS;   // @field destination address
       cbCmdLength:DWORD;          // @field length of command in bytes
       rgbCmd:array[0..MAXLENGTH_CMD-1] of byte; // @field command buffer
     end;

// @field RIL_MSGTYPE_BC_GENERAL
     TmsgBcGeneral = record
       dwGeoScope:DWORD;           // @field message protocol
       dwMsgCode:DWORD;            // @field message code
       dwUpdateNumber:DWORD;       // @field update number
       dwID:DWORD;                 // @field identity
       rmdDataCoding:RILMSGDCS;    // @field data coding scheme
       dwTotalPages:DWORD;         // @field total number of pages
       dwPageNumber:DWORD;         // @field current page number
       cchMsgLength:DWORD;         // @field length of message in bytes
       rgbMsg:array[0..MAXLENGTH_MSG-1] of byte; // @field message buffer
     end;

// @field RIL_MSGTYPE_OUT_RAW
     TmsgOutRaw = record
       cchMsgLength:DWORD;         // @field length of body in bytes
       rgbMsg:array[0..MAXLENGTH_MSG-1] of byte; // @field message buffer
     end;

// @field RIL_MSGTYPE_IN_IS637DELIVER
     TmsgIS637InDeliver = record
       raOrigAddress:RILADDRESS;      // @field originating address
       rsaOrigSubaddr:RILSUBADDRESS;  // @field
      // There is no digit-mode in incoming message because the driver can convert both of them to ASCII

       stSCReceiveTime:SYSTEMTIME;    // @field (SMSC Timestamp) receive time (UTC)

       stValidityPeriodAbs:SYSTEMTIME;    // @field UTC time
       stValidityPeriodRel:SYSTEMTIME;    // @field Relative time
       stDeferredDelTimeAbs:SYSTEMTIME;   // @field UTC time
       stDeferredDelTimeRel:SYSTEMTIME;   // @field Relative time

       dwNumMsgs:DWORD;              // @field Used for Voicemail only.  Indicates the number of Messages on Vmail
       raCallBackNumber:RILADDRESS;       // @field (Only paging and Text -s) user can give a callback number in certain messages
       dwMsgPriority:DWORD;          // @field RIL_MSGPRIORITY_ constant
       dwMsgPrivacy:DWORD;           // @field RIL_MSGPRIVACYCLASS_ constant

       bUserAckRequest:BOOL;            // @field 0 = Not Requested; 1 = Requested ; This is an ack from the end user
       dwMsgDisplayMode:DWORD;           // @field RIL_MSGDISPLAYMODE_ constant

       dwTeleservice:DWORD;              // @field RIL_MSGTELESERVICE_* Constant[Mandatory]

       dwMsgID:DWORD;                    // @field [Mandatory] Message ID.  (0-65535) (In the WAP architecture each part of a multipart message share the same MsgID)
       dwMsgLang:DWORD;                  // @field Under Investigation
       dwMsgEncoding:DWORD;              // @field RIL_MSGCODING_* constant [5 bits] under Investigation
       cchMsgLength:DWORD;               // @field length of body in bytes
       rgbMsg:array[0..MAXLENGTH_MSG-1] of byte;      // @field body buffer
     end;

// @field RIL_MSGTYPE_OUT_IS637SUBMIT
     TmsgIS637OutSubmit = record
       raDestAddress:RILADDRESS;      // @field destination address
       rsaDestSubaddr:RILSUBADDRESS;     // @field destination subaddress
       bDigit:BOOL;             // @field specifies if the address in RILADDRESS is 4bit mode (=0) or in 8 bit mode (=1) (should be set to 1 by default)

       stValidityPeriodAbs:SYSTEMTIME;    // @field UTC time
       stValidityPeriodRel:SYSTEMTIME;    // @field Relative time
       stDeferredDelTimeAbs:SYSTEMTIME;   // @field UTC time
       stDeferredDelTimeRel:SYSTEMTIME;   // @field Relative time

       bDeliveryAckRequest:BOOL;        // @field 0 = Not Requested; 1 = Requested ; This is an delivery ack (no user confirmation)
       bUserAckRequest:BOOL;            // @field 0 = Not Requested; 1 = Requested ; This is an ack from the end user
       bBearerReplyRequest:BOOL;        // @field specifies the bearer reply field is set (technically this can be set, but it should not be) ; Boolean (0=not set, 1=set)
       dwReplySeqNumber:DWORD;           // @field the Seuqence number of the message bing replied to; (typically the MSGID)
       dwMsgDisplayMode:DWORD;           // @field RIL_MSGDISPLAYMODE_* constant

       raCallBackNumber:RILADDRESS;       // @field (Only paging and Text -s) user can give a callback number in certain messages

       dwMsgPriority:DWORD;          // @field RIL_MSGPRIORITY_ constant
       dwMsgPrivacy:DWORD;           // @field RIL_MSGPRIVACYCLASS_ constant

       dwTeleservice:DWORD;              // @field RIL_MSGTELESERVICE_* Constant[Mandatory]

       dwMsgID:DWORD;                    // @field [Mandatory] Message ID.  (0-65535) (In the WAP architecture each part of a multipart message share the same MsgID)
       dwMsgLang:DWORD;                  // @field Under Investigation
       dwMsgEncoding:DWORD;              // @field RIL_MSGCODING_* constant [5 bits] under Investigation
       cchMsgLength:DWORD;               // @field length of body in bytes
       rgbMsg:array[0..MAXLENGTH_MSG-1] of byte;      // @field body buffer
     end;

// @field RIL_MSGTYPE_IN_IS637STATUS
     TmsgIS637InStatus = record
       raOrigAddress:RILADDRESS;      // @field originating address
       rsaOrigSubaddr:RILSUBADDRESS;     // @field
      // There is no digit-mode in incoming message because the driver can convert both of them to ASCII

       stSCReceiveTime:SYSTEMTIME;    // @field (SMSC Timestamp) receive time (UTC)
       dwCauseCode:DWORD;        // @field Cause_Codes Under Investigation, most likely these will be implemented as RIL errors
       dwReplySeqNumber:DWORD;   // @field The Sequence number of the message bing replied to; (typically the MSGID)
       dwUserResponseCode:DWORD; // @field User Response Code (Carrier Specific Element when responding giving a User Ack)
       dwMsgStatusType:DWORD;    // @field type of status message RIL_MSGSTATUSTYPE_* constant

       dwMsgID:DWORD;                    // @field [Mandatory] Message ID.  (0-65535) (In the WAP architecture each part of a multipart message share the same MsgID)
       dwMsgLang:DWORD;                  // @field Under Investigation
       dwMsgEncoding:DWORD;              // @field RIL_MSGCODING_* constant [5 bits] under Investigation
       cchMsgLength:DWORD;               // @field length of body in bytes
       rgbMsg:array[0..MAXLENGTH_MSG-1] of byte;      // @field body buffer
     end;

// @field RIL_MSGTYPE_OUT_IS637STATUS
     TmsgIS637OutStatus = record
       raDestAddress:RILADDRESS;      // @field destination address
       rsaDestSubaddr:RILSUBADDRESS;     // @field destination subaddress
       bDigit:BOOL;             // @field specifies if the address in RILADDRESS is 4bit mode (=0) or in 8 bit mode (=1) (should be set to 1 by default)

       dwReplySeqNumber:DWORD;           // @field The Sequence number of the message bing replied to; (typically the MSGID)
       dwUserResponseCode:DWORD;         // @field User Response Code (Carrier Specific Element when responding giving a User Ack)

       dwMsgID:DWORD;                    // @field [Mandatory] Message ID.  (0-65535) (In the WAP architecture each part of a multipart message share the same MsgID)
       dwMsgLang:DWORD;                  // @field Under Investigation
       dwMsgEncoding:DWORD;              // @field RIL_MSGCODING_* constant [5 bits] under Investigation
       cchMsgLength:DWORD;               // @field length of body in bytes
       rgbMsg:array[0..MAXLENGTH_MSG-1] of byte;      // @field body buffer
     end;


     rilmessage_tag = record
       cbSize:DWORD;                       // @field structure size in bytes
       dwParams:DWORD;                     // @field indicates valid parameters
       raSvcCtrAddress:RILADDRESS;         // @field service center address
       dwType:DWORD;                       // @field type of message
       dwFlags:DWORD;                      // @field message flags
       case longint of
         0: (msgInDeliver:TmsgInDeliver); // @field RIL_MSGTYPE_IN_DELIVER
         1: (msgInStatus:TmsgInStatus);   // @field RIL_MSGTYPE_IN_STATUS
         2: (msgOutSubmit:TmsgOutSubmit); // @field RIL_MSGTYPE_OUT_SUBMIT
         3: (msgOutCommand:TmsgOutCommand); // @field RIL_MSGTYPE_OUT_COMMAND
         4: (msgBcGeneral:TmsgBcGeneral);   // @field RIL_MSGTYPE_BC_GENERAL
         5: (msgOutRaw:TmsgOutRaw);         // @field RIL_MSGTYPE_OUT_RAW
         6: (msgIS637InDeliver:TmsgIS637InDeliver); // @field RIL_MSGTYPE_IN_IS637DELIVER
         7: (msgIS637OutSubmit:TmsgIS637OutSubmit); // @field RIL_MSGTYPE_OUT_IS637SUBMIT
         8: (msgIS637InStatus:TmsgIS637InStatus); // @field RIL_MSGTYPE_IN_IS637STATUS
         9: (msgIS637OutStatus:TmsgIS637OutStatus); // @field End RIL_MSGTYPE_OUT_IS637STATUS
     end;
     RILMESSAGE = rilmessage_tag;
     LPRILMESSAGE = ^rilmessage_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILMESSAGE_IN_SIM | Message data in sim info
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rilmessage_in_sim_tag = record
       cbSize:DWORD;                       // @field structure size in bytes
       dwParams:DWORD;
       dwLocation:DWORD;                   // @field storage area (one of RIL_MSGLOC_xxxx)
       dwIndex:DWORD;                      // @field storage index occupied by the message
     end;
     RILMESSAGE_IN_SIM = rilmessage_in_sim_tag;
     LPRILMESSAGE_IN_SIM = ^rilmessage_in_sim_tag;


// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILMESSAGEINFO | Message data with additional info
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rilmessageinfo_tag = record
       cbSize:DWORD;                       // @field structure size in bytes
       dwParams:DWORD;                     // @field indicates valid parameters
       dwIndex:DWORD;                      // @field storage index occupied by the message
       dwStatus:DWORD;                     // @field message status
       rmMessage:RILMESSAGE;               // @field the message itself
     end;
     RILMESSAGEINFO = rilmessageinfo_tag;
     LPRILMESSAGEINFO = ^rilmessageinfo_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILEQUIPMENTINFO | Equipment info
//
// @comm None
//
// -----------------------------------------------------------------------------
type                               
     rilequipmentinfo_tag = record
       cbSize:DWORD;                       // @field structure size in bytes
       dwParams:DWORD;                     // @field indicates valid parameters
       szManufacturer:array[0..MAXLENGTH_EQUIPINFO-1] of AnsiChar; // @field manufacturer of the radio hardware
       szModel:array[0..MAXLENGTH_EQUIPINFO-1] of AnsiChar;  // @field model of the radio hardware
       szRevision:array[0..MAXLENGTH_EQUIPINFO-1] of AnsiChar; // @field software version of the radio stack
       szSerialNumber:array[0..MAXLENGTH_EQUIPINFO-1] of AnsiChar; // @field equipment identity (IMEI)
     end;
     RILEQUIPMENTINFO = rilequipmentinfo_tag;
     LPRILEQUIPMENTINFO = ^rilequipmentinfo_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILEQUIPMENTSTATE | Equipment state
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rilequipmentstate_tag = record
       cbSize:DWORD;                       // @field structure size in bytes
       dwParams:DWORD;                     // @field indicates valid parameters
       dwRadioSupport:DWORD;               // @field RIL_RADIOSUPPORT_* Parameter
       dwEqState:DWORD;                    // @field RIL_EQSTATE_* Parameter
       dwReadyState:DWORD;                 // @field RIL_READYSTATE_* Parameter
     end;
     RILEQUIPMENTSTATE = rilequipmentstate_tag;
     LPRILEQUIPMENTSTATE = ^rilequipmentstate_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILPHONEBOOKINFO | Phonebook settings
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rilphonebookinfo_tag = record
       cbSize:DWORD;                       // @field structure size in bytes
       dwParams:DWORD;                     // @field indicates valid parameters
       dwStoreLocation:DWORD;              // @field location of phonebook memory
       dwUsed:DWORD;                       // @field number of locations used
       dwTotal:DWORD;                      // @field total number of phonebook locations
     end;
     RILPHONEBOOKINFO = rilphonebookinfo_tag;
     LPRILPHONEBOOKINFO = ^rilphonebookinfo_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILPHONEBOOKENTRY | A single phonebook entry
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rilphonebookentry_tag = record
       cbSize:DWORD;                       // @field structure size in bytes
       dwParams:DWORD;                     // @field indicates valid parameters
       dwIndex:DWORD;                      // @field index of the entry
       raAddress:RILADDRESS;               // @field the stored address
       wszText:array[0..MAXLENGTH_PHONEBOOKTEXT-1] of WCHAR; // @field assciated text
     end;
     RILPHONEBOOKENTRY = rilphonebookentry_tag;
     LPRILPHONEBOOKENTRY = ^rilphonebookentry_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILATRINFO | Answer to Reset information
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rilatrinfo_tag = record
       cbSize:DWORD;
       dwParams:DWORD;
       dwPhase:DWORD;
       cbATRSize:DWORD;
       rgbATR:array[0..MAXLENGTH_ATR-1] of byte;
     end;
     RILATRINFO = rilatrinfo_tag;
     LPRILATRINFO = ^rilatrinfo_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILSIMTOOLKITEVENTCAPS | SIM TOOLKIT EVENT LIST CAPABILITIES
//
// @comm This structure indicates who implements the various SIM ToolKit Events
//
// -----------------------------------------------------------------------------
type
     rilsimtoolkiteventcaps_tag =record
       cbSize:DWORD;                       // @field structure size in bytes
       dwParams:DWORD;                     // @field indicates valid parameters
       dwMTCall:DWORD;                     // @constdefine TBD
       dwCallConnected:DWORD;              // @constdefine TBD
       dwCallDisconnected:DWORD;           // @constdefine TBD
       dwLocationStatus:DWORD;             // @constdefine TBD
       dwUserActivity:DWORD;               // @constdefine TBD
       dwIdleScreen:DWORD;                 // @constdefine TBD
       dwLanguageSelection:DWORD;          // @constdefine TBD
       dwBrowserTermination:DWORD;         // @constdefine TBD
       dwDataAvailable:DWORD;              // @constdefine TBD
       dwChannelStatus:DWORD;              // @constdefine TBD
       dwDisplayChange:DWORD;              // @constdefine TBD
     end;
     RILSIMTOOLKITEVENTCAPS = rilsimtoolkiteventcaps_tag;
     LPRILSIMTOOLKITEVENTCAPS = ^rilsimtoolkiteventcaps_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILSIMTOOLKITNOTIFYCAPS | SIM TOOLKIT NOTIFY CAPABILITIES
//
// @comm This structure indicates who implements the various SIM ToolKit Notifications
//
// -----------------------------------------------------------------------------
type
     rilsimtoolkitnotifycaps_tag = record
       cbSize:DWORD;                       // @field structure size in bytes
       dwParams:DWORD;                     // @field indicates valid parameters
       dwRefresh:DWORD;                    // @constdefine TBD
       dwMoreTime:DWORD;                   // @constdefine TBD
       dwPollInterval:DWORD;               // @constdefine TBD
       dwPollingOff:DWORD;                 // @constdefine TBD
       dwSetUpCall:DWORD;                  // @constdefine TBD
       dwSendSS:DWORD;                     // @constdefine TBD
       dwSendUSSD:DWORD;                   // @constdefine TBD
       dwSendSMS:DWORD;                    // @constdefine TBD
       dwPlayTone:DWORD;                   // @constdefine TBD
       dwDisplayText:DWORD;                // @constdefine TBD
       dwGetInkey:DWORD;                   // @constdefine TBD
       dwGetInput:DWORD;                   // @constdefine TBD
       dwSelectItem:DWORD;                 // @constdefine TBD
       dwSetupMenu:DWORD;                  // @constdefine TBD
       dwSetupIdleModeText:DWORD;          // @constdefine TBD
       dwLocalInfo:DWORD;                  // @constdefine TBD
       dwNotifyFlags:DWORD;                // @combination of RIL_CAPS_NOTIFY_* flags
       dwSetupEventList:DWORD;             // @constdefine TBD
       dwSendDTMF:DWORD;                   // @constdefine TBD
       dwLaunchBrowser:DWORD;              // @constdefine TBD
       dwOpenChannel:DWORD;                // @constdefine TBD
       dwCloseChannel:DWORD;               // @constdefine TBD
       dwReceiveData:DWORD;                // @constdefine TBD
       dwSendData:DWORD;                   // @constdefine TBD
       dwTimerManagement:DWORD;            // @constdefine TBD
       dwRunAtCmd:DWORD;                   // @constdefine TBD
       rstecEvents:RILSIMTOOLKITEVENTCAPS; // @constdefine TBD
     end;
     RILSIMTOOLKITNOTIFYCAPS = rilsimtoolkitnotifycaps_tag;
     LPRILSIMTOOLKITNOTIFYCAPS = ^rilsimtoolkitnotifycaps_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILSIMTOOLKITCMD | SIM toolkit command details.

//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rilsimtoolkitcmd_tag = record
       cbSize:DWORD;
       dwParams:DWORD;
       dwId:DWORD;
       dwTag:DWORD;
       dwType:DWORD;
       dwQualifier:DWORD;
       dwError:DWORD;
       dwDetailsOffset:DWORD;
       dwDetailsSize:DWORD;
     end;
     RILSIMTOOLKITCMD = rilsimtoolkitcmd_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILSIMTOOLKITRSP | Response to a SIM toolkit command.

//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rilsimtoolkitrsp_tag = record
       cbSize:DWORD;
       dwParams:DWORD;
       dwId:DWORD;
       dwTag:DWORD;
       dwType:DWORD;
       dwQualifier:DWORD;
       dwResponse:DWORD;
       dwAdditionalInfo:DWORD;
     end;
     RILSIMTOOLKITRSP = rilsimtoolkitrsp_tag;
     LPRILSIMTOOLKITRSP = rilsimtoolkitrsp_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILSIMCMDPARAMETERS | Parameters for a restricted SIM command

//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rilsimcmdparameters_tag = record
       cbSize:DWORD;                       // @field structure size in bytes
       dwParams:DWORD;                     // @field indicates valid parameters
       dwFileID:DWORD;                     // @field SIM file ID
       dwParameter1:DWORD;                 // @field parameter specific to SIM command
       dwParameter2:DWORD;                 // @field parameter specific to SIM command
       dwParameter3:DWORD;                 // @field parameter specific to SIM command
     end;
     RILSIMCMDPARAMETERS = rilsimcmdparameters_tag;
     LPRILSIMCMDPARAMETERS = ^rilsimcmdparameters_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILSIMRESPONSE | Response to a restrcited SIM command
//
// @comm None
//
// -----------------------------------------------------------------------------
//#pragma warning(disable : 4200) // Disable "C4200: nonstandard extension used : zero-sized array in struct/union"
type
     rilsimresponse_tag = record
       cbSize:DWORD;                       // @field structure size in bytes
       dwParams:DWORD;                     // @field indicates valid parameters
       dwStatusWord1:DWORD;                // @field return parameter specific to SIM command
       dwStatusWord2:DWORD;                // @field return parameter specific to SIM command
       pbResponse:array[0..0] of byte;                  // @field additional bytes of response data
     end;
     RILSIMRESPONSE = rilsimresponse_tag;
     LPRILSIMRESPONSE = ^rilsimresponse_tag;
//#pragma warning(default : 4200)

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILSIMRECORDSTATUS | Response to a restrcited SIM command
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rilsimrecordstatus_tag = record
       cbSize:DWORD;                           // @field Size of the structure in bytes
       dwParams:DWORD;                         // @field Indicates valid parameter values
       dwRecordType:DWORD;                     // @field RIL_SIMRECORDTYPE_* Constant
       dwItemCount:DWORD;                      // @field Number of items in the record
       dwSize:DWORD;                           // @field Size in bytes of each item
     end;
     RILSIMRECORDSTATUS = rilsimrecordstatus_tag;
     LPRILSIMRECORDSTATUS = ^rilsimrecordstatus_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILCOSTINFO | Service cost info
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rilcostinfo_tag = record
       cbSize:DWORD;                       // @field structure size in bytes
       dwParams:DWORD;                     // @field indicates valid parameters
       dwCCM:DWORD;                        // @field current call meter
       dwACM:DWORD;                        // @field accumulated call meter
       dwMaxACM:DWORD;                     // @field maximum accumulated call meter
       dwCostPerUnit:DWORD;                // @field cost per unit, in 16.16 fixed point
       wszCurrency:array[0..MAXLENGTH_CURRENCY-1] of WCHAR; // @field current currency
     end;
     RILCOSTINFO = rilcostinfo_tag;
     LPRILCOSTINFO = ^rilcostinfo_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILSIGNALQUALITY | Signal quality info
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rilsignalquality_tag = record
       cbSize:DWORD;                       // @field structure size in bytes
       dwParams:DWORD;                     // @field indicates valid parameters
       nSignalStrength:longint;            // @field TBD
       nMinSignalStrength:longint;         // @field TBD
       nMaxSignalStrength:longint;         // @field TBD
       dwBitErrorRate:DWORD;               // @field bit error rate in 1/100 of a percent
       nLowSignalStrength:longint;         // @field TBD
       nHighSignalStrength:longint;        // @field TBD
     end;
     RILSIGNALQUALITY = rilsignalquality_tag;
     LPRILSIGNALQUALITY = ^rilsignalquality_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILCELLTOWERINFO | Cell tower info
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rilcelltowerinfo_tag = record
       cbSize:DWORD;                       // @field structure size in bytes
       dwParams:DWORD;                     // @field indicates valid parameters
       dwMobileCountryCode:DWORD;          // @field TBD
       dwMobileNetworkCode:DWORD;          // @field TBD
       dwLocationAreaCode:DWORD;           // @field TBD
       dwCellID:DWORD;                     // @field TBD
       dwBaseStationID:DWORD;              // @field TBD
       dwBroadcastControlChannel:DWORD;    // @field TBD
       dwRxLevel:DWORD;                    // @field Value from 0-63 (see GSM 05.08, 8.1.4)
       dwRxLevelFull:DWORD;                // @field Value from 0-63 (see GSM 05.08, 8.1.4)
       dwRxLevelSub:DWORD;                 // @field Value from 0-63 (see GSM 05.08, 8.1.4)
       dwRxQuality:DWORD;                  // @field Value from 0-7  (see GSM 05.08, 8.2.4)
       dwRxQualityFull:DWORD;              // @field Value from 0-7  (see GSM 05.08, 8.2.4)
       dwRxQualitySub:DWORD;               // @field Value from 0-7  (see GSM 05.08, 8.2.4)
       dwIdleTimeSlot:DWORD;               // @field TBD
       dwTimingAdvance:DWORD;              // @field TBD
       dwGPRSCellID:DWORD;                 // @field TBD
       dwGPRSBaseStationID:DWORD;          // @field TBD
       dwNumBCCH:DWORD;                    // @field TBD
       rgbBCCH:array[0..MAXLENGTH_BCCH-1] of byte;       // @field TBD
       rgbNMR:array[0..MAXLENGTH_NMR-1] of byte;         // @field TBD
     end;
     RILCELLTOWERINFO = rilcelltowerinfo_tag;
     LPRILCELLTOWERINFO = ^rilcelltowerinfo_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILREMOTEPARTYINFO | Incoming call info
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rilremotepartyinfo_tag = record
       cbSize:DWORD;                       // @field structure size in bytes
       dwParams:DWORD;                     // @field indicates valid parameters
       raAddress:RILADDRESS;               // @field address of caller
       rsaSubAddress:RILSUBADDRESS;        // @field subaddress of caller
       wszDescription:array[0..MAXLENGTH_DESCRIPTION-1] of WCHAR; // @field text associated with caller
       dwValidity:DWORD;                   // @field indicates validity of caller info
     end;
     RILREMOTEPARTYINFO = rilremotepartyinfo_tag;
     LPRILREMOTEPARTYINFO = ^rilremotepartyinfo_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILCALLWAITINGINFO | Call waiting info
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rilcallwaitinginfo_tag = record
       cbSize:DWORD;                       // @field structure size in bytes
       dwParams:DWORD;                     // @field indicates valid parameters
       dwCallType:DWORD;                   // @field type of call
       dwAddressId:DWORD;                  // @field indicates address ID on which the incoming call arrived (if available)
       rrpiCallerInfo:RILREMOTEPARTYINFO;  // @field caller information
     end;
     RILCALLWAITINGINFO = rilcallwaitinginfo_tag;
     LPRILCALLWAITINGINFO = ^rilcallwaitinginfo_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILINTERMEDIATESSINFO | Intermediate Supplemenary Service Info
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rilintermediatessinfo_tag = record
       cbSize:DWORD;                       // @field structure size in bytes
       dwParams:DWORD;                     // @field indicates valid parameters
       dwNotificationCode:DWORD;           // @field indicates type of notification
       dwCallUserGroupIndex:DWORD;         // @field indicates the CUG Index
     end;
     RILINTERMEDIATESSINFO = rilintermediatessinfo_tag;
     LPRILINTERMEDIATESSINFO = ^rilintermediatessinfo_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILUNSOLICITEDSSINFO | Unsolicited Supplemenary Service Info
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rilunsolicitedssinfo_tag = record
       cbSize:DWORD;                       // @field structure size in bytes
       dwParams:DWORD;                     // @field indicates valid parameters
       dwNotificationCode:DWORD;           // @field indicates type of notification
       dwCallUserGroupIndex:DWORD;         // @field indicates the CUG Index
       raAddress:RILADDRESS;               // @field call address
       rsaSubAddress:RILSUBADDRESS;        // @field subaddress
     end;
     RILUNSOLICITEDSSINFO = rilunsolicitedssinfo_tag;
     LPRILUNSOLICITEDSSINFO = ^rilunsolicitedssinfo_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILSERVICEINFO | Connection service information
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rilserviceinfo_tag = record
       cbSize:DWORD;                       // @field structure size in bytes
       dwParams:DWORD;                     // @field indicates valid parameters
       fSynchronous:BOOL;                  // @field TRUE if connection service is synchronous, FALSE if asynchronous
       fTransparent:BOOL;                  // @field TRUE if connection service is transparent, FALSE if non-transparent
     end;
     RILSERVICEINFO = rilserviceinfo_tag;
     LPRILSERVICEINFO = ^rilserviceinfo_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILRINGINFO | Ring information
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rilringinfo_tag = record
       cbSize:DWORD;                       // @field structure size in bytes
       dwParams:DWORD;                     // @field indicates valid parameters
       dwCallType:DWORD;                   // @field type of the offered call (<def RIL_CALLTYPE_> constant)
       dwAddressId:DWORD;                  // @field indicates address ID on which the incoming call arrived (if available)
       rsiServiceInfo:RILSERVICEINFO;      // @field data connection service information (set only for <def RIL_CALLTYPE_DATA>)
     end;
     RILRINGINFO = rilringinfo_tag;
     LPRILRINGINFO = ^rilringinfo_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILDIALINFO | Ring information
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rildialinfo_tag = record
       cbSize:DWORD;                       // @field structure size in bytes
       dwParams:DWORD;                     // @field indicates valid parameters
       hrCmdId:HRESULT;                    // @field handle of call being dialed
       dwCallId:DWORD;                     // @field id of call being dialed
     end;
     RILDIALINFO = rildialinfo_tag;
     LPRILDIALINFO = ^rildialinfo_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILCONNECTINFO | Connection info
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rilconnectinfo_tag = record
       cbSize:DWORD;                       // @field structure size in bytes
       dwParams:DWORD;                     // @field indicates valid parameters
       dwCallType:DWORD;                   // @field type of the established connection (<def RIL_CALLTYPE_> constant)
       dwBaudRate:DWORD;                   // @field Baud rate of the established connection (set only for <def RIL_CALLTYPE_DATA>)
     end;
     RILCONNECTINFO = rilconnectinfo_tag;
     LPRILCONNECTINFO = ^rilconnectinfo_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILMSGSTORAGEINFO | Message storage locations
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rilmsgstorageinfo_tag = record
       cbSize:DWORD;                       // @field structure size in bytes
       dwParams:DWORD;                     // @field indicates valid parameters
       dwReadLocation:DWORD;               // @field current read location
       dwWriteLocation:DWORD;              // @field current write location
       dwStoreLocation:DWORD;              // @field current store location
     end;
     RILMSGSTORAGEINFO = rilmsgstorageinfo_tag;
     LPRILMSGSTORAGEINFO = ^rilmsgstorageinfo_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILSUPSERVICEDATA | Supplementary service data
//
// @comm None
//
// -----------------------------------------------------------------------------
//#pragma warning(disable : 4200) // Disable "C4200: nonstandard extension used : zero-sized array in struct/union"
type
     rilsupservicedata_tag = record
       cbSize:DWORD;                       // @field structure size in bytes
       dwParams:DWORD;                     // @field indicates valid parameters
       dwStatus:DWORD;                     // @field additional status for message
       pbData:array[0..0] of byte;                      // @field message itself
     end;
<<<<<<< HEAD
<<<<<<< HEAD
     RILSUPSERVICEDATA = rilsupservicedata_tag;
     LPRILSUPSERVICEDATA = ^rilsupservicedata_tag;
=======
     RILSUPSERVICEDATA = rilmsgstorageinfo_tag;
     LPRILSUPSERVICEDATA = ^rilmsgstorageinfo_tag;
>>>>>>> graemeg/fixes_2_2
=======
     RILSUPSERVICEDATA = rilmsgstorageinfo_tag;
     LPRILSUPSERVICEDATA = ^rilmsgstorageinfo_tag;
>>>>>>> origin/fixes_2_2
//#pragma warning(default : 4200)

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILCAPSDIAL | Dialing capabilities
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rilcapsdial_tag = record
       cbSize:DWORD;                       // @field structure size in bytes
       dwParams:DWORD;                     // @field indicates valid parameters
       dwCallTypes:DWORD;                  // @field type of call being placed
       dwOptions:DWORD;                    // @field dialing options
     end;
     RILCAPSDIAL = rilcapsdial_tag;
     LPRILCAPSDIAL = ^rilcapsdial_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILCAPSBEARERSVC | Bearer service capabilities
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rilcapsbearersvc_tag = record
       cbSize:DWORD;                       // @field structure size in bytes
       dwParams:DWORD;                     // @field indicates valid parameters
       dwSpeeds1:DWORD;                    // @field TBD
       dwSpeeds2:DWORD;                    // @field TBD
       dwServiceNames:DWORD;               // @field TBD
       dwConnectionElements:DWORD;         // @field TBD
     end;
     RILCAPSBEARERSVC = rilcapsbearersvc_tag;
     LPRILCAPSBEARERSVC = ^rilcapsbearersvc_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILCAPSRLP | Radio Link Protocol capabilities
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rilcapsrlp_tag = record
       cbSize:DWORD;                       // @field structure size in bytes
       dwParams:DWORD;                     // @field indicates valid parameters
       dwVersion:DWORD;                    // @field TBD
       rrIWSRange:RILRANGE;                // @field TBD
       rrMWSRange:RILRANGE;                // @field TBD
       rrAckTimerRange:RILRANGE;           // @field TBD
       rrRetransmissionAttsRange:RILRANGE; // @field TBD
       rrReseqPeriodRange:RILRANGE;        // @field TBD
     end;
     RILCAPSRLP = rilcapsrlp_tag;
     LPRILCAPSRLP = ^rilcapsrlp_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILCAPSMSGMEMORYLOCATIONS | Message memory location capabilities
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rilcapsmsgmemorylocations_tag = record
       cbSize:DWORD;                       // @field structure size in bytes
       dwParams:DWORD;                     // @field indicates valid parameters
       dwReadLocations:DWORD;              // @field supported read locations
       dwWriteLocations:DWORD;             // @field supported write locations
       dwStoreLocations:DWORD;             // @field supported store locations
     end;
     RILCAPSMSGMEMORYLOCATIONS = rilcapsmsgmemorylocations_tag;
     LPRILCAPSMSGMEMORYLOCATIONS = ^rilcapsmsgmemorylocations_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILCAPSLOCKINGPWDLENGTH | Locking password length capabilities
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rilcapslockingpwdlength_tag = record
       cbSize:DWORD;                       // @field structure size in bytes
       dwParams:DWORD;                     // @field indicates valid parameters
       dwFacility:DWORD;                   // @field the locking facility
       dwPasswordLength:DWORD;             // @field maximum password length
     end;
     RILCAPSLOCKINGPWDLENGTH = rilcapslockingpwdlength_tag;
     LPRILCAPSLOCKINGPWDLENGTH = ^rilcapslockingpwdlength_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILCAPSBARRINGPWDLENGTH | Call barring password length capabilities
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rilcapsbarringpwdlength_tag = record
       cbSize:DWORD;                       // @field structure size in bytes
       dwParams:DWORD;                     // @field indicates valid parameters
       dwType:DWORD;                       // @field type of call barring
       dwPasswordLength:DWORD;             // @field maximum password length
     end;
     RILCAPSBARRINGPWDLENGTH = rilcapsbarringpwdlength_tag;
     LPRILCAPSBARRINGPWDLENGTH = ^rilcapsbarringpwdlength_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILCAPSDATACOMPRESSION | Data compression capabilities
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rilcapsdatacompression_tag = record
       cbSize:DWORD;                       // @field structure size in bytes
       dwParams:DWORD;                     // @field indicates valid parameters
       dwDirection:DWORD;                  // @field indicates supported direction values
       dwNegotiation:DWORD;                // @field indicates supported negotiation values
       rrMaxDict:RILRANGE;                 // @field range of supported max_dict values
       rrMaxString:RILRANGE;               // @field range of supported max_string values
     end;
     RILCAPSDATACOMPRESSION = rilcapsdatacompression_tag;
     LPRILCAPSDATACOMPRESSION = ^rilcapsdatacompression_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILERRORCORRECTIONINFO | Error correction settings
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rilcapserrorcorrection_tag = record
       cbSize:DWORD;                       // @field structure size in bytes
       dwParams:DWORD;                     // @field indicates valid parameters
       dwOriginalRequest:DWORD;            // @field indicates supported originator request values
       dwOriginalFallback:DWORD;           // @field indicates supported originator fallback values
       dwAnswererFallback:DWORD;           // @field indicates supported answerer fallback values
     end;
     RILCAPSERRORCORRECTION = rilcapserrorcorrection_tag;
     LPRILCAPSERRORCORRECTION = ^rilcapserrorcorrection_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILCAPSHSCSD | High Speed Circuit Switched Data capabilities
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rilcapshscsd_tag = record
       cbSize:DWORD;                       // @field structure size in bytes
       dwParams:DWORD;                     // @field indicates valid parameters
       dwMultislotClass:DWORD;             // @field multislot class supported
       dwMaxRxTimeslots:DWORD;             // @field maximum number of receive timeslots
       dwMaxTxTimeslots:DWORD;             // @field maximum number of transmit timeslots
       dwMaxTotalTimeslots:DWORD;          // @field maximum number of total timeslots
       dwChannelCodings:DWORD;             // @field supported channel codings
       dwAirInterfaceUserRates:DWORD;      // @field supported air interfacerates
       rrTopRxTimeslotRange:RILRANGE;      // @field TBD
     end;
     RILCAPSHSCSD = rilcapshscsd_tag;
     LPRILCAPSHSCSD = ^rilcapshscsd_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILCAPSPBENTRYLENGTH | Phone book entry length maximum values
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rilcapspbentrylength_tag = record
       cbSize:DWORD;                       // @field structure size in bytes
       dwParams:DWORD;                     // @field indicates valid parameters
       dwMaxAddressLength:DWORD;           // @field maximum length of the phone number portion
       dwMaxTextLength:DWORD;              // @field maximum length of the text portion
     end;
     RILCAPSPBENTRYLENGTH = rilcapspbentrylength_tag;
     LPRILCAPSPBENTRYLENGTH = ^rilcapspbentrylength_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILGPRSPROTOCOLCAPS | General Packet Radio Service capabilities
//
// @comm TBDTBD
//
// -----------------------------------------------------------------------------
//#pragma warning(disable : 4200) // Disable "C4200: nonstandard extension used : zero-sized array in struct/union"
type
     rilgprscontextcaps_tag = record
       cbSize:DWORD;                       // @field structure size in bytes (padded to DWORD)
       dwParams:DWORD;                     // @field indicates valid parameters
       dwProtocolType:DWORD;               // @field a RIL_GPRSPROTOCOL_* constant
       ContextIDRange:RILRANGE;                 // @field min/max context ids
       dwDataCompression:DWORD;            // @field valid data compression values
       dwHeaderCompression:DWORD;          // @field valid header compression values
       dwParameterLength:DWORD;           // @field length of parameters list in bytes
       szParameters:array[0..0] of AnsiChar;               // @field valid string parameters of this prococol type, delimited by \0, with final param terminated by \0\0
     end;
     RILGPRSCONTEXTCAPS = rilgprscontextcaps_tag;
     LPRILGPRSCONTEXTCAPS = ^rilgprscontextcaps_tag;
//#pragma warning(default : 4200)

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILGPRSCONTEXT | A PDP Context represents a certain configuration for
//         packet data communication.
//
// @comm None
//
// -----------------------------------------------------------------------------
//#pragma warning(disable : 4200) // Disable "C4200: nonstandard extension used : zero-sized array in struct/union"
type
     rilgprscontext_tag = record
       cbSize:DWORD;                       // @field structure size in bytes
       dwParams:DWORD;                     // @field indicates valid parameters
       dwContextID:DWORD;                  // @field the context number
       dwProtocolType:DWORD;               // @field a RIL_GPRSPROTOCOL_*constant
       wszAccessPointName:array[0..MAXLENGTH_GPRSACCESSPOINTNAME-1] of WCHAR; // @field a logical name to select the gateway gprs
                                                                              //  (which defines the external packet data network to use)
       wszAddress:array[0..MAXLENGTH_GPRSADDRESS-1] of WCHAR; // @field the packet address to use (if null, request dynamic)
       dwDataCompression:DWORD;             // @field a RIL_GPRSDATACOMP_*
       dwHeaderCompression:DWORD;           // @field a RIL_GPRSHEADERCOMP_*
       dwParameterLength:DWORD;            // @field length of parameters list
       szParameters:array[0..0] of AnsiChar;              // @field parameters specific to the prococol type
     end;
     RILGPRSCONTEXT = rilgprscontext_tag;
     LPRILGPRSCONTEXT = ^rilgprscontext_tag;
//#pragma warning(default : 4200)

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILGPRSPROTOCOLCAPS | General Packet Radio Service capabilities
//
// @comm TBDTBD
//
// -----------------------------------------------------------------------------
//#pragma warning(disable : 4200) // Disable "C4200: nonstandard extension used : zero-sized array in struct/union"
type
     rilgprsqosprofilecaps_tag = record
       cbSize:DWORD;                       // @field structure size in bytes (padded to DWORD)
       dwParams:DWORD;                     // @field indicates valid parameters
       dwProtocolType:DWORD;               // @field a RIL_GPRSPROTOCOL_* constant
       dwPrecedenceClass:DWORD;            // @field valid RIL_GPRSPRECEDENCECLASS_* constants
       dwDelayClass:DWORD;                 // @field valid RIL_GPRSDELAYCLASS_* constants
       dwReliabilityClass:DWORD;           // @field valid RIL_GPRSRELIABILITYCLASS_* constants
       dwPeakThruClass:DWORD;              // @field valid RIL_GPRSPEAKTHRUCLASS_* constants
       dwMeanThruClass:DWORD;              // @field valid RIL_GPRSMEANTHRUCLASS_* constants
     end;
     RILGPRSQOSPROFILECAPS = rilgprsqosprofilecaps_tag;
     LPRILGPRSQOSPROFILECAPS = ^rilgprsqosprofilecaps_tag;
//#pragma warning(default : 4200)

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILGPRSQOSPROFILE | A quality of service profile
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rilgprsqosprofile_tag = record
       cbSize:DWORD;                       // @field structure size in bytes
       dwParams:DWORD;                     // @field indicates valid parameters
       dwContextID:DWORD;                  // @field the context number
       dwPrecedenceClass:DWORD;            // @field a RIL_GPRSPRECEDENCECLASS_* constant
       dwDelayClass:DWORD;                 // @field a RIL_GPRSDELAYCLASS_* constant
       dwReliabilityClass:DWORD;           // @field a RIL_GPRSRELIABILITYCLASS_* constant
       dwPeakThruClass:DWORD;              // @field a RIL_GPRSPEAKTHRUCLASS_* constant
       dwMeanThruClass:DWORD;              // @field a RIL_GPRSMEANTHRUCLASS_* constant
     end;
     RILGPRSQOSPROFILE = rilgprsqosprofile_tag;
     LPRILGPRSQOSPROFILE = ^rilgprsqosprofile_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILGPRSANSWER | A quality of service profile
//
// @comm None
//
// -----------------------------------------------------------------------------
//#pragma warning(disable : 4200) // Disable "C4200: nonstandard extension used : zero-sized array in struct/union"
type
     rilgprsanswer_tag = record
       cbSize:DWORD;                       // @field structure size in bytes
       fAnswer:BOOL;                       // @parm TRUE: accept, FALSE: reject
       dwL2Protocol:DWORD;                 // @parm an optional RILL2PROTOCOL_* constant
       dwNumContexts:DWORD;                // @parm number of contexts which follow
       dwContextID:array[0..0] of DWORD;               // @parm identifies the context(s) to enter data state
     end;
     RILGPRSANSWER = rilgprsanswer_tag;
     LPRILGPRSANSWER = ^rilgprsanswer_tag;
//#pragma warning(default : 4200)

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILENTERGPRSDATAMODE | A quality of service profile
//
// @comm None
//
// -----------------------------------------------------------------------------
//#pragma warning(disable : 4200) // Disable "C4200: nonstandard extension used : zero-sized array in struct/union"
type
     rilentergprsdatamode_tag = record
       cbSize:DWORD;                       // @field structure size in bytes
       dwL2Protocol:DWORD;                 // @parm an optional RILL2PROTOCOL_* constant
       dwNumContexts:DWORD;                // @parm number of contexts which follow
       dwContextID:array[0..0] of DWORD;               // @parm identifies the context(s) to enter data state
     end;
     RILENTERGPRSDATAMODE = rilentergprsdatamode_tag;
     LPRILENTERGPRSDATAMODE = ^rilentergprsdatamode_tag;
//#pragma warning(default : 4200)

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILLOCATIONINFO | Contains network location information
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rillocationinfo_tag = record
       cbSize:DWORD;           // @field structure size in bytes
       dwLocationAreaCode:DWORD;      // @field the context number
       dwCellID:DWORD;      // @field the context number
     end;
     RILLOCATIONINFO = rillocationinfo_tag;
     LPRILLOCATIONINFO = ^rillocationinfo_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILGPRSCONTEXTACTIVATED | Shows which contexts are active
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rilgprscontextactivated_tag = record
       cbSize:DWORD;          // @field structure size in bytes
       dwEvent:DWORD;         // @field the disconnection event type: RIL_PARAM_RILGPRSCONTEXTACTIVATED_ (applies to deactivation)
       dwContextID:DWORD;     // @field the context number
       fActivated:BOOL;       // @field whether the context is activated
     end;
     RILGPRSCONTEXTACTIVATED = rilgprscontextactivated_tag;
     LPRILGPRSCONTEXTACTIVATED = ^rilgprscontextactivated_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILLOGATCOMMAND | Contains inbound and outbound AT commands/responses
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rillogatinfo_tag = record
       cbSize:DWORD;                // @field structure size in bytes
       cbLength:DWORD;              // @field command buffer length
       szRsp:array[0..MAXLENGTH_CMD-1] of byte;   // @field command buffer
       fResponse:BOOL;               // @field TRUE if szRsp contains a response; FALSE if szRsp contains a command
     end;
     RILLOGATINFO = rillogatinfo_tag;
     LPRILLOGATINFO = ^rillogatinfo_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILNDISIPV6ADDR | Encapsulates an IPv6 address.
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      IPV6_ADDRESS_LENGTH             = 16;
      IPV6_ADDRESS_LENGTH_IN_UCHAR    = IPV6_ADDRESS_LENGTH;
      IPV6_ADDRESS_LENGTH_IN_USHORT   = IPV6_ADDRESS_LENGTH div 2;

type
     rilndisipv6addr_tag = record
       case USHORT of
         0: (_Byte:array[0..IPV6_ADDRESS_LENGTH_IN_UCHAR-1] of UCHAR);
         1: (_Word:array[0..IPV6_ADDRESS_LENGTH_IN_USHORT-1] of USHORT);
     end;
     RILNDISIPV6ADDR = rilndisipv6addr_tag;
     LPRILNDISIPV6ADDR = ^rilndisipv6addr_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILNDISIPCONFIG | returned in association with  RIL_NOTIFY_NDIS_IPCONFIG
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     Tipv4 = record
       dwFlags:DWORD;           // @field bitfield of valid in_addr parameters defined by RIL_PARAM_NDISIPCONFIG_xxx
       inIPAddress:DWORD;
       inPrimaryDNS:DWORD;
       inSecondaryDNS:DWORD;
       inDefaultGateway:DWORD;
       inSubnetMask:DWORD;
     end;

     Tipv6 = record
       dwFlags:DWORD;           // @field bitfield of valid in_addr parameters defined by RIL_PARAM_NDISIPCONFIG_IPV6_xxx
       inIPAddress:RILNDISIPV6ADDR;
       inPrimaryDNS:RILNDISIPV6ADDR;
       inSecondaryDNS:RILNDISIPV6ADDR;
       inDefaultGateway:RILNDISIPV6ADDR;
       inSubnetMask:RILNDISIPV6ADDR;
       dwFlowInfo:DWORD;
       dwScopeId:DWORD;
     end;

     rilndisipconfig_tag = record
       cbSize:DWORD;                    // @field structure size in bytes
       dwContextId:DWORD;
       dwProtocol:DWORD;                // @field discriminator for the union field; defined by RIL_PARAM_NDISIPCONFIG_PROTOCOL_*
       case DWORD of
         0: (ipv4:Tipv4);
         1: (ipv6:Tipv6);
     end;
     RILNDISIPCONFIG = rilndisipconfig_tag;
     LPRILNDISIPCONFIG = ^rilndisipconfig_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILNDISBUFFER | Buffer defintion for use in rildndispacket_tag below.
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rilndisbuffer_tag = record
       pbyBytes:LPBYTE;                            // @field Pointer to the buffer
       cByteCount:DWORD;                            // @field Number of bytes pointed to by pbyBytes.
     end;
     RILNDISBUFFER = rilndisbuffer_tag;
     LPRILNDISBUFFER = ^rilndisbuffer_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILNDISPACKET |
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rilndispacket_tag = record
       dwContextId:DWORD;
       dwSize:DWORD;
       cBufferCount:DWORD;
       NDISBuffer:array[0..0] of RILNDISBUFFER;
     end;
     RILNDISPACKET = rilndispacket_tag;
     LPRILNDISPACKET = ^rilndispacket_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILNDISSTATUS |
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rilndisstatus_tag = record
       dwContextId:DWORD;    // @parm GPRS context identifier reporting status
       dwSize:DWORD;         // @parm size of this struct.
       dwStatusType:DWORD;   // @parm RIL_PARAM_RILNDISSTATUS_ type
       case longint of
         0: (pRilNdisIpConfig:LPRILNDISIPCONFIG);  // @parm points to data
         1: (dwFlowControl:DWORD);                  // @parm flow control RIL_NDIS_XON or RIL_NDIS_XOFF
     end;
     RILNDISSTATUS = rilndisstatus_tag;
     LPRILNDISSTATUS = ^rilndisstatus_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func RIL NDIS receive packet callback
//
// @comm This function is called to when an IP packet is received by the
//       RIL driver.
//
// -----------------------------------------------------------------------------
type
     RILNDISRECEIVECALLBACK = procedure(pCallbackContext:PVOID;     // @parm parameter passed to <f RIL_NDIS_SetGPRSContextActivated>
                                        pRilPacket:LPRILNDISPACKET     // @parm ponter to received packet
                                       ); cdecl;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func RIL NDIS transmit packet callback
//
// @comm This function is called to when an IP packet is sent by the
//       RIL driver.
//
// -----------------------------------------------------------------------------
type
     RILNDISTRANSMITCALLBACK = procedure(pCallbackContext:PVOID;     // @parm parameter passed to <f RIL_NDIS_SetGPRSContextActivated>
                                         pRilPacket:LPRILNDISPACKET     // @parm ponter to received packet
                                        ); cdecl;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func RIL NDIS status callback
//
// @comm This function is called to when a NDIS packet IO status change occurred in the
//       RIL driver.
//
// -----------------------------------------------------------------------------
type
     RILNDISSTATUSCALLBACK = procedure(pCallbackContext:PVOID;     // @parm parameter passed to <f RIL_NDIS_SetGPRSContextActivated>
                                       pRilStatus:LPRILNDISSTATUS     // @parm ponter to received packet
                                      ); cdecl;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func RIL NDIS Receive Packet Done callback
//
// @comm This function may optionally be used to release packets from the WWAN interface.
//
// -----------------------------------------------------------------------------
type
     PFNRILNDISRECEIVEPACKETDONE = procedure(lpPacketReceived:LPRILNDISPACKET); cdecl;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func RIL NDIS Send Packet callback
//
// @comm This function may optionally be used to send NDIS packets from the WWAN interface.
//
// -----------------------------------------------------------------------------
type
     PFNRILNDISSENDPACKET = procedure(lpPacketToSend:LPRILNDISPACKET); cdecl;

const
      NDIS_GPRS_PASSWORD_MAX_LENGTH = 64;
      NDIS_GPRS_USERNAME_MAX_LENGTH = 64;
      NDIS_GRPS_DNS_MAX_LENGTH      = 64;
      
// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILNDISGPRSCONTEXT |
//
// @comm None
//
// -----------------------------------------------------------------------------

type
     rilndisgprscontext_tag = record
       cbSize:DWORD;             // @field structure size in bytes
       dwParams:DWORD;           // @field indicates valid parameters
       dwContextID:DWORD;        // @field identifies the context
       fContextActivation:BOOL; // @field TRUE: activated, FALSE: deactivated
       tszUserName:array[0..NDIS_GPRS_USERNAME_MAX_LENGTH-1] of TCHAR;      // @field Context activation user name
       tszPassword:array[0..NDIS_GPRS_PASSWORD_MAX_LENGTH-1] of TCHAR;      // @field Context activation password
       tszDNS1:array[0..NDIS_GRPS_DNS_MAX_LENGTH-1] of TCHAR;               // @field Context activation DNS1
       tszDNS2:array[0..NDIS_GRPS_DNS_MAX_LENGTH-1] of TCHAR;               // @fielf Context activation DNS2
       pfnNdisReceive:RILNDISRECEIVECALLBACK;   // @parm function NDIS Rx packet callback
       pfnNdisTransmit:RILNDISTRANSMITCALLBACK;   // @parm function NDIS Tx packet callback
       pfnNdisStatus:RILNDISSTATUSCALLBACK;     // @parm function NDIS status callback
       pCallbackContext:PVOID;               // @parm custom parameter passed to NDIS Rx packet callback
     end;
     RILNDISGPRSCONTEXT = rilndisgprscontext_tag;
     LPRILNDISGPRSCONTEXT = ^rilndisgprscontext_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILNDISGPRSCONTEXTRESPONSE |
//
// @comm None
//
// -----------------------------------------------------------------------------

type
     rilndisgprscontextresponse_tag = record
       cbSize:DWORD;             // @field structure size in bytes
       dwParams:DWORD;           // @field RIL_PARAM_RILNDISGPRSCONTEXTRESPONSE_
       dwContextID:DWORD;        // @field identifies the context
       dwError:DWORD;            // @field error during activation.
       pfnNdisSendPacket:PFNRILNDISSENDPACKET;        // @parm function NDIS Send Packet
       pfnNdisReceivePacketDone:PFNRILNDISRECEIVEPACKETDONE;    // @parm function NDIS Receive Packet Done.
       RilNdisIPConfig:RILNDISIPCONFIG; //@parm IP configuration data obtained during activation.
     end;
     RILNDISGPRSCONTEXTRESPONSE = rilndisgprscontextresponse_tag;
     LPRILNDISGPRSCONTEXTRESPONSE = ^rilndisgprscontextresponse_tag;

type
     RILNDISSETGPRSCONTEXTACTIVATED = RILNDISGPRSCONTEXT;
     LPRILNDISSETGPRSCONTEXTACTIVATED = LPRILNDISGPRSCONTEXT;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILNITZINFO |
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rilnitzinfo_tag = record
       cbSize:DWORD;             // size of this struct.
       dwParams:DWORD;           // valid fields RIL_PARAM_NITZ_
       dwNotificationCode:DWORD; // type of notifcation RIL_NOTIFY_NITZ
       TimeZoneOffsetMinutes:longint;// Indicates the time zone offset +/-
       DaylightSavingOffsetMinutes:longint; // Indicates the daylight saving offset in minutes
       SysTime:SYSTEMTIME;        // If available from network
     end;
     RILNITZINFO = rilnitzinfo_tag;
     LPRILNITZINFO = ^rilnitzinfo_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILSIMSECURITYSTATUS | SIM security status struct
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rilsimsecuritystatus_tag = record
       cbSize:DWORD;                   // @field structure size in bytes
       dwLockedState:DWORD;            // @field one of RIL_LOCKEDSTATE_*
       dwSimSecurityState:DWORD;       // @field one of RIL_SIMSECURITYSTATE_*
     end;
     RILSIMSECURITYSTATUS = rilsimsecuritystatus_tag;
     LPRILSIMSECURITYSTATUS = ^rilsimsecuritystatus_tag;

     
//
// RIL handle type
//
type
     HRIL = HANDLE;
     LPHRIL = ^HRIL;




// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func RIL function result callback
//
// @comm This function is called to send a return value after and asynchronous
//       RIL function call
//
// -----------------------------------------------------------------------------
type
     RILRESULTCALLBACK = procedure(dwCode:DWORD;           // @parm result code
                                   hrCmdID:HRESULT;        // @parm ID returned by the command that originated this response
                                   lpData:pointer;         // @parm data associated with the notification
                                   cbData:DWORD;           // @parm size of the strcuture pointed to lpData
                                   dwParam:DWORD           // @parm parameter passed to <f RIL_Initialize>
                                  ); cdecl;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func RIL notification callback
//
// @comm This function is called when the radio sends an unsolicited notifiation
//
// -----------------------------------------------------------------------------
type
     RILNOTIFYCALLBACK = procedure(dwCode:DWORD;           // @parm notification code
                                   lpData:pointer;         // @parm data associated with the notification
                                   cbData:DWORD;           // @parm size of the strcuture pointed to lpData
                                   dwParam:DWORD           // @parm parameter passed to <f RIL_Initialize>
                                  ); cdecl;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func RIL call type determination callback
//
// @comm This function is called to determine the call type when the
//       following is present in the registry:
//       
//       [HKEY_LOCAL_MACHINE\Comm\Cellular\RIL\RemoteCalltype]
//           "CalltypeDLL"="<libraryName>.dll"
//           "CalltypeFunction"="<functionName>"
//
// -----------------------------------------------------------------------------
type
     RILCALLTYPECALLBACK = function(pRemotePartyInfo:LPRILREMOTEPARTYINFO  // @param remote party info used to determine the call type
                                   ):DWORD; cdecl;


//
// RIL Functions
//

// Comment: contains 132 functions.

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Initializes RIL for use by this client
//
// @comm Synchronous
//      RIL only supports single threaded RIL handles.
//      The RIL validates the application's RIL handle before using it.
//              No application can use/close a RIL handle that it does not own.
//
// -----------------------------------------------------------------------------
function RIL_Initialize(dwIndex:DWORD;                     // @parm index of the RIL port to use (e.g., 1 for RIL1:)
                        pfnResult:RILRESULTCALLBACK;       // @parm function result callback
                        pfnNotify:RILNOTIFYCALLBACK;       // @parm notification callback
                        dwNotificationClasses:DWORD;       // @parm classes of notifications to be enabled for this client
                        dwParam:DWORD;                     // @parm custom parameter passed to result and notififcation callbacks
                        _lphRil:LPHRIL                     // @parm returned handle to RIL instance
                       ):HRESULT; external RILDLL name 'RIL_Initialize';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Initializes RIL for use by this emergency call module
//
// @comm Synchronous
//
// -----------------------------------------------------------------------------
function RIL_InitializeEmergency(dwIndex:DWORD;                      // @parm index of the RIL port to use (e.g., 1 for RIL1:)
                                 pfnResult:RILRESULTCALLBACK;        // @parm function result callback
                                 pfnNotify:RILNOTIFYCALLBACK;        // @parm notification callback
                                 dwNotificationClasses:DWORD;        // @parm classes of notifications to be enabled for this client
                                 dwParam:DWORD;                      // @parm custom parameter passed to result and notififcation callbacks
                                 _lphRil:LPHRIL                      // @parm returned handle to RIL instance
                                ):HRESULT; external RILDLL name 'RIL_InitializeEmergency';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Deinitializes RIL
//
// @comm Synchronous
//
// -----------------------------------------------------------------------------
function RIL_Deinitialize(
                          _hRil:HRIL      // @parm handle to an RIL instance returned by <f RIL_Initialize>
                         ):HRESULT; external RILDLL name 'RIL_Deinitialize';
// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Enables additional classes of notifications for this client
//
// @comm Synchronous
//
// -----------------------------------------------------------------------------

function RIL_EnableNotifications(
                                 _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                                 dwNotificationClasses:DWORD         // @parm classes of notifications to enable
                                ):HRESULT; external RILDLL name 'RIL_EnableNotifications';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Disables classes of notifications for this client
//
// @comm Synchronous
//
// -----------------------------------------------------------------------------
function RIL_DisableNotifications(
                                  _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                                  dwNotificationClasses:DWORD         // @parm classes of notifications to disable
                                 ):HRESULT; external RILDLL name 'RIL_DisableNotifications';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Disables classes of notifications for this client
//
// @comm Synchronous
//
// -----------------------------------------------------------------------------
function RIL_RegisterATCommandLogging(_hRil:HRIL;           // @parm handle to RIL instance returned by <f RIL_Initialize>
                                      fEnable:BOOL          // @parm flag to turn feature on or off.
                                     ):HRESULT;  external RILDLL name 'RIL_RegisterATCommandLogging';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Creates a log file of recent AT commands
//
// @comm Asynchronous.
//       For Microsoft Test only. This will not be fully implemented on every
//       platform. A return response of E_NOTIMPL will be returned in the
//       default case.
//
//       DO NOT IMPLEMENT THIS.
//
// -----------------------------------------------------------------------------
function RIL_ATCommandLogFile(_hRil:HRIL;                          // @parm handle to RIL instance returned by <f RIL_Initialize>
                              pszFilename:LPCTSTR                  // @parm String containing the filename for the log.
                             ):HRESULT;  external RILDLL name 'RIL_ATCommandLogFile';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Retrieves a serial port handle to be used for data communications
//
// @comm Synchronous.  Client is responsible for closing the handle returned in <p lphSerial>.
//
// -----------------------------------------------------------------------------
function RIL_GetSerialPortHandle(
                                 _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                                 lphSerial:LPHANDLE                   // @parm pointer to the serial port handle
                                ):HRESULT; external RILDLL name 'RIL_GetSerialPortHandle';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Retrieves a serial port handle to be used for video telephony
//
// @comm Synchronous.  Client is responsible for closing the handle returned in <p lphSerial>.
//
// -----------------------------------------------------------------------------
function RIL_GetVTSerialPortHandle(_hRil:HRIL;                // @parm handle to RIL instance returned by <f RIL_Initialize>
                                   lphSerial:LPHANDLE                   // @parm pointer to the serial port handle
                                  ):HRESULT; external RILDLL name 'RIL_GetVTSerialPortHandle';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Retrieves a serial port handle to be used for data communications
//
// @comm Synchronous.  Client is responsible for closing the handle returned in <p lphSerial>.
//
// -----------------------------------------------------------------------------
function RIL_GetSerialPortHandleFromContextID(_hRil:HRIL;            // @parm handle to RIL instance returned by <f RIL_Initialize>
                                              dwContextID:DWORD;     // @parm PDP context identifier.
                                              lphSerial:LPHANDLE                   // @parm pointer to the serial port handle
                                             ):HRESULT; external RILDLL name 'RIL_GetSerialPortHandleFromContextID';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Retrieves a serial port handle statistics
//
// @comm Synchronous
//
// -----------------------------------------------------------------------------
function RIL_GetSerialPortStatistics(
                                     _hRil:HRIL;                              // @parm handle to RIL instance returned by <f RIL_Initialize>
                                     lpSerialPortStats:LPRILSERIALPORTSTATS   // @parm pointer to the statistics structure
                                    ):HRESULT; external RILDLL name 'RIL_GetSerialPortStatistics';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Restrieves the driver version
//
// @comm Synchronous
//
// -----------------------------------------------------------------------------
function RIL_GetDriverVersion(
                              _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                              pdwVersion:LPDWORD                   // @parm pointer to version.  HIWORD is major version, LOWORD is minor version
                             ):HRESULT; external RILDLL name 'RIL_GetDriverVersion';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Restrieves information about subscriber numbers
//
// @comm Asynchronous.  <p lpData> points to an array of <t RILSUBSCRIBERINFO> structures.
//
// -----------------------------------------------------------------------------
function RIL_GetSubscriberNumbers(
                                  _hRil:HRIL                           // @parm handle to RIL instance returned by <f RIL_Initialize>
                                 ):HRESULT; external RILDLL name 'RIL_GetSubscriberNumbers';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Retrieves the list of available operators
//
// @comm Asynchronous.  <p lpData> points to an array of <t RILOPERATORINFO> structures.
//
// -----------------------------------------------------------------------------
function RIL_GetOperatorList(
                             _hRil:HRIL                           // @parm handle to RIL instance returned by <f RIL_Initialize>
                            ):HRESULT; external RILDLL name 'RIL_GetOperatorList';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Retrieves the built-in list of all known operators.
//       This is not the list of operators available, for that see RIL_GetOperatorList.
//
// @comm Asynchronous.  <p lpData> points to an array of <t RILOPERATORNAMES> structures.
//
// -----------------------------------------------------------------------------
function RIL_GetAllOperatorsList(_hRil:HRIL                           // @parm handle to RIL instance returned by <f RIL_Initialize>
                                ):HRESULT; external RILDLL name 'RIL_GetAllOperatorsList';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Retrieves the list of preferred operators
//
// @comm Asynchronous.  <p lpData> points to an array of <t RILOPERATORINFO> structures.
//
// -----------------------------------------------------------------------------
function RIL_GetPreferredOperatorList(
                                      _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                                      dwFormat:DWORD                      // @parm format to use for the operator names in the list
                                     ):HRESULT; external RILDLL name 'RIL_GetPreferredOperatorList';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Adds a specified operator to the list of preferred operators
//
// @comm Asynchronous.  <p lpData> is <def NULL>
//
// -----------------------------------------------------------------------------
function RIL_AddPreferredOperator(
                                  _hRil:HRIL;                             // @parm handle to RIL instance returned by <f RIL_Initialize>
                                  dwIndex:DWORD;                          // @parm storage index to use for the added operator
                                  lpOperatorNames:LPRILOPERATORNAMES      // @parm operator name
                                 ):HRESULT; external RILDLL name 'RIL_AddPreferredOperator';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Removes a specified operator from the list of preferred operators
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_RemovePreferredOperator(
                                     _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                                     dwIndex:DWORD                       // @parm storage index of the preferred operator to remove
                                    ):HRESULT; external RILDLL name 'RIL_RemovePreferredOperator';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Retrieves the operator the ME is currently registered with
//
// @comm Asynchronous.  <p lpData> points to an <t RILOPERATORNAMES> structure.
//
// -----------------------------------------------------------------------------
function RIL_GetCurrentOperator(
                                _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize()>
                                dwFormat:DWORD                      // @parm format of the operator name to return (<def RIL_OPFORMAT_> constant)
                               ):HRESULT; external RILDLL name 'RIL_GetCurrentOperator';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Registers the ME with a network operator
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_RegisterOnNetwork(
                               _hRil:HRIL;                             // @parm handle to RIL instance returned by <f RIL_Initialize>
                               dwMode:DWORD;                           // @parm operator selection mode (<def RIL_OPSELMODE_> constant)
                               lpOperatorNames:LPRILOPERATORNAMES      // @parm operator to be selected (can be <def NULL> if <p dwMode> is <def RIL_OPSELMODE_AUTOMATIC>)
                              ):HRESULT; external RILDLL name 'RIL_RegisterOnNetwork';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Unregisters the ME from the current newtwork operator
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_UnregisterFromNetwork(
                                   _hRil:HRIL                           // @parm handle to RIL instance returned by <f RIL_Initialize>
                                  ):HRESULT; external RILDLL name 'RIL_UnregisterFromNetwork';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Retrieves the current phone registration status
//
// @comm Asynchronous.  <p lpData> points to a <def RIL_REGSTAT_> constant.
//
// -----------------------------------------------------------------------------
function RIL_GetRegistrationStatus(
                                   _hRil:HRIL;             // @parm handle to RIL instance returned by <f RIL_Initialize>
                                   pdwRegStatus:LPDWORD    // @parm points to returned <def RIL_REGSTAT_> constant
                                  ):HRESULT; external RILDLL name 'RIL_GetRegistrationStatus';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Retrieves the current CallerID settings
//
// @comm Asynchronous.  <p lpData> points to an <t RILCALLERIDSETTINGS> structure.
//
// -----------------------------------------------------------------------------
function RIL_GetCallerIdSettings(
                                 _hRil:HRIL                           // @parm handle to RIL instance returned by <f RIL_Initialize>
                                ):HRESULT; external RILDLL name 'RIL_GetCallerIdSettings';


// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Sets the current CallerID status
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_SetCallerIdStatus(
                               _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                               dwStatus:DWORD                      // @parm status to be set (<def RIL_SVCSTAT_> constant)
                              ):HRESULT; external RILDLL name 'RIL_SetCallerIdStatus';


// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Retrieves the current HideID settings
//
// @comm Asynchronous.  <p lpData> points to an <t RILHIDEIDSETTINGS> structure.
//
// -----------------------------------------------------------------------------
function RIL_GetHideIdSettings(
                               _hRil:HRIL                           // @parm handle to RIL instance returned by <f RIL_Initialize>
                              ):HRESULT; external RILDLL name 'RIL_GetHideIdSettings';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Enables or disables HideID service
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_SetHideIdStatus(
                             _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                             dwStatus:DWORD                      // @parm status to be set (<def RIL_SVCSTAT_> constant)
                            ):HRESULT; external RILDLL name 'RIL_SetHideIdStatus';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Retrieves the current DialedID settings
//
// @comm Asynchronous.  <p lpData> points to an <t RILDIALEDIDSETTINGS> structure.
//
// -----------------------------------------------------------------------------
function RIL_GetDialedIdSettings(
                                 _hRil:HRIL                           // @parm handle to RIL instance returned by <f RIL_Initialize>
                                ):HRESULT; external RILDLL name 'RIL_GetDialedIdSettings';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Sets the current DialedID settings
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_SetDialedIdStatus(
                               _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                               dwStatus:DWORD                      // @parm status to be set (<def RIL_SVCSTAT_> constant)
                              ):HRESULT; external RILDLL name 'RIL_SetDialedIdStatus';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Retrieves the current HideConnectedID settings
//
// @comm Asynchronous.  <p lpData> points to an <t RILHIDECONNECTEDIDSETTINGS> structure.
//
// -----------------------------------------------------------------------------
function RIL_GetHideConnectedIdSettings(
                                        _hRil:HRIL                           // @parm handle to RIL instance returned by <f RIL_Initialize>
                                       ):HRESULT; external RILDLL name 'RIL_GetHideConnectedIdSettings';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Sets the current HideConnectedID settings
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_SetHideConnectedIdStatus(
                                      _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                                      dwStatus:DWORD                      // @parm status to be set (<def RIL_SVCSTAT_> constant)
                                     ):HRESULT; external RILDLL name 'RIL_SetHideConnectedIdStatus';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Retrieves the status for a Completion of Call to Busy Subscriber index.
//
// @comm Asynchronous.  If active, <p lpData> points to an array of <t char>s
//       indicating the phone number for which CCBS is active.  If CCBS is not
//       active for that entry, <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_GetCCBSStatus(
                           _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                           dwCCBSIndex:DWORD                   // @parm indicates which entry to query
                          ):HRESULT; external RILDLL name 'RIL_GetCCBSStatus';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Clears registration for a Completion of Call to Busy Subscriber index.
//       Activation of CCBS is used by calling RIL_ManageCalls using the
//       <def RIL_CALLCMD_INVOKECCBS> flag.
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_ClearCCBSRegistration(
                                   _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                                   dwCCBSIndex:DWORD                   // @parm indicates which entry to clear, may be <def RIL_CCBS_ALL>
                                  ):HRESULT; external RILDLL name 'RIL_ClearCCBSRegistration';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Retrieves the current Closed User Group settings
//
// @comm Asynchronous.  <p lpData> points to an <t RILCLOSEDGROUPSETTINGS> structure.
//
// -----------------------------------------------------------------------------
function RIL_GetClosedGroupSettings(
                                    _hRil:HRIL                           // @parm handle to RIL instance returned by <f RIL_Initialize>
                                   ):HRESULT; external RILDLL name 'RIL_GetClosedGroupSettings';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Sets the Closed User Group settings
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_SetClosedGroupSettings(
                                    _hRil:HRIL;                                 // @parm handle to RIL instance returned by <f RIL_Initialize>
                                    lpSettings:LPRILCLOSEDGROUPSETTINGS         // @parm settings to be set
                                   ):HRESULT; external RILDLL name 'RIL_SetClosedGroupSettings';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Retrieves current Call Forwarding rules
//
// @comm Asynchronous.  <p lpData> points to an array of <t RILCALLFORWARDINGSETTINGS> structure.
//
// -----------------------------------------------------------------------------
function RIL_GetCallForwardingSettings(
                                       _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                                       dwReason:DWORD;                     // @parm forwarding reason to retrieve the settings for (<def RIL_FWDREASON_> constant)
                                       dwInfoClass:DWORD                   // @parm information class to retrieve barring status for (<def RIL_INFOCLASS_> constant)
                                      ):HRESULT; external RILDLL name 'RIL_GetCallForwardingSettings';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Adds a Call Forwarding rule
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_AddCallForwarding(
                               _hRil:HRIL;                                 // @parm handle to RIL instance returned by <f RIL_Initialize>
                               dwReason:DWORD;                             // @parm forwarding reason to add Call Forwarding for (<def RIL_FWDREASON_> constant)
                               lpSettings:LPRILCALLFORWARDINGSETTINGS // @parm settings for the new Call Forwarding rule
                              ):HRESULT; external RILDLL name 'RIL_AddCallForwarding';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Removes a Call Forwarding rule
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_RemoveCallForwarding(
                                  _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                                  dwReason:DWORD;                     // @parm forwarding reason to remove Call Forwarding for (<def RIL_FWDREASON_> constant)
                                  dwInfoClasses:DWORD                 // @parm information classes to remove Call Forwarding for (combination of <def RIL_INFOCLASS_> constants)
                                 ):HRESULT; external RILDLL name 'RIL_RemoveCallForwarding';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Enables or disables the specified Call Forwarding rule
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_SetCallForwardingStatus(
                                     _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                                     dwReason:DWORD;                     // @parm forwarding reason to enable/disable Call Forwarding for (<def RIL_FWDREASON_> constant)
                                     dwInfoClasses:DWORD;                // @parm information classes to enable/disable Call Forwarding for (combination of <def RIL_INFOCLASS_> constants)
                                     dwStatus:DWORD                      // @parm status to be set (<def RIL_SVCSTAT_> constant)
                                    ):HRESULT; external RILDLL name 'RIL_SetCallForwardingStatus';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Retrieves info classes that Call Waiting is currently enabled for
//
// @comm Asynchronous.  <p lpData> points to DWORD containing a combination
//       of <def RIL_INFOCLASS_> constants.
//
// -----------------------------------------------------------------------------
function RIL_GetCallWaitingSettings(
                                    _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                                    dwInfoClass:DWORD                   // @parm information class to retrieve barring status for (<def RIL_INFOCLASS_> constant)
                                   ):HRESULT; external RILDLL name 'RIL_GetCallWaitingSettings';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Enables or disables Call Waiting for the specified info class
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_SetCallWaitingStatus(
                                  _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                                  dwInfoClasses:DWORD;                // @parm information classes to enable/disable Call Waiting for
                                  dwStatus:DWORD                      // @parm status to be set (<def RIL_SVCSTAT_> constant)
                                 ):HRESULT; external RILDLL name 'RIL_SetCallWaitingStatus';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Sends supplementary service (USSD) data
//
// @comm TBD
//
// -----------------------------------------------------------------------------
function RIL_SendSupServiceData(
                                _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                                pwszData:LPCWSTR                    // @parm data to be sent
                               ):HRESULT; external RILDLL name 'RIL_SendSupServiceData';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Cancels current supplementary service session
//
// @comm TBD
//
// -----------------------------------------------------------------------------
function RIL_CancelSupServiceDataSession(
                                         _hRil:HRIL                           // @parm handle to RIL instance returned by <f RIL_Initialize>
                                        ):HRESULT; external RILDLL name 'RIL_CancelSupServiceDataSession';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Retrieves the current address identifier (see RILSUBSCRIBERINFO)
//
// @comm Asynchronous.  <p lpData> points to a <def DWORD> identifying the current address ID.
//
// -----------------------------------------------------------------------------
function RIL_GetCurrentAddressId(
                                 _hRil:HRIL                           // @parm handle to RIL instance returned by <f RIL_Initialize>
                                ):HRESULT; external RILDLL name 'RIL_GetCurrentAddressId';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Sets the current address identifier (see RILSUBSCRIBERINFO)
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_SetCurrentAddressId(
                                 _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                                 dwAddressId:DWORD                   // @parm identifies the new addressID to use
                                ):HRESULT; external RILDLL name 'RIL_SetCurrentAddressId';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Dials a specified address
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_Dial(
                  _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                  lpszAddress:PAnsiChar;                 // @parm address to dial (no longer than <def MAXLENGTH_ADDRESS> chars)
                  dwType:DWORD;                       // @parm type of the call to establish (<def RIL_CALLTYPE_> constant)
                  dwOptions:DWORD                     // @parm dialing options (any combination of <def RIL_DIALOPT_> constants)
                 ):HRESULT; external RILDLL name 'RIL_Dial';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Answers an incoming call
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_Answer(
                    _hRil:HRIL                           // @parm handle to RIL instance returned by <f RIL_Initialize>
                   ):HRESULT; external RILDLL name 'RIL_Answer';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Hangs up all calls currently in progress
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_Hangup(
                    _hRil:HRIL                           // @parm handle to RIL instance returned by <f RIL_Initialize>
                   ):HRESULT; external RILDLL name 'RIL_Hangup';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Sends DTMF tones across an established voice call
//
// @comm Asynchronous.  <p lpData> is <def NULL>.  Function does not return until
//       DTMF tone has completed.
//
// -----------------------------------------------------------------------------
function RIL_SendDTMF(
                      _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                      lpszChars:PAnsiChar;                // @parm alphanumeric string representing DTMF tones to be sent (0-9, A-D, *, #)
                      dwDuration:DWORD                    // @parm new DTMF tone duration in milliseconds (<def RIL_DTMFDURATION_DEFAULT>
                                                          // corresponds to the manufacturer's default value)
                     ):HRESULT; external RILDLL name 'RIL_SendDTMF';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Starts DTMF tone across an established voice call
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_StartDTMF(
                       _hRil:HRIL;       // @parm handle to RIL instance returned by <f RIL_Initialize>
                       ch:AnsiChar       // @parm alphanumeric char representing DTMF tones to be sent (0-9, A-D, *, #)
                      ):HRESULT; external RILDLL name 'RIL_StartDTMF';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Stops DTMF tones across an established voice call
//
// @comm Asynchronous.  <p lpData> is <def NULL>.  Function does not return until
//       DTMF tone has completed.
//
// -----------------------------------------------------------------------------
function RIL_StopDTMF(
                      _hRil:HRIL;       // @parm handle to RIL instance returned by <f RIL_Initialize>
                      ch:AnsiChar       // @parm alphanumeric char representing DTMF tones to be stopped (0-9, A-D, *, #)
                     ):HRESULT; external RILDLL name 'RIL_StopDTMF';
                     
// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Detects DTMF tones from an established voice call
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_SetDTMFMonitoring(
                               _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                               fEnable:BOOL                        // @parm TRUE to initiate DTMF monitoring; FALSE to cancel
                              ):HRESULT; external RILDLL name 'RIL_SetDTMFMonitoring';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Retrieves the list of active, held, and waiting calls
//
// @comm Asynchronous.  <p lpData> points to an array of <t RILCALLINFO> structures.
//
// -----------------------------------------------------------------------------
// Function RIL_GetCallList is not declared in ril.h for unknown reason, but
// is still exported by ril.dll.
function RIL_GetCallList(
                         _hRil:HRIL                           // @parm handle to RIL instance returned by <f RIL_Initialize>
                        ):HRESULT; external RILDLL name 'RIL_GetCallList';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Modifies the state of active, held, and waiting calls
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_ManageCalls(
                         _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                         dwCommand:DWORD;                    // @parm call modification command to be performed (<def RIL_CALLCMD_> constant)
                         dwID:DWORD                          // @parm ID of the call to be modified (only for <def RIL_CALLCMD_RELEASECALL> and <def RIL_CALLCMD_HOLDALLBUTONE>)
                        ):HRESULT; external RILDLL name 'RIL_ManageCalls';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Transfers incoming allerting call to the specified number
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_TransferCall(
                          _hRil:HRIL;                  // @parm handle to RIL instance returned by <f RIL_Initialize>
                          lpAddress:LPRILADDRESS;      // @parm address to transfer the call to
                          lpSubAddress:RILSUBADDRESS   // @parm sub-address to transfer the call to (can be <def NULL>)
                         ):HRESULT; external RILDLL name 'RIL_TransferCall';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Retrieves the phone line status
//
// @comm Asynchronous.  <p lpData> points to <t DWORD> containing <def RIL_LINESTAT_> constant.
//
// -----------------------------------------------------------------------------
// Function RIL_GetLineStatus is not declared in ril.h for unknown reason, but
// is still exported by ril.dll.
function RIL_GetLineStatus(
                           _hRil:HRIL                           // @parm handle to RIL instance returned by <f RIL_Initialize>
                          ):HRESULT; external RILDLL name 'RIL_GetLineStatus';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Retrieves audio gain information
//
// @comm Asynchronous.  <p lpData> points to an <t RILGAININFO> structure.
//
// -----------------------------------------------------------------------------
function RIL_GetAudioGain(
                          _hRil:HRIL                           // @parm handle to RIL instance returned by <f RIL_Initialize>
                         ):HRESULT; external RILDLL name 'RIL_GetAudioGain';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Sets audio gain information
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_SetAudioGain(
                          _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                          lpGainInfo:LPRILGAININFO       // @parm audio gain information to be sent
                         ):HRESULT; external RILDLL name 'RIL_SetAudioGain';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Retrieves currently used transmit and receive audio devices
//
// @comm Asynchronous.  <p lpData> points to an <t RILAUDIODEVICEINFO> structure.
//
// -----------------------------------------------------------------------------
function RIL_GetAudioDevices(
                             _hRil:HRIL                           // @parm handle to RIL instance returned by <f RIL_Initialize>
                            ):HRESULT; external RILDLL name 'RIL_GetAudioDevices';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Sets currently used transmit and receive audio devices
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_SetAudioDevices(
                             _hRil:HRIL;                                 // @parm handle to RIL instance returned by <f RIL_Initialize>
                             lpAudioDeviceInfo:LPRILAUDIODEVICEINFO // @parm audio devices to set
                            ):HRESULT; external RILDLL name 'RIL_SetAudioDevices';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Determines whether the input audio device is muted
//
// @comm Asynchronous.  <p lpData> points to a <t BOOL>.
//
// -----------------------------------------------------------------------------
function RIL_GetAudioMuting(
                            _hRil:HRIL                           // @parm handle to RIL instance returned by <f RIL_Initialize>
                           ):HRESULT; external RILDLL name 'RIL_GetAudioMuting';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Mutes or un-mutes the input audio device
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_SetAudioMuting(
                            _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                            fEnable:BOOL                        // @parm TRUE if input audio device is to be muted; FALSE otherwise
                           ):HRESULT; external RILDLL name 'RIL_SetAudioMuting';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Retrieves High Speeed Circuit Switched Data options
//
// @comm Asynchronous.  <p lpData> points to an <t RILHSCSDINFO> structure.
//
// -----------------------------------------------------------------------------
function RIL_GetHSCSDOptions(
                             _hRil:HRIL                           // @parm handle to RIL instance returned by <f RIL_Initialize>
                            ):HRESULT; external RILDLL name 'RIL_GetHSCSDOptions';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Sets High Speeed Circuit Switched Data options
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_SetHSCSDOptions(
                             _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                             lpHscsdInfo:LPRILHSCSDINFO     // @parm High Speeed Circuit Switched Data options to set
                            ):HRESULT; external RILDLL name 'RIL_SetHSCSDOptions';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Retrieves High Speeed Circuit Switched Data options
//
// @comm Asynchronous.  <p lpData> points to an <t RILCALLHSCSDINFO> structure.
//
// -----------------------------------------------------------------------------
function RIL_GetHSCSDCallSettings(
                                  _hRil:HRIL                           // @parm handle to RIL instance returned by <f RIL_Initialize>
                                 ):HRESULT; external RILDLL name 'RIL_GetHSCSDCallSettings';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Retrieves data compression options
//
// @comm Asynchronous.  <p lpData> points to an <t RILDATACOMPINFO> structure.
//
// -----------------------------------------------------------------------------
function RIL_GetDataCompression(
                                _hRil:HRIL;                          // @parm handle to RIL instance returned by <f RIL_Initialize>
                                pDataCompInfo:LPRILDATACOMPINFO      // @parm data compression options to return
                               ):HRESULT; external RILDLL name 'RIL_GetDataCompression';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Sets data compression options
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_SetDataCompression(
                                _hRil:HRIL;                             // @parm handle to RIL instance returned by <f RIL_Initialize>
                                lpDataCompInfo:LPRILDATACOMPINFO   // @parm data compression options to set
                               ):HRESULT; external RILDLL name 'RIL_SetDataCompression';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Retrieves error correction options
//
// @comm Asynchronous.  <p lpData> points to an <t RILERRORCORRECTIONINFO> structure.
//
// -----------------------------------------------------------------------------
function RIL_GetErrorCorrection(
                                _hRil:HRIL;                          // @parm handle to RIL instance returned by <f RIL_Initialize>
                                lpErrorCorrectionInfo:LPRILERRORCORRECTIONINFO // @parm error correction options to return
                               ):HRESULT; external RILDLL name 'RIL_GetErrorCorrection';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Set error correction options
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_SetErrorCorrection(
                                _hRil:HRIL;                                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                                lpErrorCorrectionInfo:LPRILERRORCORRECTIONINFO // @parm error correction options to set
                               ):HRESULT; external RILDLL name 'RIL_SetErrorCorrection';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Retrieves currently set data bearer service options
//
// @comm Asynchronous.  <p lpData> points to an <t RILBEARERSVCINFO> structure.
//
// -----------------------------------------------------------------------------
function RIL_GetBearerServiceOptions(
                                     _hRil:HRIL;                           // @parm handle to RIL instance returned by <f RIL_Initialize>
                                     lpBearerServiceInfo:LPRILBEARERSVCINFO // @parm data bearer service options to return
                                    ):HRESULT; external RILDLL name 'RIL_GetBearerServiceOptions';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Sets data bearer service options
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_SetBearerServiceOptions(
                                     _hRil:HRIL;                                 // @parm handle to RIL instance returned by <f RIL_Initialize>
                                     lpBearerServiceInfo:LPRILBEARERSVCINFO // @parm data bearer service options to set
                                    ):HRESULT; external RILDLL name 'RIL_SetBearerServiceOptions';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Retrieves currently set Radio Link Protocol options
//
// @comm Asynchronous.  <p lpData> points to an <t RILRLPINFO> structure.
//
// -----------------------------------------------------------------------------
function RIL_GetRLPOptions(
                           _hRil:HRIL;                          // @parm handle to RIL instance returned by <f RIL_Initialize>
                           lpRlpInfo:LPRILRLPINFO         // @parm Radio Link Protocol options to return
                          ):HRESULT; external RILDLL name 'RIL_GetRLPOptions';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Sets Radio Link Protocol options
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_SetRLPOptions(
                           _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                           lpRlpInfo:LPRILRLPINFO         // @parm Radio Link Protocol options to set
                          ):HRESULT; external RILDLL name 'RIL_SetRLPOptions';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Gets currently set messaging service options
//
// @comm Asynchronous.  <p lpData> points to an <t RILMSGSERVICEINFO> structure.
//
// -----------------------------------------------------------------------------
function RIL_GetMsgServiceOptions(
                                  _hRil:HRIL                           // @parm handle to RIL instance returned by <f RIL_Initialize>
                                 ):HRESULT; external RILDLL name 'RIL_GetMsgServiceOptions';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Sets messaging service options
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_SetMsgServiceOptions(
                                  _hRil:HRIL;                                 // @parm handle to RIL instance returned by <f RIL_Initialize>
                                  lpMsgServiceInfo:LPRILMSGSERVICEINFO   // @parm messaging service options to be set
                                 ):HRESULT; external RILDLL name 'RIL_SetMsgServiceOptions';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Gets currently set messaging configuration
//
// @comm Asynchronous.  <p lpData> points to an <t RILMSGCONFIG> structure.
//
// -----------------------------------------------------------------------------
function RIL_GetMsgConfig(
                          _hRil:HRIL                           // @parm handle to RIL instance returned by <f RIL_Initialize>
                         ):HRESULT; external RILDLL name 'RIL_GetMsgConfig';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Sets messaging configuration
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_SetMsgConfig(
                          _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                          lpMsgConfigInfo:LPRILMSGCONFIG // @parm messaging configuration to be set
                         ):HRESULT; external RILDLL name 'RIL_SetMsgConfig';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Gets cell broadcast messaging configuration
//
// @comm Asynchronous.  <p lpData> points to an <t RILCBMSGCONFIG> structure.
//
// -----------------------------------------------------------------------------
function RIL_GetCellBroadcastMsgConfig(
                                       _hRil:HRIL                           // @parm handle to RIL instance returned by <f RIL_Initialize>
                                      ):HRESULT; external RILDLL name 'RIL_GetCellBroadcastMsgConfig';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Sets cell broadcast messaging configuration
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_SetCellBroadcastMsgConfig(
                                       _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                                       lpCbMsgConfigInfo:LPRILCBMSGCONFIG // @parm messaging configuration to be set
                                      ):HRESULT; external RILDLL name 'RIL_SetCellBroadcastMsgConfig';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Reads a message from the current storage location
//
// @comm Asynchronous.  <p lpData> points to an <t RILMESSAGEINFO> structure.
//
// -----------------------------------------------------------------------------
function RIL_ReadMsg(
                     _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                     dwIndex:DWORD                       // @parm index of the message to be read
                    ):HRESULT; external RILDLL name 'RIL_ReadMsg';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Deletes a message from the current storage location
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_DeleteMsg(
                       _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                       dwIndex:DWORD                       // @parm index of the message to be deleted
                      ):HRESULT; external RILDLL name 'RIL_DeleteMsg';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Writes a message to the current storage location
//
// @comm Asynchronous.  <p lpData> points to a <t DWORD> conaining the index used.
//
// -----------------------------------------------------------------------------
function RIL_WriteMsg(
                      _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                      lpMessage:LPRILMESSAGE;        // @parm message to be written (of type <def RIL_MSGTYPE_IN_DELIVER> or <def RIL_MSGTYPE_OUT_SUBMIT>)
                      dwStatus:DWORD                      // @parm status to assigned to the written message (<def RIL_MSGSTATUS_> constant)
                     ):HRESULT; external RILDLL name 'RIL_WriteMsg';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Sends a message
//
// @comm Asynchronous.  <p lpData> points to a <t DWORD> conaining the reference
//       number of the sent message.
//
// -----------------------------------------------------------------------------
function RIL_SendMsg(
                     _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                     lpMessage:LPRILMESSAGE;        // @parm message to be sent
                     dwOptions:DWORD                     // @parm options (any combination of <def RIL_SENDOPT_> constants)
                    ):HRESULT; external RILDLL name 'RIL_SendMsg';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Sends a message from the current storage location
//
// @comm Asynchronous.  <p lpData> points to a <t DWORD> conaining the reference
//       number of the sent message.  This feature is not used and is untested.
//
// -----------------------------------------------------------------------------
function RIL_SendStoredMsg(
                           _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                           dwIndex:DWORD;                      // @parm index of the message to be sent
                           dwOptions:DWORD                     // @parm options (any combination of <def RIL_SENDOPT_> constants)
                          ):HRESULT; external RILDLL name 'RIL_SendStoredMsg';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Sends an message ackknowledgement
//
// @comm Asynchronous.  <p lpData> is <def NULL>.  On Phase 2 mobiles, the radio
//       automatically sends SMS message ACKs.  But in Phase 2+, the MMI is
//       responsible for these ACKs, hense this function.
//
// -----------------------------------------------------------------------------
function RIL_SendMsgAcknowledgement(
                                    _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                                    fSuccess:BOOL                       // @parm TRUE if success acknowledgment is to be sent; FALSE otherwise
                                   ):HRESULT; external RILDLL name 'RIL_SendMsgAcknowledgement';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Retrieves International Mobile Subscriber Identity of the phone user
//
// @comm Asynchronous.  <p lpData> points to an array of <t char>s
//
// -----------------------------------------------------------------------------
function RIL_GetUserIdentity(
                             _hRil:HRIL                           // @parm handle to RIL instance returned by <f RIL_Initialize>
                            ):HRESULT; external RILDLL name 'RIL_GetUserIdentity';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Retrieves current locked state of the phone
//
// @comm Asynchronous.  <p lpData> points to a <t DWORD> containing a <def RIL_LOCKEDSTATE_> constant
//
// -----------------------------------------------------------------------------
function RIL_GetPhoneLockedState(
                                 _hRil:HRIL                           // @parm handle to RIL instance returned by <f RIL_Initialize>
                                ):HRESULT; external RILDLL name 'RIL_GetPhoneLockedState';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Removes current lock applied to the phone
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_UnlockPhone(
                         _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                         lpszPassword:PAnsiChar;                // @parm password to unlock the phone (no longer than <def MAXLENGTH_PASSWORD> chars)
                         lpszNewPassword:PAnsiChar              // @parm new password (can be <def NULL>, unless the current locked state is
                                                                //     one of the <def RIL_LOCKEDSTATE_*_PUK> constants; no longer than
                                                                //     <def MAXLENGTH_PASSWORD> chars)
                        ):HRESULT; external RILDLL name 'RIL_UnlockPhone';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Retrieves locking status for the specified facility
//
// @comm Asynchronous.  <p lpData> points to a <t DWORD> containing a <def RIL_LOCKINGSTATUS_> constant.
//
// -----------------------------------------------------------------------------
function RIL_GetLockingStatus(
                              _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                              dwFacility:DWORD;                   // @parm facility to retrieve locking status for (<def RIL_LOCKFACILITY_> constant)
                              lpszPassword:PAnsiChar                 // @parm password to retrieve locking status (can be <def NULL> if password isn't required;
                                                                     //     no longer than MAXLENGTH_PASSWORD chars)
                             ):HRESULT; external RILDLL name 'RIL_GetLockingStatus';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Enables or disables locking status for the specified facility
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_SetLockingStatus(
                              _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                              dwFacility:DWORD;                   // @parm facility to enable/disable locking for (<def RIL_LOCKFACILITY_> constant)
                              lpszPassword:PAnsiChar;             // @parm password to enable/disable locking (can be <def NULL> if password isn't required;
                                                                  //     no longer than <def MAXLENGTH_PASSWORD> chars)
                              dwStatus:DWORD                      // @parm status to be set (<def RIL_LOCKINGSTATUS_> constant)
                             ):HRESULT; external RILDLL name 'RIL_SetLockingStatus';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Changes locking password for the specified facility
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_ChangeLockingPassword(
                                   _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                                   dwFacility:DWORD;                   // @parm facility to change locking password for (<def RIL_LOCKFACILITY_> constant)
                                   dwOldPasswordType:DWORD;            // @parm the type of OLD password (PIN or PUK) RIL_PASSWORDTYPE_* constant
                                   lpszOldPassword:PAnsiChar;             // @parm current locking password (no longer than <def MAXLENGTH_PASSWORD> chars)
                                   lpszNewPassword:PAnsiChar              // @parm new locking password (no longer than <def MAXLENGTH_PASSWOR> chars)
                                  ):HRESULT; external RILDLL name 'RIL_ChangeLockingPassword';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Retrieves status of the specified type of call barring
//
// @comm Asynchronous.  <p lpData> points to a <t DWORD> containing a <def RIL_BARRINGSTATUS> constant.
//
// -----------------------------------------------------------------------------
function RIL_GetCallBarringStatus(
                                  _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                                  dwType:DWORD;                       // @parm type of call barring to retrieve status for (<def RIL_BARRTYPE_> constant)
                                  dwInfoClass:DWORD;                  // @parm information class to retrieve barring status for (<def RIL_INFOCLASS_> constant)
                                  lpszPassword:PAnsiChar                 // @parm password to retrieve barring status (can be <def NULL> if password isn't required;
                                                                      //     no longer than <def MAXLENGTH_PASSWORD> chars)
                                 ):HRESULT; external RILDLL name 'RIL_GetCallBarringStatus';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Enables or disables the specified type of call barring
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_SetCallBarringStatus(
                                  _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                                  dwType:DWORD;                       // @parm type of call barring to enable/disable (<def RIL_BARRTYPE_> constant)
                                  dwInfoClass:DWORD;                  // @parm information class to enable/disable call barring for (<def RIL_INFOCLASS_> constant)
                                  lpszPassword:AnsiChar;                // @parm password to enable/disable call barring (can be <def NULL> if password isn't required;
                                                                      //     no longer than <def MAXLENGTH_PASSWORD> chars)
                                  dwStatus:DWORD                      // @parm status to be set (<def RIL_BARRINGSTATUS_> constant)
                                 ):HRESULT; external RILDLL name 'RIL_SetCallBarringStatus';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Changes password for the specified type of call barring
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_SendSecureMmiString(_hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                                 lpszMmiString:LPCSTR                // @parm MMI string to be sent
                                ):HRESULT; external RILDLL name 'RIL_SendSecureMmiString';


// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Changes password for the specified type of call barring
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_ChangeCallBarringPassword(
                                       _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                                       dwType:DWORD;                       // @parm type of call barring to retrieve status for (<def RIL_BARRTYPE_> constant)
                                       lpwszOldPassword:PAnsiChar;            // @parm current password (no longer than <def MAXLENGTH_PASSWORD> chars)
                                       lpwszNewPassword:PAnsiChar             // @parm new password (no longer than <def MAXLENGTH_PASSWORD> chars)
                                      ):HRESULT; external RILDLL name 'RIL_ChangeCallBarringPassword';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Retrieves manufacturer equipment information
//
// @comm Asynchronous.  <p lpData> points to an <t RILEQUIPMENTINFO> structure.
//
// -----------------------------------------------------------------------------
function RIL_GetEquipmentInfo(
                              _hRil:HRIL                           // @parm handle to RIL instance returned by <f RIL_Initialize>
                             ):HRESULT; external RILDLL name 'RIL_GetEquipmentInfo';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Retrieves currently set equipment state
//
// @comm Asynchronous.  <p lpData> points to an <t RILEQUIPMENTSTATE> structure.
//
// -----------------------------------------------------------------------------
function RIL_GetEquipmentState(
                               _hRil:HRIL;                          // @parm handle to RIL instance returned by <f RIL_Initialize>
                               pEqState:LPRILEQUIPMENTSTATE         // @parm equipment state to return
                              ):HRESULT; external RILDLL name 'RIL_GetEquipmentState';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Sets the equipment to the specified state
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_SetEquipmentState(
                               _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                               dwEquipmentState:DWORD              // @parm equipment state to set (<def RIL_EQSTATE_> constant)
                              ):HRESULT; external RILDLL name 'RIL_SetEquipmentState';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Proxy API to determine if the Radio is present or Not (Is the RIL driver Loaded?)
//
// @comm Synchronous
//
// -----------------------------------------------------------------------------
function RIL_GetRadioPresence(
                              _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                              dwRadioPresence:LPDWORD              // @parm pointer to a DWORD (ouput param contains values from RIL_RADIOPRESENCE_*)
                             ):HRESULT; external RILDLL name 'RIL_GetRadioPresence';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Retrieves currently set phonebook options
//
// @comm Asynchronous.  <p lpData> points to an <t RILPHONEBOOKINFO> structure.
//
// -----------------------------------------------------------------------------
function RIL_GetPhonebookOptions(
                                 _hRil:HRIL                           // @parm handle to RIL instance returned by <f RIL_Initialize>
                                ):HRESULT; external RILDLL name 'RIL_GetPhonebookOptions';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Sets phonebook options
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_SetPhonebookOptions(
                                 _hRil:HRIL;                             // @parm handle to RIL instance returned by <f RIL_Initialize>
                                 lpPhonebookInfo:LPRILPHONEBOOKINFO // @parm phonebook options to set
                                ):HRESULT; external RILDLL name 'RIL_SetPhonebookOptions';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Reads phonebook entries from the specified range of indices of the current storage location
//
// @comm Asynchronous.  <p lpData> points to an array of <t RILPHONEBOOKENTRY> structures.
//
// -----------------------------------------------------------------------------
function RIL_ReadPhonebookEntries(
                                  _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                                  dwStartIndex:DWORD;                 // @parm starting index of the range
                                  dwEndIndex:DWORD                    // @parm ending index of the range
                                 ):HRESULT; external RILDLL name 'RIL_ReadPhonebookEntries';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Writes a phonebook entry to the current storage location
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_WritePhonebookEntry(
                                 _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                                 lpEntry:LPRILPHONEBOOKENTRY    // @parm phonebook entry to write out
                                ):HRESULT; external RILDLL name 'RIL_WritePhonebookEntry';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Deletes a phonebook entry from the current storage location
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_DeletePhonebookEntry(
                                  _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                                  dwIndex:DWORD                       // @parm index of the entry to delete
                                 ):HRESULT; external RILDLL name 'RIL_DeletePhonebookEntry';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Sends a specified command to the SIM
//
// @comm Asynchronous.  <p lpData> points to an array of <t BYTE>s.
//
// -----------------------------------------------------------------------------
function RIL_SendSimCmd(
                        _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                        lpbCommand:LPBYTE;             // @parm command to be sent to the SIM
                        dwSize:DWORD                        // @parm size of the data pointed to by <p lpbCommand> in bytes
                       ):HRESULT; external RILDLL name 'RIL_SendSimCmd';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Retrieves the SIM's answer to reset data.
//
// @comm Asynchronous.  <p lpData> points to an <t RILATRINFO> structure.
// This command is not standardized and may be specific to each radio
// implementation, if implemented at all.
//
// -----------------------------------------------------------------------------
function RIL_GetATR(
                    _hRil:HRIL                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                   ):HRESULT; external RILDLL name 'RIL_GetATR';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Sends a specified restricted command to the SIM
//
// @comm Asynchronous.  <p lpData> points to an <t RILSIMRESPONSE> structure.
//
// -----------------------------------------------------------------------------
function RIL_SendRestrictedSimCmd(
                                  _hRil:HRIL;                                 // @parm handle to RIL instance returned by <f RIL_Initialize>
                                  dwCommand:DWORD;                            // @parm restricted command to be sent to the SIM (<def RIL_SIMCMD_> constant)
                                  lpParameters:LPRILSIMCMDPARAMETERS;    // @parm Parameters for the command to be sent (can be <def NULL> if parameters aren't required)
                                  lpbData:LPBYTE;                        // @parm Data to be written to the SIM (can be <def NULL> if data isn't required)
                                  dwSize:DWORD                                // @parm Size of the data pointed to by <p lpbData> in bytes
                                 ):HRESULT; external RILDLL name 'RIL_SendRestrictedSimCmd';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Retrieves SIM Record Status
//
// @comm Asynchronous.  <p lpData> points to RILSIMRECORDSTATUS
//
// -----------------------------------------------------------------------------
function RIL_GetSimRecordStatus(
                                _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                                dwFileID:DWORD                      // @parm address of the file to read
                               ):HRESULT; external RILDLL name 'RIL_GetSimRecordStatus';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Retrieves SIM Toolkit terminal profile
//
// @comm Asynchronous.  <p lpData> points to an array of <t BYTE>s.
//
// -----------------------------------------------------------------------------
function RIL_GetSimToolkitProfile(
                                  _hRil:HRIL                           // @parm handle to RIL instance returned by <f RIL_Initialize>
                                 ):HRESULT; external RILDLL name 'RIL_GetSimToolkitProfile';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Sets SIM Toolkit terminal profile
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_SetSimToolkitProfile(
                                  _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                                  lpbProfile:LPBYTE;                  // @parm SIM Toolkit profile to be set
                                  dwSize:DWORD                        // @parm size of the data pointed to by <p lpbProfile> in bytes
                                 ):HRESULT; external RILDLL name 'RIL_SetSimToolkitProfile';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Sends a SIM Toolkit envelope command
//
// @comm Asynchronous.  <p lpData> points to an array of <t BYTE>s containing a
//       response to the sent command.
//
// -----------------------------------------------------------------------------
function RIL_SendSimToolkitEnvelopeCmd(
                                       _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                                       lpbCommand:LPBYTE;             // @parm SIM Toolkit envelope command to be sent
                                       dwSize:DWORD                        // @parm size of the data pointed to by <p lpbCommand> in bytes
                                      ):HRESULT; external RILDLL name 'RIL_SendSimToolkitEnvelopeCmd';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Fetches a SIM Toolkit command from the SIM
//
// @comm Asynchronous.  <p lpData> points to an array of <t BYTE>s containing a
//       fetched command.
//
// -----------------------------------------------------------------------------
function RIL_FetchSimToolkitCmd(
                                _hRil:HRIL                           // @parm handle to RIL instance returned by <f RIL_Initialize>
                               ):HRESULT; external RILDLL name 'RIL_FetchSimToolkitCmd';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Sends a response to an executed SIM Toolkit command
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_SendSimToolkitCmdResponse(
                                       _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                                       pRsp:LPRILSIMTOOLKITRSP;       // @parm Command Response to be sent.
                                       pDetails:LPBYTE;            // @parm Detailed command response to be sent (can be <def NULL> if details aren't required)
                                       dwDetailSize:DWORD                        // @parm size of the details pointed to by <p pDetails> in bytes
                                      ):HRESULT; external RILDLL name 'RIL_SendSimToolkitCmdResponse';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Terminates the SIM Toolkit session
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_TerminateSimToolkitSession(
                                        _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                                        dwCause:DWORD                       // @parm cause for session termination (<def RIL_SIMTKITTERMCAUSE_> constant)
                                       ):HRESULT; external RILDLL name 'RIL_TerminateSimToolkitSession';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Sends a requested Event to the SIM.
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_SendSimToolkitEventDownload(
                                         _hRil:HRIL;                   // @parm handle to RIL instance returned by <f RIL_Initialize>
                                         dwEvent:DWORD;                // @parm Event to be sent.
                                         pData:LPBYTE;                 // @parm Detailed event info to be sent (can be <def NULL> if details aren't required)
                                         dwDataSize:DWORD              // @parm size of the details pointed to by <p pDetails> in bytes
                                        ):HRESULT; external RILDLL name 'RIL_SendSimToolkitEventDownload';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Retrieves advice-of-charge settings
//
// @comm Asynchronous.  <p lpData> points to a <t RILCOSTINFO> structure.
//       This feature is not used and is untested.
//
// -----------------------------------------------------------------------------
function RIL_GetCostInfo(
                         _hRil:HRIL                           // @parm handle to RIL instance returned by <f RIL_Initialize>
                        ):HRESULT; external RILDLL name 'RIL_GetCostInfo';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Sets advice-of-charge settings
//
// @comm Asynchronous.  <p lpData> points to a <t RILCOSTINFO> structure.
//       This feature is not used and is untested.
//
// -----------------------------------------------------------------------------
function RIL_SetCostInfo(
                         _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                         lpCostInfo:LPRILCOSTINFO;      // @parm advice-of-charge settings to set
                         lpszPassword:LPCSTR                 // @parm password requred to set advice-of-charge settings
                        ):HRESULT; external RILDLL name 'RIL_SetCostInfo';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Retrieves information about the received signal quality
//
// @comm Asynchronous.  <p lpData> points to a <t RILSIGNALQUALITY> structure.
//
// -----------------------------------------------------------------------------
// Function RIL_GetSignalQuality is not declared in ril.h for unknown reason, but
// is still exported by ril.dll.
function RIL_GetSignalQuality(
                              _hRil:HRIL                           // @parm handle to RIL instance returned by <f RIL_Initialize>
                             ):HRESULT; external RILDLL name 'RIL_GetSignalQuality';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Retrieves information about the cell tower currently used by the phone
//
// @comm Asynchronous.  <p lpData> points to a <t RILCELLTOWERINFO> structure.
//
// -----------------------------------------------------------------------------
function RIL_GetCellTowerInfo(
                              _hRil:HRIL                           // @parm handle to RIL instance returned by <f RIL_Initialize>
                             ):HRESULT; external RILDLL name 'RIL_GetCellTowerInfo';

// Initial size of the RILDRVNOTIFICATION structure. The size RIL Application allocates for async results of RIL_DevSpecific
const
      RIL_NOTIFYTHREAD_ALLOCSIZE      = 256;
      RIL_DEVSPECIFIC_ASYNC_RESPONSE_TIMEOUT      = 5000;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Performs an implementation-specific operation
//
// @comm Synchronous
//
// -----------------------------------------------------------------------------
function RIL_DevSpecific(_hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                         lpbParams:LPBYTE;              // @parm parameters for the operation to be performed
                         dwSize:DWORD;                        // @parm size of the data pointed to by <p lpParams> in bytes
                         pbAsyncResults:LPBYTE;               // @parm points to returned array of <t BYTE>s
                         dwAsyncResultsSize:DWORD;           // @parm the allocated size of the returned array of <t BYTE>s
                         pcbAsyncResults:LPDWORD;             // @parm points to the actual length of the returned array of <t BYTE>s
                         dwTimeOut:DWORD                     // @parm time out value (in milliseconds)
                        ):HRESULT; external RILDLL name 'RIL_DevSpecific';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Retrieves specified device capabilities
//
// @comm Asynchronous. <nl><nl><p dwCapsType> (<def RIL_CAPSTYPE_>)           <p lpData><nl>
//                     <def *_DIAL>                   points to an <t RILCAPSDIAL> structure<nl>
//                     <def *_DTMFDURATIONRANGE>      points to an <t RILRANGE> structure (values in milliseconds)<nl>
//                     <def *_CALLMGTCMDS>            points to <t DWORD> containing a combination of <def RIL_CAPS_CALLCMD_> constants<nl>
//                     <def *_BEARERSERVICE>          points to an  <t RILCAPSBEARERSVC> structure<nl>
//                     <def *_RLP>                    points to an array of <t RILAPSRLP> structures<nl>
//                     <def *_EQUIPMENTSTATES>        points to <t DWORD> containing a combination of <def RIL_CAPS_EQSTATE_> constants<nl>
//                     <def *_PBSTORELOCATIONS>       points to <t DWORD> containing a combination of <def RIL_CAPS_PBLOC_> constants<nl>
//                     <def *_PBINDEXRANGE>           points to an <t RILRANGE> structure<nl>
//                     <def *_PBENTRYLENGTH>          points to an <t RILCAPSPBENTRYLENGTH> strcuture<nl>
//                     <def *_MSGSERVICETYPES>        points to <t DWORD> containing a combination of <def RIL_CAPS_MSGSVCTYPE_> constants<nl>
//                     <def *_MSGMEMORYLOCATIONS>     points to an <t RILCAPSMSGMEMORYLOCATIONS> structure<nl>
//                     <def *_BROADCASTMSGLANGS>      points to <t DWORD> containing a combination of <def RIL_CAPS_DCSLANG_> constants<nl>
//                     <def *_MSGCONFIGINDEXRANGE>    points to an <t RILRANGE> structure<nl>
//                     <def *_MSGSTATUSVALUES>        points to <t DWORD> containing a combination of <def RIL_CAPS_MSGSTATUS_> constants<nl>
//                     <def *_PREFOPINDEXRANGE>       points to an <t RILRANGE> structure<nl>
//                     <def *_LOCKFACILITIES>         points to <t DWORD> containing a combination of <def RIL_CAPS_LOCKFACILITY_> constants<nl>
//                     <def *_LOCKINGPWDLENGTHS>      points to an array of <t RILCAPSLOCKINGPWDLENGTH> structures<nl>
//                     <def *_BARRTYPES>              points to <t DWORD> containing a combination of <def RIL_CAPS_BARRTYPE_> constants<nl>
//                     <def *_BARRINGPWDLENGTHS>      points to an array of <t RILCAPSBARRINGPWDLENGTH> structures<nl>
//                     <def *_FORWARDINGREASONS>      points to <t DWORD> containing a combination of <def RIL_CAPS_FWDREASON_> constants<nl>
//                     <def *_SIMTOOLKITNOTIFICATIONS>points to a <t TBD> SIMTOOLKIT structure <nl>
//                     <def *_INFOCLASSES>            points to <t DWORD> containing a combination of <def RIL_CAPS_INFOCLASS_> constants<nl>
//                     <def *_HSCSD>                  points to an <t RILCAPSHSCSD> structure<nl>
//                     <def *_GPRS>                   points to an <t RILCAPSGPRS> structure<nl>
//                     <def *_RIL_CAPS_NITZ_>         points to <t DWORD> containing one of <def RIL_CAPS_NITZ__> constants<nl>

function RIL_GetDevCaps(
                        _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                        dwCapsType:DWORD                    // @parm type of caps class to retrieve
                       ):HRESULT; external RILDLL name 'RIL_GetDevCaps';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Retrieves the systemtime from the network
//
// @comm Asynchronous.  <p lpData> points to a <t SYSTEMTIME> structure (containing the UTC time).
//       This feature is currently not used and is untested.
//
// -----------------------------------------------------------------------------
function RIL_GetSystemTime(
                           _hRil:HRIL                           // @parm handle to RIL instance returned by <f RIL_Initialize>
                          ):HRESULT; external RILDLL name 'RIL_GetSystemTime';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Retrieves a list GPRS contexts
//
// @comm Asynchronous.  <p lpData> points to a <t RILGPRSCONTEXT> structure.
//
// -----------------------------------------------------------------------------
function RIL_GetGPRSContextList(
                                _hRil:HRIL;                          // @parm handle to RIL instance returned by <f RIL_Initialize>
                                lpGPRSContext:LPRILGPRSCONTEXT;         // @parm points to the returned <t RILGPRSCONTEXT> structure
                                pdwDataSize:LPDWORD                    // @parm points to returned data size
                               ):HRESULT; external RILDLL name 'RIL_GetGPRSContextList';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Sets a particular GPRS context
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_SetGPRSContext(
                            _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                            lpGprsContext:LPRILGPRSCONTEXT      // @parm points to a <t RILGPRSCONTEXT> structure
                           ):HRESULT; external RILDLL name 'RIL_SetGPRSContext';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Deletes a particular GPRS context
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_DeleteGPRSContext(
                               _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                               dwContextID:DWORD                   // @parm identifies which context to delete
                              ):HRESULT; external RILDLL name 'RIL_DeleteGPRSContext';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Gets the requested quality of service profile for all contexts
//
// @comm Asynchronous.  <p lpData> points to a <t RILGPRSQOSPROFILE> structure.
//
// -----------------------------------------------------------------------------
function RIL_GetRequestedQualityOfServiceList(
                                              _hRil:HRIL;                          // @parm handle to RIL instance returned by <f RIL_Initialize>
                                              lpGprsQosProfile:LPRILGPRSQOSPROFILE;          // @parm points to returned <t RILGPRSQOSPROFILE> structure
                                              pdwDataSize:LPDWORD          // @parm points to returned data size
                                             ):HRESULT; external RILDLL name 'RIL_GetRequestedQualityOfServiceList';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Sets the requested quality of service profile for a context
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_SetRequestedQualityOfService(
                                          _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                                          lpGprsQosProfile:LPRILGPRSQOSPROFILE // @parm points to a <t RILGPRSQOSPROFILE> structure
                                         ):HRESULT; external RILDLL name 'RIL_SetRequestedQualityOfService';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Deletes the requested quality of service profile for a context
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_DeleteRequestedQualityOfService(
                                             _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                                             dwContextID:DWORD                   // @parm identifies which profile to delete
                                            ):HRESULT; external RILDLL name 'RIL_DeleteRequestedQualityOfService';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Gets the minimum quality of service profile for all contexts
//
// @comm Asynchronous.  <p lpData> points to a <t RILGPRSQOSPROFILE> structure.
//
// -----------------------------------------------------------------------------
function RIL_GetMinimumQualityOfServiceList(
                                            _hRil:HRIL;                          // @parm handle to RIL instance returned by <f RIL_Initialize>
                                            lpGprsQosProfile:LPRILGPRSQOSPROFILE;          // @parm points to returned <t RILGPRSQOSPROFILE> structure
                                            pdwDataSize:LPDWORD          // @parm points to returned data size
                                           ):HRESULT; external RILDLL name 'RIL_GetMinimumQualityOfServiceList';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Sets the minimum quality of service profile for a context
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_SetMinimumQualityOfService(
                                        _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                                        lpGprsQosProfile:LPRILGPRSQOSPROFILE // @parm points to a <t RILGPRSQOSPROFILE> structure
                                       ):HRESULT; external RILDLL name 'RIL_SetMinimumQualityOfService';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Deletes the minimum quality of service profile for a context
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_DeleteMinimumQualityOfService(
                                           _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                                           dwContextID:DWORD                   // @parm identifies which profile to delete
                                          ):HRESULT; external RILDLL name 'RIL_DeleteMinimumQualityOfService';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Sets the GPRS attach state
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_SetGPRSAttached(
                             _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                             fAttached:BOOL                      // @parm TRUE: attached, FALSE: detached
                            ):HRESULT; external RILDLL name 'RIL_SetGPRSAttached';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Gets the GPRS attach state
//
// @comm Asynchronous.  <p lpData> points to a <t BOOL> indicating attach state.
//
// -----------------------------------------------------------------------------
function RIL_GetGPRSAttached(
                             _hRil:HRIL;                           // @parm handle to RIL instance returned by <f RIL_Initialize>
                             pfGPRSAttached:LPBOOL             // @parm indicates attach state
                            ):HRESULT; external RILDLL name 'RIL_GetGPRSAttached';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Sets the GPRS activation state for a context
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_SetGPRSContextActivated(
                                     _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                                     dwContextID:DWORD;                  // @parm identifies the context
                                     fContextActivation:BOOL             // @parm TRUE: activated, FALSE: deactivated
                                    ):HRESULT; external RILDLL name 'RIL_SetGPRSContextActivated';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Gets the GPRS activation state for all contexts
//
// @comm Asynchronous.  <p lpData> points to a <t RILGPRSCONTEXTACTIVATED> indicating activation state.
//
// -----------------------------------------------------------------------------
function RIL_GetGPRSContextActivatedList(
                                         _hRil:HRIL;                          // @parm handle to RIL instance returned by <f RIL_Initialize>
                                         lpGPRSContextActivated:LPRILGPRSCONTEXTACTIVATED;        // @parm points to returned <t RILGPRSCONTEXTACTIVATED> indicating activation state
                                         pdwDataSize:LPDWORD                                     // @parm points to returned data size
                                        ):HRESULT; external RILDLL name 'RIL_GetGPRSContextActivatedList';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Enters into GPRS data state
//
// @comm Asynchronous.  <p lpData> if <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_EnterGPRSDataMode(
                               _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                               lpEnterGprsDataMode:LPRILENTERGPRSDATAMODE // @parm points to a <t RILENTERGPRSDATAMODE> structure
                              ):HRESULT; external RILDLL name 'RIL_EnterGPRSDataMode';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Gets the PDP address for a particular context
//
// @comm Asynchronous.  <p lpData> points to an array of <t WCHAR> values indicating the address.
//
// -----------------------------------------------------------------------------
function RIL_GetGPRSAddress(
                            _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                            dwContextID:DWORD;                   // @parm identifies the context
                            pwszGPRSAddress:PWCHAR;             // @parm returns the GPRS Address
                            pGprsAddrCch:LPDWORD     // @parm IN:  The dereferenced specifies the character count of the buffer pointed to by pwszGPRSAddress
                                                     //       OUT: The dereferenced specifies the number of characters copied including terminating null character
                                                     //            or the required character count of the buffer including terminating null character to succeed
                           ):HRESULT; external RILDLL name 'RIL_GetGPRSAddress';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Answers an incoming GPRS activation request
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_GPRSAnswer(
                        _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                        lpGprsAnswer:LPRILGPRSANSWER   // @param points to a <t RILGPRSANSWER> structure
                       ):HRESULT; external RILDLL name 'RIL_GPRSAnswer';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Retrieves the current GPRS registration status
//
// @comm Asynchronous.  <p lpData> points to a <def RIL_REGSTAT_> constant.
//
// -----------------------------------------------------------------------------
function RIL_GetGPRSRegistrationStatus(
                                       _hRil:HRIL;                           // @parm handle to RIL instance returned by <f RIL_Initialize>
                                       pdwGPRSRegStatus:DWORD         // @parm points to returned <def RIL_REGSTAT_> constant
                                      ):HRESULT; external RILDLL name 'RIL_GetGPRSRegistrationStatus';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Retrieves the current GPRS class
//
// @comm Asynchronous.  <p lpData> points to a <def RIL_GPRSCLASS_> constant.
//
// -----------------------------------------------------------------------------
function RIL_GetGPRSClass(
                          _hRil:HRIL                           // @parm handle to RIL instance returned by <f RIL_Initialize>
                         ):HRESULT; external RILDLL name 'RIL_GetGPRSClass';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Sets the current GPRS class
//
// @comm Asynchronous.  <p lpData> is <def NULL>
//
// -----------------------------------------------------------------------------
function RIL_SetGPRSClass(
                          _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                          dwClass:DWORD                       // @parm a RIL_GPRSCLASS_* constant
                         ):HRESULT; external RILDLL name 'RIL_SetGPRSClass';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Retrieves the preferred SMS service option for mobile originated messages
//
// @comm Asynchronous.  <p lpData> points to a <def RIL_MOSMSSERVICE_> constant.
//
// -----------------------------------------------------------------------------
function RIL_GetMOSMSService(
                             _hRil:HRIL                           // @parm handle to RIL instance returned by <f RIL_Initialize>
                            ):HRESULT; external RILDLL name 'RIL_GetMOSMSService';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Sets the preferred SMS service option for mobile originated messages
//
// @comm Asynchronous.  <p lpData> is <def NULL>
//
// -----------------------------------------------------------------------------
function RIL_SetMOSMSService(
                             _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                             dwMoSmsService:DWORD                // @parm a RIL_MOSMSSERVICE_* constant
                            ):HRESULT; external RILDLL name 'RIL_SetMOSMSService';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @params RILBYTECOUNTER
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_PARAM_BC_RXBYTECOUNT                    = $00000001; // @paramdefine
      RIL_PARAM_BC_TXBYTECOUNT                    = $00000002; // @paramdefine
      RIL_PARAM_BC_TOTALBYTECOUNT                 = $00000004; // @paramdefine
      RIL_PARAM_BC_ALL                            = $00000007; // @paramdefine

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @struct RILBYTECOUNTER | Represents the cumulative number of bytes transferred by the radio (packet).
//
// @comm None
//
// -----------------------------------------------------------------------------
type
     rilbytecounter_tag = record
       cbSize:DWORD;                           // @field structure size in bytes
       dwParams:DWORD;                         // @field indicates valid parameters
       dwRxByte:DWORD;                         // @field Number of received bytes
       dwTxByte:DWORD;                         // @field Number of transmitted bytes
       dwTotalByte:DWORD;                      // @field Total Number of bytes transferred (This comes from the radio, not RxByte+TxByte)
     end;
     RILBYTECOUNTER = rilbytecounter_tag;
     LPRILBYTECOUNTER = ^rilbytecounter_tag;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Retrieves the cumulative count of data bytes transferred by the radio (packet)
//
// @comm Asynchronous.  <p lpData> points to a <t RILBYTECOUNTER> structure.
//
// -----------------------------------------------------------------------------
function RIL_GetPacketByteCount(
                                _hRil:HRIL                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                               ):HRESULT; external RILDLL name 'RIL_GetPacketByteCount';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Resets the cumulative count of data bytes transferred by the radio (packet) to zero.
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_ResetPacketByteCount(
                                  _hRil:HRIL                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                                 ):HRESULT; external RILDLL name 'RIL_ResetPacketByteCount';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants System Coverage | Current System Coverage
//
// @comm Various Levels of CDMA and GSM Coverage
//
// -----------------------------------------------------------------------------
const
      RIL_SYSTEMTYPE_NONE         = $00000000;      // @constdegine No Networks in Coverage
      RIL_SYSTEMTYPE_IS95A        = $00000001;      // @constdefine IS-95A network support (Low Packet, or Circuit Switched Service)
      RIL_SYSTEMTYPE_IS95B        = $00000002;      // @constdefine IS-95B network support
      RIL_SYSTEMTYPE_1XRTTPACKET  = $00000004;      // @constdefine CDMA-2000 Rev A (1xRTT) network support
      RIL_SYSTEMTYPE_GSM          = $00000008;      // @constdefine GSM network support
      RIL_SYSTEMTYPE_GPRS         = $00000010;      // @constdefine GPRS support
      RIL_SYSTEMTYPE_EDGE         = $00000020;      // @constdefine GSM EDGE network support
      RIL_SYSTEMTYPE_1XEVDOPACKET = $00000040;      // @constdefine CDMA (1xEVDO) network support
      RIL_SYSTEMTYPE_1XEVDVPACKET = $00000080;      // @constdefine CDMA (1xEVDV) network support
      RIL_SYSTEMTYPE_UMTS         = $00000100;      // @constdefine UMTS network support
      RIL_SYSTEMTYPE_HSDPA        = $00000200;      // @constdefine HSDPA support

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Gets the Current System Coverage
//
// @comm Gets the Current type of System/Cellular connection that is available.
//       Asynchronous.  <p lpData> is <t DWORD> of type RIL_SYSTEMTYPE_ flags)
//
// -----------------------------------------------------------------------------
function RIL_GetCurrentSystemType(
                                  _hRil:HRIL                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                                 ):HRESULT; external RILDLL name 'RIL_GetCurrentSystemType';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Modifies the state of active, held, and waiting calls
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_SendFlash(
                       _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                       lpraRilAddress:LPRILADDRESS         // @parm flash address
                      ):HRESULT; external RILDLL name 'RIL_SendFlash';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants CDMA Roaming Types | CDMA Roaming Types
//
// @comm The meaning of AUTOMATICA and AUTOMATICB is up to network specific interpretations
//
// -----------------------------------------------------------------------------
const
      RIL_ROAMMODE_HOMEONLY           = $00000001;            // @constdefine The User will never go off the home network
      RIL_ROAMMODE_AUTOMATICA         = $00000002;            // @constdefine Network define Roaming A (The effect of this setting is carrier dependent)
      RIL_ROAMMODE_AUTOMATICB         = $00000003;            // @constdefine Network define Roaming B (The effect of this setting is carrier dependent)

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Gets the Roaming Mode in CDMA
//
// @comm Asynchronous.  <p lpData> points to a <t DWORD> of type RIL_ROAMMODE_*.
//
// -----------------------------------------------------------------------------
function RIL_GetRoamingMode(
                            _hRil:HRIL                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                           ):HRESULT; external RILDLL name 'RIL_GetRoamingMode';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Modifies the state of active, held, and waiting calls in CDMA and AMPS systems
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_SetRoamingMode(
                            _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                            dwRoamingMode:DWORD                 // @parm RIL_ROAMMODE_* constant
                           ):HRESULT; external RILDLL name 'RIL_SetRoamingMode';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants CDMA Privacy Mode | CDMA Privacy Mode
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_CALLPRIVACY_STANDARD                       = $00000001; // @constdefine Enhanced Call Privacy is OFF
      RIL_CALLPRIVACY_ENHANCED                       = $00000002; // @constdefine Enhanced Call Privacy is ON

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Gets the user's preferred privacy settings
//
// @comm Asynchronous.  <p lpData> points to a RIL_CALLPRIVACY_* constant.
//
// -----------------------------------------------------------------------------
function RIL_GetPreferredPrivacyMode(
                                     _hRil:HRIL                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                                    ):HRESULT; external RILDLL name 'RIL_GetPreferredPrivacyMode';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Sets the user's preferred privacy settings
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_SetPreferredPrivacyMode(
                                     _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                                     dwPreferredPrivacyMode:DWORD        // @parm user's preferred privacy setting, uses RIL_CALLPRIVACY_* constant
                                    ):HRESULT; external RILDLL name 'RIL_SetPreferredPrivacyMode';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Gets the privacy status of the current system
//
// @comm Asynchronous.  <p lpData> points to a <t DWORD> RIL_CALLPRIVACY_* constant.
//
// -----------------------------------------------------------------------------
function RIL_GetCurrentPrivacyStatus(
                                     _hRil:HRIL                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                                    ):HRESULT; external RILDLL name 'RIL_GetCurrentPrivacyStatus';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Sends the User string to the device for Akey verificaiton
//
// @comm Asynchronous. <p lpData> is <def NULL>. (Either it fails or succeeds)
// According to TSB-50 (up to 26 digits)
//
// -----------------------------------------------------------------------------

function RIL_SendAKey(
                      _hRil:HRIL;                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                      lpszChars:LPCSTR // @parm numeric string representing akey digits (0-9, *, #)
                     ):HRESULT; external RILDLL name 'RIL_SendAKey';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants CDMA Location Serivces Status | Location Services
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_LOCATION_OFF                             = 0;
      RIL_LOCATION_ON                              = 1;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Gets the current location status of the current system
//
// @comm Asynchronous.  <p lpData> points to a <t DWORD> RIL_LOCATION_* constant.
//
// -----------------------------------------------------------------------------
function RIL_GetCurrentLocationStatus(
                                      _hRil:HRIL                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                                     ):HRESULT; external RILDLL name 'RIL_GetCurrentLocationStatus';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants CDMA Roaming Status | CDMA Roaming Status
//
// @comm None
//
// -----------------------------------------------------------------------------
const
      RIL_ROAMSTATUS_NONE                          = 0;
      RIL_ROAMSTATUS_ANALOG                        = 1;
      RIL_ROAMSTATUS_DIGITAL                       = 2;

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Gets the current roaming status of the current system
//
// @comm Asynchronous.  <p lpData> points to a <t DWORD> RIL_ROAMSTATUS_* constant.
//
// -----------------------------------------------------------------------------
function RIL_GetCurrentRoamingStatus(
                                     _hRil:HRIL                         // @parm handle to RIL instance returned by <f RIL_Initialize>
                                    ):HRESULT; external RILDLL name 'RIL_GetCurrentRoamingStatus';

// -----------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @constants SIM Refresh Constants | Constants which indicate which cached SIM regions to refresh.
//
// @comm The notification contains a DWORD value which contains the SIM record to refresh,
//       or one of the special values below
//
// -----------------------------------------------------------------------------
const
      RIL_SIM_DATACHANGE_MSISDNS                        = $ffffffff;
      RIL_SIM_DATACHANGE_ALL_SIMRECORDS                 = $fffffffe;
      RIL_SIM_DATACHANGE_ALL_SIMPB                      = $fffffffd;
      RIL_SIM_DATACHANGE_ALL                            = $fffffffc;

// ---------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Sets the GPRS activation state for a context for an NDIS connection.
//
// @comm Asynchronous.  <p lpData> points to a <t DWORD> containing the error
//						detail on failure or the context ID on success.
//
// -----------------------------------------------------------------------------
function RIL_NDIS_SetGPRSContextActivated(
                                          _hRil:HRIL;                 // @parm handle to RIL instance returned by <f RIL_Initialize>
                                          lpNdisSetGprsContextActivated:LPRILNDISGPRSCONTEXT // @parm
                                         ):HRESULT; external RILDLL name 'RIL_NDIS_SetGPRSContextActivated';

// ---------------------------------------------------------------------------
//
// @doc EXTERNAL
//
// @func Send an arbitrary string to the radio for logging purposes
//
// @comm Asynchronous.  <p lpData> is <def NULL>.
//
// -----------------------------------------------------------------------------
function RIL_LogEventToRadio(
                             _hRil:HRIL;          // @parm handle to RIL instance returned by <f RIL_Initialize>
                             lpszChars:LPCSTR     // @parm
                            ):HRESULT; external RILDLL name 'RIL_LogEventToRadio';


implementation

function MAKE_RILERROR(errclass:byte; code:byte):ULONG; inline;
begin
  MAKE_RILERROR:=(ULONG(errclass) shl 8) or ULONG(code);
end;

function RILERRORCLASS(rilerror:ULONG):byte; inline;
begin
  RILERRORCLASS:=byte(ULONG((rilerror shr 8) and $FF));
end;

function ISRILERROR(rilerror:ULONG):BOOL; inline;
begin
  ISRILERROR:=(word(rilerror shr 16) and FACILITY_RIL)<>0;
end;

//
// Macro to extract notification class from notification code
//
function NCLASS_FROM_NOTIFICATION(code:ULONG):ULONG; inline;
begin
  NCLASS_FROM_NOTIFICATION:=code and $FFFF0000;
end;

//
// Macro to extract message class from message type
//
function MSGCLASS_FROM_MSGTYPE(itype:ULONG):ULONG; inline;
begin
  MSGCLASS_FROM_MSGTYPE:=itype and $FFFF0000;
end;

end.
