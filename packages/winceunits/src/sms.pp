{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2008 Free Pascal development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 ********************************************************************** }
//
// Module: sms.h
//

//
//  Microsoft Windows Mobile 6.0 for PocketPC SDK.
//

unit sms;

{$CALLING cdecl}

interface

uses Windows;

//
// Errors
//
const
      FACILITY_SMS                           = $0200;

// MAKE_HRESULT rules OK!      
// Specific registration errors (for SmsSetMessageNotification, SmsClearMessageNotification)
const
      SMS_E_INVALIDPROTOCOL                  = (SEVERITY_ERROR shl 31) or (FACILITY_SMS shl 16) or $0001;
      SMS_E_REGISTRATIONEXISTS               = (SEVERITY_ERROR shl 31) or (FACILITY_SMS shl 16) or $0002;
      SMS_S_NOSUCHREGISTRATION               = (SEVERITY_SUCCESS shl 31) or (FACILITY_SMS shl 16) or $0003;

// Specific messaging errors (for SmsOpen, SmsSendMessage, SmsGetSMSC, etc.)
      SMS_E_TOOMUCHDATA                      = (SEVERITY_ERROR shl 31) or (FACILITY_SMS shl 16) or $0100;
      SMS_E_INVALIDDATA                      = (SEVERITY_ERROR shl 31) or (FACILITY_SMS shl 16) or $0101;
      SMS_E_BUFFERTOOSMALL                   = (SEVERITY_ERROR shl 31) or (FACILITY_SMS shl 16) or $0102;
      SMS_E_PROVIDERSPECIFICBUFFERWRONGSIZE  = (SEVERITY_ERROR shl 31) or (FACILITY_SMS shl 16) or $0103;
      SMS_E_TIMEUNAVAILABLE                  = (SEVERITY_ERROR shl 31) or (FACILITY_SMS shl 16) or $0104;
      SMS_E_RECEIVEHANDLEALREADYOPEN         = (SEVERITY_ERROR shl 31) or (FACILITY_SMS shl 16) or $0105;
      SMS_E_DESTINATIONOUTOFSVC              = (SEVERITY_ERROR shl 31) or (FACILITY_SMS shl 16) or $0106;
      SMS_E_INVALIDADDRESS                   = (SEVERITY_ERROR shl 31) or (FACILITY_SMS shl 16) or $0107;
      SMS_E_MSGBARREDBYOPERATOR              = (SEVERITY_ERROR shl 31) or (FACILITY_SMS shl 16) or $0108;
      SMS_E_MSGCALLBARRED                    = (SEVERITY_ERROR shl 31) or (FACILITY_SMS shl 16) or $0109;
      SMS_E_NOSCSUBSCRIPTION                 = (SEVERITY_ERROR shl 31) or (FACILITY_SMS shl 16) or $010a;
      SMS_E_SCBUSY                           = (SEVERITY_ERROR shl 31) or (FACILITY_SMS shl 16) or $010b;
      SMS_E_SVCNOTSUBSCRIBED                 = (SEVERITY_ERROR shl 31) or (FACILITY_SMS shl 16) or $010c;
      SMS_E_UNASSIGNEDNUMBER                 = (SEVERITY_ERROR shl 31) or (FACILITY_SMS shl 16) or $010d;
      SMS_E_UNKNOWNSCADDRESS                 = (SEVERITY_ERROR shl 31) or (FACILITY_SMS shl 16) or $010e;
      SMS_E_UNIDENTIFIEDSUBCRIBER            = (SEVERITY_ERROR shl 31) or (FACILITY_SMS shl 16) or $010f;
      SMS_E_FDNRESTRICT                      = (SEVERITY_ERROR shl 31) or (FACILITY_SMS shl 16) or $0110;

// General SMS messaging errors
      SMS_E_MISC                             = (SEVERITY_ERROR shl 31) or (FACILITY_SMS shl 16) or $0200;
      SMS_E_PASSWORD                         = (SEVERITY_ERROR shl 31) or (FACILITY_SMS shl 16) or $0201;
      SMS_E_SIM                              = (SEVERITY_ERROR shl 31) or (FACILITY_SMS shl 16) or $0202;
      SMS_E_NETWORKACCESS                    = (SEVERITY_ERROR shl 31) or (FACILITY_SMS shl 16) or $0203;
      SMS_E_NETWORK                          = (SEVERITY_ERROR shl 31) or (FACILITY_SMS shl 16) or $0204;
      SMS_E_MOBILE                           = (SEVERITY_ERROR shl 31) or (FACILITY_SMS shl 16) or $0205;
      SMS_E_NETWORKUNSUPPORTED               = (SEVERITY_ERROR shl 31) or (FACILITY_SMS shl 16) or $0206;
      SMS_E_MOBILEUNSUPPORTED                = (SEVERITY_ERROR shl 31) or (FACILITY_SMS shl 16) or $0207;
      SMS_E_BADPARAM                         = (SEVERITY_ERROR shl 31) or (FACILITY_SMS shl 16) or $0208;
      SMS_E_STORAGE                          = (SEVERITY_ERROR shl 31) or (FACILITY_SMS shl 16) or $0209;
      SMS_E_SMSC                             = (SEVERITY_ERROR shl 31) or (FACILITY_SMS shl 16) or $020a;
      SMS_E_DESTINATION                      = (SEVERITY_ERROR shl 31) or (FACILITY_SMS shl 16) or $020b;
      SMS_E_DESTINATIONUNSUPPORTED           = (SEVERITY_ERROR shl 31) or (FACILITY_SMS shl 16) or $020c;
      SMS_E_RADIOUNAVAILABLE                 = (SEVERITY_ERROR shl 31) or (FACILITY_SMS shl 16) or $020d;


//
// Constants
//
const
      SMS_DATAGRAM_SIZE            = 140;
      SMS_BROADCAST_DATAGRAM_SIZE  = 82;
      SMS_MAX_APPNAME_LENGTH       = MAX_PATH;
      SMS_MAX_PARAMS_LENGTH        = MAX_PATH;
      SMS_MAX_PROTOCOLNAME_LENGTH  = MAX_PATH;
      SMS_MAX_MESSAGEUID_SIZE      = 1024;
      SMS_MAX_ADDRESS_LENGTH       = 256;
      SMS_MAX_SUBADDRESS_SIZE      = 256;
// dwMessageModes for SmsOpen
      SMS_MODE_RECEIVE              = $00000001;
      SMS_MODE_SEND                 = $00000002;
// dwOptions for SmsSendMessage
      SMS_OPTION_DELIVERY_NONE      = $00000000;
      SMS_OPTION_DELIVERY_NO_RETRY  = $00000001;
// dwMessageOptions for TEXT_PROVIDER_SPECIFIC_DATA
      PS_MESSAGE_OPTION_NONE          = $00000000;
      PS_MESSAGE_OPTION_REPLYPATH     = $00000001;
      PS_MESSAGE_OPTION_STATUSREPORT  = $00000002;
      PS_MESSAGE_OPTION_DISCARD       = $00000004;
// dwMessageStatus0 and dwMessageStatus1 for SmsGetMessageStatus and the SMS status-message provider (SMS_MSGTYPE_STATUS)
// Message status is unknown iff dwMessageStatus0 and dwMessageStatus1 are both set to MESSAGE_STATUS_UNKNOWN
      MESSAGE_STATUS_UNKNOWN                  = $00000000;
// Valid bits for dwMessageStatus0
      MESSAGE_STATUS_0_RECEIVEDBYSME            = $00000001;
      MESSAGE_STATUS_0_FORWARDEDTOSME           = $00000002;
      MESSAGE_STATUS_0_REPLACEDBYSC             = $00000004;
      MESSAGE_STATUS_0_CONGESTION_TRYING        = $00000008;
      MESSAGE_STATUS_0_SMEBUSY_TRYING           = $00000010;
      MESSAGE_STATUS_0_SMENOTRESPONDING_TRYING  = $00000020;
      MESSAGE_STATUS_0_SVCREJECTED_TRYING       = $00000040;
      MESSAGE_STATUS_0_QUALITYUNAVAIL_TRYING    = $00000080;
      MESSAGE_STATUS_0_SMEERROR_TRYING          = $00000100;
      MESSAGE_STATUS_0_CONGESTION               = $00000200;
      MESSAGE_STATUS_0_SMEBUSY                  = $00000400;
      MESSAGE_STATUS_0_SMENOTRESPONDING         = $00000800;
      MESSAGE_STATUS_0_SVCREJECTED              = $00001000;
      MESSAGE_STATUS_0_QUALITYUNAVAIL_TEMP      = $00002000;
      MESSAGE_STATUS_0_SMEERROR                 = $00004000;
      MESSAGE_STATUS_0_REMOTEPROCERROR          = $00008000;
      MESSAGE_STATUS_0_INCOMPATIBLEDEST         = $00010000;
      MESSAGE_STATUS_0_CONNECTIONREJECTED       = $00020000;
      MESSAGE_STATUS_0_NOTOBTAINABLE            = $00040000;
      MESSAGE_STATUS_0_NOINTERNETWORKING        = $00080000;
      MESSAGE_STATUS_0_VPEXPIRED                = $00100000;
      MESSAGE_STATUS_0_DELETEDBYORIGSME         = $00200000;
      MESSAGE_STATUS_0_DELETEDBYSC              = $00400000;
      MESSAGE_STATUS_0_NOLONGEREXISTS           = $00800000;
      MESSAGE_STATUS_0_QUALITYUNAVAIL           = $01000000;
      MESSAGE_STATUS_0_RESERVED_COMPLETED       = $02000000;
      MESSAGE_STATUS_0_RESERVED_TRYING          = $04000000;
      MESSAGE_STATUS_0_RESERVED_ERROR           = $08000000;
      MESSAGE_STATUS_0_RESERVED_TMPERROR        = $10000000;
      MESSAGE_STATUS_0_SCSPECIFIC_COMPLETED     = $20000000;
      MESSAGE_STATUS_0_SCSPECIFIC_TRYING        = $40000000;
      MESSAGE_STATUS_0_SCSPECIFIC_ERROR         = $80000000;
// Valid bits for dwMessageStatus1
      MESSAGE_STATUS_1_SCSPECIFIC_TMPERROR      = $00000001;
// Language identifiers for SMS_BROADCAST_RANGES.dwBroadcastMsgLangs
      SMS_DCSLANG_UNKNOWN                       = $00000001;
      SMS_DCSLANG_GERMAN                        = $00000002;
      SMS_DCSLANG_ENGLISH                       = $00000004;
      SMS_DCSLANG_ITALIAN                       = $00000008;
      SMS_DCSLANG_FRENCH                        = $00000010;
      SMS_DCSLANG_SPANISH                       = $00000020;
      SMS_DCSLANG_DUTCH                         = $00000040;
      SMS_DCSLANG_SWEDISH                       = $00000080;
      SMS_DCSLANG_DANISH                        = $00000100;
      SMS_DCSLANG_PORTUGUESE                    = $00000200;
      SMS_DCSLANG_FINNISH                       = $00000400;
      SMS_DCSLANG_NORWEGIAN                     = $00000800;
      SMS_DCSLANG_GREEK                         = $00001000;
      SMS_DCSLANG_TURKISH                       = $00002000;
      SMS_DCSLANG_HUNGARIAN                     = $00004000;
      SMS_DCSLANG_POLISH                        = $00008000;
      SMS_DCSLANG_CZECH                         = $00010000;
      SMS_DCSLANG_HEBREW                        = $00020000;
      SMS_DCSLANG_ARABIC                        = $00040000;
      SMS_DCSLANG_RUSSIAN                       = $00080000;
      SMS_DCSLANG_ICELANDIC                     = $00100000;
      SMS_DCSLANG_ALL                           = $001fffff;

// Valid flags for SMS_BROADCAST_RANGES.dwParams
      SMS_PARAM_SBR_BROADCASTMSGIDS             = $00000001;
      SMS_PARAM_SBR_BROADCASTMSGLANGS           = $00000002;
      SMS_PARAM_SBR_ACCEPTIDS                   = $00000004;
//Valid values for ProtocolID
      SMS_MSGPROTOCOL_UNKNOWN                     = $00000000;
      SMS_MSGPROTOCOL_SMETOSME                    = $00000001;
      SMS_MSGPROTOCOL_IMPLICIT                    = $00000002;
      SMS_MSGPROTOCOL_TELEX                       = $00000003;
      SMS_MSGPROTOCOL_TELEFAX_GROUP3              = $00000004;
      SMS_MSGPROTOCOL_TELEFAX_GROUP4              = $00000005;
      SMS_MSGPROTOCOL_VOICEPHONE                  = $00000006;
      SMS_MSGPROTOCOL_ERMES                       = $00000007;
      SMS_MSGPROTOCOL_PAGING                      = $00000008;
      SMS_MSGPROTOCOL_VIDEOTEX                    = $00000009;
      SMS_MSGPROTOCOL_TELETEX                     = $0000000a;
      SMS_MSGPROTOCOL_TELETEX_PSPDN               = $0000000b;
      SMS_MSGPROTOCOL_TELETEX_CSPDN               = $0000000c;
      SMS_MSGPROTOCOL_TELETEX_PSTN                = $0000000d;
      SMS_MSGPROTOCOL_TELETEX_ISDN                = $0000000e;
      SMS_MSGPROTOCOL_UCI                         = $0000000f;
      SMS_MSGPROTOCOL_MSGHANDLING                 = $00000010;
      SMS_MSGPROTOCOL_X400                        = $00000011;
      SMS_MSGPROTOCOL_EMAIL                       = $00000012;
      SMS_MSGPROTOCOL_SCSPECIFIC1             = $00000013;
      SMS_MSGPROTOCOL_SCSPECIFIC2             = $00000014;
      SMS_MSGPROTOCOL_SCSPECIFIC3             = $00000015;
      SMS_MSGPROTOCOL_SCSPECIFIC4             = $00000016;
      SMS_MSGPROTOCOL_SCSPECIFIC5             = $00000017;
      SMS_MSGPROTOCOL_SCSPECIFIC6             = $00000018;
      SMS_MSGPROTOCOL_SCSPECIFIC7             = $00000019;
      SMS_MSGPROTOCOL_GSMSTATION                  = $0000001a;
      SMS_MSGPROTOCOL_SM_TYPE0                    = $0000001b;
      SMS_MSGPROTOCOL_RSM_TYPE1                   = $0000001c;
      SMS_MSGPROTOCOL_RSM_TYPE2                   = $0000001d;
      SMS_MSGPROTOCOL_RSM_TYPE3                   = $0000001e;
      SMS_MSGPROTOCOL_RSM_TYPE4                   = $0000001f;
      SMS_MSGPROTOCOL_RSM_TYPE5                   = $00000020;
      SMS_MSGPROTOCOL_RSM_TYPE6                   = $00000021;
      SMS_MSGPROTOCOL_RSM_TYPE7                   = $00000022;
      SMS_MSGPROTOCOL_RETURNCALL                  = $00000023;
      SMS_MSGPROTOCOL_ME_DOWNLOAD                 = $00000024;
      SMS_MSGPROTOCOL_DEPERSONALIZATION           = $00000025;
      SMS_MSGPROTOCOL_SIM_DOWNLOAD                = $00000026;

//
// Enumerations
//
type
     SMS_ADDRESS_TYPE = (SMSAT_UNKNOWN := 0,
                         SMSAT_INTERNATIONAL,
                         SMSAT_NATIONAL,
                         SMSAT_NETWORKSPECIFIC,
                         SMSAT_SUBSCRIBER,
                         SMSAT_ALPHANUMERIC,
                         SMSAT_ABBREVIATED);

     SMS_DATA_ENCODING  = (SMSDE_OPTIMAL := 0,
                           SMSDE_GSM,
                           SMSDE_UCS2);

     PROVIDER_SPECIFIC_MESSAGE_CLASS = (PS_MESSAGE_CLASS0 := 0,
                                        PS_MESSAGE_CLASS1,
                                        PS_MESSAGE_CLASS2,
                                        PS_MESSAGE_CLASS3,
                                        PS_MESSAGE_CLASSUNSPECIFIED);

     PROVIDER_SPECIFIC_REPLACE_OPTION = (PSRO_NONE := 0,
                                         PSRO_REPLACE_TYPE1,
                                         PSRO_REPLACE_TYPE2,
                                         PSRO_REPLACE_TYPE3,
                                         PSRO_REPLACE_TYPE4,
                                         PSRO_REPLACE_TYPE5,
                                         PSRO_REPLACE_TYPE6,
                                         PSRO_REPLACE_TYPE7,
                                         PSRO_RETURN_CALL,
                                         PSRO_DEPERSONALIZATION);

//
// Types
//
type
     SMS_HANDLE = DWORD;
     LPSMS_HANDLE = ^SMS_HANDLE;

     SMS_MESSAGE_ID = DWORD;
     LPSMS_MESSAGE_ID = ^SMS_MESSAGE_ID;

const
      INVALID_MESSAGE_ID = SMS_MESSAGE_ID($ffffffff);

// Registration structure used by SmsSetMessageNotification and SmsClearMessageNotification
type
     smsregistrationdata_tag = record
       cbSize:DWORD;
       tszAppName:array[0..SMS_MAX_APPNAME_LENGTH-1] of TCHAR;
       tszParams:array[0..SMS_MAX_PARAMS_LENGTH-1] of TCHAR;
       tszProtocolName:array[0..SMS_MAX_PROTOCOLNAME_LENGTH-1] of TCHAR;
     end;

     SMSREGISTRATIONDATA = smsregistrationdata_tag;
     LPSMSREGISTRATIONDATA = ^smsregistrationdata_tag;

// SMS addressing information
type
     sms_address_tag = record
       smsatAddressType:SMS_ADDRESS_TYPE;
       ptsAddress:array[0..SMS_MAX_ADDRESS_LENGTH-1] of TCHAR;
     end;
     SMS_ADDRESS = sms_address_tag;
     LPSMS_ADDRESS = ^sms_address_tag;

// SMS status message information
type
     sms_status_information_tag = record
       smsmidMessageID:SMS_MESSAGE_ID;
       dwMessageStatus0:DWORD;
       dwMessageStatus1:DWORD;
       smsaRecipientAddress:SMS_ADDRESS;
       stServiceCenterTimeStamp:SYSTEMTIME;  // (UTC time)
       stDischargeTime:SYSTEMTIME;  // (UTC time)
     end;
     SMS_STATUS_INFORMATION = sms_status_information_tag;
     LPSMS_STATUS_INFORMATION = ^sms_status_information_tag;

// SMS broadcast message range information
type
     sms_range_tag = record
       dwMinimum:DWORD;
       dwMaximum:DWORD;
     end;
     SMS_RANGE = sms_range_tag;
     LPSMS_RANGE = ^sms_range_tag;
     
// SMS broadcast message ranges information
// Use #pragma to avoid "warning C4200: nonstandard extension used : zero-sized array in struct/union
type
     sms_broadcast_ranges_tag = record
       cbSize:DWORD;
       dwParams:DWORD;
       dwNumRanges:DWORD;
       dwBroadcastMsgLangs:DWORD;
       bAccept:BOOL;
       smsrBroadcastRanges:array[0..0] of SMS_RANGE;
     end;
     SMS_BROADCAST_RANGES = sms_broadcast_ranges_tag;
     LPSMS_BROADCAST_RANGES = ^sms_broadcast_ranges_tag;
     
//
// SMS message types (for use with SmsOpen)
//

// Text message type
const
      SMS_MSGTYPE_TEXT = 'Microsoft Text SMS Protocol';
      
// Provider-specific data for use with SmsSendMessage and SmsReadMessage

// Bitfield values for the extended parameters of the text provider structure.
const
      TEXTPSEXTPARM_NONE         = $00000000;
      TEXTPSEXTPARM_CALLBACK     = $00000001;
      TEXTPSEXTPARM_PRIORITY     = $00000002;
      TEXTPSEXTPARM_ALL          = $00000003;

// The priority enum should match up with the RIL_MSGPRIORITY_* defines.
// If any new values are added, the Priority mapping functions of sms_txtshared_cdma.cpp
// should also be adjusted.
type
     TEXT_PROVIDER_SPECIFIC_PRIORITY_TYPE = (TEXTPSPRI_NONE := 0,
                                             TEXTPSPRI_NORMAL := 1,
                                             TEXTPSPRI_INTERACTIVE,
                                             TEXTPSPRI_URGENT,
                                             TEXTPSPRI_EMERGENCY);

type
     text_provider_specific_data_tag = record
       dwMessageOptions:DWORD;
       psMessageClass:PROVIDER_SPECIFIC_MESSAGE_CLASS;
       psReplaceOption:PROVIDER_SPECIFIC_REPLACE_OPTION;
       dwHeaderDataSize:DWORD;
       pbHeaderData:array[0..SMS_DATAGRAM_SIZE-1] of byte;   // For concatenated messages, only the header from the first segment is returned.
       fMessageContainsEMSHeaders:BOOL;     // At least one segment of this message contains EMS headers.
                                            // Only set if EMS handler installed.
       dwProtocolID:DWORD;                  // PID of incoming message, or desired PID of outgoing message.
                                            // Applies only to GSM.  Set to SMS_MSGPROTOCOL_UNKNOWN if psReplaceOption
                                            // is not PSRO_NONE.
       dwExtParams:DWORD;                   // Bitfield of valid additional structure parameters (all structure
                                            // values above are considered always valid).
       tpsPriority:TEXT_PROVIDER_SPECIFIC_PRIORITY_TYPE; // Applies only to CDMA IS637. Priority indicator.
       smsaCallback:SMS_ADDRESS;                         // Applies only to CDMA IS637. Callback number
     end;
     TEXT_PROVIDER_SPECIFIC_DATA = text_provider_specific_data_tag;

// Class 2 Text message type
const
      SMS_MSGTYPE_CLASS2 = 'Microsoft Class2 SMS Protocol';

type
     class2_provider_specific_data_tag = record
       dwMessageOptions:DWORD;
       psMessageClass:PROVIDER_SPECIFIC_MESSAGE_CLASS;
       psReplaceOption:PROVIDER_SPECIFIC_REPLACE_OPTION;
       dwHeaderDataSize:DWORD;
       pbHeaderData:array[0..SMS_DATAGRAM_SIZE-1] of byte;   // For concatenated messages, only the header from the first segment is returned.
       fMessageContainsEMSHeaders:BOOL;     // At least one segment of this message contains EMS headers.
                                            // Only set if EMS handler installed.
       dwProtocolID:DWORD;                  // PID of incoming message, or desired PID of outgoing message.
                                            // Applies only to GSM.  Set to SMS_MSGPROTOCOL_UNKNOWN if psReplaceOption
                                            // is not PSRO_NONE.
       dwLocation:DWORD;
       dwIndex:DWORD;
     end;
     CLASS2_PROVIDER_SPECIFIC_DATA = class2_provider_specific_data_tag;

// Notification message type
const
      SMS_MSGTYPE_NOTIFICATION = 'Microsoft Notification SMS Protocol (Receive Only)';

// Provider-specific data for use with SmsReadMessage
type
     NOTIFICATION_PROVIDER_SPECIFIC_MSG_WAITING_TYPE = (NOTIFICATIONPSMWT_NONE := 0,
                                                        NOTIFICATIONPSMWT_GENERIC,
                                                        NOTIFICATIONPSMWT_VOICEMAIL,
                                                        NOTIFICATIONPSMWT_FAX,
                                                        NOTIFICATIONPSMWT_EMAIL,
                                                        NOTIFICATIONPSMWT_OTHER);

const
      NOTIFICATIONPS_NUM_MSG_WAITING_UNKNOWN  = -1;
      NOTIFICATIONPS_NUM_MSG_WAITING_NONZERO  = -2;

type
     NOTIFICATION_PROVIDER_SPECIFIC_INDICATOR_TYPE = (NOTIFICATIONPSIT_NONE := 0,
                                                      NOTIFICATIONPSIT_LINE1 := 1,
                                                      NOTIFICATIONPSIT_LINE2 := 2);

type
     notification_provider_specific_data_tag = record
       dwMessageOptions:DWORD;
       psMessageClass:PROVIDER_SPECIFIC_MESSAGE_CLASS;
       psReplaceOption:PROVIDER_SPECIFIC_REPLACE_OPTION;
       npsMsgWaitingType:NOTIFICATION_PROVIDER_SPECIFIC_MSG_WAITING_TYPE;
       iNumberOfMessagesWaiting:longint;
       npsIndicatorType:NOTIFICATION_PROVIDER_SPECIFIC_INDICATOR_TYPE;
     end;
     NOTIFICATION_PROVIDER_SPECIFIC_DATA = notification_provider_specific_data_tag;

// WDP message type
const
      SMS_MSGTYPE_WDP = 'Microsoft WDP SMS Protocol';

// Provider-specific data for use with SmsSendMessage and SmsReadMessage
type
     WDP_PROVIDER_SPECIFIC_PORT_ADDRESSING = (WDPPSPA_8_BIT_PORT_NUMBERS := 0,
                                              WDPPSPA_16_BIT_PORT_NUMBERS);

type
     wdp_provider_specific_data_tag = record
       wdppsPortAddressing:WDP_PROVIDER_SPECIFIC_PORT_ADDRESSING;
       wDestinationPort:word;
       wOriginatorPort:word;
     end;
     WDP_PROVIDER_SPECIFIC_DATA = wdp_provider_specific_data_tag;

// WCMP message type
const
      SMS_MSGTYPE_WCMP = 'Microsoft WCMP SMS Protocol';
      
// Provider-specific data for use with SmsSendMessage and SmsReadMessage
type
     WCMP_PROVIDER_SPECIFIC_MESSAGE_TYPE = (WCMPPSMT_UNSUPPORTED := 0,
                                            WCMPPSMT_PORT_UNREACHABLE,
                                            WCMPPSMT_MESSAGE_TOO_BIG,
                                            WCMPPSMT_ECHO_REQUEST,
                                            WCMPPSMT_ECHO_REPLY);

type
     wcmp_provider_specific_data_tag = record
       wcmppsMessageType:WCMP_PROVIDER_SPECIFIC_MESSAGE_TYPE;
       wParam1:word;
       wParam2:word;
       wParam3:word;
       smsaAddress:SMS_ADDRESS;
     end;
     WCMP_PROVIDER_SPECIFIC_DATA = wcmp_provider_specific_data_tag;

// Status message type
const
      SMS_MSGTYPE_STATUS = 'Microsoft Status Message SMS Protocol (Receive Only)';

// Provider-specific data for use with SmsReadMessage
type
     status_provider_specific_data_tag = record
       smssiStatusInformation:SMS_STATUS_INFORMATION;
     end;
     STATUS_PROVIDER_SPECIFIC_DATA = status_provider_specific_data_tag;

// Broadcast message type
const
      SMS_MSGTYPE_BROADCAST = 'Microsoft Broadcast Message SMS Protocol (Receive Only)';

type
     BROADCAST_PROVIDER_SPECIFIC_GEOGRAPHICAL_SCOPE = (BPSGS_UNKNOWN := 0,
                                                       BPSGS_CELL_DISPLAY_IMMEDIATE,
                                                       BPSGS_CELL,
                                                       BPSGS_PLMN,
                                                       BPSGS_LOCATION_AREA);

// Provider-specific data for use with SmsReadMessage
type
     broadcast_provider_specific_data_tag = record
       wMessageID:word;
       wMessageCode:word;
       bpsgsGeographicalScope:BROADCAST_PROVIDER_SPECIFIC_GEOGRAPHICAL_SCOPE;
       wUpdateNumber:word;
     end;
     BROADCAST_PROVIDER_SPECIFIC_DATA = broadcast_provider_specific_data_tag;

// Raw message type
const
      SMS_MSGTYPE_RAW = 'Microsoft Raw SMS Protocol (Receive Only)';
      
// Provider-specific data for use with SmsReadMessage
type
     raw_provider_specific_data_tag  = record
       dwHeaderDataSize:DWORD;
       pbHeaderData:array[0..SMS_DATAGRAM_SIZE-1] of byte;
     end;
     RAW_PROVIDER_SPECIFIC_DATA = raw_provider_specific_data_tag;


//
// APIs for SMS.dll
//

const
      SmsDLL = 'sms.dll';

function SmsSetMessageNotification(psmsrd:LPSMSREGISTRATIONDATA):HRESULT; external SmsDLL name 'SmsSetMessageNotification';

function SmsClearMessageNotification(tszProtocolName:LPCTSTR):HRESULT; external SmsDLL name 'SmsClearMessageNotification';

// Open the SMS Messaging component for read and/or write access
//
// Each protocol may only have one handle open with SMS_MODE_RECEIVE.
// Additional attempts to get RECEIVE mode on a given protocol will result in
// SMS_E_RECEIVEHANDLEALREADYOPEN.
//
// Currently all protocols have applications whith open RECEIVE handles.  As a
// result, using the SMS API to receive SMS messages is not supported.  Attempts
// to do so may interfere with the proper operation of Inbox, WAP, or other SMS
// applications.
//
// The IMailRuleClient interface in cemapi.h may be used to access received
// text SMS messages.
function SmsOpen(ptsMessageProtocol:LPCTSTR;
                 dwMessageModes:DWORD;
                 psmshHandle:LPSMS_HANDLE;
                 phMessageAvailableEvent:LPHANDLE):HRESULT; external SmsDLL name 'SmsOpen';

// Close a handle to the SMS messaging component
function SmsClose(smshHandle:SMS_HANDLE):HRESULT; external SmsDLL name 'SmsClose';

// Send an SMS message
function SmsSendMessage(smshHandle:SMS_HANDLE;
                        psmsaSMSCAddress:LPSMS_ADDRESS;
<<<<<<< HEAD
<<<<<<< HEAD
                        psmsaDestinationAddress:LPSMS_ADDRESS;
=======
                        psmsaDestinationAddress:SMS_ADDRESS;
>>>>>>> graemeg/fixes_2_2
=======
                        psmsaDestinationAddress:SMS_ADDRESS;
>>>>>>> origin/fixes_2_2
                        pstValidityPeriod:LPSYSTEMTIME;  // (Values in this structure are expressed relative to the current time)
                        pbData:LPBYTE;
                        dwDataSize:DWORD;
                        pbProviderSpecificData:LPBYTE;
                        dwProviderSpecificDataSize:DWORD;
                        smsdeDataEncoding:SMS_DATA_ENCODING;
                        dwOptions:DWORD;
                        psmsmidMessageID:LPSMS_MESSAGE_ID):HRESULT; external SmsDLL name 'SmsSendMessage';

// Determine an upper-bound for the size of the buffer needed by the next call to SmsReadMessage
function SmsGetMessageSize(smshHandle:SMS_HANDLE;
                           pdwDataSize:LPDWORD):HRESULT; external SmsDLL name 'SmsGetMessageSize';

// Read an SMS message (the appropriate size of the buffer can be found via a call to SmsGetMessageSize)
function SmsReadMessage(smshHandle:SMS_HANDLE;
                        psmsaSMSCAddress:LPSMS_ADDRESS;
                        psmsaSourceAddress:LPSMS_ADDRESS;
                        pstReceiveTime:LPSYSTEMTIME;  // (UTC time)
                        pbBuffer:LPBYTE;
                        dwBufferSize:DWORD;
                        pbProviderSpecificBuffer:LPBYTE;
                        dwProviderSpecificDataBuffer:DWORD;
                        pdwBytesRead:LPDWORD):HRESULT; external SmsDLL name 'SmsReadMessage';

// Waits to receive a status-report for an SMS message
function SmsGetMessageStatus(smshHandle:SMS_HANDLE;
                             smsmidMessageID:SMS_MESSAGE_ID;
                             psmssiStatusInformation:LPSMS_STATUS_INFORMATION;
                             dwTimeout:DWORD):HRESULT; external SmsDLL name 'SmsGetMessageStatus';

// Get the default SMS Service Center address
function SmsGetSMSC(psmsaSMSCAddress:LPSMS_ADDRESS):HRESULT; external SmsDLL name 'SmsGetSMSC';

// Set the default SMS Service Center address
function SmsSetSMSC(psmsaSMSCAddress:LPSMS_ADDRESS):HRESULT; external SmsDLL name 'SmsSetSMSC';

// Get the range of broadcast messages to listen for
function SmsGetBroadcastMsgRanges(psmsbrBroadcastRanges:LPSMS_BROADCAST_RANGES):HRESULT; external SmsDLL name 'SmsGetBroadcastMsgRanges';

// Set the range of broadcast messages to listen for
function SmsSetBroadcastMsgRanges(psmsbrBroadcastRanges:LPSMS_BROADCAST_RANGES):HRESULT; external SmsDLL name 'SmsSetBroadcastMsgRanges';

// Get the device's phone number for SMS
function SmsGetPhoneNumber(psmsaAddress:LPSMS_ADDRESS):HRESULT; external SmsDLL name 'SmsGetPhoneNumber';

// Approximate the system time based on the time indicated by the SMSC in the last status-report message
function SmsGetTime(ptsCurrentTime:LPSYSTEMTIME;  // (UTC time)
                    pdwErrorMargin:LPDWORD):HRESULT; external SmsDLL name 'SmsGetTime';

implementation

end.