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
// Module: bthapi.h, bthapi.idl
//

//
//  Microsoft Windows Mobile 6.0 for PocketPC SDK.
//

unit bthapi;

{$MODE OBJFPC}

interface

uses Windows;

const
      LIBID_BTHAPILib:TIID = '{00BC26C8-0A87-41d0-82BA-61FF9E0B1BB5}';

const
     IID_ISdpWalk:TIID = '{57134AE6-5D3C-462D-BF2F-810361FBD7E7}';
     IID_ISdpNodeContainer:TIID = '{43F6ED49-6E22-4F81-A8EB-DCED40811A77}';
     IID_ISdpSearch:TIID = '{D93B6B2A-5EEF-4E1E-BECF-F5A4340C65F5}';
<<<<<<< HEAD
<<<<<<< HEAD
     IID_ISdpStream:TIID = '{A6ECD9FB-0C7A-41A3-9FF0-0B617E989357}';
=======
     IID_ISdpStream = '{A6ECD9FB-0C7A-41A3-9FF0-0B617E989357}';
>>>>>>> graemeg/fixes_2_2
=======
     IID_ISdpStream = '{A6ECD9FB-0C7A-41A3-9FF0-0B617E989357}';
>>>>>>> origin/fixes_2_2
     IID_ISdpRecord:TIID = '{10276714-1456-46D7-B526-8B1E83D5116E}';
     IID_IBluetoothDevice:TIID = '{5BD0418B-D705-4766-B215-183E4EADE341}';
     IID_IBluetoothAuthenticate:TIID = '{5F0FBA2B-8300-429D-99AD-96A2835D4901}';

const
      CLSID_SdpNodeContainer:CLSID = '{D5CA76C5-0DEE-4453-96A1-E603C2401766}';

      CLSID_SdpSearch:CLSID = '{3B898402-857E-4E41-9145-BC35431B7B4D}';

      CLSID_SdpWalk:CLSID = '{ED384010-59AE-44c7-8FCA-F3DF22CDCD28}';

      CLSID_SdpStream:CLSID = '{249797FA-19DB-4dda-94D4-E0BCD30EA65E}';

      CLSID_SdpRecord:CLSID = '{ACD02BA7-9667-4085-A100-CC6ACA9621D6}';


{$IFNDEF __BTHSDPDEF_H__}
type
     SDP_LARGE_INTEGER_16 = record
       LowPart:ULONGLONG;
       HighPart:LONGLONG;
     end;
     PSDP_LARGE_INTEGER_16 = ^SDP_LARGE_INTEGER_16;
     LPSDP_LARGE_INTEGER_16 = ^SDP_LARGE_INTEGER_16;

     SDP_ULARGE_INTEGER_16 = record
       LowPart:ULONGLONG;
       HighPart:ULONGLONG;
     end;
     PSDP_ULARGE_INTEGER_16 = ^SDP_ULARGE_INTEGER_16;
     LPSDP_ULARGE_INTEGER_16 = ^SDP_ULARGE_INTEGER_16;


type
     NodeContainerType = (NodeContainerTypeSequence	:= 0,
                          NodeContainerTypeAlternative);
     PNodeContainerType = ^NodeContainerType;


type
     SDP_ERROR = USHORT;
     PSDP_ERROR = ^SDP_ERROR;

type
     SDP_TYPE = (SDP_TYPE_NIL	:= 0,
	                SDP_TYPE_UINT	:= $1,
	                SDP_TYPE_INT	:= $2,
	                SDP_TYPE_UUID	:= $3,
	                SDP_TYPE_STRING	:= $4,
	                SDP_TYPE_BOOLEAN	:= $5,
	                SDP_TYPE_SEQUENCE	:= $6,
	                SDP_TYPE_ALTERNATIVE	:= $7,
	                SDP_TYPE_URL	:= $8,
	                SDP_TYPE_CONTAINER	:= $20);
     PSDP_TYPE = ^SDP_TYPE;

     SDP_SPECIFICTYPE = (SDP_ST_NONE	:= 0,
	                        SDP_ST_UINT8	:= $10,
                         SDP_ST_INT8	:= $20,
	                        SDP_ST_UINT16	:= $110,
                         SDP_ST_INT16	:= $120,
                         SDP_ST_UUID16	:= $130,
	                        SDP_ST_UINT32	:= $210,
                         SDP_ST_INT32	:= $220,
                         SDP_ST_UUID32	:= $230,
	                        SDP_ST_UINT64	:= $310,
                         SDP_ST_INT64	:= $320,
	                        SDP_ST_UINT128	:= $410,
	                        SDP_ST_INT128	:= $420,
	                        SDP_ST_UUID128	:= $430);
     PSDP_SPECIFICTYPE = ^SDP_SPECIFICTYPE;


type
     _SdpAttributeRange = record
       minAttribute:USHORT;
       maxAttribute:USHORT;
     end;
     SdpAttributeRange = _SdpAttributeRange;
     PSdpAttributeRange = ^_SdpAttributeRange;

type
     SdpQueryUuidUnion = record
       case integer of
         0: (uuid128:GUID);
         1: (uuid32:ULONG);
         2: (uuid16:USHORT);
     end;

type
     _SdpQueryUuid = record
       u:SdpQueryUuidUnion;
       uuidType:USHORT;
     end;
     SdpQueryUuid = _SdpQueryUuid;
     PSdpQueryUuid = ^_SdpQueryUuid;


const
      BTH_SDP_VERSION = 1;

type
     _BTHNS_SETBLOB = record
       pSdpVersion:PULONG;
       pRecordHandle:PULONG;
       Reserved:array[0..3] of ULONG;
       fSecurity:ULONG;
       fOptions:ULONG;
       ulRecordLength:ULONG;
       pRecord:array[0..0] of UCHAR;
     end;
     BTHNS_SETBLOB = _BTHNS_SETBLOB;
     PBTHNS_SETBLOB = ^_BTHNS_SETBLOB;

const
      MAX_UUIDS_IN_QUERY                   = 12;
      SDP_SERVICE_SEARCH_REQUEST           = 1;
      SDP_SERVICE_ATTRIBUTE_REQUEST        = 2;
      SDP_SERVICE_SEARCH_ATTRIBUTE_REQUEST = 3;

//
// The following may be passed as parameters to BthNsLookupServiceNext as extended
// dwFlags options for device inquiry.
//
const
// Causes traversal through list to be reset to first element.
      BTHNS_LUP_RESET_ITERATOR = $00010000;
// Does not increment list, causes next query to be performed on current item as well.
      BTHNS_LUP_NO_ADVANCE     = $00020000;
// Causes LookupServiceEnd to abort current inquiry.
      BTHNS_ABORT_CURRENT_INQUIRY = $fffffffd;


type
     _BTHNS_INQUIRYBLOB = record
       LAP:ULONG;
       _length:byte;
       num_responses:byte;
     end;
     BTHNS_INQUIRYBLOB = _BTHNS_INQUIRYBLOB;
     PBTHNS_INQUIRYBLOB = ^_BTHNS_INQUIRYBLOB;

type
     _BTHNS_RESTRICTIONBLOB = record
       _type:ULONG;
       serviceHandle:ULONG;
       uuids:array[0..11] of SdpQueryUuid;
       numRange:ULONG;
       pRange:array[0..0] of SdpAttributeRange;
     end;
     BTHNS_RESTRICTIONBLOB = _BTHNS_RESTRICTIONBLOB;
     PBTHNS_RESTRICTIONBLOB = ^_BTHNS_RESTRICTIONBLOB;

{$DEFINE __BTHSDPDEF_H__}
{$ENDIF __BTHSDPDEF_H__}


type
     SdpString = record
       val:^ShortInt;
       _length:ULONG;
     end;

//
// flags for fConnect in SdpSearch::Connect
//
const
      SDP_SEARCH_LOCAL    = $00000001;
      SDP_SEARCH_CACHED   = $00000002;

type
// Forward declarations.
     ISdpNodeContainer = interface;
     ISdpRecord = interface;

     PISdpRecord = ^ISdpRecord;
     PPISdpRecord = ^PISdpRecord;

     NodeDataUnion = record
       case integer of
         0: (int128:SDP_LARGE_INTEGER_16);
         1: (uint128:SDP_ULARGE_INTEGER_16);
         2: (uuid128:GUID);
         3: (uuid32:ULONG);
         4: (uuid16:USHORT);
         5: (int64:LONGLONG);
         6: (uint64:ULONGLONG);
         7: (int32:LONG);
         8: (uint32:ULONG);
         9: (int16:SHORT);
         10: (uint16:USHORT);
         11: (int8:ShortInt);
         12: (uint8:UCHAR);
         13: (booleanVal:UCHAR);
         14: (str:SdpString);
         15: (url:SdpString);
         16: (container:pointer{ISdpNodeContainer});
     end;

     NodeData = record
       _type:USHORT;
       specificType:USHORT;
       u:NodeDataUnion;
     end;
     PNODEDATA = ^NodeData;

     BthDeviceStringType = (BthDeviceStringTypeFriendlyName	:= 0,
	                          BthDeviceStringTypeDeviceName,
	                          BthDeviceStringTypeDisplay,
	                          BthDeviceStringTypeClass,
	                          BthDeviceStringTypeAddress);

// #pragma pack(push)
// #pragma pack(1)
{$PACKRECORDS 1}
     _BthDeviceInfo = record
       btAddress:ULONGLONG;
       cod:ULONG;
       lmpSupportedFeatures:ULONGLONG;
       name:array[0..247] of ShortInt;
     end;
     BthDeviceInfo = _BthDeviceInfo;
     PBthDeviceInfo = ^_BthDeviceInfo;
{$PACKRECORDS DEFAULT}
// #pragma pack(pop)

// Needful types.
     PPUCHAR = ^PUCHAR;
     PPUSHORT = ^PUSHORT;
     PPWCHAR = ^PWCHAR;

     ISdpWalk = interface(IUnknown)
      ['{57134AE6-5D3C-462D-BF2F-810361FBD7E7}']
       function WalkNode(pData:PNODEDATA; state:ULONG):HRESULT; stdcall;
       function WalkStream(elementType:UCHAR; elementSize:ULONG; pStream:PUCHAR):HRESULT; stdcall;
     end;

		   ISdpNodeContainer = interface(IUnknown)
      ['{43F6ED49-6E22-4F81-A8EB-DCED40811A77}']
<<<<<<< HEAD
<<<<<<< HEAD
       function CreateStream(out pStream:PUCHAR;{ppStream:PPUCHAR;} pSize:PULONG):HRESULT; stdcall;
=======
       function CreateStream(out ppStream:PUCHAR;{ppStream:PPUCHAR;} pSize:PULONG):HRESULT; stdcall;
>>>>>>> graemeg/fixes_2_2
=======
       function CreateStream(out ppStream:PUCHAR;{ppStream:PPUCHAR;} pSize:PULONG):HRESULT; stdcall;
>>>>>>> origin/fixes_2_2
       function WriteStream(pStream:PUCHAR; pNumBytesWritten:PULONG):HRESULT; stdcall;
       function AppendNode(pData:PNODEDATA):HRESULT; stdcall;
       function GetType(pType:PNodeContainerType):HRESULT; stdcall;
       function SetType(_type:NodeContainerType):HRESULT; stdcall;
<<<<<<< HEAD
<<<<<<< HEAD
       function Walk(Walk:ISdpWalk):HRESULT; stdcall;
=======
       function Walk(pWalk:ISdpWalk):HRESULT; stdcall;
>>>>>>> graemeg/fixes_2_2
=======
       function Walk(pWalk:ISdpWalk):HRESULT; stdcall;
>>>>>>> origin/fixes_2_2
       function SetNode(nodeIndex:ULONG; pData:PNODEDATA):HRESULT; stdcall;
       function GetNode(nodeIndex:ULONG; pData:PNODEDATA):HRESULT; stdcall;
       function LockContainer(lock:UCHAR):HRESULT; stdcall;
       function GetNodeCount(pNodeCount:PULONG):HRESULT; stdcall;
       function CreateFromStream(pStream:PUCHAR; _size:ULONG):HRESULT; stdcall;
       function GetNodeStringData(nodeIndex:ULONG; pData:PNODEDATA):HRESULT; stdcall;
       function GetStreamSize(pSize:PULONG):HRESULT; stdcall;
     end;

   		ISdpSearch = interface(IUnknown)
      ['{D93B6B2A-5EEF-4E1E-BECF-F5A4340C65F5}']
		     function _Begin(pAddrss:PULONGLONG; fConnect:ULONG):HRESULT; stdcall;
		     function _End:HRESULT; stdcall;
		     function ServiceSearch(pUuidList:PSdpQueryUuid;
                              listSize:ULONG;
                              pHandles:PULONG;
                              pNumHandles:PUSHORT):HRESULT; stdcall;
       function AttributeSearch(_handle:ULONG;
                                pRangeListP:SdpAttributeRange;
                                numRanges:ULONG;
                                out ppSdpRecord:ISdpRecord{ppSdpRecord:PISdpRecord}):HRESULT; stdcall;
       function ServiceAndAttributeSearch(pUuidList:PSdpQueryUuid;
                                          listSize:ULONG;
                                          pRangeList:PSdpAttributeRange;
                                          numRanges:ULONG;
                                          out pppSdpRecord:PISdpRecord{pppSdpRecord:PPISdpRecord};
                                          pNumRecords:PULONG):HRESULT; stdcall;
     end;


     ISdpStream = interface(IUnknown)
      ['{A6ECD9FB-0C7A-41A3-9FF0-0B617E989357}']
<<<<<<< HEAD
<<<<<<< HEAD
{$IF DEFINED(WINCE_EMULATION)}
=======
{$IF DEFINED(WINCE) OR DEFINED(WINCE_EMULATION)}
>>>>>>> graemeg/fixes_2_2
=======
{$IF DEFINED(WINCE) OR DEFINED(WINCE_EMULATION)}
>>>>>>> origin/fixes_2_2
       function Validate(pStream:PUCHAR; _size:ULONG; pErrorByte:PULONG):HRESULT; stdcall;
{$ELSE}
       function Validate(pStream:PUCHAR; _size:ULONG; pErrorByte:PULONG_PTR):HRESULT; stdcall;
{$ENDIF}
<<<<<<< HEAD
<<<<<<< HEAD
       function Walk(Walk:ISdpWalk):HRESULT; stdcall;

       function RetrieveRecords(pStream:PUCHAR; _size:ULONG; var SdpRecords:ISdpRecord;{ppSdpRecords:PISdpRecord;} pNumRecords:PULONG):HRESULT; stdcall;
=======
       function Walk(pStream:PUCHAR; _size:ULONG; pWalk:ISdpWalk):HRESULT; stdcall;

       function RetrieveRecords(pStream:PUCHAR; _size:ULONG; var ppSdpRecords:ISdpRecord;{ppSdpRecords:PISdpRecord;} pNumRecords:PULONG):HRESULT; stdcall;
>>>>>>> graemeg/fixes_2_2
=======
       function Walk(pStream:PUCHAR; _size:ULONG; pWalk:ISdpWalk):HRESULT; stdcall;

       function RetrieveRecords(pStream:PUCHAR; _size:ULONG; var ppSdpRecords:ISdpRecord;{ppSdpRecords:PISdpRecord;} pNumRecords:PULONG):HRESULT; stdcall;
>>>>>>> origin/fixes_2_2

       function RetrieveUuid128(pStream:PUCHAR; pUuid128:PGUID):HRESULT; stdcall;

       function RetrieveUint16(pStream:PUCHAR; pUint16:PUSHORT):HRESULT; stdcall;
<<<<<<< HEAD
<<<<<<< HEAD
       function RetrieveUint32(pStream:PUCHAR; pUint32:PULONG):HRESULT; stdcall;
=======
       function RetrieveUint32(pStream:PUCHAR; pUint32:ULONG):HRESULT; stdcall;
>>>>>>> graemeg/fixes_2_2
=======
       function RetrieveUint32(pStream:PUCHAR; pUint32:ULONG):HRESULT; stdcall;
>>>>>>> origin/fixes_2_2
       function RetrieveUint64(pStream:PUCHAR; pUint64:PULONGLONG):HRESULT; stdcall;
       function RetrieveUint128(pStream:PUCHAR; pUint128:PSDP_ULARGE_INTEGER_16):HRESULT; stdcall;
       function RetrieveInt16(pStream:PUCHAR; pInt16:PSHORT):HRESULT; stdcall;
       function RetrieveInt32(pStream:PUCHAR; pInt32:PLONG):HRESULT; stdcall;
       function RetrieveInt64(pStream:PUCHAR; pInt64:PLONGLONG):HRESULT; stdcall;
       function RetrieveInt128(pStream:PUCHAR; pInt128:PSDP_LARGE_INTEGER_16):HRESULT; stdcall;

       function ByteSwapUuid128(pInUuid128:PGUID; pOutUuid128:PGUID):HRESULT; stdcall;
       function ByteSwapUint128(pInUint128:PSDP_ULARGE_INTEGER_16; pOutUint128:PSDP_ULARGE_INTEGER_16):HRESULT; stdcall;
       function ByteSwapUint64(inUint64:ULONGLONG; pOutUint64:PULONGLONG):HRESULT; stdcall;
       function ByteSwapUint32(uint32:ULONG; pUint32:PULONG):HRESULT; stdcall;
       function ByteSwapUint16(uint16:USHORT; pUint16:PUSHORT):HRESULT; stdcall;
       function ByteSwapInt128(pInInt128:PSDP_LARGE_INTEGER_16; pOutInt128:PSDP_LARGE_INTEGER_16):HRESULT; stdcall;
       function ByteSwapInt64(inInt64:LONGLONG; pOutInt64:PLONGLONG):HRESULT; stdcall;
       function ByteSwapInt32(int32:LONG; pInt32:PLONG):HRESULT; stdcall;
       function ByteSwapInt16(int16:SHORT; pInt16:PSHORT):HRESULT; stdcall;

       function NormalizeUuid(pDataUuid:PNODEDATA; pNormalizeUuid:PGUID):HRESULT; stdcall;
       function RetrieveElementInfo(pStream:PUCHAR;
                                    pElementType:PSDP_TYPE;
                                    pElementSpecificType:PSDP_SPECIFICTYPE;
                                    pElementSize:PULONG;
                                    pStorageSize:PULONG;
<<<<<<< HEAD
<<<<<<< HEAD
                                    out pData:PUCHAR{ppData:PPUCHAR}):HRESULT; stdcall;
       function VerifySequenceOf(pStream:PUCHAR; _size:ULONG; ofType:SDP_TYPE; pSpecificSizes:PUCHAR; pNumFound:PULONG):HRESULT; stdcall;
=======
                                    out ppData:PUCHAR{ppData:PPUCHAR}):HRESULT; stdcall;
       function VerifySequenceOf(pStream:PUCHAR; _size:PULONG; ofType:SDP_TYPE; pSpecificSizes:PUCHAR; pNumFound:PULONG):HRESULT; stdcall;
>>>>>>> graemeg/fixes_2_2
=======
                                    out ppData:PUCHAR{ppData:PPUCHAR}):HRESULT; stdcall;
       function VerifySequenceOf(pStream:PUCHAR; _size:PULONG; ofType:SDP_TYPE; pSpecificSizes:PUCHAR; pNumFound:PULONG):HRESULT; stdcall;
>>>>>>> origin/fixes_2_2
     end;

     ISdpRecord = interface(IUnknown)
      ['{10276714-1456-46D7-B526-8B1E83D5116E}']
       function CreateFromStream(pStream:PUCHAR; _size:ULONG):HRESULT; stdcall;
<<<<<<< HEAD
<<<<<<< HEAD
       function WriteToStream(out pStream:PUCHAR;{ppStream:PPUCHAR;} pStreamSize:PULONG; preSize:ULONG; postSize:ULONG):HRESULT; stdcall;
       function SetAttribute(attribute:USHORT; pNode:PNODEDATA):HRESULT; stdcall;
       function SetAttributeFromStream(attribute:USHORT; pStream:PUCHAR; _size:ULONG):HRESULT; stdcall;
       function GetAttribute(attribute:USHORT; pNode:PNODEDATA):HRESULT; stdcall;
       function GetAttributeAsStream(attribute:USHORT; out pStream:PUCHAR;{ppStream:PPUCHAR;} pSize:PULONG):HRESULT; stdcall;
       function Walk(Walk:ISdpWalk):HRESULT; stdcall;
       function GetAttributeList(out pList:PUSHORT;{ppList:PPUSHORT;} pListSize:PULONG):HRESULT; stdcall;
=======
=======
>>>>>>> origin/fixes_2_2
       function WriteToStream(out ppStream:PUCHAR;{ppStream:PPUCHAR;} pStreamSize:PULONG; preSize:ULONG; postSize:ULONG):HRESULT; stdcall;
       function SetAttribute(attribute:USHORT; pNode:PNODEDATA):HRESULT; stdcall;
       function SetAttributeFromStream(attribute:USHORT; pStream:PUCHAR; _size:ULONG):HRESULT; stdcall;
       function GetAttribute(attribute:USHORT; pNode:PNODEDATA):HRESULT; stdcall;
       function GetAttributeAsStream(attribute:USHORT; out ppStream:PUCHAR;{ppStream:PPUCHAR;} pSize:PULONG):HRESULT; stdcall;
       function Walk(pWalk:ISdpWalk):HRESULT; stdcall;
       function GetAttributeList(out ppList:PUSHORT;{ppList:PPUSHORT;} pListSize:PULONG):HRESULT; stdcall;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
       function GetString(_offset:USHORT; pLangId:PUSHORT; var ppString:PWCHAR{ppString:PPWCHAR}):HRESULT; stdcall;
       function GetIcon(cxRes:longint; cyRes:longint; phIcon:LPHICON):HRESULT; stdcall;
       function GetServiceClass(pServiceClass:LPGUID):HRESULT; stdcall;
     end;


     IBluetoothDevice = interface(IUnknown)
      ['{5BD0418B-D705-4766-B215-183E4EADE341}']
       function Initialize(pInfo:PBthDeviceInfo):HRESULT; stdcall;
       function GetInfo(pInfo:PBthDeviceInfo):HRESULT; stdcall;
       function GetString(_type:BthDeviceStringType; var ppString:PWCHAR{ppString:PPWCHAR}):HRESULT; stdcall;
       function SetString(_type:BthDeviceStringType; pString:PWCHAR):HRESULT; stdcall;
       function GetIcon(cxRes:longint; cyRes:longint; phIcon:LPHICON):HRESULT; stdcall;
       function GetApprovedServices(pServices:PGUID; pServiceCount:PULONG):HRESULT; stdcall;
       function GetPassKey(hwndParent:HWND; pPassKey:PUCHAR; pPassKeyLength:PUCHAR):HRESULT; stdcall;
     end;

     IBluetoothAuthenticate = interface(IUnknown)
      ['{5F0FBA2B-8300-429D-99AD-96A2835D4901}']

     end;

implementation

<<<<<<< HEAD
<<<<<<< HEAD
end.
=======
end.
>>>>>>> graemeg/fixes_2_2
=======
end.
>>>>>>> origin/fixes_2_2
