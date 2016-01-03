{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2000,2001 by the Free Pascal development team

    This include file contains the implementation for variants
    support in FPC as far as it is part of the system unit

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.


 **********************************************************************}

Type
	  TVarTypeToElementInfo = Record
 				   Validbase   : Boolean;	// COM basetype?
				   ValidElement: Boolean;       // allowable in varrarray ?	
				   Size	       : Longint;	// elementsize
				   Flags       : Word;		// see below
				  end;				

{ Variant functions }
function VariantChangeTypeEx(var VargDest: TVarData; const VargSrc: TVarData; LCID: Integer; Flags: Word; VarType: Word): HRESULT; stdcall;
function VariantClear(var Varg: TVarData): HRESULT; stdcall;
function VariantCopy(var VargDest: TVarData; const VargSrc: TVarData): HRESULT; stdcall;
function VariantCopyInd(var VargDest: TVarData;  const VargSrc: TVarData): HRESULT; stdcall;
procedure VariantInit(var Varg: TVarData); stdcall;

{  Variant array functions }

function SafeArrayAccessData(psa: PVarArray; var ppvdata: Pointer): HRESULT; stdcall;
function SafeArrayAllocData(psa: PVarArray): HRESULT; stdcall;
function SafeArrayAllocDescriptor(DimCount: longword; var psa: PVarArray): HRESULT; stdcall;
function SafeArrayCopy(psa: PVarArray; var psaout: PVarArray): HRESULT; stdcall;
function SafeArrayCopyData(psa, psaOut: PVarArray): HRESULT; stdcall;
function SafeArrayCreate(VarType, Dim: dword; const Bounds: TVarArrayBoundArray): PVarArray; stdcall;
function SafeArrayDestroy(psa: PVarArray): HRESULT; stdcall;
function SafeArrayDestroyData(psa: PVarArray): HRESULT; stdcall;
function SafeArrayDestroyDescriptor(psa: PVarArray): HRESULT; stdcall;
function SafeArrayGetDim(psa: PVarArray):HRESULT; stdcall;
function SafeArrayGetElemsize(psa: PVarArray): LongWord; stdcall;
function SafeArrayGetElement(psa: PVarArray; Indices: PVarArrayCoorArray;  Data: Pointer): HRESULT; stdcall;
function SafeArrayGetLBound(psa: PVarArray; Dim: longword;  var LBound: longint): HRESULT; stdcall;
function SafeArrayGetUBound(psa: PVarArray; Dim: longword;  var UBound: longint): HRESULT; stdcall;
function SafeArrayLock(psa: PVarArray): HRESULT; stdcall;
function SafeArrayPtrOfIndex(psa: PVarArray; Indices: PVarArrayCoorArray;  var Address: Pointer): HRESULT; stdcall;
function SafeArrayPutElement(psa: PVarArray; Indices: PVarArrayCoorArray;  const Data: Pointer): HRESULT; stdcall;
function SafeArrayRedim(psa: PVarArray; const NewBound: TVarArrayBound): HRESULT; stdcall;
function SafeArrayUnaccessData(psa: PVarArray): HRESULT; stdcall;
function SafeArrayUnlock(psa: PVarArray): HRESULT; stdcall;

{ Conversion routines NOT in windows oleaut }

function VariantToSmallInt(const VargSrc : TVarData) : SmallInt;
function VariantToLongint(const VargSrc : TVarData) : Longint;
function VariantToShortint(const VargSrc : TVarData) : ShortInt;
function VariantToCardinal(const VargSrc : TVarData) : Cardinal;
<<<<<<< HEAD
<<<<<<< HEAD
{$ifndef FPUNONE}
function VariantToSingle(const VargSrc : TVarData) : Single;
function VariantToDouble(const VargSrc : TVarData) : Double;
function VariantToDate(const VargSrc : TVarData) : TDateTime;
{$endif}
function VariantToCurrency(const VargSrc : TVarData) : Currency;
=======
=======
>>>>>>> origin/fixes_2_2
function VariantToSingle(const VargSrc : TVarData) : Single;
function VariantToDouble(const VargSrc : TVarData) : Double;
function VariantToCurrency(const VargSrc : TVarData) : Currency;
function VariantToDate(const VargSrc : TVarData) : TDateTime;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
function VariantToBoolean(const VargSrc : TVarData) : Boolean;
function VariantToByte(const VargSrc : TVarData) : Byte;
function VariantToInt64(const VargSrc : TVarData ) : Int64;
function VariantToQWord(const VargSrc : TVarData ) : Qword;
function VariantToWideString(const VargSrc : TVarData) : WideString;
function VariantToAnsiString(const VargSrc : TVarData) : AnsiString;
function VariantToShortString(const VargSrc : TVarData) : ShortString;

{Debug routines }
procedure DumpVariant(const VSrc : Variant);
procedure DumpVariant(const aName: string; const VSrc : Variant);
procedure DumpVariant(var F : Text; const VSrc : Variant);
procedure DumpVariant(var F : Text; const aName: string; const VSrc : Variant);

procedure DumpVariant(const VArgSrc : TVarData);
procedure DumpVariant(const aName: string; const VArgSrc : TVarData);
procedure DumpVariant(var F : Text; const VArgSrc : TVarData);
procedure DumpVariant(var F : Text; const aName: string; const VArgSrc : TVarData);



{$IFDEF DEBUG_VARUTILS}
var
  __DEBUG_VARUTILS: Boolean;
{$ENDIF}


{$i varerror.inc}
// Names match the ones in Borland varutils unit.
const
  ARR_NONE          = $0000;
  ARR_FIXEDSIZE     = $0010;
  ARR_OLESTR        = $0100;
  ARR_UNKNOWN       = $0200;
  ARR_DISPATCH      = $0400;
  ARR_VARIANT       = $0800;

  VAR_LOCALE_USER_DEFAULT = $400; // = Windows.LOCALE_USER_DEFAULT

<<<<<<< HEAD
<<<<<<< HEAD
=======


>>>>>>> graemeg/fixes_2_2
=======


>>>>>>> origin/fixes_2_2
  // only the byval types.
  CVarTypeToElementInfo : array[varempty..varqword] of TVarTypeToElementInfo = (
   {varempty    }     (Validbase :  true     ;ValidElement:  true    ;Size:     0       ;Flags :  ARR_NONE),
   {varnull     }     (Validbase :  true     ;ValidElement:  true    ;Size:     0       ;Flags :  ARR_NONE),
   {varsmallint }     (Validbase :  true     ;ValidElement:  true    ;Size:     2       ;Flags :  ARR_NONE),
   {varinteger  }     (Validbase :  true     ;ValidElement:  true    ;Size:     4       ;Flags :  ARR_NONE),
   {varsingle   }     (Validbase :  true     ;ValidElement:  true    ;Size:     4       ;Flags :  ARR_NONE),
   {vardouble   }     (Validbase :  true     ;ValidElement:  true    ;Size:     8       ;Flags :  ARR_NONE),
   {varcurrency }     (Validbase :  true     ;ValidElement:  true    ;Size:     8       ;Flags :  ARR_NONE),
   {vardate     }     (Validbase :  true     ;ValidElement:  true    ;Size:     8       ;Flags :  ARR_NONE),
   {varolestr   }     (Validbase :  true     ;ValidElement:  true    ;Size:     4       ;Flags :  ARR_OLESTR),
   {vardispatch }     (Validbase :  true     ;ValidElement:  true    ;Size:     4       ;Flags :  ARR_DISPATCH),
   {varerror    }     (Validbase :  true     ;ValidElement:  true    ;Size:     4       ;Flags :  ARR_NONE),
   {varboolean  }     (Validbase :  true     ;ValidElement:  true    ;Size:     2       ;Flags :  ARR_NONE),
   {varvariant  }     (Validbase :  true     ;ValidElement:  true    ;Size:    16       ;Flags :  ARR_NONE),
   {varunknown  }     (Validbase :  true     ;ValidElement:  true    ;Size:     4       ;Flags :  ARR_VARIANT),
   {vardecimal  }     (Validbase :  false    ;ValidElement:  true    ;Size:    14       ;Flags :  ARR_UNKNOWN),
   {varunused1}	      (ValidBase :  false    ;ValidElement:  false    ;Size:    4       ;Flags :  ARR_NONE), 	
   {varshortint }     (Validbase :  true     ;ValidElement:  true    ;Size:     1       ;Flags :  ARR_NONE),
   {varbyte     }     (Validbase :  true     ;ValidElement:  true    ;Size:     1       ;Flags :  ARR_NONE),
   {varword     }     (Validbase :  true     ;ValidElement:  true    ;Size:     2       ;Flags :  ARR_NONE),
   {varlongword }     (Validbase :  true     ;ValidElement:  true    ;Size:     4       ;Flags :  ARR_NONE),
   {varint64    }     (Validbase :  true     ;ValidElement:  true    ;Size:     8       ;Flags :  ARR_NONE),
   {varqword    }     (Validbase :  true     ;ValidElement:  true    ;Size:     8       ;Flags :  ARR_NONE));
