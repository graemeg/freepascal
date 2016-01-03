{
  A set of helper functions for libxml2 for FreePascal
  Copyright (C) 2008 by Ivo Steinmann
}

unit xmlxsd;

{$mode objfpc}
{$H+}

interface

uses
  ctypes,
  xml2,
  xmlxsdparser,
  Math,
  Classes,
  DateUtils,
  SysUtils;

resourcestring
  SChildNotFound = 'child %s not found';
  SPropNotFound  = 'attribute %s not found';

const
  IGNORE_LAST = Pointer(-1);

type
  XSDException = class(Exception)
  public
    constructor CreateNode(const Msg: String; name, nameSpace: xmlCharPtr);
  end;

{ Node creation functions }
function xsdNewChildBase64(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: TStream): xmlNodePtr;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdNewChildCData(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: AnsiString): xmlNodePtr;
function xsdNewChildString(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: AnsiString): xmlNodePtr;
=======
function xsdNewChildCData(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Utf8String): xmlNodePtr;
function xsdNewChildString(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Utf8String): xmlNodePtr;
>>>>>>> graemeg/cpstrnew
=======
function xsdNewChildCData(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Utf8String): xmlNodePtr;
function xsdNewChildString(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Utf8String): xmlNodePtr;
>>>>>>> graemeg/cpstrnew
=======
function xsdNewChildCData(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Utf8String): xmlNodePtr;
function xsdNewChildString(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Utf8String): xmlNodePtr;
>>>>>>> graemeg/cpstrnew
=======
function xsdNewChildCData(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Utf8String): xmlNodePtr;
function xsdNewChildString(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Utf8String): xmlNodePtr;
>>>>>>> origin/cpstrnew
function xsdNewChildBoolean(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Boolean; UseWords: Boolean = False): xmlNodePtr;
function xsdNewChildDate(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Year, Month, Day: Longword; BC: Boolean = False; Timezone: PXsdTimezone = nil): xmlNodePtr;
function xsdNewChildDate(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Date: TDateTime; Timezone: PXsdTimezone = nil): xmlNodePtr;
function xsdNewChildTime(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Hour, Minute, Second, Milliseconds: Longword; Timezone: PXsdTimezone = nil): xmlNodePtr;
function xsdNewChildTime(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Time: TDateTime; Timezone: PXsdTimezone = nil): xmlNodePtr;
function xsdNewChildDateTime(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Year, Month, Day, Hour, Minute, Second, Milliseconds: Longword; BC: Boolean = False; Timezone: PXsdTimezone = nil): xmlNodePtr;
function xsdNewChildDateTime(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; DateTime: TDateTime; Timezone: PXsdTimezone = nil): xmlNodePtr;
function xsdNewChildDecimal(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Extended; Precision: Integer = 4; Digits: Integer = 1): xmlNodePtr;
function xsdNewChildDouble(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Double): xmlNodePtr;
function xsdNewChildFloat(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Single): xmlNodePtr;
function xsdNewChildByte(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Shortint): xmlNodePtr;
function xsdNewChildShort(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Smallint): xmlNodePtr;
function xsdNewChildInt(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Longint): xmlNodePtr;
function xsdNewChildLong(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Int64): xmlNodePtr;
function xsdNewChildUnsignedByte(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Byte): xmlNodePtr;
function xsdNewChildUnsignedShort(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Word): xmlNodePtr;
function xsdNewChildUnsignedInt(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Longword): xmlNodePtr;
function xsdNewChildUnsignedLong(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: QWord): xmlNodePtr;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdNewChildEnum(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; enum: array of AnsiString; Value: Integer): xmlNodePtr;

{ Property creation functions }
function xsdNewPropString(node: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: AnsiString): xmlAttrPtr;
=======
function xsdNewChildEnum(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; enum: array of Utf8String; Value: Integer): xmlNodePtr;

{ Property creation functions }
function xsdNewPropString(node: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Utf8String): xmlAttrPtr;
>>>>>>> graemeg/cpstrnew
=======
function xsdNewChildEnum(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; enum: array of Utf8String; Value: Integer): xmlNodePtr;

{ Property creation functions }
function xsdNewPropString(node: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Utf8String): xmlAttrPtr;
>>>>>>> graemeg/cpstrnew
=======
=======
>>>>>>> origin/cpstrnew
function xsdNewChildEnum(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; enum: array of Utf8String; Value: Integer): xmlNodePtr;

{ Property creation functions }
function xsdNewPropString(node: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Utf8String): xmlAttrPtr;
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function xsdNewPropBoolean(node: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Boolean; UseWords: Boolean = False): xmlAttrPtr;
function xsdNewPropDate(node: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Year, Month, Day: Longword; BC: Boolean = False; Timezone: PXsdTimezone = nil): xmlAttrPtr;
function xsdNewPropDate(node: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Date: TDateTime; Timezone: PXsdTimezone = nil): xmlAttrPtr;
function xsdNewPropTime(node: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Hour, Minute, Second, Milliseconds: Longword; Timezone: PXsdTimezone = nil): xmlAttrPtr;
function xsdNewPropTime(node: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Time: TDateTime; Timezone: PXsdTimezone = nil): xmlAttrPtr;
function xsdNewPropDateTime(node: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Year, Month, Day, Hour, Minute, Second, Milliseconds: Longword; BC: Boolean = False; Timezone: PXsdTimezone = nil): xmlAttrPtr;
function xsdNewPropDateTime(node: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; DateTime: TDateTime; Timezone: PXsdTimezone = nil): xmlAttrPtr;
function xsdNewPropDecimal(node: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Extended; Precision: Integer = 4; Digits: Integer = 1): xmlAttrPtr;
function xsdNewPropDouble(node: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Double): xmlAttrPtr;
function xsdNewPropFloat(node: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Single): xmlAttrPtr;
function xsdNewPropByte(node: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Shortint): xmlAttrPtr;
function xsdNewPropShort(node: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Smallint): xmlAttrPtr;
function xsdNewPropInt(node: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Longint): xmlAttrPtr;
function xsdNewPropLong(node: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Int64): xmlAttrPtr;
function xsdNewPropUnsignedByte(node: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Byte): xmlAttrPtr;
function xsdNewPropUnsignedShort(node: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Word): xmlAttrPtr;
function xsdNewPropUnsignedInt(node: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Longword): xmlAttrPtr;
function xsdNewPropUnsignedLong(node: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: QWord): xmlAttrPtr;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdNewPropEnum(node: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; enum: array of AnsiString; Value: Integer): xmlAttrPtr;
=======
function xsdNewPropEnum(node: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; enum: array of Utf8String; Value: Integer): xmlAttrPtr;
>>>>>>> graemeg/cpstrnew
=======
function xsdNewPropEnum(node: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; enum: array of Utf8String; Value: Integer): xmlAttrPtr;
>>>>>>> graemeg/cpstrnew
=======
function xsdNewPropEnum(node: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; enum: array of Utf8String; Value: Integer): xmlAttrPtr;
>>>>>>> graemeg/cpstrnew
=======
function xsdNewPropEnum(node: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; enum: array of Utf8String; Value: Integer): xmlAttrPtr;
>>>>>>> origin/cpstrnew

{ Query functions }
const
  NS_IGNORE  : xmlCharPtr = nil;
  NS_EXCLUDE : xmlCharPtr = pointer(-1);

{ Node query functions }
function xsdTestNodeNs(node: xmlNodePtr; nameSpace: xmlCharPtr): Boolean;
function xsdTestNode(node: xmlNodePtr; name, nameSpace: xmlCharPtr): Boolean;

function xsdTryGetChild(node: xmlNodePtr; name, nameSpace: xmlCharPtr; Index: Integer = 0): xmlNodePtr;
function xsdTryGetChild(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out child: xmlNodePtr; Index: Integer = 0): Boolean;
function xsdTryGetChildChars(node: xmlNodePtr; name, nameSpace: xmlCharPtr; Index: Integer = 0): xmlCharPtr;
function xsdTryGetChildBase64(node: xmlNodePtr; name, nameSpace: xmlCharPtr; const Value: TStream; Index: Integer = 0): Boolean;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdTryGetChildString(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: AnsiString; Index: Integer = 0): Boolean;
=======
function xsdTryGetChildString(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Utf8String; Index: Integer = 0): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryGetChildString(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Utf8String; Index: Integer = 0): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryGetChildString(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Utf8String; Index: Integer = 0): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryGetChildString(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Utf8String; Index: Integer = 0): Boolean;
>>>>>>> origin/cpstrnew
function xsdTryGetChildBoolean(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Boolean; Index: Integer = 0): Boolean;
function xsdTryGetChildDate(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Year, Month, Day: Longword; Timezone: PXsdTimezone = nil; BC: PBoolean = nil; Index: Integer = 0): Boolean;
function xsdTryGetChildDate(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: TDateTime; Timezone: PXsdTimezone = nil; Index: Integer = 0): Boolean;
function xsdTryGetChildTime(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Hour, Minute, Second, Milliseconds: Longword; Timezone: PXsdTimezone = nil; Index: Integer = 0): Boolean;
function xsdTryGetChildTime(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: TDateTime; Timezone: PXsdTimezone = nil; Index: Integer = 0): Boolean;
function xsdTryGetChildDateTime(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Year, Month, Day, Hour, Minute, Second, Milliseconds: Longword; Timezone: PXsdTimezone = nil; BC: PBoolean = nil; Index: Integer = 0): Boolean;
function xsdTryGetChildDateTime(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: TDateTime; Timezone: PXsdTimezone = nil; Index: Integer = 0): Boolean;
function xsdTryGetChildDecimal(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Extended; Index: Integer = 0): Boolean;
function xsdTryGetChildDouble(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Double; Index: Integer = 0): Boolean;
function xsdTryGetChildFloat(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Single; Index: Integer = 0): Boolean;
function xsdTryGetChildByte(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Shortint; Index: Integer = 0): Boolean;
function xsdTryGetChildShort(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Smallint; Index: Integer = 0): Boolean;
function xsdTryGetChildInt(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Longint; Index: Integer = 0): Boolean;
function xsdTryGetChildLong(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Int64; Index: Integer = 0): Boolean;
function xsdTryGetChildUnsignedByte(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Byte; Index: Integer = 0): Boolean;
function xsdTryGetChildUnsignedShort(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Word; Index: Integer = 0): Boolean;
function xsdTryGetChildUnsignedInt(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Longword; Index: Integer = 0): Boolean;
function xsdTryGetChildUnsignedLong(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: QWord; Index: Integer = 0): Boolean;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdTryGetChildEnum(node: xmlNodePtr; name, nameSpace: xmlCharPtr; enum: array of AnsiString; out Value: Integer; Index: Integer = 0): Boolean;
=======
function xsdTryGetChildEnum(node: xmlNodePtr; name, nameSpace: xmlCharPtr; enum: array of Utf8String; out Value: Integer; Index: Integer = 0): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryGetChildEnum(node: xmlNodePtr; name, nameSpace: xmlCharPtr; enum: array of Utf8String; out Value: Integer; Index: Integer = 0): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryGetChildEnum(node: xmlNodePtr; name, nameSpace: xmlCharPtr; enum: array of Utf8String; out Value: Integer; Index: Integer = 0): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryGetChildEnum(node: xmlNodePtr; name, nameSpace: xmlCharPtr; enum: array of Utf8String; out Value: Integer; Index: Integer = 0): Boolean;
>>>>>>> origin/cpstrnew

function xsdGetChild(node: xmlNodePtr; name, nameSpace: xmlCharPtr; Index: Integer = 0): xmlNodePtr;
function xsdGetChildChars(node: xmlNodePtr; name, nameSpace: xmlCharPtr; Index: Integer = 0): xmlCharPtr;
procedure xsdGetChildBase64(node: xmlNodePtr; name, nameSpace: xmlCharPtr; const Value: TStream; Index: Integer = 0);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure xsdGetChildString(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: AnsiString; Index: Integer = 0);
=======
procedure xsdGetChildString(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Utf8String; Index: Integer = 0);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdGetChildString(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Utf8String; Index: Integer = 0);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdGetChildString(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Utf8String; Index: Integer = 0);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdGetChildString(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Utf8String; Index: Integer = 0);
>>>>>>> origin/cpstrnew
procedure xsdGetChildBoolean(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Boolean; Index: Integer = 0);
procedure xsdGetChildDate(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Year, Month, Day: Longword; Timezone: PXsdTimezone = nil; BC: PBoolean = nil; Index: Integer = 0);
procedure xsdGetChildDate(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: TDateTime; Timezone: PXsdTimezone = nil; Index: Integer = 0);
procedure xsdGetChildTime(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Hour, Minute, Second, Milliseconds: Longword; Timezone: PXsdTimezone = nil; Index: Integer = 0);
procedure xsdGetChildTime(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: TDateTime; Timezone: PXsdTimezone = nil; Index: Integer = 0);
procedure xsdGetChildDateTime(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Year, Month, Day, Hour, Minute, Second, Milliseconds: Longword; Timezone: PXsdTimezone = nil; BC: PBoolean = nil; Index: Integer = 0);
procedure xsdGetChildDateTime(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: TDateTime; Timezone: PXsdTimezone = nil; Index: Integer = 0);
procedure xsdGetChildDecimal(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Extended; Index: Integer = 0);
procedure xsdGetChildDouble(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Double; Index: Integer = 0);
procedure xsdGetChildFloat(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Single; Index: Integer = 0);
procedure xsdGetChildByte(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Shortint; Index: Integer = 0);
procedure xsdGetChildShort(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Smallint; Index: Integer = 0);
procedure xsdGetChildInt(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Longint; Index: Integer = 0);
procedure xsdGetChildLong(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Int64; Index: Integer = 0);
procedure xsdGetChildUnsignedByte(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Byte; Index: Integer = 0);
procedure xsdGetChildUnsignedShort(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Word; Index: Integer = 0);
procedure xsdGetChildUnsignedInt(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Longword; Index: Integer = 0);
procedure xsdGetChildUnsignedLong(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: QWord; Index: Integer = 0);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure xsdGetChildEnum(node: xmlNodePtr; name, nameSpace: xmlCharPtr; enum: array of AnsiString; out Value: Integer; Index: Integer = 0);
=======
procedure xsdGetChildEnum(node: xmlNodePtr; name, nameSpace: xmlCharPtr; enum: array of Utf8String; out Value: Integer; Index: Integer = 0);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdGetChildEnum(node: xmlNodePtr; name, nameSpace: xmlCharPtr; enum: array of Utf8String; out Value: Integer; Index: Integer = 0);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdGetChildEnum(node: xmlNodePtr; name, nameSpace: xmlCharPtr; enum: array of Utf8String; out Value: Integer; Index: Integer = 0);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdGetChildEnum(node: xmlNodePtr; name, nameSpace: xmlCharPtr; enum: array of Utf8String; out Value: Integer; Index: Integer = 0);
>>>>>>> origin/cpstrnew

{ Node parsing functions }
function xsdTryNext(var node: xmlNodePtr; name, nameSpace: xmlCharPtr): xmlNodePtr;
function xsdTryNext(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out last: xmlNodePtr): Boolean;
function xsdTryNextChars(var node: xmlNodePtr; name, nameSpace: xmlCharPtr): xmlCharPtr;
function xsdTryNextBase64(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; const Value: TStream): Boolean;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdTryNextString(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: AnsiString): Boolean;
=======
function xsdTryNextString(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Utf8String): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryNextString(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Utf8String): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryNextString(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Utf8String): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryNextString(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Utf8String): Boolean;
>>>>>>> origin/cpstrnew
function xsdTryNextBoolean(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Boolean): Boolean;
function xsdTryNextDate(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Year, Month, Day: Longword; Timezone: PXsdTimezone = nil; BC: PBoolean = nil): Boolean;
function xsdTryNextDate(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: TDateTime; Timezone: PXsdTimezone = nil): Boolean;
function xsdTryNextTime(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Hour, Minute, Second, Milliseconds: Longword; Timezone: PXsdTimezone = nil): Boolean;
function xsdTryNextTime(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: TDateTime; Timezone: PXsdTimezone = nil): Boolean;
function xsdTryNextDateTime(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Year, Month, Day, Hour, Minute, Second, Milliseconds: Longword; Timezone: PXsdTimezone = nil; BC: PBoolean = nil): Boolean;
function xsdTryNextDateTime(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: TDateTime; Timezone: PXsdTimezone = nil): Boolean;
function xsdTryNextDecimal(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Extended): Boolean;
function xsdTryNextDouble(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Double): Boolean;
function xsdTryNextFloat(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Single): Boolean;
function xsdTryNextByte(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Shortint): Boolean;
function xsdTryNextShort(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Smallint): Boolean;
function xsdTryNextInt(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Longint): Boolean;
function xsdTryNextLong(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Int64): Boolean;
function xsdTryNextUnsignedByte(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Byte): Boolean;
function xsdTryNextUnsignedShort(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Word): Boolean;
function xsdTryNextUnsignedInt(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Longword): Boolean;
function xsdTryNextUnsignedLong(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: QWord): Boolean;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdTryNextEnum(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; enum: array of AnsiString; out Value: Integer): Boolean;
=======
function xsdTryNextEnum(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; enum: array of Utf8String; out Value: Integer): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryNextEnum(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; enum: array of Utf8String; out Value: Integer): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryNextEnum(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; enum: array of Utf8String; out Value: Integer): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryNextEnum(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; enum: array of Utf8String; out Value: Integer): Boolean;
>>>>>>> origin/cpstrnew

function xsdNext(var node: xmlNodePtr; name, nameSpace: xmlCharPtr): xmlNodePtr;
function xsdNextChars(var node: xmlNodePtr; name, nameSpace: xmlCharPtr): xmlCharPtr;
procedure xsdNextBase64(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; const Value: TStream);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure xsdNextString(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: AnsiString);
=======
procedure xsdNextString(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Utf8String);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdNextString(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Utf8String);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdNextString(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Utf8String);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdNextString(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Utf8String);
>>>>>>> origin/cpstrnew
procedure xsdNextBoolean(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Boolean);
procedure xsdNextDate(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Year, Month, Day: Longword; Timezone: PXsdTimezone = nil; BC: PBoolean = nil);
procedure xsdNextDate(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: TDateTime; Timezone: PXsdTimezone = nil);
procedure xsdNextTime(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Hour, Minute, Second, Milliseconds: Longword; Timezone: PXsdTimezone = nil);
procedure xsdNextTime(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: TDateTime; Timezone: PXsdTimezone = nil);
procedure xsdNextDateTime(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Year, Month, Day, Hour, Minute, Second, Milliseconds: Longword; Timezone: PXsdTimezone = nil; BC: PBoolean = nil);
procedure xsdNextDateTime(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: TDateTime; Timezone: PXsdTimezone = nil);
procedure xsdNextDecimal(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Extended);
procedure xsdNextDouble(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Double);
procedure xsdNextFloat(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Single);
procedure xsdNextByte(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Shortint);
procedure xsdNextShort(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Smallint);
procedure xsdNextInt(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Longint);
procedure xsdNextLong(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Int64);
procedure xsdNextUnsignedByte(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Byte);
procedure xsdNextUnsignedShort(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Word);
procedure xsdNextUnsignedInt(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Longword);
procedure xsdNextUnsignedLong(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: QWord);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure xsdNextEnum(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; enum: array of AnsiString; out Value: Integer);
=======
procedure xsdNextEnum(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; enum: array of Utf8String; out Value: Integer);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdNextEnum(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; enum: array of Utf8String; out Value: Integer);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdNextEnum(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; enum: array of Utf8String; out Value: Integer);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdNextEnum(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; enum: array of Utf8String; out Value: Integer);
>>>>>>> origin/cpstrnew

{ Property query functions }
function xsdTestPropNs(attr: xmlAttrPtr; nameSpace: xmlCharPtr): Boolean;
function xsdTestProp(attr: xmlAttrPtr; name, nameSpace: xmlCharPtr): Boolean;

function xsdTryGetProp(node: xmlNodePtr; name, nameSpace: xmlCharPtr): xmlAttrPtr;
function xsdTryGetPropChars(node: xmlNodePtr; name, nameSpace: xmlCharPtr): xmlCharPtr;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdTryGetPropString(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: AnsiString): Boolean;
=======
function xsdTryGetPropString(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Utf8String): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryGetPropString(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Utf8String): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryGetPropString(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Utf8String): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryGetPropString(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Utf8String): Boolean;
>>>>>>> origin/cpstrnew
function xsdTryGetPropBoolean(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Boolean): Boolean;
function xsdTryGetPropDate(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Year, Month, Day: Longword; Timezone: PXsdTimezone = nil; BC: PBoolean = nil): Boolean;
function xsdTryGetPropDate(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: TDateTime; Timezone: PXsdTimezone = nil): Boolean;
function xsdTryGetPropTime(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Hour, Minute, Second, Milliseconds: Longword; Timezone: PXsdTimezone = nil): Boolean;
function xsdTryGetPropTime(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: TDateTime; Timezone: PXsdTimezone = nil): Boolean;
function xsdTryGetPropDateTime(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Year, Month, Day, Hour, Minute, Second, Milliseconds: Longword; Timezone: PXsdTimezone = nil; BC: PBoolean = nil): Boolean;
function xsdTryGetPropDateTime(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: TDateTime; Timezone: PXsdTimezone = nil): Boolean;
function xsdTryGetPropDecimal(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Extended): Boolean;
function xsdTryGetPropDouble(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Double): Boolean;
function xsdTryGetPropFloat(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Single): Boolean;
function xsdTryGetPropByte(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Shortint): Boolean;
function xsdTryGetPropShort(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Smallint): Boolean;
function xsdTryGetPropInt(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Longint): Boolean;
function xsdTryGetPropLong(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Int64): Boolean;
function xsdTryGetPropUnsignedByte(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Byte): Boolean;
function xsdTryGetPropUnsignedShort(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Word): Boolean;
function xsdTryGetPropUnsignedInt(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Longword): Boolean;
function xsdTryGetPropUnsignedLong(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: QWord): Boolean;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdTryGetPropEnum(node: xmlNodePtr; name, nameSpace: xmlCharPtr; enum: array of AnsiString; out Value: Integer): Boolean;

function xsdGetProp(node: xmlNodePtr; name, nameSpace: xmlCharPtr): xmlAttrPtr;
function xsdGetPropChars(node: xmlNodePtr; name, nameSpace: xmlCharPtr): xmlCharPtr;
procedure xsdGetPropString(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: AnsiString);
=======
=======
>>>>>>> graemeg/cpstrnew
function xsdTryGetPropEnum(node: xmlNodePtr; name, nameSpace: xmlCharPtr; enum: array of Utf8String; out Value: Integer): Boolean;

function xsdGetProp(node: xmlNodePtr; name, nameSpace: xmlCharPtr): xmlAttrPtr;
function xsdGetPropChars(node: xmlNodePtr; name, nameSpace: xmlCharPtr): xmlCharPtr;
procedure xsdGetPropString(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Utf8String);
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
function xsdTryGetPropEnum(node: xmlNodePtr; name, nameSpace: xmlCharPtr; enum: array of Utf8String; out Value: Integer): Boolean;

function xsdGetProp(node: xmlNodePtr; name, nameSpace: xmlCharPtr): xmlAttrPtr;
function xsdGetPropChars(node: xmlNodePtr; name, nameSpace: xmlCharPtr): xmlCharPtr;
procedure xsdGetPropString(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Utf8String);
>>>>>>> graemeg/cpstrnew
=======
function xsdTryGetPropEnum(node: xmlNodePtr; name, nameSpace: xmlCharPtr; enum: array of Utf8String; out Value: Integer): Boolean;

function xsdGetProp(node: xmlNodePtr; name, nameSpace: xmlCharPtr): xmlAttrPtr;
function xsdGetPropChars(node: xmlNodePtr; name, nameSpace: xmlCharPtr): xmlCharPtr;
procedure xsdGetPropString(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Utf8String);
>>>>>>> origin/cpstrnew
procedure xsdGetPropBoolean(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Boolean);
procedure xsdGetPropDate(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Year, Month, Day: Longword; Timezone: PXsdTimezone = nil; BC: PBoolean = nil);
procedure xsdGetPropDate(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: TDateTime; Timezone: PXsdTimezone = nil);
procedure xsdGetPropTime(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Hour, Minute, Second, Milliseconds: Longword; Timezone: PXsdTimezone = nil);
procedure xsdGetPropTime(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: TDateTime; Timezone: PXsdTimezone = nil);
procedure xsdGetPropDateTime(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Year, Month, Day, Hour, Minute, Second, Milliseconds: Longword; Timezone: PXsdTimezone = nil; BC: PBoolean = nil);
procedure xsdGetPropDateTime(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: TDateTime; Timezone: PXsdTimezone = nil);
procedure xsdGetPropDecimal(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Extended);
procedure xsdGetPropDouble(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Double);
procedure xsdGetPropFloat(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Single);
procedure xsdGetPropByte(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Shortint);
procedure xsdGetPropShort(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Smallint);
procedure xsdGetPropInt(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Longint);
procedure xsdGetPropLong(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Int64);
procedure xsdGetPropUnsignedByte(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Byte);
procedure xsdGetPropUnsignedShort(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Word);
procedure xsdGetPropUnsignedInt(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Longword);
procedure xsdGetPropUnsignedLong(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: QWord);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure xsdGetPropEnum(node: xmlNodePtr; name, nameSpace: xmlCharPtr; enum: array of AnsiString; out Value: Integer);

function xsdRemoveBlanks(content: xmlCharPtr; out cleaned: AnsiString): boolean;
=======
procedure xsdGetPropEnum(node: xmlNodePtr; name, nameSpace: xmlCharPtr; enum: array of Utf8String; out Value: Integer);

function xsdRemoveBlanks(content: xmlCharPtr; out cleaned: Utf8String): boolean;
>>>>>>> graemeg/cpstrnew
=======
procedure xsdGetPropEnum(node: xmlNodePtr; name, nameSpace: xmlCharPtr; enum: array of Utf8String; out Value: Integer);

function xsdRemoveBlanks(content: xmlCharPtr; out cleaned: Utf8String): boolean;
>>>>>>> graemeg/cpstrnew
=======
procedure xsdGetPropEnum(node: xmlNodePtr; name, nameSpace: xmlCharPtr; enum: array of Utf8String; out Value: Integer);

function xsdRemoveBlanks(content: xmlCharPtr; out cleaned: Utf8String): boolean;
>>>>>>> graemeg/cpstrnew
=======
procedure xsdGetPropEnum(node: xmlNodePtr; name, nameSpace: xmlCharPtr; enum: array of Utf8String; out Value: Integer);

function xsdRemoveBlanks(content: xmlCharPtr; out cleaned: Utf8String): boolean;
>>>>>>> origin/cpstrnew

implementation

constructor XSDException.CreateNode(const Msg: String; name, nameSpace: xmlCharPtr);
var
  S: String;
begin
  S := PChar(name);
  if Assigned(nameSpace) then
    S := PChar(nameSpace)+':'+S;
  inherited CreateFmt(Msg, [S]);
end;

function xsdNewChildBase64(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: TStream): xmlNodePtr;
var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Tmp: AnsiString;
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> origin/cpstrnew
begin
  Tmp := xsdFormatBase64(Value);
  Result := xmlNewChild(parent, ns, name, PChar(Tmp));
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdNewChildCData(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: AnsiString): xmlNodePtr;
=======
function xsdNewChildCData(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Utf8String): xmlNodePtr;
>>>>>>> graemeg/cpstrnew
=======
function xsdNewChildCData(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Utf8String): xmlNodePtr;
>>>>>>> graemeg/cpstrnew
=======
function xsdNewChildCData(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Utf8String): xmlNodePtr;
>>>>>>> graemeg/cpstrnew
=======
function xsdNewChildCData(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Utf8String): xmlNodePtr;
>>>>>>> origin/cpstrnew
begin
  Result := xmlNewNode(ns, name);
  xmlAddChild(Result, xmlNewCDataBlock(parent^.doc, PChar(Value), Length(Value)));
  xmlAddChild(parent, Result);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdNewChildString(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: AnsiString): xmlNodePtr;
=======
function xsdNewChildString(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Utf8String): xmlNodePtr;
>>>>>>> graemeg/cpstrnew
=======
function xsdNewChildString(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Utf8String): xmlNodePtr;
>>>>>>> graemeg/cpstrnew
=======
function xsdNewChildString(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Utf8String): xmlNodePtr;
>>>>>>> graemeg/cpstrnew
=======
function xsdNewChildString(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Utf8String): xmlNodePtr;
>>>>>>> origin/cpstrnew
begin
  Result := xmlNewChild(parent, ns, name, PChar(Value));
end;

function xsdNewChildBoolean(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Boolean; UseWords: Boolean): xmlNodePtr;
var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Tmp: AnsiString;
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> origin/cpstrnew
begin
  Tmp := xsdFormatBoolean(Value, UseWords);
  Result := xmlNewChild(parent, ns, name, PChar(Tmp));
end;

function xsdNewChildTime(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Hour, Minute, Second, Milliseconds: Longword; Timezone: PXsdTimezone): xmlNodePtr;
var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Tmp: AnsiString;
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> origin/cpstrnew
begin
  Tmp := xsdFormatTime(Hour, Minute, Second, Milliseconds, Timezone);
  Result := xmlNewChild(parent, ns, name, PChar(Tmp));
end;

function xsdNewChildTime(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Time: TDateTime; Timezone: PXsdTimezone): xmlNodePtr;
var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Tmp: AnsiString;
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> origin/cpstrnew
begin
  Tmp := xsdFormatTime(Time, Timezone);
  Result := xmlNewChild(parent, ns, name, PChar(Tmp));
end;

function xsdNewChildDate(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Year, Month, Day: Longword; BC: Boolean; Timezone: PXsdTimezone): xmlNodePtr;
var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Tmp: AnsiString;
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> origin/cpstrnew
begin
  Tmp := xsdFormatDate(Year, Month, Day, BC, Timezone);
  Result := xmlNewChild(parent, ns, name, PChar(Tmp));
end;

function xsdNewChildDate(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Date: TDateTime; Timezone: PXsdTimezone): xmlNodePtr;
var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Tmp: AnsiString;
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> origin/cpstrnew
begin
  Tmp := xsdFormatDate(Date, Timezone);
  Result := xmlNewChild(parent, ns, name, PChar(Tmp));
end;

function xsdNewChildDateTime(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Year, Month, Day, Hour, Minute, Second, Milliseconds: Longword; BC: Boolean; Timezone: PXsdTimezone): xmlNodePtr;
var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Tmp: AnsiString;
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> origin/cpstrnew
begin
  Tmp := xsdFormatDateTime(Year, Month, Day, Hour, Minute, Second, Milliseconds, BC, Timezone);
  Result := xmlNewChild(parent, ns, name, PChar(Tmp));
end;

function xsdNewChildDateTime(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; DateTime: TDateTime; Timezone: PXsdTimezone): xmlNodePtr;
var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Tmp: AnsiString;
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> origin/cpstrnew
begin
  Tmp := xsdFormatDateTime(DateTime, Timezone);
  Result := xmlNewChild(parent, ns, name, PChar(Tmp));
end;

function xsdNewChildDecimal(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Extended; Precision: Integer; Digits: Integer): xmlNodePtr;
var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Tmp: AnsiString;
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> origin/cpstrnew
begin
  Tmp := xsdFormatDecimal(Value, Precision, Digits);
  Result := xmlNewChild(parent, ns, name, PChar(Tmp));
end;

function xsdNewChildDouble(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Double): xmlNodePtr;
var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Tmp: AnsiString;
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> origin/cpstrnew
begin
  Tmp := xsdFormatDouble(Value);
  Result := xmlNewChild(parent, ns, name, PChar(Tmp));
end;

function xsdNewChildFloat(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Single): xmlNodePtr;
var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Tmp: AnsiString;
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> origin/cpstrnew
begin
  Tmp := xsdFormatFloat(Value);
  Result := xmlNewChild(parent, ns, name, PChar(Tmp));
end;

function xsdNewChildByte(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Shortint): xmlNodePtr;
var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Tmp: AnsiString;
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> origin/cpstrnew
begin
  Tmp := xsdFormatByte(Value);
  Result := xmlNewChild(parent, ns, name, PChar(Tmp));
end;

function xsdNewChildShort(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Smallint): xmlNodePtr;
var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Tmp: AnsiString;
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> origin/cpstrnew
begin
  Tmp := xsdFormatShort(Value);
  Result := xmlNewChild(parent, ns, name, PChar(Tmp));
end;

function xsdNewChildInt(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Longint): xmlNodePtr;
var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Tmp: AnsiString;
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> origin/cpstrnew
begin
  Tmp := xsdFormatInt(Value);
  Result := xmlNewChild(parent, ns, name, PChar(Tmp));
end;

function xsdNewChildLong(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Int64): xmlNodePtr;
var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Tmp: AnsiString;
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> origin/cpstrnew
begin
  Tmp := xsdFormatLong(Value);
  Result := xmlNewChild(parent, ns, name, PChar(Tmp));
end;

function xsdNewChildUnsignedByte(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Byte): xmlNodePtr;
var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Tmp: AnsiString;
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> origin/cpstrnew
begin
  Tmp := xsdFormatUnsignedByte(Value);
  Result := xmlNewChild(parent, ns, name, PChar(Tmp));
end;

function xsdNewChildUnsignedShort(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Word): xmlNodePtr;
var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Tmp: AnsiString;
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> origin/cpstrnew
begin
  Tmp := xsdFormatUnsignedShort(Value);
  Result := xmlNewChild(parent, ns, name, PChar(Tmp));
end;

function xsdNewChildUnsignedInt(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Longword): xmlNodePtr;
var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Tmp: AnsiString;
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> origin/cpstrnew
begin
  Tmp := xsdFormatUnsignedInt(Value);
  Result := xmlNewChild(parent, ns, name, PChar(Tmp));
end;

function xsdNewChildUnsignedLong(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: QWord): xmlNodePtr;
var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Tmp: AnsiString;
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> origin/cpstrnew
begin
  Tmp := xsdFormatUnsignedLong(Value);
  Result := xmlNewChild(parent, ns, name, PChar(Tmp));
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdNewChildEnum(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; enum: array of AnsiString; Value: Integer): xmlNodePtr;
var
  Tmp: AnsiString;
=======
function xsdNewChildEnum(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; enum: array of Utf8String; Value: Integer): xmlNodePtr;
var
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdNewChildEnum(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; enum: array of Utf8String; Value: Integer): xmlNodePtr;
var
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdNewChildEnum(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; enum: array of Utf8String; Value: Integer): xmlNodePtr;
var
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdNewChildEnum(parent: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; enum: array of Utf8String; Value: Integer): xmlNodePtr;
var
  Tmp: Utf8String;
>>>>>>> origin/cpstrnew
begin
  Tmp := xsdFormatEnum(enum, Value);
  Result := xmlNewChild(parent, ns, name, PChar(Tmp));
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdNewPropString(node: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: AnsiString): xmlAttrPtr;
=======
function xsdNewPropString(node: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Utf8String): xmlAttrPtr;
>>>>>>> graemeg/cpstrnew
=======
function xsdNewPropString(node: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Utf8String): xmlAttrPtr;
>>>>>>> graemeg/cpstrnew
=======
function xsdNewPropString(node: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Utf8String): xmlAttrPtr;
>>>>>>> graemeg/cpstrnew
=======
function xsdNewPropString(node: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Utf8String): xmlAttrPtr;
>>>>>>> origin/cpstrnew
begin
  Result := xmlNewNsProp(node, ns, name, PChar(Value));
end;

function xsdNewPropBoolean(node: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Boolean; UseWords: Boolean): xmlAttrPtr;
var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Tmp: AnsiString;
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> origin/cpstrnew
begin
  Tmp := xsdFormatBoolean(Value, UseWords);
  Result := xmlNewNsProp(node, ns, name, PChar(Tmp));
end;

function xsdNewPropTime(node: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Hour, Minute, Second, Milliseconds: Longword; Timezone: PXsdTimezone): xmlAttrPtr;
var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Tmp: AnsiString;
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> origin/cpstrnew
begin
  Tmp := xsdFormatTime(Hour, Minute, Second, Milliseconds, Timezone);
  Result := xmlNewNsProp(node, ns, name, PChar(Tmp));
end;

function xsdNewPropTime(node: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Time: TDateTime; Timezone: PXsdTimezone): xmlAttrPtr;
var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Tmp: AnsiString;
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> origin/cpstrnew
begin
  Tmp := xsdFormatTime(Time, Timezone);
  Result := xmlNewNsProp(node, ns, name, PChar(Tmp));
end;

function xsdNewPropDate(node: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Year, Month, Day: Longword; BC: Boolean; Timezone: PXsdTimezone): xmlAttrPtr;
var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Tmp: AnsiString;
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> origin/cpstrnew
begin
  Tmp := xsdFormatDate(Year, Month, Day, BC, Timezone);
  Result := xmlNewNsProp(node, ns, name, PChar(Tmp));
end;

function xsdNewPropDate(node: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Date: TDateTime; Timezone: PXsdTimezone): xmlAttrPtr;
var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Tmp: AnsiString;
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> origin/cpstrnew
begin
  Tmp := xsdFormatDate(Date, Timezone);
  Result := xmlNewNsProp(node, ns, name, PChar(Tmp));
end;

function xsdNewPropDateTime(node: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Year, Month, Day, Hour, Minute, Second, Milliseconds: Longword; BC: Boolean; Timezone: PXsdTimezone): xmlAttrPtr;
var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Tmp: AnsiString;
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> origin/cpstrnew
begin
  Tmp := xsdFormatDateTime(Year, Month, Day, Hour, Minute, Second, Milliseconds, BC, Timezone);
  Result := xmlNewNsProp(node, ns, name, PChar(Tmp));
end;

function xsdNewPropDateTime(node: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; DateTime: TDateTime; Timezone: PXsdTimezone): xmlAttrPtr;
var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Tmp: AnsiString;
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> origin/cpstrnew
begin
  Tmp := xsdFormatDateTime(DateTime, Timezone);
  Result := xmlNewNsProp(node, ns, name, PChar(Tmp));
end;

function xsdNewPropDecimal(node: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Extended; Precision: Integer; Digits: Integer): xmlAttrPtr;
var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Tmp: AnsiString;
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> origin/cpstrnew
begin
  Tmp := xsdFormatDecimal(Value, Precision, Digits);
  Result := xmlNewNsProp(node, ns, name, PChar(Tmp));
end;

function xsdNewPropDouble(node: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Double): xmlAttrPtr;
var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Tmp: AnsiString;
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> origin/cpstrnew
begin
  Tmp := xsdFormatDouble(Value);
  Result := xmlNewNsProp(node, ns, name, PChar(Tmp));
end;

function xsdNewPropFloat(node: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Single): xmlAttrPtr;
var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Tmp: AnsiString;
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> origin/cpstrnew
begin
  Tmp := xsdFormatFloat(Value);
  Result := xmlNewNsProp(node, ns, name, PChar(Tmp));
end;

function xsdNewPropByte(node: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Shortint): xmlAttrPtr;
var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Tmp: AnsiString;
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> origin/cpstrnew
begin
  Tmp := xsdFormatByte(Value);
  Result := xmlNewNsProp(node, ns, name, PChar(Tmp));
end;

function xsdNewPropShort(node: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Smallint): xmlAttrPtr;
var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Tmp: AnsiString;
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> origin/cpstrnew
begin
  Tmp := xsdFormatShort(Value);
  Result := xmlNewNsProp(node, ns, name, PChar(Tmp));
end;

function xsdNewPropInt(node: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Longint): xmlAttrPtr;
var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Tmp: AnsiString;
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> origin/cpstrnew
begin
  Tmp := xsdFormatInt(Value);
  Result := xmlNewNsProp(node, ns, name, PChar(Tmp));
end;

function xsdNewPropLong(node: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Int64): xmlAttrPtr;
var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Tmp: AnsiString;
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> origin/cpstrnew
begin
  Tmp := xsdFormatLong(Value);
  Result := xmlNewNsProp(node, ns, name, PChar(Tmp));
end;

function xsdNewPropUnsignedByte(node: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Byte): xmlAttrPtr;
var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Tmp: AnsiString;
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> origin/cpstrnew
begin
  Tmp := xsdFormatUnsignedByte(Value);
  Result := xmlNewNsProp(node, ns, name, PChar(Tmp));
end;

function xsdNewPropUnsignedShort(node: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Word): xmlAttrPtr;
var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Tmp: AnsiString;
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> origin/cpstrnew
begin
  Tmp := xsdFormatUnsignedShort(Value);
  Result := xmlNewNsProp(node, ns, name, PChar(Tmp));
end;

function xsdNewPropUnsignedInt(node: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: Longword): xmlAttrPtr;
var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Tmp: AnsiString;
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> origin/cpstrnew
begin
  Tmp := xsdFormatUnsignedInt(Value);
  Result := xmlNewNsProp(node, ns, name, PChar(Tmp));
end;

function xsdNewPropUnsignedLong(node: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; Value: QWord): xmlAttrPtr;
var
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Tmp: AnsiString;
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
  Tmp: Utf8String;
>>>>>>> origin/cpstrnew
begin
  Tmp := xsdFormatUnsignedLong(Value);
  Result := xmlNewNsProp(node, ns, name, PChar(Tmp));
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdNewPropEnum(node: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; enum: array of AnsiString; Value: Integer): xmlAttrPtr;
var
  Tmp: AnsiString;
=======
function xsdNewPropEnum(node: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; enum: array of Utf8String; Value: Integer): xmlAttrPtr;
var
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdNewPropEnum(node: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; enum: array of Utf8String; Value: Integer): xmlAttrPtr;
var
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdNewPropEnum(node: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; enum: array of Utf8String; Value: Integer): xmlAttrPtr;
var
  Tmp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdNewPropEnum(node: xmlNodePtr; ns: xmlNsPtr; name: xmlCharPtr; enum: array of Utf8String; Value: Integer): xmlAttrPtr;
var
  Tmp: Utf8String;
>>>>>>> origin/cpstrnew
begin
  Tmp := xsdFormatEnum(enum, Value);
  Result := xmlNewNsProp(node, ns, name, PChar(Tmp));
end;

function xsdTestNodeNs(node: xmlNodePtr; nameSpace: xmlCharPtr): Boolean;
begin
  Result :=
     (nameSpace = NS_IGNORE) or
    ((nameSpace = NS_EXCLUDE) and (node^.ns = nil)) or
    ((nameSpace <> NS_EXCLUDE) and (nameSpace <> NS_IGNORE) and (node^.ns <> nil) and (xmlStrEqual(nameSpace, node^.ns^.href) <> 0));
end;

function xsdTestNode(node: xmlNodePtr; name, nameSpace: xmlCharPtr): Boolean;
begin
  Result := (node <> nil) and (xmlStrEqual(name, node^.name) <> 0) and xsdTestNodeNs(node, nameSpace);
end;

function xsdTryGetChild(node: xmlNodePtr; name, nameSpace: xmlCharPtr; Index: integer): xmlNodePtr;
begin
  if Assigned(node) and (Index >= 0) then
  begin
    Result := node^.children;
    while Assigned(Result) do
    begin
      if xsdTestNode(Result, name, nameSpace) then
      begin
        if Index = 0 then
          Exit;
        Dec(Index);
      end;
      Result := Result^.next;
    end;
  end else
    Result := nil;
end;

function xsdTryGetChild(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out child: xmlNodePtr; Index: Integer): Boolean;
begin
  child := xsdTryGetChild(node, name, nameSpace, Index);
  Result := Assigned(child);
end;

function xsdTryGetChildChars(node: xmlNodePtr; name, nameSpace: xmlCharPtr; Index: Integer): xmlCharPtr;
begin
  Result := xmlNodeGetContent(xsdTryGetChild(node, name, nameSpace, Index));
end;

function xsdTryGetChildBase64(node: xmlNodePtr; name, nameSpace: xmlCharPtr; const Value: TStream; Index: Integer): Boolean;
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/cpstrnew
begin
  Result := xsdTryParseBase64(xsdTryGetChildChars(node, name, nameSpace, Index), -1, Value);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdTryGetChildString(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: AnsiString; Index: Integer): Boolean;
=======
function xsdTryGetChildString(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Utf8String; Index: Integer): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryGetChildString(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Utf8String; Index: Integer): Boolean;
>>>>>>> graemeg/cpstrnew
begin
=======
begin
  Result := xsdTryParseBase64(xsdTryGetChildChars(node, name, nameSpace, Index), -1, Value);
end;

function xsdTryGetChildString(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Utf8String; Index: Integer): Boolean;
begin
>>>>>>> graemeg/cpstrnew
=======
function xsdTryGetChildString(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Utf8String; Index: Integer): Boolean;
begin
>>>>>>> origin/cpstrnew
  Result := xsdTryParseString(xsdTryGetChildChars(node, name, nameSpace, Index), -1, Value);
end;

function xsdTryGetChildBoolean(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Boolean; Index: Integer): Boolean;
begin
  Result := xsdTryParseBoolean(xsdTryGetChildChars(node, name, nameSpace, Index), -1, Value);
end;

function xsdTryGetChildDate(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Year, Month, Day: Longword; Timezone: PXsdTimezone; BC: PBoolean; Index: Integer): Boolean;
begin
  Result := xsdTryParseDate(xsdTryGetChildChars(node, name, nameSpace, Index), -1, Year, Month, Day, Timezone, BC);
end;

function xsdTryGetChildDate(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: TDateTime; Timezone: PXsdTimezone; Index: Integer): Boolean;
begin
  Result := xsdTryParseDate(xsdTryGetChildChars(node, name, nameSpace, Index), -1, Value, Timezone);
end;

function xsdTryGetChildTime(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Hour, Minute, Second, Milliseconds: Longword; Timezone: PXsdTimezone; Index: Integer): Boolean;
begin
  Result := xsdTryParseTime(xsdTryGetChildChars(node, name, nameSpace, Index), -1, Hour, Minute, Second, Milliseconds, Timezone);
end;

function xsdTryGetChildTime(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: TDateTime; Timezone: PXsdTimezone; Index: Integer): Boolean;
begin
  Result := xsdTryParseTime(xsdTryGetChildChars(node, name, nameSpace, Index), -1, Value, Timezone);
end;

function xsdTryGetChildDateTime(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Year, Month, Day, Hour, Minute, Second, Milliseconds: Longword; Timezone: PXsdTimezone; BC: PBoolean; Index: Integer): Boolean;
begin
  Result := xsdTryParseDateTime(xsdTryGetChildChars(node, name, nameSpace, Index), -1, Year, Month, Day, Hour, Minute, Second, Milliseconds, Timezone, BC);
end;

function xsdTryGetChildDateTime(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: TDateTime; Timezone: PXsdTimezone; Index: Integer): Boolean;
begin
  Result := xsdTryParseDateTime(xsdTryGetChildChars(node, name, nameSpace, Index), -1, Value, Timezone);
end;

function xsdTryGetChildDecimal(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Extended; Index: Integer): Boolean;
begin
  Result := xsdTryParseDecimal(xsdTryGetChildChars(node, name, nameSpace, Index), -1, Value);
end;

function xsdTryGetChildDouble(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Double; Index: Integer): Boolean;
begin
  Result := xsdTryParseDouble(xsdTryGetChildChars(node, name, nameSpace, Index), -1, Value);
end;

function xsdTryGetChildFloat(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Single; Index: Integer): Boolean;
begin
  Result := xsdTryParseFloat(xsdTryGetChildChars(node, name, nameSpace, Index), -1, Value);
end;

function xsdTryGetChildByte(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Shortint; Index: Integer): Boolean;
begin
  Result := xsdTryParseByte(xsdTryGetChildChars(node, name, nameSpace, Index), -1, Value);
end;

function xsdTryGetChildShort(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Smallint; Index: Integer): Boolean;
begin
  Result := xsdTryParseShort(xsdTryGetChildChars(node, name, nameSpace, Index), -1, Value);
end;

function xsdTryGetChildInt(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Longint; Index: Integer): Boolean;
begin
  Result := xsdTryParseInt(xsdTryGetChildChars(node, name, nameSpace, Index), -1, Value);
end;

function xsdTryGetChildLong(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Int64; Index: Integer): Boolean;
begin
  Result := xsdTryParseLong(xsdTryGetChildChars(node, name, nameSpace, Index), -1, Value);
end;

function xsdTryGetChildUnsignedByte(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Byte; Index: Integer): Boolean;
begin
  Result := xsdTryParseUnsignedByte(xsdTryGetChildChars(node, name, nameSpace, Index), -1, Value);
end;

function xsdTryGetChildUnsignedShort(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Word; Index: Integer): Boolean;
begin
  Result := xsdTryParseUnsignedShort(xsdTryGetChildChars(node, name, nameSpace, Index), -1, Value);
end;

function xsdTryGetChildUnsignedInt(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Longword; Index: Integer): Boolean;
begin
  Result := xsdTryParseUnsignedInt(xsdTryGetChildChars(node, name, nameSpace, Index), -1, Value);
end;

function xsdTryGetChildUnsignedLong(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: QWord; Index: Integer): Boolean;
begin
  Result := xsdTryParseUnsignedLong(xsdTryGetChildChars(node, name, nameSpace, Index), -1, Value);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdTryGetChildEnum(node: xmlNodePtr; name, nameSpace: xmlCharPtr; enum: array of AnsiString; out Value: Integer; Index: Integer): Boolean;
=======
function xsdTryGetChildEnum(node: xmlNodePtr; name, nameSpace: xmlCharPtr; enum: array of Utf8String; out Value: Integer; Index: Integer): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryGetChildEnum(node: xmlNodePtr; name, nameSpace: xmlCharPtr; enum: array of Utf8String; out Value: Integer; Index: Integer): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryGetChildEnum(node: xmlNodePtr; name, nameSpace: xmlCharPtr; enum: array of Utf8String; out Value: Integer; Index: Integer): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryGetChildEnum(node: xmlNodePtr; name, nameSpace: xmlCharPtr; enum: array of Utf8String; out Value: Integer; Index: Integer): Boolean;
>>>>>>> origin/cpstrnew
begin
  Result := xsdTryParseEnum(xsdTryGetChildChars(node, name, nameSpace, Index), -1, enum, Value);
end;

function xsdGetChild(node: xmlNodePtr; name, nameSpace: xmlCharPtr; Index: Integer): xmlNodePtr;
begin
  Result := xsdTryGetChild(node, name, nameSpace, Index);
  if not Assigned(Result) then
    raise XSDException.CreateNode(SChildNotFound, name, nameSpace);
end;

function xsdGetChildChars(node: xmlNodePtr; name, nameSpace: xmlCharPtr; Index: Integer): xmlCharPtr;
begin
  Result := xmlNodeGetContent(xsdGetChild(node, name, nameSpace, Index));
end;

procedure xsdGetChildBase64(node: xmlNodePtr; name, nameSpace: xmlCharPtr; const Value: TStream; Index: Integer);
<<<<<<< HEAD
<<<<<<< HEAD
begin
  xsdParseBase64(xsdGetChildChars(node, name, nameSpace, Index), -1, Value);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure xsdGetChildString(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: AnsiString; Index: Integer);
=======
procedure xsdGetChildString(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Utf8String; Index: Integer);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdGetChildString(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Utf8String; Index: Integer);
>>>>>>> graemeg/cpstrnew
begin
=======
begin
  xsdParseBase64(xsdGetChildChars(node, name, nameSpace, Index), -1, Value);
end;

procedure xsdGetChildString(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Utf8String; Index: Integer);
begin
>>>>>>> graemeg/cpstrnew
=======
begin
  xsdParseBase64(xsdGetChildChars(node, name, nameSpace, Index), -1, Value);
end;

procedure xsdGetChildString(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Utf8String; Index: Integer);
begin
>>>>>>> origin/cpstrnew
  xsdParseString(xsdGetChildChars(node, name, nameSpace, Index), -1, Value);
end;

procedure xsdGetChildBoolean(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Boolean; Index: Integer);
begin
  xsdParseBoolean(xsdGetChildChars(node, name, nameSpace, Index), -1, Value);
end;

procedure xsdGetChildDate(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Year, Month, Day: Longword; Timezone: PXsdTimezone; BC: PBoolean; Index: Integer);
begin
  xsdParseDate(xsdGetChildChars(node, name, nameSpace, Index), -1, Year, Month, Day, Timezone, BC);
end;

procedure xsdGetChildDate(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: TDateTime; Timezone: PXsdTimezone; Index: Integer);
begin
  xsdParseDate(xsdGetChildChars(node, name, nameSpace, Index), -1, Value, Timezone);
end;

procedure xsdGetChildTime(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Hour, Minute, Second, Milliseconds: Longword; Timezone: PXsdTimezone; Index: Integer);
begin
  xsdParseTime(xsdGetChildChars(node, name, nameSpace, Index), -1, Hour, Minute, Second, Milliseconds, Timezone);
end;

procedure xsdGetChildTime(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: TDateTime; Timezone: PXsdTimezone; Index: Integer);
begin
  xsdParseTime(xsdGetChildChars(node, name, nameSpace, Index), -1, Value, Timezone);
end;

procedure xsdGetChildDateTime(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Year, Month, Day, Hour, Minute, Second, Milliseconds: Longword; Timezone: PXsdTimezone; BC: PBoolean; Index: Integer);
begin
  xsdParseDateTime(xsdGetChildChars(node, name, nameSpace, Index), -1, Year, Month, Day, Hour, Minute, Second, Milliseconds, Timezone, BC);
end;

procedure xsdGetChildDateTime(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: TDateTime; Timezone: PXsdTimezone; Index: Integer);
begin
  xsdParseDateTime(xsdGetChildChars(node, name, nameSpace, Index), -1, Value, Timezone);
end;

procedure xsdGetChildDecimal(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Extended; Index: Integer);
begin
  xsdParseDecimal(xsdGetChildChars(node, name, nameSpace, Index), -1, Value);
end;

procedure xsdGetChildDouble(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Double; Index: Integer);
begin
  xsdParseDouble(xsdGetChildChars(node, name, nameSpace, Index), -1, Value);
end;

procedure xsdGetChildFloat(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Single; Index: Integer);
begin
  xsdParseFloat(xsdGetChildChars(node, name, nameSpace, Index), -1, Value);
end;

procedure xsdGetChildByte(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Shortint; Index: Integer);
begin
  xsdParseByte(xsdGetChildChars(node, name, nameSpace, Index), -1, Value);
end;

procedure xsdGetChildShort(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Smallint; Index: Integer);
begin
  xsdParseShort(xsdGetChildChars(node, name, nameSpace, Index), -1, Value);
end;

procedure xsdGetChildInt(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Longint; Index: Integer);
begin
  xsdParseInt(xsdGetChildChars(node, name, nameSpace, Index), -1, Value);
end;

procedure xsdGetChildLong(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Int64; Index: Integer);
begin
  xsdParseLong(xsdGetChildChars(node, name, nameSpace, Index), -1, Value);
end;

procedure xsdGetChildUnsignedByte(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Byte; Index: Integer);
begin
  xsdParseUnsignedByte(xsdGetChildChars(node, name, nameSpace, Index), -1, Value);
end;

procedure xsdGetChildUnsignedShort(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Word; Index: Integer);
begin
  xsdParseUnsignedShort(xsdGetChildChars(node, name, nameSpace, Index), -1, Value);
end;

procedure xsdGetChildUnsignedInt(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Longword; Index: Integer);
begin
  xsdParseUnsignedInt(xsdGetChildChars(node, name, nameSpace, Index), -1, Value);
end;

procedure xsdGetChildUnsignedLong(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: QWord; Index: Integer);
begin
  xsdParseUnsignedLong(xsdGetChildChars(node, name, nameSpace, Index), -1, Value);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure xsdGetChildEnum(node: xmlNodePtr; name, nameSpace: xmlCharPtr; enum: array of AnsiString; out Value: Integer; Index: Integer);
=======
procedure xsdGetChildEnum(node: xmlNodePtr; name, nameSpace: xmlCharPtr; enum: array of Utf8String; out Value: Integer; Index: Integer);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdGetChildEnum(node: xmlNodePtr; name, nameSpace: xmlCharPtr; enum: array of Utf8String; out Value: Integer; Index: Integer);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdGetChildEnum(node: xmlNodePtr; name, nameSpace: xmlCharPtr; enum: array of Utf8String; out Value: Integer; Index: Integer);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdGetChildEnum(node: xmlNodePtr; name, nameSpace: xmlCharPtr; enum: array of Utf8String; out Value: Integer; Index: Integer);
>>>>>>> origin/cpstrnew
begin
  xsdParseEnum(xsdGetChildChars(node, name, nameSpace, Index), -1, enum, Value);
end;

function xsdTryNext(var node: xmlNodePtr; name, nameSpace: xmlCharPtr): xmlNodePtr;
begin
  while Assigned(node) and (node^._type = XML_TEXT_NODE) do
    node := node^.next;

  if xsdTestNode(node, name, nameSpace) then
  begin
    Result := node;
    node := node^.next;
  end else
    Result := nil;
end;

function xsdTryNext(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out last: xmlNodePtr): Boolean;
begin
  last := xsdTryNext(node, name, nameSpace);
  Result := Assigned(last);
end;

function xsdTryNextChars(var node: xmlNodePtr; name, nameSpace: xmlCharPtr): xmlCharPtr;
begin
  Result := xmlNodeGetContent(xsdTryNext(node, name, nameSpace));
end;

function xsdTryNextBase64(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; const Value: TStream): Boolean;
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/cpstrnew
begin
  Result := xsdTryParseBase64(xsdTryNextChars(node, name, nameSpace), -1, Value);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdTryNextString(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: AnsiString): Boolean;
=======
function xsdTryNextString(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Utf8String): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryNextString(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Utf8String): Boolean;
>>>>>>> graemeg/cpstrnew
begin
=======
begin
  Result := xsdTryParseBase64(xsdTryNextChars(node, name, nameSpace), -1, Value);
end;

function xsdTryNextString(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Utf8String): Boolean;
begin
>>>>>>> graemeg/cpstrnew
=======
function xsdTryNextString(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Utf8String): Boolean;
begin
>>>>>>> origin/cpstrnew
  Result := xsdTryParseString(xsdTryNextChars(node, name, nameSpace), -1, Value);
end;

function xsdTryNextBoolean(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Boolean): Boolean;
begin
  Result := xsdTryParseBoolean(xsdTryNextChars(node, name, nameSpace), -1, Value);
end;

function xsdTryNextDate(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Year, Month, Day: Longword; Timezone: PXsdTimezone; BC: PBoolean): Boolean;
begin
  Result := xsdTryParseDate(xsdTryNextChars(node, name, nameSpace), -1, Year, Month, Day, Timezone, BC);
end;

function xsdTryNextDate(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: TDateTime; Timezone: PXsdTimezone): Boolean;
begin
  Result := xsdTryParseDate(xsdTryNextChars(node, name, nameSpace), -1, Value, Timezone);
end;

function xsdTryNextTime(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Hour, Minute, Second, Milliseconds: Longword; Timezone: PXsdTimezone): Boolean;
begin
  Result := xsdTryParseTime(xsdTryNextChars(node, name, nameSpace), -1, Hour, Minute, Second, Milliseconds, Timezone);
end;

function xsdTryNextTime(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: TDateTime; Timezone: PXsdTimezone): Boolean;
begin
  Result := xsdTryParseTime(xsdTryNextChars(node, name, nameSpace), -1, Value, Timezone);
end;

function xsdTryNextDateTime(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Year, Month, Day, Hour, Minute, Second, Milliseconds: Longword; Timezone: PXsdTimezone; BC: PBoolean): Boolean;
begin
  Result := xsdTryParseDateTime(xsdTryNextChars(node, name, nameSpace), -1, Year, Month, Day, Hour, Minute, Second, Milliseconds, Timezone, BC);
end;

function xsdTryNextDateTime(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: TDateTime; Timezone: PXsdTimezone): Boolean;
begin
  Result := xsdTryParseDateTime(xsdTryNextChars(node, name, nameSpace), -1, Value, Timezone);
end;

function xsdTryNextDecimal(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Extended): Boolean;
begin
  Result := xsdTryParseDecimal(xsdTryNextChars(node, name, nameSpace), -1, Value);
end;

function xsdTryNextDouble(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Double): Boolean;
begin
  Result := xsdTryParseDouble(xsdTryNextChars(node, name, nameSpace), -1, Value);
end;

function xsdTryNextFloat(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Single): Boolean;
begin
  Result := xsdTryParseFloat(xsdTryNextChars(node, name, nameSpace), -1, Value);
end;

function xsdTryNextByte(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Shortint): Boolean;
begin
  Result := xsdTryParseByte(xsdTryNextChars(node, name, nameSpace), -1, Value);
end;

function xsdTryNextShort(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Smallint): Boolean;
begin
  Result := xsdTryParseShort(xsdTryNextChars(node, name, nameSpace), -1, Value);
end;

function xsdTryNextInt(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Longint): Boolean;
begin
  Result := xsdTryParseInt(xsdTryNextChars(node, name, nameSpace), -1, Value);
end;

function xsdTryNextLong(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Int64): Boolean;
begin
  Result := xsdTryParseLong(xsdTryNextChars(node, name, nameSpace), -1, Value);
end;

function xsdTryNextUnsignedByte(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Byte): Boolean;
begin
  Result := xsdTryParseUnsignedByte(xsdTryNextChars(node, name, nameSpace), -1, Value);
end;

function xsdTryNextUnsignedShort(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Word): Boolean;
begin
  Result := xsdTryParseUnsignedShort(xsdTryNextChars(node, name, nameSpace), -1, Value);
end;

function xsdTryNextUnsignedInt(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Longword): Boolean;
begin
  Result := xsdTryParseUnsignedInt(xsdTryNextChars(node, name, nameSpace), -1, Value);
end;

function xsdTryNextUnsignedLong(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: QWord): Boolean;
begin
  Result := xsdTryParseUnsignedLong(xsdTryNextChars(node, name, nameSpace), -1, Value);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdTryNextEnum(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; enum: array of AnsiString; out Value: Integer): Boolean;
=======
function xsdTryNextEnum(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; enum: array of Utf8String; out Value: Integer): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryNextEnum(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; enum: array of Utf8String; out Value: Integer): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryNextEnum(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; enum: array of Utf8String; out Value: Integer): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryNextEnum(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; enum: array of Utf8String; out Value: Integer): Boolean;
>>>>>>> origin/cpstrnew
begin
  Result := xsdTryParseEnum(xsdTryNextChars(node, name, nameSpace), -1, enum, Value);
end;

function xsdNext(var node: xmlNodePtr; name, nameSpace: xmlCharPtr): xmlNodePtr;
begin
  Result := xsdTryNext(node, name, nameSpace);
  if not Assigned(Result) then
    raise XSDException.CreateNode(SChildNotFound, name, nameSpace);
end;

function xsdNextChars(var node: xmlNodePtr; name, nameSpace: xmlCharPtr): xmlCharPtr;
begin
  Result := xmlNodeGetContent(xsdNext(node, name, nameSpace));
end;

procedure xsdNextBase64(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; const Value: TStream);
begin
  xsdParseBase64(xsdNextChars(node, name, nameSpace), -1, Value);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure xsdNextString(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: AnsiString);
=======
procedure xsdNextString(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Utf8String);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdNextString(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Utf8String);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdNextString(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Utf8String);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdNextString(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Utf8String);
>>>>>>> origin/cpstrnew
begin
  xsdParseString(xsdNextChars(node, name, nameSpace), -1, Value);
end;

procedure xsdNextBoolean(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Boolean);
begin
  xsdParseBoolean(xsdNextChars(node, name, nameSpace), -1, Value);
end;

procedure xsdNextDate(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Year, Month, Day: Longword; Timezone: PXsdTimezone; BC: PBoolean);
begin
  xsdParseDate(xsdNextChars(node, name, nameSpace), -1, Year, Month, Day, Timezone, BC);
end;

procedure xsdNextDate(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: TDateTime; Timezone: PXsdTimezone);
begin
  xsdParseDate(xsdNextChars(node, name, nameSpace), -1, Value, Timezone);
end;

procedure xsdNextTime(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Hour, Minute, Second, Milliseconds: Longword; Timezone: PXsdTimezone);
begin
  xsdParseTime(xsdNextChars(node, name, nameSpace), -1, Hour, Minute, Second, Milliseconds, Timezone);
end;

procedure xsdNextTime(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: TDateTime; Timezone: PXsdTimezone);
begin
  xsdParseTime(xsdNextChars(node, name, nameSpace), -1, Value, Timezone);
end;

procedure xsdNextDateTime(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Year, Month, Day, Hour, Minute, Second, Milliseconds: Longword; Timezone: PXsdTimezone; BC: PBoolean);
begin
  xsdParseDateTime(xsdNextChars(node, name, nameSpace), -1, Year, Month, Day, Hour, Minute, Second, Milliseconds, Timezone, BC);
end;

procedure xsdNextDateTime(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: TDateTime; Timezone: PXsdTimezone);
begin
  xsdParseDateTime(xsdNextChars(node, name, nameSpace), -1, Value, Timezone);
end;

procedure xsdNextDecimal(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Extended);
begin
  xsdParseDecimal(xsdNextChars(node, name, nameSpace), -1, Value);
end;

procedure xsdNextDouble(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Double);
begin
  xsdParseDouble(xsdNextChars(node, name, nameSpace), -1, Value);
end;

procedure xsdNextFloat(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Single);
begin
  xsdParseFloat(xsdNextChars(node, name, nameSpace), -1, Value);
end;

procedure xsdNextByte(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Shortint);
begin
  xsdParseByte(xsdNextChars(node, name, nameSpace), -1, Value);
end;

procedure xsdNextShort(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Smallint);
begin
  xsdParseShort(xsdNextChars(node, name, nameSpace), -1, Value);
end;

procedure xsdNextInt(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Longint);
begin
  xsdParseInt(xsdNextChars(node, name, nameSpace), -1, Value);
end;

procedure xsdNextLong(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Int64);
begin
  xsdParseLong(xsdNextChars(node, name, nameSpace), -1, Value);
end;

procedure xsdNextUnsignedByte(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Byte);
begin
  xsdParseUnsignedByte(xsdNextChars(node, name, nameSpace), -1, Value);
end;

procedure xsdNextUnsignedShort(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Word);
begin
  xsdParseUnsignedShort(xsdNextChars(node, name, nameSpace), -1, Value);
end;

procedure xsdNextUnsignedInt(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Longword);
begin
  xsdParseUnsignedInt(xsdNextChars(node, name, nameSpace), -1, Value);
end;

procedure xsdNextUnsignedLong(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: QWord);
begin
  xsdParseUnsignedLong(xsdNextChars(node, name, nameSpace), -1, Value);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure xsdNextEnum(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; enum: array of AnsiString; out Value: Integer);
=======
procedure xsdNextEnum(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; enum: array of Utf8String; out Value: Integer);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdNextEnum(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; enum: array of Utf8String; out Value: Integer);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdNextEnum(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; enum: array of Utf8String; out Value: Integer);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdNextEnum(var node: xmlNodePtr; name, nameSpace: xmlCharPtr; enum: array of Utf8String; out Value: Integer);
>>>>>>> origin/cpstrnew
begin
  xsdParseEnum(xsdNextChars(node, name, nameSpace), -1, enum, Value);
end;

function xsdTestPropNs(attr: xmlAttrPtr; nameSpace: xmlCharPtr): Boolean;
begin
  Result :=
     (nameSpace = NS_IGNORE) or
    ((nameSpace = NS_EXCLUDE) and (attr^.ns = nil)) or
    ((nameSpace <> NS_EXCLUDE) and (nameSpace <> NS_IGNORE) and (attr^.ns <> nil) and (xmlStrEqual(nameSpace, attr^.ns^.href) <> 0))
end;

function xsdTestProp(attr: xmlAttrPtr; name, nameSpace: xmlCharPtr): Boolean;
begin
  Result := (attr <> nil) and (xmlStrEqual(name, attr^.name) <> 0) and xsdTestPropNs(attr, nameSpace);
end;

function xsdTryGetProp(node: xmlNodePtr; name, nameSpace: xmlCharPtr): xmlAttrPtr;
begin
  if Assigned(node) then
  begin
    Result := node^.properties;
    while Assigned(Result) do
    begin
      if xsdTestProp(Result, name, nameSpace) then
        Exit;
      Result := Result^.next;
    end;
  end else
    Result := nil;
end;

function xsdTryGetPropChars(node: xmlNodePtr; name, nameSpace: xmlCharPtr): xmlCharPtr;
var
  Prop: xmlAttrPtr;
begin
  Prop := xsdTryGetProp(node, name, nameSpace);
  if Assigned(Prop) then
    Result := xmlNodeGetContent(Prop^.children)
  else
    Result := nil;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdTryGetPropString(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: AnsiString): Boolean;
=======
function xsdTryGetPropString(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Utf8String): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryGetPropString(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Utf8String): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryGetPropString(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Utf8String): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryGetPropString(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Utf8String): Boolean;
>>>>>>> origin/cpstrnew
begin
  Result := xsdTryParseString(xsdTryGetPropChars(node, name, nameSpace), -1, Value);
end;

function xsdTryGetPropBoolean(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Boolean): Boolean;
begin
   Result := xsdTryParseBoolean(xsdTryGetPropChars(node, name, nameSpace), -1, Value);
end;

function xsdTryGetPropDate(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Year, Month, Day: Longword; Timezone: PXsdTimezone; BC: PBoolean): Boolean;
begin
  Result := xsdTryParseDate(xsdTryGetPropChars(node, name, nameSpace), -1, Year, Month, Day, Timezone, BC);
end;

function xsdTryGetPropDate(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: TDateTime; Timezone: PXsdTimezone): Boolean;
begin
  Result := xsdTryParseDate(xsdTryGetPropChars(node, name, nameSpace), -1, Value, Timezone);
end;

function xsdTryGetPropTime(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Hour, Minute, Second, Milliseconds: Longword; Timezone: PXsdTimezone): Boolean;
begin
  Result := xsdTryParseTime(xsdTryGetPropChars(node, name, nameSpace), -1, Hour, Minute, Second, Milliseconds, Timezone);
end;

function xsdTryGetPropTime(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: TDateTime; Timezone: PXsdTimezone): Boolean;
begin
  Result := xsdTryParseTime(xsdTryGetPropChars(node, name, nameSpace), -1, Value, Timezone);
end;

function xsdTryGetPropDateTime(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Year, Month, Day, Hour, Minute, Second, Milliseconds: Longword; Timezone: PXsdTimezone; BC: PBoolean): Boolean;
begin
  Result := xsdTryParseDateTime(xsdTryGetPropChars(node, name, nameSpace), -1, Year, Month, Day, Hour, Minute, Second, Milliseconds, Timezone, BC);
end;

function xsdTryGetPropDateTime(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: TDateTime; Timezone: PXsdTimezone): Boolean;
begin
  Result := xsdTryParseDateTime(xsdTryGetPropChars(node, name, nameSpace), -1, Value, Timezone);
end;

function xsdTryGetPropDecimal(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Extended): Boolean;
begin
  Result := xsdTryParseDecimal(xsdTryGetPropChars(node, name, nameSpace), -1, Value);
end;

function xsdTryGetPropDouble(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Double): Boolean;
begin
  Result := xsdTryParseDouble(xsdTryGetPropChars(node, name, nameSpace), -1, Value);
end;

function xsdTryGetPropFloat(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Single): Boolean;
begin
  Result := xsdTryParseFloat(xsdTryGetPropChars(node, name, nameSpace), -1, Value);
end;

function xsdTryGetPropByte(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Shortint): Boolean;
begin
  Result := xsdTryParseByte(xsdTryGetPropChars(node, name, nameSpace), -1, Value);
end;

function xsdTryGetPropShort(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Smallint): Boolean;
begin
  Result := xsdTryParseShort(xsdTryGetPropChars(node, name, nameSpace), -1, Value);
end;

function xsdTryGetPropInt(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Longint): Boolean;
begin
  Result := xsdTryParseInt(xsdTryGetPropChars(node, name, nameSpace), -1, Value);
end;

function xsdTryGetPropLong(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Int64): Boolean;
begin
  Result := xsdTryParseLong(xsdTryGetPropChars(node, name, nameSpace), -1, Value);
end;

function xsdTryGetPropUnsignedByte(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Byte): Boolean;
begin
  Result := xsdTryParseUnsignedByte(xsdTryGetPropChars(node, name, nameSpace), -1, Value);
end;

function xsdTryGetPropUnsignedShort(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Word): Boolean;
begin
  Result := xsdTryParseUnsignedShort(xsdTryGetPropChars(node, name, nameSpace), -1, Value);
end;

function xsdTryGetPropUnsignedInt(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Longword): Boolean;
begin
  Result := xsdTryParseUnsignedInt(xsdTryGetPropChars(node, name, nameSpace), -1, Value);
end;

function xsdTryGetPropUnsignedLong(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: QWord): Boolean;
begin
  Result := xsdTryParseUnsignedLong(xsdTryGetPropChars(node, name, nameSpace), -1, Value);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdTryGetPropEnum(node: xmlNodePtr; name, nameSpace: xmlCharPtr; enum: array of AnsiString; out Value: Integer): Boolean;
=======
function xsdTryGetPropEnum(node: xmlNodePtr; name, nameSpace: xmlCharPtr; enum: array of Utf8String; out Value: Integer): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryGetPropEnum(node: xmlNodePtr; name, nameSpace: xmlCharPtr; enum: array of Utf8String; out Value: Integer): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryGetPropEnum(node: xmlNodePtr; name, nameSpace: xmlCharPtr; enum: array of Utf8String; out Value: Integer): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryGetPropEnum(node: xmlNodePtr; name, nameSpace: xmlCharPtr; enum: array of Utf8String; out Value: Integer): Boolean;
>>>>>>> origin/cpstrnew
begin
  Result := xsdTryParseEnum(xsdTryGetPropChars(node, name, nameSpace), -1, enum, Value);
end;

function xsdGetProp(node: xmlNodePtr; name, nameSpace: xmlCharPtr): xmlAttrPtr;
begin
  Result := xsdTryGetProp(node, name, nameSpace);
  if not Assigned(Result) then
    raise XSDException.CreateNode(SPropNotFound, name, nameSpace);
end;

function xsdGetPropChars(node: xmlNodePtr; name, nameSpace: xmlCharPtr): xmlCharPtr;
begin
  Result := xmlNodeGetContent(xsdGetProp(node, name, nameSpace)^.children);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure xsdGetPropString(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: AnsiString);
=======
procedure xsdGetPropString(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Utf8String);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdGetPropString(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Utf8String);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdGetPropString(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Utf8String);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdGetPropString(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Utf8String);
>>>>>>> origin/cpstrnew
begin
  xsdParseString(xsdGetPropChars(node, name, nameSpace), -1, Value);
end;

procedure xsdGetPropBoolean(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Boolean);
begin
   xsdParseBoolean(xsdGetPropChars(node, name, nameSpace), -1, Value);
end;

procedure xsdGetPropDate(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Year, Month, Day: Longword; Timezone: PXsdTimezone; BC: PBoolean);
begin
  xsdParseDate(xsdGetPropChars(node, name, nameSpace), -1, Year, Month, Day, Timezone, BC);
end;

procedure xsdGetPropDate(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: TDateTime; Timezone: PXsdTimezone);
begin
  xsdParseDate(xsdGetPropChars(node, name, nameSpace), -1, Value, Timezone);
end;

procedure xsdGetPropTime(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Hour, Minute, Second, Milliseconds: Longword; Timezone: PXsdTimezone);
begin
  xsdParseTime(xsdGetPropChars(node, name, nameSpace), -1, Hour, Minute, Second, Milliseconds, Timezone);
end;

procedure xsdGetPropTime(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: TDateTime; Timezone: PXsdTimezone);
begin
  xsdParseTime(xsdGetPropChars(node, name, nameSpace), -1, Value, Timezone);
end;

procedure xsdGetPropDateTime(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Year, Month, Day, Hour, Minute, Second, Milliseconds: Longword; Timezone: PXsdTimezone; BC: PBoolean);
begin
  xsdParseDateTime(xsdGetPropChars(node, name, nameSpace), -1, Year, Month, Day, Hour, Minute, Second, Milliseconds, Timezone, BC);
end;

procedure xsdGetPropDateTime(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: TDateTime; Timezone: PXsdTimezone);
begin
  xsdParseDateTime(xsdGetPropChars(node, name, nameSpace), -1, Value, Timezone);
end;

procedure xsdGetPropDecimal(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Extended);
begin
  xsdParseDecimal(xsdGetPropChars(node, name, nameSpace), -1, Value);
end;

procedure xsdGetPropDouble(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Double);
begin
  xsdParseDouble(xsdGetPropChars(node, name, nameSpace), -1, Value);
end;

procedure xsdGetPropFloat(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Single);
begin
  xsdParseFloat(xsdGetPropChars(node, name, nameSpace), -1, Value);
end;

procedure xsdGetPropByte(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Shortint);
begin
  xsdParseByte(xsdGetPropChars(node, name, nameSpace), -1, Value);
end;

procedure xsdGetPropShort(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Smallint);
begin
  xsdParseShort(xsdGetPropChars(node, name, nameSpace), -1, Value);
end;

procedure xsdGetPropInt(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Longint);
begin
  xsdParseInt(xsdGetPropChars(node, name, nameSpace), -1, Value);
end;

procedure xsdGetPropLong(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Int64);
begin
  xsdParseLong(xsdGetPropChars(node, name, nameSpace), -1, Value);
end;

procedure xsdGetPropUnsignedByte(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Byte);
begin
  xsdParseUnsignedByte(xsdGetPropChars(node, name, nameSpace), -1, Value);
end;

procedure xsdGetPropUnsignedShort(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Word);
begin
  xsdParseUnsignedShort(xsdGetPropChars(node, name, nameSpace), -1, Value);
end;

procedure xsdGetPropUnsignedInt(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: Longword);
begin
  xsdParseUnsignedInt(xsdGetPropChars(node, name, nameSpace), -1, Value);
end;

procedure xsdGetPropUnsignedLong(node: xmlNodePtr; name, nameSpace: xmlCharPtr; out Value: QWord);
begin
  xsdParseUnsignedLong(xsdGetPropChars(node, name, nameSpace), -1, Value);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure xsdGetPropEnum(node: xmlNodePtr; name, nameSpace: xmlCharPtr; enum: array of AnsiString; out Value: Integer);
=======
procedure xsdGetPropEnum(node: xmlNodePtr; name, nameSpace: xmlCharPtr; enum: array of Utf8String; out Value: Integer);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdGetPropEnum(node: xmlNodePtr; name, nameSpace: xmlCharPtr; enum: array of Utf8String; out Value: Integer);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdGetPropEnum(node: xmlNodePtr; name, nameSpace: xmlCharPtr; enum: array of Utf8String; out Value: Integer);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdGetPropEnum(node: xmlNodePtr; name, nameSpace: xmlCharPtr; enum: array of Utf8String; out Value: Integer);
>>>>>>> origin/cpstrnew
begin
  xsdParseEnum(xsdGetPropChars(node, name, nameSpace), -1, enum, Value);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdRemoveBlanks(content: xmlCharPtr; out cleaned: AnsiString): boolean;
=======
function xsdRemoveBlanks(content: xmlCharPtr; out cleaned: Utf8String): boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdRemoveBlanks(content: xmlCharPtr; out cleaned: Utf8String): boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdRemoveBlanks(content: xmlCharPtr; out cleaned: Utf8String): boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdRemoveBlanks(content: xmlCharPtr; out cleaned: Utf8String): boolean;
>>>>>>> origin/cpstrnew
var
  Space: Boolean;
  len: Integer;
begin
  cleaned := '';

  if Assigned(content) then
  begin
    Space := True;

    while content^ <> #0 do
    begin
      if xmlIsBlank(cuint(content^)) then
      begin
        if not Space then
          cleaned := cleaned + ' ';
        Space := True;
      end else begin
        cleaned := cleaned + content^;
        Space := False;
      end;

      Inc(content);
    end;
  end;

  len := Length(cleaned);
  if len > 0 then
  begin
    if cleaned[len] = ' ' then
      SetLength(cleaned, len-1);
    Result := True;
  end else
    Result := False;
end;

end.
