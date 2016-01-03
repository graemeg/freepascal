{
  Parser functions to parser xml xsd types
  See: http://books.xmlschemata.org/relaxng/relax-CHP-19.html

  Copyright (C) 2011 by Ivo Steinmann
}

unit xmlxsdparser;

{$mode objfpc}
{$H+}

interface

uses
  {$IFDEF MSWINDOWS}windows,{$ENDIF}
  {$IFDEF UNIX}unixutil,{$ENDIF}
  sysutils,
  dateutils,
  math,
  Classes;

resourcestring
  SXsdParserError = 'parsing "%s" as "%s" failed';

type
{$IFDEF MSWINDOWS}
  PBoolean = System.PBoolean;
  // PBoolean is redefined by windows unit, so redefine it here again!
{$ENDIF}

  TXsdTimezoneType = (
    tzUnknown,
    tzLocal,
    tzUtc
  );

  PXsdTimezone = ^TXsdTimezone;
  TXsdTimezone = record
    Kind   : TXsdTimezoneType;
    Hour   : Longint;  // +/- [00..23]
    Minute : Longword; // [00..59]
  end;

const
  TIMEZONE_UTC: TXsdTimezone = (Kind:tzUTC;Hour:0;Minute:0);


{ Format functions }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdFormatBase64(Value: TStream): AnsiString;
function xsdFormatBoolean(Value: Boolean; UseWords: Boolean = False): AnsiString;
function xsdFormatDate(Year, Month, Day: Longword; BC: Boolean; Timezone: PXsdTimezone = nil): AnsiString;
function xsdFormatDate(Value: TDateTime; Timezone: PXsdTimezone = nil): AnsiString;
function xsdFormatTime(Hour, Minute, Second, Milliseconds: Longword; Timezone: PXsdTimezone = nil): AnsiString;
function xsdFormatTime(Value: TDateTime; Timezone: PXsdTimezone = nil): AnsiString;
function xsdFormatDateTime(Year, Month, Day, Hour, Minute, Second, Milliseconds: Longword; BC: Boolean; Timezone: PXsdTimezone = nil): AnsiString;
function xsdFormatDateTime(Value: TDateTime; Timezone: PXsdTimezone): AnsiString;
function xsdFormatDecimal(Value: Extended; Precision: Integer = 4; Digits: Integer = 1): AnsiString;
function xsdFormatDouble(Value: Double): AnsiString;
function xsdFormatFloat(Value: Single): AnsiString;
function xsdFormatByte(Value: Shortint): AnsiString;
function xsdFormatShort(Value: Smallint): AnsiString;
function xsdFormatInt(Value: Longint): AnsiString;
function xsdFormatLong(Value: Int64): AnsiString;
function xsdFormatUnsignedByte(Value: Byte): AnsiString;
function xsdFormatUnsignedShort(Value: Word): AnsiString;
function xsdFormatUnsignedInt(Value: Longword): AnsiString;
function xsdFormatUnsignedLong(Value: QWord): AnsiString;
function xsdFormatEnum(enum: array of AnsiString; Value: Integer): AnsiString;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function xsdFormatBase64(Value: TStream): Utf8String;
function xsdFormatBoolean(Value: Boolean; UseWords: Boolean = False): Utf8String;
function xsdFormatDate(Year, Month, Day: Longword; BC: Boolean; Timezone: PXsdTimezone = nil): Utf8String;
function xsdFormatDate(Value: TDateTime; Timezone: PXsdTimezone = nil): Utf8String;
function xsdFormatTime(Hour, Minute, Second, Milliseconds: Longword; Timezone: PXsdTimezone = nil): Utf8String;
function xsdFormatTime(Value: TDateTime; Timezone: PXsdTimezone = nil): Utf8String;
function xsdFormatDateTime(Year, Month, Day, Hour, Minute, Second, Milliseconds: Longword; BC: Boolean; Timezone: PXsdTimezone = nil): Utf8String;
function xsdFormatDateTime(Value: TDateTime; Timezone: PXsdTimezone): Utf8String;
function xsdFormatDecimal(Value: Extended; Precision: Integer = 4; Digits: Integer = 1): Utf8String;
function xsdFormatDouble(Value: Double): Utf8String;
function xsdFormatFloat(Value: Single): Utf8String;
function xsdFormatByte(Value: Shortint): Utf8String;
function xsdFormatShort(Value: Smallint): Utf8String;
function xsdFormatInt(Value: Longint): Utf8String;
function xsdFormatLong(Value: Int64): Utf8String;
function xsdFormatUnsignedByte(Value: Byte): Utf8String;
function xsdFormatUnsignedShort(Value: Word): Utf8String;
function xsdFormatUnsignedInt(Value: Longword): Utf8String;
function xsdFormatUnsignedLong(Value: QWord): Utf8String;
function xsdFormatEnum(enum: array of Utf8String; Value: Integer): Utf8String;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

{ DateTime functions }
function xsdNowUTC: TDateTime;
function xsdGetLocalTimezone: TXsdTimezone;
function xsdTimezoneUtcOffsetMinutes(const Timezone: TXsdTimezone): Longint;
function xsdDateTimeToUTC(const DateTime: TDateTime; const Current: TXsdTimezone): TDateTime;
function xsdDateTimeConvert(const DateTime: TDateTime; const Current, Target: TXsdTimezone): TDateTime;

{ Parse functions }
function xsdTryParseBase64(Chars: PChar; Len: Integer; const Value: TStream): Boolean;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdTryParseString(Chars: PChar; Len: Integer; out Value: AnsiString): Boolean;
function xsdTryParseStringLower(Chars: PChar; Len: Integer; out Value: AnsiString): Boolean;
=======
function xsdTryParseString(Chars: PChar; Len: Integer; out Value: Utf8String): Boolean;
function xsdTryParseStringLower(Chars: PChar; Len: Integer; out Value: Utf8String): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseString(Chars: PChar; Len: Integer; out Value: Utf8String): Boolean;
function xsdTryParseStringLower(Chars: PChar; Len: Integer; out Value: Utf8String): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseString(Chars: PChar; Len: Integer; out Value: Utf8String): Boolean;
function xsdTryParseStringLower(Chars: PChar; Len: Integer; out Value: Utf8String): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseString(Chars: PChar; Len: Integer; out Value: Utf8String): Boolean;
function xsdTryParseStringLower(Chars: PChar; Len: Integer; out Value: Utf8String): Boolean;
>>>>>>> origin/cpstrnew
function xsdTryParseBoolean(Chars: PChar; Len: Integer; out Value: Boolean): Boolean;
function xsdTryParseDate(Chars: PChar; Len: Integer; out Year, Month, Day: Longword; Timezone: PXsdTimezone = nil; BC: PBoolean = nil): Boolean;
function xsdTryParseDate(Chars: PChar; Len: Integer; out Value: TDateTime; Timezone: PXsdTimezone = nil): Boolean;
function xsdTryParseTime(Chars: PChar; Len: Integer; out Hour, Minute, Second, Milliseconds: Longword; Timezone: PXsdTimezone = nil): Boolean;
function xsdTryParseTime(Chars: PChar; Len: Integer; out Value: TDateTime; Timezone: PXsdTimezone = nil): Boolean;
function xsdTryParseDateTime(Chars: PChar; Len: Integer; out Year, Month, Day, Hour, Minute, Second, Milliseconds: Longword; Timezone: PXsdTimezone = nil; BC: PBoolean = nil): Boolean;
function xsdTryParseDateTime(Chars: PChar; Len: Integer; out Value: TDateTime; Timezone: PXsdTimezone = nil): Boolean;
function xsdTryParseDecimal(Chars: PChar; Len: Integer; out Value: Extended): Boolean;
function xsdTryParseDouble(Chars: PChar; Len: Integer; out Value: Double): Boolean;
function xsdTryParseFloat(Chars: PChar; Len: Integer; out Value: Single): Boolean;
function xsdTryParseInteger(Chars: PChar; Len: Integer; out Value: Int64): Boolean;
function xsdTryParseNonNegativeInteger(Chars: PChar; Len: Integer; out Value: QWord): Boolean;
function xsdTryParseNonPositiveInteger(Chars: PChar; Len: Integer; out Value: Int64): Boolean;
function xsdTryParseNegativeInteger(Chars: PChar; Len: Integer; out Value: Int64): Boolean;
function xsdTryParsePositiveInteger(Chars: PChar; Len: Integer; out Value: QWord): Boolean;
function xsdTryParseByte(Chars: PChar; Len: Integer; out Value: Shortint): Boolean;
function xsdTryParseShort(Chars: PChar; Len: Integer; out Value: Smallint): Boolean;
function xsdTryParseInt(Chars: PChar; Len: Integer; out Value: Longint): Boolean;
function xsdTryParseLong(Chars: PChar; Len: Integer; out Value: Int64): Boolean;
function xsdTryParseUnsignedByte(Chars: PChar; Len: Integer; out Value: Byte): Boolean;
function xsdTryParseUnsignedShort(Chars: PChar; Len: Integer; out Value: Word): Boolean;
function xsdTryParseUnsignedInt(Chars: PChar; Len: Integer; out Value: Longword): Boolean;
function xsdTryParseUnsignedLong(Chars: PChar; Len: Integer; out Value: QWord): Boolean;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdTryParseEnum(Chars: PChar; Len: Integer; enum: array of AnsiString; out Value: Integer): Boolean;

function xsdParseStringDef(Chars: PChar; Len: Integer; Default: AnsiString): AnsiString;
function xsdParseStringLowerDef(Chars: PChar; Len: Integer; Default: AnsiString): AnsiString;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function xsdTryParseEnum(Chars: PChar; Len: Integer; enum: array of Utf8String; out Value: Integer): Boolean;

function xsdParseStringDef(Chars: PChar; Len: Integer; Default: Utf8String): Utf8String;
function xsdParseStringLowerDef(Chars: PChar; Len: Integer; Default: Utf8String): Utf8String;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function xsdParseBooleanDef(Chars: PChar; Len: Integer; Default: Boolean): Boolean;
function xsdParseDateDef(Chars: PChar; Len: Integer; Default: TDateTime; Timezone: PXsdTimezone = nil): TDateTime;
function xsdParseTimeDef(Chars: PChar; Len: Integer; Default: TDateTime; Timezone: PXsdTimezone = nil): TDateTime;
function xsdParseDateTimeDef(Chars: PChar; Len: Integer; Default: TDateTime; Timezone: PXsdTimezone = nil): TDateTime;
function xsdParseDecimalDef(Chars: PChar; Len: Integer; Default: Extended): Extended;
function xsdParseDoubleDef(Chars: PChar; Len: Integer; Default: Double): Double;
function xsdParseFloatDef(Chars: PChar; Len: Integer; Default: Single): Single;
function xsdParseIntegerDef(Chars: PChar; Len: Integer; Default: Int64): Int64;
function xsdParseNonNegativeIntegerDef(Chars: PChar; Len: Integer; Default: QWord): QWord;
function xsdParseNonPositiveIntegerDef(Chars: PChar; Len: Integer; Default: Int64): Int64;
function xsdParseNegativeIntegerDef(Chars: PChar; Len: Integer; Default: Int64): Int64;
function xsdParsePositiveIntegerDef(Chars: PChar; Len: Integer; Default: QWord): QWord;
function xsdParseByteDef(Chars: PChar; Len: Integer; Default: Shortint): Shortint;
function xsdParseShortDef(Chars: PChar; Len: Integer; Default: Smallint): Smallint;
function xsdParseIntDef(Chars: PChar; Len: Integer; Default: Longint): Longint;
function xsdParseLongDef(Chars: PChar; Len: Integer; Default: Int64): Int64;
function xsdParseUnsignedByteDef(Chars: PChar; Len: Integer; Default: Byte): Byte;
function xsdParseUnsignedShortDef(Chars: PChar; Len: Integer; Default: Word): Word;
function xsdParseUnsignedIntDef(Chars: PChar; Len: Integer; Default: Longword): Longword;
function xsdParseUnsignedLongDef(Chars: PChar; Len: Integer; Default: QWord): QWord;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdParseEnumDef(Chars: PChar; Len: Integer; enum: array of AnsiString; Default: Integer): Integer;

procedure xsdParseBase64(Chars: PChar; Len: Integer; const Value: TStream);
procedure xsdParseString(Chars: PChar; Len: Integer; out Value: AnsiString);
procedure xsdParseStringLower(Chars: PChar; Len: Integer; out Value: AnsiString);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function xsdParseEnumDef(Chars: PChar; Len: Integer; enum: array of Utf8String; Default: Integer): Integer;

procedure xsdParseBase64(Chars: PChar; Len: Integer; const Value: TStream);
procedure xsdParseString(Chars: PChar; Len: Integer; out Value: Utf8String);
procedure xsdParseStringLower(Chars: PChar; Len: Integer; out Value: Utf8String);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure xsdParseBoolean(Chars: PChar; Len: Integer; out Value: Boolean);
procedure xsdParseDate(Chars: PChar; Len: Integer; out Year, Month, Day: Longword; Timezone: PXsdTimezone = nil; BC: PBoolean = nil);
procedure xsdParseDate(Chars: PChar; Len: Integer; out Value: TDateTime; Timezone: PXsdTimezone = nil);
procedure xsdParseTime(Chars: PChar; Len: Integer; out Hour, Minute, Second, Milliseconds: Longword; Timezone: PXsdTimezone = nil);
procedure xsdParseTime(Chars: PChar; Len: Integer; out Value: TDateTime; Timezone: PXsdTimezone = nil);
procedure xsdParseDateTime(Chars: PChar; Len: Integer; out Year, Month, Day, Hour, Minute, Second, Milliseconds: Longword; Timezone: PXsdTimezone = nil; BC: PBoolean = nil);
procedure xsdParseDateTime(Chars: PChar; Len: Integer; out Value: TDateTime; Timezone: PXsdTimezone = nil);
procedure xsdParseDecimal(Chars: PChar; Len: Integer; out Value: Extended);
procedure xsdParseDouble(Chars: PChar; Len: Integer; out Value: Double);
procedure xsdParseFloat(Chars: PChar; Len: Integer; out Value: Single);
procedure xsdParseInteger(Chars: PChar; Len: Integer; out Value: Int64);
procedure xsdParseNonNegativeInteger(Chars: PChar; Len: Integer; out Value: QWord);
procedure xsdParseNonPositiveInteger(Chars: PChar; Len: Integer; out Value: Int64);
procedure xsdParseNegativeInteger(Chars: PChar; Len: Integer; out Value: Int64);
procedure xsdParsePositiveInteger(Chars: PChar; Len: Integer; out Value: QWord);
procedure xsdParseByte(Chars: PChar; Len: Integer; out Value: Shortint);
procedure xsdParseShort(Chars: PChar; Len: Integer; out Value: Smallint);
procedure xsdParseInt(Chars: PChar; Len: Integer; out Value: Longint);
procedure xsdParseLong(Chars: PChar; Len: Integer; out Value: Int64);
procedure xsdParseUnsignedByte(Chars: PChar; Len: Integer; out Value: Byte);
procedure xsdParseUnsignedShort(Chars: PChar; Len: Integer; out Value: Word);
procedure xsdParseUnsignedInt(Chars: PChar; Len: Integer; out Value: Longword);
procedure xsdParseUnsignedLong(Chars: PChar; Len: Integer; out Value: QWord);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure xsdParseEnum(Chars: PChar; Len: Integer; enum: array of AnsiString; out Value: Integer);

function xsdParseString(Chars: PChar; Len: Integer): AnsiString;
function xsdParseStringLower(Chars: PChar; Len: Integer): AnsiString;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure xsdParseEnum(Chars: PChar; Len: Integer; enum: array of Utf8String; out Value: Integer);

function xsdParseString(Chars: PChar; Len: Integer): Utf8String;
function xsdParseStringLower(Chars: PChar; Len: Integer): Utf8String;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function xsdParseBoolean(Chars: PChar; Len: Integer): Boolean;
function xsdParseDate(Chars: PChar; Len: Integer; Timezone: PXsdTimezone = nil): TDateTime;
function xsdParseTime(Chars: PChar; Len: Integer; Timezone: PXsdTimezone = nil): TDateTime;
function xsdParseDateTime(Chars: PChar; Len: Integer; Timezone: PXsdTimezone = nil): TDateTime;
function xsdParseDecimal(Chars: PChar; Len: Integer): Extended;
function xsdParseDouble(Chars: PChar; Len: Integer): Double;
function xsdParseFloat(Chars: PChar; Len: Integer): Single;
function xsdParseInteger(Chars: PChar; Len: Integer): Int64;
function xsdParseNonNegativeInteger(Chars: PChar; Len: Integer): QWord;
function xsdParseNonPositiveInteger(Chars: PChar; Len: Integer): Int64;
function xsdParseNegativeInteger(Chars: PChar; Len: Integer): Int64;
function xsdParsePositiveInteger(Chars: PChar; Len: Integer): QWord;
function xsdParseByte(Chars: PChar; Len: Integer): Shortint;
function xsdParseShort(Chars: PChar; Len: Integer): Smallint;
function xsdParseInt(Chars: PChar; Len: Integer): Longint;
function xsdParseLong(Chars: PChar; Len: Integer): Int64;
function xsdParseUnsignedByte(Chars: PChar; Len: Integer): Byte;
function xsdParseUnsignedShort(Chars: PChar; Len: Integer): Word;
function xsdParseUnsignedInt(Chars: PChar; Len: Integer): Longword;
function xsdParseUnsignedLong(Chars: PChar; Len: Integer): QWord;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdParseEnum(Chars: PChar; Len: Integer; enum: array of AnsiString): Integer;
{
function xsdTryParseBase64(const S: AnsiString; const Value: TStream): Boolean;
function xsdTryParseString(const S: AnsiString; out Value: AnsiString): Boolean;
function xsdTryParseStringLower(const S: AnsiString; out Value: AnsiString): Boolean;
function xsdTryParseBoolean(const S: AnsiString; out Value: Boolean): Boolean;
function xsdTryParseDate(const S: AnsiString; out Year, Month, Day: Longword; Timezone: PXsdTimezone = nil; BC: PBoolean = nil): Boolean;
function xsdTryParseDate(const S: AnsiString; out Value: TDateTime; Timezone: PXsdTimezone = nil): Boolean;
function xsdTryParseTime(const S: AnsiString; out Hour, Minute, Second, Milliseconds: Longword; Timezone: PXsdTimezone = nil): Boolean;
function xsdTryParseTime(const S: AnsiString; out Value: TDateTime; Timezone: PXsdTimezone = nil): Boolean;
function xsdTryParseDateTime(const S: AnsiString; out Year, Month, Day, Hour, Minute, Second, Milliseconds: Longword; Timezone: PXsdTimezone = nil; BC: PBoolean = nil): Boolean;
function xsdTryParseDateTime(const S: AnsiString; out Value: TDateTime; Timezone: PXsdTimezone = nil): Boolean;
function xsdTryParseDecimal(const S: AnsiString; out Value: Extended): Boolean;
function xsdTryParseDouble(const S: AnsiString; out Value: Double): Boolean;
function xsdTryParseFloat(const S: AnsiString; out Value: Single): Boolean;
function xsdTryParseInteger(const S: AnsiString; out Value: Int64): Boolean;
function xsdTryParseNonNegativeInteger(const S: AnsiString; out Value: QWord): Boolean;
function xsdTryParseNonPositiveInteger(const S: AnsiString; out Value: Int64): Boolean;
function xsdTryParseNegativeInteger(const S: AnsiString; out Value: Int64): Boolean;
function xsdTryParsePositiveInteger(const S: AnsiString; out Value: QWord): Boolean;
function xsdTryParseByte(const S: AnsiString; out Value: Shortint): Boolean;
function xsdTryParseShort(const S: AnsiString; out Value: Smallint): Boolean;
function xsdTryParseInt(const S: AnsiString; out Value: Longint): Boolean;
function xsdTryParseLong(const S: AnsiString; out Value: Int64): Boolean;
function xsdTryParseUnsignedByte(const S: AnsiString; out Value: Byte): Boolean;
function xsdTryParseUnsignedShort(const S: AnsiString; out Value: Word): Boolean;
function xsdTryParseUnsignedInt(const S: AnsiString; out Value: Longword): Boolean;
function xsdTryParseUnsignedLong(const S: AnsiString; out Value: QWord): Boolean;
function xsdTryParseEnum(const S: AnsiString; enum: array of AnsiString; out Value: Integer): Boolean;

function xsdParseStringDef(const S: AnsiString; Default: AnsiString): AnsiString;
function xsdParseStringLowerDef(const S: AnsiString; Default: AnsiString): AnsiString;
function xsdParseBooleanDef(const S: AnsiString; Default: Boolean): Boolean;
function xsdParseDateDef(const S: AnsiString; Default: TDateTime; Timezone: PXsdTimezone = nil): TDateTime;
function xsdParseTimeDef(const S: AnsiString; Default: TDateTime; Timezone: PXsdTimezone = nil): TDateTime;
function xsdParseDateTimeDef(const S: AnsiString; Default: TDateTime; Timezone: PXsdTimezone = nil): TDateTime;
function xsdParseDecimalDef(const S: AnsiString; Default: Extended): Extended;
function xsdParseDoubleDef(const S: AnsiString; Default: Double): Double;
function xsdParseFloatDef(const S: AnsiString; Default: Single): Single;
function xsdParseIntegerDef(const S: AnsiString; Default: Int64): Int64;
function xsdParseNonNegativeIntegerDef(const S: AnsiString; Default: QWord): QWord;
function xsdParseNonPositiveIntegerDef(const S: AnsiString; Default: Int64): Int64;
function xsdParseNegativeIntegerDef(const S: AnsiString; Default: Int64): Int64;
function xsdParsePositiveIntegerDef(const S: AnsiString; Default: QWord): QWord;
function xsdParseByteDef(const S: AnsiString; Default: Shortint): Shortint;
function xsdParseShortDef(const S: AnsiString; Default: Smallint): Smallint;
function xsdParseIntDef(const S: AnsiString; Default: Longint): Longint;
function xsdParseLongDef(const S: AnsiString; Default: Int64): Int64;
function xsdParseUnsignedByteDef(const S: AnsiString; Default: Byte): Byte;
function xsdParseUnsignedShortDef(const S: AnsiString; Default: Word): Word;
function xsdParseUnsignedIntDef(const S: AnsiString; Default: Longword): Longword;
function xsdParseUnsignedLongDef(const S: AnsiString; Default: QWord): QWord;
function xsdParseEnumDef(const S: AnsiString; enum: array of AnsiString; Default: Integer): Integer;
}
procedure xsdParseBase64(const S: AnsiString; const Value: TStream);
procedure xsdParseString(const S: AnsiString; out Value: AnsiString);
procedure xsdParseStringLower(const S: AnsiString; out Value: AnsiString);
procedure xsdParseBoolean(const S: AnsiString; out Value: Boolean);
procedure xsdParseDate(const S: AnsiString; out Year, Month, Day: Longword; Timezone: PXsdTimezone = nil; BC: PBoolean = nil);
procedure xsdParseDate(const S: AnsiString; out Value: TDateTime; Timezone: PXsdTimezone = nil);
procedure xsdParseTime(const S: AnsiString; out Hour, Minute, Second, Milliseconds: Longword; Timezone: PXsdTimezone = nil);
procedure xsdParseTime(const S: AnsiString; out Value: TDateTime; Timezone: PXsdTimezone = nil);
procedure xsdParseDateTime(const S: AnsiString; out Year, Month, Day, Hour, Minute, Second, Milliseconds: Longword; Timezone: PXsdTimezone = nil; BC: PBoolean = nil);
procedure xsdParseDateTime(const S: AnsiString; out Value: TDateTime; Timezone: PXsdTimezone = nil);
procedure xsdParseDecimal(const S: AnsiString; out Value: Extended);
procedure xsdParseDouble(const S: AnsiString; out Value: Double);
procedure xsdParseFloat(const S: AnsiString; out Value: Single);
procedure xsdParseInteger(const S: AnsiString; out Value: Int64);
procedure xsdParseNonNegativeInteger(const S: AnsiString; out Value: QWord);
procedure xsdParseNonPositiveInteger(const S: AnsiString; out Value: Int64);
procedure xsdParseNegativeInteger(const S: AnsiString; out Value: Int64);
procedure xsdParsePositiveInteger(const S: AnsiString; out Value: QWord);
procedure xsdParseByte(const S: AnsiString; out Value: Shortint);
procedure xsdParseShort(const S: AnsiString; out Value: Smallint);
procedure xsdParseInt(const S: AnsiString; out Value: Longint);
procedure xsdParseLong(const S: AnsiString; out Value: Int64);
procedure xsdParseUnsignedByte(const S: AnsiString; out Value: Byte);
procedure xsdParseUnsignedShort(const S: AnsiString; out Value: Word);
procedure xsdParseUnsignedInt(const S: AnsiString; out Value: Longword);
procedure xsdParseUnsignedLong(const S: AnsiString; out Value: QWord);
procedure xsdParseEnum(const S: AnsiString; enum: array of AnsiString; out Value: Integer);

function xsdParseString(const S: AnsiString): AnsiString;
function xsdParseStringLower(const S: AnsiString): AnsiString;
function xsdParseBoolean(const S: AnsiString): Boolean;
function xsdParseDate(const S: AnsiString; Timezone: PXsdTimezone = nil): TDateTime;
function xsdParseTime(const S: AnsiString; Timezone: PXsdTimezone = nil): TDateTime;
function xsdParseDateTime(const S: AnsiString; Timezone: PXsdTimezone = nil): TDateTime;
function xsdParseDecimal(const S: AnsiString): Extended;
function xsdParseDouble(const S: AnsiString): Double;
function xsdParseFloat(const S: AnsiString): Single;
function xsdParseInteger(const S: AnsiString): Int64;
function xsdParseNonNegativeInteger(const S: AnsiString): QWord;
function xsdParseNonPositiveInteger(const S: AnsiString): Int64;
function xsdParseNegativeInteger(const S: AnsiString): Int64;
function xsdParsePositiveInteger(const S: AnsiString): QWord;
function xsdParseByte(const S: AnsiString): Shortint;
function xsdParseShort(const S: AnsiString): Smallint;
function xsdParseInt(const S: AnsiString): Longint;
function xsdParseLong(const S: AnsiString): Int64;
function xsdParseUnsignedByte(const S: AnsiString): Byte;
function xsdParseUnsignedShort(const S: AnsiString): Word;
function xsdParseUnsignedInt(const S: AnsiString): Longword;
function xsdParseUnsignedLong(const S: AnsiString): QWord;
function xsdParseEnum(const S: AnsiString; enum: array of AnsiString): Integer;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function xsdParseEnum(Chars: PChar; Len: Integer; enum: array of Utf8String): Integer;
{
function xsdTryParseBase64(const S: UTF8String; const Value: TStream): Boolean;
function xsdTryParseString(const S: UTF8String; out Value: Utf8String): Boolean;
function xsdTryParseStringLower(const S: UTF8String; out Value: Utf8String): Boolean;
function xsdTryParseBoolean(const S: UTF8String; out Value: Boolean): Boolean;
function xsdTryParseDate(const S: UTF8String; out Year, Month, Day: Longword; Timezone: PXsdTimezone = nil; BC: PBoolean = nil): Boolean;
function xsdTryParseDate(const S: UTF8String; out Value: TDateTime; Timezone: PXsdTimezone = nil): Boolean;
function xsdTryParseTime(const S: UTF8String; out Hour, Minute, Second, Milliseconds: Longword; Timezone: PXsdTimezone = nil): Boolean;
function xsdTryParseTime(const S: UTF8String; out Value: TDateTime; Timezone: PXsdTimezone = nil): Boolean;
function xsdTryParseDateTime(const S: UTF8String; out Year, Month, Day, Hour, Minute, Second, Milliseconds: Longword; Timezone: PXsdTimezone = nil; BC: PBoolean = nil): Boolean;
function xsdTryParseDateTime(const S: UTF8String; out Value: TDateTime; Timezone: PXsdTimezone = nil): Boolean;
function xsdTryParseDecimal(const S: UTF8String; out Value: Extended): Boolean;
function xsdTryParseDouble(const S: UTF8String; out Value: Double): Boolean;
function xsdTryParseFloat(const S: UTF8String; out Value: Single): Boolean;
function xsdTryParseInteger(const S: UTF8String; out Value: Int64): Boolean;
function xsdTryParseNonNegativeInteger(const S: UTF8String; out Value: QWord): Boolean;
function xsdTryParseNonPositiveInteger(const S: UTF8String; out Value: Int64): Boolean;
function xsdTryParseNegativeInteger(const S: UTF8String; out Value: Int64): Boolean;
function xsdTryParsePositiveInteger(const S: UTF8String; out Value: QWord): Boolean;
function xsdTryParseByte(const S: UTF8String; out Value: Shortint): Boolean;
function xsdTryParseShort(const S: UTF8String; out Value: Smallint): Boolean;
function xsdTryParseInt(const S: UTF8String; out Value: Longint): Boolean;
function xsdTryParseLong(const S: UTF8String; out Value: Int64): Boolean;
function xsdTryParseUnsignedByte(const S: UTF8String; out Value: Byte): Boolean;
function xsdTryParseUnsignedShort(const S: UTF8String; out Value: Word): Boolean;
function xsdTryParseUnsignedInt(const S: UTF8String; out Value: Longword): Boolean;
function xsdTryParseUnsignedLong(const S: UTF8String; out Value: QWord): Boolean;
function xsdTryParseEnum(const S: UTF8String; enum: array of Utf8String; out Value: Integer): Boolean;

function xsdParseStringDef(const S: UTF8String; Default: Utf8String): Utf8String;
function xsdParseStringLowerDef(const S: UTF8String; Default: Utf8String): Utf8String;
function xsdParseBooleanDef(const S: UTF8String; Default: Boolean): Boolean;
function xsdParseDateDef(const S: UTF8String; Default: TDateTime; Timezone: PXsdTimezone = nil): TDateTime;
function xsdParseTimeDef(const S: UTF8String; Default: TDateTime; Timezone: PXsdTimezone = nil): TDateTime;
function xsdParseDateTimeDef(const S: UTF8String; Default: TDateTime; Timezone: PXsdTimezone = nil): TDateTime;
function xsdParseDecimalDef(const S: UTF8String; Default: Extended): Extended;
function xsdParseDoubleDef(const S: UTF8String; Default: Double): Double;
function xsdParseFloatDef(const S: UTF8String; Default: Single): Single;
function xsdParseIntegerDef(const S: UTF8String; Default: Int64): Int64;
function xsdParseNonNegativeIntegerDef(const S: UTF8String; Default: QWord): QWord;
function xsdParseNonPositiveIntegerDef(const S: UTF8String; Default: Int64): Int64;
function xsdParseNegativeIntegerDef(const S: UTF8String; Default: Int64): Int64;
function xsdParsePositiveIntegerDef(const S: UTF8String; Default: QWord): QWord;
function xsdParseByteDef(const S: UTF8String; Default: Shortint): Shortint;
function xsdParseShortDef(const S: UTF8String; Default: Smallint): Smallint;
function xsdParseIntDef(const S: UTF8String; Default: Longint): Longint;
function xsdParseLongDef(const S: UTF8String; Default: Int64): Int64;
function xsdParseUnsignedByteDef(const S: UTF8String; Default: Byte): Byte;
function xsdParseUnsignedShortDef(const S: UTF8String; Default: Word): Word;
function xsdParseUnsignedIntDef(const S: UTF8String; Default: Longword): Longword;
function xsdParseUnsignedLongDef(const S: UTF8String; Default: QWord): QWord;
function xsdParseEnumDef(const S: UTF8String; enum: array of Utf8String; Default: Integer): Integer;
}
procedure xsdParseBase64(const S: UTF8String; const Value: TStream);
procedure xsdParseString(const S: UTF8String; out Value: Utf8String);
procedure xsdParseStringLower(const S: UTF8String; out Value: Utf8String);
procedure xsdParseBoolean(const S: UTF8String; out Value: Boolean);
procedure xsdParseDate(const S: UTF8String; out Year, Month, Day: Longword; Timezone: PXsdTimezone = nil; BC: PBoolean = nil);
procedure xsdParseDate(const S: UTF8String; out Value: TDateTime; Timezone: PXsdTimezone = nil);
procedure xsdParseTime(const S: UTF8String; out Hour, Minute, Second, Milliseconds: Longword; Timezone: PXsdTimezone = nil);
procedure xsdParseTime(const S: UTF8String; out Value: TDateTime; Timezone: PXsdTimezone = nil);
procedure xsdParseDateTime(const S: UTF8String; out Year, Month, Day, Hour, Minute, Second, Milliseconds: Longword; Timezone: PXsdTimezone = nil; BC: PBoolean = nil);
procedure xsdParseDateTime(const S: UTF8String; out Value: TDateTime; Timezone: PXsdTimezone = nil);
procedure xsdParseDecimal(const S: UTF8String; out Value: Extended);
procedure xsdParseDouble(const S: UTF8String; out Value: Double);
procedure xsdParseFloat(const S: UTF8String; out Value: Single);
procedure xsdParseInteger(const S: UTF8String; out Value: Int64);
procedure xsdParseNonNegativeInteger(const S: UTF8String; out Value: QWord);
procedure xsdParseNonPositiveInteger(const S: UTF8String; out Value: Int64);
procedure xsdParseNegativeInteger(const S: UTF8String; out Value: Int64);
procedure xsdParsePositiveInteger(const S: UTF8String; out Value: QWord);
procedure xsdParseByte(const S: UTF8String; out Value: Shortint);
procedure xsdParseShort(const S: UTF8String; out Value: Smallint);
procedure xsdParseInt(const S: UTF8String; out Value: Longint);
procedure xsdParseLong(const S: UTF8String; out Value: Int64);
procedure xsdParseUnsignedByte(const S: UTF8String; out Value: Byte);
procedure xsdParseUnsignedShort(const S: UTF8String; out Value: Word);
procedure xsdParseUnsignedInt(const S: UTF8String; out Value: Longword);
procedure xsdParseUnsignedLong(const S: UTF8String; out Value: QWord);
procedure xsdParseEnum(const S: UTF8String; enum: array of Utf8String; out Value: Integer);

function xsdParseString(const S: UTF8String): Utf8String;
function xsdParseStringLower(const S: UTF8String): Utf8String;
function xsdParseBoolean(const S: UTF8String): Boolean;
function xsdParseDate(const S: UTF8String; Timezone: PXsdTimezone = nil): TDateTime;
function xsdParseTime(const S: UTF8String; Timezone: PXsdTimezone = nil): TDateTime;
function xsdParseDateTime(const S: UTF8String; Timezone: PXsdTimezone = nil): TDateTime;
function xsdParseDecimal(const S: UTF8String): Extended;
function xsdParseDouble(const S: UTF8String): Double;
function xsdParseFloat(const S: UTF8String): Single;
function xsdParseInteger(const S: UTF8String): Int64;
function xsdParseNonNegativeInteger(const S: UTF8String): QWord;
function xsdParseNonPositiveInteger(const S: UTF8String): Int64;
function xsdParseNegativeInteger(const S: UTF8String): Int64;
function xsdParsePositiveInteger(const S: UTF8String): QWord;
function xsdParseByte(const S: UTF8String): Shortint;
function xsdParseShort(const S: UTF8String): Smallint;
function xsdParseInt(const S: UTF8String): Longint;
function xsdParseLong(const S: UTF8String): Int64;
function xsdParseUnsignedByte(const S: UTF8String): Byte;
function xsdParseUnsignedShort(const S: UTF8String): Word;
function xsdParseUnsignedInt(const S: UTF8String): Longword;
function xsdParseUnsignedLong(const S: UTF8String): QWord;
function xsdParseEnum(const S: UTF8String; enum: array of Utf8String): Integer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew


{ INTERNAL HELPERS!!! }
const
  XSD_IGNORE_LAST = Pointer(-1); // maybe used as L parameter if the string is zero terminated

function __parseNonNegativeInteger(var P: PChar; const L: PChar; out Value: QWord): Boolean;
function __parseInteger(var P: PChar; const L: PChar; out Value: Int64): Boolean;
function __parseFloat(var P: PChar; const L: PChar; out Value: Extended): Boolean;
function __parseTimezone(var P: PChar; const L: PChar; out T: TXsdTimezone): Boolean;
function __parseDate(var P: PChar; const L: PChar; out Year, Month, Day: Longword; BC: PBoolean): Boolean;
function __parseTime(var P: PChar; const L: PChar; const AllowMoreThan24h: Boolean;
  out Hour, Minute, Second, Milliseconds: Longword): Boolean;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function __strpas(Chars: PChar; Len: Integer): AnsiString;

implementation

function xsdFormatBase64(Value: TStream): AnsiString;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
function __strpas(Chars: PChar; Len: Integer): Utf8String;

implementation

function xsdFormatBase64(Value: TStream): Utf8String;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
const
  Base64: array[0..63] of char = (
    'A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P',
    'Q','R','S','T','U','V','W','X','Y','Z','a','b','c','d','e','f',
    'g','h','i','j','k','l','m','n','o','p','q','r','s','t','u','v',
    'w','x','y','z','0','1','2','3','4','5','6','7','8','9','+','/'
  );
  BufferSize = 3*512; { buffer size must be a multiple of 3 }
var
  Buffer: array[0..BufferSize-1] of Byte;
  Num, Ofs: Integer;
  b1, b2, b3: Byte;
begin
  Result := '';
  while True do
  begin
    Num := Value.Read(Buffer, BufferSize);
    if Num = 0 then
      Exit;

    Ofs := 0;
    while Num >= 3 do
    begin
      b1 := Buffer[Ofs];
      b2 := Buffer[Ofs+1];
      b3 := Buffer[Ofs+2];

      Result := Result +
        Base64[b1 shr 2] +
        Base64[((b1 and $3) shl 4) or (b2 shr 4)] +
        Base64[((b2 and $F) shl 2) or (b3 shr 6)] +
        Base64[b3 and $3F];

      Num := Num - 3;
      Ofs := Ofs + 3;
    end;

    case Num of
      1: begin
        b1 := Buffer[Ofs];

        Result := Result +
          Base64[b1 shr 2] +
          Base64[((b1 and $3) shl 4)] +
          '==';

        Exit;
      end;

      2: begin
        b1 := Buffer[Ofs];
        b2 := Buffer[Ofs+1];

        Result := Result +
          Base64[b1 shr 2] +
          Base64[((b1 and $3) shl 4) or (b2 shr 4)] +
          Base64[((b2 and $F) shl 2)] +
          '=';

        Exit;
      end;
    end;
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdFormatBoolean(Value: Boolean; UseWords: Boolean): AnsiString;
=======
function xsdFormatBoolean(Value: Boolean; UseWords: Boolean): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdFormatBoolean(Value: Boolean; UseWords: Boolean): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdFormatBoolean(Value: Boolean; UseWords: Boolean): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdFormatBoolean(Value: Boolean; UseWords: Boolean): Utf8String;
>>>>>>> origin/cpstrnew
begin
  if UseWords then
    if Value then
      Result := 'true'
    else
      Result := 'false'
  else
    if Value then
      Result := '1'
    else
      Result := '0';
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdFormatDate(Year, Month, Day: Longword; BC: Boolean; Timezone: PXsdTimezone): AnsiString;
=======
function xsdFormatDate(Year, Month, Day: Longword; BC: Boolean; Timezone: PXsdTimezone): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdFormatDate(Year, Month, Day: Longword; BC: Boolean; Timezone: PXsdTimezone): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdFormatDate(Year, Month, Day: Longword; BC: Boolean; Timezone: PXsdTimezone): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdFormatDate(Year, Month, Day: Longword; BC: Boolean; Timezone: PXsdTimezone): Utf8String;
>>>>>>> origin/cpstrnew
begin
  Result := Format('%4.4d-%2.2u-%2.2u', [Year, Month, Day]);
  if BC then
    Result := '-' + Result;

  if Assigned(Timezone) then
    case Timezone^.Kind of
      tzUTC:
        Result := Result + 'Z';
      tzLOCAL:
        begin
          if Timezone^.Hour >= 0 then
            Result := Result + '+'
          else
            Result := Result + '-';
          Result := Result + Format('%2.2d:%2.2u', [Timezone^.Hour, Timezone^.Minute]);
        end;
    end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdFormatDate(Value: TDateTime; Timezone: PXsdTimezone): AnsiString;
=======
function xsdFormatDate(Value: TDateTime; Timezone: PXsdTimezone): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdFormatDate(Value: TDateTime; Timezone: PXsdTimezone): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdFormatDate(Value: TDateTime; Timezone: PXsdTimezone): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdFormatDate(Value: TDateTime; Timezone: PXsdTimezone): Utf8String;
>>>>>>> origin/cpstrnew
var
  Year, Month, Day: Word;
begin
  DecodeDate(Value, Year, Month, Day);
  Result := xsdFormatDate(Year, Month, Day, False, Timezone);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdFormatTime(Hour, Minute, Second, Milliseconds: Longword; Timezone: PXsdTimezone): AnsiString;
=======
function xsdFormatTime(Hour, Minute, Second, Milliseconds: Longword; Timezone: PXsdTimezone): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdFormatTime(Hour, Minute, Second, Milliseconds: Longword; Timezone: PXsdTimezone): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdFormatTime(Hour, Minute, Second, Milliseconds: Longword; Timezone: PXsdTimezone): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdFormatTime(Hour, Minute, Second, Milliseconds: Longword; Timezone: PXsdTimezone): Utf8String;
>>>>>>> origin/cpstrnew
begin
  Result := Format('%2.2u:%2.2u:%2.2u', [Hour, Minute, Second]);
  if Milliseconds > 0 then
    Result := Result + '.' + IntToStr(Milliseconds);

  if Assigned(Timezone) then
    case Timezone^.Kind of
      tzUTC:
        Result := Result + 'Z';
      tzLOCAL:
        begin
          if Timezone^.Hour >= 0 then
            Result := Result + '+'
          else
            Result := Result + '-';
          Result := Result + Format('%2.2d:%2.2u', [Timezone^.Hour, Timezone^.Minute]);
        end;
    end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdFormatTime(Value: TDateTime; Timezone: PXsdTimezone): AnsiString;
=======
function xsdFormatTime(Value: TDateTime; Timezone: PXsdTimezone): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdFormatTime(Value: TDateTime; Timezone: PXsdTimezone): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdFormatTime(Value: TDateTime; Timezone: PXsdTimezone): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdFormatTime(Value: TDateTime; Timezone: PXsdTimezone): Utf8String;
>>>>>>> origin/cpstrnew
var
  Hour, Minute, Second, Milliseconds: Word;
begin
  DecodeTime(Value, Hour, Minute, Second, Milliseconds);
  Result := xsdFormatTime(Hour, Minute, Second, Milliseconds, Timezone);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdFormatDateTime(Year, Month, Day, Hour, Minute, Second, Milliseconds: Longword; BC: Boolean; Timezone: PXsdTimezone): AnsiString;
=======
function xsdFormatDateTime(Year, Month, Day, Hour, Minute, Second, Milliseconds: Longword; BC: Boolean; Timezone: PXsdTimezone): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdFormatDateTime(Year, Month, Day, Hour, Minute, Second, Milliseconds: Longword; BC: Boolean; Timezone: PXsdTimezone): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdFormatDateTime(Year, Month, Day, Hour, Minute, Second, Milliseconds: Longword; BC: Boolean; Timezone: PXsdTimezone): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdFormatDateTime(Year, Month, Day, Hour, Minute, Second, Milliseconds: Longword; BC: Boolean; Timezone: PXsdTimezone): Utf8String;
>>>>>>> origin/cpstrnew
begin
  Result := xsdFormatDate(Year, Month, Day, BC, nil) + 'T' + xsdFormatTime(Hour, Minute, Second, Milliseconds, Timezone);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdFormatDateTime(Value: TDateTime; Timezone: PXsdTimezone): AnsiString;
=======
function xsdFormatDateTime(Value: TDateTime; Timezone: PXsdTimezone): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdFormatDateTime(Value: TDateTime; Timezone: PXsdTimezone): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdFormatDateTime(Value: TDateTime; Timezone: PXsdTimezone): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdFormatDateTime(Value: TDateTime; Timezone: PXsdTimezone): Utf8String;
>>>>>>> origin/cpstrnew
var
  Year, Month, Day, Hour, Minute, Second, Milliseconds: Word;
begin
  DecodeDateTime(Value, Year, Month, Day, Hour, Minute, Second, Milliseconds);
  Result := xsdFormatDateTime(Year, Month, Day, Hour, Minute, Second, Milliseconds, False, Timezone);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdFormatDecimal(Value: Extended; Precision: Integer; Digits: Integer): AnsiString;
=======
function xsdFormatDecimal(Value: Extended; Precision: Integer; Digits: Integer): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdFormatDecimal(Value: Extended; Precision: Integer; Digits: Integer): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdFormatDecimal(Value: Extended; Precision: Integer; Digits: Integer): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdFormatDecimal(Value: Extended; Precision: Integer; Digits: Integer): Utf8String;
>>>>>>> origin/cpstrnew
begin
  Result := FloatToStrF(Value, ffFixed, Precision, Digits);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdFormatDouble(Value: Double): AnsiString;
=======
function xsdFormatDouble(Value: Double): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdFormatDouble(Value: Double): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdFormatDouble(Value: Double): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdFormatDouble(Value: Double): Utf8String;
>>>>>>> origin/cpstrnew
begin
  Result := FloatToStr(Value);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdFormatFloat(Value: Single): AnsiString;
=======
function xsdFormatFloat(Value: Single): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdFormatFloat(Value: Single): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdFormatFloat(Value: Single): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdFormatFloat(Value: Single): Utf8String;
>>>>>>> origin/cpstrnew
begin
  Result := FloatToStr(Value);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdFormatByte(Value: Shortint): AnsiString;
=======
function xsdFormatByte(Value: Shortint): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdFormatByte(Value: Shortint): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdFormatByte(Value: Shortint): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdFormatByte(Value: Shortint): Utf8String;
>>>>>>> origin/cpstrnew
begin
  Result := IntToStr(Value);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdFormatShort(Value: Smallint): AnsiString;
=======
function xsdFormatShort(Value: Smallint): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdFormatShort(Value: Smallint): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdFormatShort(Value: Smallint): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdFormatShort(Value: Smallint): Utf8String;
>>>>>>> origin/cpstrnew
begin
  Result := IntToStr(Value);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdFormatInt(Value: Integer): AnsiString;
=======
function xsdFormatInt(Value: Integer): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdFormatInt(Value: Integer): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdFormatInt(Value: Integer): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdFormatInt(Value: Integer): Utf8String;
>>>>>>> origin/cpstrnew
begin
  Result := IntToStr(Value);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdFormatLong(Value: Int64): AnsiString;
=======
function xsdFormatLong(Value: Int64): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdFormatLong(Value: Int64): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdFormatLong(Value: Int64): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdFormatLong(Value: Int64): Utf8String;
>>>>>>> origin/cpstrnew
begin
  Result := IntToStr(Value);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdFormatUnsignedByte(Value: Byte): AnsiString;
=======
function xsdFormatUnsignedByte(Value: Byte): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdFormatUnsignedByte(Value: Byte): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdFormatUnsignedByte(Value: Byte): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdFormatUnsignedByte(Value: Byte): Utf8String;
>>>>>>> origin/cpstrnew
begin
  Result := IntToStr(Value);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdFormatUnsignedShort(Value: Word): AnsiString;
=======
function xsdFormatUnsignedShort(Value: Word): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdFormatUnsignedShort(Value: Word): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdFormatUnsignedShort(Value: Word): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdFormatUnsignedShort(Value: Word): Utf8String;
>>>>>>> origin/cpstrnew
begin
  Result := IntToStr(Value);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdFormatUnsignedInt(Value: Longword): AnsiString;
=======
function xsdFormatUnsignedInt(Value: Longword): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdFormatUnsignedInt(Value: Longword): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdFormatUnsignedInt(Value: Longword): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdFormatUnsignedInt(Value: Longword): Utf8String;
>>>>>>> origin/cpstrnew
begin
  Result := IntToStr(Value);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdFormatUnsignedLong(Value: QWord): AnsiString;
=======
function xsdFormatUnsignedLong(Value: QWord): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdFormatUnsignedLong(Value: QWord): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdFormatUnsignedLong(Value: QWord): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdFormatUnsignedLong(Value: QWord): Utf8String;
>>>>>>> origin/cpstrnew
begin
  Result := IntToStr(Value);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdFormatEnum(enum: array of AnsiString; Value: Integer): AnsiString;
=======
function xsdFormatEnum(enum: array of Utf8String; Value: Integer): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdFormatEnum(enum: array of Utf8String; Value: Integer): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdFormatEnum(enum: array of Utf8String; Value: Integer): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdFormatEnum(enum: array of Utf8String; Value: Integer): Utf8String;
>>>>>>> origin/cpstrnew
begin
  Result := enum[Value];
end;

function xsdNowUTC: TDateTime;
begin
  Result := xsdDateTimeToUTC(Now, xsdGetLocalTimezone);
end;

function xsdGetLocalTimezone: TXsdTimezone;
var
  Offset: Integer;
{$IFDEF MSWINDOWS}
  TZInfo: TTimeZoneInformation;
{$ENDIF}
begin
  Result.Kind := tzLOCAL;
{$IFDEF UNIX}
  Offset := Tzseconds div 60;
{$ENDIF}
{$IFDEF MSWINDOWS}
  case GetTimeZoneInformation(TZInfo) of
    1: Offset := -TZInfo.Bias - TZInfo.StandardBias;
    2: Offset := -TZInfo.Bias - TZInfo.DaylightBias;
    else Result.Kind := tzUnknown;
  end;
{$ENDIF}
  Result.Hour := Offset div 60;
  Result.Minute := abs(Offset) mod 60;
end;

function xsdTimezoneUtcOffsetMinutes(const Timezone: TXsdTimezone): Longint;
begin
  case Timezone.Kind of
    tzUTC: Result := 0;
    tzLOCAL : Result := 60*Timezone.Hour + Timezone.Minute;
    else raise Exception.Create('can''t get offset of unknown timezone');
  end;
end;

function xsdDateTimeToUTC(const DateTime: TDateTime; const Current: TXsdTimezone): TDateTime;
begin
  Result := xsdDateTimeConvert(DateTime, Current, TIMEZONE_UTC);
end;

function xsdDateTimeConvert(const DateTime: TDateTime; const Current, Target: TXsdTimezone): TDateTime;
begin
  Result := IncMinute(DateTime, xsdTimezoneUtcOffsetMinutes(Target) - xsdTimezoneUtcOffsetMinutes(Current));
end;

function __parseNonNegativeInteger(var P: PChar; const L: PChar; out Value: QWord): Boolean;
begin
  { expect integer }
  Value := 0;
  while (P < L) and (P^ in ['0'..'9']) do
  begin
    Value := 10*Value + Ord(P^) - Ord('0');
    Inc(P);
  end;

  Result := True;
end;

function __parseInteger(var P: PChar; const L: PChar; out Value: Int64): Boolean;
var
  N: Boolean;
begin
  { allow '-' }
  N := (P < L) and (P^ = '-');
  if N then
    Inc(P);

  { expect integer }
  Value := 0;
  while (P < L) and (P^ in ['0'..'9']) do
  begin
    Value := 10*Value + Ord(P^) - Ord('0');
    Inc(P);
  end;
  if N then
    Value := -Value;

  Result := True;
end;

function __parseFloat(var P: PChar; const L: PChar; out Value: Extended): Boolean;
var
  N: Boolean;
  Exp: Int64;
  Int: QWord;
begin
  { allow 'Nan' }
  if (P+2 < L) and ((P^ = 'N') or (P^ = 'n')) then
  begin
    Inc(P);
    if (P^ <> 'A') and (P^ <> 'a') then Exit(False);
    Inc(P);
    if (P^ <> 'N') and (P^ <> 'n') then Exit(False);
    Inc(P);
    Value := Nan;
    Result := True;
    Exit;
  end;

  { allow '-' }
  N := (P < L) and (P^ = '-');
  if N then
    Inc(P);

  { allow 'Inf' }
  if (P+2 < L) and ((P^ = 'I') or (P^ = 'i')) then
  begin
    Inc(P);
    if (P^ <> 'N') and (P^ <> 'n') then Exit(False);
    Inc(P);
    if (P^ <> 'F') and (P^ <> 'f') then Exit(False);
    Inc(P);
    if N then
      Value := NegInfinity
    else
      Value := Infinity;
    Result := True;
    Exit;
  end;

  { expect integer }
  Int := 0;
  while (P < L) and (P^ in ['0'..'9']) do
  begin
    Int := 10*Int + Ord(P^) - Ord('0');
    Inc(P);
  end;
  Value := Int;

  { allow '.' }
  if (P < L) and (P^ = '.') then
  begin
    Inc(P);

    { expect integer }
    Exp := 1;
    Int := 0;
    while (P < L) and (P^ in ['0'..'9']) do
    begin
      Int := 10*Int + Ord(P^) - Ord('0');
      Exp := 10*Exp;
      Inc(P);
    end;
    Value := Value + Int / Exp;
  end;

  { allow 'E' or 'e' }
  if (P < L) and ((P^ = 'E') or (P^ = 'e')) then
  begin
    Inc(P);

    { expect integer }
    if not __parseInteger(P, L, Exp) then
      Exit(False);

    while Exp > 0 do
    begin
      Value := Value * 10;
      Dec(Exp);
    end;

    while Exp < 0 do
    begin
      Value := Value * 0.1;
      Inc(Exp);
    end;
  end;

  if N then
    Value := -Value;

  Result := True;
end;

function __parseTimezone(var P: PChar; const L: PChar; out T: TXsdTimezone): Boolean;
var
  I: Integer;
  N: Boolean;
begin
  { allow 'Z' }
  if (P < L) and (P^ = 'Z') then
  begin
    T.Kind := tzUTC;
    T.Hour := 0;
    T.Minute := 0;
    Inc(P);
  end else

    { allow '+' or '-' }
    if (P < L) and (P^ in ['+','-']) then
    begin
      T.Kind := tzLOCAL;
      N := P^ = '-';
      Inc(P);

      { expect 00..13 }
      T.Hour := 0; I := 2;
      while (P < L) and (P^ in ['0'..'9']) and (I > 0) do
      begin
        T.Hour := 10*T.Hour + Ord(P^) - Ord('0');
        Dec(I); Inc(P);
      end;
      if T.Hour > 13 then
        Exit(False);
      if N then
        T.Hour := -T.Hour;

      { expect ':' }
      if (P >= L) or (P^ <> ':') then
        Exit(False);
      Inc(P);

      { expect 00..59 }
      T.Minute := 0; I := 2;
      while (P < L) and (P^ in ['0'..'9']) and (I > 0) do
      begin
        T.Minute := 10*T.Minute + Ord(P^) - Ord('0');
        Dec(I); Inc(P);
      end;
      if T.Minute > 59 then
        Exit(False);
    end else

      { unknown }
      begin
        T.Kind := tzUNKNOWN;
        T.Hour := 0;
        T.Minute := 0;
      end;

  Result := True;
end;

function __parseDate(var P: PChar; const L: PChar; out Year, Month, Day: Longword; BC: PBoolean): Boolean;
var
  I: Integer;
begin
  { allow '-' }
  if (P < L) and (P^ = '-') then
  begin
    if Assigned(BC) then
      BC^ := True
    else
      Exit(False);
    Inc(P);
  end else
    if Assigned(BC) then
      BC^ := False;

  { expect Integer }
  Year := 0;
  while (P < L) and (P^ in ['0'..'9']) do
  begin
    Year := 10*Year + Ord(P^) - Ord('0');
    Inc(P);
  end;

  { expect '-' }
  if (P >= L) or (P^ <> '-') then
    Exit(False);
  Inc(P);

  { expect 01..12 }
  Month := 0; I := 2;
  while (P < L) and (P^ in ['0'..'9']) and (I > 0) do
  begin
    Month := 10*Month + Ord(P^) - Ord('0');
    Dec(I); Inc(P);
  end;
  if (Month < 1) or (Month > 12) then
    Exit(False);

  { expect '-' }
  if (P >= L) or (P^ <> '-') then
    Exit(False);
  Inc(P);

  { expect 01..31 }
  Day := 0; I := 2;
  while (P < L) and (P^ in ['0'..'9']) and (I > 0) do
  begin
    Day := 10*Day + Ord(P^) - Ord('0');
    Dec(I); Inc(P);
  end;
  if (Day < 1) or (Day > 31) then
    Exit(False);

  Result := True;
end;

function __parseTime(var P: PChar; const L: PChar; const AllowMoreThan24h: Boolean;
  out Hour, Minute, Second, Milliseconds: Longword): Boolean;
var
  I: Integer;
  Ms: Longword;
begin
  { expect 00..24 (except if AllowMoreThan24h) }
  Hour := 0;
  if AllowMoreThan24h then I := 9 { maximal 9 digits for hour } else I := 2;
  while (P < L) and (P^ in ['0'..'9']) and (I > 0) do
  begin
    Hour := 10*Hour + Ord(P^) - Ord('0');
    Inc(P); Dec(I);
  end;
  if not AllowMoreThan24h and (Hour > 24) then
    Exit(False);

  { expect ':' }
  if (P >= L) or (P^ <> ':') then
    Exit(False);
  Inc(P);

  { expect 00..59 }
  Minute := 0; I := 2;
  while (P < L) and (P^ in ['0'..'9']) and (I > 0) do
  begin
    Minute := 10*Minute + Ord(P^) - Ord('0');
    Dec(I); Inc(P);
  end;
  if (Minute > 59) or (not AllowMoreThan24h and (Hour = 24) and (Minute > 0)) then
    Exit(False);

  { expect ':' }
  if (P >= L) or (P^ <> ':') then
    Exit(False);
  Inc(P);

  { expect 00..59 }
  Second := 0; I := 2;
  while (P < L) and (P^ in ['0'..'9']) and (I > 0) do
  begin
    Second := 10*Second + Ord(P^) - Ord('0');
    Dec(I); Inc(P);
  end;
  if (Second > 59) or (not AllowMoreThan24h and (Hour = 24) and (Second > 0)) then
    Exit(False);

  { allow '.' }
  if (P < L) and (P^ = '.') then
  begin
    Inc(P);

    { expect integer }
    Ms := 0; I := 1;
    while (P < L) and (P^ in ['0'..'9']) do
    begin
      Ms := 10*Ms + Ord(P^) - Ord('0');
      I := 10*I;
      Inc(P);
    end;
    Milliseconds := (1000*Ms) div I;
    if (Milliseconds >= 999) or (not AllowMoreThan24h and (Hour = 24) and (Milliseconds > 0)) then
      Exit(False);
  end else
    Milliseconds := 0;

  Result := True;
end;

function xsdTryParseBase64(Chars: PChar; Len: Integer; const Value: TStream): Boolean;
const
  BufferSize = 3*512;
var
  Buffer: array[0..BufferSize-1] of Byte;
  Ofs: Integer;
  P,L: PByte;
  p1,p2,p3,p4: Shortint;
begin
  if Assigned(Chars) then
  begin
    Ofs := 0;

    P := PByte(Chars);
    if Len >= 0 then
    begin
      if Len mod 4 <> 0 then
        Exit(False);

      L := P + Len;
      while P < L do
      begin
        case Chr(P^) of
          'A'..'Z': p1 := P^ - Ord('A');
          'a'..'z': p1 := P^ - Ord('a') + 26;
          '0'..'9': p1 := P^ - Ord('0') + 52;
          '+' : p1 := 62;
          '/' : p1 := 63;
          else Exit(False);
        end;
        Inc(P);

        case Chr(P^) of
          'A'..'Z': p2 := P^ - Ord('A');
          'a'..'z': p2 := P^ - Ord('a') + 26;
          '0'..'9': p2 := P^ - Ord('0') + 52;
          '+' : p2 := 62;
          '/' : p2 := 63;
          else Exit(False);
        end;
        Inc(P);

        case Chr(P^) of
          'A'..'Z': p3 := P^ - Ord('A');
          'a'..'z': p3 := P^ - Ord('a') + 26;
          '0'..'9': p3 := P^ - Ord('0') + 52;
          '+' : p3 := 62;
          '/' : p3 := 63;
          '=' : p3 := -1;
          else Exit(False);
        end;
        Inc(P);

        if (p3 >= 0) then
        begin
          case Chr(P^) of
            'A'..'Z': p4 := P^ - Ord('A');
            'a'..'z': p4 := P^ - Ord('a') + 26;
            '0'..'9': p4 := P^ - Ord('0') + 52;
            '+' : p4 := 62;
            '/' : p4 := 63;
            '=' : p4 := -1;
            else Exit(False);
          end;
        end else begin
          if P^ <> Ord('=') then
            Exit(False);
          p4 := -1;
        end;
        Inc(P);

        Buffer[Ofs] := (p1 shl 2) or (p2 shr 4);
        Ofs := Ofs + 1;

        if p3 >= 0 then
        begin
          Buffer[Ofs] := ((p2 and $F) shl 4) or (p3 shr 2);
          Ofs := Ofs + 1;

          if p4 >= 0 then
          begin
            Buffer[Ofs] := ((p3 and $3) shl 6) or p4;
            Ofs := Ofs + 1;
          end;
        end;

        if Ofs >= BufferSize-2 then
        begin
          Value.Write(Buffer, Ofs);
          Ofs := 0;
        end;
      end;
    end else begin
      while P^ <> 0 do
      begin
        case Chr(P^) of
          'A'..'Z': p1 := P^ - Ord('A');
          'a'..'z': p1 := P^ - Ord('a') + 26;
          '0'..'9': p1 := P^ - Ord('0') + 52;
          '+' : p1 := 62;
          '/' : p1 := 63;
          else Exit(False);
        end;
        Inc(P);

        case Chr(P^) of
          'A'..'Z': p2 := P^ - Ord('A');
          'a'..'z': p2 := P^ - Ord('a') + 26;
          '0'..'9': p2 := P^ - Ord('0') + 52;
          '+' : p2 := 62;
          '/' : p2 := 63;
          else Exit(False);
        end;
        Inc(P);

        case Chr(P^) of
          'A'..'Z': p3 := P^ - Ord('A');
          'a'..'z': p3 := P^ - Ord('a') + 26;
          '0'..'9': p3 := P^ - Ord('0') + 52;
          '+' : p3 := 62;
          '/' : p3 := 63;
          '=' : p3 := -1;
          else Exit(False);
        end;
        Inc(P);

        if (p3 >= 0) then
        begin
          case Chr(P^) of
            'A'..'Z': p4 := P^ - Ord('A');
            'a'..'z': p4 := P^ - Ord('a') + 26;
            '0'..'9': p4 := P^ - Ord('0') + 52;
            '+' : p4 := 62;
            '/' : p4 := 63;
            '=' : p4 := -1;
            else Exit(False);
          end;
        end else begin
          if P^ <> Ord('=') then
            Exit(False);
          p4 := -1;
        end;
        Inc(P);

        Buffer[Ofs] := (p1 shl 2) or (p2 shr 4);
        Ofs := Ofs + 1;

        if p3 >= 0 then
        begin
          Buffer[Ofs] := ((p2 and $F) shl 4) or (p3 shr 2);
          Ofs := Ofs + 1;

          if p4 >= 0 then
          begin
            Buffer[Ofs] := ((p3 and $3) shl 6) or p4;
            Ofs := Ofs + 1;
          end;
        end;

        if Ofs >= BufferSize-2 then
        begin
          Value.Write(Buffer, Ofs);
          Ofs := 0;
        end;
      end;
    end;

    if Ofs > 0 then // flush
      Value.Write(Buffer, Ofs);

    Result := True;
  end else
    Result := False;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdTryParseString(Chars: PChar; Len: Integer; out Value: AnsiString): Boolean;
=======
function xsdTryParseString(Chars: PChar; Len: Integer; out Value: Utf8String): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseString(Chars: PChar; Len: Integer; out Value: Utf8String): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseString(Chars: PChar; Len: Integer; out Value: Utf8String): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseString(Chars: PChar; Len: Integer; out Value: Utf8String): Boolean;
>>>>>>> origin/cpstrnew
const
  AllocChars = 256;
var
  P,L,D: PByte;
begin
  if Assigned(Chars) then
  begin
    P := PByte(Chars);
    if Len >= 0 then
    begin
      L := P + Len;
      SetLength(Value, Len);
      D := @Value[1];
      while P < L do
      begin
        D^ := P^;
        Inc(D);
        Inc(P);
      end;
    end else begin
      SetLength(Value, AllocChars);
      D := @Value[1];
      L := D + AllocChars;
      while P^ <> 0 do
      begin
        if D = L then
        begin
          Len := Length(Value);
          SetLength(Value, Len+AllocChars);
          D := @Value[Len+1];
          L := D + AllocChars;
        end;
        D^ := P^;
        Inc(D);
        Inc(P);
      end;
      SetLength(Value, P-PByte(Chars));
    end;
    Result := True;
  end else
    Result := False;
end;
{begin
  if Assigned(Chars) then
  begin
    if Len >= 0 then
    begin
      SetLength(Value, Len);
      Move(Chars^, Value[1], Len);
    end else
      Value := PChar(Chars);
    Result := True;
  end else
    Result := False;
end;}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdTryParseStringLower(Chars: PChar; Len: Integer; out Value: AnsiString): Boolean;
=======
function xsdTryParseStringLower(Chars: PChar; Len: Integer; out Value: Utf8String): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseStringLower(Chars: PChar; Len: Integer; out Value: Utf8String): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseStringLower(Chars: PChar; Len: Integer; out Value: Utf8String): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseStringLower(Chars: PChar; Len: Integer; out Value: Utf8String): Boolean;
>>>>>>> origin/cpstrnew
const
  AllocChars = 256;
var
  P,L,D: PByte;
  C: Byte;
begin
  if Assigned(Chars) then
  begin
    P := PByte(Chars);
    if Len >= 0 then
    begin
      L := P + Len;
      SetLength(Value, Len);
      D := @Value[1];
      while P < L do
      begin
        C := P^;
        if (C>=65) and (C<=90) then Inc(C, 32);
        D^ := C;
        Inc(D);
        Inc(P);
      end;
    end else begin
      SetLength(Value, AllocChars);
      D := @Value[1];
      L := D + AllocChars;
      while P^ <> 0 do
      begin
        C := P^;
        if (C>=65) and (C<=90) then Inc(C, 32);
        if D = L then
        begin
          Len := Length(Value);
          SetLength(Value, Len+AllocChars);
          D := @Value[Len+1];
          L := D + AllocChars;
        end;
        D^ := C;
        Inc(D);
        Inc(P);
      end;
      SetLength(Value, P-PByte(Chars));
    end;
    Result := True;
  end else
    Result := False;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function __strpas(Chars: PChar; Len: Integer): AnsiString;
=======
function __strpas(Chars: PChar; Len: Integer): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function __strpas(Chars: PChar; Len: Integer): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function __strpas(Chars: PChar; Len: Integer): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function __strpas(Chars: PChar; Len: Integer): Utf8String;
>>>>>>> origin/cpstrnew
begin
  if not xsdTryParseString(Chars, Len, Result) then
    Result := '';
end;

function xsdTryParseBoolean(Chars: PChar; Len: Integer; out Value: Boolean): Boolean;
var
  P: PChar;
  Num: QWord;
begin
  if not Assigned(Chars) then
    Exit(False);

  if Len < 0 then
  begin
    P := PChar(Chars);
    Len := 0;
    while (Len < 7) and (P^ <> #0) do
    begin
      Inc(Len);
      Inc(P);
    end;
  end;

  case Len of
    1: Num := PByte(Chars)^;
    4: Num := PLongword(Chars)^;
    5: Num := PLongword(Chars)^ or (QWord(Chars[4]) shl 32);
    else Exit(False);
  end;

  case Num of
    $30,
    $65736C6166,$65736C6146,$65736C4166,$65736C4146,$65734C6166,$65734C6146,$65734C4166,$65734C4146,
    $65536C6166,$65536C6146,$65536C4166,$65536C4146,$65534C6166,$65534C6146,$65534C4166,$65534C4146,
    $45736C6166,$45736C6146,$45736C4166,$45736C4146,$45734C6166,$45734C6146,$45734C4166,$45734C4146,
    $45536C6166,$45536C6146,$45536C4166,$45536C4146,$45534C6166,$45534C6146,$45534C4166,$45534C4146:
      Value := False;
    $31,
    $65757274,$65757254,$65755274,$65755254,$65557274,$65557254,$65555274,$65555254,
    $45757274,$45757254,$45755274,$45755254,$45557274,$45557254,$45555274,$45555254:
      Value := True;
    else Exit(False);
  end;

  Result := True;
end;

function xsdTryParseDate(Chars: PChar; Len: Integer; out Year, Month, Day: Longword; Timezone: PXsdTimezone; BC: PBoolean): Boolean;
var
  P: PChar;
  L: PChar;
  T: TXsdTimezone;
begin
  P := PChar(Chars);
  if Len >= 0 then
  begin
    L := P + Len;
    Result := Assigned(P) and
      __parseDate(P, L, Year, Month, Day, BC) and
      __parseTimezone(P, L, T) and (P = L)
  end else
    Result := Assigned(P) and
      __parseDate(P, XSD_IGNORE_LAST, Year, Month, Day, BC) and
      __parseTimezone(P, XSD_IGNORE_LAST, T) and (P^ = #0);

  { assign Timezone if requested }
  if Result and Assigned(Timezone) then
    Timezone^ := T;
end;

function xsdTryParseDate(Chars: PChar; Len: Integer; out Value: TDateTime; Timezone: PXsdTimezone): Boolean;
var
  Year, Month, Day: Longword;
begin
  if xsdTryParseDate(Chars, Len, Year, Month, Day, Timezone, nil) then
    Result := TryEncodeDate(Year, Month, Day, Value)
  else
    Result := False;
end;

function xsdTryParseTime(Chars: PChar; Len: Integer; out Hour, Minute, Second, Milliseconds: Longword; Timezone: PXsdTimezone): Boolean;
var
  P: PChar;
  L: PChar;
  T: TXsdTimezone;
begin
  P := PChar(Chars);
  if Len >= 0 then
  begin
    L := P + Len;
    Result := Assigned(P) and
      __parseTime(P, L, False, Hour, Minute, Second, Milliseconds) and
      __parseTimezone(P, L, T) and (P = L)
  end else
    Result := Assigned(P) and
      __parseTime(P, XSD_IGNORE_LAST, False, Hour, Minute, Second, Milliseconds) and
      __parseTimezone(P, XSD_IGNORE_LAST, T) and (P^ = #0);

  { assign Timezone if requested }
  if Result and Assigned(Timezone) then
    Timezone^ := T;
end;

function xsdTryParseTime(Chars: PChar; Len: Integer; out Value: TDateTime; Timezone: PXsdTimezone): Boolean;
var
  Hour, Minute, Second, Milliseconds: Longword;
begin
  if xsdTryParseTime(Chars, Len, Hour, Minute, Second, Milliseconds, Timezone) then
    Result := TryEncodeTime(Hour, Minute, Second, Milliseconds, Value)
  else
    Result := False;
end;

function xsdTryParseDateTime(Chars: PChar; Len: Integer; out Year, Month, Day, Hour, Minute, Second, Milliseconds: Longword; Timezone: PXsdTimezone; BC: PBoolean): Boolean;

    function __parseT(var P: PChar; const L: PChar): Boolean;
    begin
      Result := (P < L) and (P^ = 'T');
      if Result then Inc(P);
    end;

var
  P: PChar;
  L: PChar;
  T: TXsdTimezone;
begin
  P := PChar(Chars);
  if Len >= 0 then
  begin
    L := P + Len;
    Result := Assigned(P) and
      __parseDate(P, L, Year, Month, Day, BC) and
      __parseT(P, L) and
      __parseTime(P, L, False, Hour, Minute, Second, Milliseconds) and
      __parseTimezone(P, L, T) and (P = L)
  end else
    Result := Assigned(P) and
      __parseDate(P, XSD_IGNORE_LAST, Year, Month, Day, BC) and
      __parseT(P, XSD_IGNORE_LAST) and
      __parseTime(P, XSD_IGNORE_LAST, False, Hour, Minute, Second, Milliseconds) and
      __parseTimezone(P, XSD_IGNORE_LAST, T) and (P^ = #0);

  { assign Timezone if requested }
  if Result and Assigned(Timezone) then
    Timezone^ := T;
end;

function xsdTryParseDateTime(Chars: PChar; Len: Integer; out Value: TDateTime; Timezone: PXsdTimezone): Boolean;
var
  Year, Month, Day: Longword;
  Hour, Minute, Second, Milliseconds: Longword;
begin
  if xsdTryParseDateTime(Chars, Len, Year, Month, Day, Hour, Minute, Second, Milliseconds, Timezone) then
    Result := TryEncodeDateTime(Year, Month, Day, Hour, Minute, Second, Milliseconds, Value)
  else
    Result := False;
end;

function xsdTryParseDecimal(Chars: PChar; Len: Integer; out Value: Extended): Boolean;
var
  P: PChar;
  L: PChar;
begin
  P := PChar(Chars);
  if Len >= 0 then
  begin
    L := P + Len;
    Result := Assigned(P) and __parseFloat(P, L, Value) and (P = L)
  end else
    Result := Assigned(P) and __parseFloat(P, XSD_IGNORE_LAST, Value) and (P^ = #0);
end;

function xsdTryParseDouble(Chars: PChar; Len: Integer; out Value: Double): Boolean;
var
  P: PChar;
  L: PChar;
  Tmp: Extended;
begin
  P := PChar(Chars);
  if Len >= 0 then
  begin
    L := P + Len;
    Result := Assigned(P) and __parseFloat(P, L, Tmp) and (P = L)
  end else
    Result := Assigned(P) and __parseFloat(P, XSD_IGNORE_LAST, Tmp) and (P^ = #0);
  Value := Tmp;
end;

function xsdTryParseFloat(Chars: PChar; Len: Integer; out Value: Single): Boolean;
var
  P: PChar;
  L: PChar;
  Tmp: Extended;
begin
  P := PChar(Chars);
  if Len >= 0 then
  begin
    L := P + Len;
    Result := Assigned(P) and __parseFloat(P, L, Tmp) and (P = L)
  end else
    Result := Assigned(P) and __parseFloat(P, XSD_IGNORE_LAST, Tmp) and (P^ = #0);
  Value := Tmp;
end;

function xsdTryParseInteger(Chars: PChar; Len: Integer; out Value: Int64): Boolean;
var
  P: PChar;
  L: PChar;
begin
  P := PChar(Chars);
  if Len >= 0 then
  begin
    L := P + Len;
    Result := Assigned(P) and __parseInteger(P, L, Value) and (P = L)
  end else
    Result := Assigned(P) and __parseInteger(P, XSD_IGNORE_LAST, Value) and (P^ = #0);
end;

function xsdTryParseNonNegativeInteger(Chars: PChar; Len: Integer; out Value: QWord): Boolean;
var
  P: PChar;
  L: PChar;
begin
  P := PChar(Chars);
  if Len >= 0 then
  begin
    L := P + Len;
    Result := Assigned(P) and __parseNonNegativeInteger(P, L, Value) and (P = L)
  end else
    Result := Assigned(P) and __parseNonNegativeInteger(P, XSD_IGNORE_LAST, Value) and (P^ = #0);
end;

function xsdTryParseNonPositiveInteger(Chars: PChar; Len: Integer; out Value: Int64): Boolean;
begin
  Result := xsdTryParseInteger(Chars, Len, Value) and (Value <= 0);
end;

function xsdTryParseNegativeInteger(Chars: PChar; Len: Integer; out Value: Int64): Boolean;
begin
  Result := xsdTryParseInteger(Chars, Len, Value) and (Value <= -1);
end;

function xsdTryParsePositiveInteger(Chars: PChar; Len: Integer; out Value: QWord): Boolean;
begin
  Result := xsdTryParseNonNegativeInteger(Chars, Len, Value) and (Value >= 1);
end;

function xsdTryParseByte(Chars: PChar; Len: Integer; out Value: Shortint): Boolean;
var
  Tmp: Int64;
begin
  Result := xsdTryParseInteger(Chars, Len, Tmp) and (Tmp <= 128) and (Tmp >= -127);
  Value := Tmp;
end;

function xsdTryParseShort(Chars: PChar; Len: Integer; out Value: Smallint): Boolean;
var
  Tmp: Int64;
begin
  Result := xsdTryParseInteger(Chars, Len, Tmp) and (Tmp <= 32767) and (Tmp >= -32768);
  Value := Tmp;
end;

function xsdTryParseInt(Chars: PChar; Len: Integer; out Value: Longint): Boolean;
var
  Tmp: Int64;
begin
  Result := xsdTryParseInteger(Chars, Len, Tmp) and (Tmp <= 2147483647) and (Tmp >= -2147483648);
  Value := Tmp;
end;

function xsdTryParseLong(Chars: PChar; Len: Integer; out Value: Int64): Boolean;
begin
  Result := xsdTryParseInteger(Chars, Len, Value);
end;

function xsdTryParseUnsignedByte(Chars: PChar; Len: Integer; out Value: Byte): Boolean;
var
  Tmp: QWord;
begin
  Result := xsdTryParseNonNegativeInteger(Chars, Len, Tmp) and (Tmp <= 255);
  Value := Tmp;
end;

function xsdTryParseUnsignedShort(Chars: PChar; Len: Integer; out Value: Word): Boolean;
var
  Tmp: QWord;
begin
  Result := xsdTryParseNonNegativeInteger(Chars, Len, Tmp) and (Tmp <= 65535);
  Value := Tmp;
end;

function xsdTryParseUnsignedInt(Chars: PChar; Len: Integer; out Value: Longword): Boolean;
var
  Tmp: QWord;
begin
  Result := xsdTryParseNonNegativeInteger(Chars, Len, Tmp) and (Tmp <= 4294967295);
  Value := Tmp;
end;

function xsdTryParseUnsignedLong(Chars: PChar; Len: Integer; out Value: QWord): Boolean;
begin
  Result := xsdTryParseNonNegativeInteger(Chars, Len, Value)
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdTryParseEnum(Chars: PChar; Len: Integer; enum: array of AnsiString; out Value: Integer): Boolean;
var
  Temp: AnsiString;
=======
function xsdTryParseEnum(Chars: PChar; Len: Integer; enum: array of Utf8String; out Value: Integer): Boolean;
var
  Temp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseEnum(Chars: PChar; Len: Integer; enum: array of Utf8String; out Value: Integer): Boolean;
var
  Temp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseEnum(Chars: PChar; Len: Integer; enum: array of Utf8String; out Value: Integer): Boolean;
var
  Temp: Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseEnum(Chars: PChar; Len: Integer; enum: array of Utf8String; out Value: Integer): Boolean;
var
  Temp: Utf8String;
>>>>>>> origin/cpstrnew
  I: Integer;
begin
  Temp := '';
  Result := xsdTryParseString(Chars, Len, Temp);
  if Result then
  begin
    for I := 0 to High(enum) do
      if Temp = enum[I] then
      begin
        Value := I;
        Exit(True);
      end;
    Result := False;
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdParseStringDef(Chars: PChar; Len: Integer; Default: AnsiString): AnsiString;
=======
function xsdParseStringDef(Chars: PChar; Len: Integer; Default: Utf8String): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseStringDef(Chars: PChar; Len: Integer; Default: Utf8String): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseStringDef(Chars: PChar; Len: Integer; Default: Utf8String): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseStringDef(Chars: PChar; Len: Integer; Default: Utf8String): Utf8String;
>>>>>>> origin/cpstrnew
begin
  if not xsdTryParseString(Chars, Len, Result) then
    Result := Default;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdParseStringLowerDef(Chars: PChar; Len: Integer; Default: AnsiString): AnsiString;
=======
function xsdParseStringLowerDef(Chars: PChar; Len: Integer; Default: Utf8String): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseStringLowerDef(Chars: PChar; Len: Integer; Default: Utf8String): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseStringLowerDef(Chars: PChar; Len: Integer; Default: Utf8String): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseStringLowerDef(Chars: PChar; Len: Integer; Default: Utf8String): Utf8String;
>>>>>>> origin/cpstrnew
begin
  if not xsdTryParseStringLower(Chars, Len, Result) then
    Result := Default;
end;

function xsdParseBooleanDef(Chars: PChar; Len: Integer; Default: Boolean): Boolean;
begin
  if not xsdTryParseBoolean(Chars, Len, Result) then
    Result := Default;
end;

function xsdParseDateDef(Chars: PChar; Len: Integer; Default: TDateTime; Timezone: PXsdTimezone): TDateTime;
begin
  if not xsdTryParseDate(Chars, Len, Result, Timezone) then
    Result := Default;
end;

function xsdParseTimeDef(Chars: PChar; Len: Integer; Default: TDateTime; Timezone: PXsdTimezone): TDateTime;
begin
  if not xsdTryParseTime(Chars, Len, Result, Timezone) then
    Result := Default;
end;

function xsdParseDateTimeDef(Chars: PChar; Len: Integer; Default: TDateTime; Timezone: PXsdTimezone): TDateTime;
begin
  if not xsdTryParseDateTime(Chars, Len, Result, Timezone) then
    Result := Default;
end;

function xsdParseDecimalDef(Chars: PChar; Len: Integer; Default: Extended): Extended;
begin
  if not xsdTryParseDecimal(Chars, Len, Result) then
    Result := Default;
end;

function xsdParseDoubleDef(Chars: PChar; Len: Integer; Default: Double): Double;
begin
  if not xsdTryParseDouble(Chars, Len, Result) then
    Result := Default;
end;

function xsdParseFloatDef(Chars: PChar; Len: Integer; Default: Single): Single;
begin
  if not xsdTryParseFloat(Chars, Len, Result) then
    Result := Default;
end;

function xsdParseIntegerDef(Chars: PChar; Len: Integer; Default: Int64): Int64;
begin
  if not xsdTryParseInteger(Chars, Len, Result) then
    Result := Default;
end;

function xsdParseNonNegativeIntegerDef(Chars: PChar; Len: Integer; Default: QWord): QWord;
begin
  if not xsdTryParseNonNegativeInteger(Chars, Len, Result) then
    Result := Default;
end;

function xsdParseNonPositiveIntegerDef(Chars: PChar; Len: Integer; Default: Int64): Int64;
begin
  if not xsdTryParseNonPositiveInteger(Chars, Len, Result) then
    Result := Default;
end;

function xsdParseNegativeIntegerDef(Chars: PChar; Len: Integer; Default: Int64): Int64;
begin
  if not xsdTryParseNegativeInteger(Chars, Len, Result) then
    Result := Default;
end;

function xsdParsePositiveIntegerDef(Chars: PChar; Len: Integer; Default: QWord): QWord;
begin
  if not xsdTryParsePositiveInteger(Chars, Len, Result) then
    Result := Default;
end;

function xsdParseByteDef(Chars: PChar; Len: Integer; Default: Shortint): Shortint;
begin
  if not xsdTryParseByte(Chars, Len, Result) then
    Result := Default;
end;

function xsdParseShortDef(Chars: PChar; Len: Integer; Default: Smallint): Smallint;
begin
  if not xsdTryParseShort(Chars, Len, Result) then
    Result := Default;
end;

function xsdParseIntDef(Chars: PChar; Len: Integer; Default: Longint): Longint;
begin
  if not xsdTryParseInt(Chars, Len, Result) then
    Result := Default;
end;

function xsdParseLongDef(Chars: PChar; Len: Integer; Default: Int64): Int64;
begin
  if not xsdTryParseLong(Chars, Len, Result) then
    Result := Default;
end;

function xsdParseUnsignedByteDef(Chars: PChar; Len: Integer; Default: Byte): Byte;
begin
  if not xsdTryParseUnsignedByte(Chars, Len, Result) then
    Result := Default;
end;

function xsdParseUnsignedShortDef(Chars: PChar; Len: Integer; Default: Word): Word;
begin
  if not xsdTryParseUnsignedShort(Chars, Len, Result) then
    Result := Default;
end;

function xsdParseUnsignedIntDef(Chars: PChar; Len: Integer; Default: Longword): Longword;
begin
  if not xsdTryParseUnsignedInt(Chars, Len, Result) then
    Result := Default;
end;

function xsdParseUnsignedLongDef(Chars: PChar; Len: Integer; Default: QWord): QWord;
begin
  if not xsdTryParseUnsignedLong(Chars, Len, Result) then
    Result := Default;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdParseEnumDef(Chars: PChar; Len: Integer; enum: array of AnsiString; Default: Integer): Integer;
=======
function xsdParseEnumDef(Chars: PChar; Len: Integer; enum: array of Utf8String; Default: Integer): Integer;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseEnumDef(Chars: PChar; Len: Integer; enum: array of Utf8String; Default: Integer): Integer;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseEnumDef(Chars: PChar; Len: Integer; enum: array of Utf8String; Default: Integer): Integer;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseEnumDef(Chars: PChar; Len: Integer; enum: array of Utf8String; Default: Integer): Integer;
>>>>>>> origin/cpstrnew
begin
  if not xsdTryParseEnum(Chars, Len, enum, Result) then
    Result := Default;
end;

procedure xsdParseBase64(Chars: PChar; Len: Integer; const Value: TStream);
begin
  if not xsdTryParseBase64(Chars, Len, Value) then
    raise EConvertError.CreateFmt(SXsdParserError, [__strpas(Chars,Len), 'xs:base64Binary']);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure xsdParseString(Chars: PChar; Len: Integer; out Value: AnsiString);
=======
procedure xsdParseString(Chars: PChar; Len: Integer; out Value: Utf8String);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseString(Chars: PChar; Len: Integer; out Value: Utf8String);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseString(Chars: PChar; Len: Integer; out Value: Utf8String);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseString(Chars: PChar; Len: Integer; out Value: Utf8String);
>>>>>>> origin/cpstrnew
begin
  if not xsdTryParseString(Chars, Len, Value) then
    raise EConvertError.CreateFmt(SXsdParserError, [__strpas(Chars,Len), 'xs:string']);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure xsdParseStringLower(Chars: PChar; Len: Integer; out Value: AnsiString);
=======
procedure xsdParseStringLower(Chars: PChar; Len: Integer; out Value: Utf8String);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseStringLower(Chars: PChar; Len: Integer; out Value: Utf8String);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseStringLower(Chars: PChar; Len: Integer; out Value: Utf8String);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseStringLower(Chars: PChar; Len: Integer; out Value: Utf8String);
>>>>>>> origin/cpstrnew
begin
  if not xsdTryParseStringLower(Chars, Len, Value) then
    raise EConvertError.CreateFmt(SXsdParserError, [__strpas(Chars,Len), 'xs:string']);
end;

procedure xsdParseBoolean(Chars: PChar; Len: Integer; out Value: Boolean);
begin
  if not xsdTryParseBoolean(Chars, Len, Value) then
    raise EConvertError.CreateFmt(SXsdParserError, [__strpas(Chars,Len), 'xs:boolean']);
end;

procedure xsdParseDate(Chars: PChar; Len: Integer; out Year, Month, Day: Longword; Timezone: PXsdTimezone; BC: PBoolean);
begin
  if not xsdTryParseDate(Chars, Len, Year, Month, Day, Timezone, BC) then
    raise EConvertError.CreateFmt(SXsdParserError, [__strpas(Chars,Len), 'xs:date']);
end;

procedure xsdParseDate(Chars: PChar; Len: Integer; out Value: TDateTime; Timezone: PXsdTimezone);
begin
  if not xsdTryParseDate(Chars, Len, Value, Timezone) then
    raise EConvertError.CreateFmt(SXsdParserError, [__strpas(Chars,Len), 'xs:date']);
end;

procedure xsdParseTime(Chars: PChar; Len: Integer; out Hour, Minute, Second, Milliseconds: Longword; Timezone: PXsdTimezone);
begin
  if not xsdTryParseTime(Chars, Len, Hour, Minute, Second, Milliseconds, Timezone) then
    raise EConvertError.CreateFmt(SXsdParserError, [__strpas(Chars,Len), 'xs:time']);
end;

procedure xsdParseTime(Chars: PChar; Len: Integer; out Value: TDateTime; Timezone: PXsdTimezone);
begin
  if not xsdTryParseTime(Chars, Len, Value, Timezone) then
    raise EConvertError.CreateFmt(SXsdParserError, [__strpas(Chars,Len), 'xs:time']);
end;

procedure xsdParseDateTime(Chars: PChar; Len: Integer; out Year, Month, Day, Hour, Minute, Second, Milliseconds: Longword; Timezone: PXsdTimezone; BC: PBoolean);
begin
  if not xsdTryParseDateTime(Chars, Len, Year, Month, Day, Hour, Minute, Second, Milliseconds, Timezone, BC) then
    raise EConvertError.CreateFmt(SXsdParserError, [__strpas(Chars,Len), 'xs:dateTime']);
end;

procedure xsdParseDateTime(Chars: PChar; Len: Integer; out Value: TDateTime; Timezone: PXsdTimezone);
begin
  if not xsdTryParseDateTime(Chars, Len, Value, Timezone) then
    raise EConvertError.CreateFmt(SXsdParserError, [__strpas(Chars,Len), 'xs:dateTime']);
end;

procedure xsdParseDecimal(Chars: PChar; Len: Integer; out Value: Extended);
begin
  if not xsdTryParseDecimal(Chars, Len, Value) then
    raise EConvertError.CreateFmt(SXsdParserError, [__strpas(Chars,Len), 'xs:decimal']);
end;

procedure xsdParseDouble(Chars: PChar; Len: Integer; out Value: Double);
begin
  if not xsdTryParseDouble(Chars, Len, Value) then
    raise EConvertError.CreateFmt(SXsdParserError, [__strpas(Chars,Len), 'xs:double']);
end;

procedure xsdParseFloat(Chars: PChar; Len: Integer; out Value: Single);
begin
  if not xsdTryParseFloat(Chars, Len, Value) then
    raise EConvertError.CreateFmt(SXsdParserError, [__strpas(Chars,Len), 'xs:float']);
end;

procedure xsdParseInteger(Chars: PChar; Len: Integer; out Value: Int64);
begin
  if not xsdTryParseInteger(Chars, Len, Value) then
    raise EConvertError.CreateFmt(SXsdParserError, [__strpas(Chars,Len), 'xs:integer']);
end;

procedure xsdParseNonNegativeInteger(Chars: PChar; Len: Integer; out Value: QWord);
begin
  if not xsdTryParseNonNegativeInteger(Chars, Len, Value) then
    raise EConvertError.CreateFmt(SXsdParserError, [__strpas(Chars,Len), 'xs:nonNegativeInteger']);
end;

procedure xsdParseNonPositiveInteger(Chars: PChar; Len: Integer; out Value: Int64);
begin
  if not xsdTryParseNonPositiveInteger(Chars, Len, Value) then
    raise EConvertError.CreateFmt(SXsdParserError, [__strpas(Chars,Len), 'xs:nonPositiveInteger']);
end;

procedure xsdParseNegativeInteger(Chars: PChar; Len: Integer; out Value: Int64);
begin
  if not xsdTryParseNegativeInteger(Chars, Len, Value) then
    raise EConvertError.CreateFmt(SXsdParserError, [__strpas(Chars,Len), 'xs:negativeInteger']);
end;

procedure xsdParsePositiveInteger(Chars: PChar; Len: Integer; out Value: QWord);
begin
  if not xsdTryParsePositiveInteger(Chars, Len, Value) then
    raise EConvertError.CreateFmt(SXsdParserError, [__strpas(Chars,Len), 'xs:positiveInteger']);
end;

procedure xsdParseByte(Chars: PChar; Len: Integer; out Value: Shortint);
begin
  if not xsdTryParseByte(Chars, Len, Value) then
    raise EConvertError.CreateFmt(SXsdParserError, [__strpas(Chars,Len), 'xs:byte']);
end;

procedure xsdParseShort(Chars: PChar; Len: Integer; out Value: Smallint);
begin
  if not xsdTryParseShort(Chars, Len, Value) then
    raise EConvertError.CreateFmt(SXsdParserError, [__strpas(Chars,Len), 'xs:short']);
end;

procedure xsdParseInt(Chars: PChar; Len: Integer; out Value: Longint);
begin
  if not xsdTryParseInt(Chars, Len, Value) then
    raise EConvertError.CreateFmt(SXsdParserError, [__strpas(Chars,Len), 'xs:int']);
end;

procedure xsdParseLong(Chars: PChar; Len: Integer; out Value: Int64);
begin
  if not xsdTryParseLong(Chars, Len, Value) then
    raise EConvertError.CreateFmt(SXsdParserError, [__strpas(Chars,Len), 'xs:long']);
end;

procedure xsdParseUnsignedByte(Chars: PChar; Len: Integer; out Value: Byte);
begin
  if not xsdTryParseUnsignedByte(Chars, Len, Value) then
    raise EConvertError.CreateFmt(SXsdParserError, [__strpas(Chars,Len), 'xs:unsignedByte']);
end;

procedure xsdParseUnsignedShort(Chars: PChar; Len: Integer; out Value: Word);
begin
  if not xsdTryParseUnsignedShort(Chars, Len, Value) then
    raise EConvertError.CreateFmt(SXsdParserError, [__strpas(Chars,Len), 'xs:unsignedShort']);
end;

procedure xsdParseUnsignedInt(Chars: PChar; Len: Integer; out Value: Longword);
begin
  if not xsdTryParseUnsignedInt(Chars, Len, Value) then
    raise EConvertError.CreateFmt(SXsdParserError, [__strpas(Chars,Len), 'xs:unsignedInt']);
end;

procedure xsdParseUnsignedLong(Chars: PChar; Len: Integer; out Value: QWord);
begin
  if not xsdTryParseUnsignedLong(Chars, Len, Value) then
    raise EConvertError.CreateFmt(SXsdParserError, [__strpas(Chars,Len), 'xs:unsignedLong']);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure xsdParseEnum(Chars: PChar; Len: Integer; enum: array of AnsiString; out Value: Integer);
=======
procedure xsdParseEnum(Chars: PChar; Len: Integer; enum: array of Utf8String; out Value: Integer);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseEnum(Chars: PChar; Len: Integer; enum: array of Utf8String; out Value: Integer);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseEnum(Chars: PChar; Len: Integer; enum: array of Utf8String; out Value: Integer);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseEnum(Chars: PChar; Len: Integer; enum: array of Utf8String; out Value: Integer);
>>>>>>> origin/cpstrnew
begin
  if not xsdTryParseEnum(Chars, Len, enum, Value) then
    raise EConvertError.CreateFmt(SXsdParserError, [__strpas(Chars,Len), 'xs:enum']);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdParseString(Chars: PChar; Len: Integer): AnsiString;
=======
function xsdParseString(Chars: PChar; Len: Integer): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseString(Chars: PChar; Len: Integer): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseString(Chars: PChar; Len: Integer): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseString(Chars: PChar; Len: Integer): Utf8String;
>>>>>>> origin/cpstrnew
begin
  xsdParseString(Chars, Len, Result);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdParseStringLower(Chars: PChar; Len: Integer): AnsiString;
=======
function xsdParseStringLower(Chars: PChar; Len: Integer): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseStringLower(Chars: PChar; Len: Integer): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseStringLower(Chars: PChar; Len: Integer): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseStringLower(Chars: PChar; Len: Integer): Utf8String;
>>>>>>> origin/cpstrnew
begin
  xsdParseStringLower(Chars, Len, Result);
end;

function xsdParseBoolean(Chars: PChar; Len: Integer): Boolean;
begin
  xsdParseBoolean(Chars, Len, Result);
end;

function xsdParseDate(Chars: PChar; Len: Integer; Timezone: PXsdTimezone): TDateTime;
begin
  xsdParseDate(Chars, Len, Result, Timezone);
end;

function xsdParseTime(Chars: PChar; Len: Integer; Timezone: PXsdTimezone): TDateTime;
begin
  xsdParseTime(Chars, Len, Result, Timezone);
end;

function xsdParseDateTime(Chars: PChar; Len: Integer; Timezone: PXsdTimezone): TDateTime;
begin
  xsdParseDateTime(Chars, Len, Result, Timezone);
end;

function xsdParseDecimal(Chars: PChar; Len: Integer): Extended;
begin
  xsdParseDecimal(Chars, Len, Result);
end;

function xsdParseDouble(Chars: PChar; Len: Integer): Double;
begin
  xsdParseDouble(Chars, Len, Result);
end;

function xsdParseFloat(Chars: PChar; Len: Integer): Single;
begin
  xsdParseFloat(Chars, Len, Result);
end;

function xsdParseInteger(Chars: PChar; Len: Integer): Int64;
begin
  xsdParseInteger(Chars, Len, Result);
end;

function xsdParseNonNegativeInteger(Chars: PChar; Len: Integer): QWord;
begin
  xsdParseNonNegativeInteger(Chars, Len, Result);
end;

function xsdParseNonPositiveInteger(Chars: PChar; Len: Integer): Int64;
begin
  xsdParseNonPositiveInteger(Chars, Len, Result);
end;

function xsdParseNegativeInteger(Chars: PChar; Len: Integer): Int64;
begin
  xsdParseNegativeInteger(Chars, Len, Result);
end;

function xsdParsePositiveInteger(Chars: PChar; Len: Integer): QWord;
begin
  xsdParsePositiveInteger(Chars, Len, Result);
end;

function xsdParseByte(Chars: PChar; Len: Integer): Shortint;
begin
  xsdParseByte(Chars, Len, Result);
end;

function xsdParseShort(Chars: PChar; Len: Integer): Smallint;
begin
  xsdParseShort(Chars, Len, Result);
end;

function xsdParseInt(Chars: PChar; Len: Integer): Longint;
begin
  xsdParseInt(Chars, Len, Result);
end;

function xsdParseLong(Chars: PChar; Len: Integer): Int64;
begin
  xsdParseLong(Chars, Len, Result);
end;

function xsdParseUnsignedByte(Chars: PChar; Len: Integer): Byte;
begin
  xsdParseUnsignedByte(Chars, Len, Result);
end;

function xsdParseUnsignedShort(Chars: PChar; Len: Integer): Word;
begin
  xsdParseUnsignedShort(Chars, Len, Result);
end;

function xsdParseUnsignedInt(Chars: PChar; Len: Integer): Longword;
begin
  xsdParseUnsignedInt(Chars, Len, Result);
end;

function xsdParseUnsignedLong(Chars: PChar; Len: Integer): QWord;
begin
  xsdParseUnsignedLong(Chars, Len, Result);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdParseEnum(Chars: PChar; Len: Integer; enum: array of AnsiString): Integer;
=======
function xsdParseEnum(Chars: PChar; Len: Integer; enum: array of Utf8String): Integer;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseEnum(Chars: PChar; Len: Integer; enum: array of Utf8String): Integer;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseEnum(Chars: PChar; Len: Integer; enum: array of Utf8String): Integer;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseEnum(Chars: PChar; Len: Integer; enum: array of Utf8String): Integer;
>>>>>>> origin/cpstrnew
begin
  xsdParseEnum(Chars, Len, enum, Result);
end;
(*
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdTryParseBase64(const S: AnsiString; const Value: TStream): Boolean;
=======
function xsdTryParseBase64(const S: UTF8String; const Value: TStream): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseBase64(const S: UTF8String; const Value: TStream): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseBase64(const S: UTF8String; const Value: TStream): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseBase64(const S: UTF8String; const Value: TStream): Boolean;
>>>>>>> origin/cpstrnew
begin

end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdTryParseString(const S: AnsiString; out Value: AnsiString): Boolean;
=======
function xsdTryParseString(const S: UTF8String; out Value: Utf8String): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseString(const S: UTF8String; out Value: Utf8String): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseString(const S: UTF8String; out Value: Utf8String): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseString(const S: UTF8String; out Value: Utf8String): Boolean;
>>>>>>> origin/cpstrnew
begin

end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdTryParseStringLower(const S: AnsiString; out Value: AnsiString
=======
function xsdTryParseStringLower(const S: UTF8String; out Value: Utf8String
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseStringLower(const S: UTF8String; out Value: Utf8String
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseStringLower(const S: UTF8String; out Value: Utf8String
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseStringLower(const S: UTF8String; out Value: Utf8String
>>>>>>> origin/cpstrnew
  ): Boolean;
begin

end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdTryParseBoolean(const S: AnsiString; out Value: Boolean): Boolean;
=======
function xsdTryParseBoolean(const S: UTF8String; out Value: Boolean): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseBoolean(const S: UTF8String; out Value: Boolean): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseBoolean(const S: UTF8String; out Value: Boolean): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseBoolean(const S: UTF8String; out Value: Boolean): Boolean;
>>>>>>> origin/cpstrnew
begin

end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdTryParseDate(const S: AnsiString; out Year, Month, Day: Longword;
=======
function xsdTryParseDate(const S: UTF8String; out Year, Month, Day: Longword;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseDate(const S: UTF8String; out Year, Month, Day: Longword;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseDate(const S: UTF8String; out Year, Month, Day: Longword;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseDate(const S: UTF8String; out Year, Month, Day: Longword;
>>>>>>> origin/cpstrnew
  Timezone: PXsdTimezone; BC: PBoolean): Boolean;
begin

end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdTryParseDate(const S: AnsiString; out Value: TDateTime;
=======
function xsdTryParseDate(const S: UTF8String; out Value: TDateTime;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseDate(const S: UTF8String; out Value: TDateTime;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseDate(const S: UTF8String; out Value: TDateTime;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseDate(const S: UTF8String; out Value: TDateTime;
>>>>>>> origin/cpstrnew
  Timezone: PXsdTimezone): Boolean;
begin

end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdTryParseTime(const S: AnsiString; out Hour, Minute, Second,
=======
function xsdTryParseTime(const S: UTF8String; out Hour, Minute, Second,
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseTime(const S: UTF8String; out Hour, Minute, Second,
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseTime(const S: UTF8String; out Hour, Minute, Second,
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseTime(const S: UTF8String; out Hour, Minute, Second,
>>>>>>> origin/cpstrnew
  Milliseconds: Longword; Timezone: PXsdTimezone): Boolean;
begin

end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdTryParseTime(const S: AnsiString; out Value: TDateTime;
=======
function xsdTryParseTime(const S: UTF8String; out Value: TDateTime;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseTime(const S: UTF8String; out Value: TDateTime;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseTime(const S: UTF8String; out Value: TDateTime;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseTime(const S: UTF8String; out Value: TDateTime;
>>>>>>> origin/cpstrnew
  Timezone: PXsdTimezone): Boolean;
begin

end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdTryParseDateTime(const S: AnsiString; out Year, Month, Day, Hour,
=======
function xsdTryParseDateTime(const S: UTF8String; out Year, Month, Day, Hour,
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseDateTime(const S: UTF8String; out Year, Month, Day, Hour,
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseDateTime(const S: UTF8String; out Year, Month, Day, Hour,
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseDateTime(const S: UTF8String; out Year, Month, Day, Hour,
>>>>>>> origin/cpstrnew
  Minute, Second, Milliseconds: Longword; Timezone: PXsdTimezone; BC: PBoolean
  ): Boolean;
begin

end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdTryParseDateTime(const S: AnsiString; out Value: TDateTime;
=======
function xsdTryParseDateTime(const S: UTF8String; out Value: TDateTime;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseDateTime(const S: UTF8String; out Value: TDateTime;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseDateTime(const S: UTF8String; out Value: TDateTime;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseDateTime(const S: UTF8String; out Value: TDateTime;
>>>>>>> origin/cpstrnew
  Timezone: PXsdTimezone): Boolean;
begin

end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdTryParseDecimal(const S: AnsiString; out Value: Extended): Boolean;
=======
function xsdTryParseDecimal(const S: UTF8String; out Value: Extended): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseDecimal(const S: UTF8String; out Value: Extended): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseDecimal(const S: UTF8String; out Value: Extended): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseDecimal(const S: UTF8String; out Value: Extended): Boolean;
>>>>>>> origin/cpstrnew
begin

end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdTryParseDouble(const S: AnsiString; out Value: Double): Boolean;
=======
function xsdTryParseDouble(const S: UTF8String; out Value: Double): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseDouble(const S: UTF8String; out Value: Double): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseDouble(const S: UTF8String; out Value: Double): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseDouble(const S: UTF8String; out Value: Double): Boolean;
>>>>>>> origin/cpstrnew
begin

end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdTryParseFloat(const S: AnsiString; out Value: Single): Boolean;
=======
function xsdTryParseFloat(const S: UTF8String; out Value: Single): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseFloat(const S: UTF8String; out Value: Single): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseFloat(const S: UTF8String; out Value: Single): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseFloat(const S: UTF8String; out Value: Single): Boolean;
>>>>>>> origin/cpstrnew
begin

end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdTryParseInteger(const S: AnsiString; out Value: Int64): Boolean;
=======
function xsdTryParseInteger(const S: UTF8String; out Value: Int64): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseInteger(const S: UTF8String; out Value: Int64): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseInteger(const S: UTF8String; out Value: Int64): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseInteger(const S: UTF8String; out Value: Int64): Boolean;
>>>>>>> origin/cpstrnew
begin

end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdTryParseNonNegativeInteger(const S: AnsiString; out Value: QWord
=======
function xsdTryParseNonNegativeInteger(const S: UTF8String; out Value: QWord
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseNonNegativeInteger(const S: UTF8String; out Value: QWord
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseNonNegativeInteger(const S: UTF8String; out Value: QWord
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseNonNegativeInteger(const S: UTF8String; out Value: QWord
>>>>>>> origin/cpstrnew
  ): Boolean;
begin

end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdTryParseNonPositiveInteger(const S: AnsiString; out Value: Int64
=======
function xsdTryParseNonPositiveInteger(const S: UTF8String; out Value: Int64
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseNonPositiveInteger(const S: UTF8String; out Value: Int64
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseNonPositiveInteger(const S: UTF8String; out Value: Int64
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseNonPositiveInteger(const S: UTF8String; out Value: Int64
>>>>>>> origin/cpstrnew
  ): Boolean;
begin

end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdTryParseNegativeInteger(const S: AnsiString; out Value: Int64
=======
function xsdTryParseNegativeInteger(const S: UTF8String; out Value: Int64
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseNegativeInteger(const S: UTF8String; out Value: Int64
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseNegativeInteger(const S: UTF8String; out Value: Int64
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseNegativeInteger(const S: UTF8String; out Value: Int64
>>>>>>> origin/cpstrnew
  ): Boolean;
begin

end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdTryParsePositiveInteger(const S: AnsiString; out Value: QWord
=======
function xsdTryParsePositiveInteger(const S: UTF8String; out Value: QWord
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParsePositiveInteger(const S: UTF8String; out Value: QWord
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParsePositiveInteger(const S: UTF8String; out Value: QWord
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParsePositiveInteger(const S: UTF8String; out Value: QWord
>>>>>>> origin/cpstrnew
  ): Boolean;
begin

end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdTryParseByte(const S: AnsiString; out Value: Shortint): Boolean;
=======
function xsdTryParseByte(const S: UTF8String; out Value: Shortint): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseByte(const S: UTF8String; out Value: Shortint): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseByte(const S: UTF8String; out Value: Shortint): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseByte(const S: UTF8String; out Value: Shortint): Boolean;
>>>>>>> origin/cpstrnew
begin

end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdTryParseShort(const S: AnsiString; out Value: Smallint): Boolean;
=======
function xsdTryParseShort(const S: UTF8String; out Value: Smallint): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseShort(const S: UTF8String; out Value: Smallint): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseShort(const S: UTF8String; out Value: Smallint): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseShort(const S: UTF8String; out Value: Smallint): Boolean;
>>>>>>> origin/cpstrnew
begin

end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdTryParseInt(const S: AnsiString; out Value: Longint): Boolean;
=======
function xsdTryParseInt(const S: UTF8String; out Value: Longint): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseInt(const S: UTF8String; out Value: Longint): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseInt(const S: UTF8String; out Value: Longint): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseInt(const S: UTF8String; out Value: Longint): Boolean;
>>>>>>> origin/cpstrnew
begin

end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdTryParseLong(const S: AnsiString; out Value: Int64): Boolean;
=======
function xsdTryParseLong(const S: UTF8String; out Value: Int64): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseLong(const S: UTF8String; out Value: Int64): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseLong(const S: UTF8String; out Value: Int64): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseLong(const S: UTF8String; out Value: Int64): Boolean;
>>>>>>> origin/cpstrnew
begin

end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdTryParseUnsignedByte(const S: AnsiString; out Value: Byte): Boolean;
=======
function xsdTryParseUnsignedByte(const S: UTF8String; out Value: Byte): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseUnsignedByte(const S: UTF8String; out Value: Byte): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseUnsignedByte(const S: UTF8String; out Value: Byte): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseUnsignedByte(const S: UTF8String; out Value: Byte): Boolean;
>>>>>>> origin/cpstrnew
begin

end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdTryParseUnsignedShort(const S: AnsiString; out Value: Word
=======
function xsdTryParseUnsignedShort(const S: UTF8String; out Value: Word
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseUnsignedShort(const S: UTF8String; out Value: Word
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseUnsignedShort(const S: UTF8String; out Value: Word
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseUnsignedShort(const S: UTF8String; out Value: Word
>>>>>>> origin/cpstrnew
  ): Boolean;
begin

end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdTryParseUnsignedInt(const S: AnsiString; out Value: Longword
=======
function xsdTryParseUnsignedInt(const S: UTF8String; out Value: Longword
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseUnsignedInt(const S: UTF8String; out Value: Longword
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseUnsignedInt(const S: UTF8String; out Value: Longword
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseUnsignedInt(const S: UTF8String; out Value: Longword
>>>>>>> origin/cpstrnew
  ): Boolean;
begin

end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdTryParseUnsignedLong(const S: AnsiString; out Value: QWord
=======
function xsdTryParseUnsignedLong(const S: UTF8String; out Value: QWord
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseUnsignedLong(const S: UTF8String; out Value: QWord
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseUnsignedLong(const S: UTF8String; out Value: QWord
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseUnsignedLong(const S: UTF8String; out Value: QWord
>>>>>>> origin/cpstrnew
  ): Boolean;
begin

end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdTryParseEnum(const S: AnsiString; enum: array of AnsiString;
=======
function xsdTryParseEnum(const S: UTF8String; enum: array of Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseEnum(const S: UTF8String; enum: array of Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseEnum(const S: UTF8String; enum: array of Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdTryParseEnum(const S: UTF8String; enum: array of Utf8String;
>>>>>>> origin/cpstrnew
  out Value: Integer): Boolean;
begin

end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdParseStringDef(const S: AnsiString; Default: AnsiString
  ): AnsiString;
=======
function xsdParseStringDef(const S: UTF8String; Default: Utf8String
  ): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseStringDef(const S: UTF8String; Default: Utf8String
  ): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseStringDef(const S: UTF8String; Default: Utf8String
  ): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseStringDef(const S: UTF8String; Default: Utf8String
  ): Utf8String;
>>>>>>> origin/cpstrnew
begin

end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdParseStringLowerDef(const S: AnsiString; Default: AnsiString
  ): AnsiString;
=======
function xsdParseStringLowerDef(const S: UTF8String; Default: Utf8String
  ): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseStringLowerDef(const S: UTF8String; Default: Utf8String
  ): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseStringLowerDef(const S: UTF8String; Default: Utf8String
  ): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseStringLowerDef(const S: UTF8String; Default: Utf8String
  ): Utf8String;
>>>>>>> origin/cpstrnew
begin

end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdParseBooleanDef(const S: AnsiString; Default: Boolean): Boolean;
=======
function xsdParseBooleanDef(const S: UTF8String; Default: Boolean): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseBooleanDef(const S: UTF8String; Default: Boolean): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseBooleanDef(const S: UTF8String; Default: Boolean): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseBooleanDef(const S: UTF8String; Default: Boolean): Boolean;
>>>>>>> origin/cpstrnew
begin

end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdParseDateDef(const S: AnsiString; Default: TDateTime;
=======
function xsdParseDateDef(const S: UTF8String; Default: TDateTime;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseDateDef(const S: UTF8String; Default: TDateTime;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseDateDef(const S: UTF8String; Default: TDateTime;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseDateDef(const S: UTF8String; Default: TDateTime;
>>>>>>> origin/cpstrnew
  Timezone: PXsdTimezone): TDateTime;
begin

end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdParseTimeDef(const S: AnsiString; Default: TDateTime;
=======
function xsdParseTimeDef(const S: UTF8String; Default: TDateTime;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseTimeDef(const S: UTF8String; Default: TDateTime;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseTimeDef(const S: UTF8String; Default: TDateTime;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseTimeDef(const S: UTF8String; Default: TDateTime;
>>>>>>> origin/cpstrnew
  Timezone: PXsdTimezone): TDateTime;
begin

end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdParseDateTimeDef(const S: AnsiString; Default: TDateTime;
=======
function xsdParseDateTimeDef(const S: UTF8String; Default: TDateTime;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseDateTimeDef(const S: UTF8String; Default: TDateTime;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseDateTimeDef(const S: UTF8String; Default: TDateTime;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseDateTimeDef(const S: UTF8String; Default: TDateTime;
>>>>>>> origin/cpstrnew
  Timezone: PXsdTimezone): TDateTime;
begin

end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdParseDecimalDef(const S: AnsiString; Default: Extended): Extended;
=======
function xsdParseDecimalDef(const S: UTF8String; Default: Extended): Extended;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseDecimalDef(const S: UTF8String; Default: Extended): Extended;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseDecimalDef(const S: UTF8String; Default: Extended): Extended;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseDecimalDef(const S: UTF8String; Default: Extended): Extended;
>>>>>>> origin/cpstrnew
begin

end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdParseDoubleDef(const S: AnsiString; Default: Double): Double;
=======
function xsdParseDoubleDef(const S: UTF8String; Default: Double): Double;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseDoubleDef(const S: UTF8String; Default: Double): Double;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseDoubleDef(const S: UTF8String; Default: Double): Double;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseDoubleDef(const S: UTF8String; Default: Double): Double;
>>>>>>> origin/cpstrnew
begin

end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdParseFloatDef(const S: AnsiString; Default: Single): Single;
=======
function xsdParseFloatDef(const S: UTF8String; Default: Single): Single;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseFloatDef(const S: UTF8String; Default: Single): Single;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseFloatDef(const S: UTF8String; Default: Single): Single;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseFloatDef(const S: UTF8String; Default: Single): Single;
>>>>>>> origin/cpstrnew
begin

end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdParseIntegerDef(const S: AnsiString; Default: Int64): Int64;
=======
function xsdParseIntegerDef(const S: UTF8String; Default: Int64): Int64;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseIntegerDef(const S: UTF8String; Default: Int64): Int64;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseIntegerDef(const S: UTF8String; Default: Int64): Int64;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseIntegerDef(const S: UTF8String; Default: Int64): Int64;
>>>>>>> origin/cpstrnew
begin

end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdParseNonNegativeIntegerDef(const S: AnsiString; Default: QWord
=======
function xsdParseNonNegativeIntegerDef(const S: UTF8String; Default: QWord
>>>>>>> graemeg/cpstrnew
=======
function xsdParseNonNegativeIntegerDef(const S: UTF8String; Default: QWord
>>>>>>> graemeg/cpstrnew
=======
function xsdParseNonNegativeIntegerDef(const S: UTF8String; Default: QWord
>>>>>>> graemeg/cpstrnew
=======
function xsdParseNonNegativeIntegerDef(const S: UTF8String; Default: QWord
>>>>>>> origin/cpstrnew
  ): QWord;
begin

end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdParseNonPositiveIntegerDef(const S: AnsiString; Default: Int64
=======
function xsdParseNonPositiveIntegerDef(const S: UTF8String; Default: Int64
>>>>>>> graemeg/cpstrnew
=======
function xsdParseNonPositiveIntegerDef(const S: UTF8String; Default: Int64
>>>>>>> graemeg/cpstrnew
=======
function xsdParseNonPositiveIntegerDef(const S: UTF8String; Default: Int64
>>>>>>> graemeg/cpstrnew
=======
function xsdParseNonPositiveIntegerDef(const S: UTF8String; Default: Int64
>>>>>>> origin/cpstrnew
  ): Int64;
begin

end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdParseNegativeIntegerDef(const S: AnsiString; Default: Int64): Int64;
=======
function xsdParseNegativeIntegerDef(const S: UTF8String; Default: Int64): Int64;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseNegativeIntegerDef(const S: UTF8String; Default: Int64): Int64;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseNegativeIntegerDef(const S: UTF8String; Default: Int64): Int64;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseNegativeIntegerDef(const S: UTF8String; Default: Int64): Int64;
>>>>>>> origin/cpstrnew
begin

end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdParsePositiveIntegerDef(const S: AnsiString; Default: QWord): QWord;
=======
function xsdParsePositiveIntegerDef(const S: UTF8String; Default: QWord): QWord;
>>>>>>> graemeg/cpstrnew
=======
function xsdParsePositiveIntegerDef(const S: UTF8String; Default: QWord): QWord;
>>>>>>> graemeg/cpstrnew
=======
function xsdParsePositiveIntegerDef(const S: UTF8String; Default: QWord): QWord;
>>>>>>> graemeg/cpstrnew
=======
function xsdParsePositiveIntegerDef(const S: UTF8String; Default: QWord): QWord;
>>>>>>> origin/cpstrnew
begin

end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdParseByteDef(const S: AnsiString; Default: Shortint): Shortint;
=======
function xsdParseByteDef(const S: UTF8String; Default: Shortint): Shortint;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseByteDef(const S: UTF8String; Default: Shortint): Shortint;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseByteDef(const S: UTF8String; Default: Shortint): Shortint;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseByteDef(const S: UTF8String; Default: Shortint): Shortint;
>>>>>>> origin/cpstrnew
begin

end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdParseShortDef(const S: AnsiString; Default: Smallint): Smallint;
=======
function xsdParseShortDef(const S: UTF8String; Default: Smallint): Smallint;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseShortDef(const S: UTF8String; Default: Smallint): Smallint;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseShortDef(const S: UTF8String; Default: Smallint): Smallint;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseShortDef(const S: UTF8String; Default: Smallint): Smallint;
>>>>>>> origin/cpstrnew
begin

end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdParseIntDef(const S: AnsiString; Default: Longint): Longint;
=======
function xsdParseIntDef(const S: UTF8String; Default: Longint): Longint;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseIntDef(const S: UTF8String; Default: Longint): Longint;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseIntDef(const S: UTF8String; Default: Longint): Longint;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseIntDef(const S: UTF8String; Default: Longint): Longint;
>>>>>>> origin/cpstrnew
begin

end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdParseLongDef(const S: AnsiString; Default: Int64): Int64;
=======
function xsdParseLongDef(const S: UTF8String; Default: Int64): Int64;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseLongDef(const S: UTF8String; Default: Int64): Int64;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseLongDef(const S: UTF8String; Default: Int64): Int64;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseLongDef(const S: UTF8String; Default: Int64): Int64;
>>>>>>> origin/cpstrnew
begin

end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdParseUnsignedByteDef(const S: AnsiString; Default: Byte): Byte;
=======
function xsdParseUnsignedByteDef(const S: UTF8String; Default: Byte): Byte;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseUnsignedByteDef(const S: UTF8String; Default: Byte): Byte;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseUnsignedByteDef(const S: UTF8String; Default: Byte): Byte;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseUnsignedByteDef(const S: UTF8String; Default: Byte): Byte;
>>>>>>> origin/cpstrnew
begin

end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdParseUnsignedShortDef(const S: AnsiString; Default: Word): Word;
=======
function xsdParseUnsignedShortDef(const S: UTF8String; Default: Word): Word;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseUnsignedShortDef(const S: UTF8String; Default: Word): Word;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseUnsignedShortDef(const S: UTF8String; Default: Word): Word;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseUnsignedShortDef(const S: UTF8String; Default: Word): Word;
>>>>>>> origin/cpstrnew
begin

end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdParseUnsignedIntDef(const S: AnsiString; Default: Longword
=======
function xsdParseUnsignedIntDef(const S: UTF8String; Default: Longword
>>>>>>> graemeg/cpstrnew
=======
function xsdParseUnsignedIntDef(const S: UTF8String; Default: Longword
>>>>>>> graemeg/cpstrnew
=======
function xsdParseUnsignedIntDef(const S: UTF8String; Default: Longword
>>>>>>> graemeg/cpstrnew
=======
function xsdParseUnsignedIntDef(const S: UTF8String; Default: Longword
>>>>>>> origin/cpstrnew
  ): Longword;
begin

end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdParseUnsignedLongDef(const S: AnsiString; Default: QWord): QWord;
=======
function xsdParseUnsignedLongDef(const S: UTF8String; Default: QWord): QWord;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseUnsignedLongDef(const S: UTF8String; Default: QWord): QWord;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseUnsignedLongDef(const S: UTF8String; Default: QWord): QWord;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseUnsignedLongDef(const S: UTF8String; Default: QWord): QWord;
>>>>>>> origin/cpstrnew
begin

end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdParseEnumDef(const S: AnsiString; enum: array of AnsiString;
=======
function xsdParseEnumDef(const S: UTF8String; enum: array of Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseEnumDef(const S: UTF8String; enum: array of Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseEnumDef(const S: UTF8String; enum: array of Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseEnumDef(const S: UTF8String; enum: array of Utf8String;
>>>>>>> origin/cpstrnew
  Default: Integer): Integer;
begin

end;*)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure xsdParseBase64(const S: AnsiString; const Value: TStream);
=======
procedure xsdParseBase64(const S: UTF8String; const Value: TStream);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseBase64(const S: UTF8String; const Value: TStream);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseBase64(const S: UTF8String; const Value: TStream);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseBase64(const S: UTF8String; const Value: TStream);
>>>>>>> origin/cpstrnew
begin
  xsdParseBase64(PChar(S), Length(S), Value);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure xsdParseString(const S: AnsiString; out Value: AnsiString);
=======
procedure xsdParseString(const S: UTF8String; out Value: Utf8String);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseString(const S: UTF8String; out Value: Utf8String);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseString(const S: UTF8String; out Value: Utf8String);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseString(const S: UTF8String; out Value: Utf8String);
>>>>>>> origin/cpstrnew
begin
  xsdParseString(PChar(S), Length(S), Value);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure xsdParseStringLower(const S: AnsiString; out Value: AnsiString);
=======
procedure xsdParseStringLower(const S: UTF8String; out Value: Utf8String);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseStringLower(const S: UTF8String; out Value: Utf8String);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseStringLower(const S: UTF8String; out Value: Utf8String);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseStringLower(const S: UTF8String; out Value: Utf8String);
>>>>>>> origin/cpstrnew
begin
  xsdParseStringLower(PChar(S), Length(S), Value);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure xsdParseBoolean(const S: AnsiString; out Value: Boolean);
=======
procedure xsdParseBoolean(const S: UTF8String; out Value: Boolean);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseBoolean(const S: UTF8String; out Value: Boolean);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseBoolean(const S: UTF8String; out Value: Boolean);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseBoolean(const S: UTF8String; out Value: Boolean);
>>>>>>> origin/cpstrnew
begin
  xsdParseBoolean(PChar(S), Length(S), Value);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure xsdParseDate(const S: AnsiString; out Year, Month, Day: Longword;
=======
procedure xsdParseDate(const S: UTF8String; out Year, Month, Day: Longword;
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseDate(const S: UTF8String; out Year, Month, Day: Longword;
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseDate(const S: UTF8String; out Year, Month, Day: Longword;
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseDate(const S: UTF8String; out Year, Month, Day: Longword;
>>>>>>> origin/cpstrnew
  Timezone: PXsdTimezone; BC: PBoolean);
begin
  xsdParseDate(PChar(S), Length(S), Year, Month, Day, Timezone, BC);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure xsdParseDate(const S: AnsiString; out Value: TDateTime;
=======
procedure xsdParseDate(const S: UTF8String; out Value: TDateTime;
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseDate(const S: UTF8String; out Value: TDateTime;
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseDate(const S: UTF8String; out Value: TDateTime;
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseDate(const S: UTF8String; out Value: TDateTime;
>>>>>>> origin/cpstrnew
  Timezone: PXsdTimezone);
begin
  xsdParseDate(PChar(S), Length(S), Value);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure xsdParseTime(const S: AnsiString; out Hour, Minute, Second,
=======
procedure xsdParseTime(const S: UTF8String; out Hour, Minute, Second,
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseTime(const S: UTF8String; out Hour, Minute, Second,
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseTime(const S: UTF8String; out Hour, Minute, Second,
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseTime(const S: UTF8String; out Hour, Minute, Second,
>>>>>>> origin/cpstrnew
  Milliseconds: Longword; Timezone: PXsdTimezone);
begin
  xsdParseTime(PChar(S), Length(S), Hour, Minute, Second,
    Milliseconds, Timezone);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure xsdParseTime(const S: AnsiString; out Value: TDateTime;
=======
procedure xsdParseTime(const S: UTF8String; out Value: TDateTime;
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseTime(const S: UTF8String; out Value: TDateTime;
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseTime(const S: UTF8String; out Value: TDateTime;
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseTime(const S: UTF8String; out Value: TDateTime;
>>>>>>> origin/cpstrnew
  Timezone: PXsdTimezone);
begin
  xsdParseTime(PChar(S), Length(S), Value, Timezone);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure xsdParseDateTime(const S: AnsiString; out Year, Month, Day, Hour,
=======
procedure xsdParseDateTime(const S: UTF8String; out Year, Month, Day, Hour,
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseDateTime(const S: UTF8String; out Year, Month, Day, Hour,
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseDateTime(const S: UTF8String; out Year, Month, Day, Hour,
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseDateTime(const S: UTF8String; out Year, Month, Day, Hour,
>>>>>>> origin/cpstrnew
  Minute, Second, Milliseconds: Longword; Timezone: PXsdTimezone; BC: PBoolean);
begin
  xsdParseDateTime(PChar(S), Length(S), Year, Month, Day, Hour,
    Minute, Second, Milliseconds, Timezone, BC);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure xsdParseDateTime(const S: AnsiString; out Value: TDateTime;
=======
procedure xsdParseDateTime(const S: UTF8String; out Value: TDateTime;
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseDateTime(const S: UTF8String; out Value: TDateTime;
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseDateTime(const S: UTF8String; out Value: TDateTime;
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseDateTime(const S: UTF8String; out Value: TDateTime;
>>>>>>> origin/cpstrnew
  Timezone: PXsdTimezone);
begin
  xsdParseDateTime(PChar(S), Length(S), Value);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure xsdParseDecimal(const S: AnsiString; out Value: Extended);
=======
procedure xsdParseDecimal(const S: UTF8String; out Value: Extended);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseDecimal(const S: UTF8String; out Value: Extended);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseDecimal(const S: UTF8String; out Value: Extended);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseDecimal(const S: UTF8String; out Value: Extended);
>>>>>>> origin/cpstrnew
begin
  xsdParseDecimal(PChar(S), Length(S), Value);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure xsdParseDouble(const S: AnsiString; out Value: Double);
=======
procedure xsdParseDouble(const S: UTF8String; out Value: Double);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseDouble(const S: UTF8String; out Value: Double);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseDouble(const S: UTF8String; out Value: Double);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseDouble(const S: UTF8String; out Value: Double);
>>>>>>> origin/cpstrnew
begin
  xsdParseDouble(PChar(S), Length(S), Value);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure xsdParseFloat(const S: AnsiString; out Value: Single);
=======
procedure xsdParseFloat(const S: UTF8String; out Value: Single);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseFloat(const S: UTF8String; out Value: Single);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseFloat(const S: UTF8String; out Value: Single);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseFloat(const S: UTF8String; out Value: Single);
>>>>>>> origin/cpstrnew
begin
  xsdParseFloat(PChar(S), Length(S), Value);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure xsdParseInteger(const S: AnsiString; out Value: Int64);
=======
procedure xsdParseInteger(const S: UTF8String; out Value: Int64);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseInteger(const S: UTF8String; out Value: Int64);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseInteger(const S: UTF8String; out Value: Int64);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseInteger(const S: UTF8String; out Value: Int64);
>>>>>>> origin/cpstrnew
begin
  xsdParseInteger(PChar(S), Length(S), Value);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure xsdParseNonNegativeInteger(const S: AnsiString; out Value: QWord);
=======
procedure xsdParseNonNegativeInteger(const S: UTF8String; out Value: QWord);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseNonNegativeInteger(const S: UTF8String; out Value: QWord);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseNonNegativeInteger(const S: UTF8String; out Value: QWord);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseNonNegativeInteger(const S: UTF8String; out Value: QWord);
>>>>>>> origin/cpstrnew
begin
  xsdParseNonNegativeInteger(PChar(S), Length(S), Value);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure xsdParseNonPositiveInteger(const S: AnsiString; out Value: Int64);
=======
procedure xsdParseNonPositiveInteger(const S: UTF8String; out Value: Int64);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseNonPositiveInteger(const S: UTF8String; out Value: Int64);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseNonPositiveInteger(const S: UTF8String; out Value: Int64);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseNonPositiveInteger(const S: UTF8String; out Value: Int64);
>>>>>>> origin/cpstrnew
begin
  xsdParseNonPositiveInteger(PChar(S), Length(S), Value);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure xsdParseNegativeInteger(const S: AnsiString; out Value: Int64);
=======
procedure xsdParseNegativeInteger(const S: UTF8String; out Value: Int64);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseNegativeInteger(const S: UTF8String; out Value: Int64);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseNegativeInteger(const S: UTF8String; out Value: Int64);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseNegativeInteger(const S: UTF8String; out Value: Int64);
>>>>>>> origin/cpstrnew
begin
  xsdParseNegativeInteger(PChar(S), Length(S), Value);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure xsdParsePositiveInteger(const S: AnsiString; out Value: QWord);
=======
procedure xsdParsePositiveInteger(const S: UTF8String; out Value: QWord);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParsePositiveInteger(const S: UTF8String; out Value: QWord);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParsePositiveInteger(const S: UTF8String; out Value: QWord);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParsePositiveInteger(const S: UTF8String; out Value: QWord);
>>>>>>> origin/cpstrnew
begin
  xsdParsePositiveInteger(PChar(S), Length(S), Value);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure xsdParseByte(const S: AnsiString; out Value: Shortint);
=======
procedure xsdParseByte(const S: UTF8String; out Value: Shortint);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseByte(const S: UTF8String; out Value: Shortint);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseByte(const S: UTF8String; out Value: Shortint);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseByte(const S: UTF8String; out Value: Shortint);
>>>>>>> origin/cpstrnew
begin
  xsdParseByte(PChar(S), Length(S), Value);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure xsdParseShort(const S: AnsiString; out Value: Smallint);
=======
procedure xsdParseShort(const S: UTF8String; out Value: Smallint);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseShort(const S: UTF8String; out Value: Smallint);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseShort(const S: UTF8String; out Value: Smallint);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseShort(const S: UTF8String; out Value: Smallint);
>>>>>>> origin/cpstrnew
begin
  xsdParseShort(PChar(S), Length(S), Value);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure xsdParseInt(const S: AnsiString; out Value: Longint);
=======
procedure xsdParseInt(const S: UTF8String; out Value: Longint);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseInt(const S: UTF8String; out Value: Longint);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseInt(const S: UTF8String; out Value: Longint);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseInt(const S: UTF8String; out Value: Longint);
>>>>>>> origin/cpstrnew
begin
  xsdParseInt(PChar(S), Length(S), Value);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure xsdParseLong(const S: AnsiString; out Value: Int64);
=======
procedure xsdParseLong(const S: UTF8String; out Value: Int64);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseLong(const S: UTF8String; out Value: Int64);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseLong(const S: UTF8String; out Value: Int64);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseLong(const S: UTF8String; out Value: Int64);
>>>>>>> origin/cpstrnew
begin
  xsdParseLong(PChar(S), Length(S), Value);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure xsdParseUnsignedByte(const S: AnsiString; out Value: Byte);
=======
procedure xsdParseUnsignedByte(const S: UTF8String; out Value: Byte);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseUnsignedByte(const S: UTF8String; out Value: Byte);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseUnsignedByte(const S: UTF8String; out Value: Byte);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseUnsignedByte(const S: UTF8String; out Value: Byte);
>>>>>>> origin/cpstrnew
begin
  xsdParseUnsignedByte(PChar(S), Length(S), Value);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure xsdParseUnsignedShort(const S: AnsiString; out Value: Word);
=======
procedure xsdParseUnsignedShort(const S: UTF8String; out Value: Word);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseUnsignedShort(const S: UTF8String; out Value: Word);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseUnsignedShort(const S: UTF8String; out Value: Word);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseUnsignedShort(const S: UTF8String; out Value: Word);
>>>>>>> origin/cpstrnew
begin
  xsdParseUnsignedShort(PChar(S), Length(S), Value);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure xsdParseUnsignedInt(const S: AnsiString; out Value: Longword);
=======
procedure xsdParseUnsignedInt(const S: UTF8String; out Value: Longword);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseUnsignedInt(const S: UTF8String; out Value: Longword);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseUnsignedInt(const S: UTF8String; out Value: Longword);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseUnsignedInt(const S: UTF8String; out Value: Longword);
>>>>>>> origin/cpstrnew
begin
  xsdParseUnsignedInt(PChar(S), Length(S), Value);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure xsdParseUnsignedLong(const S: AnsiString; out Value: QWord);
=======
procedure xsdParseUnsignedLong(const S: UTF8String; out Value: QWord);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseUnsignedLong(const S: UTF8String; out Value: QWord);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseUnsignedLong(const S: UTF8String; out Value: QWord);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseUnsignedLong(const S: UTF8String; out Value: QWord);
>>>>>>> origin/cpstrnew
begin
  xsdParseUnsignedLong(PChar(S), Length(S), Value);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure xsdParseEnum(const S: AnsiString; enum: array of AnsiString; out Value: Integer);
=======
procedure xsdParseEnum(const S: UTF8String; enum: array of Utf8String; out Value: Integer);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseEnum(const S: UTF8String; enum: array of Utf8String; out Value: Integer);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseEnum(const S: UTF8String; enum: array of Utf8String; out Value: Integer);
>>>>>>> graemeg/cpstrnew
=======
procedure xsdParseEnum(const S: UTF8String; enum: array of Utf8String; out Value: Integer);
>>>>>>> origin/cpstrnew
begin
  xsdParseEnum(PChar(S), Length(S), enum, Value);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdParseString(const S: AnsiString): AnsiString;
=======
function xsdParseString(const S: UTF8String): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseString(const S: UTF8String): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseString(const S: UTF8String): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseString(const S: UTF8String): Utf8String;
>>>>>>> origin/cpstrnew
begin
  xsdParseString(PChar(S), Length(S), Result);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdParseStringLower(const S: AnsiString): AnsiString;
=======
function xsdParseStringLower(const S: UTF8String): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseStringLower(const S: UTF8String): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseStringLower(const S: UTF8String): Utf8String;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseStringLower(const S: UTF8String): Utf8String;
>>>>>>> origin/cpstrnew
begin
  xsdParseStringLower(PChar(S), Length(S), Result);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdParseBoolean(const S: AnsiString): Boolean;
=======
function xsdParseBoolean(const S: UTF8String): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseBoolean(const S: UTF8String): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseBoolean(const S: UTF8String): Boolean;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseBoolean(const S: UTF8String): Boolean;
>>>>>>> origin/cpstrnew
begin
  xsdParseBoolean(PChar(S), Length(S), Result);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdParseDate(const S: AnsiString; Timezone: PXsdTimezone): TDateTime;
=======
function xsdParseDate(const S: UTF8String; Timezone: PXsdTimezone): TDateTime;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseDate(const S: UTF8String; Timezone: PXsdTimezone): TDateTime;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseDate(const S: UTF8String; Timezone: PXsdTimezone): TDateTime;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseDate(const S: UTF8String; Timezone: PXsdTimezone): TDateTime;
>>>>>>> origin/cpstrnew
begin
  xsdParseDate(PChar(S), Length(S), Result, Timezone);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdParseTime(const S: AnsiString; Timezone: PXsdTimezone): TDateTime;
=======
function xsdParseTime(const S: UTF8String; Timezone: PXsdTimezone): TDateTime;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseTime(const S: UTF8String; Timezone: PXsdTimezone): TDateTime;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseTime(const S: UTF8String; Timezone: PXsdTimezone): TDateTime;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseTime(const S: UTF8String; Timezone: PXsdTimezone): TDateTime;
>>>>>>> origin/cpstrnew
begin
  xsdParseTime(PChar(S), Length(S), Result, Timezone);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdParseDateTime(const S: AnsiString; Timezone: PXsdTimezone): TDateTime;
=======
function xsdParseDateTime(const S: UTF8String; Timezone: PXsdTimezone): TDateTime;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseDateTime(const S: UTF8String; Timezone: PXsdTimezone): TDateTime;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseDateTime(const S: UTF8String; Timezone: PXsdTimezone): TDateTime;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseDateTime(const S: UTF8String; Timezone: PXsdTimezone): TDateTime;
>>>>>>> origin/cpstrnew
begin
  xsdParseDateTime(PChar(S), Length(S), Result, Timezone);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdParseDecimal(const S: AnsiString): Extended;
=======
function xsdParseDecimal(const S: UTF8String): Extended;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseDecimal(const S: UTF8String): Extended;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseDecimal(const S: UTF8String): Extended;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseDecimal(const S: UTF8String): Extended;
>>>>>>> origin/cpstrnew
begin
  xsdParseDecimal(PChar(S), Length(S), Result);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdParseDouble(const S: AnsiString): Double;
=======
function xsdParseDouble(const S: UTF8String): Double;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseDouble(const S: UTF8String): Double;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseDouble(const S: UTF8String): Double;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseDouble(const S: UTF8String): Double;
>>>>>>> origin/cpstrnew
begin
  xsdParseDouble(PChar(S), Length(S), Result);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdParseFloat(const S: AnsiString): Single;
=======
function xsdParseFloat(const S: UTF8String): Single;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseFloat(const S: UTF8String): Single;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseFloat(const S: UTF8String): Single;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseFloat(const S: UTF8String): Single;
>>>>>>> origin/cpstrnew
begin
  xsdParseFloat(PChar(S), Length(S), Result);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdParseInteger(const S: AnsiString): Int64;
=======
function xsdParseInteger(const S: UTF8String): Int64;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseInteger(const S: UTF8String): Int64;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseInteger(const S: UTF8String): Int64;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseInteger(const S: UTF8String): Int64;
>>>>>>> origin/cpstrnew
begin
  xsdParseInteger(PChar(S), Length(S), Result);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdParseNonNegativeInteger(const S: AnsiString): QWord;
=======
function xsdParseNonNegativeInteger(const S: UTF8String): QWord;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseNonNegativeInteger(const S: UTF8String): QWord;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseNonNegativeInteger(const S: UTF8String): QWord;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseNonNegativeInteger(const S: UTF8String): QWord;
>>>>>>> origin/cpstrnew
begin
  xsdParseNonNegativeInteger(PChar(S), Length(S), Result);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdParseNonPositiveInteger(const S: AnsiString): Int64;
=======
function xsdParseNonPositiveInteger(const S: UTF8String): Int64;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseNonPositiveInteger(const S: UTF8String): Int64;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseNonPositiveInteger(const S: UTF8String): Int64;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseNonPositiveInteger(const S: UTF8String): Int64;
>>>>>>> origin/cpstrnew
begin
  xsdParseNonPositiveInteger(PChar(S), Length(S), Result);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdParseNegativeInteger(const S: AnsiString): Int64;
=======
function xsdParseNegativeInteger(const S: UTF8String): Int64;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseNegativeInteger(const S: UTF8String): Int64;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseNegativeInteger(const S: UTF8String): Int64;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseNegativeInteger(const S: UTF8String): Int64;
>>>>>>> origin/cpstrnew
begin
  xsdParseNegativeInteger(PChar(S), Length(S), Result);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdParsePositiveInteger(const S: AnsiString): QWord;
=======
function xsdParsePositiveInteger(const S: UTF8String): QWord;
>>>>>>> graemeg/cpstrnew
=======
function xsdParsePositiveInteger(const S: UTF8String): QWord;
>>>>>>> graemeg/cpstrnew
=======
function xsdParsePositiveInteger(const S: UTF8String): QWord;
>>>>>>> graemeg/cpstrnew
=======
function xsdParsePositiveInteger(const S: UTF8String): QWord;
>>>>>>> origin/cpstrnew
begin
  xsdParsePositiveInteger(PChar(S), Length(S), Result);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdParseByte(const S: AnsiString): Shortint;
=======
function xsdParseByte(const S: UTF8String): Shortint;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseByte(const S: UTF8String): Shortint;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseByte(const S: UTF8String): Shortint;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseByte(const S: UTF8String): Shortint;
>>>>>>> origin/cpstrnew
begin
  xsdParseByte(PChar(S), Length(S), Result);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdParseShort(const S: AnsiString): Smallint;
=======
function xsdParseShort(const S: UTF8String): Smallint;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseShort(const S: UTF8String): Smallint;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseShort(const S: UTF8String): Smallint;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseShort(const S: UTF8String): Smallint;
>>>>>>> origin/cpstrnew
begin
  xsdParseShort(PChar(S), Length(S), Result);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdParseInt(const S: AnsiString): Longint;
=======
function xsdParseInt(const S: UTF8String): Longint;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseInt(const S: UTF8String): Longint;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseInt(const S: UTF8String): Longint;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseInt(const S: UTF8String): Longint;
>>>>>>> origin/cpstrnew
begin
  xsdParseInt(PChar(S), Length(S), Result);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdParseLong(const S: AnsiString): Int64;
=======
function xsdParseLong(const S: UTF8String): Int64;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseLong(const S: UTF8String): Int64;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseLong(const S: UTF8String): Int64;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseLong(const S: UTF8String): Int64;
>>>>>>> origin/cpstrnew
begin
  xsdParseLong(PChar(S), Length(S), Result);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdParseUnsignedByte(const S: AnsiString): Byte;
=======
function xsdParseUnsignedByte(const S: UTF8String): Byte;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseUnsignedByte(const S: UTF8String): Byte;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseUnsignedByte(const S: UTF8String): Byte;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseUnsignedByte(const S: UTF8String): Byte;
>>>>>>> origin/cpstrnew
begin
  xsdParseUnsignedByte(PChar(S), Length(S), Result);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdParseUnsignedShort(const S: AnsiString): Word;
=======
function xsdParseUnsignedShort(const S: UTF8String): Word;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseUnsignedShort(const S: UTF8String): Word;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseUnsignedShort(const S: UTF8String): Word;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseUnsignedShort(const S: UTF8String): Word;
>>>>>>> origin/cpstrnew
begin
  xsdParseUnsignedShort(PChar(S), Length(S), Result);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdParseUnsignedInt(const S: AnsiString): Longword;
=======
function xsdParseUnsignedInt(const S: UTF8String): Longword;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseUnsignedInt(const S: UTF8String): Longword;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseUnsignedInt(const S: UTF8String): Longword;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseUnsignedInt(const S: UTF8String): Longword;
>>>>>>> origin/cpstrnew
begin
  xsdParseUnsignedInt(PChar(S), Length(S), Result);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdParseUnsignedLong(const S: AnsiString): QWord;
=======
function xsdParseUnsignedLong(const S: UTF8String): QWord;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseUnsignedLong(const S: UTF8String): QWord;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseUnsignedLong(const S: UTF8String): QWord;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseUnsignedLong(const S: UTF8String): QWord;
>>>>>>> origin/cpstrnew
begin
  xsdParseUnsignedLong(PChar(S), Length(S), Result);
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function xsdParseEnum(const S: AnsiString; enum: array of AnsiString): Integer;
=======
function xsdParseEnum(const S: UTF8String; enum: array of Utf8String): Integer;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseEnum(const S: UTF8String; enum: array of Utf8String): Integer;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseEnum(const S: UTF8String; enum: array of Utf8String): Integer;
>>>>>>> graemeg/cpstrnew
=======
function xsdParseEnum(const S: UTF8String; enum: array of Utf8String): Integer;
>>>>>>> origin/cpstrnew
begin
  xsdParseEnum(PChar(S), Length(S), enum, Result);
end;

end.
