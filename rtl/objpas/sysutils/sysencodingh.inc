{
    *********************************************************************
    Copyright (C) 2012 Paul Ishenin,
    member of the Free Pascal Development Team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
    *********************************************************************
}

type
  EEncodingError = class(Exception);

  { TEncoding }

  TEncoding = class
  strict private
    type
      TStandardEncoding = (
        seAnsi,
        seAscii,
        seUnicode,
        seBigEndianUnicode,
        seUTF7,
        seUTF8);
    var
      FStandardEncodings: array[TStandardEncoding] of TEncoding; static;
      FSystemEncodings: array of TEncoding; static;
    Class Var
      FLock : TRTLCriticalSection;
    class function GetANSI: TEncoding; static;
    class function GetASCII: TEncoding; static;
    class function GetBigEndianUnicode: TEncoding; static;
    class function GetDefault: TEncoding; static;
    class function GetSystemEncoding: TEncoding; static;
    class function GetUnicode: TEncoding; static;
    class function GetUTF7: TEncoding; static;
    class function GetUTF8: TEncoding; static;

    class constructor Create;
    class destructor Destroy;
  strict protected
    FIsSingleByte: Boolean;
    FMaxCharSize: Integer;
    class procedure FreeEncodings;
    function GetByteCount(Chars: PUnicodeChar; CharCount: Integer): Integer; overload; virtual; abstract;
    function GetBytes(Chars: PUnicodeChar; CharCount: Integer; Bytes: PByte; ByteCount: Integer): Integer; overload; virtual; abstract;
    function GetCharCount(Bytes: PByte; ByteCount: Integer): Integer; overload; virtual; abstract;
    function GetChars(Bytes: PByte; ByteCount: Integer; Chars: PUnicodeChar; CharCount: Integer): Integer; overload; virtual; abstract;
    function GetAnsiBytes(Chars: PChar; CharCount: Integer): TBytes; virtual; abstract;
    function GetAnsiString(Bytes: PByte; ByteCount: Integer): string; virtual; abstract;
    function GetCodePage: Cardinal; virtual; abstract;
    function GetEncodingName: UnicodeString; virtual; abstract;
  public
    function Clone: TEncoding; virtual;
    class function Convert(Source, Destination: TEncoding; const Bytes: TBytes): TBytes; overload;
    class function Convert(Source, Destination: TEncoding; const Bytes: TBytes; StartIndex, Count: Integer): TBytes; overload;
    class function IsStandardEncoding(AEncoding: TEncoding): Boolean; static;
    class function GetBufferEncoding(const Buffer: TBytes; var AEncoding: TEncoding): Integer; overload; static;
    class function GetBufferEncoding(const Buffer: TBytes; var AEncoding: TEncoding;
      ADefaultEncoding: TEncoding): Integer; overload; static;
    function GetByteCount(const Chars: TUnicodeCharArray): Integer; overload;
    function GetByteCount(const Chars: TUnicodeCharArray; CharIndex, CharCount: Integer): Integer; overload;
    function GetByteCount(const S: UnicodeString): Integer; overload;
    function GetByteCount(const S: UnicodeString; CharIndex, CharCount: Integer): Integer; overload;
    function GetBytes(const Chars: TUnicodeCharArray): TBytes; overload;
    function GetBytes(const Chars: TUnicodeCharArray; CharIndex, CharCount: Integer): TBytes; overload;
    function GetBytes(const Chars: TUnicodeCharArray; CharIndex, CharCount: Integer;
      const Bytes: TBytes; ByteIndex: Integer): Integer; overload;
    function GetBytes(const S: UnicodeString): TBytes; overload;
    function GetBytes(const S: UnicodeString; CharIndex, CharCount: Integer;
      const Bytes: TBytes; ByteIndex: Integer): Integer; overload;
    function GetCharCount(const Bytes: TBytes): Integer; overload;
    function GetCharCount(const Bytes: TBytes; ByteIndex, ByteCount: Integer): Integer; overload;
    function GetChars(const Bytes: TBytes): TUnicodeCharArray; overload;
    function GetChars(const Bytes: TBytes; ByteIndex, ByteCount: Integer): TUnicodeCharArray; overload;
    function GetChars(const Bytes: TBytes; ByteIndex, ByteCount: Integer;
      const Chars: TUnicodeCharArray; CharIndex: Integer): Integer; overload;
    class function GetEncoding(CodePage: Integer): TEncoding; overload; static;
    class function GetEncoding(const EncodingName: UnicodeString): TEncoding; overload; static;
    function GetMaxByteCount(CharCount: Integer): Integer; virtual; abstract;
    function GetMaxCharCount(ByteCount: Integer): Integer; virtual; abstract;
    function GetPreamble: TBytes; virtual; abstract;
    function GetString(const Bytes: TBytes): UnicodeString; overload;
    function GetString(const Bytes: TBytes; ByteIndex, ByteCount: Integer): UnicodeString; overload;
    function GetAnsiBytes(const S: string): TBytes; overload;
    function GetAnsiBytes(const S: string; CharIndex, CharCount: Integer): TBytes; overload;
    function GetAnsiString(const Bytes: TBytes): string; overload;
    function GetAnsiString(const Bytes: TBytes; ByteIndex, ByteCount: Integer): string; overload;

    property CodePage: Cardinal read GetCodePage;
    property EncodingName: UnicodeString read GetEncodingName;
    property IsSingleByte: Boolean read FIsSingleByte;

    class property ANSI: TEncoding read GetANSI;
    class property ASCII: TEncoding read GetASCII;
    class property BigEndianUnicode: TEncoding read GetBigEndianUnicode;
    class property Default: TEncoding read GetDefault;
    class property SystemEncoding: TEncoding read GetSystemEncoding;
    class property Unicode: TEncoding read GetUnicode;
    class property UTF7: TEncoding read GetUTF7;
    class property UTF8: TEncoding read GetUTF8;
  end;

  { TMBCSEncoding }

  TMBCSEncoding = class(TEncoding)
  strict private
    FCodePage: Integer;
    FMBToWCharFlags: Integer;
    FWCharToMBFlags: Integer;
  strict protected
    function GetByteCount(Chars: PUnicodeChar; CharCount: Integer): Integer; overload; override;
    function GetBytes(Chars: PUnicodeChar; CharCount: Integer; Bytes: PByte; ByteCount: Integer): Integer; overload; override;
    function GetCharCount(Bytes: PByte; ByteCount: Integer): Integer; overload; override;
    function GetChars(Bytes: PByte; ByteCount: Integer; Chars: PUnicodeChar; CharCount: Integer): Integer; overload; override;
    function GetAnsiBytes(Chars: PChar; CharCount: Integer): TBytes; override;
    function GetAnsiString(Bytes: PByte; ByteCount: Integer): string; override;
    function GetCodePage: Cardinal; override;
    function GetEncodingName: UnicodeString; override;
  public
    constructor Create; overload; virtual;
    constructor Create(ACodePage: Integer); overload; virtual;
    constructor Create(ACodePage, MBToWCharFlags, WCharToMBFlags: Integer); overload; virtual;
    function Clone: TEncoding; override;
    function GetMaxByteCount(CharCount: Integer): Integer; override;
    function GetMaxCharCount(ByteCount: Integer): Integer; override;
    function GetPreamble: TBytes; override;
  end;

  { TUTF7Encoding }

  TUTF7Encoding = class(TMBCSEncoding)
  public
    constructor Create; override;
    function Clone: TEncoding; override;
    function GetMaxByteCount(CharCount: Integer): Integer; override;
    function GetMaxCharCount(ByteCount: Integer): Integer; override;
  end;

  { TUTF8Encoding }

  TUTF8Encoding = class(TUTF7Encoding)
  public
    constructor Create; override;
    function Clone: TEncoding; override;
    function GetMaxByteCount(CharCount: Integer): Integer; override;
    function GetMaxCharCount(ByteCount: Integer): Integer; override;
    function GetPreamble: TBytes; override;
  end;

  { TUnicodeEncoding }

  TUnicodeEncoding = class(TEncoding)
  strict protected
    function GetByteCount(Chars: PUnicodeChar; CharCount: Integer): Integer; overload; override;
    function GetBytes(Chars: PUnicodeChar; CharCount: Integer; Bytes: PByte; ByteCount: Integer): Integer; overload; override;
    function GetCharCount(Bytes: PByte; ByteCount: Integer): Integer; overload; override;
    function GetChars(Bytes: PByte; ByteCount: Integer; Chars: PUnicodeChar; CharCount: Integer): Integer; overload; override;
    function GetAnsiBytes(Chars: PChar; CharCount: Integer): TBytes; override;
    function GetAnsiString(Bytes: PByte; ByteCount: Integer): string; override;
    function GetCodePage: Cardinal; override;
    function GetEncodingName: UnicodeString; override;
  public
    constructor Create; virtual;
    function Clone: TEncoding; override;
    function GetMaxByteCount(CharCount: Integer): Integer; override;
    function GetMaxCharCount(ByteCount: Integer): Integer; override;
    function GetPreamble: TBytes; override;
  end;

  { TBigEndianUnicodeEncoding }

  TBigEndianUnicodeEncoding = class(TUnicodeEncoding)
  strict protected
    procedure Swap(var B: TBytes);
    function GetBytes(Chars: PUnicodeChar; CharCount: Integer; Bytes: PByte; ByteCount: Integer): Integer; overload; override;
    function GetChars(Bytes: PByte; ByteCount: Integer; Chars: PUnicodeChar; CharCount: Integer): Integer; overload; override;
    function GetAnsiBytes(Chars: PChar; CharCount: Integer): TBytes; override;
    function GetAnsiString(Bytes: PByte; ByteCount: Integer): string; override;
    function GetCodePage: Cardinal; override;
    function GetEncodingName: UnicodeString; override;
  public
    function Clone: TEncoding; override;
    function GetPreamble: TBytes; override;
  end;
