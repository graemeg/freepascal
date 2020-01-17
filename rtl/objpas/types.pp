{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2002 by Florian Klaempfl,
    member of the Free Pascal development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{$MODE OBJFPC}
unit Types;

  interface
{$modeswitch advancedrecords}
{$modeswitch class}
{$if defined(win32) or defined(win64) or defined(wince)}
    uses
       Windows;
{$elseif defined(win16)}
    uses
       WinTypes;
{$endif}

{$if defined(win32) or defined(win64)}
const
  RT_RCDATA = Windows.RT_RCDATA deprecated 'Use Windows.RT_RCDATA instead';
{$elseif defined(win16)}
const
  RT_RCDATA = WinTypes.RT_RCDATA deprecated 'Use WinTypes.RT_RCDATA instead';
{$endif}

type
  TEndian =  Objpas.TEndian;
  TDirection = (FromBeginning, FromEnd);
  TValueRelationship = -1..1;
  
  DWORD = LongWord;

  PLongint = System.PLongint;
  PSmallInt = System.PSmallInt;
{$ifndef FPUNONE}
  PDouble = System.PDouble;
{$endif}
  PByte = System.PByte;
  Largeint = int64;
  LARGE_INT = LargeInt;
  PLargeInt = ^LargeInt;
  LargeUint = qword;
  LARGE_UINT= LargeUInt;
  PLargeuInt = ^LargeuInt;

  TBooleanDynArray = array of Boolean;
  TByteDynArray = array of Byte;
  TCardinalDynArray = array of Cardinal;
  TInt64DynArray = array of Int64;
  TIntegerDynArray = array of Integer;
  TLongWordDynArray = array of LongWord;
  TPointerDynArray = array of Pointer;
  TQWordDynArray = array of QWord;
  TShortIntDynArray = array of ShortInt;
  TSmallIntDynArray = array of SmallInt;
  TStringDynArray = array of AnsiString;
  TWideStringDynArray   = array of WideString;
  TWordDynArray = array of Word;
  TCurrencyArray = Array of currency;
{$ifndef FPUNONE}
  TSingleDynArray = array of Single;
  TDoubleDynArray = array of Double;
  TExtendedDynArray = array of Extended;
  TCompDynArray = array of Comp;
{$endif}

{$if defined(win32) or defined(win64) or defined(wince)}
  TArray4IntegerType = Windows.TArray4IntegerType;
  TSmallPoint = Windows.TSmallPoint;
  PSmallPoint = Windows.PSmallPoint;

  TSize  = Windows.TSize;
  TagSize  = Windows.tagSize deprecated;
  PSize  = Windows.PSize;

  TPoint = Windows.TPoint;
  TagPoint = Windows.TagPoint deprecated;
  PPoint = Windows.PPoint;

  TRect  = Windows.TRect;
  PRect  = Windows.PRect;
  TSplitRectType = Windows.TSplitRectType;
const
  srLeft = TSplitRectType.srLeft;
  srRight = TSplitRectType.srRight;
  srTop = TSplitRectType.srTop;
  srBottom = TSplitRectType.srBottom;
type
{$else}
  {$i typshrdh.inc}
   TagSize = tSize deprecated;
   TagPoint = TPoint deprecated;
{$endif}

  { TPointF }
  TPointF =
{$ifndef FPC_REQUIRES_PROPER_ALIGNMENT}
  packed
{$endif FPC_REQUIRES_PROPER_ALIGNMENT}
  record
       x,y : Single;
       public
          function Add(const apt: TPoint): TPointF;
          function Add(const apt: TPointF): TPointF;
          function Distance(const apt : TPointF) : Single;
          function DotProduct(const apt : TPointF) : Single;
          function IsZero : Boolean;
          function Subtract(const apt : TPointF): TPointF;
          function Subtract(const apt : TPoint): TPointF;
          procedure SetLocation(const apt :TPointF);
          procedure SetLocation(const apt :TPoint);
          procedure SetLocation(ax,ay : Longint);
          procedure Offset(const apt :TPointF);
          procedure Offset(const apt :TPoint);
          procedure Offset(dx,dy : Longint);

          function  Scale (afactor:Single)  : TPointF;
          function  Ceiling : TPoint;
          function  Truncate: TPoint;
          function  Floor   : TPoint;
          function  Round   : TPoint;
          function  Length  : Single;
          class function Create(const ax, ay: Single): TPointF; overload; static; inline;
          class function Create(const apt: TPoint): TPointF; overload; static; inline;
          class operator = (const apt1, apt2 : TPointF) : Boolean;
          class operator <> (const apt1, apt2 : TPointF): Boolean;
          class operator + (const apt1, apt2 : TPointF): TPointF;
          class operator - (const apt1, apt2 : TPointF): TPointF;
          class operator - (const apt1 : TPointF): TPointF;
          class operator * (const apt1, apt2: TPointF): Single; // scalar product
          class operator * (const apt1: TPointF; afactor: single): TPointF;
          class operator * (afactor: single; const apt1: TPointF): TPointF;
       end;
  { TRectF }

  TRectF =
{$ifndef FPC_REQUIRES_PROPER_ALIGNMENT}
  packed
{$endif FPC_REQUIRES_PROPER_ALIGNMENT}
  record
  private
    function GetHeight: Single; inline;
    function GetWidth: Single;  inline;
    procedure SetHeight(AValue: Single);
    procedure SetWidth (AValue: Single);
  public
    function  Union  (const r: TRectF):TRectF; inline;
    procedure Offset (const dx,dy : Single); inline;
    property  Width  : Single read GetWidth write SetWidth;
    property  Height : Single read GetHeight write SetHeight;
    case Integer of
     0: (Left, Top, Right, Bottom: Single);
     1: (TopLeft, BottomRight: TPointF);
    end;

  TDuplicates = (dupIgnore, dupAccept, dupError);

type
  TOleChar = WideChar;
  POleStr = PWideChar;
  PPOleStr = ^POleStr;

  TListCallback = procedure(data,arg:pointer) of object;
  TListStaticCallback = procedure(data,arg:pointer);

const
  GUID_NULL: TGUID  = '{00000000-0000-0000-0000-000000000000}';

  STGTY_STORAGE   = 1;
  STGTY_STREAM    = 2;
  STGTY_LOCKBYTES = 3;
  STGTY_PROPERTY  = 4;

  STREAM_SEEK_SET = 0;
  STREAM_SEEK_CUR = 1;
  STREAM_SEEK_END = 2;

  LOCK_WRITE     = 1;
  LOCK_EXCLUSIVE = 2;
  LOCK_ONLYONCE  = 4;

  STATFLAG_DEFAULT   	      = 0;
  STATFLAG_NONAME    	      = 1;
  STATFLAG_NOOPEN    	      = 2;

{$ifndef Wince}
  // in Wince these are in unit windows. Under 32/64 in ActiveX.
  // for now duplicate them. Not that bad for untyped constants.

  E_FAIL 		      = HRESULT($80004005);
  E_INVALIDARG                = HRESULT($80070057);

  STG_E_INVALIDFUNCTION       = HRESULT($80030001);
  STG_E_FILENOTFOUND          = HRESULT($80030002);
  STG_E_PATHNOTFOUND          = HRESULT($80030003);
  STG_E_TOOMANYOPENFILES      = HRESULT($80030004);
  STG_E_ACCESSDENIED          = HRESULT($80030005);
  STG_E_INVALIDHANDLE         = HRESULT($80030006);
  STG_E_INSUFFICIENTMEMORY    = HRESULT($80030008);
  STG_E_INVALIDPOINTER        = HRESULT($80030009);
  STG_E_NOMOREFILES           = HRESULT($80030012);
  STG_E_DISKISWRITEPROTECTED  = HRESULT($80030013);
  STG_E_SEEKERROR             = HRESULT($80030019);
  STG_E_WRITEFAULT            = HRESULT($8003001D);
  STG_E_READFAULT             = HRESULT($8003001E);
  STG_E_SHAREVIOLATION        = HRESULT($80030020);
  STG_E_LOCKVIOLATION         = HRESULT($80030021);
  STG_E_FILEALREADYEXISTS     = HRESULT($80030050);
  STG_E_INVALIDPARAMETER      = HRESULT($80030057);
  STG_E_MEDIUMFULL            = HRESULT($80030070);
  STG_E_PROPSETMISMATCHED     = HRESULT($800300F0);
  STG_E_ABNORMALAPIEXIT       = HRESULT($800300FA);
  STG_E_INVALIDHEADER         = HRESULT($800300FB);
  STG_E_INVALIDNAME           = HRESULT($800300FC);
  STG_E_UNKNOWN               = HRESULT($800300FD);
  STG_E_UNIMPLEMENTEDFUNCTION = HRESULT($800300FE);
  STG_E_INVALIDFLAG           = HRESULT($800300FF);
  STG_E_INUSE                 = HRESULT($80030100);
  STG_E_NOTCURRENT            = HRESULT($80030101);
  STG_E_REVERTED              = HRESULT($80030102);
  STG_E_CANTSAVE              = HRESULT($80030103);
  STG_E_OLDFORMAT             = HRESULT($80030104);
  STG_E_OLDDLL                = HRESULT($80030105);
  STG_E_SHAREREQUIRED         = HRESULT($80030106);
  STG_E_EXTANTMARSHALLINGS    = HRESULT($80030108);
  STG_E_DOCFILECORRUPT        = HRESULT($80030109);
  STG_E_BADBASEADDRESS        = HRESULT($80030110);
  STG_E_INCOMPLETE            = HRESULT($80030201);
  STG_E_TERMINATED            = HRESULT($80030202);

  STG_S_CONVERTED             = $00030200;
  STG_S_BLOCK                 = $00030201;
  STG_S_RETRYNOW              = $00030202;
  STG_S_MONITORING            = $00030203;
{$endif}

{$if (not defined(win32)) and (not defined(win64)) and (not defined(wince))}
type
  PCLSID = PGUID;
  TCLSID = TGUID;

  PDWord = ^DWord;

  PDisplay = Pointer;
  PEvent = Pointer;

  TXrmOptionDescRec = record
  end;
  XrmOptionDescRec = TXrmOptionDescRec;
  PXrmOptionDescRec = ^TXrmOptionDescRec;

  Widget = Pointer;
  WidgetClass = Pointer;
  ArgList = Pointer;
  Region = Pointer;

  _FILETIME =
{$ifndef FPC_REQUIRES_PROPER_ALIGNMENT}
  packed
{$endif FPC_REQUIRES_PROPER_ALIGNMENT}
  record
     dwLowDateTime : DWORD;
     dwHighDateTime : DWORD;
  end;
  TFileTime = _FILETIME;
  FILETIME = _FILETIME;
  PFileTime = ^TFileTime;
{$else}
type
  PCLSID    = Windows.PCLSID;
  TCLSID    = Windows.CLSID;
  TFiletime = Windows.TFileTime;
  Filetime  = Windows.FileTime;
  PFiletime = Windows.PFileTime;
{$endif Windows}

type
  tagSTATSTG = record
     pwcsName      : POleStr;
     dwType        : DWord;
     cbSize        : Large_uint;
     mtime         : TFileTime;
     ctime         : TFileTime;
     atime         : TFileTime;
     grfMode       : DWord;
     grfLocksSupported : DWord;
     clsid         : TCLSID;
     grfStateBits  : DWord;
     reserved      : DWord;
  end;
  TStatStg = tagSTATSTG;
  STATSTG = TStatStg;
  PStatStg = ^TStatStg;

  { classes depends on these interfaces, we can't use the activex unit in classes though }
  IClassFactory = Interface(IUnknown) ['{00000001-0000-0000-C000-000000000046}']
     Function CreateInstance(Const unkOuter : IUnknown;Const riid : TGUID;Out vObject) : HResult;StdCall;
     Function LockServer(fLock : LongBool) : HResult;StdCall;
  End;

  ISequentialStream = interface(IUnknown)
     ['{0c733a30-2a1c-11ce-ade5-00aa0044773d}']
     function Read(pv : Pointer;cb : DWORD;pcbRead : PDWORD) : HRESULT;stdcall;
     function Write(pv : Pointer;cb : DWORD;pcbWritten : PDWORD): HRESULT;stdcall;
  end;

  IStream = interface(ISequentialStream) ['{0000000C-0000-0000-C000-000000000046}']
     function Seek(dlibMove : LargeInt; dwOrigin : DWORD; out libNewPosition : LargeUInt) : HResult;stdcall;
     function SetSize(libNewSize : LargeUInt) : HRESULT;stdcall;
     function CopyTo(stm: IStream;cb : LargeUInt;out cbRead : LargeUInt; out cbWritten : LargeUInt) : HRESULT;stdcall;
     function Commit(grfCommitFlags : DWORD) : HRESULT;stdcall;
     function Revert : HRESULT;stdcall;
     function LockRegion(libOffset : LargeUInt;cb : LargeUInt; dwLockType : DWORD) : HRESULT;stdcall;
     function UnlockRegion(libOffset : LargeUInt;cb : LargeUInt; dwLockType : DWORD) : HRESULT;stdcall;
     Function Stat(out statstg : TStatStg;grfStatFlag : DWORD) : HRESULT;stdcall;
     function Clone(out stm : IStream) : HRESULT;stdcall;
  end;

function EqualRect(const r1,r2 : TRect) : Boolean;
function Rect(Left,Top,Right,Bottom : Integer) : TRect; inline;
function Bounds(ALeft,ATop,AWidth,AHeight : Integer) : TRect; inline;
function Point(x,y : Integer) : TPoint; inline;
function PtInRect(const Rect : TRect; const p : TPoint) : Boolean;
function IntersectRect(var Rect : TRect; const R1,R2 : TRect) : Boolean;
function UnionRect(var Rect : TRect; const R1,R2 : TRect) : Boolean;
function IsRectEmpty(const Rect : TRect) : Boolean;
function OffsetRect(var Rect : TRect;DX : Integer;DY : Integer) : Boolean;
function CenterPoint(const Rect: TRect): TPoint;
function InflateRect(var Rect: TRect; dx: Integer; dy: Integer): Boolean;
function Size(AWidth, AHeight: Integer): TSize; inline;
function Size(const ARect: TRect): TSize;

implementation

Uses Math;

{$if (not defined(win32)) and (not defined(win64)) and (not defined(wince))}
  {$i typshrd.inc}
{$endif}

function EqualRect(const r1,r2 : TRect) : Boolean;

begin
  EqualRect:=(r1.left=r2.left) and (r1.right=r2.right) and (r1.top=r2.top) and (r1.bottom=r2.bottom);
end;

function Rect(Left,Top,Right,Bottom : Integer) : TRect; inline;

begin
  Rect.Left:=Left;
  Rect.Top:=Top;
  Rect.Right:=Right;
  Rect.Bottom:=Bottom;
end;

function Bounds(ALeft,ATop,AWidth,AHeight : Integer) : TRect; inline;

begin
  Bounds.Left:=ALeft;
  Bounds.Top:=ATop;
  Bounds.Right:=ALeft+AWidth;
  Bounds.Bottom:=ATop+AHeight;
end;

function Point(x,y : Integer) : TPoint; inline;

begin
  Point.x:=x;
  Point.y:=y;
end;

function PtInRect(const Rect : TRect;const p : TPoint) : Boolean;

begin
  PtInRect:=(p.y>=Rect.Top) and
            (p.y<Rect.Bottom) and
            (p.x>=Rect.Left) and
            (p.x<Rect.Right);
end;

function IntersectRect(var Rect : TRect;const R1,R2 : TRect) : Boolean;
var
  lRect: TRect;
begin
  lRect := R1;
  if R2.Left > R1.Left then
    lRect.Left := R2.Left;
  if R2.Top > R1.Top then
    lRect.Top := R2.Top;
  if R2.Right < R1.Right then
    lRect.Right := R2.Right;
  if R2.Bottom < R1.Bottom then
    lRect.Bottom := R2.Bottom;

  // The var parameter is only assigned in the end to avoid problems
  // when passing the same rectangle in the var and const parameters.
  // See http://bugs.freepascal.org/view.php?id=17722
  if IsRectEmpty(lRect) then
  begin
    FillChar(Rect,SizeOf(Rect),0);
    IntersectRect:=false;
  end
  else
  begin
    IntersectRect:=true;
    Rect := lRect;
  end;	
end;

function UnionRect(var Rect : TRect;const R1,R2 : TRect) : Boolean;
var
  lRect: TRect;
begin
  lRect:=R1;
  if R2.Left<R1.Left then
    lRect.Left:=R2.Left;
  if R2.Top<R1.Top then
    lRect.Top:=R2.Top;
  if R2.Right>R1.Right then
    lRect.Right:=R2.Right;
  if R2.Bottom>R1.Bottom then
    lRect.Bottom:=R2.Bottom;

  if IsRectEmpty(lRect) then
  begin
    FillChar(Rect,SizeOf(Rect),0);
    UnionRect:=false;
  end
  else
  begin
    Rect:=lRect;
    UnionRect:=true;
  end;
end;

function IsRectEmpty(const Rect : TRect) : Boolean;
begin
  IsRectEmpty:=(Rect.Right<=Rect.Left) or (Rect.Bottom<=Rect.Top);
end;

function OffsetRect(var Rect : TRect;DX : Integer;DY : Integer) : Boolean;
begin
  if assigned(@Rect) then
    begin
    with Rect do
      begin
      inc(Left,dx);
      inc(Top,dy);
      inc(Right,dx);
      inc(Bottom,dy);
      end;
    OffsetRect:=true;
    end
  else
    OffsetRect:=false;
end;

function Avg(a, b: Longint): Longint;
begin
  if a < b then
    Result := a + ((b - a) shr 1)
  else
    Result := b + ((a - b) shr 1);
end;

function CenterPoint(const Rect: TRect): TPoint;
begin
  with Rect do
    begin
      Result.X := Avg(Left, Right);
      Result.Y := Avg(Top, Bottom);
    end;
end;

function InflateRect(var Rect: TRect; dx: Integer; dy: Integer): Boolean;
begin
  if Assigned(@Rect) then
  begin
    with Rect do
    begin
      dec(Left, dx);
      dec(Top, dy);
      inc(Right, dx);
      inc(Bottom, dy);
    end;
    Result := True;
  end
  else
    Result := False;
end;

function Size(AWidth, AHeight: Integer): TSize; inline;
begin
  Result.cx := AWidth;
  Result.cy := AHeight;
end;

function Size(const ARect: TRect): TSize; inline;
begin
  Result.cx := ARect.Right - ARect.Left;
  Result.cy := ARect.Bottom - ARect.Top;
end;


{ TPointF}

function TPointF.Add(const apt: TPoint): TPointF;
begin
  result.x:=x+apt.x;
  result.y:=y+apt.y;
end;

function TPointF.Add(const apt: TPointF): TPointF;
begin
  result.x:=x+apt.x;
  result.y:=y+apt.y;
end;

function TPointF.Subtract(const apt : TPointF): TPointF;
begin
  result.x:=x-apt.x;
  result.y:=y-apt.y;
end;

function TPointF.Subtract(const apt: TPoint): TPointF;
begin
  result.x:=x-apt.x;
  result.y:=y-apt.y;
end;

function TPointF.Distance(const apt : TPointF) : Single;
begin
  result:=sqrt(sqr(apt.x-x)+sqr(apt.y-y));
end;

function TPointF.DotProduct(const apt: TPointF): Single;
begin
  result:=x*apt.x+y*apt.y;
end;

function TPointF.IsZero : Boolean;
begin
  result:=SameValue(x,0.0) and SameValue(y,0.0);
end;

procedure TPointF.Offset(const apt :TPointF);
begin
  x:=x+apt.x;
  y:=y+apt.y;
end;

procedure TPointF.Offset(const apt: TPoint);
begin
  x:=x+apt.x;
  y:=y+apt.y;
end;

procedure TPointF.Offset(dx,dy : Longint);
begin
  x:=x+dx;
  y:=y+dy;
end;

function TPointF.Scale(afactor: Single): TPointF;
begin
  result.x:=afactor*x;
  result.y:=afactor*y;
end;

function TPointF.Ceiling: TPoint;
begin
  result.x:=ceil(x);
  result.y:=ceil(y);
end;

function TPointF.Truncate: TPoint;
begin
  result.x:=trunc(x);
  result.y:=trunc(y);
end;

function TPointF.Floor: TPoint;
begin
  result.x:=Math.floor(x);
  result.y:=Math.floor(y);
end;

function TPointF.Round: TPoint;
begin
  result.x:=System.round(x);
  result.y:=System.round(y);
end;

function TPointF.Length: Single;
begin     //distance(self) ?
  result:=sqrt(sqr(x)+sqr(y));
end;

class operator TPointF.= (const apt1, apt2 : TPointF) : Boolean;
begin
  result:=SameValue(apt1.x,apt2.x) and SameValue(apt1.y,apt2.y);
end;

class operator TPointF.<> (const apt1, apt2 : TPointF): Boolean;
begin
  result:=NOT (SameValue(apt1.x,apt2.x) and Samevalue(apt1.y,apt2.y));
end;

class operator TPointF. * (const apt1, apt2: TPointF): Single;
begin
  result:=apt1.x*apt2.x + apt1.y*apt2.y;
end;

class operator TPointF. * (afactor: single; const apt1: TPointF): TPointF;
begin
  result:=apt1.Scale(afactor);
end;

class operator TPointF. * (const apt1: TPointF; afactor: single): TPointF;
begin
  result:=apt1.Scale(afactor);
end;

class operator TPointF.+ (const apt1, apt2 : TPointF): TPointF;
begin
  result.x:=apt1.x+apt2.x;
  result.y:=apt1.y+apt2.y;
end;

class operator TPointF.- (const apt1, apt2 : TPointF): TPointF;
begin
  result.x:=apt1.x-apt2.x;
  result.y:=apt1.y-apt2.y;
end;

class operator TPointF. - (const apt1: TPointF): TPointF;
begin
  Result.x:=-apt1.x;
  Result.y:=-apt1.y;
end;

procedure TPointF.SetLocation(const apt :TPointF);
begin
 x:=apt.x; y:=apt.y;
end;

procedure TPointF.SetLocation(const apt: TPoint);
begin
  x:=apt.x; y:=apt.y;
end;

procedure TPointF.SetLocation(ax,ay : Longint);
begin
  x:=ax; y:=ay;
end;

class function TPointF.Create(const ax, ay: Single): TPointF;
begin
  Result.x := ax;
  Result.y := ay;
end;

class function TPointF.Create(const apt: TPoint): TPointF;
begin
  Result.x := apt.X;
  Result.y := apt.Y;
end;
{ TRectF }

function TRectF.GetHeight: Single;
begin
  result:=bottom-top;
end;

function TRectF.GetWidth: Single;
begin
 result:=right-left;
end;

procedure TRectF.SetHeight(AValue: Single);
begin
  bottom:=top+avalue;
end;

procedure TRectF.SetWidth(AValue: Single);
begin
  right:=left+avalue;
end;

function TRectF.Union(const r: TRectF): TRectF;
begin
  result.left:=min(r.left,left);
  result.top:=min(r.top,top);
  result.right:=min(r.right,right);
  result.bottom:=min(r.bottom,bottom);
end;

procedure TRectF.Offset(const dx, dy: Single);
begin
  left:=left+dx; right:=right+dx;
  bottom:=bottom+dy; top:=top+dy;
end;

end.
