{
    This file is part of the PinGUI - Platform Independent GUI Project
    Copyright (c) 1999 by Berczi Gabor

    VESA support routines

    See the file COPYING.GUI, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit VESA;

{$ifdef DEBUG}
{$define TESTGRAPHIC}
{$endif DEBUG}

interface

uses
  Dos,
  Objects,Strings,WUtils;

const
     { Video Mode Attributes mask constants }
     vesa_vma_CanBeSetInCurrentConfig = $0001;
     vesa_vma_OptionalBlockPresent    = $0002;
     vesa_vma_BIOSSupport             = $0004;
     vesa_vma_ColorMode               = $0008; { else mono }
     vesa_vma_GraphicsMode            = $0010; { else text }
     { -- VBE 2.0 --- }
     vesa_vma_VGACompatibleMode       = $0020;
     vesa_vma_VGACompWindowedAvail    = $0040;
     vesa_vma_LinearFrameBufferAvail  = $0080;

     { Windows Attributes mask constants }
     vesa_wa_Present                  = $0001;
     vesa_wa_Readable                 = $0002;
     vesa_wa_Writeable                = $0004;

     { Memory Model value constants }
     vesa_mm_Text                     = $0000;
     vesa_mm_CGAGraphics              = $0001;
     vesa_mm_HerculesGraphics         = $0002;
     vesa_mm_4planePlanar             = $0003;
     vesa_mm_PackedPixel              = $0004;
     vesa_mm_NonChain4_256color       = $0005;
     vesa_mm_DirectColor              = $0006;
     vesa_mm_YUV                      = $0007;

     { Memory Window value constants }
     vesa_mw_WindowA                  = $0000;
     vesa_mw_WindowB                  = $0001;

type
     tregisters=registers;

     PtrRec16 = record
       Ofs,Seg: word;
     end;

     TVESAInfoBlock = packed record
       Signature    : longint; {  'VESA' }
       Version      : word;
       OEMString    : PString;
       Capabilities : longint;
       VideoModeList: PWordArray;
       TotalMemory  : word; { in 64KB blocks }
       Fill         : array[1..236] of byte;
       VBE2Fill     : array[1..256] of byte;
     end;

     TVESAModeInfoBlock = packed record
       Attributes      : word;
       WinAAttrs       : byte;
       WinBAttrs       : byte;
       Granularity     : word;
       Size            : word;
       ASegment        : word;
       BSegment        : word;
       FuncPtr         : pointer;
       BytesPerLine    : word;
     { optional }
       XResolution     : word;
       YResolution     : word;
       XCharSize       : byte;
       YCharSize       : byte;
       NumberOfPlanes  : byte;
       BitsPerPixel    : byte;
       NumberOfBanks   : byte;
       MemoryModel     : byte;
       BankSize        : byte;
       NumberOfImagePages: byte;
       Reserved        : byte;
     { direct color fields }
       RedMaskSize     : byte;
       RedFieldPosition: byte;
       GreenMaskSize   : byte;
       GreenFieldPosition: byte;
       BlueMaskSize    : byte;
       BlueFieldPosition: byte;
       ReservedMaskSize: byte;
       ReservedPosition: byte;
       DirectColorModeInfo: byte;
      { --- VBE 2.0 optional --- }
       LinearFrameAddr : longint;
       OffScreenAddr   : longint;
       OffScreenSize   : word;
       Reserved2       : array[1..216-(4+4+2)] of byte;
     end;

     TVESAModeList = record
       Count        : word;
       Modes        : array[1..256] of word;
     end;

function VESAInit: boolean;
function VESAGetInfo(var B: TVESAInfoBlock): boolean;
function VESAGetModeInfo(Mode: word; var B: TVESAModeInfoBlock): boolean;
function VESAGetModeList(var B: TVESAModeList): boolean;
function VESASearchMode(XRes,YRes,BPX: word; LFB: boolean; var Mode: word; var ModeInfo: TVESAModeInfoBlock): boolean;
function VESAGetOemString: string;
function VESASetMode(Mode: word): boolean;
function VESAGetMode(var Mode: word): boolean;
function VESASelectMemoryWindow(Window: byte; Position: word): boolean;
function VESAReturnMemoryWindow(Window: byte; var Position: word): boolean;
function RegisterVesaVideoMode(Mode : word) : boolean;
Procedure FreeVesaModes;

const
  DisableVESA : boolean = false;

implementation

uses
  video, mouse,
{$ifdef TESTGRAPHIC}
  graph,
{$endif TESTGRAPHIC}
  pmode;

type

       PVesaVideoMode = ^TVesaVideoMode;
       TVesaVideoMode = record
         {Col,Row      : word;
          Color        : boolean;}
         V            : TVideoMode;
         Mode         : word;
         IsGraphic    : boolean;
         { zero based vesa specific driver count }
         VideoIndex   : word;
         Next         : PVesaVideoMode;
       end;
       CursorBitMap = Record
        width,height,size : longint;
        colors : array[0..8*8-1] of word;
       end;
const
  VesaVideoModeHead : PVesaVideoMode = nil;
  VesaRegisteredModes : word = 0;
{$ifdef TESTGRAPHIC}
  IsGraphicMode : boolean = false;
  GraphDriver   : integer = 0;
  GraphMode     : Integer = 0;
  FirstCallAfterSetVesaMode : boolean = false;
  LastCursorX : word = $ffff;
  LastCursorY : word = $ffff;
  LastCursorType : word = crHidden;

var
  UnderLineImage : CursorBitMap;
  BlockImage : CursorBitMap;
  HalfBlockImage : CursorBitMap;
{$endif TESTGRAPHIC}

Var
  SysGetVideoModeCount : function : word;
  SysSetVideoMode      : function (Const VideoMode : TVideoMode) : boolean;
  SysGetVideoModeData  : function (Index : Word; Var Data : TVideoMode) : boolean;
  SysUpdateScreen      : procedure(Force : Boolean);
  SysDoneVideo         : procedure;
  SysInitVideo         : procedure;
  SysSetCursorPos      : procedure(NewCursorX, NewCursorY: Word);
  SysSetCursorType     : procedure(NewCurosrType : word);


function VESAGetInfo(var B: TVESAInfoBlock): boolean;
var r: registers;
    OK: boolean;
    M: MemPtr;
begin
  if disableVESA then
    exit(false);
  StrToMem('VBE2',B.Signature);
  GetDosMem(M,SizeOf(B));
  M.MoveDataTo(B,sizeof(B));
  r.ah:=$4f; r.al:=0;
  r.es:=M.DosSeg; r.di:=M.DosOfs;
  realintr($10,r);
  M.MoveDataFrom(sizeof(B),B);
  FreeDosMem(M);
  OK:=(r.ax=$004f){ and (MemToStr(B.Signature,4)='VESA')};
  VESAGetInfo:=OK;
end;

function VESAGetModeList(var B: TVESAModeList): boolean;
var OK: boolean;
    VI: TVESAInfoBlock;
begin
  FillChar(B,SizeOf(B),0);
  if disableVESA then
    exit(false);
  OK:=VESAGetInfo(VI);
  if OK then
  begin
    OK:=MoveDosToPM(VI.VideoModeList,@B.Modes,sizeof(B.Modes));
    if OK then
      while (B.Modes[B.Count+1]<>$ffff) and (B.Count<High(B.Modes)) do
            Inc(B.Count);
  end;
  VESAGetModeList:=OK;
end;

function VESASearchMode(XRes,YRes,BPX: word; LFB: boolean; var Mode: word; var ModeInfo: TVESAModeInfoBlock): boolean;
var B: TVESAModeList;
    OK: boolean;
    I: integer;
    MI: TVESAModeInfoBlock;
begin
  OK:=VESAGetModeList(B);
  I:=1; Mode:=0;
  repeat
    OK:=VESAGetModeInfo(B.Modes[I],MI);
    if OK and (MI.XResolution=XRes) and (MI.YResolution=YRes) and
       (MI.BitsPerPixel=BPX) and
       ((LFB=false) or ((MI.Attributes and vesa_vma_LinearFrameBufferAvail)<>0)) then
      begin Mode:=B.Modes[I]; ModeInfo:=MI; end;
    Inc(I);
  until (OK=false) or (I>=B.Count) or (Mode<>0);
  OK:=Mode<>0;
  VESASearchMode:=OK;
end;

function VESAGetOemString: string;
var OK: boolean;
    VI: TVESAInfoBlock;
    S: array[0..256] of char;
begin
  if disableVESA then
    begin
      VESAGetOemString:='VESA disabled';
      exit;
    end;
  FillChar(S,SizeOf(S),0);
  OK:=VESAGetInfo(VI);
  if OK then
    OK:=MoveDosToPM(VI.OemString,@S,sizeof(S));
  VESAGetOemString:=StrPas(@S);
end;

function VESAGetModeInfo(Mode: word; var B: TVESAModeInfoBlock): boolean;
var r : registers;
    M : MemPtr;
    OK: boolean;
begin
  if disableVESA then
    exit(false);
  r.ah:=$4f; r.al:=$01; r.cx:=Mode;
  GetDosMem(M,sizeof(B));
  r.es:=M.DosSeg; r.di:=M.DosOfs; {r.ds:=r.es;}
  realintr($10,r);
  M.MoveDataFrom(sizeof(B),B);
  FreeDosMem(M);
  OK:=(r.ax=$004f);
  VESAGetModeInfo:=OK;
end;

function RegisterVesaVideoMode(Mode : word) : boolean;
var B: TVESAModeInfoBlock;
    VH : PVesaVideoMode;
    DoAdd : boolean;
begin
  if not VESAGetModeInfo(Mode,B) then
    RegisterVesaVideoMode:=false
  else
    begin
      VH:=VesaVideoModeHead;
      DoAdd:=true;
      RegisterVesaVideoMode:=false;
      while assigned(VH) do
        begin
          if VH^.mode=mode then
            DoAdd:=false;
          VH:=VH^.next;
        end;
      if DoAdd then
        begin
          New(VH);
          VH^.next:=VesaVideoModeHead;
          VH^.mode:=mode;
          VH^.IsGraphic:=(B.Attributes and vesa_vma_GraphicsMode)<>0;
          VH^.v.color:=(B.Attributes and vesa_vma_ColorMode)<>0;
          if VH^.IsGraphic then
            begin
              VH^.v.col:=B.XResolution div 8;
              VH^.v.row:=B.YResolution div 8;
            end
          else
            begin
              VH^.v.col:=B.XResolution;
              VH^.v.row:=B.YResolution;
            end;
          VH^.VideoIndex:=VesaRegisteredModes;
          Inc(VesaRegisteredModes);
          RegisterVesaVideoMode:=true;
          VesaVideoModeHead:=VH;
        end;
    end;
end;

function VESASetMode(Mode: word): boolean;
var r: registers;
    OK: boolean;
begin
  if disableVESA then
    exit(false);
  r.ah:=$4f; r.al:=$02; r.bx:=Mode;
  dos.intr($10,r);
  OK:=(r.ax=$004f);
  VESASetMode:=OK;
end;

function VESAGetMode(var Mode: word): boolean;
var r : registers;
    OK: boolean;
begin
  if disableVESA then
    exit(false);
  if disableVESA then
    exit(false);
  r.ah:=$4f; r.al:=$03;
  dos.intr($10,r);
  OK:=(r.ax=$004f);
  if OK then Mode:=r.bx;
  VESAGetMode:=OK;
end;

function VESASelectMemoryWindow(Window: byte; Position: word): boolean;
var r : registers;
    OK : boolean;
begin
  if disableVESA then
    exit(false);
  r.ah:=$4f; r.al:=$05; r.bh:=0; r.bl:=Window; r.dx:=Position;
  dos.intr($10,r);
  OK:=(r.ax=$004f);
  VESASelectMemoryWindow:=OK;
end;

function VESAReturnMemoryWindow(Window: byte; var Position: word): boolean;
var r  : registers;
    OK : boolean;
begin
  if disableVESA then
    exit(false);
  r.ah:=$4f; r.al:=$05; r.bh:=1; r.bl:=Window;
  dos.intr($10,r);
  OK:=(r.ax=$004f);
  if OK then Position:=r.dx;
  VESAReturnMemoryWindow:=OK;
end;

function VESAInit: boolean;
var OK: boolean;
    VI: TVESAInfoBlock;
begin
  if disableVESA then
    OK:=false
  else
    OK:=VESAGetInfo(VI);
  VESAInit:=OK;
end;

Function VesaGetVideoModeData (Index : Word; Var Data : TVideoMode) : boolean;
Var
  PrevCount : word;
  VH : PVesaVideoMode;

begin
  PrevCount:=SysGetVideoModeCount();
  VesaGetVideoModeData:=(Index<PrevCount);
  If VesaGetVideoModeData then
    begin
      VesaGetVideoModeData:=SysGetVideoModeData(Index,Data);
      exit;
    end;
  VesaGetVideoModeData:=(Index-PrevCount)<VesaRegisteredModes;
  If VesaGetVideoModeData then
    begin
      VH:=VesaVideoModeHead;
      while assigned(VH) and (VH^.VideoIndex<>Index-PrevCount) do
        VH:=VH^.next;
      if assigned(VH) then
        Data:=VH^.v
      else
        VesaGetVideoModeData:=false;
    end;
end;

function SetVESAMode(const VideoMode: TVideoMode): Boolean;

  var
     res : boolean;
     VH : PVesaVideoMode;

  begin
     res:=false;
     if disableVESA then
       exit(res);
     VH:=VesaVideoModeHead;
     while assigned(VH) do
       begin
         if (VideoMode.col=VH^.v.col) and
            (VideoMode.row=VH^.v.row) and
            (VideoMode.color=VH^.v.color) then
           begin
{$ifdef TESTGRAPHIC}
             if VH^.IsGraphic then
               begin
                 if IsGraphicMode then
                   CloseGraph;
                 GraphDriver:=Graph.Vesa;
                 if (VideoMode.col = 100) and (VideoMode.row = 75) then
                   GraphMode:=m800x600x256
                 else if (VideoMode.col = 80) and (VideoMode.row = 60) then
                   GraphMode:=m640x480x256
                 else if (VideoMode.col = 128) and (VideoMode.row = 96) then
                   GraphMode:=m1024x768x256
                 else
                   GraphMode:=Graph.Detect;
                 InitGraph(GraphDriver,GraphMode,'');
                 res:=(GraphResult=grOK);
                 if not res then
                   begin
                     SetVesaMode:=false;
                     exit;
                   end;
               end
             else
{$endif TESTGRAPHIC}
               res:=VESASetMode(VH^.mode);
             if res then
               begin
                  ScreenWidth:=VideoMode.Col;
                  ScreenHeight:=VideoMode.Row;
                  ScreenColor:=VideoMode.Color;
{$ifdef TESTGRAPHIC}
                  IsGraphicMode:=VH^.IsGraphic;
                  FirstCallAfterSetVesaMode:=true;
                  LastCursorX:=$ffff;
                  LastCursorY:=$ffff;
                  LastCursorType:=crHidden;
                  if IsGraphicMode then
                    DoCustomMouse(false)
                  else
{$endif TESTGRAPHIC}
                    DoCustomMouse(true);
               end;
           end;
         if res then
           begin
             SetVesaMode:=true;
             exit;
           end;
         VH:=VH^.next;
       end;
     SetVESAMode:=SysSetVideoMode(VideoMode);
  end;

procedure VesaSetCursorPos(NewCursorX, NewCursorY: Word);
begin
{$ifdef TESTGRAPHIC}
  if not IsGraphicMode then
{$endif TESTGRAPHIC}
    begin
      SysSetCursorPos(NewCursorX,NewCursorY);
      exit;
    end;
{$ifdef TESTGRAPHIC}
  if (NewCursorX<>LastCursorX) or (NewCursorY<>LastCursorY) then
    begin
      Case GetCursorType of
        crHidden  : ;
        crUnderLine :
          Begin
            PutImage(LastCursorX*8,LastCursorY*8+7,UnderLineImage,XORPut);
            PutImage(NewCursorX*8,NewCursorY*8+7,UnderLineImage,XORPut);
          End;
        crBlock     :
          Begin
            PutImage(LastCursorX*8,LastCursorY*8,BlockImage,XORPut);
            PutImage(NewCursorX*8,NewCursorY*8,BlockImage,XORPut);
          End;
        crHalfBlock :
          Begin
            PutImage(LastCursorX*8,LastCursorY*8+4,HalfBlockImage,XORPut);
            PutImage(NewCursorX*8,NewCursorY*8+4,HalfBlockImage,XORPut);
          End;
      end;
      LastCursorX:=NewCursorX;
      LastCursorY:=NewCursorY;
    end;
{$endif TESTGRAPHIC}
end;

procedure VesaSetCursorType(NewType : Word);
begin
{$ifdef TESTGRAPHIC}
  if not IsGraphicMode then
{$endif TESTGRAPHIC}
    begin
      SysSetCursorType(NewType);
      exit;
    end;
{$ifdef TESTGRAPHIC}
  if (NewType<>LastCursorType) then
    begin
      Case LastCursorType of
        crHidden  : ;
        crUnderLine :
          Begin
            PutImage(LastCursorX*8,LastCursorY*8+7,UnderLineImage,XORPut);
          End;
        crBlock     :
          Begin
            PutImage(LastCursorX*8,LastCursorY*8,BlockImage,XORPut);
          End;
        crHalfBlock :
          Begin
            PutImage(LastCursorX*8,LastCursorY*8+4,HalfBlockImage,XORPut);
          End;
      end;
      SysSetCursorType(NewType);
      Case NewType of
        crHidden  : ;
        crUnderLine :
          Begin
            PutImage(LastCursorX*8,LastCursorY*8+7,UnderLineImage,XORPut);
          End;
        crBlock     :
          Begin
            PutImage(LastCursorX*8,LastCursorY*8,BlockImage,XORPut);
          End;
        crHalfBlock :
          Begin
            PutImage(LastCursorX*8,LastCursorY*8+4,HalfBlockImage,XORPut);
          End;
      end;
      LastCursorType:=NewType;
    end;
{$endif TESTGRAPHIC}
end;

procedure VesaUpdateScreen(Force: Boolean);
{$ifdef TESTGRAPHIC}
var
  StoreDrawTextBackground,
  MustUpdate : boolean;
  x,y : longint;
  w, prevcolor,
  prevbkcolor, StoreCursorType : word;
  Color,BkCol,Col : byte;
  Ch : char;
{$endif TESTGRAPHIC}
begin
{$ifdef TESTGRAPHIC}
  if not IsGraphicMode then
{$endif TESTGRAPHIC}
    begin
      SysUpdateScreen(Force);
      exit;
    end;
{$ifdef TESTGRAPHIC}
  if FirstCallAfterSetVesaMode then
    begin
      { Make sure to redraw all }
      Fillchar(OldVideoBuf^,VideoBufSize,#0);
      FirstCallAfterSetVesaMode:=false;
    end;
  if not force then
   begin
     MustUpdate:=false;
     asm
        movl    VideoBuf,%esi
        movl    OldVideoBuf,%edi
        movl    VideoBufSize,%ecx
        shrl    $2,%ecx
        repe
        cmpsl
        setne   MustUpdate
     end;
   end;
  StoreDrawTextBackground:=DrawTextBackground;
  DrawTextBackground:=true;
  if Force or MustUpdate then
   begin
     PrevColor:=GetColor;
     PrevBkColor:=GetBkColor{$ifdef FPC}(){$endif};

     for y:=0 to ScreenHeight-1 do
       for x:=0 to Screenwidth-1 do
         begin
           w:=VideoBuf^[x+y*ScreenWidth];
           if Force or
              (w<>OldVideoBuf^[x+y*ScreenWidth]) then
             Begin
               Color:=w shr 8;
               Ch:=chr(w and $ff);
               Col:=Color and $f;
               if (Col = 0) and (GetMaxColor=255) then
                 Col:=255;
               SetColor(Col);
               BkCol:=(Color shr 4) and 7;
               if (BkCol = 0) and (GetMaxColor=255) then
                 BkCol:=255;
               SetBkColor(BkCol);
               if (x=LastCursorX) and (Y=LastCursorY) then
                 begin
                   StoreCursorType:=LastCursorType;
                   VesaSetCursorType(crHidden);
                 end;
               OutTextXY(x*8,y*8,Ch);
               if (x=LastCursorX) and (Y=LastCursorY) then
                 VesaSetCursorType(StoreCursorType);
               if not force then
                 OldVideoBuf^[x+y*ScreenWidth]:=w;
             End;
         end;
     if Force then
       move(videobuf^,oldvideobuf^,
         VideoBufSize);
     SetColor(PrevColor);
     SetBkColor(GetBkColor{$ifdef FPC}(){$endif});
   end;
  DrawTextBackground:=StoreDrawTextBackground;
{$endif TESTGRAPHIC}
end;

procedure VesaDoneVideo;
begin
{$ifdef TESTGRAPHIC}
  if IsGraphicMode then
    begin
      CloseGraph;
      IsGraphicMode:=false;
    end;
{$endif TESTGRAPHIC}
  SysDoneVideo();
end;

function SetVESAVideoDriver : boolean; forward;

procedure VesaInitVideo;
begin
  if not SetVESAVideoDriver then
    exit;
{$ifdef TESTGRAPHIC}
  if IsGraphicMode then
    begin
      SysInitVideo();
      InitGraph(GraphDriver,GraphMode,'');
    end
  else
{$endif TESTGRAPHIC}
    SysInitVideo();
end;

Function VesaGetVideoModeCount : Word;

begin
  VesaGetVideoModeCount:=SysGetVideoModeCount()+VesaRegisteredModes;
end;

Procedure FreeVesaModes;
var
  VH : PVesaVideoMode;
begin
  VH:=VesaVideoModeHead;
  While assigned(VH) do
    begin
      VesaVideoModeHead:=VH^.Next;
      FreeMem(VH,Sizeof(TVesaVideoMode));
      VH:=VesaVideoModeHead;
    end;
end;

Var
  Driver : TVideoDriver;
{$ifdef TESTGRAPHIC}
  i : longint;
{$endif TESTGRAPHIC}

function SetVESAVideoDriver : boolean;
BEGIN
  if disableVESA then
    exit(false);
{ Get the videodriver to be used }
  GetVideoDriver (Driver);
{ Change needed functions }
  SysGetVideoModeCount:=Driver.GetVideoModeCount;
  Driver.GetVideoModeCount:=@VesaGetVideoModeCount;
  SysGetVideoModeData:=Driver.GetVideoModeData;
  Driver.GetVideoModeData:=@VesaGetVideoModeData;
  SysSetVideoMode:=Driver.SetVideoMode;
  Driver.SetVideoMode:=@SetVESAMode;
  SysSetCursorPos:=Driver.SetCursorPos;
  Driver.SetCursorPos:=@VESASetCursorPos;
  SysSetCursorType:=Driver.SetCursorType;
  Driver.SetCursorType:=@VESASetCursorType;
  SysUpdateScreen:=Driver.UpdateScreen;
  Driver.UpdateScreen:=@VesaUpdateScreen;
  SysDoneVideo:=Driver.DoneDriver;
  Driver.DoneDriver:=@VesaDoneVideo;
  SysInitVideo:=Driver.InitDriver;
  Driver.InitDriver:=@VesaInitVideo;

{$ifdef TESTGRAPHIC}
  BlockImage.width:=7;
  BlockImage.height:=7;
  For i:=0 to 8*8-1 do
    BlockImage.colors[i]:=White;
  HalfBlockImage:=BlockImage;
  HalfBlockImage.height:=3;
  UnderLineImage:=BlockImage;
  UnderLineImage.height:=0;
{$endif TESTGRAPHIC}

  SetVideoDriver (Driver);
<<<<<<< HEAD
<<<<<<< HEAD
  SetVESAVideoDriver:=true;
END;

function ChkWinNT: boolean;
var
  R: Registers;
begin
  ChkWinNT := false;
  R.AX := $3306;
  RealIntr ($21, R);
  if (R.AL = 255) or (R.BX <> 50 * 256 + 5) then
   Exit;
  R.AX := $3000;
  RealIntr ($21, R);
  if (R.AX = 5) and (R.BH = 255) then
   ChkWinNT := true;
end;

begin
(* Let's disable VESA functions by default if running under MS Windows NT+ *)
  if ChkWinNT then
   DisableVESA := true;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
END.
