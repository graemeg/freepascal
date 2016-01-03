{
    This file is part of the Free Pascal Integrated Development Environment
    Copyright (c) 1998 by Berczi Gabor

    User screen support routines

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{$i globdir.inc}
unit FPUsrScr;

interface

uses
{$ifdef Windows}
  windows,
{$endif Windows}
{$ifdef Unix}
  baseunix,
  termio,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef linux}
  linuxvcs,
{$endif}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
{$ifdef linux}
  linuxvcs,
{$endif}
>>>>>>> origin/fixes_2.4
{$endif}
  video,Objects;

type

    PScreen = ^TScreen;
    TScreen = object(TObject)
      function    GetWidth: integer; virtual;
      function    GetHeight: integer; virtual;
      procedure   GetLine(Line: integer; var Text, Attr: string); virtual;
      procedure   GetCursorPos(var P: TPoint); virtual;
      { remember the initial video screen }
      procedure   Capture; virtual;
      { restore the initial video mode }
      procedure   Restore; virtual;
      { move up or down if supported by OS }
      function    Scroll(i : integer) : integer; virtual;
      { is moving supported by OS }
      function    CanScroll : boolean; virtual;
      { saves the current IDE screen }
      procedure   SaveIDEScreen; virtual;
      { saves the current console screen }
      procedure   SaveConsoleScreen; virtual;
      { restores the saved console screen }
      procedure   SwitchToConsoleScreen; virtual;
      { restores the saved IDE screen }
      procedure   SwitchBackToIDEScreen; virtual;
    end;

{$IFDEF netwlibc}
    PNWLScreen = ^TNWLScreen;
    TNWLScreen = object(TScreen)
      function    GetWidth: integer; virtual;
      function    GetHeight: integer; virtual;
      procedure   GetLine(Line: integer; var Text, Attr: string); virtual;
      procedure   GetCursorPos(var P: TPoint); virtual;
      { remember the initial video screen }
      procedure   Capture; virtual;
      { restore the initial video mode }
      procedure   Restore; virtual;
      { saves the current IDE screen }
      procedure   SaveIDEScreen; virtual;
      { saves the current console screen }
      procedure   SaveConsoleScreen; virtual;
      { restores the saved console screen }
      procedure   SwitchToConsoleScreen; virtual;
      { restores the saved IDE screen }
      procedure   SwitchBackToIDEScreen; virtual;
    end;
{$ENDIF}

{$IFDEF AMIGA}
  {$DEFINE AMIGASCREEN}
{$ENDIF}

{$IFDEF MORPHOS}
  {$DEFINE AMIGASCREEN}
{$ENDIF}

{$IFDEF AROS}
  {$DEFINE AMIGASCREEN}
{$ENDIF}


{$IFDEF AMIGASCREEN}
    PAmigaScreen = ^TAmigaScreen;
    TAmigaScreen = object(TScreen)
      function    GetWidth: integer; virtual;
      function    GetHeight: integer; virtual;
      procedure   GetLine(Line: integer; var Text, Attr: string); virtual;
      procedure   GetCursorPos(var P: TPoint); virtual;
      { remember the initial video screen }
      procedure   Capture; virtual;
      { restore the initial video mode }
      procedure   Restore; virtual;
      { saves the current IDE screen }
      procedure   SaveIDEScreen; virtual;
      { saves the current console screen }
      procedure   SaveConsoleScreen; virtual;
      { restores the saved console screen }
      procedure   SwitchToConsoleScreen; virtual;
      { restores the saved IDE screen }
      procedure   SwitchBackToIDEScreen; virtual;
    end;
{$ENDIF}

{$IFDEF OS2}
    POS2Screen = ^TOS2Screen;
    TOS2Screen = object(TScreen)
      constructor Init;
      destructor  Done; virtual;
    public
      function    GetWidth: integer; virtual;
      function    GetHeight: integer; virtual;
      procedure   GetLine(Line: integer; var Text, Attr: string); virtual;
      procedure   GetCursorPos(var P: TPoint); virtual;
      { remember the initial video screen }
      procedure   Capture; virtual;
      { restore the initial video mode }
      procedure   Restore; virtual;
      { move up or down if supported by OS }
      function    Scroll(i : integer) : integer; virtual;
      { saves the current IDE screen }
      procedure   SaveIDEScreen; virtual;
      { saves the current console screen }
      procedure   SaveConsoleScreen; virtual;
      { restores the saved console screen }
      procedure   SwitchToConsoleScreen; virtual;
      { restores the saved IDE screen }
      procedure   SwitchBackToIDEScreen; virtual;
    end;
{$ENDIF}

{$ifdef DOS}
    TDOSVideoInfo = record
      Mode      : word;
      ScreenSize: word;
      Page      : byte;
      Rows,Cols : integer;
      CurPos    : TPoint;
      CurShapeT : integer;
      CurShapeB : integer;
      StateSize : word;
      StateBuf  : pointer;
    end;

    PDOSScreen = ^TDOSScreen;
    TDOSScreen = object(TScreen)
      constructor Init;
      destructor  Done; virtual;
    public
      function    GetWidth: integer; virtual;
      function    GetHeight: integer; virtual;
      procedure   GetLine(Line: integer; var Text, Attr: string); virtual;
      procedure   GetCursorPos(var P: TPoint); virtual;
      procedure   Capture; virtual;
      procedure   Restore; virtual;
      procedure   SaveIDEScreen; virtual;
      procedure   SaveConsoleScreen; virtual;
      procedure   SwitchToConsoleScreen; virtual;
      procedure   SwitchBackToIDEScreen; virtual;
      procedure   FreeGraphBuffer;
    private
      LastTextConsoleVideoInfo,
      ConsoleVideoInfo : TDOSVideoInfo;
      VBufferSize  : longint;
      VIDEBufferSize : longint;
      VBuffer      : PByteArray;
      VIDEBuffer   : PByteArray;
      IDEVideoInfo : TDOSVideoInfo;
      ctrl_c_state : boolean;
{$ifdef USE_GRAPH_SWITCH}
      GraphImageSize : longint;
      GraphDriverName,
      GraphModeName : string;
      GraphXres,GraphYres : longint;
      GraphBuffer : pointer;
      GraphCGABkColor: Integer;
      ConsoleGraphDriver, ConsoleGraphMode : smallint;
{$endif USE_GRAPH_SWITCH}
      function    GetLineStartOfs(Line: integer): word;
      procedure   GetBuffer(Size: word);
      procedure   FreeBuffer;
      procedure   GetVideoMode(var MI: TDOSVideoInfo);
      procedure   SetVideoMode(MI: TDOSVideoInfo);
    end;
{$endif}

{$ifdef Unix}

    TConsoleType = (ttyNetwork,ttyLinux,ttyFreeBSD,ttyNetBSD);

    PLinuxScreen = ^TLinuxScreen;
    TLinuxScreen = object(TScreen)
      constructor Init;
      destructor  Done; virtual;
    public
      function    GetWidth: integer; virtual;
      function    GetHeight: integer; virtual;
      procedure   GetLine(Line: integer; var Text, Attr: string); virtual;
      procedure   GetCursorPos(var P: TPoint); virtual;
      procedure   Capture; virtual;
      procedure   Restore; virtual;
      procedure   SaveIDEScreen; virtual;
      procedure   SaveConsoleScreen; virtual;
      procedure   SwitchToConsoleScreen; virtual;
      procedure   SwitchBackToIDEScreen; virtual;
    private
      IdeScreen: PByteArray;
      IdeSize : longint;
      IsXterm : boolean;
      Console : TConsoleType;
      TTyfd : longint;
      ConsVideoBuf : PByteArray;
      ConsHeight, ConsWidth,
      ConsCursorX, ConsCursorY : byte;
      ConsVideoBufSize : longint;
      ConsTio : termios;
      ConsTioValid : boolean;
    end;
{$endif}

{$ifdef Windows}
    PWindowsScreen = ^TWindowsScreen;
    TWindowsScreen = object(TScreen)
      constructor Init;
      destructor  Done; virtual;
    public
      function    GetWidth: integer; virtual;
      function    GetHeight: integer; virtual;
      procedure   GetLine(Line: integer; var Text, Attr: string); virtual;
      procedure   GetCursorPos(var P: TPoint); virtual;
      function    CanScroll : boolean; virtual;
      function    Scroll(i : integer) : integer; virtual;
      procedure   Capture; virtual;
      procedure   Restore; virtual;
      procedure   SaveIDEScreen; virtual;
      procedure   SaveConsoleScreen; virtual;
      procedure   SwitchToConsoleScreen; virtual;
      procedure   SwitchBackToIDEScreen; virtual;
    private
      DosScreenBufferHandle,
      IDEScreenBufferHandle,
      StartScreenBufferHandle,
      DummyScreenBufferHandle,
      NewScreenBufferHandle : THandle;
      IDEActive : boolean;
      ConsoleMode,IdeMode      : Dword;
      IdeScreenMode : TVideoMode;
      procedure BufferCopy(src,dest : THandle);
{$ifdef debug}
      procedure Complain(St : string);
      Procedure SetConsoleMode(FH : Handle;Mode : DWord);
{$endif debug}
    end;
{$endif}

procedure InitUserScreen;
procedure DoneUserScreen;

const UserScreen : PScreen = nil;

implementation

uses
  Dos,WUtils
  {$ifdef GO32V2}
  ,Dpmiexcp, Go32
  {$endif}
    ,Drivers,App
  {$ifdef USE_GRAPH_SWITCH}
    ,Graph,VESA
  {$else not USE_GRAPH_SWITCH}
  {$ifdef VESA}
    ,VESA
  {$endif VESA}
  {$endif not USE_GRAPH_SWITCH}
  ;

function TScreen.GetWidth: integer;
begin
  Getwidth:=0;
  Abstract;
end;

function TScreen.GetHeight: integer;
begin
  Getheight:=0;
  Abstract;
end;

procedure TScreen.GetLine(Line: integer; var Text, Attr: string);
begin
  Abstract;
end;

procedure TScreen.GetCursorPos(var P: TPoint);
begin
  Abstract;
end;

procedure TScreen.Capture;
begin
  Abstract;
end;

procedure TScreen.Restore;
begin
  Abstract;
end;

procedure TScreen.SwitchToConsoleScreen;
begin
  Abstract;
end;

procedure TScreen.SwitchBackToIDEScreen;
begin
  Abstract;
end;

procedure TScreen.SaveIDEScreen;
begin
  Abstract;
end;

function TScreen.Scroll(i : integer) : integer;
begin
  Scroll:=0;
end;

function TScreen.CanScroll : boolean;
begin
  CanScroll:=false;
end;

procedure TScreen.SaveConsoleScreen;
begin
  Abstract;
end;


{****************************************************************************
                                 TDOSScreen
****************************************************************************}

{$ifdef DOS}

constructor TDOSScreen.Init;
begin
  inherited Init;
  FillChar(LastTextConsoleVideoInfo,Sizeof(TDOSVideoInfo),#0);
  Capture;
  { get the current ctrl-C state }
  Ctrl_c_state:=djgpp_set_ctrl_c(false);
  djgpp_set_ctrl_c(Ctrl_c_state);
end;


destructor TDOSScreen.Done;
begin
  FreeBuffer;
  if assigned(VIDEBuffer) then
    FreeMem(VIDEBuffer,VIDEBufferSize);
  inherited Done;
end;


function TDOSScreen.GetWidth: integer;
begin
  GetWidth:=ConsoleVideoInfo.Cols;
end;


function TDOSScreen.GetHeight: integer;
begin
  GetHeight:=ConsoleVideoInfo.Rows;
end;


procedure TDOSScreen.GetLine(Line: integer; var Text, Attr: string);
var
  X: integer;
  W: word;
begin
  Text:=''; Attr:='';
  if (Line<GetHeight) and
{$ifdef USE_GRAPH_SWITCH}
     not assigned(GraphBuffer) and
{$endif USE_GRAPH_SWITCH}
     assigned(VBuffer) then
    begin
      W:=GetLineStartOfs(Line);
      for X:=0 to GetWidth-1 do
        begin
          {Text:=Text+chr(VBuffer^[W+X*2]);
          Attr:=Attr+chr(VBuffer^[W+X*2+1]);}
          System.Insert(chr(VBuffer^[W+X*2]),Text,Length(Text)+1);
          System.Insert(chr(VBuffer^[W+X*2+1]),Attr,Length(Attr)+1);
        end;
{$ifdef USE_GRAPH_SWITCH}
    end
  else if assigned(GraphBuffer) then
    begin
      if (Line=0) then
        Text:='Console in graph mode, use Alt+F5'
      else if (Line=1) then
        Text:='Graph driver: '+GraphDriverName
      else if (Line=2) then
        Text:='Graph mode: '+GraphModeName+' ('+
              IntToStr(GraphXres+1)+'x'+IntToStr(GraphYres+1)+')';
      Attr:=CharStr(chr($0F),Length(Text));
    end;
{$else not USE_GRAPH_SWITCH}
  end;
{$endif USE_GRAPH_SWITCH}
end;


procedure TDOSScreen.GetCursorPos(var P: TPoint);
begin
  P:=ConsoleVideoInfo.CurPos;
end;


procedure TDOSScreen.Capture;
begin
  SaveConsoleScreen;
end;

procedure TDOSScreen.FreeGraphBuffer;
begin
  { We don't want to restore the last user screen if
    it was a grpahic screen, for example if we
    leave in the middle of the debugging of a
    graphic program, so we first
    dispose the graphic buffer, thus
    SwitchToConsoleScreen will restore the
    last used text mode }
  if LastTextConsoleVideoInfo.Mode<>0 then
    begin
      ConsoleVideoInfo:=LastTextConsoleVideoInfo;
{$ifdef USE_GRAPH_SWITCH}
      if assigned(GraphBuffer) then
        begin
          FreeMem(GraphBuffer,GraphImageSize);
          GraphBuffer:=nil;
          GraphImageSize:=0;
        end;
{$endif USE_GRAPH_SWITCH}
    end;
end;

procedure TDosScreen.Restore;
begin
  FreeGraphBuffer;
  SwitchToConsoleScreen;
end;

procedure TDosScreen.SaveIDEScreen;
var
  VSeg,SOfs: word;
begin
  GetVideoMode(IDEVideoInfo);
  { First keep a copy of IDE screen }
  if ConsoleVideoInfo.Mode=7 then
   VSeg:=SegB000
  else
   VSeg:=SegB800;
  SOfs:=MemW[Seg0040:$4e];
  if not assigned(VIDEBuffer) or (VIDEBufferSize<>IDEVideoInfo.ScreenSize) then
    begin
      if assigned(VIDEBuffer) then
        FreeMem(VIDEBuffer,VIDEBufferSize);
      GetMem(VIDEBuffer,IDEVideoInfo.ScreenSize);
      VIDEBufferSize:=IDEVideoInfo.ScreenSize;
    end;
<<<<<<< HEAD
<<<<<<< HEAD
  HideMouse;
  DosmemGet(VSeg,SOfs,VIDEBuffer^,IDEVideoInfo.ScreenSize);
  ShowMouse;
=======
  DosmemGet(VSeg,SOfs,VIDEBuffer^,IDEVideoInfo.ScreenSize);
>>>>>>> graemeg/fixes_2_2
=======
  DosmemGet(VSeg,SOfs,VIDEBuffer^,IDEVideoInfo.ScreenSize);
>>>>>>> origin/fixes_2_2
end;

procedure TDosScreen.SaveConsoleScreen;
var
  VSeg,SOfs: word;
{$ifdef USE_GRAPH_SWITCH}
  saved : boolean;
  GraphDriver,GraphMode : integer;
{$endif USE_GRAPH_SWITCH}
begin
  GetVideoMode(ConsoleVideoInfo);
{$ifdef USE_GRAPH_SWITCH}
  saved:=false;
  if assigned(GraphBuffer) then
    begin
      FreeMem(GraphBuffer,GraphImageSize);
      GraphBuffer:=nil;
      GraphImageSize:=0;
    end;
  if (ConsoleVideoInfo.Mode>= $100) or
     (ConsoleVideoInfo.Mode=$13) or
     (ConsoleVideoInfo.Mode=$12) or
     (ConsoleVideoInfo.Mode=$10) or
     (ConsoleVideoInfo.Mode=$E) or
     (ConsoleVideoInfo.Mode=$6) or
     (ConsoleVideoInfo.Mode=$4) then
    begin
      Graph.DontClearGraphMemory:=true;
      if ConsoleVideoInfo.Mode>=$100 then
        begin
          GraphDriver:=Graph.Vesa;
          GraphMode:=ConsoleVideoInfo.Mode and $fff;
        end
      else
        begin
          case ConsoleVideoInfo.Mode of
           $4 : begin
                  GraphDriver:=Graph.CGA;
                  case (Mem[$40:$66] shr 4) and 3 of
                    0: GraphMode:=CGAC2;
                    1: GraphMode:=CGAC0;
                    2: GraphMode:=CGAC3;
                    3: GraphMode:=CGAC1;
                  end;
                  GraphCGABkColor:=Mem[$40:$66] and $0F;
                end;
           $6 : begin
                  GraphDriver:=Graph.CGA;
                  GraphMode:=CGAHi;
                end;
           $E : begin
                  GraphDriver:=Graph.VGA;
                  GraphMode:=VGALo;
                end;
           $10 : begin
                   GraphDriver:=Graph.VGA;
                   GraphMode:=VGAMed;
                 end;
           $12 : begin
                   GraphDriver:=Graph.VGA;
                   GraphMode:=VGAHi;
                 end;
           $13 : begin
                   GraphDriver:=Graph.LowRes;
                   GraphMode:=0;
                 end;
          end;
        end;
      Graph.InitGraph(GraphDriver,GraphMode,'');
      if graphresult=grOk then
        begin
          ConsoleGraphDriver:=GraphDriver;
          GraphDriverName:=GetDriverName;
          GraphModeName:=GetModeName(GraphMode);
          ConsoleGraphMode:=GraphMode;
          Graph.DontClearGraphMemory:=false;
          GraphXres:=Graph.GetmaxX;
          GraphYres:=Graph.GetmaxY;
          GraphImageSize:=ImageSize(0,0,GraphXres,GraphYres);
          GetMem(GraphBuffer,GraphImageSize);
          FillChar(GraphBuffer^,GraphImageSize,#0);
          GetImage(0,0,GraphXres,GraphYres,GraphBuffer^);
          ConsoleVideoInfo.Rows:=GraphYres div 8;
          ConsoleVideoInfo.Cols:=GraphXres div 8;
          {FreeBuffer;}
          saved:=true;
        end
{$ifdef DEBUG}
      else
        Writeln(stderr,'Error in InitGraph ',Graphdriver, ' ',Graphmode)
{$endif DEBUG}
        ;
    end;
  { mode < $100 so use standard Save code }
  if not saved then
{$endif USE_GRAPH_SWITCH}
  begin
    LastTextConsoleVideoInfo:=ConsoleVideoInfo;
    GetBuffer(ConsoleVideoInfo.ScreenSize);
    if ConsoleVideoInfo.Mode=7 then
     VSeg:=SegB000
    else
     VSeg:=SegB800;
    SOfs:=MemW[Seg0040:$4e];
    DosmemGet(VSeg,SOfs,VBuffer^,ConsoleVideoInfo.ScreenSize);
  end;
end;

procedure TDOSScreen.SwitchToConsoleScreen;
var
  VSeg,SOfs: word;
{$ifdef USE_GRAPH_SWITCH}
  restored : boolean;
{$endif USE_GRAPH_SWITCH}
begin
{$ifdef USE_GRAPH_SWITCH}
  restored:=false;
  if assigned(GraphBuffer) then
    begin
      Graph.InitGraph(ConsoleGraphDriver,ConsoleGraphMode,'');
      if graphresult=grOk then
        begin
          if (ConsoleGraphDriver=CGA) and
             (ConsoleGraphMode>=CGAC0) and
             (ConsoleGraphMode<=CGAC3) then
            SetBkColor(GraphCGABkColor);
          PutImage(0,0,GraphBuffer^,CopyPut);
          FreeMem(GraphBuffer,GraphImageSize);
          GraphBuffer:=nil;
          GraphImageSize:=0;
          restored:=true;
        end;
    end;
  { mode < $100 so use standard Save code }
  if not restored then
{$endif USE_GRAPH_SWITCH}
    begin
      SetVideoMode(ConsoleVideoInfo);
      if ConsoleVideoInfo.Mode=7 then
        VSeg:=SegB000
      else
        VSeg:=SegB800;
      SOfs:=MemW[Seg0040:$4e];
      DosmemPut(VSeg,SOfs,VBuffer^,ConsoleVideoInfo.ScreenSize);
      djgpp_set_ctrl_c(Ctrl_c_state);
    end;
end;


procedure TDOSScreen.SwitchBackToIDEScreen;
var
  VSeg,SOfs: word;
begin
  SetVideoMode(IDEVideoInfo);
  if ConsoleVideoInfo.Mode=7 then
   VSeg:=SegB000
  else
   VSeg:=SegB800;
  SOfs:=MemW[Seg0040:$4e];
  if assigned(VIDEBuffer) then
  DosmemPut(VSeg,SOfs,VIDEBuffer^,IDEVideoInfo.ScreenSize);
  Ctrl_c_state := djgpp_set_ctrl_c(false);
  { Its difficult to know
    the state of the mouse
    so simply show it always
    fixes bug 2253 PM }
  ShowMouse;
end;


function TDOSScreen.GetLineStartOfs(Line: integer): word;
begin
  GetLineStartOfs:=(ConsoleVideoInfo.Cols*Line)*2;
end;


procedure TDOSScreen.GetBuffer(Size: word);
begin
  if (VBuffer<>nil) and (VBufferSize=Size) then Exit;
  if VBuffer<>nil then FreeBuffer;
  VBufferSize:=Size;
  GetMem(VBuffer,VBufferSize);
end;


procedure TDOSScreen.FreeBuffer;
begin
  if (VBuffer<>nil) and (VBufferSize>0) then FreeMem(VBuffer,VBufferSize);
  VBuffer:=nil;
end;


procedure TDOSScreen.GetVideoMode(var MI: TDOSVideoInfo);
var
  r: registers;
begin
  if (MI.StateSize>0) and (MI.StateBuf<>nil) then
     begin FreeMem(MI.StateBuf,MI.StateSize); MI.StateBuf:=nil; end;

  MI.ScreenSize:=MemW[Seg0040:$4c];
  r.ah:=$0f;
  intr($10,r);
  MI.Mode:=r.al;
  MI.Page:=r.bh;
  MI.Cols:=r.ah;
{$ifdef VESA}
  VESAGetMode(MI.Mode);
  MI.Mode:=MI.Mode and $fff;
{$endif}
  MI.Rows:=MI.ScreenSize div (MI.Cols*2);
  if MI.Rows=51 then MI.Rows:=50;
  r.ah:=$03;
  r.bh:=MI.Page;
  intr($10,r);
  with MI do
  begin
    CurPos.X:=r.dl; CurPos.Y:=r.dh;
    CurShapeT:=r.ch; CurShapeB:=r.cl;
  end;
end;


procedure TDOSScreen.SetVideoMode(MI: TDOSVideoInfo);
var r: registers;
    CM: TDOSVideoInfo;
begin
  FillChar(CM,sizeof(CM),0);
  GetVideoMode(CM);

  if (CM.Mode<>MI.Mode) or (CM.Cols<>MI.Cols) or (CM.Rows<>MI.Rows) then
   begin
     {$ifdef VESA}
     if MI.Mode>=$100 then
       VESASetMode(MI.Mode)
     else
     {$endif}
       begin
         r.ah:=$00; r.al:=MI.Mode; intr($10,r);
       end;
     if (MI.Mode=3) and (MI.Cols=80) and (MI.Rows=50) then
     begin
       r.ax:=$1112; r.bx:=$0;
       intr($10,r);
     end;
   end;
  r.ah:=$05; r.al:=MI.Page; intr($10,r);
  r.ah:=$02; r.bh:=MI.Page; r.dl:=MI.CurPos.X; r.dh:=MI.CurPos.Y; intr($10,r);
  r.ah:=$01; r.ch:=MI.CurShapeT; r.cl:=MI.CurShapeB; intr($10,r);
end;

{$endif}


{****************************************************************************
                                 TLinuxScreen
****************************************************************************}

{$ifdef Unix}

constructor TLinuxScreen.Init;
var
  ThisTTY: string[30];
  FName: string;
  WS: packed record
    ws_row, ws_col, ws_xpixel, ws_ypixel: Word;
  end;
begin
  inherited Init;
  IdeScreen := nil;
  TTYFd:=-1;
  IsXterm:=getenv('TERM')='xterm';
  ThisTTY:=TTYName(stdinputhandle);
  if Not IsXterm and (IsATTY(stdinputhandle)<>-1) then
    begin
      Console:=TTyNetwork;  {Default: Network or other vtxxx tty}
      if ((Copy(ThisTTY, 1, 8) = '/dev/tty') and (ThisTTY[9]<>'p')) or (Copy(ThisTTY,1,8)='/dev/vc/') Then
        begin
          Case ThisTTY[9] of
            '0'..'9' :
              begin { running Linux on native console or native-emulation }
{$ifdef linux}
                FName:='/dev/vcsa' + ThisTTY[9];
                TTYFd:=fpOpen(FName, &666, O_RdWr); { open console }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
                if TTYFd = -1 then
                begin
                  if try_grab_vcsa then
                    TTYFd:=fpOpen(FName, &666, O_RdWr); { try again }
                end;
<<<<<<< HEAD
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
                If TTYFd <>-1 Then
                  Console:=ttyLinux;
{$endif}
              end;
         'v'  :  { check for (Free?)BSD native}
                If (ThisTTY[10]>='0') and (ThisTTY[10]<='9') Then
                 Console:=ttyFreeBSD;   {TTYFd ?}
         end;
       end;
     If Copy(GetEnv('TERM'),1,6)='cons25' Then
       Console:=ttyFreeBSD;
     fpioctl(stdinputhandle, TIOCGWINSZ, @WS);
     if WS.ws_Col=0 then
       WS.ws_Col:=80;
     if WS.ws_Row=0 then
       WS.ws_Row:=25;
     ConsWidth:=WS.ws_Col;
     ConsHeight:=WS.ws_row;
   end;
  Capture;
end;


destructor TLinuxScreen.Done;
begin
  if assigned(IdeScreen) then
    freemem(IdeScreen,IdeSize);
  if assigned(ConsVideoBuf) then
    freemem(ConsVideoBuf,ConsVideoBufSize);
  inherited Done;
end;


function TLinuxScreen.GetWidth: integer;
begin
  GetWidth:=ConsWidth;
end;


function TLinuxScreen.GetHeight: integer;
begin
  GetHeight:=ConsHeight;
end;


procedure TLinuxScreen.GetLine(Line: integer; var Text, Attr: string);
var
  X, W : longint;
begin
  Text:='';
  Attr:='';
  if (TtyFd<>-1) and assigned(ConsVideoBuf) then
    begin
      if Line<GetHeight then
        begin
          W:=(ConsWidth*Line)*Sizeof(word);
          for X:=0 to GetWidth-1 do
             begin
               {Text:=Text+chr(VBuffer^[W+X*2]);
               Attr:=Attr+chr(VBuffer^[W+X*2+1]);}
               System.Insert(chr(ConsVideoBuf^[W+X*2]),Text,Length(Text)+1);
               System.Insert(chr(ConsVideoBuf^[W+X*2+1]),Attr,Length(Attr)+1);
             end;
        end;
    end;
end;


procedure TLinuxScreen.GetCursorPos(var P: TPoint);
begin
  P.X:=ConsCursorX+1;
  P.Y:=ConsCursorY+1;
end;


procedure TLinuxScreen.Capture;
begin
  SaveConsoleScreen;
end;

procedure TLinuxScreen.Restore;
begin
  SwitchToConsoleScreen;
end;

procedure TLinuxScreen.SaveIDEScreen;
begin
  if assigned(IdeScreen) then
    freemem(IdeScreen,IdeSize);
  getmem(IdeScreen,videobufsize);
  IdeSize:=videobufsize;
  move(videobuf^,IdeScreen^,videobufsize);
end;

procedure TLinuxScreen.SaveConsoleScreen;
var
  NewSize : longint;
begin
  if IsXTerm then
    write(#27'7'#27'[?47h')
  else if (TTYfd<>-1) then
    begin
     fpLSeek(TTYFd, 0, Seek_Set);
     fpread(TTYFd,ConsHeight,sizeof(byte));
     fpread(TTYFd,ConsWidth,sizeof(byte));
     fpread(TTYFd,ConsCursorX,sizeof(byte));
     fpread(TTYFd,ConsCursorY,sizeof(byte));
     NewSize:=ConsWidth*ConsHeight*sizeof(word);
     if (NewSize<>ConsVideoBufSize) and
        assigned(ConsVideoBuf) then
       Begin
         FreeMem(ConsVideoBuf,ConsVideoBufSize);
         ConsVideoBuf:=nil;
       End;
     If not assigned(ConsVideoBuf) then
       GetMem(ConsVideoBuf,NewSize);
     ConsVideoBufSize:=NewSize;
     fpread(TTYFd,ConsVideoBuf^,ConsVideoBufSize);
    end
  else
    begin
      ConsWidth:=80;
      ConsHeight:=25;
      ConsCursorX:=0;
      ConsCursorY:=0;
      ConsVideoBuf:=nil;
    end;
  ConsTioValid:=(TCGetAttr(1,ConsTio)<>-1);
end;


procedure TLinuxScreen.SwitchToConsoleScreen;
begin
  if IsXterm then
    begin
      write(#27'[0m');
      write(#27'[?47l'#27'8'#27'[m');
    end
  else if (TTyfd<>-1) then
    begin
      fplSeek(TTYFd, 2, Seek_Set);
      fpwrite(TTYFd, ConsCursorX, sizeof(byte));
      fpwrite(TTYFd, ConsCursorY, sizeof(byte));
      fpwrite(TTYFd, ConsVideoBuf^,ConsVideoBufSize);
      { FreeMem(ConsVideoBuf,ConsVideoBufSize);
      ConsVideoBuf:=nil; }
    end;
  If ConsTioValid then
    TCSetAttr(1,TCSANOW,ConsTio);
end;

procedure TLinuxScreen.SwitchBackToIDEScreen;
begin
  if IdeScreen = nil then
    exit;
  move(IdeScreen^,videobuf^,videobufsize);
  freemem(IdeScreen,IdeSize);
  IdeScreen := nil;
end;

{$endif}

{****************************************************************************
                                 TWindowsScreen
****************************************************************************}

{$ifdef Windows}

{ Seems to be missing in windows unit PM }
const
  ENABLE_INSERT_MODE     = $20;
  ENABLE_QUICK_EDIT_MODE = $40;
  ENABLE_EXTENDED_FLAGS  = $80;
  ENABLE_AUTO_POSITION   = $100;



procedure UpdateFileHandles;
begin
  {StdInputHandle:=longint(GetStdHandle(STD_INPUT_HANDLE));}
  StdOutputHandle:=longint(GetStdHandle(cardinal(STD_OUTPUT_HANDLE)));
  {StdErrorHandle:=longint(GetStdHandle(STD_ERROR_HANDLE));}
  TextRec(Output).Handle:=StdOutputHandle;
  TextRec(StdOut).Handle:=StdOutputHandle;
  {TextRec(StdErr).Handle:=StdErrorHandle;}
end;

constructor TWindowsScreen.Init;
var
  SecurityAttr : Security_attributes;
  BigWin : Coord;
  res : longbool;
  Error : dword;
  ConsoleScreenBufferInfo : Console_screen_buffer_info;
begin
  inherited Init;
  {if GetConsoleOutputCP<>437 then
    res:=SetConsoleOutputCP(437);}
  SecurityAttr.nLength:=SizeOf(Security_attributes);
  SecurityAttr.lpSecurityDescriptor:=nil;
  SecurityAttr.bInheritHandle:=true;
  NewScreenBufferHandle:=CreateConsoleScreenBuffer(
    GENERIC_READ or GENERIC_WRITE,
    FILE_SHARE_READ or FILE_SHARE_WRITE,SecurityAttr,
    CONSOLE_TEXTMODE_BUFFER,nil);
  DummyScreenBufferHandle:=CreateConsoleScreenBuffer(
    GENERIC_READ or GENERIC_WRITE,
    FILE_SHARE_READ or FILE_SHARE_WRITE,SecurityAttr,
    CONSOLE_TEXTMODE_BUFFER,nil);
  StartScreenBufferHandle:=GetStdHandle(cardinal(STD_OUTPUT_HANDLE));
  GetConsoleMode(GetStdHandle(cardinal(Std_Input_Handle)), @ConsoleMode);
  IdeMode:=ConsoleMode;
{$ifdef debug}
  Complain('Starting ConsoleMode is $'+hexstr(ConsoleMode,8));
{define Windowsbigwin}
{$endif debug}
{$ifdef Windowsbigwin}
  GetConsoleScreenBufferInfo(StartScreenBufferHandle,
    @ConsoleScreenBufferInfo);
  BigWin.X:=ConsoleScreenBufferInfo.dwSize.X;
  BigWin.Y:=ConsoleScreenBufferInfo.srwindow.bottom-ConsoleScreenBufferInfo.srwindow.top; // mants 15779 was 200
  { Try to allow to store more info }
  res:=SetConsoleScreenBufferSize(NewScreenBufferHandle,BigWin);
  if not res then
    error:=GetLastError;
  res:=SetConsoleScreenBufferSize(StartScreenBufferHandle,BigWin);
  if not res then
    error:=GetLastError;
{$endif Windowsbigwin}
  GetConsoleScreenBufferInfo(StartScreenBufferHandle,
    @ConsoleScreenBufferInfo);
  { make sure that the IDE Screen Handle has the maximum display size
    this removes the scroll bars if it is maximized }

  BigWin.X:=ConsoleScreenBufferInfo.dwSize.X;
  BigWin.Y:=ConsoleScreenBufferInfo.srwindow.bottom-ConsoleScreenBufferInfo.srwindow.top;
  res:=SetConsoleScreenBufferSize(NewScreenBufferHandle,
     BigWin);
// mants 15779 : was
//  res:=SetConsoleScreenBufferSize(NewScreenBufferHandle,
//         ConsoleScreenBufferInfo.dwMaximumWindowSize);
  if not res then
    error:=GetLastError;
  IDEScreenBufferHandle:=NewScreenBufferHandle;
  DosScreenBufferHandle:=StartScreenBufferHandle;
  Capture;
  IdeScreenMode.row:=0;
  SwitchBackToIDEScreen;
end;

destructor TWindowsScreen.Done;
begin
  { copy the Dos buffer content into the original ScreenBuffer
    which remains the startup std_output_handle PM }
  {if StartScreenBufferHandle=IDEScreenBufferHandle then}
    BufferCopy(DosScreenBufferHandle,IDEScreenBufferHandle);
  SetConsoleActiveScreenBuffer(StartScreenBufferHandle);
  SetStdHandle(cardinal(Std_Output_Handle),StartScreenBufferHandle);
  UpdateFileHandles;
  CloseHandle(NewScreenBufferHandle);
  CloseHandle(DummyScreenBufferHandle);
  inherited Done;
end;

function TWindowsScreen.GetWidth: integer;
var
  ConsoleScreenBufferInfo : Console_screen_buffer_info;
begin
  GetConsoleScreenBufferInfo(DosScreenBufferHandle,
    @ConsoleScreenBufferInfo);
  GetWidth:=ConsoleScreenBufferInfo.dwSize.X;
end;

function TWindowsScreen.GetHeight: integer;
var
  ConsoleScreenBufferInfo : Console_screen_buffer_info;
begin
  GetConsoleScreenBufferInfo(DosScreenBufferHandle,
    @ConsoleScreenBufferInfo);
  GetHeight:=ConsoleScreenBufferInfo.dwSize.Y;
end;

function TWindowsScreen.CanScroll : boolean;
var
  ConsoleScreenBufferInfo : Console_screen_buffer_info;
  BufferLines : longint;
  WindowLines : longint;
begin
  GetConsoleScreenBufferInfo(DosScreenBufferHandle,
    @ConsoleScreenBufferInfo);
  WindowLines:=ConsoleScreenBufferInfo.srWindow.Bottom-
    ConsoleScreenBufferInfo.srWindow.Top;
  BufferLines:= ConsoleScreenBufferInfo.dwSize.Y-1;
  CanScroll:=(BufferLines>WindowLines);
end;

function TWindowsScreen.Scroll(i : integer) : integer;
var
  ConsoleScreenBufferInfo : Console_screen_buffer_info;
  ConsoleWindow : Small_rect;
begin
  GetConsoleScreenBufferInfo(DosScreenBufferHandle,
    @ConsoleScreenBufferInfo);
  if (ConsoleScreenBufferInfo.srWindow.Top + i < 0) then
    i:= -ConsoleScreenBufferInfo.srWindow.Top;
  if (ConsoleScreenBufferInfo.srWindow.Bottom + i > ConsoleScreenBufferInfo.dwSize.Y) then
    i:= ConsoleScreenBufferInfo.dwSize.Y - ConsoleScreenBufferInfo.srWindow.Bottom;
  if i<>0 then
    begin
      ConsoleWindow.Left:=ConsoleScreenBufferInfo.srWindow.Left;
      ConsoleWindow.Right:=ConsoleScreenBufferInfo.srWindow.Right;
      ConsoleWindow.Top:=ConsoleScreenBufferInfo.srWindow.Top+i;
      ConsoleWindow.Bottom:=ConsoleScreenBufferInfo.srWindow.Bottom+i;
      SetConsoleWindowInfo(DosScreenBufferHandle,true,ConsoleWindow);
      Scroll:=i;
    end
  else
    Scroll:=0;
end;

procedure TWindowsScreen.GetLine(Line: integer; var Text, Attr: string);
type
  CharInfoArray = Array [0..255] of Char_Info;
var
  LineBuf : ^CharInfoArray;
  BufSize,BufCoord : Coord;
  i,LineSize : longint;
  WriteRegion : SMALL_RECT;
begin
  GetMem(LineBuf,SizeOf(CharInfoArray));
  LineSize:=ScreenWidth;
  If LineSize>256 then
    LineSize:=256;
  BufSize.X:=LineSize;
  BufSize.Y:=1;
  BufCoord.X:=0;
  BufCoord.Y:=0;
  with WriteRegion do
    begin
      Top :=Line;
      Left :=0;
      Bottom := Line+1;
      Right := LineSize-1;
    end;
  ReadConsoleOutput(DosScreenBufferHandle, PChar_info(LineBuf),
    BufSize, BufCoord, @WriteRegion);
  for i:=1 to LineSize do
    begin
      Text[i]:=LineBuf^[i-1].AsciiChar;
      Attr[i]:=char(byte(LineBuf^[i-1].Attributes));
    end;
  FreeMem(LineBuf,SizeOf(CharInfoArray));
  Text[0]:=char(byte(LineSize));
  Attr[0]:=char(byte(LineSize));
end;


procedure TWindowsScreen.GetCursorPos(var P: TPoint);
var
  ConsoleScreenBufferInfo : Console_screen_buffer_info;
begin
  GetConsoleScreenBufferInfo(DosScreenBufferHandle,
    @ConsoleScreenBufferInfo);
  P.X:=ConsoleScreenBufferInfo.dwCursorPosition.X;
  P.Y:=ConsoleScreenBufferInfo.dwCursorPosition.Y;
end;

procedure TWindowsScreen.BufferCopy(Src, Dest : THandle);
type
  CharInfoArray = Array [0..256*255-1] of Char_Info;
var
  LineBuf : ^CharInfoArray;
  BufSize,BufCoord : Coord;
  Error, LineSize,
  Part, OnePartY: longint;
  res : boolean;
  WriteRegion : SMALL_RECT;
  ConsoleScreenBufferInfo : Console_screen_buffer_info;
  DestConsoleScreenBufferInfo : Console_screen_buffer_info;
begin
  GetConsoleScreenBufferInfo(Src,
    @ConsoleScreenBufferInfo);
  GetConsoleScreenBufferInfo(Dest,
    @DestConsoleScreenBufferInfo);
  GetMem(LineBuf,SizeOf(CharInfoArray));
  FillChar(LineBuf^,SizeOf(CharInfoArray),#0);

  LineSize:=ConsoleScreenBufferInfo.dwSize.X;
  If LineSize>256 then
    LineSize:=256;
  BufSize.X:=LineSize;
  BufSize.Y:=ConsoleScreenBufferInfo.dwSize.Y;
  BufCoord.X:=0;
  BufCoord.Y:=0;
  with WriteRegion do
    begin
      Top :=0;
      Left :=0;
      Bottom := ConsoleScreenBufferInfo.dwSize.Y-1;
      Right := LineSize-1;
    end;
  if BufSize.X*BufSize.Y*Sizeof(CHAR_INFO) >= $8000 then
    begin
      OnePartY := ($8000  -1) div (BufSize.X * SizeOf(Char_Info) );
      BufSize.Y:=OnePartY;
      Part:=0;
      while ((Part+1)*OnePartY < ConsoleScreenBufferInfo.dwSize.Y) do
        begin
          WriteRegion.Top := Part*OnePartY;
          WriteRegion.Bottom := (Part+1)*OnePartY-1;
          res:=ReadConsoleOutput(Src, PChar_info(LineBuf),
            BufSize, BufCoord, @WriteRegion);
          if not res then
            Error:=GetLastError;
          res:=WriteConsoleOutput(Dest, PChar_info(LineBuf),
            BufSize, BufCoord, @WriteRegion);
          if not res then
            Error:=GetLastError;
          Inc(Part);
        end;
      BufSize.Y:=ConsoleScreenBufferInfo.dwSize.Y - Part*OnePartY;
      WriteRegion.Top := Part*OnePartY;
      WriteRegion.Bottom := ConsoleScreenBufferInfo.dwSize.Y-1;
      res:=ReadConsoleOutput(Src, PChar_info(LineBuf),
        BufSize, BufCoord, @WriteRegion);
      if not res then
        Error:=GetLastError;
      res:=WriteConsoleOutput(Dest, PChar_info(LineBuf),
        BufSize, BufCoord, @WriteRegion);
      if not res then
        Error:=GetLastError;
    end
  else
    begin
      res:=ReadConsoleOutput(Src, PChar_info(LineBuf),
        BufSize, BufCoord, @WriteRegion);
      if not res then
        Error:=GetLastError;
      res:=WriteConsoleOutput(Dest, PChar_info(LineBuf),
        BufSize, BufCoord, @WriteRegion);
      if not res then
        Error:=GetLastError;
    end;
  FreeMem(LineBuf,SizeOf(CharInfoArray));
  SetConsoleCursorPosition(Dest, ConsoleScreenBufferInfo.dwCursorPosition);
end;

procedure TWindowsScreen.Capture;
begin
  {if StartScreenBufferHandle=IdeScreenBufferHandle then
    BufferCopy(IDEScreenBufferHandle,DosScreenBufferHandle)
  else
    BufferCopy(DosScreenBufferHandle,IDEScreenBufferHandle);}
  SaveConsoleScreen;
end;

procedure TWindowsScreen.Restore;
begin
  SwitchToConsoleScreen;
end;

{ dummy for Windows as the Buffer screen
  do hold all the info }
procedure TWindowsScreen.SaveIDEScreen;
var
  NowIdeMode : Dword;
begin
  IdeScreenMode:=ScreenMode;
  GetConsoleMode(GetStdHandle(cardinal(Std_Input_Handle)), @NowIdeMode);
{$ifdef debug}
  Complain('IDE ConsoleMode is $'+hexstr(NowIdeMode,8));
  if NowIdeMode<>IdeMode then
    Complain('is not equal to IDEMode  $'+hexstr(IdeMode,8));
{$endif debug}
  IdeMode:=NowIdeMode;
  { set the dummy buffer as active already now PM }
  SetStdHandle(cardinal(Std_Output_Handle),DummyScreenBufferHandle);
  UpdateFileHandles;
end;

{ dummy for Windows as the Buffer screen
  do hold all the info }
procedure TWindowsScreen.SaveConsoleScreen;
begin
  GetConsoleMode(GetStdHandle(cardinal(Std_Input_Handle)), @ConsoleMode);
{$ifdef debug}
  Complain('ConsoleMode now is $'+hexstr(ConsoleMode,8));
{$endif debug}
  { set the dummy buffer as active already now PM }
  SetStdHandle(cardinal(Std_Output_Handle),DummyScreenBufferHandle);
  UpdateFileHandles;
end;

procedure TWindowsScreen.SwitchToConsoleScreen;
begin
  SetConsoleActiveScreenBuffer(DosScreenBufferHandle);
  SetStdHandle(cardinal(Std_Output_Handle),DosScreenBufferHandle);
  SetConsoleMode(GetStdHandle(cardinal(Std_Input_Handle)), ConsoleMode);
  UpdateFileHandles;
  IDEActive:=false;
end;

procedure TWindowsScreen.SwitchBackToIDEScreen;
var
  ConsoleScreenBufferInfo : Console_screen_buffer_info;
  WindowPos : Small_rect;
  res : boolean;
  error : longint;
begin
  SetStdHandle(cardinal(Std_Output_Handle),IDEScreenBufferHandle);
  UpdateFileHandles;
  GetConsoleScreenBufferInfo(IDEScreenBufferHandle,
    @ConsoleScreenBufferInfo);
  SetConsoleActiveScreenBuffer(IDEScreenBufferHandle);
  { Needed to force InitSystemMsg to use the right console handle }
  DoneEvents;
  InitEvents;
  IdeMode:=({IdeMode or }ENABLE_MOUSE_INPUT or
                   ENABLE_WINDOW_INPUT or
                   ENABLE_EXTENDED_FLAGS)
           and not (ENABLE_PROCESSED_INPUT or
                    ENABLE_LINE_INPUT or
                    ENABLE_ECHO_INPUT or
                    ENABLE_INSERT_MODE or
                    ENABLE_QUICK_EDIT_MODE);
  SetConsoleMode(GetStdHandle(cardinal(Std_Input_Handle)), IdeMode);
  WindowPos.left:=0;
  WindowPos.right:=ConsoleScreenBufferInfo.srWindow.right
                   -ConsoleScreenBufferInfo.srWindow.left;
  WindowPos.top:=0;
  WindowPos.bottom:=ConsoleScreenBufferInfo.srWindow.bottom
                   -ConsoleScreenBufferInfo.srWindow.top;
  with ConsoleScreenBufferInfo.dwMaximumWindowSize do
    begin
    if WindowPos.Right<X-1 then
      WindowPos.right:=X-1;
    if WindowPos.Bottom<Y-1 then
      WindowPos.Bottom:=Y-1;
    end;
  res:=SetConsoleWindowInfo(IDEScreenBufferHandle,true,WindowPos);
  if not res then
    error:=GetLastError;
{$ifdef DEBUG}
  IdeScreenMode.row:=WindowPos.bottom+1;
  IdeScreenMode.col:=WindowPos.right+1;
{$endif DEBUG}
  { needed to force the correct size for videobuf }
  if Assigned(Application) and (IdeScreenMode.row<>0)then
    Application^.SetScreenVideoMode(IdeScreenMode);
  IDEActive:=true;
end;

{$ifdef debug}

procedure TWindowsScreen.Complain(St : string);
begin
  if IDEActive then
    DebugMessage('',St,0,0)
  else
    Writeln(stderr,St);
end;

procedure TWindowsScreen.SetConsoleMode(FH : Handle;Mode: DWord);
var
  Test: DWord;
begin
  If not Windows.SetConsoleMode(FH,Mode) then
    begin
      Complain('SetConsoleMode call failed GetLastError='+IntToStr(GetLastError));
    end
  else
    begin
      if not GetConsoleMode(FH,Test) then
        begin
          Complain('GetConsoleMode call failed GetLastError='+IntToStr(GetLastError));
        end
      else if (Test<>Mode) then
        begin
          Complain('GetConsoleMode result '+IntToStr(Test)+' <> '+
            IntToStr(Mode));
        end;
    end;
end;
{$endif DEBUG}

{$endif}

{****************************************************************************
                                 TOS2Screen
****************************************************************************}


{$ifdef OS2}
function TOS2Screen.GetWidth: integer;
begin
  GetWidth:=80;
end;

function TOS2Screen.GetHeight: integer;
begin
  GetHeight:=25;
end;

procedure TOS2Screen.GetLine(Line: integer; var Text, Attr: string);
begin
  Text:='                                                                               ';
  Attr:='                                                                               ';
end;

procedure TOS2Screen.GetCursorPos(var P: TPoint);
begin
  P.X:=1;
  P.Y:=1;
end;

{ remember the initial video screen }
procedure TOS2Screen.Capture;
begin
end;

{ restore the initial video mode }
procedure TOS2Screen.Restore;
begin
end;

{ move up or down if supported by OS }
function TOS2Screen.Scroll(i : integer) : integer;
begin
end;

{ saves the current IDE screen }
procedure TOS2Screen.SaveIDEScreen;
begin
end;

{ saves the current console screen }
procedure TOS2Screen.SaveConsoleScreen;
begin
end;

{ restores the saved console screen }
procedure TOS2Screen.SwitchToConsoleScreen;
begin
end;

{ restores the saved IDE screen }
procedure TOS2Screen.SwitchBackToIDEScreen;
begin
end;

constructor TOS2Screen.Init;
begin
end;

destructor TOS2Screen.Done;
begin
end;

{$ENDIF}


{****************************************************************************
                                 TNWLScreen
****************************************************************************}


{$ifdef netwlibc}
function TNWLScreen.GetWidth: integer;
begin
  GetWidth:=80;
end;

function TNWLScreen.GetHeight: integer;
begin
  GetHeight:=25;
end;

procedure TNWLScreen.GetLine(Line: integer; var Text, Attr: string);
begin
  Text:='                                                                               ';
  Attr:='                                                                               ';
end;

procedure TNWLScreen.GetCursorPos(var P: TPoint);
begin
  P.X:=1;
  P.Y:=1;
end;

{ remember the initial video screen }
procedure TNWLScreen.Capture;
begin
end;

{ restore the initial video mode }
procedure TNWLScreen.Restore;
begin
end;

{ saves the current IDE screen }
procedure TNWLScreen.SaveIDEScreen;
begin
end;

{ saves the current console screen }
procedure TNWLScreen.SaveConsoleScreen;
begin
end;

{ restores the saved console screen }
procedure TNWLScreen.SwitchToConsoleScreen;
begin
end;

{ restores the saved IDE screen }
procedure TNWLScreen.SwitchBackToIDEScreen;
begin
end;

{$ENDIF}


{****************************************************************************
                                 TAmigaScreen
****************************************************************************}


{$IFDEF AMIGASCREEN}
function TAmigaScreen.GetWidth: integer;
begin
  GetWidth:=80;
end;

function TAmigaScreen.GetHeight: integer;
begin
  GetHeight:=25;
end;

procedure TAmigaScreen.GetLine(Line: integer; var Text, Attr: string);
begin
  Text:='                                                                               ';
  Attr:='                                                                               ';
end;

procedure TAmigaScreen.GetCursorPos(var P: TPoint);
begin
  P.X:=1;
  P.Y:=1;
end;

{ remember the initial video screen }
procedure TAmigaScreen.Capture;
begin
end;

{ restore the initial video mode }
procedure TAmigaScreen.Restore;
begin
end;

{ saves the current IDE screen }
procedure TAmigaScreen.SaveIDEScreen;
begin
end;

{ saves the current console screen }
procedure TAmigaScreen.SaveConsoleScreen;
begin
end;

{ restores the saved console screen }
procedure TAmigaScreen.SwitchToConsoleScreen;
begin
end;

{ restores the saved IDE screen }
procedure TAmigaScreen.SwitchBackToIDEScreen;
begin
end;

{$ENDIF}


{****************************************************************************
                                 Initialize
****************************************************************************}

procedure InitUserScreen;
begin
{$ifdef DOS}
  UserScreen:=New(PDOSScreen, Init);
{$else}
  {$ifdef Unix}
    UserScreen:=New(PLinuxScreen, Init);
  {$else}

    {$ifdef Windows}
      UserScreen:=New(PWindowsScreen, Init);
    {$else}
      {$ifdef OS2}
        UserScreen:=New(POS2Screen, Init);
      {$else}
        {$ifdef netwlibc}
          UserScreen:=New(PNWLScreen, Init);
        {$else}
          {$ifdef AMIGASCREEN}
            UserScreen:=nil; //New(PAmigaScreen, Init);
          {$else}
            UserScreen:=New(PScreen, Init);
          {$endif AMIGASCREEN}
        {$endif netwlibc}
      {$endif OS2}
    {$endif Windows}
  {$endif Unix}
{$endif Dos}
end;


procedure DoneUserScreen;
begin
  if UserScreen<>nil then
   begin
     UserScreen^.Restore;
     Dispose(UserScreen, Done);
     UserScreen:=nil;
   end;
end;

end.
