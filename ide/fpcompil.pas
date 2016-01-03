{
    This file is part of the Free Pascal Integrated Development Environment
    Copyright (c) 1998 by Berczi Gabor

    Compiler call routines for the IDE

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{$i globdir.inc}
unit FPCompil;

{2.0 compatibility}
{$ifdef VER2_0}
  {$macro on}
  {$define resourcestring := const}
{$endif}

interface

{ don't redir under linux, because all stdout (also from the ide!) will
  then be redired (PFV) }
{ this should work now correctly because
  RedirDisableAll and RedirEnableAll function are added in fpredir (PM) }

{ $define VERBOSETXT}

{$mode objfpc}

uses
  { We need to include the exceptions from SysUtils, but the types from
    Objects need to be used. Keep the order SysUtils,Objects }
  SysUtils,
  Objects,
  FInput,
  Drivers,Views,Dialogs,
  WUtils,WViews,WCEdit,
  FPSymbol,
  FPViews;

type
  TCompileMode = (cBuild,cMake,cCompile,cRun);

type
    PCompilerMessage = ^TCompilerMessage;
    TCompilerMessage = object(TMessageItem)
      function GetText(MaxLen: Sw_Integer): String; virtual;
    end;

    PCompilerMessageListBox = ^TCompilerMessageListBox;
    TCompilerMessageListBox = object(TMessageListBox)
      function  GetPalette: PPalette; virtual;
      procedure SelectFirstError;
    end;

    PCompilerMessageWindow = ^TCompilerMessageWindow;
    TCompilerMessageWindow = object(TFPWindow)
      constructor Init;
      procedure   HandleEvent(var Event: TEvent); virtual;
      function    GetPalette: PPalette; virtual;
      procedure   Close;virtual;
      destructor  Done; virtual;
      procedure   SizeLimits(var Min, Max: TPoint); virtual;
      procedure   AddMessage(AClass: longint;const Msg, Module: string; Line, Column: longint);
      procedure   ClearMessages;
      constructor Load(var S: TStream);
      procedure   Store(var S: TStream);
      procedure   SetState(AState: Word; Enable: Boolean); virtual;
      procedure   UpdateCommands; virtual;
    private
      {CompileShowed : boolean;}
      {Mode   : TCompileMode;}
      MsgLB  : PCompilerMessageListBox;
      {CurrST,
      InfoST : PColorStaticText;}
    end;

    PCompilerStatusDialog = ^TCompilerStatusDialog;
    TCompilerStatusDialog = object(TCenterDialog)
      ST    : PAdvancedStaticText;
      KeyST : PColorStaticText;
      starttime : real;
      constructor Init;
      destructor Done;virtual;
      procedure   Update;
      procedure SetStartTime(r : real);
    end;

    TFPInputFile = class(tinputfile)
      constructor Create(AEditor: PFileEditor);
    protected
      function fileopen(const filename: ansistring): boolean; override;
      function fileseek(pos: longint): boolean; override;
      function fileread(var databuf; maxsize: longint): longint; override;
      function fileeof: boolean; override;
      function fileclose: boolean; override;
      procedure filegettime; override;
    private
      Editor: PFileEditor;
      S: PStream;
    end;

const
    CompilerMessageWindow : PCompilerMessageWindow  = nil;
    CompilerStatusDialog  : PCompilerStatusDialog = nil;
    CompileStamp          : longint = 0;
    RestartingDebugger    : boolean = false;

procedure DoCompile(Mode: TCompileMode);
function  NeedRecompile(Mode :TCompileMode; verbose : boolean): boolean;
procedure ParseUserScreen;

procedure RegisterFPCompile;

const
  CompilingHiddenFile : PSourceWindow = nil;

implementation

uses
{$ifdef Unix}
  Unix, BaseUnix,
{$endif}
{$ifdef go32v2}
  dpmiexcp,
{$endif}
{$ifdef Windows}
  {$ifdef HasSignal}
    signals,
  {$endif}
{$endif}
{ $ifdef HasSignal}
  fpcatch,
{ $endif HasSignal}
  Dos,
  Video,
  globals,
  StdDlg,App,tokens,
  FVConsts,
  CompHook, Compiler, systems, browcol,
  WEditor,
  FPRedir,FPDesk,
  FPUsrScr,FPHelp,
{$ifndef NODEBUG}FPDebug,{$endif}
  FPConst,FPVars,FPUtils,
  FPCodCmp,FPIntf,FPSwitch;

{$ifndef NOOBJREG}
const
  RCompilerMessageListBox: TStreamRec = (
     ObjType: 1211;
     VmtLink: Ofs(TypeOf(TCompilerMessageListBox)^);
     Load:    @TCompilerMessageListBox.Load;
     Store:   @TCompilerMessageListBox.Store
  );
  RCompilerMessageWindow: TStreamRec = (
     ObjType: 1212;
     VmtLink: Ofs(TypeOf(TCompilerMessageWindow)^);
     Load:    @TCompilerMessageWindow.Load;
     Store:   @TCompilerMessageWindow.Store
  );
{$endif}
{$ifdef useresstrings}
resourcestring
{$else}
const
{$endif}
                dialog_compilermessages = 'Compiler Messages';
                dialog_compilingwithmode = 'Compiling  (%s mode)';

                { Compiler message classes }
                msg_class_normal   = '';
                msg_class_fatal    = 'Fatal';
                msg_class_error    = 'Error';
                msg_class_warning  = 'Warning';
                msg_class_note     = 'Note';
                msg_class_hint     = 'Hint';
                msg_class_macro    = 'Macro';
                msg_class_procedure= 'Procedure';
                msg_class_conditional = 'Conditional';
                msg_class_info     = 'Info';
                msg_class_status   = 'Status';
                msg_class_used     = 'Used';
                msg_class_tried    = 'Tried';
                msg_class_debug    = 'Debug';

                { Compile status dialog texts }
                msg_compilingfile      = 'Compiling %s';
                msg_loadingunit        = 'Loading %s unit';
                msg_linkingfile        = 'Linking %s';
                msg_compiledone        = 'Done.';
                msg_failedtocompile    = 'Failed to compile...';
                msg_compilationaborted = 'Compilation aborted...';

                msg_nothingtocompile = 'Oooops, nothing to compile.';
                msg_cantcompileunsavedfile = 'Can''t compile unsaved file.';

                msg_couldnotcreatefile = 'could not create %s';
                msg_therearemoreerrorsinfile = 'There are more errors in file %s';
                msg_firstcompilationof = 'First compilation of %s';
                msg_recompilingbecauseof = 'Recompiling because of %s';

                msg_errorinexternalcompilation = 'Error in external compilation';
                msg_iostatusis = 'IOStatus = %d';
                msg_executeresultis = 'ExecuteResult = %d';

                { Status hints during compilation }
                msg_hint_pressesctocancel = 'Press ESC to cancel';
                msg_hint_compilesuccessfulpressenter = 'Compile successful: ~Press any key~';
                msg_hint_compilefailed = 'Compile failed';
                msg_hint_compileaborted = 'Compile aborted';
                msg_hint_pleasewait = 'Please wait...';

                msg_cantopenfile = 'Can''t open %s';

procedure ParseUserScreen;
var
  Y,YMax : longint;
  LEvent : TEvent;
  Text,Attr : String;
  DisplayCompilerWindow : boolean;
  cc: integer;

    procedure SearchBackTrace;
      var AText,ModuleName,st : String;
          row : longint;
      begin
        if pos('  $',Text)=1 then
          begin
            AText:=Text;
            Delete(Text,1,11);
            While pos(' ',Text)=1 do
              Delete(Text,1,1);
            if pos('of ',Text)>0 then
              begin
                ModuleName:=Copy(Text,pos('of ',Text)+3,255);
                While ModuleName[Length(ModuleName)]=' ' do
                  Delete(ModuleName,Length(ModuleName),1);
              end
            else
              ModuleName:='';
            if pos('line ',Text)>0 then
              begin
                Text:=Copy(Text,Pos('line ',Text)+5,255);
                st:=Copy(Text,1,Pos(' ',Text)-1);
                Val(st,row,cc);
              end
            else
              row:=0;
            CompilerMessageWindow^.AddMessage(V_Fatal or v_lineinfo,AText
                  ,ModuleName,row,1);
            DisplayCompilerWindow:=true;
          end;
      end;

    procedure InsertInMessages(Const TypeStr : String;_Type : longint;EnableDisplay : boolean);
      var p,p2,col,row : longint;
          St,ModuleName : string;

      begin
        p:=pos(TypeStr,Text);
        p2:=Pos('(',Text);
        if (p>0)  and (p2>0) and (p2<p) then
          begin
            ModuleName:=Copy(Text,1,p2-1);
            st:=Copy(Text,p2+1,255);
            Val(Copy(st,1,pos(',',st)-1),row,cc);
            st:=Copy(st,Pos(',',st)+1,255);
            Val(Copy(st,1,pos(')',st)-1),col,cc);
            CompilerMessageWindow^.AddMessage(_type,Copy(Text,pos(':',Text)+1,255)
              ,ModuleName,row,col);
            If EnableDisplay then
              DisplayCompilerWindow:=true;
          end;
      end;

begin
  if not assigned(UserScreen) then
    exit;
  DisplayCompilerWindow:=false;
  YMax:=UserScreen^.GetHeight;
  PushStatus('Parsing User Screen');
  CompilerMessageWindow^.Lock;
  for Y:=0 to YMax do
    begin
      UserScreen^.GetLine(Y,Text,Attr);
      if (y mod 10) = 0 then
        begin
          CompilerMessageWindow^.Unlock;
          SetStatus('Parsing User Screen line '+IntToStr(y)+'/'+IntToStr(YMax));
          CompilerMessageWindow^.Lock;
        end;
      GetKeyEvent(LEvent);
      if (LEvent.What=evKeyDown) and (LEvent.KeyCode=kbEsc) then
        break;
      SearchBackTrace;
      InsertInMessages(' Fatal:',v_Fatal or v_lineinfo,true);
      InsertInMessages(' Error:',v_Error or v_lineinfo,true);
      InsertInMessages(' Warning:',v_Warning or v_lineinfo,false);
      InsertInMessages(' Note:',v_Note or v_lineinfo,false);
      InsertInMessages(' Info:',v_Info or v_lineinfo,false);
      InsertInMessages(' Hint:',v_Hint or v_lineinfo,false);
    end;
  if DisplayCompilerWindow then
    begin
      if not CompilerMessageWindow^.GetState(sfVisible) then
        CompilerMessageWindow^.Show;
      CompilerMessageWindow^.MakeFirst;
      CompilerMessageWindow^.MsgLB^.SelectFirstError;
    end;
  CompilerMessageWindow^.UnLock;
  PopStatus;
end;

{*****************************************************************************
                               TCompilerMessage
*****************************************************************************}

function TCompilerMessage.GetText(MaxLen: Sw_Integer): String;
var
  ClassS: string[20];
  S: string;
begin
  case TClass and V_LevelMask of
    V_Fatal   : ClassS:=msg_class_Fatal;
    V_Error   : ClassS:=msg_class_Error;
    V_Normal  : ClassS:=msg_class_Normal;
    V_Warning : ClassS:=msg_class_Warning;
    V_Note    : ClassS:=msg_class_Note;
    V_Hint    : ClassS:=msg_class_Hint;
{$ifdef VERBOSETXT}
    V_Conditional : ClassS:=msg_class_conditional;
    V_Info    : ClassS:=msg_class_info;
    V_Status  : ClassS:=msg_class_status;
    V_Used    : ClassS:=msg_class_used;
    V_Tried   : ClassS:=msg_class_tried;
    V_Debug   : ClassS:=msg_class_debug;
    else
      ClassS:='???';
{$endif}
    else
      ClassS:='';
  end;
  if ClassS<>'' then
   ClassS:=RExpand(ClassS,0)+': ';
  if assigned(Module) and
     ((TClass and V_LineInfo)=V_LineInfo) then
    begin
      if Row>0 then
       begin
         if Col>0 then
          S:=NameAndExtOf(Module^)+'('+IntToStr(Row)+','+IntToStr(Col)+') '+ClassS
         else
          S:=NameAndExtOf(Module^)+'('+IntToStr(Row)+') '+ClassS;
       end
      else
       S:=NameAndExtOf(Module^)+'('+IntToStr(Row)+') '+ClassS
    end
  else
    S:=ClassS;
  if assigned(Text) then
    S:=S+Text^;
  if length(S)>MaxLen then
    S:=copy(S,1,MaxLen-2)+'..';
  GetText:=S;
end;


{*****************************************************************************
                             TCompilerMessageListBox
*****************************************************************************}

function TCompilerMessageListBox.GetPalette: PPalette;
const
  P: string[length(CBrowserListBox)] = CBrowserListBox;
begin
  GetPalette:=PPalette(@P);
end;

procedure TCompilerMessageListBox.SelectFirstError;
  function IsError(P : PCompilerMessage) : boolean;
    begin
      IsError:=(P^.TClass and (V_Fatal or V_Error))<>0;
    end;
  var
    P : PCompilerMessage;
begin
  P:=List^.FirstThat(@IsError);
  If Assigned(P) then
    Begin
      FocusItem(List^.IndexOf(P));
      DrawView;
    End;
end;


{*****************************************************************************
                                TCompilerMessageWindow
*****************************************************************************}

constructor TCompilerMessageWindow.Init;
var R: TRect;
    HSB,VSB: PScrollBar;
begin
  Desktop^.GetExtent(R);
  R.A.Y:=R.B.Y-7;
  inherited Init(R,dialog_compilermessages,{SearchFreeWindowNo}wnNoNumber);
  HelpCtx:=hcCompilerMessagesWindow;

  AutoNumber:=true;

  HSB:=StandardScrollBar(sbHorizontal+sbHandleKeyboard);
  HSB^.GrowMode:=gfGrowLoY+gfGrowHiX+gfGrowHiY;
  Insert(HSB);
  VSB:=StandardScrollBar(sbVertical+sbHandleKeyboard);
  VSB^.GrowMode:=gfGrowLoX+gfGrowHiX+gfGrowHiY;
  Insert(VSB);

  GetExtent(R);
  R.Grow(-1,-1);
  New(MsgLB, Init(R, HSB, VSB));

  MsgLB^.GrowMode:=gfGrowHiX+gfGrowHiY;
  Insert(MsgLB);
  CompilerMessageWindow:=@self;
end;


procedure TCompilerMessageWindow.AddMessage(AClass: longint;const Msg, Module: string; Line, Column: longint);
begin
  if (AClass and V_LineInfo)<>V_LineInfo then
    Line:=0;
  MsgLB^.AddItem(New(PCompilerMessage,Init(AClass, Msg, MsgLB^.AddModuleName(Module), Line, Column)));
  if (@Self=CompilerMessageWindow) and ((AClass = V_fatal) or (AClass = V_Error)) then
    begin
      if not GetState(sfVisible) then
        Show;
      if Desktop^.First<>PView(CompilerMessageWindow) then
        MakeFirst;
    end;
end;


procedure TCompilerMessageWindow.ClearMessages;
begin
  MsgLB^.Clear;
  ReDraw;
end;


{procedure TCompilerMessageWindow.Updateinfo;
begin
  if CompileShowed then
   begin
     InfoST^.SetText(
       RExpand(' Main file : '#1#$7f+Copy(SmartPath(MainFile),1,39),40)+#2+
         'Total lines  : '#1#$7e+IntToStr(Status.CompiledLines)+#2#13+
       RExpand(' Target    : '#1#$7f+KillTilde(TargetSwitches^.ItemName(TargetSwitches^.GetCurrSel)),40)+#2+
         'Total errors : '#1#$7e+IntToStr(Status.ErrorCount)
     );
     if status.currentline>0 then
      CurrST^.SetText(' Status: '#1#$7e+status.currentsource+'('+IntToStr(status.currentline)+')'#2)
     else
      CurrST^.SetText(' Status: '#1#$7e+status.currentsource+#2);
   end;
  ReDraw;
end;}


procedure TCompilerMessageWindow.HandleEvent(var Event: TEvent);
begin
  case Event.What of
    evBroadcast :
      case Event.Command of
        cmListFocusChanged :
          if Event.InfoPtr=MsgLB then
            Message(Application,evBroadcast,cmClearLineHighlights,@Self);
      end;
  end;
  inherited HandleEvent(Event);
end;


procedure TCompilerMessageWindow.SizeLimits(var Min, Max: TPoint);
begin
  inherited SizeLimits(Min,Max);
  Min.X:=20;
  Min.Y:=4;
end;


procedure TCompilerMessageWindow.Close;
begin
  Hide;
end;


function TCompilerMessageWindow.GetPalette: PPalette;
const
  S : string[length(CBrowserWindow)] = CBrowserWindow;
begin
  GetPalette:=PPalette(@S);
end;


constructor TCompilerMessageWindow.Load(var S: TStream);
begin
  inherited Load(S);
  GetSubViewPtr(S,MsgLB);
end;


procedure TCompilerMessageWindow.Store(var S: TStream);
begin
  if MsgLB^.List=nil then
    MsgLB^.NewList(New(PCollection, Init(100,100)));
  inherited Store(S);
  PutSubViewPtr(S,MsgLB);
end;

procedure TCompilerMessageWindow.UpdateCommands;
var Active: boolean;
begin
  Active:=GetState(sfActive);
  SetCmdState(CompileCmds,Active);
  Message(Application,evBroadcast,cmCommandSetChanged,nil);
end;

procedure TCompilerMessageWindow.SetState(AState: Word; Enable: Boolean);
var OldState: word;
begin
  OldState:=State;
  inherited SetState(AState,Enable);
  if ((AState and sfActive)<>0) and (((OldState xor State) and sfActive)<>0) then
    UpdateCommands;
end;

destructor TCompilerMessageWindow.Done;
begin
  CompilerMessageWindow:=nil;
  inherited Done;
end;


{****************************************************************************
                          CompilerStatusDialog
****************************************************************************}

function getrealtime : real;
var
{$IFDEF USE_SYSUTILS}
  h,m,s,s1000 : word;
{$ELSE USE_SYSUTILS}
  h,m,s,s100 : word;
{$ENDIF USE_SYSUTILS}
begin
{$IFDEF USE_SYSUTILS}
  DecodeTime(Time,h,m,s,s1000);
  getrealtime:=h*3600.0+m*60.0+s+s1000/1000.0;
{$ELSE USE_SYSUTILS}
  gettime(h,m,s,s100);
  getrealtime:=h*3600.0+m*60.0+s+s100/100.0;
{$ENDIF USE_SYSUTILS}
end;

constructor TCompilerStatusDialog.Init;
var R: TRect;
begin
  R.Assign(0,0,56,11);
  ClearFormatParams; AddFormatParamStr(KillTilde(SwitchesModeName[SwitchesMode]));
  inherited Init(R, FormatStrF(dialog_compilingwithmode, FormatParams));
  starttime:=getrealtime;
  GetExtent(R); R.B.Y:=11;
  R.Grow(-3,-2);
  New(ST, Init(R, ''));
  Insert(ST);
  GetExtent(R); R.B.Y:=11;
  R.Grow(-1,-1); R.A.Y:=R.B.Y-1;
  New(KeyST, Init(R, '', Blue*16+White+longint($80+Blue*16+White)*256,true));
  Insert(KeyST);
  { Reset Status infos see bug 1585 }
  Fillchar(Status,SizeOf(Status),#0);
end;

destructor TCompilerStatusDialog.Done;
begin
  if @Self=CompilerStatusDialog then
    CompilerStatusDialog:=nil;
  Inherited Done;
end;

procedure TCompilerStatusDialog.SetStartTime(r : real);
  begin
    starttime:=r;
  end;

procedure TCompilerStatusDialog.Update;
var
  StatusS,KeyS: string;
  hstatus : TFPCHeapStatus;
  r : real;
const
  MaxFileNameSize = 46;
begin
  case CompilationPhase of
    cpCompiling :
      begin
        ClearFormatParams;
        if Upcase(Status.currentmodulestate)='COMPILE' then
          begin
            AddFormatParamStr(ShrinkPath(SmartPath(Status.Currentsourcepath+Status.CurrentSource),
              MaxFileNameSize - Length(msg_compilingfile)));
            StatusS:=FormatStrF(msg_compilingfile,FormatParams);
          end
        else
          begin
            if Status.CurrentSource='' then
              StatusS:=''
            else
              begin
                StatusS:=ShrinkPath(SmartPath(DirAndNameOf(Status.Currentsourcepath+Status.CurrentSource)),
                  MaxFileNameSize-Length(msg_loadingunit));
                AddFormatParamStr(StatusS);
                StatusS:=FormatStrF(msg_loadingunit,FormatParams);
              end;
          end;
        KeyS:=msg_hint_pressesctocancel;
      end;
    cpLinking   :
      begin
        ClearFormatParams;
        AddFormatParamStr(ShrinkPath(ExeFile,
          MaxFileNameSize-Length(msg_linkingfile)));
        StatusS:=FormatStrF(msg_linkingfile,FormatParams);
        KeyS:=msg_hint_pleasewait;
      end;
    cpDone      :
      begin
        StatusS:=msg_compiledone;
        KeyS:=msg_hint_compilesuccessfulpressenter;
      end;
    cpFailed    :
      begin
        StatusS:=msg_failedtocompile;
        KeyS:=msg_hint_compilefailed;
      end;
    cpAborted    :
      begin
        StatusS:=msg_compilationaborted;
        KeyS:=msg_hint_compileaborted;
      end;
  end;
  ClearFormatParams;
  AddFormatParamStr(ShrinkPath(SmartPath(MainFile),
    MaxFileNameSize-Length('Main file: %s')));
  AddFormatParamStr(StatusS);
  AddFormatParamStr(KillTilde(TargetSwitches^.ItemName(TargetSwitches^.GetCurrSel)));
  AddFormatParamInt(Status.CurrentLine);
  AddFormatParamInt(Status.CompiledLines);
  hstatus:=GetFPCHeapStatus;
  AddFormatParamInt(hstatus.CurrHeapUsed div 1024);
  AddFormatParamInt(hstatus.CurrHeapSize div 1024);
  AddFormatParamInt(Status.ErrorCount);
  r:=getrealtime;
  AddFormatParamInt(trunc(r-starttime));
  AddFormatParamInt(trunc(frac(r-starttime)*10));
  ST^.SetText(
   FormatStrF(
    'Main file: %s'#13+
    '%s'+#13#13+
    'Target: %s'#13+
    'Line number: %6d     '+'Total lines:      %6d'+#13+
    'Used memory: %6dK    '+'Allocated memory: %6dK'#13+
    'Total errors:%6d     '+'Compile time: %8d.%1ds',
   FormatParams)
  );
  KeyST^.SetText(^C+KeyS);
end;


{****************************************************************************
                               Compiler Hooks
****************************************************************************}

const
  lasttime  : real = 0;

function CompilerStatus: boolean;
  var
     event : tevent;

begin
  GetKeyEvent(Event);
  if (Event.What=evKeyDown) and (Event.KeyCode=kbEsc) then
    begin
       CompilationPhase:=cpAborted;
       { update info messages }
       if assigned(CompilerStatusDialog) then
        begin
{$ifdef redircompiler}
          RedirDisableAll;
{$endif}
          CompilerStatusDialog^.Update;
{$ifdef redircompiler}
          RedirEnableAll;
{$endif}
        end;
       CompilerStatus:=true;
       exit;
    end;
{ only display line info every 100 lines, ofcourse all other messages
  will be displayed directly }
  if (getrealtime-lasttime>=CompilerStatusUpdateDelay) or (status.compiledlines=1) then
   begin
     lasttime:=getrealtime;
     { update info messages }
{$ifdef redircompiler}
          RedirDisableAll;
{$endif}
     if assigned(CompilerStatusDialog) then
      CompilerStatusDialog^.Update;
{$ifdef redircompiler}
          RedirEnableAll;
{$endif}
     { update memory usage }
     { HeapView^.Update; }
   end;
  CompilerStatus:=false;
end;

<<<<<<< HEAD
<<<<<<< HEAD
Function  CompilerGetNamedFileTime(const filename : ansistring) : Longint;
=======
Function  CompilerGetNamedFileTime(const filename : string) : Longint;
>>>>>>> graemeg/fixes_2_2
=======
Function  CompilerGetNamedFileTime(const filename : string) : Longint;
>>>>>>> origin/fixes_2_2
var t: longint;
    W: PSourceWindow;
begin
  W:=EditorWindowFile(FExpand(filename));
  if Assigned(W) and (W^.Editor^.GetModified) then
    t:=Now
  else
    t:=def_getnamedfiletime(filename);
  CompilerGetNamedFileTime:=t;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function CompilerOpenInputFile(const filename: ansistring): tinputfile;
=======
function CompilerOpenInputFile(const filename: string): tinputfile;
>>>>>>> graemeg/fixes_2_2
=======
function CompilerOpenInputFile(const filename: string): tinputfile;
>>>>>>> origin/fixes_2_2
var f: tinputfile;
    W: PSourceWindow;
begin
  if assigned(CompilingHiddenFile) and
     (NameandExtof(filename)=CompilingHiddenFile^.Editor^.Filename) then
    W:=CompilingHiddenFile
  else
    W:=EditorWindowFile(FExpand(filename));
  if Assigned(W) and (W^.Editor^.GetModified) then
    f:=TFPInputFile.Create(W^.Editor)
  else
    f:=def_openinputfile(filename);
  if assigned(W) then
    W^.Editor^.CompileStamp:=CompileStamp;
  CompilerOpenInputFile:=f;
end;

function CompilerComment(Level:Longint; const s:ansistring):boolean;
begin
  CompilerComment:=false;
  if (status.verbosity and Level)<>0 then
   begin
{$ifdef redircompiler}
     RedirDisableAll;
{$endif}

     if not CompilerMessageWindow^.GetState(sfVisible) then
       CompilerMessageWindow^.Show;
     if Desktop^.First<>PView(CompilerMessageWindow) then
       CompilerMessageWindow^.MakeFirst;
     CompilerMessageWindow^.AddMessage(Level,S,status.currentsourcepath+status.currentsource,
       status.currentline,status.currentcolumn);
     { update info messages }
     if assigned(CompilerStatusDialog) then
      CompilerStatusDialog^.Update;
{$ifdef redircompiler}
      RedirEnableAll;
{$endif}
     { update memory usage }
     { HeapView^.Update; }
   end;
end;


{****************************************************************************
                                 DoCompile
****************************************************************************}

{ This function must return '' if
  "Options|Directories|Exe and PPU directory" is empty }
function GetExePath: string;
var Path: string;
    I: Sw_integer;
begin
  Path:='';
  if DirectorySwitches<>nil then
    with DirectorySwitches^ do
    for I:=0 to ItemCount-1 do
      begin
        if ItemParam(I)='-FE' then
          begin
            Path:=GetStringItem(I);
            Break;
          end;
      end;
  if Path<>'' then
    GetExePath:=CompleteDir(FExpand(Path))
  else
    GetExePath:='';
end;

function GetMainFile(Mode: TCompileMode): string;
var FileName: string;
    P : PSourceWindow;
begin
  if assigned(CompilingHiddenFile) then
    P:=CompilingHiddenFile
  else
    P:=Message(Desktop,evBroadcast,cmSearchWindow,nil);
  if (PrimaryFileMain='') and (P=nil) then
    FileName:='' { nothing to compile }
  else
    begin
      if (PrimaryFileMain<>'') and (Mode<>cCompile) then
        FileName:=PrimaryFileMain
      else if assigned(P) then
        begin
          FileName:=P^.Editor^.FileName;
          if FileName='' then
            begin
              P^.Editor^.SaveAsk(true);
              FileName:=P^.Editor^.FileName;
            end;
        end
      else
        FileName:='';
    end;
  {$ifdef Unix}
  If (FileName<>'') then
    FileName:=FExpand(FileName);
  {$else}
  If (FileName<>'') then
    FileName:=FixFileName(FExpand(FileName));
  {$endif}
  GetMainFile:=FileName;
end;

procedure ResetErrorMessages;
  procedure ResetErrorLine(P: PView);
  begin
    if assigned(P) and
       (TypeOf(P^)=TypeOf(TSourceWindow)) then
       PSourceWindow(P)^.Editor^.SetErrorMessage('');
  end;
begin
  Desktop^.ForEach(@ResetErrorLine);
end;


procedure DoCompile(Mode: TCompileMode);

  function IsExitEvent(E: TEvent): boolean;
  begin
    { following suggestion by Harsha Senanayake }
    IsExitEvent:=(E.What=evKeyDown);
  end;
  function GetTargetExeExt : string;
    begin
        GetTargetExeExt:=target_info.exeext;
     end;
var
  s,FileName: string;
  ErrFile : Text;
  MustRestartDebugger : boolean;
  Error,LinkErrorCount : longint;
  E : TEvent;
  DummyView: PView;
  PPasFile : string[64];
begin
  AskRecompileIfModifiedFlag:=true;
{ Get FileName }
  FileName:=GetMainFile(Mode);
  if FileName='' then
    begin
      ErrorBox(msg_nothingtocompile,nil);
      Exit;
    end else
  { THis is not longer necessary as unsaved files are loaded from a memorystream,
    and with the file as primaryfile set it is already incompatible with itself
   if FileName='*' then
    begin
      ErrorBox(msg_cantcompileunsavedfile,nil);
      Exit;
    end; }
  PushStatus('Beginning compilation...');
{ Show Compiler Messages Window }
{  if not CompilerMessageWindow^.GetState(sfVisible) then
   CompilerMessageWindow^.Show;
  CompilerMessageWindow^.MakeFirst;}
  CompilerMessageWindow^.ClearMessages;
  { Tell why we compile }
  NeedRecompile(Mode,true);

  MainFile:=FileName;
  SetStatus('Writing switches to file...');
  WriteSwitches(SwitchesPath);
  { leaving open browsers leads to crashes !! (PM) }
  SetStatus('Preparing symbol info...');
  CloseAllBrowsers;
  if ((DesktopFileFlags and dfSymbolInformation)<>0) then
    WriteSymbolsFile(BrowserName);
{  MainFile:=FixFileName(FExpand(FileName));}
  SetStatus('Preparing to compile...'+NameOf(MainFile));
{ Reset }
  CtrlBreakHit:=false;
{ Create Compiler Status Dialog }
  CompilationPhase:=cpCompiling;
  if not assigned(CompilingHiddenFile) then
    begin
      New(CompilerStatusDialog, Init);
      CompilerStatusDialog^.SetStartTime(getrealtime);
      CompilerStatusDialog^.SetState(sfModal,true);
      { disable window closing }
      CompilerStatusDialog^.Flags:=CompilerStatusDialog^.Flags and not wfclose;
      Application^.Insert(CompilerStatusDialog);
      CompilerStatusDialog^.Update;
    end;
  { Restore dir that could be changed during debugging }
  {$I-}
   ChDir(StartUpDir);
  {$I+}
  EatIO;
{ hook compiler output }
  do_status:=@CompilerStatus;
  do_comment:=@CompilerComment;
  do_openinputfile:=@CompilerOpenInputFile;
  do_getnamedfiletime:=@CompilerGetNamedFileTime;
  do_initsymbolinfo:=@InitBrowserCol;
  do_donesymbolinfo:=@DoneBrowserCol;
  do_extractsymbolinfo:=@CreateBrowserCol;
{ Compile ! }
{$ifdef redircompiler}
  ChangeRedirOut(FPOutFileName,false);
  ChangeRedirError(FPErrFileName,false);
{$endif}
  { insert "" around name so that spaces are allowed }
  { only supported in compiler after 2000/01/14 PM   }
  if pos(' ',FileName)>0 then
    FileName:='"'+FileName+'"';
  if mode=cBuild then
    FileName:='-B '+FileName;
  { tokens are created and distroed by compiler.compile !! PM }
  DoneTokens;
  PPasFile:='ppas'+source_info.scriptext;
  WUtils.DeleteFile(GetExePath+PpasFile);
  SetStatus('Compiling...');
  inc(CompileStamp);
  ResetErrorMessages;
  {$ifndef NODEBUG}
  MustRestartDebugger:=false;
  if assigned(Debugger) then
  if Debugger^.HasExe then
    begin
      Debugger^.Reset;
      MustRestartDebugger:=true;
    end;
  {$endif NODEBUG}
  try
    FpIntF.Compile(FileName,SwitchesPath);
  except
    on ECompilerAbort do
      CompilerMessageWindow^.AddMessage(V_error,'Error during compilation','',0,0);
    on E:Exception do
      CompilerMessageWindow^.AddMessage(V_error,E.Message+' during compilation','',0,0);
  end;
  SetStatus('Finished compiling...');

  { Retrieve created exefile }
  If GetEXEPath<>'' then
    EXEFile:=FixFileName(GetEXEPath+NameOf(MainFile)+GetTargetExeExt)
  else
    EXEFile:=DirOf(MainFile)+NameOf(MainFile)+GetTargetExeExt;
  DefaultReplacements(ExeFile);
  { tokens are created and distroyed by compiler.compile !! PM }
  InitTokens;
  if LinkAfter and
     ExistsFile(GetExePath+PpasFile) and
     (CompilationPhase<>cpAborted) and
     (status.errorCount=0) then
    begin
       CompilationPhase:=cpLinking;
       if assigned(CompilerStatusDialog) then
         CompilerStatusDialog^.Update;
       SetStatus('Assembling and/or linking...');
{$ifndef redircompiler}
       { At least here we want to catch output
        of batch file PM }
       ChangeRedirOut(FPOutFileName,false);
       ChangeRedirError(FPErrFileName,false);
{$endif}
{$ifdef Unix}
       error:=0;
       If fpsystem(GetExePath+PpasFile)=-1 Then
        Error:=fpgeterrno;
{$else}
       DosExecute(GetEnv('COMSPEC'),'/C '+GetExePath+PpasFile);
       Error:=DosError;
{$endif}
       SetStatus('Finished linking...');
       RestoreRedirOut;
       RestoreRedirError;
       if Error<>0 then
         Inc(status.errorCount);
       if Status.IsExe and not Status.IsLibrary and not ExistsFile(EXEFile) then
         begin
           Inc(status.errorCount);
           ClearFormatParams; AddFormatParamStr(ExeFile);
           CompilerMessageWindow^.AddMessage(V_error,FormatStrF(msg_couldnotcreatefile,FormatParams),'',0,0);
         {$I-}
           Assign(ErrFile,FPErrFileName);
           Reset(ErrFile);
           if EatIO<>0 then
             ErrorBox(FormatStrStr(msg_cantopenfile,FPErrFileName),nil)
           else
           begin
             LinkErrorCount:=0;
             While not eof(ErrFile) and (LinkErrorCount<25) do
               begin
                 readln(ErrFile,s);
                 CompilerMessageWindow^.AddMessage(V_error,s,'',0,0);
                 inc(LinkErrorCount);
               end;
             if not eof(ErrFile) then
             begin
               ClearFormatParams; AddFormatParamStr(FPErrFileName);
               CompilerMessageWindow^.AddMessage(V_error,
                 FormatStrF(msg_therearemoreerrorsinfile,FormatParams),'',0,0);
             end;

             Close(ErrFile);
           end;
           EatIO;
         {$I+}
         end
       else if error=0 then
         WUtils.DeleteFile(GetExePath+PpasFile);
    end;
{$ifdef redircompiler}
  RestoreRedirOut;
  RestoreRedirError;
{$endif}
  PopStatus;
{ Set end status }
  if not (CompilationPhase in [cpAborted,cpFailed]) then
    if (status.errorCount=0) then
      begin
        CompilationPhase:=cpDone;
        LastCompileTime := cardinal(Now);
      end
    else
      CompilationPhase:=cpFailed;
{ Show end status }
  { reenable window closing }
  if assigned(CompilerStatusDialog) then
    begin
      CompilerStatusDialog^.Flags:=CompilerStatusDialog^.Flags or wfclose;
      CompilerStatusDialog^.Update;
      CompilerStatusDialog^.ReDraw;
      CompilerStatusDialog^.SetState(sfModal,false);
      if ((CompilationPhase in [cpAborted,cpDone,cpFailed]) or (ShowStatusOnError))
        and ((Mode<>cRun) or (CompilationPhase<>cpDone)) then
       repeat
         CompilerStatusDialog^.GetEvent(E);
         if IsExitEvent(E)=false then
          CompilerStatusDialog^.HandleEvent(E);
       until IsExitEvent(E) or not assigned(CompilerStatusDialog);
       {if IsExitEvent(E) then
         Application^.PutEvent(E);}
      if assigned(CompilerStatusDialog) then
        begin
          Application^.Delete(CompilerStatusDialog);
          Dispose(CompilerStatusDialog, Done);
        end;
    end;
  CompilerStatusDialog:=nil;
{ end compilation returns true if the messagewindow should be removed }
  if CompilationPhase=cpDone then
   begin
     CompilerMessageWindow^.Hide;
     { This is the last compiled main file }
     PrevMainFile:=MainFile;
     MainHasDebugInfo:=DebugInfoSwitches^.GetCurrSelParam<>'-';
   end;
{ Update the app }
  Message(Application,evCommand,cmUpdate,nil);
  DummyView:=Desktop^.First;
  while (DummyView<>nil) and (DummyView^.GetState(sfVisible)=false) do
  begin
    DummyView:=DummyView^.NextView;
  end;
  with DummyView^ do
   if GetState(sfVisible) then
    begin
      SetState(sfSelected,false);
      SetState(sfSelected,true);
    end;
  if Assigned(CompilerMessageWindow) then
    with CompilerMessageWindow^ do
      begin
        if GetState(sfVisible) then
          begin
            SetState(sfSelected,false);
            SetState(sfSelected,true);
          end;
        if (status.errorCount>0) then
          MsgLB^.SelectFirstError;
      end;
  { ^^^ we need this trick to reactivate the desktop }
  EditorModified:=false;
{$ifndef NODEBUG}
  if MustRestartDebugger then
    InitDebugger;
{$endif NODEBUG}
  { In case we have something that the compiler touched }
  AskToReloadAllModifiedFiles;
  { Try to read Browser info in again if compilation failure !! }
  if Not Assigned(Modules) and (CompilationPhase<>cpDone) and
     ((DesktopFileFlags and dfSymbolInformation)<>0) then
    ReadSymbolsFile(BrowserName);
  if UseAllUnitsInCodeComplete and not assigned(CompilingHiddenFile) then
    AddAvailableUnitsToCodeComplete(false);
end;

function NeedRecompile(Mode :TCompileMode; verbose : boolean): boolean;
var Need: boolean;
    I: sw_integer;
    SF: PSourceFile;
    SourceTime,PPUTime,ObjTime: longint;
    W: PSourceWindow;
begin
  if Assigned(SourceFiles)=false then
     Need:={(EditorModified=true)}true
  else
    begin
      Need:=(PrevMainFile<>GetMainFile(Mode)) and (PrevMainFile<>'');
      if Need then
        begin
          if verbose then
          begin
            ClearFormatParams; AddFormatParamStr(GetMainFile(Mode));
            CompilerMessageWindow^.AddMessage(V_info,
              FormatStrF(msg_firstcompilationof,FormatParams),
              '',0,0);
          end;
        end
      else
        for I:=0 to SourceFiles^.Count-1 do
          begin
            SF:=SourceFiles^.At(I);
            SourceTime:=wutils.GetFileTime(SF^.GetSourceFileName);
            PPUTime:=wutils.GetFileTime(SF^.GetPPUFileName);
            ObjTime:=wutils.GetFileTime(SF^.GetObjFileName);
{            writeln('S: ',SF^.GetSourceFileName,' - ',SourceTime);
            writeln('P: ',SF^.GetPPUFileName,' - ',PPUTime);
            writeln('O: ',SF^.GetObjFileName,' - ',ObjTime);
            writeln('------');}
            { some units don't generate object files }
            W:=EditorWindowFile(SF^.GetSourceFileName);
            if (SourceTime<>-1) then
              if ((SourceTime>PPUTime) or
                 ((SourceTime>ObjTime) and
                 (ObjTime<>-1))) or
                 (assigned(W) and (W^.Editor^.CompileStamp<0)) then
                begin
                  Need:=true;
                  if verbose then
                  begin
                    ClearFormatParams; AddFormatParamStr(SF^.GetSourceFileName);
                    CompilerMessageWindow^.AddMessage(V_info,
                      FormatStrF(msg_recompilingbecauseof,FormatParams),
                      SF^.GetSourceFileName,1,1);
                  end;
                  Break;
                end;
          end;
{      writeln('Need?', Need); system.readln;}
    end;

  NeedRecompile:=Need;
end;


constructor TFPInputFile.Create(AEditor: PFileEditor);
begin
  if not Assigned(AEditor) then Fail;
  if inherited Create(AEditor^.FileName)=nil then
    Fail;
  Editor:=AEditor;
end;


function TFPInputFile.fileopen(const filename: ansistring): boolean;
var OK: boolean;
begin
  S:=New(PMemoryStream, Init(0,0));
  OK:=Assigned(S) and (S^.Status=stOK);
  if OK then OK:=Editor^.SaveToStream(S);
  if OK then
    S^.Seek(0)
  else
    begin
      if Assigned(S) then Dispose(S, Done);
      S:=nil;
    end;
  fileopen:=OK;
end;

function TFPInputFile.fileseek(pos: longint): boolean;
var OK: boolean;
begin
  OK:=assigned(S);
  if OK then
  begin
    S^.Reset;
    S^.Seek(pos);
    OK:=(S^.Status=stOK);
  end;
  fileseek:=OK;
end;

function TFPInputFile.fileread(var databuf; maxsize: longint): longint;
var
    size: longint;
begin
  if not assigned(S) then size:=0 else
  begin
    size:=min(maxsize,(S^.GetSize-S^.GetPos));
    S^.Read(databuf,size);
    if S^.Status<>stOK then size:=0;
  end;
  fileread:=size;
end;

function TFPInputFile.fileeof: boolean;
var EOF: boolean;
begin
  EOF:=not assigned(S);
  if not EOF then
    EOF:=(S^.Status<>stOK) or (S^.GetPos=S^.GetSize);
  fileeof:=EOF;
end;

function TFPInputFile.fileclose: boolean;
var OK: boolean;
begin
  OK:=assigned(S);
  if OK then
  begin
    S^.Reset;
    Dispose(S, Done);
    S:=nil;
    OK:=true;
  end;
  fileclose:=OK;
end;

procedure tfpinputfile.filegettime;
var
  dt : datetime;
  hsec,wday : word;
begin
  { current time }
  dos.getdate(dt.year,dt.month,dt.day,wday);
  dos.gettime(dt.hour,dt.min,dt.sec,hsec);
  packtime(dt,filetime);
end;

procedure RegisterFPCompile;
begin
{$ifndef NOOBJREG}
  RegisterType(RCompilerMessageListBox);
  RegisterType(RCompilerMessageWindow);
{$endif}
end;


end.
