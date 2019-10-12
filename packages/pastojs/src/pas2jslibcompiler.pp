{
    This file is part of the Free Component Library (FCL)
    Copyright (c) 2018  Michael Van Canneyt

    Pascal to Javascript converter class. Library version

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************
}
unit pas2jslibcompiler;

{$mode objfpc}
{$H+}

{$IFDEF darwin}
{$DEFINE UseCDecl}
{$ENDIF}

interface

uses
  SysUtils, Classes,
  FPPJsSrcMap, Pas2jsFileCache, Pas2JSCompiler, Pas2jsPCUCompiler,
  Pas2JSCompilerCfg, Pas2JSCompilerPP;

{ ---------------------------------------------------------------------
  Compiler descendant, usable in library
  ---------------------------------------------------------------------}
Const
  DefaultReadBufferSize = 32*1024; // 32kb buffer

Type
  PDirectoryCache = Pointer;

  TLibLogCallBack = Procedure (Data : Pointer; Msg : PAnsiChar; MsgLen : Integer); {$IFDEF UseCDecl}cdecl{$ELSE}stdcall{$ENDIF};
  TWriteJSCallBack = Procedure (Data : Pointer;
    AFileName: PAnsiChar; AFileNameLen : Integer;
    AFileData : PAnsiChar; AFileDataLen: Int32); {$IFDEF UseCDecl}cdecl{$ELSE}stdcall{$ENDIF};
  TReadPasCallBack = Procedure (Data : Pointer;
    AFileName: PAnsiChar; AFileNameLen : Integer;
    AFileData : PAnsiChar; Var AFileDataLen: Int32); {$IFDEF UseCDecl}cdecl{$ELSE}stdcall{$ENDIF};
  TReadDirCallBack = Function (Data : Pointer;
    P : PDirectoryCache; ADirPath: PAnsiChar): boolean; {$IFDEF UseCDecl}cdecl{$ELSE}stdcall{$ENDIF};

  { TLibraryPas2JSCompiler }

  TLibraryPas2JSCompiler = Class(TPas2JSPCUCompiler)
  private
    FLastError: String;
    FLastErrorClass: String;
    FOnLibLogCallBack: TLibLogCallBack;
    FOnLibLogData: Pointer;
    FOnReadDir: TReadDirCallBack;
    FOnReadDirData: Pointer;
    FOnReadPasData: Pointer;
    FOnReadPasFile: TReadPasCallBack;
    FOnWriteJSCallBack: TWriteJSCallBack;
    FOnWriteJSData: Pointer;
    FReadBufferLen: Cardinal;
  Protected
    Function DoWriteJSFile(const DestFilename: String; aWriter: TPas2JSMapper): Boolean; override;
    Procedure GetLastError(AError : PAnsiChar; Var AErrorLength : Longint;
      AErrorClass : PAnsiChar; Var AErrorClassLength : Longint);
    Function ReadFile(aFilename: string; var aSource: string): boolean; virtual;
    Function ReadDirectory(Dir: TPas2jsCachedDirectory): boolean; virtual;
  Public
    Constructor Create; override;
    Procedure DoLibraryLog(Sender : TObject; Const Msg : String);
    Function LibraryRun(ACompilerExe, AWorkingDir : PAnsiChar; CommandLine : PPAnsiChar; DoReset : Boolean) :Boolean; {$IFDEF UseCDecl}cdecl{$ELSE}stdcall{$ENDIF};
    Property LastError : String Read FLastError Write FLastError;
    Property LastErrorClass : String Read FLastErrorClass Write FLastErrorClass;
    Property OnLibLogCallBack : TLibLogCallBack Read FOnLibLogCallBack Write FOnLibLogCallBack;
    Property OnLibLogData : Pointer Read FOnLibLogData Write FOnLibLogData;
    Property OnWriteJSCallBack : TWriteJSCallBack Read FOnWriteJSCallBack Write FOnWriteJSCallBack;
    Property OnWriteJSData : Pointer Read FOnWriteJSData Write FOnWriteJSData;
    Property OnReadPasFile : TReadPasCallBack Read FOnReadPasFile Write FOnReadPasFile;
    Property OnReadPasData : Pointer Read FOnReadPasData Write FOnReadPasData;
    Property ReadBufferLen : Cardinal Read FReadBufferLen Write FReadBufferLen;
    Property OnReadDir: TReadDirCallBack read FOnReadDir write FOnReadDir;
    Property OnReadDirData: Pointer read FOnReadDirData write FOnReadDirData;
  end;

Type
  PPas2JSCompiler = Pointer;

Procedure SetPas2JSWriteJSCallBack(P : PPas2JSCompiler; ACallBack : TWriteJSCallBack; CallBackData : Pointer); {$IFDEF UseCDecl}cdecl{$ELSE}stdcall{$ENDIF};
Procedure SetPas2JSCompilerLogCallBack(P : PPas2JSCompiler; ACallBack : TLibLogCallBack; CallBackData : Pointer); {$IFDEF UseCDecl}cdecl{$ELSE}stdcall{$ENDIF};
Procedure SetPas2JSReadPasCallBack(P : PPas2JSCompiler; ACallBack : TReadPasCallBack; CallBackData : Pointer; ABufferSize : Cardinal); {$IFDEF UseCDecl}cdecl{$ELSE}stdcall{$ENDIF};
Procedure SetPas2JSReadDirCallBack(P : PPas2JSCompiler; ACallBack : TReadDirCallBack; CallBackData : Pointer); {$IFDEF UseCDecl}cdecl{$ELSE}stdcall{$ENDIF};
Procedure AddPas2JSDirectoryEntry(P: PDirectoryCache; AFilename: PAnsiChar;
  AAge: TPas2jsFileAgeTime; AAttr: TPas2jsFileAttr; ASize: TPas2jsFileSize); {$IFDEF UseCDecl}cdecl{$ELSE}stdcall{$ENDIF};
Function RunPas2JSCompiler(P : PPas2JSCompiler; ACompilerExe, AWorkingDir : PAnsiChar; CommandLine : PPAnsiChar; DoReset : Boolean) : Boolean; {$IFDEF UseCDecl}cdecl{$ELSE}stdcall{$ENDIF};
Procedure FreePas2JSCompiler(P : PPas2JSCompiler); {$IFDEF UseCDecl}cdecl{$ELSE}stdcall{$ENDIF};
Function GetPas2JSCompiler : PPas2JSCompiler; {$IFDEF UseCDecl}cdecl{$ELSE}stdcall{$ENDIF};
Procedure GetPas2JSCompilerLastError(P : PPas2JSCompiler; AError : PAnsiChar; Var AErrorLength : Longint; AErrorClass : PAnsiChar; Var AErrorClassLength : Longint); {$IFDEF UseCDecl}cdecl{$ELSE}stdcall{$ENDIF};

implementation

{ TLibraryPas2JSCompiler }

function TLibraryPas2JSCompiler.ReadDirectory(Dir: TPas2jsCachedDirectory
  ): boolean;
begin
  Result:=false; // return false to call the default TPas2jsCachedDirectory.DoReadDir
  if Assigned(OnReadDir) then
    Result:=OnReadDir(FOnReadDirData,Dir,PAnsiChar(Dir.Path));
end;

function TLibraryPas2JSCompiler.DoWriteJSFile(const DestFilename: String; aWriter: TPas2JSMapper): Boolean;

Var
  Src : string;

begin
  Result:=Assigned(OnWriteJSCallBack);
  if Result then
    try
      Src:=aWriter.{$IF FPC_FULLVERSION>30300}AsString{$ELSE}AsAnsistring{$ENDIF};
      OnWriteJSCallBack(OnWriteJSData,PAnsiChar(DestFileName),Length(DestFileName),PAnsiChar(Src),Length(Src));
    except
      Result:=False;
    end;
end;

procedure TLibraryPas2JSCompiler.GetLastError(AError: PAnsiChar;
  Var AErrorLength: Longint; AErrorClass: PAnsiChar;
  Var AErrorClassLength: Longint);

Var
  L : Integer;

begin
  L:=Length(LastError);
  if (L>AErrorLength) then
    L:=AErrorLength;
  if (L>0) then
    Move(FLastError[1],AError^,L);
  AErrorLength:=L;
  L:=Length(LastErrorClass);
  if L>AErrorClassLength then
    L:=AErrorClassLength;
  if (L>0) then
    Move(FLastErrorClass[1],AErrorClass^,L);
  AErrorClassLength:=L;
end;

function TLibraryPas2JSCompiler.ReadFile(aFilename: string; var aSource: string): boolean;

Var
  Buf : Array of AnsiChar;
  S : TStringStream;
  BytesRead : Cardinal;

begin
  if Not Assigned(OnReadPasFile) then
    Exit(False);
  S:=nil;
  try
    if ReadBufferLen=0 then
      ReadBufferLen:=DefaultReadBufferSize;
    SetLength(Buf,ReadBufferLen);
    S:=TStringStream.Create(''{$IF FPC_FULLVERSION>=30101},CP_ACP{$ENDIF});
    Repeat
      BytesRead:=ReadBufferLen;
      FOnReadPasFile(OnReadPasData,PAnsiChar(aFileName),Length(aFileName),@Buf[0],BytesRead);
      If BytesRead>0 then
        S.Write(Buf[0],BytesRead);
    Until (BytesRead<ReadBufferLen);
    Result:=S.Size<>0;
    if Result then
      aSource:=S.DataString;
  finally
    SetLength(Buf,0);
    S.Free;
  end;
end;

constructor TLibraryPas2JSCompiler.Create;
begin
  inherited Create;
  Log.OnLog:=@DoLibraryLog;
  FileCache.OnReadFile:=@ReadFile;
  FReadBufferLen:=DefaultReadBufferSize;
  FileCache.OnReadDirectory:=@ReadDirectory;
  ConfigSupport:=TPas2JSFileConfigSupport.Create(Self);
  PostProcessorSupport:=TPas2JSFSPostProcessorSupport.Create(Self);
end;

procedure TLibraryPas2JSCompiler.DoLibraryLog(Sender: TObject; const Msg: String);
begin
  if Assigned(FOnLibLogCallBack) then
    FOnLibLogCallBack(FOnLibLogData,PAnsiChar(Msg),Length(Msg))
  else if isConsole then
    {AllowWriteln}Writeln(Msg);{AllowWriteln-}
end;

function TLibraryPas2JSCompiler.LibraryRun(ACompilerExe, AWorkingDir: PAnsiChar;
  CommandLine: PPAnsiChar; DoReset: Boolean): Boolean; {$IFDEF UseCDecl}cdecl{$ELSE}stdcall{$ENDIF};

Var
  C,W : AnsiString;
  CmdLine : TStrings;
  PP : PPAnsiChar;

begin
  Result:=False;
  C:=ACompilerExe;
  W:=AWorkingDir;
  CmdLine:=TStringList.Create;
  try
    PP:=CommandLine;
    While (PP^<>Nil) do
      begin
      CmdLine.Add(pp^);
      Inc(PP);
      end;
    try
      Run(C,W,CmdLine,DoReset);
      Result:=(ExitCode=0);
      if Not Result then
        begin
        LastError:=Format('Compiler exited with exit code %d',[ExitCode]);
        LastErrorClass:=ECompilerTerminate.ClassName;
        end;
    except
      On E : Exception do
        begin
        LastError:=E.Message;
        LastErrorClass:=E.ClassName;
        end;
    end;
  finally
    CmdLine.free;
  end;
end;

{ ---------------------------------------------------------------------
  Flat interface
  ---------------------------------------------------------------------}

procedure SetPas2JSWriteJSCallBack(P: PPas2JSCompiler;
  ACallBack: TWriteJSCallBack; CallBackData: Pointer); {$IFDEF UseCDecl}cdecl{$ELSE}stdcall{$ENDIF};

begin
  TLibraryPas2JSCompiler(P).OnWriteJSCallBack:=ACallBack;
  TLibraryPas2JSCompiler(P).OnWriteJSData:=CallBackData;
end;

procedure SetPas2JSCompilerLogCallBack(P: PPas2JSCompiler;
  ACallBack: TLibLogCallBack; CallBackData: Pointer); {$IFDEF UseCDecl}cdecl{$ELSE}stdcall{$ENDIF};

begin
  TLibraryPas2JSCompiler(P).OnLibLogCallBack:=ACallBack;
  TLibraryPas2JSCompiler(P).OnLibLogData:=CallBackData;
end;

procedure SetPas2JSReadPasCallBack(P: PPas2JSCompiler;
  ACallBack: TReadPasCallBack; CallBackData: Pointer; ABufferSize: Cardinal);
  {$IFDEF UseCDecl}cdecl{$ELSE}stdcall{$ENDIF};

begin
  TLibraryPas2JSCompiler(P).OnReadPasData:=CallBackData;
  TLibraryPas2JSCompiler(P).OnReadPasFile:=ACallback;
  if (ABufferSize=0) then
    ABufferSize:=DefaultReadBufferSize;
  TLibraryPas2JSCompiler(P).ReadBufferLen:=ABufferSize;
end;

procedure SetPas2JSReadDirCallBack(P: PPas2JSCompiler;
  ACallBack: TReadDirCallBack; CallBackData: Pointer); {$IFDEF UseCDecl}cdecl{$ELSE}stdcall{$ENDIF};
begin
  TLibraryPas2JSCompiler(P).OnReadDir:=ACallBack;
  TLibraryPas2JSCompiler(P).OnReadDirData:=CallBackData;
end;

procedure AddPas2JSDirectoryEntry(P: PDirectoryCache; AFilename: PAnsiChar;
  AAge: TPas2jsFileAgeTime; AAttr: TPas2jsFileAttr; ASize: TPas2jsFileSize);
  {$IFDEF UseCDecl}cdecl{$ELSE}stdcall{$ENDIF};
begin
  TPas2jsCachedDirectory(P).Add(AFilename,AAge,AAttr,ASize);
end;

function RunPas2JSCompiler(P: PPas2JSCompiler; ACompilerExe,
  AWorkingDir: PAnsiChar; CommandLine: PPAnsiChar; DoReset: Boolean): Boolean;
  {$IFDEF UseCDecl}cdecl{$ELSE}stdcall{$ENDIF};

begin
  Result:=TLibraryPas2JSCompiler(P).LibraryRun(ACompilerExe,AWorkingDir,CommandLine,DoReset)
end;

procedure FreePas2JSCompiler(P: PPas2JSCompiler); {$IFDEF UseCDecl}cdecl{$ELSE}stdcall{$ENDIF};

begin
  TLibraryPas2JSCompiler(P).Free;
end;

function GetPas2JSCompiler: PPas2JSCompiler; {$IFDEF UseCDecl}cdecl{$ELSE}stdcall{$ENDIF};

begin
  Result:=TLibraryPas2JSCompiler.Create;
end;

procedure GetPas2JSCompilerLastError(P: PPas2JSCompiler; AError: PAnsiChar;
  Var AErrorLength: Longint; AErrorClass: PAnsiChar;
  Var AErrorClassLength: Longint); {$IFDEF UseCDecl}cdecl{$ELSE}stdcall{$ENDIF};

begin
  TLibraryPas2JSCompiler(P).GetLastError(AError,AErrorLength,AErrorClass,AErrorClassLength);
end;

end.

