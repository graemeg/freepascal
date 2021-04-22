{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2006 by Florian Klaempfl
    member of the Free Pascal development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{$mode objfpc}
{$H+}
{$inline on}
unit ComServ;

interface

uses
  Classes, SysUtils, comobj, ActiveX;

{ $define DEBUG_COM}

//according to doc
// * ComServer Variable
// * DllCanUnloadNow Routine
// * DllGetClassObject Routine
// * DllRegisterServer Routine
// * DllUnregisterServer Routine
// * TComServer Class

//TODO Fix this
const
  CLASS_E_CLASSNOTAVAILABLE = -1;
  SELFREG_E_CLASS = -2;

type
  TStartMode = (smStandalone, smAutomation,smRegserver,smUnregserver);
  TLastReleaseEvent = procedure(var shutdown: Boolean) of object;

  { TComServer }

  TComServer = class(TComServerObject)
  class var orgInitProc: codepointer;
  private
    fCountObject: Integer;
    fCountFactory: Integer;
    fTypeLib: ITypeLib;
    fServerName,
    fServerFileName: String;
    fHelpFileName : String;
    fRegister: Boolean;
    fStartSuspended : Boolean;
    FIsInproc: Boolean;
    FIsInteractive: Boolean;
    FStartMode: TStartMode;
    FOnLastRelease: TLastReleaseEvent;

    class function AutomationDone: Boolean;
    class procedure AutomationStart;
    procedure CheckCmdLine;
    procedure FactoryFree(Factory: TComObjectFactory);
    procedure FactoryRegisterClassObject(Factory: TComObjectFactory);
    procedure FactoryUpdateRegistry(Factory: TComObjectFactory);
    procedure CheckReleased;
    function GetTypeLibName: widestring;
    procedure RegisterObjectWith(Factory: TComObjectFactory);
    procedure Start;
  protected
    function CountObject(Created: Boolean): Integer; override;
    function CountFactory(Created: Boolean): Integer; override;
    function GetHelpFileName: string; override;
    function GetServerFileName: string; override;
    function GetServerKey: string; override;
    function GetServerName: string; override;
    function GetStartSuspended: Boolean; override;
    function GetTypeLib: ITypeLib; override;
    procedure SetHelpFileName(const Value: string); override;


    procedure RegisterServerFactory(Factory: TComObjectFactory);
    procedure UnregisterServerFactory(Factory: TComObjectFactory);
  public
    constructor Create;
    destructor Destroy; override;
    function CanUnloadNow: Boolean;
    procedure RegisterServer;
    procedure UnRegisterServer;
    property IsInprocServer: Boolean read FIsInproc write FIsInproc;
    property IsInteractive: Boolean read fIsInteractive;
    property StartMode: TStartMode read FStartMode;
    property ServerObjects:integer read fCountObject;
  end;

var
  ComServer: TComServer = nil;
  haut :TLibHandle;


//http://msdn.microsoft.com/en-us/library/ms690368%28VS.85%29.aspx
//If the function succeeds, the return value is S_OK. Otherwise, it is S_FALSE.
function DllCanUnloadNow: HResult; stdcall;


//S_OK - The object was retrieved successfully.
//CLASS_E_CLASSNOTAVAILABLE - The DLL does not support the class (object definition).
function DllGetClassObject(constref rclsid: REFIID {should be REFCLSID}; constref riid: REFIID; out ppv: Pointer): HResult; stdcall;

//S_OK - The registry entries were created successfully.
//SELFREG_E_TYPELIB - The server was unable to complete the registration of all the type libraries used by its classes.
//SELFREG_E_CLASS - The server was unable to complete the registration of all the object classes.
function DllRegisterServer: HResult; stdcall;

//S_OK - The registry entries were deleted successfully.
//S_FALSE - Unregistration of this server's known entries was successful, but other entries still exist for this server's classes.
//SELFREG_E_TYPELIB - The server was unable to remove the entries of all the type libraries used by its classes.
//SELFREG_E_CLASS - The server was unable to remove the entries of all the object classes.
function DllUnregisterServer: HResult; stdcall;

implementation

uses
  Windows;

function DllCanUnloadNow: HResult; stdcall;
begin
{$ifdef DEBUG_COM}
  WriteLn('DllCanUnloadNow called');
{$endif}
  if ComServer.CanUnloadNow then
    Result := S_OK
  else
    Result := S_FALSE;
{$ifdef DEBUG_COM}
    WriteLn('DllCanUnloadNow return: ', Result);
{$endif}
end;

(*
    //FROM MSDN (Error messages are different in MSDN.DllGetClassObject)

    HRESULT hres = E_OUTOFMEMORY;
    *ppvObj = NULL;

    CClassFactory *pClassFactory = new CClassFactory(rclsid);
    if (pClassFactory != NULL)   {
        hRes = pClassFactory->QueryInterface(riid, ppvObj);
        pClassFactory->Release();
    }
    return hRes;
*)

function DllGetClassObject(constref rclsid: REFIID {should be REFCLSID}; constref riid: REFIID; out ppv: Pointer): HResult; stdcall;
var
  factory: TComObjectFactory;
begin
{$ifdef DEBUG_COM}
  WriteLn('DllGetClassObject called: ', GUIDToString(rclsid), ' ', GUIDToString(riid));
{$endif}

  factory := ComClassManager.GetFactoryFromClassID(rclsid);
  if factory = nil then
    Result := CLASS_E_CLASSNOTAVAILABLE
  else
  begin
    if factory.GetInterface(riid,ppv) then
      Result := S_OK
    else
      Result := CLASS_E_CLASSNOTAVAILABLE;
{$ifdef DEBUG_COM}
    WriteLn('DllGetClassObject return: ', Result);
{$endif}
  end;
end;

function DllRegisterServer: HResult; stdcall;
begin
{$ifdef DEBUG_COM}
  WriteLn('DllRegisterServer called');
{$endif}
  try
    ComServer.RegisterServer;
    Result := S_OK;
  except
    Result := SELFREG_E_CLASS;
  end;

end;

function DllUnregisterServer: HResult; stdcall;
begin
{$ifdef DEBUG_COM}
  WriteLn('DllUnregisterServer called');
{$endif}
  try
    ComServer.UnregisterServer;
    Result := S_OK;
  except
    Result := SELFREG_E_CLASS;
  end;
end;

function GetModuleFileName: String;
const
  MAX_PATH_SIZE = 2048;
var
  FileName: WideString;
begin
  SetLength(FileName, MAX_PATH_SIZE);
  SetLength(FileName, Windows.GetModuleFileNameW(HInstance, @FileName[1], MAX_PATH_SIZE));
  Result := FileName;
end;

function GetModuleName: String;
begin
  Result := ExtractFileName(GetModuleFileName);
  Result := Copy(Result, 1,LastDelimiter('.', Result)-1);
end;

procedure RegisterTypeLib(TypeLib: ITypeLib; const ModuleName: string);
var
  FullPath: WideString;
begin
  FullPath := ModuleName;
  //according to MSDN helpdir can be null
  OleCheck(ActiveX.RegisterTypeLib(TypeLib, @FullPath[1], nil));
end;

procedure UnRegisterTypeLib(TypeLib: ITypeLib);
var
  ptla: PTLibAttr;
begin
  //http://www.experts-exchange.com/Programming/Misc/Q_20634807.html
  OleCheck(TypeLib.GetLibAttr(ptla));
  try
    ActiveX.UnRegisterTypeLib(ptla^.guid, ptla^.wMajorVerNum, ptla^.wMinorVerNum, ptla^.lcid, ptla^.syskind);
  finally
    TypeLib.ReleaseTLibAttr(ptla);
  end;
end;


{ TComServer }

function TComServer.CountObject(Created: Boolean): Integer;
begin
  if Created then
  begin
    Result := InterlockedIncrement(FCountObject);
    if (not IsInProcServer) and (StartMode = smAutomation)
      and Assigned(ComObj.CoAddRefServerProcess) then
      ComObj.CoAddRefServerProcess;
  end
  else
  begin
    Result := InterlockedDecrement(FCountObject);
    if (not IsInProcServer) and (StartMode = smAutomation)
      and Assigned(ComObj.CoReleaseServerProcess) then
    begin
      if ComObj.CoReleaseServerProcess() = 0 then
        CheckReleased;
    end
    else if Result = 0 then
      CheckReleased;
  end;
end;

function TComServer.CountFactory(Created: Boolean): Integer;
begin
  if Created then
    Result:=InterLockedIncrement(fCountFactory)
  else
    Result:=InterLockedDecrement(fCountFactory);
end;

procedure TComServer.FactoryFree(Factory: TComObjectFactory);
begin
  Factory.Free;
end;

procedure TComServer.FactoryRegisterClassObject(Factory: TComObjectFactory);
begin
  Factory.RegisterClassObject;
end;

procedure TComServer.FactoryUpdateRegistry(Factory: TComObjectFactory);
begin
  if Factory.Instancing <> ciInternal then
    Factory.UpdateRegistry(FRegister);
end;

function TComServer.GetHelpFileName: string;
begin
  result:=fhelpfilename;
end;

function TComServer.GetServerFileName: string;
begin
  Result := fServerFileName;
end;

function TComServer.GetServerKey: string;
begin
  if FIsInproc then
    Result := 'InprocServer32'
  else
    Result := 'LocalServer32';
end;

function TComServer.GetServerName: string;
begin
  if FServerName <> '' then
    Result := FServerName
  else
    if FTypeLib <> nil then
      Result := GetTypeLibName
    else
      Result := GetModuleName;
end;

function TComServer.GetTypeLibName: widestring;
begin
  OleCheck(TypeLib.GetDocumentation(-1, @Result, nil, nil, nil));
end;


function TComServer.GetStartSuspended: Boolean;
begin
  result:=fStartSuspended;
end;

function TComServer.GetTypeLib: ITypeLib;
begin
  Result := fTypeLib;
end;

procedure TComServer.RegisterObjectWith(Factory: TComObjectFactory);
begin
  Factory.RegisterClassObject;
end;


procedure TComServer.Start;
begin
  case fStartMode of
  smRegServer:
    begin
      Self.RegisterServer;
      Halt;
    end;
  smUnregServer:
    begin
      Self.UnRegisterServer;
      Halt;
    end;
  end;
  ComClassManager.ForEachFactory(Self, @RegisterObjectWith);
end;


procedure TComServer.SetHelpFileName(const Value: string);
begin
  FHelpFileName:=value;
end;

procedure TComServer.RegisterServerFactory(Factory: TComObjectFactory);
begin
  Factory.UpdateRegistry(True);
end;

procedure TComServer.UnregisterServerFactory(Factory: TComObjectFactory);
begin
  Factory.UpdateRegistry(False);
end;

procedure TComServer.CheckCmdLine;
const
  sw_set:TSysCharSet = ['/','-'];
begin
  if FindCmdLineSwitch('automation',sw_set,true) or
     FindCmdLineSwitch('embedding',sw_set,true) then
    fStartMode := smAutomation
  else if FindCmdlIneSwitch('regserver',sw_set,true) then
    fStartMode := smRegServer
  else if FindCmdLineSwitch('unregserver',sw_set,true) then
    fStartMode := smUnregServer;
end;

constructor TComServer.Create;
var
  name: WideString;
begin
  haut := SafeLoadLibrary('oleaut32.DLL');
  CheckCmdLine;
  inherited Create;
{$ifdef DEBUG_COM}
  WriteLn('TComServer.Create');
{$endif}
  fCountFactory := 0;
  fCountObject := 0;

  FTypeLib := nil;
  FIsInproc := ModuleIsLib;

  fServerFileName := GetModuleFileName();

  name := fServerFileName;
  if not(Succeeded(LoadTypeLib(@name[1], fTypeLib))) then
    fTypeLib := nil;

  if FTypeLib <> nil then
  begin
    fTypeLib.GetDocumentation(-1, @name, nil, nil, nil);
    fServerName := name;
  end
  else
    fServerName := GetModuleName;

  if not ModuleIsLib then
  begin
    orgInitProc := InitProc;
    InitProc := @TComServer.AutomationStart;
  //  AddTerminateProc(TTerminateProc(@TComServer.AutomationDone));
  end;

  Self.FIsInteractive := True;
end;

class procedure TComServer.AutomationStart;
begin
  if orgInitProc <> nil then TProcedure(orgInitProc)();
  ComServer.FStartSuspended := (CoInitFlags <> -1) and
    Assigned(ComObj.CoInitializeEx) and Assigned(ComObj.CoResumeClassObjects);
  ComServer.Start;
  if ComServer.FStartSuspended then
    ComObj.CoResumeClassObjects;
end;

class function TComServer.AutomationDone: Boolean;
begin
  Result := True;
  if (ComServer <> nil) and (ComServer.ServerObjects > 0) and ComServer.IsInteractive then
  begin
    Result := MessageBox(0, PChar('COM server is in use'),
      PChar('OLE Automation'), MB_YESNO or MB_TASKMODAL or
      MB_ICONWARNING or MB_DEFBUTTON2) = IDYES;
  end;
end;


procedure TComServer.CheckReleased;
var
  Shutdown: Boolean;
begin
  if not FIsInproc then
  begin
    Shutdown := FStartMode = smAutomation;
    try
      if Assigned(FOnLastRelease) then FOnLastRelease(Shutdown);
    finally
      if Shutdown then PostThreadMessage(MainThreadID, WM_QUIT, 0, 0);
    end;
  end;
end;


destructor TComServer.Destroy;
begin
  ComClassManager.ForEachFactory(Self, @FactoryFree,true);
  Self.fTypeLib:=nil;
  inherited Destroy;
  FreeLibrary(haut);
{$ifdef DEBUG_COM}
  WriteLn('TComServer.Destroy');
{$endif}
end;

function TComServer.CanUnloadNow: Boolean;
begin
  Result := False;
end;

procedure TComServer.RegisterServer;
begin
  if fTypeLib <> nil then
    RegisterTypeLib(fTypeLib, fServerFileName);

  ComClassManager.ForEachFactory(self, @RegisterServerFactory);
end;

procedure TComServer.UnRegisterServer;
begin
  if fTypeLib <> nil then
    UnRegisterTypeLib(fTypeLib);

  ComClassManager.ForEachFactory(self, @UnregisterServerFactory);
end;


initialization
{$ifdef DEBUG_COM}
  WriteLn('comserv initialization begin');
{$endif}
  ComServer := TComServer.Create;

{$ifdef DEBUG_COM}
  WriteLn('comserv initialization end');
{$endif}
finalization
  ComServer.AutomationDone;
  FreeAndNil(ComServer);
end.
