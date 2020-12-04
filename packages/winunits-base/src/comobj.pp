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
unit ComObj;

  interface

{ $define DEBUG_COM}
{ $define DEBUG_COMDISPATCH}

{$ifdef wince}
  {$define DUMMY_REG}
{$endif}
    uses
      Windows,Types,Variants,Sysutils,ActiveX,classes;

    type
      EOleError = class(Exception);
     
      // apparantly used by axctrls.
      // http://lazarus.freepascal.org/index.php/topic,11612.0.html
      TConnectEvent = procedure(const Sink: IUnknown; Connecting: Boolean) of object;

      EOleSysError = class(EOleError)
      private
        FErrorCode: HRESULT;
      public
        constructor Create(const Msg: string; aErrorCode: HRESULT;aHelpContext: Integer);
        property ErrorCode: HRESULT read FErrorCode write FErrorCode;
      end;

      EOleException = class(EOleSysError)
      private
        FHelpFile: string;
        FSource: string;
      public
        constructor Create(const Msg: string; aErrorCode: HRESULT;const aSource,aHelpFile: string;aHelpContext: Integer);
        property HelpFile: string read FHelpFile write FHelpFile;
        property Source: string read FSource write FSource;
      end;

      EOleRegistrationError = class(EOleSysError);

      TOleStream = Class(TProxyStream)
        procedure Check(err:integer);override;
      end;

      TComServerObject = class(TObject)
      protected
        function CountObject(Created: Boolean): Integer; virtual; abstract;
        function CountFactory(Created: Boolean): Integer; virtual; abstract;
        function GetHelpFileName: string; virtual; abstract;
        function GetServerFileName: string; virtual; abstract;
        function GetServerKey: string; virtual; abstract;
        function GetServerName: string; virtual; abstract;
        function GetStartSuspended: Boolean; virtual; abstract;
        function GetTypeLib: ITypeLib; virtual; abstract;
        procedure SetHelpFileName(const Value: string); virtual; abstract;
      public
        property HelpFileName: string read GetHelpFileName write SetHelpFileName;
        property ServerFileName: string read GetServerFileName;
        property ServerKey: string read GetServerKey;
        property ServerName: string read GetServerName;
        property TypeLib: ITypeLib read GetTypeLib;
        property StartSuspended: Boolean read GetStartSuspended;
      end;

      TComObjectFactory = class;

      TFactoryProc = procedure(Factory: TComObjectFactory) of object;

      { TComClassManager }

      TComClassManager = class(TObject)
      private
        fClassFactoryList: TList;
      public
        constructor Create;
        destructor Destroy; override;
        procedure AddObjectFactory(factory: TComObjectFactory);
        procedure RemoveObjectFactory(factory: TComObjectFactory);
        procedure ForEachFactory(ComServer: TComServerObject; FactoryProc: TFactoryProc;const bBackward:boolean=false);
        function GetFactoryFromClass(ComClass: TClass): TComObjectFactory;
        function GetFactoryFromClassID(const ClassID: TGUID): TComObjectFactory;
      end;

      IServerExceptionHandler = interface
        ['{6A8D432B-EB81-11D1-AAB1-00C04FB16FBC}']
        procedure OnException(const ServerClass, ExceptionClass, ErrorMessage: WideString;
          ExceptAddr: PtrInt; const ErrorIID, ProgID: WideString; var Handled: Integer; var Result: HResult); dispid 2;
      end;

      TComObject = class(TObject, IUnknown, ISupportErrorInfo)
      private
        FController : Pointer;
        FFactory : TComObjectFactory;
        FRefCount : Integer;
        FServerExceptionHandler : IServerExceptionHandler;
        FCounted : Boolean;
        function GetController : IUnknown;
      protected
        { IUnknown }
        function IUnknown.QueryInterface = ObjQueryInterface;
        function IUnknown._AddRef = ObjAddRef;
        function IUnknown._Release = ObjRelease;

        { IUnknown methods for other interfaces }
        function QueryInterface(constref IID: TGUID; out Obj): HResult; stdcall;
        function _AddRef: Integer; stdcall;
        function _Release: Integer; stdcall;

        { ISupportErrorInfo }
        function InterfaceSupportsErrorInfo(const iid: TIID): HResult; stdcall;
      public
        constructor Create;
        constructor CreateAggregated(const Controller: IUnknown);
        constructor CreateFromFactory(Factory: TComObjectFactory; const Controller: IUnknown);
        destructor Destroy; override;
        procedure Initialize; virtual;
        function ObjAddRef: Integer; virtual; stdcall;
        function ObjQueryInterface(constref IID: TGUID; out Obj): HResult; virtual; stdcall;
        function ObjRelease: Integer; virtual; stdcall;
        function SafeCallException(ExceptObject: TObject; ExceptAddr: Pointer): HResult; override;
        property Controller: IUnknown read GetController;
        property Factory: TComObjectFactory read FFactory;
        property RefCount: Integer read FRefCount;
        property ServerExceptionHandler: IServerExceptionHandler read FServerExceptionHandler write FServerExceptionHandler;
      end;
      TComClass = class of TComObject;

      TClassInstancing = (ciInternal, ciSingleInstance, ciMultiInstance);
      TThreadingModel = (tmSingle, tmApartment, tmFree, tmBoth, tmNeutral);

      { TComObjectFactory }

      TComObjectFactory = class(TObject, IUnknown, IClassFactory, IClassFactory2)
      private
        FRefCount : Integer;
        //Next: TComObjectFactory;
        FComServer: TComServerObject;
        FComClass: TClass;
        FClassID: TGUID;
        FClassName: string;
        FClassVersion : String;
        FDescription: string;
        FErrorIID: TGUID;
        FInstancing: TClassInstancing;
        FLicString: WideString;
        FIsRegistered: dword;
        FShowErrors: Boolean;
        FSupportsLicensing: Boolean;
        FThreadingModel: TThreadingModel;
        function GetProgID: string;
        function reg_flags(): integer;
      protected
        { IUnknown }
        function QueryInterface(constref IID: TGUID; out Obj): HResult; stdcall;
        function _AddRef: Integer; stdcall;
        function _Release: Integer; stdcall;
        { IClassFactory }
        function CreateInstance(const UnkOuter: IUnknown; const IID: TGUID;
          out Obj): HResult; stdcall;
        function LockServer(fLock: BOOL): HResult; stdcall;
        { IClassFactory2 }
        function GetLicInfo(var licInfo: TLicInfo): HResult; stdcall;
        function RequestLicKey(dwResrved: DWORD; out bstrKey: WideString): HResult; stdcall;
        function CreateInstanceLic(const unkOuter: IUnknown; const unkReserved: IUnknown;
          const iid: TIID; const bstrKey: WideString; out vObject): HResult; stdcall;
      public
        constructor Create(ComServer: TComServerObject; ComClass: TComClass;
          const ClassID: TGUID; const Name, Description: string;
          Instancing: TClassInstancing; ThreadingModel: TThreadingModel = tmSingle);
        constructor Create(ComServer: TComServerObject; ComClass: TComClass;
          const ClassID: TGUID; const Name, Version, Description: string;
          Instancing: TClassInstancing; ThreadingModel: TThreadingModel = tmSingle);
        destructor Destroy; override;
        function CreateComObject(const Controller: IUnknown): TComObject; virtual;
        procedure RegisterClassObject;
        procedure UpdateRegistry(Register: Boolean); virtual;
        property ClassID: TGUID read FClassID;
        property ClassName: string read FClassName;
        property ClassVersion: string read FClassVersion;
        property ComClass: TClass read FComClass;
        property ComServer: TComServerObject read FComServer;
        property Description: string read FDescription;
        property ErrorIID: TGUID read FErrorIID write FErrorIID;
        property LicString: WideString read FLicString write FLicString;
        property ProgID: string read GetProgID;
        property Instancing: TClassInstancing read FInstancing;
        property ShowErrors: Boolean read FShowErrors write FShowErrors;
        property SupportsLicensing: Boolean read FSupportsLicensing write FSupportsLicensing;
        property ThreadingModel: TThreadingModel read FThreadingModel;
      end;

      { TTypedComObject }

      TTypedComObject = class(TComObject, IProvideClassInfo)
        function GetClassInfo(out pptti : ITypeInfo):HResult; StdCall;
      end;

      TTypedComClass = class of TTypedComObject;

      { TTypedComObjectFactory }

      TTypedComObjectFactory = class(TComObjectFactory)
      private
        FClassInfo: ITypeInfo;
        FTypeInfoCount:integer;
      public
        constructor Create(AComServer: TComServerObject; TypedComClass: TTypedComClass; const AClassID: TGUID;
          AInstancing: TClassInstancing; AThreadingModel: TThreadingModel = tmSingle);
        function GetInterfaceTypeInfo(TypeFlags: Integer) : ITypeInfo;
        procedure UpdateRegistry(Register: Boolean);override;
        property ClassInfo : ITypeInfo read FClassInfo;
      end;

      { TAutoObject }

      TAutoObject = class(TTypedComObject, IDispatch)
      protected
        { IDispatch }
        function GetTypeInfoCount(out count : longint) : HResult;stdcall;
        function GetTypeInfo(Index,LocaleID : longint; out TypeInfo): HResult;stdcall;
        function GetIDsOfNames(const iid: TGUID; names: Pointer; NameCount, LocaleID: LongInt; DispIDs: Pointer) : HResult;stdcall;
        function Invoke(DispID: LongInt;const iid : TGUID; LocaleID : longint; Flags: Word;var params; VarResult,ExcepInfo,ArgErr : pointer) : HResult;stdcall;
      public

      end;

      TAutoClass = class of TAutoObject;

      { TAutoObjectFactory }
      TAutoObjectFactory = class(TTypedComObjectFactory)
      private
        FDispIntfEntry: PInterfaceEntry;
        FDispTypeInfo: ITypeInfo;
      public
        constructor Create(AComServer: TComServerObject; AutoClass: TAutoClass; const AClassID: TGUID;
          AInstancing: TClassInstancing; AThreadingModel: TThreadingModel = tmSingle);
        function GetIntfEntry(Guid: TGUID): PInterfaceEntry; virtual;
        property DispIntfEntry: PInterfaceEntry read FDispIntfEntry;
        property DispTypeInfo: ITypeInfo read FDispTypeInfo;
      end;

      { TAutoIntfObject }

      //example of how to implement IDispatch: http://www.opensource.apple.com/source/vim/vim-34/vim/src/if_ole.cpp
      TAutoIntfObject = class(TInterfacedObject, IDispatch, ISupportErrorInfo)
      private
        fTypeInfo: ITypeInfo;
        fInterfacePointer: Pointer;
      protected
        { IDispatch }
        function GetTypeInfoCount(out count : longint) : HResult;stdcall;
        function GetTypeInfo(Index,LocaleID : longint; out TypeInfo): HResult;stdcall;
        function GetIDsOfNames(const iid: TGUID; names: Pointer; NameCount, LocaleID: LongInt; DispIDs: Pointer) : HResult;stdcall;
        function Invoke(DispID: LongInt;const iid : TGUID; LocaleID : longint; Flags: Word;var params; VarResult,ExcepInfo,ArgErr : pointer) : HResult;stdcall;

        { ISupportErrorInfo }
        function  InterfaceSupportsErrorInfo(CONST riid: TIID):HResult;StdCall;
      public
        function SafeCallException(ExceptObject: TObject; ExceptAddr: Pointer): HResult; override;
        constructor Create(TypeLib: ITypeLib; const Guid: TGuid);
      end;

    function CreateClassID : ansistring;

    function CreateComObject(const ClassID: TGUID) : IUnknown;
    function CreateRemoteComObject(const MachineName : WideString;const ClassID : TGUID) : IUnknown;
    function CreateOleObject(const ClassName : string) : IDispatch;
    function GetActiveOleObject(const ClassName: string) : IDispatch;

    procedure OleCheck(Value : HResult);inline;
    procedure OleError(Code: HResult);

    function ProgIDToClassID(const id : string) : TGUID;
    function ClassIDToProgID(const classID: TGUID): string;

    function StringToLPOLESTR(const Source: string): POLEStr;

    procedure InterfaceConnect(const Source: IUnknown; const IID: TIID; const Sink: IUnknown; var Connection: DWORD);
    procedure InterfaceDisconnect(const Source: IUnknown; const IID: TIID; var Connection: DWORD);

    procedure DispatchInvoke(const Dispatch: IDispatch; CallDesc: PCallDesc;
       DispIDs: PDispIDList; Params: Pointer; Result: PVariant);
    procedure DispatchInvokeError(Status: HRESULT; const ExceptInfo: TExcepInfo);

    function HandleSafeCallException(ExceptObject: TObject; ExceptAddr: Pointer; const ErrorIID: TGUID; const ProgID,
      HelpFileName: WideString): HResult;

    function ComClassManager : TComClassManager;

    procedure CreateRegKey(const Key, ValueName, Value: string; RootKey: HKey= HKEY_CLASSES_ROOT);
    procedure DeleteRegKey(const Key: string; RootKey: HKey = HKEY_CLASSES_ROOT);
    function GetRegStringValue(const Key, ValueName: string; RootKey: HKey = HKEY_CLASSES_ROOT): string;

    type
      TCoCreateInstanceExProc = function(const clsid: TCLSID; unkOuter: IUnknown; dwClsCtx: DWORD; ServerInfo: PCoServerInfo;
      dwCount: ULONG; rgmqResults: PMultiQIArray): HResult stdcall;
      TCoInitializeExProc = function (pvReserved: Pointer;
      coInit: DWORD): HResult; stdcall;
      TCoAddRefServerProcessProc = function : ULONG; stdcall;
      TCoReleaseServerProcessProc = function : ULONG; stdcall;
      TCoResumeClassObjectsProc = function : HResult; stdcall;
      TCoSuspendClassObjectsProc = function : HResult; stdcall;

    const
      CoCreateInstanceEx : TCoCreateInstanceExProc = nil;
      CoInitializeEx : TCoInitializeExProc = nil;
      CoAddRefServerProcess : TCoAddRefServerProcessProc = nil;
      CoReleaseServerProcess : TCoReleaseServerProcessProc = nil;
      CoResumeClassObjects : TCoResumeClassObjectsProc = nil;
      CoSuspendClassObjects : TCoSuspendClassObjectsProc = nil;
      CoInitFlags : Longint = -1;
      CoInitDisable : Boolean = False;

  {$ifdef DEBUG_COM}
     var printcom : boolean=true;
  {$endif}
implementation

    uses
      ComConst, Ole2, {$ifndef dummy_reg} Registry, {$endif} RtlConsts;

    var
      Uninitializing : boolean;

    function HandleSafeCallException(ExceptObject: TObject; ExceptAddr: Pointer; const ErrorIID: TGUID; const ProgID,
      HelpFileName: WideString): HResult;
{$ifndef wince}
      var
        _CreateErrorInfo : ICreateErrorInfo;
        ErrorInfo : IErrorInfo;
{$endif wince}
      begin
        Result:=E_UNEXPECTED;
{$ifndef wince}
        if Succeeded(CreateErrorInfo(_CreateErrorInfo)) then
          begin
            _CreateErrorInfo.SetGUID(ErrorIID);
            if ProgID<>'' then
              _CreateErrorInfo.SetSource(PWidechar(ProgID));
            if HelpFileName<>'' then
              _CreateErrorInfo.SetHelpFile(PWidechar(HelpFileName));
            if ExceptObject is Exception then
              begin
                _CreateErrorInfo.SetDescription(PWidechar(Widestring(Exception(ExceptObject).Message)));
                _CreateErrorInfo.SetHelpContext(Exception(ExceptObject).HelpContext);
                if (ExceptObject is EOleSyserror) and (EOleSysError(ExceptObject).ErrorCode<0) then
                  Result:=EOleSysError(ExceptObject).ErrorCode
              end;
            if _CreateErrorInfo.QueryInterface(IErrorInfo,ErrorInfo)=S_OK then
              SetErrorInfo(0,ErrorInfo);
          end;
{$endif wince}
      end;


    constructor EOleSysError.Create(const Msg: string; aErrorCode: HRESULT; aHelpContext: Integer);
      var
        m : string;
      begin
        if Msg='' then
          m:=SysErrorMessage(aErrorCode)
        else
          m:=Msg;
        inherited CreateHelp(m,HelpContext);
        FErrorCode:=aErrorCode;
      end;


    constructor EOleException.Create(const Msg: string; aErrorCode: HRESULT;const aSource,aHelpFile: string; aHelpContext: Integer);
      begin
        inherited Create(Msg,aErrorCode,aHelpContext);
        FHelpFile:=aHelpFile;
        FSource:=aSource;
      end;

    {$define FPC_COMOBJ_HAS_CREATE_CLASS_ID}
    function CreateClassID : ansistring;
      var
         ClassID : TCLSID;
         p : PWideChar;
      begin
         CoCreateGuid(ClassID);
         StringFromCLSID(ClassID,p);
         result:=p;
         CoTaskMemFree(p);
      end;


   function CreateComObject(const ClassID : TGUID) : IUnknown;
     begin
       OleCheck(CoCreateInstance(ClassID,nil,CLSCTX_INPROC_SERVER or CLSCTX_LOCAL_SERVER,IUnknown,result));
     end;


   function CreateRemoteComObject(const MachineName : WideString;const ClassID : TGUID) : IUnknown;
     var
       flags : DWORD;
       localhost : array[0..MAX_COMPUTERNAME_LENGTH] of WideChar;
       server : TCoServerInfo;
       mqi : TMultiQI;
       size : DWORD;
     begin
       if not(assigned(CoCreateInstanceEx)) then
         raise Exception.CreateRes(@SDCOMNotInstalled);

       FillChar(server,sizeof(server),0);
       server.pwszName:=PWideChar(MachineName);

       FillChar(mqi,sizeof(mqi),0);
       mqi.iid:=@IID_IUnknown;

       flags:=CLSCTX_LOCAL_SERVER or CLSCTX_REMOTE_SERVER or CLSCTX_INPROC_SERVER;

       { actually a remote call? }
{$ifndef wince}
       //roozbeh although there is a way to retrive computer name...HKLM\Ident\Name..but are they same?
	     size:=sizeof(localhost);
       if (MachineName<>'') and
          (not(GetComputerNameW(localhost,size)) or
          (WideCompareText(localhost,MachineName)<>0)) then
           flags:=CLSCTX_REMOTE_SERVER;
{$endif}

       OleCheck(CoCreateInstanceEx(ClassID,nil,flags,@server,1,@mqi));
       OleCheck(mqi.hr);
       Result:=mqi.itf;
     end;


   function CreateOleObject(const ClassName : string) : IDispatch;
     var
       id : TCLSID;
     begin
        id:=ProgIDToClassID(ClassName);
        OleCheck(CoCreateInstance(id,nil,CLSCTX_INPROC_SERVER or CLSCTX_LOCAL_SERVER,IDispatch,result));
     end;

   function GetActiveOleObject(const ClassName : string) : IDispatch;
{$ifndef wince}
     var
     	 intf : IUnknown;
       id : TCLSID;
     begin
       id:=ProgIDToClassID(ClassName);
       OleCheck(GetActiveObject(id,nil,intf));
       OleCheck(intf.QueryInterface(IDispatch,Result));
     end;
{$else}
     begin
       Result:=nil;
     end;
{$endif wince}

    procedure CreateRegKey(const Key, ValueName, Value: string; RootKey: HKEY = HKEY_CLASSES_ROOT);
{$ifndef DUMMY_REG}
      var
        Reg: TRegistry;
{$endif}
      begin
{$ifdef DEBUG_COM}
         if printcom then 
        WriteLn('CreateRegKey: ', Key, ': ', ValueName, ': ', Value );
{$endif}
{$ifndef DUMMY_REG}
        Reg := TRegistry.Create;
        try
          Reg.RootKey := RootKey;
          if Reg.OpenKey(Key, True) then
          begin
            try
              Reg.WriteString(ValueName, Value);
            finally
              Reg.CloseKey;
            end;
          end
          else
            raise EOleRegistrationError.CreateResFmt(@SRegCreateFailed,[Key]);
        finally
          Reg.Free;
        end;
{$endif}
{$ifdef DEBUG_COM}
         if printcom then 
        WriteLn('CreateRegKey exit: ', Key, ': ', ValueName, ': ', Value );
{$endif}
      end;


    procedure DeleteRegKey(const Key: string; RootKey: HKEY = HKEY_CLASSES_ROOT);
{$ifndef DUMMY_REG}
      var
        Reg: TRegistry;
{$endif}
      begin
{$ifdef DEBUG_COM}
         if printcom then 
        WriteLn('DeleteRegKey: ', Key);
{$endif}
{$ifndef DUMMY_REG}
        Reg := TRegistry.Create;
        try
          Reg.RootKey := RootKey;
          Reg.DeleteKey(Key);
        finally
          Reg.Free;
        end;
{$endif}
      end;


    function GetRegStringValue(const Key, ValueName: string; RootKey: HKEY = HKEY_CLASSES_ROOT): string;
    {$ifndef DUMMY_REG}
      var
        Reg: TRegistry;
    {$endif}
      begin
       {$ifndef DUMMY_REG}
        Reg := TRegistry.Create();
        try
          Reg.RootKey := RootKey;
          if Reg.OpenKeyReadOnly(Key) then
          begin
            try
              Result := Reg.ReadString(ValueName)
            finally
              Reg.CloseKey;
            end;
          end
          else
            Result := '';
        finally
          Reg.Free;
        end;
       {$endif}
      end;


   procedure OleError(Code: HResult);
     begin
       raise EOleSysError.Create('',Code,0);
     end;


   procedure OleCheck(Value : HResult);inline;
     begin
       if not(Succeeded(Value)) then
         OleError(Value);
     end;


   function ProgIDToClassID(const id : string) : TGUID;
     begin
       OleCheck(CLSIDFromProgID(PWideChar(WideString(id)),result));
     end;


   function ClassIDToProgID(const classID: TGUID): string;
     var
       progid : LPOLESTR;
     begin
       OleCheck(ProgIDFromCLSID(@classID,progid));
       result:=progid;
       CoTaskMemFree(progid);
     end;


   function StringToLPOLESTR(const Source: string): POLEStr;
     var
       Src: WideString;
     begin
       Src := WideString(Source);
       Result := CoTaskMemAlloc((Length(Src)+1) * SizeOf(WideChar));
       if Result <> nil then
         Move(PWideChar(Src)^, Result^, (Length(Src)+1) * SizeOf(WideChar));
     end;


   procedure InterfaceConnect(const Source: IUnknown; const IID: TIID; const Sink: IUnknown; var Connection: DWORD);
     var
       CPC: IConnectionPointContainer;
       CP: IConnectionPoint;
       i: hresult;
     begin
       Connection := 0;
       if Succeeded(Source.QueryInterface(IConnectionPointContainer, CPC)) then
         if Succeeded(CPC.FindConnectionPoint(IID, CP)) then
           i:=CP.Advise(Sink, Connection);
     end;


   procedure InterfaceDisconnect(const Source: IUnknown; const IID: TIID; var Connection: DWORD);
     var
       CPC: IConnectionPointContainer;
       CP: IConnectionPoint;
       i: hresult;
     begin
       if Connection <> 0 then
         if Succeeded(Source.QueryInterface(IConnectionPointContainer, CPC)) then
           if Succeeded(CPC.FindConnectionPoint(IID, CP)) then
           begin
             i:=CP.Unadvise(Connection);
             if Succeeded(i) then Connection := 0;
           end;
     end;


   procedure SafeCallErrorHandler(err : HResult;addr : pointer);
{$ifndef wince}
     var
       info : IErrorInfo;
       descr,src,helpfile : widestring;
       helpctx : DWORD;
{$endif wince}
     begin
{$ifndef wince}
       if GetErrorInfo(0,info)=S_OK then
         begin
           info.GetDescription(descr);
           info.GetSource(src);
           info.GetHelpFile(helpfile);
           info.GetHelpContext(helpctx);
           raise EOleException.Create(descr,err,src,helpfile,helpctx) at addr;
         end
       else
{$endif wince}
         raise EOleException.Create('',err,'','',0) at addr;
     end;


    procedure DispatchInvokeError(Status: HRESULT; const ExceptInfo: TExcepInfo);
      begin
        if Status=DISP_E_EXCEPTION then
          raise EOleException.Create(ExceptInfo.Description,ExceptInfo.scode,ExceptInfo.Source,
            ExceptInfo.HelpFile,ExceptInfo.dwHelpContext)
        else
          raise EOleSysError.Create('',Status,0);
      end;

    var
      _ComClassManager : TComClassManager;

    function ComClassManager: TComClassManager;
      begin
        if not(assigned(_ComClassManager)) then
          _ComClassManager:=TComClassManager.Create;
        Result:=_ComClassManager;
      end;


    constructor TComClassManager.Create;
      begin
        fClassFactoryList := TList.create({true});
      end;


    destructor TComClassManager.Destroy;
      var i : integer;
      begin
        if fClassFactoryList.count>0 Then
           begin
             for i:=fClassFactoryList.count-1 downto 0 do
                tobject(fClassFactoryList[i]).Free;
           end;
        fClassFactoryList.Free;
      end;

    procedure TComClassManager.AddObjectFactory(factory: TComObjectFactory);
      begin
{$ifdef DEBUG_COM}
         if printcom then 
        WriteLn('AddObjectFactory: ', GUIDToString(factory.FClassID), ' ', factory.FClassName);
{$endif}
        fClassFactoryList.Add(factory);
      end;

    procedure TComClassManager.RemoveObjectFactory(
          factory: TComObjectFactory);
      begin
        fClassFactoryList.Remove(factory);
      end;

    procedure TComClassManager.ForEachFactory(ComServer: TComServerObject;
      FactoryProc: TFactoryProc;const bBackward:boolean=false);
      var
        i: Integer;
        obj: TComObjectFactory;
      begin
{$ifdef DEBUG_COM}
         if printcom then 
        WriteLn('ForEachFactory');
{$endif}
        if not bBackward then
        for i := 0 to fClassFactoryList.Count - 1 do
        begin
          obj := TComObjectFactory(fClassFactoryList[i]);
          if obj.ComServer = ComServer then
            FactoryProc(obj);
        end
        else
        for i := fClassFactoryList.Count - 1 downto 0 do
        begin
          obj := TComObjectFactory(fClassFactoryList[i]);
          if obj.ComServer = ComServer then
            FactoryProc(obj);
        end
      end;


    function TComClassManager.GetFactoryFromClass(ComClass: TClass
      ): TComObjectFactory;
      var
        i: Integer;
      begin
{$ifdef DEBUG_COM}
         if printcom then 
        WriteLn('GetFactoryFromClass: ', ComClass.ClassName);
{$endif}
        for i := 0 to fClassFactoryList.Count - 1 do
        begin
          Result := TComObjectFactory(fClassFactoryList[i]);
          if ComClass = Result.ComClass then
            Exit();
        end;
        Result := nil;
      end;


    function TComClassManager.GetFactoryFromClassID(const ClassID: TGUID
      ): TComObjectFactory;
      var
        i: Integer;
      begin
{$ifdef DEBUG_COM}
         if printcom then 
        WriteLn('GetFactoryFromClassID: ', GUIDToString(ClassId));
{$endif}
        for i := 0 to fClassFactoryList.Count - 1 do
        begin
          Result := TComObjectFactory(fClassFactoryList[i]);
          if IsEqualGUID(ClassID, Result.ClassID) then
            Exit();
        end;
{$ifdef DEBUG_COM}
         if printcom then 
        WriteLn('GetFactoryFromClassID not found: ', GUIDToString(ClassId));
{$endif}
        Result := nil;
      end;


    function TComObject.GetController: IUnknown;
      begin
        Result:=IUnknown(Controller);
      end;


    function TComObject.QueryInterface(constref IID: TGUID; out Obj): HResult; stdcall;
      begin
        if assigned(FController) then
          Result:=IUnknown(FController).QueryInterface(IID,Obj)
        else
          Result:=ObjQueryInterface(IID,Obj);
      end;


    function TComObject._AddRef: Integer; stdcall;
      begin
        if assigned(FController) then
          Result:=IUnknown(FController)._AddRef
        else
          Result:=ObjAddRef;
      end;


    function TComObject._Release: Integer; stdcall;
      begin
        if assigned(FController) then
          Result:=IUnknown(FController)._Release
        else
          Result:=ObjRelease;
      end;


    function TComObject.InterfaceSupportsErrorInfo(const iid: TIID): HResult; stdcall;
      begin
        if assigned(GetInterfaceEntry(iid)) then
          Result:=S_OK
        else
          Result:=S_FALSE;
      end;


    constructor TComObject.Create;
      begin
         CreateFromFactory(ComClassManager.GetFactoryFromClass(ClassType),nil);
      end;


    constructor TComObject.CreateAggregated(const Controller: IUnknown);
      begin
        CreateFromFactory(ComClassManager.GetFactoryFromClass(ClassType),Controller);
      end;


    constructor TComObject.CreateFromFactory(Factory: TComObjectFactory;
      const Controller: IUnknown);
      begin
        FFactory:=Factory;
        FRefCount:=1;
        FController:=Pointer(Controller);
        FFactory.Comserver.CountObject(True);
        FCounted:=true;
        Initialize;
        Dec(FRefCount);
      end;


    destructor TComObject.Destroy;
      begin
        if not(Uninitializing) then
          begin
            if assigned(FFactory) and FCounted then
              FFactory.Comserver.CountObject(false);
{$ifndef wince}
            if FRefCount>0 then
              CoDisconnectObject(Self,0);
{$endif wince}
          end;
      end;


    procedure TComObject.Initialize;
      begin
      end;


    function TComObject.ObjAddRef: Integer; stdcall;
      begin
        Result:=InterlockedIncrement(FRefCount);
      end;


    function TComObject.ObjQueryInterface(constref IID: TGUID; out Obj): HResult; stdcall;
      begin
        if GetInterface(IID,Obj) then
          Result:=S_OK
        else
          Result:=E_NOINTERFACE;
      end;


    function TComObject.ObjRelease: Integer; stdcall;
      begin
        Result:=InterlockedDecrement(FRefCount);
        if Result=0 then
          Self.Destroy;
      end;


    function TComObject.SafeCallException(ExceptObject: TObject; ExceptAddr: Pointer): HResult;
      var
        Message: string;
        Handled: Integer;
      begin
        Handled:=0;
        Result:=0;
        if assigned(ServerExceptionHandler) then
          begin
            if ExceptObject is Exception then
              Message:=Exception(ExceptObject).Message;

            ServerExceptionHandler.OnException(ClassName,ExceptObject.ClassName,
              Message,PtrInt(ExceptAddr),WideString(GUIDToString(FFactory.ErrorIID)),
              FFactory.ProgID,Handled,Result);
          end;
        if Handled=0 then
          Result:=HandleSafeCallException(ExceptObject,ExceptAddr,FFactory.ErrorIID,
            FFactory.ProgID,FFactory.ComServer.HelpFileName);
      end;


    function TComObjectFactory.GetProgID: string;
      begin
        Result := FComServer.GetServerName + '.' + FClassName;
      end;


    function TComObjectFactory.QueryInterface(constref IID: TGUID; out Obj): HResult; stdcall;
      begin
        if GetInterface(IID,Obj) then
          Result:=S_OK
        else
          Result:=E_NOINTERFACE;
      end;


    function TComObjectFactory._AddRef: Integer; stdcall;
      begin
        Result:=InterlockedIncrement(FRefCount);
      end;


    function TComObjectFactory._Release: Integer; stdcall;
      begin
        Result:=InterlockedDecrement(FRefCount);
        if Result=0 then
          Self.Destroy;
      end;


    function TComObjectFactory.CreateInstance(const UnkOuter: IUnknown;
      const IID: TGUID; out Obj): HResult; stdcall;
      var
        comObject: TComObject;
      begin
{$ifdef DEBUG_COM}
         if printcom then 
        WriteLn('CreateInstance: ', GUIDToString(IID));
{$endif}
        comObject := CreateComObject(UnkOuter);
        if comObject.GetInterface(IID, Obj) then
          Result := S_OK
        else
          Result := E_NOINTERFACE;
      end;


    function TComObjectFactory.LockServer(fLock: BOOL): HResult; stdcall;
      begin
{$ifdef DEBUG_COM}
         if printcom then 
        WriteLn('LockServer: ', fLock);
{$endif}
{$ifndef wince}
          Result := CoLockObjectExternal(Self, fLock, True);
          ComServer.CountObject(fLock);
{$else}
          RunError(217);
          Result:=0;
{$endif}
      end;


    function TComObjectFactory.GetLicInfo(var licInfo: TLicInfo): HResult; stdcall;
      begin
{$ifdef DEBUG_COM}
         if printcom then 
        WriteLn('GetLicInfo');
{$endif}
        RunError(217);
        Result:=0;
      end;


    function TComObjectFactory.RequestLicKey(dwResrved: DWORD; out bstrKey: WideString): HResult; stdcall;
      begin
{$ifdef DEBUG_COM}
         if printcom then 
        WriteLn('RequestLicKey');
{$endif}
        RunError(217);
        Result:=0;
      end;


    function TComObjectFactory.CreateInstanceLic(const unkOuter: IUnknown;
      const unkReserved: IUnknown; const iid: TIID; const bstrKey: WideString; out
      vObject): HResult; stdcall;
      begin
{$ifdef DEBUG_COM}
         if printcom then 
        WriteLn('CreateInstanceLic');
{$endif}
        RunError(217);
        Result:=0;
      end;


    constructor TComObjectFactory.Create(ComServer: TComServerObject;
      ComClass: TComClass; const ClassID: TGUID; const Name,
      Description: string; Instancing: TClassInstancing;
      ThreadingModel: TThreadingModel);
      begin
        Create(ComServer, ComClass, ClassID, Name, '', Description, Instancing, ThreadingModel);
      end;

    constructor TComObjectFactory.Create(ComServer: TComServerObject;
      ComClass: TComClass; const ClassID: TGUID; const Name, Version, Description: string; Instancing: TClassInstancing;
      ThreadingModel: TThreadingModel);
    begin
{$ifdef DEBUG_COM}
         if printcom then 
        WriteLn('TComObjectFactory.Create');
{$endif}
        FRefCount := 1;
        FClassID := ClassID;
        FThreadingModel := ThreadingModel;
        FDescription := Description;
        FClassName := Name;
        FClassVersion := Version;
        FComServer := ComServer;
        FComClass := ComClass;
        FInstancing := Instancing;;
        ComClassManager.AddObjectFactory(Self);
        fIsRegistered := dword(-1);
      end;


    destructor TComObjectFactory.Destroy;
      begin
{$ifndef wince}
        if fIsRegistered <> dword(-1) then CoRevokeClassObject(fIsRegistered);
{$endif}
        ComClassManager.RemoveObjectFactory(Self);
      end;


    function TComObjectFactory.CreateComObject(const Controller: IUnknown
      ): TComObject;
      begin
{$ifdef DEBUG_COM}
         if printcom then 
        WriteLn('TComObjectFactory.CreateComObject');
{$endif}
        Result := TComClass(FComClass).Create();
      end;

    function TComObjectFactory.reg_flags():integer;inline;
    begin
       Result:=0;
       case Self.FInstancing of
       ciSingleInstance: Result:=Result or REGCLS_SINGLEUSE;
       ciMultiInstance: Result:=Result or REGCLS_MULTIPLEUSE;
       end;
       if FComServer.StartSuspended then
         Result:=Result or REGCLS_SUSPENDED;
    end;

    procedure TComObjectFactory.RegisterClassObject;
    begin
      {$ifdef DEBUG_COM}
         if printcom then 
        WriteLn('TComObjectFactory.RegisterClassObject');
      {$endif}
{$ifndef wince}
      if FInstancing <> ciInternal then
      OleCheck(CoRegisterClassObject(FClassID, Self, CLSCTX_LOCAL_SERVER,
         reg_flags(), @FIsRegistered));
{$else}
      RunError(217);
{$endif}
    end;


(* Copy from Sample.RGS (http://www.codeproject.com/KB/atl/RegistryMap.aspx)
HKCR
{
    %PROGID%.%VERSION% = s '%DESCRIPTION%'
    {
        CLSID = s '%CLSID%'
    }
    %PROGID% = s '%DESCRIPTION%'
    {
        CLSID = s '%CLSID%'
        CurVer = s '%PROGID%.%VERSION%'
    }
    NoRemove CLSID
    {
        ForceRemove %CLSID% = s '%DESCRIPTION%'
        {
            ProgID = s '%PROGID%.%VERSION%'
            VersionIndependentProgID = s '%PROGID%'
            ForceRemove 'Programmable'
            InprocServer32 = s '%MODULE%'
            {
                val ThreadingModel = s '%THREADING%'
            }
            'TypeLib' = s '%LIBID%'
        }
    }
}
*)

    procedure TComObjectFactory.UpdateRegistry(Register: Boolean);
      var
        classidguid: String;
        srv_type: string;

        function ThreadModelToString(model: TThreadingModel): String;
        begin
          case model of
            tmSingle: Result := '';
            tmApartment: Result := 'Apartment';
            tmFree: Result := 'Free';
            tmBoth: Result := 'Both';
            tmNeutral: Result := 'Neutral';
          end;
        end;

      begin
{$ifndef DUMMY_REG}
{$ifdef DEBUG_COM}
         if printcom then 
        WriteLn('UpdateRegistry begin');
{$endif}
        if Instancing = ciInternal then Exit;

        if System.ModuleIsLib then srv_type:='InprocServer32' else srv_type:='LocalServer32';

        if Register then
        begin
          classidguid := GUIDToString(ClassID);
          CreateRegKey('CLSID\' + classidguid + '\'+srv_type, '', FComServer.ServerFileName);
          //tmSingle, tmApartment, tmFree, tmBoth, tmNeutral
          CreateRegKey('CLSID\' + classidguid + '\'+srv_type, 'ThreadingModel', ThreadModelToString(ThreadingModel));
          CreateRegKey('CLSID\' + classidguid, '', Description);
          if ClassName <> '' then
          begin
            if ClassVersion <> '' then
            begin
              CreateRegKey('CLSID\' + classidguid + '\ProgID', '', ProgID + '.' + ClassVersion);
              CreateRegKey('CLSID\' + classidguid + '\VersionIndependentProgID', '', ProgID);
            end
            else
              CreateRegKey('CLSID\' + classidguid + '\ProgID', '', ProgID);

            CreateRegKey(ProgID, '', Description);
            CreateRegKey(ProgID + '\CLSID', '', GUIDToString(ClassID));
            if ClassVersion <> '' then
            begin
              CreateRegKey(ProgID + '\CurVer', '', ProgID + '.' + ClassVersion);
              CreateRegKey(ProgID + '.' + ClassVersion, '', Description);
              CreateRegKey(ProgID + '.' + ClassVersion + '\CLSID', '', GUIDToString(ClassID));
            end;
          end;
        end else
        begin
          classidguid := GUIDToString(ClassID);
          DeleteRegKey('CLSID\' + classidguid + '\'+srv_type);
          DeleteRegKey('CLSID\' + classidguid + '\VersionIndependentProgID');
          if ClassName <> '' then
          begin
            DeleteRegKey('CLSID\' + classidguid + '\ProgID');
            DeleteRegKey(ProgID + '\CLSID');
            if ClassVersion <> '' then
            begin
              DeleteRegKey(ProgID + '\CurVer');
              DeleteRegKey(ProgID + '.' + ClassVersion + '\CLSID');
              DeleteRegKey(ProgID + '.' + ClassVersion);
            end;
            DeleteRegKey(ProgID);
          end;
          DeleteRegKey('CLSID\' + classidguid);
        end;
{$ifdef DEBUG_COM}
         if printcom then 
        WriteLn('UpdateRegistry end');
{$endif}
{$endif DUMMY_REG}
      end;



    procedure DispatchInvoke(const Dispatch: IDispatch; CallDesc: PCallDesc;
      DispIDs: PDispIDList; Params: Pointer; Result: PVariant);

      var
        { we can't pass pascal ansistrings to COM routines so we've to convert them
          to/from widestring. This array contains the mapping to do so
        }
        StringMap : array[0..255] of record passtr : pansistring; paswstr : punicodestring; comstr : pwidechar; end;
        invokekind,
        i : longint;
        invokeresult : HResult;
        exceptioninfo : TExcepInfo;
        dispparams : TDispParams;
        NextString : SizeInt;
        Arguments : array[0..255] of TVarData;
        CurrType : byte;
        MethodID : TDispID;
      begin
        NextString:=0;
        fillchar(dispparams,sizeof(dispparams),0);
        try
{$ifdef DEBUG_COMDISPATCH}
         if printcom then 
          writeln('DispatchInvoke: Got ',CallDesc^.ArgCount,' arguments   NamedArgs = ',CallDesc^.NamedArgCount);
{$endif DEBUG_COMDISPATCH}
          { copy and prepare arguments }
          for i:=0 to CallDesc^.ArgCount-1 do
            begin
{$ifdef DEBUG_COMDISPATCH}
         if printcom then 
              writeln('DispatchInvoke: Params = ',hexstr(Params));
{$endif DEBUG_COMDISPATCH}
              { get plain type }
              CurrType:=CallDesc^.ArgTypes[i] and $7f;
              { a skipped parameter? Don't increment Params pointer if so. }
              if CurrType=varError then
                begin
                  Arguments[i].vType:=varError;
                  Arguments[i].vError:=DISP_E_PARAMNOTFOUND;
                  continue;
                end;
              { by reference? }
              if (CallDesc^.ArgTypes[i] and $80)<>0 then
                begin
                  case CurrType of
                    varStrArg:
                      begin
{$ifdef DEBUG_COMDISPATCH}
                        if printcom then 
                        writeln('Translating var ansistring argument ',PString(Params^)^);
{$endif DEBUG_COMDISPATCH}
                        StringMap[NextString].ComStr:=StringToOleStr(PString(Params^)^);
                        StringMap[NextString].PasStr:=PString(Params^);
                        StringMap[NextString].PasWStr:=Nil;
                        Arguments[i].VType:=varOleStr or varByRef;
                        Arguments[i].VPointer:=@StringMap[NextString].ComStr;
                        inc(NextString);
                        inc(PPointer(Params));
                      end;
                    varUStrArg:
                      begin
{$ifdef DEBUG_COMDISPATCH}
                        if printcom then
                        writeln('Translating var unicodestring argument ',PUnicodeString(Params^)^);
{$endif DEBUG_COMDISPATCH}
                        StringMap[NextString].ComStr:=StringToOleStr(PUnicodeString(Params^)^);
                        StringMap[NextString].PasStr:=Nil;
                        StringMap[NextString].PasWStr:=PUnicodeString(Params^);
                        Arguments[i].VType:=varOleStr or varByRef;
                        Arguments[i].VPointer:=@StringMap[NextString].ComStr;
                        inc(NextString);
                        inc(PPointer(Params));
                      end;
                    varVariant:
                      begin
{$ifdef DEBUG_COMDISPATCH}
                        if printcom then 
                        writeln('Got ref. variant containing type: ',PVarData(PPointer(Params)^)^.VType);
{$endif DEBUG_COMDISPATCH}
                        if PVarData(PPointer(Params)^)^.VType=varString then
                          begin
{$ifdef DEBUG_COMDISPATCH}
                            if printcom then   
                            writeln('  Casting nested varString: ',Ansistring(PVarData(Params^)^.vString));
{$endif DEBUG_COMDISPATCH}
                            VarCast(PVariant(Params^)^,PVariant(Params^)^,varOleStr);
                          end;

                        Arguments[i].VType:=varVariant or varByRef;
                        Arguments[i].VPointer:=PPointer(Params)^;
                        inc(PPointer(Params));
                      end
                    else
                      begin
{$ifdef DEBUG_COMDISPATCH}
                                 if printcom then 
                        write('DispatchInvoke: Got ref argument with type = ',CurrType);
                        case CurrType of
                          varOleStr:         if printcom then 
                            write(' Value = ',pwidestring(PPointer(Params)^)^);
                        end;
                        if printcom then 
                        writeln;
{$endif DEBUG_COMDISPATCH}
                        Arguments[i].VType:=CurrType or VarByRef;
                        Arguments[i].VPointer:=PPointer(Params)^;
                        inc(PPointer(Params));
                      end;
                  end
                end
              else   { by-value argument }
                case CurrType of
                  varStrArg:
                    begin
{$ifdef DEBUG_COMDISPATCH}
                    if printcom then 
                      writeln('Translating ansistring argument ',PString(Params)^);
{$endif DEBUG_COMDISPATCH}
                      StringMap[NextString].ComStr:=StringToOleStr(PString(Params)^);
                      StringMap[NextString].PasStr:=nil;
                      StringMap[NextString].PasWStr:=nil;
                      Arguments[i].VType:=varOleStr;
                      Arguments[i].VPointer:=StringMap[NextString].ComStr;
                      inc(NextString);
                      inc(PPointer(Params));
                    end;

                  varUStrArg:
                    begin
{$ifdef DEBUG_COMDISPATCH}
                    if printcom then
                      writeln('Translating unicodestring argument ',PUnicodeString(Params)^);
{$endif DEBUG_COMDISPATCH}
                      StringMap[NextString].ComStr:=StringToOleStr(PUnicodeString(Params)^);
                      StringMap[NextString].PasStr:=nil;
                      StringMap[NextString].PasWStr:=nil;
                      Arguments[i].VType:=varOleStr;
                      Arguments[i].VPointer:=StringMap[NextString].ComStr;
                      inc(NextString);
                      inc(PPointer(Params));
                    end;

                  varVariant:
                    begin
{$ifdef DEBUG_COMDISPATCH}
		   if printcom then 	
                      writeln('By-value Variant, making a copy');
{$endif DEBUG_COMDISPATCH}
                      { Codegen always passes a pointer to variant,
                       *unlike* Delphi which pushes the entire TVarData }
                      Arguments[i]:=PVarData(PPointer(Params)^)^;
                      Inc(PPointer(Params));
                    end;
                  varCurrency,
                  varDouble,
                  varInt64,
                  varQWord,
                  varDate:
                    begin
{$ifdef DEBUG_COMDISPATCH}
                      if printcom then 
                      writeln('Got 8 byte argument');
{$endif DEBUG_COMDISPATCH}
                      Arguments[i].VType:=CurrType;
                      Arguments[i].VDouble:=PDouble(Params)^;
                      inc(PDouble(Params));
                    end;
                  else
                    begin
{$ifdef DEBUG_COMDISPATCH}
                      if printcom then 
                      write('DispatchInvoke: Got argument with type ',CurrType);
                      case CurrType of
                        varOleStr:         if printcom then 
                          write(' Value = ',pwidestring(Params)^);
                        else
                          if printcom then 
                          write(' Value = ',hexstr(PtrInt(PPointer(Params)^),SizeOf(Pointer)*2));
                      end;
                      writeln;
{$endif DEBUG_COMDISPATCH}
                      Arguments[i].VType:=CurrType;
                      Arguments[i].VPointer:=PPointer(Params)^;
                      inc(PPointer(Params));
                    end;
                end;
            end;

          { finally prepare the call }
          with DispParams do
            begin
              rgvarg:=@Arguments;
              cNamedArgs:=CallDesc^.NamedArgCount;
              if cNamedArgs=0 then
                rgdispidNamedArgs:=nil
              else
                rgdispidNamedArgs:=@DispIDs^[1];
              cArgs:=CallDesc^.ArgCount;
            end;
          InvokeKind:=CallDesc^.CallType;
          MethodID:=DispIDs^[0];
          case InvokeKind of
            DISPATCH_PROPERTYPUT:
              begin
                if ((Arguments[0].VType and varTypeMask) in [varDispatch]) or
                    { if we have a variant that's passed as a reference we pass it
                      to the property as a reference as well }
                    (
                      ((Arguments[0].VType and varTypeMask) in [varVariant]) and
                      ((CallDesc^.argtypes[0] and $80) <> 0)
                    ) then
                  InvokeKind:=DISPATCH_PROPERTYPUTREF;
                { first name is actually the name of the property to set }
                DispIDs^[0]:=DISPID_PROPERTYPUT;
                DispParams.rgdispidNamedArgs:=@DispIDs^[0];
                inc(DispParams.cNamedArgs);
              end;
            DISPATCH_METHOD:
              { It appears that certain COM servers expect both DISPATCH_METHOD and DISPATCH_PROPERTYGET
                flags for anything returning a result, see bug #24352 }
              if assigned(Result) then
                InvokeKind:=DISPATCH_METHOD or DISPATCH_PROPERTYGET;
          end;
{$ifdef DEBUG_COMDISPATCH}
         if printcom then 
          writeln('DispatchInvoke: MethodID: ',MethodID,' InvokeKind: ',InvokeKind);
{$endif DEBUG_COMDISPATCH}
          { do the call and check the result }
          invokeresult:=Dispatch.Invoke(MethodID,GUID_NULL,0,InvokeKind,DispParams,result,@exceptioninfo,nil);
          if invokeresult<>0 then
            DispatchInvokeError(invokeresult,exceptioninfo);

          { translate strings back }
          for i:=0 to NextString-1 do begin
            if assigned(StringMap[i].passtr) then
              OleStrToStrVar(StringMap[i].comstr,StringMap[i].passtr^)
            else if assigned(StringMap[i].paswstr) then
              OleStrToStrVar(StringMap[i].comstr,StringMap[i].paswstr^);
          end;
        finally
          for i:=0 to NextString-1 do
            SysFreeString(StringMap[i].ComStr);
        end;
      end;


    procedure SearchIDs(const DispatchInterface : IDispatch; Names: PChar;
      Count: Integer; IDs: PDispIDList);
      var
      	res : HRESULT;
      	NamesArray : ^PWideChar;
      	NamesData : PWideChar;
      	OrigNames : PChar;
        NameCount,
      	NameLen,
      	NewNameLen,
        CurrentNameDataUsed,
      	CurrentNameDataSize : SizeInt;
      	i : longint;
      begin
      	getmem(NamesArray,Count*sizeof(PWideChar));
      	CurrentNameDataSize:=256;
      	CurrentNameDataUsed:=0;
      	getmem(NamesData,CurrentNameDataSize);
        NameCount:=0;
   	    OrigNames:=Names;
{$ifdef DEBUG_COMDISPATCH} 
                if printcom then 
        writeln('SearchIDs: Searching ',Count,' IDs');
{$endif DEBUG_COMDISPATCH}
      	for i:=1 to Count do
      	  begin
       	    NameLen:=strlen(Names);
{$ifdef DEBUG_COMDISPATCH}
                     if printcom then 
            writeln('SearchIDs: Original name: ',Names,' Len: ',NameLen);
{$endif DEBUG_COMDISPATCH}
      	    NewNameLen:=MultiByteToWideChar(0,0,Names,NameLen,nil,0)+1;
      	    if (CurrentNameDataUsed+NewNameLen)*2>CurrentNameDataSize then
      	      begin
      	      	inc(CurrentNameDataSize,256);
      	        reallocmem(NamesData,CurrentNameDataSize);
      	      end;
      	    NamesArray[i-1]:=@NamesData[CurrentNameDataUsed];
      	    MultiByteToWideChar(0,0,Names,NameLen,@NamesData[CurrentNameDataUsed],NewNameLen);
      	    NamesData[CurrentNameDataUsed+NewNameLen-1]:=#0;
{$ifdef DEBUG_COMDISPATCH}
                   if printcom then 
            writeln('SearchIDs: Translated name: ',WideString(PWideChar(@NamesData[CurrentNameDataUsed])));
{$endif DEBUG_COMDISPATCH}
      	    inc(CurrentNameDataUsed,NewNameLen);
      	    inc(Names,NameLen+1);
            inc(NameCount);
      	  end;
      	res:=DispatchInterface.GetIDsOfNames(GUID_NULL,NamesArray,NameCount,
{$ifdef wince}
		     LOCALE_SYSTEM_DEFAULT
{$else wince}
         GetThreadLocale
{$endif wince}
         ,IDs);
{$ifdef DEBUG_COMDISPATCH}
                 if printcom then 
        writeln('SearchIDs: GetIDsOfNames result = ',hexstr(res,SizeOf(HRESULT)*2));
        for i:=0 to Count-1 do
          writeln('SearchIDs: ID[',i,'] = ',ids^[i]);
{$endif DEBUG_COMDISPATCH}
      	if res=DISP_E_UNKNOWNNAME then
      	  raise EOleError.createresfmt(@snomethod,[OrigNames])
      	else
      	  OleCheck(res);
      	freemem(NamesArray);
      	freemem(NamesData);
      end;


    procedure ComObjDispatchInvoke(dest : PVariant;const source : Variant;
        calldesc : pcalldesc;params : pointer);cdecl;
      var
      	dispatchinterface : pointer;
      	ids : array[0..255] of TDispID;
      begin
        fillchar(ids,sizeof(ids),0);
{$ifdef DEBUG_COMDISPATCH}
         if printcom then 
        writeln('ComObjDispatchInvoke called');
         if printcom then 
        writeln('ComObjDispatchInvoke: @CallDesc = $',hexstr(PtrInt(CallDesc),SizeOf(Pointer)*2),' CallDesc^.ArgCount = ',CallDesc^.ArgCount);
{$endif DEBUG_COMDISPATCH}
      	if tvardata(source).vtype=VarDispatch then
      	  dispatchinterface:=tvardata(source).vdispatch
      	else if tvardata(source).vtype=(VarDispatch or VarByRef) then
      	  dispatchinterface:=pvardata(tvardata(source).vpointer)^.vdispatch
      	else
      	  raise eoleerror.createres(@SVarNotObject);
      	SearchIDs(IDispatch(dispatchinterface),@CallDesc^.ArgTypes[CallDesc^.ArgCount],
          CallDesc^.NamedArgCount+1,@ids);
      	if assigned(dest) then
      	  VarClear(dest^);
      	DispatchInvoke(IDispatch(dispatchinterface),calldesc,@ids,params,dest);
      end;


{ $define DEBUG_DISPATCH}
    procedure DoDispCallByID(res : Pointer; const disp : IDispatch;desc : PDispDesc; params : Pointer);
      var
        exceptioninfo : TExcepInfo;
        dispparams : TDispParams;
        flags : WORD;
        invokeresult : HRESULT;
        preallocateddata : array[0..15] of TVarData;
        Arguments : PVarData;
        CurrType, i : byte;
        dispidNamed: TDispID;
      begin
        { use preallocated space, i.e. can we avoid a getmem call? }
        if desc^.calldesc.argcount<=Length(preallocateddata) then
          Arguments:=@preallocateddata
        else
          GetMem(Arguments,desc^.calldesc.argcount*sizeof(TVarData));

        { prepare parameters }
        if desc^.CallDesc.ArgCount > 0 then
          for i:=0 to desc^.CallDesc.ArgCount-1 do
            begin
  {$ifdef DEBUG_DISPATCH}
              writeln('DoDispCallByID: Params = ',hexstr(Params));
  {$endif DEBUG_DISPATCH}
              { get plain type }
              CurrType:=desc^.CallDesc.ArgTypes[i] and $3f;
              { by reference? }
              if (desc^.CallDesc.ArgTypes[i] and $80)<>0 then
                begin
  {$ifdef DEBUG_DISPATCH}
                  write('DispatchInvoke: Got ref argument with type = ',CurrType);
                  writeln;
  {$endif DEBUG_DISPATCH}
                  Arguments[i].VType:=CurrType or VarByRef;
                  Arguments[i].VPointer:=PPointer(Params)^;
                  inc(PPointer(Params));
                end
              else
                begin
  {$ifdef DEBUG_DISPATCH}
                  writeln('DispatchInvoke: Got value argument with type = ',CurrType);
  {$endif DEBUG_DISPATCH}
                  case CurrType of
                    varVariant:
                      begin
                       { Codegen always passes a pointer to variant,
                         *unlike* Delphi which pushes the entire TVarData }
                        Arguments[i]:=PVarData(PPointer(Params)^)^;
                        inc(PPointer(Params));
                      end;
                    varCurrency,
                    varDouble,
                    varInt64,
                    varQWord,
                    varDate:
                      begin
  {$ifdef DEBUG_DISPATCH}
                        writeln('DispatchInvoke: Got 8 byte argument');
  {$endif DEBUG_DISPATCH}
                        Arguments[i].VType:=CurrType;
                        Arguments[i].VDouble:=PDouble(Params)^;
                        inc(PDouble(Params));
                      end;
                  else
                    begin
  {$ifdef DEBUG_DISPATCH}
                      writeln('DispatchInvoke: Got argument with type ',CurrType);
  {$endif DEBUG_DISPATCH}
                      Arguments[i].VType:=CurrType;
                      Arguments[i].VPointer:=PPointer(Params)^;
                      inc(PPointer(Params));
                    end;
                end;
              end;
            end;
        dispparams.cArgs:=desc^.calldesc.argcount;
        dispparams.rgvarg:=pointer(Arguments);
        dispparams.cNamedArgs:=desc^.calldesc.namedargcount;
        dispparams.rgdispidNamedArgs:=@desc^.CallDesc.ArgTypes[desc^.CallDesc.ArgCount];
        flags:=desc^.calldesc.calltype;

        case flags of
          DISPATCH_PROPERTYPUT:
            begin
              inc(dispparams.cNamedArgs);
              if (Arguments[0].VType and varTypeMask) = varDispatch then
                flags:=DISPATCH_PROPERTYPUTREF;
              dispidNamed:=DISPID_PROPERTYPUT;
              DispParams.rgdispidNamedArgs:=@dispidNamed;
            end;
          DISPATCH_METHOD:
            { It appears that certain COM servers expect both DISPATCH_METHOD and DISPATCH_PROPERTYGET
              flags for anything returning a result, see bug #24352 }
            if assigned(res) then
              flags:=DISPATCH_METHOD or DISPATCH_PROPERTYGET;
        end;

        invokeresult:=disp.Invoke(
                desc^.DispId, { DispID: LongInt; }
                GUID_NULL, { const iid : TGUID; }
                0, { LocaleID : longint; }
                flags, { Flags: Word; }
                dispparams, { var params; }
                res,@exceptioninfo,nil { VarResult,ExcepInfo,ArgErr : pointer) }
          );
        if invokeresult<>0 then
          DispatchInvokeError(invokeresult,exceptioninfo);
        if desc^.calldesc.argcount>Length(preallocateddata) then
          FreeMem(Arguments);
      end;

    { TTypedComObject }

    function TTypedComObject.GetClassInfo(out pptti: ITypeInfo): HResult;stdcall;
      begin
        Result:=S_OK;
        pptti:=TTypedComObjectFactory(factory).classinfo;
      end;


    { TTypedComObjectFactory }

    constructor TTypedComObjectFactory.Create(AComServer: TComServerObject; TypedComClass: TTypedComClass; const AClassID: TGUID;
      AInstancing: TClassInstancing; AThreadingModel: TThreadingModel = tmSingle);
      var
        TypedName, TypedDescription, TypedVersion: WideString;
        ppTypeAttr: lpTYPEATTR;
      begin
        //TDB get name and description from typelib (check if this is a valid guid)
        OleCheck(AComServer.GetTypeLib.GetTypeInfoOfGuid(AClassID, FClassInfo));

        //bug FPC 0010569 - http://msdn2.microsoft.com/en-us/library/ms221396(VS.85).aspx
        OleCheck(FClassInfo.GetDocumentation(-1, @TypedName, @TypedDescription, nil, nil));
        FClassInfo.GetTypeAttr(ppTypeAttr);
        try
          FTypeInfoCount := ppTypeAttr^.cImplTypes;
          TypedVersion := '';
          if (ppTypeAttr^.wMajorVerNum <> 0) or (ppTypeAttr^.wMinorVerNum <> 0) then
          begin
            TypedVersion := IntToStr(ppTypeAttr^.wMajorVerNum);
            if ppTypeAttr^.wMinorVerNum <> 0 then
              TypedVersion := TypedVersion + '.' + IntToStr(ppTypeAttr^.wMinorVerNum)
          end;
        finally
          FClassInfo.ReleaseTypeAttr(ppTypeAttr);
        end;

        inherited Create(AComServer, TypedComClass, AClassID, TypedName, TypedVersion, TypedDescription, AInstancing, AThreadingModel);
      end;


    function TTypedComObjectFactory.GetInterfaceTypeInfo(TypeFlags: Integer): ITypeInfo;
    var
      index, ImplTypeFlags: Integer;
      RefType: HRefType;
    begin
      Result := nil;
      for index := 0 to FTypeInfoCount - 1 do
      begin
        OleCheck(ClassInfo.GetImplTypeFlags(index, ImplTypeFlags));
        if ImplTypeFlags = TypeFlags then
        begin
          OleCheck(ClassInfo.GetRefTypeOfImplType(index, RefType));
          OleCheck(ClassInfo.GetRefTypeInfo(RefType, Result));
          break;
        end;
      end;
    end;


    procedure TTypedComObjectFactory.UpdateRegistry(Register: Boolean);
         var
        ptla: PTLibAttr;
      begin
        if Instancing = ciInternal then
          Exit;

        if Register then
        begin
          inherited UpdateRegistry(Register);

          //http://www.experts-exchange.com/Programming/Misc/Q_20634807.html
          //There seems to also be Version according to Process Monitor
          //http://technet.microsoft.com/en-us/sysinternals/bb896645.aspx
          if FComServer.TypeLib = nil then
            raise Exception.Create('TypeLib is not set!');

          OleCheck(FComServer.TypeLib.GetLibAttr(ptla));
          try
            CreateRegKey('CLSID\' + GUIDToString(ClassID) + '\TypeLib', '', GUIDToString(ptla^.GUID));
          finally
            FComServer.TypeLib.ReleaseTLibAttr(ptla);
          end;
        end else
        begin
          DeleteRegKey('CLSID\' + GUIDToString(ClassID) + '\TypeLib');
          inherited UpdateRegistry(Register);
        end;
      end;

   { TAutoIntfObject }

    function TAutoIntfObject.GetTypeInfoCount(out count: longint): HResult; stdcall;
      begin
{$ifdef DEBUG_COM}
                if printcom then 
        WriteLn('TAutoIntfObject.GetTypeInfoCount');
{$endif}
        count := 1;
        Result := S_OK;
      end;

    function TAutoIntfObject.GetTypeInfo(Index, LocaleID: longint; out TypeInfo
      ): HResult; stdcall;
      begin
{$ifdef DEBUG_COM}
         if printcom then 
        WriteLn('TAutoIntfObject.GetTypeInfo: ', Index);
{$endif}
        if Index <> 0 then
          Result := DISP_E_BADINDEX
        else
        begin
          ITypeInfo(TypeInfo) := fTypeInfo;
          Result := S_OK;
        end;
      end;

    function TAutoIntfObject.GetIDsOfNames(const iid: TGUID; names: Pointer;
      NameCount, LocaleID: LongInt; DispIDs: Pointer): HResult; stdcall;
      begin
{$ifdef DEBUG_COM}
         if printcom then 
        WriteLn('TAutoIntfObject.GetIDsOfNames: ', GUIDToString(iid));
{$endif}
        //return typeinfo->GetIDsOfNames(names, n, dispids);
        Result := fTypeInfo.GetIDsOfNames(names, NameCount, lpDISPID(DispIDs)^);
      end;

    function TAutoIntfObject.Invoke(DispID: LongInt; const iid: TGUID;
      LocaleID: longint; Flags: Word; var params; VarResult, ExcepInfo,
      ArgErr: pointer): HResult; stdcall;
      begin
{$ifdef DEBUG_COM}
         if printcom then 
        WriteLn('TAutoIntfObject.Invoke: ', DispID, ': ', Flags, ': ', TDispParams(params).cArgs, ': ', GUIDToString(iid));
        //WriteLn('TAutoIntfObject.Invoke: ', DispID, ': ', Flags, ': ', TDispParams(params).cArgs, ': ', TDispParams(params).rgvarg^, ': ', GUIDToString(iid));
{$endif}
        if not IsEqualGUID(iid, GUID_NULL) then
          Result := DISP_E_UNKNOWNINTERFACE
        else
      //  Function  Invoke(pvInstance: Pointer; memid: MEMBERID; wFlags: WORD; VAR pDispParams: DISPPARAMS; OUT pVarResult: VARIANT; OUT pExcepInfo: EXCEPINFO; OUT puArgErr: UINT):HResult;StdCall;
      //  Result := fTypeInfo.Invoke(IDispatch(Self), DispID, Flags, TDispParams(params), PVariant(VarResult)^, PExcepInfo(ExcepInfo)^, PUINT(ArgErr)^);
          Result := fTypeInfo.Invoke(fInterfacePointer, DispID, Flags, TDispParams(params), VarResult, ExcepInfo, ArgErr);
      end;

    function TAutoIntfObject.InterfaceSupportsErrorInfo(const riid: TIID): HResult;
      StdCall;
      begin
{$ifdef DEBUG_COM}
         if printcom then 
        WriteLn('TAutoIntfObject.InterfaceSupportsErrorInfo: ', GUIDToString(riid));
{$endif}
        if assigned(GetInterfaceEntry(riid)) then
          Result:=S_OK
        else
          Result:=S_FALSE;
      end;

    function TAutoIntfObject.SafeCallException(ExceptObject: TObject;
      ExceptAddr: Pointer): HResult;
      var
        //Message: string;
        Handled: Integer;
      begin
{$ifdef DEBUG_COM}
         if printcom then 
        WriteLn('TAutoIntfObject.SafeCallException');
{$endif}
        Handled:=0;
        Result:=0;
        //TODO: DO WE NEED THIS ?
        //if assigned(ServerExceptionHandler) then
        //  begin
        //    if ExceptObject is Exception then
        //      Message:=Exception(ExceptObject).Message;
        //
        //    ServerExceptionHandler.OnException(ClassName,ExceptObject.ClassName,
        //      Message,PtrInt(ExceptAddr),WideString(GUIDToString(FFactory.ErrorIID)),
        //      FFactory.ProgID,Handled,Result);
        //  end;
        if Handled=0 then
          Result:=HandleSafeCallException(ExceptObject,ExceptAddr,StringToGuid('{7C538328-8A75-4EC4-A02E-FB3B27FAA411}'),
            '','');
      end;

    constructor TAutoIntfObject.Create(TypeLib: ITypeLib; const Guid: TGuid);
      begin
{$ifdef DEBUG_COM}
         if printcom then 
        WriteLn('TAutoIntfObject.Create: ', GUIDToString(Guid));
{$endif}
        OleCheck(TypeLib.GetTypeInfoOfGuid(Guid, fTypeInfo));
        OleCheck(QueryInterface(Guid, fInterfacePointer));
      end;

    { TAutoObject }

    function TAutoObject.GetTypeInfoCount(out count: longint): HResult; stdcall;
      begin
{$ifdef DEBUG_COM}
         if printcom then 
        WriteLn('TAutoObject.GetTypeInfoCount');
{$endif}
        count := 1;
        Result := S_OK;
      end;

    function TAutoObject.GetTypeInfo(Index, LocaleID: longint; out TypeInfo
      ): HResult; stdcall;
      begin
{$ifdef DEBUG_COM}
         if printcom then 
        WriteLn('TAutoIntfObject.GetTypeInfo: ', Index);
{$endif}
        if Index <> 0 then
          Result := DISP_E_BADINDEX
        else
        begin
          ITypeInfo(TypeInfo) := TAutoObjectFactory(Factory).ClassInfo;
          Result := S_OK;
        end;
      end;

    function TAutoObject.GetIDsOfNames(const iid: TGUID; names: Pointer; NameCount,
      LocaleID: LongInt; DispIDs: Pointer): HResult; stdcall;
      begin
{$ifdef DEBUG_COM}
         if printcom then 
        WriteLn('TAutoIntfObject.GetIDsOfNames: ', GUIDToString(iid));
{$endif}
        //return typeinfo->GetIDsOfNames(names, n, dispids);
        Result := TAutoObjectFactory(Factory).ClassInfo.GetIDsOfNames(names, NameCount, lpDISPID(DispIDs)^);
      end;

    function TAutoObject.Invoke(DispID: LongInt; const iid: TGUID;
      LocaleID: longint; Flags: Word; var params; VarResult, ExcepInfo,
      ArgErr: pointer): HResult; stdcall;
      begin
{$ifdef DEBUG_COM}
         if printcom then 
        WriteLn('TAutoIntfObject.Invoke: ', DispID, ': ', Flags, ': ', TDispParams(params).cArgs, ': ', GUIDToString(iid));
        //WriteLn('TAutoIntfObject.Invoke: ', DispID, ': ', Flags, ': ', TDispParams(params).cArgs, ': ', TDispParams(params).rgvarg^, ': ', GUIDToString(iid));
{$endif}
        if not IsEqualGUID(iid, GUID_NULL) then
          Result := DISP_E_UNKNOWNINTERFACE
        else
        begin
          Result := TAutoObjectFactory(Factory).DispTypeInfo.Invoke(Pointer(
            PtrUint(Self) + TAutoObjectFactory(Factory).DispIntfEntry^.IOffset),
            DispID, Flags, TDispParams(Params), VarResult, ExcepInfo, ArgErr);
        end;
      end;

    { TAutoObjectFactory }

    constructor TAutoObjectFactory.Create(AComServer: TComServerObject;
      AutoClass: TAutoClass; const AClassID: TGUID; AInstancing: TClassInstancing;
      AThreadingModel: TThreadingModel);
         var
           ppTypeAttr: lpTYPEATTR;
      begin
        inherited Create(AComServer, AutoClass, AClassID, AInstancing, AThreadingModel);
        FDispTypeInfo := GetInterfaceTypeInfo(IMPLTYPEFLAG_FDEFAULT);
        OleCheck(FDispTypeInfo.GetTypeAttr(ppTypeAttr));
        try
          FDispIntfEntry := GetIntfEntry(ppTypeAttr^.guid);
        finally
          FDispTypeInfo.ReleaseTypeAttr(ppTypeAttr);
        end;
      end;

    function TAutoObjectFactory.GetIntfEntry(Guid: TGUID): PInterfaceEntry;
    begin
      Result := FComClass.GetInterfaceEntry(Guid);
    end;


    procedure TOleStream.Check(err:integer);
      begin
        OleCheck(err);
      end;

const
  Initialized : boolean = false;
var
  Ole32Dll : HModule;
  SaveInitProc : CodePointer;

procedure InitComObj;
begin
  if SaveInitProc<>nil then
    TProcedure(SaveInitProc)();
  if not CoInitDisable then
{$ifndef wince}
    if (CoInitFlags=-1) or not(assigned(ComObj.CoInitializeEx)) then
      Initialized:=Succeeded(CoInitialize(nil))
    else
{$endif wince}
      Initialized:=Succeeded(ComObj.CoInitializeEx(nil, CoInitFlags));
end;

initialization
  Uninitializing:=false;
  _ComClassManager:=nil;
  Ole32Dll:=GetModuleHandle('ole32.dll');
  if Ole32Dll<>0 then
    begin
      Pointer(CoCreateInstanceEx):=GetProcAddress(Ole32Dll,'CoCreateInstanceEx');
      Pointer(CoInitializeEx):=GetProcAddress(Ole32Dll,'CoInitializeEx');
      Pointer(CoAddRefServerProcess):=GetProcAddress(Ole32Dll,'CoAddRefServerProcess');
      Pointer(CoReleaseServerProcess):=GetProcAddress(Ole32Dll,'CoReleaseServerProcess');
      Pointer(CoResumeClassObjects):=GetProcAddress(Ole32Dll,'CoResumeClassObjects');
      Pointer(CoSuspendClassObjects):=GetProcAddress(Ole32Dll,'CoSuspendClassObjects');
    end;

  if not(IsLibrary) then
    begin
      SaveInitProc:=InitProc;
      InitProc:=@InitComObj;
    end;

  SafeCallErrorProc:=@SafeCallErrorHandler;
  VarDispProc:=@ComObjDispatchInvoke;
  DispCallByIDProc:=@DoDispCallByID;
finalization
  Uninitializing:=true;
  _ComClassManager.Free;
  VarDispProc:=nil;
  SafeCallErrorProc:=nil;
  if Initialized then
    CoUninitialize;
end.
