{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2005 by the Free Pascal development team

    This unit makes Free Pascal as much as possible Delphi compatible,
    defining several internal structures for classes, interfaces, and
    resource strings.

    Additionally this file defines the interface of TObject, providing
    their basic implementation in the corresponding objpas.inc file.

    WARNING: IF YOU CHANGE SOME OF THESE INTERNAL RECORDS, MAKE SURE
    TO MODIFY THE COMPILER AND OBJPAS.INC ACCORDINGLY, OTHERWISE
    THIS WILL LEAD TO CRASHES IN THE RESULTING COMPILER AND/OR RTL.

    IN PARTICULAR, THE IMPLEMENTATION PART OF THIS INCLUDE FILE,
    OBJPAS.INC, USES SOME HARDCODED RECORD MEMBER OFFSETS.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{*****************************************************************************
                            Basic Types/constants
*****************************************************************************}

    const
       vmtInstanceSize         = 0;
       vmtParent               = sizeof(SizeInt)*2;
       { These were negative value's, but are now positive, else classes
         couldn't be used with shared linking which copies only all data from
         the .global directive and not the data before the directive (PFV) }
       vmtClassName            = vmtParent+sizeof(pointer);
       vmtDynamicTable         = vmtParent+sizeof(pointer)*2;
       vmtMethodTable          = vmtParent+sizeof(pointer)*3;
       vmtFieldTable           = vmtParent+sizeof(pointer)*4;
       vmtTypeInfo             = vmtParent+sizeof(pointer)*5;
       vmtInitTable            = vmtParent+sizeof(pointer)*6;
       vmtAutoTable            = vmtParent+sizeof(pointer)*7;
       vmtIntfTable            = vmtParent+sizeof(pointer)*8;
       vmtMsgStrPtr            = vmtParent+sizeof(pointer)*9;
       { methods }
       vmtMethodStart          = vmtParent+sizeof(pointer)*10;
       vmtDestroy              = vmtMethodStart;
<<<<<<< HEAD
       vmtNewInstance          = vmtMethodStart+sizeof(codepointer);
       vmtFreeInstance         = vmtMethodStart+sizeof(codepointer)*2;
       vmtSafeCallException    = vmtMethodStart+sizeof(codepointer)*3;
       vmtDefaultHandler       = vmtMethodStart+sizeof(codepointer)*4;
       vmtAfterConstruction    = vmtMethodStart+sizeof(codepointer)*5;
       vmtBeforeDestruction    = vmtMethodStart+sizeof(codepointer)*6;
       vmtDefaultHandlerStr    = vmtMethodStart+sizeof(codepointer)*7;
       vmtDispatch             = vmtMethodStart+sizeof(codepointer)*8;
       vmtDispatchStr          = vmtMethodStart+sizeof(codepointer)*9;
       vmtEquals               = vmtMethodStart+sizeof(codepointer)*10;
       vmtGetHashCode          = vmtMethodStart+sizeof(codepointer)*11;
       vmtToString             = vmtMethodStart+sizeof(codepointer)*12;
=======
       vmtNewInstance          = vmtMethodStart+sizeof(pointer);
       vmtFreeInstance         = vmtMethodStart+sizeof(pointer)*2;
       vmtSafeCallException    = vmtMethodStart+sizeof(pointer)*3;
       vmtDefaultHandler       = vmtMethodStart+sizeof(pointer)*4;
       vmtAfterConstruction    = vmtMethodStart+sizeof(pointer)*5;
       vmtBeforeDestruction    = vmtMethodStart+sizeof(pointer)*6;
       vmtDefaultHandlerStr    = vmtMethodStart+sizeof(pointer)*7;
       vmtDispatch             = vmtMethodStart+sizeof(pointer)*8;
       vmtDispatchStr          = vmtMethodStart+sizeof(pointer)*9;
       vmtEquals               = vmtMethodStart+sizeof(pointer)*10;
       vmtGetHashCode          = vmtMethodStart+sizeof(pointer)*11;
       vmtToString             = vmtMethodStart+sizeof(pointer)*12;
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

       { IInterface }
       S_OK          = 0;
       S_FALSE       = 1;
       E_NOINTERFACE = hresult($80004002);
       E_UNEXPECTED  = hresult($8000FFFF);
       E_NOTIMPL     = hresult($80004001);

     type
       TextFile = Text;

       { now the let's declare the base classes for the class object
         model. The compiler expects TObject and IUnknown to be defined
         first as forward classes }
       TObject = class;
       IUnknown = interface;

       TClass  = class of tobject;
       PClass  = ^tclass;


       { to access the message table from outside }
       TMsgStrTable = record
          name   : pshortstring;
          method : codepointer;
       end;

       PMsgStrTable = ^TMsgStrTable;

       TStringMessageTable = record
          count : longint;
          msgstrtable : array[0..0] of tmsgstrtable;
       end;

       pstringmessagetable = ^tstringmessagetable;
       pinterfacetable = ^tinterfacetable;

       PVmt = ^TVmt;
       TVmt = record
         vInstanceSize: SizeInt;
         vInstanceSize2: SizeInt;
         vParent: PVmt;
         vClassName: PShortString;
         vDynamicTable: Pointer;
         vMethodTable: Pointer;
         vFieldTable: Pointer;
         vTypeInfo: Pointer;
         vInitTable: Pointer;
         vAutoTable: Pointer;
         vIntfTable: PInterfaceTable;
         vMsgStrPtr: pstringmessagetable;
<<<<<<< HEAD
         vDestroy: CodePointer;
         vNewInstance: CodePointer;
         vFreeInstance: CodePointer;
         vSafeCallException: CodePointer;
         vDefaultHandler: CodePointer;
         vAfterConstruction: CodePointer;
         vBeforeDestruction: CodePointer;
         vDefaultHandlerStr: CodePointer;
         vDispatch: CodePointer;
         vDispatchStr: CodePointer;
         vEquals: CodePointer;
         vGetHashCode: CodePointer;
         vToString: CodePointer;
=======
         vDestroy: Pointer;
         vNewInstance: Pointer;
         vFreeInstance: Pointer;
         vSafeCallException: Pointer;
         vDefaultHandler: Pointer;
         vAfterConstruction: Pointer;
         vBeforeDestruction: Pointer;
         vDefaultHandlerStr: Pointer;
         vDispatch: Pointer;
         vDispatchStr: Pointer;
         vEquals: Pointer;
         vGetHashCode: Pointer;
         vToString: Pointer;
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
       end;

       PGuid = ^TGuid;
       TGuid = packed record
          case integer of
             1 : (
                  Data1 : DWord;
                  Data2 : word;
                  Data3 : word;
                  Data4 : array[0..7] of byte;
                 );
             2 : (
                  D1 : DWord;
                  D2 : word;
                  D3 : word;
                  D4 : array[0..7] of byte;
                 );
             3 : ( { uuid fields according to RFC4122 }
                  time_low : dword;			// The low field of the timestamp
                  time_mid : word;                      // The middle field of the timestamp
                  time_hi_and_version : word;           // The high field of the timestamp multiplexed with the version number
                  clock_seq_hi_and_reserved : byte;     // The high field of the clock sequence multiplexed with the variant
                  clock_seq_low : byte;                 // The low field of the clock sequence
                  node : array[0..5] of byte;           // The spatially unique node identifier
                 );
       end;

       // This enumerate is found both in the rtl and compiler. Do not change the order of the fields.
       tinterfaceentrytype = (etStandard,
         etVirtualMethodResult,
         etStaticMethodResult,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
         etFieldValue,
         etVirtualMethodClass,
         etStaticMethodClass,
         etFieldValueClass
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
         etFieldValue, 
         etVirtualMethodClass,
         etStaticMethodClass,
         etFieldValueClass           
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
       );

       pinterfaceentry = ^tinterfaceentry;
       tinterfaceentry = record
         IID         : pguid; { if assigned(IID) then Com else Corba}
         VTable      : Pointer;
         IOffset     : sizeuint;
         IIDStr      : pshortstring; { never nil. Com: upper(GuidToString(IID^)) }
{$ifdef VER2_6}
         case boolean of
{$ifdef ENDIAN_BIG}
           true      : ({$IFDEF CPU64}__pad: longint;{$ENDIF}IType : tinterfaceentrytype);
{$else ENDIAN_BIG}
           true      : (IType       : tinterfaceentrytype);
{$endif ENDIAN_BIG}
<<<<<<< HEAD
<<<<<<< HEAD
           false     : (__pad_dummy : sizeint);
{$else  VER2_6}
         IType       : tinterfaceentrytype;
{$endif VER2_6}
=======
=======
>>>>>>> origin/fixes_2_2
           false     : (__pad_dummy : pointer);
>>>>>>> graemeg/fixes_2_2
       end;

       tinterfacetable = record
         EntryCount : sizeuint;
         Entries    : array[0..0] of tinterfaceentry;
       end;

       TMethod = record
         Code : CodePointer;
         Data : Pointer;
       end;

       TObject = class
       public
          { please don't change the order of virtual methods, because
            their vmt offsets are used by some assembler code which uses
            hard coded addresses      (FK)                                 }
          constructor Create;
          { the virtual procedures must be in THAT order }
          destructor Destroy;virtual;
          class function newinstance : tobject;virtual;
          procedure FreeInstance;virtual;
          function SafeCallException(exceptobject : tobject;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
            exceptaddr : codepointer) : HResult;virtual;
=======
            exceptaddr : pointer) : HResult;virtual;
>>>>>>> graemeg/cpstrnew
=======
            exceptaddr : pointer) : HResult;virtual;
>>>>>>> graemeg/cpstrnew
=======
            exceptaddr : pointer) : HResult;virtual;
>>>>>>> graemeg/cpstrnew
=======
            exceptaddr : pointer) : HResult;virtual;
>>>>>>> origin/cpstrnew
          procedure DefaultHandler(var message);virtual;

          procedure Free;
          class function InitInstance(instance : pointer) : tobject; {$ifdef SYSTEMINLINE} inline; {$endif}
          procedure CleanupInstance;
          class function ClassType : tclass;{$ifdef SYSTEMINLINE}inline;{$endif}
          class function ClassInfo : pointer;
          class function ClassName : shortstring;
          class function ClassNameIs(const name : string) : boolean;
          class function ClassParent : tclass;{$ifdef SYSTEMINLINE}inline;{$endif}
          class function InstanceSize : SizeInt;{$ifdef SYSTEMINLINE}inline;{$endif}
          class function InheritsFrom(aclass : tclass) : boolean;
          class function StringMessageTable : pstringmessagetable;

          class function MethodAddress(const name : shortstring) : codepointer;
          class function MethodName(address : codepointer) : shortstring;
          function FieldAddress(const name : shortstring) : pointer;

          { new since Delphi 4 }
          procedure AfterConstruction;virtual;
          procedure BeforeDestruction;virtual;

          { new for gtk, default handler for text based messages }
          procedure DefaultHandlerStr(var message);virtual;

          { message handling routines }
          procedure Dispatch(var message);virtual;
          procedure DispatchStr(var message);virtual;

          { interface functions }
          function GetInterface(const iid : tguid; out obj) : boolean;
<<<<<<< HEAD
<<<<<<< HEAD
          function GetInterface(const iidstr : shortstring;out obj) : boolean;
          function GetInterfaceByStr(const iidstr : shortstring; out obj) : boolean;
          function GetInterfaceWeak(const iid : tguid; out obj) : boolean; // equal to GetInterface but the interface returned is not referenced
=======
=======
>>>>>>> origin/fixes_2_2
          function GetInterface(const iidstr : string;out obj) : boolean;{$ifdef SYSTEMINLINE}inline;{$endif}
          function GetInterfaceByStr(const iidstr : string; out obj) : boolean;
>>>>>>> graemeg/fixes_2_2
          class function GetInterfaceEntry(const iid : tguid) : pinterfaceentry;
          class function GetInterfaceEntryByStr(const iidstr : shortstring) : pinterfaceentry;
          class function GetInterfaceTable : pinterfacetable;

          { new since Delphi 2009 }
          class function UnitName : ansistring;
          function Equals(Obj: TObject) : boolean;virtual;
          function GetHashCode: PtrInt;virtual;
          function ToString: ansistring;virtual;
       end;

       IUnknown = interface
         ['{00000000-0000-0000-C000-000000000046}']
         function QueryInterface({$IFDEF FPC_HAS_CONSTREF}constref{$ELSE}const{$ENDIF} iid : tguid;out obj) : longint;{$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
         function _AddRef : longint;{$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
         function _Release : longint;{$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
       end;
       IInterface = IUnknown;

       {$M+}
       IInvokable = interface(IInterface)
       end;
       {$M-}

       { enumerator support }
       IEnumerator = interface(IInterface)
         function GetCurrent: TObject;
         function MoveNext: Boolean;
         procedure Reset;
         property Current: TObject read GetCurrent;
       end;

       IEnumerable = interface(IInterface)
         function GetEnumerator: IEnumerator;
       end;

       { for native dispinterface support }
       IDispatch = interface(IUnknown)
          ['{00020400-0000-0000-C000-000000000046}']
          function GetTypeInfoCount(out count : longint) : HResult;stdcall;
          function GetTypeInfo(Index,LocaleID : longint;
            out TypeInfo): HResult;stdcall;
          function GetIDsOfNames(const iid: TGUID; names: Pointer;
            NameCount, LocaleID: LongInt; DispIDs: Pointer) : HResult;stdcall;
          function Invoke(DispID: LongInt;const iid : TGUID;
            LocaleID : longint; Flags: Word;var params;
            VarResult,ExcepInfo,ArgErr : pointer) : HResult;stdcall;
       end;

       TInterfacedObject = class(TObject,IUnknown)
       protected
          frefcount : longint;
          { implement methods of IUnknown }
          function QueryInterface({$IFDEF FPC_HAS_CONSTREF}constref{$ELSE}const{$ENDIF} iid : tguid;out obj) : longint;{$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
          function _AddRef : longint;{$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
          function _Release : longint;{$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
        public
          procedure AfterConstruction;override;
          procedure BeforeDestruction;override;
          class function NewInstance : TObject;override;
          property RefCount : longint read frefcount;
       end;
       TInterfacedClass = class of TInterfacedObject;

       TAggregatedObject = class(TObject)
       private
          fcontroller: Pointer;
          function GetController: IUnknown;
       protected
          { implement methods of IUnknown }
          function QueryInterface({$IFDEF FPC_HAS_CONSTREF}constref{$ELSE}const{$ENDIF} iid : tguid;out obj) : longint;{$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
          function _AddRef : longint;{$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
          function _Release : longint;{$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
       public
          constructor Create(const aController: IUnknown);
          property Controller : IUnknown read GetController;
       end;

       TContainedObject = class(TAggregatedObject,IInterface)
<<<<<<< HEAD
<<<<<<< HEAD
         protected
           function QueryInterface({$IFDEF FPC_HAS_CONSTREF}constref{$ELSE}const{$ENDIF} iid : tguid;out obj) : longint;virtual; {$IFNDEF WINDOWS}cdecl{$ELSE}stdcall{$ENDIF};
=======
         protected 
           function QueryInterface(const iid : tguid;out obj) : longint;virtual; stdcall;
>>>>>>> graemeg/fixes_2_2
=======
         protected 
           function QueryInterface(const iid : tguid;out obj) : longint;virtual; stdcall;
>>>>>>> origin/fixes_2_2
         end;

       { some pointer definitions }
       PUnknown = ^IUnknown;
       PPUnknown = ^PUnknown;
       PDispatch = ^IDispatch;
       PPDispatch = ^PDispatch;
       PInterface = PUnknown;


       TExceptProc = Procedure (Obj : TObject; Addr : CodePointer; FrameCount:Longint; Frame: PCodePointer);

       { Exception object stack }
       PExceptObject = ^TExceptObject;
       TExceptObject = record
         FObject    : TObject;
         Addr       : codepointer;
         Next       : PExceptObject;
         refcount   : Longint;
         Framecount : Longint;
         Frames     : PCodePointer;
{$ifdef FPC_USE_WIN32_SEH}
         SEHFrame   : Pointer;
         ExceptRec  : Pointer;
         ReraiseBuf : jmp_buf;
{$endif FPC_USE_WIN32_SEH}
       end;

    Const
       ExceptProc : TExceptProc = Nil;
       RaiseProc : TExceptProc = Nil;
       RaiseMaxFrameCount : Longint = 16;

    Function RaiseList : PExceptObject;

    { @abstract(increase exception reference count)
      When leaving an except block, the exception object is normally
      freed automatically. To avoid this, call this function.
      If within the exception object you decide that you don't need
      the exception after all, call @link(ReleaseExceptionObject).
      Otherwise, if the reference count is > 0, the exception object
      goes into your "property" and you need to free it manually.
      The effect of this function is countered by re-raising an exception
      via "raise;", this zeroes the reference count again.
      Calling this method is only valid within an except block.
      @return(pointer to the exception object) }
    function AcquireExceptionObject: Pointer;

    { @abstract(decrease exception reference count)
      After calling @link(AcquireExceptionObject) you can call this method
      to decrease the exception reference count again.
      If the reference count is > 0, the exception object
      goes into your "property" and you need to free it manually.
      Calling this method is only valid within an except block. }
    procedure ReleaseExceptionObject;

{*****************************************************************************
                              Array of const support
*****************************************************************************}

   const
      vtInteger       = 0;
      vtBoolean       = 1;
      vtChar          = 2;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifndef FPUNONE}
      vtExtended      = 3;
{$endif}
=======
{$ifndef FPUNONE}     
      vtExtended      = 3;
{$endif}              
>>>>>>> graemeg/cpstrnew
=======
{$ifndef FPUNONE}     
      vtExtended      = 3;
{$endif}              
>>>>>>> graemeg/cpstrnew
=======
{$ifndef FPUNONE}     
      vtExtended      = 3;
{$endif}              
>>>>>>> graemeg/cpstrnew
=======
{$ifndef FPUNONE}     
      vtExtended      = 3;
{$endif}              
>>>>>>> origin/cpstrnew
      vtString        = 4;
      vtPointer       = 5;
      vtPChar         = 6;
      vtObject        = 7;
      vtClass         = 8;
      vtWideChar      = 9;
      vtPWideChar     = 10;
      vtAnsiString    = 11;
      vtCurrency      = 12;
      vtVariant       = 13;
      vtInterface     = 14;
      vtWideString    = 15;
      vtInt64         = 16;
      vtQWord         = 17;
      vtUnicodeString = 18;

   type
      PVarRec = ^TVarRec;
      TVarRec = record
         case VType : sizeint of
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef ENDIAN_BIG}
=======
{$ifdef ENDIAN_BIG}        
>>>>>>> graemeg/cpstrnew
=======
{$ifdef ENDIAN_BIG}        
>>>>>>> graemeg/cpstrnew
=======
{$ifdef ENDIAN_BIG}        
>>>>>>> graemeg/cpstrnew
=======
{$ifdef ENDIAN_BIG}        
>>>>>>> origin/cpstrnew
           vtInteger       : ({$IFDEF CPU64}integerdummy1 : Longint;{$ENDIF CPU64}VInteger: Longint);
           vtBoolean       : ({$IFDEF CPU64}booldummy : Longint;{$ENDIF CPU64}booldummy1,booldummy2,booldummy3: byte; VBoolean: Boolean);
           vtChar          : ({$IFDEF CPU64}chardummy : Longint;{$ENDIF CPU64}chardummy1,chardummy2,chardummy3: byte; VChar: Char);
           vtWideChar      : ({$IFDEF CPU64}widechardummy : Longint;{$ENDIF CPU64}wchardummy1,VWideChar: WideChar);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$else ENDIAN_BIG}
=======
{$else ENDIAN_BIG}         
>>>>>>> graemeg/cpstrnew
=======
{$else ENDIAN_BIG}         
>>>>>>> graemeg/cpstrnew
=======
{$else ENDIAN_BIG}         
>>>>>>> graemeg/cpstrnew
=======
{$else ENDIAN_BIG}         
>>>>>>> origin/cpstrnew
           vtInteger       : (VInteger: Longint);
           vtBoolean       : (VBoolean: Boolean);
           vtChar          : (VChar: Char);
           vtWideChar      : (VWideChar: WideChar);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$endif ENDIAN_BIG}
{$ifndef FPUNONE}
           vtExtended      : (VExtended: PExtended);
{$endif}
           vtString        : (VString: PShortString);
           vtPointer       : (VPointer: Pointer);
           vtPChar         : (VPChar: PAnsiChar);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$endif ENDIAN_BIG}        
{$ifndef FPUNONE}          
           vtExtended      : (VExtended: PExtended);
{$endif}                   
           vtString        : (VString: PShortString);
           vtPointer       : (VPointer: Pointer);
           vtPChar         : (VPChar: PChar);
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
           vtObject        : (VObject: TObject);
           vtClass         : (VClass: TClass);
           vtPWideChar     : (VPWideChar: PWideChar);
           vtAnsiString    : (VAnsiString: Pointer);
           vtCurrency      : (VCurrency: PCurrency);
           vtVariant       : (VVariant: PVariant);
           vtInterface     : (VInterface: Pointer);
           vtWideString    : (VWideString: Pointer);
           vtInt64         : (VInt64: PInt64);
           vtUnicodeString : (VUnicodeString: Pointer);
           vtQWord         : (VQWord: PQWord);
       end;

  var
<<<<<<< HEAD
    DispCallByIDProc : codepointer;
=======
    DispCallByIDProc : pointer;
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
    { for safe as operator support }
    IObjectInstance: TGuid = '{D91C9AF4-3C93-420F-A303-BF5BA82BFD23}';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD


{*****************************************************************************
                              Resourcestring support
*****************************************************************************}

  type
    PResourceStringRecord = ^TResourceStringRecord;
    TResourceStringRecord = Record
       Name,
       CurrentValue,
       DefaultValue : AnsiString;
       HashValue    : LongWord;
     end;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
