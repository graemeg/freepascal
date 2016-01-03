  
type
  //Obsolete types, kept for backwards compatibility
  TResourceHandle = PtrUint;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef Win16}
  HMODULE = THandle; {HINST}
  HGLOBAL = THandle;
{$else Win16}
  HMODULE = PtrUint;
  HGLOBAL = PtrUint;
{$endif Win16}
=======
  HMODULE = PtrUint;
  HGLOBAL = PtrUint;
>>>>>>> graemeg/cpstrnew
=======
  HMODULE = PtrUint;
  HGLOBAL = PtrUint;
>>>>>>> graemeg/cpstrnew
=======
  HMODULE = PtrUint;
  HGLOBAL = PtrUint;
>>>>>>> graemeg/cpstrnew
=======
  HMODULE = PtrUint;
  HGLOBAL = PtrUint;
>>>>>>> origin/cpstrnew
  //New types
  TFPResourceHandle = PtrUInt;
  TFPResourceHMODULE = PtrUInt;
  TFPResourceHGLOBAL = PtrUInt;
{$ifndef Win16}
{$ifdef FPC_OS_UNICODE}
  {roozbeh : maybe it shoud be moved after ifndef mswindows,as there is one declared in wince,base.inc}
  MAKEINTRESOURCE = pwidechar;
{$else}
  MAKEINTRESOURCE = pchar;
{$endif}  
{$endif not Win16}
  
{$ifndef MSWINDOWS }
{ windows has this in the windows unit. It probably would better
  fit into the classes/sysutils unit but because it is resource
  related, I put it here (FK)
}
const
  RT_CURSOR = MAKEINTRESOURCE(1);
  RT_BITMAP = MAKEINTRESOURCE(2);
  RT_ICON = MAKEINTRESOURCE(3);
  RT_MENU = MAKEINTRESOURCE(4);
  RT_DIALOG = MAKEINTRESOURCE(5);
  RT_STRING = MAKEINTRESOURCE(6);
  RT_FONTDIR = MAKEINTRESOURCE(7);
  RT_FONT = MAKEINTRESOURCE(8);
  RT_ACCELERATOR = MAKEINTRESOURCE(9);
  RT_RCDATA = MAKEINTRESOURCE(10);
  RT_MESSAGETABLE = MAKEINTRESOURCE(11);
  RT_GROUP_CURSOR = MAKEINTRESOURCE(12);
  RT_GROUP_ICON = MAKEINTRESOURCE(14);
  RT_VERSION = MAKEINTRESOURCE(16);
  RT_ANICURSOR = MAKEINTRESOURCE(21);
  RT_ANIICON = MAKEINTRESOURCE(22);
  RT_HTML = MAKEINTRESOURCE(23);
  RT_MANIFEST = MAKEINTRESOURCE(24);

{$endif MSWINDOWS }

// Win32 API compatible Resource callbacks
type
  EnumResTypeProc = function(ModuleHandle : TFPResourceHMODULE; ResourceType : PChar; lParam : PtrInt) : LongBool; stdcall;
  EnumResNameProc = function(ModuleHandle : TFPResourceHMODULE; ResourceType, ResourceName : PChar; lParam : PtrInt) : LongBool; stdcall;
  EnumResLangProc = function(ModuleHandle : TFPResourceHMODULE; ResourceType, ResourceName : PChar; IDLanguage : word; lParam : PtrInt) : LongBool; stdcall;

// Useful functions (macros in win32 headers)
function Is_IntResource(aStr : pchar) : boolean; {$ifdef SYSTEMINLINE}inline;{$endif}
function MakeLangID(primary,sub : word) : word; {$ifdef SYSTEMINLINE}inline;{$endif}

// Win32 API compatible Resource functions
{$ifndef Win16}
Function HINSTANCE : TFPResourceHMODULE;
{$endif Win16}
Function EnumResourceTypes(ModuleHandle : TFPResourceHMODULE; EnumFunc : EnumResTypeProc; lParam : PtrInt) : LongBool;
Function EnumResourceNames(ModuleHandle : TFPResourceHMODULE; ResourceType : PChar; EnumFunc : EnumResNameProc; lParam : PtrInt) : LongBool;
Function EnumResourceLanguages(ModuleHandle : TFPResourceHMODULE; ResourceType, ResourceName : PChar; EnumFunc : EnumResLangProc; lParam : PtrInt) : LongBool;
{$ifndef Win16}
Function FindResource(ModuleHandle: TFPResourceHMODULE; ResourceName, ResourceType: PChar): TFPResourceHandle;
Function FindResourceEx(ModuleHandle: TFPResourceHMODULE; ResourceType, ResourceName: PChar; Language : word): TFPResourceHandle;
Function LoadResource(ModuleHandle: TFPResourceHMODULE; ResHandle: TFPResourceHandle): TFPResourceHGLOBAL;
Function SizeofResource(ModuleHandle: TFPResourceHMODULE; ResHandle: TFPResourceHandle): LongWord;
Function LockResource(ResData: TFPResourceHGLOBAL): Pointer;
Function UnlockResource(ResData: TFPResourceHGLOBAL): LongBool;
Function FreeResource(ResData: TFPResourceHGLOBAL): LongBool;
{$endif Win16}
{$ifdef FPC_HAS_FEATURE_ANSISTRINGS}
Function FindResource(ModuleHandle: TFPResourceHMODULE; ResourceName, ResourceType: AnsiString): TFPResourceHandle;
Function FindResourceEx(ModuleHandle: TFPResourceHMODULE; ResourceType, ResourceName: AnsiString; Language : word): TFPResourceHandle;
{$endif}

type
  TResourceManager = record
    HINSTANCEFunc : function : TFPResourceHMODULE;
    EnumResourceTypesFunc : function (ModuleHandle : TFPResourceHMODULE; EnumFunc : EnumResTypeProc; lParam : PtrInt) : LongBool;
    EnumResourceNamesFunc : function (ModuleHandle : TFPResourceHMODULE; ResourceType : PChar; EnumFunc : EnumResNameProc; lParam : PtrInt) : LongBool;
    EnumResourceLanguagesFunc : function (ModuleHandle : TFPResourceHMODULE; ResourceType,ResourceName : PChar; EnumFunc : EnumResLangProc; lParam : PtrInt) : LongBool;
    FindResourceFunc : function (ModuleHandle: TFPResourceHMODULE; ResourceName, ResourceType: PChar): TFPResourceHandle;
    FindResourceExFunc : function (ModuleHandle: TFPResourceHMODULE; ResourceType, ResourceName: PChar; Language : word): TFPResourceHandle;
    LoadResourceFunc : function (ModuleHandle: TFPResourceHMODULE; ResHandle: TFPResourceHandle): TFPResourceHGLOBAL;
    SizeofResourceFunc : function (ModuleHandle: TFPResourceHMODULE; ResHandle: TFPResourceHandle): LongWord;
    LockResourceFunc : function (ResData: TFPResourceHGLOBAL): Pointer;
    UnlockResourceFunc : function (ResData: TFPResourceHGLOBAL): LongBool;
    FreeResourceFunc : function (ResData: TFPResourceHGLOBAL): LongBool;
  end;

procedure GetResourceManager (var Manager : TResourceManager);
procedure SetResourceManager (const New : TResourceManager);

