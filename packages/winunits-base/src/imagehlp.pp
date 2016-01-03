{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2005 by Marco van de Voort
    member of the Free Pascal development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

    Notes 
	  - some callback types had Pxxx naming in the original header. 
	     Since that is a-typical, I can't predict how Borland did 
		 handle that. If you have problems, add a bugreport.
	  - some anonymous unions were not representable in FPC. As a first approx 
	    I added a dummy nested record union.
	  
}
unit imagehlp;

Interface

Uses Windows;

<<<<<<< HEAD
<<<<<<< HEAD
{$packrecords 8}
{$ifdef win64}
 {$define imagehlp64}
{$endif}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef FPC_OS_UNICODE}
  {$define UNICODE}
{$endif}

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
Const 
     IMAGE_SEPARATION    = 64*1024;  	
     DBHHEADER_DEBUGDIRS = $1;     
     API_VERSION_NUMBER  = 9;  	 
     SLMFLAG_VIRTUAL     = $1;     
     MINIDUMP_SIGNATURE  = 'PMDM';  // dword value?
     MINIDUMP_VERSION    = 42899;
     External_Library	 = 'imagehlp.dll';
     MAX_SYM_NAME        = 2000;

     BIND_NO_BOUND_IMPORTS  = $00000001;
     BIND_NO_UPDATE         = $00000002;
     BIND_ALL_IMAGES        = $00000004;
     BIND_CACHE_IMPORT_DLLS = $00000008;       // Cache dll's across
                                                //  calls to BindImageEx
                                                //  (same as NT 3.1->NT 4.0)
     BIND_REPORT_64BIT_VA   = $00000010;
     CHECKSUM_SUCCESS            = 0;
     CHECKSUM_OPEN_FAILURE       = 1;
     CHECKSUM_MAP_FAILURE        = 2;
     CHECKSUM_MAPVIEW_FAILURE    = 3;
     CHECKSUM_UNICODE_FAILURE    = 4;


     SPLITSYM_REMOVE_PRIVATE     = $00000001;      // Remove CV types/symbols and Fixup debug
                                                    //  Used for creating .dbg files that ship
                                                    //  as part of the product.

     SPLITSYM_EXTRACT_ALL        = $00000002;      // Extract all debug info from image.
                                                    //  Normally, FPO is left/* IN */ the image
                                                    //  to allow stack traces through the code.
                                                    //  Using this switch is similar to linking
                                                    //  with -debug:none except the .dbg file
                                                    //  exists...

     SPLITSYM_SYMBOLPATH_IS_SRC  = $00000004;      // The SymbolFilePath contains an alternate
                                                    //  path to locate the pdb.

     CERT_PE_IMAGE_DIGEST_DEBUG_INFO         = $01;
     CERT_PE_IMAGE_DIGEST_RESOURCES          = $02;
     CERT_PE_IMAGE_DIGEST_ALL_IMPORT_INFO    = $04;
     CERT_PE_IMAGE_DIGEST_NON_PE_INFO        = $08;      // include data outside the PE image

     CERT_SECTION_TYPE_ANY                   = $FF;      // Any Certificate type
     SSRVOPT_CALLBACK            = $0001;
     SSRVOPT_DWORD               = $0002;
     SSRVOPT_DWORDPTR            = $0004;
     SSRVOPT_GUIDPTR             = $0008;
     SSRVOPT_OLDGUIDPTR          = $0010;
     SSRVOPT_UNATTENDED          = $0020;
     SSRVOPT_NOCOPY              = $0040;
     SSRVOPT_PARENTWIN           = $0080;
     SSRVOPT_PARAMTYPE           = $0100;
     SSRVOPT_SECURE              = $0200;
     SSRVOPT_TRACE               = $0400;
     SSRVOPT_SETCONTEXT          = $0800;
     SSRVOPT_PROXY               = $1000;
     SSRVOPT_DOWNSTREAM_STORE    = $2000;
     SSRVOPT_RESET               = ULONG_PTR(-1);

     SSRVACTION_TRACE            = 1;
     SSRVACTION_QUERYCANCEL      = 2;
     SSRVACTION_EVENT            = 3;
     UNDNAME_COMPLETE                 = $0000;  // Enable full undecoration
     UNDNAME_NO_LEADING_UNDERSCORES   = $0001;  // Remove leading underscores from MS extended keywords
     UNDNAME_NO_MS_KEYWORDS           = $0002;  // Disable expansion of MS extended keywords
     UNDNAME_NO_FUNCTION_RETURNS      = $0004;  // Disable expansion of return type for primary declaration
     UNDNAME_NO_ALLOCATION_MODEL      = $0008;  // Disable expansion of the declaration model
     UNDNAME_NO_ALLOCATION_LANGUAGE   = $0010;  // Disable expansion of the declaration language specifier
     UNDNAME_NO_MS_THISTYPE           = $0020;  // NYI Disable expansion of MS keywords on the 'this' type for primary declaration
     UNDNAME_NO_CV_THISTYPE           = $0040;  // NYI Disable expansion of CV modifiers on the 'this' type for primary declaration
     UNDNAME_NO_THISTYPE              = $0060;  // Disable all modifiers on the 'this' type
     UNDNAME_NO_ACCESS_SPECIFIERS     = $0080;  // Disable expansion of access specifiers for members
     UNDNAME_NO_THROW_SIGNATURES      = $0100;  // Disable expansion of 'throw-signatures' for functions and pointers to functions
     UNDNAME_NO_MEMBER_TYPE           = $0200;  // Disable expansion of 'static' or 'virtual'ness of members
     UNDNAME_NO_RETURN_UDT_MODEL      = $0400;  // Disable expansion of MS model for UDT returns
     UNDNAME_32_BIT_DECODE            = $0800;  // Undecorate 32-bit decorated names
     UNDNAME_NAME_ONLY                = $1000;  // Crack only the name for primary declaration;
                                                                                                   //  return just [scope::]name.  Does expand template params
     UNDNAME_NO_ARGUMENTS             = $2000;  // Don't undecorate arguments to function
     UNDNAME_NO_SPECIAL_SYMS          = $4000;  // Don't undecorate special names (v-table, vcall, vector xxx, metatype, etc)
     SYMFLAG_VALUEPRESENT     = $00000001;
     SYMFLAG_REGISTER         = $00000008;
     SYMFLAG_REGREL           = $00000010;
     SYMFLAG_FRAMEREL         = $00000020;
     SYMFLAG_PARAMETER        = $00000040;
     SYMFLAG_LOCAL            = $00000080;
     SYMFLAG_CONSTANT         = $00000100;
     SYMFLAG_EXPORT           = $00000200;
     SYMFLAG_FORWARDER        = $00000400;
     SYMFLAG_FUNCTION         = $00000800;
     SYMFLAG_VIRTUAL          = $00001000;
     SYMFLAG_THUNK            = $00002000;
     SYMFLAG_TLSREL           = $00004000;
     CBA_DEFERRED_SYMBOL_LOAD_START          = $00000001;
     CBA_DEFERRED_SYMBOL_LOAD_COMPLETE       = $00000002;
     CBA_DEFERRED_SYMBOL_LOAD_FAILURE        = $00000003;
     CBA_SYMBOLS_UNLOADED                    = $00000004;
     CBA_DUPLICATE_SYMBOL                    = $00000005;
     CBA_READ_MEMORY                         = $00000006;
     CBA_DEFERRED_SYMBOL_LOAD_CANCEL         = $00000007;
     CBA_SET_OPTIONS                         = $00000008;
     CBA_EVENT                               = $00000010;
     CBA_DEFERRED_SYMBOL_LOAD_PARTIAL        = $00000020;
     CBA_DEBUG_INFO                          = $10000000;
     DSLFLAG_MISMATCHED_PDB	     = $1;
     DSLFLAG_MISMATCHED_DBG          = $2;
     SYMOPT_CASE_INSENSITIVE         = $00000001;
     SYMOPT_UNDNAME                  = $00000002;
     SYMOPT_DEFERRED_LOADS           = $00000004;
     SYMOPT_NO_CPP                   = $00000008;
     SYMOPT_LOAD_LINES               = $00000010;
     SYMOPT_OMAP_FIND_NEAREST        = $00000020;
     SYMOPT_LOAD_ANYTHING            = $00000040;
     SYMOPT_IGNORE_CVREC             = $00000080;
     SYMOPT_NO_UNQUALIFIED_LOADS     = $00000100;
     SYMOPT_FAIL_CRITICAL_ERRORS     = $00000200;
     SYMOPT_EXACT_SYMBOLS            = $00000400;
     SYMOPT_ALLOW_ABSOLUTE_SYMBOLS   = $00000800;
     SYMOPT_IGNORE_NT_SYMPATH        = $00001000;
     SYMOPT_INCLUDE_32BIT_MODULES    = $00002000;
     SYMOPT_PUBLICS_ONLY             = $00004000;
     SYMOPT_NO_PUBLICS               = $00008000;
     SYMOPT_AUTO_PUBLICS             = $00010000;
     SYMOPT_NO_IMAGE_SEARCH          = $00020000;
     SYMOPT_SECURE                   = $00040000;
     SYMOPT_NO_PROMPTS               = $00080000;

     SYMOPT_DEBUG                    = $80000000;
     SYMENUMFLAG_FULLSRCH            = 1;
     SYMENUMFLAG_SPEEDSRCH           = 2;
     SYMF_OMAP_GENERATED   = $00000001;
     SYMF_OMAP_MODIFIED    = $00000002;
     SYMF_REGISTER         = $00000008;
     SYMF_REGREL           = $00000010;
     SYMF_FRAMEREL         = $00000020;
     SYMF_PARAMETER        = $00000040;
     SYMF_LOCAL            = $00000080;
     SYMF_CONSTANT         = $00000100;
     SYMF_EXPORT           = $00000200;
     SYMF_FORWARDER        = $00000400;
     SYMF_FUNCTION         = $00000800;
     SYMF_VIRTUAL          = $00001000;
     SYMF_THUNK            = $00002000;
     SYMF_TLSREL           = $00004000;

// These values should also not be used.
// They have been replaced by SYMFLAG_ values.

     IMAGEHLP_SYMBOL_INFO_VALUEPRESENT          = 1;
     IMAGEHLP_SYMBOL_INFO_REGISTER              = SYMF_REGISTER      ;  // = $0008
     IMAGEHLP_SYMBOL_INFO_REGRELATIVE           = SYMF_REGREL       ;   // = $0010
     IMAGEHLP_SYMBOL_INFO_FRAMERELATIVE         = SYMF_FRAMEREL    ;    // = $0020
     IMAGEHLP_SYMBOL_INFO_PARAMETER             = SYMF_PARAMETER  ;     // = $0040
     IMAGEHLP_SYMBOL_INFO_LOCAL                 = SYMF_LOCAL     ;      // = $0080
     IMAGEHLP_SYMBOL_INFO_CONSTANT              = SYMF_CONSTANT ;       // = $0100
     IMAGEHLP_SYMBOL_FUNCTION                   = SYMF_FUNCTION;        // = $0800
     IMAGEHLP_SYMBOL_VIRTUAL                    = SYMF_VIRTUAL;         // = $1000
     IMAGEHLP_SYMBOL_THUNK                      = SYMF_THUNK ;          // = $2000
     IMAGEHLP_SYMBOL_INFO_TLSRELATIVE           = SYMF_TLSREL;          // = $4000
     MINIDUMP_MISC1_PROCESS_ID    = $00000001;
     MINIDUMP_MISC1_PROCESS_TIMES = $00000002;

 
Type
   DIGEST_HANDLE = POINTER;
   TDIGEST_HANDLE = DIGEST_HANDLE;
   PRVA = ^TRVA;
   TRVA = dword;
   RVA  = TRVA;
   ULONG64 = QWORD; // to windows unit ?
   ULONG32 = CARDINAL;
   PRVA64 = ^TRVA64;   
   TRVA64 = ULONG64;
   RVA64  = TRVA64;
   PPSTR  = ^PSTR;

 _IMAGEHLP_STATUS_REASON =(
    BindOutOfMemory,
    BindRvaToVaFailed,
    BindNoRoomInImage,
    BindImportModuleFailed,
    BindImportProcedureFailed,
    BindImportModule,
    BindImportProcedure,
    BindForwarder,
    BindForwarderNOT,
    BindImageModified,
    BindExpandFileHeaders,
    BindImageComplete,
    BindMismatchedSymbols,
    BindSymbolsNotUpdated,
    BindImportProcedure32,
    BindImportProcedure64,
    BindForwarder32,
    BindForwarder64,
    BindForwarderNOT32,
    BindForwarderNOT64);
  IMAGEHLP_STATUS_REASON = _IMAGEHLP_STATUS_REASON;	
  TIMAGEHLP_STATUS_REASON = _IMAGEHLP_STATUS_REASON;	
  ADDRESS_MODE =(
    AddrMode1616,
    AddrMode1632,
    AddrModeReal,
    AddrModeFlat);
  TADDRESS_MODE = ADDRESS_MODE;
  SYM_TYPE =(
    SymNone = 0,
    SymCoff,
    SymCv,
    SymPdb,
    SymExport,
    SymDeferred,
    SymSym,       // .sym file
    SymDia,
    SymVirtual,
    NumSymTypes);
  TSYM_TYPE = SYM_TYPE;
  
  _IMAGEHLP_SYMBOL_TYPE_INFO = (
    TI_GET_SYMTAG,
    TI_GET_SYMNAME,
    TI_GET_LENGTH,
    TI_GET_TYPE,
    TI_GET_TYPEID,
    TI_GET_BASETYPE,
    TI_GET_ARRAYINDEXTYPEID,
    TI_FINDCHILDREN,
    TI_GET_DATAKIND,
    TI_GET_ADDRESSOFFSET,
    TI_GET_OFFSET,
    TI_GET_VALUE,
    TI_GET_COUNT,
    TI_GET_CHILDRENCOUNT,
    TI_GET_BITPOSITION,
    TI_GET_VIRTUALBASECLASS,
    TI_GET_VIRTUALTABLESHAPEID,
    TI_GET_VIRTUALBASEPOINTEROFFSET,
    TI_GET_CLASSPARENTID,
    TI_GET_NESTED,
    TI_GET_SYMINDEX,
    TI_GET_LEXICALPARENT,
    TI_GET_ADDRESS,
    TI_GET_THISADJUST,
    TI_GET_UDTKIND,
    TI_IS_EQUIV_TO,
    TI_GET_CALLING_CONVENTION);
  IMAGEHLP_SYMBOL_TYPE_INFO = _IMAGEHLP_SYMBOL_TYPE_INFO;
  TIMAGEHLP_SYMBOL_TYPE_INFO = _IMAGEHLP_SYMBOL_TYPE_INFO;
  _MINIDUMP_STREAM_TYPE = (
    UnusedStream                = 0,
    ReservedStream0             = 1,
    ReservedStream1             = 2,
    ThreadListStream            = 3,
    ModuleListStream            = 4,
    MemoryListStream            = 5,
    ExceptionStream             = 6,
    SystemInfoStream            = 7,
    ThreadExListStream          = 8,
    Memory64ListStream          = 9,
    CommentStreamA              = 10,
    CommentStreamW              = 11,
    HandleDataStream            = 12,
    FunctionTableStream         = 13,
    UnloadedModuleListStream    = 14,
    MiscInfoStream              = 15,

    LastReservedStream          = $ffff);

  MINIDUMP_STREAM_TYPE = _MINIDUMP_STREAM_TYPE;
	
  _MINIDUMP_CALLBACK_TYPE =(
    ModuleCallback,
    ThreadCallback,
    ThreadExCallback,
    IncludeThreadCallback,
    IncludeModuleCallback,
    MemoryCallback);
  MINIDUMP_CALLBACK_TYPE = _MINIDUMP_CALLBACK_TYPE;

  _THREAD_WRITE_FLAGS =(
    ThreadWriteThread            = $0001,
    ThreadWriteStack             = $0002,
    ThreadWriteContext           = $0004,
    ThreadWriteBackingStore      = $0008,
    ThreadWriteInstructionWindow = $0010,
    ThreadWriteThreadData        = $0020);
  THREAD_WRITE_FLAGS = _THREAD_WRITE_FLAGS;
  _MODULE_WRITE_FLAGS =(
    ModuleWriteModule        = $0001,
    ModuleWriteDataSeg       = $0002,
    ModuleWriteMiscRecord    = $0004,
    ModuleWriteCvRecord      = $0008,
    ModuleReferencedByMemory = $0010);
  MODULE_WRITE_FLAGS = _MODULE_WRITE_FLAGS;
  
  tagan = (
    sevInfo = 0,
    sevProblem,
    sevAttn,
    sevFatal,
    sevMax  );
	
  Anonymous = (
    hdBase = 0, // root directory for dbghelp
    hdSym,      // where symbols are stored
    hdSrc,      // where source is stored
    hdMax       // end marker
	);
	
  _MINIDUMP_TYPE = (
    MiniDumpNormal                         = $0000,
    MiniDumpWithDataSegs                   = $0001,
    MiniDumpWithFullMemory                 = $0002,
    MiniDumpWithHandleData                 = $0004,
    MiniDumpFilterMemory                   = $0008,
    MiniDumpScanMemory                     = $0010,
    MiniDumpWithUnloadedModules            = $0020,
    MiniDumpWithIndirectlyReferencedMemory = $0040,
    MiniDumpFilterModulePaths              = $0080,
    MiniDumpWithProcessThreadData          = $0100,
    MiniDumpWithPrivateReadWriteMemory     = $0200,
    MiniDumpWithoutOPTIONALData            = $0400);
  MINIDUMP_TYPE = _MINIDUMP_TYPE;
  TMINIDUMP_TYPE = _MINIDUMP_TYPE;
  
<<<<<<< HEAD
<<<<<<< HEAD
  TLOADED_IMAGE = record
=======
  TLOADED_IMAGE = packed record
>>>>>>> graemeg/fixes_2_2
=======
  TLOADED_IMAGE = packed record
>>>>>>> origin/fixes_2_2
          ModuleName : PSTR;
          hFile : THANDLE;
          MappedAddress : PUCHAR;
{$ifdef IMAGEHLP64}
          FileHeader : PIMAGE_NT_HEADERS64;
{$else}		  
          FileHeader : PIMAGE_NT_HEADERS32;
{$endif}		  
          LastRvaSection : PIMAGE_SECTION_HEADER;
          NumberOfSections : ULONG;
          Sections : PIMAGE_SECTION_HEADER;
          Characteristics : ULONG;
          fSystemImage : bytebool;
          fDOSImage : bytebool;
          Links : TLISTENTRY;
          SizeOfImage : ULONG;
       end;
  LOADED_IMAGE   = TLOADED_IMAGE;
  PLOADED_IMAGE  = ^TLOADED_IMAGE;
  PPLOADED_IMAGE = ^PLOADED_IMAGE;
{$ifndef win64}  
  PIMAGE_DEBUG_INFORMATION = ^TIMAGE_DEBUG_INFORMATION;
<<<<<<< HEAD
<<<<<<< HEAD
  TIMAGE_DEBUG_INFORMATION = record
=======
  TIMAGE_DEBUG_INFORMATION = packed record
>>>>>>> graemeg/fixes_2_2
=======
  TIMAGE_DEBUG_INFORMATION = packed record
>>>>>>> origin/fixes_2_2
          List : TLISTENTRY;
          ReservedSize : DWORD;
          ReservedMappedBase : POINTER;
          ReservedMachine : USHORT;
          ReservedCharacteristics : USHORT;
          ReservedCheckSum : DWORD;
          ImageBase : DWORD;
          SizeOfImage : DWORD;
          ReservedNumberOfSections : DWORD;
          ReservedSections : PIMAGE_SECTION_HEADER;
          ReservedExportedNamesSize : DWORD;
          ReservedExportedNames : PSTR;
          ReservedNumberOfFunctionTableEntries : DWORD;
          ReservedFunctionTableEntries : PIMAGE_FUNCTION_ENTRY;
          ReservedLowestFunctionStartingAddress : DWORD;
          ReservedHighestFunctionEndingAddress : DWORD;
          ReservedNumberOfFpoTableEntries : DWORD;
          ReservedFpoTableEntries : PFPO_DATA;
          SizeOfCoffSymbols : DWORD;
          CoffSymbols : PIMAGE_COFF_SYMBOLS_HEADER;
          ReservedSizeOfCodeViewSymbols : DWORD;
          ReservedCodeViewSymbols : pointer;
          ImageFilePath : PSTR;
          ImageFileName : PSTR;
          ReservedDebugFilePath : PSTR;
          ReservedTimeDateStamp : DWORD;
          ReservedRomImage : bool;
          ReservedDebugDirectory : PIMAGE_DEBUG_DIRECTORY;
          ReservedNumberOfDebugDirectories : DWORD;
          ReservedOriginalFunctionTableBaseAddress : DWORD;
          Reserved : array[0..1] of DWORD;
       end;
  IMAGE_DEBUG_INFORMATION = TIMAGE_DEBUG_INFORMATION;
{$ENDIF}  
  PMODLOAD_DATA = ^TMODLOAD_DATA;
<<<<<<< HEAD
<<<<<<< HEAD
  TMODLOAD_DATA = record
=======
  TMODLOAD_DATA = packed record
>>>>>>> graemeg/fixes_2_2
=======
  TMODLOAD_DATA = packed record
>>>>>>> origin/fixes_2_2
          ssize : DWORD;
          ssig  : DWORD;
          data  : POINTER;
          _size : DWORD;
          flags : DWORD;
       end;
  MODLOAD_DATA	= TMODLOAD_DATA;
<<<<<<< HEAD
<<<<<<< HEAD
  tagADDRESS64 = record
=======
  tagADDRESS64 = packed record
>>>>>>> graemeg/fixes_2_2
=======
  tagADDRESS64 = packed record
>>>>>>> origin/fixes_2_2
          Offset : dword64;
          Segment : word;
          Mode : TADDRESS_MODE;
       end;
  TADDRESS64 = tagADDRESS64;
  PADDRESS64 = ^TADDRESS64;
  LPADDRESS64 = PADDRESS64;
  {$IFDEF IMAGEHLP64}
<<<<<<< HEAD
<<<<<<< HEAD
    TADDRESS=TADDRESS64;
	PADDRESS=PADDRESS64;
	LPADDRESS=PADDRESS64;
  {$ELSE}
    tagADDRESS = record
=======
=======
>>>>>>> origin/fixes_2_2
    TADRESS=TADDRESS64;
	PADRESS=PADDRESS64;
	LPADRESS=PADDRESS64;
  {$ELSE}
    tagADDRESS = packed record
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
          Offset : dword;
          Segment : word;
          Mode : TADDRESS_MODE;
       end;
    TADDRESS = tagADDRESS;
    PADDRESS = ^TADDRESS;
    LPADDRESS = PADDRESS;
  {$ENDIF}
  PKDHELP64 = ^TKDHELP64;
<<<<<<< HEAD
<<<<<<< HEAD
  TKDHELP64 = record          
=======
  TKDHELP64 = packed record          
>>>>>>> graemeg/fixes_2_2
=======
  TKDHELP64 = packed record          
>>>>>>> origin/fixes_2_2
          Thread : DWORD64;
          ThCallbackStack : DWORD;
          ThCallbackBStore : DWORD;
          NextCallback : DWORD;
          FramePointer : DWORD;
          KiCallUserMode : DWORD64;
          KeUserCallbackDispatcher : DWORD64;
          SystemRangeStart : DWORD64;
          Reserved : array[0..7] of DWORD64;
       end;
  KDHELP64 = TKDHELP64;
  {$IFDEF IMAGEHLP64}
    TKDHELP  = KDHELP64;
    KDHELP   = KDHELP64;
    PKDHELP  = PKDHELP64;
  {$ELSE}
    PKDHELP = ^TKDHELP;
<<<<<<< HEAD
<<<<<<< HEAD
    TKDHELP = record
=======
    TKDHELP = packed record
>>>>>>> graemeg/fixes_2_2
=======
    TKDHELP = packed record
>>>>>>> origin/fixes_2_2
          Thread : DWORD;
          ThCallbackStack : DWORD;
          NextCallback : DWORD;
          FramePointer : DWORD;
          KiCallUserMode : DWORD;
          KeUserCallbackDispatcher : DWORD;
          SystemRangeStart : DWORD;
          ThCallbackBStore : DWORD;
          Reserved : array[0..7] of DWORD;
       end;
    KDHELP = TKDHELP;     
  {$ENDIF}
<<<<<<< HEAD
<<<<<<< HEAD
  tagSTACKFRAME64 = record
=======
  tagSTACKFRAME64 = packed record
>>>>>>> graemeg/fixes_2_2
=======
  tagSTACKFRAME64 = packed record
>>>>>>> origin/fixes_2_2
          AddrPC : TADDRESS64;
          AddrReturn : TADDRESS64;
          AddrFrame : TADDRESS64;
          AddrStack : TADDRESS64;
          AddrBStore : TADDRESS64;
          FuncTableEntry : POINTER;
          Params : array[0..3] of DWORD64;
          Far : BOOL;
          Virtual : BOOL;
          Reserved : array[0..2] of DWORD64;
          KdHelp : TKDHELP64;
       end;
  TSTACKFRAME64 = tagSTACKFRAME64;
  PSTACKFRAME64 = ^TSTACKFRAME64;
  LPSTACKFRAME64= ^TSTACKFRAME64;  
  {$IFDEF IMAGEHLP64}
<<<<<<< HEAD
<<<<<<< HEAD
    STACKFRAME   = TSTACKFRAME64;     
=======
    STACKFRAME   = STACKFRAME64;     
>>>>>>> graemeg/fixes_2_2
=======
    STACKFRAME   = STACKFRAME64;     
>>>>>>> origin/fixes_2_2
    LPSTACKFRAME = LPSTACKFRAME64;     
	TSTACKFRAME  = TSTACKFRAME64;     
    PSTACKFRAME  = PSTACKFRAME64;     	
  {$ELSE}    
<<<<<<< HEAD
<<<<<<< HEAD
    tagSTACKFRAME = record
=======
    tagSTACKFRAME = packed record
>>>>>>> graemeg/fixes_2_2
=======
    tagSTACKFRAME = packed record
>>>>>>> origin/fixes_2_2
          AddrPC : TADDRESS;
          AddrReturn : TADDRESS;
          AddrFrame : TADDRESS;
          AddrStack : TADDRESS;
          FuncTableEntry : POINTER;
          Params : array[0..3] of DWORD;
          _Far : BOOL;
          _Virtual : BOOL;
          Reserved : array[0..2] of DWORD;
          KdHelp : TKDHELP;
          AddrBStore : TADDRESS;
       end;
    TSTACKFRAME = tagSTACKFRAME;
    PSTACKFRAME = ^TSTACKFRAME;
    LPSTACKFRAME= ^TSTACKFRAME;	
  {$ENDIF}
   PAPI_VERSION = ^TAPI_VERSION;
   LPAPI_VERSION= PAPI_VERSION;   
<<<<<<< HEAD
<<<<<<< HEAD
   TAPI_VERSION = record
=======
   TAPI_VERSION = packed record
>>>>>>> graemeg/fixes_2_2
=======
   TAPI_VERSION = packed record
>>>>>>> origin/fixes_2_2
          MajorVersion : ushort;
          MinorVersion : ushort;
          Revision : ushort;
          Reserved : ushort;
       end;
   API_VERSION = TAPI_VERSION;
      
   PIMAGEHLP_SYMBOL64 = ^TIMAGEHLP_SYMBOL64;
<<<<<<< HEAD
<<<<<<< HEAD
   TIMAGEHLP_SYMBOL64 = record
=======
   TIMAGEHLP_SYMBOL64 = packed record
>>>>>>> graemeg/fixes_2_2
=======
   TIMAGEHLP_SYMBOL64 = packed record
>>>>>>> origin/fixes_2_2
          SizeOfStruct : dword;
          Address : dword64;
          Size : dword;
          Flags : dword;
          MaxNameLength : dword;
          Name : array[0..0] of TCHAR;
       end;
   IMAGEHLP_SYMBOL64 = TIMAGEHLP_SYMBOL64;
   LPIMAGEHLP_SYMBOL64 = PIMAGEHLP_SYMBOL64;
   
   PIMAGEHLP_SYMBOL64_PACKAGE = ^TIMAGEHLP_SYMBOL64_PACKAGE;
<<<<<<< HEAD
<<<<<<< HEAD
   TIMAGEHLP_SYMBOL64_PACKAGE = record
=======
   TIMAGEHLP_SYMBOL64_PACKAGE = packed record
>>>>>>> graemeg/fixes_2_2
=======
   TIMAGEHLP_SYMBOL64_PACKAGE = packed record
>>>>>>> origin/fixes_2_2
          sym : TIMAGEHLP_SYMBOL64;
          name : array[0..(MAX_SYM_NAME+1)-1] of TCHAR;
       end;
   IMAGEHLP_SYMBOL64_PACKAGE = TIMAGEHLP_SYMBOL64_PACKAGE;
   LPIMAGEHLP_SYMBOL64_PACKAGE = ^TIMAGEHLP_SYMBOL64_PACKAGE;
   
   {$IFDEF IMAGEHLP64}
    IMAGEHLP_SYMBOL  = IMAGEHLP_SYMBOL64;
    TIMAGEHLP_SYMBOL = IMAGEHLP_SYMBOL64;
<<<<<<< HEAD
<<<<<<< HEAD
    PIMAGEHLP_SYMBOL = PIMAGEHLP_SYMBOL64;
    TIMAGEHLP_SYMBOL_PACKAGE = IMAGEHLP_SYMBOL64_PACKAGE;	
    IMAGEHLP_SYMBOL_PACKAGE = IMAGEHLP_SYMBOL64_PACKAGE;
    PIMAGEHLP_SYMBOL_PACKAGE= PIMAGEHLP_SYMBOL64_PACKAGE;
   {$ELSE}
    PIMAGEHLP_SYMBOL = ^TIMAGEHLP_SYMBOL;
    TIMAGEHLP_SYMBOL = record
=======
=======
>>>>>>> origin/fixes_2_2
    PIMAGEHLP_SYMBOL = PIMAGEHLP_SYMBOL64
    TIMAGEHLP_SYMBOL_PACKAGE = IMAGEHLP_SYMBOL64_PACKAGE	
    IMAGEHLP_SYMBOL_PACKAGE = IMAGEHLP_SYMBOL64_PACKAGE
    PIMAGEHLP_SYMBOL_PACKAGE= PIMAGEHLP_SYMBOL64_PACKAGE
   {$ELSE}
    PIMAGEHLP_SYMBOL = ^TIMAGEHLP_SYMBOL;
    TIMAGEHLP_SYMBOL = packed record
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
          SizeOfStruct : dword;
          Address : dword;
          Size : dword;
          Flags : dword;
          MaxNameLength : dword;
          Name : array[0..0] of TCHAR;
       end;
    IMAGEHLP_SYMBOL  = TIMAGEHLP_SYMBOL;
    LPIMAGEHLP_SYMBOL = PIMAGEHLP_SYMBOL;

    PIMAGEHLP_SYMBOL_PACKAGE = ^TIMAGEHLP_SYMBOL_PACKAGE;
<<<<<<< HEAD
<<<<<<< HEAD
    TIMAGEHLP_SYMBOL_PACKAGE = record
=======
    TIMAGEHLP_SYMBOL_PACKAGE = packed record
>>>>>>> graemeg/fixes_2_2
=======
    TIMAGEHLP_SYMBOL_PACKAGE = packed record
>>>>>>> origin/fixes_2_2
          sym : TIMAGEHLP_SYMBOL;
          name : array[0..(MAX_SYM_NAME+1)-1] of TCHAR;
       end;
    IMAGEHLP_SYMBOL_PACKAGE  = TIMAGEHLP_SYMBOL_PACKAGE;
    LPIMAGEHLP_SYMBOL_PACKAGE = PIMAGEHLP_SYMBOL_PACKAGE;
   {$ENDIF}
   PIMAGEHLP_MODULE64 = ^TIMAGEHLP_MODULE64;
<<<<<<< HEAD
<<<<<<< HEAD
   TIMAGEHLP_MODULE64 = record
=======
   TIMAGEHLP_MODULE64 = packed record
>>>>>>> graemeg/fixes_2_2
=======
   TIMAGEHLP_MODULE64 = packed record
>>>>>>> origin/fixes_2_2
          SizeOfStruct : dword;
          BaseOfImage : dword64;
          ImageSize : dword;
          TimeDateStamp : dword;
          CheckSum : dword;
          NumSyms : dword;
          SymType : TSYM_TYPE;
          ModuleName : array[0..31] of TCHAR;
          ImageName : array[0..255] of TCHAR;
          LoadedImageName : array[0..255] of TCHAR;
          LoadedPdbName : array[0..255] of TCHAR;
          CVSig : dword;
          CVData : array[0..(MAX_PATH*3)-1] of TCHAR;
          PdbSig : dword;
          PdbSig70 : TGUID;
          PdbAge : dword;
          PdbUnmatched : bool;
          DbgUnmatched : bool;
          LineNumbers : bool;
          GlobalSymbols : bool;
          TypeInfo : bool;
       end;
   IMAGEHLP_MODULE64 = TIMAGEHLP_MODULE64;
     
   PIMAGEHLP_MODULE64W = ^TIMAGEHLP_MODULE64W;
<<<<<<< HEAD
<<<<<<< HEAD
   TIMAGEHLP_MODULE64W = record
=======
   TIMAGEHLP_MODULE64W = packed record
>>>>>>> graemeg/fixes_2_2
=======
   TIMAGEHLP_MODULE64W = packed record
>>>>>>> origin/fixes_2_2
          SizeOfStruct : dword;
          BaseOfImage : dword64;
          ImageSize : dword;
          TimeDateStamp : dword;
          CheckSum : dword;
          NumSyms : dword;
          SymType : TSYM_TYPE;
          ModuleName : array[0..31] of wchar;
          ImageName : array[0..255] of wchar;
          LoadedImageName : array[0..255] of wchar;
          LoadedPdbName : array[0..255] of wchar;
          CVSig : dword;
          CVData : array[0..(MAX_PATH*3)-1] of wchar;
          PdbSig : dword;
          PdbSig70 : TGUID;
          PdbAge : dword;
          PdbUnmatched : bool;
          DbgUnmatched : bool;
          LineNumbers : bool;
          GlobalSymbols : bool;
          TypeInfo : bool;
       end;
  IMAGEHLP_MODULE64W = TIMAGEHLP_MODULE64W;
  LPIMAGEHLP_MODULE64W = PIMAGEHLP_MODULE64W;
  IMAGEHLP_MODULEW64 = TIMAGEHLP_MODULE64W;
  TIMAGEHLP_MODULEW64 = TIMAGEHLP_MODULE64W;    
  PIMAGEHLP_MODULEW64 = PIMAGEHLP_MODULE64W;
  {$IFDEF IMAGEHLP64}
   IMAGEHLP_MODULE   = TIMAGEHLP_MODULE64;
   LPIMAGEHLP_MODULE = PIMAGEHLP_MODULE64; 
   IMAGEHLP_MODULEW = TIMAGEHLP_MODULE64W;
   LPIMAGEHLP_MODULEW = PIMAGEHLP_MODULE64W;  
  {$ELSE}
   PIMAGEHLP_MODULE = ^TIMAGEHLP_MODULE;
<<<<<<< HEAD
<<<<<<< HEAD
   TIMAGEHLP_MODULE = record
=======
   TIMAGEHLP_MODULE = packed record
>>>>>>> graemeg/fixes_2_2
=======
   TIMAGEHLP_MODULE = packed record
>>>>>>> origin/fixes_2_2
          SizeOfStruct : dword;
          BaseOfImage : dword;
          ImageSize : dword;
          TimeDateStamp : dword;
          CheckSum : dword;
          NumSyms : dword;
          SymType : TSYM_TYPE;
          ModuleName : array[0..31] of TCHAR;
          ImageName : array[0..255] of TCHAR;
          LoadedImageName : array[0..255] of TCHAR;
       end;
   IMAGEHLP_MODULE = TIMAGEHLP_MODULE;
   LPIMAGEHLP_MODULE = PIMAGEHLP_MODULE;
   
   PIMAGEHLP_MODULEW = ^TIMAGEHLP_MODULEW;
<<<<<<< HEAD
<<<<<<< HEAD
   TIMAGEHLP_MODULEW = record
=======
   TIMAGEHLP_MODULEW = packed record
>>>>>>> graemeg/fixes_2_2
=======
   TIMAGEHLP_MODULEW = packed record
>>>>>>> origin/fixes_2_2
          SizeOfStruct : dword;
          BaseOfImage : dword;
          ImageSize : dword;
          TimeDateStamp : dword;
          CheckSum : dword;
          NumSyms : dword;
          SymType : TSYM_TYPE;
          ModuleName : array[0..31] of wchar;
          ImageName : array[0..255] of wchar;
          LoadedImageName : array[0..255] of wchar;
       end;
   IMAGEHLP_MODULEW = TIMAGEHLP_MODULEW;
   LPIMAGEHLP_MODULEW = PIMAGEHLP_MODULEW;
  {$ENDIF}
  
   PIMAGEHLP_LINE64 = ^TIMAGEHLP_LINE64;
<<<<<<< HEAD
<<<<<<< HEAD
   TIMAGEHLP_LINE64 = record
=======
   TIMAGEHLP_LINE64 = packed record
>>>>>>> graemeg/fixes_2_2
=======
   TIMAGEHLP_LINE64 = packed record
>>>>>>> origin/fixes_2_2
          SizeOfStruct : dword;
          Key : pointer;
          LineNumber : dword;
          FileName : pchar;
          Address : dword64;
       end;
   IMAGEHLP_LINE64 = TIMAGEHLP_LINE64;
   LPIMAGEHLP_LINE64 = PIMAGEHLP_LINE64;
	 
{$ifdef IMAGEHLP64}
     IMAGEHLP_LINE = IMAGEHLP_LINE64;     
     TIMAGEHLP_LINE = IMAGEHLP_LINE64;     	 
     PIMAGEHLP_LINE = PIMAGEHLP_LINE64;     
	 LPIMAGEHLP_LINE = PIMAGEHLP_LINE64;     
{$else}
     PIMAGEHLP_LINE = ^TIMAGEHLP_LINE;
<<<<<<< HEAD
<<<<<<< HEAD
     TIMAGEHLP_LINE = record
=======
     TIMAGEHLP_LINE = packed record
>>>>>>> graemeg/fixes_2_2
=======
     TIMAGEHLP_LINE = packed record
>>>>>>> origin/fixes_2_2
          SizeOfStruct : dword;
          Key : pointer;
          LineNumber : dword;
          FileName : pchar;
          Address : dword;
       end;
     IMAGEHLP_LINE   = TIMAGEHLP_LINE;
     LPIMAGEHLP_LINE = PIMAGEHLP_LINE;
{$endif}
  
  PSOURCEFILE = ^TSOURCEFILE;
<<<<<<< HEAD
<<<<<<< HEAD
  TSOURCEFILE = record
=======
  TSOURCEFILE = packed record
>>>>>>> graemeg/fixes_2_2
=======
  TSOURCEFILE = packed record
>>>>>>> origin/fixes_2_2
          ModBase : dword64;
          FileName : pchar;
       end;  
  
  PIMAGEHLP_CBA_READ_MEMORY = ^TIMAGEHLP_CBA_READ_MEMORY;
<<<<<<< HEAD
<<<<<<< HEAD
  TIMAGEHLP_CBA_READ_MEMORY = record
=======
  TIMAGEHLP_CBA_READ_MEMORY = packed record
>>>>>>> graemeg/fixes_2_2
=======
  TIMAGEHLP_CBA_READ_MEMORY = packed record
>>>>>>> origin/fixes_2_2
          addr : dword64;
          buf : pointer;
          bytes : dword;
          bytesread : PDWORD;
       end;
  IMAGEHLP_CBA_READ_MEMORY = TIMAGEHLP_CBA_READ_MEMORY;
  LPIMAGEHLP_CBA_READ_MEMORY = PIMAGEHLP_CBA_READ_MEMORY;
   
  PIMAGEHLP_CBA_EVENT = ^TIMAGEHLP_CBA_EVENT;
<<<<<<< HEAD
<<<<<<< HEAD
  TIMAGEHLP_CBA_EVENT = record
=======
  TIMAGEHLP_CBA_EVENT = packed record
>>>>>>> graemeg/fixes_2_2
=======
  TIMAGEHLP_CBA_EVENT = packed record
>>>>>>> origin/fixes_2_2
          severity : dword;
          code : dword;
          desc : pchar;
          _object : pointer;
       end;
  IMAGEHLP_CBA_EVENT = TIMAGEHLP_CBA_EVENT;
  LPIMAGEHLP_CBA_EVENT = PIMAGEHLP_CBA_EVENT;
    
  PIMAGEHLP_DEFERRED_SYMBOL_LOAD64 = ^TIMAGEHLP_DEFERRED_SYMBOL_LOAD64;
<<<<<<< HEAD
<<<<<<< HEAD
  TIMAGEHLP_DEFERRED_SYMBOL_LOAD64 = record
=======
  TIMAGEHLP_DEFERRED_SYMBOL_LOAD64 = packed record
>>>>>>> graemeg/fixes_2_2
=======
  TIMAGEHLP_DEFERRED_SYMBOL_LOAD64 = packed record
>>>>>>> origin/fixes_2_2
          SizeOfStruct : dword;
          BaseOfImage : dword64;
          CheckSum : dword;
          TimeDateStamp : dword;
          FileName : array[0..(MAX_PATH)-1] of TCHAR;
          Reparse : bytebool;
          hFile : THANDLE;
          Flags : dword;
       end;
  IMAGEHLP_DEFERRED_SYMBOL_LOAD64 = TIMAGEHLP_DEFERRED_SYMBOL_LOAD64;
  LPIMAGEHLP_DEFERRED_SYMBOL_LOAD64 = PIMAGEHLP_DEFERRED_SYMBOL_LOAD64;
     
{$ifdef IMAGEHLP64}
  IMAGEHLP_DEFERRED_SYMBOL_LOAD = IMAGEHLP_DEFERRED_SYMBOL_LOAD64;     
  TIMAGEHLP_DEFERRED_SYMBOL_LOAD = IMAGEHLP_DEFERRED_SYMBOL_LOAD64;       
  PIMAGEHLP_DEFERRED_SYMBOL_LOAD = PIMAGEHLP_DEFERRED_SYMBOL_LOAD64;     
{$else}
  PIMAGEHLP_DEFERRED_SYMBOL_LOAD = ^TIMAGEHLP_DEFERRED_SYMBOL_LOAD;
<<<<<<< HEAD
<<<<<<< HEAD
  TIMAGEHLP_DEFERRED_SYMBOL_LOAD = record
=======
  TIMAGEHLP_DEFERRED_SYMBOL_LOAD = packed record
>>>>>>> graemeg/fixes_2_2
=======
  TIMAGEHLP_DEFERRED_SYMBOL_LOAD = packed record
>>>>>>> origin/fixes_2_2
          SizeOfStruct : dword;
          BaseOfImage : dword;
          CheckSum : dword;
          TimeDateStamp : dword;
          FileName : array[0..(MAX_PATH)-1] of TCHAR;
          Reparse : bytebool;
          hFile : THANDLE;
       end;
  IMAGEHLP_DEFERRED_SYMBOL_LOAD = TIMAGEHLP_DEFERRED_SYMBOL_LOAD;
  LPIMAGEHLP_DEFERRED_SYMBOL_LOAD = PIMAGEHLP_DEFERRED_SYMBOL_LOAD;
{$endif}

  PIMAGEHLP_DUPLICATE_SYMBOL64 = ^TIMAGEHLP_DUPLICATE_SYMBOL64;
<<<<<<< HEAD
<<<<<<< HEAD
  TIMAGEHLP_DUPLICATE_SYMBOL64 = record
=======
  TIMAGEHLP_DUPLICATE_SYMBOL64 = packed record
>>>>>>> graemeg/fixes_2_2
=======
  TIMAGEHLP_DUPLICATE_SYMBOL64 = packed record
>>>>>>> origin/fixes_2_2
          SizeOfStruct : dword;
          NumberOfDups : dword;
          Symbol : PIMAGEHLP_SYMBOL64;
          SelectedSymbol : dword;
       end;
  IMAGEHLP_DUPLICATE_SYMBOL64   = TIMAGEHLP_DUPLICATE_SYMBOL64;
  LPIMAGEHLP_DUPLICATE_SYMBOL64 = PIMAGEHLP_DUPLICATE_SYMBOL64;
{$ifdef IMAGEHLP64}
  IMAGEHLP_DUPLICATE_SYMBOL   = IMAGEHLP_DUPLICATE_SYMBOL64;     
  PIMAGEHLP_DUPLICATE_SYMBOL  = PIMAGEHLP_DUPLICATE_SYMBOL64;     
  TIMAGEHLP_DUPLICATE_SYMBOL  = IMAGEHLP_DUPLICATE_SYMBOL64;     
  LPIMAGEHLP_DUPLICATE_SYMBOL = PIMAGEHLP_DUPLICATE_SYMBOL64;       
{$else}
  PIMAGEHLP_DUPLICATE_SYMBOL = ^TIMAGEHLP_DUPLICATE_SYMBOL;
<<<<<<< HEAD
<<<<<<< HEAD
  TIMAGEHLP_DUPLICATE_SYMBOL = record
=======
  TIMAGEHLP_DUPLICATE_SYMBOL = packed record
>>>>>>> graemeg/fixes_2_2
=======
  TIMAGEHLP_DUPLICATE_SYMBOL = packed record
>>>>>>> origin/fixes_2_2
          SizeOfStruct : dword;
          NumberOfDups : dword;
          Symbol : PIMAGEHLP_SYMBOL;
          SelectedSymbol : dword;
       end;
  IMAGEHLP_DUPLICATE_SYMBOL = TIMAGEHLP_DUPLICATE_SYMBOL;
  LPIMAGEHLP_DUPLICATE_SYMBOL = PIMAGEHLP_DUPLICATE_SYMBOL;
{$endif}
  PSRCCODEINFO = ^TSRCCODEINFO;
<<<<<<< HEAD
<<<<<<< HEAD
  TSRCCODEINFO = record
=======
  TSRCCODEINFO = packed record
>>>>>>> graemeg/fixes_2_2
=======
  TSRCCODEINFO = packed record
>>>>>>> origin/fixes_2_2
          SizeOfStruct : dword;
          Key : pointer;
          ModBase : dword64;
          Obj : array[0..(MAX_PATH+1)-1] of TCHAR;
          FileName : array[0..(MAX_PATH+1)-1] of TCHAR;
          LineNumber : dword;
          Address : dword64;
       end;
  SRCCODEINFO = TSRCCODEINFO;
  LPSRCCODEINFO = PSRCCODEINFO;

  PIMAGEHLP_SYMBOL_SRC = ^TIMAGEHLP_SYMBOL_SRC;
<<<<<<< HEAD
<<<<<<< HEAD
  TIMAGEHLP_SYMBOL_SRC = record
=======
  TIMAGEHLP_SYMBOL_SRC = packed record
>>>>>>> graemeg/fixes_2_2
=======
  TIMAGEHLP_SYMBOL_SRC = packed record
>>>>>>> origin/fixes_2_2
          sizeofstruct : dword;
          _type : dword;
          _file : array[0..(MAX_PATH)-1] of char;
       end;
  IMAGEHLP_SYMBOL_SRC = TIMAGEHLP_SYMBOL_SRC;
  LPIMAGEHLP_SYMBOL_SRC = PIMAGEHLP_SYMBOL_SRC;
  
  PMODULE_TYPE_INFO = ^TMODULE_TYPE_INFO;
<<<<<<< HEAD
<<<<<<< HEAD
  TMODULE_TYPE_INFO = record
=======
  TMODULE_TYPE_INFO = packed record
>>>>>>> graemeg/fixes_2_2
=======
  TMODULE_TYPE_INFO = packed record
>>>>>>> origin/fixes_2_2
          dataLength : USHORT;
          leaf : USHORT;
          data : array[0..0] of TBYTE;
       end;
  MODULE_TYPE_INFO = TMODULE_TYPE_INFO;
  LPMODULE_TYPE_INFO = PMODULE_TYPE_INFO;
  
  PSYMBOL_INFO = ^TSYMBOL_INFO;
<<<<<<< HEAD
<<<<<<< HEAD
  TSYMBOL_INFO = record
=======
  TSYMBOL_INFO = packed record
>>>>>>> graemeg/fixes_2_2
=======
  TSYMBOL_INFO = packed record
>>>>>>> origin/fixes_2_2
          SizeOfStruct : ULONG;
          TypeIndex : ULONG;
          Reserved : array[0..1] of ULONG64;
          info : ULONG;
          Size : ULONG;
          ModBase : ULONG64;
          Flags : ULONG;
          Value : ULONG64;
          Address : ULONG64;
          Register : ULONG;
          Scope : ULONG;
          Tag : ULONG;
          NameLen : ULONG;
          MaxNameLen : ULONG;
          Name : array[0..0] of TCHAR;
       end;
  SYMBOL_INFO = TSYMBOL_INFO;
  LPSYMBOL_INFO = PSYMBOL_INFO;

  PSYMBOL_INFO_PACKAGE = ^TSYMBOL_INFO_PACKAGE;
<<<<<<< HEAD
<<<<<<< HEAD
  TSYMBOL_INFO_PACKAGE = record
=======
  TSYMBOL_INFO_PACKAGE = packed record
>>>>>>> graemeg/fixes_2_2
=======
  TSYMBOL_INFO_PACKAGE = packed record
>>>>>>> origin/fixes_2_2
          si : TSYMBOL_INFO;
          name : array[0..(MAX_SYM_NAME+1)-1] of TCHAR;
       end;
  SYMBOL_INFO_PACKAGE = TSYMBOL_INFO_PACKAGE;
  LPSYMBOL_INFO_PACKAGE = PSYMBOL_INFO_PACKAGE;

  PIMAGEHLP_STACK_FRAME = ^TIMAGEHLP_STACK_FRAME;
<<<<<<< HEAD
<<<<<<< HEAD
  TIMAGEHLP_STACK_FRAME = record
=======
  TIMAGEHLP_STACK_FRAME = packed record
>>>>>>> graemeg/fixes_2_2
=======
  TIMAGEHLP_STACK_FRAME = packed record
>>>>>>> origin/fixes_2_2
          InstructionOffset : ULONG64;
          ReturnOffset : ULONG64;
          FrameOffset : ULONG64;
          StackOffset : ULONG64;
          BackingStoreOffset : ULONG64;
          FuncTableEntry : ULONG64;
          Params : array[0..3] of ULONG64;
          Reserved : array[0..4] of ULONG64;
          Virtual : BOOL;
          Reserved2 : ULONG;
       end;
  IMAGEHLP_STACK_FRAME = TIMAGEHLP_STACK_FRAME;
  LPIMAGEHLP_STACK_FRAME = PIMAGEHLP_STACK_FRAME;

  PIMAGEHLP_CONTEXT = ^TIMAGEHLP_CONTEXT;
  TIMAGEHLP_CONTEXT = pointer;
  PPIMAGEHLP_CONTEXT = ^PIMAGEHLP_CONTEXT;
  
  PTI_FINDCHILDREN_PARAMS = ^TTI_FINDCHILDREN_PARAMS;
<<<<<<< HEAD
<<<<<<< HEAD
  TTI_FINDCHILDREN_PARAMS = record
=======
  TTI_FINDCHILDREN_PARAMS = packed record
>>>>>>> graemeg/fixes_2_2
=======
  TTI_FINDCHILDREN_PARAMS = packed record
>>>>>>> origin/fixes_2_2
          Count : ULONG;
          Start : ULONG;
          ChildId : array[0..0] of ULONG;
       end;
  PMINIDUMP_LOCATION_DESCRIPTOR = ^TMINIDUMP_LOCATION_DESCRIPTOR;
<<<<<<< HEAD
<<<<<<< HEAD
  TMINIDUMP_LOCATION_DESCRIPTOR = record
=======
  TMINIDUMP_LOCATION_DESCRIPTOR = packed record
>>>>>>> graemeg/fixes_2_2
=======
  TMINIDUMP_LOCATION_DESCRIPTOR = packed record
>>>>>>> origin/fixes_2_2
          DataSize : ULONG32;
          Rva : TRVA;
       end;
  MINIDUMP_LOCATION_DESCRIPTOR = TMINIDUMP_LOCATION_DESCRIPTOR ;

  PMINIDUMP_LOCATION_DESCRIPTOR64 = ^TMINIDUMP_LOCATION_DESCRIPTOR64;
<<<<<<< HEAD
<<<<<<< HEAD
  TMINIDUMP_LOCATION_DESCRIPTOR64 = record
=======
  TMINIDUMP_LOCATION_DESCRIPTOR64 = packed record
>>>>>>> graemeg/fixes_2_2
=======
  TMINIDUMP_LOCATION_DESCRIPTOR64 = packed record
>>>>>>> origin/fixes_2_2
          DataSize : ULONG64;
          Rva : TRVA64;
       end;
  MINIDUMP_LOCATION_DESCRIPTOR64 =   TMINIDUMP_LOCATION_DESCRIPTOR64;
  
  PMINIDUMP_MEMORY_DESCRIPTOR = ^TMINIDUMP_MEMORY_DESCRIPTOR;
<<<<<<< HEAD
<<<<<<< HEAD
  TMINIDUMP_MEMORY_DESCRIPTOR = record
=======
  TMINIDUMP_MEMORY_DESCRIPTOR = packed record
>>>>>>> graemeg/fixes_2_2
=======
  TMINIDUMP_MEMORY_DESCRIPTOR = packed record
>>>>>>> origin/fixes_2_2
          StartOfMemoryRange : ULONG64;
          Memory : TMINIDUMP_LOCATION_DESCRIPTOR;
       end;
  MINIDUMP_MEMORY_DESCRIPTOR = TMINIDUMP_MEMORY_DESCRIPTOR;
  LPMINIDUMP_MEMORY_DESCRIPTOR = PMINIDUMP_MEMORY_DESCRIPTOR;
  
  PMINIDUMP_MEMORY_DESCRIPTOR64 = ^TMINIDUMP_MEMORY_DESCRIPTOR64;
<<<<<<< HEAD
<<<<<<< HEAD
  TMINIDUMP_MEMORY_DESCRIPTOR64 = record
=======
  TMINIDUMP_MEMORY_DESCRIPTOR64 = packed record
>>>>>>> graemeg/fixes_2_2
=======
  TMINIDUMP_MEMORY_DESCRIPTOR64 = packed record
>>>>>>> origin/fixes_2_2
          StartOfMemoryRange : ULONG64;
          DataSize : ULONG64;
       end;
  MINIDUMP_MEMORY_DESCRIPTOR64 = TMINIDUMP_MEMORY_DESCRIPTOR64;
  LPMINIDUMP_MEMORY_DESCRIPTOR64 = PMINIDUMP_MEMORY_DESCRIPTOR64;

  PMINIDUMP_HEADER = ^TMINIDUMP_HEADER;
<<<<<<< HEAD
<<<<<<< HEAD
  TMINIDUMP_HEADER = record
=======
  TMINIDUMP_HEADER = packed record
>>>>>>> graemeg/fixes_2_2
=======
  TMINIDUMP_HEADER = packed record
>>>>>>> origin/fixes_2_2
          Signature : ULONG32;
          Version : ULONG32;
          NumberOfStreams : ULONG32;
          StreamDirectoryRva : TRVA;
          CheckSum : ULONG32;
          a : packed record
              case longint of
                 0 : ( Reserved : ULONG32 );
                 1 : ( TimeDateStamp : ULONG32 );
              end;
          Flags : ULONG64;
       end;
  MINIDUMP_HEADER = TMINIDUMP_HEADER;
  LPMINIDUMP_HEADER = PMINIDUMP_HEADER;
  
  PMINIDUMP_DIRECTORY = ^TMINIDUMP_DIRECTORY;
<<<<<<< HEAD
<<<<<<< HEAD
  TMINIDUMP_DIRECTORY = record
=======
  TMINIDUMP_DIRECTORY = packed record
>>>>>>> graemeg/fixes_2_2
=======
  TMINIDUMP_DIRECTORY = packed record
>>>>>>> origin/fixes_2_2
          StreamType : ULONG32;
          Location : TMINIDUMP_LOCATION_DESCRIPTOR;
       end;
  MINIDUMP_DIRECTORY = TMINIDUMP_DIRECTORY;
  LPMINIDUMP_DIRECTORY = PMINIDUMP_DIRECTORY;
  PPMINIDUMP_DIRECTORY = ^PMINIDUMP_DIRECTORY;
  
  PMINIDUMP_STRING = ^TMINIDUMP_STRING;
<<<<<<< HEAD
<<<<<<< HEAD
  TMINIDUMP_STRING = record
=======
  TMINIDUMP_STRING = packed record
>>>>>>> graemeg/fixes_2_2
=======
  TMINIDUMP_STRING = packed record
>>>>>>> origin/fixes_2_2
          Length : ULONG32;
          Buffer : array[0..0] of WCHAR;
       end;
  MINIDUMP_STRING = TMINIDUMP_STRING;
  LPMINIDUMP_STRING = PMINIDUMP_STRING;
  
  
  PCPU_INFORMATION = ^TCPU_INFORMATION;
<<<<<<< HEAD
<<<<<<< HEAD
  TCPU_INFORMATION = record
=======
  TCPU_INFORMATION = packed record
>>>>>>> graemeg/fixes_2_2
=======
  TCPU_INFORMATION = packed record
>>>>>>> origin/fixes_2_2
         case longint of
            0 : ( X86CpuInfo : packed record
                 VendorId : array[0..2] of ULONG32;
                 VersionInformation : ULONG32;
                 FeatureInformation : ULONG32;
                 AMDExtendedCpuFeatures : ULONG32;
              end );
            1 : ( OtherCpuInfo : packed record
                 ProcessorFeatures : array[0..1] of ULONG64;
              end );
         end;
  CPU_INFORMATION = TCPU_INFORMATION;
  LPCPU_INFORMATION = PCPU_INFORMATION;
  
  PMINIDUMP_SYSTEM_INFO = ^TMINIDUMP_SYSTEM_INFO;
<<<<<<< HEAD
<<<<<<< HEAD
  TMINIDUMP_SYSTEM_INFO = record
=======
  TMINIDUMP_SYSTEM_INFO = packed record
>>>>>>> graemeg/fixes_2_2
=======
  TMINIDUMP_SYSTEM_INFO = packed record
>>>>>>> origin/fixes_2_2
          ProcessorArchitecture : USHORT;
          ProcessorLevel : USHORT;
          ProcessorRevision : USHORT;
          c : packed record
              case longint of
                 0 : ( Reserved0 : USHORT );
                 1 : ( b : packed record
                      NumberOfProcessors : UCHAR;
                      ProductType : UCHAR;
                   end );
              end;
          MajorVersion : ULONG32;
          MinorVersion : ULONG32;
          BuildNumber : ULONG32;
          PlatformId : ULONG32;
          CSDVersionRva : TRVA;
          e : packed record
              case longint of
                 0 : ( Reserved1 : ULONG32 );
                 1 : ( d : packed record
                      SuiteMask : USHORT;
                      Reserved2 : USHORT;
                   end );
              end;
          Cpu : TCPU_INFORMATION;
       end;
  MINIDUMP_SYSTEM_INFO = TMINIDUMP_SYSTEM_INFO;
  LPMINIDUMP_SYSTEM_INFO = PMINIDUMP_SYSTEM_INFO;
  
  PMINIDUMP_THREAD = ^TMINIDUMP_THREAD;
<<<<<<< HEAD
<<<<<<< HEAD
  TMINIDUMP_THREAD = record
=======
  TMINIDUMP_THREAD = packed record
>>>>>>> graemeg/fixes_2_2
=======
  TMINIDUMP_THREAD = packed record
>>>>>>> origin/fixes_2_2
          ThreadId : ULONG32;
          SuspendCount : ULONG32;
          PriorityClass : ULONG32;
          Priority : ULONG32;
          Teb : ULONG64;
          Stack : TMINIDUMP_MEMORY_DESCRIPTOR;
          ThreadContext : TMINIDUMP_LOCATION_DESCRIPTOR;
       end;
  MINIDUMP_THREAD = TMINIDUMP_THREAD;
  LPMINIDUMP_THREAD = PMINIDUMP_THREAD;
  
  PMINIDUMP_THREAD_LIST = ^TMINIDUMP_THREAD_LIST;
<<<<<<< HEAD
<<<<<<< HEAD
  TMINIDUMP_THREAD_LIST = record
=======
  TMINIDUMP_THREAD_LIST = packed record
>>>>>>> graemeg/fixes_2_2
=======
  TMINIDUMP_THREAD_LIST = packed record
>>>>>>> origin/fixes_2_2
          NumberOfThreads : ULONG32;
          Threads : array[0..0] of TMINIDUMP_THREAD;
       end;
  MINIDUMP_THREAD_LIST = TMINIDUMP_THREAD_LIST;
  LPMINIDUMP_THREAD_LIST = PMINIDUMP_THREAD_LIST;

  PMINIDUMP_THREAD_EX = ^TMINIDUMP_THREAD_EX;
<<<<<<< HEAD
<<<<<<< HEAD
  TMINIDUMP_THREAD_EX = record
=======
  TMINIDUMP_THREAD_EX = packed record
>>>>>>> graemeg/fixes_2_2
=======
  TMINIDUMP_THREAD_EX = packed record
>>>>>>> origin/fixes_2_2
          ThreadId : ULONG32;
          SuspendCount : ULONG32;
          PriorityClass : ULONG32;
          Priority : ULONG32;
          Teb : ULONG64;
          Stack : TMINIDUMP_MEMORY_DESCRIPTOR;
          ThreadContext : TMINIDUMP_LOCATION_DESCRIPTOR;
          BackingStore : TMINIDUMP_MEMORY_DESCRIPTOR;
       end;
  MINIDUMP_THREAD_EX = PMINIDUMP_THREAD_EX;
  LPMINIDUMP_THREAD_EX = PMINIDUMP_THREAD_EX;
  

  PMINIDUMP_THREAD_EX_LIST = ^TMINIDUMP_THREAD_EX_LIST;
<<<<<<< HEAD
<<<<<<< HEAD
  TMINIDUMP_THREAD_EX_LIST = record
=======
  TMINIDUMP_THREAD_EX_LIST = packed record
>>>>>>> graemeg/fixes_2_2
=======
  TMINIDUMP_THREAD_EX_LIST = packed record
>>>>>>> origin/fixes_2_2
          NumberOfThreads : ULONG32;
          Threads : array[0..0] of TMINIDUMP_THREAD_EX;
       end;
  MINIDUMP_THREAD_EX_LIST = TMINIDUMP_THREAD_EX_LIST;
  LPMINIDUMP_THREAD_EX_LIST = PMINIDUMP_THREAD_EX_LIST;
 

  PMINIDUMP_EXCEPTION = ^TMINIDUMP_EXCEPTION;
<<<<<<< HEAD
<<<<<<< HEAD
  TMINIDUMP_EXCEPTION = record
=======
  TMINIDUMP_EXCEPTION = packed record
>>>>>>> graemeg/fixes_2_2
=======
  TMINIDUMP_EXCEPTION = packed record
>>>>>>> origin/fixes_2_2
          ExceptionCode : ULONG32;
          ExceptionFlags : ULONG32;
          ExceptionRecord : ULONG64;
          ExceptionAddress : ULONG64;
          NumberParameters : ULONG32;
          __unusedAlignment : ULONG32;
          ExceptionInformation : array[0..(EXCEPTION_MAXIMUM_PARAMETERS)-1] of ULONG64;
       end;
  MINIDUMP_EXCEPTION = TMINIDUMP_EXCEPTION;
  LPMINIDUMP_EXCEPTION = PMINIDUMP_EXCEPTION;
   

  PMINIDUMP_EXCEPTION_STREAM = ^TMINIDUMP_EXCEPTION_STREAM;
<<<<<<< HEAD
<<<<<<< HEAD
  TMINIDUMP_EXCEPTION_STREAM = record
=======
  TMINIDUMP_EXCEPTION_STREAM = packed record
>>>>>>> graemeg/fixes_2_2
=======
  TMINIDUMP_EXCEPTION_STREAM = packed record
>>>>>>> origin/fixes_2_2
          ThreadId : ULONG32;
          __alignment : ULONG32;
          ExceptionRecord : TMINIDUMP_EXCEPTION;
          ThreadContext : TMINIDUMP_LOCATION_DESCRIPTOR;
       end;
  MINIDUMP_EXCEPTION_STREAM = TMINIDUMP_EXCEPTION_STREAM;
  LPMINIDUMP_EXCEPTION_STREAM = PMINIDUMP_EXCEPTION_STREAM;
  
  PMINIDUMP_MODULE = ^TMINIDUMP_MODULE;
<<<<<<< HEAD
<<<<<<< HEAD
  TMINIDUMP_MODULE = record
=======
  TMINIDUMP_MODULE = packed record
>>>>>>> graemeg/fixes_2_2
=======
  TMINIDUMP_MODULE = packed record
>>>>>>> origin/fixes_2_2
          BaseOfImage : ULONG64;
          SizeOfImage : ULONG32;
          CheckSum : ULONG32;
          TimeDateStamp : ULONG32;
          ModuleNameRva : TRVA;
          VersionInfo : TVSFIXEDFILEINFO;
          CvRecord : TMINIDUMP_LOCATION_DESCRIPTOR;
          MiscRecord : TMINIDUMP_LOCATION_DESCRIPTOR;
          Reserved0 : ULONG64;
          Reserved1 : ULONG64;
       end;
  MINIDUMP_MODULE = TMINIDUMP_MODULE;
  LPMINIDUMP_MODULE = PMINIDUMP_MODULE;
  

  PMINIDUMP_MODULE_LIST = ^TMINIDUMP_MODULE_LIST;
<<<<<<< HEAD
<<<<<<< HEAD
  TMINIDUMP_MODULE_LIST = record
=======
  TMINIDUMP_MODULE_LIST = packed record
>>>>>>> graemeg/fixes_2_2
=======
  TMINIDUMP_MODULE_LIST = packed record
>>>>>>> origin/fixes_2_2
          NumberOfModules : ULONG32;
          Modules : array[0..0] of TMINIDUMP_MODULE;
       end;
  MINIDUMP_MODULE_LIST = TMINIDUMP_MODULE_LIST;
  LPMINIDUMP_MODULE_LIST = PMINIDUMP_MODULE_LIST;
   

  PMINIDUMP_MEMORY_LIST = ^TMINIDUMP_MEMORY_LIST;
<<<<<<< HEAD
<<<<<<< HEAD
  TMINIDUMP_MEMORY_LIST = record
=======
  TMINIDUMP_MEMORY_LIST = packed record
>>>>>>> graemeg/fixes_2_2
=======
  TMINIDUMP_MEMORY_LIST = packed record
>>>>>>> origin/fixes_2_2
          NumberOfMemoryRanges : ULONG32;
          MemoryRanges : array[0..0] of TMINIDUMP_MEMORY_DESCRIPTOR;
       end;
  MINIDUMP_MEMORY_LIST = TMINIDUMP_MEMORY_LIST;
  LPMINIDUMP_MEMORY_LIST = PMINIDUMP_MEMORY_LIST;

  PMINIDUMP_MEMORY64_LIST = ^TMINIDUMP_MEMORY64_LIST;
<<<<<<< HEAD
<<<<<<< HEAD
  TMINIDUMP_MEMORY64_LIST = record
=======
  TMINIDUMP_MEMORY64_LIST = packed record
>>>>>>> graemeg/fixes_2_2
=======
  TMINIDUMP_MEMORY64_LIST = packed record
>>>>>>> origin/fixes_2_2
          NumberOfMemoryRanges : ULONG64;
          BaseRva : TRVA64;
          MemoryRanges : array[0..0] of TMINIDUMP_MEMORY_DESCRIPTOR64;
       end;
  MINIDUMP_MEMORY64_LIST = TMINIDUMP_MEMORY64_LIST;
  LPMINIDUMP_MEMORY64_LIST = PMINIDUMP_MEMORY64_LIST;
  
  PMINIDUMP_EXCEPTION_INFORMATION = ^TMINIDUMP_EXCEPTION_INFORMATION;
<<<<<<< HEAD
<<<<<<< HEAD
  TMINIDUMP_EXCEPTION_INFORMATION = record
=======
  TMINIDUMP_EXCEPTION_INFORMATION = packed record
>>>>>>> graemeg/fixes_2_2
=======
  TMINIDUMP_EXCEPTION_INFORMATION = packed record
>>>>>>> origin/fixes_2_2
          ThreadId : dword;
          ExceptionPointers : PEXCEPTION_POINTERS;
          ClientPointers : BOOL;
       end;
  MINIDUMP_EXCEPTION_INFORMATION = TMINIDUMP_EXCEPTION_INFORMATION;
  LPMINIDUMP_EXCEPTION_INFORMATION = PMINIDUMP_EXCEPTION_INFORMATION;

  PMINIDUMP_EXCEPTION_INFORMATION64 = ^TMINIDUMP_EXCEPTION_INFORMATION64;
<<<<<<< HEAD
<<<<<<< HEAD
  TMINIDUMP_EXCEPTION_INFORMATION64 = record
=======
  TMINIDUMP_EXCEPTION_INFORMATION64 = packed record
>>>>>>> graemeg/fixes_2_2
=======
  TMINIDUMP_EXCEPTION_INFORMATION64 = packed record
>>>>>>> origin/fixes_2_2
          ThreadId : dword;
          ExceptionRecord : ULONG64;
          ContextRecord : ULONG64;
          ClientPointers : BOOL;
       end;
  MINIDUMP_EXCEPTION_INFORMATION64 = TMINIDUMP_EXCEPTION_INFORMATION64;
  LPMINIDUMP_EXCEPTION_INFORMATION64 = PMINIDUMP_EXCEPTION_INFORMATION64;
  
  PMINIDUMP_HANDLE_DESCRIPTOR = ^TMINIDUMP_HANDLE_DESCRIPTOR;
<<<<<<< HEAD
<<<<<<< HEAD
  TMINIDUMP_HANDLE_DESCRIPTOR = record
=======
  TMINIDUMP_HANDLE_DESCRIPTOR = packed record
>>>>>>> graemeg/fixes_2_2
=======
  TMINIDUMP_HANDLE_DESCRIPTOR = packed record
>>>>>>> origin/fixes_2_2
          Handle : ULONG64;
          TypeNameRva : TRVA;
          ObjectNameRva : TRVA;
          Attributes : ULONG32;
          GrantedAccess : ULONG32;
          HandleCount : ULONG32;
          PointerCount : ULONG32;
       end;
  MINIDUMP_HANDLE_DESCRIPTOR = TMINIDUMP_HANDLE_DESCRIPTOR;
  LPMINIDUMP_HANDLE_DESCRIPTOR = PMINIDUMP_HANDLE_DESCRIPTOR;

  PMINIDUMP_HANDLE_DATA_STREAM = ^TMINIDUMP_HANDLE_DATA_STREAM;
<<<<<<< HEAD
<<<<<<< HEAD
  TMINIDUMP_HANDLE_DATA_STREAM = record
=======
  TMINIDUMP_HANDLE_DATA_STREAM = packed record
>>>>>>> graemeg/fixes_2_2
=======
  TMINIDUMP_HANDLE_DATA_STREAM = packed record
>>>>>>> origin/fixes_2_2
          SizeOfHeader : ULONG32;
          SizeOfDescriptor : ULONG32;
          NumberOfDescriptors : ULONG32;
          Reserved : ULONG32;
       end;
  MINIDUMP_HANDLE_DATA_STREAM = TMINIDUMP_HANDLE_DATA_STREAM;
  LPMINIDUMP_HANDLE_DATA_STREAM = PMINIDUMP_HANDLE_DATA_STREAM;
  

  PMINIDUMP_FUNCTION_TABLE_DESCRIPTOR = ^TMINIDUMP_FUNCTION_TABLE_DESCRIPTOR;
<<<<<<< HEAD
<<<<<<< HEAD
  TMINIDUMP_FUNCTION_TABLE_DESCRIPTOR = record
=======
  TMINIDUMP_FUNCTION_TABLE_DESCRIPTOR = packed record
>>>>>>> graemeg/fixes_2_2
=======
  TMINIDUMP_FUNCTION_TABLE_DESCRIPTOR = packed record
>>>>>>> origin/fixes_2_2
          MinimumAddress : ULONG64;
          MaximumAddress : ULONG64;
          BaseAddress : ULONG64;
          EntryCount : ULONG32;
          SizeOfAlignPad : ULONG32;
       end;
  MINIDUMP_FUNCTION_TABLE_DESCRIPTOR = TMINIDUMP_FUNCTION_TABLE_DESCRIPTOR;
  LPMINIDUMP_FUNCTION_TABLE_DESCRIPTOR = PMINIDUMP_FUNCTION_TABLE_DESCRIPTOR;

  PMINIDUMP_FUNCTION_TABLE_STREAM = ^TMINIDUMP_FUNCTION_TABLE_STREAM;
<<<<<<< HEAD
<<<<<<< HEAD
  TMINIDUMP_FUNCTION_TABLE_STREAM = record
=======
  TMINIDUMP_FUNCTION_TABLE_STREAM = packed record
>>>>>>> graemeg/fixes_2_2
=======
  TMINIDUMP_FUNCTION_TABLE_STREAM = packed record
>>>>>>> origin/fixes_2_2
          SizeOfHeader : ULONG32;
          SizeOfDescriptor : ULONG32;
          SizeOfNativeDescriptor : ULONG32;
          SizeOfFunctionEntry : ULONG32;
          NumberOfDescriptors : ULONG32;
          SizeOfAlignPad : ULONG32;
       end;
  MINIDUMP_FUNCTION_TABLE_STREAM = TMINIDUMP_FUNCTION_TABLE_STREAM;
  LPPMINIDUMP_FUNCTION_TABLE_STREAM = PMINIDUMP_FUNCTION_TABLE_STREAM;
  

  PMINIDUMP_UNLOADED_MODULE = ^TMINIDUMP_UNLOADED_MODULE;
<<<<<<< HEAD
<<<<<<< HEAD
  TMINIDUMP_UNLOADED_MODULE = record
=======
  TMINIDUMP_UNLOADED_MODULE = packed record
>>>>>>> graemeg/fixes_2_2
=======
  TMINIDUMP_UNLOADED_MODULE = packed record
>>>>>>> origin/fixes_2_2
          BaseOfImage : ULONG64;
          SizeOfImage : ULONG32;
          CheckSum : ULONG32;
          TimeDateStamp : ULONG32;
          ModuleNameRva : TRVA;
       end;
  MINIDUMP_UNLOADED_MODULE = TMINIDUMP_UNLOADED_MODULE;
  LPMINIDUMP_UNLOADED_MODULE = PMINIDUMP_UNLOADED_MODULE;
  
  PMINIDUMP_UNLOADED_MODULE_LIST = ^TMINIDUMP_UNLOADED_MODULE_LIST;
<<<<<<< HEAD
<<<<<<< HEAD
  TMINIDUMP_UNLOADED_MODULE_LIST = record
=======
  TMINIDUMP_UNLOADED_MODULE_LIST = packed record
>>>>>>> graemeg/fixes_2_2
=======
  TMINIDUMP_UNLOADED_MODULE_LIST = packed record
>>>>>>> origin/fixes_2_2
          SizeOfHeader : ULONG32;
          SizeOfEntry : ULONG32;
          NumberOfEntries : ULONG32;
       end;
  MINIDUMP_UNLOADED_MODULE_LIST = TMINIDUMP_UNLOADED_MODULE_LIST;
  LPMINIDUMP_UNLOADED_MODULE_LIST = PMINIDUMP_UNLOADED_MODULE_LIST;
  
  PMINIDUMP_MISC_INFO = ^TMINIDUMP_MISC_INFO;
<<<<<<< HEAD
<<<<<<< HEAD
  TMINIDUMP_MISC_INFO = record
=======
  TMINIDUMP_MISC_INFO = packed record
>>>>>>> graemeg/fixes_2_2
=======
  TMINIDUMP_MISC_INFO = packed record
>>>>>>> origin/fixes_2_2
          SizeOfInfo : ULONG32;
          Flags1 : ULONG32;
          ProcessId : ULONG32;
          ProcessCreateTime : ULONG32;
          ProcessUserTime : ULONG32;
          ProcessKernelTime : ULONG32;
       end;
  MINIDUMP_MISC_INFO = TMINIDUMP_MISC_INFO;
  LPMINIDUMP_MISC_INFO = PMINIDUMP_MISC_INFO;

  PMINIDUMP_USER_RECORD = ^TMINIDUMP_USER_RECORD;
<<<<<<< HEAD
<<<<<<< HEAD
  TMINIDUMP_USER_RECORD = record
=======
  TMINIDUMP_USER_RECORD = packed record
>>>>>>> graemeg/fixes_2_2
=======
  TMINIDUMP_USER_RECORD = packed record
>>>>>>> origin/fixes_2_2
          _Type : ULONG32;
          Memory : TMINIDUMP_LOCATION_DESCRIPTOR;
       end;
  MINIDUMP_USER_RECORD = TMINIDUMP_USER_RECORD;
  LPMINIDUMP_USER_RECORD = PMINIDUMP_USER_RECORD;

  PMINIDUMP_USER_STREAM = ^TMINIDUMP_USER_STREAM;
<<<<<<< HEAD
<<<<<<< HEAD
  TMINIDUMP_USER_STREAM = record
=======
  TMINIDUMP_USER_STREAM = packed record
>>>>>>> graemeg/fixes_2_2
=======
  TMINIDUMP_USER_STREAM = packed record
>>>>>>> origin/fixes_2_2
          _Type : ULONG32;
          BufferSize : ULONG;
          Buffer : pointer;
       end;
  MINIDUMP_USER_STREAM = TMINIDUMP_USER_STREAM;
  LPMINIDUMP_USER_STREAM = PMINIDUMP_USER_STREAM;

  PMINIDUMP_USER_STREAM_INFORMATION = ^TMINIDUMP_USER_STREAM_INFORMATION;
<<<<<<< HEAD
<<<<<<< HEAD
  TMINIDUMP_USER_STREAM_INFORMATION = record
=======
  TMINIDUMP_USER_STREAM_INFORMATION = packed record
>>>>>>> graemeg/fixes_2_2
=======
  TMINIDUMP_USER_STREAM_INFORMATION = packed record
>>>>>>> origin/fixes_2_2
          UserStreamCount : ULONG;
          UserStreamArray : PMINIDUMP_USER_STREAM;
       end;
  MINIDUMP_USER_STREAM_INFORMATION = TMINIDUMP_USER_STREAM_INFORMATION;
  LPMINIDUMP_USER_STREAM_INFORMATION = PMINIDUMP_USER_STREAM_INFORMATION;
  	 
  PMINIDUMP_THREAD_CALLBACK = ^TMINIDUMP_THREAD_CALLBACK;
<<<<<<< HEAD
<<<<<<< HEAD
  TMINIDUMP_THREAD_CALLBACK = record
=======
  TMINIDUMP_THREAD_CALLBACK = packed record
>>>>>>> graemeg/fixes_2_2
=======
  TMINIDUMP_THREAD_CALLBACK = packed record
>>>>>>> origin/fixes_2_2
          ThreadId : ULONG;
          ThreadHandle : THANDLE;
          Context : TCONTEXT;
          SizeOfContext : ULONG;
          StackBase : ULONG64;
          StackEnd : ULONG64;
       end;
  MINIDUMP_THREAD_CALLBACK = TMINIDUMP_THREAD_CALLBACK;
  LPMINIDUMP_THREAD_CALLBACK = PMINIDUMP_THREAD_CALLBACK;

  PMINIDUMP_THREAD_EX_CALLBACK = ^TMINIDUMP_THREAD_EX_CALLBACK;
<<<<<<< HEAD
<<<<<<< HEAD
  TMINIDUMP_THREAD_EX_CALLBACK = record
=======
  TMINIDUMP_THREAD_EX_CALLBACK = packed record
>>>>>>> graemeg/fixes_2_2
=======
  TMINIDUMP_THREAD_EX_CALLBACK = packed record
>>>>>>> origin/fixes_2_2
          ThreadId : ULONG;
          ThreadHandle : THANDLE;
          Context : TCONTEXT;
          SizeOfContext : ULONG;
          StackBase : ULONG64;
          StackEnd : ULONG64;
          BackingStoreBase : ULONG64;
          BackingStoreEnd : ULONG64;
       end;
  MINIDUMP_THREAD_EX_CALLBACK = TMINIDUMP_THREAD_EX_CALLBACK;
  LPMINIDUMP_THREAD_EX_CALLBACK = PMINIDUMP_THREAD_EX_CALLBACK;

  PMINIDUMP_INCLUDE_THREAD_CALLBACK = ^TMINIDUMP_INCLUDE_THREAD_CALLBACK;
<<<<<<< HEAD
<<<<<<< HEAD
  TMINIDUMP_INCLUDE_THREAD_CALLBACK = record
=======
  TMINIDUMP_INCLUDE_THREAD_CALLBACK = packed record
>>>>>>> graemeg/fixes_2_2
=======
  TMINIDUMP_INCLUDE_THREAD_CALLBACK = packed record
>>>>>>> origin/fixes_2_2
          ThreadId : ULONG;
       end;
  MINIDUMP_INCLUDE_THREAD_CALLBACK = TMINIDUMP_INCLUDE_THREAD_CALLBACK;
  LPMINIDUMP_INCLUDE_THREAD_CALLBACK = PMINIDUMP_INCLUDE_THREAD_CALLBACK;

  PMINIDUMP_MODULE_CALLBACK = ^TMINIDUMP_MODULE_CALLBACK;
<<<<<<< HEAD
<<<<<<< HEAD
  TMINIDUMP_MODULE_CALLBACK = record
=======
  TMINIDUMP_MODULE_CALLBACK = packed record
>>>>>>> graemeg/fixes_2_2
=======
  TMINIDUMP_MODULE_CALLBACK = packed record
>>>>>>> origin/fixes_2_2
          FullPath : PWCHAR;
          BaseOfImage : ULONG64;
          SizeOfImage : ULONG;
          CheckSum : ULONG;
          TimeDateStamp : ULONG;
          VersionInfo : TVSFIXEDFILEINFO;
          CvRecord : pointer;
          SizeOfCvRecord : ULONG;
          MiscRecord : pointer;
          SizeOfMiscRecord : ULONG;
       end;
  MINIDUMP_MODULE_CALLBACK = TMINIDUMP_MODULE_CALLBACK;
  LPMINIDUMP_MODULE_CALLBACK = PMINIDUMP_MODULE_CALLBACK;

  PMINIDUMP_INCLUDE_MODULE_CALLBACK = ^TMINIDUMP_INCLUDE_MODULE_CALLBACK;
<<<<<<< HEAD
<<<<<<< HEAD
  TMINIDUMP_INCLUDE_MODULE_CALLBACK = record
=======
  TMINIDUMP_INCLUDE_MODULE_CALLBACK = packed record
>>>>>>> graemeg/fixes_2_2
=======
  TMINIDUMP_INCLUDE_MODULE_CALLBACK = packed record
>>>>>>> origin/fixes_2_2
          BaseOfImage : ULONG64;
       end;
  MINIDUMP_INCLUDE_MODULE_CALLBACK = TMINIDUMP_INCLUDE_MODULE_CALLBACK;
  LPMINIDUMP_INCLUDE_MODULE_CALLBACK = PMINIDUMP_INCLUDE_MODULE_CALLBACK;

     
  PMINIDUMP_CALLBACK_INPUT = ^TMINIDUMP_CALLBACK_INPUT;
<<<<<<< HEAD
<<<<<<< HEAD
  TMINIDUMP_CALLBACK_INPUT = record
=======
  TMINIDUMP_CALLBACK_INPUT = packed record
>>>>>>> graemeg/fixes_2_2
=======
  TMINIDUMP_CALLBACK_INPUT = packed record
>>>>>>> origin/fixes_2_2
          ProcessId : ULONG;
          ProcessHandle : THANDLE;
          CallbackType : ULONG;
          anony : packed record
              case longint of
                 0 : ( Thread : TMINIDUMP_THREAD_CALLBACK );
                 1 : ( ThreadEx : TMINIDUMP_THREAD_EX_CALLBACK );
                 2 : ( Module : TMINIDUMP_MODULE_CALLBACK );
                 3 : ( IncludeThread : TMINIDUMP_INCLUDE_THREAD_CALLBACK );
                 4 : ( IncludeModule : TMINIDUMP_INCLUDE_MODULE_CALLBACK );
              end;
       end;
  MINIDUMP_CALLBACK_INPUT = TMINIDUMP_CALLBACK_INPUT;
  LPMINIDUMP_CALLBACK_INPUT = PMINIDUMP_CALLBACK_INPUT;

  PMINIDUMP_CALLBACK_OUTPUT = ^TMINIDUMP_CALLBACK_OUTPUT;
<<<<<<< HEAD
<<<<<<< HEAD
  TMINIDUMP_CALLBACK_OUTPUT = record
=======
  TMINIDUMP_CALLBACK_OUTPUT = packed record
>>>>>>> graemeg/fixes_2_2
=======
  TMINIDUMP_CALLBACK_OUTPUT = packed record
>>>>>>> origin/fixes_2_2
          anony3 : packed record
              case longint of
                 0 : ( ModuleWriteFlags : ULONG );
                 1 : ( ThreadWriteFlags : ULONG );
                 2 : ( anony2 : packed record
                      MemoryBase : ULONG64;
                      MemorySize : ULONG;
                   end );
              end;
       end;
  MINIDUMP_CALLBACK_OUTPUT = TMINIDUMP_CALLBACK_OUTPUT;
  LPMINIDUMP_CALLBACK_OUTPUT = PMINIDUMP_CALLBACK_OUTPUT;
  
  
  
  
  TIMAGEHLP_STATUS_ROUTINE   = function (Reason:TIMAGEHLP_STATUS_REASON; ImageName:PSTR; DllName:PSTR; Va:ULONG_PTR; Parameter:ULONG_PTR):BOOL;stdcall;
  TIMAGEHLP_STATUS_ROUTINE32 = function (Reason:TIMAGEHLP_STATUS_REASON; ImageName:PSTR; DllName:PSTR; Va:ULONG; Parameter:ULONG_PTR):BOOL;stdcall;
  TIMAGEHLP_STATUS_ROUTINE64 = function (Reason:TIMAGEHLP_STATUS_REASON; ImageName:PSTR; DllName:PSTR; Va:ULONG64; Parameter:ULONG_PTR):BOOL;stdcall;
  TDIGEST_FUNCTION 			 = function (refdata:TDIGEST_HANDLE; pData:pbyte; dwLength:DWORD):bool;stdcall;
  TFIND_DEBUG_FILE_CALLBACK  = function (FileHandle:THANDLE; FileName:PSTR; CallerData:pointer):bool;stdcall;	    // callback
  TFINDFILEINPATHCALLBACK    = function (filename:PSTR; context:pointer):bool;stdcall;
  TFIND_EXE_FILE_CALLBACK    = function (FileHandle:THANDLE; FileName:PSTR; CallerData:pointer):bool;stdcall;  
  
  TSYMBOLSERVERPROC          = function (para1:LPCSTR; para2:LPCSTR; para3:pointer; para4:DWORD; para5:DWORD; para6:lpstr):bool;stdcall;
  TSYMBOLSERVEROPENPROC      = function (para1:pointer):bool;stdcall;
  TSYMBOLSERVERCLOSEPROC     = function (para1:pointer):bool;stdcall;
  TSYMBOLSERVERSETOPTIONSPROC= function (para1:UINT_PTR; para2:ULONG64):bool;stdcall;
  TSYMBOLSERVERCALLBACKPROC  = function (action:UINT_PTR; data:ULONG64; context:ULONG64):bool;stdcall;
  TSYMBOLSERVERGETOPTIONSPROC= function :UINT_PTR;stdcall;
  TSYMBOLSERVERPINGPROC      = function (para1:LPCSTR):bool;stdcall;
  TENUMDIRTREE_CALLBACK      = function (FilePath:LPCSTR; CallerData:pointer):bool;stdcall;
  
  TREAD_PROCESS_MEMORY_ROUTINE64  = function (hProcess:THANDLE; qwBaseAddress:dword64; lpBuffer:pointer; nSize:dword; lpNumberOfBytesRead:lpdword):bool;stdcall;
  TFUNCTION_TABLE_ACCESS_ROUTINE64= function (hProcess:THANDLE; AddrBase:dword64):pointer;stdcall;
  TGET_MODULE_BASE_ROUTINE64      = function (hProcess:THANDLE; Address:dword64):dword64;stdcall;
  TTRANSLATE_ADDRESS_ROUTINE64    = function (hProcess:THANDLE; hThread:THANDLE; lpaddr:LPADDRESS64):dword64;stdcall;
  {$IFDEF IMAGEHLP64}
<<<<<<< HEAD
<<<<<<< HEAD
    TREAD_PROCESS_MEMORY_ROUTINE    = TREAD_PROCESS_MEMORY_ROUTINE64 ;
    TFUNCTION_TABLE_ACCESS_ROUTINE  = TFUNCTION_TABLE_ACCESS_ROUTINE64;
    TGET_MODULE_BASE_ROUTINE        = TGET_MODULE_BASE_ROUTINE64;
    TTRANSLATE_ADDRESS_ROUTINE      = TTRANSLATE_ADDRESS_ROUTINE64;
=======
=======
>>>>>>> origin/fixes_2_2
    TREAD_PROCESS_MEMORY_ROUTINE PREAD_PROCESS_MEMORY_ROUTINE =TREAD_PROCESS_MEMORY_ROUTINE PREAD_PROCESS_MEMORY_ROUTINE64;
    TFUNCTION_TABLE_ACCESS_ROUTINE PFUNCTION_TABLE_ACCESS_ROUTINE = TFUNCTION_TABLE_ACCESS_ROUTINE PFUNCTION_TABLE_ACCESS_ROUTINE64;
    TGET_MODULE_BASE_ROUTINE PGET_MODULE_BASE_ROUTINE = TGET_MODULE_BASE_ROUTINE PGET_MODULE_BASE_ROUTINE64;
    TTRANSLATE_ADDRESS_ROUTINE PTRANSLATE_ADDRESS_ROUTINE = TTRANSLATE_ADDRESS_ROUTINE PTRANSLATE_ADDRESS_ROUTINE64;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  {$ELSE}
    TREAD_PROCESS_MEMORY_ROUTINE = function (hProcess:THANDLE; lpBaseAddress:dword; lpBuffer:pointer; nSize:dword; lpNumberOfBytesRead:PDWORD):bool;stdcall;
    TFUNCTION_TABLE_ACCESS_ROUTINE = function (hProcess:THANDLE; AddrBase:dword):pointer;stdcall;
    TGET_MODULE_BASE_ROUTINE = function (hProcess:THANDLE; Address:dword):dword;stdcall;
    TTRANSLATE_ADDRESS_ROUTINE = function (hProcess:THANDLE; hThread:THANDLE; lpaddr:LPADDRESS):dword;stdcall;
  {$ENDIF}
  TSYM_ENUMMODULES_CALLBACK64 = function (ModuleName:PSTR; BaseOfDll:dword64; UserContext:pointer):bool;stdcall;   {CALLBACK }
  TSYM_ENUMSYMBOLS_CALLBACK64 = function (SymbolName:PSTR; SymbolAddress:dword64; SymbolSize:ULONG; UserContext:pointer):bool;stdcall;   {CALLBACK }
  TSYM_ENUMSYMBOLS_CALLBACK64W = function (SymbolName:lpwstr; SymbolAddress:dword64; SymbolSize:ULONG; UserContext:pointer):bool;stdcall;   {CALLBACK }
  TENUMLOADED_MODULES_CALLBACK64 = function (ModuleName:PSTR; ModuleBase:dword64; ModuleSize:ULONG; UserContext:pointer):bool;stdcall;   {CALLBACK }
  TSYMBOL_REGISTERED_CALLBACK64 = function (hProcess:THANDLE; ActionCode:ULONG; CallbackData:ULONG64; UserContext:ULONG64):bool;stdcall;   {CALLBACK } 
  TSYMBOL_FUNCENTRY_CALLBACK64 = function (hProcess:THANDLE; AddrBase:ULONG64; UserContext:ULONG64):pointer;stdcall;  {CALLBACK }
  {$IFDEF IMAGEHLP64}
   TSYM_ENUMMODULES_CALLBACK    = TSYM_ENUMMODULES_CALLBACK64;
   TSYM_ENUMSYMBOLS_CALLBACK    = TSYM_ENUMSYMBOLS_CALLBACK64;
   TSYM_ENUMSYMBOLS_CALLBACKW   = TSYM_ENUMSYMBOLS_CALLBACK64W;
   TENUMLOADED_MODULES_CALLBACK = TENUMLOADED_MODULES_CALLBACK64;
   TSYMBOL_REGISTERED_CALLBACK  = TSYMBOL_REGISTERED_CALLBACK64; 
   TSYMBOL_FUNCENTRY_CALLBACK   = TSYMBOL_FUNCENTRY_CALLBACK64;
  {$ELSE}
   TSYM_ENUMMODULES_CALLBACK = function (ModuleName:PSTR; BaseOfDll:ULONG; UserContext:pointer):bool;stdcall;  
   TSYM_ENUMSYMBOLS_CALLBACK = function (SymbolName:PSTR; SymbolAddress:ULONG; SymbolSize:ULONG; UserContext:pointer):bool;stdcall;  
   TSYM_ENUMSYMBOLS_CALLBACKW = function (SymbolName:lpwstr; SymbolAddress:ULONG; SymbolSize:ULONG; UserContext:pointer):bool;stdcall;  
   TENUMLOADED_MODULES_CALLBACK = function (ModuleName:PSTR; ModuleBase:ULONG; ModuleSize:ULONG; UserContext:pointer):bool;stdcall;
   TSYMBOL_REGISTERED_CALLBACK = function (hProcess:THANDLE; ActionCode:ULONG; CallbackData:pointer; UserContext:pointer):bool;stdcall;  
   TSYMBOL_FUNCENTRY_CALLBACK = function (hProcess:THANDLE; AddrBase:dword; UserContext:pointer):pointer;stdcall;   {CALLBACK }  
  {$ENDIF}
   TSYM_ENUMSOURCFILES_CALLBACK = function (pSourceFile:PSOURCEFILE; UserContext:pointer):BOOL;stdcall;
   TSYM_ENUMLINES_CALLBACK = function (LineInfo:PSRCCODEINFO; UserContext:pointer):BOOL;stdcall;
   TSYM_ENUMERATESYMBOLS_CALLBACK = function (pSymInfo:PSYMBOL_INFO; SymbolSize:ULONG; UserContext:pointer):BOOL;stdcall;
   TDBGHELP_CREATE_USER_DUMP_CALLBACK = function (DataType:dword; Data:PPOINTER; DataLength:lpdword; UserData:pointer):BOOL;stdcall;   
   TMINIDUMP_CALLBACK_ROUTINE = function (CallbackParam:pointer; CallbackInput:PMINIDUMP_CALLBACK_INPUT; CallbackOutput:PMINIDUMP_CALLBACK_OUTPUT):BOOL;stdcall;
   PMINIDUMP_CALLBACK_INFORMATION = ^TMINIDUMP_CALLBACK_INFORMATION;
<<<<<<< HEAD
<<<<<<< HEAD
   TMINIDUMP_CALLBACK_INFORMATION = record
=======
   TMINIDUMP_CALLBACK_INFORMATION = packed record
>>>>>>> graemeg/fixes_2_2
=======
   TMINIDUMP_CALLBACK_INFORMATION = packed record
>>>>>>> origin/fixes_2_2
          CallbackRoutine : TMINIDUMP_CALLBACK_ROUTINE;
          CallbackParam : pointer;
       end;
   MINIDUMP_CALLBACK_INFORMATION = TMINIDUMP_CALLBACK_INFORMATION;
   LPMINIDUMP_CALLBACK_INFORMATION = PMINIDUMP_CALLBACK_INFORMATION;
   
function BindImage(ImageName:PSTR; DllPath:PSTR; SymbolPath:PSTR):BOOL;stdcall;external External_library name 'BindImage'; 
function BindImageEx(Flags:DWORD; ImageName:PSTR; DllPath:PSTR; SymbolPath:PSTR; StatusRoutine:TIMAGEHLP_STATUS_ROUTINE):BOOL;stdcall;external External_library name 'BindImageEx';
function ReBaseImage(CurrentImageName:PSTR; SymbolPath:PSTR; fReBase:bool; fRebaseSysfileOk:bool; fGoingDown:bool; 
             CheckImageSize:ULONG; OldImageSize:PULONG; OldImageBase:PULONG_PTR; NewImageSize:PULONG; NewImageBase:PULONG_PTR; 
             TimeStamp:ULONG):bool;stdcall;external External_library name 'ReBaseImage'; 
function ReBaseImage64(CurrentImageName:PSTR; SymbolPath:PSTR; fReBase:bool; fRebaseSysfileOk:bool; fGoingDown:bool; 
             CheckImageSize:ULONG; OldImageSize:PULONG; OldImageBase:PULONG64; NewImageSize:PULONG; NewImageBase:PULONG64; 
             TimeStamp:ULONG):bool;stdcall;external External_library name 'ReBaseImage64';
function CheckSumMappedFile(BaseAddress:pointer; FileLength:DWORD; HeaderSum:PDWORD; CheckSum:PDWORD):PIMAGE_NT_HEADERS;stdcall;external External_library name 'CheckSumMappedFile';
function MapFileAndCheckSumA(Filename:PSTR; HeaderSum:PDWORD; CheckSum:PDWORD):DWORD;stdcall;external External_library name 'MapFileAndCheckSumA';
function MapFileAndCheckSumW(Filename:lpwstr; HeaderSum:PDWORD; CheckSum:PDWORD):DWORD;stdcall;external External_library name 'MapFileAndCheckSumW';
{$IFDEF UNICODE}
function MapFileAndCheckSum(Filename:lpwstr; HeaderSum:PDWORD; CheckSum:PDWORD):DWORD;stdcall;external External_library name 'MapFileAndCheckSumW';
{$ELSE}
function MapFileAndCheckSum(Filename:PSTR; HeaderSum:PDWORD; CheckSum:PDWORD):DWORD;stdcall;external External_library name 'MapFileAndCheckSumA';
{$ENDIF}
function GetImageConfigInformation(LoadedImage:PLOADED_IMAGE; ImageConfigInformation:PIMAGE_LOAD_CONFIG_DIRECTORY):bool;stdcall;external External_library name 'GetImageConfigInformation';
function GetImageUnusedHeaderBytes(LoadedImage:PLOADED_IMAGE; SizeUnusedHeaderBytes:PDWORD):DWORD;stdcall;external External_library name 'GetImageUnusedHeaderBytes';
function SetImageConfigInformation(LoadedImage:PLOADED_IMAGE; ImageConfigInformation:PIMAGE_LOAD_CONFIG_DIRECTORY):bool;stdcall;external External_library name 'SetImageConfigInformation';
function ImageGetDigestStream(FileHandle:THANDLE; DigestLevel:DWORD; DigestFunction:TDIGEST_FUNCTION; DigestHandle:TDIGEST_HANDLE):bool;stdcall;external External_library name 'ImageGetDigestStream';
function ImageAddCertificate(FileHandle:THANDLE; Certificate:PWINCERTIFICATE; Index:PDWORD):bool;stdcall;external External_library name 'ImageAddCertificate';
function ImageRemoveCertificate(FileHandle:THANDLE; Index:DWORD):bool;stdcall;external External_library name 'ImageRemoveCertificate';
function ImageEnumerateCertificates(FileHandle:THANDLE; TypeFilter:word; CertificateCount:PDWORD; Indices:PDWORD; IndexCount:DWORD):bool;stdcall;external External_library name 'ImageEnumerateCertificates';
function ImageGetCertificateData(FileHandle:THANDLE; CertificateIndex:DWORD; Certificate:PWINCERTIFICATE; RequiredLength:PDWORD):bool;stdcall;external External_library name 'ImageGetCertificateData';
function ImageGetCertificateHeader(FileHandle:THANDLE; CertificateIndex:DWORD; Certificateheader:PWINCERTIFICATE):bool;stdcall;external External_library name 'ImageGetCertificateHeader';

function ImageLoad(DllName:PSTR; DllPath:PSTR):PLOADED_IMAGE;stdcall;external External_library name 'ImageLoad';
function ImageUnload(LoadedImage:PLOADED_IMAGE):bool;stdcall;external External_library name 'ImageUnload';
function MapAndLoad(ImageName:PSTR; DllPath:PSTR; LoadedImage:PLOADED_IMAGE; DotDll:bool; ReadOnly:BOOL):bool;stdcall;external External_library name 'MapAndLoad';
function UnMapAndLoad(LoadedImage:PLOADED_IMAGE):bool;stdcall;external External_library name 'UnMapAndLoad';
function TouchFileTimes(FileHandle:THANDLE; pSystemTime:PSYSTEMTIME):bool;stdcall;external External_library name 'TouchFileTimes';
function SplitSymbols(ImageName:PSTR; SymbolsPath:PSTR; SymbolFilePath:PSTR; Flags:DWORD):bool;stdcall;external External_library name 'SplitSymbols';
function UpdateDebugInfoFile(ImageFileName:PSTR; SymbolPath:PSTR; DebugFilePath:PSTR; NtHeaders:PIMAGE_NT_HEADERS32):bool;stdcall;external External_library name 'UpdateDebugInfoFile';
function UpdateDebugInfoFileEx(ImageFileName:PSTR; SymbolPath:PSTR; DebugFilePath:PSTR; NtHeaders:PIMAGE_NT_HEADERS32; OldChecksum:DWORD):bool;stdcall;external External_library name 'UpdateDebugInfoFileEx';
function FindDebugInfoFile(FileName:PSTR; SymbolPath:PSTR; DebugFilePath:PSTR):THANDLE;stdcall;external External_library name 'FindDebugInfoFile';

function FindDebugInfoFileEx(FileName:PSTR; SymbolPath:PSTR; DebugFilePath:PSTR; Callback:TFIND_DEBUG_FILE_CALLBACK; CallerData:pointer):THANDLE;stdcall;external External_library name 'FindDebugInfoFileEx';  
function SymFindFileInPath(hprocess:THANDLE; SearchPath:lpstr; FileName:lpstr; id:pointer; two:DWORD; 
             three:DWORD; flags:DWORD; FoundFile:lpstr; callback:TFINDFILEINPATHCALLBACK; context:pointer):bool;stdcall;external External_library name 'SymFindFileInPath';
function FindExecutableImage(FileName:PSTR; SymbolPath:PSTR; ImageFilePath:PSTR):THANDLE;stdcall;external External_library name 'FindExecutableImage';
function FindExecutableImageEx(FileName:PSTR; SymbolPath:PSTR; ImageFilePath:PSTR; Callback:TFIND_EXE_FILE_CALLBACK; CallerData:pointer):THANDLE;stdcall;external External_library name 'FindExecutableImageEx';
function ImageNtHeader(Base:pointer):PIMAGE_NT_HEADERS;stdcall;external External_library name 'ImageNtHeader';
function ImageDirectoryEntryToDataEx(Base:pointer; MappedAsImage:bytebool; DirectoryEntry:ushort; Size:PULONG; FoundHeader:PPIMAGE_SECTION_HEADER):pointer;stdcall;external External_library name 'ImageDirectoryEntryToDataEx';
function ImageDirectoryEntryToData(Base:pointer; MappedAsImage:bytebool; DirectoryEntry:ushort; Size:PULONG):pointer;stdcall;external External_library name 'ImageDirectoryEntryToData';
function ImageRvaToSection(NtHeaders:PIMAGE_NT_HEADERS; Base:pointer; Rva:ULONG):PIMAGE_SECTION_HEADER;stdcall;external External_library name 'ImageRvaToSection';
function ImageRvaToVa(NtHeaders:PIMAGE_NT_HEADERS; Base:pointer; Rva:ULONG; LastRvaSection:PPIMAGE_SECTION_HEADER):pointer;stdcall;external External_library name 'ImageRvaToVa';
{$IFNDEF WIN64}
function MapDebugInformation(FileHandle:THANDLE; FileName:PSTR; SymbolPath:PSTR; ImageBase:dword):PIMAGE_DEBUG_INFORMATION;stdcall;external External_library name 'MapDebugInformation';
function UnmapDebugInformation(DebugInfo:PIMAGE_DEBUG_INFORMATION):bool;stdcall;external External_library name 'UnmapDebugInformation';
{$ENDIF}
function SearchTreeForFile(RootPath:PSTR; InputPathName:PSTR; OutputPathBuffer:PSTR):bool;stdcall;external External_library name 'SearchTreeForFile';
function EnumDirTree(hProcess:THANDLE; RootPath:PSTR; InputPathName:PSTR; OutputPathBuffer:PSTR; Callback:TENUMDIRTREE_CALLBACK; 
             CallbackData:pointer):bool;stdcall;external External_library name 'EnumDirTree';
function MakeSureDirectoryPathExists(DirPath:LPCSTR):bool;stdcall;external External_library name 'MakeSureDirectoryPathExists';
function UnDecorateSymbolName(DecoratedName:LPCSTR; UnDecoratedName:PSTR; UndecoratedLength:dword; Flags:dword):dword;stdcall;external External_library name 'UnDecorateSymbolName';
function StackWalk64(MachineType:dword; hProcess:THANDLE; hThread:THANDLE; StackFrame:LPSTACKFRAME64; ContextRecord:pointer; 
      ReadMemoryRoutine:TREAD_PROCESS_MEMORY_ROUTINE64; FunctionTableAccessRoutine:TFUNCTION_TABLE_ACCESS_ROUTINE64; GetModuleBaseRoutine:TGET_MODULE_BASE_ROUTINE64; TranslateAddress:TTRANSLATE_ADDRESS_ROUTINE64):bool;stdcall;external External_library name 'StackWalk64';
{$IFDEF IMAGEHLP64}
function StackWalk(MachineType:dword; hProcess:THANDLE; hThread:THANDLE; StackFrame:LPSTACKFRAME64; ContextRecord:pointer; 
      ReadMemoryRoutine:TREAD_PROCESS_MEMORY_ROUTINE64; FunctionTableAccessRoutine:TFUNCTION_TABLE_ACCESS_ROUTINE64; GetModuleBaseRoutine:TGET_MODULE_BASE_ROUTINE64; TranslateAddress:TTRANSLATE_ADDRESS_ROUTINE64):bool;stdcall;external External_library name 'StackWalk64';
{$ELSE}
function StackWalk(MachineType:dword; hProcess:THANDLE; hThread:THANDLE; StackFrame:LPSTACKFRAME; ContextRecord:pointer; 
      ReadMemoryRoutine:TREAD_PROCESS_MEMORY_ROUTINE; FunctionTableAccessRoutine:TFUNCTION_TABLE_ACCESS_ROUTINE; GetModuleBaseRoutine:TGET_MODULE_BASE_ROUTINE; TranslateAddress:TTRANSLATE_ADDRESS_ROUTINE):bool;stdcall;external External_library name 'StackWalk';
{$ENDIF}
function ImagehlpApiVersion:LPAPI_VERSION;stdcall;external External_library name 'ImagehlpApiVersion';
function ImagehlpApiVersionEx(AppVersion:LPAPI_VERSION):LPAPI_VERSION;stdcall;external External_library name 'ImagehlpApiVersionEx';
function GetTimestampForLoadedLibrary(Module:HMODULE):dword;stdcall;external External_library name 'GetTimestampForLoadedLibrary';
function SymSetParentWindow(hwnd:HWND):bool;stdcall;external External_library name 'SymSetParentWindow';
function SymSetHomeDirectory(dir:LPCSTR):pchar;stdcall;external External_library name 'SymSetHomeDirectory';
function SymGetHomeDirectory(_type:dword; dir:PSTR; size:Tsize):pchar;stdcall;external External_library name 'SymGetHomeDirectory';
function SymSetOptions(SymOptions:dword):dword;stdcall;external External_library name 'SymSetOptions';
function SymGetOptions:dword;stdcall;external External_library name 'SymGetOptions';
function SymCleanup(hProcess:THANDLE):BOOL;stdcall;external External_library name 'SymCleanup';
function SymMatchString(_string:lpstr; expression:lpstr; fCase:BOOL):BOOL;stdcall;external External_library name 'SymMatchString';
function SymEnumSourceFiles(hProcess:THANDLE; ModBase:ULONG64; Mask:lpstr; cbSrcFiles:TSYM_ENUMSOURCFILES_CALLBACK; UserContext:pointer):BOOL;stdcall;external External_library name 'SymEnumSourceFiles';

function SymEnumerateModules64(hProcess:THANDLE; EnumModulesCallback:TSYM_ENUMMODULES_CALLBACK64; UserContext:pointer):BOOL;stdcall;external External_library name 'SymEnumerateModules64';
function SymEnumerateSymbols64(hProcess:THANDLE; BaseOfDll:dword64; EnumSymbolsCallback:TSYM_ENUMSYMBOLS_CALLBACK64; UserContext:pointer):BOOL;stdcall;external External_library name 'SymEnumerateSymbols64';  
function SymEnumerateSymbolsW64(hProcess:THANDLE; BaseOfDll:dword64; EnumSymbolsCallback:TSYM_ENUMSYMBOLS_CALLBACK64W; UserContext:pointer):BOOL;stdcall;external External_library name 'SymEnumerateSymbolsW64';
function EnumerateLoadedModules64(hProcess:THANDLE; EnumLoadedModulesCallback:TENUMLOADED_MODULES_CALLBACK64; UserContext:pointer):BOOL;stdcall;external External_library name 'EnumerateLoadedModules64';
function SymFunctionTableAccess64(hProcess:THANDLE; AddrBase:dword64):pointer;stdcall;external External_library name 'SymFunctionTableAccess64';
function SymGetModuleInfo64(hProcess:THANDLE; qwAddr:dword64; ModuleInfo:PIMAGEHLP_MODULE64):BOOL;stdcall;external External_library name 'SymGetModuleInfo64';
function SymGetModuleInfoW64(hProcess:THANDLE; qwAddr:dword64; ModuleInfo:PIMAGEHLP_MODULEW64):BOOL;stdcall;external External_library name 'SymGetModuleInfoW64';
function SymGetModuleBase64(hProcess:THANDLE; qwAddr:dword64):dword64;stdcall;external External_library name 'SymGetModuleBase64';  
function SymGetSymNext64(hProcess:THANDLE; Symbol:PIMAGEHLP_SYMBOL64):BOOL;stdcall;external External_library name 'SymGetSymNext64';
function SymGetSymPrev64(hProcess:THANDLE; Symbol:PIMAGEHLP_SYMBOL64):BOOL;stdcall;external External_library name 'SymGetSymPrev64';

{$ifdef IMAGEHLP64}
function SymEnumerateModules(hProcess:THANDLE; EnumModulesCallback:TSYM_ENUMMODULES_CALLBACK64; UserContext:pointer):BOOL;stdcall;external External_library name 'SymEnumerateModules64';
function SymEnumerateSymbols(hProcess:THANDLE; BaseOfDll:dword64; EnumSymbolsCallback:TSYM_ENUMSYMBOLS_CALLBACK64; UserContext:pointer):BOOL;stdcall;external External_library name 'SymEnumerateSymbols64';  
<<<<<<< HEAD
<<<<<<< HEAD
function SymEnumerateSymbolsW(hProcess:THANDLE; BaseOfDll:dword64; EnumSymbolsCallback:TSYM_ENUMSYMBOLS_CALLBACK64W; UserContext:pointer):BOOL;stdcall;external External_library name 'SymEnumerateSymbolsW64';
function EnumerateLoadedModules(hProcess:THANDLE; EnumLoadedModulesCallback:TENUMLOADED_MODULES_CALLBACK64; UserContext:pointer):BOOL;stdcall;external External_library name 'EnumerateLoadedModules64';
=======
function SymEnumerateSymbolsW(hProcess:THANDLE; BaseOfDll:dword64; EnumSymbolsCallback:TSYM_ENUMSYMBOLS_CALLBACK64W; UserContext:pointer):BOOL;stdcall;external External_library name 'SymEnumerateSymbolsW64'
function EnumerateLoadedModules(hProcess:THANDLE; EnumLoadedModulesCallback:TPENUMLOADED_MODULES_CALLBACK64; UserContext:pointer):BOOL;stdcall;external External_library name 'EnumerateLoadedModules64';;
>>>>>>> graemeg/fixes_2_2
=======
function SymEnumerateSymbolsW(hProcess:THANDLE; BaseOfDll:dword64; EnumSymbolsCallback:TSYM_ENUMSYMBOLS_CALLBACK64W; UserContext:pointer):BOOL;stdcall;external External_library name 'SymEnumerateSymbolsW64'
function EnumerateLoadedModules(hProcess:THANDLE; EnumLoadedModulesCallback:TPENUMLOADED_MODULES_CALLBACK64; UserContext:pointer):BOOL;stdcall;external External_library name 'EnumerateLoadedModules64';;
>>>>>>> origin/fixes_2_2
function SymFunctionTableAccess(hProcess:THANDLE; AddrBase:dword64):pointer;stdcall;external External_library name 'SymFunctionTableAccess64';
function SymGetModuleInfo(hProcess:THANDLE; qwAddr:dword64; ModuleInfo:PIMAGEHLP_MODULE64):BOOL;stdcall;external External_library name 'SymGetModuleInfo64';
function SymGetModuleInfoW(hProcess:THANDLE; qwAddr:dword64; ModuleInfo:PIMAGEHLP_MODULEW64):BOOL;stdcall;external External_library name 'SymGetModuleInfoW64';
function SymGetModuleBase(hProcess:THANDLE; qwAddr:dword64):dword64;stdcall;external External_library name 'SymGetModuleBase64';  
function SymGetSymNext(hProcess:THANDLE; Symbol:PIMAGEHLP_SYMBOL64):BOOL;stdcall;external External_library name 'SymGetSymNext64';
function SymGetSymPrev(hProcess:THANDLE; Symbol:PIMAGEHLP_SYMBOL64):BOOL;stdcall;external External_library name 'SymGetSymPrev64';
{$ELSE}
function SymEnumerateModules(hProcess:THANDLE; EnumModulesCallback:TSYM_ENUMMODULES_CALLBACK; UserContext:pointer):BOOL;stdcall;external External_library name 'SymEnumerateModules';  
function SymEnumerateSymbols(hProcess:THANDLE; BaseOfDll:dword; EnumSymbolsCallback:TSYM_ENUMSYMBOLS_CALLBACK; UserContext:pointer):BOOL;stdcall;external External_library name 'SymEnumerateSymbols';
function SymEnumerateSymbolsW(hProcess:THANDLE; BaseOfDll:dword; EnumSymbolsCallback:TSYM_ENUMSYMBOLS_CALLBACKW; UserContext:pointer):BOOL;stdcall;external External_library name 'SymEnumerateSymbolsW';
function EnumerateLoadedModules(hProcess:THANDLE; EnumLoadedModulesCallback:TENUMLOADED_MODULES_CALLBACK; UserContext:pointer):BOOL;stdcall;external External_library name 'EnumerateLoadedModules';
function SymFunctionTableAccess(hProcess:THANDLE; AddrBase:dword):pointer;stdcall;external External_library name 'SymFunctionTableAccess';
function SymGetModuleInfo(hProcess:THANDLE; dwAddr:dword; ModuleInfo:PIMAGEHLP_MODULE):BOOL;stdcall;external External_library name 'SymGetModuleInfo';
function SymGetModuleInfoW(hProcess:THANDLE; dwAddr:dword; ModuleInfo:PIMAGEHLP_MODULEW):BOOL;stdcall;external External_library name 'SymGetModuleInfoW';
function SymGetSymPrev(hProcess:THANDLE; Symbol:PIMAGEHLP_SYMBOL):BOOL;stdcall;external External_library name 'SymGetSymPrev';
function SymGetSymNext(hProcess:THANDLE; Symbol:PIMAGEHLP_SYMBOL):BOOL;stdcall;external External_library name 'SymGetSymNext';  
function SymGetModuleBase(hProcess:THANDLE; dwAddr:dword):dword;stdcall;external External_library name 'SymGetModuleBase';
{$ENDIF}
function SymGetLineFromAddr64(hProcess:THANDLE; qwAddr:dword64; pdwDisplacement:PDWORD; Line64:PIMAGEHLP_LINE64):BOOL;stdcall;external External_library name 'SymGetLineFromAddr64';
function SymGetLineFromName64(hProcess:THANDLE; ModuleName:PSTR; FileName:PSTR; dwLineNumber:dword; plDisplacement:PLONG; 
             Line:PIMAGEHLP_LINE64):BOOL;stdcall;external External_library name 'SymGetLineFromName64';
function SymGetLineNext64(hProcess:THANDLE; Line:PIMAGEHLP_LINE64):BOOL;stdcall;external External_library name 'SymGetLineNext64';			 
function SymGetLinePrev64(hProcess:THANDLE; Line:PIMAGEHLP_LINE64):BOOL;stdcall;external External_library name 'SymGetLinePrev64'; 
function SymLoadModule64(hProcess:THANDLE; hFile:THANDLE; ImageName:PSTR; ModuleName:PSTR; BaseOfDll:dword64; 
             SizeOfDll:dword):dword64;stdcall;external External_library name 'SymLoadModule64';               
function SymUnloadModule64(hProcess:THANDLE; BaseOfDll:dword64):BOOL;stdcall;external External_library name 'SymUnloadModule64'; 
function SymUnDName64(sym:PIMAGEHLP_SYMBOL64; UnDecName:PSTR; UnDecNameLength:dword):BOOL;stdcall;external External_library name 'SymUnDName64';
function SymRegisterCallback64(hProcess:THANDLE; CallbackFunction:TSYMBOL_REGISTERED_CALLBACK64; UserContext:ULONG64):BOOL;stdcall;external External_library name 'SymRegisterCallback64';
function SymRegisterFunctionEntryCallback64(hProcess:THANDLE; CallbackFunction:TSYMBOL_FUNCENTRY_CALLBACK64; UserContext:ULONG64):BOOL;stdcall;external External_library name 'SymRegisterFunctionEntryCallback64';     

{$ifdef IMAGEHLP64}
function SymGetLineFromAddr(hProcess:THANDLE; qwAddr:dword64; pdwDisplacement:PDWORD; Line64:PIMAGEHLP_LINE64):BOOL;stdcall;external External_library name 'SymGetLineFromAddr64';
<<<<<<< HEAD
<<<<<<< HEAD
function SymGetLineFromName(hProcess:THANDLE; ModuleName:PSTR; FileName:PSTR; dwLineNumber:dword; plDisplacement:PLONG; 
=======
function SymGetLineFromName(hProcess:THANDLE; ModuleName:PSTR; FileName:PSTR; dwLineNumber:dword; plDisplacement:TPLONG; 
>>>>>>> graemeg/fixes_2_2
=======
function SymGetLineFromName(hProcess:THANDLE; ModuleName:PSTR; FileName:PSTR; dwLineNumber:dword; plDisplacement:TPLONG; 
>>>>>>> origin/fixes_2_2
             Line:PIMAGEHLP_LINE64):BOOL;stdcall;external External_library name 'SymGetLineFromName64';
function SymGetLineNext(hProcess:THANDLE; Line:PIMAGEHLP_LINE64):BOOL;stdcall;external External_library name 'SymGetLineNext64';			 
function SymGetLinePrev(hProcess:THANDLE; Line:PIMAGEHLP_LINE64):BOOL;stdcall;external External_library name 'SymGetLinePrev64'; 
function SymLoadModule(hProcess:THANDLE; hFile:THANDLE; ImageName:PSTR; ModuleName:PSTR; BaseOfDll:dword64; 
             SizeOfDll:dword):dword64;stdcall;external External_library name 'SymLoadModule64';               
function SymUnloadModule(hProcess:THANDLE; BaseOfDll:dword64):BOOL;stdcall;external External_library name 'SymUnloadModule64'; 
function SymUnDName(sym:PIMAGEHLP_SYMBOL64; UnDecName:PSTR; UnDecNameLength:dword):BOOL;stdcall;external External_library name 'SymUnDName64';
function SymRegisterCallback(hProcess:THANDLE; CallbackFunction:TSYMBOL_REGISTERED_CALLBACK64; UserContext:ULONG64):BOOL;stdcall;external External_library name 'SymRegisterCallback64';
function SymRegisterFunctionEntryCallback(hProcess:THANDLE; CallbackFunction:TSYMBOL_FUNCENTRY_CALLBACK64; UserContext:ULONG64):BOOL;stdcall;external External_library name 'SymRegisterFunctionEntryCallback64';     
{$else}
function SymGetLineFromAddr(hProcess:THANDLE; dwAddr:dword; pdwDisplacement:PDWORD; Line:PIMAGEHLP_LINE):BOOL;stdcall;external External_library name 'SymGetLineFromAddr';
function SymGetLineFromName(hProcess:THANDLE; ModuleName:PSTR; FileName:PSTR; dwLineNumber:dword; plDisplacement:PLONG; 
             Line:PIMAGEHLP_LINE):BOOL;stdcall;external External_library name 'SymGetLineFromName';
function SymGetLineNext(hProcess:THANDLE; Line:PIMAGEHLP_LINE):BOOL;stdcall;external External_library name 'SymGetLineNext';
function SymGetLinePrev(hProcess:THANDLE; Line:PIMAGEHLP_LINE):BOOL;stdcall;external External_library name 'SymGetLinePrev';     		 
function SymLoadModule(hProcess:THANDLE; hFile:THANDLE; ImageName:PSTR; ModuleName:PSTR; BaseOfDll:dword; 
             SizeOfDll:dword):dword;stdcall;external External_library name 'SymLoadModule';
function SymUnloadModule(hProcess:THANDLE; BaseOfDll:dword):BOOL;stdcall;external External_library name 'SymUnloadModule';
function SymUnDName(sym:PIMAGEHLP_SYMBOL; UnDecName:PSTR; UnDecNameLength:dword):BOOL;stdcall;external External_library name 'SymUnDName';     
function SymRegisterCallback(hProcess:THANDLE; CallbackFunction:TSYMBOL_REGISTERED_CALLBACK; UserContext:pointer):BOOL;stdcall;external External_library name 'SymRegisterCallback';
function SymRegisterFunctionEntryCallback(hProcess:THANDLE; CallbackFunction:TSYMBOL_FUNCENTRY_CALLBACK; UserContext:pointer):BOOL;stdcall;external External_library name 'SymRegisterFunctionEntryCallback';
{$endif}

function SymEnumLines(hProcess:THANDLE; Base:ULONG64; Obj:LPCSTR; _File:LPCSTR; EnumLinesCallback:TSYM_ENUMLINES_CALLBACK; 
             UserContext:pointer):BOOL;stdcall;external External_library name 'SymEnumLines';
function SymMatchFileName(FileName:PSTR; Match:PSTR; FileNameStop:PPSTR; MatchStop:PPSTR):BOOL;stdcall;external External_library name 'SymMatchFileName';
function SymInitialize(hProcess:THANDLE; UserSearchPath:PSTR; fInvadeProcess:BOOL):BOOL;stdcall;external External_library name 'SymInitialize';
function SymGetSearchPath(hProcess:THANDLE; SearchPath:PSTR; SearchPathLength:dword):BOOL;stdcall;external External_library name 'SymGetSearchPath';
function SymSetSearchPath(hProcess:THANDLE; SearchPath:PSTR):BOOL;stdcall;external External_library name 'SymSetSearchPath';
function SymLoadModuleEx(hProcess:THANDLE; hFile:THANDLE; ImageName:PSTR; ModuleName:PSTR; BaseOfDll:dword64; 
             DllSize:dword; Data:PMODLOAD_DATA; Flags:dword):dword64;stdcall;external External_library name 'SymLoadModuleEx';

function SymSetContext(hProcess:THANDLE; StackFrame:PIMAGEHLP_STACK_FRAME; Context:PIMAGEHLP_CONTEXT):BOOL;stdcall;external External_library name 'SymSetContext';
function SymFromAddr(hProcess:THANDLE; Address:dword64; Displacement:PDWORD64; Symbol:PSYMBOL_INFO):BOOL;stdcall;external External_library name 'SymFromAddr';
function SymFromToken(hProcess:THANDLE; Base:dword64; Token:dword; Symbol:PSYMBOL_INFO):BOOL;stdcall;external External_library name 'SymFromToken';
function SymFromName(hProcess:THANDLE; Name:lpstr; Symbol:PSYMBOL_INFO):BOOL;stdcall;external External_library name 'SymFromName';
function SymEnumSymbols(hProcess:THANDLE; BaseOfDll:ULONG64; Mask:LPCSTR; EnumSymbolsCallback:TSYM_ENUMERATESYMBOLS_CALLBACK; UserContext:pointer):BOOL;stdcall;external External_library name 'SymEnumSymbols';
function SymEnumSymbolsForAddr(hProcess:THANDLE; Address:dword64; EnumSymbolsCallback:TSYM_ENUMERATESYMBOLS_CALLBACK; UserContext:pointer):BOOL;stdcall;external External_library name 'SymEnumSymbolsForAddr';
	
function SymGetTypeInfo(hProcess:THANDLE; ModBase:dword64; TypeId:ULONG; GetType:TIMAGEHLP_SYMBOL_TYPE_INFO; pInfo:pointer):BOOL;stdcall;external External_library name 'SymGetTypeInfo';
function SymEnumTypes(hProcess:THANDLE; BaseOfDll:ULONG64; EnumSymbolsCallback:TSYM_ENUMERATESYMBOLS_CALLBACK; UserContext:pointer):BOOL;stdcall;external External_library name 'SymEnumTypes';
function SymGetTypeFromName(hProcess:THANDLE; BaseOfDll:ULONG64; Name:lpstr; Symbol:PSYMBOL_INFO):BOOL;stdcall;external External_library name 'SymGetTypeFromName';
function SymAddSymbol(hProcess:THANDLE; BaseOfDll:ULONG64; Name:LPCSTR; Address:dword64; Size:dword; 
             Flags:dword):BOOL;stdcall;external External_library name 'SymAddSymbol';
function SymDeleteSymbol(hProcess:THANDLE; BaseOfDll:ULONG64; Name:LPCSTR; Address:dword64; Flags:dword):BOOL;stdcall;external External_library name 'SymDeleteSymbol';
function DbgHelpCreateUserDump(FileName:lpstr; Callback:TDBGHELP_CREATE_USER_DUMP_CALLBACK; UserData:pointer):BOOL;stdcall;external External_library name 'DbgHelpCreateUserDump';
function DbgHelpCreateUserDumpW(FileName:LPWSTR; Callback:TDBGHELP_CREATE_USER_DUMP_CALLBACK; UserData:pointer):BOOL;stdcall;external External_library name 'DbgHelpCreateUserDumpW';

function SymGetSymFromAddr64(hProcess:THANDLE; qwAddr:dword64; pdwDisplacement:PDWORD64; Symbol:PIMAGEHLP_SYMBOL64):BOOL;stdcall;external External_library name 'SymGetSymFromAddr64';
function SymGetSymFromName64(hProcess:THANDLE; Name:PSTR; Symbol:PIMAGEHLP_SYMBOL64):BOOL;stdcall;external External_library name 'SymGetSymFromName64';
{$ifdef IMAGEHLP64}
{$else}
function SymGetSymFromAddr(hProcess:THANDLE; dwAddr:dword; pdwDisplacement:PDWORD; Symbol:PIMAGEHLP_SYMBOL):BOOL;stdcall;external External_library name 'SymGetSymFromAddr';
function SymGetSymFromName(hProcess:THANDLE; Name:PSTR; Symbol:PIMAGEHLP_SYMBOL):BOOL;stdcall;external External_library name 'SymGetSymFromName';
{$endif}

function FindFileInPath(hprocess:THANDLE; SearchPath:lpstr; FileName:lpstr; id:pointer; two:dword; 
             three:dword; flags:dword; FilePath:lpstr):BOOL;stdcall;external External_library name 'FindFileInPath';
function FindFileInSearchPath(hprocess:THANDLE; SearchPath:lpstr; FileName:lpstr; one:dword; two:dword; 
             three:dword; FilePath:lpstr):BOOL;stdcall;external External_library name 'FindFileInSearchPath';
function SymEnumSym(hProcess:THANDLE; BaseOfDll:ULONG64; EnumSymbolsCallback:TSYM_ENUMERATESYMBOLS_CALLBACK; UserContext:pointer):BOOL;stdcall;external External_library name 'SymEnumSym';
function MiniDumpWriteDump(hProcess:THANDLE; ProcessId:dword; hFile:THANDLE; DumpType:TMINIDUMP_TYPE; ExceptionParam:PMINIDUMP_EXCEPTION_INFORMATION; 
             UserStreamParam:PMINIDUMP_USER_STREAM_INFORMATION; CallbackParam:TMINIDUMP_CALLBACK_INFORMATION):BOOL;stdcall;external External_library name 'MiniDumpWriteDump';
	
function MiniDumpReadDumpStream(BaseOfDump:pointer; StreamNumber:ULONG; Dir:PPMINIDUMP_DIRECTORY; StreamPointer:PPOINTER; StreamSize:PULONG):BOOL;stdcall;external External_library name 'MiniDumpReadDumpStream';

function RVA_TO_ADDR(Mapping:ULONG_PTR;RVA:CARDINAL):POINTER; inline;
procedure Address64To32(a64: LPADDRESS64 ;a32:LPADDRESS); inline;
Procedure Address32To64(a32: LPADDRESS ;a64:LPADDRESS64); inline;
procedure KdHelp32To64(p32:PKDHELP;p64:PKDHELP64); inline;
implementation

function RVA_TO_ADDR(Mapping:ULONG_PTR;RVA:CARDINAL):POINTER; 
begin
  rva_to_addr:=pointer(ulong_ptr(mapping)+rva);
end;
 
Procedure Address32To64(a32: LPADDRESS ;a64:LPADDRESS64);
begin
    a64^.Offset := ULONG64(LONG64(LONG(a32^.Offset))); // I assume these multiple typecasts are to avoid sign problems?
    a64^.Segment := a32^.Segment;
    a64^.Mode := a32^.Mode;
end;

procedure Address64To32(a64: LPADDRESS64 ;a32:LPADDRESS);
begin
    a32^.Offset := ULONG(a64^.Offset);
    a32^.Segment:= a64^.Segment;
    a32^.Mode   := a64^.Mode;
end;

procedure KdHelp32To64(p32:PKDHELP;p64:PKDHELP64);
begin
    p64^.Thread := p32^.Thread;
    p64^.ThCallbackStack := p32^.ThCallbackStack;
    p64^.NextCallback := p32^.NextCallback;
    p64^.FramePointer := p32^.FramePointer;
    p64^.KiCallUserMode := p32^.KiCallUserMode;
    p64^.KeUserCallbackDispatcher := p32^.KeUserCallbackDispatcher;
    p64^.SystemRangeStart := p32^.SystemRangeStart;
end;
 
<<<<<<< HEAD
<<<<<<< HEAD
end.
=======
end.
>>>>>>> graemeg/fixes_2_2
=======
end.
>>>>>>> origin/fixes_2_2
