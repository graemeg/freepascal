{
    Copyright (c) 1989, 1991 IBM Corporation
    Copyright (c) 2003 by Yuri Prokushev (prokushev@freemail.ru).

    REXX SAA Interface

    This program is free software; you can redistribute it and/or modify it
    under the terms of the GNU Library General Public License (LGPL) as
    published by the Free Software Foundation; either version 2 of the
    License, or (at your option) any later version. This program is
    distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or
    FITNESS FOR A PARTICULAR PURPOSE.

    See the GNU Library General Public License for more details. You should
    have received a copy of the GNU Library General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.

 **********************************************************************}

{
@abstract(REXX SAA interface)
@author(Yuri Prokushev (prokushev@freemail.ru))
@created(01 Feb 2003)
@lastmod(13 Feb 2003)
REXX SAA Interface
Warning: This code is alfa. Future versions of this unit will propably
not be compatible.
}
(*********************************************************************
*
* REXX SAA Interface
*
*********************************************************************)
Unit RexxSAA;

Interface

{$Mode ObjFpc}

Const
{$IFDEF OS2}
  REXX='REXX';
  REXXAPI='REXXAPI';
{$ELSE}
  {$IFDEF UNIX}
    REXX='libregina';
    REXXAPI='libregina';
  {$ELSE}
    REXX='REGINA';
    REXXAPI='REGINA';
  {$ENDIF}
{$ENDIF}
//********************************************************************
//                                                                   *
//                            Common                                 *
//                                                                   *
//********************************************************************

//* This section defines return codes and constants that are the      */
//* same for both 16-bit and 32-bit REXX calls.                       */

//Structure for external interface string (RXSTRING)

Type
  RxString=record
    StrLength: Cardinal; // Length of string
    StrPtr: PChar;       // Pointer to a string
  end;
  PRxString=^RxString;   // pointer to a RXSTRING

Const
  RxAutoBufLen=256;

Type
  PFN=Pointer;

//Structure for system exit block (RXSYSEXIT) 32-bit

Type
  RxSysExit=record
    sysexit_name: PChar;                 // subcom enviro for sysexit
    sysexit_code: Longint;               // sysexit function code
  end;
  PRxSysExit=^RxSysExit;        // pointer to a RXSYSEXIT

// Macros for RXSTRING manipulation
Function RXNULLSTRING(CONST r:RXSTRING):BOOLEAN;
Function RXZEROLENSTRING(CONST r:RXSTRING):BOOLEAN;
Function RXVALIDSTRING(CONST r:RXSTRING):BOOLEAN;
Function RXSTRLEN(CONST r:RXSTRING):LONGINT;
Function RXSTRPTR(CONST r:RXSTRING):PChar;
Procedure MAKERXSTRING(VAR r:RXSTRING;p:PChar;l:LONGINT);

//Call type codes for use on interpreter startup
Const
  RxCommand=0;              // Program called as Command
  RxSubroutine=1;           // Program called as Subroutin
  RxFunction=2;             // Program called as Function

// Subcommand Interface defines


// Drop Authority for RXSUBCOM interface

Const
  RxSubCom_Droppable=$00;     // handler to be dropped by all
  RxSubCom_Nondrop=$01;       // process with same PID as the
                              // registrant may drop environ

// Return Codes from RXSUBCOM interface
Const
  RXSUBCOM_ISREG     = $01;    // Subcommand is registered
  RXSUBCOM_ERROR     = $01;    // Subcommand Ended in Error
  RXSUBCOM_FAILURE   = $02;    // Subcommand Ended in Failure
  RXSUBCOM_BADENTRY  = 1001;    // Invalid Entry Conditions
  RXSUBCOM_NOEMEM    = 1002;    // Insuff stor to complete req
  RXSUBCOM_BADTYPE   = 1003;    // Bad registration type.
  RXSUBCOM_NOTINIT   = 1004;    // API system not initialized.
  RXSUBCOM_OK        =  0;      // Function Complete
  RXSUBCOM_DUP       = 10;      // Duplicate Environment Name-
                                // but Registration Completed
  RXSUBCOM_MAXREG    = 20;      // Cannot register more
                                // handlers
  RXSUBCOM_NOTREG    = 30;      // Name Not Registered
  RXSUBCOM_NOCANDROP = 40;      // Name not droppable
  RXSUBCOM_LOADERR   = 50;      // Could not load function
  RXSUBCOM_NOPROC    =127;      // RXSUBCOM routine - not found

// Shared Variable Pool Interface defines

// Function Codes for Variable Pool Interface (shvcode)
Const
  RXSHV_SET     =     $00;      // Set var from given value
  RXSHV_FETCH   =     $01;      // Copy value of var to buffer
  RXSHV_DROPV   =     $02;      // Drop variable
  RXSHV_SYSET   =     $03;      // Symbolic name Set variable
  RXSHV_SYFET   =     $04;      // Symbolic name Fetch variable
  RXSHV_SYDRO   =     $05;      // Symbolic name Drop variable
  RXSHV_NEXTV   =     $06;      // Fetch "next" variable
  RXSHV_PRIV    =     $07;      // Fetch private information
  RXSHV_EXIT    =     $08;      // Set function exit value

// Return Codes for Variable Pool Interface

Const
  RXSHV_NOAVL       =  144;      // Interface not available

// Return Code Flags for Variable Pool Interface (shvret)

Const
  RXSHV_OK       =    $00;      // Execution was OK
  RXSHV_NEWV     =    $01;      // Variable did not exist
  RXSHV_LVAR     =    $02;      // Last var trans via SHVNEXTV
  RXSHV_TRUNC    =    $04;      // Truncation occurred-Fetch
  RXSHV_BADN     =    $08;      // Invalid variable name
  RXSHV_MEMFL    =    $10;      // Out of memory failure
  RXSHV_BADF     =    $80;      // Invalid funct code (shvcode)

// Structure of Shared Variable Request Block (SHVBLOCK)
Type
  PSHVBLOCK=^SHVBLOCK;
  SHVBLOCK=record
    shvnext: PSHVBLOCK;      // pointer to the next block
    shvname: RxString;       // Pointer to the name buffer
    shvvalue: RxString;      // Pointer to the value buffer
    shvnamelen: Cardinal;    // Length of the name value
    shvvaluelen: Cardinal;   // Length of the fetch value
    shvcode: Byte;           // Function code for this block
    shvret: Byte;            // Individual Return Code Flags
  end;

// External Function Interface

// Registration Type Identifiers for Available Function Table
Const
  RXFUNC_DYNALINK=1;        // Function Available in DLL
  RXFUNC_CALLENTRY=2;       // Registered as mem entry pt.

// Return Codes from RxFunction interface
Const
  RXFUNC_OK          =   0;        // REXX-API Call Successful
  RXFUNC_DEFINED     =  10;        // Function Defined in AFT
  RXFUNC_NOMEM       =  20;        // Not Enough Mem to Add
  RXFUNC_NOTREG      =  30;        // Funct Not Registered in AFT
  RXFUNC_MODNOTFND   =  40;        // Funct Dll Module Not Found
  RXFUNC_ENTNOTFND   =  50;        // Funct Entry Point Not Found
  RXFUNC_NOTINIT     =  60;        // API not initialized
  RXFUNC_BADTYPE     =  70;        // Bad function type

// System Exits defines

// Drop Authority for Rexx Exit interface
Const
  RXEXIT_DROPPABLE   =  $00;     // handler to be dropped by all
  RXEXIT_NONDROP     =  $01;     // process with same PID as the
                                 // registrant may drop environ
// Exit return actions
Const
  RXEXIT_HANDLED     = 0;        // Exit handled exit event
  RXEXIT_NOT_HANDLED = 1;        // Exit passes on exit event
  RXEXIT_RAISE_ERROR =-1;        // Exit handler error occurred

// Return Codes from RXEXIT interface
Const
  RXEXIT_ISREG       =  $01;    // Exit is registered
  RXEXIT_ERROR       =  $01;    // Exit Ended in Error
  RXEXIT_FAILURE     =  $02;    // Exit Ended in Failure
  RXEXIT_BADENTRY    = 1001;    // Invalid Entry Conditions
  RXEXIT_NOEMEM      = 1002;    // Insuff stor to complete req
  RXEXIT_BADTYPE     = 1003;    // Bad registration type.
  RXEXIT_NOTINIT     = 1004;    // API system not initialized.
  RXEXIT_OK          =  0;      // Function Complete
  RXEXIT_DUP         = 10;      // Duplicate Exit Name-
                                // but Registration Completed
  RXEXIT_MAXREG      = 20;      // Cannot register more
                                // handlers
  RXEXIT_NOTREG      = 30;      // Name Not Registered
  RXEXIT_NOCANDROP   = 40;      // Name not droppable
  RXEXIT_LOADERR     = 50;      // Could not load function
  RXEXIT_NOPROC      = 127;     // RXEXIT routine - not found

// System Exit function and sub-function definitions
Const
  RXENDLST    = 0;                 // End of exit list.
  RXFNC       = 2;                 // Process external functions.
    RXFNCCAL  = 1;                 // subcode value.
  RXCMD       = 3;                 // Process host commands.
    RXCMDHST  = 1;                 // subcode value.
  RXMSQ       = 4;                 // Manipulate queue.
    RXMSQPLL  = 1;                 // Pull a line from queue
    RXMSQPSH  = 2;                 // Place a line on queue
    RXMSQSIZ  = 3;                 // Return num of lines on queue
    RXMSQNAM  = 20;                // Set active queue name
  RXSIO       = 5;                 // Session I/O.
    RXSIOSAY  = 1;                 // SAY a line to STDOUT
    RXSIOTRC  = 2;                 // Trace output
    RXSIOTRD  = 3;                 // Read from char stream
    RXSIODTR  = 4;                 // DEBUG read from char stream
    RXSIOTLL  = 5;                 // Return linelength(N/A OS/2)
  RXHLT       = 7;                 // Halt processing.
    RXHLTCLR  = 1;                 // Clear HALT indicator
    RXHLTTST  = 2;                 // Test HALT indicator
  RXTRC       = 8;                 // Test ext trace indicator.
    RXTRCTST  = 1;                 // subcode value.
  RXINI       = 9;                 // Initialization processing.
    RXINIEXT  = 1;                 // subcode value.
  RXTER       = 10;                // Termination processing.
    RXTEREXT  = 1;                 // subcode value.
  RXNOOFEXITS = 11;                // 1 + largest exit number.

Type
  PEXIT=^Byte;                 // ptr to exit parameter block

// Asynchronous Request Interface defines

// Return Codes from Asynchronous Request interface
Const
  RXARI_OK                 = 0;  // Interface completed
  RXARI_NOT_FOUND          = 1;  // Target program not found
  RXARI_PROCESSING_ERROR   = 2;  // Error processing request

// Macro Space Interface defines

// Registration Search Order Flags
Const
  RXMACRO_SEARCH_BEFORE      = 1;  // Beginning of search order
  RXMACRO_SEARCH_AFTER       = 2;  // End of search order

// Return Codes from RxMacroSpace interface
Const
  RXMACRO_OK                = 0;  // Macro interface completed
  RXMACRO_NO_STORAGE        = 1;  // Not Enough Storage Available
  RXMACRO_NOT_FOUND         = 2;  // Requested function not found
  RXMACRO_EXTENSION_REQUIRED= 3;  // File ext required for save
  RXMACRO_ALREADY_EXISTS    = 4;  // Macro functions exist
  RXMACRO_FILE_ERROR        = 5;  // File I/O error in save/load
  RXMACRO_SIGNATURE_ERROR   = 6;  // Incorrect format for load
  RXMACRO_SOURCE_NOT_FOUND  = 7;  // Requested cannot be found
  RXMACRO_INVALID_POSITION  = 8;  // Invalid search order pos
  RXMACRO_NOT_INIT          = 9;  // API not initialized

//*********************************************************************/
//*                                                                   */
//*                            32-bit                                 */
//*                                                                   */
//*********************************************************************/

// Main Entry Point to the REXXSAA Interpreter

Function RexxStart(ArgC: Longint;      // Num of args passed to rexx
         ArgV: PRXSTRING;              // Array of args passed to rex
         Filename: PChar;              // [d:][path] filename[.ext]
         Proc: PRXSTRING;              // Loc of rexx proc in memory
         Env: PChar;                   // ASCIIZ initial environment.
         rType: Longint;               // type (command,subrtn,funct)
         Exit_: PRXSYSEXIT;             // SysExit env. names &  codes
         Ret: PInteger;                   // Ret code from if numeric
         RetVal: PRXSTRING): Longint; cdecl;  // Retvalue from the rexx proc

Function RexxStart(ArgC: Longint;      // Num of args passed to rexx
         ArgV: PRXSTRING;              // Array of args passed to rex
         Filename: PChar;              // [d:][path] filename[.ext]
         Proc: PRXSTRING;              // Loc of rexx proc in memory
         Env: PChar;                   // ASCIIZ initial environment.
         rType: Longint;               // type (command,subrtn,funct)
         Exit_: PRXSYSEXIT;             // SysExit env. names &  codes
     var Ret: Integer;                    // Ret code from if numeric
     var RetVal: RXSTRING): Longint; cdecl;  // Retvalue from the rexx proc

// Subcommand Interface

// This type simplifies coding of a Subcommand handler.
Type
  RexxSubcomHandler=function(a: PRXSTRING; b: PWord; c: PRXSTRING): Cardinal; cdecl;

// RexxRegisterSubcomDll -- Register a DLL entry point
// as a Subcommand handler

Function RexxRegisterSubcomDll(
         HName:    PChar;       // Name of subcom handler
         DllName:  PChar;       // Name of DLL
         ProcName: PChar;       // Name of procedure in DLL
         UserArea: PWord;       // User area
         Drop: Cardinal): Cardinal; cdecl;  // Drop authority.

// RexxRegisterSubcomExe -- Register an EXE entry point
// as a Subcommand handler

Function RexxRegisterSubcomExe(
         HName:       PChar;            // Name of subcom handler
         HandlerAddr: PFn;              // address of handler in EXE
         UserArea:    PWord): Cardinal; cdecl; // User area

// RexxQuerySubcom - Query an environment for Existance

Function RexxQuerySubcom(
         EnvName: PChar;                // Name of the Environment
         DllName: PChar;                // DLL Module Name
         ExCode:  PWord;                // Stor for existance code
         User:    PWord): Cardinal; cdecl;       // Stor for user word

Function RexxQuerySubcom(
         EnvName: PChar;                // Name of the Environment
         DllName: PChar;                // DLL Module Name
     var ExCode:  Word;                 // Stor for existance code
     var User:    Word): Cardinal; cdecl;        // Stor for user word

// RexxDeregisterSubcom - Drop registration of a Subcommand
// environment

Function RexxDeregisterSubcom(
         EnvName: PChar;                   // Name of the Environment
         DllName: PChar): Cardinal; cdecl; // DLL Module Name

// Shared Variable Pool Interface

// RexxVariablePool - Request Variable Pool Service

Function RexxVariablePool(Pool: PShvBlock): Cardinal; cdecl;
Function RexxVariablePool(var Pool: ShvBlock): Cardinal; cdecl; // Pointer to list of SHVBLOCKs

// External Function Interface

// This typedef simplifies coding of an External Function.
Type
  RexxFunctionHandler=Function(a: PByte;
                                b: Cardinal;
                             var c: RXSTRING;
                                 d: PChar;
                             var e: RXSTRING): Cardinal; cdecl;

// RexxRegisterFunctionDll - Register a function in the AFT

Function RexxRegisterFunctionDll(
        FnName:  PChar;                   // Name of function to add
        DllName: PChar;                   // Dll file name (if in dll)
        Entry:   PChar): Cardinal; cdecl; // Entry in dll

// RexxRegisterFunctionExe - Register a function in the AFT

Function RexxRegisterFunctionExe(
        FnName: PChar;                  // Name of function to add
        Entry:  PFn): Cardinal; cdecl;  // Entry point in EXE

// RexxDeregisterFunction - Delete a function from the AFT

Function RexxDeregisterFunction(FnName: PChar): Cardinal; cdecl; // Name of function to remove

// RexxQueryFunction - Scan the AFT for a function

Function RexxQueryFunction(FnName: PChar): Cardinal; cdecl; // Name of function to find

// System Exits

// Subfunction RXFNCCAL - External Function Calls

//rxfnc_flags flags
const
  rxfferr                       = $01;  // Invalid call to routine.
  rxffnfnd                      = $02;  // Function not found.
  rxffsub                       = $04;  // Called as a subroutine

type
  RxFnCCal_Parm = record
    rxfnc_flags: Byte;          // function flags
    rxfnc_name:  PChar;         // Pointer to function name.
    rxfnc_namel: Word;          // Length of function name.
    rxfnc_que:   PChar;         // Current queue name.
    rxfnc_quel:  Word;          // Length of queue name.
    rxfnc_argc:  Word;          // Number of args in list.
    rxfnc_argv:  PRxString;     // Pointer to argument list.
    rxfnc_retc:  RxString;      // Return value.
  end;

// Subfunction RXCMDHST -- Process Host Commands

// rxcmd_flags flags
const
  rxfcfail                      = $01;  // Command failed.
  rxfcerr                       = $02;  // Command ERROR occurred.

type
  RxCmdHst_Parm = record
    rxcmd_flags:    Byte;               // error/failure flags
    rxcmd_address:  PChar;              // Pointer to address name.
    rxcmd_addressl: Word;               // Length of address name.
    rxcmd_dll:      PChar;              // dll name for command.
    rxcmd_dll_len:  Word;               // Length of dll name.
    rxcmd_command:  RxString;           // The command string.
    rxcmd_retc:     RxString;           // Pointer to return buffer
  end;


// Subfunction RXMSQPLL -- Pull Entry from Queue

  RxMsqPll_Parm = record
    rxmsq_retc: RxString;               // Pointer to dequeued entry
  end;                                  // buffer.  User allocated.


// Subfunction RXMSQPSH -- Push Entry on Queue

// rxmsq_flags flags
const
   rxfmlifo                     = $01;  // Stack entry LIFO if set

type
  RxMsqPsh_Parm = record
     rxmsq_flags: Byte;                 // LIFO/FIFO flag
     rxmsq_value: RxString              // The entry to be pushed.
  end;

// Subfunction RXMSQSIZ -- Return the Current Queue Size
Type
  RxMsqSiz_Parm = record
    rxmsq_size: Cardinal;                  // Number of Lines in Queue
  end;


// Subfunction RXMSQNAM -- Set Current Queue Name
Type
  RxMsqNam_Parm = record
    rxmsq_name: RxString;               // RxString containing
  end;                                  // queue name.


// Subfunction RXSIOSAY -- Perform SAY Clause

Type
  RxSioSay_Parm = record
    rxsio_string: RxString;             // String to display.
  end;


// Subfunction RXSIOTRC -- Write Trace Output
Type
  RxSioTrc_Parm = record
    rxsio_string: RxString;             // Trace line to display.
  end;

// Subfunction RXSIOTRD -- Read Input from the Terminal
Type
  RxSioTrd_Parm = record
    rxsiotrd_retc: RxString;            // RxString for output.
  end;

// Subfunction RXSIODTR -- Read Debug Input from the Terminal
Type
  RxSioDtr_Parm = record
    rxsiodtr_retc: RxString;            // RxString for output.
  end;

// Subfunction RXHSTTST -- Test for HALT Condition

// rxhlt_flags flags
const
  rxfhhalt                      = $01;  // Set if HALT occurred.

type
  RxHltTst_Parm = record
    rxhlt_flags: Byte;                  // Set if HALT occurred
  end;


// Subfunction RXTRCTST -- Test for TRACE Condition

// rxtrc_flags flags
const
  rxftrace                      = $01;  // Set to run external trace.
type
  RxTrcTst_Parm = record
    rxtrc_flags: Byte;                  // Set to run external trace
  end;

// This typedef simplifies coding of an Exit handler.
Type
  RexxExitHandler=function(A: Longint;
                           B: Longint;
                           C: PEXIT): Cardinal; cdecl;

// RexxRegisterExitDll - Register a system exit.

Function RexxRegisterExitDll(
         HExit:    PChar;               // Name of the exit handler
         DllName:  PChar;               // Name of the DLL
         ProcName: PChar;               // Name of the procedure
         UserArea: PByte;               // User area
         Drop:     Cardinal): Cardinal; cdecl;  // Drop authority


// RexxRegisterExitExe - Register a system exit.

Function RexxRegisterExitExe(
         HExit: PChar;                         // Name of the exit handler
         HandlerAddr: PFn;                     // Address of exit handler
         UserArea: PByte): Cardinal; cdecl;    // User area

// RexxDeregisterExit - Drop registration of a system exit.

Function RexxDeregisterExit(
         ExitName: PChar;                   // Exit name
         DllName:  PChar): Cardinal; cdecl; // DLL module name

// RexxQueryExit - Query an exit for existance.

Function RexxQueryExit(
         ExitName: PChar;               // Exit name
         DllName:  PChar;               // DLL Module name.
     var ExFlag:   Word;                // Existance flag.
         UserArea: Pointer): Cardinal; cdecl;    // User data.

Function RexxQueryExit(
         ExitName: PChar;                   // Exit name
         DllName:  PChar;                   // DLL Module name.
     var ExFlag:   Word;                    // Existance flag.
         UserArea: PByte): Cardinal; cdecl; // User data.

Function RexxQueryExit(
         ExitName: PChar;                   // Exit name
         DllName:  PChar;                   // DLL Module name.
         ExFlag:   PWord;                   // Existance flag.
         UserArea: PByte): Cardinal; cdecl; // User data.

Function RexxQueryExit(
         ExitName: PChar;                   // Exit name
         DllName:  PChar;                   // DLL Module name.
         ExFlag:   PWord;                   // Existance flag.
         UserArea: Pointer): Cardinal; cdecl; // User data.

// Asynchronous Request Interface

// RexxSetHalt - Request Program Halt

Function RexxSetHalt(
         Pid: Longint;                      // Process Id
         Tid: Longint): Cardinal; cdecl;    // Thread Id

// RexxSetTrace - Request Program Trace

Function RexxSetTrace(
         Pid: Longint;                      // Process Id
         Tid: Longint): Cardinal; cdecl;    // Thread Id

// RexxResetTrace - Turn Off Program Trace

Function RexxResetTrace(
         Pid: Longint;                      // Process Id
         Tid: Longint): Cardinal; cdecl;    // Thread Id

// Macro Space Interface

// RexxAddMacro - Register a function in the Macro Space

Function RexxAddMacro(
         FnName:   PChar;                      // Function to add or change
         FileName: PChar;                      // Name of file to get function
         SrchPos:  Cardinal): Cardinal; cdecl; // Flag indicating search pos

// RexxDropMacro - Remove a function from the Macro Space

Function RexxDropMacro(FnName: PChar): Cardinal; cdecl; // Name of function to remove

// RexxSaveMacroSpace - Save Macro Space functions to a file

Function RexxSaveMacroSpace(
         ArgC: Cardinal;                  // Argument count (0==save all)
     var NameLst: PChar;                  // List of funct names to save
         FileName: PChar): Cardinal; cdecl; // File to save functions in

// RexxLoadMacroSpace - Load Macro Space functions from a file

Function RexxLoadMacroSpace(
         ArgC:     Cardinal;                // Argument count (0==load all)
     var NameLst:  PChar;                   // List of funct names to load
         FileName: PChar): Cardinal; cdecl; // File to load functions from

// RexxQueryMacro - Find a function's search-order position

Function RexxQueryMacro(
         FnName: PChar;                  // Function to search for
     var PtrPos: Word): Cardinal; cdecl; // Ptr for position flag return

// RexxReorderMacro - Change a function's search-order
//                          position

Function RexxReorderMacro(
         FnName: PChar;                      // Name of funct change order
         NewPos: Cardinal): Cardinal; cdecl; // New position for function

// RexxClearMacroSpace - Remove all functions from a MacroSpace

Function RexxClearMacroSpace: Cardinal; cdecl; // No Arguments.

(* Not supported yet!!
/* REGINA EXTENSIONS *********************************************************/
/* The following function is an extension to the standard. Never try to
 * address the function directly. Use the dynamic linking machanism of
 * your operating system instead. This function was introduced in version
 * 2.0.
 * Returns: ULONG, in lower byte the two-digit fraction part of the version.
 *          The higher bytes will hold the integer part of the version.
 *          Examples: 0x10A codes the Version "1.10".
 * VersionString will be filled if VersionString is non-NULL.
 * If VersionString is non-NULL then there are two possibilities:
 * a) VersionString->strlength == 0: VersionString is filled with the
 *                                   appropriate values. VersionString->strptr
 *                                   is always created.
 * b) VersionString->strlength != 0: VersionString->strptr is filled up to
 *                                   this value. VersionString->strlength will
 *                                   hold the copied bytes.
 * Note: A terminating ASCII-zero is appended if there is enough space
 *       although it is never counted in VersionString.strlength.
 *       RexxAllocateMemory is used if needed.
 */
*)
{
Type
  ReginaVersion=Function(var VersionString: RxString);
}

Implementation

Function RexxStart(ArgC: Longint;      // Num of args passed to rexx
         ArgV: PRXSTRING;              // Array of args passed to rex
         Filename: PChar;              // [d:][path] filename[.ext]
         Proc: PRXSTRING;              // Loc of rexx proc in memory
         Env: PChar;                   // ASCIIZ initial environment.
         rType: Longint;               // type (command,subrtn,funct)
         Exit_: PRXSYSEXIT;            // SysExit env. names &  codes
         Ret: PInteger;                // Ret code from if numeric
         RetVal: PRXSTRING): Longint; cdecl;  // Retvalue from the rexx proc
    external REXX name 'RexxStart';

Function RexxStart(ArgC: Longint;      // Num of args passed to rexx
         ArgV: PRXSTRING;              // Array of args passed to rex
         Filename: PChar;              // [d:][path] filename[.ext]
         Proc: PRXSTRING;              // Loc of rexx proc in memory
         Env: PChar;                   // ASCIIZ initial environment.
         rType: Longint;               // type (command,subrtn,funct)
         Exit_: PRXSYSEXIT;            // SysExit env. names &  codes
     var Ret: integer;                 // Ret code from if numeric
     var RetVal: RXSTRING): Longint; cdecl;  // Retvalue from the rexx proc
    external REXX name 'RexxStart';

Function RexxRegisterSubcomDll(
         HName:    PChar;       // Name of subcom handler
         DllName:  PChar;       // Name of DLL
         ProcName: PChar;       // Name of procedure in DLL
         UserArea: PWord;       // User area
         Drop: Cardinal): Cardinal; cdecl;  // Drop authority.
    external REXXAPI name 'RexxRegisterSubcomDll';

Function RexxRegisterSubcomExe(
         HName:       PChar;            // Name of subcom handler
         HandlerAddr: PFn;              // address of handler in EXE
         UserArea:    PWord): Cardinal; cdecl; // User area
    external REXXAPI name 'RexxRegisterSubcomExe';

Function RexxQuerySubcom(
         EnvName: PChar;                // Name of the Environment
         DllName: PChar;                // DLL Module Name
         ExCode:  PWord;                // Stor for existance code
         User:    PWord): Cardinal; cdecl;       // Stor for user word
    external REXXAPI name 'RexxQuerySubcom';

Function RexxQuerySubcom(
         EnvName: PChar;                // Name of the Environment
         DllName: PChar;                // DLL Module Name
     var ExCode:  Word;                 // Stor for existance code
     var User:    Word): Cardinal; cdecl;        // Stor for user word
    external REXXAPI name 'RexxQuerySubcom';

Function RexxDeregisterSubcom(
         EnvName: PChar;                   // Name of the Environment
         DllName: PChar): Cardinal; cdecl; // DLL Module Name
    external REXXAPI name 'RexxDeregisterSubcom';

Function RexxVariablePool(Pool: PShvBlock): Cardinal; cdecl;
    external REXX name 'RexxVariablePool';
Function RexxVariablePool(var Pool: ShvBlock): Cardinal; cdecl; // Pointer to list of SHVBLOCKs
    external REXX name 'RexxVariablePool';

Function RexxRegisterFunctionDll(
        FnName:  PChar;                   // Name of function to add
        DllName: PChar;                   // Dll file name (if in dll)
        Entry:   PChar): Cardinal; cdecl; // Entry in dll
    external REXXAPI name 'RexxRegisterFunctionDll';

Function RexxRegisterFunctionExe(
        FnName: PChar;                  // Name of function to add
        Entry:  PFn): Cardinal; cdecl;  // Entry point in EXE
    external REXXAPI name 'RexxRegisterFunctionExe';

Function RexxDeregisterFunction(FnName: PChar): Cardinal; cdecl; // Name of function to remove
    external REXXAPI name 'RexxDeregisterFunction';

Function RexxQueryFunction(FnName: PChar): Cardinal; cdecl; // Name of function to find
    external REXXAPI name 'RexxQueryFunction';

Function RexxRegisterExitDll(
         HExit:    PChar;               // Name of the exit handler
         DllName:  PChar;               // Name of the DLL
         ProcName: PChar;               // Name of the procedure
         UserArea: PByte;               // User area
         Drop:     Cardinal): Cardinal; cdecl;  // Drop authority
    external REXXAPI name 'RexxRegisterExitDll';

Function RexxRegisterExitExe(
         HExit: PChar;                         // Name of the exit handler
         HandlerAddr: PFn;                     // Address of exit handler
         UserArea: PByte): Cardinal; cdecl;    // User area
    external REXXAPI name 'RexxRegisterExitExe';

Function RexxDeregisterExit(
         ExitName: PChar;                   // Exit name
         DllName:  PChar): Cardinal; cdecl; // DLL module name
    external REXXAPI name 'RexxDeregisterExit';

Function RexxQueryExit(
         ExitName: PChar;               // Exit name
         DllName:  PChar;               // DLL Module name.
     var ExFlag:   Word;                // Existance flag.
         UserArea: Pointer): Cardinal; cdecl;    // User data.
    external REXXAPI name 'RexxQueryExit';

Function RexxQueryExit(
         ExitName: PChar;                   // Exit name
         DllName:  PChar;                   // DLL Module name.
     var ExFlag:   Word;                    // Existance flag.
         UserArea: PByte): Cardinal; cdecl; // User data.
    external REXXAPI name 'RexxQueryExit';

Function RexxQueryExit(
         ExitName: PChar;                   // Exit name
         DllName:  PChar;                   // DLL Module name.
         ExFlag:   PWord;                   // Existance flag.
         UserArea: PByte): Cardinal; cdecl; // User data.
    external REXXAPI name 'RexxQueryExit';

Function RexxQueryExit(
         ExitName: PChar;                   // Exit name
         DllName:  PChar;                   // DLL Module name.
         ExFlag:   PWord;                   // Existance flag.
         UserArea: Pointer): Cardinal; cdecl; // User data.
    external REXXAPI name 'RexxQueryExit';

Function RexxSetHalt(
         Pid: Longint;                      // Process Id
         Tid: Longint): Cardinal; cdecl;    // Thread Id
    external REXX name 'RexxSetHalt';

Function RexxSetTrace(
         Pid: Longint;                      // Process Id
         Tid: Longint): Cardinal; cdecl;    // Thread Id
    external REXX name 'RexxSetTrace';

Function RexxResetTrace(
         Pid: Longint;                      // Process Id
         Tid: Longint): Cardinal; cdecl;    // Thread Id
    external REXX name 'RexxResetTrace';

Function RexxAddMacro(
         FnName:   PChar;                      // Function to add or change
         FileName: PChar;                      // Name of file to get function
         SrchPos:  Cardinal): Cardinal; cdecl; // Flag indicating search pos
    external REXXAPI name 'RexxAddMacro';

Function RexxDropMacro(FnName: PChar): Cardinal; cdecl; // Name of function to remove
    external REXXAPI name 'RexxDropMacro';

Function RexxSaveMacroSpace(
         ArgC: Cardinal;                  // Argument count (0==save all)
     var NameLst: PChar;                  // List of funct names to save
         FileName: PChar): Cardinal; cdecl; // File to save functions in
    external REXXAPI name 'RexxSaveMacroSpace';

Function RexxLoadMacroSpace(
         ArgC:     Cardinal;                // Argument count (0==load all)
     var NameLst:  PChar;                   // List of funct names to load
         FileName: PChar): Cardinal; cdecl; // File to load functions from
    external REXXAPI name 'RexxLoadMacroSpace';

Function RexxQueryMacro(
         FnName: PChar;                  // Function to search for
     var PtrPos: Word): Cardinal; cdecl; // Ptr for position flag return
    external REXXAPI name 'RexxQueryLoadMacro';

Function RexxReorderMacro(
         FnName: PChar;                      // Name of funct change order
         NewPos: Cardinal): Cardinal; cdecl; // New position for function
    external REXXAPI name 'RexxReorderMacro';

Function RexxClearMacroSpace: Cardinal; cdecl; // No Arguments.
    external REXXAPI name 'RexxClearMacroSpace';

Function RxNullString(const r: RxString): Boolean;
Begin
  RxNullString:=r.strptr=nil;
End;

Function RxZeroLenString(const r: RxString): Boolean;
Begin
  RxZeroLenString:=((r.strptr<>nil) and (r.strlength=0));
End;

Function RxValidString(const r: RxString): Boolean;
Begin
  RxValidString:=((r.strptr<>nil) and (r.strlength>0));
End;

Function RxStrLen(const r: RxString): Longint;
Begin
  If RxNullString(r) then
    RxStrLen:=0
  else
    RxStrLen:=r.strlength;
End;

Function RxStrPtr(Const r: RxString): PChar;
Begin
  RxStrPtr:=r.strptr;
End;

Procedure MakeRxString(Var r: RxString; p: PChar; l: Longint);
Begin
  r.strptr:=p;
  r.strlength:=l;
End;

End.
