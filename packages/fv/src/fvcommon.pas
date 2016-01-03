{********************[ COMMON UNIT ]***********************}
{                                                          }
{    System independent COMMON TYPES & DEFINITIONS         }
{                                                          }
{    Parts Copyright (c) 1997 by Balazs Scheidler          }
{    bazsi@balabit.hu                                      }
{                                                          }
{    Parts Copyright (c) 1999, 2000 by Leon de Boer        }
{    ldeboer@attglobal.net  - primary e-mail address       }
{    ldeboer@projectent.com.au - backup e-mail address     }
{                                                          }
{****************[ THIS CODE IS FREEWARE ]*****************}
{                                                          }
{     This sourcecode is released for the purpose to       }
{   promote the pascal language on all platforms. You may  }
{   redistribute it and/or modify with the following       }
{   DISCLAIMER.                                            }
{                                                          }
{     This SOURCE CODE is distributed "AS IS" WITHOUT      }
{   WARRANTIES AS TO PERFORMANCE OF MERCHANTABILITY OR     }
{   ANY OTHER WARRANTIES WHETHER EXPRESSED OR IMPLIED.     }
{                                                          }
{*****************[ SUPPORTED PLATFORMS ]******************}
{     16 and 32 Bit compilers                              }
{        DOS      - Turbo Pascal 7.0 +      (16 Bit)       }
{        DPMI     - Turbo Pascal 7.0 +      (16 Bit)       }
{                 - FPC 0.9912+ (GO32V2)    (32 Bit)       }
{        WINDOWS  - Turbo Pascal 7.0 +      (16 Bit)       }
{                 - Delphi 1.0+             (16 Bit)       }
{        WIN95/NT - Delphi 2.0+             (32 Bit)       }
{                 - Virtual Pascal 2.0+     (32 Bit)       }
{                 - Speedsoft Sybil 2.0+    (32 Bit)       }
{                 - FPC 0.9912+             (32 Bit)       }
{        OS2      - Virtual Pascal 1.0+     (32 Bit)       }
{                 - Speed Pascal 1.0+       (32 Bit)       }
{                 - C'T patch to BP         (16 Bit)       }
{                                                          }
{******************[ REVISION HISTORY ]********************}
{  Version  Date      Who    Fix                           }
{  -------  --------  ---    ----------------------------  }
{  0.1     12 Jul 97  Bazsi  Initial implementation        }
{  0.2     18 Jul 97  Bazsi  Linux specific error codes    }
{  0.2.2   28 Jul 97  Bazsi  Base error code for Video     }
{  0.2.3   29 Jul 97  Bazsi  Basic types added (PByte etc) }
{  0.2.5   08 Aug 97  Bazsi  Error handling code added     }
{  0.2.6   06 Sep 97  Bazsi  Base code for keyboard        }
{  0.2.7   06 Nov 97  Bazsi  Base error code for filectrl  }
{  0.2.8   21 Jan 99  LdB    Max data sizes added.         }
{  0.2.9   22 Jan 99  LdB    General array types added.    }
{  0.3.0   27 Oct 99  LdB    Delphi3+ MaxAvail, MemAvail   }
{  0.4.0   14 Nov 00  LdB    Revamp of whole unit          }
{**********************************************************}

UNIT FVCommon;

{<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>}
                                  INTERFACE
{<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>}

{====Include file to sort compiler platform out =====================}
{$I platform.inc}
{====================================================================}

<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef OS_WINDOWS}
=======
{$ifdef win32}
>>>>>>> graemeg/fixes_2_2
=======
{$ifdef win32}
>>>>>>> origin/fixes_2_2
  uses
    Windows;
{$endif}

{***************************************************************************}
{                              PUBLIC CONSTANTS                             }
{***************************************************************************}

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                        SYSTEM ERROR BASE CONSTANTS                        }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

{---------------------------------------------------------------------------}
{  The following ranges have been defined for error codes:                  }
{---------------------------------------------------------------------------}
{        0 -  1000    OS dependant error codes                              }
{     1000 - 10000    API reserved error codes                              }
{    10000 -          Add-On unit error codes                               }
{---------------------------------------------------------------------------}

{---------------------------------------------------------------------------}
{                         DEFINED BASE ERROR CONSTANTS                      }
{---------------------------------------------------------------------------}
CONST
   errOk                = 0;                          { No error }
   errVioBase           = 1000;                       { Video base offset }
   errKbdBase           = 1010;                       { Keyboard base offset }
   errFileCtrlBase      = 1020;                       { File IO base offset }
   errMouseBase         = 1030;                       { Mouse base offset }

{---------------------------------------------------------------------------}
{                            MAXIUM DATA SIZES                              }
{---------------------------------------------------------------------------}
CONST
{$IFDEF BIT_16}                                       { 16 BIT DEFINITION }
   MaxBytes = 65520;                                  { Maximum data size }
{$ENDIF}
<<<<<<< HEAD
<<<<<<< HEAD
{$IFDEF BIT_32_OR_MORE}                                       { 32 BIT DEFINITION }
=======
{$IFDEF BIT_32}                                       { 32 BIT DEFINITION }
>>>>>>> graemeg/fixes_2_2
=======
{$IFDEF BIT_32}                                       { 32 BIT DEFINITION }
>>>>>>> origin/fixes_2_2
   MaxBytes = 128*1024*1024;                          { Maximum data size }
{$ENDIF}
   MaxWords = MaxBytes DIV SizeOf(Word);              { Max words }
   MaxInts  = MaxBytes DIV SizeOf(Integer);           { Max integers }
   MaxLongs = MaxBytes DIV SizeOf(LongInt);           { Max longints }
   MaxPtrs  = MaxBytes DIV SizeOf(Pointer);           { Max pointers }
   MaxReals = MaxBytes DIV SizeOf(Real);              { Max reals }
   MaxStr   = MaxBytes DIV SizeOf(String);            { Max strings }

{***************************************************************************}
{                          PUBLIC TYPE DEFINITIONS                          }
{***************************************************************************}

{---------------------------------------------------------------------------}
{                           CPU TYPE DEFINITIONS                            }
{---------------------------------------------------------------------------}
TYPE
<<<<<<< HEAD
<<<<<<< HEAD
{$IFDEF BIT_32_OR_MORE}                               { 32 BIT CODE }
=======
{$IFDEF BIT_32}                                       { 32 BIT CODE }
>>>>>>> graemeg/fixes_2_2
=======
{$IFDEF BIT_32}                                       { 32 BIT CODE }
>>>>>>> origin/fixes_2_2
   CPUWord = Longint;                                 { CPUWord is 32 bit }
   CPUInt = Longint;                                  { CPUInt is 32 bit }
{$ELSE}                                               { 16 BIT CODE }
   CPUWord = Word;                                    { CPUWord is 16 bit }
   CPUInt = Integer;                                  { CPUInt is 16 bit }
{$ENDIF}

{---------------------------------------------------------------------------}
{                     16/32 BIT SWITCHED TYPE CONSTANTS                     }
{---------------------------------------------------------------------------}
TYPE
{$IFDEF BIT_16}                                       { 16 BIT DEFINITIONS }
   Sw_Word    = Word;                                 { Standard word }
   Sw_Integer = Integer;                              { Standard integer }
{$ENDIF}
<<<<<<< HEAD
<<<<<<< HEAD
{$IFDEF BIT_32_OR_MORE}                               { 32 BIT DEFINITIONS }
=======
{$IFDEF BIT_32}                                       { 32 BIT DEFINITIONS }
>>>>>>> graemeg/fixes_2_2
=======
{$IFDEF BIT_32}                                       { 32 BIT DEFINITIONS }
>>>>>>> origin/fixes_2_2
   Sw_Word    = Cardinal;                             { Long integer now }
   Sw_Integer = LongInt;                              { Long integer now }
{$ENDIF}

{---------------------------------------------------------------------------}
{                               GENERAL ARRAYS                              }
{---------------------------------------------------------------------------}
TYPE
   TByteArray = ARRAY [0..MaxBytes-1] Of Byte;        { Byte array }
   PByteArray = ^TByteArray;                          { Byte array pointer }

   TWordArray = ARRAY [0..MaxWords-1] Of Word;        { Word array }
   PWordArray = ^TWordArray;                          { Word array pointer }

   TIntegerArray = ARRAY [0..MaxInts-1] Of Integer;   { Integer array }
   PIntegerArray = ^TIntegerArray;                    { Integer array pointer }

   TLongIntArray = ARRAY [0..MaxLongs-1] Of LongInt;  { LongInt array }
   PLongIntArray = ^TLongIntArray;                    { LongInt array pointer }

   TRealArray = Array [0..MaxReals-1] Of Real;        { Real array }
   PRealarray = ^TRealArray;                          { Real array pointer }

   TPointerArray = Array [0..MaxPtrs-1] Of Pointer;   { Pointer array }
   PPointerArray = ^TPointerArray;                    { Pointer array ptr }

   TStrArray = Array [0..MaxStr-1] Of String;         { String array }
   PStrArray = ^TStrArray;                            { String array ptr }

{***************************************************************************}
{                            INTERFACE ROUTINES                             }
{***************************************************************************}

{-GetErrorCode-------------------------------------------------------
Returns the last error code and resets ErrorCode to errOk.
07/12/97 Bazsi
---------------------------------------------------------------------}
FUNCTION GetErrorCode: LongInt;

{-GetErrorInfo-------------------------------------------------------
Returns the info assigned to the previous error, doesn't reset the
value to nil. Would usually only be called if ErrorCode <> errOk.
07/12/97 Bazsi
---------------------------------------------------------------------}
FUNCTION GetErrorInfo: Pointer;

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                        MINIMUM AND MAXIMUM ROUTINES                       }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

FUNCTION Min (I, J: Sw_Integer): Sw_Integer;
FUNCTION Max (I, J: Sw_Integer): Sw_Integer;

{-MinimumOf----------------------------------------------------------
Given two real numbers returns the minimum real of the two.
04Oct99 LdB
---------------------------------------------------------------------}
FUNCTION MinimumOf (A, B: Real): Real;

{-MaximumOf----------------------------------------------------------
Given two real numbers returns the maximum real of the two.
04Oct99 LdB
---------------------------------------------------------------------}
FUNCTION MaximumOf (A, B: Real): Real;

{-MinIntegerOf-------------------------------------------------------
Given two integer values returns the lowest integer of the two.
04Oct99 LdB
---------------------------------------------------------------------}
FUNCTION MinIntegerOf (A, B: Integer): Integer;

{-MaxIntegerof-------------------------------------------------------
Given two integer values returns the biggest integer of the two.
04Oct99 LdB
---------------------------------------------------------------------}
FUNCTION MaxIntegerOf (A, B: Integer): Integer;

{-MinLongIntOf-------------------------------------------------------
Given two long integers returns the minimum longint of the two.
04Oct99 LdB
---------------------------------------------------------------------}
FUNCTION MinLongIntOf (A, B: LongInt): LongInt;

{-MaxLongIntOf-------------------------------------------------------
Given two long integers returns the maximum longint of the two.
04Oct99 LdB
---------------------------------------------------------------------}
FUNCTION MaxLongIntOf (A, B: LongInt): LongInt;

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                          MISSING DELPHI3 ROUTINES                         }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

{ ******************************* REMARK ****************************** }
{  Delphi 3+ does not define these standard routines so I have made     }
{  some public functions here to complete compatability.                }
{ ****************************** END REMARK *** Leon de Boer, 14Aug98 * }

{-MemAvail-----------------------------------------------------------
Returns the free memory available under Delphi 3+.
14Aug98 LdB
---------------------------------------------------------------------}
FUNCTION MemAvail: LongInt;

{-MaxAvail-----------------------------------------------------------
Returns the max free memory block size available under Delphi 3+.
14Aug98 LdB
---------------------------------------------------------------------}
FUNCTION MaxAvail: LongInt;

{***************************************************************************}
{                        INITIALIZED PUBLIC VARIABLES                       }
{***************************************************************************}

{---------------------------------------------------------------------------}
{                INITIALIZED DOS/DPMI/WIN/NT/OS2 VARIABLES                  }
{---------------------------------------------------------------------------}
CONST
   ErrorCode: Longint = errOk;                        { Last error code }
   ErrorInfo: Pointer = Nil;                          { Last error info }

{<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>}
                               IMPLEMENTATION
{<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>}

{$IFDEF PPC_DELPHI3}                                  { DELPHI 3+ COMPILER }
USES WinTypes, WinProcs;                              { Stardard units }
{$ENDIF}

{***************************************************************************}
{                            INTERFACE ROUTINES                             }
{***************************************************************************}

{---------------------------------------------------------------------------}
{  GetErrorCode -> Platforms ALL - Updated 12Jul97 Bazsi                    }
{---------------------------------------------------------------------------}
FUNCTION GetErrorCode: LongInt;
BEGIN
   GetErrorCode := ErrorCode;                         { Return last error }
   ErrorCode := 0;                                    { Now clear errorcode }
END;

{---------------------------------------------------------------------------}
{  GetErrorInfo -> Platforms ALL - Updated 12Jul97 Bazsi                    }
{---------------------------------------------------------------------------}
FUNCTION GetErrorInfo: Pointer;
BEGIN
   GetErrorInfo := ErrorInfo;                         { Return errorinfo ptr }
END;

{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{                        MINIMUM AND MAXIMUM ROUTINES                       }
{+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}

FUNCTION Min (I, J: Sw_Integer): Sw_Integer;
BEGIN
  If (I < J) Then Min := I Else Min := J;          { Select minimum }
END;

FUNCTION Max (I, J: Sw_Integer): Sw_Integer;
BEGIN
  If (I > J) Then Max := I Else Max := J;          { Select maximum }
END;


{---------------------------------------------------------------------------}
{  MinimumOf -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 04Oct99 LdB         }
{---------------------------------------------------------------------------}
FUNCTION MinimumOf (A, B: Real): Real;
BEGIN
   If (B < A) Then MinimumOf := B                     { B smaller take it }
     Else MinimumOf := A;                             { Else take A }
END;

{---------------------------------------------------------------------------}
{  MaximumOf -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 04Oct99 LdB         }
{---------------------------------------------------------------------------}
FUNCTION MaximumOf (A, B: Real): Real;
BEGIN
   If (B > A) Then MaximumOf := B                     { B bigger take it }
     Else MaximumOf := A;                             { Else take A }
END;

{---------------------------------------------------------------------------}
{  MinIntegerOf -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 04Oct99 LdB      }
{---------------------------------------------------------------------------}
FUNCTION MinIntegerOf (A, B: Integer): Integer;
BEGIN
   If (B < A) Then MinIntegerOf := B                  { B smaller take it }
     Else MinIntegerOf := A;                          { Else take A }
END;

{---------------------------------------------------------------------------}
{  MaxIntegerOf -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 04Oct99 LdB      }
{---------------------------------------------------------------------------}
FUNCTION MaxIntegerOf (A, B: Integer): Integer;
BEGIN
   If (B > A) Then MaxIntegerOf := B                  { B bigger take it }
     Else MaxIntegerOf := A;                          { Else take A }
END;

{---------------------------------------------------------------------------}
{  MinLongIntOf -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 04Oct99 LdB      }
{---------------------------------------------------------------------------}
FUNCTION MinLongIntOf (A, B: LongInt): LongInt;
BEGIN
   If (B < A) Then MinLongIntOf := B                  { B smaller take it }
     Else MinLongIntOf := A;                          { Else take A }
END;

{---------------------------------------------------------------------------}
{  MaxLongIntOf -> Platforms DOS/DPMI/WIN/NT/OS2 - Updated 04Oct99 LdB      }
{---------------------------------------------------------------------------}
FUNCTION MaxLongIntOf (A, B: LongInt): LongInt;
BEGIN
   If (B > A) Then MaxLongIntOf := B                  { B bigger take it }
     Else MaxLongIntOf := A;                          { Else take A }
END;

FUNCTION MemAvail: LongInt;
BEGIN
  { Unlimited }
  MemAvail:=high(longint);
END;

{---------------------------------------------------------------------------}
{  MaxAvail -> Platforms WIN/NT - Updated 14Aug98 LdB                       }
{---------------------------------------------------------------------------}
FUNCTION MaxAvail: LongInt;
BEGIN
  { Unlimited }
  MaxAvail:=high(longint);
END;

END.
