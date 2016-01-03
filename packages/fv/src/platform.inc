{***************[ PLATFORM INCLUDE UNIT ]******************}
{                                                          }
{    System independent INCLUDE file to sort PLATFORMS     }
{                                                          }
{    Parts Copyright (c) 1997 by Balazs Scheidler          }
{    bazsi@tas.vein.hu                                     }
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
<<<<<<< HEAD
<<<<<<< HEAD
{       WIN64     - FPC 2.4.3               (64 Bit)       }
{        OS2      - Virtual Pascal 1.0+     (32 Bit)       }
{                 - C'T patch to BP         (16 Bit)       }
{        LINUX    - FPC 0.9912+             (32 Bit)       }
{        LINUX    - FPC 2.4.3               (64 Bit)       }
{        LINUX    - FPC 2.4.3               (64 Bit)       }
=======
{        OS2      - Virtual Pascal 1.0+     (32 Bit)       }
{                 - C'T patch to BP         (16 Bit)       }
{        LINUX    - FPC 0.9912+             (32 Bit)       }
>>>>>>> graemeg/fixes_2_2
=======
{        OS2      - Virtual Pascal 1.0+     (32 Bit)       }
{                 - C'T patch to BP         (16 Bit)       }
{        LINUX    - FPC 0.9912+             (32 Bit)       }
>>>>>>> origin/fixes_2_2
{                                                          }
{******************[ REVISION HISTORY ]********************}
{  Version  Date      Who    Fix                           }
{  -------  --------  ---    ----------------------------  }
{  0.1     02 Jul 97  Bazsi  Initial implementation        }
{  0.2     28 Aug 97  LdB    Fixed OS2 platform sort       }
{  0.3     29 Aug 97  LdB    Added assembler type changes  }
{  0.4     29 Aug 97  LdB    OS_DOS removed from WINDOWS   }
{  0.5     23 Oct 97  LdB    Delphi & speed compilers      }
{  0.6     05 May 98  LdB    Virtual Pascal 2.0 added      }
{  0.7     19 May 98  LdB    Delphi 2/3 definitions added  }
{  0.8     06 Aug 98  CEC    FPC only support fixed WIN32  }
{  0.9     10 Aug 98  LdB    BP_VMTLink def/Undef added    }
{  1.0     27 Aug 98  LdB    Atari, Mac etc not undef dos  }
{  1.1     25 Oct 98  PfV    Delphi 4 definitions added    }
{  1.2     06 Jun 99  LdB    Sybil 2.0 support added       }
{  1.3     13 Jun 99  LdB    Sybil 2.0 undef BP_VMT link   }
{  1.31    03 Nov 99  LdB    FPC windows defines WIN32     }
{  1.32    04 Nov 99  LdB    Delphi 5 definitions added    }
{  1.33    16 Oct 00  LdB    WIN32/WIN16 defines added     }
{  1.34    02 May 02  MvdV   FreeBSD, NetBSD, OS_UNIX      }
<<<<<<< HEAD
<<<<<<< HEAD
{  1.35    28 Nov 10  PM     64 bit OS Linux, Win64, FreeBSD }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{  1.36    22 Mar 15  PM     64 bit OS DragonFly           }
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
{**********************************************************}

{ ****************************************************************************

   This include file defines some conditional defines to allow us to select
   the compiler/platform/target in a consequent way.

    OS_XXXX         The operating system used (XXXX may be one of:
                       DOS, OS2, Linux, Windows, Go32)
    PPC_XXXX        The compiler used: BP, FPK, Virtual, Speed
    BIT_XX          The number of bits of the target platform: 16 or 32
    PROC_XXXX       The mode of the target processor (Real or Protected)
                    This shouldn't be used, except for i386 specific parts.
    ASM_XXXX        This is the assembler type: BP, ISO-ANSI, FPK

 ****************************************************************************

    This is how the IFDEF and UNDEF statements below should translate.


 PLATFORM  SYSTEM    COMPILER  COMP ID      CPU MODE        BITS    ASSEMBLER
 --------  ------    --------  -------      --------        ----    ---------

 DOS      OS_DOS      BP/TP7   PPC_BP       PROC_Real       BIT_16  ASM_BP

 DPMI     OS_DOS      BP/TP7   PPC_BP       PROC_Protected  BIT_16  ASM_BP
                      FPC      PPC_FPC      PROC_Protected  BIT_32  ASM_FPC

 LINUX    OS_LINUX    FPC      PPC_FPC      PROC_Protected  BIT_32  ASM_FPC
<<<<<<< HEAD
<<<<<<< HEAD
 LINUX    OS_LINUX    FPC      PPC_FPC      PROC_Protected  BIT_64  ASM_FPC
          OS_UNIX

 FREEBSD  OS_FREEBSD  FPC      PPC_FPC      PROC_Protected  BIT_32  ASM_FPC
 FREEBSD  OS_FREEBSD  FPC      PPC_FPC      PROC_Protected  BIT_64  ASM_FPC
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          OS_UNIX

 DRAGONFLY OS_DRAGONFLYBSD FPC PPC_FPC      PROC_Protected  BIT_64  ASM_FPC
          OS_UNIX

 OPENBSD  OS_OPENBSD  FPC      PPC_FPC      PROC_Protected  BIT_32  ASM_FPC
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
          OS_UNIX

 FREEBSD  OS_FREEBSD  FPC      PPC_FPC      PROC_Protected  BIT_32  ASM_FPC
>>>>>>> graemeg/fixes_2_2
=======
          OS_UNIX

 FREEBSD  OS_FREEBSD  FPC      PPC_FPC      PROC_Protected  BIT_32  ASM_FPC
>>>>>>> origin/fixes_2_2
          OS_UNIX

 NETBSD   OS_NETBSD   FPC      PPC_FPC      PROC_Protected  BIT_32  ASM_FPC
          OS_UNIX

 WINDOWS  OS_WINDOWS  BP/TP7   PPC_BP       PROC_Protected  BIT_16  ASM_BP
                      DELPHI   PPC_DELPHI   PROC_Protected  BIT_16  ASM_BP
                      DELPHI2  PPC_DELPHI2  PROC_Protected  BIT_16  ASM_BP

 WIN95/NT OS_WINDOWS  DELPHI2  PPC_DELPHI2  PROC_Protected  BIT_32  ASM_BP
                      DELPHI3  PPC_DELPHI3  PROC_Protected  BIT_32  ASM_BP
                      DELPHI4  PPC_DELPHI3  PROC_Protected  BIT_32  ASM_BP
                      DELPHI5  PPC_DELPHI3  PROC_Protected  BIT_32  ASM_BP
                      VIRTUAL  PPC_VIRTUAL  PROC_Protected  BIT 32  ASM_BP
                      SYBIL2   PPC_SPEED    PROC_Protected  BIT_32  ASM_BP
                      FPC      PPC_FPC      PROC_Protected  BIT_32  ASM_FPC
<<<<<<< HEAD
<<<<<<< HEAD
 WINDOWS 64-BITS      FPC      PPC_FPC      PROC_Protected  BIT_64  ASM_FPC
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

 OS2      OS_OS2      BPOS2    PPC_BPOS2    PROC_Protected  BIT_16  ASM_BP
                      VIRTUAL  PPC_VIRTUAL  PROC_Protected  BIT_32  ASM_BP
                      SPEED    PPC_SPEED    PROC_Protected  BIT_32  ASM_BP
                      SYBIL2   PPC_SPEED    PROC_Protected  BIT_32  ASM_BP
                      FPC      PPC_FPC      PROC_Protected  BIT_32  ASM_FPC
 ****************************************************************************}
{****************************************************************************

FOR ALL COMPILERS BP_VMTLink defined but FPC and Delphi3/Delphi4 undefine it

 ****************************************************************************}
{****************************************************************************

FOR FPC THESE ARE THE TRANSLATIONS

  PLATFORM  SYSTEM          COMPILER  HANDLE SIZE      ASM          CPU
 --------  ------           --------  -----------      ----         ---

 DOS      OS_DOS,OS_GO32       FPC     32-bit         AT&T         CPU86

 WIN32    OS_WINDOWS           FPC     32-bit           AT&T         ----

 LINUX    OS_LINUX,OS_UNIX     FPC     32-bit           AT&T         ----
<<<<<<< HEAD
<<<<<<< HEAD
 FREEBSD  OS_FREEBSD,OS_UNIX   FPC     32-bit           AT&T         ----
 OPENBSD  OS_OPENBSD,OS_UNIX   FPC     32-bit           AT&T         ----
 NETBSD   OS_NETBSD,OS_UNIX    FPC     32-bit           AT&T         ----
=======
 FREEBSD  OS_NETBSD,OS_UNIX    FPC     32-bit           AT&T         ----
 NETBSD   OS_FREEBSD,OS_UNIX   FPC     32-bit           AT&T         ----
>>>>>>> graemeg/fixes_2_2
=======
 FREEBSD  OS_NETBSD,OS_UNIX    FPC     32-bit           AT&T         ----
 NETBSD   OS_FREEBSD,OS_UNIX   FPC     32-bit           AT&T         ----
>>>>>>> origin/fixes_2_2

 OS2      OS_OS2               FPC     ?????            AT&T         CPU86

 ATARI    OS_ATARI             FPC     32-bit           Internal     CPU68

 MACOS    OS_MAC               FPC     ?????            Internal     CPU68

 AMIGA    OS_AMIGA             FPC     32-bit           Internal     CPU68

<<<<<<< HEAD
<<<<<<< HEAD
 DRAGONFLY OS_DRAGONFLYBSD,OS_UNIX   FPC 32-bit         AT&T         ----

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
 ****************************************************************************}

{---------------------------------------------------------------------------}
{  Initial assume BORLAND 16 BIT DOS COMPILER - Updated 27Aug98 LdB         }
{---------------------------------------------------------------------------}
{$DEFINE OS_DOS}
{$DEFINE PROC_Real}
{$DEFINE BIT_16}
{$DEFINE PPC_BP}
{$DEFINE ASM_BP}
{$DEFINE BP_VMTLink}
{$DEFINE CPU86}

{---------------------------------------------------------------------------}
{  FPC 32 BIT COMPILER changes ASM, 32 bits etc - Updated 27Aug98 LdB       }
<<<<<<< HEAD
<<<<<<< HEAD
{  FPC 64 BIT COMPILER added - Update 28Nov2010 PM                          }
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{---------------------------------------------------------------------------}
{$IFDEF FPC}
  {$mode fpc}

  {$UNDEF PROC_Real}
  {$DEFINE PROC_Protected}
  {$UNDEF BIT_16}
<<<<<<< HEAD
<<<<<<< HEAD
  {$IFDEF CPU64}
    {$UNDEF BIT_32}
    {$DEFINE BIT_64}
  {$ELSE}
    {$DEFINE BIT_32}
    {$UNDEF BIT_64}
  {$ENDIF}
=======
  {$DEFINE BIT_32}
>>>>>>> graemeg/fixes_2_2
=======
  {$DEFINE BIT_32}
>>>>>>> origin/fixes_2_2
  {$UNDEF PPC_BP}
  {$DEFINE PPC_FPC}
  {$UNDEF ASM_BP}
  {$DEFINE ASM_FPC}
  {$UNDEF BP_VMTLink}
  {$DEFINE Use_API}
  {$DEFINE NO_WINDOW}
{$ENDIF}

{---------------------------------------------------------------------------}
{  FPC LINUX COMPILER changes operating system - Updated 27Aug98 LdB        }
{  Note: Other linux compilers would need to change other details           }
{---------------------------------------------------------------------------}
{$IFDEF LINUX}
  {$UNDEF OS_DOS}
  {$DEFINE OS_LINUX}
<<<<<<< HEAD
<<<<<<< HEAD
  {$DEFINE OS_UNIX}{$ENDIF}

{$IFDEF FreeBSD}
  {$UNDEF OS_DOS}
  {$DEFINE OS_FREEBSD}
  {$DEFINE OS_UNIX}
{$ENDIF}

{$IFDEF DRAGONFLY}
  {$UNDEF OS_DOS}
  {$DEFINE OS_DRAGONFLYBSD}
  {$DEFINE OS_UNIX}
{$ENDIF}

{$IFDEF OPENBSD}
  {$UNDEF OS_DOS}
  {$DEFINE OS_OPENBSD}
=======
=======
>>>>>>> origin/fixes_2_2
  {$DEFINE OS_UNIX}
{$ENDIF}

{$IFDEF FreeBSD}
  {$UNDEF OS_DOS}
  {$DEFINE OS_FREEBSD}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  {$DEFINE OS_UNIX}
{$ENDIF}

{$IFDEF NETBSD}
  {$UNDEF OS_DOS}
  {$DEFINE OS_NETBSD}
  {$DEFINE OS_UNIX}
{$ENDIF}


{$IFDEF Darwin}
  {$UNDEF OS_DOS}
  {$DEFINE OS_DARWIN}
  {$DEFINE OS_UNIX}
{$ENDIF}


{$IFDEF SOLARIS}
  {$UNDEF OS_DOS}
  {$DEFINE OS_SOLARIS}
  {$DEFINE OS_UNIX}
{$ENDIF}

{$IFDEF BEOS}
  {$UNDEF OS_DOS}
  {$DEFINE OS_BEOS}
  {$DEFINE OS_UNIX}
{$ENDIF}

<<<<<<< HEAD
<<<<<<< HEAD
{$IFDEF AIX}
  {$UNDEF OS_DOS}
  {$DEFINE OS_AIX}
  {$DEFINE OS_UNIX}
{$ENDIF}

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{------------------------------------------------}
{  FPC Netware COMPILER changes operating system }
{------------------------------------------------}
{$IFDEF Netware}
  {$UNDEF OS_DOS}
  {$DEFINE OS_NETWARE}
  {$IFDEF NETWARE_LIBC}
    {$DEFINE OS_NETWARE_LIBC}
  {$ELSE}
    {$DEFINE OS_NETWARE_CLIB}
  {$ENDIF}
  {$DEFINE HasSysMsgUnit}
{$ENDIF}

{---------------------------------------------------------------------------}
{  FPC GO32V2 COMPILER changes operating system - Updated 27Aug98 LdB       }
{---------------------------------------------------------------------------}
{$IFDEF GO32V2}
  {$DEFINE OS_GO32}
{$ENDIF}

{---------------------------------------------------------------------------}
{  32 BIT WINDOWS COMPILERS changes bit size - Updated 27Aug98 LdB          }
{---------------------------------------------------------------------------}
{$IFDEF WIN32}
  {$IFNDEF WINDOWS}
    {$DEFINE WINDOWS}
  {$ENDIF}
  {$UNDEF BIT_16}
  {$DEFINE BIT_32}
{$ENDIF}

{---------------------------------------------------------------------------}
<<<<<<< HEAD
<<<<<<< HEAD
{  64 BIT WINDOWS COMPILERS changes bit size - Updated 28Nov10 PM           }
{---------------------------------------------------------------------------}
{$IFDEF WIN64}
  {$IFNDEF WINDOWS}
    {$DEFINE WINDOWS}
  {$ENDIF}
  {$UNDEF BIT_16}
  {$UNDEF BIT_32}
  {$DEFINE BIT_64}
{$ENDIF}

{---------------------------------------------------------------------------}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{  WINDOWS COMPILERS change op system and proc mode - Updated 03Nov99 LdB   }
{---------------------------------------------------------------------------}
{$IFDEF WINDOWS}
  {$UNDEF OS_DOS}
  {$DEFINE OS_WINDOWS}
  {$UNDEF PROC_Real}
  {$DEFINE PROC_Protected}
  {$IFDEF FPC}
<<<<<<< HEAD
<<<<<<< HEAD
    // {$DEFINE WIN32}
=======
    {$DEFINE WIN32}
>>>>>>> graemeg/fixes_2_2
=======
    {$DEFINE WIN32}
>>>>>>> origin/fixes_2_2
  {$ENDIF}
{$ENDIF}

{---------------------------------------------------------------------------}
{  DELPHI1 COMPILER changes compiler type - Updated 27Aug98 LdB             }
{---------------------------------------------------------------------------}
{$IFDEF VER80}
  {$UNDEF PPC_BP}
  {$DEFINE PPC_DELPHI}
{$ENDIF}

{---------------------------------------------------------------------------}
{  DELPHI2 COMPILER changes compiler type - Updated 27Aug98 LdB             }
{---------------------------------------------------------------------------}
{$IFDEF VER90}
  {$UNDEF PPC_BP}
  {$DEFINE PPC_DELPHI}
  {$DEFINE PPC_DELPHI2}
{$ENDIF}

{---------------------------------------------------------------------------}
{  DELPHI3 COMPILER changes compiler type - Updated 27Aug98 LdB             }
{---------------------------------------------------------------------------}
{$IFDEF VER100}
  {$UNDEF PPC_BP}
  {$DEFINE PPC_DELPHI}
  {$DEFINE PPC_DELPHI3}
  {$UNDEF BP_VMTLink}
{$ENDIF}

{---------------------------------------------------------------------------}
{  DELPHI4 COMPILER changes compiler type - Updated 25Oct98 pfv             }
{---------------------------------------------------------------------------}
{$IFDEF VER120}
  {$UNDEF PPC_BP}
  {$DEFINE PPC_DELPHI}
  {$DEFINE PPC_DELPHI3}
  {$DEFINE PPC_DELPHI4}
  {$UNDEF BP_VMTLink}
{$ENDIF}

{---------------------------------------------------------------------------}
{  DELPHI5 COMPILER changes compiler type - Updated 04Nov99 pfv             }
{---------------------------------------------------------------------------}
{$IFDEF VER130}
  {$UNDEF PPC_BP}
  {$DEFINE PPC_DELPHI}
  {$DEFINE PPC_DELPHI3}
  {$DEFINE PPC_DELPHI4}
  {$DEFINE PPC_DELPHI5}
  {$UNDEF BP_VMTLink
  }
{$ENDIF}

{---------------------------------------------------------------------------}
{  OS2 COMPILERS change compiler type and mode - Updated 27Aug98 LdB        }
{  Note: Assumes BPOS2 16BIT OS2 patch except for FPC which undefines this  }
{---------------------------------------------------------------------------}
{$IFDEF OS2}
  {$UNDEF OS_DOS}
  {$DEFINE OS_OS2}
  {$UNDEF PROC_Real}
  {$DEFINE PROC_Protected}
  {$UNDEF PPC_BP}
  {$DEFINE PPC_BPOS2}
  {$IFDEF FPC}
    {$UNDEF PPC_BPOS2}
  {$ENDIF}
{$ENDIF}

{---------------------------------------------------------------------------}
{  VIRTUAL PASCAL changes compiler type/32 bit - Updated 27Aug98 LdB        }
{  Note: VP2 can compile win 32 code so changes op system as needed         }
{---------------------------------------------------------------------------}
{$IFDEF VirtualPascal}
  {$UNDEF BIT_16}
  {$DEFINE BIT_32}
  {$IFDEF PPC_BPOS2}
    {$UNDEF PPC_BPOS2}
  {$ENDIF}
  {$DEFINE PPC_VIRTUAL}
  {$IFDEF WIN32}
    {$UNDEF PPC_BP}
    {$UNDEF OS_OS2}
    {$DEFINE OS_WINDOWS}
  {$ENDIF}
{$ENDIF}

{---------------------------------------------------------------------------}
{  SPEED COMPILER changes compiler type/32 bit  - Updated 27Aug98 LdB       }
{---------------------------------------------------------------------------}
{$IFDEF Speed}
  {$UNDEF BIT_16}
  {$DEFINE BIT_32}
  {$UNDEF PPC_BPOS2}
  {$DEFINE PPC_SPEED}
  {$UNDEF BP_VMTLink}
{$ENDIF}

{---------------------------------------------------------------------------}
{  FPC AMIGA COMPILER changes op system and CPU type - Updated 27Aug98 LdB  }
{---------------------------------------------------------------------------}
{$IFDEF AMIGA}
  {$UNDEF OS_DOS}
  {$DEFINE OS_AMIGA}
  {$IFDEF CPU86}
    {$UNDEF CPU86}
  {$ENDIF}
  {$IFNDEF CPU68}
    {$DEFINE CPU68}
  {$ENDIF}
{$ENDIF}

<<<<<<< HEAD
<<<<<<< HEAD
{ MorphOS has AmigaOS API, so define OS_AMIGA }
{$IFDEF MORPHOS}
  {$UNDEF OS_DOS}
  {$DEFINE OS_AMIGA}
  {$IFDEF CPU86}
    {$UNDEF CPU86}
  {$ENDIF}
  {$IFNDEF CPUPPC}
    {$DEFINE CPUPPC}
  {$ENDIF}
{$ENDIF}

{ AROS has AmigaOS API, so define OS_AMIGA }
{$IFDEF AROS}
  {$UNDEF OS_DOS}
  {$DEFINE OS_AMIGA}
{$ENDIF}

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{---------------------------------------------------------------------------}
{  FPC ATARI COMPILER changes op system and CPU type - Updated 27Aug98 LdB  }
{---------------------------------------------------------------------------}
{$IFDEF ATARI}
  {$UNDEF OS_DOS}
  {$DEFINE OS_ATARI}
  {$IFDEF CPU86}
    {$UNDEF CPU86}
  {$ENDIF}
  {$IFNDEF CPU68}
    {$DEFINE CPU68}
  {$ENDIF}
{$ENDIF}

{---------------------------------------------------------------------------}
{  FPC MAC COMPILER changes op system and CPU type - Updated 27Aug98 LdB    }
{---------------------------------------------------------------------------}
{$IFDEF MACOS}
  {$UNDEF OS_DOS}
  {$DEFINE OS_MAC}
  {$IFDEF CPU86}
    {$UNDEF CPU86}
  {$ENDIF}
  {$IFNDEF CPU68}
    {$DEFINE CPU68}
  {$ENDIF}
{$ENDIF}

{$IFDEF OS_DOS}
  {$DEFINE NO_WINDOW}
{$ENDIF}

{---------------------------------------------------------------------------}
{  WIN16 AND WIN32 set if in windows - Updated 16Oct2000 LdB                }
<<<<<<< HEAD
<<<<<<< HEAD
{  WIN64 added - Update 28Nov2010 PM                                        }
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{---------------------------------------------------------------------------}
{$IFDEF OS_WINDOWS}                                   { WINDOWS SYSTEM }
  {$IFDEF BIT_16}
    {$DEFINE WIN16}                                   { 16 BIT WINDOWS }
  {$ENDIF}
  {$IFDEF BIT_32}
    {$DEFINE WIN32}                                   { 32 BIT WINDOWS }
  {$ENDIF}
<<<<<<< HEAD
<<<<<<< HEAD
  {$IFDEF BIT_64}
    {$DEFINE WIN64}                                   { 64 BIT WINDOWS }
  {$ENDIF}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$ENDIF}


{---------------------------------------------------------------------------}
{  BIT_32_OR_MORE                                                           }
{---------------------------------------------------------------------------}

{$ifdef BIT_16}
  {$UNDEF BIT_32_OR_MORE}
{$ELSE}
  {$DEFINE BIT_32_OR_MORE}
=======
>>>>>>> graemeg/cpstrnew
{$ENDIF}


{---------------------------------------------------------------------------}
{  BIT_32_OR_MORE                                                           }
{---------------------------------------------------------------------------}

{$ifdef BIT_16}
  {$UNDEF BIT_32_OR_MORE}
{$ELSE}
  {$DEFINE BIT_32_OR_MORE}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$ENDIF}


{---------------------------------------------------------------------------}
{  BIT_32_OR_MORE                                                           }
{---------------------------------------------------------------------------}

{$ifdef BIT_16}
  {$UNDEF BIT_32_OR_MORE}
{$ELSE}
  {$DEFINE BIT_32_OR_MORE}
<<<<<<< HEAD
=======
>>>>>>> graemeg/cpstrnew
{$ENDIF}


{---------------------------------------------------------------------------}
{  BIT_32_OR_MORE                                                           }
{---------------------------------------------------------------------------}

{$ifdef BIT_16}
  {$UNDEF BIT_32_OR_MORE}
{$ELSE}
  {$DEFINE BIT_32_OR_MORE}
{$ENDIF}





=======
{$ENDIF}


>>>>>>> origin/cpstrnew

=======
{$ENDIF}


>>>>>>> graemeg/fixes_2_2
=======
{$ENDIF}


>>>>>>> origin/fixes_2_2

