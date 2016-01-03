{
    This file is part of the Free Pascal Run time library.
    Copyright (c) 2001 by the Free Pascal development team

    This file contains the OS independent declarations of the system unit
    for unix styled systems

    See the File COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{$define newsignal}
{$define FPC_SYSTEM_HAS_SYSDLH}

{$I systemh.inc}

{$ifdef cpum68k}
{ used for single computations }
const
  BIAS4 = $7f-1;
{$endif cpum68k}

const
 LineEnding = #10;
 LFNSupport = true;
 DirectorySeparator = '/';
<<<<<<< HEAD
<<<<<<< HEAD
 DriveSeparator = '';
=======
 DriveSeparator = '/';
>>>>>>> graemeg/fixes_2_2
=======
 DriveSeparator = '/';
>>>>>>> origin/fixes_2_2
 ExtensionSeparator = '.';
 PathSeparator = ':';
 AllowDirectorySeparators : set of char = ['\','/'];
 AllowDriveSeparators : set of char = [];
<<<<<<< HEAD
<<<<<<< HEAD
{ FileNameCaseSensitive and FileNameCasePreserving are defined below! }
=======
=======
>>>>>>> origin/fixes_2_2
{ FileNameCaseSensitive is defined below! }
>>>>>>> graemeg/fixes_2_2
 maxExitCode = 255;
 {$ifdef LINUX}
 MaxPathLen = 4096; // linux has always got to be BIGGER
 {$else}
 MaxPathLen = 1024; // BSDs since 1993, Solaris 10, Darwin
 {$endif}
 AllFilesMask = '*';

const
  UnusedHandle    = -1;
  StdInputHandle  = 0;
  StdOutputHandle = 1;
  StdErrorHandle  = 2;

  FileNameCaseSensitive : boolean = true;
  FileNameCasePreserving: boolean = true;
  CtrlZMarksEOF: boolean = false; (* #26 not considered as end of file *)

  sLineBreak = LineEnding;
  DefaultTextLineBreakStyle : TTextLineBreakStyle = tlbsLF;

{$if not defined(solaris) and not defined(darwin) and not defined(aix)}
var argc:longint;external name 'operatingsystem_parameter_argc';
    argv:PPchar;external name 'operatingsystem_parameter_argv';
    envp:PPchar;external name 'operatingsystem_parameter_envp';
{$endif}

{$ifdef unix}
const
  { hook for lineinfo, to get the module name from an address,
    unit dl sets it if it is used
  }
  UnixGetModuleByAddrHook : procedure (addr: pointer; var baseaddr: pointer; var filename: string) = nil;
{$endif unix}
