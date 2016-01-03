{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2005 by the Free Pascal development team

    International settings for Sysutils unit.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}


procedure InitInternationalGeneric;
  begin
    fillchar(SysLocale,sizeof(SysLocale),0);

    { keep these routines out of the executable? }
{$ifndef FPC_NOGENERICANSIROUTINES}
    { don't override a previously installed widestring manager }
    if not assigned(widestringmanager.UpperAnsiStringProc) then
      widestringmanager.UpperAnsiStringProc:=@GenericAnsiUpperCase;
    if not assigned(widestringmanager.LowerAnsiStringProc) then
<<<<<<< HEAD
<<<<<<< HEAD
      widestringmanager.LowerAnsiStringProc:=@GenericAnsiLowerCase;
=======
      widestringmanager.LowerAnsiStringProc:=@GenericAnsiLowerCase;    
>>>>>>> graemeg/fixes_2_2
=======
      widestringmanager.LowerAnsiStringProc:=@GenericAnsiLowerCase;    
>>>>>>> origin/fixes_2_2
    if not assigned(widestringmanager.CompareStrAnsiStringProc) then
      widestringmanager.CompareStrAnsiStringProc:=@GenericAnsiCompareStr;
    if not assigned(widestringmanager.CompareTextAnsiStringProc) then
      widestringmanager.CompareTextAnsiStringProc:=@GenericAnsiCompareText;
    if not assigned(widestringmanager.StrCompAnsiStringProc) then
      widestringmanager.StrCompAnsiStringProc:=@GenericAnsiStrComp;
    if not assigned(widestringmanager.StrICompAnsiStringProc) then
      widestringmanager.StrICompAnsiStringProc:=@GenericAnsiStrIComp;
    if not assigned(widestringmanager.StrLCompAnsiStringProc) then
      widestringmanager.StrLCompAnsiStringProc:=@GenericAnsiStrLComp;
    if not assigned(widestringmanager.StrLICompAnsiStringProc) then
      widestringmanager.StrLICompAnsiStringProc:=@GenericAnsiStrLIComp;
    if not assigned(widestringmanager.StrLowerAnsiStringProc) then
      widestringmanager.StrLowerAnsiStringProc:=@GenericAnsiStrLower;
    if not assigned(widestringmanager.StrUpperAnsiStringProc) then
      widestringmanager.StrUpperAnsiStringProc:=@GenericAnsiStrUpper;
<<<<<<< HEAD
<<<<<<< HEAD
{$endif FPC_NOGENERICANSIROUTINES}
=======
=======
>>>>>>> origin/fixes_2_2
{$endif FPC_NOGENERICANSIROUTINES}    
>>>>>>> graemeg/fixes_2_2
  end;

