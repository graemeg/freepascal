{
    This file is part of the Free Pascal run time library.

    A file in Amiga system run time library.
    Copyright (c) 1998-2003 by Nils Sjoholm
    member of the Amiga RTL development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{
     The boolean IsConsole is in system.
     Just check the value of system.IsConsole
     or just IsConsole, if it is false then you
     started from workbench.

     Added the define use_amiga_smartlink.
     13 Jan 2003.

     Changed integer > smallint.
     10 Feb 2003.

     nils.sjoholm@mailbox.swipnet.se Nils Sjoholm
}

unit WBArgs;

interface

uses workbench,amigados;

function GetStartupMsg: pWBStartup;
function ProgramName: string;
function WBArgCount: smallint;
function GetWBArg(num : smallint): STRING;

implementation

function GetStartupMsg: pWBStartup;
begin
    if system._WBenchMsg <> nil then
        GetStartupMsg := pWBStartup(_WBenchMsg)
    else
       GetStartupMsg := nil;
end;

function ProgramName: string;
var
    WBMsg : pWBStartup;
    buffer : array[0..255] of char;
begin
    WBMsg := GetStartupMsg;
    if WBMsg <> nil then begin
       ProgramName := strpas(WBMsg^.sm_ArgList^[1].wa_Name);
    end else begin
       if GetprogramName(buffer,255) then begin
           ProgramName := strpas(buffer);
       end else begin
           ProgramName := '';
       end;
    end;
end;

function WBArgCount: smallint;
var
   WBMsg : pWBStartup;
begin
   WBMsg := GetStartupMsg;
   if WBMsg <> nil then
       WBArgCount := WBMsg^.sm_NumArgs -1
   else WBArgCount := 0;
end;

function GetWBArg(num : smallint): string;
var
    WBMsg : pWBStartup;
    param : smallint;
begin
    WBMsg := GetStartupMsg;
    if WBMsg <> nil then begin
       param := WBMsg^.sm_NumArgs;
    end else begin
       param := 0;
    end;
    if (param > 0) AND (num <= param) then begin
       GetWBArg := strpas(WBMsg^.sm_ArgList^[num+1].wa_Name);
    end else begin
       GetWBArg := '';
    end;
end;

end.
