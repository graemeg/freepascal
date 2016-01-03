{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2009 by the Free Pascal development team

    Misc windows utility functions

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$mode objfpc}{$H+}
=======
{$mode objfpc}
>>>>>>> graemeg/cpstrnew
=======
{$mode objfpc}
>>>>>>> graemeg/cpstrnew
=======
{$mode objfpc}
>>>>>>> graemeg/cpstrnew
=======
{$mode objfpc}
>>>>>>> origin/cpstrnew
=======
{$mode objfpc}
>>>>>>> origin/fixes_2.4
unit winutils;

Interface

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Uses Windows, ComObj, ActiveX;
=======
Uses Windows;
>>>>>>> graemeg/cpstrnew
=======
Uses Windows;
>>>>>>> graemeg/cpstrnew
=======
Uses Windows;
>>>>>>> graemeg/cpstrnew
=======
Uses Windows;
>>>>>>> origin/cpstrnew
=======
Uses Windows;
>>>>>>> origin/fixes_2.4

// returns True if the currently logged Windows user has Administrator rights. Delphi.about.com
// From Delphi.about.com with permission, http://delphi.about.com/od/delphitips2007/qt/is_win_admin.htm
function IsWindowsAdmin: Boolean;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
// Removes Browsers "downloaded" attribute from a file.
procedure UnBlockFile(const name:String);

const
  NET_FW_PROFILE2_DOMAIN  = 1;
  NET_FW_PROFILE2_PRIVATE = 2;
  NET_FW_PROFILE2_PUBLIC  = 4;
  NET_FW_IP_PROTOCOL_TCP = 6;
  NET_FW_IP_PROTOCOL_UDP = 17;
  NET_FW_ACTION_ALLOW    = 1;  

// add firewall rule e.g. 
// AddProgramExceptionToFireWall( Application.Title,Application.Title, Application.ExeName, NET_FW_IP_PROTOCOL_TCP, NET_FW_PROFILE2_DOMAIN or NET_FW_PROFILE2_PRIVATE or NET_FW_PROFILE2_PUBLIC);
procedure AddProgramExceptionToFireWall(Const wsCaption, wsDescription, wsExecutable: WideString; iProtocol,iProfile:Integer);

// remove firewall rule, e.g.  RemoveExceptionFromFW(Application.Title);
procedure RemoveExceptionFromFW(Const exCaption: WideString);

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
implementation

const
  SECURITY_NT_AUTHORITY: TSIDIdentifierAuthority = (Value: (0, 0, 0, 0, 0, 5)) ;

const
  SECURITY_BUILTIN_DOMAIN_RID = $00000020;
  DOMAIN_ALIAS_RID_ADMINS = $00000220;


function IsWindowsAdmin: Boolean;
var
  hAccessToken: THandle;
  ptgGroups: PTokenGroups;
  dwInfoBufferSize: DWORD;
  psidAdministrators: PSID;
  g: Integer;
  bSuccess: BOOL;
begin
  Result := False;

  bSuccess := OpenThreadToken(GetCurrentThread, TOKEN_QUERY, True, hAccessToken) ;
  if not bSuccess then
  begin
    if GetLastError = ERROR_NO_TOKEN then
    bSuccess := OpenProcessToken(GetCurrentProcess, TOKEN_QUERY, hAccessToken) ;
  end;


  if bSuccess then
  begin
    GetMem(ptgGroups, 1024) ;

    bSuccess := GetTokenInformation(hAccessToken, TokenGroups, ptgGroups, 1024, dwInfoBufferSize) ;

    CloseHandle(hAccessToken) ;

    if bSuccess then
    begin
      AllocateAndInitializeSid(SECURITY_NT_AUTHORITY, 2, SECURITY_BUILTIN_DOMAIN_RID, DOMAIN_ALIAS_RID_ADMINS, 0, 0, 0, 0, 0, 0, psidAdministrators) ;

      for g := 0 to ptgGroups^.GroupCount - 1 do
        if EqualSid(psidAdministrators, ptgGroups^.Groups[g].Sid) then
        begin
          Result := True;
          Break;
        end;

      FreeSid(psidAdministrators) ;
    end;

    FreeMem(ptgGroups) ;
  end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure UnBlockFile(const name:String);
var f : file;
begin
 assignfile(f,name+':Zone.Identifier');
 rewrite(f,1);
 truncate(f);
 closefile(f);
end;

procedure AddProgramExceptionToFireWall(Const wsCaption, wsDescription, wsExecutable: WideString; iProtocol, iProfile:Integer);
var
  fwPolicy2                :  OleVariant;
  RulesObject              :  OleVariant;
  NewRule                  :  OleVariant;
begin
  fwPolicy2                := CreateOleObject('HNetCfg.FwPolicy2');
  RulesObject              := fwPolicy2.Rules;
  NewRule                  := CreateOleObject('HNetCfg.FWRule');
  NewRule.Name             := wsCaption;
  NewRule.Description      := wsDescription;
  NewRule.Applicationname  := wsExecutable;
  NewRule.Protocol         := iProtocol;
  NewRule.Enabled          := TRUE;
  NewRule.Profiles         := iProfile;
  NewRule.Action           := NET_FW_ACTION_ALLOW;
  RulesObject.Add(NewRule);
end; 

procedure RemoveExceptionFromFW(Const exCaption: WideString);
var
  fwPolicy2      : OleVariant;
begin
  fwPolicy2      := CreateOleObject('HNetCfg.FwPolicy2');
  fwPolicy2.Rules.Remove(exCaption);
end;   

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
end.
