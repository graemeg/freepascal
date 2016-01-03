unit users;

Interface
{$mode delphi}
<<<<<<< HEAD
<<<<<<< HEAD
uses UnixType,BaseUnix,pwd,grp, {$ifdef Linux} shadow,{$endif}SysUtils,Classes;
=======
uses BaseUnix,pwd,grp, {$ifdef Linux} shadow,{$endif}SysUtils,Classes;
>>>>>>> graemeg/fixes_2_2
=======
uses BaseUnix,pwd,grp, {$ifdef Linux} shadow,{$endif}SysUtils,Classes;
>>>>>>> origin/fixes_2_2

Type
  EUserLookupError = Class(Exception);
  EGroupLookupError = Class(Exception);
  EShadowLookupError = Class(Exception);
  TPasswordRecord = Tpasswd;
  PPasswordRecord = ^TPasswordRecord;

{ User functions }

Function  getpwnam(Const UserName: String) : PPasswordRecord;
Procedure GetUserData(Const UserName : String; Var Data : TPasswordRecord); overload;
Procedure GetUserData(Uid : TUID; Var Data : TPasswordRecord); overload;
function  GetUserName(UID : TUID) : String;
function  GetUserId(Const UserName : String) : TUID;
function  GetUserGid(Const UserName : String) : TGID;
function  GetUserDir(Const UserName : String): String;
function  GetUserDescription(Const UserName : String): String;
Procedure GetUserList(List : Tstrings);overload;
Procedure GetUserList(List : TStrings; WithIDs : Boolean);overload;

{ Group functions }

Function  getgrnam(Const GroupName: String) : PGroup;
Procedure GetGroupData(Const GroupName : String; Var Data : TGroup); overload;
Procedure GetGroupData(Gid : TGID; Var Data : TGroup); overload;
function  GetGroupName(GID : TGID) : String;
function  GetGroupId(Const GroupName : String) : TGID;
Procedure GetGroupList(List : Tstrings);overload;
Procedure GetGroupList(List : TStrings; WithIDs : Boolean);overload;
Procedure GetGroupMembers(GID : TGID;List : TStrings);overload;
Procedure GetGroupMembers(Const GroupName : String;List : TStrings);overload;

{ Shadow password functions }

{$ifdef Linux}
function getspnam(UserName : String): PPasswordFileEntry;
function sgetspent(Line : String): PPasswordFileEntry;

Procedure GetUserShadowData(Const UserName : String; Var Data : TPasswordFileEntry);overload;
Procedure GetUserShadowData(UID : TUID; Var Data : TPasswordFileEntry);overload;
{$endif}

{ Extra functions }

Function GetUserGroup(Const UserName : String) : String;

Implementation

ResourceString

EnoSuchUserName = 'Unknown username: "%s"';
EnoSuchUserID = 'Unknown user ID: %d';
EnoSuchGroupName = 'Unknown groupname: "%s"';
EnoSuchGroupID = 'Unknown group ID: %d';
ENoShadowEntry = 'No shadow file entry for "%s"';
EShadowNotPermitted = 'Not enough permissions to access shadow password file';

Function getpwnam(Const UserName: String) : PPasswordRecord;

begin
  Result:=pwd.fpgetpwnam(Pchar(UserName));
end;

Procedure GetUserData(Const UserName : String; Var Data : TPasswordRecord);

Var P : PPasswordRecord;

begin
  P:=fpGetpwnam(pchar(UserName));
  If P<>Nil then
    Data:=P^
  else
    Raise EUserLookupError.CreateFmt(ENoSuchUserName,[UserName]);
end;

Procedure GetUserData(Uid : tuid; Var Data : TPasswordRecord);

Var P : PPasswordRecord;

begin
  P:=fpGetpwuid(Uid);
  If P<>Nil then
    Data:=P^
  else
    Raise EUserLookupError.CreateFmt(ENoSuchUserID,[Uid]);
end;

function GetUserName(UID : TUID) : String;

Var
  UserData : TPasswordRecord;

begin
  GetuserData(UID,UserData);
  Result:=UserData.pw_Name;
end;

function  GetUserId(Const UserName : String) : TUID;

Var
  UserData : TPasswordRecord;

begin
  GetUserData(UserName,UserData);
  Result:=UserData.pw_uid;
end;

function  GetUserGId(Const UserName : String) : TGID;

Var
  UserData : TPasswordRecord;

begin
  GetUserData(UserName,UserData);
  Result:=UserData.pw_gid;
end;

function GetUserDir(Const UserName : String): String;

Var
  UserData : TPasswordRecord;

begin
  GetUserData(UserName,UserData);
  Result:=UserData.pw_dir;
end;

function  GetUserDescription(Const UserName : String): String;

Var
  UserData : TPasswordRecord;

begin
  GetUserData(UserName,UserData);
  Result:=strpas(UserData.pw_gecos);
end;

Procedure GetUserList(List : Tstrings);

begin
  GetUserList(List,False);
end;

Procedure GetUserList(List : TStrings; WithIDs : Boolean);

Var
  P : PPasswordRecord;

begin
  List.Clear;
  fpsetpwent;
  try
    Repeat
      P:=fpgetpwent;
      If P<>Nil then
        begin
        If WithIDs then
          List.Add(Format('%d=%s',[P^.pw_uid,strpas(p^.pw_name)]))
        else
          List.Add(strpas(p^.pw_name));
        end;
    until (P=Nil);
  finally
    fpendpwent;
  end;
end;

{ ---------------------------------------------------------------------
    Group Functions
  ---------------------------------------------------------------------}


Function  getgrnam(Const GroupName: String) : PGroup;

begin
  Result:=grp.fpgetgrnam(Pchar(GroupName));
end;

Procedure GetGroupData(Const GroupName : String; Var Data : TGroup); overload;

Var P : PGroup;

begin
  P:=fpGetgrnam(pchar(GroupName));
  If P<>Nil then
    Data:=P^
  else
    Raise EGroupLookupError.CreateFmt(ENoSuchGroupName,[GroupName]);
end;

Procedure GetGroupData(Gid : TGID; Var Data : TGroup); overload;

Var P : PGroup;

begin
  P:=fpGetgrgid(gid);
  If P<>Nil then
    Data:=P^
  else
    Raise EGroupLookupError.CreateFmt(ENoSuchGroupID,[Gid]);
end;

function GetGroupName(GID : TGID) : String;

Var
  G : TGroup;

begin
  GetGroupData(Gid,G);
  Result:=G.gr_name;
end;

function  GetGroupId(Const GroupName : String) : TGID;

Var
  G : TGroup;

begin
  GetGroupData(GroupName,G);
  Result:=G.gr_gid;
end;

Procedure GetGroupList(List : Tstrings);overload;

begin
  GetGroupList(List,False);
end;

Procedure GetGroupList(List : TStrings; WithIDs : Boolean);overload;

Var
  G : PGroup;

begin
  List.Clear;
  fpsetgrent;
  try
    Repeat
      G:=fpgetgrent;
      If G<>Nil then
        begin
        If WithIDs then
          List.Add(Format('%d=%s',[G^.gr_gid,strpas(G^.gr_name)]))
        else
          List.Add(strpas(G^.gr_name));
        end;
    until (G=Nil);
  finally
    fpendgrent;
  end;
end;

Function PCharListToStrings(P : PPChar; List : TStrings) : Integer;

begin
  List.Clear;
  While P^<>Nil do
    begin
    List.Add(StrPas(P^));
    P:=PPChar(PChar(P)+SizeOf(PChar));
    end;
  Result:=List.Count;
end;


Procedure GetGroupMembers(GID : TGID;List : TStrings);

Var
  G : TGroup;

begin
  GetGroupData(GID,G);
  PCharListToStrings(G.gr_mem,List);
end;

Procedure GetGroupMembers(Const GroupName : String;List : TStrings);

Var
  G : TGroup;

begin
  GetGroupData(GroupName,G);
  PCharListToStrings(g.gr_mem,List);
end;

{ Shadow password functions }
{$ifdef linux}
function getspnam(UserName : String): PPasswordFileEntry;

begin
  result:=shadow.getspnam(Pchar(UserName));
end;

function sgetspent(Line : String): PPasswordFileEntry;

begin
  Result:=shadow.sgetspent(Pchar(Line));
end;

Procedure GetUserShadowData(Const UserName : String; Var Data : TPasswordFileEntry);

Var
  P : PPasswordFileEntry;

begin
  P:=getspnam(UserName);
  If P=Nil then
    If (fpGetUID<>0) and (fpGetEUID<>0) then
      Raise EShadowLookupError.Create(EShadowNotPermitted)
    else
      Raise EShadowLookupError.CreateFmt(ENoShadowEntry,[UserName])
  else
    Data:=P^;
end;


Procedure GetUserShadowData(UID : TUID; Var Data : TPasswordFileEntry);

begin
  GetUserShadowData(GetUserName(UID),Data);
end;
{$endif}
{ Extra functions }

Function GetUserGroup(Const UserName : String) : String;

begin
  GetGroupName(GetUserGid(UserName));
end;

end.
