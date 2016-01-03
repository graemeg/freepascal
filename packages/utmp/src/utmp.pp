unit Utmp;

interface

uses
  BaseUnix,
  Unix;

const
  Device_name_length = 12;
  Host_name_length = 245;
  User_name_length = 32;

type
  tTime = Longint;
  tPid = Longint;
  tIP_Address = array[1..4] of Byte;
  tDevice_name = String[Device_name_length];
  tUser_name = String[User_name_length];
  tHost_name = String[Host_name_length];
  tLogin_type = (Unkown, Run_level, Boot_time, New_time, Old_time,
    Init_process, Login_process, User_process, Dead_process);
  tLogin_types = set of tLogin_type;
  tParameter_type = (Include, Exclude);

  tUser = packed record
    Type_of_login : tLogin_type;
    Pid           : tPid;
    Device        : tDevice_name;
    Tty_name      : String[4];
    Login_time    : tTime;
    User_name     : tUser_name;
    Host_name     : tHost_name;
    IP_Address    : tIP_Address;
  end;
  pUser = ^tUser;

Const
  DefaultLoginType : TLogin_Types = [User_Process];
  Login_type_names : array [TLogin_type] of string[20] =
  ('Unkown', 'Run level', 'Boot time','New time', 'Old time',
    'Init process', 'Login process', 'User process', 'Dead process');
 All_Login_types : TLogin_types = [Unkown, Run_level, Boot_time, New_time, Old_time,
    Init_process, Login_process, User_process, Dead_process];

procedure Read_logged_users;
function Get_next_user : tUser;
function Get_next_user(var Last : Boolean) : tUser;
procedure Set_search_parameters(
  const Parameter_type : tParameter_type;
  Login_types : tLogin_types);
procedure Reset_user_search;
function More_users : Boolean;
function Number_of_logged_users : Word;


{Low level rutines}
function Number_of_utmp_entries : Word;
procedure Set_utmp_file(const File_name : String);


type
  texitstatus = record
    e_termination,
    e_exit : integer;
    end;

  tLL_Utmp = record
    ut_type : integer;
    ut_pid : longint;
    ut_line : tdevice_name;
    ut_id : array[1..4] of char;
    ut_user : tuser_name;
    ut_host : thost_name;
    ut_exit : texitstatus;
    ut_session : longint;
    ut_tv : Array [1..2] of longint;
    ut_addr : Array[1..4] of longint;
    pad : array [1..48] of char
    end;
  pUser_list = ^tUser_list;
  tUser_list = record
    User : tUser;
    Next : pUser_list;
  end;

implementation

Type
  tSearch_parameters = record
    Type_of_login_types : tParameter_type;
    Login_types         : tLogin_types;
  end;

var
  User_list           : pUser_list;
  Current_user        : pUser_list;
  Utmp_file           : String;
  Search_parameters   : tSearch_parameters;

  procedure Set_search_parameters(
    const Parameter_type : tParameter_type;
    Login_types : tLogin_types);

  begin
    Search_parameters.Type_of_login_types := Parameter_type;
    Search_parameters.Login_types := Login_types;
  end;

  function More_users : Boolean;

  var
    UL   : pUser_list;
    Last : Boolean;

  begin
    UL := Current_user;
    Last := True;
      while (UL <> nil) and Last do begin
          if Search_parameters.Type_of_login_types = Exclude then begin
            Last := (UL^.User.Type_of_login in Search_parameters.Login_types);
          end else begin
            Last := not (UL^.User.Type_of_login in Search_parameters.Login_types);
          end;
        UL := UL^.Next;
      end;
    More_users := not Last;
  end;

  function Number_of_logged_users : Word;

  var
    I : Word;
    UL: pUser_list;

  begin
    I := 0;
    UL := User_list;
      while UL <> nil do begin
          if UL^.User.Type_of_login = User_process then begin
            I := I + 1;
          end;
        UL := UL^.Next;
      end;
    Number_of_logged_users := I;
  end;

  function Get_next_user : tUser;

  var
    Found : Boolean;
    User  : pUser;

  begin
    if Current_user <> nil then begin
      Found := False;
        while (Current_user <> nil) and not Found do begin
          User := @Current_user^.User;
            if Search_parameters.Type_of_login_types = Exclude then begin
              Found := not (User^.Type_of_login in Search_parameters.Login_types);
            end else begin
              Found := (User^.Type_of_login in Search_parameters.Login_types);
            end;
          Current_user := Current_user^.Next;
        end;
        if Found then begin
          Get_next_user := User^;
        end else begin
          New(User);
          FillChar(User^, SizeOf(tUser), 0);
          Get_next_user := User^;
          Dispose(User);
        end;
    end else begin
      New(User);
      FillChar(User^, SizeOf(tUser), 0);
      Get_next_user := User^;
      Dispose(User);
    end;
  end;

  function Get_next_user(var Last : Boolean) : tUser;

  var
    Found : Boolean;
    User  : pUser;
    UL    : pUser_list;

  begin
    if Current_user <> nil then begin
      Found := False;
        while (Current_user <> nil) and not Found do begin
          User := @Current_user^.User;
            if Search_parameters.Type_of_login_types = Exclude then begin
              Found := not (User^.Type_of_login in Search_parameters.Login_types);
            end else begin
              Found := (User^.Type_of_login in Search_parameters.Login_types);
            end;
          Current_user := Current_user^.Next;
        end;
        if Found then begin
          Get_next_user := User^;
          UL := Current_user;
          Last := True;
            while (UL <> nil) and Last do begin
                if Search_parameters.Type_of_login_types = Exclude then begin
                  Last := (UL^.User.Type_of_login in Search_parameters.Login_types);
                end else begin
                  Last := not (UL^.User.Type_of_login in Search_parameters.Login_types);
                end;
              UL := UL^.Next;
            end;
        end else begin
          New(User);
          FillChar(User^, SizeOf(tUser), 0);
          Get_next_user := User^;
          Dispose(User);
        end;
    end else begin
      New(User);
      FillChar(User^, SizeOf(tUser), 0);
      Get_next_user := User^;
      Dispose(User);
    end;
  end;

  procedure Reset_user_search;

  begin
    Current_user := User_list;
  end;

  procedure Set_utmp_file(const File_name : String);

  begin
    Utmp_file := File_name;
  end;

  function Number_of_utmp_entries : Word;

  var
    S : Stat;

  begin
    fpstat(Utmp_file, S);
    Number_of_utmp_entries := s.st_size div System.SizeOf(tLL_Utmp);
  end;


  procedure Read_logged_users;

    procedure Read_entry(var F : File; var Entry : tUser; var User : Boolean);

    var
      LL_Entry : tLL_Utmp;
      I        : Byte;

    begin
      BlockRead(F, LL_Entry, SizeOf(tLL_Utmp));
      //Byte(Entry.Type_of_login) := LL_Entry.ut_type;
      Entry.Type_of_login := tLogin_type(LL_Entry.ut_type);
      Entry.Pid := Longint(LL_Entry.ut_id);
      Entry.Device:=LL_entry.ut_line;
      Entry.TTy_Name:=LL_Entry.ut_id;
      Entry.Login_time := LL_Entry.ut_tv[1];
      Entry.User_name:=LL_entry.ut_user;
      Entry.Host_name:=LL_entry.ut_host;
      For I:=1 to 4 do
          Entry.Ip_Address[I] := LL_Entry.ut_addr[i];
    end;

  var
    F : File;
    I : Longint;
    UL: pUser_list;
    U : Boolean;

  begin
    System.Assign(F, Utmp_file);
{$IFOPT I+}
{$DEFINE I_was_on}
{$ENDIF}
{$I-}
    System.Reset(F,1);
{$IFDEF I_was_on}
{$UNDEF I_was_on}
{$I+}
{$ENDIF}
    UL := User_list;
      while UL <> nil do begin
        User_list := UL;
        UL := UL^.Next;
        Dispose(User_list);
      end;
    User_list := nil;
      if System.IOResult = 0 then begin
          for I := 1 to Number_of_utmp_entries do begin
              if User_list = nil then begin
                New(User_list);
                UL := User_list;
                UL^.Next := nil;
              end else begin
                New(UL^.Next);
                UL := UL^.Next;
                UL^.Next := nil;
              end;
            Read_entry(F, UL^.User, U);
          end;
        UL^.Next := nil;
      end else begin
        User_list := nil;
      end;
    System.Close(F);
    Current_user := User_list;
  end;

begin
  User_list := nil;
  Current_user := nil;
  Utmp_file := '/var/run/utmp';
  Set_search_parameters(Include,DefaultLoginType);
end.
