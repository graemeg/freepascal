program createusers;

{$mode objfpc}{$H+}

uses
  {$IFDEF UNIX}{$IFDEF UseCThreads}
  cthreads,
  {$ENDIF}{$ENDIF}
  Classes, SysUtils, dbf, CustApp, db
  { you can add units after this };

type

  { TMyApplication }

  TMyApplication = class(TCustomApplication)
  private
    procedure CreateUsers(DS: TDataset);
  protected
    procedure DoRun; override;
  public
    constructor Create(TheOwner: TComponent); override;
    destructor Destroy; override;
    procedure WriteHelp; virtual;
  end;

{ TMyApplication }

Type
  TUserRecord = Record
    L,N,E : string;
  end;

Const
  UserCount = 8;
  Users : Array[1..UserCount] of TUserRecord = (
   (L:'Daniel';N:'Daniel mantione'; E:'daniel@freepascal.org'),
   (L:'Florian';N:'Florian Klaempfl'; E:'florian@freepascal.org'),
   (L:'Joost';N:'Joost van der Sluis'; E:'joost@freepascal.org'),
   (L:'Jonas';N:'Jonas Maebe'; E:'jonas@freepascal.org'),
   (L:'Michael';N:'Michael van canneyt'; E:'michael@freepascal.org'),
   (L:'Marco';N:'Marco Van De Voort'; E:'marco@freepascal.org'),
   (L:'Pierre';N:'Pierre Muller'; E:'pierre@freepascal.org'),
   (L:'Tomas';N:'Tomas Hajny'; E:'tomas@freepascal.org')
   )  ;


procedure TMyApplication.CreateUsers(DS : TDataset);

Var
  I : integer;

begin
  For I:=1 to UserCount do
    begin
    DS.Append;
    DS.FieldByName('Login').AsString:=Users[i].L;
    DS.FieldByName('Name').AsString:=Users[i].N;
    DS.FieldByName('Email').AsString:=Users[i].E;
    If Random(2)<1 then
      DS.FieldByname('LastLogin').AsDatetime:=Date-Random(10);
    DS.Post;
    end;
end;
procedure TMyApplication.DoRun;
var
  ErrorMsg: String;
  DB : TDBF;
begin
  // quick check parameters
  ErrorMsg:=CheckOptions('h','help');
  if ErrorMsg<>'' then begin
    ShowException(Exception.Create(ErrorMsg));
    Terminate;
    Exit;
  end;

  // parse parameters
  if HasOption('h','help') then begin
    WriteHelp;
    Terminate;
    Exit;
  end;

  { add your program here }
  DB:=TDBF.Create(Self);
  try
    With DB.FieldDefs do
      begin
      Add('ID',ftAutoInc,0,True);
      Add('Login',ftString,30,true);
      Add('Name',ftString,50,True);
      Add('Email',ftString,50,False);
      Add('LastLogin',ftDate,0,False);
      end;
    DB.TableName:='users.dbf';
    DB.TableLevel:=7;
    DB.CreateTable;
    DB.Open;
    CreateUsers(DB);
  finally
    DB.Free;
  end;
  // stop program loop
  Terminate;
end;

constructor TMyApplication.Create(TheOwner: TComponent);
begin
  inherited Create(TheOwner);
  StopOnException:=True;
end;

destructor TMyApplication.Destroy;
begin
  inherited Destroy;
end;

procedure TMyApplication.WriteHelp;
begin
  { add your help code here }
  writeln('Usage: ',ExeName,' -h');
end;

var
  Application: TMyApplication;

{$IFDEF WINDOWS}{$R createusers.rc}{$ENDIF}

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$R *.res}

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
begin
  Application:=TMyApplication.Create(nil);
  Application.Title:='My Application';
  Application.Run;
  Application.Free;
end.

