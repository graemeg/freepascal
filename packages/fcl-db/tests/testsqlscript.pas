{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2008 by the Free Pascal development team

    FPCUnit SQLScript test.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
<<<<<<< HEAD
<<<<<<< HEAD
unit testsqlscript;
=======
unit testcsqlscript;
>>>>>>> graemeg/fixes_2_2
=======
unit testcsqlscript;
>>>>>>> origin/fixes_2_2

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, testregistry, sqlscript, fpcunit;

type

  { TMyScript }

  TMyScript = class (TCustomSQLScript)
  private
    FExcept: string;
    FStatements : TStrings;
    FDirectives : TStrings;
    FCommits : integer;
  protected
    procedure ExecuteStatement (SQLStatement: TStrings; var StopExecution: Boolean); override;
    procedure ExecuteDirective (Directive, Argument: String; var StopExecution: Boolean); override;
<<<<<<< HEAD
<<<<<<< HEAD
    procedure ExecuteCommit(CommitRetaining: boolean=true); override;
=======
    procedure ExecuteCommit; override;
>>>>>>> graemeg/fixes_2_2
=======
    procedure ExecuteCommit; override;
>>>>>>> origin/fixes_2_2
    procedure DefaultDirectives; override;
  public
    constructor create (AnOwner: TComponent); override;
    destructor destroy; override;
    function StatementsExecuted : string;
    function DirectivesExecuted : string;
    property DoException : string read FExcept write FExcept;
    property Aborted;
    property Line;
    property Directives;
    property Defines;
    property Script;
    property Terminator;
    property CommentsinSQL;
    property UseSetTerm;
    property UseCommit;
    property UseDefines;
    property OnException;
  end;

  { TTestSQLScript }

  TTestSQLScript = class (TTestCase)
  private
    Script : TMyScript;
    exceptionstatement,
    exceptionmessage : string;
    UseContinue : boolean;
    procedure Add (s :string);
    procedure AssertStatDir (Statements, Directives : string);
    procedure DoExecution;
    procedure ExceptionHandler(Sender: TObject; Statement: TStrings; TheException: Exception; var Continue: boolean);
    procedure TestDirectiveOnException3;
  protected
    procedure SetUp; override;
    procedure TearDown; override;
  published
    procedure TestCreateDefaults;
    procedure TestTerminator;
    procedure TestSetTerm;
    procedure TestUseSetTerm;
    procedure TestComments;
    procedure TestUseComments;
    procedure TestCommit;
    procedure TestUseCommit;
    procedure TestDefine;
    procedure TestUndefine;
    procedure TestUndef;
    procedure TestIfdef1;
    procedure TestIfdef2;
    procedure TestIfndef1;
    procedure TestIfndef2;
    procedure TestElse1;
    procedure TestElse2;
    procedure TestEndif1;
    procedure TestEndif2;
    procedure TestUseDefines;
    procedure TestTermInComment;
    procedure TestTermInQuotes1;
    procedure TestTermInQuotes2;
    procedure TestCommentInComment;
    procedure TestCommentInQuotes1;
    procedure TestCommentInQuotes2;
<<<<<<< HEAD
<<<<<<< HEAD
    Procedure TestDashDashComment;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    procedure TestQuote1InComment;
    procedure TestQuote2InComment;
    procedure TestQuoteInQuotes1;
    procedure TestQuoteInQuotes2;
    procedure TestStatementStop;
    procedure TestDirectiveStop;
    procedure TestStatementExeception;
    procedure TestDirectiveException;
    procedure TestCommitException;
    procedure TestStatementOnExeception1;
    procedure TestStatementOnExeception2;
    procedure TestDirectiveOnException1;
    procedure TestDirectiveOnException2;
    procedure TestCommitOnException1;
    procedure TestCommitOnException2;
  end;

  { TTestEventSQLScript }

  TTestEventSQLScript = class (TTestCase)
  private
    Script : TEventSQLScript;
    StopToSend : boolean;
    Received : string;
    notifycount : integer;
    LastSender : TObject;
    procedure Notify (Sender : TObject);
    procedure NotifyStatement (Sender: TObject; SQL_Statement: TStrings; var StopExecution: Boolean);
    procedure NotifyDirective (Sender: TObject; Directive, Argument: AnsiString; var StopExecution: Boolean);
  protected
    procedure SetUp; override;
    procedure TearDown; override;
  published
    procedure TestStatement;
    procedure TestStatementStop;
    procedure TestDirective;
    procedure TestDirectiveStop;
    procedure TestCommit;
    procedure TestBeforeExec;
    procedure TestAfterExec;
  end;

implementation

{ TMyScript }

procedure TMyScript.ExecuteStatement(SQLStatement: TStrings; var StopExecution: Boolean);
var s : string;
    r : integer;
begin
  if (SQLStatement.count = 1) and (compareText(SQLStatement[0],'END')=0) then
    StopExecution := true;
  s := '';
  for r := 0 to SQLstatement.count-1 do
    begin
    if s <> '' then
      s := s + ' ';
    s := s + SQLStatement[r];
    end;
  FStatements.Add (s);
  if DoException <> '' then
    raise exception.create(DoException);
end;

procedure TMyScript.ExecuteDirective(Directive, Argument: String; var StopExecution: Boolean);
begin
  if Directive = 'STOP' then
    StopExecution := true;
  if Argument = '' then
    FDirectives.Add (Directive)
  else
    FDirectives.Add (format('%s(%s)', [Directive, Argument]));
  if DoException <> '' then
    raise exception.create(DoException);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TMyScript.ExecuteCommit(CommitRetaining: boolean=true);
=======
procedure TMyScript.ExecuteCommit;
>>>>>>> graemeg/fixes_2_2
=======
procedure TMyScript.ExecuteCommit;
>>>>>>> origin/fixes_2_2
begin
  inc (FCommits);
  if DoException <> '' then
    raise exception.create(DoException);
end;

procedure TMyScript.DefaultDirectives;
begin
  inherited DefaultDirectives;
  directives.add ('STOP');
end;

constructor TMyScript.create (AnOwner: TComponent);
begin
  inherited create (AnOwner);
  FStatements := TStringlist.Create;
  FDirectives := TStringlist.Create;
  FCommits := 0;
  DoException := '';
end;

destructor TMyScript.destroy;
begin
  FStatements.Free;
  FDirectives.Free;
  inherited destroy;
end;

function TMyScript.StatementsExecuted: string;
begin
  result := FStatements.Commatext;
end;

function TMyScript.DirectivesExecuted: string;
begin
  result := FDirectives.Commatext;
end;


{ TTestSQLScript }

procedure TTestSQLScript.Add(s: string);
begin
  Script.Script.Add (s);
end;

procedure TTestSQLScript.AssertStatDir(Statements, Directives: string);
begin
  AssertEquals ('Executed Statements', Statements, script.StatementsExecuted);
  AssertEquals ('Executed Directives', Directives, script.DirectivesExecuted);
end;

procedure TTestSQLScript.DoExecution;
begin
  script.execute;
end;

procedure TTestSQLScript.ExceptionHandler(Sender: TObject; Statement: TStrings;
  TheException: Exception; var Continue: boolean);
var r : integer;
    s : string;
begin
  Continue := UseContinue;
  if Statement.count > 0 then
    s := Statement[0];
  for r := 1 to Statement.count-1 do
    s := s + ',' + Statement[r];
  exceptionstatement := s;
  exceptionmessage := TheException.message;
end;

procedure TTestSQLScript.SetUp;
begin
  inherited SetUp;
  Script := TMyscript.Create (nil);
end;

procedure TTestSQLScript.TearDown;
begin
  Script.Free;
  inherited TearDown;
end;

procedure TTestSQLScript.TestCreateDefaults;
begin
  with Script do
    begin
    AssertEquals ('Terminator', ';', Terminator);
    AssertTrue ('UseCommit', UseCommit);
    AssertTrue ('UseSetTerm', UseSetTerm);
    AssertTrue ('UseDefines', UseDefines);
    AssertTrue ('CommentsInSQL', CommentsInSQL);
    AssertFalse ('Aborted', Aborted);
    AssertEquals ('Line', 0, Line);
    AssertEquals ('Defines', 0, Defines.count);
<<<<<<< HEAD
<<<<<<< HEAD
    AssertEquals ('Directives', 12, Directives.count);
    AssertTrue('Have SET TERM',Directives.IndexOf('SET TERM')<>-1);
    AssertTrue('Have COMMIT WORK',Directives.IndexOf('COMMIT WORK')<>-1);
    AssertTrue('Have COMMIT RETAIN',Directives.IndexOf('COMMIT RETAIN')<>-1);
    AssertTrue('Have COMMIT',Directives.IndexOf('COMMIT')<>-1);
    AssertTrue('Have #IFDEF',Directives.IndexOf('#IFDEF')<>-1);
    AssertTrue('Have #IFNDEF',Directives.IndexOf('#IFNDEF')<>-1);
    AssertTrue('Have #ELSE',Directives.IndexOf('#ELSE')<>-1);
    AssertTrue('Have #ENDIF',Directives.IndexOf('#ENDIF')<>-1);
    AssertTrue('Have #DEFINE',Directives.IndexOf('#DEFINE')<>-1);
    AssertTrue('Have #UNDEF',Directives.IndexOf('#UNDEF')<>-1);
    AssertTrue('Have #UNDEFINE',Directives.IndexOf('#UNDEFINE')<>-1);
    // This is defined in our test class.
    AssertTrue('Have STOP',Directives.IndexOf('STOP')<>-1);
=======
    AssertEquals ('Directives', 10, Directives.count);
>>>>>>> graemeg/fixes_2_2
=======
    AssertEquals ('Directives', 10, Directives.count);
>>>>>>> origin/fixes_2_2
    end;
end;

procedure TTestSQLScript.TestTerminator;
begin
  script.terminator := '!';
  Add('doe!iets!');
  Add('anders!');
  script.execute;
  AssertStatDir('doe,iets,anders', '');
end;

procedure TTestSQLScript.TestSetTerm;
begin
  script.UseSetTerm:=true;
  Add('SET TERM !;');
  script.execute;
  AssertEquals ('terminator', '!', script.terminator);
  AssertStatDir('', '');
end;

procedure TTestSQLScript.TestUseSetTerm;
begin
  script.UseSetTerm:=false;
  Script.Directives.Add ('SET TERM');
  Add('SET TERM !;');
  script.execute;
  AssertEquals ('terminator', ';', script.terminator);
  AssertStatDir('', '"SET TERM(!)"');
end;

procedure TTestSQLScript.TestComments;
begin
  script.CommentsInSQL := true;
  Add('/* comment */');
  Add('statement;');
  script.execute;
  AssertStatDir ('"/* comment */ statement"', '');
end;

procedure TTestSQLScript.TestUseComments;
begin
  script.CommentsInSQL := false;
  Add('/* comment */');
  Add('statement;');
  script.execute;
  AssertStatDir ('statement', '');
end;

procedure TTestSQLScript.TestCommit;
begin
  script.UseCommit := true;
  Add('commit;');
  script.execute;
  AssertEquals ('Commits', 1, script.FCommits);
  AssertStatDir ('', '');
end;

procedure TTestSQLScript.TestUseCommit;
begin
  script.UseCommit := false;
  with script.Directives do
    Delete(IndexOf('COMMIT'));
  Add('commit;');
  script.execute;
  AssertEquals ('Commits', 0, script.FCommits);
  AssertStatDir ('commit', '');
end;

procedure TTestSQLScript.TestDefine;
begin
  script.UseDefines := true;
  Add ('#define iets;');
  script.execute;
  AssertStatDir ('', '');
  AssertEquals ('Aantal defines', 1, script.defines.count);
  AssertEquals ('Juiste define', 'iets', script.Defines[0]);
end;

procedure TTestSQLScript.TestUndefine;
begin
  script.UseDefines := true;
  script.defines.Add ('iets');
  Add ('#undefine iets;');
  script.execute;
  AssertStatDir ('', '');
  AssertEquals ('Aantal defines', 0, script.defines.count);
end;

procedure TTestSQLScript.TestUndef;
begin
  script.UseDefines := true;
  script.defines.Add ('iets');
  Add ('#Undef iets;');
  script.execute;
  AssertStatDir ('', '');
  AssertEquals ('Aantal defines', 0, script.defines.count);
end;

procedure TTestSQLScript.TestIfdef1;
begin
  script.UseDefines := true;
  script.defines.add ('iets');
  Add('#ifdef iets;');
  Add('doe iets;');
  script.execute;
  AssertStatDir('"doe iets"', '');
end;

procedure TTestSQLScript.TestIfdef2;
begin
  script.UseDefines := true;
  Add('#ifdef iets;');
  Add('doe iets;');
  script.execute;
  AssertStatDir('', '');
end;

procedure TTestSQLScript.TestIfndef1;
begin
  script.UseDefines := true;
  Add('#ifndef iets;');
  Add('doe iets;');
  script.execute;
  AssertStatDir('"doe iets"', '');
end;

procedure TTestSQLScript.TestIfndef2;
begin
  script.UseDefines := true;
  script.defines.add ('iets');
  Add('#ifndef iets;');
  Add('doe iets;');
  script.execute;
  AssertStatDir('', '');
end;

procedure TTestSQLScript.TestElse1;
begin
  script.UseDefines := true;
  script.defines.add ('iets');
  Add('#ifdef iets;');
  Add('doe iets;');
  add('#else;');
  add('anders;');
  script.execute;
  AssertStatDir('"doe iets"', '');
end;

procedure TTestSQLScript.TestElse2;
begin
  script.UseDefines := true;
  script.defines.add ('iets');
  Add('#ifndef iets;');
  Add('doe iets;');
  add('#else;');
  add('anders;');
  script.execute;
  AssertStatDir('anders', '');
end;

procedure TTestSQLScript.TestEndif1;
begin
  script.UseDefines := true;
  Add('#ifdef iets;');
  Add('doe iets;');
  add('#endif;');
  add('anders;');
  script.execute;
  AssertStatDir('anders', '');
end;

procedure TTestSQLScript.TestEndif2;
begin
  script.UseDefines := true;
  Add('#ifndef iets;');
  Add('doe iets;');
  add('#endif;');
  add('anders;');
  script.execute;
  AssertStatDir('"doe iets",anders', '');
end;

procedure TTestSQLScript.TestUseDefines;
begin
  script.UseDefines := false;
  Add('#ifndef iets;');
  Add('doe iets;');
  add('#endif;');
  add('anders;');
  script.execute;
  AssertStatDir('"doe iets",anders', '#IFNDEF(iets),#ENDIF');
end;

procedure TTestSQLScript.TestTermInComment;
begin
  script.CommentsInSQL := false;
  Add('/* terminator ; */iets;');
  script.execute;
  AssertStatDir('iets', '');
end;

procedure TTestSQLScript.TestTermInQuotes1;
begin
  script.CommentsInSQL := false;
  Add('iets '';'';');
  script.execute;
  AssertStatDir('"iets '';''"', '');
end;

procedure TTestSQLScript.TestTermInQuotes2;
begin
  script.CommentsInSQL := false;
  Add('iets ";";');
  script.execute;
  AssertStatDir('"iets "";"""', '');
end;

procedure TTestSQLScript.TestCommentInComment;
begin
  script.CommentsInSQL := false;
  Add('/* meer /* */iets;');
  script.execute;
  AssertStatDir('iets', '');
end;

procedure TTestSQLScript.TestCommentInQuotes1;
begin
  script.CommentsInSQL := false;
  Add('iets ''/* meer */'';');
  script.execute;
  AssertStatDir('"iets ''/* meer */''"', '');
end;

procedure TTestSQLScript.TestCommentInQuotes2;
begin
  script.CommentsInSQL := false;
  Add('iets "/* meer */";');
  script.execute;
  AssertStatDir('"iets ""/* meer */"""', '');
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TTestSQLScript.TestDashDashComment;
begin
  script.CommentsInSQL := false;
  Add('-- my comment');
  Add('CREATE TABLE "tPatients" (');
  Add('  "BloodGroup" character(2),');
  Add('  CONSTRAINT "ck_tPatients_BloodGroup" CHECK (("BloodGroup" = ANY (ARRAY[''A''::bpchar, ''B''::bpchar, ''AB''::bpchar, ''0''::bpchar]))),');
  Add(');');
  script.execute;
  AssertStatDir('"CREATE TABLE ""tPatients"" (   ""BloodGroup"" character(2),   CONSTRAINT ""ck_tPatients_BloodGroup"" CHECK ((""BloodGroup"" = ANY (ARRAY[''A''::bpchar, ''B''::bpchar, ''AB''::bpchar, ''0''::bpchar]))), )"', '');
end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
procedure TTestSQLScript.TestQuote1InComment;
begin
  script.CommentsInSQL := false;
  Add('/* s''morgens */iets;');
  script.execute;
  AssertStatDir('iets', '');
end;

procedure TTestSQLScript.TestQuote2InComment;
begin
  script.CommentsInSQL := false;
  Add('/* s"morgens */iets;');
  script.execute;
  AssertStatDir('iets', '');
end;

procedure TTestSQLScript.TestQuoteInQuotes1;
begin
  script.CommentsInSQL := false;
  Add('iets ''s"morgens'';');
  script.execute;
  AssertStatDir('"iets ''s""morgens''"', '');
end;

procedure TTestSQLScript.TestQuoteInQuotes2;
begin
  script.CommentsInSQL := false;
  Add('iets "s''morgens";');
  script.execute;
  AssertStatDir('"iets ""s''morgens"""', '');
end;

procedure TTestSQLScript.TestStatementStop;
begin
  Add('END;meer;');
  script.execute;
  AssertStatDir('END', '');
end;

procedure TTestSQLScript.TestDirectiveStop;
begin
  Add('Stop;meer;');
  script.execute;
  AssertStatDir('', 'STOP');
end;

procedure TTestSQLScript.TestStatementExeception;
begin
  Add('iets;');
  script.DoException:='FOUT';
  AssertException (exception, @DoExecution);
  AssertStatDir('iets', '');
end;

procedure TTestSQLScript.TestDirectiveException;
begin
  Add('iets;');
  script.Directives.Add('IETS');
  script.DoException := 'FOUT';
  AssertException (exception, @DoExecution);
  AssertStatDir('', 'IETS');
end;

procedure TTestSQLScript.TestCommitException;
begin
  Add ('commit;');
  script.DoException := 'FOUT';
  AssertException (exception, @DoExecution);
  AssertStatDir('', '');
  AssertEquals ('Commit count', 1, Script.FCommits);
end;

procedure TTestSQLScript.TestStatementOnExeception1;
begin
  UseContinue := true;
  script.DoException := 'Fout';
  Add ('foutief;');
  script.OnException:=@ExceptionHandler;
  Script.Execute;
  AssertEquals ('exception message', 'Fout', exceptionmessage);
  AssertEquals ('exception statement', 'foutief', exceptionstatement);
end;

procedure TTestSQLScript.TestStatementOnExeception2;
begin
  UseContinue := false;
  script.DoException := 'Fout';
  Add ('foutief;');
  script.OnException:=@ExceptionHandler;
  AssertException (exception, @DoExecution);
  AssertEquals ('exception message', 'Fout', exceptionmessage);
  AssertEquals ('exception statement', 'foutief', exceptionstatement);
end;

procedure TTestSQLScript.TestDirectiveOnException1;
begin
  UseContinue := true;
  script.DoException := 'Fout';
  Add ('foutief;');
  Script.Directives.Add ('FOUTIEF');
  script.OnException:=@ExceptionHandler;
  Script.Execute;
  AssertEquals ('exception message', 'Fout', exceptionmessage);
  AssertEquals ('exception statement', 'FOUTIEF', exceptionstatement);
end;

procedure TTestSQLScript.TestDirectiveOnException2;
begin
  UseContinue := False;
  script.DoException := 'Fout';
  Add ('foutief;');
  Script.Directives.Add ('FOUTIEF');
  script.OnException:=@ExceptionHandler;
  AssertException (exception, @DoExecution);
  AssertEquals ('exception message', 'Fout', exceptionmessage);
  AssertEquals ('exception statement', 'FOUTIEF', exceptionstatement);
end;

procedure TTestSQLScript.TestDirectiveOnException3;
begin
  UseContinue := true;
  script.DoException := 'Fout';
  Add ('foutief probleem;');
  Script.Directives.Add ('FOUTIEF');
  script.OnException:=@ExceptionHandler;
  Script.Execute;
  AssertEquals ('exception message', 'Fout', exceptionmessage);
  AssertEquals ('exception statement', 'FOUTIEF,probleem', exceptionstatement);
end;

procedure TTestSQLScript.TestCommitOnException1;
begin
  UseContinue := true;
  script.DoException := 'Fout';
  Add ('Commit;');
  script.OnException:=@ExceptionHandler;
  Script.Execute;
  AssertEquals ('exception message', 'Fout', exceptionmessage);
  AssertEquals ('exception statement', 'COMMIT', exceptionstatement);
  AssertEquals ('commit count', 1, Script.FCommits);
end;

procedure TTestSQLScript.TestCommitOnException2;
begin
  UseContinue := false;
  script.DoException := 'Fout';
  Add ('Commit;');
  script.OnException:=@ExceptionHandler;
  AssertException (exception, @DoExecution);
  AssertEquals ('exception message', 'Fout', exceptionmessage);
  AssertEquals ('exception statement', 'COMMIT', exceptionstatement);
  AssertEquals ('commit count', 1, Script.FCommits);
end;

{ TTestEventSQLScript }

procedure TTestEventSQLScript.Notify(Sender: TObject);
begin
  inc (NotifyCount);
  LastSender := Sender;
end;

procedure TTestEventSQLScript.NotifyStatement(Sender: TObject;
  SQL_Statement: TStrings; var StopExecution: Boolean);
var r : integer;
    s : string;
begin
  StopExecution := StopToSend;
  if SQL_Statement.count > 0 then
    begin
    s := SQL_Statement[0];
    for r := 1 to SQL_Statement.count-1 do
      s := s + ';' + SQL_Statement[r];
    if SQL_Statement.count > 1 then
      s := '"' + s + '"';
    end
  else
    s := '';
  if received <> '' then
    received := received + ';' + s
  else
    received := s;
  LastSender := Sender;
end;

procedure TTestEventSQLScript.NotifyDirective(Sender: TObject; Directive,
  Argument: AnsiString; var StopExecution: Boolean);
var s : string;
begin
  StopExecution := StopToSend;
  if Argument = '' then
    s := Directive
  else
    s := format ('%s(%s)', [Directive, Argument]);
  if received <> '' then
    received := received + ';' + s
  else
    received := s;
  LastSender := Sender;
end;

procedure TTestEventSQLScript.SetUp;
begin
  inherited SetUp;
  Script := TEventSQLScript.Create (nil);
  notifycount := 0;
  Received := '';
  LastSender := nil;
end;

procedure TTestEventSQLScript.TearDown;
begin
  Script.Free;
  inherited TearDown;
end;

procedure TTestEventSQLScript.TestStatement;
begin
  StopToSend:=false;
  Script.OnSQLStatement := @NotifyStatement;
  Script.Script.Text := 'stat1;stat2;';
  script.execute;
  AssertEquals ('Received', 'stat1;stat2', received);
  AssertSame ('Sender', script, LastSender);
end;

procedure TTestEventSQLScript.TestStatementStop;
begin
  StopToSend:=true;
  Script.OnSQLStatement := @NotifyStatement;
  Script.Script.Text := 'stat1;stat2;';
  script.execute;
  AssertEquals ('Received', 'stat1', received);
  AssertSame ('Sender', script, LastSender);
end;

procedure TTestEventSQLScript.TestDirective;
begin
  StopToSend:=false;
  Script.OnSQLStatement := @NotifyStatement;
  Script.OnDirective := @NotifyDirective;
  script.Directives.Add ('STAT1');
  Script.Script.Text := 'stat1 ik;stat2;';
  script.execute;
  AssertEquals ('Received', 'STAT1(ik);stat2', received);
  AssertSame ('Sender', script, LastSender);
end;

procedure TTestEventSQLScript.TestDirectiveStop;
begin
  StopToSend:=true;
  Script.OnSQLStatement := @NotifyStatement;
  Script.OnDirective := @NotifyDirective;
  script.Directives.Add ('STAT1');
  Script.Script.Text := 'stat1 ik;stat2;';
  script.execute;
  AssertEquals ('Received', 'STAT1(ik)', received);
  AssertSame ('Sender', script, LastSender);
end;

procedure TTestEventSQLScript.TestCommit;
begin
  Script.OnCommit := @Notify;
  Script.Script.Text := 'iets; commit; anders;';
  script.execute;
  AssertEquals ('NotifyCount', 1, NotifyCount);
  AssertSame ('Sender', script, LastSender);
end;

procedure TTestEventSQLScript.TestBeforeExec;
begin
  Script.BeforeExecute := @Notify;
  Script.Script.Text := 'update iets; anders iets;';
  script.execute;
  AssertEquals ('NotifyCount', 1, NotifyCount);
  AssertSame ('Sender', script, LastSender);
end;

procedure TTestEventSQLScript.TestAfterExec;
begin
  Script.AfterExecute := @Notify;
  Script.Script.Text := 'update iets; anders iets; en meer;';
  script.execute;
  AssertEquals ('NotifyCount', 1, NotifyCount);
  AssertSame ('Sender', script, LastSender);
end;

initialization

  RegisterTests ([TTestSQLScript, TTestEventSQLScript]);

end.

