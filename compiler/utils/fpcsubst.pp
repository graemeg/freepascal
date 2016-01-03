{$Mode objfpc}
{$H+}
program fpcsubst;

uses SysUtils,Classes,Usubst;

Const
  BuildVersion={$I %FPCVERSION%};
  BuildTarget={$I %FPCTARGET%};

Resourcestring
  SUsage00 = 'Usage: %s [options]';
  SUsage10 = 'Where options is one or more of';
  SUSage20 = '  -i filename   Set input file. Default is standard input';
  SUSage30 = '  -o filename   Set output file. Default is standard output.';
  SUsage40 = '  -d name=value define name=value pair.';
  SUsage50 = '  -h            show this help and exit.';
  SUsage60 = '  -u name       remove name from list of name/value pairs.';
  SUsage70 = '  -l filename   read name/value pairs from filename';
  SUsage80 = '  -b            show builtin list and exit.';
  SUsage90 = '  -v            be verbose.';
  SErrUnknownOption = 'Error: Unknown option.';
  SErrArgExpected = 'Error: Option "%s" requires an argument.';
  SErrNoSuchFile = 'Error: File "%s" does not exist.';
  SErrBackupFailed = 'Error: Backup of file "%s" to "%s" failed.';
  SErrDelBackupFailed = 'Error: Delete of old backup file "%s" failed.';
  SWarnIgnoringFile = 'Warning: Ignoring non-existent file: ';
  SWarnIgnoringPair = 'Warning: ignoring wrong name/value pair: ';
  SStats = 'Replaced %d placeholders in %d lines.';
  SSubstInLine = 'Replaced %s placeholders in line %d.';
  SWarningDeprecated = 'Warning: This utility is deprecated and will be removed from fpc in the future. Please use fpcmkcfg instead.';


Var
  List : TStringList;
  InputFileName : String;
  OutputFileName : String;
  Verbose : Boolean;
  SkipBackup : Boolean;





procedure Init;

begin
  Verbose:=False;
  List:=TStringList.Create;
  AddToList(List,'FPCVERSION',BuildVersion);
  AddToList(List,'FPCTARGET',BuildTarget);
  AddToList(List,'PWD',GetCurrentDir);
  AddToList(List,'BUILDDATE',DateToStr(Date));
  AddToList(List,'BUILDTIME',TimeToStr(Time));
end;

Procedure Done;

begin
  FreeAndNil(List);
end;

Procedure Usage;

begin
  Writeln(Format(SUsage00,[ExtractFileName(Paramstr(0))]));
  Writeln(SUsage10);
  Writeln(SUsage20);
  Writeln(SUsage30);
  Writeln(SUsage40);
  Writeln(SUsage50);
  Writeln(SUsage60);
  Writeln(SUsage70);
  Writeln(SUsage80);
  Writeln(SUsage90);
  Halt(1);
end;

Procedure ShowBuiltIns;

var
  I : Integer;

begin
  for I:=0 to List.Count-1 do
    Writeln(List[i]);
end;




Procedure AddFromFile(FN : String);

Var
  F : Text;
  S : String;

begin
  If Not FileExists(FN) then
    begin
    Writeln(StdErr,SWarnIgnoringFile,FN);
    Exit;
    end;
  Assign(F,FN);
  Reset(F);
  Try
    While not EOF(F) do
      begin
      ReadLn(F,S);
      If (Length(S)>0) and (not (S[1] in ['#',';'])) then
        If not AddPair(List,S) then
         If Verbose then
           Writeln(StdErr,SWarnIgnoringPair,S)
      end;
  finally
    Close(F);
  end;
end;

Procedure UnknownOption(Const S : String);

begin
  Writeln(SErrUnknownOption,S);
  Usage;
end;

Procedure ProcessCommandline;

Var
  I : Integer;
  S : String;

  Function GetOptArg : String;

  begin
    If I=ParamCount then
      begin
      Writeln(StdErr,Format(SErrArgExpected,[S]));
      Halt(1);
      end;
    inc(I);
    Result:=ParamStr(I);
  end;

begin
  I:=1;
  While( I<=ParamCount) do
    begin
    S:=Paramstr(i);
    If (Length(S)<=1) or (S[1]<>'-') then
      UnknownOption(S)
    else
      case S[2] of
        'v' : Verbose:=True;
        'h' : Usage;
        'b' : begin
              ShowBuiltins;
              halt(0);
              end;
        'l' : AddFromFile(GetOptArg);
        'd' : AddPair(List,GetOptArg);
        'u' : AddPair(List,GetOptArg+'=');
        'i' : InputFileName:=GetOptArg;
        'o' : OutputFileName:=GetoptArg;
        's' : SkipBackup:=True;
      else
        UnknownOption(S);
      end;
    Inc(I);
    end;
end;



Procedure DoFile;

Var
  Fin,Fout : Text;
  S,BFN : String;
  N,LCount,RCount : Integer;


begin
  If (InputFileName<>'') and not FileExists(InputFIleName) then
    begin
    Writeln(StdErr,Format(SErrNoSuchFile,[InputFileName]));
    Halt(1)
    end;
  If (OutputFileName<>'')
     and FileExists(OutputFileName)
     and not SkipBackup then
    begin
    BFN:=ChangeFileExt(OutputFileName,'.bak');
    If FileExists(BFN) and not DeleteFile(BFN) then
      begin
      Writeln(StdErr,Format(SErrDelBackupFailed,[BFN]));
      Halt(1);
      end;
    If not RenameFile(OutputFileName,BFN) then
      begin
      Writeln(StdErr,Format(SErrBackupFailed,[OutputFileName,BFN]));
      Halt(1);
      end;
    end;
  Assign(Fin,InputFileName);
  Assign(Fout,OutputFileName);
  Reset(Fin);
  Try
    Rewrite(FOut);
    Try
      LCount:=0;
      RCount:=0;
      While Not EOF(Fin) do
        begin
        Inc(LCount);
        ReadLn(Fin,S);
        N:=DoSubstitutions(List,S);
        If Verbose and (N>0) then
          Writeln(StdErr,Format(SSubstInLine,[N,LCount]));
        Inc(RCount,N);
        Writeln(Fout,S);
        end;
     If Verbose then
       Writeln(StdErr,Format(SStats,[RCount,LCount]));
    Finally
      Close(Fout);
    end;
  Finally
    Close(Fin);
  end;

end;

begin
  WriteLn(StdErr,SWarningDeprecated);
  Init;
  Try
    ProcessCommandLine;
    DoFile;
  Finally
    Done;
  end;
end.
