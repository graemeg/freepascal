{ Author: Mattias Gaertner  2018  mattias@freepascal.org

  Abstract:
    Command line interface for the pas2js compiler.
}
program pas2js;

{$mode objfpc}{$H+}

uses
  {$IFDEF UNIX}
  cthreads, cwstring,
  {$ENDIF}
  Classes, SysUtils, CustApp,
  Pas2jsFileUtils, Pas2jsLogger, Pas2jsCompiler,
  Pas2JSFSCompiler, Pas2JSCompilerPP, Pas2JSCompilerCfg;

Type

  { TPas2jsCLI }

  TPas2jsCLI = class(TCustomApplication)
  private
    FCompiler: TPas2JSFSCompiler;
    FWriteOutputToStdErr: Boolean;
  protected
    procedure DoRun; override;
  public
    constructor Create(TheOwner: TComponent); override;
    destructor Destroy; override;
    property Compiler: TPas2JSFSCompiler read FCompiler;
    property WriteOutputToStdErr: Boolean read FWriteOutputToStdErr write FWriteOutputToStdErr;
  end;

procedure TPas2jsCLI.DoRun;
var
  ParamList: TStringList;
  i: Integer;
begin
  ParamList:=TStringList.Create;
  try
    for i:=1 to ParamCount do
      ParamList.Add(Params[i]);
    try
      Compiler.Run(ParamStr(0),GetCurrentDirPJ,ParamList);
    except
      on E: ECompilerTerminate do ;
      on E: Exception do
      begin
        {AllowWriteln}
        writeln('Error: Unhandled exception '+E.ClassName+': '+E.Message);
        {AllowWriteln-}
        if ExitCode=0 then
          ExitCode:=ExitCodeErrorInternal;
      end;
    end;
  finally
    ParamList.Free;
    Compiler.Log.CloseOutputFile;
  end;

  // stop program loop
  Terminate; // Keep ExitCode!
end;

constructor TPas2jsCLI.Create(TheOwner: TComponent);
begin
  inherited Create(TheOwner);
  StopOnException:=True;
  FCompiler:=TPas2JSFSCompiler.Create;
  FCompiler.ConfigSupport:=TPas2JSFileConfigSupport.Create(FCompiler);
  FCompiler.PostProcessorSupport:=TPas2JSFSPostProcessorSupport.Create(FCompiler);
end;

destructor TPas2jsCLI.Destroy;
begin
  FreeAndNil(FCompiler);
  inherited Destroy;
end;

var
  Application: TPas2jsCLI;
begin
  Application:=TPas2jsCLI.Create(nil);
  Application.Run;
  Application.Free;
end.

