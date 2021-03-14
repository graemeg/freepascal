{ This file is part of wasmbin - a collection of WebAssembly binary utils.

  Copyright (C) 2019, 2020 Dmitry Boyarintsev <skalogryz.lists@gmail.com>
  Copyright (C) 2020 by the Free Pascal development team

  This source is free software; you can redistribute it and/or modify it under
  the terms of the GNU General Public License as published by the Free
  Software Foundation; either version 2 of the License, or (at your option)
  any later version.

  This code is distributed in the hope that it will be useful, but WITHOUT ANY
  WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
  FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
  details.

  A copy of the GNU General Public License is available on the World Wide Web
  at <http://www.gnu.org/copyleft/gpl.html>. You can also obtain it by writing
  to the Free Software Foundation, Inc., 51 Franklin Street - Fifth Floor,
  Boston, MA 02110-1335, USA.
}

program wasmtool;

{$mode objfpc}{$H+}

uses
  {$IFDEF UNIX}{$IFDEF UseCThreads}
  cthreads,
  {$ENDIF}{$ENDIF}
  { you can add units after this }
  Classes, SysUtils, wasmbin, lebutils,
  wasmbindebug, wasmlink, wasmlinkchange,
  wasmtoolutils;

const
  ACT_EXPORTRENAME   = 'exportrename';
  ACT_SYMBOLFLAG     = 'symbolflag';
  ACT_SYMBOLAUTO     = 'symbolauto';
  ACT_WEAK           = 'weak';

  VERSION = '1.1';

procedure PrintHelp;
begin
  writeln('wasmtool [options] .wasm file...');
  writeln();
  writeln('version: ',VERSION);
  writeln;
  writeln('options:');
  writeln('  --exportrename @inputfile - renaming export names');
  writeln('  --symbolflag   @inputfile - update symbol flags as specified in input');
  writeln('  --symbolauto              - update symbol by the use ');
  writeln('    --weak %name%           - specify symbol names for global variables to be marked weak reference');
end;

type

  { TToolActions }

  // it's assumed that the wasmtool will be instructed
  // to take mulitple actions on the same file.
  // thus every action is recorded as TToolActions
  // Those are created during the parameters parsing

  TToolActions = class(TObject)
    action   : string; // action to take place
    paramsFn : string; // input file name
    constructor Create(const aaction, afilename: string);
  end;

{ TToolActions }

constructor TToolActions.Create(const aaction, afilename: string);
begin
  inherited Create;
  action := aaction;
  paramsFn := afilename;
end;

procedure ProcessParams(acts: TList; const inputFn: string; doVerbose: Boolean);
var
  i  : integer;
  j  : integer;
  ta : TToolActions;
  wk : TStringList;
begin
  for i:=0 to acts.Count-1 do begin
    ta := TToolActions(acts[i]);
    if doVerbose then writeln('action: "',ta.action,'"');
    if ta.action = ACT_EXPORTRENAME then begin
      if doVerbose then begin
        writeln('  input:  ',inputFn);
        writeln('  params: ',ta.paramsFn);
      end;
      ExportRename(inputFn, ta.paramsFn, doVerbose);
    end else if ta.action = ACT_SYMBOLFLAG then begin
      ChangeSymbolFlag(inputFn, ta.paramsFn);
    end else if (ta.action = ACT_SYMBOLAUTO) then begin
      wk := TStringList.Create;
      for j:=0 to acts.Count-1 do begin
        if TToolActions(acts[j]).action = ACT_WEAK then
          wk.Add( TToolActions(acts[j]).paramsFn );
      end;
      PredictSymbolsFromLink(inputFn, wk, doVerbose);
      wk.free;
    end;
  end;
end;

var
  acts: TList = nil;
  inputFn: string = '';
  verbose: Boolean = false;

procedure ParseParams;
var
  i : integer;
  s : string;
  ls : string;
  fn : string;
begin
  i:=1;
  while i<=ParamCount do begin
    s := ParamStr(i);
    ls := AnsiLowerCase(s);
    inc(i);

    if Pos('-', ls)=1 then begin
      if ls = '-v' then
        ls := '--verbose';
    end;

    if Pos('--',ls)=1 then begin
      ls := Copy(ls, 3, length(ls)-2);
      if (ls = 'verbose') then
        verbose := true
      else if (ls = ACT_SYMBOLAUTO) then
        acts.Add( TToolActions.Create(ls, ''))
      else begin
        if i<=ParamCount then begin
          fn:=ParamStr(i);
          inc(i);
        end else
          fn := '';
        if fn <> '' then
          acts.Add( TToolActions.Create(ls, fn));
      end;
    end else begin
      if inputFn ='' then
        inputFn:=s;
    end;
  end;
end;

var
  i : integer;
begin
  if ParamCount=0 then begin
    PrintHelp;
    Exit;
  end;

  try
    acts := TList.Create;
    try
      ParseParams;
      ProcessParams(acts, inputFn, verbose);
    finally
      for i:=0 to acts.Count-1 do
        TObject(acts[i]).Free;
      acts.Free;
    end;
  except
    on e:exception do begin
      writeln('error: ', e.Message);
      ExitCode:=1;
    end;
  end;
end.

