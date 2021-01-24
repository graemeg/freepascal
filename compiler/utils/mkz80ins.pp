{
    Copyright (c) 2020 by Nikolay Nikolov

    Convert z80ins.dat to a set of .inc files for usage with
    the Free pascal compiler

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

program mkz80ins;

{$mode objfpc}{$H+}

uses
  SysUtils;

const
  Version = '1.0.0';
  HeaderStr = '{ don''t edit, this file is generated from z80ins.dat; to regenerate, run ''make insdat'' in the compiler directory }';
  max_operands = 2;

  ParamTypes: array [0..40,0..1] of string = (
    ('void',  'OT_NONE'),
    ('r',     'OT_REG8'),
    ('r''',   'OT_REG8'),
    ('b',     'OT_IMM3'),
    ('n',     'OT_IMM8'),
    ('p',     'OT_IMM_RST'),
    ('e',     'OT_RELJMP8'),
    ('nn',    'OT_IMM16'),
    ('0',     'OT_IMM_VAL0'),
    ('1',     'OT_IMM_VAL1'),
    ('2',     'OT_IMM_VAL2'),
    ('cc',    'OT_COND'),
    ('C',     'OT_COND_C'),
    ('NC',    'OT_COND_NC'),
    ('Z',     'OT_COND_Z'),
    ('NZ',    'OT_COND_NZ'),
    ('dd',    'OT_REG16_BC_DE_HL_SP'),
    ('qq',    'OT_REG16_BC_DE_HL_AF'),
    ('pp',    'OT_REG16_BC_DE_IX_SP'),
    ('rr',    'OT_REG16_BC_DE_IY_SP'),
    ('A',     'OT_REG8_A'),
    ('I',     'OT_REG8_I'),
    ('R',     'OT_REG8_R'),
    ('IX',    'OT_REG16_IX'),
    ('IY',    'OT_REG16_IY'),
    ('SP',    'OT_REG16_SP'),
    ('DE',    'OT_REG16_DE'),
    ('HL',    'OT_REG16_HL'),
    ('AF',    'OT_REG16_AF'),
    ('AF''',  'OT_REG16_AF_'),
    ('(C)',   'OT_REG8_C_PORT'),
    ('(n)',   'OT_IMM_PORT'),
    ('(nn)',  'OT_REF_ADDR16'),
    ('(BC)',  'OT_REF_BC'),
    ('(DE)',  'OT_REF_DE'),
    ('(HL)',  'OT_REF_HL'),
    ('(SP)',  'OT_REF_SP'),
    ('(IX)',  'OT_REF_IX'),
    ('(IY)',  'OT_REF_IY'),
    ('(IX+d)','OT_REF_IX_d'),
    ('(IY+d)','OT_REF_IY_d')
  );

type

  { TZ80InsDatOutputFiles }

  TZ80InsDatOutputFiles = class
  public
    OpFile: TextFile;
    NOpFile: TextFile;
    StdOpNames: TextFile;
    InsTabFile: TextFile;

    constructor Create;
    destructor Destroy;override;
  end;

{ ***************************************************************************
  the routines LeftStr, AnsiStartsStr are copied and reformatted
  from StrUtils and thus covered by the copyright of strutils (see below) as compiler utilities cannot
  depend on packages

    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2005 by the Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*************************************************************************** }

function LeftStr(const AText: AnsiString; const ACount: SizeInt): AnsiString;
  begin
    Result:=Copy(AText,1,ACount);
  end;


function AnsiStartsStr(const ASubText, AText: string): Boolean;
  begin
    Result := (ASubText = '') or (LeftStr(AText, Length(ASubText)) = ASubText);
  end;

{ ***************************************************************************
 end of StrUtils code
***************************************************************************}

function PasEncode(const S: string): string;
  var
    Ch: Char;
    InQuotes: Boolean;
  begin
    Result:='';
    InQuotes:=False;
    for Ch in S do
      if (Ch>=#32) and (Ch<=#126) then
        begin
          if not InQuotes then
            begin
              Result:=Result+'''';
              InQuotes:=True;
            end;
          if Ch='''' then
            Result:=Result+''''''
          else
            Result:=Result+Ch;
        end
      else
        begin
          if InQuotes then
            begin
              Result:=Result+'''';
              InQuotes:=False;
            end;
          Result:=Result+'#'+IntToStr(Ord(Ch));
        end;
    if InQuotes then
      Result:=Result+'''';
    if Result='' then
      Result:='''''';
  end;

constructor TZ80InsDatOutputFiles.Create;
  begin
    AssignFile(OpFile,'z80op.inc');
    Rewrite(OpFile);
    Writeln(OpFile,HeaderStr);
    Writeln(OpFile,'(');
    AssignFile(NOpFile,'z80nop.inc');
    Rewrite(NOpFile);
    Writeln(NOpFile,HeaderStr);
    AssignFile(StdOpNames,'z80stdopnames.inc');
    Rewrite(StdOpNames);
    Writeln(StdOpNames,HeaderStr);
    Writeln(StdOpNames,'(');
    AssignFile(InsTabFile,'z80tab.inc');
    Rewrite(InsTabFile);
    Writeln(InsTabFile,HeaderStr);
    Writeln(InsTabFile,'(');
  end;

destructor TZ80InsDatOutputFiles.Destroy;
  begin
    CloseFile(OpFile);
    CloseFile(NOpFile);
    CloseFile(StdOpNames);
    CloseFile(InsTabFile);
    inherited Destroy;
  end;

function FindParamType(const ParamTypeStr: string): Integer;
var
  I: Integer;
begin
  for I:=Low(ParamTypes) to High(ParamTypes) do
    if ParamTypes[I,0]=ParamTypeStr then
      exit(I);
  raise Exception.Create('Invalid param type: '''+ParamTypeStr+'''');
end;

var
  InsDatFile: TextFile;
  OutputFiles: TZ80InsDatOutputFiles=nil;
  S, op, ParamsStr: string;
  FirstIns: Boolean=true;
  OpCount: Integer=0;
  S_Split, S_Params: TStringArray;
  ParamIdx: Integer;
begin
  writeln('FPC Z80 Instruction Table Converter Version ',Version);
  AssignFile(InsDatFile,'../z80/z80ins.dat');
  Reset(InsDatFile);
  try
    OutputFiles:=TZ80InsDatOutputFiles.Create;
    while not EoF(InsDatFile) do
      begin
        Readln(InsDatFile,S);
        S:=Trim(S);
        if AnsiStartsStr(';',S) then
          continue
        else if AnsiStartsStr('[',S) then
          begin
            op:=Copy(S,2,Length(S)-2);
            if not FirstIns then
              begin
                Writeln(OutputFiles.OpFile,',');
                Writeln(OutputFiles.StdOpNames,',');
              end;
            FirstIns:=False;
            Write(OutputFiles.OpFile,'A_'+op);
            Write(OutputFiles.StdOpNames,''''+LowerCase(op)+'''');
          end
        else if S<>'' then
          begin
            Inc(OpCount);
            if OpCount<>1 then
              Writeln(OutputFiles.InsTabFile,',');
            S_Split:=S.Split(' ',TStringSplitOptions.ExcludeEmpty);
            S_Params:=S_Split[0].Split(',',TStringSplitOptions.ExcludeEmpty);
            if (Length(S_Params)=1) and (S_Params[0]='void') then
              SetLength(S_Params,0);
            Writeln(OutputFiles.InsTabFile,'  (');
            Writeln(OutputFiles.InsTabFile,'    opcode  : A_',op,';');
            Writeln(OutputFiles.InsTabFile,'    ops     : ',Length(S_Params),';');
            Write(OutputFiles.InsTabFile,  '    optypes : (');
            if Length(S_Params)>max_operands then
              raise Exception.Create('Too many operands');
            for ParamIdx:=0 to max_operands-1 do
              begin
                if ParamIdx<>0 then
                  Write(OutputFiles.InsTabFile,',');
                if ParamIdx<=High(S_Params) then
                  Write(OutputFiles.InsTabFile,ParamTypes[FindParamType(S_Params[ParamIdx]),1])
                else
                  Write(OutputFiles.InsTabFile,'OT_NONE');
              end;
            Writeln(OutputFiles.InsTabFile, ');');
            Writeln(OutputFiles.InsTabFile,  '    code    : ',PasEncode(S_Split[1]),';');
            Writeln(OutputFiles.InsTabFile,  '    flags   : 0');
            Write(OutputFiles.InsTabFile,  '  )');
          end;
      end;
    Writeln(OutputFiles.OpFile,');');
    Writeln(OutputFiles.StdOpNames,');');
    Writeln(OutputFiles.NOpFile,OpCount,';');
    Writeln(OutputFiles.InsTabFile);
    Writeln(OutputFiles.InsTabFile,');');
  finally
    FreeAndNil(OutputFiles);
    CloseFile(InsDatFile);
  end;
end.

