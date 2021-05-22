{
    Copyright (c) 2021 Karoly Balogh

    Test system variable offsets on a Sinclair QL
    A test program for Free Pascal's Sinclair QL support

    This test program is in the Public Domain under the terms of
    Unlicense: http://unlicense.org/

 **********************************************************************}

program tsysvars;

uses
  qdos;

var
  system_vars: SystemVariables;

type
  offset_test = record
    name: string[16];
    offset: longint;
    sysvar: pointer;
  end;

const
  { example offsets. feel free to extend... }
  sysvar_offsets: array of offset_test = (
    ( name: 'SV_RAND'; offset: $2E; sysvar: @system_vars.SV_RAND ),
    ( name: 'SV_TMODE'; offset: $A0; sysvar: @system_vars.SV_TMODE ),
    ( name: 'SV_CSUB'; offset: $A2; sysvar: @system_vars.SV_CSUB ),
    ( name: 'SV_PRGD'; offset: $AC; sysvar: @system_vars.SV_PRGD ),
    ( name: 'SV_TURBO'; offset: $160; sysvar: @system_vars.SV_TURBO )
  );

function check_sysvar_offset(const test: offset_test): boolean;
var
  actual_offset: longint;
begin
  actual_offset:=pbyte(test.sysvar)-pbyte(@system_vars);
  writeln(test.name,' at: ',actual_offset,' $',hexstr(actual_offset,3));
  check_sysvar_offset:=(test.offset = actual_offset);
end;

function test_sysvar_offsets: boolean;
var
  i: longint;
begin
  test_sysvar_offsets:=false;
  for i:=low(sysvar_offsets) to high(sysvar_offsets) do
    if not check_sysvar_offset(sysvar_offsets[i]) then
      exit;
  test_sysvar_offsets:=true;
end;

begin
  if test_sysvar_offsets then
    writeln('All OK!')
  else
    writeln('Error! Wrong offset!');
end.
