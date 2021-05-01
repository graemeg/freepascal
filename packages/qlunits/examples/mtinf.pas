{
    Copyright (c) 2021 Karoly Balogh

    System info/System variables access on a Sinclair QL
    Example program for Free Pascal's Sinclair QL support

    This example program is in the Public Domain under the terms of
    Unlicense: http://unlicense.org/

 **********************************************************************}

program mtinf;

uses
  qdos;

type
  Tver = array[0..3] of char;

var
  job_id: longint;
  ver_ascii: longint;
  system_vars: pbyte;

function get_id_str(const id: dword): string;
const
  QDOS = $D2540000;
  SMS = $53324154; { S2AT }
  SMSQ = $534D5351; { SMSQ }
  ARGOS_THOR = $DC010000;
begin
  case id of
    QDOS: get_id_str:='QDOS';
    SMS: get_id_str:='SMS';
    SMSQ: get_id_str:='SMSQ';
    ARGOS_THOR: get_id_str:='Thor (ARGOS)';
  else
    get_id_str:='unknown ($'+hexstr(id,8)+')';
  end;
end;

begin
  job_id:=mt_inf(@system_vars,@ver_ascii);

  writeln('Job ID:',lo(job_id),' Tag:',hi(job_id));
  writeln('Identification: ',get_id_str(pdword(@system_vars[SV_IDENT])^));
  writeln('Version: ',Tver(ver_ascii));

  writeln('System vars are at: $',hexstr(system_vars));
  writeln('Processor type: 680',hexstr(system_vars[SV_PTYP],2));
  writeln('Monitor mode: ',system_vars[SV_TVMOD]);
  writeln('Random number: ',pword(@system_vars[SV_RAND])^);
end.
