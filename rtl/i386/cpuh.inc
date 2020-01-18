{

    This file is part of the Free Pascal run time library.
    Copyright (c) 2016 by the Free Pascal development team.

    CPU specific system unit header file

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

const
  {$I cpuinnr.inc}
  { Always i386 or newer }
  Test8086 : byte = 2;
  { Always 387 or newer. Emulated if needed. }
  Test8087 : byte = 3;
  { will be detected at startup }
  has_sse_support : boolean = false;
  has_sse2_support : boolean = false;
  has_sse3_support : boolean = false;
  has_mmx_support : boolean = false;

function fpc_x86_inportb(port : word) : byte;[internproc:fpc_in_x86_inportb];
function fpc_x86_inportw(port : word) : word;[internproc:fpc_in_x86_inportw];
function fpc_x86_inportl(port : word) : longint;[internproc:fpc_in_x86_inportl];
procedure fpc_x86_outportb(port : word;data : byte);[internproc:fpc_in_x86_outportb];
procedure fpc_x86_outportw(port : word;data : word);[internproc:fpc_in_x86_outportw];
procedure fpc_x86_outportl(port : word;data : longint);[internproc:fpc_in_x86_outportl];
procedure fpc_x86_cli;[internproc:fpc_in_x86_cli];
procedure fpc_x86_sti;[internproc:fpc_in_x86_sti];
function fpc_x86_get_cs:longint;[internproc:fpc_in_x86_get_cs];
function fpc_x86_get_ss:longint;[internproc:fpc_in_x86_get_ss];
function fpc_x86_get_ds:longint;[internproc:fpc_in_x86_get_ds];
function fpc_x86_get_es:longint;[internproc:fpc_in_x86_get_es];
function fpc_x86_get_fs:longint;[internproc:fpc_in_x86_get_fs];
function fpc_x86_get_gs:longint;[internproc:fpc_in_x86_get_gs];

{ include automatically generated procs }
{$if not defined(VER3_0) and not defined(VER3_2)}
{ do not active yet, they are not usable yet neither is the naming fixed }
{ $i cpummprocs.inc}
{$endif not VER3_0 and not VER3_2}

