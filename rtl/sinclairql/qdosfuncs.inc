{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2020 by Karoly Balogh

    Headers to QDOS OS functions used by the Sinclair QL RTL

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{$i qdosh.inc}

function mt_inf(sys_vars: ppchar; ver_ascii: plongint): Tjobid; external name '_mt_inf';

procedure mt_dmode(s_mode: pword; d_type: pword); external name '_mt_dmode';

function mt_alchp(size: dword; sizegot: pdword; jobid: Tjobid): pointer; external name '_mt_alchp';
procedure mt_rechp(area: pointer); external name '_mt_rechp';

function io_open_qlstr(name_qlstr: pointer; mode: longint): Tchanid; external name '_io_open_qlstr';
function io_open(name: pchar; mode: longint): Tchanid; external name '_io_open';
function io_close(chan: Tchanid): longint; external name '_io_close';

function io_sbyte(chan: Tchanid; timeout: Ttimeout; c: char): longint; external name '_io_sbyte';
function io_sstrg(chan: Tchanid; timeout: Ttimeout; buf: pointer; len: smallint): smallint; external name '_io_sstrg';

function sd_wdef(chan: Tchanid; timeout: Ttimeout; border_colour: byte; border_width: word; window: PQLRect): longint; external name '_sd_wdef'; 
function sd_clear(chan: Tchanid; timeout: Ttimeout): longint; external name '_sd_clear';

function ut_con(params: PWindowDef): Tchanid; external name '_ut_con';
function ut_scr(params: PWindowDef): Tchanid; external name '_ut_scr';
