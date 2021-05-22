{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2021 by Karoly Balogh

    Headers to SMS only OS functions used by the Sinclair QL RTL

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

function sms_info(sys_vars: ppchar; ver_ascii: plongint): Tjobid; external name '_mt_inf';
function iob_fbyt(chan: Tchanid; timeout: Ttimeout): longint; external name '_io_fbyte';
function iof_mkdr(chan: Tchanid): longint; external name '_iof_mkdr';
