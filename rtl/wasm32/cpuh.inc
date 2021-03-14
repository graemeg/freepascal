{

    This file is part of the Free Pascal run time library.
    Copyright (c) 2021 by the Free Pascal development team.

    CPU specific system unit header file

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

const
  {$I cpuinnr.inc}

function fpc_wasm32_memory_size: longword;[internproc:fpc_in_wasm32_memory_size];
function fpc_wasm32_memory_grow(n: longword): longword;[internproc:fpc_in_wasm32_memory_grow];
procedure fpc_wasm32_unreachable;[internproc:fpc_in_wasm32_unreachable];
