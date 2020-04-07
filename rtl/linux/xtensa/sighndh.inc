{
    This file is part of the Free Pascal run time library.
    Copyright (c) 1999-2000 by Jonas Maebe,
    member of the Free Pascal development team.

    TSigContext

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{$packrecords C}

type
  PSigContext = ^TSigContext;
  TSigContext = record
    sc_pc,
    sc_ps,
    sc_lbeg,
    sc_lend,
    sc_lcount,
    sc_sar,
    sc_acclo,
    sc_acchi : culong;
    sc_a : array[0..15] of culong;
  end;

  stack_t = record
    ss_sp : pointer;
    ss_flags : cint;
    ss_size : size_t;
  end;

  pucontext = ^tucontext;
  tucontext = record
    uc_flags : culong;
    uc_link : pucontext;
    uc_stack : stack_t;
    uc_mcontext : TSigContext;
    uc_sigmask : tsigset;
  end;
