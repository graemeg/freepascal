{

    This file is part of the Free Pascal run time library.
    Copyright (c) 2016 by the Free Pascal development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

  in_avr_cli = fpc_in_cpu_first;
  in_avr_sei = fpc_in_cpu_first+1;
  in_avr_wdr = fpc_in_cpu_first+2;
  in_avr_sleep = fpc_in_cpu_first+3;
  in_avr_nop = fpc_in_cpu_first+4;
  in_avr_save = fpc_in_cpu_first+5;
  in_avr_restore = fpc_in_cpu_first+6;
  in_avr_des = fpc_in_cpu_first+7;
