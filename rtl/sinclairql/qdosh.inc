{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2020 by Karoly Balogh

    Types and Constants used by QDOS OS functions in the Sinclair QL RTL

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

type
  Tchanid = longint;
  Tjobid = longint;
  Ttimeout = smallint;


const
  ERR_NC = -1;   { Operation not complete }
  ERR_NJ = -2;   { Not a (valid) job. }
  ERR_OM = -3;   { Out of memory. }
  ERR_OR = -4;   { Out of range. }
  ERR_BO = -5;   { Buffer overflow. }
  ERR_NO = -6;   { Channel not open. }
  ERR_NF = -7;   { File or device not found. }
  ERR_FX = -8;   { File already exists. }
  ERR_IU = -9;   { File or device already in use. }
  ERR_EF = -10;  { End of file. }
  ERR_DF = -11;  { Drive full. }
  ERR_BN = -12;  { Bad device. }
  ERR_TE = -13;  { Transmission error. }
  ERR_FF = -14;  { Format failed. }
  ERR_BP = -15;  { Bad parameter. }
  ERR_FE = -16;  { File error. }
  ERR_EX = -17;  { Expression error. }
  ERR_OV = -18;  { Arithmetic overflow. }
  ERR_NI = -19;  { Not implemented. }
  ERR_RO = -20;  { Read only. }
  ERR_BL = -21;  { Bad line of Basic. }

const
  Q_OPEN = 0;
  Q_OPEN_IN = 1;
  Q_OPEN_NEW = 2;
  Q_OPEN_OVER = 3;  { Not available on microdrives. }
  Q_OPEN_DIR = 4;

type
  Tqlfloat = array[0..5] of byte;
  Pqlfloat = ^Tqlfloat;

type
  TQLRect = record
    q_width : word;
    q_height : word;
    q_x : word;
    q_y : word;
  end;
  PQLRect = ^TQLRect;

type
  TWindowDef = record
    border_colour : byte;
    border_width : byte;
    paper : byte;
    ink : byte;
    width : word;
    height : word;
    x_origin: word;
    y_origin: word;
  end;
  PWindowDef = ^TWindowDef;

