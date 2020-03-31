{
    This file is part of the Free Pascal run time library.

    timer.device definitions (V50) for MorphOS/PowerPC
    Copyright (c) 2002-3 The MorphOS Development Team, All Rights Reserved.

    Free Pascal conversion
    Copyright (c) 2004 Karoly Balogh for Genesi S.a.r.l. <www.genesi.lu>

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}


{ * timer.device definitions (V50)
  *********************************************************************
  * }

{$PACKRECORDS 2}

const
  UNIT_MICROHZ      = 0;
  UNIT_VBLANK       = 1;
  UNIT_ECLOCK       = 2;
  UNIT_WAITUNTIL    = 3;
  UNIT_WAITECLOCK   = 4;
  { *** V50 *** }
  UNIT_CPUCLOCK     = 5;
  UNIT_WAITCPUCLOCK = 6;

const
  TIMERNAME = 'timer.device';


type
  PTimeVal = ^TTimeVal;
  TTimeVal = record
    tv_secs : DWord;
    tv_micro: DWord;
  end;

type
  PEClockVal = ^TEClockVal;
  TEClockVal = record
    ev_hi: DWord;
    ev_lo: DWord;
  end;

type
  PTimeRequest = ^TTimeRequest;
  TTimeRequest = record
    tr_node: TIORequest;
    tr_time: TTimeVal;
  end;


const
   TR_ADDREQUEST = (CMD_NONSTD);
   TR_GETSYSTIME = (CMD_NONSTD + 1);
   TR_SETSYSTIME = (CMD_NONSTD + 2);

