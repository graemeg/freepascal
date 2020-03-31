{
    This file is part of the Free Pascal run time library.

    utility definitions (V50) for MorphOS/PowerPC
    Copyright (c) 2002 The MorphOS Development Team, All Rights Reserved.

    Free Pascal conversion, second part
    Copyright (c) 2004 Karoly Balogh for Genesi S.a.r.l. <www.genesi.lu>

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}


{ * utility.library include
  *********************************************************************
  * }

{$PACKRECORDS 2}

const
  UtilityName = 'utility.library';


type
  PUtilityBase = ^TUtilityName;
  TUtilityName = record
    ub_LibNode : TLibrary;
    ub_Language: Byte;
    ub_Reserved: Byte;
  end;



{ * utility.library hook defines
  *********************************************************************
  * }


type
  PHook = ^THook;
  THook = record
    h_MinNode : TMinNode;
    h_Entry   : Cardinal;
    h_SubEntry: Cardinal;
    h_Data    : Pointer;
  end;


