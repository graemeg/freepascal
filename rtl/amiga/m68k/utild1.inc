{
    This file is part of the Free Pascal run time library.

    utility definitions (V50) for MorphOS/PowerPC
    Copyright (c) 2002 The MorphOS Development Team, All Rights Reserved.

    Free Pascal conversion, first part
    Copyright (c) 2004 Karoly Balogh for Genesi S.a.r.l. <www.genesi.lu>

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}


{ * utility.library date defines
  *********************************************************************
  * }

{$PACKRECORDS 2}

type
  PClockData = ^TClockData;
  TClockData = record
    sec  : Word;
    min  : Word;
    hour : Word;
    mday : Word;
    month: Word;
    year : Word;
    wday : Word;
  end;



{ * utility.library tagitem defines
  *********************************************************************
  * }


type
  Tag = Cardinal;

type
  PPTagItem = ^PTagItem;
  PTagItem = ^TTagItem;
  TTagItem = record
    ti_Tag : Tag;
    ti_Data: Cardinal;
  end;


const
  TAG_DONE   = 0;
  TAG_END    = 0;
  TAG_IGNORE = 1;
  TAG_MORE   = 2;
  TAG_SKIP   = 3;

const
  TAG_USER   = 1 Shl 31;

const
  TAGFILTER_AND = 0;
  TAGFILTER_NOT = 1;

const
  MAP_REMOVE_NOT_FOUND = 0;
  MAP_KEEP_NOT_FOUND   = 1;



{ * utility.library namespace defines
  *********************************************************************
  * }


type
  PNamedObject = ^TNamedObject;
  TNamedObject = record
    no_Object: Pointer;
  end;


const
  ANO_NameSpace  = 4000;
  ANO_UserSpace  = 4001;
  ANO_Priority   = 4002;
  ANO_Flags      = 4003;

  NSB_NODUPS = 0;
  NSB_CASE   = 1;

  NSF_NODUPS = 1 Shl NSB_NODUPS;
  NSF_CASE   = 1 Shl NSB_CASE;



{ * utility.library pack attributes and macros
  *********************************************************************
  * }


const
  PSTB_SIGNED = 31;
  PSTB_UNPACK = 30;
  PSTB_PACK   = 29;
  PSTB_EXISTS = 26;

  PSTF_SIGNED = (1 Shl PSTB_SIGNED);
  PSTF_UNPACK = (1 Shl PSTB_UNPACK);
  PSTF_PACK   = (1 Shl PSTB_PACK);
  PSTF_EXISTS = (1 Shl PSTB_EXISTS);

const
  PKCTRL_PACKUNPACK = $00000000;
  PKCTRL_PACKONLY   = $40000000;
  PKCTRL_UNPACKONLY = $20000000;

  PKCTRL_BYTE       = $80000000;
  PKCTRL_WORD       = $88000000;
  PKCTRL_LONG       = $90000000;

  PKCTRL_UBYTE      = $00000000;
  PKCTRL_UWORD      = $08000000;
  PKCTRL_ULONG      = $10000000;

  PKCTRL_BIT        = $18000000;
  PKCTRL_FLIPBIT    = $98000000;

{$WARNING FIX ME!!! Some macros to convert}
{
  PK_BITNUM1(flg)            ((flg) == 0x01 ? 0 : (flg) == 0x02 ? 1 : (flg) == 0x04 ? 2 : (flg) == 0x08 ? 3 : (flg) == 0x10 ? 4 : (flg) == 0x20 ? 5 : (flg) == 0x40 ? 6 : 7)
  PK_BITNUM2(flg)            ((flg < 0x100 ? PK_BITNUM1(flg) : 8 + PK_BITNUM1(flg >> 8)))
  PK_BITNUM(flg)             ((flg < 0x10000 ? PK_BITNUM2(flg) : 16 + PK_BITNUM2(flg >> 16)))
  PK_WORDOFFSET(flg)         ((flg) < 0x100 ? 1 : 0)
  PK_LONGOFFSET(flg)         ((flg) < 0x100  ? 3 : (flg) < 0x10000 ? 2 : (flg) < 0x1000000 ? 1 : 0)
  PK_CALCOFFSET(type,field)  ((ULONG)(&((struct type *)0)->field))


  PACK_STARTTABLE(tagbase)                           (tagbase)
  PACK_NEWOFFSET(tagbase)                            (-1L),(tagbase)
  PACK_ENDTABLE                                      0
  PACK_ENTRY(tagbase,tag,type,field,control)         (control | ((tag-tagbase) << 16L) | PK_CALCOFFSET(type,field))
  PACK_BYTEBIT(tagbase,tag,type,field,control,flags) (control | ((tag-tagbase) << 16L) | PK_CALCOFFSET(type,field) | (PK_BITNUM(flags) << 13L))
  PACK_WORDBIT(tagbase,tag,type,field,control,flags) (control | ((tag-tagbase) << 16L) | (PK_CALCOFFSET(type,field) + PK_WORDOFFSET(flags)) | ((PK_BITNUM(flags) & 7) << 13L))
  PACK_LONGBIT(tagbase,tag,type,field,control,flags) (control | ((tag-tagbase) << 16L) | (PK_CALCOFFSET(type,field) + PK_LONGOFFSET(flags)) | ((PK_BITNUM(flags) & 7) << 13L))
}


