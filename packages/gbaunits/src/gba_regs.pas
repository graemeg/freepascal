(*
  gba_regs.pas  18/06/2006 4.22.49
  ------------------------------------------------------------------------------
  This lib is a raw porting of libgba library for gba (you can find it at
  http://www.devkitpro.org).
  
  As this is a direct port from c, I'm pretty sure that something could not work
  as you expect. I am even more sure that this code could be written better, so 
  if you think that I have made some mistakes or you have some better 
  implemented functions, let me know [francky74 (at) gmail (dot) com]
  Enjoy!

  Conversion by Legolas (http://itaprogaming.free.fr) for freepascal compiler
  (http://www.freepascal.org)
  
  Copyright (C) 2006  Francesco Lombardi
  
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
  
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
  ------------------------------------------------------------------------------
*)

unit gba_regs;
{$i def.inc}
interface

uses
  gba_types;

const
  OAMMem          : ^word = pointer($7000000); // Sprites(128), coordinates, size..(total 1Kb)
  VideoBuffer     : ^word = pointer($6000000); // Front Display Memory (the screen in mode 3-5)
  SecondBuffer    : ^word = pointer($600A000); // Back Display Memory
  
  OAMData         : ^word = pointer($6010000); // Sprite data (bitmapped)
  OAMdataTile     : ^word = pointer($6010000);
  OAMdataBmp      : ^word = pointer($6014000);

  BG_COLORS       : ^word = pointer($5000000); // Background Palette(256/16 colors)
  OBJ_COLORS      : ^word = pointer($5000200); // Sprite Palette(256/16 colors)

  REG_INTERRUPT   : ^dword = pointer($3007FFC);
  REG_DISPCNT     : ^dword = pointer($4000000);  // Display control mode
  REG_DISPCNT_L   : ^word = pointer($4000000);
  REG_DISPCNT_H   : ^word = pointer($4000002);
  REG_DISPSTAT    : ^word = pointer($4000004);
{$OPTIMIZATION OFF}
  REG_VCOUNT      : ^word = pointer($4000006);  // Vertical control sync
{$OPTIMIZATION ON}
  BGCTRL          : ^word = pointer($4000008);
  REG_BG0CNT      : ^word = pointer($4000008);
  REG_BG1CNT      : ^word = pointer($400000A);
  REG_BG2CNT      : ^word = pointer($400000C);
  REG_BG3CNT      : ^word = pointer($400000E);
  
  BG_OFFSET       : PBgScroll = pointer($4000010);
  
  REG_BG0HOFS     : ^word = pointer($4000010);
  REG_BG0VOFS     : ^word = pointer($4000012);
  REG_BG1HOFS     : ^word = pointer($4000014);
  REG_BG1VOFS     : ^word = pointer($4000016);
  REG_BG2HOFS     : ^word = pointer($4000018);
  REG_BG2VOFS     : ^word = pointer($400001A);
  REG_BG3HOFS     : ^word = pointer($400001C);
  REG_BG3VOFS     : ^word = pointer($400001E);

  REG_BG2PA       : ^word = pointer($4000020);
  REG_BG2PB       : ^word = pointer($4000022);
  REG_BG2PC       : ^word = pointer($4000024);
  REG_BG2PD       : ^word = pointer($4000026);
  REG_BG2X        : ^dword = pointer($4000028);
  REG_BG2X_L      : ^word = pointer($4000028);
  REG_BG2X_H      : ^word = pointer($400002A);
  REG_BG2Y        : ^dword = pointer($400002C);
  REG_BG2Y_L      : ^word = pointer($400002C);
  REG_BG2Y_H      : ^word = pointer($400002E);
  REG_BG3PA       : ^word = pointer($4000030);
  REG_BG3PB       : ^word = pointer($4000032);
  REG_BG3PC       : ^word = pointer($4000034);
  REG_BG3PD       : ^word = pointer($4000036);
  REG_BG3X        : ^dword = pointer($4000038);
  REG_BG3X_L      : ^word = pointer($4000038);
  REG_BG3X_H      : ^word = pointer($400003A);
  REG_BG3Y        : ^dword = pointer($400003C);
  REG_BG3Y_L      : ^word = pointer($400003C);
  REG_BG3Y_H      : ^word = pointer($400003E);
  REG_WIN0H       : ^word = pointer($4000040);
  REG_WIN1H       : ^word = pointer($4000042);
  REG_WIN0V       : ^word = pointer($4000044);
  REG_WIN1V       : ^word = pointer($4000046);
  REG_WININ       : ^word = pointer($4000048);
  REG_WINOUT      : ^word = pointer($400004A);
  
  REG_MOSAIC      : ^dword = pointer($400004C);
  REG_MOSAIC_L    : ^dword = pointer($400004C);
  REG_MOSAIC_H    : ^dword = pointer($400004E);

  REG_BLDMOD      : ^word = pointer($4000050);
  REG_BLDCNT      : ^word = pointer($4000050);
  REG_COLEV       : ^word = pointer($4000052);
  REG_BLDALPHA    : ^word = pointer($4000052);
  REG_COLEY       : ^word = pointer($4000054);
  REG_BLDY        : ^word = pointer($4000054);
  
{$OPTIMIZATION OFF}
  REG_SG10        : ^dword = pointer($4000060);
  REG_SG10_L      : ^word = pointer($4000060);
  REG_SG10_H      : ^word = pointer($4000062);
  REG_SG11        : ^word = pointer($4000064);
  REG_SG20        : ^word = pointer($4000068);
  REG_SG21        : ^word = pointer($400006C);
  REG_SG30        : ^dword = pointer($4000070);
  REG_SG30_L      : ^word = pointer($4000070);
  REG_SG30_H      : ^word = pointer($4000072);
  REG_SG31        : ^word = pointer($4000074);
  REG_SG40        : ^word = pointer($4000078);
  REG_SG41        : ^word = pointer($400007C);
  REG_SGCNT0      : ^dword = pointer($4000080);
  REG_SGCNT0_L    : ^word = pointer($4000080);
  REG_SGCNT0_H    : ^word = pointer($4000082);
  REG_SGCNT1      : ^word = pointer($4000084);
  REG_SGBIAS      : ^word = pointer($4000088);
  REG_SGWR0       : ^dword = pointer($4000090);
  REG_SGWR0_L     : ^word = pointer($4000090);
  REG_SGWR0_H     : ^word = pointer($4000092);
  REG_SGWR1       : ^dword = pointer($4000094);
  REG_SGWR1_L     : ^word = pointer($4000094);
  REG_SGWR1_H     : ^word = pointer($4000096);
{$OPTIMIZATION ON}
  REG_SGWR2       : ^dword = pointer($4000098);
{$OPTIMIZATION OFF}
  REG_SGWR2_L     : ^word = pointer($4000098);
  REG_SGWR2_H     : ^word = pointer($400009A);
  REG_SGWR3       : ^dword = pointer($400009C);
  REG_SGWR3_L     : ^word = pointer($400009C);
  REG_SGWR3_H     : ^word = pointer($400009E);
  REG_SGFIF0A     : ^dword = pointer($40000A0);
  REG_SGFIFOA_L   : ^word = pointer($40000A0);
  REG_SGFIFOA_H   : ^word = pointer($40000A2);
  REG_SGFIFOB     : ^dword = pointer($40000A4);
  REG_SGFIFOB_L   : ^word = pointer($40000A4);
  REG_SGFIFOB_H   : ^word = pointer($40000A6);
{$OPTIMIZATION ON}

  REG_DM0SAD      : ^dword = pointer($40000B0);
  REG_DM0SAD_L    : ^word = pointer($40000B0);
  REG_DM0SAD_H    : ^word = pointer($40000B2);
  REG_DM0DAD      : ^dword = pointer($40000B4);
  REG_DM0DAD_L    : ^word = pointer($40000B4);
  REG_DM0DAD_H    : ^word = pointer($40000B6);
  REG_DM0CNT      : ^dword = pointer($40000B8);
  REG_DM0CNT_L    : ^word = pointer($40000B8);
  REG_DM0CNT_H    : ^word = pointer($40000BA);
  REG_DM1SAD      : ^dword = pointer($40000BC);
  REG_DM1SAD_L    : ^word = pointer($40000BC);
  REG_DM1SAD_H    : ^word = pointer($40000BE);
  REG_DM1DAD      : ^dword = pointer($40000C0);
  REG_DM1DAD_L    : ^word = pointer($40000C0);
  REG_DM1DAD_H    : ^word = pointer($40000C2);
  REG_DM1CNT      : ^dword = pointer($40000C4);
  REG_DM1CNT_L    : ^word = pointer($40000C4);
  REG_DM1CNT_H    : ^word = pointer($40000C6);
  REG_DM2SAD      : ^dword = pointer($40000C8);
  REG_DM2SAD_L    : ^word = pointer($40000C8);
  REG_DM2SAD_H    : ^word = pointer($40000CA);
  REG_DM2DAD      : ^dword = pointer($40000CC);
  REG_DM2DAD_L    : ^word = pointer($40000CC);
  REG_DM2DAD_H    : ^word = pointer($40000CE);
  REG_DM2CNT      : ^dword = pointer($40000D0);
  REG_DM2CNT_L    : ^word = pointer($40000D0);
  REG_DM2CNT_H    : ^word = pointer($40000D2);
  REG_DM3SAD      : ^dword = pointer($40000D4);
  REG_DM3SAD_L    : ^word = pointer($40000D4);
  REG_DM3SAD_H    : ^word = pointer($40000D6);
  REG_DM3DAD      : ^dword = pointer($40000D8);
  REG_DM3DAD_L    : ^word = pointer($40000D8);
  REG_DM3DAD_H    : ^word = pointer($40000DA);
  REG_DM3CNT      : ^dword = pointer($40000DC);
  REG_DM3CNT_L    : ^word = pointer($40000DC);
  REG_DM3CNT_H    : ^word = pointer($40000DE);
 
{$OPTIMIZATION OFF} 
  REG_TM0CNT_L    : ^word = pointer($4000100);
  REG_TM0CNT_H	  : ^word = pointer($4000102);
  REG_TM1CNT_L	  : ^word = pointer($4000104);
  REG_TM1CNT_H	  : ^word = pointer($4000106);
  REG_TM2CNT_L	  : ^word = pointer($4000108);
  REG_TM2CNT_H	  : ^word = pointer($400010A);
  REG_TM3CNT_L	  : ^word = pointer($400010C);
  REG_TM3CNT_H	  : ^word = pointer($400010E);
{$OPTIMIZATION ON}

{$OPTIMIZATION OFF}
  REG_SCD0        : ^word = pointer($4000120);
  REG_SCD1        : ^word = pointer($4000122);
  REG_SCD2        : ^word = pointer($4000124);
  REG_SCD3        : ^word = pointer($4000126);
 
  REG_SCCNT       : ^dword = pointer($4000128);
  REG_SCCNT_L     : ^word = pointer($4000128);
  REG_SIOCNT      : ^word = pointer($4000128);
  REG_SCCNT_H     : ^word = pointer($400012A);
  REG_SIODATA8    : ^word = pointer($400012A);
  REG_SIODATA32   : ^dword = pointer($4000120);
  REG_SIOMLT_SEND : ^word = pointer($400012A);
  REG_SIOMLT_RECV : ^word = pointer($4000120);
  REG_SIOMULTI0   : ^word = pointer($4000120);
  REG_SIOMULTI1   : ^word = pointer($4000122);
  REG_SIOMULTI2   : ^word = pointer($4000124);
  REG_SIOMULTI3   : ^word = pointer($4000126);

  REG_RCNT        : ^word = pointer($4000134);

  REG_P1          : ^word = pointer($4000130);
  REG_KEYINPUT    : ^word = pointer($4000130);
  REG_P1CNT       : ^word = pointer($4000132);
  REG_KEYCNT      : ^word = pointer($4000132);
  REG_R           : ^word = pointer($4000134);
  
{$OPTIMIZATION ON}


  REG_HS_CTRL     : ^word = pointer($4000140);
  REG_JOYRE       : ^dword = pointer($4000150);
  REG_JOYRE_L     : ^word = pointer($4000150);
  REG_JOYRE_H     : ^word = pointer($4000152);
  REG_JOYTR       : ^dword = pointer($4000154);
  REG_JOYTR_L     : ^word = pointer($4000154);
  REG_JOYTR_H     : ^word = pointer($4000156);
  REG_JSTAT       : ^dword = pointer($4000158);
  REG_JSTAT_L     : ^word = pointer($4000158);
  REG_JSTAT_H     : ^word = pointer($400015A);
  REG_IE          : ^word = pointer($4000200);
  REG_IF          : ^word = pointer($4000202);
  REG_WSCNT       : ^word = pointer($4000204);
  REG_IME         : ^word = pointer($4000208);
  REG_PAUSE       : ^word = pointer($4000300);

  BIT0  = 1;
  BIT1  = 2;
  BIT2  = 4;
  BIT3  = 8;
  BIT4  = 16;
  BIT5  = 32;
  BIT6  = 64;
  BIT7  = 128;
  BIT8  = 256;
  BIT9  = 512;
  BIT10 = 1024;
  BIT11 = 2048;
  BIT12 = 4096;
  BIT13 = 8192;
  BIT14 = 16384;
  BIT15 = 32768;

implementation

end.
