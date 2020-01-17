{
    This file is part of the Free Pascal run time library.

    A file in Amiga system run time library.
    Copyright (c) 1998-2003 by Nils Sjoholm
    member of the Amiga RTL development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{
    To call the two routines defined below, you'll need to set
    ConsoleBase to an appropriate value.

    nils.sjoholm@mailbox.swipnet.se  Nils Sjoholm
}

unit console;

interface

uses
  exec, inputevent, keymap;

const

{***** Console commands *****}
  CD_ASKKEYMAP        = CMD_NONSTD + 0;
  CD_SETKEYMAP        = CMD_NONSTD + 1;
  CD_ASKDEFAULTKEYMAP = CMD_NONSTD + 2;
  CD_SETDEFAULTKEYMAP = CMD_NONSTD + 3;

{***** SGR parameters *****}

  SGR_PRIMARY    = 0;
  SGR_BOLD       = 1;
  SGR_ITALIC     = 3;
  SGR_UNDERSCORE = 4;
  SGR_NEGATIVE   = 7;

  SGR_NORMAL        = 22; // default foreground color, not bold
  SGR_NOTITALIC     = 23;
  SGR_NOTUNDERSCORE = 24;
  SGR_POSITIVE      = 27;

{ these names refer to the ANSI standard, not the implementation }

  SGR_BLACK   = 30;
  SGR_RED     = 31;
  SGR_GREEN   = 32;
  SGR_YELLOW  = 33;
  SGR_BLUE    = 34;
  SGR_MAGENTA = 35;
  SGR_CYAN    = 36;
  SGR_WHITE   = 37;
  SGR_DEFAULT = 39;

  SGR_BLACKBG   = 40;
  SGR_REDBG     = 41;
  SGR_GREENBG   = 42;
  SGR_YELLOWBG  = 43;
  SGR_BLUEBG    = 44;
  SGR_MAGENTABG = 45;
  SGR_CYANBG    = 46;
  SGR_WHITEBG   = 47;
  SGR_DEFAULTBG = 49;

{ these names refer to the implementation, they are the preferred   }
{ names for use with the Amiga console device. }

  SGR_CLR0 = 30;
  SGR_CLR1 = 31;
  SGR_CLR2 = 32;
  SGR_CLR3 = 33;
  SGR_CLR4 = 34;
  SGR_CLR5 = 35;
  SGR_CLR6 = 36;
  SGR_CLR7 = 37;

  SGR_CLR0BG = 40;
  SGR_CLR1BG = 41;
  SGR_CLR2BG = 42;
  SGR_CLR3BG = 43;
  SGR_CLR4BG = 44;
  SGR_CLR5BG = 45;
  SGR_CLR6BG = 46;
  SGR_CLR7BG = 47;

{***** DSR parameters *****}
  DSR_CPR = 6;

{***** CTC parameters *****}
  CTC_HSETTAB     = 0;
  CTC_HCLRTAB     = 2;
  CTC_HCLRTABSALL = 5;

{*****   TBC parameters *****}
  TBC_HCLRTAB     = 0;
  TBC_HCLRTABSALL = 3;

{*****   SM and RM parameters *****}
  M_LNM = 20;   // linefeed newline mode
  M_ASM = '>1'; // auto scroll mode
  M_AWM = '?7'; // auto wrap mode

var
  ConsoleDevice: PDevice = nil;

function CDInputHandler(Events: PInputEvent location 'a0'; ConsoleDev: PLibrary location 'a1'): PInputEvent; syscall ConsoleDevice 42;
function RawKeyConvert(Events: PInputEvent location 'a0'; Buffer: PChar location 'a1'; Length: LongInt location 'd1'; KeyMap: PKeyMap location 'a2'): LongInt; syscall ConsoleDevice 48;

implementation

end.
