{
    Free Pascal port of the OpenPTC C++ library.
    Copyright (C) 2001-2003  Nikolay Nikolov (nickysn@users.sourceforge.net)
    Original C++ version by Glenn Fiedler (ptc@gaffer.org)

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version
    with the following modification:

    As a special exception, the copyright holders of this library give you
    permission to link this library with independent modules to produce an
    executable, regardless of the license terms of these independent modules,and
    to copy and distribute the resulting executable under terms of your choice,
    provided that you also meet, for each linked independent module, the terms
    and conditions of the license of that module. An independent module is a
    module which is not derived from or based on this library. If you modify
    this library, you may extend this exception to your version of the library,
    but you are not obligated to do so. If you do not wish to do so, delete this
    exception statement from your version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
}

type
  TPTCKeyEvent = class(TPTCEvent)
  private
    FCode: Integer;
    FUnicode: Integer;
    FAlt: Boolean;
    FShift: Boolean;
    FControl: Boolean;
    FPress: Boolean;

    function GetRelease: Boolean;
  protected
    function GetType: TPTCEventType; override;
  public
    constructor Create;
    constructor Create(ACode: Integer);
    constructor Create(ACode, AUnicode: Integer);
    constructor Create(ACode, AUnicode: Integer; APress: Boolean);
    constructor Create(ACode: Integer; AAlt, AShift, AControl: Boolean);
    constructor Create(ACode: Integer; AAlt, AShift, AControl, APress: Boolean);
    constructor Create(ACode, AUnicode: Integer;
                       AAlt, AShift, AControl: Boolean);
    constructor Create(ACode, AUnicode: Integer;
                       AAlt, AShift, AControl, APress: Boolean);
    constructor Create(const AKey: TPTCKeyEvent);
    procedure Assign(const AKey: TPTCKeyEvent);
    function Equals(const AKey: TPTCKeyEvent): Boolean;
    property Code: Integer read FCode;
    property Unicode: Integer read FUnicode;
    property Alt: Boolean read FAlt;
    property Shift: Boolean read FShift;
    property Control: Boolean read FControl;
    property Press: Boolean read FPress;
    property Release: Boolean read GetRelease;
  end;

const
  PTCKEY_UNDEFINED    = $00;
  PTCKEY_CANCEL       = $03;
  PTCKEY_BACKSPACE    = $08; {'\b'}
  PTCKEY_TAB          = $09; {'\t'}
  PTCKEY_ENTER        = $0A; {'\n'}
  PTCKEY_CLEAR        = $0C;
  PTCKEY_SHIFT        = $10;
  PTCKEY_CONTROL      = $11;
  PTCKEY_ALT          = $12;
  PTCKEY_PAUSE        = $13;
  PTCKEY_CAPSLOCK     = $14;
  PTCKEY_KANA         = $15;
  PTCKEY_FINAL        = $18;
  PTCKEY_KANJI        = $19;
  PTCKEY_ESCAPE       = $1B;
  PTCKEY_CONVERT      = $1C;
  PTCKEY_NONCONVERT   = $1D;
  PTCKEY_ACCEPT       = $1E;
  PTCKEY_MODECHANGE   = $1F;
  PTCKEY_SPACE        = $20;
  PTCKEY_PAGEUP       = $21;
  PTCKEY_PAGEDOWN     = $22;
  PTCKEY_END          = $23;
  PTCKEY_HOME         = $24;
  PTCKEY_LEFT         = $25;
  PTCKEY_UP           = $26;
  PTCKEY_RIGHT        = $27;
  PTCKEY_DOWN         = $28;
  PTCKEY_COMMA        = $2C; {','}
  PTCKEY_PERIOD       = $2E; {'.'}
  PTCKEY_SLASH        = $2F; {'/'}
  PTCKEY_ZERO         = $30;
  PTCKEY_ONE          = $31;
  PTCKEY_TWO          = $32;
  PTCKEY_THREE        = $33;
  PTCKEY_FOUR         = $34;
  PTCKEY_FIVE         = $35;
  PTCKEY_SIX          = $36;
  PTCKEY_SEVEN        = $37;
  PTCKEY_EIGHT        = $38;
  PTCKEY_NINE         = $39;
  PTCKEY_SEMICOLON    = $3B; {';'}
  PTCKEY_EQUALS       = $3D; {'='}
  PTCKEY_A            = $41;
  PTCKEY_B            = $42;
  PTCKEY_C            = $43;
  PTCKEY_D            = $44;
  PTCKEY_E            = $45;
  PTCKEY_F            = $46;
  PTCKEY_G            = $47;
  PTCKEY_H            = $48;
  PTCKEY_I            = $49;
  PTCKEY_J            = $4A;
  PTCKEY_K            = $4B;
  PTCKEY_L            = $4C;
  PTCKEY_M            = $4D;
  PTCKEY_N            = $4E;
  PTCKEY_O            = $4F;
  PTCKEY_P            = $50;
  PTCKEY_Q            = $51;
  PTCKEY_R            = $52;
  PTCKEY_S            = $53;
  PTCKEY_T            = $54;
  PTCKEY_U            = $55;
  PTCKEY_V            = $56;
  PTCKEY_W            = $57;
  PTCKEY_X            = $58;
  PTCKEY_Y            = $59;
  PTCKEY_Z            = $5A;
  PTCKEY_OPENBRACKET  = $5B; {'['}
  PTCKEY_BACKSLASH    = $5C; {'\'}
  PTCKEY_CLOSEBRACKET = $5D; {']'}
  PTCKEY_NUMPAD0      = $60;
  PTCKEY_NUMPAD1      = $61;
  PTCKEY_NUMPAD2      = $62;
  PTCKEY_NUMPAD3      = $63;
  PTCKEY_NUMPAD4      = $64;
  PTCKEY_NUMPAD5      = $65;
  PTCKEY_NUMPAD6      = $66;
  PTCKEY_NUMPAD7      = $67;
  PTCKEY_NUMPAD8      = $68;
  PTCKEY_NUMPAD9      = $69;
  PTCKEY_MULTIPLY     = $6A; {numpad '*'}
  PTCKEY_ADD          = $6B; {numpad '+'}
  PTCKEY_SEPARATOR    = $6C;
  PTCKEY_SUBTRACT     = $6D; {numpad '-'}
  PTCKEY_DECIMAL      = $6E; {numpad '.'}
  PTCKEY_DIVIDE       = $6F; {numpad '/'}
  PTCKEY_F1           = $70;
  PTCKEY_F2           = $71;
  PTCKEY_F3           = $72;
  PTCKEY_F4           = $73;
  PTCKEY_F5           = $74;
  PTCKEY_F6           = $75;
  PTCKEY_F7           = $76;
  PTCKEY_F8           = $77;
  PTCKEY_F9           = $78;
  PTCKEY_F10          = $79;
  PTCKEY_F11          = $7A;
  PTCKEY_F12          = $7B;
  PTCKEY_DELETE       = $7F;
  PTCKEY_NUMLOCK      = $90;
  PTCKEY_SCROLLLOCK   = $91;
  PTCKEY_PRINTSCREEN  = $9A;
  PTCKEY_INSERT       = $9B;
  PTCKEY_HELP         = $9C;
  PTCKEY_META         = $9D;
  PTCKEY_BACKQUOTE    = $C0;
  PTCKEY_QUOTE        = $DE;
