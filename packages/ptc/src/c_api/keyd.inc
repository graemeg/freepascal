{ setup }
Function ptc_key_create(code : Integer; alt, shift, control : Boolean) : TPTC_KEY;
Procedure ptc_key_destroy(obj : TPTC_KEY);

{ key code }
Function ptc_key_code(obj : TPTC_KEY) : Integer;

{ modifiers }
Function ptc_key_alt(obj : TPTC_KEY) : Boolean;
Function ptc_key_shift(obj : TPTC_KEY) : Boolean;
Function ptc_key_control(obj : TPTC_KEY) : Boolean;

{ operators }
Procedure ptc_key_assign(obj : TPTC_KEY; key : TPTC_KEY);
Function ptc_key_equals(obj : TPTC_KEY; key : TPTC_KEY) : Boolean;

{ key codes }
{#define PTC_KEY_ENTER            '\n'
#define PTC_KEY_BACKSPACE        '\b'
#define PTC_KEY_TAB              '\t'
#define PTC_KEY_CANCEL           0x03
#define PTC_KEY_CLEAR            0x0C
#define PTC_KEY_SHIFT            0x10
#define PTC_KEY_CONTROL          0x11
#define PTC_KEY_ALT              0x12
#define PTC_KEY_PAUSE            0x13
#define PTC_KEY_CAPSLOCK         0x14
#define PTC_KEY_ESCAPE           0x1B
#define PTC_KEY_SPACE            0x20
#define PTC_KEY_PAGEUP           0x21
#define PTC_KEY_PAGEDOWN         0x22
#define PTC_KEY_END              0x23
#define PTC_KEY_HOME             0x24
#define PTC_KEY_LEFT             0x25
#define PTC_KEY_UP               0x26
#define PTC_KEY_RIGHT            0x27
#define PTC_KEY_DOWN             0x28
#define PTC_KEY_COMMA            0x2C
#define PTC_KEY_PERIOD           0x2E
#define PTC_KEY_SLASH            0x2F
#define PTC_KEY_ZERO             0x30
#define PTC_KEY_ONE              0x31
#define PTC_KEY_TWO              0x32
#define PTC_KEY_THREE            0x33
#define PTC_KEY_FOUR             0x34
#define PTC_KEY_FIVE             0x35
#define PTC_KEY_SIX              0x36
#define PTC_KEY_SEVEN            0x37
#define PTC_KEY_EIGHT            0x38
#define PTC_KEY_NINE             0x39
#define PTC_KEY_SEMICOLON        0x3B
#define PTC_KEY_EQUALS           0x3D
#define PTC_KEY_A                0x41
#define PTC_KEY_B                0x42
#define PTC_KEY_C                0x43
#define PTC_KEY_D                0x44
#define PTC_KEY_E                0x45
#define PTC_KEY_F                0x46
#define PTC_KEY_G                0x47
#define PTC_KEY_H                0x48
#define PTC_KEY_I                0x49
#define PTC_KEY_J                0x4A
#define PTC_KEY_K                0x4B
#define PTC_KEY_L                0x4C
#define PTC_KEY_M                0x4D
#define PTC_KEY_N                0x4E
#define PTC_KEY_O                0x4F
#define PTC_KEY_P                0x50
#define PTC_KEY_Q                0x51
#define PTC_KEY_R                0x52
#define PTC_KEY_S                0x53
#define PTC_KEY_T                0x54
#define PTC_KEY_U                0x55
#define PTC_KEY_V                0x56
#define PTC_KEY_W                0x57
#define PTC_KEY_X                0x58
#define PTC_KEY_Y                0x59
#define PTC_KEY_Z                0x5A
#define PTC_KEY_OPENBRACKET      0x5B
#define PTC_KEY_BACKSLASH        0x5C
#define PTC_KEY_CLOSEBRACKET     0x5D
#define PTC_KEY_NUMPAD0          0x60
#define PTC_KEY_NUMPAD1          0x61
#define PTC_KEY_NUMPAD2          0x62
#define PTC_KEY_NUMPAD3          0x63
#define PTC_KEY_NUMPAD4          0x64
#define PTC_KEY_NUMPAD5          0x65
#define PTC_KEY_NUMPAD6          0x66
#define PTC_KEY_NUMPAD7          0x67
#define PTC_KEY_NUMPAD8          0x68
#define PTC_KEY_NUMPAD9          0x69
#define PTC_KEY_MULTIPLY         0x6A
#define PTC_KEY_ADD              0x6B
#define PTC_KEY_SEPARATOR        0x6C
#define PTC_KEY_SUBTRACT         0x6D
#define PTC_KEY_DECIMAL          0x6E
#define PTC_KEY_DIVIDE           0x6F
#define PTC_KEY_F1               0x70
#define PTC_KEY_F2               0x71
#define PTC_KEY_F3               0x72
#define PTC_KEY_F4               0x73
#define PTC_KEY_F5               0x74
#define PTC_KEY_F6               0x75
#define PTC_KEY_F7               0x76
#define PTC_KEY_F8               0x77
#define PTC_KEY_F9               0x78
#define PTC_KEY_F10              0x79
#define PTC_KEY_F11              0x7A
#define PTC_KEY_F12              0x7B
#define PTC_KEY_DELETE           0x7F
#define PTC_KEY_NUMLOCK          0x90
#define PTC_KEY_SCROLLLOCK       0x91
#define PTC_KEY_PRINTSCREEN      0x9A
#define PTC_KEY_INSERT           0x9B
#define PTC_KEY_HELP             0x9C
#define PTC_KEY_META             0x9D
#define PTC_KEY_BACKQUOTE        0xC0
#define PTC_KEY_QUOTE            0xDE
#define PTC_KEY_FINAL            0x18
#define PTC_KEY_CONVERT          0x1C
#define PTC_KEY_NONCONVERT       0x1D
#define PTC_KEY_ACCEPT           0x1E
#define PTC_KEY_MODECHANGE       0x1F
#define PTC_KEY_KANA             0x15
#define PTC_KEY_KANJI            0x19
#define PTC_KEY_UNDEFINED        0x0}
