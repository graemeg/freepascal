Type
  VGAConsole = Class(TPTCBaseConsole)
  Private
    { internal console management routines }
    Procedure internal_pre_open_setup(Const _title : String);
    Procedure internal_open_fullscreen_start;
    Procedure internal_open_fullscreen(ModeType : Integer);
    Procedure internal_open_fullscreen_finish(_pages : Integer);
    Procedure internal_post_open_setup;
    Procedure internal_reset;
    Procedure internal_close;
    Procedure internal_clear_mode_list;
    Procedure internal_SetPalette(data : Pint32);

    Procedure vga_load(data : Pointer);

    { console debug checks }
    Procedure check_open;
    Procedure check_unlocked;

    { data }
    m_modes : Array[0..31{255}] Of TPTCMode;
{    m_modes : PPTCMode;}
{    m_modes_last : Integer;
    m_modes_n : PInteger;}
    m_title : Array[0..1023] Of Char;
    m_information : Array[0..1023] Of Char;
    m_CurrentMode : Integer;
{    m_VESACurrentMode : Integer;}
    m_faketype : Integer;
    m_width, m_height, m_pitch, m_pages : Integer;
    m_primary : Pointer;

    { flags }
    m_open : Boolean;
    m_locked : Boolean;

    { option data }
    m_default_width : Integer;
    m_default_height : Integer;
    m_default_pages : Integer;
    m_default_format : TPTCFormat;

    { objects }
    m_copy : TPTCCopy;
    m_area : TPTCArea;
    m_clip : TPTCArea;
    m_format : TPTCFormat;

    m_clear : TPTCClear;
    m_palette : TPTCPalette;

    { Dos objects }
    m_keyboard : TDosKeyboard;
{    m_primary : TPTCSurface;}
{    DosKeyboard *m_keyboard;}
{    m_160x100buffer : TPTCSurface;}
  Protected
    Procedure internal_ReadKey(k : TPTCKey); Override;
    Function internal_PeekKey(k : TPTCKey) : Boolean; Override;
  Public
    Constructor Create;
    Destructor Destroy; Override;
    Procedure configure(Const _file : String); Override;
    Function option(Const _option : String) : Boolean; Override;
    Function modes : PPTCMode; Override;
    Procedure open(Const _title : String; _pages : Integer); Overload; Override;
    Procedure open(Const _title : String; Const _format : TPTCFormat;
                   _pages : Integer); Overload; Override;
    Procedure open(Const _title : String; _width, _height : Integer;
                   Const _format : TPTCFormat; _pages : Integer); Overload; Override;
    Procedure open(Const _title : String; Const _mode : TPTCMode;
                   _pages : Integer); Overload; Override;
    Procedure close; Override;
    Procedure flush; Override;
    Procedure finish; Override;
    Procedure update; Override;
    Procedure update(Const _area : TPTCArea); Override;
    Procedure copy(Var surface : TPTCBaseSurface); Override;
    Procedure copy(Var surface : TPTCBaseSurface;
                   Const source, destination : TPTCArea); Override;
    Function lock : Pointer; Override;
    Procedure unlock; Override;
    Procedure load(Const pixels : Pointer;
                   _width, _height, _pitch : Integer;
                   Const _format : TPTCFormat;
                   Const _palette : TPTCPalette); Override;
    Procedure load(Const pixels : Pointer;
                   _width, _height, _pitch : Integer;
                   Const _format : TPTCFormat;
                   Const _palette : TPTCPalette;
                   Const source, destination : TPTCArea); Override;
    Procedure save(pixels : Pointer;
                   _width, _height, _pitch : Integer;
                   Const _format : TPTCFormat;
                   Const _palette : TPTCPalette); Override;
    Procedure save(pixels : Pointer;
                   _width, _height, _pitch : Integer;
                   Const _format : TPTCFormat;
                   Const _palette : TPTCPalette;
                   Const source, destination : TPTCArea); Override;
    Procedure clear; Override;
    Procedure clear(Const color : TPTCColor); Override;
    Procedure clear(Const color : TPTCColor;
                    Const _area : TPTCArea); Override;
    Procedure palette(Const _palette : TPTCPalette); Override;
    Function palette : TPTCPalette; Override;
    Procedure clip(Const _area : TPTCArea); Override;
    Function width : Integer; Override;
    Function height : Integer; Override;
    Function pitch : Integer; Override;
    Function pages : Integer; Override;
    Function area : TPTCArea; Override;
    Function clip : TPTCArea; Override;
    Function format : TPTCFormat; Override;
    Function name : String; Override;
    Function title : String; Override;
    Function information : String; Override;
  End;
