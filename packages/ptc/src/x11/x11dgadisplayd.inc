Type
  TX11DGADisplay = Class(TX11Display)
  Private
    Function NextEvent(Var event : TPTCEvent; wait : Boolean; Const EventMask : TPTCEventMask) : Boolean; Override;
    Function PeekEvent(wait : Boolean; Const EventMask : TPTCEventMask) : TPTCEvent; Override;
    
    Procedure HandleEvents;
    
    modeinfo : PPXF86VidModeModeInfo;
    num_modeinfo : Integer;
    previousmode : Integer;
    
    dga_addr : PByte;
    dga_linewidth : Integer;
    dga_banksize : Integer;
    dga_memsize : Integer;
    dga_width, dga_height : Integer;
    
    { Coordinates of upper left frame corner }
    m_destx, m_desty : Integer;
    
    m_indirect, m_inmode : Boolean;
  Public
    Constructor Create;
    Destructor Destroy; Override;
    
    Procedure open(title : String; _width, _height : Integer; Const _format : TPTCFormat; disp : PDisplay; screen : Integer); Override;
    Procedure open(disp : PDisplay; screen : Integer; w : TWindow; Const _format : TPTCFormat); Override;
    Procedure open(disp : PDisplay; screen : Integer; _window : TWindow; Const _format : TPTCFormat; x, y, w, h : Integer); Override;
    Procedure close; Override;
    Procedure update; Override;
    Procedure update(Const _area : TPTCArea); Override;
    Function lock : Pointer; Override;
    Procedure unlock; Override;
    Procedure palette(Const _palette : TPTCPalette); Override;
    Function pitch : Integer; Override;
    Function getX11Window : TWindow; Override;
    Function isFullScreen : Boolean; Override;
    Procedure SetCursor(visible : Boolean); Override;
  End;
