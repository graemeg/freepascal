{$IFDEF OGC_INTERFACE}
type
  ir_position_t = integer;
  data_req_s = integer;
  cmd_blk_s = integer;
  aspect_t = integer;
  win_bt_stack_t = integer;
  WIIUSE_EVENT_TYPE = integer;
  ubyte = cuchar;
  Pubyte = ^ubyte;
  sbyte = cchar;
  uword = cushort;
  sword = cshort;
  uint = cuint;
  sint = cchar;

const
  WIIMOTE_LED_NONE = $00;
  WIIMOTE_LED_1 = $10;
  WIIMOTE_LED_2 = $20;
  WIIMOTE_LED_3 = $40;
  WIIMOTE_LED_4 = $80;

  WIIMOTE_BUTTON_TWO = $0001;
  WIIMOTE_BUTTON_ONE = $0002;
  WIIMOTE_BUTTON_B = $0004;
  WIIMOTE_BUTTON_A = $0008;
  WIIMOTE_BUTTON_MINUS = $0010;
  WIIMOTE_BUTTON_ZACCEL_BIT6 = $0020;
  WIIMOTE_BUTTON_ZACCEL_BIT7 = $0040;
  WIIMOTE_BUTTON_HOME = $0080;
  WIIMOTE_BUTTON_LEFT = $0100;
  WIIMOTE_BUTTON_RIGHT = $0200;
  WIIMOTE_BUTTON_DOWN = $0400;
  WIIMOTE_BUTTON_UP = $0800;
  WIIMOTE_BUTTON_PLUS = $1000;
  WIIMOTE_BUTTON_ZACCEL_BIT4 = $2000;
  WIIMOTE_BUTTON_ZACCEL_BIT5 = $4000;
  WIIMOTE_BUTTON_UNKNOWN = $8000;
  WIIMOTE_BUTTON_ALL = $1F9F;

  NUNCHUK_BUTTON_Z = $01;
  NUNCHUK_BUTTON_C = $02;
  NUNCHUK_BUTTON_ALL = $03;

  CLASSIC_CTRL_BUTTON_UP = $0001;
  CLASSIC_CTRL_BUTTON_LEFT = $0002;
  CLASSIC_CTRL_BUTTON_ZR = $0004;
  CLASSIC_CTRL_BUTTON_X = $0008;
  CLASSIC_CTRL_BUTTON_A = $0010;
  CLASSIC_CTRL_BUTTON_Y = $0020;
  CLASSIC_CTRL_BUTTON_B = $0040;
  CLASSIC_CTRL_BUTTON_ZL = $0080;
  CLASSIC_CTRL_BUTTON_FULL_R = $0200;
  CLASSIC_CTRL_BUTTON_PLUS = $0400;
  CLASSIC_CTRL_BUTTON_HOME = $0800;
  CLASSIC_CTRL_BUTTON_MINUS = $1000;
  CLASSIC_CTRL_BUTTON_FULL_L = $2000;
  CLASSIC_CTRL_BUTTON_DOWN = $4000;
  CLASSIC_CTRL_BUTTON_RIGHT = $8000;
  CLASSIC_CTRL_BUTTON_ALL = $FEFF;

  GUITAR_HERO_3_BUTTON_STRUM_UP = $0001;
  GUITAR_HERO_3_BUTTON_YELLOW = $0008;
  GUITAR_HERO_3_BUTTON_GREEN = $0010;
  GUITAR_HERO_3_BUTTON_BLUE = $0020;
  GUITAR_HERO_3_BUTTON_RED = $0040;
  GUITAR_HERO_3_BUTTON_ORANGE = $0080;
  GUITAR_HERO_3_BUTTON_PLUS = $0400;
  GUITAR_HERO_3_BUTTON_MINUS = $1000;
  GUITAR_HERO_3_BUTTON_STRUM_DOWN = $4000;
  GUITAR_HERO_3_BUTTON_ALL = $FEFF;

  GUITAR_HERO_3_TOUCH_AVAILABLE = $1000;
  GUITAR_HERO_3_TOUCH_GREEN = $1001;
  GUITAR_HERO_3_TOUCH_RED = $1002;
  GUITAR_HERO_3_TOUCH_YELLOW = $1004;
  GUITAR_HERO_3_TOUCH_BLUE = $1008;
  GUITAR_HERO_3_TOUCH_ORANGE = $1010;

  WIIUSE_SMOOTHING = $01;
  WIIUSE_CONTINUOUS = $02;
  WIIUSE_ACCEL_THRESH = $04;
  WIIUSE_IR_THRESH = $08;
  WIIUSE_JS_THRESH = $10;
  WIIUSE_INIT_FLAGS = WIIUSE_SMOOTHING;
  WIIUSE_ORIENT_PRECISION = 100.0;

  EXP_NONE = 0;
  EXP_NUNCHUK = 1;
  EXP_CLASSIC = 2;
  EXP_GUITAR_HERO_3 = 3;
  EXP_WII_BOARD = 4;
  EXP_MOTION_PLUS = 5;

  MAX_PAYLOAD = 32;

  WIIUSE_IR_ABOVE: ir_position_t = 0;
  WIIUSE_IR_BELOW: ir_position_t = 1;

  REQ_READY: data_req_s = 0;
  REQ_SENT: data_req_s = 1;
  REQ_DONE: data_req_s = 2;

  CMD_READY: cmd_blk_s = 0;
  CMD_SENT: cmd_blk_s = 1;
  CMD_DONE: cmd_blk_s = 2;

  WIIUSE_ASPECT_4_3: aspect_t = 0;
  WIIUSE_ASPECT_16_9: aspect_t = 1;


  WIIUSE_STACK_UNKNOWN: win_bt_stack_t = 0;
  WIIUSE_STACK_MS: win_bt_stack_t = 1;
  WIIUSE_STACK_BLUESOLEIL: win_bt_stack_t = 2;


  WIIUSE_NONE: WIIUSE_EVENT_TYPE = 0;
  WIIUSE_EVENT: WIIUSE_EVENT_TYPE = 1;
  _WIIUSE_STATUS: WIIUSE_EVENT_TYPE = 2;
  _WIIUSE_CONNECT: WIIUSE_EVENT_TYPE = 3;
  _WIIUSE_DISCONNECT: WIIUSE_EVENT_TYPE = 4;
  WIIUSE_UNEXPECTED_DISCONNECT: WIIUSE_EVENT_TYPE = 5;
  _WIIUSE_READ_DATA: WIIUSE_EVENT_TYPE = 6;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  WIIUSE_ACK: WIIUSE_EVENT_TYPE = 7;
=======
  _WIIUSE_WRITE_DATA: WIIUSE_EVENT_TYPE = 7;
>>>>>>> graemeg/cpstrnew
=======
  _WIIUSE_WRITE_DATA: WIIUSE_EVENT_TYPE = 7;
>>>>>>> graemeg/cpstrnew
=======
  _WIIUSE_WRITE_DATA: WIIUSE_EVENT_TYPE = 7;
>>>>>>> graemeg/cpstrnew
=======
  _WIIUSE_WRITE_DATA: WIIUSE_EVENT_TYPE = 7;
>>>>>>> origin/cpstrnew
  WIIUSE_NUNCHUK_INSERTED: WIIUSE_EVENT_TYPE = 8;
  WIIUSE_NUNCHUK_REMOVED: WIIUSE_EVENT_TYPE = 9;
  WIIUSE_CLASSIC_CTRL_INSERTED: WIIUSE_EVENT_TYPE = 10;
  WIIUSE_CLASSIC_CTRL_REMOVED: WIIUSE_EVENT_TYPE = 11;
  WIIUSE_GUITAR_HERO_3_CTRL_INSERTED: WIIUSE_EVENT_TYPE = 12;
  WIIUSE_GUITAR_HERO_3_CTRL_REMOVED: WIIUSE_EVENT_TYPE = 13;
  WIIUSE_WII_BOARD_INSERTED: WIIUSE_EVENT_TYPE = 14;
  WIIUSE_WII_BOARD_REMOVED: WIIUSE_EVENT_TYPE = 15;
  WIIUSE_MOTION_PLUS_ACTIVATED: WIIUSE_EVENT_TYPE = 16;
  WIIUSE_MOTION_PLUS_REMOVED: WIIUSE_EVENT_TYPE = 17;




//struct wiimote_t;
//struct vec3b_t;
//struct orient_t;
//struct gforce_t;
type
  Pwiimote_t = ^wiimote_t;

{ $ifdef GEKKO}
  wii_event_cb = procedure(par0: Pwiimote_t; event: cint); cdecl;
  pwii_event_cb = ^wii_event_cb;
{ $endif}

  wiiuse_data_cb = procedure(wm: Pwiimote_t; data: Pubyte; len: cushort); cdecl;

  Pdata_req_t = ^data_req_t;
  data_req_t = packed record
    node : lwp_node;
    data : array [0..47] of ubyte;  (**< buffer where read data is written						 *)
    len : cuint;
    state : data_req_s;  (**< set to 1 if not using callback and needs to be cleaned up	 *)
    cb : wiiuse_data_cb;  (**< read data callback											 *)
    next : Pdata_req_t;
  end;

  cmd_blk_cb = procedure(wm: Pwiimote_t; data: Pubyte; len: uword); cdecl;


  Pcmd_blk_t = ^cmd_blk_t;
  cmd_blk_t = packed record
    node : lwp_node;
    data : array [0..47] of ubyte;
    len : uint;
    state : cmd_blk_s;
    cb : cmd_blk_cb;
    next : Pcmd_blk_t;
  end;

  vec2b_t = packed record
    x : ubyte;
    y : ubyte;
  end;
  pvec2b_t = ^vec2b_t;

  vec3b_t = packed record
    x : ubyte;
    y : ubyte;
    z : ubyte;
  end;
  pvec3b_t = ^vec3b_t;


  vec3w_t = packed record
    x : uword;
    y : uword;
    z : uword;
  end;
  pvec3w_t = ^vec3w_t;

  vec3f_t = packed record
    x : cfloat;
    y : cfloat;
    z : cfloat;
  end;
  pvec3f_t = ^vec3f_t;

  orient_t = packed record
    roll : cfloat;  (**< roll, this may be smoothed if enabled	 *)
    pitch : cfloat;  (**< pitch, this may be smoothed if enabled	 *)
    yaw : cfloat;
    a_roll : cfloat;  (**< absolute roll, unsmoothed				 *)
    a_pitch : cfloat;  (**< absolute pitch, unsmoothed				 *)
  end;
  porient_t =^orient_t;

  gforce_t = packed record
    x : cfloat;
    y : cfloat;
    z : cfloat;
  end;
  pgforce_t = ^gforce_t;

  accel_t = packed record
    cal_zero : vec3w_t;  (**< zero calibration					 *)
    cal_g : vec3w_t;  (**< 1g difference around 0cal			 *)
    st_roll : cfloat;  (**< last smoothed roll value			 *)
    st_pitch : cfloat;  (**< last smoothed roll pitch			 *)
    st_alpha : cfloat;  (**< alpha value for smoothing [0-1]	 *)
  end;
  paccel_t = ^accel_t;

  ir_dot_t = packed record
    visible : ubyte;  (**< if the IR source is visible		 *)
    rx : cshort;  (**< raw X coordinate (0-1023)			 *)
    ry : cshort;  (**< raw Y coordinate (0-767)			 *)
    size : ubyte;  (**< size of the IR dot (0-15)			 *)
  end;
  pir_dot_t = ^ir_dot_t;

  fdot_t = packed record
    x : cfloat;
    y : cfloat;
  end;
  pfdot_t = ^fdot_t;

  sb_t = packed record
    dots : array [0..1] of fdot_t;
    acc_dots : array [0..1] of fdot_t;
    rot_dots : array [0..1] of fdot_t;
    angle : cfloat;
    off_angle : cfloat;
    score : cfloat;
  end;
  psb_t = ^sb_t;

  ir_t = packed record
    dot : array [0..3] of ir_dot_t;  (**< IR dots							 *)
    num_dots : ubyte;  (**< number of dots at this time		 *)
    state : cint;  (**< keeps track of the IR state		 *)
    raw_valid : cint;  (**< is the raw position valid? 		 *)
    sensorbar : sb_t;  (**< sensor bar, detected or guessed	 *)
    ax : cfloat;  (**< raw X coordinate					 *)
    ay : cfloat;  (**< raw Y coordinate					 *)
    distance : cfloat;  (**< pixel width of the sensor bar		 *)
    z : cfloat;  (**< calculated distance in meters		 *)
    angle : cfloat;  (**< angle of the wiimote to the sensor bar *)
    smooth_valid : cint;  (**< is the smoothed position valid? 	 *)
    sx : cfloat;  (**< smoothed X coordinate				 *)
    sy : cfloat;  (**< smoothed Y coordinate				 *)
    error_cnt : cfloat;  (**< error count, for smoothing algorithm *)
    glitch_cnt : cfloat;  (**< glitch count, same					 *)
    valid : cint;  (**< is the bounded position valid? 	 *)
    x : cfloat;  (**< bounded X coordinate				 *)
    y : cfloat;  (**< bounded Y coordinate				 *)
    aspect : aspect_t;  (**< aspect ratio of the screen			 *)
    pos : ir_position_t;  (**< IR sensor bar position				 *)
    vres : array [0..1] of cuint;  (**< IR virtual screen resolution		 *)
    offset : array [0..1] of cint;  (**< IR XY correction offset			 *)
  end;
  pir_t = ^ir_t;

  joystick_t = packed record
    max : vec2b_t;  (**< maximum joystick values	 *)
    min : vec2b_t;  (**< minimum joystick values	 *)
    center : vec2b_t;  (**< center joystick values		 *)
    pos : vec2b_t;  (**< raw position values         *)
    ang : cfloat;  (**< angle the joystick is being held		 *)
    mag : cfloat;  (**< magnitude of the joystick (range 0-1)	 *)
  end;
  pjoystick_t = ^joystick_t;

  nunchuk_t = packed record
    accel_calib : accel_t;  (**< nunchuk accelerometer calibration		 *)
    js : joystick_t;  (**< joystick calibration					 *)
    flags : pcint;  (**< options flag (points to wiimote_t.flags)  *)
    btns : ubyte;  (**< what buttons have just been pressed	 *)
    btns_last : ubyte;  (**< what buttons have just been pressed	 *)
    btns_held : ubyte;  (**< what buttons are being held down		 *)
    btns_released : ubyte;  (**< what buttons were just released this	 *)
    accel : vec3w_t;  (**< current raw acceleration data			 *)
    orient : orient_t;  (**< current orientation on each axis		 *)
    gforce : gforce_t;  (**< current gravity forces on each axis	 *)
  end;
  pnunchuk_t = ^nunchuk_t;

  classic_ctrl_t = packed record
    btns : cshort;  (**< what buttons have just been pressed	 *)
    btns_last : cshort;  (**< what buttons have just been pressed	 *)
    btns_held : cshort;  (**< what buttons are being held down		 *)
    btns_released : cshort;  (**< what buttons were just released this	 *)
    rs_raw : ubyte;
    ls_raw : ubyte;
    r_shoulder : cfloat;  (**< right shoulder button (range 0-1)		 *)
    l_shoulder : cfloat;  (**< left shoulder button (range 0-1)		 *)
    ljs : joystick_t;  (**< left joystick calibration				 *)
    rjs : joystick_t;  (**< right joystick calibration				 *)
  end;
  pclassic_ctrl_t = ^classic_ctrl_t;

  guitar_hero_3_t = packed record
    btns : cshort;  (**< what buttons have just been pressed	 *)
    btns_last : cshort;  (**< what buttons have just been pressed	 *)
    btns_held : cshort;  (**< what buttons are being held down		 *)
    btns_released : cshort;  (**< what buttons were just released this	 *)
    wb_raw : ubyte;
    whammy_bar : cfloat;  (**< whammy bar (range 0-1)					 *)
    tb_raw : ubyte;
    touch_bar : cint;  (**< touch bar								 *)
    js : joystick_t;  (**< joystick calibration					 *)
  end;
  pguitar_hero_3_t = ^guitar_hero_3_t;

  wii_board_t = packed record
    tl : cfloat;  (* Interpolated  *)
    tr : cfloat;
    bl : cfloat;
    br : cfloat;  (* End interp  *)
    rtl : cshort;  (* RAW  *)
    rtr : cshort;
    rbl : cshort;
    rbr : cshort;  (* /RAW  *)
    ctl : array [0..2] of cshort;  (* Calibration  *)
    ctr : array [0..2] of cshort;
    cbl : array [0..2] of cshort;
    cbr : array [0..2] of cshort;  (* /Calibration  *)
    x : cfloat;
    y : cfloat;
  end;
  pwii_board_t = ^wii_board_t;

  motion_plus_t = packed record
    rx : cshort;
    ry : cshort;
    rz : cshort;
    status : ubyte;
    ext : ubyte;
  end;
  pmotion_plus_t = ^motion_plus_t;

  expansion_t = packed record
	  _type: cint;
    case integer of
      0:  (nunchuk: nunchuk_t);
  		1:  (classic: classic_ctrl_t);
  		2:  (gh3: guitar_hero_3_t);
 		  3:  (wb: wii_board_t);
		  4:  (mp: motion_plus_t);
	end;
  Pexpansion_t = ^expansion_t;

  wiimote_state_t = packed record
    btns : cushort;
    ir : ir_t;
    accel : vec3w_t;
    exp : expansion_t;
  end;
  Pwiimote_state_t = ^wiimote_state_t;

//  Pbte_pcb = ^bte_pcb;
  wiimote_t = packed record
    unid : cint;  (**< user specified id						 *)
    cmdq : lwp_queue;
    bdaddr : _bd_addr;  (**< bt address								 *)
    bdaddr_str : array [0..17] of char;  (**< readable bt address					 *)
    sock : Pbte_pcb;  (**< output socket							 *)
    event_cb : wii_event_cb;  (**< event callback							 *)
    state : cint;  (**< various state flags					 *)
    leds : ubyte;  (**< currently lit leds						 *)
    battery_level : ubyte;  (**< battery level							 *)
    flags : cint;  (**< options flag							 *)
    handshake_state : ubyte;  (**< the state of the connection handshake	 *)
    expansion_state : ubyte;  (**< the state of the expansion handshake	 *)
    data_req : Pdata_req_t;  (**< list of data read requests				 *)
    cmd_head : Pcmd_blk_t;
    cmd_tail : Pcmd_blk_t;
    accel_calib : accel_t;  (**< wiimote accelerometer calibration		 *)
    exp : expansion_t;  (**< wiimote expansion device				 *)
    accel : vec3w_t;  (**< current raw acceleration data			 *)
    orient : orient_t;  (**< current orientation on each axis		 *)
    gforce : gforce_t;  (**< current gravity forces on each axis	 *)
    ir : ir_t;  (**< IR data								 *)
    btns : cushort;  (**< what buttons are down					 *)
    btns_last : cushort;  (**< what buttons were down before			 *)
    btns_held : cushort;  (**< what buttons are and were held down	 *)
    btns_released : cushort;  (**< what buttons were just released		 *)
    lstate : wiimote_state_t;  (**< last saved state						 *)
    event : WIIUSE_EVENT_TYPE;  (**< type of event that occured				 *)
    event_buf : array [0..MAX_PAYLOAD-1] of ubyte;  (**< event buffer							 *)
    motion_plus_id : array [0..5] of ubyte;
  end;
  wiimote = wiimote_t;

  function IS_PRESSED(var dev: wiimote_t; button: cint): boolean; inline;
  function IS_PRESSED(var dev: nunchuk_t; button: cint): boolean; inline;
  function IS_PRESSED(var dev: classic_ctrl_t; button: cint): boolean; inline;
  function IS_PRESSED(var dev: guitar_hero_3_t; button: cint): boolean; inline;
  function IS_HELD(var dev: wiimote_t; button: cint): boolean; inline;
  function IS_HELD(var dev: nunchuk_t; button: cint): boolean; inline;
  function IS_HELD(var dev: classic_ctrl_t; button: cint): boolean; inline;
  function IS_HELD(var dev: guitar_hero_3_t; button: cint): boolean; inline;
  function IS_RELEASED(var dev: wiimote_t; button: cint): boolean; inline;
  function IS_RELEASED(var dev: nunchuk_t; button: cint): boolean; inline;
  function IS_RELEASED(var dev: classic_ctrl_t; button: cint): boolean; inline;
  function IS_RELEASED(var dev: guitar_hero_3_t; button: cint): boolean; inline;
  function IS_JUST_PRESSED(var dev: wiimote_t; button: cint): boolean; inline;
  function IS_JUST_PRESSED(var dev: nunchuk_t; button: cint): boolean; inline;
  function IS_JUST_PRESSED(var dev: classic_ctrl_t; button: cint): boolean; inline;
  function IS_JUST_PRESSED(var dev: guitar_hero_3_t; button: cint): boolean; inline;
  procedure WIIUSE_GET_IR_SENSITIVITY(var wm: wiimote_t; var lvl: cint);

//  /**
//   *	@brief Return the IR sensitivity level.
//   *	@param wm		Pointer to a wiimote_t structure.
//   *	@param lvl		[out] Pointer to an int that will hold the level setting.
//   *	If no level is set 'lvl' will be set to 0.
//   */
//#define WIIUSE_GET_IR_SENSITIVITY(dev, lvl)								\
//  			do {														\
//  				if ((wm->state & 0x01000) == 0x01000) 		*lvl = 1;	\
//  				else if ((wm->state & 0x02000) == 0x02000) 	*lvl = 2;	\
//  				else if ((wm->state & 0x04000) == 0x04000) 	*lvl = 3;	\
//  				else if ((wm->state & 0x08000) == 0x08000) 	*lvl = 4;	\
//  				else if ((wm->state & 0x10000) == 0x10000) 	*lvl = 5;	\
//  				else										*lvl = 0;	\
//  			} while (0)

  function WIIUSE_USING_ACC(var wm: wiimote_t): boolean; inline;
  function WIIUSE_USING_EXP(var wm: wiimote_t): boolean; inline;
  function WIIUSE_USING_IR(var wm: wiimote_t): boolean; inline;
  function WIIUSE_USING_SPEAKER(var wm: wiimote_t): boolean; inline;
  function WIIUSE_IS_LED_SET(var wm: wiimote_t; num: byte): boolean; inline;


{$ifdef GEKKO}
type
  P_bd_addr = ^_bd_addr;
  wiimote_listen_t = packed record
    bdaddr : _bd_addr;
    sock : Pbte_pcb;
    assign_cb : function(bdaddr: P_bd_addr): Pwiimote_t; cdecl;
    wm : Pwiimote_t;
  end;
  wiimote_listen = wiimote_listen_t;
  pwiimote_listen = ^wiimote_listen_t;
{$endif}

///*****************************************
// *
// *	Include API specific stuff
// *
// *****************************************/

//#ifdef _WIN32
//	#define WIIUSE_EXPORT_DECL __declspec(dllexport)
//	#define WIIUSE_IMPORT_DECL __declspec(dllimport)
//#else
//	#define WIIUSE_EXPORT_DECL
//	#define WIIUSE_IMPORT_DECL
//#endif

//#ifdef WIIUSE_COMPILE_LIB
//	#define WIIUSE_EXPORT WIIUSE_EXPORT_DECL
//#else
//	#define WIIUSE_EXPORT WIIUSE_IMPORT_DECL
//#endif


function wiiuse_version: pcchar; cdecl; external;


{$ifndef GEKKO}
type
  PPwiimote_t = ^Pwiimote_t;
function wiiuse_init(wiimotes: cint): PPwiimote_t; cdecl; external;

{$else}
type
  Tassign_cb = function(bdaddr: P_bd_addr): Pwiimote_t; cdecl;
function wiiuse_register(wml: Pwiimote_listen_t; bdaddr: P_bd_addr;
  assign_cb: Tassign_cb): cint; cdecl; external;
function wiiuse_init(wiimotes: cint; event_cb: wii_event_cb): PPwiimote_t; cdecl; external;
procedure wiiuse_sensorbar_enable(enable: cint); cdecl; external;
{$endif}

procedure wiiuse_disconnected(wm: Pwiimote_t); cdecl; external;
procedure wiiuse_cleanup(wm: PPwiimote_t; wiimotes: cint); cdecl; external;
procedure wiiuse_rumble(wm: Pwiimote_t; status: cint); cdecl; external;
procedure wiiuse_toggle_rumble(wm: Pwiimote_t); cdecl; external;
procedure wiiuse_set_leds(wm: Pwiimote_t; leds: cint; cb: cmd_blk_cb); cdecl; external;
procedure wiiuse_motion_sensing(wm: Pwiimote_t; status: cint); cdecl; external;
function wiiuse_read_data(wm: Pwiimote_t; buffer: Pubyte; offset: cuint;
  len: cushort; cb: cmd_blk_cb): cint; cdecl; external;
function wiiuse_write_data(wm: Pwiimote_t; addr: cuint; data: Pubyte;
  len: ubyte; cb: cmd_blk_cb): cint; cdecl; external;
procedure wiiuse_status(wm: Pwiimote_t; cb: cmd_blk_cb); cdecl; external;
function wiiuse_get_by_id(wm: PPwiimote_t; wiimotes, unid: cint): Pwiimote_t; cdecl; external;
function wiiuse_set_flags(wm: Pwiimote_t; enable, disable: cint): cint; cdecl; external;
function wiiuse_set_smooth_alpha(wm: Pwiimote_t; alpha: cfloat): cfloat; cdecl; external;
procedure wiiuse_set_bluetooth_stack(wm: PPwiimote_t; wiimotes: cint;
  type_: win_bt_stack_t); cdecl; external;
procedure wiiuse_resync(wm: Pwiimote_t); cdecl; external;
procedure wiiuse_set_timeout(wm: PPwiimote_t; wiimotes: cint;
  normal_timeout, exp_timeout: ubyte); cdecl; external;
function wiiuse_write_streamdata(wm: Pwiimote_t; data: Pubyte; len: ubyte;
  cb: cmd_blk_cb): cint; cdecl; external;

(* connect.c  *)

function wiiuse_find(wm: PPwiimote_t; max_wiimotes, timeout: cint): cint; cdecl; external;
function wiiuse_connect(wm: PPwiimote_t; wiimotes: cint): cint; cdecl; external;
procedure wiiuse_disconnect(wm: Pwiimote_t); cdecl; external;

(* events.c  *)
function wiiuse_poll(wm: PPwiimote_t; wiimotes: cint): cint; cdecl; external;

(* ir.c  *)
procedure wiiuse_set_ir_mode(wm: Pwiimote_t); cdecl; external;
procedure wiiuse_set_ir(wm: Pwiimote_t; status: cint); cdecl; external;
procedure wiiuse_set_ir_vres(wm: Pwiimote_t; x, y: cuint); cdecl; external;
procedure wiiuse_set_ir_position(wm: Pwiimote_t; pos: ir_position_t); cdecl; external;
procedure wiiuse_set_aspect_ratio(wm: Pwiimote_t; aspect: aspect_t); cdecl; external;
procedure wiiuse_set_ir_sensitivity(wm: Pwiimote_t; level: cint); cdecl; external;

(* motion_plus.c  *)
procedure wiiuse_set_motion_plus(wm: Pwiimote_t; status: cint); cdecl; external;

(* speaker.c  *)
procedure wiiuse_set_speaker(wm: Pwiimote_t; status: cint); cdecl; external;
{$ENDIF OGC_INTERFACE}



{$IFDEF OGC_IMPLEMENTATION}
function IS_PRESSED(var dev: wiimote_t; button: cint): boolean; inline;
begin
  IS_PRESSED := (dev.btns and button) = button;
end;

function IS_PRESSED(var dev: nunchuk_t; button: cint): boolean; inline;
begin
  IS_PRESSED := (dev.btns and button) = button;
end;

function IS_PRESSED(var dev: classic_ctrl_t; button: cint): boolean; inline;
begin
  IS_PRESSED := (dev.btns and button) = button;
end;

function IS_PRESSED(var dev: guitar_hero_3_t; button: cint): boolean; inline;
begin
  IS_PRESSED := (dev.btns and button) = button;
end;

function IS_HELD(var dev: wiimote_t; button: cint): boolean; inline;
begin
  IS_HELD := (dev.btns_held and button) = button;
end;

function IS_HELD(var dev: nunchuk_t; button: cint): boolean; inline;
begin
  IS_HELD := (dev.btns_held and button) = button;
end;

function IS_HELD(var dev: classic_ctrl_t; button: cint): boolean; inline;
begin
  IS_HELD := (dev.btns_held and button) = button;
end;

function IS_HELD(var dev: guitar_hero_3_t; button: cint): boolean; inline;
begin
  IS_HELD := (dev.btns_held and button) = button;
end;

function IS_RELEASED(var dev: wiimote_t; button: cint): boolean; inline;
begin
  IS_RELEASED := (dev.btns_released and button) = button;
end;

function IS_RELEASED(var dev: nunchuk_t; button: cint): boolean; inline;
begin
  IS_RELEASED := (dev.btns_released and button) = button;
end;

function IS_RELEASED(var dev: classic_ctrl_t; button: cint): boolean; inline;
begin
  IS_RELEASED := (dev.btns_released and button) = button;
end;

function IS_RELEASED(var dev: guitar_hero_3_t; button: cint): boolean; inline;
begin
  IS_RELEASED := (dev.btns_released and button) = button;
end;

function IS_JUST_PRESSED(var dev: wiimote_t; button: cint): boolean; inline;
begin
  IS_JUST_PRESSED := IS_PRESSED(dev, button) and not IS_HELD(dev, button);
end;

function IS_JUST_PRESSED(var dev: nunchuk_t; button: cint): boolean; inline;
begin
  IS_JUST_PRESSED := IS_PRESSED(dev, button) and not IS_HELD(dev, button);
end;

function IS_JUST_PRESSED(var dev: classic_ctrl_t; button: cint): boolean; inline;
begin
  IS_JUST_PRESSED := IS_PRESSED(dev, button) and not IS_HELD(dev, button);
end;

function IS_JUST_PRESSED(var dev: guitar_hero_3_t; button: cint): boolean; inline;
begin
  IS_JUST_PRESSED := IS_PRESSED(dev, button) and not IS_HELD(dev, button);
end;

procedure WIIUSE_GET_IR_SENSITIVITY(var wm: wiimote_t; var lvl: cint);
begin
  repeat
  	if (wm.state and $01000) = $01000 then
      lvl := 1
  	else
    if (wm.state and $02000) = $02000 then
      lvl := 2
  	else
    if (wm.state and $04000) = $04000 then
      lvl := 3
    else
    if (wm.state and $08000) = $08000 then
      lvl := 4
    else
    if (wm.state and $10000) = $10000 then
      lvl := 5
    else
      lvl := 0;
  until false;
end;

//  /**
//   *	@brief Return the IR sensitivity level.
//   *	@param wm		Pointer to a wiimote_t structure.
//   *	@param lvl		[out] Pointer to an int that will hold the level setting.
//   *	If no level is set 'lvl' will be set to 0.
//   */
//#define WIIUSE_GET_IR_SENSITIVITY(dev, lvl)								\
//  			do {														\
//  				if ((wm->state & 0x01000) == 0x01000) 		*lvl = 1;	\
//  				else if ((wm->state & 0x02000) == 0x02000) 	*lvl = 2;	\
//  				else if ((wm->state & 0x04000) == 0x04000) 	*lvl = 3;	\
//  				else if ((wm->state & 0x08000) == 0x08000) 	*lvl = 4;	\
//  				else if ((wm->state & 0x10000) == 0x10000) 	*lvl = 5;	\
//  				else										*lvl = 0;	\
//  			} while (0)

function WIIUSE_USING_ACC(var wm: wiimote_t): boolean; inline;
begin
  WIIUSE_USING_ACC := (wm.state and $00100) = $00100;
end;

function WIIUSE_USING_EXP(var wm: wiimote_t): boolean; inline;
begin
  WIIUSE_USING_EXP := (wm.state and $00200) = $00200;
end;

function WIIUSE_USING_IR(var wm: wiimote_t): boolean; inline;
begin
  WIIUSE_USING_IR := (wm.state and $00400) = $00400;
end;

function WIIUSE_USING_SPEAKER(var wm: wiimote_t): boolean; inline;
begin
  WIIUSE_USING_SPEAKER := (wm.state and $00800) = $00800;
end;

function WIIUSE_IS_LED_SET(var wm: wiimote_t; num: byte): boolean; inline;
begin
  WIIUSE_IS_LED_SET := (wm.leds and (num shl 4)) = (num shl 4);
end;

{$ENDIF OGC_IMPLEMENTATION}

