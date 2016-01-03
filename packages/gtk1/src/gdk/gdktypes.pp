{
}

{****************************************************************************
                                 Interface
****************************************************************************}

{$ifdef read_interface}

   type
<<<<<<< HEAD
<<<<<<< HEAD
=======
     PPPChar   = ^PPchar;
>>>>>>> graemeg/fixes_2_2
=======
     PPPChar   = ^PPchar;
>>>>>>> origin/fixes_2_2
     PPguchar  = Pguchar;
     PPgchar   = ^Pgchar;
     PPPgchar  = ^PPgchar;
     PPgint    = ^Pgint;

     PGdkWChar = ^TGdkWChar;
     TGdkWChar = guint32;

     TGdkIC = pointer;
     PGdkIc = ^TGdkIC;

     TGdkAtom = gulong;
     PTGdkAtom = ^TGdkAtom;

     PPGdkWindow = ^PGdkWindow;
     PGdkWindow = ^TGdkWindow;
     TGdkWindow = record
          user_data : gpointer;
       end;

     TGdkPixmap = TGdkWindow;
     PGdkPixmap = ^TGdkWindow;
     PPGdkPixmap = ^PGdkPixmap;
     TGdkBitmap = TGdkWindow;
     PGdkBitMap = ^TGdkBitMap;
     PPGdkBitMap = ^PGdkBitMap;
     TGdkDrawable = TGdkWindow;
     PGdkDrawable = ^TGdkDrawable;

type
   PGdkWindowType = ^TGdkWindowType;
   TGdkWindowType =  Longint;
Const
   GDK_WINDOW_ROOT = 0;
   GDK_WINDOW_TOPLEVEL = 1;
   GDK_WINDOW_CHILD = 2;
   GDK_WINDOW_DIALOG = 3;
   GDK_WINDOW_TEMP = 4;
   GDK_WINDOW_PIXMAP = 5;
   GDK_WINDOW_FOREIGN = 6;

type
   PGdkWindowClass = ^TGdkWindowClass;
   TGdkWindowClass =  Longint;
Const
   GDK_INPUT_OUTPUT = 0;
   GDK_INPUT_ONLY = 1;

type
   PGdkImageType = ^TGdkImageType;
   TGdkImageType =  Longint;
Const
   GDK_IMAGE_NORMAL = 0;
   GDK_IMAGE_SHARED = 1;
   GDK_IMAGE_FASTEST = 2;

type
   PGdkVisualType = ^TGdkVisualType;
   TGdkVisualType =  Longint;
Const
     GDK_VISUAL_STATIC_GRAY = 0;
     GDK_VISUAL_GRAYSCALE = 1;
     GDK_VISUAL_STATIC_COLOR = 2;
     GDK_VISUAL_PSEUDO_COLOR = 3;
     GDK_VISUAL_TRUE_COLOR = 4;
     GDK_VISUAL_DIRECT_COLOR = 5;

type
   PGdkFontType = ^TGdkFontType;
   TGdkFontType =  Longint;
Const
     GDK_FONT_FONT = 0;
     GDK_FONT_FONTSET = 1;

type
   PGdkWindowAttributesType = ^TGdkWindowAttributesType;
   TGdkWindowAttributesType =  Longint;
Const
     GDK_WA_TITLE = (1) shl (1);
     GDK_WA_X = (1) shl (2);
     GDK_WA_Y = (1) shl (3);
     GDK_WA_CURSOR = (1) shl (4);
     GDK_WA_COLORMAP = (1) shl (5);
     GDK_WA_VISUAL = (1) shl (6);
     GDK_WA_WMCLASS = (1) shl (7);
     GDK_WA_NOREDIR = (1) shl (8);

type
   PGdkWindowHints = ^TGdkWindowHints;
   TGdkWindowHints =  Longint;
Const
     GDK_HINT_POS = 1 shl 0;
     GDK_HINT_MIN_SIZE = 1 shl 1;
     GDK_HINT_MAX_SIZE = 1 shl 2;
     GDK_HINT_BASE_SIZE = 1 shl 3;
     GDK_HINT_ASPECT = 1 shl 4;
     GDK_HINT_RESIZE_INC = 1 shl 5;

type
   PGdkfunction = ^TGdkfunction;
   TGdkfunction = Longint;
Const
     GDK_COPY = 0;
     GDK_INVERT = 1;
     GDK_XOR = 2;
     GDK_CLEAR = 3;
     GDK_AND = 4;
     GDK_AND_REVERSE = 5;
     GDK_AND_INVERT = 6;
     GDK_NOOP = 7;
     GDK_OR = 8;
     GDK_EQUIV = 9;
     GDK_OR_REVERSE = 10;
     GDK_COPY_INVERT = 11;
     GDK_OR_INVERT = 12;
     GDK_NAND = 13;
     GDK_SET = 14;

type
   PGdkFill = ^TGdkFill;
   TGdkFill =  Longint;
Const
     GDK_SOLID = 0;
     GDK_TILED = 1;
     GDK_STIPPLED = 2;
     GDK_OPAQUE_STIPPLED = 3;

type
   PGdkFillRule = ^TGdkFillRule;
   TGdkFillRule =  Longint;
Const
     GDK_EVEN_ODD_RULE = 0;
     GDK_WINDING_RULE = 1;

type
   PGdkLineStyle = ^TGdkLineStyle;
   TGdkLineStyle =  Longint;
Const
     GDK_LINE_SOLID = 0;
     GDK_LINE_ON_OFF_DASH = 1;
     GDK_LINE_DOUBLE_DASH = 2;

type
   PGdkCapStyle = ^TGdkCapStyle;
   TGdkCapStyle =  Longint;
Const
     GDK_CAP_NOT_LAST = 0;
     GDK_CAP_BUTT = 1;
     GDK_CAP_ROUND = 2;
     GDK_CAP_PROJECTING = 3;

type
   PGdkJoinStyle = ^TGdkJoinStyle;
   TGdkJoinStyle =  Longint;
Const
     GDK_JOIN_MITER = 0;
     GDK_JOIN_ROUND = 1;
     GDK_JOIN_BEVEL = 2;

type
   PGdkCursorType = ^TGdkCursorType;
   TGdkCursorType =  Longint;

Const
     GDK_CURSOR_IS_PIXMAP = -1;
     GDK_X_CURSOR = 0;
     GDK_ARROW = 2;
     GDK_BASED_ARROW_DOWN = 4;
     GDK_BASED_ARROW_UP = 6;
     GDK_BOAT = 8;
     GDK_BOGOSITY = 10;
     GDK_BOTTOM_LEFT_CORNER = 12;
     GDK_BOTTOM_RIGHT_CORNER = 14;
     GDK_BOTTOM_SIDE = 16;
     GDK_BOTTOM_TEE = 18;
     GDK_BOX_SPIRAL = 20;
     GDK_CENTER_PTR = 22;
     GDK_CIRCLE = 24;
     GDK_CLOCK = 26;
     GDK_COFFEE_MUG = 28;
     GDK_CROSS = 30;
     GDK_CROSS_REVERSE = 32;
     GDK_CROSSHAIR = 34;
     GDK_DIAMOND_CROSS = 36;
     GDK_DOT = 38;
     GDK_DOTBOX = 40;
     GDK_DOUBLE_ARROW = 42;
     GDK_DRAFT_LARGE = 44;
     GDK_DRAFT_SMALL = 46;
     GDK_DRAPED_BOX = 48;
     GDK_EXCHANGE = 50;
     GDK_FLEUR = 52;
     GDK_GOBBLER = 54;
     GDK_GUMBY = 56;
     GDK_HAND1 = 58;
     GDK_HAND2 = 60;
     GDK_HEART = 62;
     GDK_ICON = 64;
     GDK_IRON_CROSS = 66;
     GDK_LEFT_PTR = 68;
     GDK_LEFT_SIDE = 70;
     GDK_LEFT_TEE = 72;
     GDK_LEFTBUTTON = 74;
     GDK_LL_ANGLE = 76;
     GDK_LR_ANGLE = 78;
     GDK_MAN = 80;
     GDK_MIDDLEBUTTON = 82;
     GDK_MOUSE = 84;
     GDK_PENCIL = 86;
     GDK_PIRATE = 88;
     GDK_PLUS = 90;
     GDK_QUESTION_ARROW = 92;
     GDK_RIGHT_PTR = 94;
     GDK_RIGHT_SIDE = 96;
     GDK_RIGHT_TEE = 98;
     GDK_RIGHTBUTTON = 100;
     GDK_RTL_LOGO = 102;
     GDK_SAILBOAT = 104;
     GDK_SB_DOWN_ARROW = 106;
     GDK_SB_H_DOUBLE_ARROW = 108;
     GDK_SB_LEFT_ARROW = 110;
     GDK_SB_RIGHT_ARROW = 112;
     GDK_SB_UP_ARROW = 114;
     GDK_SB_V_DOUBLE_ARROW = 116;
     GDK_SHUTTLE = 118;
     GDK_SIZING = 120;
     GDK_SPIDER = 122;
     GDK_SPRAYCAN = 124;
     GDK_STAR = 126;
     GDK_TARGET = 128;
     GDK_TCROSS = 130;
     GDK_TOP_LEFT_ARROW = 132;
     GDK_TOP_LEFT_CORNER = 134;
     GDK_TOP_RIGHT_CORNER = 136;
     GDK_TOP_SIDE = 138;
     GDK_TOP_TEE = 140;
     GDK_TREK = 142;
     GDK_UL_ANGLE = 144;
     GDK_UMBRELLA = 146;
     GDK_UR_ANGLE = 148;
     GDK_WATCH = 150;
     GDK_XTERM = 152;
     GDK_LAST_CURSOR = 153;
     GDK_NUM_GLYPHS = 154;

type
   PGdkFilterReturn = ^TGdkFilterReturn;
   TGdkFilterReturn =  Longint;
Const
     GDK_FILTER_CONTINUE = 0;
     GDK_FILTER_TRANSLATE = 1;
     GDK_FILTER_REMOVE = 2;

type
   PGdkVisibilityState = ^TGdkVisibilityState;
   TGdkVisibilityState =  Longint;
Const
     GDK_VISIBILITY_UNOBSCURED = 0;
     GDK_VISIBILITY_PARTIAL = 1;
     GDK_VISIBILITY_FULLY_OBSCURED = 2;

type
   PGdkEventType = ^TGdkEventType;
   TGdkEventType =  Longint;
Const
     GDK_NOTHING = -1;
     GDK_DELETE = 0;
     GDK_DESTROY = 1;
     GDK_EXPOSE = 2;
     GDK_MOTION_NOTIFY = 3;
     GDK_BUTTON_PRESS = 4;
     GDK_2BUTTON_PRESS = 5;
     GDK_3BUTTON_PRESS = 6;
     GDK_BUTTON_RELEASE = 7;
     GDK_KEY_PRESS = 8;
     GDK_KEY_RELEASE = 9;
     GDK_ENTER_NOTIFY = 10;
     GDK_LEAVE_NOTIFY = 11;
     GDK_FOCUS_CHANGE = 12;
     GDK_CONFIGURE = 13;
     GDK_MAP = 14;
     GDK_UNMAP = 15;
     GDK_PROPERTY_NOTIFY = 16;
     GDK_SELECTION_CLEAR = 17;
     GDK_SELECTION_REQUEST = 18;
     GDK_SELECTION_NOTIFY = 19;
     GDK_PROXIMITY_IN = 20;
     GDK_PROXIMITY_OUT = 21;
     GDK_DRAG_ENTER = 22;
     GDK_DRAG_LEAVE = 23;
     GDK_DRAG_MOTION_EVENT = 24;
     GDK_DRAG_STATUS_EVENT = 25;
     GDK_DROP_START = 26;
     GDK_DROP_FINISHED = 27;
     GDK_CLIENT_EVENT = 28;
     GDK_VISIBILITY_NOTIFY = 29;
     GDK_NO_EXPOSE = 30;

type
   PGdkEventMask = ^TGdkEventMask;
   TGdkEventMask =  Longint;
Const
     GDK_EXPOSURE_MASK = (1) shl (1);
     GDK_POINTER_MOTION_MASK = (1) shl (2);
     GDK_POINTER_MOTION_HINT_MASK = (1) shl (3);
     GDK_BUTTON_MOTION_MASK = (1) shl (4);
     GDK_BUTTON1_MOTION_MASK = (1) shl (5);
     GDK_BUTTON2_MOTION_MASK = (1) shl (6);
     GDK_BUTTON3_MOTION_MASK = (1) shl (7);
     GDK_BUTTON_PRESS_MASK = (1) shl (8);
     GDK_BUTTON_RELEASE_MASK = (1) shl (9);
     GDK_KEY_PRESS_MASK = (1) shl (10);
     GDK_KEY_RELEASE_MASK = (1) shl (11);
     GDK_ENTER_NOTIFY_MASK = (1) shl (12);
     GDK_LEAVE_NOTIFY_MASK = (1) shl (13);
     GDK_FOCUS_CHANGE_MASK = (1) shl (14);
     GDK_STRUCTURE_MASK = (1) shl (15);
     GDK_PROPERTY_CHANGE_MASK = (1) shl (16);
     GDK_VISIBILITY_NOTIFY_MASK = (1) shl (17);
     GDK_PROXIMITY_IN_MASK = (1) shl (18);
     GDK_PROXIMITY_OUT_MASK = (1) shl (19);
     GDK_SUBSTRUCTURE_MASK = 1 shl 20;
     GDK_ALL_EVENTS_MASK = $0FFFFF;

type
   PGdkNotifyType = ^TGdkNotifyType;
   TGdkNotifyType =  Longint;
Const
     GDK_NOTIFY_ANCESTOR = 0;
     GDK_NOTIFY_VIRTUAL = 1;
     GDK_NOTIFY_INFERIOR = 2;
     GDK_NOTIFY_NONLINEAR = 3;
     GDK_NOTIFY_NONLINEAR_VIRTUAL = 4;
     GDK_NOTIFY_UNKNOWN = 5;

type
     PGdkCrossingMode = ^TGdkCrossingMode;
     TGdkCrossingMode = longint;
const
     GDK_CROSSING_NORMAL = 0;
     GDK_CROSSING_GRAB = 1;
     GDK_CROSSING_UNGRAB = 2;

type
   PGdkModifierType = ^TGdkModifierType;
   TGdkModifierType =  Longint;
Const
     GDK_SHIFT_MASK = (1) shl (0);
     GDK_LOCK_MASK = (1) shl (1);
     GDK_CONTROL_MASK = (1) shl (2);
     GDK_MOD1_MASK = (1) shl (3);
     GDK_MOD2_MASK = (1) shl (4);
     GDK_MOD3_MASK = (1) shl (5);
     GDK_MOD4_MASK = (1) shl (6);
     GDK_MOD5_MASK = (1) shl (7);
     GDK_BUTTON1_MASK = (1) shl (8);
     GDK_BUTTON2_MASK = (1) shl (9);
     GDK_BUTTON3_MASK = (1) shl (10);
     GDK_BUTTON4_MASK = (1) shl (11);
     GDK_BUTTON5_MASK = (1) shl (12);
     GDK_RELEASE_MASK = 1 shl 13;
     GDK_MODIFIER_MASK = $3fff;

type
   PGdkSubwindowMode = ^TGdkSubwindowMode;
   TGdkSubwindowMode =  Longint;
Const
     GDK_CLIP_BY_CHILDREN = 0;
     GDK_INCLUDE_INFERIORS = 1;

type
   PGdkInputCondition = ^TGdkInputCondition;
   TGdkInputCondition =  Longint;
Const
     GDK_INPUT_READ = (1) shl (0);
     GDK_INPUT_WRITE = (1) shl (1);
     GDK_INPUT_EXCEPTION = (1) shl (2);

type
   PGdkStatus = ^TGdkStatus;
   TGdkStatus =  Longint;
Const
     GDK_OK = 0;
     GDK_ERROR = -1;
     GDK_ERROR_PARAM = -2;
     GDK_ERROR_FILE = -3;
     GDK_ERROR_MEM = -4;

type
   PGdkByteOrder = ^TGdkByteOrder;
   TGdkByteOrder =  Longint;
Const
     GDK_LSB_FIRST = 0;
     GDK_MSB_FIRST = 1;

type
   PGdkGCValuesMask = ^TGdkGCValuesMask;
   TGdkGCValuesMask =  Longint;
Const
     GDK_GC_FOREGROUND = (1) shl (0);
     GDK_GC_BACKGROUND = (1) shl (1);
     GDK_GC_FONT = (1) shl (2);
     GDK_GC_function = (1) shl (3);
     GDK_GC_FILL = (1) shl (4);
     GDK_GC_TILE = (1) shl (5);
     GDK_GC_STIPPLE = (1) shl (6);
     GDK_GC_CLIP_MASK = (1) shl (7);
     GDK_GC_SUBWINDOW = (1) shl (8);
     GDK_GC_TS_X_ORIGIN = (1) shl (9);
     GDK_GC_TS_Y_ORIGIN = (1) shl (10);
     GDK_GC_CLIP_X_ORIGIN = (1) shl (11);
     GDK_GC_CLIP_Y_ORIGIN = (1) shl (12);
     GDK_GC_EXPOSURES = (1) shl (13);
     GDK_GC_LINE_WIDTH = (1) shl (14);
     GDK_GC_LINE_STYLE = (1) shl (15);
     GDK_GC_CAP_STYLE = (1) shl (16);
     GDK_GC_JOIN_STYLE = (1) shl (17);

type
   PGdkSelection = ^TGdkSelection;
   TGdkSelection =  Longint;
Const
     GDK_SELECTION_PRIMARY = 1;
     GDK_SELECTION_SECONDARY = 2;

type
   PGdkPropertyState = ^TGdkPropertyState;
   TGdkPropertyState =  Longint;
Const
     GDK_PROPERTY_NEW_VALUE = 0;
     GDK_PROPERTY_DELETE_STATE = 1;

type
   PGdkPropMode = ^TGdkPropMode;
   TGdkPropMode =  Longint;
Const
     GDK_PROP_MODE_REPLACE = 0;
     GDK_PROP_MODE_PREPEND = 1;
     GDK_PROP_MODE_APPEND = 2;

type
   PGdkInputSource = ^TGdkInputSource;
   TGdkInputSource =  Longint;
Const
     GDK_SOURCE_MOUSE = 0;
     GDK_SOURCE_PEN = 1;
     GDK_SOURCE_ERASER = 2;
     GDK_SOURCE_CURSOR = 3;

type
   PGdkInputMode = ^TGdkInputMode;
   TGdkInputMode =  Longint;
Const
     GDK_MODE_DISABLED = 0;
     GDK_MODE_SCREEN = 1;
     GDK_MODE_WINDOW = 2;

type
   PGdkAxisUse = ^TGdkAxisUse;
   TGdkAxisUse =  Longint;
Const
     GDK_AXIS_IGNORE = 0;
     GDK_AXIS_X = 1;
     GDK_AXIS_Y = 2;
     GDK_AXIS_PRESSURE = 3;
     GDK_AXIS_XTILT = 4;
     GDK_AXIS_YTILT = 5;
     GDK_AXIS_LAST = 6;

type
   PGdkTarget = ^TGdkTarget;
   TGdkTarget =  Longint;
Const
     GDK_TARGET_BITMAP = 5;
     GDK_TARGET_COLORMAP = 7;
     GDK_TARGET_DRAWABLE = 17;
     GDK_TARGET_PIXMAP = 20;
     GDK_TARGET_STRING = 31;

type
   PGdkSelectionType = ^TGdkSelectionType;
   TGdkSelectionType =  Longint;
Const
     GDK_SELECTION_TYPE_ATOM = 4;
     GDK_SELECTION_TYPE_BITMAP = 5;
     GDK_SELECTION_TYPE_COLORMAP = 7;
     GDK_SELECTION_TYPE_DRAWABLE = 17;
     GDK_SELECTION_TYPE_INTEGER = 19;
     GDK_SELECTION_TYPE_PIXMAP = 20;
     GDK_SELECTION_TYPE_WINDOW = 33;
     GDK_SELECTION_TYPE_STRING = 31;

type
   PGdkExtensionMode = ^TGdkExtensionMode;
   TGdkExtensionMode =  Longint;
Const
     GDK_EXTENSION_EVENTS_NONE = 0;
     GDK_EXTENSION_EVENTS_ALL = 1;
     GDK_EXTENSION_EVENTS_CURSOR = 2;

type
   PGdkIMStyle = ^TGdkIMStyle;
   TGdkIMStyle =  Longint;
Const
     GDK_IM_PREEDIT_AREA = $0001;
     GDK_IM_PREEDIT_CALLBACKS = $0002;
     GDK_IM_PREEDIT_POSITION = $0004;
     GDK_IM_PREEDIT_NOTHING = $0008;
     GDK_IM_PREEDIT_NONE = $0010;
     GDK_IM_PREEDIT_MASK = $001f;
     GDK_IM_STATUS_AREA = $0100;
     GDK_IM_STATUS_CALLBACKS = $0200;
     GDK_IM_STATUS_NOTHING = $0400;
     GDK_IM_STATUS_NONE = $0800;
     GDK_IM_STATUS_MASK = $0f00;

type
     PGdkICAttributesType = ^TGdkICAttributesType;
     TGdkICAttributesType = longint;
const
     GDK_IC_STYLE = 1 shl 0;
     GDK_IC_CLIENT_WINDOW = 1 shl 1;
       GDK_IC_ALL_REQ = 1 shl 0 + 1 shl 1;
       GDK_IC_FOCUS_WINDOW = 1 shl 2;
       GDK_IC_FILTER_EVENTS = 1 shl 3;
       GDK_IC_SPOT_LOCATION = 1 shl 4;
       GDK_IC_LINE_SPACING = 1 shl 5;
       GDK_IC_CURSOR = 1 shl 6;
       GDK_IC_PREEDIT_FONTSET = 1 shl 10;
       GDK_IC_PREEDIT_AREA = 1 shl 11;
       GDK_IC_PREEDIT_AREA_NEEDED = 1 shl 12;
       GDK_IC_PREEDIT_FOREGROUND = 1 shl 13;
       GDK_IC_PREEDIT_BACKGROUND = 1 shl 14;
       GDK_IC_PREEDIT_PIXMAP = 1 shl 15;
       GDK_IC_PREEDIT_COLORMAP = 1 shl 16;
       GDK_IC_STATUS_FONTSET = 1 shl 21;
       GDK_IC_STATUS_AREA = 1 shl 22;
       GDK_IC_STATUS_AREA_NEEDED = 1 shl 23;
       GDK_IC_STATUS_FOREGROUND = 1 shl 24;
       GDK_IC_STATUS_BACKGROUND = 1 shl 25;
       GDK_IC_STATUS_PIXMAP = 1 shl 26;
       GDK_IC_STATUS_COLORMAP = 1 shl 27;
       GDK_IC_PREEDIT_AREA_REQ = ((1 shl 11) + (1 shl 10));
       GDK_IC_PREEDIT_POSITION_REQ = (1 shl 11 + 1 shl 4 + 1 shl 10);
       GDK_IC_STATUS_AREA_REQ = (1 shl 22 or 1 shl 21);

type
   PGdkWMDecoration = ^TGdkWMDecoration;
   TGdkWMDecoration =  Longint;
Const
     GDK_DECOR_ALL = (1) shl (0);
     GDK_DECOR_BORDER = (1) shl (1);
     GDK_DECOR_RESIZEH = (1) shl (2);
     GDK_DECOR_TITLE = (1) shl (3);
     GDK_DECOR_MENU = (1) shl (4);
     GDK_DECOR_MINIMIZE = (1) shl (5);
     GDK_DECOR_MAXIMIZE = (1) shl (6);

type
   PGdkWMfunction = ^TGdkWMfunction;
   TGdkWMfunction = Longint;
Const
     GDK_FUNC_ALL = (1) shl (0);
     GDK_FUNC_RESIZE = (1) shl (1);
     GDK_FUNC_MOVE = (1) shl (2);
     GDK_FUNC_MINIMIZE = (1) shl (3);
     GDK_FUNC_MAXIMIZE = (1) shl (4);
     GDK_FUNC_CLOSE = (1) shl (5);

type
     PGdkInputfunction = ^TGdkInputfunction;
     TGdkInputfunction = procedure (data:gpointer; source:gint; condition:TGdkInputCondition);cdecl;

     PGdkDestroyNotify = ^TGdkDestroyNotify;
     TGdkDestroyNotify = procedure (data:gpointer);cdecl;

type
     PGdkColorContextMode = ^TGdkColorContextMode;
     TGdkColorContextMode = longint;
Const
     GDK_CC_MODE_UNDEFINED = 0;
     GDK_CC_MODE_BW = 1;
     GDK_CC_MODE_STD_CMAP = 2;
     GDK_CC_MODE_TRUE = 3;
     GDK_CC_MODE_MY_GRAY = 4;
     GDK_CC_MODE_PALETTE = 5;

type
   PGdkOverlapType = ^TGdkOverlapType;
   TGdkOverlapType =  Longint;
Const
     GDK_OVERLAP_RECTANGLE_IN = 0;
     GDK_OVERLAP_RECTANGLE_OUT = 1;
     GDK_OVERLAP_RECTANGLE_PART = 2;

type
     PGdkDragAction = ^TGdkDragAction;
     TGdkDragAction = longint;
const
     GDK_ACTION_DEFAULT = 1 shl 0;
     GDK_ACTION_COPY = 1 shl 1;
     GDK_ACTION_MOVE = 1 shl 2;
     GDK_ACTION_LINK = 1 shl 3;
     GDK_ACTION_PRIVATE = 1 shl 4;
     GDK_ACTION_ASK = 1 shl 5;

type
     PGdkDragProtocol = ^TGdkDragProtocol;
     TGdkDragProtocol = longint;
const
     GDK_DRAG_PROTO_MOTIF = 0;
     GDK_DRAG_PROTO_XDND = 1;
     GDK_DRAG_PROTO_ROOTWIN = 2;
     GDK_DRAG_PROTO_NONE = 3;

type
     PGdkColor = ^TGdkColor;
     TGdkColor = record
          pixel : gulong;
          red : gushort;
          green : gushort;
          blue : gushort;
       end;

     PGdkColormap = ^TGdkColormap;
     TGdkColormap = record
          size : gint;
          colors : PGdkColor;
       end;

     PGdkVisual = ^TGdkVisual;
     TGdkVisual = record
          thetype : TGdkVisualType;
          depth : gint;
          byte_order : TGdkByteOrder;
          colormap_size : gint;
          bits_per_rgb : gint;
          red_mask : guint32;
          red_shift : gint;
          red_prec : gint;
          green_mask : guint32;
          green_shift : gint;
          green_prec : gint;
          blue_mask : guint32;
          blue_shift : gint;
          blue_prec : gint;
       end;

     PGdkCursor = ^TGdkCursor;

     PGdkWindowAttr = ^TGdkWindowAttr;
     TGdkWindowAttr = record
          title : ^gchar;
          event_mask : gint;
          x : gint16;
          y : gint16;
          width : gint16;
          height : gint16;
          wclass : TGdkWindowClass;
          visual : PGdkVisual;
          colormap : PGdkColormap;
          window_type : TGdkWindowType;
          cursor : PGdkCursor;
          wmclass_name : ^gchar;
          wmclass_class : ^gchar;
          override_redirect : gboolean;
       end;

     PGdkGeometry = ^TGdkGeometry;
     TGdkGeometry = record
          min_width : gint;
          min_height : gint;
          max_width : gint;
          max_height : gint;
          base_width : gint;
          base_height : gint;
          width_inc : gint;
          height_inc : gint;
          min_aspect : gdouble;
          max_aspect : gdouble;
       end;

     PGdkImage = ^TGdkImage;
     PPGdkImage = ^PGdkImage;
     TGdkImage = record
          thetype : TGdkImageType;
          visual : PGdkVisual;
          byte_order : TGdkByteOrder;
          width : guint16;
          height : guint16;
          depth : guint16;
          bpp : guint16;
          bpl : guint16;
          mem : gpointer;
       end;

     PGdkFont = ^TGdkFont;
     TGdkFont = record
          thetype : TGdkFontType;
          ascent : gint;
          descent : gint;
       end;

     PGdkGCValues = ^TGdkGCValues;
     TGdkGCValues = record
          foreground : TGdkColor;
          background : TGdkColor;
          font : PGdkFont;
          thefunction : TGdkfunction;
          fill : TGdkFill;
          tile : PGdkPixmap;
          stipple : PGdkPixmap;
          clip_mask : PGdkPixmap;
          subwindow_mode : TGdkSubwindowMode;
          ts_x_origin : gint;
          ts_y_origin : gint;
          clip_x_origin : gint;
          clip_y_origin : gint;
          graphics_exposures : gint;
          line_width : gint;
          line_style : TGdkLineStyle;
          cap_style : TGdkCapStyle;
          join_style : TGdkJoinStyle;
       end;

     PGdkGC = ^TGdkGC;
     TGdkGC = record
          dummy_var : gint;
       end;

     PGdkPoint = ^TGdkPoint;
     TGdkPoint = record
          x : gint16;
          y : gint16;
       end;

     PGdkRectangle = ^TGdkRectangle;
     TGdkRectangle = record
          x : gint16;
          y : gint16;
          width : guint16;
          height : guint16;
       end;

     PGdkSegment = ^TGdkSegment;
     TGdkSegment = record
          x1 : gint16;
          y1 : gint16;
          x2 : gint16;
          y2 : gint16;
       end;

     TGdkCursor = record
          thetype : TGdkCursorType;
       end;

     PGdkColorContextDither = ^TGdkColorContextDither;
     TGdkColorContextDither = record
          fast_rgb : array[0..31] of array[0..31] of array[0..31] of gint;
          fast_err : array[0..31] of array[0..31] of array[0..31] of gint;
          fast_erg : array[0..31] of array[0..31] of array[0..31] of gint;
          fast_erb : array[0..31] of array[0..31] of array[0..31] of gint;
       end;

     PGdkColorContext = ^TGdkColorContext;
     TGdkColorContext = record
          visual : PGdkVisual;
          colormap : PGdkColormap;
          num_colors : gint;
          max_colors : gint;
          num_allocated : gint;
          mode : TGdkColorContextMode;
          need_to_free_colormap : gint;
          std_cmap_atom : TGdkAtom;
          clut : Pgulong;
          cmap : PGdkColor;
          color_hash : PGHashTable;
          palette : PGdkColor;
          num_palette : gint;
          fast_dither : PGdkColorContextDither;
          shifts : record
               red : gint;
               green : gint;
               blue : gint;
            end;
          masks : record
               red : gulong;
               green : gulong;
               blue : gulong;
            end;
          bits : record
               red : gint;
               green : gint;
               blue : gint;
            end;
          max_entry : gulong;
          black_pixel : gulong;
          white_pixel : gulong;
       end;

     PGdkDeviceKey = ^TGdkDeviceKey;
     TGdkDeviceKey = record
          keyval : guint;
          modifiers : TGdkModifierType;
       end;

     PGdkDeviceInfo = ^TGdkDeviceInfo;
     TGdkDeviceInfo = record
          deviceid : guint32;
          name : ^gchar;
          source : TGdkInputSource;
          mode : TGdkInputMode;
          has_cursor : gint;
          num_axes : gint;
          axes : ^TGdkAxisUse;
          num_keys : gint;
          keys : PGdkDeviceKey;
       end;

     PGdkTimeCoord = ^TGdkTimeCoord;
     TGdkTimeCoord = record
          time : guint32;
          x : gdouble;
          y : gdouble;
          pressure : gdouble;
          xtilt : gdouble;
          ytilt : gdouble;
       end;

     PGdkDragContext = ^TGdkDragContext;
     TGdkDragContext = record
          protocol : TGdkDragProtocol;
          is_source : gboolean;
          source_window : PGdkWindow;
          dest_window : PGdkWindow;
          targets : PGList;
          actions : TGdkDragAction;
          suggested_action : TGdkDragAction;
          action : TGdkDragAction;
          start_time : guint32;
       end;

     TGdkXEvent = pointer;
     PGdkXEvent = ^TGdkXEvent;

     PGdkEvent = ^TGdkEvent;

     TGdkEventFunc = procedure(event:PGdkEvent;data:gpointer);cdecl;
     TGdkFilterFunc = function (xevent:PGdkXEvent; event:PGdkEvent; data:gpointer):TGdkFilterReturn;cdecl;

     PGdkEventAny = ^TGdkEventAny;
     TGdkEventAny = record
          thetype : TGdkEventType;
          window : PGdkWindow;
          send_event : gint8;
       end;

     PGdkEventExpose = ^TGdkEventExpose;
     TGdkEventExpose = record
          thetype : TGdkEventType;
          window : PGdkWindow;
          send_event : gint8;
          area : TGdkRectangle;
          count : gint;
       end;

     PGdkEventNoExpose = ^TGdkEventNoExpose;
     TGdkEventNoExpose = record
          thetype : TGdkEventType;
          window : PGdkWindow;
          send_event : gint8;
       end;

     PGdkEventVisibility = ^TGdkEventVisibility;
     TGdkEventVisibility = record
          thetype : TGdkEventType;
          window : PGdkWindow;
          send_event : gint8;
          state : TGdkVisibilityState;
       end;

     PGdkEventMotion = ^TGdkEventMotion;
     TGdkEventMotion = record
          thetype : TGdkEventType;
          window : PGdkWindow;
          send_event : gint8;
          time : guint32;
          x : gdouble;
          y : gdouble;
          pressure : gdouble;
          xtilt : gdouble;
          ytilt : gdouble;
          state : guint;
          is_hint : gint16;
          source : TGdkInputSource;
          deviceid : guint32;
          x_root : gdouble;
          y_root : gdouble;
       end;

     PGdkEventButton = ^TGdkEventButton;
     TGdkEventButton = record
          thetype : TGdkEventType;
          window : PGdkWindow;
          send_event : gint8;
          time : guint32;
          x : gdouble;
          y : gdouble;
          pressure : gdouble;
          xtilt : gdouble;
          ytilt : gdouble;
          state : guint;
          button : guint;
          source : TGdkInputSource;
          deviceid : guint32;
          x_root : gdouble;
          y_root : gdouble;
       end;

     PGdkEventKey = ^TGdkEventKey;
     TGdkEventKey = record
          thetype : TGdkEventType;
          window : PGdkWindow;
          send_event : gint8;
          time : guint32;
          state : guint;
          keyval : guint;
          length : gint;
          thestring : Pgchar;
       end;

     PGdkEventCrossing = ^TGdkEventCrossing;
     TGdkEventCrossing = record
          thetype : TGdkEventType;
          window : PGdkWindow;
          send_event : gint8;
          subwindow : PGdkWindow;
          time : guint32;
          x : gdouble;
          y : gdouble;
          x_root : gdouble;
          y_root : gdouble;
          mode : TGdkCrossingMode;
          detail : TGdkNotifyType;
          focus : gboolean;
          state : guint;
       end;

     PGdkEventFocus = ^TGdkEventFocus;
     TGdkEventFocus = record
          thetype : TGdkEventType;
          window : PGdkWindow;
          send_event : gint8;
          thein : gint16;
       end;

     PGdkEventConfigure = ^TGdkEventConfigure;
     TGdkEventConfigure = record
          thetype : TGdkEventType;
          window : PGdkWindow;
          send_event : gint8;
          x : gint16;
          y : gint16;
          width : gint16;
          height : gint16;
       end;

     PGdkEventProperty = ^TGdkEventProperty;
     TGdkEventProperty = record
          thetype : TGdkEventType;
          window : PGdkWindow;
          send_event : gint8;
          atom : TGdkAtom;
          time : guint32;
          state : guint;
       end;

     PGdkEventSelection = ^TGdkEventSelection;
     TGdkEventSelection = record
          thetype : TGdkEventType;
          window : PGdkWindow;
          send_event : gint8;
          selection : TGdkAtom;
          target : TGdkAtom;
          theproperty : TGdkAtom;
          requestor : guint32;
          time : guint32;
       end;

     PGdkEventProximity = ^TGdkEventProximity;
     TGdkEventProximity = record
          thetype : TGdkEventType;
          window : PGdkWindow;
          send_event : gint8;
          time : guint32;
          source : TGdkInputSource;
          deviceid : guint32;
       end;

     PGdkEventClient = ^TGdkEventClient;
     TGdkEventClient = record
          thetype : TGdkEventType;
          window : PGdkWindow;
          send_event : gint8;
          message_type : TGdkAtom;
          data_format : gushort;
          dummy : gushort;
          data : record
              case longint of
                 0 : ( b : array[0..19] of char );
                 1 : ( s : array[0..9] of system.integer );
                 2 : ( l : array[0..4] of longint );
              end;
       end;

     PGdkEventDND = ^TGdkEventDND;
     TGdkEventDND = record
          thetype : TGdkEventType;
          window : PGdkWindow;
          send_event : gint8;
          context : PGdkDragContext;
          time : guint32;
          x_root : gshort;
          y_root : gshort;
       end;

     TGdkEvent = record
         case longint of
            0 : ( thetype : TGdkEventType );
            1 : ( any : TGdkEventAny );
            2 : ( expose : TGdkEventExpose );
            3 : ( no_expose : TGdkEventNoExpose );
            4 : ( visibility : TGdkEventVisibility );
            5 : ( motion : TGdkEventMotion );
            6 : ( button : TGdkEventButton );
            7 : ( key : TGdkEventKey );
            8 : ( crossing : TGdkEventCrossing );
            9 : ( focus_change : TGdkEventFocus );
            10 : ( configure : TGdkEventConfigure );
            11 : ( theproperty : TGdkEventProperty );
            12 : ( selection : TGdkEventSelection );
            13 : ( proximity : TGdkEventProximity );
            14 : ( client : TGdkEventClient );
            15 : ( dnd : TGdkEventDND );
         end;

     PGdkRegion = ^TGdkRegion;
     TGdkRegion = record
          user_data : gpointer;
       end;

     PGdkICAttr = ^TGdkICAttr;
     TGdkICAttr = record
          style : TGdkIMStyle;
          client_window : PGdkWindow;
          focus_window : PGdkWindow;
          filter_events : TGdkEventMask;
          spot_location : TGdkPoint;
          line_spacing : gint;
          cursor : PGdkCursor;
          preedit_fontset : PGdkFont;
          preedit_area : TGdkRectangle;
          preedit_area_needed : TGdkRectangle;
          preedit_foreground : TGdkColor;
          preedit_background : TGdkColor;
          preedit_pixmap : PGdkPixmap;
          preedit_colormap : PGdkColormap;
          status_fontset : PGdkFont;
          status_area : TGdkRectangle;
          status_area_needed : TGdkRectangle;
          status_foreground : TGdkColor;
          status_background : TGdkColor;
          status_pixmap : PGdkPixmap;
          status_colormap : PGdkColormap;
       end;

{$endif read_interface}

