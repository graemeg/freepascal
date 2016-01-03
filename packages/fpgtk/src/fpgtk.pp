{$mode objfpc}{$h+} {$ifdef win32}{$define gtkwin}{$endif}
UNIT FPgtk;

// Generated with GtkWrite by Luk Vandelaer (version 1.08)

INTERFACE

USES classes, sysutils, gtk, gdk, glib, FPglib;

TYPE

  TFPgtkObject = class;
  TFPgtkData = class;
  TFPgtkAdjustment = class;
  TFPgtkToolTips = class;
  TFPgtkWidget = class;
  TFPgtkGroup = class;
  TFPgtkWidgetGroup = class;
  TFPgtkMisc = class;
  TFPgtkLabel = class;
  TFPgtkAccelLabel = class;
  TFPgtkTipsQuery = class;
  TFPgtkArrow = class;
  TFPgtkImage = class;
  TFPgtkPixmap = class;
  TFPgtkContainer = class;
  TFPgtkBin = class;
  TFPgtkAlignment = class;
  TFPgtkFrame = class;
  TFPgtkAspectFrame = class;
  TFPgtkButton = class;
  TFPgtkToggleButton = class;
  TFPgtkCheckButton = class;
  TFPgtkRadioButton = class;
  TFPgtkRadioButtonGroup = class;
  TFPgtkOptionMenu = class;
  TFPgtkItem = class;
  TFPgtkItemGroup = class;
  TFPgtkMenuItem = class;
  TFPgtkCheckMenuItem = class;
  TFPgtkRadioMenuItem = class;
  TFPgtkRadioMenuGroup = class;
  TFPgtkTearOffMenuItem = class;
  TFPgtkListItem = class;
  TFPgtkListItemGroup = class;
  TFPgtkTreeItem = class;
  TFPgtkWindow = class;
  TFPgtkColorSelectionDialog = class;
  TFPgtkDialog = class;
  TFPgtkInputDialog = class;
  TFPgtkFileSelection = class;
  TFPgtkFontSelectionDialog = class;
  TFPgtkEventBox = class;
  TFPgtkHandleBox = class;
  TFPgtkScrolledWindow = class;
  TFPgtkViewport = class;
  TFPgtkBox = class;
  TFPgtkButtonBox = class;
  TFPgtkHButtonBox = class;
  TFPgtkVButtonBox = class;
  TFPgtkVBox = class;
  TFPgtkColorSelection = class;
  TFPgtkGammaCurve = class;
  TFPgtkHBox = class;
  TFPgtkCombo = class;
  TFPgtkStatusbar = class;
  TFPgtkCList = class;
  TFPgtkCTree = class;
  TFPgtkFixed = class;
  TFPgtkNotebook = class;
  TFPgtkFontSelection = class;
  TFPgtkPaned = class;
  TFPgtkHPaned = class;
  TFPgtkVPaned = class;
  TFPgtkLayout = class;
  TFPgtkList = class;
  TFPgtkMenuShell = class;
  TFPgtkMenuBar = class;
  TFPgtkMenu = class;
  TFPgtkPacker = class;
  TFPgtkTable = class;
  TFPgtkToolbar = class;
  TFPgtkTree = class;
  TFPgtkCalendar = class;
  TFPgtkDrawingArea = class;
  TFPgtkCurve = class;
  TFPgtkEditable = class;
  TFPgtkEntry = class;
  TFPgtkSpinButton = class;
  TFPgtkText = class;
  TFPgtkRuler = class;
  TFPgtkHRuler = class;
  TFPgtkVRuler = class;
  TFPgtkRange = class;
  TFPgtkScale = class;
  TFPgtkHScale = class;
  TFPgtkVScale = class;
  TFPgtkScrollbar = class;
  TFPgtkHScrollbar = class;
  TFPgtkVScrollbar = class;
  TFPgtkSeparator = class;
  TFPgtkHSeparator = class;
  TFPgtkVSeparator = class;
  TFPgtkPreview = class;
  TFPgtkProgress = class;
  TFPgtkProgressBar = class;
  TFPgtkItemFactory = class;

  TFPgtkSignalFunction = procedure (Sender:TFPgtkObject; Data:pointer) of Object;
  TFPgtkBooleanSignalFunction = procedure (Sender:TFPgtkObject; Bool:boolean; data:pointer) of Object;
  FPgtkException = class (Exception) end;
  PPascalClassData = ^TPascalClassData;
  TPascalClassData = record
    TheInstance : TFPgtkObject;
  end;
  PSignalData = ^TSignalData;
  TSignalData = record
    TheData : pointer;
    TheWidget : TFPgtkObject;
    TheSignalProc : TFPgtkSignalFunction;
  end;
  TDestroyState = (dsAlive, dsWaiting, dsDestroying);
  TFPgtkObjectClass = Class of TFPgtkObject;

  PFPgtkObject = ^TFPgtkObject;
  TFPgtkObject = class
  Private
    FDestroying : TDestroyState;
    PascalInstance:TPascalClassData;
    ConvertDatas:TStringList;
    SignalDatas:TList;
    NotifyList:TList;
    function ConvertSignalData (proc:TFPgtkSignalFunction; data:pointer; FreeIt:boolean) : PSignalData;
    procedure FreeClass (Sender:TFPgtkObject; Data:pointer);
    procedure CheckConvertDatas;
    procedure CheckNotifyList;
    procedure InitCreate;
    procedure FinalCreate;
    function GetUserData : pointer;
    procedure SetUserData (TheValue : pointer);
  Protected
    FGtkObject:PGtkObject;
    procedure CreateGtkObject; Virtual; Abstract;
    procedure NotifyDestroy (AnObject:TFPgtkObject); Virtual;
  Public
    function TheGtkObject : PGtkObject;
    function SignalConnect (Signal:string; Proc:TFPgtkSignalFunction; data:pointer) : guint;
    function SignalConnectAfter (Signal:string; Proc:TFPgtkSignalFunction; data:pointer) : guint;
    function BooleanSignalConnect (Signal:string; Proc:TFPgtkBooleanSignalFunction; data:pointer) : guint;
    function BooleanSignalConnectAfter (Signal:string; Proc:TFPgtkBooleanSignalFunction; data:pointer) : guint;
    constructor Create;
    constructor CreateFromObject (GtkObject:PGtkObject);
    property Destroying : TDestroyState read FDestroying;
    procedure AskNotification (AnObject:TFPgtkObject);
    destructor Destroy; Override;
    function ConnectDestroy (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectAfterDestroy (proc:TFPgtkSignalFunction; data:pointer) : guint;
    procedure SignalDisconnect (SignalHandler:guint);
    procedure SignalBlockHandler (SignalHandler:guint);
    procedure SignalUnblockHandler (SignalHandler:guint);
    procedure SignalEmit (aName:string; Args:array of const);
    function SignalNEmissions (aName:string) : guint;
    procedure SignalEmitStop (aName:string);
    procedure SetData (Key:string; Data:pointer);
    property UserData : pointer read GetUserData write SetUserData;
    procedure SetDataFull (Key:string; Data:pointer; Destroyer:TFPgtkSignalFunction);
    procedure RemoveData (Key:string);
    function GetData (Key:string) : pointer;
    function GtkDestroyed : boolean;
    procedure Constructed;
    procedure ConstructedDefault;
    procedure Sink;
    procedure Ref;
    procedure Unref;
    procedure WeakRef (Notify:TFPgtkSignalFunction; data:pointer);
    procedure WeakUnref (notify:TFPgtkSignalFunction; data:pointer);
  end;


  TFPgtkData = class (TFPgtkObject)
  Public
    function TheGtkObject : PGtkData;
    function ConnectDisconnect (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectAfterDisconnect (proc:TFPgtkSignalFunction; data:pointer) : guint;
  end;


  TFPgtkAdjustment = class (TFPgtkData)
  Private
    function GetValue : gfloat;
    procedure SetValue (TheValue : gfloat);
    function GetLower : gfloat;
    procedure SetLower (TheValue : gfloat);
    function GetUpper : gfloat;
    procedure SetUpper (TheValue : gfloat);
    function GetStepIncrement : gfloat;
    procedure SetStepIncrement (TheValue : gfloat);
    function GetPageIncrement : gfloat;
    procedure SetPageIncrement (TheValue : gfloat);
    function GetPageSize : gfloat;
    procedure SetPageSize (TheValue : gfloat);
  Protected
    procedure CreateGtkObject; override;
  Public
    function TheGtkObject : PGtkAdjustment;
    procedure Configure (aLower:gfloat; anUpper:gfloat; aValue:gfloat; aStepInc:gfloat; aPageInc:gfloat; aPageSize:gfloat);
    function ConnectValueChanged (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectAfterValueChanged (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectChanged (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectAfterChanged (proc:TFPgtkSignalFunction; data:pointer) : guint;
    procedure ValueChanged;
    procedure Changed;
    procedure ClampPage (aLower:gfloat; aUpper:gfloat);
    property Value : gfloat read GetValue write SetValue;
    property Lower : gfloat read GetLower write SetLower;
    property Upper : gfloat read GetUpper write SetUpper;
    property StepIncrement : gfloat read GetStepIncrement write SetStepIncrement;
    property PageIncrement : gfloat read GetPageIncrement write SetPageIncrement;
    property PageSize : gfloat read GetPageSize write SetPageSize;
  end;


  TFPgtkToolTips = class (TFPgtkData)
  Private
    function GetEnabled : boolean;
    procedure SetEnabled (TheValue : boolean);
    function GetDelay : integer;
    procedure SetDelay (TheValue : integer);
    function GetColorForeground : PGdkColor;
    procedure SetColorForeground (TheValue : PGdkColor);
    function GetColorBackground : PGdkColor;
    procedure SetColorBackground (TheValue : PGdkColor);
  Protected
    procedure CreateGtkObject; override;
  Public
    function TheGtkObject : PGtkToolTips;
    procedure SetColors (Fore:PGdkColor; Back:PGdkColor);
    procedure SetTip (Widget:TFPgtkWidget; TipText:string; TipPrivate:string);
    property Enabled : boolean read GetEnabled write SetEnabled;
    property Delay : integer read GetDelay write SetDelay;
    property ColorForeground : PGdkColor read GetColorForeground write SetColorForeground;
    property ColorBackground : PGdkColor read GetColorBackground write SetColorBackground;
    procedure ForceWindow;
  end;

  TFPgtkWidgetSignalFunction = procedure (Sender:TFPgtkObject; Widget:TFPgtkWidget; Data:pointer) of Object;
  TFPgtkEventFunction = function (Sender:TFPgtkWidget; Event:PGdkEvent; data:pointer): boolean of Object;
  TFPgtkEventButtonFunction = function (Sender:TFPgtkWidget; Event:PGdkEventButton; data:pointer): boolean of Object;
  TFPgtkEventMotionFunction = function (Sender:TFPgtkWidget; Event:PGdkEventMotion; data:pointer): boolean of Object;
  TFPgtkEventExposeFunction = function (Sender:TFPgtkWidget; Event:PGdkEventExpose; data:pointer): boolean of Object;
  TFPgtkEventKeyFunction = function (Sender:TFPgtkWidget; Event:PGdkEventKey; data:pointer): boolean of Object;
  TFPgtkEventCrossingFunction = function (Sender:TFPgtkWidget; Event:PGdkEventCrossing; data:pointer): boolean of Object;
  TFPgtkEventConfigureFunction = function (Sender:TFPgtkWidget; Event:PGdkEventConfigure; data:pointer): boolean of Object;
  TFPgtkEventFocusFunction = function (Sender:TFPgtkWidget; Event:PGdkEventFocus; data:pointer): boolean of Object;
  TFPgtkEventPropertyFunction = function (Sender:TFPgtkWidget; Event:PGdkEventProperty; data:pointer): boolean of Object;
  TFPgtkEventSelectionFunction = function (Sender:TFPgtkWidget; Event:PGdkEventSelection; data:pointer): boolean of Object;
  TFPgtkEventProximityFunction = function (Sender:TFPgtkWidget; Event:PGdkEventProximity; data:pointer): boolean of Object;
  TFPgtkEventClientFunction = function (Sender:TFPgtkWidget; Event:PGdkEventClient; data:pointer): boolean of Object;
  TFPgtkEventNoExposeFunction = function (Sender:TFPgtkWidget; Event:PGdkEventNoExpose; data:pointer): boolean of Object;

  TFPgtkWidget = class (TFPgtkObject)
  Private
    function GetTheGtkWidget : PGtkWidget;
    procedure SetTheGtkWidget (TheValue : PGtkWidget);
    function GetAllocation : TGtkAllocation;
    function GetName : string;
    procedure SetName (TheValue : string);
    function GetPropFlags : longint;
    procedure SetPropFlags (TheValue : longint);
    function GetState : longint;
    function GetSavedState : longint;
    function GetParent : TFPgtkWidget;
    procedure SetParent (TheValue : TFPgtkWidget);
    function GetParentWindow : PGdkWindow;
    procedure SetParentWindow (TheValue : PGdkWindow);
    procedure Reparent (NewParent:TFPgtkWidget);
    function GetVisible : boolean;
    procedure SetVisible (TheValue : boolean);
    function GetNoWindow : boolean;
    procedure SetNoWindow (TheValue : boolean);
    function GetRealized : boolean;
    procedure SetRealized (TheValue : boolean);
    function GetMapped : boolean;
    procedure SetMapped (TheValue : boolean);
    function GetDrawable : boolean;
    function GetIsSensitive : boolean;
    function GetSensitive : boolean;
    procedure SetSensitive (TheValue : boolean);
    function GetParentSensitive : boolean;
    procedure SetParentSensitive (TheValue : boolean);
    function GetAppPaintable : boolean;
    function GetCanFocus : boolean;
    procedure SetCanFocus (TheValue : boolean);
    function GetHasFocus : boolean;
    function GetCanDefault : boolean;
    procedure SetCanDefault (TheValue : boolean);
    function GetHasDefault : boolean;
    function GetReceivesDefault : boolean;
    function GetCompositeChild : boolean;
    function GetTooltip : string;
    procedure SetTooltip (TheValue : string);
    function GetColormap : PGdkColormap;
    procedure SetColormap (TheValue : PGdkColormap);
  Protected
    procedure SetFlags (NewFlags:longint);
    procedure UnsetFlags (NewUnsetFlags:longint);
    procedure Map;
    procedure Unmap;
    procedure QueueDraw;
    procedure QueueResize;
    procedure DrawFocus;
    procedure DrawDefault;
  Public
    function TheGtkObject : PGtkWidget;
    property TheGtkWidget : PGtkWidget read GetTheGtkWidget write SetTheGtkWidget;
    function WidgetSignalConnect (Signal:string; Proc:TFPgtkWidgetSignalFunction; data:pointer) : guint;
    function WidgetSignalConnectAfter (Signal:string; Proc:TFPgtkWidgetSignalFunction; data:pointer) : guint;
    procedure Draw (Rectangle:PGdkRectangle); Overload;
    procedure Show;
    procedure Hide;
    procedure Realize;
    procedure Unrealize;
    procedure ShowNow;
    procedure ShowAll;
    procedure HideAll;
    procedure SetAllocation (AnAllocation:TGtkAllocation); Overload;
    procedure SetAllocation (x:integer; y:integer; width:integer; height:integer); Overload;
    property Allocation : TGtkAllocation read GetAllocation write SetAllocation;
    procedure SetUPosition (x:integer; y:integer);
    procedure SetUsize (width:integer; height:integer);
    property Name : string read GetName write SetName;
    property Flags : longint read GetPropFlags write SetPropFlags;
    property State : longint read GetState;
    property SavedState : longint read GetSavedState;
    property Parent : TFPgtkWidget read GetParent write SetParent;
    property ParentWindow : PGdkWindow read GetParentWindow write SetParentWindow;
    procedure Unparent;
    property Visible : boolean read GetVisible write SetVisible;
    property NoWindow : boolean read GetNoWindow write SetNoWindow;
    property Realized : boolean read GetRealized write SetRealized;
    property Mapped : boolean read GetMapped write SetMapped;
    property Drawable : boolean read GetDrawable;
    property IsSensitive : boolean read GetIsSensitive;
    property Sensitive : boolean read GetSensitive write SetSensitive;
    property ParentSensitive : boolean read GetParentSensitive write SetParentSensitive;
    property AppPaintable : boolean read GetAppPaintable;
    property CanFocus : boolean read GetCanFocus write SetCanFocus;
    procedure GrabFocus;
    property HasFocus : boolean read GetHasFocus;
    property CanDefault : boolean read GetCanDefault write SetCanDefault;
    procedure GrabDefault;
    property HasDefault : boolean read GetHasDefault;
    property ReceivesDefault : boolean read GetReceivesDefault;
    property CompositeChild : boolean read GetCompositeChild;
    property Tooltip : string read GetTooltip write SetTooltip;
    procedure HideOnDelete;
    property Colormap : PGdkColormap read GetColormap write SetColormap;
    function ConnectShow (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectAfterShow (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function Connecthide (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectAfterhide (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function Connectmap (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectAftermap (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function Connectunmap (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectAfterunmap (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function Connectrealize (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectAfterrealize (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function Connectunrealize (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectAfterunrealize (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectDrawFocus (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectAfterDrawFocus (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectDrawDefault (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectAfterDrawDefault (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectParentSet (proc:TFPgtkWidgetSignalFunction; data:pointer) : guint;
    function ConnectAfterParentSet (proc:TFPgtkWidgetSignalFunction; data:pointer) : guint;
    function ConnectGrabFocus (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectAfterGrabFocus (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function EventConnect (Signal:string; Proc:TFPgtkEventFunction; data:pointer) : guint;
    function EventConnectAfter (Signal:string; Proc:TFPgtkEventFunction; data:pointer) : guint;
    function ConnectEvent (proc:TFPgtkEventFunction; data:pointer) : guint;
    function ConnectAfterEvent (proc:TFPgtkEventFunction; data:pointer) : guint;
    function EventButtonConnect (Signal:string; Proc:TFPgtkEventButtonFunction; data:pointer) : guint;
    function EventButtonConnectAfter (Signal:string; Proc:TFPgtkEventButtonFunction; data:pointer) : guint;
    function ConnectButtonPressEvent (proc:TFPgtkEventButtonFunction; data:pointer) : guint;
    function ConnectAfterButtonPressEvent (proc:TFPgtkEventButtonFunction; data:pointer) : guint;
    function ConnectButtonReleaseEvent (proc:TFPgtkEventButtonFunction; data:pointer) : guint;
    function ConnectAfterButtonReleaseEvent (proc:TFPgtkEventButtonFunction; data:pointer) : guint;
    function EventMotionConnect (Signal:string; Proc:TFPgtkEventMotionFunction; data:pointer) : guint;
    function EventMotionConnectAfter (Signal:string; Proc:TFPgtkEventMotionFunction; data:pointer) : guint;
    function ConnectMotionNotifyEvent (proc:TFPgtkEventMotionFunction; data:pointer) : guint;
    function ConnectAfterMotionNotifyEvent (proc:TFPgtkEventMotionFunction; data:pointer) : guint;
    function ConnectDeleteEvent (proc:TFPgtkEventFunction; data:pointer) : guint;
    function ConnectAfterDeleteEvent (proc:TFPgtkEventFunction; data:pointer) : guint;
    function ConnectDestroyEvent (proc:TFPgtkEventFunction; data:pointer) : guint;
    function ConnectAfterDestroyEvent (proc:TFPgtkEventFunction; data:pointer) : guint;
    function EventExposeConnect (Signal:string; Proc:TFPgtkEventExposeFunction; data:pointer) : guint;
    function EventExposeConnectAfter (Signal:string; Proc:TFPgtkEventExposeFunction; data:pointer) : guint;
    function ConnectExposeEvent (proc:TFPgtkEventExposeFunction; data:pointer) : guint;
    function ConnectAfterExposeEvent (proc:TFPgtkEventExposeFunction; data:pointer) : guint;
    function EventKeyConnect (Signal:string; Proc:TFPgtkEventKeyFunction; data:pointer) : guint;
    function EventKeyConnectAfter (Signal:string; Proc:TFPgtkEventKeyFunction; data:pointer) : guint;
    function ConnectKeyPressEvent (proc:TFPgtkEventKeyFunction; data:pointer) : guint;
    function ConnectAfterKeyPressEvent (proc:TFPgtkEventKeyFunction; data:pointer) : guint;
    function ConnectKeyReleaseEvent (proc:TFPgtkEventKeyFunction; data:pointer) : guint;
    function ConnectAfterKeyReleaseEvent (proc:TFPgtkEventKeyFunction; data:pointer) : guint;
    function EventCrossingConnect (Signal:string; Proc:TFPgtkEventCrossingFunction; data:pointer) : guint;
    function EventCrossingConnectAfter (Signal:string; Proc:TFPgtkEventCrossingFunction; data:pointer) : guint;
    function ConnectEnterNotifyEvent (proc:TFPgtkEventCrossingFunction; data:pointer) : guint;
    function ConnectAfterEnterNotifyEvent (proc:TFPgtkEventCrossingFunction; data:pointer) : guint;
    function ConnectLeaveNotifyEvent (proc:TFPgtkEventCrossingFunction; data:pointer) : guint;
    function ConnectAfterLeaveNotifyEvent (proc:TFPgtkEventCrossingFunction; data:pointer) : guint;
    function EventConfigureConnect (Signal:string; Proc:TFPgtkEventConfigureFunction; data:pointer) : guint;
    function EventConfigureConnectAfter (Signal:string; Proc:TFPgtkEventConfigureFunction; data:pointer) : guint;
    function ConnectConfigureEvent (proc:TFPgtkEventConfigureFunction; data:pointer) : guint;
    function ConnectAfterConfigureEvent (proc:TFPgtkEventConfigureFunction; data:pointer) : guint;
    function EventFocusConnect (Signal:string; Proc:TFPgtkEventFocusFunction; data:pointer) : guint;
    function EventFocusConnectAfter (Signal:string; Proc:TFPgtkEventFocusFunction; data:pointer) : guint;
    function ConnectFocusInEvent (proc:TFPgtkEventFocusFunction; data:pointer) : guint;
    function ConnectAfterFocusInEvent (proc:TFPgtkEventFocusFunction; data:pointer) : guint;
    function ConnectFocusOutEvent (proc:TFPgtkEventFocusFunction; data:pointer) : guint;
    function ConnectAfterFocusOutEvent (proc:TFPgtkEventFocusFunction; data:pointer) : guint;
    function ConnectMapEvent (proc:TFPgtkEventFunction; data:pointer) : guint;
    function ConnectAfterMapEvent (proc:TFPgtkEventFunction; data:pointer) : guint;
    function ConnectUnmapEvent (proc:TFPgtkEventFunction; data:pointer) : guint;
    function ConnectAfterUnmapEvent (proc:TFPgtkEventFunction; data:pointer) : guint;
    function EventPropertyConnect (Signal:string; Proc:TFPgtkEventPropertyFunction; data:pointer) : guint;
    function EventPropertyConnectAfter (Signal:string; Proc:TFPgtkEventPropertyFunction; data:pointer) : guint;
    function ConnectPropertyNotifyEvent (proc:TFPgtkEventPropertyFunction; data:pointer) : guint;
    function ConnectAfterPropertyNotifyEvent (proc:TFPgtkEventPropertyFunction; data:pointer) : guint;
    function EventSelectionConnect (Signal:string; Proc:TFPgtkEventSelectionFunction; data:pointer) : guint;
    function EventSelectionConnectAfter (Signal:string; Proc:TFPgtkEventSelectionFunction; data:pointer) : guint;
    function ConnectSelectionClearEvent (proc:TFPgtkEventSelectionFunction; data:pointer) : guint;
    function ConnectAfterSelectionClearEvent (proc:TFPgtkEventSelectionFunction; data:pointer) : guint;
    function ConnectSelectionRequestEvent (proc:TFPgtkEventSelectionFunction; data:pointer) : guint;
    function ConnectAfterSelectionRequestEvent (proc:TFPgtkEventSelectionFunction; data:pointer) : guint;
    function ConnectSelectionNotifyEvent (proc:TFPgtkEventSelectionFunction; data:pointer) : guint;
    function ConnectAfterSelectionNotifyEvent (proc:TFPgtkEventSelectionFunction; data:pointer) : guint;
    function EventProximityConnect (Signal:string; Proc:TFPgtkEventProximityFunction; data:pointer) : guint;
    function EventProximityConnectAfter (Signal:string; Proc:TFPgtkEventProximityFunction; data:pointer) : guint;
    function ConnectProximityInEvent (proc:TFPgtkEventProximityFunction; data:pointer) : guint;
    function ConnectAfterProximityInEvent (proc:TFPgtkEventProximityFunction; data:pointer) : guint;
    function ConnectProximityOutEvent (proc:TFPgtkEventProximityFunction; data:pointer) : guint;
    function ConnectAfterProximityOutEvent (proc:TFPgtkEventProximityFunction; data:pointer) : guint;
    function EventClientConnect (Signal:string; Proc:TFPgtkEventClientFunction; data:pointer) : guint;
    function EventClientConnectAfter (Signal:string; Proc:TFPgtkEventClientFunction; data:pointer) : guint;
    function ConnectClientEvent (proc:TFPgtkEventClientFunction; data:pointer) : guint;
    function ConnectAfterClientEvent (proc:TFPgtkEventClientFunction; data:pointer) : guint;
    function EventNoExposeConnect (Signal:string; Proc:TFPgtkEventNoExposeFunction; data:pointer) : guint;
    function EventNoExposeConnectAfter (Signal:string; Proc:TFPgtkEventNoExposeFunction; data:pointer) : guint;
    function ConnectNoExposeEvent (proc:TFPgtkEventNoExposeFunction; data:pointer) : guint;
    function ConnectAfterNoExposeEvent (proc:TFPgtkEventNoExposeFunction; data:pointer) : guint;
    function ConnectVisibilityNotifyEvent (proc:TFPgtkEventFunction; data:pointer) : guint;
    function ConnectAfterVisibilityNotifyEvent (proc:TFPgtkEventFunction; data:pointer) : guint;
    procedure LockAccelerators;
    procedure UnlockAccelerators;
    procedure RemoveAccelerators (aSignal:string; OnlyVisible:boolean);
    procedure ActivateAccelGroups (Key:guint; Mods:TGdkModifierType);
    procedure AcceleratorAdd (AG:PGtkAccelGroup; aSignal:string; Key:guint; Mods:TGdkModifierType; acFlags:TGtkAccelFlags); Overload;
  end;

  TFPgtkForEachProcedure = procedure (item : pointer; data : pointer) of object;

  TFPgtkGroup = class (TList)
  Private
    FManageLists : boolean;
    FListChanged:boolean;
    FSListChanged:boolean;
    FClassesChanged:boolean;
    FNotUpdating:boolean;
    FGList:PGList;
    FGSList:PGSList;
    procedure FreeList;
    procedure FreeSList;
    function CreateGList : PGList;
    function CreateGSList : PGSList;
    function GetGtkListProp : PGList;
    procedure SetGtkListProp (TheValue : PGList);
    function GetGtkSListProp : PGSList;
    procedure SetGtkSListProp (TheValue : PGSList);
  Protected
    procedure BuildFromGtkList;
    procedure BuildFromGtkSList;
    procedure Notify (ptr:pointer; Action:TListNotification); Override;
    function GetData (index:integer) : pointer; Dynamic;
    function UngetData (data:pointer) : pointer; Dynamic;
  Public
    property ManageLists : boolean read FManageLists write FManageLists;
    constructor Create;
    destructor Destroy; Override;
    function GetGtkList (buffered:boolean) : PGList;
    function GetGtkSList (buffered:boolean) : PGSList;
    procedure BeginUpdate;
    procedure EndUpdate;
    procedure ForEach (Proc:TFPgtkForEachProcedure; data:pointer);
    property GtkList : PGList read GetGtkListProp write SetGtkListProp;
    property GtkSList : PGSList read GetGtkSListProp write SetGtkSListProp;
  end;


  TFPgtkWidgetGroup = class (TFPgtkGroup)
  Private
    function GetItem(Index:integer) : TFPgtkWidget;
    procedure SetItem (Index:integer; TheValue : TFPgtkWidget);
    function GetTooltips(index:integer) : string;
    procedure SetTooltips (index:integer; TheValue : string);
  Public
    function GetData (index:integer) : pointer; Override;
    function UnGetData (data:pointer) : pointer; Override;
    procedure AddToContainer (Container:TFPgtkContainer);
    procedure PackInBox (box:TFPgtkBox; AtStart:boolean; Expanding:boolean; Fill:boolean; Padding:integer);
    property Items [Index:integer]  : TFPgtkWidget read GetItem write SetItem;
    property Tooltips [index:integer]  : string read GetTooltips write SetTooltips;
  end;


  TFPgtkMisc = class (TFPgtkWidget)
  Private
    function GetXAlign : gfloat;
    procedure SetXAlign (TheValue : gfloat);
    function GetYAlign : gfloat;
    procedure SetYAlign (TheValue : gfloat);
    function GetXPad : word;
    procedure SetXPad (TheValue : word);
    function GetYPad : word;
    procedure SetYPad (TheValue : word);
  Public
    function TheGtkObject : PGtkMisc;
    procedure SetAlignment (x:gfloat; y:gfloat);
    procedure SetPadding (x:word; y:word);
    property XAlign : gfloat read GetXAlign write SetXAlign;
    property YAlign : gfloat read GetYAlign write SetYAlign;
    property XPad : word read GetXPad write SetXPad;
    property YPad : word read GetYPad write SetYPad;
  end;

  TFPgtkLabelClass = class of TFPgtkLabel;

  TFPgtkLabel = class (TFPgtkMisc)
  Private
    function GetText : string;
    procedure SetText (TheValue : string);
    function GetPattern : string;
    procedure SetPattern (TheValue : string);
    function GetJustify : TGtkJustification;
    procedure SetJustify (TheValue : TGtkJustification);
    function GetLineWrap : boolean;
    procedure SetLineWrap (TheValue : boolean);
  Protected
    procedure CreateGtkObject; override;
  Public
    function TheGtkObject : PGtkLabel;
    constructor Create (aText:string);
    property Text : string read GetText write SetText;
    property Pattern : string read GetPattern write SetPattern;
    property Justify : TGtkJustification read GetJustify write SetJustify;
    property LineWrap : boolean read GetLineWrap write SetLineWrap;
    function ParseUline (aText:string) : guint;
  end;


  TFPgtkAccelLabel = class (TFPgtkLabel)
  Private
    function GetAccelWidget : TFPgtkWidget;
    procedure SetAccelWidget (TheValue : TFPgtkWidget);
  Protected
    procedure CreateGtkObject; override;
  Public
    function TheGtkObject : PGtkAccelLabel;
    property AccelWidget : TFPgtkWidget read GetAccelWidget write SetAccelWidget;
    function AccelText : string;
    procedure Refetch;
  end;


  TFPgtkTipsQuery = class (TFPgtkLabel)
  Protected
    procedure CreateGtkObject; override;
  Public
    function TheGtkObject : PGtkTipsQuery;
  end;


  TFPgtkArrow = class (TFPgtkMisc)
  Private
    function GetArrowType : TGtkArrowType;
    procedure SetArrowType (TheValue : TGtkArrowType);
    function GetShadowType : TGtkShadowType;
    procedure SetShadowType (TheValue : TGtkShadowType);
  Protected
    procedure CreateGtkObject; override;
  Public
    function TheGtkObject : PGtkArrow;
    property ArrowType : TGtkArrowType read GetArrowType write SetArrowType;
    property ShadowType : TGtkShadowType read GetShadowType write SetShadowType;
    procedure SetTypes (AnArrowType:TGtkArrowType; AShadowtype:TGtkShadowType);
    constructor Create (AnArrowType:TGtkArrowType; AShadowType:TGtkShadowType);
  end;


  TFPgtkImage = class (TFPgtkMisc)
  Private
    function GetImageProp : PGdkImage;
    procedure SetImageProp (TheValue : PGdkImage);
    function GetMask : PGdkBitMap;
    procedure SetMask (TheValue : PGdkBitMap);
  Protected
    procedure CreateGtkObject; override;
  Public
    function TheGtkObject : PGtkImage;
    FMask:PGdkBitMap;
    FImage:PGdkImage;
    property Image : PGdkImage read GetImageProp write SetImageProp;
    property Mask : PGdkBitMap read GetMask write SetMask;
    procedure SetImage (anImage:PGdkImage; aMask:PGdkBitmap);
    constructor Create (anImage:PGdkImage; aMask:PGdkBitmap);
  end;

  TStringArray = array[0..32000] of pgchar;
  PStringArray = ^TStringArray;

  TFPgtkPixmap = class (TFPgtkMisc)
  Private
    function GetBuildInsensitive : longbool;
    procedure SetBuildInsensitive (TheValue : longbool);
    function GetPixmapProp : PGdkPixMap;
    procedure SetPixmapProp (TheValue : PGdkPixMap);
    function GetMask : PGdkBitMap;
    procedure SetMask (TheValue : PGdkBitMap);
  Protected
    procedure CreateGtkObject; override;
  Public
    function TheGtkObject : PGtkPixmap;
    FMask:PGdkBitMap;
    FPixMap:PGdkPixmap;
    property BuildInsensitive : longbool read GetBuildInsensitive write SetBuildInsensitive;
    constructor Create;
    constructor CreateFromFile (Filename:string; Window:TFPgtkWidget);
    constructor CreateFromStrings (Data:TStrings; Window:TFPgtkWidget);
    constructor CreateFromText (Data:string; Window:TFPgtkWidget);
    property PixMap : PGdkPixMap read GetPixmapProp write SetPixmapProp;
    property Mask : PGdkBitMap read GetMask write SetMask;
    procedure SetPixmap (aPixmap:PGdkPixMap; aMask:PGdkBitmap);
    procedure GetPixmap (var aPixmap:PGdkPixmap; var aMask:PGdkBitmap);
    procedure LoadFromFile (Filename:string);
    procedure LoadFromStrings (data:TStrings);
    procedure LoadFromText (data:string);
    procedure LoadFromArray (data:array of string);
  end;

  TFPgtkDirectionFunctionSignalFunction = function (Sender:TFPgtkObject; Direction:TGtkDirectionType; data:pointer): TGtkDirectionType of Object;

  TFPgtkContainer = class (TFPgtkWidget)
  Private
    function GetBorder : integer;
    procedure SetBorder (TheValue : integer);
    function GetChildren : TFPgtkWidgetGroup;
  Public
    function TheGtkObject : PGtkContainer;
    FChildren:TFPgtkWidgetGroup;
    property Border : integer read GetBorder write SetBorder;
    procedure Add (AWidget:TFPgtkWidget; IsVisible:boolean); Overload;
    procedure Add (AWidget:TFPgtkWidget); Overload;
    procedure Remove (AWidget:TFPgtkWidget);
    constructor Create;
    destructor Destroy; Override;
    property Children : TFPgtkWidgetGroup read GetChildren;
    procedure Focus (Direction:TGtkDirectionType);
    procedure FocusChild (Child:TFPgtkWidget);
    procedure RegisterToplevel;
    procedure UnregisterToplevel;
    procedure ResizeChildren;
    function DirectionFunctionSignalConnect (Signal:string; Proc:TFPgtkDirectionFunctionSignalFunction; data:pointer) : guint;
    function DirectionFunctionSignalConnectAfter (Signal:string; Proc:TFPgtkDirectionFunctionSignalFunction; data:pointer) : guint;
    function ConnectAdd (proc:TFPgtkWidgetSignalFunction; data:pointer) : guint;
    function ConnectAfterAdd (proc:TFPgtkWidgetSignalFunction; data:pointer) : guint;
    function ConnectRemove (proc:TFPgtkWidgetSignalFunction; data:pointer) : guint;
    function ConnectAfterRemove (proc:TFPgtkWidgetSignalFunction; data:pointer) : guint;
    function ConnectCheckResize (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectAfterCheckResize (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectFocus (proc:TFPgtkDirectionFunctionSignalFunction; data:pointer) : guint;
    function ConnectAfterFocus (proc:TFPgtkDirectionFunctionSignalFunction; data:pointer) : guint;
    function ConnectSetFocusChild (proc:TFPgtkWidgetSignalFunction; data:pointer) : guint;
    function ConnectAfterSetFocusChild (proc:TFPgtkWidgetSignalFunction; data:pointer) : guint;
  end;


  TFPgtkBin = class (TFPgtkContainer)
  Private
    function GetChild : TFPgtkWidget;
    procedure SetChild (TheValue : TFPgtkWidget);
  Protected
    property Child : TFPgtkWidget read GetChild write SetChild;
  Public
    function TheGtkObject : PGtkBin;
  end;


  TFPgtkAlignment = class (TFPgtkBin)
  Public
    function TheGtkObject : PGtkAlignment;
    procedure Configure (anXAlign:gfloat; anYAlign:gfloat; anXScale:gfloat; anYScale:gfloat);
  end;


  TFPgtkFrame = class (TFPgtkBin)
  Private
    function GetText : string;
    procedure SetText (TheValue : string);
    function GetAlignment : gfloat;
    procedure SetAlignment (TheValue : gfloat);
    function GetShadowType : TgtkShadowType;
    procedure SetShadowType (TheValue : TgtkShadowType);
  Protected
    procedure CreateGtkObject; override;
  Public
    function TheGtkObject : PGtkFrame;
    property Text : string read GetText write SetText;
    property Alignment : gfloat read GetAlignment write SetAlignment;
    property ShadowType : TgtkShadowType read GetShadowType write SetShadowType;
  end;


  TFPgtkAspectFrame = class (TFPgtkFrame)
  Protected
    procedure CreateGtkObject; override;
  Public
    function TheGtkObject : PGtkAspectFrame;
    procedure Configure (anXAlign:gfloat; anYAlign:gfloat; Ratio:gfloat; ObeyChild:longbool);
  end;


  TFPgtkButton = class (TFPgtkBin)
  Private
    FAccelKey : guint;
    FAddContainer : TFPgtkContainer;
    FLabel : TFPgtkLabel;
    procedure CreateLabel (aText:string);
    function GetText : string;
    procedure SetText (TheValue : string);
    function GetReliefStyle : TGtkReliefStyle;
    procedure SetReliefStyle (TheValue : TGtkReliefStyle);
  Protected
    procedure CreateGtkObject; override;
    function LabelClass : TFPgtkLabelClass; Virtual;
    procedure NotifyDestroy (AnObject:TFPgtkObject); Override;
    procedure LabelCreated; Virtual;
  Public
    function TheGtkObject : PGtkButton;
    function ConnectClicked (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectAfterClicked (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectPressed (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectAfterPressed (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectReleased (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectAfterReleased (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectEnter (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectAfterEnter (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectLeave (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectAfterLeave (proc:TFPgtkSignalFunction; data:pointer) : guint;
    procedure Clicked;
    procedure Pressed;
    procedure Released;
    procedure Enter;
    procedure Leave;
    constructor Create;
    constructor CreateWithLabel (aText:string); Overload;
    constructor CreateWithLabel (aText:string; AccelGroup:PGtkAccelGroup); Overload;
    property TheLabel : TFPgtkLabel read FLabel;
    property Text : string read GetText write SetText;
    property ReliefStyle : TGtkReliefStyle read GetReliefStyle write SetReliefStyle;
    property AddContainer : TFPgtkContainer read FAddContainer write FAddContainer;
    property AccelKey : guint read FAccelKey;
  end;


  TFPgtkToggleButton = class (TFPgtkButton)
  Private
    function GetActive : boolean;
    procedure SetActive (TheValue : boolean);
    function GetDrawIndicator : boolean;
    procedure SetDrawIndicator (TheValue : boolean);
  Protected
    procedure CreateGtkObject; override;
  Public
    function TheGtkObject : PGtkToggleButton;
    function ConnectToggled (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectAfterToggled (proc:TFPgtkSignalFunction; data:pointer) : guint;
    procedure Toggled;
    property Active : boolean read GetActive write SetActive;
    property DrawIndicator : boolean read GetDrawIndicator write SetDrawIndicator;
  end;


  TFPgtkCheckButton = class (TFPgtkToggleButton)
  Protected
    procedure CreateGtkObject; override;
  Public
    function TheGtkObject : PGtkCheckButton;
  end;


  TFPgtkRadioButton = class (TFPgtkCheckButton)
  Protected
    procedure CreateGtkObject; Override;
  Public
    function TheGtkObject : PGtkRadioButton;
    FGroup:TFPgtkRadioButtonGroup;
    constructor Create (AGroup:TFPgtkRadioButtonGroup);
    constructor CreateWithLabel (AGroup:TFPgtkRadioButtonGroup; aText:string);
  end;


  TFPgtkRadioButtonGroup = class (TFPgtkWidgetGroup)
  Private
    function GetItem(index:integer) : TFPgtkRadioButton;
    procedure SetItem (index:integer; TheValue : TFPgtkRadioButton);
  Public
    property Items [index:integer]  : TFPgtkRadioButton read GetItem write SetItem;
    function ActiveButtonText : string;
    function ActiveButtonIndex : integer;
    function ActiveButton : TFPgtkRadioButton;
  end;


  TFPgtkOptionMenu = class (TFPgtkButton)
  Private
    function GetMenu : TFPgtkMenu;
    procedure setmenu (TheValue : TFPgtkMenu);
  Protected
    procedure CreateGtkObject; override;
  Public
    function TheGtkObject : PGtkOptionMenu;
    property Menu : TFPgtkMenu read GetMenu write setmenu;
    procedure RemoveMenu;
    procedure SetHistory (index:integer);
    procedure Clear;
  end;

  TFPgtkItemClass = class of TFPgtkItem;

  TFPgtkItem = class (TFPgtkBin)
  Private
    FAccelKey : guint;
    FAddContainer : TFPgtkContainer;
    FLabel : TFPgtkLabel;
    procedure CreateLabel (aText:string);
    function GetText : string;
    procedure SetText (TheValue : string);
  Protected
    function LabelClass : TFPgtkLabelClass; Virtual;
    procedure NotifyDestroy (AnObject:TFPgtkObject); Override;
    procedure LabelCreated; Virtual;
  Public
    function TheGtkObject : PGtkItem;
    function ConnectSelect (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectAfterSelect (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectDeselect (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectAfterDeselect (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectToggle (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectAfterToggle (proc:TFPgtkSignalFunction; data:pointer) : guint;
    procedure Select;
    procedure Deselect;
    procedure Toggle;
    constructor Create;
    constructor CreateWithLabel (aText:string);
    property TheLabel : TFPgtkLabel read FLabel;
    property Text : string read GetText write SetText;
    property AddContainer : TFPgtkContainer read FAddContainer write FAddContainer;
    property AccelKey : guint read FAccelKey;
  end;


  TFPgtkItemGroup = class (TFPgtkWidgetGroup)
  Private
    FItemClass : TFPgtkItemClass;
    function GetItem(index:integer) : TFPgtkItem;
    procedure SetItem (index:integer; TheValue : TFPgtkItem);
  Public
    property Items [index:integer]  : TFPgtkItem read GetItem write SetItem;
    procedure FillFromList (aList:TStrings);
    procedure FillFromCommaText (aList:string);
    procedure FillFromArray (aList:array of string);
    property ItemClass : TFPgtkItemClass read FItemClass write FItemClass;
    procedure SignalConnect (Signal:string; proc:TFPgtkSignalFunction; data:pointer);
    constructor create (AnItemClass:TFPgtkItemClass);
    function AddTextItem (aText:string) : TFPgtkItem;
  end;


  TFPgtkMenuItem = class (TFPgtkItem)
  Private
    function GetPlacement : TGtkSubmenuPlacement;
    procedure SetPlacement (TheValue : TGtkSubmenuPlacement);
    function GetToggleIndicator : boolean;
    procedure SetToggleIndicator (TheValue : boolean);
    function GetSubMenuIndicator : boolean;
    procedure SetSubMenuIndicator (TheValue : boolean);
    function GetJustifyRight : boolean;
    procedure SetJustifyRight (TheValue : boolean);
    function GetSubMenu : TFPgtkMenuShell;
    procedure SetPropSubMenu (TheValue : TFPgtkMenuShell);
  Protected
    procedure CreateGtkObject; override;
    function LabelClass : TFPgtkLabelClass; Override;
    procedure LabelCreated; Override;
  Public
    function TheGtkObject : PGtkMenuItem;
    function ConnectActivate (proc:TFPgtksignalFunction; data:pointer) : guint;
    function ConnectAfterActivate (proc:TFPgtksignalFunction; data:pointer) : guint;
    function ConnectActivateItem (proc:TFPgtksignalFunction; data:pointer) : guint;
    function ConnectAfterActivateItem (proc:TFPgtksignalFunction; data:pointer) : guint;
    procedure Activate;
    procedure SetSubMenu (aSubMenu:TFPgtkWidget);
    procedure RemoveSubMenu;
    procedure Configure (ShowToggleIndicator:boolean; ShowSubmenuIndicator:boolean);
    procedure RightJustify;
    property Placement : TGtkSubmenuPlacement read GetPlacement write SetPlacement;
    property ToggleIndicator : boolean read GetToggleIndicator write SetToggleIndicator;
    property SubMenuIndicator : boolean read GetSubMenuIndicator write SetSubMenuIndicator;
    property JustifyRight : boolean read GetJustifyRight write SetJustifyRight;
    property SubMenu : TFPgtkMenuShell read GetSubMenu write SetPropSubMenu;
  end;


  TFPgtkCheckMenuItem = class (TFPgtkMenuItem)
  Private
    function GetActive : boolean;
    procedure SetActive (TheValue : boolean);
    function GetShowToggle : boolean;
    procedure SetShowToggle (TheValue : boolean);
  Protected
    procedure CreateGtkObject; override;
  Public
    function TheGtkObject : PGtkCheckMenuItem;
    function ConnectToggled (proc:TFPgtksignalFunction; data:pointer) : guint;
    function ConnectAfterToggled (proc:TFPgtksignalFunction; data:pointer) : guint;
    procedure Toggled;
    property Active : boolean read GetActive write SetActive;
    property ShowToggle : boolean read GetShowToggle write SetShowToggle;
  end;


  TFPgtkRadioMenuItem = class (TFPgtkCheckMenuItem)
  Private
    FGroup : TFPgtkRadioMenuGroup;
  Protected
    procedure CreateGtkObject; Override;
  Public
    function TheGtkObject : PGtkRadioMenuItem;
    constructor Create (AGroup:TFPgtkRadioMenuGroup);
    constructor CreateWithLabel (Agroup:TFPgtkRadioMenuGroup; aText:string);
    property Group : TFPgtkRadioMenuGroup read FGroup;
  end;


  TFPgtkRadioMenuGroup = class (TFPgtkItemGroup)
  Private
    function GetItem(index:integer) : TFPgtkRadioMenuItem;
    procedure SetItem (index:integer; TheValue : TFPgtkRadioMenuItem);
  Public
    property Items [index:integer]  : TFPgtkRadioMenuItem read GetItem write SetItem;
    function ActiveMenuText : string;
    function ActiveMenuIndex : integer;
    function ActiveMenu : TFPgtkRadioMenuItem;
    constructor create;
  end;


  TFPgtkTearOffMenuItem = class (TFPgtkMenuItem)
  Protected
    procedure CreateGtkObject; override;
  Public
    function TheGtkObject : PGtkTearOffMenuItem;
  end;

  TFPgtkScrollSignalFunction = procedure (Sender:TFPgtkObject; ScrollType:TgtkScrollType; position:gfloat; data:pointer) of Object;
  TFPgtkScrollBooleanSignalFunction = procedure (Sender:TFPgtkObject; ScrolType:TgtkScrollType; Position:gfloat; AutoStartSelection:boolean; data:pointer) of Object;

  TFPgtkListItem = class (TFPgtkItem)
  Protected
    procedure CreateGtkObject; override;
  Public
    function TheGtkObject : PGtkListItem;
    function ScrollSignalConnect (Signal:string; Proc:TFPgtkScrollSignalFunction; data:pointer) : guint;
    function ScrollSignalConnectAfter (Signal:string; Proc:TFPgtkScrollSignalFunction; data:pointer) : guint;
    function ScrollBooleanSignalConnect (Signal:string; Proc:TFPgtkScrollBooleanSignalFunction; data:pointer) : guint;
    function ScrollBooleanSignalConnectAfter (Signal:string; Proc:TFPgtkScrollBooleanSignalFunction; data:pointer) : guint;
    function ConnectToggleFocusRow (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectAfterToggleFocusRow (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectSelectAll (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectAfterSelectAll (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectUnselectAll (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectAfterUnselectAll (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectUndoSelection (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectAfterUndoSelection (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectStartSelection (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectAfterStartSelection (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectEndSelection (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectAfterEndSelection (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectToggleAddMode (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectAfterToggleAddMode (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectExtendSelection (proc:TFPgtkScrollBooleanSignalFunction; data:pointer) : guint;
    function ConnectAfterExtendSelection (proc:TFPgtkScrollBooleanSignalFunction; data:pointer) : guint;
    function ConnectScrollVertical (proc:TFPgtkScrollSignalFunction; data:pointer) : guint;
    function ConnectAfterScrollVertical (proc:TFPgtkScrollSignalFunction; data:pointer) : guint;
    function ConnectScrollHorizontal (proc:TFPgtkScrollSignalFunction; data:pointer) : guint;
    function ConnectAfterScrollHorizontal (proc:TFPgtkScrollSignalFunction; data:pointer) : guint;
    procedure Select;
    procedure Deselect;
  end;


  TFPgtkListItemGroup = class (TFPgtkItemGroup)
  Public
    constructor create;
  end;


  TFPgtkTreeItem = class (TFPgtkItem)
  Private
    function GetSubTree : TFPgtkWidget;
    procedure SetSubTree (TheValue : TFPgtkWidget);
    function GetPixPlus : TFPgtkWidget;
    function GetPixMinus : TFPgtkWidget;
    function GetExpanded : boolean;
    procedure SetExpanded (TheValue : boolean);
  Protected
    procedure CreateGtkObject; override;
  Public
    function TheGtkObject : PGtkTreeItem;
    property SubTree : TFPgtkWidget read GetSubTree write SetSubTree;
    property PixPlus : TFPgtkWidget read GetPixPlus;
    property PixMinus : TFPgtkWidget read GetPixMinus;
    property Expanded : boolean read GetExpanded write SetExpanded;
    procedure Select;
    procedure Deselect;
    procedure Expand;
    procedure Collapse;
    function ConnectCollapse (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectAfterCollapse (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectExpand (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectAfterExpand (proc:TFPgtkSignalFunction; data:pointer) : guint;
  end;

  DialogResultCallback = procedure (Sender:TFPgtkWindow; DialogResult:pointer;
                                    Action:integer; initiator:TFPgtkObject) of object;
  DialogInitCallback = procedure (Sender : TFPgtkWindow; InitData : pointer) of object;
  TFPgtkWindowClass = class of TFPgtkWindow;

  TFPgtkWindow = class (TFPgtkBin)
  Private
    FAccelGroups:TList;
    FMainLevel : guint;
    FModalAction : integer;
    FOnDialogInit : DialogInitCallback;
    FOnDialogResult : DialogResultCallback;
    FDialogResult : pointer;
    TheWindowType:TGtkWindowType;
    function GetWindowType : TGtkWindowType;
    procedure SetWindowType (TheValue : TGtkWindowType);
    function GetTitle : string;
    procedure SetTitle (TheValue : string);
    function GetModal : boolean;
    procedure SetModal (TheValue : boolean);
    procedure SetModalAction (TheValue : integer);
    procedure ExecuteEnds (Sender:TFPgtkObject; data:pointer);
    function GetUserSizable : boolean;
    procedure SetUserSizable (TheValue : boolean);
    function GetPosition : TGtkWindowPosition;
    procedure SetPosition (TheValue : TGtkWindowPosition);
    function GetAccelGroups(ID:integer) : PGtkAccelGroup;
  Protected
    procedure CreateGtkObject; override;
    property DialogResult : pointer read FDialogResult write FDialogResult;
    procedure DoDialogResult (Action:integer; Sender:TFPgtkObject); Virtual;
    procedure DoDialogInit (InitData:pointer); Virtual;
  Public
    function TheGtkObject : PGtkWindow;
    constructor Create (AType:TGtkWindowType);
    destructor Destroy; Override;
    property WindowType : TGtkWindowType read GetWindowType write SetWindowType;
    property Title : string read GetTitle write SetTitle;
    property Modal : boolean read GetModal write SetModal;
    property OnDialogResult : DialogResultCallback read FOnDialogResult write FOnDialogResult;
    property OnDialogInit : DialogInitCallback read FOnDialogInit write FOnDialogInit;
    procedure Close;
    procedure CloseWindow (Sender:TFPgtkObject; data:pointer);
    procedure CloseWithResult (Sender:TFPgtkObject; data:pointer);
    property ModalAction : integer read FModalAction write SetModalAction;
    property MainLevel : guint read FMainLevel;
    function Execute (anOnDialogInit:DialogInitCallBack; anInitData:pointer; anOnDialogResult:DialogResultCallBack) : integer;
    function ConnectSetFocus (proc:TFPgtkWidgetSignalFunction; data:pointer) : guint;
    function ConnectAfterSetFocus (proc:TFPgtkWidgetSignalFunction; data:pointer) : guint;
    procedure SetTransientFor (aParent:TFPgtkWindow);
    procedure DefaultWidget (Widget:TFPgtkWidget);
    procedure FocusedWidget (NewFocus:TFPgtkWidget);
    property UserSizable : boolean read GetUserSizable write SetUserSizable;
    procedure ActivateFocus;
    procedure ActivateDefault;
    procedure SetDefaultSize (Width:gint; Height:gint);
    property Position : TGtkWindowPosition read GetPosition write SetPosition;
    property AccelGroups [ID:integer]  : PGtkAccelGroup read GetAccelGroups;
    function AccelGroupNew : integer;
    procedure AccelGroupDelete (ID:integer);
    procedure AcceleratorAdd (AG:integer; aWidget:TFPgtkWidget; aSignal:string; Key:guint; Mods:TGdkModifierType; acFlags:TGtkAccelFlags); Overload;
    procedure AcceleratorRemove (AG:integer; aWidget:TFPgtkWidget; Key:guint; Mods:TGdkModifierType); Overload;
    procedure AccelGroupLock (AG:integer);
    procedure AccelGroupUnlock (AG:integer);
    procedure AccelGroupActivate (AG:integer; Key:guint; Mods:TGdkModifierType);
  end;


  TFPgtkColorSelectionDialog = class (TFPgtkWindow)
  Private
    function GetColorSel : TFPgtkColorSelection;
    function GetButtonOK : TFPgtkButton;
    function GetButtonCancel : TFPgtkButton;
    function GetButtonHelp : TFPgtkButton;
  Protected
    procedure CreateGtkObject; override;
  Public
    function TheGtkObject : PGtkColorSelectionDialog;
    property ColorSel : TFPgtkColorSelection read GetColorSel;
    property ButtonOK : TFPgtkButton read GetButtonOK;
    property ButtonCancel : TFPgtkButton read GetButtonCancel;
    property ButtonHelp : TFPgtkButton read GetButtonHelp;
  end;


  TFPgtkDialog = class (TFPgtkWindow)
  Private
    function GetActionArea : TFPgtkHBox;
    function GetVBox : TFPgtkVBox;
  Protected
    procedure CreateGtkObject; override;
  Public
    function TheGtkObject : PGtkDialog;
    property ActionArea : TFPgtkHBox read GetActionArea;
    property VBox : TFPgtkVBox read GetVBox;
    constructor create;
  end;

  TFPgtkDeviceSignalFunction = procedure (Sender:TFPgtkInputDialog; DeviceID:integer; Data:pointer) of Object;

  TFPgtkInputDialog = class (TFPgtkDialog)
  Private
    function GetButtonClose : TFPgtkButton;
    function GetButtonSave : TFPgtkButton;
  Protected
    procedure CreateGtkObject; override;
  Public
    function TheGtkObject : PGtkInputDialog;
    property ButtonClose : TFPgtkButton read GetButtonClose;
    property ButtonSave : TFPgtkButton read GetButtonSave;
    function DeviceSignalConnect (Signal:string; Proc:TFPgtkDeviceSignalFunction; data:pointer) : guint;
    function DeviceSignalConnectAfter (Signal:string; Proc:TFPgtkDeviceSignalFunction; data:pointer) : guint;
    function ConnectEnableDevice (proc:TFPgtkDeviceSignalFunction; data:pointer) : guint;
    function ConnectAfterEnableDevice (proc:TFPgtkDeviceSignalFunction; data:pointer) : guint;
    function ConnectDisableDevice (proc:TFPgtkDeviceSignalFunction; data:pointer) : guint;
    function ConnectAfterDisableDevice (proc:TFPgtkDeviceSignalFunction; data:pointer) : guint;
  end;


  TFPgtkFileSelection = class (TFPgtkWindow)
  Private
    function GetFilename : string;
    procedure SetFilename (TheValue : string);
    function GetDirList : TFPgtkCList;
    function GetFileList : TFPgtkCList;
    function GetOkButton : TFPgtkButton;
    function GetCancelButton : TFPgtkButton;
    function GetHistoryPulldown : TFPgtkOptionMenu;
    function GetFileOpDialog : TFPgtkDialog;
    function GetFileOpCreateDir : TFPgtkButton;
    function GetFileOpDelFile : TFPgtkButton;
    function GetFileOpRenFile : TFPgtkButton;
  Protected
    procedure CreateGtkObject; override;
  Public
    function TheGtkObject : PGtkFileSelection;
    property Filename : string read GetFilename write SetFilename;
    procedure Complete (Pattern:string);
    procedure ShowFileOpButtons;
    procedure HideFileOpButtons;
    property DirList : TFPgtkCList read GetDirList;
    property FileList : TFPgtkCList read GetFileList;
    property OkButton : TFPgtkButton read GetOkButton;
    property CancelButton : TFPgtkButton read GetCancelButton;
    property HistoryPulldown : TFPgtkOptionMenu read GetHistoryPulldown;
    property FileOpDialog : TFPgtkDialog read GetFileOpDialog;
    property FileOpCreateDir : TFPgtkButton read GetFileOpCreateDir;
    property FileOpDelFile : TFPgtkButton read GetFileOpDelFile;
    property FileOpRenFile : TFPgtkButton read GetFileOpRenFile;
  end;


  TFPgtkFontSelectionDialog = class (TFPgtkWindow)
  Private
    function GetFontSel : TFPgtkFontSelection;
    function GetButtonOk : TFPgtkButton;
    function GetButtonApply : TFPgtkButton;
    function GetButtonCancel : TFPgtkButton;
  Protected
    procedure CreateGtkObject; override;
  Public
    function TheGtkObject : PGtkFontSelectionDialog;
    property FontSel : TFPgtkFontSelection read GetFontSel;
    property ButtonOk : TFPgtkButton read GetButtonOk;
    property ButtonApply : TFPgtkButton read GetButtonApply;
    property ButtonCancel : TFPgtkButton read GetButtonCancel;
  end;


  TFPgtkEventBox = class (TFPgtkBin)
  Protected
    procedure CreateGtkObject; override;
  Public
    function TheGtkObject : PGtkEventBox;
  end;


  TFPgtkHandleBox = class (TFPgtkBin)
  Private
    function GetShadowType : TGtkShadowtype;
    procedure SetShadowType (TheValue : TGtkShadowtype);
    function GetHandlePosition : TGtkPositionType;
    procedure SetHandlePosition (TheValue : TGtkPositionType);
    function GetSnapEdge : TGtkPositionType;
    procedure SetSnapEdge (TheValue : TGtkPositionType);
    function GetChildDetached : boolean;
  Protected
    procedure CreateGtkObject; override;
  Public
    function TheGtkObject : PGtkHandleBox;
    property ShadowType : TGtkShadowtype read GetShadowType write SetShadowType;
    property HandlePosition : TGtkPositionType read GetHandlePosition write SetHandlePosition;
    property SnapEdge : TGtkPositionType read GetSnapEdge write SetSnapEdge;
    property ChildDetached : boolean read GetChildDetached;
    function ConnectChildAttached (proc:TFPgtkWidgetSignalFunction; data:pointer) : guint;
    function ConnectAfterChildAttached (proc:TFPgtkWidgetSignalFunction; data:pointer) : guint;
    function ConnectChildDetached (proc:TFPgtkWidgetSignalFunction; data:pointer) : guint;
    function ConnectAfterChildDetached (proc:TFPgtkWidgetSignalFunction; data:pointer) : guint;
  end;


  TFPgtkScrolledWindow = class (TFPgtkBin)
  Private
    FVScroll:TFPgtkAdjustment;
    FHScroll:TFPgtkAdjustment;
    function GetHPolicy : TGtkPolicyType;
    procedure SetHPolicy (TheValue : TGtkPolicyType);
    function GetVPolicy : TGtkPolicyType;
    procedure SetVPolicy (TheValue : TGtkPolicyType);
    function GetHAdjustment : TFPgtkAdjustment;
    procedure SetHAdjustment (TheValue : TFPgtkAdjustment);
    function GetVAdjustment : TFPgtkAdjustment;
    procedure SetVAdjustment (TheValue : TFPgtkAdjustment);
    function GetPlacement : TGtkCornerType;
    procedure SetPlacement (TheValue : TGtkCornerType);
    function GetHScrollbar : TFPgtkScrollbar;
    function GetVScrollbar : TFPgtkScrollbar;
  Protected
    procedure CreateGtkObject; Override;
  Public
    function TheGtkObject : PGtkScrolledWindow;
    constructor Create (hadj:TFPgtkAdjustment; vadj:TFPgtkAdjustment);
    property HPolicy : TGtkPolicyType read GetHPolicy write SetHPolicy;
    property VPolicy : TGtkPolicyType read GetVPolicy write SetVPolicy;
    procedure SetPolicy (aHScrollBar:TGtkPolicyType; aVScrollbar:TGtkPolicyType); Overload;
    procedure SetPolicy (aPolicy:TGtkPolicyType); Overload;
    property HAdjustment : TFPgtkAdjustment read GetHAdjustment write SetHAdjustment;
    property VAdjustment : TFPgtkAdjustment read GetVAdjustment write SetVAdjustment;
    procedure AddWithViewport (aChild:TFPgtkWidget);
    property Placement : TGtkCornerType read GetPlacement write SetPlacement;
    property HScrollbar : TFPgtkScrollbar read GetHScrollbar;
    property VScrollbar : TFPgtkScrollbar read GetVScrollbar;
    procedure UpdatePolicy (UpdPolicy:TGtkUpdateType);
  end;


  TFPgtkViewport = class (TFPgtkBin)
  Private
    FVScroll:TFPgtkAdjustment;
    FHScroll:TFPgtkAdjustment;
    function GetHAdjustment : TFPgtkAdjustment;
    procedure SetHAdjustment (TheValue : TFPgtkAdjustment);
    function GetVAdjustment : TFPgtkAdjustment;
    procedure SetVAdjustment (TheValue : TFPgtkAdjustment);
    function GetShadowType : TgtkShadowType;
    procedure SetShadowType (TheValue : TgtkShadowType);
  Protected
    procedure CreateGtkObject; Override;
  Public
    function TheGtkObject : PGtkViewport;
    constructor Create (hadj:TFPgtkAdjustment; vadj:TFPgtkAdjustment);
    property HAdjustment : TFPgtkAdjustment read GetHAdjustment write SetHAdjustment;
    property VAdjustment : TFPgtkAdjustment read GetVAdjustment write SetVAdjustment;
    property ShadowType : TgtkShadowType read GetShadowType write SetShadowType;
  end;


  TFPgtkBox = class (TFPgtkContainer)
  Private
    function GetHomogeneous : boolean;
    procedure SetHomogeneous (TheValue : boolean);
    function GetSpacing : integer;
    procedure SetSpacing (TheValue : integer);
  Public
    function TheGtkObject : PGtkBox;
    property Homogeneous : boolean read GetHomogeneous write SetHomogeneous;
    property Spacing : integer read GetSpacing write SetSpacing;
    procedure ReorderChild (Widget:TFPgtkWidget; Position:integer);
    procedure GetChildPacking (Widget:TFPgtkWidget; var Expand:boolean; var Fill:boolean; var Padding:integer; var PackType:TGtkPackType);
    procedure SetChildPacking (Widget:TFPgtkWidget; Expand:boolean; Fill:boolean; Padding:integer; PackType:TGtkPackType);
    procedure PackStart (Widget:TFPgtkWidget); Overload;
    procedure PackStart (Widget:TFPgtkWidget; IsVisible:boolean); Overload;
    procedure PackStart (Widget:TFPgtkWidget; expand:boolean; fill:boolean; padding:integer); Overload;
    procedure PackStart (Widget:TFPgtkWidget; expand:boolean; fill:boolean; padding:integer; IsVisible:boolean); Overload;
    procedure PackEnd (Widget:TFPgtkWidget); Overload;
    procedure PackEnd (Widget:TFPgtkWidget; IsVisible:boolean); Overload;
    procedure PackEnd (Widget:TFPgtkWidget; expand:boolean; fill:boolean; padding:integer); Overload;
    procedure PackEnd (Widget:TFPgtkWidget; expand:boolean; fill:boolean; padding:integer; IsVisible:boolean); Overload;
  end;


  TFPgtkButtonBox = class (TFPgtkBox)
  Private
    function GetSpacing : integer;
    procedure SetSpacing (TheValue : integer);
    function GetLayout : TGtkButtonBoxStyle;
    procedure SetLayout (TheValue : TGtkButtonBoxStyle);
    function GetMinWidth : integer;
    procedure SetMinWidth (TheValue : integer);
    function GetMinHeight : integer;
    procedure SetMinHeight (TheValue : integer);
    function GetChildPadX : integer;
    procedure SetChildPadX (TheValue : integer);
    function GetChildPadY : integer;
    procedure SetChildPadY (TheValue : integer);
  Public
    function TheGtkObject : PGtkButtonBox;
    property Spacing : integer read GetSpacing write SetSpacing;
    property Layout : TGtkButtonBoxStyle read GetLayout write SetLayout;
    property ChildMinWidth : integer read GetMinWidth write SetMinWidth;
    property ChildMinHeight : integer read GetMinHeight write SetMinHeight;
    property ChildPadX : integer read GetChildPadX write SetChildPadX;
    property ChildPadY : integer read GetChildPadY write SetChildPadY;
  end;


  TFPgtkHButtonBox = class (TFPgtkButtonBox)
  Protected
    procedure CreateGtkObject; override;
  Public
    function TheGtkObject : PGtkHButtonBox;
  end;


  TFPgtkVButtonBox = class (TFPgtkButtonBox)
  Protected
    procedure CreateGtkObject; override;
  Public
    function TheGtkObject : PGtkVButtonBox;
  end;


  TFPgtkVBox = class (TFPgtkBox)
  Protected
    procedure CreateGtkObject; override;
  Public
    function TheGtkObject : PGtkVBox;
  end;


  TFPgtkColorSelection = class (TFPgtkVBox)
  Private
    function GetUpdatePolicy : TGtkUpdateType;
    procedure SetUpdatePolicy (TheValue : TGtkUpdateType);
    function GetColor : double;
    procedure SetColor (TheValue : double);
    function GetUseOpacity : longbool;
    procedure SetUseOpacity (TheValue : longbool);
  Protected
    procedure CreateGtkObject; override;
  Public
    function TheGtkObject : PGtkColorSelection;
    property UpdatePolicy : TGtkUpdateType read GetUpdatePolicy write SetUpdatePolicy;
    property Color : double read GetColor write SetColor;
    property UseOpacity : longbool read GetUseOpacity write SetUseOpacity;
  end;


  TFPgtkGammaCurve = class (TFPgtkVBOX)
  Protected
    procedure CreateGtkObject; override;
  Public
    function TheGtkObject : PGtkGammaCurve;
  end;


  TFPgtkHBox = class (TFPgtkBox)
  Protected
    procedure CreateGtkObject; override;
  Public
    function TheGtkObject : PGtkHBox;
  end;


  TFPgtkCombo = class (TFPgtkHBox)
  Private
    function GetEntry : TFPgtkEntry;
    function GetList : TFPgtkList;
    function GetButton : TFpGtkButton;
    function GetValueInList : longbool;
    procedure SetValueInListProp (TheValue : longbool);
    function GetOkIfEmpty : longbool;
    procedure SetOkIfEmpty (TheValue : longbool);
    function GetUseArrows : longbool;
    procedure SetUseArrows (TheValue : longbool);
    function GetUseArrowsAlways : longbool;
    procedure SetUseArrowsAlways (TheValue : longbool);
    function GetCaseSensitive : longbool;
    procedure SetCaseSensitive (TheValue : longbool);
  Protected
    procedure CreateGtkObject; override;
  Public
    function TheGtkObject : PGtkCombo;
    property Entry : TFPgtkEntry read GetEntry;
    property List : TFPgtkList read GetList;
    property Button : TFpGtkButton read GetButton;
    property ValueInList : longbool read GetValueInList write SetValueInListProp;
    property OkIfEmpty : longbool read GetOkIfEmpty write SetOkIfEmpty;
    property UseArrows : longbool read GetUseArrows write SetUseArrows;
    property UseArrowsAlways : longbool read GetUseArrowsAlways write SetUseArrowsAlways;
    property CaseSensitive : longbool read GetCaseSensitive write SetCaseSensitive;
    procedure SetItemString (Item:TFPgtkItem; ItemValue:string);
    procedure DisableActivate;
    procedure SetValueInList (Val:longbool; IsOkIfEmpty:longbool);
  end;

  TFPgtkStatusbarSignalFunction = procedure (Sender:TFPgtkObject; contextID:integer; text:string; data:pointer) of Object;

  TFPgtkStatusbar = class (TFPgtkHBox)
  Protected
    procedure CreateGtkObject; override;
  Public
    function TheGtkObject : PGtkStatusbar;
    function GetContextID (ContextDescr:string) : integer;
    function Push (contextID:integer; text:string) : integer;
    procedure Pop (contextID:integer);
    procedure Remove (contextID:integer; MessageID:integer);
    function StatusbarSignalConnect (Signal:string; Proc:TFPgtkStatusbarSignalFunction; data:pointer) : guint;
    function StatusbarSignalConnectAfter (Signal:string; Proc:TFPgtkStatusbarSignalFunction; data:pointer) : guint;
    function ConnectTextPopped (proc:TFPgtkStatusbarSignalFunction; data:pointer) : guint;
    function ConnectAfterTextPopped (proc:TFPgtkStatusbarSignalFunction; data:pointer) : guint;
    function ConnectTextPushed (proc:TFPgtkStatusbarSignalFunction; data:pointer) : guint;
    function ConnectAfterTextPushed (proc:TFPgtkStatusbarSignalFunction; data:pointer) : guint;
  end;

  TFPgtkCListScrollSignalFunction = procedure (Sender:TFPgtkObject; ScrollType:TgtkScrollType; position:gfloat; data:pointer) of Object;
  TFPgtkCListScrollBooleanSignalFunction = procedure (Sender:TFPgtkObject; ScrollType:TgtkScrollType; Position:gfloat; AutoStartSelection:boolean; data:pointer) of Object;
  TFPgtkSelectRowSignalFunction = procedure (Sender:TFPgtkObject; row:integer; column:integer; event:PGdkEventButton; data:pointer) of Object;
  TFPgtkMoveSignalFunction = procedure (Sender:TFPgtkObject; arg1:integer; arg2:integer; data:pointer) of Object;
  TFPgtkColumnClickedSignalFunction = procedure (Sender:TFPgtkObject; column:integer; data:pointer) of Object;
  TFPgtkResizeColumnSignalFunction = procedure (Sender:TFPgtkObject; column:integer; width:integer; data:pointer) of Object;

  TFPgtkCList = class (TFPgtkContainer)
  Private
    compare : TGtkCListCompareFunc;
    FColumnCount : integer;
    function GetShadowType : TGtkShadowType;
    procedure SetShadowType (TheValue : TGtkShadowType);
    function GetSelectionMode : TGtkSelectionMode;
    procedure SetSelectionMode (TheValue : TGtkSelectionMode);
    function GetColumnTitle(column:integer) : string;
    procedure SetColumnTitle (column:integer; TheValue : string);
    function GetColumnWidget(column:integer) : TFPgtkWidget;
    procedure SetColumnWidget (column:integer; TheValue : TFPgtkWidget);
    function GetCellText(Row:integer; Column:integer) : string;
    procedure SetCellText (Row:integer; Column:integer; TheValue : string);
    function GetCellStyle(row:integer; column:integer) : PGtkStyle;
    procedure SetCellStyle (row:integer; column:integer; TheValue : PGtkStyle);
    function GetRowStyle(row:integer) : PGtkStyle;
    procedure SetRowStyle (row:integer; TheValue : PGtkStyle);
    function GetRowData(row:integer) : pointer;
    procedure SetRowData (row:integer; TheValue : pointer);
    procedure SetCompareFunc (TheValue : TGtkCListCompareFunc);
    function GetSortColumn : integer;
    procedure SetSortColumn (TheValue : integer);
    function GetSetSortType : TGtkSortType;
    procedure SetSetSortType (TheValue : TGtkSortType);
    function GetHAdjustment : TFPgtkAdjustment;
    procedure SetHAdjustment (TheValue : TFPgtkAdjustment);
    function GetVAdjustment : TFPgtkAdjustment;
    procedure SetVAdjustment (TheValue : TFPgtkAdjustment);
  Protected
    procedure CreateGtkObject; override;
  Public
    function TheGtkObject : PGtkCList;
    constructor Create (aColumnCount:integer);
    property ColumnCount : integer read FColumnCount;
    property ShadowType : TGtkShadowType read GetShadowType write SetShadowType;
    property SelectionMode : TGtkSelectionMode read GetSelectionMode write SetSelectionMode;
    procedure Freeze;
    procedure Thaw;
    procedure ShowTitles;
    procedure HideTitles;
    procedure ActiveTitles;
    procedure PassiveTitles;
    procedure ActiveTitle (column:integer);
    procedure PassiveTitle (column:integer);
    property ColumnTitle [column:integer]  : string read GetColumnTitle write SetColumnTitle;
    property ColumnWidget [column:integer]  : TFPgtkWidget read GetColumnWidget write SetColumnWidget;
    procedure SetColumnJustification (column:integer; justification:TGtkJustification);
    procedure SetColumnVisibility (column:integer; aVisible:boolean);
    procedure SetColumnResizeable (column:integer; Resizeable:boolean);
    procedure SetColumnAutoResize (column:integer; autoResize:boolean);
    function OptimalColumnWidth (column:integer) : integer;
    procedure SetColumnWidth (column:integer; width:integer);
    procedure SetColumnMinWidth (column:integer; MinWidth:integer);
    procedure SetColumnMaxWidth (column:integer; MaxWidth:integer);
    function AutoSizeColumns : integer;
    procedure ConfigureColumnWidth (column:integer; Width:integer; MinWidth:integer; MaxWidth:integer);
    procedure ConfigureColumn (column:integer; Justification:TGtkJustification; Visibility:boolean; Resizeable:boolean; AutoSize:boolean);
    procedure SetRowHeight (height:integer);
    procedure MoveTo (row:integer; column:integer; RowAlign:gfloat; ColAlign:gfloat);
    function RowIsVisible (Row:integer) : TGtkVisibility;
    function GetCellType (Row:integer; column:integer) : TGtkCellType;
    property CellText [Row:integer; Column:integer]  : string read GetCellText write SetCellText;
    procedure SetPixmap (row:integer; column:integer; pixmap:PGdkPixmap; mask:PGdkBitmap);
    procedure GetPixmap (row:integer; column:integer; var pixmap:PGdkPixmap; var mask:PGdkBitmap);
    procedure SetPixText (row:integer; column:integer; text:string; spacing:guint8; pixmap:PGdkPixmap; mask:PGdkBitmap);
    procedure GetPixText (row:integer; column:integer; var text:string; var aspacing:guint8; var pixmap:PGdkPixmap; var mask:PGdkBitmap);
    procedure SetForeground (row:integer; color:PGdkColor);
    procedure SetBackground (row:integer; color:PGdkColor);
    property CellStyle [row:integer; column:integer]  : PGtkStyle read GetCellStyle write SetCellStyle;
    property RowStyle [row:integer]  : PGtkStyle read GetRowStyle write SetRowStyle;
    procedure SetShift (row:integer; column:integer; vertical:integer; horizontal:integer);
    procedure Remove (row:integer);
    procedure Prepend (Data:TStrings); Overload;
    procedure Prepend (Text:string; separator:string); Overload;
    procedure Prepend (data:array of string); Overload;
    function Append (data:TStrings) : Integer; Overload;
    function Append (Text:string; Separator:string) : Integer; Overload;
    function Append (data:array of string) : Integer; Overload;
    procedure Insert (row:integer; data:TStrings); Overload;
    procedure Insert (row:integer; Text:string; Separator:string); Overload;
    procedure Insert (row:integer; data:array of string); Overload;
    property RowData [row:integer]  : pointer read GetRowData write SetRowData;
    function FindRowFromData (data:pointer) : integer;
    procedure SelectRow (row:integer; column:integer);
    procedure UnselectRow (row:integer; column:integer);
    procedure Clear;
    procedure SelectAll;
    procedure UnselectAll;
    procedure SwapRows (row1:integer; row2:integer);
    procedure RowMove (sourceRow:integer; destRow:integer);
    procedure Sort;
    property CompareFunc : TGtkCListCompareFunc read compare write SetCompareFunc;
    property SortColumn : integer read GetSortColumn write SetSortColumn;
    property SetSortType : TGtkSortType read GetSetSortType write SetSetSortType;
    procedure SetAutoSort (autoSort:boolean);
    property HAdjustment : TFPgtkAdjustment read GetHAdjustment write SetHAdjustment;
    property VAdjustment : TFPgtkAdjustment read GetVAdjustment write SetVAdjustment;
    procedure SetReorderable (reorderable:boolean);
    function Count : integer;
    function CListScrollSignalConnect (Signal:string; Proc:TFPgtkCListScrollSignalFunction; data:pointer) : guint;
    function CListScrollSignalConnectAfter (Signal:string; Proc:TFPgtkCListScrollSignalFunction; data:pointer) : guint;
    function CListScrollBooleanSignalConnect (Signal:string; Proc:TFPgtkCListScrollBooleanSignalFunction; data:pointer) : guint;
    function CListScrollBooleanSignalConnectAfter (Signal:string; Proc:TFPgtkCListScrollBooleanSignalFunction; data:pointer) : guint;
    function SelectRowSignalConnect (Signal:string; Proc:TFPgtkSelectRowSignalFunction; data:pointer) : guint;
    function SelectRowSignalConnectAfter (Signal:string; Proc:TFPgtkSelectRowSignalFunction; data:pointer) : guint;
    function ConnectSelectRow (proc:TFPgtkSelectRowSignalFunction; data:pointer) : guint;
    function ConnectAfterSelectRow (proc:TFPgtkSelectRowSignalFunction; data:pointer) : guint;
    function ConnectUnselectRow (proc:TFPgtkSelectRowSignalFunction; data:pointer) : guint;
    function ConnectAfterUnselectRow (proc:TFPgtkSelectRowSignalFunction; data:pointer) : guint;
    function MoveSignalConnect (Signal:string; Proc:TFPgtkMoveSignalFunction; data:pointer) : guint;
    function MoveSignalConnectAfter (Signal:string; Proc:TFPgtkMoveSignalFunction; data:pointer) : guint;
    function ConnectRowMove (proc:TFPgtkMoveSignalFunction; data:pointer) : guint;
    function ConnectAfterRowMove (proc:TFPgtkMoveSignalFunction; data:pointer) : guint;
    function ConnectScrollVertical (proc:TFPgtkCListScrollSignalFunction; data:pointer) : guint;
    function ConnectAfterScrollVertical (proc:TFPgtkCListScrollSignalFunction; data:pointer) : guint;
    function ConnectScrolHorizontal (proc:TFPgtkCListScrollSignalFunction; data:pointer) : guint;
    function ConnectAfterScrolHorizontal (proc:TFPgtkCListScrollSignalFunction; data:pointer) : guint;
    function ConnectToggleFocusRow (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectAfterToggleFocusRow (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectSelectAll (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectAfterSelectAll (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectUnselectAll (proc:TFPgtksignalFunction; data:pointer) : guint;
    function ConnectAfterUnselectAll (proc:TFPgtksignalFunction; data:pointer) : guint;
    function ConnectUndoSelection (proc:TFPgtksignalFunction; data:pointer) : guint;
    function ConnectAfterUndoSelection (proc:TFPgtksignalFunction; data:pointer) : guint;
    function ConnectStartSelection (proc:TFPgtksignalFunction; data:pointer) : guint;
    function ConnectAfterStartSelection (proc:TFPgtksignalFunction; data:pointer) : guint;
    function ConnectEndSelection (proc:TFPgtksignalFunction; data:pointer) : guint;
    function ConnectAfterEndSelection (proc:TFPgtksignalFunction; data:pointer) : guint;
    function ConnectToggleAddMode (proc:TFPgtksignalFunction; data:pointer) : guint;
    function ConnectAfterToggleAddMode (proc:TFPgtksignalFunction; data:pointer) : guint;
    function ConnectAbortColumnResize (proc:TFPgtksignalFunction; data:pointer) : guint;
    function ConnectAfterAbortColumnResize (proc:TFPgtksignalFunction; data:pointer) : guint;
    function ConnectExtendSelection (proc:TFPgtkCListScrollBooleanSignalFunction; data:pointer) : guint;
    function ConnectAfterExtendSelection (proc:TFPgtkCListScrollBooleanSignalFunction; data:pointer) : guint;
    function ColumnClickedSignalConnect (Signal:string; Proc:TFPgtkColumnClickedSignalFunction; data:pointer) : guint;
    function ColumnClickedSignalConnectAfter (Signal:string; Proc:TFPgtkColumnClickedSignalFunction; data:pointer) : guint;
    function ConnectClickColumn (proc:TFPgtkColumnClickedSignalFunction; data:pointer) : guint;
    function ConnectAfterClickColumn (proc:TFPgtkColumnClickedSignalFunction; data:pointer) : guint;
    function ResizeColumnSignalConnect (Signal:string; Proc:TFPgtkResizeColumnSignalFunction; data:pointer) : guint;
    function ResizeColumnSignalConnectAfter (Signal:string; Proc:TFPgtkResizeColumnSignalFunction; data:pointer) : guint;
    function ConnectResizeColumn (proc:TFPgtkResizeColumnSignalFunction; data:pointer) : guint;
    function ConnectAfterResizeColumn (proc:TFPgtkResizeColumnSignalFunction; data:pointer) : guint;
  end;

  TFPgtkCTreeFunction = procedure (TheTree:TFPgtkCTree; TheNode:PGtkCTreeNode; data:pointer) of object;

  TFPgtkCTree = class (TFPgtkCList)
  Private
    FTreeColumn:integer;
    function GetLineStyle : TGtkCTreeLineStyle;
    procedure SetLineStyle (TheValue : TGtkCTreeLineStyle);
    function GetShowStub : boolean;
    procedure SetShowStub (TheValue : boolean);
    function GetExpanderStyle : TGtkCTreeExpanderStyle;
    procedure SetExpanderStyle (TheValue : TGtkCTreeExpanderStyle);
    function GetSpacing : guint;
    procedure SetSpacing (TheValue : guint);
    function GetIndent : guint;
    procedure SetIndent (TheValue : guint);
    function GetTreeColumn : integer;
    function GetNodeCellText(Node:PGtkCTreeNode; Column:integer) : string;
    procedure SetNodeCellText (Node:PGtkCTreeNode; Column:integer; TheValue : string);
    function GetNodeSelectable(Node:PGtkCTreeNode) : boolean;
    procedure SetNodeSelectable (Node:PGtkCTreeNode; TheValue : boolean);
    function GetNodeCellStyle(Node:PGtkCTreeNode; column:integer) : PGtkStyle;
    procedure SetNodeCellStyle (Node:PGtkCTreeNode; column:integer; TheValue : PGtkStyle);
    function GetNodeRowStyle(Node:PGtkCTreeNode) : PGtkStyle;
    procedure SetNodeRowStyle (Node:PGtkCTreeNode; TheValue : PGtkStyle);
    function GetNodeData(Node:PGtkCTreeNode) : pointer;
    procedure SetNodeData (Node:PGtkCTreeNode; TheValue : pointer);
    function GetCompareDragFunc : TGtkCTreeCompareDragFunc;
    procedure SetCompareDragFunc (TheValue : TGtkCTreeCompareDragFunc);
  Public
    function TheGtkObject : PGtkCTree;
    property LineStyle : TGtkCTreeLineStyle read GetLineStyle write SetLineStyle;
    property ShowStub : boolean read GetShowStub write SetShowStub;
    property ExpanderStyle : TGtkCTreeExpanderStyle read GetExpanderStyle write SetExpanderStyle;
    property Spacing : guint read GetSpacing write SetSpacing;
    property Indent : guint read GetIndent write SetIndent;
    property TreeColumn : integer read GetTreeColumn;
    constructor Create (aColumnCount:integer; aTreeColumn:integer);
    procedure RemoveNode (node:PGtkCTreeNode);
    function InsertNode (aParent:PGtkCTreeNode; Sibling:PGtkCTreeNode; data:string; aSpacing:guint8; PixmapClosed:PGdkPixmap; MaskClosed:PGdkBitmap; PixmapOpened:PGdkPixmap; MaskOpened:PGdkBitmap; IsLeaf:boolean; Expanded:boolean) : PGtkCTreeNode; Overload;
    function InsertNode (aParent:PGtkCTreeNode; Sibling:PGtkCTreeNode; data:string; aSpacing:guint8; IsLeaf:boolean; Expanded:boolean) : PGtkCTreeNode; Overload;
    procedure PostRecursive (aNode:PGtkCTreeNode; func:TFPgtkCTreeFunction; data:pointer);
    procedure PostRecursiveToDepth (aNode:PGtkCTreeNode; aDepth:integer; func:TFPgtkCTreeFunction; data:pointer);
    procedure PreRecursive (aNode:PGtkCTreeNode; func:TFPgtkCTreeFunction; data:pointer);
    procedure PreRecursiveToDepth (aNode:PGtkCTreeNode; aDepth:integer; func:TFPgtkCTreeFunction; data:pointer);
    procedure IsViewable (aNode:PGtkCTreeNode);
    procedure LastChild (aNode:PGtkCTreeNode);
    function IsChild (anAncestor:PGtkCTreeNode; aChild:PGtkCTreeNode) : boolean;
    function IsAncestor (anAncestor:PGtkCTreeNode; aChild:PGtkCTreeNode) : boolean;
    function IsHotSpot (X:integer; Y:integer) : boolean;
    procedure MoveNode (aNode:PGtkCTreeNode; NewParent:PGtkCTreeNode; NewSibling:PGtkCTreeNode);
    procedure Expand (aNode:PGtkCTreeNode);
    procedure ExpandRecursive (aNode:PGtkCTreeNode);
    procedure ExpandToDepth (aNode:PGtkCTreeNode; aDepth:integer);
    procedure Collapse (aNode:PGtkCTreeNode);
    procedure CollapseRecursive (aNode:PGtkCTreeNode);
    procedure CollapseToDepth (aNode:PGtkCTreeNode; aDepth:integer);
    procedure SelectNode (aNode:PGtkCTreeNode);
    procedure SelectRecursive (aNode:PGtkCTreeNode);
    procedure UnselectNode (aNode:PGtkCTreeNode);
    procedure UnselectRecursive (aNode:PGtkCTreeNode);
    procedure RealSelectRecursive (aNode:PGtkCTreeNode; aState:boolean);
    function NodeGetCellType (Node:PGtkCTreeNode; column:integer) : TGtkCellType;
    property NodeCellText [Node:PGtkCTreeNode; Column:integer]  : string read GetNodeCellText write SetNodeCellText;
    procedure NodeSetPixmap (Node:PGtkCTreeNode; column:integer; pixmap:PGdkPixmap; mask:PGdkBitmap);
    procedure NodeGetPixmap (Node:PGtkCTreeNode; column:integer; var pixmap:PGdkPixmap; var mask:PGdkBitmap);
    procedure NodeSetPixText (Node:PGtkCTreeNode; column:integer; text:string; aspacing:guint8; pixmap:PGdkPixmap; mask:PGdkBitmap);
    procedure NodeGetPixText (Node:PGtkCTreeNode; column:integer; var text:string; var aspacing:guint8; var pixmap:PGdkPixmap; var mask:PGdkBitmap);
    procedure SetNodeInfo (aNode:PGtkCTreeNode; aText:string; aSpacing:guint8; PixmapClosed:PGdkPixmap; MaskClosed:PGdkBitmap; PixmapOpened:PGdkPixmap; MaskOpened:PGdkBitmap; IsLeaf:boolean; Expanded:boolean); Overload;
    procedure GetNodeInfo (aNode:PGtkCTreeNode; var aText:string; var aSpacing:guint8; var PixmapClosed:PGdkPixmap; var MaskClosed:PGdkBitmap; var PixmapOpened:PGdkPixmap; var MaskOpened:PGdkBitmap; var IsLeaf:boolean; var Expanded:boolean); Overload;
    procedure NodeSetShift (Node:PGtkCTreeNode; column:integer; vertical:integer; horizontal:integer);
    property NodeSelectable [Node:PGtkCTreeNode]  : boolean read GetNodeSelectable write SetNodeSelectable;
    procedure NodeSetForeground (Node:PGtkCTreeNode; color:PGdkColor);
    procedure NodeSetBackground (Node:PGtkCTreeNode; color:PGdkColor);
    property NodeCellStyle [Node:PGtkCTreeNode; column:integer]  : PGtkStyle read GetNodeCellStyle write SetNodeCellStyle;
    property NodeRowStyle [Node:PGtkCTreeNode]  : PGtkStyle read GetNodeRowStyle write SetNodeRowStyle;
    property NodeData [Node:PGtkCTreeNode]  : pointer read GetNodeData write SetNodeData;
    procedure NodeMoveTo (aNode:PGtkCTreeNode; column:integer; RowAlign:gfloat; ColAlign:gfloat);
    function IsVisible (aNode:PGtkCTreeNode) : TGtkVisibility;
    property CompareDragFunc : TGtkCTreeCompareDragFunc read GetCompareDragFunc write SetCompareDragFunc;
    procedure SortNode (aNode:PGtkCTreeNode);
    procedure SortRecursive (aNode:PGtkCTreeNode);
    function NthNode (Row:integer) : PGtkCTreeNode;
  end;


  TFPgtkFixed = class (TFPgtkContainer)
  Protected
    procedure CreateGtkObject; override;
  Public
    function TheGtkObject : PGtkFixed;
    procedure Put (Widget:TFPgtkWidget; x:integer; y:integer);
    procedure Move (Widget:TFPgtkWidget; x:integer; y:integer);
    procedure GetPos (Widget:TFPgtkWidget; var PosX:integer; var PosY:integer);
  end;

  TFPgtkPageSwitchSignalFunction = procedure (Sender:TFPgtkObject; PageRec:PGtkNotebookPage; aPageNum:integer; data:pointer) of Object;

  TFPgtkNotebook = class (TFPgtkContainer)
  Private
    function GetPageIndex : integer;
    procedure SetPageIndex (TheValue : integer);
    function GetPage : TFPgtkWidget;
    procedure SetPage (TheValue : TFPgtkWidget);
    function GetTabPos : TGtkPositionType;
    procedure SetTabPos (TheValue : TGtkPositionType);
    function GetShowTabs : boolean;
    procedure SetShowTabs (TheValue : boolean);
    function GetShowBorder : boolean;
    procedure SetShowBorder (TheValue : boolean);
    function GetScrollable : boolean;
    procedure SetScrollable (TheValue : boolean);
    function GetHomogenous : boolean;
    procedure SetHomogenous (TheValue : boolean);
    function GetTabHBorder : word;
    procedure SetTabHBorder (TheValue : word);
    function GetTabVBorder : word;
    procedure SetTabVBorder (TheValue : word);
  Protected
    procedure CreateGtkObject; override;
  Public
    function TheGtkObject : PGtkNotebook;
    procedure AppendPage (Child:TFPgtkWidget; TabLabel:TFPgtkWidget);
    procedure AppendPageFull (Child:TFPgtkWidget; TabLabel:TFPgtkWidget; MenuLabel:TFPgtkWidget; IsVisible:boolean);
    procedure PrependPage (Child:TFPgtkWidget; TabLabel:TFPgtkWidget);
    procedure PrependPageFull (Child:TFPgtkWidget; TabLabel:TFPgtkWidget; MenuLabel:TFPgtkWidget; IsVisible:boolean);
    procedure InsertPage (Child:TFPgtkWidget; TabLabel:TFPgtkWidget; Position:integer);
    procedure InsertPageFull (Child:TFPgtkWidget; TabLabel:TFPgtkWidget; MenuLabel:TFPgtkWidget; IsVisible:boolean; Position:integer);
    procedure RemovePage (PageNumber:integer);
    function PageNumberOf (Child:TFPgtkWidget) : integer;
    procedure NextPage;
    procedure PrevPage;
    procedure ReorderPage (Child:TFPgtkWidget; PageNum:integer);
    property PageIndex : integer read GetPageIndex write SetPageIndex;
    property Page : TFPgtkWidget read GetPage write SetPage;
    property TabPos : TGtkPositionType read GetTabPos write SetTabPos;
    property ShowTabs : boolean read GetShowTabs write SetShowTabs;
    property ShowBorder : boolean read GetShowBorder write SetShowBorder;
    property Scrollable : boolean read GetScrollable write SetScrollable;
    property Homogenous : boolean read GetHomogenous write SetHomogenous;
    property TabHBorder : word read GetTabHBorder write SetTabHBorder;
    property TabVBorder : word read GetTabVBorder write SetTabVBorder;
    procedure SetTabBorders (BorderWidth:word);
    function GetMenuLabelOf (Child:TFPgtkWidget) : TFPgtkWidget;
    procedure SetMenuLabel (Child:TFPgtkWidget; MenuLabel:TFPgtkWidget);
    function GetTabLabelOf (Child:TFPgtkWidget) : TFPgtkWidget;
    procedure SetTabLabel (Child:TFPgtkWidget; TabLabel:TFPgtkWidget);
    function GetChildOnPage (PageNum:integer) : TFPgtkWidget;
    procedure GetTabLabelPacking (Widget:TFPgtkWidget; var Expand:boolean; var Fill:boolean; var PackType:TGtkPackType);
    procedure SetTabLabelPacking (Widget:TFPgtkWidget; Expand:boolean; Fill:boolean; PackType:TGtkPackType);
    procedure EnablePopup;
    procedure DisablePopup;
    function PageSwitchSignalConnect (Signal:string; Proc:TFPgtkPageSwitchSignalFunction; data:pointer) : guint;
    function PageSwitchSignalConnectAfter (Signal:string; Proc:TFPgtkPageSwitchSignalFunction; data:pointer) : guint;
    function ConnectSwitchPage (proc:TFPgtkPageSwitchSignalFunction; data:pointer) : guint;
    function ConnectAfterSwitchPage (proc:TFPgtkPageSwitchSignalFunction; data:pointer) : guint;
  end;


  TFPgtkFontSelection = class (TFPgtkNotebook)
  Private
    function GetFontName : string;
    procedure SetFontName (TheValue : string);
    function GetPreviewText : string;
    procedure SetPreviewText (TheValue : string);
  Protected
    procedure CreateGtkObject; override;
  Public
    function TheGtkObject : PGtkFontSelection;
    property FontName : string read GetFontName write SetFontName;
    function GetFont : PGdkFont;
    property PreviewText : string read GetPreviewText write SetPreviewText;
    procedure SetFilter (FilterType:TGtkFontFilterType; FontType:TGtkFontType; Foundries:array of string; Weights:array of string; Slants:array of string; SetWidths:array of string; Spacings:array of string; CharSets:array of string);
  end;


  TFPgtkPaned = class (TFPgtkContainer)
  Private
    function GetGutterSize : word;
    procedure SetGutterSize (TheValue : word);
    function GetHandleSize : word;
    procedure SetHandleSize (TheValue : word);
    function GetPosition : integer;
    procedure SetPosition (TheValue : integer);
  Public
    function TheGtkObject : PGtkPaned;
    property GutterSize : word read GetGutterSize write SetGutterSize;
    property HandleSize : word read GetHandleSize write SetHandleSize;
    property Position : integer read GetPosition write SetPosition;
    procedure ComputePosition (AnAllocation:integer; Child1Req:integer; Child2Req:integer);
    procedure Add1 (Child:TFPgtkWidget); Overload;
    procedure Pack1 (Child:TFPgtkWidget; Resize:boolean; Shrink:boolean); Overload;
    procedure Add1 (Child:TFPgtkWidget; isVisible:boolean); Overload;
    procedure Pack1 (Child:TFPgtkWidget; Resize:boolean; Shrink:boolean; IsVisible:boolean); Overload;
    procedure Add2 (Child:TFPgtkWidget); Overload;
    procedure Pack2 (Child:TFPgtkWidget; Resize:boolean; Shrink:boolean); Overload;
    procedure Add2 (Child:TFPgtkWidget; IsVisible:boolean); Overload;
    procedure Pack2 (Child:TFPgtkWidget; Resize:boolean; Shrink:boolean; IsVisible:boolean); Overload;
  end;


  TFPgtkHPaned = class (TFPgtkPaned)
  Protected
    procedure CreateGtkObject; override;
  Public
    function TheGtkObject : PGtkHPaned;
  end;


  TFPgtkVPaned = class (TFPgtkPaned)
  Protected
    procedure CreateGtkObject; override;
  Public
    function TheGtkObject : PGtkVPaned;
  end;


  TFPgtkLayout = class (TFPgtkContainer)
  Private
    function GetHAdj : TFPgtkAdjustment;
    procedure SetHAdj (TheValue : TFPgtkAdjustment);
    function GetVAdj : TFPgtkAdjustment;
    procedure SetVAdj (TheValue : TFPgtkAdjustment);
  Protected
    procedure CreateGtkObject; override;
  Public
    function TheGtkObject : PGtkLayout;
    property HAdj : TFPgtkAdjustment read GetHAdj write SetHAdj;
    property VAdj : TFPgtkAdjustment read GetVAdj write SetVAdj;
    procedure Freeze;
    procedure Thaw;
    procedure Put (aWidget:TFPgtkWidget; X:integer; Y:integer); Overload;
    procedure Put (aWidget:TFPgtkWidget; X:integer; Y:integer; aVisible:boolean); Overload;
    procedure Move (aWidget:TFPgtkWidget; X:integer; Y:integer);
    procedure SetSize (aWidth:integer; aHeight:integer);
  end;


  TFPgtkList = class (TFPgtkContainer)
  Private
    function GetSelectionMode : TGtkSelectionMode;
    procedure SetSelectionMode (TheValue : TGtkSelectionMode);
  Protected
    procedure CreateGtkObject; override;
  Public
    function TheGtkObject : PGtkList;
    function ConnectSelectionChanged (proc:TFPgtksignalFunction; data:pointer) : guint;
    function ConnectAfterSelectionChanged (proc:TFPgtksignalFunction; data:pointer) : guint;
    function ConnectSelectChild (proc:TFPgtkWidgetSignalFunction; data:pointer) : guint;
    function ConnectAfterSelectChild (proc:TFPgtkWidgetSignalFunction; data:pointer) : guint;
    function ConnectUnselectChild (proc:TFPgtkWidgetSignalFunction; data:pointer) : guint;
    function ConnectAfterUnselectChild (proc:TFPgtkWidgetSignalFunction; data:pointer) : guint;
    property SelectionMode : TGtkSelectionMode read GetSelectionMode write SetSelectionMode;
    procedure InsertItems (TheItems:TFPgtkListItemGroup; position:integer);
    procedure AppendItems (TheItems:TFPgtkListItemGroup);
    procedure PrependItems (TheItems:TFPgtkListItemGroup);
    procedure RemoveItems (TheItems:TFPgtkListItemGroup);
    procedure RemoveItemsNoUnref (TheItems:TFPgtkListItemGroup);
    procedure ClearItems (FromItem:integer; ToItem:integer);
    procedure ClearAll;
    procedure SelectItem (Item:integer);
    procedure UnselectItem (Item:integer);
    procedure SelectChild (Child:TFPgtkWidget);
    procedure UnselectChild (Child:TFPgtkWidget);
    function ChildPosition (Child:TFPgtkWidget) : integer;
    procedure ExtendSelection (ScrollType:TGtkScrollType; Position:gfloat; AutoStartSelection:boolean);
    procedure StartSelection;
    procedure EndSelection;
    procedure SelectAll;
    procedure UnselectAll;
    procedure ScrollHorizontal (ScrollType:TGtkScrollType; Position:gfloat);
    procedure ScrollVertical (ScrollType:TGtkScrollType; Position:gfloat);
    procedure ToggleAddMode;
    procedure ToggleFocusRow;
    procedure ToggleRow (Child:TFPgtkWidget);
    procedure UndoSelection;
    procedure EndDragSelection;
    procedure GetSelection (aGroup:TFPgtkGroup);
  end;

  TFPgtkMoveCurrentSignalFunction = procedure (Sender:TFPgtkObject; dir:TGtkMenuDirectionType; data:pointer) of Object;

  TFPgtkMenuShell = class (TFPgtkContainer)
  Protected
    procedure GtkPrepend (MenuItem:TFPgtkWidget); Virtual;
    procedure GtkInsert (MenuItem:TFPgtkWidget; position:integer); Virtual;
    procedure GtkAppend (MenuItem:TFPgtkWidget); Virtual;
  Public
    function TheGtkObject : PGtkMenuShell;
    function MoveCurrentSignalConnect (Signal:string; Proc:TFPgtkMoveCurrentSignalFunction; data:pointer) : guint;
    function MoveCurrentSignalConnectAfter (Signal:string; Proc:TFPgtkMoveCurrentSignalFunction; data:pointer) : guint;
    procedure ActivateItem (MenuItem:TFPgtkWidget; ForceDeactivate:boolean);
    procedure SelectItem (MenuItem:TFPgtkWidget);
    procedure DeActivate;
    procedure Prepend (MenuItem:TFPgtkWidget); Overload;
    procedure Prepend (MenuItem:TFPgtkWidget; CreateVisible:boolean); Overload;
    procedure Insert (MenuItem:TFPgtkWidget; position:integer); Overload;
    procedure Insert (MenuItem:TFPgtkWidget; position:integer; CreateVisible:boolean); Overload;
    procedure Append (MenuItem:TFPgtkWidget); Overload;
    procedure Append (MenuItem:TFPgtkWidget; CreateVisible:boolean); Overload;
    function ConnectDeActivate (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectAfterDeActivate (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectSelectionDone (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectAfterSelectionDone (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectCancel (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectAfterCancel (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectMoveCurrent (proc:TFPgtkMoveCurrentSignalFunction; data:pointer) : guint;
    function ConnectAfterMoveCurrent (proc:TFPgtkMoveCurrentSignalFunction; data:pointer) : guint;
    function ConnectActivateCurrent (proc:TFPgtkBooleanSignalFunction; data:pointer) : guint;
    function ConnectAfterActivateCurrent (proc:TFPgtkBooleanSignalFunction; data:pointer) : guint;
  end;


  TFPgtkMenuBar = class (TFPgtkMenuShell)
  Private
    function GetShadow : TgtkShadowType;
    procedure SetShadow (TheValue : TgtkShadowType);
  Protected
    procedure CreateGtkObject; override;
    procedure GtkPrepend (MenuItem:TFPgtkWidget); Override;
    procedure GtkInsert (MenuItem:TFPgtkWidget; position:integer); Override;
    procedure GtkAppend (MenuItem:TFPgtkWidget); Override;
  Public
    function TheGtkObject : PGtkMenuBar;
    property Shadow : TgtkShadowType read GetShadow write SetShadow;
  end;

  TFPgtkMenuDetachFunction = procedure (Widget:TFPgtkWidget; menu:TFPgtkMenu) of object;
  TFPgtkMenuPosFunction = procedure (menu:TFPgtkMenu; var x,y:integer; data:pointer) of object;

  TFPgtkMenu = class (TFPgtkMenuShell)
  Private
    procedure SetTitle (TheValue : string);
    function GetActive : TFPgtkWidget;
    procedure SetActive (TheValue : TFPgtkWidget);
    function GetActiveIndex : integer;
    procedure SetActiveIndex (TheValue : integer);
    function GetTearOffState : boolean;
    procedure SetTearOffState (TheValue : boolean);
    function GetAttachedTo : TFPgtkWidget;
    procedure SetAttachedTo (TheValue : TFPgtkWidget);
    function GetAccelGroup : PGtkAccelGroup;
    procedure SetAccelGroup (TheValue : PGtkAccelGroup);
  Protected
    procedure CreateGtkObject; override;
    procedure GtkPrepend (MenuItem:TFPgtkWidget); Override;
    procedure GtkInsert (MenuItem:TFPgtkWidget; position:integer); Override;
    procedure GtkAppend (MenuItem:TFPgtkWidget); Override;
  Public
    function TheGtkObject : PGtkMenu;
    FDetacher:TFPgtkMenuDetachFunction;
    procedure ReorderChild (MenuItem:TFPgtkWidget; position:integer);
    procedure Popup (button:guint); Overload;
    procedure Popup (ParentShell:TFPgtkWidget; ParentItem:TFPgtkWidget; func:TFPgtkMenuPosFunction; data:pointer; button:guint; ActivateTime:guint32); Overload;
    procedure PopDown;
    procedure Reposition;
    procedure AttachToWidget (Widget:TFPgtkWidget; detacher:TFPgtkMenuDetachFunction);
    procedure Detach;
    property Title : string write SetTitle;
    property Active : TFPgtkWidget read GetActive write SetActive;
    property ActiveIndex : integer read GetActiveIndex write SetActiveIndex;
    property TearOffState : boolean read GetTearOffState write SetTearOffState;
    property AttachedTo : TFPgtkWidget read GetAttachedTo write SetAttachedTo;
    property AccelGroup : PGtkAccelGroup read GetAccelGroup write SetAccelGroup;
  end;


  TFPgtkPacker = class (TFPgtkContainer)
  Private
    function GetSpacing : guint;
    procedure SetSpacing (TheValue : guint);
  Protected
    procedure CreateGtkObject; override;
  Public
    function TheGtkObject : PGtkPacker;
    procedure Add (Child:TFPgtkWidget; Side:TGtkSideType; Anchor:TGtkAnchorType; options:TGtkPackerOptions); Overload;
    procedure Add (Child:TFPgtkWidget; Side:TGtkSideType; Anchor:TGtkAnchorType; options:TGtkPackerOptions; aVisible:boolean); Overload;
    procedure Add (Child:TFPgtkWidget; Side:TGtkSideType; Anchor:TGtkAnchorType; options:TGtkPackerOptions; aBorder:guint; PadX:Guint; PadY:guint; IPadX:guint; IPadY:guint); Overload;
    procedure Add (Child:TFPgtkWidget; Side:TGtkSideType; Anchor:TGtkAnchorType; options:TGtkPackerOptions; aBorder:guint; PadX:Guint; PadY:guint; IPadX:guint; IPadY:guint; aVisible:boolean); Overload;
    procedure ReorderChild (aChild:TFPgtkWidget; position:integer);
    property Spacing : guint read GetSpacing write SetSpacing;
    procedure DefaultBorder (aBorder:guint);
    procedure DefaultPad (PadX:guint; PadY:guint);
    procedure DefaultIPad (IPadX:guint; IPadY:guint);
    procedure Configure (aChild:TFPgtkWidget; Side:TGtkSideType; Anchor:TGtkAnchorType; options:TGtkPackerOptions; aBorder:guint; PadX:Guint; PadY:guint; IPadX:guint; IPadY:guint); Overload;
  end;


  TFPgtkTable = class (TFPgtkContainer)
  Private
    function GetRowCount : integer;
    function GetColCount : integer;
    function GetHomogeneous : boolean;
    procedure SetHomogeneous (TheValue : boolean);
    function GetRowSpacings : integer;
    procedure SetRowSpacings (TheValue : integer);
    function GetColSpacings : integer;
    procedure SetColSpacings (TheValue : integer);
  Protected
    procedure CreateGtkObject; override;
  Public
    function TheGtkObject : PGtkTable;
    constructor Create (AColumns:integer; ARows:integer);
    procedure Resize (AColumns:integer; ARows:integer);
    procedure Attach (Widget:TFPgtkWidget; left:integer; right:integer; top:integer; bottom:integer; XOptions:integer; YOptions:integer; XPadding:integer; YPadding:integer; IsVisible:boolean);
    procedure Attach (Widget:TFPgtkWidget; left:integer; right:integer; top:integer; bottom:integer; XOptions:integer; YOptions:integer; XPadding:integer; YPadding:integer);
    procedure Attach (Widget:TFPgtkWidget; left:integer; right:integer; top:integer; bottom:integer; IsVisible:boolean);
    procedure Attach (Widget:TFPgtkWidget; left:integer; right:integer; top:integer; bottom:integer);
    property RowCount : integer read GetRowCount;
    property ColCount : integer read GetColCount;
    property Homogeneous : boolean read GetHomogeneous write SetHomogeneous;
    property RowSpacings : integer read GetRowSpacings write SetRowSpacings;
    property ColSpacings : integer read GetColSpacings write SetColSpacings;
    procedure SetOneRowSpacing (row:integer; TheValue:integer);
    procedure SetOneColSpacing (Column:integer; TheValue:integer);
  end;


  TFPgtkToolbar = class (TFPgtkContainer)
  Private
    function GetButtonRelief : TGtkReliefStyle;
    procedure SetButtonRelief (TheValue : TGtkReliefStyle);
    function GetTooltips : TFPgtkTooltips;
    function GetEnableTooltips : longbool;
    procedure SetEnableTooltips (TheValue : longbool);
    function GetSpaceStyle : TGtkToolbarSpaceStyle;
    procedure SetSpaceStyle (TheValue : TGtkToolbarSpaceStyle);
    function GetSpaceSize : integer;
    procedure SetSpaceSize (TheValue : integer);
    function GetStyle : TGtkToolbarStyle;
    procedure SetStyle (TheValue : TGtkToolbarStyle);
    function GetOrientation : tGtkOrientation;
    procedure SetOrientation (TheValue : tGtkOrientation);
  Protected
    procedure CreateGtkObject; override;
  Public
    function TheGtkObject : PGtkToolbar;
    property ButtonRelief : TGtkReliefStyle read GetButtonRelief write SetButtonRelief;
    property Tooltips : TFPgtkTooltips read GetTooltips;
    property EnableTooltips : longbool read GetEnableTooltips write SetEnableTooltips;
    property SpaceStyle : TGtkToolbarSpaceStyle read GetSpaceStyle write SetSpaceStyle;
    property SpaceSize : integer read GetSpaceSize write SetSpaceSize;
    property Style : TGtkToolbarStyle read GetStyle write SetStyle;
    property Orientation : tGtkOrientation read GetOrientation write SetOrientation;
    procedure InsertWidget (Widget:TFPgtkWidget; TooltipText:string; TooltipPrivate:string; Position:integer);
    procedure PrependWidget (Widget:TFPgtkWidget; TooltipText:string; TooltipPrivate:string);
    procedure AppendWidget (Widget:TFPgtkWidget; TooltipText:string; TooltipPrivate:string);
    function InsertElement (ButtonType:TGtkToolbarChildType; PrevRadioBut:TFPgtkWidget; Text:string; TooltipText:string; TooltipPrivate:string; Icon:TFPgtkWidget; CallBack:TFPgtkSignalFunction; data:pointer; position:integer) : TFPgtkWidget;
    function AppendElement (ButtonType:TGtkToolbarChildType; PrevRadioBut:TFPgtkWidget; Text:string; TooltipText:string; TooltipPrivate:string; Icon:TFPgtkWidget; CallBack:TFPgtkSignalFunction; data:pointer) : TFPgtkWidget;
    function PrependElement (ButtonType:TGtkToolbarChildType; PrevRadioBut:TFPgtkWidget; Text:string; TooltipText:string; TooltipPrivate:string; Icon:TFPgtkWidget; CallBack:TFPgtkSignalFunction; data:pointer) : TFPgtkWidget;
    function InsertItem (Text:string; TooltipText:string; TooltipPrivate:string; Icon:TFPgtkWidget; CallBack:TFPgtkSignalFunction; data:pointer; position:integer) : TFPgtkWidget; Overload;
    function AppendItem (Text:string; TooltipText:string; TooltipPrivate:string; Icon:TFPgtkWidget; CallBack:TFPgtkSignalFunction; data:pointer) : TFPgtkWidget; Overload;
    function PrependItem (Text:string; TooltipText:string; TooltipPrivate:string; Icon:TFPgtkWidget; CallBack:TFPgtkSignalFunction; data:pointer) : TFPgtkWidget; Overload;
    function InsertItem (Text:string; TooltipText:string; TooltipPrivate:string; Icon:array of string; CallBack:TFPgtkSignalFunction; data:pointer; position:integer) : TFPgtkWidget; Overload;
    function AppendItem (Text:string; TooltipText:string; TooltipPrivate:string; Icon:array of string; CallBack:TFPgtkSignalFunction; data:pointer) : TFPgtkWidget; Overload;
    function PrependItem (Text:string; TooltipText:string; TooltipPrivate:string; Icon:array of string; CallBack:TFPgtkSignalFunction; data:pointer) : TFPgtkWidget; Overload;
    procedure InsertSpace (position:integer);
    procedure AppendSpace;
    procedure PrependSpace;
  end;


  TFPgtkTree = class (TFPgtkContainer)
  Private
    function GetSelectionMode : TGtkSelectionMode;
    procedure SetSelectionMode (TheValue : TGtkSelectionMode);
    function GetViewLines : boolean;
    procedure SetViewLines (TheValue : boolean);
    function GetViewMode : TGtkTreeViewMode;
    procedure SetViewMode (TheValue : TGtkTreeViewMode);
  Protected
    procedure CreateGtkObject; override;
  Public
    function TheGtkObject : PGtkTree;
    function ConnectSelectionChanged (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectAfterSelectionChanged (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectSelectChild (proc:TFPgtkWidgetSignalFunction; data:pointer) : guint;
    function ConnectAfterSelectChild (proc:TFPgtkWidgetSignalFunction; data:pointer) : guint;
    function ConnectUnselectChild (proc:TFPgtkWidgetSignalFunction; data:pointer) : guint;
    function ConnectAfterUnselectChild (proc:TFPgtkWidgetSignalFunction; data:pointer) : guint;
    property SelectionMode : TGtkSelectionMode read GetSelectionMode write SetSelectionMode;
    property ViewLines : boolean read GetViewLines write SetViewLines;
    property ViewMode : TGtkTreeViewMode read GetViewMode write SetViewMode;
    procedure Append (TreeItem:TFPgtkWidget);
    procedure Prepend (TreeItem:TFPgtkWidget);
    procedure Insert (TreeItem:TFPgtkWidget; position:integer);
    procedure Remove (TreeItem:TFPgtkWidget);
    procedure ClearItems (StartPos:integer; EndPos:integer);
    procedure SelectItem (Item:integer);
    procedure UnselectItem (Item:integer);
    procedure SelectChild (TreeItem:TFPgtkWidget);
    procedure UnselectChild (TreeItem:TFPgtkWidget);
    function ChildPosition (TreeItem:TFPgtkWidget) : integer;
    function RootTree : TFPgtkTree;
    function IsRootTree : boolean;
    procedure GetSelection (aGroup:TFPgtkGroup);
    function Level : integer;
  end;


  TFPgtkCalendar = class (TFPgtkWidget)
  Private
    function GetDisplayOptions : TGtkCalendarDisplayOptions;
    procedure SetDisplayOptions (TheValue : TGtkCalendarDisplayOptions);
    function GetDate : TDatetime;
    procedure SetDate (TheValue : TDatetime);
  Protected
    procedure CreateGtkObject; override;
  Public
    function TheGtkObject : PGtkCalendar;
    function SelectMonth (aMonth:guint; aYear:guint) : integer;
    procedure SelectDay (aDay:guint);
    function MarkDay (aDay:guint) : integer;
    function UnmarkDay (aDay:guint) : integer;
    procedure ClearMarks;
    property DisplayOptions : TGtkCalendarDisplayOptions read GetDisplayOptions write SetDisplayOptions;
    property Date : TDatetime read GetDate write SetDate;
    procedure Freeze;
    procedure Thaw;
    function ConnectMonthChanged (proc:TFPgtksignalFunction; data:pointer) : guint;
    function ConnectAfterMonthChanged (proc:TFPgtksignalFunction; data:pointer) : guint;
    function ConnectDaySelected (proc:TFPgtksignalFunction; data:pointer) : guint;
    function ConnectAfterDaySelected (proc:TFPgtksignalFunction; data:pointer) : guint;
    function ConnectDaySelectedDoubleClick (proc:TFPgtksignalFunction; data:pointer) : guint;
    function ConnectAfterDaySelectedDoubleClick (proc:TFPgtksignalFunction; data:pointer) : guint;
    function ConnectPrevMonth (proc:TFPgtksignalFunction; data:pointer) : guint;
    function ConnectAfterPrevMonth (proc:TFPgtksignalFunction; data:pointer) : guint;
    function ConnectNextMonth (proc:TFPgtksignalFunction; data:pointer) : guint;
    function ConnectAfterNextMonth (proc:TFPgtksignalFunction; data:pointer) : guint;
    function ConnectPrevYear (proc:TFPgtksignalFunction; data:pointer) : guint;
    function ConnectAfterPrevYear (proc:TFPgtksignalFunction; data:pointer) : guint;
    function ConnectNextYear (proc:TFPgtksignalFunction; data:pointer) : guint;
    function ConnectAfterNextYear (proc:TFPgtksignalFunction; data:pointer) : guint;
  end;


  TFPgtkDrawingArea = class (TFPgtkWidget)
  Protected
    procedure CreateGtkObject; override;
  Public
    function TheGtkObject : PGtkDrawingArea;
    procedure SetSize (Width:integer; Height:integer);
  end;


  TFPgtkCurve = class (TFPgtkDrawingArea)
  Private
    function GetCurveType : TGtkCurveType;
    procedure SetCurveType (TheValue : TGtkCurveType);
  Protected
    procedure CreateGtkObject; override;
  Public
    function TheGtkObject : PGtkCurve;
    procedure SetRange (MinX:float; MaxX:float; MinY:float; MaxY:float);
    procedure Reset;
    procedure SetGamma (GammaValue:float);
    property CurveType : TGtkCurveType read GetCurveType write SetCurveType;
  end;

  TFPgtkInsertSignalFunction = procedure (Sender:TFPgtkObject; NewText:string; TextLength:integer; var Position:integer; data:pointer) of Object;
  TFPgtkDeleteSignalFunction = procedure (Sender:TFPgtkObject; StartPos:integer; EndPos:integer; data:pointer) of Object;
  TFPgtkXYSignalFunction = procedure (Sender:TFPgtkObject; x:integer; y:integer; data:pointer) of Object;
  TFPgtkDirectionSignalFunction = procedure (Sender:TFPgtkObject; Direction:integer; data:pointer) of Object;
  TFPgtkMoveWordSignalFunction = procedure (Sender:TFPgtkObject; NumWords:integer; data:pointer) of Object;
  TFPgtkMovetoSignalFunction = procedure (Sender:TFPgtkObject; MoveTo:integer; data:pointer) of Object;

  TFPgtkEditable = class (TFPgtkWidget)
  Private
    function GetEditable : boolean;
    procedure SetEditable (TheValue : boolean);
    function GetVisible : boolean;
    procedure SetVisible (TheValue : boolean);
    function GetPosition : integer;
    procedure SetPosition (TheValue : integer);
    function GetSelectionStart : integer;
    procedure SetSelectionStart (TheValue : integer);
    function GetSelectionEnd : integer;
    procedure SetSelectionEnd (TheValue : integer);
    function GetSelection : string;
  Protected
    function GetHasSelection : boolean; Dynamic;
    procedure SetSelection (TheValue:string); Dynamic;
    function GetText : string; Dynamic;
    procedure SetText (TheValue:string); Dynamic; Abstract;
  Public
    function TheGtkObject : PGtkEditable;
    property HasSelection : boolean read GetHasSelection;
    property Editable : boolean read GetEditable write SetEditable;
    property Visible : boolean read GetVisible write SetVisible;
    property Position : integer read GetPosition write SetPosition;
    property SelectionStart : integer read GetSelectionStart write SetSelectionStart;
    property SelectionEnd : integer read GetSelectionEnd write SetSelectionEnd;
    property Selection : string read GetSelection write SetSelection;
    property Text : string read GetText write SetText;
    procedure Changed;
    procedure InsertText (NewText:string; AtPosition:integer);
    procedure DeleteText (StartPos:integer; EndPos:integer);
    procedure GetChars (StartPos:integer; EndPos:integer);
    procedure CutClipboard;
    procedure CopyClipboard;
    procedure PasteClipboard;
    procedure SelectRegion (StartPos:integer; EndPos:integer);
    procedure ClaimSelection (claim:boolean; time:guint32);
    procedure DeleteSelection;
    procedure Clear;
    function InsertSignalConnect (Signal:string; Proc:TFPgtkInsertSignalFunction; data:pointer) : guint;
    function InsertSignalConnectAfter (Signal:string; Proc:TFPgtkInsertSignalFunction; data:pointer) : guint;
    function DeleteSignalConnect (Signal:string; Proc:TFPgtkDeleteSignalFunction; data:pointer) : guint;
    function DeleteSignalConnectAfter (Signal:string; Proc:TFPgtkDeleteSignalFunction; data:pointer) : guint;
    function XYSignalConnect (Signal:string; Proc:TFPgtkXYSignalFunction; data:pointer) : guint;
    function XYSignalConnectAfter (Signal:string; Proc:TFPgtkXYSignalFunction; data:pointer) : guint;
    function DirectionSignalConnect (Signal:string; Proc:TFPgtkDirectionSignalFunction; data:pointer) : guint;
    function DirectionSignalConnectAfter (Signal:string; Proc:TFPgtkDirectionSignalFunction; data:pointer) : guint;
    function MoveWordSignalConnect (Signal:string; Proc:TFPgtkMoveWordSignalFunction; data:pointer) : guint;
    function MoveWordSignalConnectAfter (Signal:string; Proc:TFPgtkMoveWordSignalFunction; data:pointer) : guint;
    function MovetoSignalConnect (Signal:string; Proc:TFPgtkMovetoSignalFunction; data:pointer) : guint;
    function MovetoSignalConnectAfter (Signal:string; Proc:TFPgtkMovetoSignalFunction; data:pointer) : guint;
    function ConnectChanged (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectAfterChanged (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectActivate (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectAfterActivate (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectInsertText (proc:TFPgtkInsertSignalFunction; data:pointer) : guint;
    function ConnectAfterInsertText (proc:TFPgtkInsertSignalFunction; data:pointer) : guint;
    function ConnectDeleteText (proc:TFPgtkDeleteSignalFunction; data:pointer) : guint;
    function ConnectAfterDeleteText (proc:TFPgtkDeleteSignalFunction; data:pointer) : guint;
    function ConnectSetEditable (proc:TFPgtkBooleanSignalFunction; data:pointer) : guint;
    function ConnectAfterSetEditable (proc:TFPgtkBooleanSignalFunction; data:pointer) : guint;
    function ConnectMoveCursor (proc:TFPgtkXYSignalFunction; data:pointer) : guint;
    function ConnectAfterMoveCursor (proc:TFPgtkXYSignalFunction; data:pointer) : guint;
    function ConnectMoveWord (proc:TFPgtkMoveWordSignalFunction; data:pointer) : guint;
    function ConnectAfterMoveWord (proc:TFPgtkMoveWordSignalFunction; data:pointer) : guint;
    function ConnectMovePage (proc:TFPgtkXYSignalFunction; data:pointer) : guint;
    function ConnectAfterMovePage (proc:TFPgtkXYSignalFunction; data:pointer) : guint;
    function ConnectMoveToRow (proc:TFPgtkMoveToSignalFunction; data:pointer) : guint;
    function ConnectAfterMoveToRow (proc:TFPgtkMoveToSignalFunction; data:pointer) : guint;
    function ConnectMoveToCol (proc:TFPgtkMoveToSignalFunction; data:pointer) : guint;
    function ConnectAfterMoveToCol (proc:TFPgtkMoveToSignalFunction; data:pointer) : guint;
    function ConnectKillChar (proc:TFPgtkDirectionSignalFunction; data:pointer) : guint;
    function ConnectAfterKillChar (proc:TFPgtkDirectionSignalFunction; data:pointer) : guint;
    function ConnectKillWord (proc:TFPgtkDirectionSignalFunction; data:pointer) : guint;
    function ConnectAfterKillWord (proc:TFPgtkDirectionSignalFunction; data:pointer) : guint;
    function ConnectKillLine (proc:TFPgtkDirectionSignalFunction; data:pointer) : guint;
    function ConnectAfterKillLine (proc:TFPgtkDirectionSignalFunction; data:pointer) : guint;
    function ConnectCutClipboard (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectAfterCutClipboard (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectCopyClipboard (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectAfterCopyClipboard (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectPasteClipboard (proc:TFPgtkSignalFunction; data:pointer) : guint;
    function ConnectAfterPasteClipboard (proc:TFPgtkSignalFunction; data:pointer) : guint;
  end;


  TFPgtkEntry = class (TFPgtkEditable)
  Private
    function GetVisibility : boolean;
    procedure SetVisibility (TheValue : boolean);
    function GetMaxLength : word;
    procedure SetMaxLength (TheValue : word);
  Protected
    procedure CreateGtkObject; override;
    procedure SetText (TheValue:string); Override;
  Public
    function TheGtkObject : PGtkEntry;
    procedure AppendText (aText:string);
    procedure PrependText (aText:string);
    property Visibility : boolean read GetVisibility write SetVisibility;
    property MaxLength : word read GetMaxLength write SetMaxLength;
  end;


  TFPgtkSpinButton = class (TFPgtkEntry)
  Private
    function GetAdjustment : TFPgtkAdjustment;
    procedure SetAdjustment (TheValue : TFPgtkAdjustment);
    function GetClimbRate : gfloat;
    procedure SetClimbRate (TheValue : gfloat);
    function GetDigits : integer;
    procedure SetDigits (TheValue : integer);
    function GetAsInteger : integer;
    procedure SetAsInteger (TheValue : integer);
    function GetAsFloat : gfloat;
    procedure SetAsFloat (TheValue : gfloat);
    function GetUpdatePolicy : TGtkSpinButtonUpdatePolicy;
    procedure SetUpdatePolicy (TheValue : TGtkSpinButtonUpdatePolicy);
    function GetNumeric : boolean;
    procedure SetNumeric (TheValue : boolean);
    function GetWrap : boolean;
    procedure SetWrap (TheValue : boolean);
    function GetShadowType : TGtkShadowType;
    procedure SetShadowType (TheValue : TGtkShadowType);
    function GetSnapToTicks : boolean;
    procedure SetSnapToTicks (TheValue : boolean);
  Protected
    procedure CreateGtkObject; override;
  Public
    function TheGtkObject : PGtkSpinButton;
    procedure Configure (Adj:TFPgtkAdjustment; aClimbRate:gfloat; aDigits:integer);
    property Adjustment : TFPgtkAdjustment read GetAdjustment write SetAdjustment;
    property ClimbRate : gfloat read GetClimbRate write SetClimbRate;
    property Digits : integer read GetDigits write SetDigits;
    property AsInteger : integer read GetAsInteger write SetAsInteger;
    property AsFloat : gfloat read GetAsFloat write SetAsFloat;
    property UpdatePolicy : TGtkSpinButtonUpdatePolicy read GetUpdatePolicy write SetUpdatePolicy;
    property Numeric : boolean read GetNumeric write SetNumeric;
    procedure Spin (direction:TGtkSpinType; increment:gfloat);
    property Wrap : boolean read GetWrap write SetWrap;
    property ShadowType : TGtkShadowType read GetShadowType write SetShadowType;
    property SnapToTicks : boolean read GetSnapToTicks write SetSnapToTicks;
    procedure Update;
  end;


  TFPgtkText = class (TFPgtkEditable)
  Private
    FLines:TStrings;
    FIsChanged:boolean;
    procedure SigChanged (Sender:TFPgtkObject; data:pointer);
    function GetLines : TStrings;
    function GetWordWrap : boolean;
    procedure SetWordWrap (TheValue : boolean);
    function GetLineWrap : boolean;
    procedure SetLineWrap (TheValue : boolean);
    function GetPoint : integer;
    procedure SetPoint (TheValue : integer);
    function GetHAdjustment : TFPgtkAdjustment;
    procedure SetHAdjustment (TheValue : TFPgtkAdjustment);
    function GetVAdjustment : TFPgtkAdjustment;
    procedure SetVAdjustment (TheValue : TFPgtkAdjustment);
  Protected
    procedure CreateGtkObject; override;
    procedure RefreshLines;
    procedure SetText (TheValue:string); Override;
  Public
    function TheGtkObject : PGtkText;
    constructor Create;
    destructor Destroy; Override;
    property Lines : TStrings read GetLines;
    procedure Freeze;
    procedure Thaw;
    function TextLength : guint;
    procedure Insert (font:PgdkFont; fore:PgdkColor; back:PgdkColor; TheText:string);
    procedure DeleteBackward (number:longword);
    procedure DeleteForward (number:longword);
    property WordWrap : boolean read GetWordWrap write SetWordWrap;
    property LineWrap : boolean read GetLineWrap write SetLineWrap;
    property Point : integer read GetPoint write SetPoint;
    procedure SetAdjustments (hadj:TFPgtkAdjustment; vadj:TFPgtkAdjustment);
    property HAdjustment : TFPgtkAdjustment read GetHAdjustment write SetHAdjustment;
    property VAdjustment : TFPgtkAdjustment read GetVAdjustment write SetVAdjustment;
  end;


  TFPgtkRuler = class (TFPgtkWidget)
  Public
    function TheGtkObject : PGtkRuler;
    procedure SetMetric (aMetric:TGtkMetricType);
    procedure SetRange (Lower:float; Upper:float; Position:float; MaxSize:float);
  end;


  TFPgtkHRuler = class (TFPgtkRuler)
  Protected
    procedure CreateGtkObject; override;
  Public
    function TheGtkObject : PGtkHRuler;
  end;


  TFPgtkVRuler = class (TFPgtkRuler)
  Protected
    procedure CreateGtkObject; override;
  Public
    function TheGtkObject : PGtkVRuler;
  end;


  TFPgtkRange = class (TFPgtkWidget)
  Private
    function GetAdjustment : TFPgtkAdjustment;
    procedure SetAdjustment (TheValue : TFPgtkAdjustment);
    function GetUpdatePolicy : TgtkUpdateType;
    procedure SetUpdatePolicy (TheValue : TgtkUpdateType);
  Protected
    FAdj:TFPgtkAdjustment;
  Public
    function TheGtkObject : PGtkRange;
    property Adjustment : TFPgtkAdjustment read GetAdjustment write SetAdjustment;
    property UpdatePolicy : TgtkUpdateType read GetUpdatePolicy write SetUpdatePolicy;
    constructor Create (AnAdjustment:TFPgtkAdjustment);
    procedure DrawBackground;
    procedure DrawTrough;
    procedure DrawStepForw;
    procedure DrawStepBack;
    procedure DrawSlider;
    procedure SliderUpdate;
    function TroughClick (X:integer; Y:integer; var JumpPerc:gfloat) : integer;
    procedure DefaultHSliderUpdate;
    procedure DefaultVSliderUpdate;
    function DefaultHTroughClick (X:integer; Y:integer; var JumpPerc:gfloat) : integer;
    function DefaultVTroughClick (X:integer; Y:integer; var JumpPerc:gfloat) : integer;
    procedure defaultHMotion (XDelta:integer; YDelta:integer);
    procedure defaultVMotion (XDelta:integer; YDelta:integer);
    procedure ClearBackground;
  end;


  TFPgtkScale = class (TFPgtkRange)
  Private
    function GetDrawValue : boolean;
    procedure SetDrawValue (TheValue : boolean);
    function GetValuePos : TGtkPositionType;
    procedure SetValuePos (TheValue : TGtkPositionType);
  Public
    function TheGtkObject : PGtkScale;
    procedure SetDigits (TheValue:integer);
    property DrawValue : boolean read GetDrawValue write SetDrawValue;
    property ValuePos : TGtkPositionType read GetValuePos write SetValuePos;
  end;


  TFPgtkHScale = class (TFPgtkScale)
  Protected
    procedure CreateGtkObject; override;
  Public
    function TheGtkObject : PGtkHScale;
  end;


  TFPgtkVScale = class (TFPgtkScale)
  Protected
    procedure CreateGtkObject; override;
  Public
    function TheGtkObject : PGtkVScale;
  end;


  TFPgtkScrollbar = class (TFPgtkRange)
  Public
    function TheGtkObject : PGtkScrollbar;
  end;


  TFPgtkHScrollbar = class (TFPgtkScrollbar)
  Protected
    procedure CreateGtkObject; Override;
  Public
    function TheGtkObject : PGtkHScrollbar;
  end;


  TFPgtkVScrollbar = class (TFPgtkScrollbar)
  Protected
    procedure CreateGtkObject; Override;
  Public
  end;


  TFPgtkSeparator = class (TFPgtkWidget)
  Public
    function TheGtkObject : PGtkSeparator;
  end;


  TFPgtkHSeparator = class (TFPgtkSeparator)
  Protected
    procedure CreateGtkObject; override;
  Public
    function TheGtkObject : PGtkHSeparator;
  end;


  TFPgtkVSeparator = class (TFPgtkSeparator)
  Protected
    procedure CreateGtkObject; override;
  Public
    function TheGtkObject : PGtkVSeparator;
  end;


  TFPgtkPreview = class (TFPgtkWidget)
  Private
    function GetExpand : longbool;
    procedure SetExpand (TheValue : longbool);
    function GetDither : TGdkRgbDither;
    procedure SetDither (TheValue : TGdkRgbDither);
  Protected
    procedure CreateGtkObject; override;
  Public
    function TheGtkObject : PGtkPreview;
    procedure Size (aWidth:integer; aHeight:integer);
    procedure Put (aWindow:PGdkWindow; gc:PGdkGC; SrcX:integer; SrcY:integer; destX:integer; DestY:integer; aWidth:integer; aHeight:integer);
    procedure DrawRow (data:pguchar; X:integer; Y:integer; W:integer);
    property Expand : longbool read GetExpand write SetExpand;
    property Dither : TGdkRgbDither read GetDither write SetDither;
  end;


  TFPgtkProgress = class (TFPgtkWidget)
  Private
    function GetShowtext : longbool;
    procedure SetShowtext (TheValue : longbool);
    function GetTextXAlign : gfloat;
    procedure SetTextXAlign (TheValue : gfloat);
    function GetTextYAlign : gfloat;
    procedure SetTextYAlign (TheValue : gfloat);
    function GetCurrentValue : float;
    procedure SetCurrentValue (TheValue : float);
    function GetPercentage : float;
    procedure SetPercentage (TheValue : float);
    function GetFormatString : string;
    procedure SetFormatString (TheValue : string);
    function GetAdjustment : TFPgtkAdjustment;
    procedure SetAdjustment (TheValue : TFPgtkAdjustment);
    function GetActivityMode : longbool;
    procedure SetActivityMode (TheValue : longbool);
  Public
    function TheGtkObject : PGtkProgress;
    property Showtext : longbool read GetShowtext write SetShowtext;
    property TextXAlign : gfloat read GetTextXAlign write SetTextXAlign;
    property TextYAlign : gfloat read GetTextYAlign write SetTextYAlign;
    procedure SetTextAlignment (anXalign:gfloat; anYAlign:gfloat);
    property CurrentValue : float read GetCurrentValue write SetCurrentValue;
    property Percentage : float read GetPercentage write SetPercentage;
    function PercentageFromValue (aValue:gfloat) : gfloat;
    property FormatString : string read GetFormatString write SetFormatString;
    property Adjustment : TFPgtkAdjustment read GetAdjustment write SetAdjustment;
    property ActivityMode : longbool read GetActivityMode write SetActivityMode;
    function CurrentText : string;
    function TextFromValue (aValue:gfloat) : string;
    procedure Configure (aValue:gfloat; aMin:gfloat; aMax:gfloat);
  end;


  TFPgtkProgressBar = class (TFPgtkProgress)
  Private
    FAdj:TFPgtkAdjustment;
    function GetBarStyle : TGtkProgressBarStyle;
    procedure SetBarStyle (TheValue : TGtkProgressBarStyle);
    function GetDiscreteBlocks : longword;
    procedure SetDiscreteBlocks (TheValue : longword);
    function GetActivityStep : longword;
    procedure SetActivityStep (TheValue : longword);
    function GetActivityBlocks : longword;
    procedure SetActivityBlocks (TheValue : longword);
    function GetOrientation : TGtkProgressBarOrientation;
    procedure SetOrientation (TheValue : TGtkProgressBarOrientation);
  Protected
    procedure CreateGtkObject; Override;
  Public
    function TheGtkObject : PGtkProgressBar;
    constructor Create (adj:TFPgtkAdjustment);
    property BarStyle : TGtkProgressBarStyle read GetBarStyle write SetBarStyle;
    property DiscreteBlocks : longword read GetDiscreteBlocks write SetDiscreteBlocks;
    property ActivityStep : longword read GetActivityStep write SetActivityStep;
    property ActivityBlocks : longword read GetActivityBlocks write SetActivityBlocks;
    property Orientation : TGtkProgressBarOrientation read GetOrientation write SetOrientation;
  end;


  TFPgtkItemFactory = class (TFPgtkObject)
  Public
  end;

{ TFPgtkToolTips }
var
  TheTooltips : TFPgtkTooltips;
{ TFPgtkButton }
const
  DefaultButtonModifiers : TGdkModifierType = GDK_MOD1_MASK;
{ TFPgtkWindow }
const
  drNone = 0;
  drOk = 1;
  drCancel = 2;
  drYes = 3;
  drNo = 4;
  drRetry = 5;
  NoMainLevel = high (guint);
{ TFPgtkFontSelection }
resourcestring
  sFontNotFound = 'Can''t find font "%s" on this system';

Const
// TFPgtkObject
  sgDestroy = 'destroy';
// TFPgtkData
  sgDisconnect = 'disconnect';
// TFPgtkAdjustment
  sgValueChanged = 'value_changed';
  sgChanged = 'changed';
// TFPgtkWidget
  sgShow = 'show';
  sghide = 'hide';
  sgmap = 'map';
  sgunmap = 'unmap';
  sgrealize = 'realize';
  sgunrealize = 'unrealize';
  sgDrawFocus = 'draw-focus';
  sgDrawDefault = 'draw-defaut';
  sgParentSet = 'parent-set';
  sgGrabFocus = 'grab-focus';
  sgEvent = 'event';
  sgButtonPressEvent = 'button-press-event';
  sgButtonReleaseEvent = 'button-release-event';
  sgMotionNotifyEvent = 'motion-notify-event';
  sgDeleteEvent = 'delete-event';
  sgDestroyEvent = 'destroy-event';
  sgExposeEvent = 'expose-event';
  sgKeyPressEvent = 'key-press-event';
  sgKeyReleaseEvent = 'key-release-event';
  sgEnterNotifyEvent = 'enter-notify-event';
  sgLeaveNotifyEvent = 'leave-notify-event';
  sgConfigureEvent = 'configure-event';
  sgFocusInEvent = 'focus-in-event';
  sgFocusOutEvent = 'focus-out-event';
  sgMapEvent = 'map-event';
  sgUnmapEvent = 'unmap-event';
  sgPropertyNotifyEvent = 'property-notify-event';
  sgSelectionClearEvent = 'selection-clear-event';
  sgSelectionRequestEvent = 'selection-request-event';
  sgSelectionNotifyEvent = 'selection-notify-event';
  sgProximityInEvent = 'proximity-in-event';
  sgProximityOutEvent = 'proximity-out-event';
  sgClientEvent = 'client-event';
  sgNoExposeEvent = 'no-expose-event';
  sgVisibilityNotifyEvent = 'visibility-notify-event';
// TFPgtkContainer
  sgAdd = 'add';
  sgRemove = 'remove';
  sgCheckResize = 'check-resize';
  sgFocus = 'focus';
  sgSetFocusChild = 'set-focus';
// TFPgtkButton
  sgClicked = 'clicked';
  sgPressed = 'pressed';
  sgReleased = 'released';
  sgEnter = 'enter';
  sgLeave = 'leave';
// TFPgtkToggleButton
  sgToggled = 'toggled';
// TFPgtkItem
  sgSelect = 'select';
  sgDeselect = 'deselect';
  sgToggle = 'toggle';
// TFPgtkMenuItem
  sgActivate = 'activate';
  sgActivateItem = 'activate-item';
// TFPgtkListItem
  sgToggleFocusRow = 'toggle-focus-row';
  sgSelectAll = 'select-all';
  sgUnselectAll = 'unselect-all';
  sgUndoSelection = 'undo-selection';
  sgStartSelection = 'start-selection';
  sgEndSelection = 'end-selection';
  sgToggleAddMode = 'toggle-add-mode';
  sgExtendSelection = 'extend-selection';
  sgScrollVertical = 'scroll-vertical';
  sgScrollHorizontal = 'scroll-horizontal';
// TFPgtkTreeItem
  sgCollapse = 'collapse';
  sgExpand = 'expand';
// TFPgtkWindow
  sgSetFocus = 'set-focus';
// TFPgtkInputDialog
  sgEnableDevice = 'enable-device';
  sgDisableDevice = 'disable-device';
// TFPgtkHandleBox
  sgChildAttached = 'child-attached';
  sgChildDetached = 'child-detached';
// TFPgtkStatusbar
  sgTextPopped = 'text-popped';
  sgTextPushed = 'test-pushed';
// TFPgtkCList
  sgSelectRow = 'select-row';
  sgUnselectRow = 'unselect-row';
  sgRowMove = 'row-move';
  sgScrolHorizontal = 'scroll-horizontal';
  sgAbortColumnResize = 'abort-column-resize';
  sgClickColumn = 'click-column';
  sgResizeColumn = 'resize-column';
// TFPgtkNotebook
  sgSwitchPage = 'switch-page';
// TFPgtkList
  sgSelectionChanged = 'selection-changed';
  sgSelectChild = 'select-child';
  sgUnselectChild = 'unselect-child';
// TFPgtkMenuShell
  sgDeActivate = 'deactivate';
  sgSelectionDone = 'selection-done';
  sgCancel = 'cancel';
  sgMoveCurrent = 'move-current';
  sgActivateCurrent = 'activate-current';
// TFPgtkCalendar
  sgMonthChanged = 'month-changed';
  sgDaySelected = 'day-selected';
  sgDaySelectedDoubleClick = 'day-selected-double-click';
  sgPrevMonth = 'prev-month';
  sgNextMonth = 'next-month';
  sgPrevYear = 'prev-year';
  sgNextYear = 'next-year';
// TFPgtkEditable
  sgInsertText = 'insert-text';
  sgDeleteText = 'delete-text';
  sgSetEditable = 'set-editable';
  sgMoveCursor = 'move-cursor';
  sgMoveWord = 'move-word';
  sgMovePage = 'move-page';
  sgMoveToRow = 'move-to-row';
  sgMoveToCol = 'move-to-column';
  sgKillChar = 'kill-char';
  sgKillWord = 'kill-word';
  sgKillLine = 'kill-line';
  sgCutClipboard = 'cut-clipboard';
  sgCopyClipboard = 'copy-clipboard';
  sgPasteClipboard = 'paste-clipboard';

// TFPgtkObject
function GetPascalInstance (gtkObject:PGtkObject; ObjClass:TFPgtkObjectClass) : TFPgtkObject; Overload;
function GetPascalInstance (gtkObject:PGtkObject) : TFPgtkObject; Overload;
function ConvertToGtkObject (AnObject:TFPgtkObject) : PGtkObject;
function ConvertToPgChar (AString:string) : PgChar;
function FreeFPgtkObjects (Data:pointer) : longbool; Cdecl;
procedure DestroyData (data:pointer); Cdecl;
function IntToPointer (Value:integer) : pointer;
function PointerToInt (Value:pointer) : integer;
// TFPgtkToolTips
function GetTooltipsData (Widget:TFPgtkWidget) : PGtkTooltipsData;
function ComposeTooltip (TooltipText:string; PrivText:string) : string;
procedure DecomposeTooltip (Tooltip:string; var TooltipText:string; var PrivText:string);
// TFPgtkWidget
function GetPascalInstance (Widget:PGtkWidget) : TFPgtkWidget; Overload;
function GetPascalInstance (Widget:PGtkWidget; ObjClass:TFPgtkObjectClass) : TFPgtkWidget; Overload;
function ConvertToGtkWidget (AnObject:TFPgtkWidget) : PGtkWidget;
// TFPgtkImage
function NewImage (aWidth:integer; aHeight:integer) : PGdkImage;
// TFPgtkPixmap
function StringsToPPgchar (Data:TStrings) : PPgchar;
function ArrayToPPgchar (Data:array of string) : PPgchar;
procedure CreateGdkPixmap (var ThePixmap:PGdkPixmap; var TheMask:PGdkBitmap; aWindow:PGdkWindow; data:array of string);
// TFPgtkRadioButtonGroup
function RadioButtonGroupCreateFromStrings (TheItems:TStrings; ToggledFunction:TFPgtkSignalFunction) : TFPgtkRadioButtonGroup;
// TFPgtkWindow
procedure AcceleratorAdd (AG:PGtkAccelGroup; aWidget:TFPgtkWidget; aSignal:string; Key:guint; Mods:TGdkModifierType; Flags:TGtkAccelFlags);
procedure AcceleratorRemove (AG:PGtkAccelGroup; aWidget:TFPgtkWidget; Key:guint; Mods:TGdkModifierType); Overload;
procedure AccelGroupLock (AG:PGtkAccelGroup);
procedure AccelGroupUnlock (AG:PGtkAccelGroup);
function AccelKeyName (Key:guint; Mods:TGdkModifierType) : string;
procedure AccelKeyParse (AccelName:string; var Key:guint; var Mods:TGdkModifierType);
procedure AccelGroupActivate (AG:PGtkAccelGroup; Key:guint; Mods:TGdkModifierType);
// TFPgtkButtonBox
procedure SetButtonBoxDefaultSize (aMinWidth:integer; aMinHeight:integer);
procedure GetButtonBoxDefaultSize (var aMinWidth:integer; var aMinHeight:integer);
procedure SetButtonBoxDefaultPadding (aIPadX:integer; aIPadY:integer);
procedure GetButtonBoxDefaultPadding (var aIPadX:integer; var aIPadY:integer);
// TFPgtkPreview
procedure SetGamma (aGamma:double);

IMPLEMENTATION

 { TFPgtkObject }

function TFPgtkObject.TheGtkObject : PGtkObject;
begin
  result := PgtkObject(FGtkObject);
end;

const
  dtPascalInstance = 'Pascal_Instance';

type
  TIntegerPointer = record
    case word of
      0 : (i : integer);
      1 : (p : pointer);
  end;

var
  ObjectsToFree : TList;
  ip : TIntegerPointer;

procedure Signalproc (Sender:PGtkobject; Data:pointer); cdecl;
var p : TFPgtkSignalFunction;
begin
with PSignalData(data)^ do
  begin
  p := TFPgtkSignalFunction (TheSignalProc);
  p (TheWidget as TFPgtkObject, TheData)
  end;
end;

function TFPgtkObject.SignalConnect (signal:string; proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect (FGtkObject, pgChar(signal), gtk_signal_func(@Signalproc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

function TFPgtkObject.SignalConnectAfter (signal:string; proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect_After (FGtkObject, pgChar(signal), gtk_signal_func(@Signalproc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

procedure BooleanSignalproc (Sender:PGtkobject; Bool:boolean; data:pointer); cdecl;
var p : TFPgtkBooleanSignalFunction;
begin
with PSignalData(data)^ do
  begin
  p := TFPgtkBooleanSignalFunction (TheSignalProc);
  p (TheWidget as TFPgtkObject, Bool, TheData)
  end;
end;

function TFPgtkObject.BooleanSignalConnect (signal:string; proc:TFPgtkBooleanSignalFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect (FGtkObject, pgChar(signal), gtk_signal_func(@BooleanSignalproc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

function TFPgtkObject.BooleanSignalConnectAfter (signal:string; proc:TFPgtkBooleanSignalFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect_After (FGtkObject, pgChar(signal), gtk_signal_func(@BooleanSignalproc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

function GetPascalInstance (gtkObject:PGtkObject; ObjClass:TFPgtkObjectClass) : TFPgtkObject; Overload;
begin
  result := GetPascalInstance(GtkObject);
  if not assigned(result) and assigned(GtkObject) then
    result := ObjClass.CreateFromObject (GtkObject);
end;


function GetPascalInstance (gtkObject:PGtkObject) : TFPgtkObject; Overload;
var p : pointer;
begin
  result := nil;
  if assigned (gtkobject) then
    begin
    p := gtk_object_get_data (gtkObject, dtPascalInstance);
    if assigned(p) then
      result := PPascalClassData(p)^.TheInstance;
    end;
end;


function ConvertToGtkObject (AnObject:TFPgtkObject) : PGtkObject;
begin
  if assigned(AnObject) then
    result := AnObject.TheGtkObject
  else
    result := nil;
end;


function ConvertToPgChar (AString:string) : PgChar;
begin
  result := pointer(aString);
end;


function TFPgtkObject.ConvertSignalData (proc:TFPgtkSignalFunction; data:pointer; FreeIt:boolean) : PSignalData;
begin
  new (result);
  with result^ do
    begin
    TheSignalProc := proc;
    TheWidget := self;
    TheData := data;
    end;
  if FreeIt then
    SignalDatas.Add (result);
end;

function FreeFPgtkObjects (Data:pointer) : longbool; Cdecl;
var r : integer;
    obj : TFPgtkObject;
begin
  for r := ObjectsToFree.Count-1 downto 0 do
    begin
    obj := TFPgtkObject(ObjectsToFree[r]);
    if assigned (Obj) then
      Obj.Free;
    end;
  ObjectsToFree.Clear;
  result := False;
end;


procedure TFPgtkObject.FreeClass (Sender:TFPgtkObject; Data:pointer);
begin
  if FDestroying = dsAlive then
    begin
    if ObjectsToFree.Count = 0 then
      g_idle_Add (@FreeFPgtkObjects, null);
    ObjectsToFree.Add (self);
    FGtkObject := null;
    FDestroying := dsWaiting;
    end;
end;

procedure TFPgtkObject.CheckConvertDatas;
begin
  if not assigned (ConvertDatas) then
    begin
    ConvertDatas := TStringList.Create;
    ConvertDatas.Sorted := True;
    end;
end;

procedure TFPgtkObject.CheckNotifyList;
begin
  if not assigned (Notifylist) then
    NotifyList := TList.Create;
end;

procedure TFPgtkObject.InitCreate;
begin
  inherited create;
  SignalDatas := TList.Create;
end;

procedure TFPgtkObject.FinalCreate;
begin
  PascalInstance.TheInstance := Self;
  SetData (dtPascalInstance, @PascalInstance);
  ConnectDestroy (@FreeClass, nil);
end;

constructor TFPgtkObject.Create;
begin
  InitCreate;
  CreateGtkObject;
  FinalCreate;
end;


constructor TFPgtkObject.CreateFromObject (GtkObject:PGtkObject);
begin
  InitCreate;
  FGtkObject := GtkObject;
  FinalCreate;
end;


procedure TFPgtkObject.AskNotification (AnObject:TFPgtkObject);
begin
  CheckNotifyList;
  with NotifyList do
    if indexof(AnObject) < 0 then
      begin
      Add (AnObject);
      AnObject.AskNotification (Self);
      end;
end;

destructor TFPgtkObject.Destroy;
var r : integer;
    datapointer : PSignalData;
begin
  FDestroying := dsDestroying;
  if assigned(NotifyList) then
    begin
    for r := 0 to NotifyList.count-1 do
      TFPgtkObject(NotifyList[r]).NotifyDestroy (Self);
    NotifyList.Free;
    NotifyList := nil;
    end;
  if assigned(FGtkObject) and not Gtk_Object_destroyed(FGtkObject) then
    begin
    gtk_object_destroy (FGtkObject);
    FGtkObject := nil;
    end;
  for r := 0 to SignalDatas.count-1 do
    begin
    datapointer := signaldatas[r];
    dispose (datapointer);
    end;
  signaldatas.Free;
  if assigned (convertDatas) then
    ConvertDatas.Free;
  r := ObjectsToFree.indexof (self);
  if r >= 0 then
    ObjectsToFree[r] := nil;
  inherited destroy;
end;


procedure TFPgtkObject.NotifyDestroy (AnObject:TFPgtkObject);
var r : integer;
begin
  if assigned(NotifyList) then
    begin
    r := NotifyList.indexOf (AnObject);
    if r >= 0 then
      NotifyList.Delete (r);
    end;
end;

function TFPgtkObject.ConnectDestroy (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnect (sgDestroy, proc, data);
end;

function TFPgtkObject.ConnectAfterDestroy (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnectAfter (sgDestroy, proc, data);
end;

procedure TFPgtkObject.SignalDisconnect (SignalHandler:guint);
begin
  gtk_signal_disconnect (TheGtkObject, SignalHandler);
end;

procedure TFPgtkObject.SignalBlockHandler (SignalHandler:guint);
begin
  gtk_signal_handler_block (TheGtkObject, SignalHandler);
end;

procedure TFPgtkObject.SignalUnblockHandler (SignalHandler:guint);
begin
  gtk_signal_handler_unblock (TheGtkObject, SignalHandler);
end;

procedure TFPgtkObject.SignalEmit (aName:string; Args:array of const);
begin
//  gtk_signal_emit_by_name (TheGtkObject, pgchar(aName), Args);
end;

function TFPgtkObject.SignalNEmissions (aName:string) : guint;
begin
  result := gtk_signal_n_emissions_by_name (FGtkObject, pgchar(aName));
end;

procedure TFPgtkObject.SignalEmitStop (aName:string);
begin
  gtk_signal_emit_stop_by_name (FGtkObject, pgchar(aName));
end;

procedure TFPgtkObject.SetData (Key:string; Data:pointer);
begin
  gtk_object_set_data (TheGtkObject, ConvertToPgchar(Key), Data);
end;

function TFPgtkObject.GetUserData : pointer;
begin
  result := gtk_object_get_user_data(TheGtkObject);
end;

procedure TFPgtkObject.SetUserData (TheValue:pointer);
begin
  gtk_object_set_user_data(TheGtkObject,TheValue);
end;

procedure TFPgtkObject.SetDataFull (Key:string; Data:pointer; Destroyer:TFPgtkSignalFunction);
begin
  gtk_object_set_data_full (TheGtkObject, pgChar(Key), ConvertSignalData (Destroyer, data, false), TGtkDestroyNotify(@DestroyData));
  CheckConvertDatas;
  ConvertDatas.Add (Key);
end;

procedure TFPgtkObject.RemoveData (Key:string);
var r : integer;
begin
  gtk_object_remove_data (TheGtkObject, pgChar(Key));
  if assigned (ConvertDatas) then
    begin
    r := ConvertDatas.indexof (Key);
    if r >= 0 then
    ConvertDatas.Delete (r);
    end;
end;

function TFPgtkObject.GetData (Key:string) : pointer;
var p  : pointer;
begin
  p := gtk_object_get_data (TheGtkObject, pgChar(Key));
  if assigned(ConvertDatas) and (ConvertDatas.IndexOf (Key) >= 0) then
    result := PPascalClassData (PSignalData(p)^.TheData)^.TheInstance
  else
    result := p;
end;

procedure DestroyData (data:pointer); Cdecl;
begin
  with PSignaldata(data)^ do
    TheSignalProc (TheWidget, TheData);
end;


function IntToPointer (Value:integer) : pointer;
begin
  ip.i := Value;
  result := ip.p;
end;


function PointerToInt (Value:pointer) : integer;
begin
  ip.p := Value;
  result := ip.i;
end;


function TFPgtkObject.GtkDestroyed : boolean;
begin
  result := gtk_object_destroyed (TheGtkObject);
end;

procedure TFPgtkObject.Constructed;
begin
  gtk_object_constructed (TheGtkObject);
end;

procedure TFPgtkObject.ConstructedDefault;
begin
  gtk_object_default_construct (TheGtkObject);
end;

procedure TFPgtkObject.Sink;
begin
  gtk_object_sink (TheGtkObject);
end;

procedure TFPgtkObject.Ref;
begin
  gtk_object_ref (TheGtkObject);
end;

procedure TFPgtkObject.Unref;
begin
  gtk_object_unref (TheGtkObject);
end;

procedure TFPgtkObject.WeakRef (Notify:TFPgtkSignalFunction; data:pointer);
begin
  gtk_object_weakref (TheGtkObject, TGtkDestroyNotify(@DestroyData), ConvertSignalData (Notify, data, true));
end;

procedure TFPgtkObject.WeakUnref (notify:TFPgtkSignalFunction; data:pointer);
begin
  gtk_object_weakunref (TheGtkObject, TGtkDestroyNotify(@DestroyData), ConvertSignalData (Notify, data, true));
end;

 { TFPgtkData }

function TFPgtkData.TheGtkObject : PGtkData;
begin
  result := PgtkData(FGtkObject);
end;


function TFPgtkData.ConnectDisconnect (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnect (sgDisconnect, proc, data);
end;

function TFPgtkData.ConnectAfterDisconnect (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnectAfter (sgDisconnect, proc, data);
end;

 { TFPgtkAdjustment }

function TFPgtkAdjustment.TheGtkObject : PGtkAdjustment;
begin
  result := PgtkAdjustment(FGtkObject);
end;

procedure TFPgtkAdjustment.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_Adjustment_new (0,0,10,1,2,2));
end;


procedure TFPgtkAdjustment.Configure (aLower:gfloat; anUpper:gfloat; aValue:gfloat; aStepInc:gfloat; aPageInc:gfloat; aPageSize:gfloat);
begin
  Lower := aLower;
  Upper := anUpper;
  Value := aValue;
  StepIncrement := aStepInc;
  PageIncrement := aPageInc;
  PageSize := aPageSize;
end;

function TFPgtkAdjustment.ConnectValueChanged (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnect (sgValueChanged, proc, data);
end;

function TFPgtkAdjustment.ConnectAfterValueChanged (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnectAfter (sgValueChanged, proc, data);
end;

function TFPgtkAdjustment.ConnectChanged (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnect (sgChanged, proc, data);
end;

function TFPgtkAdjustment.ConnectAfterChanged (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnectAfter (sgChanged, proc, data);
end;

procedure TFPgtkAdjustment.ValueChanged;
begin
  gtk_Adjustment_Value_Changed (TheGtkObject);
end;

procedure TFPgtkAdjustment.Changed;
begin
  gtk_Adjustment_Changed (TheGtkObject);
end;

procedure TFPgtkAdjustment.ClampPage (aLower:gfloat; aUpper:gfloat);
begin
  gtk_Adjustment_Clamp_Page (TheGtkObject, aLower, aUpper);
end;

function TFPgtkAdjustment.GetValue : gfloat;
begin
  result := TheGtkObject^.Value;
end;

procedure TFPgtkAdjustment.SetValue (TheValue:gfloat);
begin
  gtk_Adjustment_set_Value(TheGtkObject,TheValue);
end;

function TFPgtkAdjustment.GetLower : gfloat;
begin
  result := TheGtkObject^.Lower;
end;

procedure TFPgtkAdjustment.SetLower (TheValue:gfloat);
begin
  TheGtkObject^.Lower := TheValue;
end;

function TFPgtkAdjustment.GetUpper : gfloat;
begin
  result := TheGtkObject^.Upper;
end;

procedure TFPgtkAdjustment.SetUpper (TheValue:gfloat);
begin
  TheGtkObject^.Upper := TheValue;
end;

function TFPgtkAdjustment.GetStepIncrement : gfloat;
begin
  result := TheGtkObject^.Step_Increment;
end;

procedure TFPgtkAdjustment.SetStepIncrement (TheValue:gfloat);
begin
  TheGtkObject^.Step_Increment := TheValue;
end;

function TFPgtkAdjustment.GetPageIncrement : gfloat;
begin
  result := TheGtkObject^.Page_Increment;
end;

procedure TFPgtkAdjustment.SetPageIncrement (TheValue:gfloat);
begin
  TheGtkObject^.Page_increment := TheValue;
end;

function TFPgtkAdjustment.GetPageSize : gfloat;
begin
  result := TheGtkObject^.Page_Size;
end;

procedure TFPgtkAdjustment.SetPageSize (TheValue:gfloat);
begin
  TheGtkObject^.Page_Size := TheValue;
end;

 { TFPgtkToolTips }

function TFPgtkToolTips.TheGtkObject : PGtkToolTips;
begin
  result := PgtkToolTips(FGtkObject);
end;

procedure TFPgtkToolTips.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_tooltips_new);
end;


procedure TFPgtkToolTips.SetColors (Fore:PGdkColor; Back:PGdkColor);
begin
  gtk_tooltips_set_colors (TheGtkObject, Fore, Back);
end;

procedure TFPgtkToolTips.SetTip (Widget:TFPgtkWidget; TipText:string; TipPrivate:string);
begin
  gtk_tooltips_set_tip (TheGtkObject, PGtkwidget(ConvertToGtkObject(Widget)), ConvertToPgchar(TipText), ConvertToPgchar(TipPrivate));
end;

function TFPgtkToolTips.GetEnabled : boolean;
begin
  result := boolean(gtk.enabled(TheGtkObject^));
end;

procedure TFPgtkToolTips.SetEnabled (TheValue:boolean);
begin
  if TheValue then
    gtk_tooltips_enable (TheGtkObject)
  else
    gtk_tooltips_disable (TheGtkObject);
end;

function TFPgtkToolTips.GetDelay : integer;
begin
  result := gtk.delay(TheGtkObject^);
end;

procedure TFPgtkToolTips.SetDelay (TheValue:integer);
begin
  gtk_tooltips_set_delay(TheGtkObject,TheValue);
end;

function TFPgtkToolTips.GetColorForeground : PGdkColor;
begin
  result := TheGtkObject^.foreground;
end;

procedure TFPgtkToolTips.SetColorForeground (TheValue:PGdkColor);
begin
  SetColors (TheValue, ColorBackGround);
end;

function TFPgtkToolTips.GetColorBackground : PGdkColor;
begin
  result := TheGtkObject^.background;
end;

procedure TFPgtkToolTips.SetColorBackground (TheValue:PGdkColor);
begin
  SetColors (ColorForeground, TheValue);
end;

function GetTooltipsData (Widget:TFPgtkWidget) : PGtkTooltipsData;
begin
  result := gtk_tooltips_data_get (ConvertToGtkWidget(Widget));
end;


function ComposeTooltip (TooltipText:string; PrivText:string) : string;
begin
  result := TooltipText;
  if PrivText <> '' then
    result := result + '|' + PrivText;
end;


procedure DecomposeTooltip (Tooltip:string; var TooltipText:string; var PrivText:string);
var r : integer;
begin
  r := pos ('|', tooltip);
  if r > 0 then
    begin
    TooltipText := copy(Tooltip, 1, r-1);
    PrivText := copy (Tooltip, r+1, maxint);
    end
  else
    begin
    TooltipText := Tooltip;
    PrivText := '';
    end;
end;


procedure CheckTooltips;
begin
if not assigned (TheTooltips) then
  TheTooltips := TFPgtkTooltips.Create;
end;


procedure TFPgtkToolTips.ForceWindow;
begin
  gtk_tooltips_force_window (TheGtkObject);
end;

 { TFPgtkWidget }

function TFPgtkWidget.TheGtkObject : PGtkWidget;
begin
  result := PgtkWidget(FGtkObject);
end;


function TFPgtkWidget.GetTheGtkWidget : PGtkWidget;
begin
  result := PGtkWidget (TheGtkObject);
end;

procedure TFPgtkWidget.SetTheGtkWidget (TheValue:PGtkWidget);
begin
  FGtkObject := PgtkObject (TheValue);
end;

function GetPascalInstance (Widget:PGtkWidget) : TFPgtkWidget; Overload;
begin
  result := TFPgtkWidget (GetPascalInstance (PGtkObject(widget)));
end;


function GetPascalInstance (Widget:PGtkWidget; ObjClass:TFPgtkObjectClass) : TFPgtkWidget; Overload;
begin
  result := TFPgtkWidget (GetPascalInstance (PGtkObject(Widget), ObjClass));
end;


function ConvertToGtkWidget (AnObject:TFPgtkWidget) : PGtkWidget;
begin
  if assigned(AnObject) then
    result := AnObject.TheGtkWidget
  else
    result := nil;
end;


procedure WidgetSignalproc (Sender:PGtkobject; Widget:PGtkwidget; Data:pointer); cdecl;
var p : TFPgtkWidgetSignalFunction;
begin
with PSignalData(data)^ do
  begin
  p := TFPgtkWidgetSignalFunction (TheSignalProc);
  p (TheWidget as TFPgtkObject, GetPascalInstance(PGtkObject(Widget),TFPgtkwidget) as TFPgtkwidget, TheData)
  end;
end;

function TFPgtkWidget.WidgetSignalConnect (signal:string; proc:TFPgtkWidgetSignalFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect (FGtkObject, pgChar(signal), gtk_signal_func(@WidgetSignalproc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

function TFPgtkWidget.WidgetSignalConnectAfter (signal:string; proc:TFPgtkWidgetSignalFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect_After (FGtkObject, pgChar(signal), gtk_signal_func(@WidgetSignalproc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

procedure TFPgtkWidget.SetFlags (NewFlags:longint);
begin
  gtk_widget_set_flags (TheGtkObject, NewFlags);
end;

procedure TFPgtkWidget.UnsetFlags (NewUnsetFlags:longint);
begin
  gtk_widget_unset_flags (TheGtkObject, NewUnsetFlags);
end;

procedure TFPgtkWidget.Map;
begin
  gtk_widget_map (TheGtkObject);
end;

procedure TFPgtkWidget.Unmap;
begin
  gtk_widget_unmap (TheGtkObject);
end;

procedure TFPgtkWidget.QueueDraw;
begin
  gtk_widget_queue_draw (TheGtkObject);
end;

procedure TFPgtkWidget.QueueResize;
begin
  gtk_widget_queue_resize (TheGtkObject);
end;

procedure TFPgtkWidget.Draw (Rectangle:PGdkRectangle); Overload;
begin
  gtk_widget_draw (TheGtkObject, Rectangle);
end;

procedure TFPgtkWidget.DrawFocus;
begin
  gtk_widget_draw_focus (TheGtkObject);
end;

procedure TFPgtkWidget.DrawDefault;
begin
  gtk_widget_draw_default (TheGtkObject);
end;

procedure TFPgtkWidget.Show;
begin
  gtk_widget_show (TheGtkObject);
end;

procedure TFPgtkWidget.Hide;
begin
  gtk_widget_hide (TheGtkObject);
end;

procedure TFPgtkWidget.Realize;
begin
  gtk_widget_realize (TheGtkObject);
end;

procedure TFPgtkWidget.Unrealize;
begin
  gtk_widget_unrealize (TheGtkObject);
end;

procedure TFPgtkWidget.ShowNow;
begin
  gtk_widget_show_now (TheGtkObject);
end;

procedure TFPgtkWidget.ShowAll;
begin
  gtk_widget_show_all (TheGtkObject);
end;

procedure TFPgtkWidget.HideAll;
begin
  gtk_widget_hide_all (TheGtkObject);
end;

procedure TFPgtkWidget.SetAllocation (AnAllocation:TGtkAllocation); Overload;
begin
  with AnAllocation do
    SetAllocation (x, y, width, height);
end;

procedure TFPgtkWidget.SetAllocation (x:integer; y:integer; width:integer; height:integer); Overload;
begin
  SetUPosition (x, y);
  SetUSize (width, height);
end;

function TFPgtkWidget.GetAllocation : TGtkAllocation;
begin
  result := TheGtkObject^.allocation;
end;

procedure TFPgtkWidget.SetUPosition (x:integer; y:integer);
begin
  gtk_widget_set_uposition (TheGtkObject, x, y);
end;

procedure TFPgtkWidget.SetUsize (width:integer; height:integer);
begin
  gtk_widget_set_usize (TheGtkObject, width, height);
end;

function TFPgtkWidget.GetName : string;
begin
  result := gtk_widget_get_name(TheGtkObject);
end;

procedure TFPgtkWidget.SetName (TheValue:string);
begin
  gtk_widget_set_name(TheGtkObject,ConvertToPgchar(TheValue));
end;

function TFPgtkWidget.GetPropFlags : longint;
begin
  result := gtk_widget_Flags (TheGtkObject);
end;

procedure TFPgtkWidget.SetPropFlags (TheValue:longint);
var f : integer;
begin
  f := GetPropFlags;
  UnsetFlags (f and not TheValue);
  SetFlags (not f and TheValue);
end;

function TFPgtkWidget.GetState : longint;
begin
  result := gtk_widget_State(TheGtkObject);
end;

function TFPgtkWidget.GetSavedState : longint;
begin
  result := gtk_widget_Saved_State(TheGtkObject);
end;

function TFPgtkWidget.GetParent : TFPgtkWidget;
var gtkparent : PgtkWidget;
    o : TFPgtkObject;
begin
  gtkParent := TheGtkObject^.parent;
  o := GetPascalInstance (PgtkObject(GtkParent));
  if o is TFPgtkWidget then
    result := TFPgtkWidget(o)
  else
    result := nil;
end;

procedure TFPgtkWidget.SetParent (TheValue:TFPgtkWidget);
var gtkparent : PgtkWidget;
begin
  gtkParent := TheGtkObject^.parent;
  if assigned(TheValue) then
    if assigned(gtkParent) then
      reparent (TheValue)
    else
      gtk_widget_set_parent (TheGtkWidget, ConvertToGtkWidget(TheValue))
  else
    if assigned(gtkParent) then
      gtk_widget_unparent (TheGtkWidget);
end;


function TFPgtkWidget.GetParentWindow : PGdkWindow;
begin
  result := gtk_widget_get_parent_window(TheGtkObject);
end;

procedure TFPgtkWidget.SetParentWindow (TheValue:PGdkWindow);
begin
  gtk_widget_set_parent_window(TheGtkObject,TheValue);
end;

procedure TFPgtkWidget.Unparent;
begin
  gtk_widget_unparent (TheGtkObject);
end;

procedure TFPgtkWidget.Reparent (NewParent:TFPgtkWidget);
begin
  if (NewParent is TFpgtkContainer) then
    begin
    ref;
    TFPgtkContainer(Parent).remove (self);
    TFPgtkContainer(NewParent).Add (Self);
    unref;
    end;
end;

function TFPgtkWidget.GetVisible : boolean;
begin
  result := gtk_widget_Visible(TheGtkObject);
end;

procedure TFPgtkWidget.SetVisible (TheValue:boolean);
begin
  if TheValue then
    Show
  else
    Hide;
end;

function TFPgtkWidget.GetNoWindow : boolean;
begin
  result := gtk_widget_No_Window(TheGtkObject);
end;

procedure TFPgtkWidget.SetNoWindow (TheValue:boolean);
begin
  if TheValue then
    SetFlags (GTK_NO_WINDOW)
  else
    UnSetFlags (GTK_NO_WINDOW);
end;

function TFPgtkWidget.GetRealized : boolean;
begin
  result := gtk_widget_realized(TheGtkObject);
end;

procedure TFPgtkWidget.SetRealized (TheValue:boolean);
begin
  if TheValue then
    Realize
  else
    Unrealize;
end;

function TFPgtkWidget.GetMapped : boolean;
begin
  result := gtk_widget_Mapped(TheGtkObject);
end;

procedure TFPgtkWidget.SetMapped (TheValue:boolean);
begin
  if TheValue then
    Map
  else
    Unmap;
end;

function TFPgtkWidget.GetDrawable : boolean;
begin
  result := gtk_widget_Drawable(TheGtkObject);
end;

function TFPgtkWidget.GetIsSensitive : boolean;
begin
  result := gtk_widget_Is_Sensitive(TheGtkObject);
end;

function TFPgtkWidget.GetSensitive : boolean;
begin
  result := gtk_widget_Sensitive(TheGtkObject);
end;

procedure TFPgtkWidget.SetSensitive (TheValue:boolean);
begin
  gtk_widget_set_sensitive(TheGtkObject,TheValue);
end;

function TFPgtkWidget.GetParentSensitive : boolean;
begin
  result := gtk_widget_Parent_Sensitive(TheGtkObject);
end;

procedure TFPgtkWidget.SetParentSensitive (TheValue:boolean);
begin
  if TheValue then
    SetFlags (GTK_PARENT_SENSITIVE)
  else
    UnSetFlags (GTK_PARENT_SENSITIVE);
end;

function TFPgtkWidget.GetAppPaintable : boolean;
begin
  result := gtk_widget_App_Paintable(TheGtkObject);
end;

function TFPgtkWidget.GetCanFocus : boolean;
begin
  result := gtk_widget_Can_Focus(TheGtkObject);
end;

procedure TFPgtkWidget.SetCanFocus (TheValue:boolean);
begin
  if TheValue then
    SetFlags (GTK_CAN_FOCUS)
  else
    UnSetFlags (GTK_CAN_FOCUS);
end;

procedure TFPgtkWidget.GrabFocus;
begin
  gtk_widget_grab_focus (TheGtkObject);
end;

function TFPgtkWidget.GetHasFocus : boolean;
begin
  result := gtk_widget_Has_Focus(TheGtkObject);
end;

function TFPgtkWidget.GetCanDefault : boolean;
begin
  result := gtk_widget_Can_Default(TheGtkObject);
end;

procedure TFPgtkWidget.SetCanDefault (TheValue:boolean);
begin
  if TheValue then
    SetFlags (GTK_CAN_DEFAULT)
  else
    UnSetFlags (GTK_CAN_DEFAULT);
end;

procedure TFPgtkWidget.GrabDefault;
begin
  gtk_widget_grab_default (TheGtkObject);
end;

function TFPgtkWidget.GetHasDefault : boolean;
begin
  result := gtk_widget_Has_Default(TheGtkObject);
end;

function TFPgtkWidget.GetReceivesDefault : boolean;
begin
  result := gtk_widget_Receives_Default(TheGtkObject);
end;

function TFPgtkWidget.GetCompositeChild : boolean;
begin
  result := gtk_widget_Composite_Child(TheGtkObject);
end;

function TFPgtkWidget.GetTooltip : string;
var data : PGtkTooltipsData;
begin
  data := Gtk_Tooltips_Data_Get (TheGtkObject);
  if assigned(data) then
    with data^ do
      result := ComposeTooltip (Tip_Text, tip_private)
  else
    result := '';
end;

procedure TFPgtkWidget.SetTooltip (TheValue:string);
var t, p : string;
    ttdata : PGtkTooltipsData;
begin
  if TheValue = '' then
    begin
    ttdata := GetTooltipsData (Self);
    if assigned (ttdata) then
      ; // find a way to remove the hint. Setting '' does not remove
    end
  else
    begin
    CheckTooltips;
    DecomposeTooltip (TheValue, t, p);
    TheToolTips.SetTip (self, t, p);
    end;
end;

procedure TFPgtkWidget.HideOnDelete;
begin
  gtk_widget_hide_on_delete (TheGtkObject);
end;

function TFPgtkWidget.GetColormap : PGdkColormap;
begin
  result := gtk_widget_get_colormap(TheGtkObject);
end;

procedure TFPgtkWidget.SetColormap (TheValue:PGdkColormap);
begin
  gtk_widget_set_colormap(TheGtkObject,TheValue);
end;

function TFPgtkWidget.ConnectShow (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnect (sgShow, proc, data);
end;

function TFPgtkWidget.ConnectAfterShow (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnectAfter (sgShow, proc, data);
end;

function TFPgtkWidget.Connecthide (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnect (sghide, proc, data);
end;

function TFPgtkWidget.ConnectAfterhide (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnectAfter (sghide, proc, data);
end;

function TFPgtkWidget.Connectmap (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnect (sgmap, proc, data);
end;

function TFPgtkWidget.ConnectAftermap (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnectAfter (sgmap, proc, data);
end;

function TFPgtkWidget.Connectunmap (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnect (sgunmap, proc, data);
end;

function TFPgtkWidget.ConnectAfterunmap (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnectAfter (sgunmap, proc, data);
end;

function TFPgtkWidget.Connectrealize (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnect (sgrealize, proc, data);
end;

function TFPgtkWidget.ConnectAfterrealize (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnectAfter (sgrealize, proc, data);
end;

function TFPgtkWidget.Connectunrealize (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnect (sgunrealize, proc, data);
end;

function TFPgtkWidget.ConnectAfterunrealize (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnectAfter (sgunrealize, proc, data);
end;

function TFPgtkWidget.ConnectDrawFocus (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnect (sgDrawFocus, proc, data);
end;

function TFPgtkWidget.ConnectAfterDrawFocus (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnectAfter (sgDrawFocus, proc, data);
end;

function TFPgtkWidget.ConnectDrawDefault (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnect (sgDrawDefault, proc, data);
end;

function TFPgtkWidget.ConnectAfterDrawDefault (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnectAfter (sgDrawDefault, proc, data);
end;

function TFPgtkWidget.ConnectParentSet (proc:TFPgtkWidgetSignalFunction; data:pointer) : guint;
begin
  result := WidgetSignalConnect (sgParentSet, proc, data);
end;

function TFPgtkWidget.ConnectAfterParentSet (proc:TFPgtkWidgetSignalFunction; data:pointer) : guint;
begin
  result := WidgetSignalConnectAfter (sgParentSet, proc, data);
end;

function TFPgtkWidget.ConnectGrabFocus (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnect (sgGrabFocus, proc, data);
end;

function TFPgtkWidget.ConnectAfterGrabFocus (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnectAfter (sgGrabFocus, proc, data);
end;

function Eventfunc (Sender:PGtkwidget; Event:PGdkEvent; data:pointer) : boolean; cdecl;
var p : TFPgtkEventFunction;
begin
with PSignalData(data)^ do
  begin
  p := TFPgtkEventFunction (TheSignalProc);
  result := p (TheWidget as TFPgtkWidget, Event, TheData)
  end;
end;

function TFPgtkWidget.EventConnect (signal:string; proc:TFPgtkEventFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect (FGtkObject, pgChar(signal), gtk_signal_func(@Eventfunc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

function TFPgtkWidget.EventConnectAfter (signal:string; proc:TFPgtkEventFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect_After (FGtkObject, pgChar(signal), gtk_signal_func(@Eventfunc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

function TFPgtkWidget.ConnectEvent (proc:TFPgtkEventFunction; data:pointer) : guint;
begin
  result := EventConnect (sgEvent, proc, data);
end;

function TFPgtkWidget.ConnectAfterEvent (proc:TFPgtkEventFunction; data:pointer) : guint;
begin
  result := EventConnectAfter (sgEvent, proc, data);
end;

function EventButtonfunc (Sender:PGtkwidget; Event:PGdkEventButton; data:pointer) : boolean; cdecl;
var p : TFPgtkEventButtonFunction;
begin
with PSignalData(data)^ do
  begin
  p := TFPgtkEventButtonFunction (TheSignalProc);
  result := p (TheWidget as TFPgtkWidget, Event, TheData)
  end;
end;

function TFPgtkWidget.EventButtonConnect (signal:string; proc:TFPgtkEventButtonFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect (FGtkObject, pgChar(signal), gtk_signal_func(@EventButtonfunc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

function TFPgtkWidget.EventButtonConnectAfter (signal:string; proc:TFPgtkEventButtonFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect_After (FGtkObject, pgChar(signal), gtk_signal_func(@EventButtonfunc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

function TFPgtkWidget.ConnectButtonPressEvent (proc:TFPgtkEventButtonFunction; data:pointer) : guint;
begin
  result := EventButtonConnect (sgButtonPressEvent, proc, data);
end;

function TFPgtkWidget.ConnectAfterButtonPressEvent (proc:TFPgtkEventButtonFunction; data:pointer) : guint;
begin
  result := EventButtonConnectAfter (sgButtonPressEvent, proc, data);
end;

function TFPgtkWidget.ConnectButtonReleaseEvent (proc:TFPgtkEventButtonFunction; data:pointer) : guint;
begin
  result := EventButtonConnect (sgButtonReleaseEvent, proc, data);
end;

function TFPgtkWidget.ConnectAfterButtonReleaseEvent (proc:TFPgtkEventButtonFunction; data:pointer) : guint;
begin
  result := EventButtonConnectAfter (sgButtonReleaseEvent, proc, data);
end;

function EventMotionfunc (Sender:PGtkwidget; Event:PGdkEventMotion; data:pointer) : boolean; cdecl;
var p : TFPgtkEventMotionFunction;
begin
with PSignalData(data)^ do
  begin
  p := TFPgtkEventMotionFunction (TheSignalProc);
  result := p (TheWidget as TFPgtkWidget, Event, TheData)
  end;
end;

function TFPgtkWidget.EventMotionConnect (signal:string; proc:TFPgtkEventMotionFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect (FGtkObject, pgChar(signal), gtk_signal_func(@EventMotionfunc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

function TFPgtkWidget.EventMotionConnectAfter (signal:string; proc:TFPgtkEventMotionFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect_After (FGtkObject, pgChar(signal), gtk_signal_func(@EventMotionfunc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

function TFPgtkWidget.ConnectMotionNotifyEvent (proc:TFPgtkEventMotionFunction; data:pointer) : guint;
begin
  result := EventMotionConnect (sgMotionNotifyEvent, proc, data);
end;

function TFPgtkWidget.ConnectAfterMotionNotifyEvent (proc:TFPgtkEventMotionFunction; data:pointer) : guint;
begin
  result := EventMotionConnectAfter (sgMotionNotifyEvent, proc, data);
end;

function TFPgtkWidget.ConnectDeleteEvent (proc:TFPgtkEventFunction; data:pointer) : guint;
begin
  result := EventConnect (sgDeleteEvent, proc, data);
end;

function TFPgtkWidget.ConnectAfterDeleteEvent (proc:TFPgtkEventFunction; data:pointer) : guint;
begin
  result := EventConnectAfter (sgDeleteEvent, proc, data);
end;

function TFPgtkWidget.ConnectDestroyEvent (proc:TFPgtkEventFunction; data:pointer) : guint;
begin
  result := EventConnect (sgDestroyEvent, proc, data);
end;

function TFPgtkWidget.ConnectAfterDestroyEvent (proc:TFPgtkEventFunction; data:pointer) : guint;
begin
  result := EventConnectAfter (sgDestroyEvent, proc, data);
end;

function EventExposefunc (Sender:PGtkwidget; Event:PGdkEventExpose; data:pointer) : boolean; cdecl;
var p : TFPgtkEventExposeFunction;
begin
with PSignalData(data)^ do
  begin
  p := TFPgtkEventExposeFunction (TheSignalProc);
  result := p (TheWidget as TFPgtkWidget, Event, TheData)
  end;
end;

function TFPgtkWidget.EventExposeConnect (signal:string; proc:TFPgtkEventExposeFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect (FGtkObject, pgChar(signal), gtk_signal_func(@EventExposefunc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

function TFPgtkWidget.EventExposeConnectAfter (signal:string; proc:TFPgtkEventExposeFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect_After (FGtkObject, pgChar(signal), gtk_signal_func(@EventExposefunc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

function TFPgtkWidget.ConnectExposeEvent (proc:TFPgtkEventExposeFunction; data:pointer) : guint;
begin
  result := EventExposeConnect (sgExposeEvent, proc, data);
end;

function TFPgtkWidget.ConnectAfterExposeEvent (proc:TFPgtkEventExposeFunction; data:pointer) : guint;
begin
  result := EventExposeConnectAfter (sgExposeEvent, proc, data);
end;

function EventKeyfunc (Sender:PGtkwidget; Event:PGdkEventKey; data:pointer) : boolean; cdecl;
var p : TFPgtkEventKeyFunction;
begin
with PSignalData(data)^ do
  begin
  p := TFPgtkEventKeyFunction (TheSignalProc);
  result := p (TheWidget as TFPgtkWidget, Event, TheData)
  end;
end;

function TFPgtkWidget.EventKeyConnect (signal:string; proc:TFPgtkEventKeyFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect (FGtkObject, pgChar(signal), gtk_signal_func(@EventKeyfunc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

function TFPgtkWidget.EventKeyConnectAfter (signal:string; proc:TFPgtkEventKeyFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect_After (FGtkObject, pgChar(signal), gtk_signal_func(@EventKeyfunc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

function TFPgtkWidget.ConnectKeyPressEvent (proc:TFPgtkEventKeyFunction; data:pointer) : guint;
begin
  result := EventKeyConnect (sgKeyPressEvent, proc, data);
end;

function TFPgtkWidget.ConnectAfterKeyPressEvent (proc:TFPgtkEventKeyFunction; data:pointer) : guint;
begin
  result := EventKeyConnectAfter (sgKeyPressEvent, proc, data);
end;

function TFPgtkWidget.ConnectKeyReleaseEvent (proc:TFPgtkEventKeyFunction; data:pointer) : guint;
begin
  result := EventKeyConnect (sgKeyReleaseEvent, proc, data);
end;

function TFPgtkWidget.ConnectAfterKeyReleaseEvent (proc:TFPgtkEventKeyFunction; data:pointer) : guint;
begin
  result := EventKeyConnectAfter (sgKeyReleaseEvent, proc, data);
end;

function EventCrossingfunc (Sender:PGtkwidget; Event:PGdkEventCrossing; data:pointer) : boolean; cdecl;
var p : TFPgtkEventCrossingFunction;
begin
with PSignalData(data)^ do
  begin
  p := TFPgtkEventCrossingFunction (TheSignalProc);
  result := p (TheWidget as TFPgtkWidget, Event, TheData)
  end;
end;

function TFPgtkWidget.EventCrossingConnect (signal:string; proc:TFPgtkEventCrossingFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect (FGtkObject, pgChar(signal), gtk_signal_func(@EventCrossingfunc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

function TFPgtkWidget.EventCrossingConnectAfter (signal:string; proc:TFPgtkEventCrossingFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect_After (FGtkObject, pgChar(signal), gtk_signal_func(@EventCrossingfunc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

function TFPgtkWidget.ConnectEnterNotifyEvent (proc:TFPgtkEventCrossingFunction; data:pointer) : guint;
begin
  result := EventCrossingConnect (sgEnterNotifyEvent, proc, data);
end;

function TFPgtkWidget.ConnectAfterEnterNotifyEvent (proc:TFPgtkEventCrossingFunction; data:pointer) : guint;
begin
  result := EventCrossingConnectAfter (sgEnterNotifyEvent, proc, data);
end;

function TFPgtkWidget.ConnectLeaveNotifyEvent (proc:TFPgtkEventCrossingFunction; data:pointer) : guint;
begin
  result := EventCrossingConnect (sgLeaveNotifyEvent, proc, data);
end;

function TFPgtkWidget.ConnectAfterLeaveNotifyEvent (proc:TFPgtkEventCrossingFunction; data:pointer) : guint;
begin
  result := EventCrossingConnectAfter (sgLeaveNotifyEvent, proc, data);
end;

function EventConfigurefunc (Sender:PGtkwidget; Event:PGdkEventConfigure; data:pointer) : boolean; cdecl;
var p : TFPgtkEventConfigureFunction;
begin
with PSignalData(data)^ do
  begin
  p := TFPgtkEventConfigureFunction (TheSignalProc);
  result := p (TheWidget as TFPgtkWidget, Event, TheData)
  end;
end;

function TFPgtkWidget.EventConfigureConnect (signal:string; proc:TFPgtkEventConfigureFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect (FGtkObject, pgChar(signal), gtk_signal_func(@EventConfigurefunc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

function TFPgtkWidget.EventConfigureConnectAfter (signal:string; proc:TFPgtkEventConfigureFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect_After (FGtkObject, pgChar(signal), gtk_signal_func(@EventConfigurefunc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

function TFPgtkWidget.ConnectConfigureEvent (proc:TFPgtkEventConfigureFunction; data:pointer) : guint;
begin
  result := EventConfigureConnect (sgConfigureEvent, proc, data);
end;

function TFPgtkWidget.ConnectAfterConfigureEvent (proc:TFPgtkEventConfigureFunction; data:pointer) : guint;
begin
  result := EventConfigureConnectAfter (sgConfigureEvent, proc, data);
end;

function EventFocusfunc (Sender:PGtkwidget; Event:PGdkEventFocus; data:pointer) : boolean; cdecl;
var p : TFPgtkEventFocusFunction;
begin
with PSignalData(data)^ do
  begin
  p := TFPgtkEventFocusFunction (TheSignalProc);
  result := p (TheWidget as TFPgtkWidget, Event, TheData)
  end;
end;

function TFPgtkWidget.EventFocusConnect (signal:string; proc:TFPgtkEventFocusFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect (FGtkObject, pgChar(signal), gtk_signal_func(@EventFocusfunc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

function TFPgtkWidget.EventFocusConnectAfter (signal:string; proc:TFPgtkEventFocusFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect_After (FGtkObject, pgChar(signal), gtk_signal_func(@EventFocusfunc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

function TFPgtkWidget.ConnectFocusInEvent (proc:TFPgtkEventFocusFunction; data:pointer) : guint;
begin
  result := EventFocusConnect (sgFocusInEvent, proc, data);
end;

function TFPgtkWidget.ConnectAfterFocusInEvent (proc:TFPgtkEventFocusFunction; data:pointer) : guint;
begin
  result := EventFocusConnectAfter (sgFocusInEvent, proc, data);
end;

function TFPgtkWidget.ConnectFocusOutEvent (proc:TFPgtkEventFocusFunction; data:pointer) : guint;
begin
  result := EventFocusConnect (sgFocusOutEvent, proc, data);
end;

function TFPgtkWidget.ConnectAfterFocusOutEvent (proc:TFPgtkEventFocusFunction; data:pointer) : guint;
begin
  result := EventFocusConnectAfter (sgFocusOutEvent, proc, data);
end;

function TFPgtkWidget.ConnectMapEvent (proc:TFPgtkEventFunction; data:pointer) : guint;
begin
  result := EventConnect (sgMapEvent, proc, data);
end;

function TFPgtkWidget.ConnectAfterMapEvent (proc:TFPgtkEventFunction; data:pointer) : guint;
begin
  result := EventConnectAfter (sgMapEvent, proc, data);
end;

function TFPgtkWidget.ConnectUnmapEvent (proc:TFPgtkEventFunction; data:pointer) : guint;
begin
  result := EventConnect (sgUnmapEvent, proc, data);
end;

function TFPgtkWidget.ConnectAfterUnmapEvent (proc:TFPgtkEventFunction; data:pointer) : guint;
begin
  result := EventConnectAfter (sgUnmapEvent, proc, data);
end;

function EventPropertyfunc (Sender:PGtkwidget; Event:PGdkEventProperty; data:pointer) : boolean; cdecl;
var p : TFPgtkEventPropertyFunction;
begin
with PSignalData(data)^ do
  begin
  p := TFPgtkEventPropertyFunction (TheSignalProc);
  result := p (TheWidget as TFPgtkWidget, Event, TheData)
  end;
end;

function TFPgtkWidget.EventPropertyConnect (signal:string; proc:TFPgtkEventPropertyFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect (FGtkObject, pgChar(signal), gtk_signal_func(@EventPropertyfunc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

function TFPgtkWidget.EventPropertyConnectAfter (signal:string; proc:TFPgtkEventPropertyFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect_After (FGtkObject, pgChar(signal), gtk_signal_func(@EventPropertyfunc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

function TFPgtkWidget.ConnectPropertyNotifyEvent (proc:TFPgtkEventPropertyFunction; data:pointer) : guint;
begin
  result := EventPropertyConnect (sgPropertyNotifyEvent, proc, data);
end;

function TFPgtkWidget.ConnectAfterPropertyNotifyEvent (proc:TFPgtkEventPropertyFunction; data:pointer) : guint;
begin
  result := EventPropertyConnectAfter (sgPropertyNotifyEvent, proc, data);
end;

function EventSelectionfunc (Sender:PGtkwidget; Event:PGdkEventSelection; data:pointer) : boolean; cdecl;
var p : TFPgtkEventSelectionFunction;
begin
with PSignalData(data)^ do
  begin
  p := TFPgtkEventSelectionFunction (TheSignalProc);
  result := p (TheWidget as TFPgtkWidget, Event, TheData)
  end;
end;

function TFPgtkWidget.EventSelectionConnect (signal:string; proc:TFPgtkEventSelectionFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect (FGtkObject, pgChar(signal), gtk_signal_func(@EventSelectionfunc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

function TFPgtkWidget.EventSelectionConnectAfter (signal:string; proc:TFPgtkEventSelectionFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect_After (FGtkObject, pgChar(signal), gtk_signal_func(@EventSelectionfunc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

function TFPgtkWidget.ConnectSelectionClearEvent (proc:TFPgtkEventSelectionFunction; data:pointer) : guint;
begin
  result := EventSelectionConnect (sgSelectionClearEvent, proc, data);
end;

function TFPgtkWidget.ConnectAfterSelectionClearEvent (proc:TFPgtkEventSelectionFunction; data:pointer) : guint;
begin
  result := EventSelectionConnectAfter (sgSelectionClearEvent, proc, data);
end;

function TFPgtkWidget.ConnectSelectionRequestEvent (proc:TFPgtkEventSelectionFunction; data:pointer) : guint;
begin
  result := EventSelectionConnect (sgSelectionRequestEvent, proc, data);
end;

function TFPgtkWidget.ConnectAfterSelectionRequestEvent (proc:TFPgtkEventSelectionFunction; data:pointer) : guint;
begin
  result := EventSelectionConnectAfter (sgSelectionRequestEvent, proc, data);
end;

function TFPgtkWidget.ConnectSelectionNotifyEvent (proc:TFPgtkEventSelectionFunction; data:pointer) : guint;
begin
  result := EventSelectionConnect (sgSelectionNotifyEvent, proc, data);
end;

function TFPgtkWidget.ConnectAfterSelectionNotifyEvent (proc:TFPgtkEventSelectionFunction; data:pointer) : guint;
begin
  result := EventSelectionConnectAfter (sgSelectionNotifyEvent, proc, data);
end;

function EventProximityfunc (Sender:PGtkwidget; Event:PGdkEventProximity; data:pointer) : boolean; cdecl;
var p : TFPgtkEventProximityFunction;
begin
with PSignalData(data)^ do
  begin
  p := TFPgtkEventProximityFunction (TheSignalProc);
  result := p (TheWidget as TFPgtkWidget, Event, TheData)
  end;
end;

function TFPgtkWidget.EventProximityConnect (signal:string; proc:TFPgtkEventProximityFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect (FGtkObject, pgChar(signal), gtk_signal_func(@EventProximityfunc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

function TFPgtkWidget.EventProximityConnectAfter (signal:string; proc:TFPgtkEventProximityFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect_After (FGtkObject, pgChar(signal), gtk_signal_func(@EventProximityfunc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

function TFPgtkWidget.ConnectProximityInEvent (proc:TFPgtkEventProximityFunction; data:pointer) : guint;
begin
  result := EventProximityConnect (sgProximityInEvent, proc, data);
end;

function TFPgtkWidget.ConnectAfterProximityInEvent (proc:TFPgtkEventProximityFunction; data:pointer) : guint;
begin
  result := EventProximityConnectAfter (sgProximityInEvent, proc, data);
end;

function TFPgtkWidget.ConnectProximityOutEvent (proc:TFPgtkEventProximityFunction; data:pointer) : guint;
begin
  result := EventProximityConnect (sgProximityOutEvent, proc, data);
end;

function TFPgtkWidget.ConnectAfterProximityOutEvent (proc:TFPgtkEventProximityFunction; data:pointer) : guint;
begin
  result := EventProximityConnectAfter (sgProximityOutEvent, proc, data);
end;

function EventClientfunc (Sender:PGtkwidget; Event:PGdkEventClient; data:pointer) : boolean; cdecl;
var p : TFPgtkEventClientFunction;
begin
with PSignalData(data)^ do
  begin
  p := TFPgtkEventClientFunction (TheSignalProc);
  result := p (TheWidget as TFPgtkWidget, Event, TheData)
  end;
end;

function TFPgtkWidget.EventClientConnect (signal:string; proc:TFPgtkEventClientFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect (FGtkObject, pgChar(signal), gtk_signal_func(@EventClientfunc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

function TFPgtkWidget.EventClientConnectAfter (signal:string; proc:TFPgtkEventClientFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect_After (FGtkObject, pgChar(signal), gtk_signal_func(@EventClientfunc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

function TFPgtkWidget.ConnectClientEvent (proc:TFPgtkEventClientFunction; data:pointer) : guint;
begin
  result := EventClientConnect (sgClientEvent, proc, data);
end;

function TFPgtkWidget.ConnectAfterClientEvent (proc:TFPgtkEventClientFunction; data:pointer) : guint;
begin
  result := EventClientConnectAfter (sgClientEvent, proc, data);
end;

function EventNoExposefunc (Sender:PGtkwidget; Event:PGdkEventNoExpose; data:pointer) : boolean; cdecl;
var p : TFPgtkEventNoExposeFunction;
begin
with PSignalData(data)^ do
  begin
  p := TFPgtkEventNoExposeFunction (TheSignalProc);
  result := p (TheWidget as TFPgtkWidget, Event, TheData)
  end;
end;

function TFPgtkWidget.EventNoExposeConnect (signal:string; proc:TFPgtkEventNoExposeFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect (FGtkObject, pgChar(signal), gtk_signal_func(@EventNoExposefunc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

function TFPgtkWidget.EventNoExposeConnectAfter (signal:string; proc:TFPgtkEventNoExposeFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect_After (FGtkObject, pgChar(signal), gtk_signal_func(@EventNoExposefunc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

function TFPgtkWidget.ConnectNoExposeEvent (proc:TFPgtkEventNoExposeFunction; data:pointer) : guint;
begin
  result := EventNoExposeConnect (sgNoExposeEvent, proc, data);
end;

function TFPgtkWidget.ConnectAfterNoExposeEvent (proc:TFPgtkEventNoExposeFunction; data:pointer) : guint;
begin
  result := EventNoExposeConnectAfter (sgNoExposeEvent, proc, data);
end;

function TFPgtkWidget.ConnectVisibilityNotifyEvent (proc:TFPgtkEventFunction; data:pointer) : guint;
begin
  result := EventConnect (sgVisibilityNotifyEvent, proc, data);
end;

function TFPgtkWidget.ConnectAfterVisibilityNotifyEvent (proc:TFPgtkEventFunction; data:pointer) : guint;
begin
  result := EventConnectAfter (sgVisibilityNotifyEvent, proc, data);
end;

procedure TFPgtkWidget.LockAccelerators;
begin
  gtk_widget_lock_accelerators (TheGtkObject);
end;

procedure TFPgtkWidget.UnlockAccelerators;
begin
  gtk_widget_unlock_accelerators (TheGtkObject);
end;

procedure TFPgtkWidget.RemoveAccelerators (aSignal:string; OnlyVisible:boolean);
begin
  gtk_widget_remove_accelerators (TheGtkObject, ConvertToPgchar(aSignal), OnlyVisible);
end;

procedure TFPgtkWidget.ActivateAccelGroups (Key:guint; Mods:TGdkModifierType);
begin
  gtk_accel_groups_activate (FGtkObject, Key, Mods);
end;

procedure TFPgtkWidget.AcceleratorAdd (AG:PGtkAccelGroup; aSignal:string; Key:guint; Mods:TGdkModifierType; acFlags:TGtkAccelFlags); Overload;
begin
  gtk_widget_add_accelerator (TheGtkWidget, pgchar(aSignal),
        AG, Key, Mods, acFlags);
end;

 { TFPgtkGroup }


procedure TFPgtkGroup.FreeList;
begin
  if FGList <> null then
    begin
    if FManageLists then
      g_list_free (FGList);
    FGList := null;
    end;
end;

procedure TFPgtkGroup.FreeSList;
begin
  if FGSList <> null then
    begin
    if FManageLists then
      g_slist_free (FGSList);
    FGSlist := null;
    end;
end;

function TFPgtkGroup.CreateGList : PGList;
var r : integer;
begin
  FreeList;
  result := null;
  for r := pred(count) downto 0 do
    result := g_list_prepend (result, GetData(r));
  FGList := result;
end;

function TFPgtkGroup.CreateGSList : PGSList;
var r : integer;
begin
  FreeSList;
  result := null;
  for r := pred(count) downto 0 do
    result := g_slist_prepend (result, GetData(r));
  FGSList := result;
end;

procedure TFPgtkGroup.BuildFromGtkList;
var p : PGList;
begin
  clear;
  p := FGList;
  while p <> null do
    begin
    add (UngetData(p^.data));
    p := p^.Next;
    end;
  FListChanged := False;
  FSListChanged := False;
  FClassesChanged := False;
  FreeSList;
end;

procedure TFPgtkGroup.BuildFromGtkSList;
var p :PGSList;
begin
  clear;
  p := FGSList;
  while p <> null do
    begin
    add (UngetData(p^.data));
    p := p^.Next;
    end;
  FListChanged := False;
  FSListChanged := False;
  FClassesChanged := False;
  FreeList;
end;

procedure TFPgtkGroup.Notify (ptr:pointer; Action:TListNotification);
begin
  inherited;
  FClassesChanged := True;
end;

function TFPgtkGroup.GetData (index:integer) : pointer;
// GetData needs to give the pointer to the data in the List or SList of GTK
begin
  result := items[index];
end;

function TFPgtkGroup.UngetData (data:pointer) : pointer;
// UngetData needs to give the item in this list from the datapointer of GTK
begin
  result := data
end;

constructor TFPgtkGroup.Create;
begin
  inherited create;
  FClassesChanged := False;
  FListChanged := false;
  FSListChanged := False;
  FGList := null;
  FGSList := null;
  FNotUpdating := True;
  FManageLists := True;
end;


destructor TFPgtkGroup.Destroy;
begin
  if ManageLists then
    begin
    FreeList;
    FreeSList;
    end;
  inherited Destroy;
end;


function TFPgtkGroup.GetGtkList (buffered:boolean) : PGList;
begin
  if buffered then
    if FClasseschanged then
      result := CreateGList
    else if FSListChanged then
      begin
      BuildFromGtkSList;
      result := CreateGList;
      end
    else
      result := FGlist
  else
    result := CreateGList;
end;

function TFPgtkGroup.GetGtkSList (buffered:boolean) : PGSList;
begin
  if buffered then
    if FClassesChanged then
      result := CreateGSList
    else if FListChanged then
      begin
      BuildFromGtkList;
      result := CreateGSList;
      end
    else
      result := FGSlist
  else
    result := CreateGSList;
end;

procedure TFPgtkGroup.BeginUpdate;
begin
  FNotUpdating := False;
end;

procedure TFPgtkGroup.EndUpdate;
begin
  FNotUpdating := True;
  if FlistChanged then
    BuildFromGtkSList
  else if FSListChanged then
    BuildFromGtkSList
  else if FClassesChanged then
    begin
    FreeSList;
    FreeList;
    end;
end;

procedure TFPgtkGroup.ForEach (Proc:TFPgtkForEachProcedure; data:pointer);
var r: integer;
begin
  for r := 0 to pred(count) do
    Proc (items[r], data);
end;

function TFPgtkGroup.GetGtkListProp : PGList;
begin
  result := GetGtkList (True);
end;

procedure TFPgtkGroup.SetGtkListProp (TheValue:PGList);
begin
  FGList := TheValue;
  if FNotUpdating then
    BuildFromGtkList
  else
    FListChanged := True;
end;

function TFPgtkGroup.GetGtkSListProp : PGSList;
begin
  result := GetGtkSList (True);
end;

procedure TFPgtkGroup.SetGtkSListProp (TheValue:PGSList);
begin
  FGSlist := TheValue;
  if FNotUpdating then
    BuildFromGtkSList
  else
    FSListChanged := True;
end;

 { TFPgtkWidgetGroup }


function TFPgtkWidgetGroup.GetData (index:integer) : pointer;
begin
  result := items[index].FgtkObject;
end;

function TFPgtkWidgetGroup.UnGetData (data:pointer) : pointer;
begin
  result := GetPascalInstance (PGtkObject(Data));
end;

procedure TFPgtkWidgetGroup.AddToContainer (Container:TFPgtkContainer);
var r : integer;
begin
  for r := 0 to pred(count) do
    Container.Add (items[r]);
end;

procedure TFPgtkWidgetGroup.PackInBox (box:TFPgtkBox; AtStart:boolean; Expanding:boolean; Fill:boolean; Padding:integer);
var r : integer;
begin
  if AtStart then
    for r := 0 to pred(Count) do
      box.PackStart (items[r], expanding, fill, padding)
  else
    for r := pred(Count) downto 0 do
      box.PackEnd (items[r], expanding, fill, padding);
end;

function TFPgtkWidgetGroup.GetItem (Index:integer) : TFPgtkWidget;
begin
  result := TFPgtkWidget (Inherited items[index]);
end;

procedure TFPgtkWidgetGroup.SetItem (Index:integer; TheValue:TFPgtkWidget);
begin
  inherited items[index] := TheValue;
end;

function TFPgtkWidgetGroup.GetTooltips (index:integer) : string;
begin
  result := items[index].Tooltip;
end;

procedure TFPgtkWidgetGroup.SetTooltips (index:integer; TheValue:string);
begin
  Items[index].Tooltip := TheValue;
end;

 { TFPgtkMisc }

function TFPgtkMisc.TheGtkObject : PGtkMisc;
begin
  result := PgtkMisc(FGtkObject);
end;


procedure TFPgtkMisc.SetAlignment (x:gfloat; y:gfloat);
begin
  gtk_misc_set_alignment (TheGtkObject, x, y);
end;

procedure TFPgtkMisc.SetPadding (x:word; y:word);
begin
  gtk_misc_set_padding (TheGtkObject, x, y);
end;

function TFPgtkMisc.GetXAlign : gfloat;
begin
  result := TheGtkObject^.XAlign;
end;

procedure TFPgtkMisc.SetXAlign (TheValue:gfloat);
begin
  SetAlignment (TheValue, YAlign);
end;

function TFPgtkMisc.GetYAlign : gfloat;
begin
  result := TheGtkObject^.YAlign;
end;

procedure TFPgtkMisc.SetYAlign (TheValue:gfloat);
begin
  SetAlignment (XAlign, TheValue);
end;

function TFPgtkMisc.GetXPad : word;
begin
  result := TheGtkObject^.XPad;
end;

procedure TFPgtkMisc.SetXPad (TheValue:word);
begin
  SetPadding (TheValue, YPad);
end;

function TFPgtkMisc.GetYPad : word;
begin
  result := TheGtkObject^.YPad;
end;

procedure TFPgtkMisc.SetYPad (TheValue:word);
begin
  SetPadding (XPad, TheValue);
end;

 { TFPgtkLabel }

function TFPgtkLabel.TheGtkObject : PGtkLabel;
begin
  result := PgtkLabel(FGtkObject);
end;

procedure TFPgtkLabel.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_label_new (null));
end;


constructor TFPgtkLabel.Create (aText:string);
begin
  inherited create;
  Text := aText;
  SetAlignment (0.0, 0.5);
end;


function TFPgtkLabel.GetText : string;
begin
  result := TheGtkObject^.TheLabel;
end;

procedure TFPgtkLabel.SetText (TheValue:string);
begin
  gtk_label_set_text(TheGtkObject,ConvertToPgchar(TheValue));
end;

function TFPgtkLabel.GetPattern : string;
begin
  result := TheGtkObject^.pattern;
end;

procedure TFPgtkLabel.SetPattern (TheValue:string);
begin
  gtk_label_set_pattern(TheGtkObject,ConvertToPgchar(TheValue));
end;

function TFPgtkLabel.GetJustify : TGtkJustification;
begin
  result := gtk.jtype(TheGtkObject^);
end;

procedure TFPgtkLabel.SetJustify (TheValue:TGtkJustification);
begin
  gtk_label_set_justify(TheGtkObject,TheValue);
end;

function TFPgtkLabel.GetLineWrap : boolean;
begin
  result := TheGtkObject^.wrap;
end;

procedure TFPgtkLabel.SetLineWrap (TheValue:boolean);
begin
  gtk_label_set_line_wrap(TheGtkObject,TheValue);
end;

function TFPgtkLabel.ParseUline (aText:string) : guint;
begin
  result := gtk_label_parse_uline (TheGtkObject, ConvertToPgchar(aText));
end;

 { TFPgtkAccelLabel }

function TFPgtkAccelLabel.TheGtkObject : PGtkAccelLabel;
begin
  result := PgtkAccelLabel(FGtkObject);
end;

procedure TFPgtkAccelLabel.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_accel_label_new (''));
end;


function TFPgtkAccelLabel.GetAccelWidget : TFPgtkWidget;
begin
  result := GetPascalInstance(PGtkObject(TheGtkObject^.accel_widget),tfpgtkwidget) as tfpgtkwidget;
end;

procedure TFPgtkAccelLabel.SetAccelWidget (TheValue:TFPgtkWidget);
begin
  gtk_accel_label_set_accel_widget(TheGtkObject,PGtkwidget(ConvertToGtkObject(TheValue)));
end;

function TFPgtkAccelLabel.AccelText : string;
begin
  result := TheGtkObject^.accel_string;
end;

procedure TFPgtkAccelLabel.Refetch;
begin
  gtk_accel_label_refetch (TheGtkObject);
end;

 { TFPgtkTipsQuery }

function TFPgtkTipsQuery.TheGtkObject : PGtkTipsQuery;
begin
  result := PgtkTipsQuery(FGtkObject);
end;

procedure TFPgtkTipsQuery.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_tips_query_new);
end;


 { TFPgtkArrow }

function TFPgtkArrow.TheGtkObject : PGtkArrow;
begin
  result := PgtkArrow(FGtkObject);
end;

procedure TFPgtkArrow.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_arrow_new (GTK_ARROW_LEFT,GTK_SHADOW_NONE));
end;


function TFPgtkArrow.GetArrowType : TGtkArrowType;
begin
  result := TGtkArrowType (TheGtkObject^.arrow_type);
end;

procedure TFPgtkArrow.SetArrowType (TheValue:TGtkArrowType);
begin
  gtk_arrow_set (TheGtkObject, TheValue, ShadowType);
end;

function TFPgtkArrow.GetShadowType : TGtkShadowType;
begin
  result := TGtkShadowtype (TheGtkObject^.shadow_type);
end;

procedure TFPgtkArrow.SetShadowType (TheValue:TGtkShadowType);
begin
  gtk_arrow_set (TheGtkObject, ArrowType, TheValue);
end;

procedure TFPgtkArrow.SetTypes (AnArrowType:TGtkArrowType; AShadowtype:TGtkShadowType);
begin
  gtk_arrow_set (TheGtkObject, AnArrowType, AShadowtype);
end;

constructor TFPgtkArrow.Create (AnArrowType:TGtkArrowType; AShadowType:TGtkShadowType);
begin
  inherited create;
  SetTypes (AnArrowType, AShadowType);
end;


 { TFPgtkImage }

function TFPgtkImage.TheGtkObject : PGtkImage;
begin
  result := PgtkImage(FGtkObject);
end;

procedure TFPgtkImage.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_image_new (FImage, FMask));
end;


function TFPgtkImage.GetImageProp : PGdkImage;
var m : PGdkBitmap;
begin
  gtk_image_get (TheGtkObject, @result, @m);
end;

procedure TFPgtkImage.SetImageProp (TheValue:PGdkImage);
begin
  gtk_Image_set (TheGtkObject, TheValue, nil);
end;

function TFPgtkImage.GetMask : PGdkBitMap;
var p : PGdkImage;
begin
  gtk_image_get (TheGtkObject, @p, @result);
end;

procedure TFPgtkImage.SetMask (TheValue:PGdkBitMap);
begin
  gtk_image_set (TheGtkObject, Image, TheValue);
end;

procedure TFPgtkImage.SetImage (anImage:PGdkImage; aMask:PGdkBitmap);
begin
  gtk_image_set (TheGtkObject, anImage, aMask);
end;

constructor TFPgtkImage.Create (anImage:PGdkImage; aMask:PGdkBitmap);
begin
  FImage := anImage;
  FMask := aMask;
  inherited create;
end;


function NewImage (aWidth:integer; aHeight:integer) : PGdkImage;
begin
  result := gdk_image_new (gdk_image_fastest, gdk_visual_get_system, aWidth, aHeight);
end;


 { TFPgtkPixmap }

function TFPgtkPixmap.TheGtkObject : PGtkPixmap;
begin
  result := PgtkPixmap(FGtkObject);
end;

procedure TFPgtkPixmap.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_pixmap_new (FPixMap, FMask));
end;

var
  EmptyBitmap : PGdkPixmap;

function StringsToPPgchar (Data:TStrings) : PPgchar;
var r : integer;
    a : PStringArray;
begin
  getmem (a, sizeof (pgchar) * Data.count);
  for r := 0 to Data.Count-1 do
    a^[r] := pchar (Data[r]);
  result := ppgchar (a);
end;


function ArrayToPPgchar (Data:array of string) : PPgchar;
var r,t : integer;
    a : PStringArray;
begin
  getmem (a, sizeof (pgchar) * (high(data)-low(data)+1));
  t := 0;
  for r := low(data) to high(data) do
    begin
    a^[r] := pchar (data[t]);
    inc (t);
    end;
  result := ppgchar (a);
end;


function TFPgtkPixmap.GetBuildInsensitive : longbool;
begin
  result := longbool(gtk.build_insensitive(TheGtkObject^));
end;

procedure TFPgtkPixmap.SetBuildInsensitive (TheValue:longbool);
begin
  gtk_pixmap_set_build_insensitive(TheGtkObject,gint(TheValue));
end;

constructor TFPgtkPixmap.Create;
begin
  if not assigned (EmptyBitmap) then
    EmptyBitmap := gdk_pixmap_new (null, 1, 1, 1);
  FPixMap := EmptyBitmap;
  FMask := PGdkBitmap (EmptyBitmap);
  inherited create;
end;


constructor TFPgtkPixmap.CreateFromFile (Filename:string; Window:TFPgtkWidget);
begin
  FPixMap := gdk_pixmap_create_from_xpm (ConvertToGtkWidget(Window)^.window, @FMask, nil, pgchar(Filename));
  inherited create;
end;


constructor TFPgtkPixmap.CreateFromStrings (Data:TStrings; Window:TFPgtkWidget);
var ppdata : ppgchar;
begin
  ppdata := StringsToPPgchar(Data);
  FPixMap := gdk_pixmap_create_from_xpm_d (ConvertToGtkWidget(Window)^.window, @FMask, nil, ppdata);
  inherited create;
  freemem (ppdata, sizeof (pgchar) * Data.count);
end;


constructor TFPgtkPixmap.CreateFromText (Data:string; Window:TFPgtkWidget);
var l : TStrings;
begin
  l := TStringList.Create;
  try
    l.Text := data;
    CreateFromStrings (l, Window);
  finally
    l.Free;
  end;
end;


function TFPgtkPixmap.GetPixmapProp : PGdkPixMap;
var m : PGdkBitmap;
begin
  gtk_pixmap_get (TheGtkObject, @result, @m);
end;

procedure TFPgtkPixmap.SetPixmapProp (TheValue:PGdkPixMap);
begin
  gtk_pixmap_set (TheGtkObject, TheValue, nil);
end;

function TFPgtkPixmap.GetMask : PGdkBitMap;
var p : PGdkPixmap;
begin
  gtk_pixmap_get (TheGtkObject, @p, @result);
end;

procedure TFPgtkPixmap.SetMask (TheValue:PGdkBitMap);
begin
  gtk_pixmap_set (TheGtkObject, Pixmap, TheValue);
end;

procedure TFPgtkPixmap.SetPixmap (aPixmap:PGdkPixMap; aMask:PGdkBitmap);
begin
  gtk_pixmap_set (TheGtkObject, aPixmap, aMask);
end;

procedure TFPgtkPixmap.GetPixmap (var aPixmap:PGdkPixmap; var aMask:PGdkBitmap);
var P:PGdkPixmap;
   M:PGdkBitmap;
begin
  gtk_pixmap_get (TheGtkObject, @p, @m);
  apixmap := p;
  amask := m;
end;

procedure TFPgtkPixmap.LoadFromFile (Filename:string);
var bm : PGdkBitmap;
    pm : PGdkPixmap;
begin
  pm := gdk_pixmap_colormap_create_from_xpm (nil, Colormap, @bm, nil, pgchar(Filename));
  SetPixmap (pm, bm);
end;

procedure TFPgtkPixmap.LoadFromStrings (data:TStrings);
var bm : PGdkBitmap;
    pm : PGdkPixmap;
    ppdata : ppgchar;
begin
  ppdata := StringsToPPgchar(Data);
  pm := gdk_pixmap_colormap_create_from_xpm_d (nil, Colormap, @bm, nil, ppdata);
  SetPixmap (pm, bm);
  freemem (ppdata, sizeof (pgchar) * Data.count);
end;

procedure TFPgtkPixmap.LoadFromText (data:string);
var l : TStrings;
begin
  l := TStringList.Create;
  try
    l.Text := data;
    LoadFromStrings (l);
  finally
    l.Free;
  end;
end;

procedure TFPgtkPixmap.LoadFromArray (data:array of string);
var bm : PGdkBitmap;
    pm : PGdkPixmap;
    ppdata : ppgchar;
begin
  ppdata := ArrayToPPgchar(Data);
  pm := gdk_pixmap_colormap_create_from_xpm_d (nil, Colormap, @bm, nil, ppdata);
  SetPixmap (pm, bm);
  freemem (ppdata, sizeof (pgchar) * (high(data)-low(data)+1));
end;

procedure CreateGdkPixmap (var ThePixmap:PGdkPixmap; var TheMask:PGdkBitmap; aWindow:PGdkWindow; data:array of string);
var ppdata : ppgchar;
begin
  ppdata := ArrayToPPgchar(Data);
  ThePixmap := gdk_pixmap_create_from_xpm_d (aWindow, @TheMask, nil, ppdata);
  freemem (ppdata, sizeof (pgchar) * (high(data)-low(data)+1));
end;


 { TFPgtkContainer }

function TFPgtkContainer.TheGtkObject : PGtkContainer;
begin
  result := PgtkContainer(FGtkObject);
end;


function TFPgtkContainer.GetBorder : integer;
begin
  result := gtk.border_width(TheGtkObject^);
end;

procedure TFPgtkContainer.SetBorder (TheValue:integer);
begin
  gtk_container_set_border_width(TheGtkObject,TheValue);
end;

procedure TFPgtkContainer.Add (AWidget:TFPgtkWidget; IsVisible:boolean); Overload;
begin
  gtk_container_add (TheGtkObject, ConvertToGtkWidget(AWidget));
  if IsVisible then
    AWidget.Show;
end;

procedure TFPgtkContainer.Add (AWidget:TFPgtkWidget); Overload;
begin
  gtk_container_add (TheGtkObject, ConvertToGtkWidget(AWidget));
  AWidget.Show;
end;

procedure TFPgtkContainer.Remove (AWidget:TFPgtkWidget);
begin
  gtk_container_remove (TheGtkObject, PGtkwidget(ConvertToGtkObject(AWidget)));
end;

constructor TFPgtkContainer.Create;
begin
  inherited create;
  FChildren := TFPgtkWidgetGroup.Create;
end;


destructor TFPgtkContainer.Destroy;
begin
  if assigned(FChildren) then
    FChildren.Free;
  inherited destroy;
end;


function TFPgtkContainer.GetChildren : TFPgtkWidgetGroup;
begin
  FChildren.GtkList := gtk_container_children (TheGtkObject);
  result := FChildren;
end;

procedure TFPgtkContainer.Focus (Direction:TGtkDirectionType);
begin
  gtk_container_focus (TheGtkObject, Direction);
end;

procedure TFPgtkContainer.FocusChild (Child:TFPgtkWidget);
begin
  gtk_container_set_focus_child (TheGtkObject, PGtkwidget(ConvertToGtkObject(Child)));
end;

procedure TFPgtkContainer.RegisterToplevel;
begin
  gtk_container_register_toplevel (TheGtkObject);
end;

procedure TFPgtkContainer.UnregisterToplevel;
begin
  gtk_container_unregister_toplevel (TheGtkObject);
end;

procedure TFPgtkContainer.ResizeChildren;
begin
  gtk_container_resize_children (TheGtkObject);
end;

function DirectionFunctionSignalfunc (Sender:PGtkobject; Direction:TGtkDirectionType; data:pointer) : TGtkDirectionType; cdecl;
var p : TFPgtkDirectionFunctionSignalFunction;
begin
with PSignalData(data)^ do
  begin
  p := TFPgtkDirectionFunctionSignalFunction (TheSignalProc);
  result := p (TheWidget as TFPgtkObject, Direction, TheData)
  end;
end;

function TFPgtkContainer.DirectionFunctionSignalConnect (signal:string; proc:TFPgtkDirectionFunctionSignalFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect (FGtkObject, pgChar(signal), gtk_signal_func(@DirectionFunctionSignalfunc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

function TFPgtkContainer.DirectionFunctionSignalConnectAfter (signal:string; proc:TFPgtkDirectionFunctionSignalFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect_After (FGtkObject, pgChar(signal), gtk_signal_func(@DirectionFunctionSignalfunc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

function TFPgtkContainer.ConnectAdd (proc:TFPgtkWidgetSignalFunction; data:pointer) : guint;
begin
  result := WidgetSignalConnect (sgAdd, proc, data);
end;

function TFPgtkContainer.ConnectAfterAdd (proc:TFPgtkWidgetSignalFunction; data:pointer) : guint;
begin
  result := WidgetSignalConnectAfter (sgAdd, proc, data);
end;

function TFPgtkContainer.ConnectRemove (proc:TFPgtkWidgetSignalFunction; data:pointer) : guint;
begin
  result := WidgetSignalConnect (sgRemove, proc, data);
end;

function TFPgtkContainer.ConnectAfterRemove (proc:TFPgtkWidgetSignalFunction; data:pointer) : guint;
begin
  result := WidgetSignalConnectAfter (sgRemove, proc, data);
end;

function TFPgtkContainer.ConnectCheckResize (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnect (sgCheckResize, proc, data);
end;

function TFPgtkContainer.ConnectAfterCheckResize (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnectAfter (sgCheckResize, proc, data);
end;

function TFPgtkContainer.ConnectFocus (proc:TFPgtkDirectionFunctionSignalFunction; data:pointer) : guint;
begin
  result := DirectionFunctionSignalConnect (sgFocus, proc, data);
end;

function TFPgtkContainer.ConnectAfterFocus (proc:TFPgtkDirectionFunctionSignalFunction; data:pointer) : guint;
begin
  result := DirectionFunctionSignalConnectAfter (sgFocus, proc, data);
end;

function TFPgtkContainer.ConnectSetFocusChild (proc:TFPgtkWidgetSignalFunction; data:pointer) : guint;
begin
  result := WidgetSignalConnect (sgSetFocusChild, proc, data);
end;

function TFPgtkContainer.ConnectAfterSetFocusChild (proc:TFPgtkWidgetSignalFunction; data:pointer) : guint;
begin
  result := WidgetSignalConnectAfter (sgSetFocusChild, proc, data);
end;

 { TFPgtkBin }

function TFPgtkBin.TheGtkObject : PGtkBin;
begin
  result := PgtkBin(FGtkObject);
end;


function TFPgtkBin.GetChild : TFPgtkWidget;
begin
  result := GetPascalInstance(PGtkObject(TheGtkObject^.Child),tfpgtkwidget) as tfpgtkwidget;
end;

procedure TFPgtkBin.SetChild (TheValue:TFPgtkWidget);
begin
  Add (TheValue);
end;

 { TFPgtkAlignment }

function TFPgtkAlignment.TheGtkObject : PGtkAlignment;
begin
  result := PgtkAlignment(FGtkObject);
end;


procedure TFPgtkAlignment.Configure (anXAlign:gfloat; anYAlign:gfloat; anXScale:gfloat; anYScale:gfloat);
begin
  gtk_alignment_set (TheGtkObject, anXAlign, anYAlign, anXScale, anYScale);
end;

 { TFPgtkFrame }

function TFPgtkFrame.TheGtkObject : PGtkFrame;
begin
  result := PgtkFrame(FGtkObject);
end;

procedure TFPgtkFrame.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_frame_new (nil));
end;


function TFPgtkFrame.GetText : string;
begin
  result := TheGtkObject^.thelabel;
end;

procedure TFPgtkFrame.SetText (TheValue:string);
begin
  gtk_frame_set_label(TheGtkObject,ConvertToPgchar(TheValue));
end;

function TFPgtkFrame.GetAlignment : gfloat;
begin
  result := TheGtkObject^.label_xalign;
end;

procedure TFPgtkFrame.SetAlignment (TheValue:gfloat);
begin
  gtk_frame_set_label_align (ThegtkObject, TheValue, 0.0);
end;

function TFPgtkFrame.GetShadowType : TgtkShadowType;
begin
  result := TheGtkObject^.shadow_type;
end;

procedure TFPgtkFrame.SetShadowType (TheValue:TgtkShadowType);
begin
  gtk_frame_set_shadow_type(TheGtkObject,TheValue);
end;

 { TFPgtkAspectFrame }

function TFPgtkAspectFrame.TheGtkObject : PGtkAspectFrame;
begin
  result := PgtkAspectFrame(FGtkObject);
end;

procedure TFPgtkAspectFrame.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_aspect_frame_new (nil,0,0,1,1));
end;


procedure TFPgtkAspectFrame.Configure (anXAlign:gfloat; anYAlign:gfloat; Ratio:gfloat; ObeyChild:longbool);
begin
  gtk_aspect_frame_set (TheGtkObject, anXAlign, anYAlign, Ratio, gint(ObeyChild));
end;

 { TFPgtkButton }

function TFPgtkButton.TheGtkObject : PGtkButton;
begin
  result := PgtkButton(FGtkObject);
end;

procedure TFPgtkButton.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_button_new);
end;


function TFPgtkButton.LabelClass : TFPgtkLabelClass;
begin
  result := TFPgtkLabel;
end;

procedure TFPgtkButton.CreateLabel (aText:string);
begin
if not assigned (FLabel) then
  begin
  FLabel := LabelClass.Create ('');
  with FLabel do
    begin
    AskNotification (Self);
    FAccelKey := ParseULine (aText);
    end;
  if assigned(AddContainer) then
    AddContainer.Add (FLabel)
  else
    Add (FLabel);
  LabelCreated;
  end;
end;

procedure TFPgtkButton.NotifyDestroy (AnObject:TFPgtkObject);
begin
  inherited;
  if AnObject = FLabel then
    FLabel := nil;
end;

function TFPgtkButton.ConnectClicked (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnect (sgClicked, proc, data);
end;

function TFPgtkButton.ConnectAfterClicked (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnectAfter (sgClicked, proc, data);
end;

function TFPgtkButton.ConnectPressed (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnect (sgPressed, proc, data);
end;

function TFPgtkButton.ConnectAfterPressed (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnectAfter (sgPressed, proc, data);
end;

function TFPgtkButton.ConnectReleased (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnect (sgReleased, proc, data);
end;

function TFPgtkButton.ConnectAfterReleased (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnectAfter (sgReleased, proc, data);
end;

function TFPgtkButton.ConnectEnter (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnect (sgEnter, proc, data);
end;

function TFPgtkButton.ConnectAfterEnter (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnectAfter (sgEnter, proc, data);
end;

function TFPgtkButton.ConnectLeave (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnect (sgLeave, proc, data);
end;

function TFPgtkButton.ConnectAfterLeave (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnectAfter (sgLeave, proc, data);
end;

procedure TFPgtkButton.Clicked;
begin
  gtk_button_Clicked (TheGtkObject);
end;

procedure TFPgtkButton.Pressed;
begin
  gtk_button_Pressed (TheGtkObject);
end;

procedure TFPgtkButton.Released;
begin
  gtk_button_Released (TheGtkObject);
end;

procedure TFPgtkButton.Enter;
begin
  gtk_button_Enter (TheGtkObject);
end;

procedure TFPgtkButton.Leave;
begin
  gtk_button_Leave (TheGtkObject);
end;

constructor TFPgtkButton.Create;
begin
  inherited create;
  FAddContainer := nil;
end;


constructor TFPgtkButton.CreateWithLabel (aText:string);
begin
  create;
  Text := aText;
end;


constructor TFPgtkButton.CreateWithLabel (aText:string; AccelGroup:PGtkAccelGroup);
begin
  create;
  Text := aText;
  if (FAccelKey <> 0) and assigned(AccelGroup) then
    AcceleratorAdd (AccelGroup, sgClicked, FAccelKey, DefaultButtonModifiers, GTK_ACCEL_Visible);
end;


function TFPgtkButton.GetText : string;
begin
  if assigned (FLabel) then
    result := FLabel.Text
  else
    result := '';
end;

procedure TFPgtkButton.SetText (TheValue:string);
begin
  if assigned (FLabel) then
    FLabel.Text := TheValue
  else
    if TheValue <> '' then
      CreateLabel (TheValue);
end;

function TFPgtkButton.GetReliefStyle : TGtkReliefStyle;
begin
  result := gtk_button_get_relief(TheGtkObject);
end;

procedure TFPgtkButton.SetReliefStyle (TheValue:TGtkReliefStyle);
begin
  gtk_button_set_relief(TheGtkObject,TheValue);
end;

procedure TFPgtkButton.LabelCreated;
begin
  FLabel.setalignment (0.5,0.5);
end;

 { TFPgtkToggleButton }

function TFPgtkToggleButton.TheGtkObject : PGtkToggleButton;
begin
  result := PgtkToggleButton(FGtkObject);
end;

procedure TFPgtkToggleButton.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_toggle_button_new);
end;


function TFPgtkToggleButton.ConnectToggled (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnect (sgToggled, proc, data);
end;

function TFPgtkToggleButton.ConnectAfterToggled (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnectAfter (sgToggled, proc, data);
end;

procedure TFPgtkToggleButton.Toggled;
begin
  gtk_toggle_button_toggled (TheGtkObject);
end;

function TFPgtkToggleButton.GetActive : boolean;
begin
  result := gtk_toggle_button_get_active(TheGtkObject);
end;

procedure TFPgtkToggleButton.SetActive (TheValue:boolean);
begin
  gtk_toggle_button_set_active(TheGtkObject,TheValue);
end;

function TFPgtkToggleButton.GetDrawIndicator : boolean;
begin
  result := boolean(gtk.draw_indicator(TheGtkObject^));
end;

procedure TFPgtkToggleButton.SetDrawIndicator (TheValue:boolean);
begin
  gtk.Set_draw_indicator(TheGtkObject^,guint(TheValue))
end;

 { TFPgtkCheckButton }

function TFPgtkCheckButton.TheGtkObject : PGtkCheckButton;
begin
  result := PgtkCheckButton(FGtkObject);
end;

procedure TFPgtkCheckButton.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_check_button_new);
end;


 { TFPgtkRadioButton }

function TFPgtkRadioButton.TheGtkObject : PGtkRadioButton;
begin
  result := PgtkRadioButton(FGtkObject);
end;


constructor TFPgtkRadioButton.Create (AGroup:TFPgtkRadioButtonGroup);
begin
  FGroup := AGroup;
  inherited create;
end;


constructor TFPgtkRadioButton.CreateWithLabel (AGroup:TFPgtkRadioButtonGroup; aText:string);
begin
  FGroup := AGroup;
  inherited CreateWithLabel (aText);
end;


procedure TFPgtkRadioButton.CreateGtkObject;
begin
  if not assigned (FGroup) then
    FGroup := TFPgtkRadioButtonGroup.Create;
  TheGtkWidget := gtk_radio_button_new (FGroup.GtkSList);
  FGroup.GtkSList := gtk_radio_button_group (TheGtkObject);
end;

 { TFPgtkRadioButtonGroup }


function TFPgtkRadioButtonGroup.GetItem (index:integer) : TFPgtkRadioButton;
begin
  result := TFPgtkRadioButton(Inherited items[index]);
end;

procedure TFPgtkRadioButtonGroup.SetItem (index:integer; TheValue:TFPgtkRadioButton);
begin
  inherited items[index] := TheValue;
end;

function TFPgtkRadioButtonGroup.ActiveButtonText : string;
begin
  result := ActiveButton.Text;
end;

function TFPgtkRadioButtonGroup.ActiveButtonIndex : integer;
begin
  Result := pred(count);
  while (Result >= 0) and (not items[Result].Active) do
    dec (Result);
end;

function TFPgtkRadioButtonGroup.ActiveButton : TFPgtkRadioButton;
var r : integer;
begin
  r := ActiveButtonIndex;
  if r >= 0 then
    result := items[r]
  else
    result := nil;
end;

function RadioButtonGroupCreateFromStrings (TheItems:TStrings; ToggledFunction:TFPgtkSignalFunction) : TFPgtkRadioButtonGroup;
var r : integer;
    b : TFPgtkRadioButton;
begin
  result := TFPgtkRadioButtonGroup.Create;
  result.BeginUpdate;
  for r := TheItems.count-1 downto 0 do
    begin
    b := TFPgtkRadioButton.CreateWithLabel (result, TheItems[r]);
    if assigned(toggledfunction) then
      b.connecttoggled (ToggledFunction, IntToPointer(r));
    end;
  b.active := true;
  result.EndUpdate;
end;


 { TFPgtkOptionMenu }

function TFPgtkOptionMenu.TheGtkObject : PGtkOptionMenu;
begin
  result := PgtkOptionMenu(FGtkObject);
end;

procedure TFPgtkOptionMenu.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_option_menu_new);
end;


function TFPgtkOptionMenu.GetMenu : TFPgtkMenu;
begin
  result := GetPascalInstance(PGtkObject(gtk_option_menu_get_menu(TheGtkObject)),tfpgtkmenu) as tfpgtkmenu;
end;

procedure TFPgtkOptionMenu.setmenu (TheValue:TFPgtkMenu);
begin
  gtk_option_menu_set_menu(TheGtkObject, ConvertToGtkWidget(TheValue));
end;

procedure TFPgtkOptionMenu.RemoveMenu;
begin
  gtk_option_menu_remove_menu (TheGtkObject);
end;

procedure TFPgtkOptionMenu.SetHistory (index:integer);
begin
  gtk_option_menu_set_history (TheGtkObject, index);
end;

procedure TFPgtkOptionMenu.Clear;
var w : TFPgtkWidget;
begin
  w := Menu;
  if assigned(w) then
    begin
    w := TFPgtkMenu(w).Active;
    if assigned (w) then
      TFPgtkItem(w).Deselect;
    end;
end;

 { TFPgtkItem }

function TFPgtkItem.TheGtkObject : PGtkItem;
begin
  result := PgtkItem(FGtkObject);
end;


function TFPgtkItem.LabelClass : TFPgtkLabelClass;
begin
  result := TFPgtkLabel;
end;

procedure TFPgtkItem.CreateLabel (aText:string);
begin
  if not assigned (FLabel) then
    begin
    FLabel := LabelClass.Create ('');
    with FLabel do
      begin
      AskNotification (Self);
      FAccelKey := ParseULine (aText);
      end;
    if assigned(AddContainer) then
      AddContainer.Add (FLabel)
    else
      Add (FLabel);
    LabelCreated;
    end;
end;

procedure TFPgtkItem.NotifyDestroy (AnObject:TFPgtkObject);
begin
  inherited;
  if AnObject = FLabel then
    FLabel := nil;
end;

function TFPgtkItem.ConnectSelect (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnect (sgSelect, proc, data);
end;

function TFPgtkItem.ConnectAfterSelect (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnectAfter (sgSelect, proc, data);
end;

function TFPgtkItem.ConnectDeselect (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnect (sgDeselect, proc, data);
end;

function TFPgtkItem.ConnectAfterDeselect (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnectAfter (sgDeselect, proc, data);
end;

function TFPgtkItem.ConnectToggle (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnect (sgToggle, proc, data);
end;

function TFPgtkItem.ConnectAfterToggle (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnectAfter (sgToggle, proc, data);
end;

procedure TFPgtkItem.Select;
begin
  gtk_item_Select (TheGtkObject);
end;

procedure TFPgtkItem.Deselect;
begin
  gtk_item_Deselect (TheGtkObject);
end;

procedure TFPgtkItem.Toggle;
begin
  gtk_item_Toggle (TheGtkObject);
end;

constructor TFPgtkItem.Create;
begin
  inherited;
  FAddContainer := nil;
end;


constructor TFPgtkItem.CreateWithLabel (aText:string);
begin
  inherited create;
  Text := aText;
end;


function TFPgtkItem.GetText : string;
begin
  if assigned (FLabel) then
    result := FLabel.Text
  else
    result := '';
end;

procedure TFPgtkItem.SetText (TheValue:string);
begin
  if assigned (FLabel) then
    FLabel.Text := TheValue
  else
    if TheValue <> '' then
      CreateLabel (TheValue);
end;

procedure TFPgtkItem.LabelCreated;
begin
end;

 { TFPgtkItemGroup }


function TFPgtkItemGroup.GetItem (index:integer) : TFPgtkItem;
begin
  result := TFPgtkItem (inherited items[index]);
end;

procedure TFPgtkItemGroup.SetItem (index:integer; TheValue:TFPgtkItem);
begin
  inherited items[index] := TheValue;
end;

procedure TFPgtkItemGroup.FillFromList (aList:TStrings);
var r : integer;
    i : TFPgtkItem;
begin
  BeginUpdate;
  for r := 0 to aList.count-1 do
    begin
    i := FItemClass.CreateWithLabel (aList[r]);
    add (i);
    i.Show;
    end;
  EndUpdate;
end;

procedure TFPgtkItemGroup.FillFromCommaText (aList:string);
var l : TStrings;
begin
  l := TStringList.Create;
  try
    l.commatext := aList;
    FillFromList (l);
  finally
    l.Free;
  end;
end;

procedure TFPgtkItemGroup.FillFromArray (aList:array of string);
var r : integer;
    l : TStrings;
begin
  l := TStringlist.Create;
  try
    for r := low (aList) to high(aList) do
      l.Add (aList[r]);
    FillFromList (l);
  finally
    l.Free;
  end;
end;

procedure TFPgtkItemGroup.SignalConnect (Signal:string; proc:TFPgtkSignalFunction; data:pointer);
var r : integer;
begin
  if assigned (Proc) then
    for r := 0 to count-1 do
      Items[r].SignalConnect (Signal, proc, data);
end;

constructor TFPgtkItemGroup.create (AnItemClass:TFPgtkItemClass);
begin
  inherited create;
  FItemClass := AnItemClass;
end;


function TFPgtkItemGroup.AddTextItem (aText:string) : TFPgtkItem;
begin
  result := FItemClass.CreateWithLabel (aText);
  Add (result);
  result.Show;
end;

 { TFPgtkMenuItem }

function TFPgtkMenuItem.TheGtkObject : PGtkMenuItem;
begin
  result := PgtkMenuItem(FGtkObject);
end;

procedure TFPgtkMenuItem.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_menu_item_new);
end;


function TFPgtkMenuItem.ConnectActivate (proc:TFPgtksignalFunction; data:pointer) : guint;
begin
  result := signalConnect (sgActivate, proc, data);
end;

function TFPgtkMenuItem.ConnectAfterActivate (proc:TFPgtksignalFunction; data:pointer) : guint;
begin
  result := signalConnectAfter (sgActivate, proc, data);
end;

function TFPgtkMenuItem.ConnectActivateItem (proc:TFPgtksignalFunction; data:pointer) : guint;
begin
  result := signalConnect (sgActivateItem, proc, data);
end;

function TFPgtkMenuItem.ConnectAfterActivateItem (proc:TFPgtksignalFunction; data:pointer) : guint;
begin
  result := signalConnectAfter (sgActivateItem, proc, data);
end;

procedure TFPgtkMenuItem.Activate;
begin
  gtk_menu_item_activate (TheGtkObject);
end;

procedure TFPgtkMenuItem.SetSubMenu (aSubMenu:TFPgtkWidget);
begin
  gtk_menu_item_Set_submenu (TheGtkObject, PGtkwidget(ConvertToGtkObject(aSubMenu)));
end;

procedure TFPgtkMenuItem.RemoveSubMenu;
begin
  gtk_menu_item_remove_submenu (TheGtkObject);
end;

procedure TFPgtkMenuItem.Configure (ShowToggleIndicator:boolean; ShowSubmenuIndicator:boolean);
begin
  gtk_menu_item_configure (TheGtkObject, ord(ShowToggleIndicator), ord(ShowSubmenuIndicator));
end;

procedure TFPgtkMenuItem.RightJustify;
begin
  gtk_menu_item_right_justify (TheGtkObject);
end;

function TFPgtkMenuItem.GetPlacement : TGtkSubmenuPlacement;
begin
  result := TGtkSubmenuPlacement(submenu_placement(TheGtkObject^));
end;

procedure TFPgtkMenuItem.SetPlacement (TheValue:TGtkSubmenuPlacement);
begin
  gtk_menu_item_set_placement(TheGtkObject,TheValue);
end;

function TFPgtkMenuItem.GetToggleIndicator : boolean;
begin
  result := boolean(gtk.show_toggle_indicator(TheGtkObject^));
end;

procedure TFPgtkMenuItem.SetToggleIndicator (TheValue:boolean);
begin
  Configure (TheValue, SubMenuIndicator);
end;

function TFPgtkMenuItem.GetSubMenuIndicator : boolean;
begin
  result := boolean(gtk.show_submenu_indicator(TheGtkObject^));
end;

procedure TFPgtkMenuItem.SetSubMenuIndicator (TheValue:boolean);
begin
  configure (ToggleIndicator, TheValue);
end;

function TFPgtkMenuItem.GetJustifyRight : boolean;
begin
  result := boolean(gtk.right_justify(TheGtkObject^));
end;

procedure TFPgtkMenuItem.SetJustifyRight (TheValue:boolean);
begin
  gtk.Set_right_justify(TheGtkObject^,guint(TheValue))
end;

function TFPgtkMenuItem.GetSubMenu : TFPgtkMenuShell;
begin
  result := GetPascalInstance(PGtkObject(TheGtkObject^.submenu),tfpgtkmenushell) as tfpgtkmenushell;
end;

procedure TFPgtkMenuItem.SetPropSubMenu (TheValue:TFPgtkMenuShell);
begin
  SetSubMenu (TheValue);
end;

function TFPgtkMenuItem.LabelClass : TFPgtkLabelClass;
begin
  result := TFPgtkAccelLabel;
end;

procedure TFPgtkMenuItem.LabelCreated;
begin
  with (TheLabel as TFPgtkAccelLabel) do
    AccelWidget := Self;
end;

 { TFPgtkCheckMenuItem }

function TFPgtkCheckMenuItem.TheGtkObject : PGtkCheckMenuItem;
begin
  result := PgtkCheckMenuItem(FGtkObject);
end;

procedure TFPgtkCheckMenuItem.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_check_menu_item_new);
end;


function TFPgtkCheckMenuItem.ConnectToggled (proc:TFPgtksignalFunction; data:pointer) : guint;
begin
  result := signalConnect (sgToggled, proc, data);
end;

function TFPgtkCheckMenuItem.ConnectAfterToggled (proc:TFPgtksignalFunction; data:pointer) : guint;
begin
  result := signalConnectAfter (sgToggled, proc, data);
end;

procedure TFPgtkCheckMenuItem.Toggled;
begin
  gtk_check_menu_item_toggled (TheGtkObject);
end;

function TFPgtkCheckMenuItem.GetActive : boolean;
begin
  result := boolean(gtk.active(TheGtkObject^));
end;

procedure TFPgtkCheckMenuItem.SetActive (TheValue:boolean);
begin
  gtk_check_menu_item_set_active(TheGtkObject,TheValue);
end;

function TFPgtkCheckMenuItem.GetShowToggle : boolean;
begin
  result := boolean(gtk.always_show_toggle(TheGtkObject^));
end;

procedure TFPgtkCheckMenuItem.SetShowToggle (TheValue:boolean);
begin
  gtk_check_menu_item_set_show_toggle(TheGtkObject,TheValue);
end;

 { TFPgtkRadioMenuItem }

function TFPgtkRadioMenuItem.TheGtkObject : PGtkRadioMenuItem;
begin
  result := PgtkRadioMenuItem(FGtkObject);
end;


procedure TFPgtkRadioMenuItem.CreateGtkObject;
begin
  if not assigned(FGroup) then
    FGroup := TFPgtkRadioMenuGroup.Create;
  TheGtkWidget := gtk_radio_menu_item_new (FGroup.GtkSList);
  FGroup.GtkSList := gtk_radio_menu_item_group (TheGtkObject);
end;

constructor TFPgtkRadioMenuItem.Create (AGroup:TFPgtkRadioMenuGroup);
begin
  FGroup := AGroup;
  inherited create;
end;


constructor TFPgtkRadioMenuItem.CreateWithLabel (Agroup:TFPgtkRadioMenuGroup; aText:string);
begin
  FGroup := Agroup;
  inherited CreateWithLabel (aText);
end;


 { TFPgtkRadioMenuGroup }


function TFPgtkRadioMenuGroup.GetItem (index:integer) : TFPgtkRadioMenuItem;
begin
  result := TFPgtkRadioMenuItem(Inherited items[index]);
end;

procedure TFPgtkRadioMenuGroup.SetItem (index:integer; TheValue:TFPgtkRadioMenuItem);
begin
  inherited items[index] := TheValue;
end;

function TFPgtkRadioMenuGroup.ActiveMenuText : string;
begin
  result := ActiveMenu.Text;
end;

function TFPgtkRadioMenuGroup.ActiveMenuIndex : integer;
begin
  Result := pred(count);
  while (Result >= 0) and (not items[Result].Active) do
    dec (Result);
end;

function TFPgtkRadioMenuGroup.ActiveMenu : TFPgtkRadioMenuItem;
var r : integer;
begin
  r := ActiveMenuIndex;
  if r >= 0 then
    result := items[r]
  else
    result := nil;
end;

constructor TFPgtkRadioMenuGroup.create;
begin
  inherited create (TFPgtkRadioMenuItem);
end;


 { TFPgtkTearOffMenuItem }

function TFPgtkTearOffMenuItem.TheGtkObject : PGtkTearOffMenuItem;
begin
  result := PgtkTearOffMenuItem(FGtkObject);
end;

procedure TFPgtkTearOffMenuItem.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_tearoff_menu_item_new);
end;


 { TFPgtkListItem }

function TFPgtkListItem.TheGtkObject : PGtkListItem;
begin
  result := PgtkListItem(FGtkObject);
end;

procedure TFPgtkListItem.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_list_item_new);
end;


procedure ScrollSignalproc (Sender:PGtkobject; ScrollType:TgtkScrollType; position:gfloat; data:pointer); cdecl;
var p : TFPgtkScrollSignalFunction;
begin
with PSignalData(data)^ do
  begin
  p := TFPgtkScrollSignalFunction (TheSignalProc);
  p (TheWidget as TFPgtkObject, ScrollType, position, TheData)
  end;
end;

function TFPgtkListItem.ScrollSignalConnect (signal:string; proc:TFPgtkScrollSignalFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect (FGtkObject, pgChar(signal), gtk_signal_func(@ScrollSignalproc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

function TFPgtkListItem.ScrollSignalConnectAfter (signal:string; proc:TFPgtkScrollSignalFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect_After (FGtkObject, pgChar(signal), gtk_signal_func(@ScrollSignalproc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

procedure ScrollBooleanSignalproc (Sender:PGtkobject; ScrolType:TgtkScrollType; Position:gfloat; AutoStartSelection:boolean; data:pointer); cdecl;
var p : TFPgtkScrollBooleanSignalFunction;
begin
with PSignalData(data)^ do
  begin
  p := TFPgtkScrollBooleanSignalFunction (TheSignalProc);
  p (TheWidget as TFPgtkObject, ScrolType, Position, AutoStartSelection, TheData)
  end;
end;

function TFPgtkListItem.ScrollBooleanSignalConnect (signal:string; proc:TFPgtkScrollBooleanSignalFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect (FGtkObject, pgChar(signal), gtk_signal_func(@ScrollBooleanSignalproc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

function TFPgtkListItem.ScrollBooleanSignalConnectAfter (signal:string; proc:TFPgtkScrollBooleanSignalFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect_After (FGtkObject, pgChar(signal), gtk_signal_func(@ScrollBooleanSignalproc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

function TFPgtkListItem.ConnectToggleFocusRow (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnect (sgToggleFocusRow, proc, data);
end;

function TFPgtkListItem.ConnectAfterToggleFocusRow (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnectAfter (sgToggleFocusRow, proc, data);
end;

function TFPgtkListItem.ConnectSelectAll (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnect (sgSelectAll, proc, data);
end;

function TFPgtkListItem.ConnectAfterSelectAll (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnectAfter (sgSelectAll, proc, data);
end;

function TFPgtkListItem.ConnectUnselectAll (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnect (sgUnselectAll, proc, data);
end;

function TFPgtkListItem.ConnectAfterUnselectAll (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnectAfter (sgUnselectAll, proc, data);
end;

function TFPgtkListItem.ConnectUndoSelection (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnect (sgUndoSelection, proc, data);
end;

function TFPgtkListItem.ConnectAfterUndoSelection (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnectAfter (sgUndoSelection, proc, data);
end;

function TFPgtkListItem.ConnectStartSelection (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnect (sgStartSelection, proc, data);
end;

function TFPgtkListItem.ConnectAfterStartSelection (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnectAfter (sgStartSelection, proc, data);
end;

function TFPgtkListItem.ConnectEndSelection (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnect (sgEndSelection, proc, data);
end;

function TFPgtkListItem.ConnectAfterEndSelection (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnectAfter (sgEndSelection, proc, data);
end;

function TFPgtkListItem.ConnectToggleAddMode (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnect (sgToggleAddMode, proc, data);
end;

function TFPgtkListItem.ConnectAfterToggleAddMode (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnectAfter (sgToggleAddMode, proc, data);
end;

function TFPgtkListItem.ConnectExtendSelection (proc:TFPgtkScrollBooleanSignalFunction; data:pointer) : guint;
begin
  result := ScrollBooleanSignalConnect (sgExtendSelection, proc, data);
end;

function TFPgtkListItem.ConnectAfterExtendSelection (proc:TFPgtkScrollBooleanSignalFunction; data:pointer) : guint;
begin
  result := ScrollBooleanSignalConnectAfter (sgExtendSelection, proc, data);
end;

function TFPgtkListItem.ConnectScrollVertical (proc:TFPgtkScrollSignalFunction; data:pointer) : guint;
begin
  result := ScrollSignalConnect (sgScrollVertical, proc, data);
end;

function TFPgtkListItem.ConnectAfterScrollVertical (proc:TFPgtkScrollSignalFunction; data:pointer) : guint;
begin
  result := ScrollSignalConnectAfter (sgScrollVertical, proc, data);
end;

function TFPgtkListItem.ConnectScrollHorizontal (proc:TFPgtkScrollSignalFunction; data:pointer) : guint;
begin
  result := ScrollSignalConnect (sgScrollHorizontal, proc, data);
end;

function TFPgtkListItem.ConnectAfterScrollHorizontal (proc:TFPgtkScrollSignalFunction; data:pointer) : guint;
begin
  result := ScrollSignalConnectAfter (sgScrollHorizontal, proc, data);
end;

procedure TFPgtkListItem.Select;
begin
  gtk_list_item_select (TheGtkObject);
end;

procedure TFPgtkListItem.Deselect;
begin
  gtk_list_item_deselect (TheGtkObject);
end;

 { TFPgtkListItemGroup }


constructor TFPgtkListItemGroup.create;
begin
  inherited create (TFPgtkListItem);
  ManageLists := false;
end;


 { TFPgtkTreeItem }

function TFPgtkTreeItem.TheGtkObject : PGtkTreeItem;
begin
  result := PgtkTreeItem(FGtkObject);
end;

procedure TFPgtkTreeItem.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_tree_item_new);
end;


function TFPgtkTreeItem.GetSubTree : TFPgtkWidget;
begin
  result := GetPascalInstance(PGtkObject(gtk_tree_item_subtree(TheGtkObject)),tfpgtkwidget) as tfpgtkwidget;
end;

procedure TFPgtkTreeItem.SetSubTree (TheValue:TFPgtkWidget);
begin
  if assigned(TheValue) then
    gtk_tree_item_set_subtree (TheGtkObject, ConvertToGtkWidget(TheValue))
  else
    gtk_tree_item_remove_subtree (TheGtkObject);
end;

function TFPgtkTreeItem.GetPixPlus : TFPgtkWidget;
begin
  result := GetPascalInstance(PGtkObject(TheGtkObject^.plus_pix_widget),tfpgtkwidget) as tfpgtkwidget;
end;

function TFPgtkTreeItem.GetPixMinus : TFPgtkWidget;
begin
  result := GetPascalInstance(PGtkObject(TheGtkObject^.minus_pix_widget),tfpgtkwidget) as tfpgtkwidget;
end;

function TFPgtkTreeItem.GetExpanded : boolean;
begin
  result := boolean(gtk.expanded(TheGtkObject^));
end;

procedure TFPgtkTreeItem.SetExpanded (TheValue:boolean);
begin
  if TheValue then
    Expand
  else
    collapse;
end;

procedure TFPgtkTreeItem.Select;
begin
  gtk_tree_item_select (TheGtkObject);
end;

procedure TFPgtkTreeItem.Deselect;
begin
  gtk_tree_item_deselect (TheGtkObject);
end;

procedure TFPgtkTreeItem.Expand;
begin
  gtk_tree_item_expand (TheGtkObject);
end;

procedure TFPgtkTreeItem.Collapse;
begin
  gtk_tree_item_collapse (TheGtkObject);
end;

function TFPgtkTreeItem.ConnectCollapse (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnect (sgCollapse, proc, data);
end;

function TFPgtkTreeItem.ConnectAfterCollapse (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnectAfter (sgCollapse, proc, data);
end;

function TFPgtkTreeItem.ConnectExpand (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnect (sgExpand, proc, data);
end;

function TFPgtkTreeItem.ConnectAfterExpand (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnectAfter (sgExpand, proc, data);
end;

 { TFPgtkWindow }

function TFPgtkWindow.TheGtkObject : PGtkWindow;
begin
  result := PgtkWindow(FGtkObject);
end;

procedure TFPgtkWindow.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_window_new (TheWindowType));
end;


constructor TFPgtkWindow.Create (AType:TGtkWindowType);
begin
  TheWindowType := AType;
  inherited Create;
  FAccelGroups := TList.Create;
  FMainLevel := NoMainLevel;
end;


destructor TFPgtkWindow.Destroy;
begin
  FAccelGroups.Free;
  inherited;
end;


function TFPgtkWindow.GetWindowType : TGtkWindowType;
begin
  result := TheGtkObject^.thetype;
end;

procedure TFPgtkWindow.SetWindowType (TheValue:TGtkWindowType);
begin
  TheGtkObject^.thetype := TheValue;
end;

function TFPgtkWindow.GetTitle : string;
begin
  result := TheGtkObject^.title;
end;

procedure TFPgtkWindow.SetTitle (TheValue:string);
begin
  gtk_window_set_title(TheGtkObject,ConvertToPgchar(TheValue));
end;

function TFPgtkWindow.GetModal : boolean;
begin
  result := boolean(gtk.modal(TheGtkObject^));
end;

procedure TFPgtkWindow.SetModal (TheValue:boolean);
begin
  gtk_window_set_modal(TheGtkObject,TheValue);
end;

procedure TFPgtkWindow.DoDialogResult (Action:integer; Sender:TFPgtkObject);
begin
  if assigned (OnDialogResult) then
    OnDialogResult (self, FDialogResult, Action, Sender);
end;

procedure TFPgtkWindow.DoDialogInit (InitData:pointer);
begin
  if assigned (OnDialogInit) then
    OnDialogInit (self, InitData);
  FDialogResult := InitData;
end;

procedure TFPgtkWindow.Close;
begin
  if (FDestroying = dsAlive) then
    gtk_widget_destroy (TheGtkWidget);
end;

procedure TFPgtkWindow.CloseWindow (Sender:TFPgtkObject; data:pointer);
begin
  Close;
end;

procedure TFPgtkWindow.CloseWithResult (Sender:TFPgtkObject; data:pointer);
begin
  ModalAction := pointertoint(data);
end;

procedure TFPgtkWindow.SetModalAction (TheValue:integer);
begin
  FModalAction := TheValue;
  if TheValue <> 0 then
    begin
    DoDialogResult (FModalAction, self);
    close;
    end;
end;

procedure TFPgtkWindow.ExecuteEnds (Sender:TFPgtkObject; data:pointer);
begin
  if gtk_main_level = FMainLevel then
    gtk_main_quit;
end;

function TFPgtkWindow.Execute (anOnDialogInit:DialogInitCallBack; anInitData:pointer; anOnDialogResult:DialogResultCallBack) : integer;
begin
  FModalAction := drNone;
  if assigned (anOnDialogInit) then
    OnDialogInit := anOnDialogInit;
  DoDialogInit (anInitData);
  if assigned (anOnDialogResult) then
    OnDialogResult := anOnDialogResult;
  ConnectDestroy (@ExecuteEnds, nil);
  Modal := True;
  Show;
  FMainLevel := gtk_main_level + 1;
  try
    gtk_main;
    result := FModalAction;
  finally
    FMainLevel := NoMainLevel;
  end;
end;

function TFPgtkWindow.ConnectSetFocus (proc:TFPgtkWidgetSignalFunction; data:pointer) : guint;
begin
  result := WidgetSignalConnect (sgSetFocus, proc, data);
end;

function TFPgtkWindow.ConnectAfterSetFocus (proc:TFPgtkWidgetSignalFunction; data:pointer) : guint;
begin
  result := WidgetSignalConnectAfter (sgSetFocus, proc, data);
end;

procedure TFPgtkWindow.SetTransientFor (aParent:TFPgtkWindow);
begin
  gtk_window_set_transient_for (TheGtkObject, PGtkwindow(ConvertToGtkObject(aParent)));
end;

procedure TFPgtkWindow.DefaultWidget (Widget:TFPgtkWidget);
begin
  gtk_window_set_default (TheGtkObject, PGtkwidget(ConvertToGtkObject(Widget)));
end;

procedure TFPgtkWindow.FocusedWidget (NewFocus:TFPgtkWidget);
begin
  gtk_window_set_focus (TheGtkObject, PGtkwidget(ConvertToGtkObject(NewFocus)));
end;

function TFPgtkWindow.GetUserSizable : boolean;
begin
  result := (allow_grow(TheGtkObject^)=1) and (auto_shrink(TheGtkObject^)=0);
end;

procedure TFPgtkWindow.SetUserSizable (TheValue:boolean);
begin
  if TheValue then
    gtk_window_set_policy (TheGtkObject, gint(FALSE), gint(TRUE), gint(FALSE))
  else
    gtk_window_set_policy (TheGtkObject, gint(FALSE), gint(FALSE), gint(TRUE));
end;

procedure TFPgtkWindow.ActivateFocus;
begin
  gtk_window_activate_focus (TheGtkObject);
end;

procedure TFPgtkWindow.ActivateDefault;
begin
  gtk_window_activate_default (TheGtkObject);
end;

procedure TFPgtkWindow.SetDefaultSize (Width:gint; Height:gint);
begin
  gtk_window_set_default_size (TheGtkObject, Width, Height);
end;

function TFPgtkWindow.GetPosition : TGtkWindowPosition;
begin
  result := TGtkWindowPosition (gtk.position (TheGtkObject^));
end;

procedure TFPgtkWindow.SetPosition (TheValue:TGtkWindowPosition);
begin
  gtk_window_set_position(TheGtkObject,TheValue);
end;

function TFPgtkWindow.GetAccelGroups (ID:integer) : PGtkAccelGroup;
begin
  result := FAccelGroups[ID];
  if result = nil then
    result := FAccelGroups[-1];
end;

function TFPgtkWindow.AccelGroupNew : integer;
var ag : Pgtkaccelgroup;
begin
  result := FAccelGroups.Count;
  ag := gtk_accel_group_new;
  FAccelGroups.Add (ag);
  gtk_window_add_accel_group (TheGtkObject, ag);
end;

procedure TFPgtkWindow.AccelGroupDelete (ID:integer);
begin
  gtk_accel_group_detach (FAccelGroups[ID], FGtkObject);
  FAccelGroups[ID] := nil;
end;

procedure TFPgtkWindow.AcceleratorAdd (AG:integer; aWidget:TFPgtkWidget; aSignal:string; Key:guint; Mods:TGdkModifierType; acFlags:TGtkAccelFlags); Overload;
begin
  gtk_widget_add_accelerator (ConvertToGtkWidget(aWidget), pgchar(aSignal),
        AccelGroups[AG], Key, Mods, acFlags);
end;

procedure AcceleratorAdd (AG:PGtkAccelGroup; aWidget:TFPgtkWidget; aSignal:string; Key:guint; Mods:TGdkModifierType; Flags:TGtkAccelFlags);
begin
  gtk_widget_add_accelerator (ConvertToGtkWidget(aWidget), pgchar(aSignal),
        AG, Key, Mods, Flags);
end;


procedure TFPgtkWindow.AcceleratorRemove (AG:integer; aWidget:TFPgtkWidget; Key:guint; Mods:TGdkModifierType); Overload;
begin
  gtk_widget_remove_accelerator (ConvertToGtkWidget(aWidget), AccelGroups[AG], Key, Mods);
end;

procedure AcceleratorRemove (AG:PGtkAccelGroup; aWidget:TFPgtkWidget; Key:guint; Mods:TGdkModifierType); Overload;
begin
  gtk_widget_remove_accelerator (ConvertToGtkWidget(aWidget), AG, Key, Mods);
end;


procedure TFPgtkWindow.AccelGroupLock (AG:integer);
begin
  gtk_accel_group_lock (AccelGroups[AG]);
end;

procedure AccelGroupLock (AG:PGtkAccelGroup);
begin
  gtk_accel_group_lock (AG);
end;


procedure TFPgtkWindow.AccelGroupUnlock (AG:integer);
begin
  gtk_accel_group_unlock (AccelGroups[AG]);
end;

procedure AccelGroupUnlock (AG:PGtkAccelGroup);
begin
  gtk_accel_group_unlock (AG);
end;


function AccelKeyName (Key:guint; Mods:TGdkModifierType) : string;
begin
  result := string (gtk_accelerator_name(Key, Mods));
end;


procedure AccelKeyParse (AccelName:string; var Key:guint; var Mods:TGdkModifierType);
var k : guint;
    m : TGdkModifierType;
begin
  gtk_accelerator_parse (pgchar(AccelName), @k, @m);
  Key := k;
  Mods := m;
end;


procedure TFPgtkWindow.AccelGroupActivate (AG:integer; Key:guint; Mods:TGdkModifierType);
begin
  gtk_accel_group_activate (AccelGroups[AG], Key, Mods);
end;

procedure AccelGroupActivate (AG:PGtkAccelGroup; Key:guint; Mods:TGdkModifierType);
begin
  gtk_accel_group_activate (AG, Key, Mods);
end;


 { TFPgtkColorSelectionDialog }

function TFPgtkColorSelectionDialog.TheGtkObject : PGtkColorSelectionDialog;
begin
  result := PgtkColorSelectionDialog(FGtkObject);
end;

procedure TFPgtkColorSelectionDialog.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_color_selection_dialog_new (''));
end;


function TFPgtkColorSelectionDialog.GetColorSel : TFPgtkColorSelection;
begin
  result := GetPascalInstance(PGtkObject(TheGtkObject^.Colorsel),tfpgtkcolorselection) as tfpgtkcolorselection;
end;

function TFPgtkColorSelectionDialog.GetButtonOK : TFPgtkButton;
begin
  result := GetPascalInstance(PGtkObject(TheGtkObject^.ok_button),tfpgtkbutton) as tfpgtkbutton;
end;

function TFPgtkColorSelectionDialog.GetButtonCancel : TFPgtkButton;
begin
  result := GetPascalInstance(PGtkObject(TheGtkObject^.cancel_button),tfpgtkbutton) as tfpgtkbutton;
end;

function TFPgtkColorSelectionDialog.GetButtonHelp : TFPgtkButton;
begin
  result := GetPascalInstance(PGtkObject(TheGtkObject^.help_button),tfpgtkbutton) as tfpgtkbutton;
end;

 { TFPgtkDialog }

function TFPgtkDialog.TheGtkObject : PGtkDialog;
begin
  result := PgtkDialog(FGtkObject);
end;

procedure TFPgtkDialog.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_dialog_new);
end;


function TFPgtkDialog.GetActionArea : TFPgtkHBox;
begin
  result := GetPascalInstance(PGtkObject(TheGtkObject^.action_area),tfpgtkhbox) as tfpgtkhbox;
end;

function TFPgtkDialog.GetVBox : TFPgtkVBox;
begin
  result := GetPascalInstance(PGtkObject(TheGtkObject^.vbox),tfpgtkvbox) as tfpgtkvbox;
end;

constructor TFPgtkDialog.create;
begin
  inherited create (gtk_window_dialog);
end;


 { TFPgtkInputDialog }

function TFPgtkInputDialog.TheGtkObject : PGtkInputDialog;
begin
  result := PgtkInputDialog(FGtkObject);
end;

procedure TFPgtkInputDialog.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_input_dialog_new);
end;


function TFPgtkInputDialog.GetButtonClose : TFPgtkButton;
begin
  result := GetPascalInstance(PGtkObject(TheGtkObject^.close_button),tfpgtkbutton) as tfpgtkbutton;
end;

function TFPgtkInputDialog.GetButtonSave : TFPgtkButton;
begin
  result := GetPascalInstance(PGtkObject(TheGtkObject^.save_button),tfpgtkbutton) as tfpgtkbutton;
end;

procedure DeviceSignalproc (Sender:PGtkinputdialog; DeviceID:integer; Data:pointer); cdecl;
var p : TFPgtkDeviceSignalFunction;
begin
with PSignalData(data)^ do
  begin
  p := TFPgtkDeviceSignalFunction (TheSignalProc);
  p (TheWidget as TFPgtkInputDialog, DeviceID, TheData)
  end;
end;

function TFPgtkInputDialog.DeviceSignalConnect (signal:string; proc:TFPgtkDeviceSignalFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect (FGtkObject, pgChar(signal), gtk_signal_func(@DeviceSignalproc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

function TFPgtkInputDialog.DeviceSignalConnectAfter (signal:string; proc:TFPgtkDeviceSignalFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect_After (FGtkObject, pgChar(signal), gtk_signal_func(@DeviceSignalproc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

function TFPgtkInputDialog.ConnectEnableDevice (proc:TFPgtkDeviceSignalFunction; data:pointer) : guint;
begin
  result := DeviceSignalConnect (sgEnableDevice, proc, data);
end;

function TFPgtkInputDialog.ConnectAfterEnableDevice (proc:TFPgtkDeviceSignalFunction; data:pointer) : guint;
begin
  result := DeviceSignalConnectAfter (sgEnableDevice, proc, data);
end;

function TFPgtkInputDialog.ConnectDisableDevice (proc:TFPgtkDeviceSignalFunction; data:pointer) : guint;
begin
  result := DeviceSignalConnect (sgDisableDevice, proc, data);
end;

function TFPgtkInputDialog.ConnectAfterDisableDevice (proc:TFPgtkDeviceSignalFunction; data:pointer) : guint;
begin
  result := DeviceSignalConnectAfter (sgDisableDevice, proc, data);
end;

 { TFPgtkFileSelection }

function TFPgtkFileSelection.TheGtkObject : PGtkFileSelection;
begin
  result := PgtkFileSelection(FGtkObject);
end;

procedure TFPgtkFileSelection.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_file_selection_new ('Select a file'));
end;


function TFPgtkFileSelection.GetFilename : string;
begin
  result := gtk_file_selection_get_filename(TheGtkObject);
end;

procedure TFPgtkFileSelection.SetFilename (TheValue:string);
begin
  gtk_file_selection_set_filename(TheGtkObject,Pgchar(TheValue));
end;

procedure TFPgtkFileSelection.Complete (Pattern:string);
begin
  gtk_file_selection_complete (TheGtkObject, ConvertToPgchar(Pattern));
end;

procedure TFPgtkFileSelection.ShowFileOpButtons;
begin
  gtk_file_selection_show_fileop_buttons (TheGtkObject);
end;

procedure TFPgtkFileSelection.HideFileOpButtons;
begin
  gtk_file_selection_hide_fileop_buttons (TheGtkObject);
end;

function TFPgtkFileSelection.GetDirList : TFPgtkCList;
begin
  result := GetPascalInstance(PGtkObject(TheGtkObject^.dir_list),tfpgtkclist) as tfpgtkclist;
end;

function TFPgtkFileSelection.GetFileList : TFPgtkCList;
begin
  result := GetPascalInstance(PGtkObject(TheGtkObject^.file_list),tfpgtkclist) as tfpgtkclist;
end;

function TFPgtkFileSelection.GetOkButton : TFPgtkButton;
begin
  result := GetPascalInstance(PGtkObject(TheGtkObject^.ok_button),tfpgtkbutton) as tfpgtkbutton;
end;

function TFPgtkFileSelection.GetCancelButton : TFPgtkButton;
begin
  result := GetPascalInstance(PGtkObject(TheGtkObject^.cancel_button),tfpgtkbutton) as tfpgtkbutton;
end;

function TFPgtkFileSelection.GetHistoryPulldown : TFPgtkOptionMenu;
begin
  result := GetPascalInstance(PGtkObject(TheGtkObject^.history_pulldown),tfpgtkoptionmenu) as tfpgtkoptionmenu;
end;

function TFPgtkFileSelection.GetFileOpDialog : TFPgtkDialog;
begin
  result := GetPascalInstance(PGtkObject(TheGtkObject^.fileop_dialog),tfpgtkdialog) as tfpgtkdialog;
end;

function TFPgtkFileSelection.GetFileOpCreateDir : TFPgtkButton;
begin
  result := GetPascalInstance(PGtkObject(TheGtkObject^.fileop_c_dir),tfpgtkbutton) as tfpgtkbutton;
end;

function TFPgtkFileSelection.GetFileOpDelFile : TFPgtkButton;
begin
  result := GetPascalInstance(PGtkObject(TheGtkObject^.fileop_del_file),tfpgtkbutton) as tfpgtkbutton;
end;

function TFPgtkFileSelection.GetFileOpRenFile : TFPgtkButton;
begin
  result := GetPascalInstance(PGtkObject(TheGtkObject^.fileop_ren_file),tfpgtkbutton) as tfpgtkbutton;
end;

 { TFPgtkFontSelectionDialog }

function TFPgtkFontSelectionDialog.TheGtkObject : PGtkFontSelectionDialog;
begin
  result := PgtkFontSelectionDialog(FGtkObject);
end;

procedure TFPgtkFontSelectionDialog.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_font_selection_dialog_new (''));
end;


function TFPgtkFontSelectionDialog.GetFontSel : TFPgtkFontSelection;
begin
  result := GetPascalInstance(PGtkObject(TheGtkObject^.fontsel),tfpgtkfontselection) as tfpgtkfontselection;
end;

function TFPgtkFontSelectionDialog.GetButtonOk : TFPgtkButton;
begin
  result := GetPascalInstance(PGtkObject(TheGtkObject^.ok_button),tfpgtkbutton) as tfpgtkbutton;
end;

function TFPgtkFontSelectionDialog.GetButtonApply : TFPgtkButton;
begin
  result := GetPascalInstance(PGtkObject(TheGtkObject^.apply_button),tfpgtkbutton) as tfpgtkbutton;
end;

function TFPgtkFontSelectionDialog.GetButtonCancel : TFPgtkButton;
begin
  result := GetPascalInstance(PGtkObject(TheGtkObject^.cancel_button),tfpgtkbutton) as tfpgtkbutton;
end;

 { TFPgtkEventBox }

function TFPgtkEventBox.TheGtkObject : PGtkEventBox;
begin
  result := PgtkEventBox(FGtkObject);
end;

procedure TFPgtkEventBox.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_event_box_new);
end;


 { TFPgtkHandleBox }

function TFPgtkHandleBox.TheGtkObject : PGtkHandleBox;
begin
  result := PgtkHandleBox(FGtkObject);
end;

procedure TFPgtkHandleBox.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_handle_box_new);
end;


function TFPgtkHandleBox.GetShadowType : TGtkShadowtype;
begin
  result := TheGtkObject^.shadow_type;
end;

procedure TFPgtkHandleBox.SetShadowType (TheValue:TGtkShadowtype);
begin
  gtk_handle_box_set_shadow_type(TheGtkObject,TheValue);
end;

function TFPgtkHandleBox.GetHandlePosition : TGtkPositionType;
begin
  result := TGtkPositionType (gtk.handle_position(TheGtkObject^));
end;

procedure TFPgtkHandleBox.SetHandlePosition (TheValue:TGtkPositionType);
begin
  gtk_handle_box_set_handle_position(TheGtkObject,TheValue);
end;

function TFPgtkHandleBox.GetSnapEdge : TGtkPositionType;
begin
  result := TGtkPositionType (gtk.snap_edge(TheGtkObject^));
end;

procedure TFPgtkHandleBox.SetSnapEdge (TheValue:TGtkPositionType);
begin
  gtk_handle_box_set_snap_edge(TheGtkObject,TheValue);
end;

function TFPgtkHandleBox.GetChildDetached : boolean;
begin
  result := boolean(gtk.child_detached(TheGtkObject^));
end;

function TFPgtkHandleBox.ConnectChildAttached (proc:TFPgtkWidgetSignalFunction; data:pointer) : guint;
begin
  result := WidgetSignalConnect (sgChildAttached, proc, data);
end;

function TFPgtkHandleBox.ConnectAfterChildAttached (proc:TFPgtkWidgetSignalFunction; data:pointer) : guint;
begin
  result := WidgetSignalConnectAfter (sgChildAttached, proc, data);
end;

function TFPgtkHandleBox.ConnectChildDetached (proc:TFPgtkWidgetSignalFunction; data:pointer) : guint;
begin
  result := WidgetSignalConnect (sgChildDetached, proc, data);
end;

function TFPgtkHandleBox.ConnectAfterChildDetached (proc:TFPgtkWidgetSignalFunction; data:pointer) : guint;
begin
  result := WidgetSignalConnectAfter (sgChildDetached, proc, data);
end;

 { TFPgtkScrolledWindow }

function TFPgtkScrolledWindow.TheGtkObject : PGtkScrolledWindow;
begin
  result := PgtkScrolledWindow(FGtkObject);
end;


procedure TFPgtkScrolledWindow.CreateGtkObject;
var h, v : PgtkAdjustment;
begin
  if assigned (FHScroll) then
    h := PGtkAdjustment(ConvertToGtkObject(FHScroll))
  else
    h := null;
  if assigned (FVScroll) then
    v := PGtkAdjustment(ConvertToGtkObject(FVScroll))
  else
    v := null;
  FGtkObject := PGtkObject (gtk_scrolled_window_new (h, v));
end;

constructor TFPgtkScrolledWindow.Create (hadj:TFPgtkAdjustment; vadj:TFPgtkAdjustment);
begin
  FVScroll := vadj;
  FHScroll := hadj;
  inherited create;
  setusize (200,170);
end;


function TFPgtkScrolledWindow.GetHPolicy : TGtkPolicyType;
begin
  result := gtk.hscrollbar_policy(TheGtkObject^);
end;

procedure TFPgtkScrolledWindow.SetHPolicy (TheValue:TGtkPolicyType);
begin
  gtk_scrolled_window_set_policy (TheGtkObject, TheValue, VPolicy);
end;

function TFPgtkScrolledWindow.GetVPolicy : TGtkPolicyType;
begin
  result := gtk.vscrollbar_policy(TheGtkObject^);
end;

procedure TFPgtkScrolledWindow.SetVPolicy (TheValue:TGtkPolicyType);
begin
  gtk_scrolled_window_set_policy (TheGtkObject, HPolicy, TheValue);
end;

procedure TFPgtkScrolledWindow.SetPolicy (aHScrollBar:TGtkPolicyType; aVScrollbar:TGtkPolicyType); Overload;
begin
  gtk_scrolled_window_set_policy (TheGtkObject, aHScrollBar, aVScrollbar);
end;

procedure TFPgtkScrolledWindow.SetPolicy (aPolicy:TGtkPolicyType); Overload;
begin
  SetPolicy (aPolicy, aPolicy);
end;

function TFPgtkScrolledWindow.GetHAdjustment : TFPgtkAdjustment;
begin
  result := GetPascalInstance(PGtkObject(gtk_scrolled_window_get_hadjustment(TheGtkObject)),tfpgtkadjustment) as tfpgtkadjustment;
end;

procedure TFPgtkScrolledWindow.SetHAdjustment (TheValue:TFPgtkAdjustment);
begin
  gtk_scrolled_window_set_hadjustment(TheGtkObject,PGtkadjustment(ConvertToGtkObject(TheValue)));
end;

function TFPgtkScrolledWindow.GetVAdjustment : TFPgtkAdjustment;
begin
  result := GetPascalInstance(PGtkObject(gtk_scrolled_window_get_vadjustment(TheGtkObject)),tfpgtkadjustment) as tfpgtkadjustment;
end;

procedure TFPgtkScrolledWindow.SetVAdjustment (TheValue:TFPgtkAdjustment);
begin
  gtk_scrolled_window_set_vadjustment(TheGtkObject,PGtkadjustment(ConvertToGtkObject(TheValue)));
end;

procedure TFPgtkScrolledWindow.AddWithViewport (aChild:TFPgtkWidget);
begin
  gtk_scrolled_window_add_with_viewport (TheGtkObject, ConvertToGtkWidget(aChild));
  TFPgtkViewport.createFromObject (PGtkObject(PGtkBin(TheGtkObject)^.child));
  aChild.Show;
end;

function TFPgtkScrolledWindow.GetPlacement : TGtkCornerType;
begin
  result := gtk.window_placement(TheGtkObject^);
end;

procedure TFPgtkScrolledWindow.SetPlacement (TheValue:TGtkCornerType);
begin
  gtk_scrolled_window_set_placement(TheGtkObject,TheValue);
end;

function TFPgtkScrolledWindow.GetHScrollbar : TFPgtkScrollbar;
var w : TFPgtkObject;
    gtkwidg : PGtkObject;
begin
  gtkwidg := PGtkObject(TheGtkObject^.hscrollbar);
  w := GetPascalInstance (gtkwidg);
  if assigned (w) then
    result := (w as TFPgtkScrollbar)
  else
    result := TFPgtkHScrollbar.CreateFromObject (gtkwidg);
end;

function TFPgtkScrolledWindow.GetVScrollbar : TFPgtkScrollbar;
var w : TFPgtkObject;
    gtkwidg : PGtkObject;
begin
  gtkwidg := PGtkObject(TheGtkObject^.vscrollbar);
  w := GetPascalInstance (gtkwidg);
  if assigned (w) then
    result := (w as TFPgtkScrollbar)
  else
    result := TFPgtkVScrollbar.CreateFromObject (gtkwidg);
end;

procedure TFPgtkScrolledWindow.UpdatePolicy (UpdPolicy:TGtkUpdateType);
var sb : TFpgtkScrollbar;
begin
  sb := HScrollbar;
  if assigned(sb) then
    sb.UpdatePolicy := UpdPolicy;
  sb := VScrollbar;
  if assigned(sb) then
    sb.UpdatePolicy := UpdPolicy;
end;

 { TFPgtkViewport }

function TFPgtkViewport.TheGtkObject : PGtkViewport;
begin
  result := PgtkViewport(FGtkObject);
end;


procedure TFPgtkViewport.CreateGtkObject;
var h, v : PgtkAdjustment;
begin
  if assigned (FHScroll) then
    h := PGtkAdjustment(ConvertToGtkObject(FHScroll))
  else
    h := null;
  if assigned (FVScroll) then
    v := PGtkAdjustment(ConvertToGtkObject(FVScroll))
  else
    v := null;
  FGtkObject := PGtkObject (gtk_scrolled_window_new (h, v));
end;

constructor TFPgtkViewport.Create (hadj:TFPgtkAdjustment; vadj:TFPgtkAdjustment);
begin
  FVScroll := vadj;
  FHScroll := hadj;
  inherited create;
end;


function TFPgtkViewport.GetHAdjustment : TFPgtkAdjustment;
begin
  result := GetPascalInstance(PGtkObject(gtk_viewport_get_hadjustment(TheGtkObject)),tfpgtkadjustment) as tfpgtkadjustment;
end;

procedure TFPgtkViewport.SetHAdjustment (TheValue:TFPgtkAdjustment);
begin
  gtk_viewport_set_hadjustment(TheGtkObject,PGtkadjustment(ConvertToGtkObject(TheValue)));
end;

function TFPgtkViewport.GetVAdjustment : TFPgtkAdjustment;
begin
  result := GetPascalInstance(PGtkObject(gtk_viewport_get_vadjustment(TheGtkObject)),tfpgtkadjustment) as tfpgtkadjustment;
end;

procedure TFPgtkViewport.SetVAdjustment (TheValue:TFPgtkAdjustment);
begin
  gtk_viewport_set_vadjustment(TheGtkObject,PGtkadjustment(ConvertToGtkObject(TheValue)));
end;

function TFPgtkViewport.GetShadowType : TgtkShadowType;
begin
  result := TheGtkObject^.shadow_type;
end;

procedure TFPgtkViewport.SetShadowType (TheValue:TgtkShadowType);
begin
  gtk_viewport_set_shadow_type(TheGtkObject,TheValue);
end;

 { TFPgtkBox }

function TFPgtkBox.TheGtkObject : PGtkBox;
begin
  result := PgtkBox(FGtkObject);
end;


function TFPgtkBox.GetHomogeneous : boolean;
begin
  result := boolean(gtk.homogeneous(TheGtkObject^));
end;

procedure TFPgtkBox.SetHomogeneous (TheValue:boolean);
begin
  gtk_Box_set_homogeneous(TheGtkObject,TheValue);
end;

function TFPgtkBox.GetSpacing : integer;
begin
  result := TheGtkObject^.spacing;
end;

procedure TFPgtkBox.SetSpacing (TheValue:integer);
begin
  gtk_Box_set_spacing(TheGtkObject,TheValue);
end;

procedure TFPgtkBox.ReorderChild (Widget:TFPgtkWidget; Position:integer);
begin
  gtk_Box_reorder_child (TheGtkObject, PGtkwidget(ConvertToGtkObject(Widget)), Position);
end;

procedure TFPgtkBox.GetChildPacking (Widget:TFPgtkWidget; var Expand:boolean; var Fill:boolean; var Padding:integer; var PackType:TGtkPackType);
var PT : PGtkPackType;
begin
  pt := @PackType;
  gtk_box_query_child_packing (TheGtkObject, ConvertToGtkWidget(Widget),
                               pgboolean(@expand), pgboolean(@fill), pguint(@padding), pt);
end;

procedure TFPgtkBox.SetChildPacking (Widget:TFPgtkWidget; Expand:boolean; Fill:boolean; Padding:integer; PackType:TGtkPackType);
begin
  gtk_Box_set_child_packing (TheGtkObject, PGtkwidget(ConvertToGtkObject(Widget)), Expand, Fill, Padding, PackType);
end;

procedure TFPgtkBox.PackStart (Widget:TFPgtkWidget); Overload;
begin
  gtk_box_pack_start_defaults (TheGtkObject, ConvertToGtkWidget(Widget));
  widget.Show;
end;

procedure TFPgtkBox.PackStart (Widget:TFPgtkWidget; IsVisible:boolean); Overload;
begin
  gtk_box_pack_start_defaults (TheGtkObject, ConvertToGtkWidget(Widget));
  if isvisible then
    widget.Show;
end;

procedure TFPgtkBox.PackStart (Widget:TFPgtkWidget; expand:boolean; fill:boolean; padding:integer); Overload;
begin
  gtk_box_pack_start (TheGtkObject, ConvertToGtkWidget(Widget), expand, fill, padding);
  widget.Show;
end;

procedure TFPgtkBox.PackStart (Widget:TFPgtkWidget; expand:boolean; fill:boolean; padding:integer; IsVisible:boolean); Overload;
begin
  gtk_box_pack_start (TheGtkObject, ConvertToGtkWidget(Widget), expand, fill, padding);
  if isvisible then
    widget.Show;
end;

procedure TFPgtkBox.PackEnd (Widget:TFPgtkWidget); Overload;
begin
  gtk_box_pack_end_defaults (TheGtkObject, ConvertToGtkWidget(Widget));
  widget.Show;
end;

procedure TFPgtkBox.PackEnd (Widget:TFPgtkWidget; IsVisible:boolean); Overload;
begin
  gtk_box_pack_end_defaults (TheGtkObject, ConvertToGtkWidget(Widget));
  if isvisible then
    widget.Show;
end;

procedure TFPgtkBox.PackEnd (Widget:TFPgtkWidget; expand:boolean; fill:boolean; padding:integer); Overload;
begin
  gtk_box_pack_end (TheGtkObject, ConvertToGtkWidget(Widget), expand, fill, padding);
  widget.Show;
end;

procedure TFPgtkBox.PackEnd (Widget:TFPgtkWidget; expand:boolean; fill:boolean; padding:integer; IsVisible:boolean); Overload;
begin
  gtk_box_pack_end (TheGtkObject, ConvertToGtkWidget(Widget), expand, fill, padding);
  if isvisible then
    widget.Show;
end;

 { TFPgtkButtonBox }

function TFPgtkButtonBox.TheGtkObject : PGtkButtonBox;
begin
  result := PgtkButtonBox(FGtkObject);
end;


procedure SetButtonBoxDefaultSize (aMinWidth:integer; aMinHeight:integer);
begin
  gtk_button_box_set_child_size_default (aMinWidth, aMinheight);
end;


procedure GetButtonBoxDefaultSize (var aMinWidth:integer; var aMinHeight:integer);
begin
  gtk_button_box_get_child_size_default (@aMinWidth, @aMinheight);
end;


procedure SetButtonBoxDefaultPadding (aIPadX:integer; aIPadY:integer);
begin
  gtk_button_box_set_child_size_default (aIPadX, aIPadY);
end;


procedure GetButtonBoxDefaultPadding (var aIPadX:integer; var aIPadY:integer);
begin
  gtk_button_box_get_child_size_default (@aIPadX, @aIPadY);
end;


function TFPgtkButtonBox.GetSpacing : integer;
begin
  result := gtk_button_box_get_spacing(TheGtkObject);
end;

procedure TFPgtkButtonBox.SetSpacing (TheValue:integer);
begin
  gtk_button_box_set_spacing(TheGtkObject,TheValue);
end;

function TFPgtkButtonBox.GetLayout : TGtkButtonBoxStyle;
begin
  result := gtk_button_box_get_layout(TheGtkObject);
end;

procedure TFPgtkButtonBox.SetLayout (TheValue:TGtkButtonBoxStyle);
begin
  gtk_button_box_set_layout(TheGtkObject,TheValue);
end;

function TFPgtkButtonBox.GetMinWidth : integer;
var x, y : integer;
begin
  gtk_button_box_get_child_size (TheGtkObject, @x, @y);
  result := x;
end;

procedure TFPgtkButtonBox.SetMinWidth (TheValue:integer);
begin
  gtk_button_box_set_child_size (TheGtkObject, TheValue, ChildMinHeight);
end;

function TFPgtkButtonBox.GetMinHeight : integer;
var x, y : integer;
begin
  gtk_button_box_get_child_size (TheGtkObject, @x, @y);
  result := y;
end;

procedure TFPgtkButtonBox.SetMinHeight (TheValue:integer);
begin
  gtk_button_box_set_child_size (TheGtkObject, ChildMinWidth, TheValue);
end;

function TFPgtkButtonBox.GetChildPadX : integer;
var x, y : integer;
begin
  gtk_button_box_get_child_ipadding (TheGtkObject, @x, @y);
  result := x;
end;

procedure TFPgtkButtonBox.SetChildPadX (TheValue:integer);
begin
  gtk_button_box_set_child_ipadding (TheGtkObject, TheValue, ChildPadY);
end;

function TFPgtkButtonBox.GetChildPadY : integer;
var x, y : integer;
begin
  gtk_button_box_get_child_ipadding (TheGtkObject, @x, @y);
  result := y;
end;

procedure TFPgtkButtonBox.SetChildPadY (TheValue:integer);
begin
  gtk_button_box_set_child_ipadding (TheGtkObject, ChildPadX, TheValue);
end;

 { TFPgtkHButtonBox }

function TFPgtkHButtonBox.TheGtkObject : PGtkHButtonBox;
begin
  result := PgtkHButtonBox(FGtkObject);
end;

procedure TFPgtkHButtonBox.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_hbutton_box_new);
end;


 { TFPgtkVButtonBox }

function TFPgtkVButtonBox.TheGtkObject : PGtkVButtonBox;
begin
  result := PgtkVButtonBox(FGtkObject);
end;

procedure TFPgtkVButtonBox.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_vbutton_box_new);
end;


 { TFPgtkVBox }

function TFPgtkVBox.TheGtkObject : PGtkVBox;
begin
  result := PgtkVBox(FGtkObject);
end;

procedure TFPgtkVBox.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_VBox_new (False, 1));
end;


 { TFPgtkColorSelection }

function TFPgtkColorSelection.TheGtkObject : PGtkColorSelection;
begin
  result := PgtkColorSelection(FGtkObject);
end;

procedure TFPgtkColorSelection.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_color_selection_new);
end;


function TFPgtkColorSelection.GetUpdatePolicy : TGtkUpdateType;
begin
  result := TheGtkObject^.policy;
end;

procedure TFPgtkColorSelection.SetUpdatePolicy (TheValue:TGtkUpdateType);
begin
  gtk_color_selection_set_update_policy(TheGtkObject,TheValue);
end;

function TFPgtkColorSelection.GetColor : double;
var c : double;
begin
  gtk_color_selection_get_color (TheGtkObject, @c);
  result := c;
end;

procedure TFPgtkColorSelection.SetColor (TheValue:double);
begin
  gtk_color_selection_set_color (TheGtkObject, @TheValue);
end;

function TFPgtkColorSelection.GetUseOpacity : longbool;
begin
  result := longbool(TheGtkObject^.use_opacity);
end;

procedure TFPgtkColorSelection.SetUseOpacity (TheValue:longbool);
begin
  gtk_color_selection_set_opacity(TheGtkObject,gint(TheValue));
end;

 { TFPgtkGammaCurve }

function TFPgtkGammaCurve.TheGtkObject : PGtkGammaCurve;
begin
  result := PgtkGammaCurve(FGtkObject);
end;

procedure TFPgtkGammaCurve.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_gamma_curve_new);
end;


 { TFPgtkHBox }

function TFPgtkHBox.TheGtkObject : PGtkHBox;
begin
  result := PgtkHBox(FGtkObject);
end;

procedure TFPgtkHBox.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_HBox_new (False, 1));
end;


 { TFPgtkCombo }

function TFPgtkCombo.TheGtkObject : PGtkCombo;
begin
  result := PgtkCombo(FGtkObject);
end;

procedure TFPgtkCombo.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_combo_new);
end;


function TFPgtkCombo.GetEntry : TFPgtkEntry;
begin
  result := GetPascalInstance(PGtkObject(TheGtkObject^.entry), TFPgtkEntry) as tfpgtkentry;
end;

function TFPgtkCombo.GetList : TFPgtkList;
begin
  result := GetPascalInstance(PGtkObject(TheGtkObject^.list), TFPgtkList) as TFPgtkList;
end;

function TFPgtkCombo.GetButton : TFpGtkButton;
begin
  result := GetPascalInstance(PGtkObject(TheGtkObject^.button), TFPgtkButton) as TFPgtkButton;
end;

function TFPgtkCombo.GetValueInList : longbool;
begin
  result := longbool(gtk.value_in_list(TheGtkObject^));
end;

procedure TFPgtkCombo.SetValueInListProp (TheValue:longbool);
begin
  gtk_combo_set_value_in_list (TheGtkObject, gint(TheValue), gint(OkIfEmpty));
end;

function TFPgtkCombo.GetOkIfEmpty : longbool;
begin
  result := longbool(gtk.ok_if_empty(TheGtkObject^));
end;

procedure TFPgtkCombo.SetOkIfEmpty (TheValue:longbool);
begin
  gtk_combo_set_value_in_list (TheGtkObject, gint(ValueInList), gint(TheValue));
end;

function TFPgtkCombo.GetUseArrows : longbool;
begin
  result := longbool(gtk.use_arrows(TheGtkObject^));
end;

procedure TFPgtkCombo.SetUseArrows (TheValue:longbool);
begin
  gtk_combo_set_use_arrows(TheGtkObject,gint(TheValue));
end;

function TFPgtkCombo.GetUseArrowsAlways : longbool;
begin
  result := longbool(gtk.use_arrows_always(TheGtkObject^));
end;

procedure TFPgtkCombo.SetUseArrowsAlways (TheValue:longbool);
begin
  gtk_combo_set_use_arrows_always(TheGtkObject,gint(TheValue));
end;

function TFPgtkCombo.GetCaseSensitive : longbool;
begin
  result := longbool(gtk.case_sensitive(TheGtkObject^));
end;

procedure TFPgtkCombo.SetCaseSensitive (TheValue:longbool);
begin
  gtk_combo_set_case_sensitive(TheGtkObject,gint(TheValue));
end;

procedure TFPgtkCombo.SetItemString (Item:TFPgtkItem; ItemValue:string);
begin
  gtk_combo_set_item_string (TheGtkObject, PGtkitem(ConvertToGtkObject(Item)), ConvertToPgchar(ItemValue));
end;

procedure TFPgtkCombo.DisableActivate;
begin
  gtk_combo_disable_activate (TheGtkObject);
end;

procedure TFPgtkCombo.SetValueInList (Val:longbool; IsOkIfEmpty:longbool);
begin
  gtk_combo_set_value_in_list (TheGtkObject, gint(Val), gint(IsOkIfEmpty));
end;

 { TFPgtkStatusbar }

function TFPgtkStatusbar.TheGtkObject : PGtkStatusbar;
begin
  result := PgtkStatusbar(FGtkObject);
end;

procedure TFPgtkStatusbar.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_statusbar_new);
end;


function TFPgtkStatusbar.GetContextID (ContextDescr:string) : integer;
begin
  result := gtk_statusbar_get_context_id (TheGtkObject, ConvertToPgchar(ContextDescr));
end;

function TFPgtkStatusbar.Push (contextID:integer; text:string) : integer;
begin
  result := gtk_statusbar_push (TheGtkObject, contextID, ConvertToPgchar(text));
end;

procedure TFPgtkStatusbar.Pop (contextID:integer);
begin
  gtk_statusbar_pop (TheGtkObject, contextID);
end;

procedure TFPgtkStatusbar.Remove (contextID:integer; MessageID:integer);
begin
  gtk_statusbar_remove (TheGtkObject, contextID, MessageID);
end;

procedure StatusbarSignalproc (Sender:PGtkobject; contextID:integer; text:pgChar; data:pointer); cdecl;
var p : TFPgtkStatusbarSignalFunction;
begin
with PSignalData(data)^ do
  begin
  p := TFPgtkStatusbarSignalFunction (TheSignalProc);
  p (TheWidget as TFPgtkObject, contextID, text, TheData)
  end;
end;

function TFPgtkStatusbar.StatusbarSignalConnect (signal:string; proc:TFPgtkStatusbarSignalFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect (FGtkObject, pgChar(signal), gtk_signal_func(@StatusbarSignalproc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

function TFPgtkStatusbar.StatusbarSignalConnectAfter (signal:string; proc:TFPgtkStatusbarSignalFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect_After (FGtkObject, pgChar(signal), gtk_signal_func(@StatusbarSignalproc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

function TFPgtkStatusbar.ConnectTextPopped (proc:TFPgtkStatusbarSignalFunction; data:pointer) : guint;
begin
  result := StatusbarSignalConnect (sgTextPopped, proc, data);
end;

function TFPgtkStatusbar.ConnectAfterTextPopped (proc:TFPgtkStatusbarSignalFunction; data:pointer) : guint;
begin
  result := StatusbarSignalConnectAfter (sgTextPopped, proc, data);
end;

function TFPgtkStatusbar.ConnectTextPushed (proc:TFPgtkStatusbarSignalFunction; data:pointer) : guint;
begin
  result := StatusbarSignalConnect (sgTextPushed, proc, data);
end;

function TFPgtkStatusbar.ConnectAfterTextPushed (proc:TFPgtkStatusbarSignalFunction; data:pointer) : guint;
begin
  result := StatusbarSignalConnectAfter (sgTextPushed, proc, data);
end;

 { TFPgtkCList }

function TFPgtkCList.TheGtkObject : PGtkCList;
begin
  result := PgtkCList(FGtkObject);
end;

procedure TFPgtkCList.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_clist_new (FColumnCount));
end;


constructor TFPgtkCList.Create (aColumnCount:integer);
begin
  FColumnCount := aColumnCount;
  inherited create;
end;


function TFPgtkCList.GetShadowType : TGtkShadowType;
begin
  result := TheGtkObject^.shadow_type;
end;

procedure TFPgtkCList.SetShadowType (TheValue:TGtkShadowType);
begin
  gtk_clist_set_shadow_type(TheGtkObject,TheValue);
end;

function TFPgtkCList.GetSelectionMode : TGtkSelectionMode;
begin
  result := TheGtkObject^.selection_mode;
end;

procedure TFPgtkCList.SetSelectionMode (TheValue:TGtkSelectionMode);
begin
  gtk_clist_set_selection_mode(TheGtkObject,TheValue);
end;

procedure TFPgtkCList.Freeze;
begin
  gtk_clist_freeze (TheGtkObject);
end;

procedure TFPgtkCList.Thaw;
begin
  gtk_clist_thaw (TheGtkObject);
end;

procedure TFPgtkCList.ShowTitles;
begin
  gtk_clist_Column_titles_show (TheGtkObject);
end;

procedure TFPgtkCList.HideTitles;
begin
  gtk_clist_column_titles_hide (TheGtkObject);
end;

procedure TFPgtkCList.ActiveTitles;
begin
  gtk_clist_column_titles_active (TheGtkObject);
end;

procedure TFPgtkCList.PassiveTitles;
begin
  gtk_clist_column_titles_passive (TheGtkObject);
end;

procedure TFPgtkCList.ActiveTitle (column:integer);
begin
  gtk_clist_column_title_active (TheGtkObject, column);
end;

procedure TFPgtkCList.PassiveTitle (column:integer);
begin
  gtk_clist_column_title_passive (TheGtkObject, column);
end;

function TFPgtkCList.GetColumnTitle (column:integer) : string;
begin
  result := gtk_clist_get_column_title(TheGtkObject,column);
end;

procedure TFPgtkCList.SetColumnTitle (column:integer; TheValue:string);
begin
  gtk_clist_set_column_title(TheGtkObject,column,ConvertToPgchar(TheValue));
end;

function TFPgtkCList.GetColumnWidget (column:integer) : TFPgtkWidget;
begin
  result := GetPascalInstance(PGtkObject(gtk_clist_get_column_widget(TheGtkObject,column)),tfpgtkwidget) as tfpgtkwidget;
end;

procedure TFPgtkCList.SetColumnWidget (column:integer; TheValue:TFPgtkWidget);
begin
  gtk_clist_set_column_widget(TheGtkObject,column,PGtkwidget(ConvertToGtkObject(TheValue)));
end;

procedure TFPgtkCList.SetColumnJustification (column:integer; justification:TGtkJustification);
begin
  gtk_clist_set_column_justification (TheGtkObject, column, justification);
end;

procedure TFPgtkCList.SetColumnVisibility (column:integer; aVisible:boolean);
begin
  gtk_clist_set_column_visibility (TheGtkObject, column, aVisible);
end;

procedure TFPgtkCList.SetColumnResizeable (column:integer; Resizeable:boolean);
begin
  gtk_clist_set_column_resizeable (TheGtkObject, column, Resizeable);
end;

procedure TFPgtkCList.SetColumnAutoResize (column:integer; autoResize:boolean);
begin
  gtk_clist_set_column_auto_resize (TheGtkObject, column, autoResize);
end;

function TFPgtkCList.OptimalColumnWidth (column:integer) : integer;
begin
  result := gtk_clist_optimal_column_width (TheGtkObject, column);
end;

procedure TFPgtkCList.SetColumnWidth (column:integer; width:integer);
begin
  gtk_clist_set_column_width (TheGtkObject, column, width);
end;

procedure TFPgtkCList.SetColumnMinWidth (column:integer; MinWidth:integer);
begin
  gtk_clist_set_column_min_width (TheGtkObject, column, MinWidth);
end;

procedure TFPgtkCList.SetColumnMaxWidth (column:integer; MaxWidth:integer);
begin
  gtk_clist_set_column_max_width (TheGtkObject, column, MaxWidth);
end;

function TFPgtkCList.AutoSizeColumns : integer;
begin
  result := gtk_clist_columns_autosize (TheGtkObject);
end;

procedure TFPgtkCList.ConfigureColumnWidth (column:integer; Width:integer; MinWidth:integer; MaxWidth:integer);
begin
  SetColumnWidth (column, Width);
  SetColumnMaxWidth (column, MaxWidth);
  SetColumnMinWidth (column, MinWidth);
end;

procedure TFPgtkCList.ConfigureColumn (column:integer; Justification:TGtkJustification; Visibility:boolean; Resizeable:boolean; AutoSize:boolean);
begin
  SetColumnJustification (column, Justification);
  SetColumnVisibility (column, Visibility);
  SetColumnResizeable (column, Resizeable);
  SetColumnAutoResize (column, AutoSize);
end;

procedure TFPgtkCList.SetRowHeight (height:integer);
begin
  gtk_clist_set_row_height (TheGtkObject, height);
end;

procedure TFPgtkCList.MoveTo (row:integer; column:integer; RowAlign:gfloat; ColAlign:gfloat);
begin
  gtk_clist_moveto (TheGtkObject, row, column, RowAlign, ColAlign);
end;

function TFPgtkCList.RowIsVisible (Row:integer) : TGtkVisibility;
begin
  result := gtk_clist_row_is_visible (TheGtkObject, Row);
end;

function TFPgtkCList.GetCellType (Row:integer; column:integer) : TGtkCellType;
begin
  result := gtk_clist_get_cell_type (TheGtkObject, Row, column);
end;

function TFPgtkCList.GetCellText (Row:integer; Column:integer) : string;
var s : pgchar;
    r : integer;
begin
  r := gtk_clist_get_text (TheGtkObject, row, column, @s);
  if (r = 0) then
    result := ''
  else
    result := strpas(s);
end;

procedure TFPgtkCList.SetCellText (Row:integer; Column:integer; TheValue:string);
begin
  gtk_clist_set_text(TheGtkObject,Row, Column,ConvertToPgchar(TheValue));
end;

procedure TFPgtkCList.SetPixmap (row:integer; column:integer; pixmap:PGdkPixmap; mask:PGdkBitmap);
begin
  gtk_clist_set_pixmap (TheGtkObject, row, column, pixmap, mask);
end;

procedure TFPgtkCList.GetPixmap (row:integer; column:integer; var pixmap:PGdkPixmap; var mask:PGdkBitmap);
begin
  gtk_clist_get_pixmap (TheGtkObject, row, column, @pixmap, @mask);
end;

procedure TFPgtkCList.SetPixText (row:integer; column:integer; text:string; spacing:guint8; pixmap:PGdkPixmap; mask:PGdkBitmap);
begin
  gtk_clist_set_pixtext (TheGtkObject, row, column, ConvertToPgchar(text), spacing, pixmap, mask);
end;

procedure TFPgtkCList.GetPixText (row:integer; column:integer; var text:string; var aspacing:guint8; var pixmap:PGdkPixmap; var mask:PGdkBitmap);
var r : integer;
    s : PPgchar;
begin
  s := nil;
  r := gtk_clist_get_pixtext (TheGtkObject, row, column, s, @aspacing, @pixmap, @mask);
  if r = 0 then
    begin
    text := '';
    pixmap := nil;
    mask := nil;
    end
  else
    text := string (s^);
end;

procedure TFPgtkCList.SetForeground (row:integer; color:PGdkColor);
begin
  gtk_clist_set_foreground (TheGtkObject, row, color);
end;

procedure TFPgtkCList.SetBackground (row:integer; color:PGdkColor);
begin
  gtk_clist_set_background (TheGtkObject, row, color);
end;

function TFPgtkCList.GetCellStyle (row:integer; column:integer) : PGtkStyle;
begin
  result := gtk_clist_get_cell_style(TheGtkObject,row, column);
end;

procedure TFPgtkCList.SetCellStyle (row:integer; column:integer; TheValue:PGtkStyle);
begin
  gtk_clist_set_cell_style(TheGtkObject,row, column,TheValue);
end;

function TFPgtkCList.GetRowStyle (row:integer) : PGtkStyle;
begin
  result := gtk_clist_get_row_style(TheGtkObject,row);
end;

procedure TFPgtkCList.SetRowStyle (row:integer; TheValue:PGtkStyle);
begin
  gtk_clist_set_row_style(TheGtkObject,row,TheValue);
end;

procedure TFPgtkCList.SetShift (row:integer; column:integer; vertical:integer; horizontal:integer);
begin
  gtk_clist_set_shift (TheGtkObject, row, column, vertical, horizontal);
end;

procedure TFPgtkCList.Remove (row:integer);
begin
  gtk_clist_remove (TheGtkObject, row);
end;

procedure TFPgtkCList.Prepend (Data:TStrings); Overload;
var ppdata : ppgchar;
begin
  ppdata := StringsToPPgchar (Data);
  gtk_clist_prepend (TheGtkObject, ppdata);
  freemem (ppdata, sizeof (pgchar) * data.count);
end;

procedure TFPgtkCList.Prepend (Text:string; separator:string); Overload;
var l : TStrings;
    s : string;
begin
  l := TStringList.Create;
  try
    if pos('"',separator) = 0 then
      s := stringreplace (Text, '"', '""', [rfReplaceAll]);
    if separator <> '' then
      s := stringreplace(Text, separator, '","', [rfReplaceAll]);
    l.CommaText := '"'+s+'"';
    Prepend (l);
  finally
    l.Free;
  end;
end;

procedure TFPgtkCList.Prepend (data:array of string); Overload;
var ppdata : ppgchar;
begin
  ppdata := ArrayToPPgchar (Data);
  gtk_clist_prepend (TheGtkObject, ppdata);
  freemem (ppdata, sizeof (pgchar) * (high(data)-low(data)+1));
end;

Function TFPgtkCList.Append (data:TStrings) : Integer; Overload;
var ppdata : ppgchar;
begin
  ppdata := StringsToPPgchar (Data);
  Result:=gtk_clist_append (TheGtkObject, ppdata);
  freemem (ppdata, sizeof (pgchar) * data.count);
end;

Function TFPgtkCList.Append (Text:string; Separator:string) : Integer; Overload;
var l : TStrings;
    s : string;
begin
  l := TStringList.Create;
  try
    if pos('"',separator) = 0 then
      s := stringreplace (Text, '"', '""', [rfReplaceAll]);
    if separator <> '' then
      s := stringreplace(Text, separator, '","', [rfReplaceAll]);
    l.CommaText := '"' + s + '"';
    Result:=Append (l);
  finally
    l.Free;
  end;
end;

Function TFPgtkCList.Append (data:array of string) : Integer; Overload;
var ppdata : ppgchar;
begin
  ppdata := ArrayToPPgchar (Data);
  Result:=gtk_clist_append (TheGtkObject, ppdata);
  freemem (ppdata, sizeof (pgchar) * (high(data)-low(data)+1));
end;

procedure TFPgtkCList.Insert (row:integer; data:TStrings); Overload;
var ppdata : ppgchar;
begin
  ppdata := StringsToPPgchar (Data);
  gtk_clist_insert (TheGtkObject, row, ppdata);
  freemem (ppdata, sizeof (pgchar) * data.count);
end;

procedure TFPgtkCList.Insert (row:integer; Text:string; Separator:string); Overload;
var l : TStrings;
    s : string;
begin
  l := TStringList.Create;
  try
    if pos('"',separator) = 0 then
      s := stringreplace (Text, '"', '""', [rfReplaceAll]);
    if separator <> '' then
      s := stringreplace(Text, separator, '","', [rfReplaceAll]);
    l.CommaText := '"' + s + '"';
    Insert (row, l);
  finally
    l.Free;
  end;
end;

procedure TFPgtkCList.Insert (row:integer; data:array of string); Overload;
var ppdata : ppgchar;
begin
  ppdata := ArrayToPPgchar (Data);
  gtk_clist_insert (TheGtkObject, row, ppdata);
  freemem (ppdata, sizeof (pgchar) * (high(data)-low(data)+1));
end;

function TFPgtkCList.GetRowData (row:integer) : pointer;
begin
  result := gtk_clist_get_row_data(TheGtkObject,row);
end;

procedure TFPgtkCList.SetRowData (row:integer; TheValue:pointer);
begin
  gtk_clist_set_row_data(TheGtkObject,row,TheValue);
end;

function TFPgtkCList.FindRowFromData (data:pointer) : integer;
begin
  result := gtk_clist_find_row_from_data (TheGtkObject, data);
end;

procedure TFPgtkCList.SelectRow (row:integer; column:integer);
begin
  gtk_clist_select_row (TheGtkObject, row, column);
end;

procedure TFPgtkCList.UnselectRow (row:integer; column:integer);
begin
  gtk_clist_unselect_row (TheGtkObject, row, column);
end;

procedure TFPgtkCList.Clear;
begin
  gtk_clist_clear (TheGtkObject);
end;

procedure TFPgtkCList.SelectAll;
begin
  gtk_clist_select_all (TheGtkObject);
end;

procedure TFPgtkCList.UnselectAll;
begin
  gtk_clist_unselect_all (TheGtkObject);
end;

procedure TFPgtkCList.SwapRows (row1:integer; row2:integer);
begin
  gtk_clist_swap_rows (TheGtkObject, row1, row2);
end;

procedure TFPgtkCList.RowMove (sourceRow:integer; destRow:integer);
begin
  if sourceRow = DestRow then
    Exit;
  gtk_clist_row_move (TheGtkObject, sourceRow, destRow);
end;

procedure TFPgtkCList.Sort;
begin
  gtk_clist_sort (TheGtkObject);
end;

procedure TFPgtkCList.SetCompareFunc (TheValue:TGtkCListCompareFunc);
begin
  gtk_clist_set_Compare_func(TheGtkObject,TheValue);
end;

function TFPgtkCList.GetSortColumn : integer;
begin
  result := TheGtkObject^.sort_column;
end;

procedure TFPgtkCList.SetSortColumn (TheValue:integer);
begin
  gtk_clist_set_sort_column(TheGtkObject,TheValue);
end;

function TFPgtkCList.GetSetSortType : TGtkSortType;
begin
  result := TheGtkObject^.sort_type;
end;

procedure TFPgtkCList.SetSetSortType (TheValue:TGtkSortType);
begin
  gtk_clist_set_sort_type(TheGtkObject,TheValue);
end;

procedure TFPgtkCList.SetAutoSort (autoSort:boolean);
begin
  gtk_clist_set_auto_sort (TheGtkObject, autoSort);
end;

function TFPgtkCList.GetHAdjustment : TFPgtkAdjustment;
begin
  result := GetPascalInstance(PGtkObject(gtk_clist_get_hadjustment(TheGtkObject)),tfpgtkadjustment) as tfpgtkadjustment;
end;

procedure TFPgtkCList.SetHAdjustment (TheValue:TFPgtkAdjustment);
begin
  gtk_clist_set_hadjustment(TheGtkObject,PGtkadjustment(ConvertToGtkObject(TheValue)));
end;

function TFPgtkCList.GetVAdjustment : TFPgtkAdjustment;
begin
  result := GetPascalInstance(PGtkObject(gtk_clist_get_vadjustment(TheGtkObject)),tfpgtkadjustment) as tfpgtkadjustment;
end;

procedure TFPgtkCList.SetVAdjustment (TheValue:TFPgtkAdjustment);
begin
  gtk_clist_set_vadjustment(TheGtkObject,PGtkadjustment(ConvertToGtkObject(TheValue)));
end;

procedure TFPgtkCList.SetReorderable (reorderable:boolean);
begin
  gtk_clist_set_reorderable (TheGtkObject, reorderable);
end;

function TFPgtkCList.Count : integer;
begin
  result := TheGtkObject^.rows;
end;

procedure CListScrollSignalproc (Sender:PGtkobject; ScrollType:TgtkScrollType; position:gfloat; data:pointer); cdecl;
var p : TFPgtkCListScrollSignalFunction;
begin
with PSignalData(data)^ do
  begin
  p := TFPgtkCListScrollSignalFunction (TheSignalProc);
  p (TheWidget as TFPgtkObject, ScrollType, position, TheData)
  end;
end;

function TFPgtkCList.CListScrollSignalConnect (signal:string; proc:TFPgtkCListScrollSignalFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect (FGtkObject, pgChar(signal), gtk_signal_func(@CListScrollSignalproc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

function TFPgtkCList.CListScrollSignalConnectAfter (signal:string; proc:TFPgtkCListScrollSignalFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect_After (FGtkObject, pgChar(signal), gtk_signal_func(@CListScrollSignalproc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

procedure CListScrollBooleanSignalproc (Sender:PGtkobject; ScrollType:TgtkScrollType; Position:gfloat; AutoStartSelection:boolean; data:pointer); cdecl;
var p : TFPgtkCListScrollBooleanSignalFunction;
begin
with PSignalData(data)^ do
  begin
  p := TFPgtkCListScrollBooleanSignalFunction (TheSignalProc);
  p (TheWidget as TFPgtkObject, ScrollType, Position, AutoStartSelection, TheData)
  end;
end;

function TFPgtkCList.CListScrollBooleanSignalConnect (signal:string; proc:TFPgtkCListScrollBooleanSignalFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect (FGtkObject, pgChar(signal), gtk_signal_func(@CListScrollBooleanSignalproc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

function TFPgtkCList.CListScrollBooleanSignalConnectAfter (signal:string; proc:TFPgtkCListScrollBooleanSignalFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect_After (FGtkObject, pgChar(signal), gtk_signal_func(@CListScrollBooleanSignalproc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

procedure SelectRowSignalproc (Sender:PGtkobject; row:integer; column:integer; event:PGdkEventButton; data:pointer); cdecl;
var p : TFPgtkSelectRowSignalFunction;
begin
with PSignalData(data)^ do
  begin
  p := TFPgtkSelectRowSignalFunction (TheSignalProc);
  p (TheWidget as TFPgtkObject, row, column, event, TheData)
  end;
end;

function TFPgtkCList.SelectRowSignalConnect (signal:string; proc:TFPgtkSelectRowSignalFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect (FGtkObject, pgChar(signal), gtk_signal_func(@SelectRowSignalproc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

function TFPgtkCList.SelectRowSignalConnectAfter (signal:string; proc:TFPgtkSelectRowSignalFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect_After (FGtkObject, pgChar(signal), gtk_signal_func(@SelectRowSignalproc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

function TFPgtkCList.ConnectSelectRow (proc:TFPgtkSelectRowSignalFunction; data:pointer) : guint;
begin
  result := SelectRowSignalConnect (sgSelectRow, proc, data);
end;

function TFPgtkCList.ConnectAfterSelectRow (proc:TFPgtkSelectRowSignalFunction; data:pointer) : guint;
begin
  result := SelectRowSignalConnectAfter (sgSelectRow, proc, data);
end;

function TFPgtkCList.ConnectUnselectRow (proc:TFPgtkSelectRowSignalFunction; data:pointer) : guint;
begin
  result := SelectRowSignalConnect (sgUnselectRow, proc, data);
end;

function TFPgtkCList.ConnectAfterUnselectRow (proc:TFPgtkSelectRowSignalFunction; data:pointer) : guint;
begin
  result := SelectRowSignalConnectAfter (sgUnselectRow, proc, data);
end;

procedure MoveSignalproc (Sender:PGtkobject; arg1:integer; arg2:integer; data:pointer); cdecl;
var p : TFPgtkMoveSignalFunction;
begin
with PSignalData(data)^ do
  begin
  p := TFPgtkMoveSignalFunction (TheSignalProc);
  p (TheWidget as TFPgtkObject, arg1, arg2, TheData)
  end;
end;

function TFPgtkCList.MoveSignalConnect (signal:string; proc:TFPgtkMoveSignalFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect (FGtkObject, pgChar(signal), gtk_signal_func(@MoveSignalproc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

function TFPgtkCList.MoveSignalConnectAfter (signal:string; proc:TFPgtkMoveSignalFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect_After (FGtkObject, pgChar(signal), gtk_signal_func(@MoveSignalproc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

function TFPgtkCList.ConnectRowMove (proc:TFPgtkMoveSignalFunction; data:pointer) : guint;
begin
  result := MoveSignalConnect (sgRowMove, proc, data);
end;

function TFPgtkCList.ConnectAfterRowMove (proc:TFPgtkMoveSignalFunction; data:pointer) : guint;
begin
  result := MoveSignalConnectAfter (sgRowMove, proc, data);
end;

function TFPgtkCList.ConnectScrollVertical (proc:TFPgtkCListScrollSignalFunction; data:pointer) : guint;
begin
  result := CListScrollSignalConnect (sgScrollVertical, proc, data);
end;

function TFPgtkCList.ConnectAfterScrollVertical (proc:TFPgtkCListScrollSignalFunction; data:pointer) : guint;
begin
  result := CListScrollSignalConnectAfter (sgScrollVertical, proc, data);
end;

function TFPgtkCList.ConnectScrolHorizontal (proc:TFPgtkCListScrollSignalFunction; data:pointer) : guint;
begin
  result := CListScrollSignalConnect (sgScrolHorizontal, proc, data);
end;

function TFPgtkCList.ConnectAfterScrolHorizontal (proc:TFPgtkCListScrollSignalFunction; data:pointer) : guint;
begin
  result := CListScrollSignalConnectAfter (sgScrolHorizontal, proc, data);
end;

function TFPgtkCList.ConnectToggleFocusRow (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnect (sgToggleFocusRow, proc, data);
end;

function TFPgtkCList.ConnectAfterToggleFocusRow (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnectAfter (sgToggleFocusRow, proc, data);
end;

function TFPgtkCList.ConnectSelectAll (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnect (sgSelectAll, proc, data);
end;

function TFPgtkCList.ConnectAfterSelectAll (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnectAfter (sgSelectAll, proc, data);
end;

function TFPgtkCList.ConnectUnselectAll (proc:TFPgtksignalFunction; data:pointer) : guint;
begin
  result := signalConnect (sgUnselectAll, proc, data);
end;

function TFPgtkCList.ConnectAfterUnselectAll (proc:TFPgtksignalFunction; data:pointer) : guint;
begin
  result := signalConnectAfter (sgUnselectAll, proc, data);
end;

function TFPgtkCList.ConnectUndoSelection (proc:TFPgtksignalFunction; data:pointer) : guint;
begin
  result := signalConnect (sgUndoSelection, proc, data);
end;

function TFPgtkCList.ConnectAfterUndoSelection (proc:TFPgtksignalFunction; data:pointer) : guint;
begin
  result := signalConnectAfter (sgUndoSelection, proc, data);
end;

function TFPgtkCList.ConnectStartSelection (proc:TFPgtksignalFunction; data:pointer) : guint;
begin
  result := signalConnect (sgStartSelection, proc, data);
end;

function TFPgtkCList.ConnectAfterStartSelection (proc:TFPgtksignalFunction; data:pointer) : guint;
begin
  result := signalConnectAfter (sgStartSelection, proc, data);
end;

function TFPgtkCList.ConnectEndSelection (proc:TFPgtksignalFunction; data:pointer) : guint;
begin
  result := signalConnect (sgEndSelection, proc, data);
end;

function TFPgtkCList.ConnectAfterEndSelection (proc:TFPgtksignalFunction; data:pointer) : guint;
begin
  result := signalConnectAfter (sgEndSelection, proc, data);
end;

function TFPgtkCList.ConnectToggleAddMode (proc:TFPgtksignalFunction; data:pointer) : guint;
begin
  result := signalConnect (sgToggleAddMode, proc, data);
end;

function TFPgtkCList.ConnectAfterToggleAddMode (proc:TFPgtksignalFunction; data:pointer) : guint;
begin
  result := signalConnectAfter (sgToggleAddMode, proc, data);
end;

function TFPgtkCList.ConnectAbortColumnResize (proc:TFPgtksignalFunction; data:pointer) : guint;
begin
  result := signalConnect (sgAbortColumnResize, proc, data);
end;

function TFPgtkCList.ConnectAfterAbortColumnResize (proc:TFPgtksignalFunction; data:pointer) : guint;
begin
  result := signalConnectAfter (sgAbortColumnResize, proc, data);
end;

function TFPgtkCList.ConnectExtendSelection (proc:TFPgtkCListScrollBooleanSignalFunction; data:pointer) : guint;
begin
  result := CListScrollBooleanSignalConnect (sgExtendSelection, proc, data);
end;

function TFPgtkCList.ConnectAfterExtendSelection (proc:TFPgtkCListScrollBooleanSignalFunction; data:pointer) : guint;
begin
  result := CListScrollBooleanSignalConnectAfter (sgExtendSelection, proc, data);
end;

procedure ColumnClickedSignalproc (Sender:PGtkobject; column:integer; data:pointer); cdecl;
var p : TFPgtkColumnClickedSignalFunction;
begin
with PSignalData(data)^ do
  begin
  p := TFPgtkColumnClickedSignalFunction (TheSignalProc);
  p (TheWidget as TFPgtkObject, column, TheData)
  end;
end;

function TFPgtkCList.ColumnClickedSignalConnect (signal:string; proc:TFPgtkColumnClickedSignalFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect (FGtkObject, pgChar(signal), gtk_signal_func(@ColumnClickedSignalproc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

function TFPgtkCList.ColumnClickedSignalConnectAfter (signal:string; proc:TFPgtkColumnClickedSignalFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect_After (FGtkObject, pgChar(signal), gtk_signal_func(@ColumnClickedSignalproc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

function TFPgtkCList.ConnectClickColumn (proc:TFPgtkColumnClickedSignalFunction; data:pointer) : guint;
begin
  result := ColumnClickedSignalConnect (sgClickColumn, proc, data);
end;

function TFPgtkCList.ConnectAfterClickColumn (proc:TFPgtkColumnClickedSignalFunction; data:pointer) : guint;
begin
  result := ColumnClickedSignalConnectAfter (sgClickColumn, proc, data);
end;

procedure ResizeColumnSignalproc (Sender:PGtkobject; column:integer; width:integer; data:pointer); cdecl;
var p : TFPgtkResizeColumnSignalFunction;
begin
with PSignalData(data)^ do
  begin
  p := TFPgtkResizeColumnSignalFunction (TheSignalProc);
  p (TheWidget as TFPgtkObject, column, width, TheData)
  end;
end;

function TFPgtkCList.ResizeColumnSignalConnect (signal:string; proc:TFPgtkResizeColumnSignalFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect (FGtkObject, pgChar(signal), gtk_signal_func(@ResizeColumnSignalproc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

function TFPgtkCList.ResizeColumnSignalConnectAfter (signal:string; proc:TFPgtkResizeColumnSignalFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect_After (FGtkObject, pgChar(signal), gtk_signal_func(@ResizeColumnSignalproc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

function TFPgtkCList.ConnectResizeColumn (proc:TFPgtkResizeColumnSignalFunction; data:pointer) : guint;
begin
  result := ResizeColumnSignalConnect (sgResizeColumn, proc, data);
end;

function TFPgtkCList.ConnectAfterResizeColumn (proc:TFPgtkResizeColumnSignalFunction; data:pointer) : guint;
begin
  result := ResizeColumnSignalConnectAfter (sgResizeColumn, proc, data);
end;

 { TFPgtkCTree }

function TFPgtkCTree.TheGtkObject : PGtkCTree;
begin
  result := PgtkCTree(FGtkObject);
end;


function TFPgtkCTree.GetLineStyle : TGtkCTreeLineStyle;
begin
  result := TGtkCTreeLineStyle(gtk.line_style(TheGtkObject^));
end;

procedure TFPgtkCTree.SetLineStyle (TheValue:TGtkCTreeLineStyle);
begin
  gtk_ctree_set_line_style(TheGtkObject,TheValue);
end;

function TFPgtkCTree.GetShowStub : boolean;
begin
  result := boolean(gtk.show_stub(TheGtkObject^));
end;

procedure TFPgtkCTree.SetShowStub (TheValue:boolean);
begin
  gtk_ctree_set_show_stub(TheGtkObject,TheValue);
end;

function TFPgtkCTree.GetExpanderStyle : TGtkCTreeExpanderStyle;
begin
  result := TGtkCTreeExpanderStyle(gtk.expander_style(TheGtkObject^));
end;

procedure TFPgtkCTree.SetExpanderStyle (TheValue:TGtkCTreeExpanderStyle);
begin
  gtk_ctree_set_expander_style(TheGtkObject,TheValue);
end;

function TFPgtkCTree.GetSpacing : guint;
begin
  result := TheGtkObject^.tree_spacing;
end;

procedure TFPgtkCTree.SetSpacing (TheValue:guint);
begin
  gtk_ctree_set_spacing(TheGtkObject,TheValue);
end;

function TFPgtkCTree.GetIndent : guint;
begin
  result := TheGtkObject^.tree_indent;
end;

procedure TFPgtkCTree.SetIndent (TheValue:guint);
begin
  gtk_ctree_set_indent(TheGtkObject,TheValue);
end;

function TFPgtkCTree.GetTreeColumn : integer;
begin
  result := TheGtkObject^.tree_column;
end;

constructor TFPgtkCTree.Create (aColumnCount:integer; aTreeColumn:integer);
begin
  FTreeColumn := aTreeColumn;
  inherited Create (aColumnCount);
end;


procedure TFPgtkCTree.RemoveNode (node:PGtkCTreeNode);
begin
  gtk_ctree_remove_node (TheGtkObject, node);
end;

function TFPgtkCTree.InsertNode (aParent:PGtkCTreeNode; Sibling:PGtkCTreeNode; data:string; aSpacing:guint8; PixmapClosed:PGdkPixmap; MaskClosed:PGdkBitmap; PixmapOpened:PGdkPixmap; MaskOpened:PGdkBitmap; IsLeaf:boolean; Expanded:boolean) : PGtkCTreeNode; Overload;
var
  temppgc : pgchar;
begin
  temppgc:=ConvertToPgchar(data);
  result := gtk_ctree_insert_node (TheGtkObject, aParent, Sibling, @temppgc, aSpacing, PixmapClosed, MaskClosed, PixmapOpened, MaskOpened, IsLeaf, Expanded);
end;

function TFPgtkCTree.InsertNode (aParent:PGtkCTreeNode; Sibling:PGtkCTreeNode; data:string; aSpacing:guint8; IsLeaf:boolean; Expanded:boolean) : PGtkCTreeNode; Overload;
begin
  result := InsertNode (aParent, Sibling, data, aSpacing, nil, nil, nil, nil, IsLeaf, Expanded);
end;

procedure FPgtkCTreeFunc (Tree:PGtkCTree; Node:PGtkCTreeNode; data:pointer); Cdecl;
var p : TFPgtkCTreeFunction;
begin
  with PSignalData(data)^ do
  begin
    p := TFPgtkCTreeFunction (TheSignalProc);
    p (TFPgtkCTree(GetPascalInstance(PgtkObject(Tree))), Node, data);
  end;
end;


procedure TFPgtkCTree.PostRecursive (aNode:PGtkCTreeNode; func:TFPgtkCTreeFunction; data:pointer);
begin
  gtk_CTree_post_recursive (TheGtkObject, aNode, @FPgtkCTreeFunc,
        ConvertSignalData(TFPgtkSignalFunction(func), data, true));
end;

procedure TFPgtkCTree.PostRecursiveToDepth (aNode:PGtkCTreeNode; aDepth:integer; func:TFPgtkCTreeFunction; data:pointer);
begin
  gtk_CTree_post_recursive_to_depth (TheGtkObject, aNode, aDepth, @FPgtkCTreeFunc,
        ConvertSignalData(TFPgtkSignalFunction(func), data, true));
end;

procedure TFPgtkCTree.PreRecursive (aNode:PGtkCTreeNode; func:TFPgtkCTreeFunction; data:pointer);
begin
  gtk_CTree_pre_recursive (TheGtkObject, aNode, @FPgtkCTreeFunc,
        ConvertSignalData(TFPgtkSignalFunction(func), data, true));
end;

procedure TFPgtkCTree.PreRecursiveToDepth (aNode:PGtkCTreeNode; aDepth:integer; func:TFPgtkCTreeFunction; data:pointer);
begin
  gtk_CTree_pre_recursive_to_depth (TheGtkObject, aNode, aDepth, @FPgtkCTreeFunc,
        ConvertSignalData(TFPgtkSignalFunction(func), data, true));
end;

procedure TFPgtkCTree.IsViewable (aNode:PGtkCTreeNode);
begin
  gtk_ctree_is_viewable (TheGtkObject, aNode);
end;

procedure TFPgtkCTree.LastChild (aNode:PGtkCTreeNode);
begin
  gtk_ctree_last (TheGtkObject, aNode);
end;

function TFPgtkCTree.IsChild (anAncestor:PGtkCTreeNode; aChild:PGtkCTreeNode) : boolean;
begin
  result := gtk_ctree_find (TheGtkObject, anAncestor, aChild);
end;

function TFPgtkCTree.IsAncestor (anAncestor:PGtkCTreeNode; aChild:PGtkCTreeNode) : boolean;
begin
  result := gtk_ctree_is_ancestor (TheGtkObject, anAncestor, aChild);
end;

function TFPgtkCTree.IsHotSpot (X:integer; Y:integer) : boolean;
begin
  result := gtk_ctree_is_hot_spot (TheGtkObject, X, Y);
end;

procedure TFPgtkCTree.MoveNode (aNode:PGtkCTreeNode; NewParent:PGtkCTreeNode; NewSibling:PGtkCTreeNode);
begin
  gtk_ctree_move (TheGtkObject, aNode, NewParent, NewSibling);
end;

procedure TFPgtkCTree.Expand (aNode:PGtkCTreeNode);
begin
  gtk_ctree_expand (TheGtkObject, aNode);
end;

procedure TFPgtkCTree.ExpandRecursive (aNode:PGtkCTreeNode);
begin
  gtk_ctree_expand_recursive (TheGtkObject, aNode);
end;

procedure TFPgtkCTree.ExpandToDepth (aNode:PGtkCTreeNode; aDepth:integer);
begin
  gtk_ctree_expand_to_depth (TheGtkObject, aNode, aDepth);
end;

procedure TFPgtkCTree.Collapse (aNode:PGtkCTreeNode);
begin
  gtk_ctree_collapse (TheGtkObject, aNode);
end;

procedure TFPgtkCTree.CollapseRecursive (aNode:PGtkCTreeNode);
begin
  gtk_ctree_collapse_recursive (TheGtkObject, aNode);
end;

procedure TFPgtkCTree.CollapseToDepth (aNode:PGtkCTreeNode; aDepth:integer);
begin
  gtk_ctree_collapse_to_depth (TheGtkObject, aNode, aDepth);
end;

procedure TFPgtkCTree.SelectNode (aNode:PGtkCTreeNode);
begin
  gtk_ctree_select (TheGtkObject, aNode);
end;

procedure TFPgtkCTree.SelectRecursive (aNode:PGtkCTreeNode);
begin
  gtk_ctree_select_recursive (TheGtkObject, aNode);
end;

procedure TFPgtkCTree.UnselectNode (aNode:PGtkCTreeNode);
begin
  gtk_ctree_unselect (TheGtkObject, aNode);
end;

procedure TFPgtkCTree.UnselectRecursive (aNode:PGtkCTreeNode);
begin
  gtk_ctree_unselect_recursive (TheGtkObject, aNode);
end;

procedure TFPgtkCTree.RealSelectRecursive (aNode:PGtkCTreeNode; aState:boolean);
begin
  gtk_ctree_real_select_recursive (TheGtkObject, aNode, ord(aState));
end;

function TFPgtkCTree.NodeGetCellType (Node:PGtkCTreeNode; column:integer) : TGtkCellType;
begin
  result := gtk_ctree_node_get_cell_type (TheGtkObject, Node, column);
end;

function TFPgtkCTree.GetNodeCellText (Node:PGtkCTreeNode; Column:integer) : string;
var s : pgchar;
    r : integer;
begin
  r := gtk_ctree_node_get_text (TheGtkObject, node, column, @s);
  if (r = 0) then
    result := ''
  else
    result := string(s^);
end;

procedure TFPgtkCTree.SetNodeCellText (Node:PGtkCTreeNode; Column:integer; TheValue:string);
begin
  gtk_ctree_node_set_text(TheGtkObject,Node, Column,ConvertToPgchar(TheValue));
end;

procedure TFPgtkCTree.NodeSetPixmap (Node:PGtkCTreeNode; column:integer; pixmap:PGdkPixmap; mask:PGdkBitmap);
begin
  gtk_ctree_node_set_pixmap (TheGtkObject, Node, column, pixmap, mask);
end;

procedure TFPgtkCTree.NodeGetPixmap (Node:PGtkCTreeNode; column:integer; var pixmap:PGdkPixmap; var mask:PGdkBitmap);
begin
  gtk_ctree_node_get_pixmap (TheGtkObject, node, column, @pixmap, @mask);
end;

procedure TFPgtkCTree.NodeSetPixText (Node:PGtkCTreeNode; column:integer; text:string; aspacing:guint8; pixmap:PGdkPixmap; mask:PGdkBitmap);
begin
  gtk_ctree_node_set_pixtext (TheGtkObject, Node, column, ConvertToPgchar(text), aspacing, pixmap, mask);
end;

procedure TFPgtkCTree.NodeGetPixText (Node:PGtkCTreeNode; column:integer; var text:string; var aspacing:guint8; var pixmap:PGdkPixmap; var mask:PGdkBitmap);
var r : integer;
    s : PPgchar;
begin
  s := nil;
  r := gtk_ctree_node_get_pixtext (TheGtkObject, node, column, s, @aspacing, @pixmap, @mask);
  if r = 0 then
    begin
    text := '';
    pixmap := nil;
    mask := nil;
    end
  else
    text := string (s^);
end;

procedure TFPgtkCTree.SetNodeInfo (aNode:PGtkCTreeNode; aText:string; aSpacing:guint8; PixmapClosed:PGdkPixmap; MaskClosed:PGdkBitmap; PixmapOpened:PGdkPixmap; MaskOpened:PGdkBitmap; IsLeaf:boolean; Expanded:boolean); Overload;
begin
  gtk_ctree_set_node_info (TheGtkObject, aNode, ConvertToPgchar(aText), aSpacing, PixmapClosed, MaskClosed, PixmapOpened, MaskOpened, IsLeaf, Expanded);
end;

procedure TFPgtkCTree.GetNodeInfo (aNode:PGtkCTreeNode; var aText:string; var aSpacing:guint8; var PixmapClosed:PGdkPixmap; var MaskClosed:PGdkBitmap; var PixmapOpened:PGdkPixmap; var MaskOpened:PGdkBitmap; var IsLeaf:boolean; var Expanded:boolean); Overload;
var r : integer;
    s : PPgchar;
begin
  s := nil;
  r := gtk_ctree_get_node_info (TheGtkObject, aNode, s,
      @aspacing, @pixmapClosed, @maskClosed, @pixmapOpened, @maskOpened,
      pgboolean(@IsLeaf), pgboolean(@expanded));
  if r = 0 then
    begin
    atext := '';
    Spacing := 0;
    pixmapClosed := nil;
    maskClosed := nil;
    pixmapOpened := nil;
    maskOpened := nil;
    IsLeaf := false;
    Expanded := false;
    end
  else
    atext := string (s^);
end;

procedure TFPgtkCTree.NodeSetShift (Node:PGtkCTreeNode; column:integer; vertical:integer; horizontal:integer);
begin
  gtk_ctree_node_set_shift (TheGtkObject, Node, column, vertical, horizontal);
end;

function TFPgtkCTree.GetNodeSelectable (Node:PGtkCTreeNode) : boolean;
begin
  result := gtk_ctree_node_get_selectable(TheGtkObject,Node);
end;

procedure TFPgtkCTree.SetNodeSelectable (Node:PGtkCTreeNode; TheValue:boolean);
begin
  gtk_ctree_node_set_selectable(TheGtkObject,Node,TheValue);
end;

procedure TFPgtkCTree.NodeSetForeground (Node:PGtkCTreeNode; color:PGdkColor);
begin
  gtk_ctree_node_set_foreground (TheGtkObject, Node, color);
end;

procedure TFPgtkCTree.NodeSetBackground (Node:PGtkCTreeNode; color:PGdkColor);
begin
  gtk_ctree_node_set_background (TheGtkObject, Node, color);
end;

function TFPgtkCTree.GetNodeCellStyle (Node:PGtkCTreeNode; column:integer) : PGtkStyle;
begin
  result := gtk_ctree_node_get_cell_style(TheGtkObject,Node, column);
end;

procedure TFPgtkCTree.SetNodeCellStyle (Node:PGtkCTreeNode; column:integer; TheValue:PGtkStyle);
begin
  gtk_ctree_node_set_cell_style(TheGtkObject,Node, column,TheValue);
end;

function TFPgtkCTree.GetNodeRowStyle (Node:PGtkCTreeNode) : PGtkStyle;
begin
  result := gtk_ctree_node_get_row_style(TheGtkObject,Node);
end;

procedure TFPgtkCTree.SetNodeRowStyle (Node:PGtkCTreeNode; TheValue:PGtkStyle);
begin
  gtk_ctree_node_set_row_style(TheGtkObject,Node,TheValue);
end;

function TFPgtkCTree.GetNodeData (Node:PGtkCTreeNode) : pointer;
begin
  result := gtk_ctree_node_get_row_data(TheGtkObject,Node);
end;

procedure TFPgtkCTree.SetNodeData (Node:PGtkCTreeNode; TheValue:pointer);
begin
  gtk_ctree_node_set_row_data(TheGtkObject,Node,TheValue);
end;

procedure TFPgtkCTree.NodeMoveTo (aNode:PGtkCTreeNode; column:integer; RowAlign:gfloat; ColAlign:gfloat);
begin
  gtk_ctree_node_moveto (TheGtkObject, aNode, column, RowAlign, ColAlign);
end;

function TFPgtkCTree.IsVisible (aNode:PGtkCTreeNode) : TGtkVisibility;
begin
  result := gtk_ctree_node_is_visible (TheGtkObject, aNode);
end;

function TFPgtkCTree.GetCompareDragFunc : TGtkCTreeCompareDragFunc;
begin
  result := TheGtkObject^.drag_compare;
end;

procedure TFPgtkCTree.SetCompareDragFunc (TheValue:TGtkCTreeCompareDragFunc);
begin
  gtk_ctree_set_drag_compare_func(TheGtkObject,TheValue);
end;

procedure TFPgtkCTree.SortNode (aNode:PGtkCTreeNode);
begin
  gtk_ctree_sort_node (TheGtkObject, aNode);
end;

procedure TFPgtkCTree.SortRecursive (aNode:PGtkCTreeNode);
begin
  gtk_ctree_sort_recursive (TheGtkObject, aNode);
end;

function TFPgtkCTree.NthNode (Row:integer) : PGtkCTreeNode;
begin
  result := gtk_ctree_node_Nth (TheGtkObject, Row);
end;

 { TFPgtkFixed }

function TFPgtkFixed.TheGtkObject : PGtkFixed;
begin
  result := PgtkFixed(FGtkObject);
end;

procedure TFPgtkFixed.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_fixed_new);
end;


procedure TFPgtkFixed.Put (Widget:TFPgtkWidget; x:integer; y:integer);
begin
  gtk_fixed_put (TheGtkObject, PGtkwidget(ConvertToGtkObject(Widget)), x, y);
end;

procedure TFPgtkFixed.Move (Widget:TFPgtkWidget; x:integer; y:integer);
begin
  gtk_fixed_move (TheGtkObject, PGtkwidget(ConvertToGtkObject(Widget)), x, y);
end;

procedure TFPgtkFixed.GetPos (Widget:TFPgtkWidget; var PosX:integer; var PosY:integer);
var g : TFPgtkGroup;
    r : integer;
begin
  g := TFPgtkGroup.Create;
  try
    g.ManageLists := false;
    g.gtkList := TheGtkObject^.children;
    r := g.indexof (Widget);
    if r < 0 then
      begin
      PosX := -1;
      PosY := -1;
      end
    else
      with PGtkFixedChild(g.Items[r])^ do
        begin
        PosX := x;
        PosY := Y;
        end;
  finally
    g.Free;
  end;
end;

 { TFPgtkNotebook }

function TFPgtkNotebook.TheGtkObject : PGtkNotebook;
begin
  result := PgtkNotebook(FGtkObject);
end;

procedure TFPgtkNotebook.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_notebook_new);
end;


procedure TFPgtkNotebook.AppendPage (Child:TFPgtkWidget; TabLabel:TFPgtkWidget);
begin
  gtk_notebook_append_page (TheGtkObject, ConvertTogtkWidget(Child), ConvertTogtkWidget(TabLabel));
  Child.Show;
end;

procedure TFPgtkNotebook.AppendPageFull (Child:TFPgtkWidget; TabLabel:TFPgtkWidget; MenuLabel:TFPgtkWidget; IsVisible:boolean);
begin
  if assigned (MenuLabel) then
    gtk_notebook_append_page_menu (TheGtkObject, ConvertTogtkWidget(Child), ConvertTogtkWidget(TabLabel), ConvertTogtkWidget(MenuLabel))
  else
    gtk_notebook_append_page (TheGtkObject, ConvertTogtkWidget(Child), ConvertTogtkWidget(TabLabel));
  if isvisible then
    Child.Show;
end;

procedure TFPgtkNotebook.PrependPage (Child:TFPgtkWidget; TabLabel:TFPgtkWidget);
begin
  gtk_notebook_Prepend_page (TheGtkObject, PGtkwidget(ConvertToGtkObject(Child)), PGtkwidget(ConvertToGtkObject(TabLabel)));
end;

procedure TFPgtkNotebook.PrependPageFull (Child:TFPgtkWidget; TabLabel:TFPgtkWidget; MenuLabel:TFPgtkWidget; IsVisible:boolean);
begin
  if assigned (MenuLabel) then
    gtk_notebook_prepend_page_menu (TheGtkObject, ConvertTogtkWidget(Child), ConvertTogtkWidget(TabLabel), ConvertTogtkWidget(MenuLabel))
  else
    gtk_notebook_prepend_page (TheGtkObject, ConvertTogtkWidget(Child), ConvertTogtkWidget(TabLabel));
  if isvisible then
    Child.Show;
end;

procedure TFPgtkNotebook.InsertPage (Child:TFPgtkWidget; TabLabel:TFPgtkWidget; Position:integer);
begin
  gtk_notebook_insert_page (TheGtkObject, PGtkwidget(ConvertToGtkObject(Child)), PGtkwidget(ConvertToGtkObject(TabLabel)), Position);
end;

procedure TFPgtkNotebook.InsertPageFull (Child:TFPgtkWidget; TabLabel:TFPgtkWidget; MenuLabel:TFPgtkWidget; IsVisible:boolean; Position:integer);
begin
  if assigned (MenuLabel) then
    gtk_notebook_insert_page_menu (TheGtkObject, ConvertTogtkWidget(Child), ConvertTogtkWidget(TabLabel), ConvertTogtkWidget(MenuLabel), position)
  else
    gtk_notebook_insert_page (TheGtkObject, ConvertTogtkWidget(Child), ConvertTogtkWidget(TabLabel), position);
  if isvisible then
    Child.Show;
end;

procedure TFPgtkNotebook.RemovePage (PageNumber:integer);
begin
  gtk_notebook_remove_page (TheGtkObject, PageNumber);
end;

function TFPgtkNotebook.PageNumberOf (Child:TFPgtkWidget) : integer;
begin
  result := gtk_notebook_page_num (TheGtkObject, PGtkwidget(ConvertToGtkObject(Child)));
end;

procedure TFPgtkNotebook.NextPage;
begin
  gtk_notebook_next_page (TheGtkObject);
end;

procedure TFPgtkNotebook.PrevPage;
begin
  gtk_notebook_prev_page (TheGtkObject);
end;

procedure TFPgtkNotebook.ReorderPage (Child:TFPgtkWidget; PageNum:integer);
begin
  gtk_notebook_reorder_child (TheGtkObject, PGtkwidget(ConvertToGtkObject(Child)), PageNum);
end;

function TFPgtkNotebook.GetPageIndex : integer;
begin
  result := gtk_notebook_get_current_page(TheGtkObject);
end;

procedure TFPgtkNotebook.SetPageIndex (TheValue:integer);
begin
  gtk_notebook_set_page(TheGtkObject,TheValue);
end;

function TFPgtkNotebook.GetPage : TFPgtkWidget;
begin
  result := GetChildOnPage (PageIndex);
end;

procedure TFPgtkNotebook.SetPage (TheValue:TFPgtkWidget);
var r : integer;
begin
  r := PageNumberOf (TheValue);
  if r > -1 then
    PageIndex := r;
end;

function TFPgtkNotebook.GetTabPos : TGtkPositionType;
begin
  result := gtk.tab_pos(TheGtkObject^);
end;

procedure TFPgtkNotebook.SetTabPos (TheValue:TGtkPositionType);
begin
  gtk_notebook_set_tab_pos(TheGtkObject,TheValue);
end;

function TFPgtkNotebook.GetShowTabs : boolean;
begin
  result := boolean(gtk.show_tabs(TheGtkObject^));
end;

procedure TFPgtkNotebook.SetShowTabs (TheValue:boolean);
begin
  gtk_notebook_set_show_tabs(TheGtkObject,TheValue);
end;

function TFPgtkNotebook.GetShowBorder : boolean;
begin
  result := boolean(gtk.show_border(TheGtkObject^));
end;

procedure TFPgtkNotebook.SetShowBorder (TheValue:boolean);
begin
  gtk_notebook_set_show_border(TheGtkObject,TheValue);
end;

function TFPgtkNotebook.GetScrollable : boolean;
begin
  result := boolean(gtk.scrollable(TheGtkObject^));
end;

procedure TFPgtkNotebook.SetScrollable (TheValue:boolean);
begin
  gtk_notebook_set_scrollable(TheGtkObject,TheValue);
end;

function TFPgtkNotebook.GetHomogenous : boolean;
begin
  result := boolean(gtk.homogeneous(TheGtkObject^));
end;

procedure TFPgtkNotebook.SetHomogenous (TheValue:boolean);
begin
  gtk_notebook_set_homogeneous_tabs(TheGtkObject,TheValue);
end;

function TFPgtkNotebook.GetTabHBorder : word;
begin
  result := TheGtkObject^.tab_hborder;
end;

procedure TFPgtkNotebook.SetTabHBorder (TheValue:word);
begin
  gtk_notebook_set_tab_hborder(TheGtkObject,TheValue);
end;

function TFPgtkNotebook.GetTabVBorder : word;
begin
  result := TheGtkObject^.tab_vborder;
end;

procedure TFPgtkNotebook.SetTabVBorder (TheValue:word);
begin
  gtk_notebook_set_tab_vborder(TheGtkObject,TheValue);
end;

procedure TFPgtkNotebook.SetTabBorders (BorderWidth:word);
begin
  gtk_notebook_set_tab_border (TheGtkObject, BorderWidth);
end;

function TFPgtkNotebook.GetMenuLabelOf (Child:TFPgtkWidget) : TFPgtkWidget;
begin
  result := GetPascalInstance (PGtkObject(gtk_notebook_get_menu_label (TheGtkObject, ConvertTogtkWidget(Child)))) as TFPgtkWidget;
end;

procedure TFPgtkNotebook.SetMenuLabel (Child:TFPgtkWidget; MenuLabel:TFPgtkWidget);
begin
  gtk_notebook_set_menu_label (TheGtkObject, PGtkwidget(ConvertToGtkObject(Child)), PGtkwidget(ConvertToGtkObject(MenuLabel)));
end;

function TFPgtkNotebook.GetTabLabelOf (Child:TFPgtkWidget) : TFPgtkWidget;
begin
  result := GetPascalInstance (PGtkObject(gtk_notebook_get_tab_label (TheGtkObject, ConvertTogtkWidget(Child)))) as TFPgtkWidget;
end;

procedure TFPgtkNotebook.SetTabLabel (Child:TFPgtkWidget; TabLabel:TFPgtkWidget);
begin
  gtk_notebook_set_tab_label (TheGtkObject, PGtkwidget(ConvertToGtkObject(Child)), PGtkwidget(ConvertToGtkObject(TabLabel)));
end;

function TFPgtkNotebook.GetChildOnPage (PageNum:integer) : TFPgtkWidget;
begin
  result := GetPascalInstance (PGtkObject(gtk_notebook_get_nth_page (TheGtkObject, PageNum))) as TFPgtkWidget;
end;

procedure TFPgtkNotebook.GetTabLabelPacking (Widget:TFPgtkWidget; var Expand:boolean; var Fill:boolean; var PackType:TGtkPackType);
var PT : PGtkPackType;
begin
  pt := @PackType;
  gtk_notebook_query_tab_label_packing (TheGtkObject, ConvertTogtkWidget(widget),
                               pgboolean(@expand), pgboolean(@fill), pt);
end;

procedure TFPgtkNotebook.SetTabLabelPacking (Widget:TFPgtkWidget; Expand:boolean; Fill:boolean; PackType:TGtkPackType);
begin
  gtk_notebook_set_tab_label_packing (TheGtkObject, PGtkwidget(ConvertToGtkObject(Widget)), Expand, Fill, PackType);
end;

procedure TFPgtkNotebook.EnablePopup;
begin
  gtk_notebook_popup_enable (TheGtkObject);
end;

procedure TFPgtkNotebook.DisablePopup;
begin
  gtk_notebook_popup_disable (TheGtkObject);
end;

procedure PageSwitchSignalproc (Sender:PGtkobject; PageRec:PGtkNotebookPage; aPageNum:integer; data:pointer); cdecl;
var p : TFPgtkPageSwitchSignalFunction;
begin
with PSignalData(data)^ do
  begin
  p := TFPgtkPageSwitchSignalFunction (TheSignalProc);
  p (TheWidget as TFPgtkObject, PageRec, aPageNum, TheData)
  end;
end;

function TFPgtkNotebook.PageSwitchSignalConnect (signal:string; proc:TFPgtkPageSwitchSignalFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect (FGtkObject, pgChar(signal), gtk_signal_func(@PageSwitchSignalproc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

function TFPgtkNotebook.PageSwitchSignalConnectAfter (signal:string; proc:TFPgtkPageSwitchSignalFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect_After (FGtkObject, pgChar(signal), gtk_signal_func(@PageSwitchSignalproc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

function TFPgtkNotebook.ConnectSwitchPage (proc:TFPgtkPageSwitchSignalFunction; data:pointer) : guint;
begin
  result := PageSwitchSignalConnect (sgSwitchPage, proc, data);
end;

function TFPgtkNotebook.ConnectAfterSwitchPage (proc:TFPgtkPageSwitchSignalFunction; data:pointer) : guint;
begin
  result := PageSwitchSignalConnectAfter (sgSwitchPage, proc, data);
end;

 { TFPgtkFontSelection }

function TFPgtkFontSelection.TheGtkObject : PGtkFontSelection;
begin
  result := PgtkFontSelection(FGtkObject);
end;

procedure TFPgtkFontSelection.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_font_selection_new);
end;


function TFPgtkFontSelection.GetFontName : string;
begin
  result := gtk_font_selection_get_font_name(TheGtkObject);
end;

procedure TFPgtkFontSelection.SetFontName (TheValue:string);
begin
  if not gtk_font_selection_set_font_name (TheGtkObject, pgchar(TheValue)) then
    raise exception.CreateFmt (sFontNotFound, [TheValue]);
end;

function TFPgtkFontSelection.GetFont : PGdkFont;
begin
  result := gtk_font_selection_get_font (TheGtkObject);
end;

function TFPgtkFontSelection.GetPreviewText : string;
begin
  result := gtk_font_selection_get_preview_text(TheGtkObject);
end;

procedure TFPgtkFontSelection.SetPreviewText (TheValue:string);
begin
  gtk_font_selection_set_preview_text(TheGtkObject,ConvertToPgchar(TheValue));
end;

procedure TFPgtkFontSelection.SetFilter (FilterType:TGtkFontFilterType; FontType:TGtkFontType; Foundries:array of string; Weights:array of string; Slants:array of string; SetWidths:array of string; Spacings:array of string; CharSets:array of string);
var ppF, ppW, ppSl, ppSW, ppSp, ppC : ppgchar;

  function MakePP (data : array of string) : ppgchar;
  begin
    if high(data) > low(data) then
      result := ArrayToPPgchar(data)
    else
      result := nil;
  end;

  procedure FreePP (ppdata : ppgchar; data : array of string);
  begin
    if assigned (ppdata) then
      freemem (ppdata, sizeof (pgchar) * (high(data)-low(data)+1));
  end;

begin
  ppF := MakePP(Foundries);
  ppW := MakePP(Weights);
  ppSl := MakePP(Slants);
  ppSW := MakePP(SetWidths);
  ppSp := MakePP(Spacings);
  ppC := MakePP(CharSets);
  gtk_font_selection_set_filter (TheGtkObject, FilterType, FontType, ppF, ppW, ppSl, ppSW, ppSp, ppC);
  FreePP (ppF, Foundries);
  FreePP (ppW, Weights);
  FreePP (ppSl, Slants);
  FreePP (ppSW, SetWidths);
  FreePP (ppSp, Spacings);
  FreePP (ppC, CharSets);
end;

 { TFPgtkPaned }

function TFPgtkPaned.TheGtkObject : PGtkPaned;
begin
  result := PgtkPaned(FGtkObject);
end;


function TFPgtkPaned.GetGutterSize : word;
begin
  result := TheGtkObject^.gutter_size;
end;

procedure TFPgtkPaned.SetGutterSize (TheValue:word);
begin
  {$ifdef gtkwin}
  TheGtkObject^.gutter_size := TheValue;
  {$else}
  gtk_paned_set_gutter_size(TheGtkObject,TheValue);
  {$endif}
end;

function TFPgtkPaned.GetHandleSize : word;
begin
  result := TheGtkObject^.handle_size;
end;

procedure TFPgtkPaned.SetHandleSize (TheValue:word);
begin
  gtk_paned_set_handle_size(TheGtkObject,TheValue);
end;

function TFPgtkPaned.GetPosition : integer;
begin
  result := TheGtkObject^.child1_size;
end;

procedure TFPgtkPaned.SetPosition (TheValue:integer);
begin
  gtk_paned_set_position(TheGtkObject,TheValue);
end;

procedure TFPgtkPaned.ComputePosition (AnAllocation:integer; Child1Req:integer; Child2Req:integer);
begin
  gtk_paned_compute_position (TheGtkObject, AnAllocation, Child1Req, Child2Req);
end;

procedure TFPgtkPaned.Add1 (Child:TFPgtkWidget); Overload;
begin
  gtk_paned_add1 (TheGtkObject, ConvertToGtkWidget(Child));
  Child.Show;
end;

procedure TFPgtkPaned.Pack1 (Child:TFPgtkWidget; Resize:boolean; Shrink:boolean); Overload;
begin
  gtk_paned_pack1 (TheGtkObject, ConvertToGtkWidget(Child), Resize, Shrink);
  Child.Show;
end;

procedure TFPgtkPaned.Add1 (Child:TFPgtkWidget; isVisible:boolean); Overload;
begin
  gtk_paned_add1 (TheGtkObject, ConvertToGtkWidget(Child));
  if isvisible then
    Child.Show;
end;

procedure TFPgtkPaned.Pack1 (Child:TFPgtkWidget; Resize:boolean; Shrink:boolean; IsVisible:boolean); Overload;
begin
  gtk_paned_pack1 (TheGtkObject, ConvertToGtkWidget(Child), Resize, Shrink);
  if isvisible then
    Child.Show;
end;

procedure TFPgtkPaned.Add2 (Child:TFPgtkWidget); Overload;
begin
  gtk_paned_add2 (TheGtkObject, ConvertToGtkWidget(Child));
  Child.Show;
end;

procedure TFPgtkPaned.Pack2 (Child:TFPgtkWidget; Resize:boolean; Shrink:boolean); Overload;
begin
  gtk_paned_pack2 (TheGtkObject, ConvertToGtkWidget(Child), Resize, Shrink);
  Child.Show;
end;

procedure TFPgtkPaned.Add2 (Child:TFPgtkWidget; IsVisible:boolean); Overload;
begin
  gtk_paned_add2 (TheGtkObject, ConvertToGtkWidget(Child));
  if isvisible then
    Child.Show;
end;

procedure TFPgtkPaned.Pack2 (Child:TFPgtkWidget; Resize:boolean; Shrink:boolean; IsVisible:boolean); Overload;
begin
  gtk_paned_pack2 (TheGtkObject, ConvertToGtkWidget(Child), Resize, Shrink);
  if isvisible then
    Child.Show;
end;

 { TFPgtkHPaned }

function TFPgtkHPaned.TheGtkObject : PGtkHPaned;
begin
  result := PgtkHPaned(FGtkObject);
end;

procedure TFPgtkHPaned.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_hpaned_new);
end;


 { TFPgtkVPaned }

function TFPgtkVPaned.TheGtkObject : PGtkVPaned;
begin
  result := PgtkVPaned(FGtkObject);
end;

procedure TFPgtkVPaned.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_vpaned_new);
end;


 { TFPgtkLayout }

function TFPgtkLayout.TheGtkObject : PGtkLayout;
begin
  result := PgtkLayout(FGtkObject);
end;

procedure TFPgtkLayout.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_layout_new (nil,nil));
end;


function TFPgtkLayout.GetHAdj : TFPgtkAdjustment;
begin
  result := GetPascalInstance(PGtkObject(gtk_layout_get_hadjustment(TheGtkObject)),tfpgtkadjustment) as tfpgtkadjustment;
end;

procedure TFPgtkLayout.SetHAdj (TheValue:TFPgtkAdjustment);
begin
  gtk_layout_set_hadjustment(TheGtkObject,PGtkadjustment(ConvertToGtkObject(TheValue)));
end;

function TFPgtkLayout.GetVAdj : TFPgtkAdjustment;
begin
  result := GetPascalInstance(PGtkObject(gtk_layout_get_vadjustment(TheGtkObject)),tfpgtkadjustment) as tfpgtkadjustment;
end;

procedure TFPgtkLayout.SetVAdj (TheValue:TFPgtkAdjustment);
begin
  gtk_layout_set_vadjustment(TheGtkObject,PGtkadjustment(ConvertToGtkObject(TheValue)));
end;

procedure TFPgtkLayout.Freeze;
begin
  gtk_layout_freeze (TheGtkObject);
end;

procedure TFPgtkLayout.Thaw;
begin
  gtk_layout_thaw (TheGtkObject);
end;

procedure TFPgtkLayout.Put (aWidget:TFPgtkWidget; X:integer; Y:integer); Overload;
begin
  gtk_layout_put (TheGtkObject, PGtkwidget(ConvertToGtkObject(aWidget)), X, Y);
  aWidget.Show;
end;

procedure TFPgtkLayout.Put (aWidget:TFPgtkWidget; X:integer; Y:integer; aVisible:boolean); Overload;
begin
  gtk_layout_put (TheGtkObject, PGtkwidget(ConvertToGtkObject(aWidget)), X, Y);
  if aVisible then
    aWidget.Show;
end;

procedure TFPgtkLayout.Move (aWidget:TFPgtkWidget; X:integer; Y:integer);
begin
  gtk_layout_move (TheGtkObject, PGtkwidget(ConvertToGtkObject(aWidget)), X, Y);
end;

procedure TFPgtkLayout.SetSize (aWidth:integer; aHeight:integer);
begin
  gtk_layout_set_size (TheGtkObject, aWidth, aHeight);
end;

 { TFPgtkList }

function TFPgtkList.TheGtkObject : PGtkList;
begin
  result := PgtkList(FGtkObject);
end;

procedure TFPgtkList.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_list_new);
end;


function TFPgtkList.ConnectSelectionChanged (proc:TFPgtksignalFunction; data:pointer) : guint;
begin
  result := signalConnect (sgSelectionChanged, proc, data);
end;

function TFPgtkList.ConnectAfterSelectionChanged (proc:TFPgtksignalFunction; data:pointer) : guint;
begin
  result := signalConnectAfter (sgSelectionChanged, proc, data);
end;

function TFPgtkList.ConnectSelectChild (proc:TFPgtkWidgetSignalFunction; data:pointer) : guint;
begin
  result := WidgetSignalConnect (sgSelectChild, proc, data);
end;

function TFPgtkList.ConnectAfterSelectChild (proc:TFPgtkWidgetSignalFunction; data:pointer) : guint;
begin
  result := WidgetSignalConnectAfter (sgSelectChild, proc, data);
end;

function TFPgtkList.ConnectUnselectChild (proc:TFPgtkWidgetSignalFunction; data:pointer) : guint;
begin
  result := WidgetSignalConnect (sgUnselectChild, proc, data);
end;

function TFPgtkList.ConnectAfterUnselectChild (proc:TFPgtkWidgetSignalFunction; data:pointer) : guint;
begin
  result := WidgetSignalConnectAfter (sgUnselectChild, proc, data);
end;

function TFPgtkList.GetSelectionMode : TGtkSelectionMode;
begin
  result := TGtkSelectionMode(Selection_mode(TheGtkObject^));
end;

procedure TFPgtkList.SetSelectionMode (TheValue:TGtkSelectionMode);
begin
  gtk_list_set_selection_mode(TheGtkObject,TheValue);
end;

procedure TFPgtkList.InsertItems (TheItems:TFPgtkListItemGroup; position:integer);
begin
  gtk_list_insert_items (TheGtkObject, TheItems.GtkList, position);
end;

procedure TFPgtkList.AppendItems (TheItems:TFPgtkListItemGroup);
begin
  gtk_list_append_items (TheGtkObject, TheItems.GtkList);
end;

procedure TFPgtkList.PrependItems (TheItems:TFPgtkListItemGroup);
begin
  gtk_list_prepend_items (TheGtkObject, TheItems.GtkList);
end;

procedure TFPgtkList.RemoveItems (TheItems:TFPgtkListItemGroup);
begin
  gtk_list_remove_items (TheGtkObject, TheItems.GtkList);
end;

procedure TFPgtkList.RemoveItemsNoUnref (TheItems:TFPgtkListItemGroup);
begin
  gtk_list_remove_items_no_unref (TheGtkObject, TheItems.GtkList);
end;

procedure TFPgtkList.ClearItems (FromItem:integer; ToItem:integer);
begin
  if ToItem >= 0 then
    inc (ToItem);
  gtk_list_clear_items (TheGtkObject, FromItem, ToItem);
end;

procedure TFPgtkList.ClearAll;
begin
  ClearItems (0,-1);
end;

procedure TFPgtkList.SelectItem (Item:integer);
begin
  gtk_list_select_item (TheGtkObject, Item);
end;

procedure TFPgtkList.UnselectItem (Item:integer);
begin
  gtk_list_unselect_item (TheGtkObject, Item);
end;

procedure TFPgtkList.SelectChild (Child:TFPgtkWidget);
begin
  gtk_list_select_child (TheGtkObject, PGtkwidget(ConvertToGtkObject(Child)));
end;

procedure TFPgtkList.UnselectChild (Child:TFPgtkWidget);
begin
  gtk_list_unselect_child (TheGtkObject, PGtkwidget(ConvertToGtkObject(Child)));
end;

function TFPgtkList.ChildPosition (Child:TFPgtkWidget) : integer;
begin
  result := gtk_list_child_position (TheGtkObject, PGtkwidget(ConvertToGtkObject(Child)));
end;

procedure TFPgtkList.ExtendSelection (ScrollType:TGtkScrollType; Position:gfloat; AutoStartSelection:boolean);
begin
  gtk_list_extend_selection (TheGtkObject, ScrollType, Position, AutoStartSelection);
end;

procedure TFPgtkList.StartSelection;
begin
  gtk_list_start_selection (TheGtkObject);
end;

procedure TFPgtkList.EndSelection;
begin
  gtk_list_end_selection (TheGtkObject);
end;

procedure TFPgtkList.SelectAll;
begin
  gtk_list_select_all (TheGtkObject);
end;

procedure TFPgtkList.UnselectAll;
begin
  gtk_list_unselect_all (TheGtkObject);
end;

procedure TFPgtkList.ScrollHorizontal (ScrollType:TGtkScrollType; Position:gfloat);
begin
  gtk_list_scroll_horizontal (TheGtkObject, ScrollType, Position);
end;

procedure TFPgtkList.ScrollVertical (ScrollType:TGtkScrollType; Position:gfloat);
begin
  gtk_list_scroll_vertical (TheGtkObject, ScrollType, Position);
end;

procedure TFPgtkList.ToggleAddMode;
begin
  gtk_list_toggle_add_mode (TheGtkObject);
end;

procedure TFPgtkList.ToggleFocusRow;
begin
  gtk_list_toggle_focus_row (TheGtkObject);
end;

procedure TFPgtkList.ToggleRow (Child:TFPgtkWidget);
begin
  gtk_list_toggle_row (TheGtkObject, PGtkwidget(ConvertToGtkObject(Child)));
end;

procedure TFPgtkList.UndoSelection;
begin
  gtk_list_undo_selection (TheGtkObject);
end;

procedure TFPgtkList.EndDragSelection;
begin
  gtk_list_end_drag_selection (TheGtkObject);
end;

procedure TFPgtkList.GetSelection (aGroup:TFPgtkGroup);
begin
  with aGroup do
    begin
    ManageLists := False;
    GtkList := TheGtkObject^.Selection;
    end;
end;

 { TFPgtkMenuShell }

function TFPgtkMenuShell.TheGtkObject : PGtkMenuShell;
begin
  result := PgtkMenuShell(FGtkObject);
end;


procedure MoveCurrentSignalproc (Sender:PGtkobject; dir:TGtkMenuDirectionType; data:pointer); cdecl;
var p : TFPgtkMoveCurrentSignalFunction;
begin
with PSignalData(data)^ do
  begin
  p := TFPgtkMoveCurrentSignalFunction (TheSignalProc);
  p (TheWidget as TFPgtkObject, dir, TheData)
  end;
end;

function TFPgtkMenuShell.MoveCurrentSignalConnect (signal:string; proc:TFPgtkMoveCurrentSignalFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect (FGtkObject, pgChar(signal), gtk_signal_func(@MoveCurrentSignalproc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

function TFPgtkMenuShell.MoveCurrentSignalConnectAfter (signal:string; proc:TFPgtkMoveCurrentSignalFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect_After (FGtkObject, pgChar(signal), gtk_signal_func(@MoveCurrentSignalproc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

procedure TFPgtkMenuShell.GtkPrepend (MenuItem:TFPgtkWidget);
begin
  gtk_menu_shell_prepend (TheGtkObject, PGtkwidget(ConvertToGtkObject(MenuItem)));
end;

procedure TFPgtkMenuShell.GtkInsert (MenuItem:TFPgtkWidget; position:integer);
begin
  gtk_menu_shell_insert (TheGtkObject, PGtkwidget(ConvertToGtkObject(MenuItem)), position);
end;

procedure TFPgtkMenuShell.GtkAppend (MenuItem:TFPgtkWidget);
begin
  gtk_menu_shell_append (TheGtkObject, PGtkwidget(ConvertToGtkObject(MenuItem)));
end;

procedure TFPgtkMenuShell.ActivateItem (MenuItem:TFPgtkWidget; ForceDeactivate:boolean);
begin
  gtk_menu_shell_activate_item (TheGtkObject, PGtkwidget(ConvertToGtkObject(MenuItem)), ForceDeactivate);
end;

procedure TFPgtkMenuShell.SelectItem (MenuItem:TFPgtkWidget);
begin
  gtk_menu_shell_select_item (TheGtkObject, PGtkwidget(ConvertToGtkObject(MenuItem)));
end;

procedure TFPgtkMenuShell.DeActivate;
begin
  gtk_menu_shell_deactivate (TheGtkObject);
end;

procedure TFPgtkMenuShell.Prepend (MenuItem:TFPgtkWidget); Overload;
begin
  GtkPrepend (MenuItem);
  MenuItem.Show;
end;

procedure TFPgtkMenuShell.Prepend (MenuItem:TFPgtkWidget; CreateVisible:boolean); Overload;
begin
  GtkPrepend (MenuItem);
  if createvisible then
    MenuItem.Show;
end;

procedure TFPgtkMenuShell.Insert (MenuItem:TFPgtkWidget; position:integer); Overload;
begin
  GtkInsert (MenuItem, position);
  MenuItem.Show;
end;

procedure TFPgtkMenuShell.Insert (MenuItem:TFPgtkWidget; position:integer; CreateVisible:boolean); Overload;
begin
  GtkInsert (MenuItem, position);
  if createvisible then
    MenuItem.Show;
end;

procedure TFPgtkMenuShell.Append (MenuItem:TFPgtkWidget); Overload;
begin
  GtkAppend (MenuItem);
  MenuItem.Show;
end;

procedure TFPgtkMenuShell.Append (MenuItem:TFPgtkWidget; CreateVisible:boolean); Overload;
begin
  GtkAppend (MenuItem);
  if createvisible then
    MenuItem.Show;
end;

function TFPgtkMenuShell.ConnectDeActivate (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnect (sgDeActivate, proc, data);
end;

function TFPgtkMenuShell.ConnectAfterDeActivate (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnectAfter (sgDeActivate, proc, data);
end;

function TFPgtkMenuShell.ConnectSelectionDone (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnect (sgSelectionDone, proc, data);
end;

function TFPgtkMenuShell.ConnectAfterSelectionDone (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnectAfter (sgSelectionDone, proc, data);
end;

function TFPgtkMenuShell.ConnectCancel (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnect (sgCancel, proc, data);
end;

function TFPgtkMenuShell.ConnectAfterCancel (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnectAfter (sgCancel, proc, data);
end;

function TFPgtkMenuShell.ConnectMoveCurrent (proc:TFPgtkMoveCurrentSignalFunction; data:pointer) : guint;
begin
  result := MoveCurrentSignalConnect (sgMoveCurrent, proc, data);
end;

function TFPgtkMenuShell.ConnectAfterMoveCurrent (proc:TFPgtkMoveCurrentSignalFunction; data:pointer) : guint;
begin
  result := MoveCurrentSignalConnectAfter (sgMoveCurrent, proc, data);
end;

function TFPgtkMenuShell.ConnectActivateCurrent (proc:TFPgtkBooleanSignalFunction; data:pointer) : guint;
begin
  result := BooleanSignalConnect (sgActivateCurrent, proc, data);
end;

function TFPgtkMenuShell.ConnectAfterActivateCurrent (proc:TFPgtkBooleanSignalFunction; data:pointer) : guint;
begin
  result := BooleanSignalConnectAfter (sgActivateCurrent, proc, data);
end;

 { TFPgtkMenuBar }

function TFPgtkMenuBar.TheGtkObject : PGtkMenuBar;
begin
  result := PgtkMenuBar(FGtkObject);
end;

procedure TFPgtkMenuBar.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_menu_bar_new);
end;


procedure TFPgtkMenuBar.GtkPrepend (MenuItem:TFPgtkWidget);
begin
  gtk_menu_bar_prepend (TheGtkObject, PGtkwidget(ConvertToGtkObject(MenuItem)));
end;

procedure TFPgtkMenuBar.GtkInsert (MenuItem:TFPgtkWidget; position:integer);
begin
  gtk_menu_bar_insert (TheGtkObject, PGtkwidget(ConvertToGtkObject(MenuItem)), position);
end;

procedure TFPgtkMenuBar.GtkAppend (MenuItem:TFPgtkWidget);
begin
  gtk_menu_bar_append (TheGtkObject, PGtkwidget(ConvertToGtkObject(MenuItem)));
end;

function TFPgtkMenuBar.GetShadow : TgtkShadowType;
begin
  result := TheGtkObject^.shadow_type;
end;

procedure TFPgtkMenuBar.SetShadow (TheValue:TgtkShadowType);
begin
  gtk_menu_bar_set_shadow_type(TheGtkObject,TheValue);
end;

 { TFPgtkMenu }

function TFPgtkMenu.TheGtkObject : PGtkMenu;
begin
  result := PgtkMenu(FGtkObject);
end;

procedure TFPgtkMenu.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_menu_new);
end;


procedure TFPgtkMenu.GtkPrepend (MenuItem:TFPgtkWidget);
begin
  gtk_menu_prepend (TheGtkObject, PGtkwidget(ConvertToGtkObject(MenuItem)));
end;

procedure TFPgtkMenu.GtkInsert (MenuItem:TFPgtkWidget; position:integer);
begin
  gtk_menu_insert (TheGtkObject, PGtkwidget(ConvertToGtkObject(MenuItem)), position);
end;

procedure TFPgtkMenu.GtkAppend (MenuItem:TFPgtkWidget);
begin
  gtk_menu_append (TheGtkObject, PGtkwidget(ConvertToGtkObject(MenuItem)));
end;

procedure FPgtkMenuPos (Sender:PgtkMenu; x:pgint; y:pgint; data:pointer); Cdecl;
var p : TFPgtkMenuPosFunction;
begin
  with PSignalData (data)^ do
    begin
    p := TFPgtkMenuPosFunction (TheSignalProc);
    p(TFPgtkMenu(GetPascalInstance(PgtkObject(Sender))), x^, y^, data);
    end;
end;


procedure FPgtkMenuDetacher (AttachedWidget:PgtkWidget; TheMenu:PgtkMenu); Cdecl;
var m : TFPgtkMenu;
    a : TFPgtkWidget;
begin
  m := (GetPascalInstance(PgtkObject(TheMenu)) as TFPgtkMenu);
  if assigned(m) and assigned(m.FDetacher) then
    begin
    a := TFPgtkWidget (GetPascalInstance(PgtkObject(AttachedWidget)));
    m.FDetacher (a, m);
    end
end;


procedure TFPgtkMenu.ReorderChild (MenuItem:TFPgtkWidget; position:integer);
begin
  gtk_menu_reorder_child (TheGtkObject, PGtkwidget(ConvertToGtkObject(MenuItem)), position);
end;

procedure TFPgtkMenu.Popup (button:guint); Overload;
begin
  gtk_menu_popup (TheGtkObject, null, null, null, null, button, 0);
end;

procedure TFPgtkMenu.Popup (ParentShell:TFPgtkWidget; ParentItem:TFPgtkWidget; func:TFPgtkMenuPosFunction; data:pointer; button:guint; ActivateTime:guint32); Overload;
begin
  gtk_menu_popup (TheGtkObject, ConvertTogtkWidget(ParentShell), ConvertTogtkWidget(ParentItem),
      @FPgtkMenuPos, ConvertSignalData(TFPgtkSignalFunction(func), data, true), button, ActivateTime);
end;

procedure TFPgtkMenu.PopDown;
begin
  gtk_menu_popdown (TheGtkObject);
end;

procedure TFPgtkMenu.Reposition;
begin
  gtk_menu_reposition (TheGtkObject);
end;

procedure TFPgtkMenu.AttachToWidget (Widget:TFPgtkWidget; detacher:TFPgtkMenuDetachFunction);
begin
  FDetacher := detacher;
  gtk_menu_attach_to_widget (TheGtkObject, ConvertTogtkWidget(Widget), @FPgtkMenuDetacher);
end;

procedure TFPgtkMenu.Detach;
begin
  gtk_menu_detach (TheGtkObject);
end;

procedure TFPgtkMenu.SetTitle (TheValue:string);
begin
  gtk_menu_set_title(TheGtkObject,Pgchar(TheValue));
end;

function TFPgtkMenu.GetActive : TFPgtkWidget;
begin
  result := GetPascalInstance(PGtkObject(gtk_menu_get_active(TheGtkObject)),tfpgtkwidget) as tfpgtkwidget;
end;

procedure TFPgtkMenu.SetActive (TheValue:TFPgtkWidget);
var r : integer;
begin
  r := Children.indexof (TheValue);
  if r >= 0 then
    SetActiveIndex (r);
end;

function TFPgtkMenu.GetActiveIndex : integer;
begin
  result := Children.indexof (GetActive);
end;

procedure TFPgtkMenu.SetActiveIndex (TheValue:integer);
begin
  gtk_menu_set_active(TheGtkObject,TheValue);
end;

function TFPgtkMenu.GetTearOffState : boolean;
begin
  result := boolean(gtk.torn_off(TheGtkObject^));
end;

procedure TFPgtkMenu.SetTearOffState (TheValue:boolean);
begin
  gtk_menu_set_tearoff_state(TheGtkObject,TheValue);
end;

function TFPgtkMenu.GetAttachedTo : TFPgtkWidget;
begin
  result := GetPascalInstance(PGtkObject(gtk_menu_get_attach_widget(TheGtkObject)),tfpgtkwidget) as tfpgtkwidget;
end;

procedure TFPgtkMenu.SetAttachedTo (TheValue:TFPgtkWidget);
begin
  AttachToWidget (TheValue, nil);
end;

function TFPgtkMenu.GetAccelGroup : PGtkAccelGroup;
begin
  result := gtk_menu_ensure_uline_accel_group(TheGtkObject);
end;

procedure TFPgtkMenu.SetAccelGroup (TheValue:PGtkAccelGroup);
begin
  gtk_menu_set_accel_group(TheGtkObject,TheValue);
end;

 { TFPgtkPacker }

function TFPgtkPacker.TheGtkObject : PGtkPacker;
begin
  result := PgtkPacker(FGtkObject);
end;

procedure TFPgtkPacker.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_packer_new);
end;


procedure TFPgtkPacker.Add (Child:TFPgtkWidget; Side:TGtkSideType; Anchor:TGtkAnchorType; options:TGtkPackerOptions); Overload;
begin
  gtk_packer_add_defaults (TheGtkObject, Child.TheGtkWidget, Side, anchor, options);
  Child.Show;
end;

procedure TFPgtkPacker.Add (Child:TFPgtkWidget; Side:TGtkSideType; Anchor:TGtkAnchorType; options:TGtkPackerOptions; aVisible:boolean); Overload;
begin
  gtk_packer_add_defaults (TheGtkObject, Child.TheGtkWidget, Side, anchor, options);
  if aVisible then
    Child.Show;
end;

procedure TFPgtkPacker.Add (Child:TFPgtkWidget; Side:TGtkSideType; Anchor:TGtkAnchorType; options:TGtkPackerOptions; aBorder:guint; PadX:Guint; PadY:guint; IPadX:guint; IPadY:guint); Overload;
begin
  gtk_packer_add (TheGtkObject, Child.TheGtkWidget, Side, anchor, options, aborder, padX, PadY, IPadX, IPadY);
  Child.Show;
end;

procedure TFPgtkPacker.Add (Child:TFPgtkWidget; Side:TGtkSideType; Anchor:TGtkAnchorType; options:TGtkPackerOptions; aBorder:guint; PadX:Guint; PadY:guint; IPadX:guint; IPadY:guint; aVisible:boolean); Overload;
begin
  gtk_packer_add (TheGtkObject, Child.TheGtkWidget, Side, anchor, options, aborder, padX, PadY, IPadX, IPadY);
  if aVisible then
    Child.Show;
end;

procedure TFPgtkPacker.ReorderChild (aChild:TFPgtkWidget; position:integer);
begin
  gtk_packer_reorder_child (TheGtkObject, PGtkwidget(ConvertToGtkObject(aChild)), position);
end;

function TFPgtkPacker.GetSpacing : guint;
begin
  result := TheGtkObject^.spacing;
end;

procedure TFPgtkPacker.SetSpacing (TheValue:guint);
begin
  gtk_packer_set_spacing(TheGtkObject,TheValue);
end;

procedure TFPgtkPacker.DefaultBorder (aBorder:guint);
begin
  gtk_packer_set_default_border_width (TheGtkObject, aBorder);
end;

procedure TFPgtkPacker.DefaultPad (PadX:guint; PadY:guint);
begin
  gtk_packer_set_default_pad (TheGtkObject, PadX, PadY);
end;

procedure TFPgtkPacker.DefaultIPad (IPadX:guint; IPadY:guint);
begin
  gtk_packer_set_default_ipad (TheGtkObject, IPadX, IPadY);
end;

procedure TFPgtkPacker.Configure (aChild:TFPgtkWidget; Side:TGtkSideType; Anchor:TGtkAnchorType; options:TGtkPackerOptions; aBorder:guint; PadX:Guint; PadY:guint; IPadX:guint; IPadY:guint); Overload;
begin
  gtk_packer_set_child_packing (TheGtkObject, PGtkwidget(ConvertToGtkObject(aChild)), Side, Anchor, options, aBorder, PadX, PadY, IPadX, IPadY);
end;

 { TFPgtkTable }

function TFPgtkTable.TheGtkObject : PGtkTable;
begin
  result := PgtkTable(FGtkObject);
end;

procedure TFPgtkTable.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_table_new (1,1,False));
end;


constructor TFPgtkTable.Create (AColumns:integer; ARows:integer);
begin
  inherited create;
  resize (AColumns, ARows);
end;


procedure TFPgtkTable.Resize (AColumns:integer; ARows:integer);
begin
  gtk_table_resize (TheGtkObject, ARows, AColumns);
end;

procedure TFPgtkTable.Attach (Widget:TFPgtkWidget; left:integer; right:integer; top:integer; bottom:integer; XOptions:integer; YOptions:integer; XPadding:integer; YPadding:integer; IsVisible:boolean);
begin
  gtk_table_attach (TheGtkObject, ConvertToGtkWidget(Widget), left, right, top, bottom,
                    XOptions, YOptions, XPadding, YPadding);
  if isvisible then
    widget.Show;
end;

procedure TFPgtkTable.Attach (Widget:TFPgtkWidget; left:integer; right:integer; top:integer; bottom:integer; XOptions:integer; YOptions:integer; XPadding:integer; YPadding:integer);
begin
  gtk_table_attach (TheGtkObject, ConvertTogtkWidget(Widget), left, right, top, bottom,
                    XOptions, YOptions, XPadding, YPadding);
  widget.Show;
end;

procedure TFPgtkTable.Attach (Widget:TFPgtkWidget; left:integer; right:integer; top:integer; bottom:integer; IsVisible:boolean);
begin
  gtk_table_attach_defaults (TheGtkObject, ConvertTogtkWidget(Widget), left, right, top, bottom);
  if isvisible then
    widget.Show;
end;

procedure TFPgtkTable.Attach (Widget:TFPgtkWidget; left:integer; right:integer; top:integer; bottom:integer);
begin
  gtk_table_attach_defaults (TheGtkObject, ConvertTogtkWidget(Widget), left, right, top, bottom);
  widget.Show;
end;

function TFPgtkTable.GetRowCount : integer;
begin
  result := TheGtkObject^.nrows;
end;

function TFPgtkTable.GetColCount : integer;
begin
  result := TheGtkObject^.ncols;
end;

function TFPgtkTable.GetHomogeneous : boolean;
begin
  result := boolean(gtk.homogeneous(TheGtkObject^));
end;

procedure TFPgtkTable.SetHomogeneous (TheValue:boolean);
begin
  gtk_table_set_homogeneous(TheGtkObject,TheValue);
end;

function TFPgtkTable.GetRowSpacings : integer;
begin
  result := TheGtkObject^.column_spacing;
end;

procedure TFPgtkTable.SetRowSpacings (TheValue:integer);
begin
  gtk_table_set_row_spacings(TheGtkObject,TheValue);
end;

function TFPgtkTable.GetColSpacings : integer;
begin
  result := TheGtkObject^.row_spacing;
end;

procedure TFPgtkTable.SetColSpacings (TheValue:integer);
begin
  gtk_table_set_col_spacings(TheGtkObject,TheValue);
end;

procedure TFPgtkTable.SetOneRowSpacing (row:integer; TheValue:integer);
begin
  gtk_table_set_row_spacing (TheGtkObject, row, TheValue);
end;

procedure TFPgtkTable.SetOneColSpacing (Column:integer; TheValue:integer);
begin
  gtk_table_set_col_spacing (TheGtkObject, Column, TheValue);
end;

 { TFPgtkToolbar }

function TFPgtkToolbar.TheGtkObject : PGtkToolbar;
begin
  result := PgtkToolbar(FGtkObject);
end;

procedure TFPgtkToolbar.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_toolbar_new (GTK_ORIENTATION_HORIZONTAL,GTK_TOOLBAR_BOTH));
end;


function TFPgtkToolbar.GetButtonRelief : TGtkReliefStyle;
begin
  result := gtk_toolbar_get_button_relief(TheGtkObject);
end;

procedure TFPgtkToolbar.SetButtonRelief (TheValue:TGtkReliefStyle);
begin
  gtk_toolbar_set_button_relief(TheGtkObject,TheValue);
end;

function TFPgtkToolbar.GetTooltips : TFPgtkTooltips;
begin
  result := GetPascalInstance(PGtkObject(TheGtkObject^.tooltips),tfpgtktooltips) as tfpgtktooltips;
end;

function TFPgtkToolbar.GetEnableTooltips : longbool;
begin
  result := tooltips.enabled;
end;

procedure TFPgtkToolbar.SetEnableTooltips (TheValue:longbool);
begin
  gtk_toolbar_set_tooltips(TheGtkObject,gint(TheValue));
end;

function TFPgtkToolbar.GetSpaceStyle : TGtkToolbarSpaceStyle;
begin
  result := TheGtkObject^.space_style;
end;

procedure TFPgtkToolbar.SetSpaceStyle (TheValue:TGtkToolbarSpaceStyle);
begin
  gtk_toolbar_set_space_style(TheGtkObject,TheValue);
end;

function TFPgtkToolbar.GetSpaceSize : integer;
begin
  result := TheGtkObject^.space_size;
end;

procedure TFPgtkToolbar.SetSpaceSize (TheValue:integer);
begin
  gtk_toolbar_set_space_size(TheGtkObject,TheValue);
end;

function TFPgtkToolbar.GetStyle : TGtkToolbarStyle;
begin
  result := TheGtkObject^.style;
end;

procedure TFPgtkToolbar.SetStyle (TheValue:TGtkToolbarStyle);
begin
  gtk_toolbar_set_style(TheGtkObject,TheValue);
end;

function TFPgtkToolbar.GetOrientation : tGtkOrientation;
begin
  result := TheGtkObject^.orientation;
end;

procedure TFPgtkToolbar.SetOrientation (TheValue:tGtkOrientation);
begin
  gtk_toolbar_set_orientation(TheGtkObject,TheValue);
end;

procedure TFPgtkToolbar.InsertWidget (Widget:TFPgtkWidget; TooltipText:string; TooltipPrivate:string; Position:integer);
begin
  gtk_toolbar_insert_widget (TheGtkObject, ConvertToGtkWidget(Widget), ConvertToPgchar(TooltipText), ConvertTopgchar(TooltipPrivate), Position);
  Widget.Show;
end;

procedure TFPgtkToolbar.PrependWidget (Widget:TFPgtkWidget; TooltipText:string; TooltipPrivate:string);
begin
  gtk_toolbar_prepend_widget (TheGtkObject, ConvertToGtkWidget(Widget), ConvertTopgchar(TooltipText), ConvertTopgchar(TooltipPrivate));
  Widget.Show;
end;

procedure TFPgtkToolbar.AppendWidget (Widget:TFPgtkWidget; TooltipText:string; TooltipPrivate:string);
begin
  gtk_toolbar_append_widget (TheGtkObject, ConvertToGtkWidget(Widget), ConvertTopgchar(TooltipText), ConvertTopgchar(TooltipPrivate));
  Widget.Show;
end;

function TFPgtkToolbar.InsertElement (ButtonType:TGtkToolbarChildType; PrevRadioBut:TFPgtkWidget; Text:string; TooltipText:string; TooltipPrivate:string; Icon:TFPgtkWidget; CallBack:TFPgtkSignalFunction; data:pointer; position:integer) : TFPgtkWidget;
var w : PGtkWidget;
    t : TFPgtkObjectClass;
begin
  w := gtk_toolbar_insert_element (TheGtkObject, ButtonType,
           ConvertToGtkwidget(PrevRadioBut), ConvertTopgchar(Text),
           ConvertTopgchar(TooltipText), ConvertTopgchar(TooltipPrivate),
           ConvertToGtkwidget(Icon),
           gtk_signal_func(@SignalProc),
           ConvertSignalData(TFPgtkSignalFunction(callback), data, true),
           position);
  if assigned (w) then
    begin
    case ButtonType of
      GTK_TOOLBAR_CHILD_WIDGET:
        t := TFPgtkWidget;
      GTK_TOOLBAR_CHILD_BUTTON:
        t := TFPgtkButton;
      GTK_TOOLBAR_CHILD_TOGGLEBUTTON:
        t := TFPgtkToggleButton;
      GTK_TOOLBAR_CHILD_RADIOBUTTON:
        t := TFPgtkRadioButton;
    end;
    if t = TFPgtkWidget then
      result := GetPascalInstance (w)
    else
      result := GetPascalInstance (w, t);
    end
  else
    result := nil;
end;

function TFPgtkToolbar.AppendElement (ButtonType:TGtkToolbarChildType; PrevRadioBut:TFPgtkWidget; Text:string; TooltipText:string; TooltipPrivate:string; Icon:TFPgtkWidget; CallBack:TFPgtkSignalFunction; data:pointer) : TFPgtkWidget;
var w : PGtkWidget;
    t : TFPgtkObjectClass;
begin
  w := gtk_toolbar_append_element (TheGtkObject, ButtonType, ConvertToGtkwidget(PrevRadioBut),
          ConvertTopgchar(Text), ConvertTopgchar(TooltipText), ConvertTopgchar(TooltipPrivate),
          ConvertToGtkwidget(Icon), gtk_signal_func(@SignalProc),
          ConvertSignalData(TFPgtkSignalFunction(callback), data, true));
  if assigned (w) then
    begin
    case ButtonType of
      GTK_TOOLBAR_CHILD_WIDGET:
        t := TFPgtkWidget;
      GTK_TOOLBAR_CHILD_BUTTON:
        t := TFPgtkButton;
      GTK_TOOLBAR_CHILD_TOGGLEBUTTON:
        t := TFPgtkToggleButton;
      GTK_TOOLBAR_CHILD_RADIOBUTTON:
        t := TFPgtkRadioButton;
    end;
    if t = TFPgtkWidget then
      result := GetPascalInstance (w)
    else
      result := GetPascalInstance (w, t);
    end
  else
    result := nil;
end;

function TFPgtkToolbar.PrependElement (ButtonType:TGtkToolbarChildType; PrevRadioBut:TFPgtkWidget; Text:string; TooltipText:string; TooltipPrivate:string; Icon:TFPgtkWidget; CallBack:TFPgtkSignalFunction; data:pointer) : TFPgtkWidget;
var w : PGtkWidget;
    t : TFPgtkObjectClass;
begin
  w := gtk_toolbar_prepend_element (TheGtkObject, ButtonType, ConvertToGtkwidget(PrevRadioBut),
          ConvertTopgchar(Text), ConvertTopgchar(TooltipText), ConvertTopgchar(TooltipPrivate),
          ConvertToGtkwidget(Icon), gtk_signal_func(@SignalProc),
          ConvertSignalData(TFPgtkSignalFunction(callback), data, true));
  if assigned (w) then
    begin
    case ButtonType of
      GTK_TOOLBAR_CHILD_WIDGET:
        t := TFPgtkWidget;
      GTK_TOOLBAR_CHILD_BUTTON:
        t := TFPgtkButton;
      GTK_TOOLBAR_CHILD_TOGGLEBUTTON:
        t := TFPgtkToggleButton;
      GTK_TOOLBAR_CHILD_RADIOBUTTON:
        t := TFPgtkRadioButton;
    end;
    if t = TFPgtkWidget then
      result := GetPascalInstance (w)
    else
      result := GetPascalInstance (w, t);
    end
  else
    result := nil;
end;

function TFPgtkToolbar.InsertItem (Text:string; TooltipText:string; TooltipPrivate:string; Icon:TFPgtkWidget; CallBack:TFPgtkSignalFunction; data:pointer; position:integer) : TFPgtkWidget; Overload;
begin
  result := GetPascalInstance (
      gtk_toolbar_insert_item (TheGtkObject, ConvertTopgchar(Text), ConvertTopgchar(TooltipText), ConvertTopgchar(TooltipPrivate), ConvertToGtkWidget(Icon),
              gtk_signal_func(@SignalProc), ConvertSignalData(TFPgtkSignalFunction(callback), data, true), position),
      TFPgtkButton);
end;

function TFPgtkToolbar.AppendItem (Text:string; TooltipText:string; TooltipPrivate:string; Icon:TFPgtkWidget; CallBack:TFPgtkSignalFunction; data:pointer) : TFPgtkWidget; Overload;
begin
  result := GetPascalInstance (
      gtk_toolbar_append_item (TheGtkObject, ConvertTopgchar(Text), ConvertTopgchar(TooltipText), ConvertTopgchar(TooltipPrivate),
              ConvertToGtkWidget(Icon), gtk_signal_func(@SignalProc), ConvertSignalData(TFPgtkSignalFunction(callback), data, true)),
      TFPgtkButton);
end;

function TFPgtkToolbar.PrependItem (Text:string; TooltipText:string; TooltipPrivate:string; Icon:TFPgtkWidget; CallBack:TFPgtkSignalFunction; data:pointer) : TFPgtkWidget; Overload;
begin
  result := GetPascalInstance (
      gtk_toolbar_prepend_item (TheGtkObject, Converttopgchar(Text), Converttopgchar(TooltipText),
              Converttopgchar(TooltipPrivate), ConvertToGtkWidget(Icon), gtk_signal_func(@SignalProc),
              ConvertSignalData(TFPgtkSignalFunction(callback), data, true)),
      TFPgtkButton);
end;

function TFPgtkToolbar.InsertItem (Text:string; TooltipText:string; TooltipPrivate:string; Icon:array of string; CallBack:TFPgtkSignalFunction; data:pointer; position:integer) : TFPgtkWidget; Overload;
var pm : TFPgtkPixmap;
begin
  if low(icon) < high(icon) then
    begin
    pm := TFPgtkPixmap.Create;
    pm.loadFromArray (icon);
    end
  else
    pm := nil;
  result := GetPascalInstance (
      gtk_toolbar_insert_item (TheGtkObject, ConvertTopgchar(Text), ConvertTopgchar(TooltipText), ConvertTopgchar(TooltipPrivate), ConvertToGtkWidget(pm),
              gtk_signal_func(@SignalProc), ConvertSignalData(TFPgtkSignalFunction(callback), data, true), position),
      TFPgtkButton);
end;

function TFPgtkToolbar.AppendItem (Text:string; TooltipText:string; TooltipPrivate:string; Icon:array of string; CallBack:TFPgtkSignalFunction; data:pointer) : TFPgtkWidget; Overload;
var pm : TFPgtkPixmap;
begin
  if low(icon) < high(icon) then
    begin
    pm := TFPgtkPixmap.Create;
    pm.loadFromArray (icon);
    end
  else
    pm := nil;
  result := GetPascalInstance (
      gtk_toolbar_append_item (TheGtkObject, ConvertTopgchar(Text), ConvertTopgchar(TooltipText), ConvertTopgchar(TooltipPrivate),
              ConvertToGtkWidget(pm), gtk_signal_func(@SignalProc), ConvertSignalData(TFPgtkSignalFunction(callback), data, true)),
      TFPgtkButton);
end;

function TFPgtkToolbar.PrependItem (Text:string; TooltipText:string; TooltipPrivate:string; Icon:array of string; CallBack:TFPgtkSignalFunction; data:pointer) : TFPgtkWidget; Overload;
var pm : TFPgtkPixmap;
begin
  if low(icon) < high(icon) then
    begin
    pm := TFPgtkPixmap.Create;
    pm.loadFromArray (icon);
    end
  else
    pm := nil;
  result := GetPascalInstance (
      gtk_toolbar_prepend_item (TheGtkObject, Converttopgchar(Text), Converttopgchar(TooltipText),
              Converttopgchar(TooltipPrivate), ConvertToGtkWidget(pm), gtk_signal_func(@SignalProc),
              ConvertSignalData(TFPgtkSignalFunction(callback), data, true)),
      TFPgtkButton);
end;

procedure TFPgtkToolbar.InsertSpace (position:integer);
begin
  gtk_toolbar_insert_space (TheGtkObject, position);
end;

procedure TFPgtkToolbar.AppendSpace;
begin
  gtk_toolbar_append_space (TheGtkObject);
end;

procedure TFPgtkToolbar.PrependSpace;
begin
  gtk_toolbar_prepend_space (TheGtkObject);
end;

 { TFPgtkTree }

function TFPgtkTree.TheGtkObject : PGtkTree;
begin
  result := PgtkTree(FGtkObject);
end;

procedure TFPgtkTree.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_tree_new);
end;


function TFPgtkTree.ConnectSelectionChanged (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnect (sgSelectionChanged, proc, data);
end;

function TFPgtkTree.ConnectAfterSelectionChanged (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnectAfter (sgSelectionChanged, proc, data);
end;

function TFPgtkTree.ConnectSelectChild (proc:TFPgtkWidgetSignalFunction; data:pointer) : guint;
begin
  result := WidgetSignalConnect (sgSelectChild, proc, data);
end;

function TFPgtkTree.ConnectAfterSelectChild (proc:TFPgtkWidgetSignalFunction; data:pointer) : guint;
begin
  result := WidgetSignalConnectAfter (sgSelectChild, proc, data);
end;

function TFPgtkTree.ConnectUnselectChild (proc:TFPgtkWidgetSignalFunction; data:pointer) : guint;
begin
  result := WidgetSignalConnect (sgUnselectChild, proc, data);
end;

function TFPgtkTree.ConnectAfterUnselectChild (proc:TFPgtkWidgetSignalFunction; data:pointer) : guint;
begin
  result := WidgetSignalConnectAfter (sgUnselectChild, proc, data);
end;

function TFPgtkTree.GetSelectionMode : TGtkSelectionMode;
begin
  result := gtk.selection_mode(TheGtkObject^);
end;

procedure TFPgtkTree.SetSelectionMode (TheValue:TGtkSelectionMode);
begin
  gtk_tree_set_selection_mode(TheGtkObject,TheValue);
end;

function TFPgtkTree.GetViewLines : boolean;
begin
  result := boolean(gtk.view_line(TheGtkObject^));
end;

procedure TFPgtkTree.SetViewLines (TheValue:boolean);
begin
  gtk_tree_set_view_lines(TheGtkObject,guint(TheValue));
end;

function TFPgtkTree.GetViewMode : TGtkTreeViewMode;
begin
  result := gtk.view_mode(TheGtkObject^);
end;

procedure TFPgtkTree.SetViewMode (TheValue:TGtkTreeViewMode);
begin
  gtk_tree_set_view_mode(TheGtkObject,TheValue);
end;

procedure TFPgtkTree.Append (TreeItem:TFPgtkWidget);
begin
  gtk_tree_append (TheGtkObject, PGtkwidget(ConvertToGtkObject(TreeItem)));
  TreeItem.Show;
end;

procedure TFPgtkTree.Prepend (TreeItem:TFPgtkWidget);
begin
  gtk_tree_prepend (TheGtkObject, PGtkwidget(ConvertToGtkObject(TreeItem)));
  TreeItem.Show;
end;

procedure TFPgtkTree.Insert (TreeItem:TFPgtkWidget; position:integer);
begin
  gtk_tree_insert (TheGtkObject, PGtkwidget(ConvertToGtkObject(TreeItem)),position);
  TreeItem.show;
end;

procedure TFPgtkTree.Remove (TreeItem:TFPgtkWidget);
var l : PGList;
begin
{$ifndef win32}
  gtk_tree_remove_item (TheGtkObject, ConvertToGtkWidget(TreeItem));
{$else}
  l := null;
  l := g_list_append (l, ConvertToGtkWidget(TreeItem));
  gtk_tree_remove_items (TheGtkObject, l);
  g_list_free (l);
{$endif}
end;

procedure TFPgtkTree.ClearItems (StartPos:integer; EndPos:integer);
begin
  gtk_tree_clear_items (TheGtkObject, StartPos, EndPos);
end;

procedure TFPgtkTree.SelectItem (Item:integer);
begin
  gtk_tree_select_item (TheGtkObject, Item);
end;

procedure TFPgtkTree.UnselectItem (Item:integer);
begin
  gtk_tree_unselect_item (TheGtkObject, Item);
end;

procedure TFPgtkTree.SelectChild (TreeItem:TFPgtkWidget);
begin
  gtk_tree_select_child (TheGtkObject, PGtkwidget(ConvertToGtkObject(TreeItem)));
end;

procedure TFPgtkTree.UnselectChild (TreeItem:TFPgtkWidget);
begin
  gtk_tree_unselect_child (TheGtkObject, PGtkwidget(ConvertToGtkObject(TreeItem)));
end;

function TFPgtkTree.ChildPosition (TreeItem:TFPgtkWidget) : integer;
begin
  result := gtk_tree_child_position (TheGtkObject, PGtkwidget(ConvertToGtkObject(TreeItem)));
end;

function TFPgtkTree.RootTree : TFPgtkTree;
begin
  result := GetPascalInstance(PGtkObject(GTK_TREE_ROOT_TREE(TheGtkObject))) as TFPgtkTree;
end;

function TFPgtkTree.IsRootTree : boolean;
begin
  result := GTK_IS_ROOT_TREE (TheGtkObject);
end;

procedure TFPgtkTree.GetSelection (aGroup:TFPgtkGroup);
begin
  aGroup.ManageLists := false;
  aGroup.GtkList := Gtk_Tree_selection (TheGtkObject);
end;

function TFPgtkTree.Level : integer;
begin
  result := TheGtkObject^.level;
end;

 { TFPgtkCalendar }

function TFPgtkCalendar.TheGtkObject : PGtkCalendar;
begin
  result := PgtkCalendar(FGtkObject);
end;

procedure TFPgtkCalendar.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_calendar_new);
end;


function TFPgtkCalendar.SelectMonth (aMonth:guint; aYear:guint) : integer;
begin
  result := gtk_calendar_select_month (TheGtkObject, aMonth-1, aYear);
end;

procedure TFPgtkCalendar.SelectDay (aDay:guint);
begin
  gtk_calendar_select_day (TheGtkObject, aDay);
end;

function TFPgtkCalendar.MarkDay (aDay:guint) : integer;
begin
  result := gtk_calendar_mark_day (TheGtkObject, aDay);
end;

function TFPgtkCalendar.UnmarkDay (aDay:guint) : integer;
begin
  result := gtk_calendar_unmark_day (TheGtkObject, aDay);
end;

procedure TFPgtkCalendar.ClearMarks;
begin
  gtk_calendar_clear_marks (TheGtkObject);
end;

function TFPgtkCalendar.GetDisplayOptions : TGtkCalendarDisplayOptions;
begin
  result := TheGtkObject^.display_flags;
end;

procedure TFPgtkCalendar.SetDisplayOptions (TheValue:TGtkCalendarDisplayOptions);
begin
  gtk_calendar_display_options(TheGtkObject,TheValue);
end;

function TFPgtkCalendar.GetDate : TDatetime;
var y, m, d : guint;
begin
  gtk_calendar_get_date (TheGtkObject, @y, @m, @d);
  result := encodedate (y,m+1,d);
end;

procedure TFPgtkCalendar.SetDate (TheValue:TDatetime);
var y,m,d : word;
begin
  decodedate (TheValue, y,m,d);
  SelectMonth(m,y);
  SelectDay(d);
end;

procedure TFPgtkCalendar.Freeze;
begin
  gtk_calendar_freeze (TheGtkObject);
end;

procedure TFPgtkCalendar.Thaw;
begin
  gtk_calendar_thaw (TheGtkObject);
end;

function TFPgtkCalendar.ConnectMonthChanged (proc:TFPgtksignalFunction; data:pointer) : guint;
begin
  result := signalConnect (sgMonthChanged, proc, data);
end;

function TFPgtkCalendar.ConnectAfterMonthChanged (proc:TFPgtksignalFunction; data:pointer) : guint;
begin
  result := signalConnectAfter (sgMonthChanged, proc, data);
end;

function TFPgtkCalendar.ConnectDaySelected (proc:TFPgtksignalFunction; data:pointer) : guint;
begin
  result := signalConnect (sgDaySelected, proc, data);
end;

function TFPgtkCalendar.ConnectAfterDaySelected (proc:TFPgtksignalFunction; data:pointer) : guint;
begin
  result := signalConnectAfter (sgDaySelected, proc, data);
end;

function TFPgtkCalendar.ConnectDaySelectedDoubleClick (proc:TFPgtksignalFunction; data:pointer) : guint;
begin
  result := signalConnect (sgDaySelectedDoubleClick, proc, data);
end;

function TFPgtkCalendar.ConnectAfterDaySelectedDoubleClick (proc:TFPgtksignalFunction; data:pointer) : guint;
begin
  result := signalConnectAfter (sgDaySelectedDoubleClick, proc, data);
end;

function TFPgtkCalendar.ConnectPrevMonth (proc:TFPgtksignalFunction; data:pointer) : guint;
begin
  result := signalConnect (sgPrevMonth, proc, data);
end;

function TFPgtkCalendar.ConnectAfterPrevMonth (proc:TFPgtksignalFunction; data:pointer) : guint;
begin
  result := signalConnectAfter (sgPrevMonth, proc, data);
end;

function TFPgtkCalendar.ConnectNextMonth (proc:TFPgtksignalFunction; data:pointer) : guint;
begin
  result := signalConnect (sgNextMonth, proc, data);
end;

function TFPgtkCalendar.ConnectAfterNextMonth (proc:TFPgtksignalFunction; data:pointer) : guint;
begin
  result := signalConnectAfter (sgNextMonth, proc, data);
end;

function TFPgtkCalendar.ConnectPrevYear (proc:TFPgtksignalFunction; data:pointer) : guint;
begin
  result := signalConnect (sgPrevYear, proc, data);
end;

function TFPgtkCalendar.ConnectAfterPrevYear (proc:TFPgtksignalFunction; data:pointer) : guint;
begin
  result := signalConnectAfter (sgPrevYear, proc, data);
end;

function TFPgtkCalendar.ConnectNextYear (proc:TFPgtksignalFunction; data:pointer) : guint;
begin
  result := signalConnect (sgNextYear, proc, data);
end;

function TFPgtkCalendar.ConnectAfterNextYear (proc:TFPgtksignalFunction; data:pointer) : guint;
begin
  result := signalConnectAfter (sgNextYear, proc, data);
end;

 { TFPgtkDrawingArea }

function TFPgtkDrawingArea.TheGtkObject : PGtkDrawingArea;
begin
  result := PgtkDrawingArea(FGtkObject);
end;

procedure TFPgtkDrawingArea.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_drawing_area_new);
end;


procedure TFPgtkDrawingArea.SetSize (Width:integer; Height:integer);
begin
  gtk_drawing_area_Size (TheGtkObject, Width, Height);
end;

 { TFPgtkCurve }

function TFPgtkCurve.TheGtkObject : PGtkCurve;
begin
  result := PgtkCurve(FGtkObject);
end;

procedure TFPgtkCurve.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_curve_new);
end;


procedure TFPgtkCurve.SetRange (MinX:float; MaxX:float; MinY:float; MaxY:float);
begin
  gtk_curve_set_range (TheGtkObject, MinX, MaxX, MinY, MaxY);
end;

procedure TFPgtkCurve.Reset;
begin
  gtk_curve_reset (TheGtkObject);
end;

procedure TFPgtkCurve.SetGamma (GammaValue:float);
begin
  gtk_curve_set_gamma (TheGtkObject, GammaValue);
end;

function TFPgtkCurve.GetCurveType : TGtkCurveType;
begin
  result := TheGtkObject^.curve_type;
end;

procedure TFPgtkCurve.SetCurveType (TheValue:TGtkCurveType);
begin
  gtk_curve_set_curve_type(TheGtkObject,TheValue);
end;

 { TFPgtkEditable }

function TFPgtkEditable.TheGtkObject : PGtkEditable;
begin
  result := PgtkEditable(FGtkObject);
end;


function TFPgtkEditable.GetHasSelection : boolean;
begin
  result := SelectionStart <> SelectionEnd;
end;

function TFPgtkEditable.GetEditable : boolean;
begin
  result := boolean(gtk.editable(TheGtkObject^));
end;

procedure TFPgtkEditable.SetEditable (TheValue:boolean);
begin
  gtk_Editable_set_editable(TheGtkObject,TheValue);
end;

function TFPgtkEditable.GetVisible : boolean;
begin
  result := boolean(gtk.visible(TheGtkObject^));
end;

procedure TFPgtkEditable.SetVisible (TheValue:boolean);
begin
  gtk.Set_visible(TheGtkObject^,guint(TheValue))
end;

function TFPgtkEditable.GetPosition : integer;
begin
  result := gtk_Editable_get_position(TheGtkObject);
end;

procedure TFPgtkEditable.SetPosition (TheValue:integer);
begin
  gtk_Editable_set_position(TheGtkObject,TheValue);
end;

function TFPgtkEditable.GetSelectionStart : integer;
begin
  result := TheGtkObject^.selection_start_pos;
end;

procedure TFPgtkEditable.SetSelectionStart (TheValue:integer);
begin
  gtk_editable_select_region (TheGtkObject, TheValue, SelectionEnd);
end;

function TFPgtkEditable.GetSelectionEnd : integer;
begin
  result := TheGtkObject^.Selection_end_pos;
end;

procedure TFPgtkEditable.SetSelectionEnd (TheValue:integer);
begin
  gtk_editable_select_region (TheGtkObject, SelectionStart, TheValue);
end;

procedure TFPgtkEditable.SetSelection (TheValue:string);
var b : integer;
begin
  if HasSelection then
    begin
    b := SelectionStart;
    deleteText (SelectionStart, SelectionEnd);
    end
  else
    b := position;
  InsertText (TheValue, b);
  Position := b + length(TheValue);
  SelectRegion (b, position);
end;

function TFPgtkEditable.GetSelection : string;
var c : pgchar;
begin
  c := gtk_editable_get_chars (TheGtkObject, SelectionStart, SelectionEnd);
  result := string (c);
  g_free (c);
end;

function TFPgtkEditable.GetText : string;
var c : pgchar;
begin
  c := gtk_editable_get_chars (TheGtkObject, 0, -1);
  result := string (c);
  g_free (c);
end;

procedure TFPgtkEditable.Changed;
begin
  gtk_Editable_Changed (TheGtkObject);
end;

procedure TFPgtkEditable.InsertText (NewText:string; AtPosition:integer);
var p : integer;
begin
  p := AtPosition;
  gtk_editable_insert_text (TheGtkObject, pgchar(NewText), length(NewText), @p);
end;

procedure TFPgtkEditable.DeleteText (StartPos:integer; EndPos:integer);
begin
  gtk_Editable_Delete_Text (TheGtkObject, StartPos, EndPos);
end;

procedure TFPgtkEditable.GetChars (StartPos:integer; EndPos:integer);
begin
  gtk_Editable_get_chars (TheGtkObject, StartPos, EndPos);
end;

procedure TFPgtkEditable.CutClipboard;
begin
  gtk_Editable_cut_clipboard (TheGtkObject);
end;

procedure TFPgtkEditable.CopyClipboard;
begin
  gtk_Editable_copy_clipboard (TheGtkObject);
end;

procedure TFPgtkEditable.PasteClipboard;
begin
  gtk_Editable_paste_clipboard (TheGtkObject);
end;

procedure TFPgtkEditable.SelectRegion (StartPos:integer; EndPos:integer);
begin
  gtk_Editable_select_region (TheGtkObject, StartPos, EndPos);
end;

procedure TFPgtkEditable.ClaimSelection (claim:boolean; time:guint32);
begin
  gtk_Editable_claim_selection (TheGtkObject, claim, time);
end;

procedure TFPgtkEditable.DeleteSelection;
begin
  gtk_Editable_delete_selection (TheGtkObject);
end;

procedure TFPgtkEditable.Clear;
begin
  DeleteText (0,-1);
end;

procedure InsertSignalproc (Sender:PGtkobject; NewText:pgChar; TextLength:integer; var Position:integer; data:pointer); cdecl;
var p : TFPgtkInsertSignalFunction;
begin
with PSignalData(data)^ do
  begin
  p := TFPgtkInsertSignalFunction (TheSignalProc);
  p (TheWidget as TFPgtkObject, NewText, TextLength, Position, TheData)
  end;
end;

function TFPgtkEditable.InsertSignalConnect (signal:string; proc:TFPgtkInsertSignalFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect (FGtkObject, pgChar(signal), gtk_signal_func(@InsertSignalproc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

function TFPgtkEditable.InsertSignalConnectAfter (signal:string; proc:TFPgtkInsertSignalFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect_After (FGtkObject, pgChar(signal), gtk_signal_func(@InsertSignalproc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

procedure DeleteSignalproc (Sender:PGtkobject; StartPos:integer; EndPos:integer; data:pointer); cdecl;
var p : TFPgtkDeleteSignalFunction;
begin
with PSignalData(data)^ do
  begin
  p := TFPgtkDeleteSignalFunction (TheSignalProc);
  p (TheWidget as TFPgtkObject, StartPos, EndPos, TheData)
  end;
end;

function TFPgtkEditable.DeleteSignalConnect (signal:string; proc:TFPgtkDeleteSignalFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect (FGtkObject, pgChar(signal), gtk_signal_func(@DeleteSignalproc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

function TFPgtkEditable.DeleteSignalConnectAfter (signal:string; proc:TFPgtkDeleteSignalFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect_After (FGtkObject, pgChar(signal), gtk_signal_func(@DeleteSignalproc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

procedure XYSignalproc (Sender:PGtkobject; x:integer; y:integer; data:pointer); cdecl;
var p : TFPgtkXYSignalFunction;
begin
with PSignalData(data)^ do
  begin
  p := TFPgtkXYSignalFunction (TheSignalProc);
  p (TheWidget as TFPgtkObject, x, y, TheData)
  end;
end;

function TFPgtkEditable.XYSignalConnect (signal:string; proc:TFPgtkXYSignalFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect (FGtkObject, pgChar(signal), gtk_signal_func(@XYSignalproc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

function TFPgtkEditable.XYSignalConnectAfter (signal:string; proc:TFPgtkXYSignalFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect_After (FGtkObject, pgChar(signal), gtk_signal_func(@XYSignalproc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

procedure DirectionSignalproc (Sender:PGtkobject; Direction:integer; data:pointer); cdecl;
var p : TFPgtkDirectionSignalFunction;
begin
with PSignalData(data)^ do
  begin
  p := TFPgtkDirectionSignalFunction (TheSignalProc);
  p (TheWidget as TFPgtkObject, Direction, TheData)
  end;
end;

function TFPgtkEditable.DirectionSignalConnect (signal:string; proc:TFPgtkDirectionSignalFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect (FGtkObject, pgChar(signal), gtk_signal_func(@DirectionSignalproc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

function TFPgtkEditable.DirectionSignalConnectAfter (signal:string; proc:TFPgtkDirectionSignalFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect_After (FGtkObject, pgChar(signal), gtk_signal_func(@DirectionSignalproc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

procedure MoveWordSignalproc (Sender:PGtkobject; NumWords:integer; data:pointer); cdecl;
var p : TFPgtkMoveWordSignalFunction;
begin
with PSignalData(data)^ do
  begin
  p := TFPgtkMoveWordSignalFunction (TheSignalProc);
  p (TheWidget as TFPgtkObject, NumWords, TheData)
  end;
end;

function TFPgtkEditable.MoveWordSignalConnect (signal:string; proc:TFPgtkMoveWordSignalFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect (FGtkObject, pgChar(signal), gtk_signal_func(@MoveWordSignalproc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

function TFPgtkEditable.MoveWordSignalConnectAfter (signal:string; proc:TFPgtkMoveWordSignalFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect_After (FGtkObject, pgChar(signal), gtk_signal_func(@MoveWordSignalproc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

procedure MovetoSignalproc (Sender:PGtkobject; MoveTo:integer; data:pointer); cdecl;
var p : TFPgtkMovetoSignalFunction;
begin
with PSignalData(data)^ do
  begin
  p := TFPgtkMovetoSignalFunction (TheSignalProc);
  p (TheWidget as TFPgtkObject, MoveTo, TheData)
  end;
end;

function TFPgtkEditable.MovetoSignalConnect (signal:string; proc:TFPgtkMovetoSignalFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect (FGtkObject, pgChar(signal), gtk_signal_func(@MovetoSignalproc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

function TFPgtkEditable.MovetoSignalConnectAfter (signal:string; proc:TFPgtkMovetoSignalFunction; data:pointer) : guint;
begin
  result := gtk_signal_connect_After (FGtkObject, pgChar(signal), gtk_signal_func(@MovetoSignalproc), ConvertSignalData(TFPgtkSignalFunction(proc), data, true));
end;

function TFPgtkEditable.ConnectChanged (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnect (sgChanged, proc, data);
end;

function TFPgtkEditable.ConnectAfterChanged (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnectAfter (sgChanged, proc, data);
end;

function TFPgtkEditable.ConnectActivate (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnect (sgActivate, proc, data);
end;

function TFPgtkEditable.ConnectAfterActivate (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnectAfter (sgActivate, proc, data);
end;

function TFPgtkEditable.ConnectInsertText (proc:TFPgtkInsertSignalFunction; data:pointer) : guint;
begin
  result := InsertSignalConnect (sgInsertText, proc, data);
end;

function TFPgtkEditable.ConnectAfterInsertText (proc:TFPgtkInsertSignalFunction; data:pointer) : guint;
begin
  result := InsertSignalConnectAfter (sgInsertText, proc, data);
end;

function TFPgtkEditable.ConnectDeleteText (proc:TFPgtkDeleteSignalFunction; data:pointer) : guint;
begin
  result := DeleteSignalConnect (sgDeleteText, proc, data);
end;

function TFPgtkEditable.ConnectAfterDeleteText (proc:TFPgtkDeleteSignalFunction; data:pointer) : guint;
begin
  result := DeleteSignalConnectAfter (sgDeleteText, proc, data);
end;

function TFPgtkEditable.ConnectSetEditable (proc:TFPgtkBooleanSignalFunction; data:pointer) : guint;
begin
  result := BooleanSignalConnect (sgSetEditable, proc, data);
end;

function TFPgtkEditable.ConnectAfterSetEditable (proc:TFPgtkBooleanSignalFunction; data:pointer) : guint;
begin
  result := BooleanSignalConnectAfter (sgSetEditable, proc, data);
end;

function TFPgtkEditable.ConnectMoveCursor (proc:TFPgtkXYSignalFunction; data:pointer) : guint;
begin
  result := XYSignalConnect (sgMoveCursor, proc, data);
end;

function TFPgtkEditable.ConnectAfterMoveCursor (proc:TFPgtkXYSignalFunction; data:pointer) : guint;
begin
  result := XYSignalConnectAfter (sgMoveCursor, proc, data);
end;

function TFPgtkEditable.ConnectMoveWord (proc:TFPgtkMoveWordSignalFunction; data:pointer) : guint;
begin
  result := MoveWordSignalConnect (sgMoveWord, proc, data);
end;

function TFPgtkEditable.ConnectAfterMoveWord (proc:TFPgtkMoveWordSignalFunction; data:pointer) : guint;
begin
  result := MoveWordSignalConnectAfter (sgMoveWord, proc, data);
end;

function TFPgtkEditable.ConnectMovePage (proc:TFPgtkXYSignalFunction; data:pointer) : guint;
begin
  result := XYSignalConnect (sgMovePage, proc, data);
end;

function TFPgtkEditable.ConnectAfterMovePage (proc:TFPgtkXYSignalFunction; data:pointer) : guint;
begin
  result := XYSignalConnectAfter (sgMovePage, proc, data);
end;

function TFPgtkEditable.ConnectMoveToRow (proc:TFPgtkMoveToSignalFunction; data:pointer) : guint;
begin
  result := MoveToSignalConnect (sgMoveToRow, proc, data);
end;

function TFPgtkEditable.ConnectAfterMoveToRow (proc:TFPgtkMoveToSignalFunction; data:pointer) : guint;
begin
  result := MoveToSignalConnectAfter (sgMoveToRow, proc, data);
end;

function TFPgtkEditable.ConnectMoveToCol (proc:TFPgtkMoveToSignalFunction; data:pointer) : guint;
begin
  result := MoveToSignalConnect (sgMoveToCol, proc, data);
end;

function TFPgtkEditable.ConnectAfterMoveToCol (proc:TFPgtkMoveToSignalFunction; data:pointer) : guint;
begin
  result := MoveToSignalConnectAfter (sgMoveToCol, proc, data);
end;

function TFPgtkEditable.ConnectKillChar (proc:TFPgtkDirectionSignalFunction; data:pointer) : guint;
begin
  result := DirectionSignalConnect (sgKillChar, proc, data);
end;

function TFPgtkEditable.ConnectAfterKillChar (proc:TFPgtkDirectionSignalFunction; data:pointer) : guint;
begin
  result := DirectionSignalConnectAfter (sgKillChar, proc, data);
end;

function TFPgtkEditable.ConnectKillWord (proc:TFPgtkDirectionSignalFunction; data:pointer) : guint;
begin
  result := DirectionSignalConnect (sgKillWord, proc, data);
end;

function TFPgtkEditable.ConnectAfterKillWord (proc:TFPgtkDirectionSignalFunction; data:pointer) : guint;
begin
  result := DirectionSignalConnectAfter (sgKillWord, proc, data);
end;

function TFPgtkEditable.ConnectKillLine (proc:TFPgtkDirectionSignalFunction; data:pointer) : guint;
begin
  result := DirectionSignalConnect (sgKillLine, proc, data);
end;

function TFPgtkEditable.ConnectAfterKillLine (proc:TFPgtkDirectionSignalFunction; data:pointer) : guint;
begin
  result := DirectionSignalConnectAfter (sgKillLine, proc, data);
end;

function TFPgtkEditable.ConnectCutClipboard (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnect (sgCutClipboard, proc, data);
end;

function TFPgtkEditable.ConnectAfterCutClipboard (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnectAfter (sgCutClipboard, proc, data);
end;

function TFPgtkEditable.ConnectCopyClipboard (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnect (sgCopyClipboard, proc, data);
end;

function TFPgtkEditable.ConnectAfterCopyClipboard (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnectAfter (sgCopyClipboard, proc, data);
end;

function TFPgtkEditable.ConnectPasteClipboard (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnect (sgPasteClipboard, proc, data);
end;

function TFPgtkEditable.ConnectAfterPasteClipboard (proc:TFPgtkSignalFunction; data:pointer) : guint;
begin
  result := SignalConnectAfter (sgPasteClipboard, proc, data);
end;

 { TFPgtkEntry }

function TFPgtkEntry.TheGtkObject : PGtkEntry;
begin
  result := PgtkEntry(FGtkObject);
end;

procedure TFPgtkEntry.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_Entry_new);
end;


procedure TFPgtkEntry.SetText (TheValue:string);
begin
  gtk_Entry_set_text (TheGtkObject, Pgchar(TheValue));
end;

procedure TFPgtkEntry.AppendText (aText:string);
begin
  gtk_Entry_append_text (TheGtkObject, ConvertToPgchar(aText));
end;

procedure TFPgtkEntry.PrependText (aText:string);
begin
  gtk_Entry_prepend_text (TheGtkObject, ConvertToPgchar(aText));
end;

function TFPgtkEntry.GetVisibility : boolean;
begin
  result := boolean(gtk.visible(TheGtkObject^));
end;

procedure TFPgtkEntry.SetVisibility (TheValue:boolean);
begin
  gtk_Entry_set_visibility(TheGtkObject,TheValue);
end;

function TFPgtkEntry.GetMaxLength : word;
begin
  result := TheGtkObject^.text_max_length;
end;

procedure TFPgtkEntry.SetMaxLength (TheValue:word);
begin
  gtk_Entry_set_max_length(TheGtkObject,TheValue);
end;

 { TFPgtkSpinButton }

function TFPgtkSpinButton.TheGtkObject : PGtkSpinButton;
begin
  result := PgtkSpinButton(FGtkObject);
end;

procedure TFPgtkSpinButton.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_spin_button_new (TFPgtkAdjustment.Create.TheGtkObject,1,0));
end;


procedure TFPgtkSpinButton.Configure (Adj:TFPgtkAdjustment; aClimbRate:gfloat; aDigits:integer);
begin
  if assigned (Adj) then
    gtk_spin_button_configure (TheGtkObject, PGtkadjustment(Adj.TheGtkObject), aClimbRate, aDigits)
  else
    gtk_spin_button_configure (TheGtkObject, nil, aClimbRate, aDigits);
end;

function TFPgtkSpinButton.GetAdjustment : TFPgtkAdjustment;
begin
  result := GetPascalInstance(PGtkObject(gtk_spin_button_get_adjustment(TheGtkObject)),TFPGtkAdjustment) as TFPgtkAdjustment;
end;

procedure TFPgtkSpinButton.SetAdjustment (TheValue:TFPgtkAdjustment);
begin
  gtk_spin_button_set_adjustment(TheGtkObject,PGtkadjustment(ConvertToGtkObject(TheValue)));
end;

function TFPgtkSpinButton.GetClimbRate : gfloat;
begin
  result := TheGtkObject^.climb_rate;
end;

procedure TFPgtkSpinButton.SetClimbRate (TheValue:gfloat);
begin
  TheGtkObject^.climb_rate := TheValue;
end;

function TFPgtkSpinButton.GetDigits : integer;
begin
  result := gtk.digits(TheGtkObject^);
end;

procedure TFPgtkSpinButton.SetDigits (TheValue:integer);
begin
  gtk_spin_button_set_digits(TheGtkObject,TheValue);
end;

function TFPgtkSpinButton.GetAsInteger : integer;
begin
  result := gtk_spin_button_get_value_as_int(TheGtkObject);
end;

procedure TFPgtkSpinButton.SetAsInteger (TheValue:integer);
begin
  gtk_spin_button_set_Value(TheGtkObject,TheValue);
end;

function TFPgtkSpinButton.GetAsFloat : gfloat;
begin
  result := gtk_spin_button_get_value_as_int(TheGtkObject);
end;

procedure TFPgtkSpinButton.SetAsFloat (TheValue:gfloat);
begin
  gtk_spin_button_set_Value(TheGtkObject,TheValue);
end;

function TFPgtkSpinButton.GetUpdatePolicy : TGtkSpinButtonUpdatePolicy;
begin
  result := TheGtkObject^.update_policy;
end;

procedure TFPgtkSpinButton.SetUpdatePolicy (TheValue:TGtkSpinButtonUpdatePolicy);
begin
  gtk_spin_button_set_update_policy(TheGtkObject,TheValue);
end;

function TFPgtkSpinButton.GetNumeric : boolean;
begin
  result := boolean(gtk.numeric(TheGtkObject^));
end;

procedure TFPgtkSpinButton.SetNumeric (TheValue:boolean);
begin
  gtk_spin_button_set_numeric(TheGtkObject,TheValue);
end;

procedure TFPgtkSpinButton.Spin (direction:TGtkSpinType; increment:gfloat);
begin
  gtk_spin_button_spin (TheGtkObject, direction, increment);
end;

function TFPgtkSpinButton.GetWrap : boolean;
begin
  result := boolean(gtk.wrap(TheGtkObject^));
end;

procedure TFPgtkSpinButton.SetWrap (TheValue:boolean);
begin
  gtk_spin_button_set_wrap(TheGtkObject,TheValue);
end;

function TFPgtkSpinButton.GetShadowType : TGtkShadowType;
begin
  result := TheGtkObject^.shadow_type;
end;

procedure TFPgtkSpinButton.SetShadowType (TheValue:TGtkShadowType);
begin
  gtk_spin_button_set_shadow_type(TheGtkObject,TheValue);
end;

function TFPgtkSpinButton.GetSnapToTicks : boolean;
begin
  result := boolean(gtk.snap_to_ticks(TheGtkObject^));
end;

procedure TFPgtkSpinButton.SetSnapToTicks (TheValue:boolean);
begin
  gtk_spin_button_set_snap_to_ticks(TheGtkObject,TheValue);
end;

procedure TFPgtkSpinButton.Update;
begin
  gtk_spin_button_update (TheGtkObject);
end;

 { TFPgtkText }

function TFPgtkText.TheGtkObject : PGtkText;
begin
  result := PgtkText(FGtkObject);
end;

procedure TFPgtkText.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_Text_new (null,null));
end;


constructor TFPgtkText.Create;
begin
  inherited create;
  editable := true;
  wordwrap := true;
  linewrap := true;
  FLines := TStringlist.Create;
  ConnectChanged (@SigChanged, nil);
end;


destructor TFPgtkText.Destroy;
begin
  FLines.Free;
  inherited;
end;


procedure TFPgtkText.SigChanged (Sender:TFPgtkObject; data:pointer);
begin
  FIsChanged := True;
end;

procedure TFPgtkText.RefreshLines;
begin
  if not assigned (FLines) then
    FLines := TStringlist.Create;
  FLines.Text := Text;
end;

function TFPgtkText.GetLines : TStrings;
begin
  if FIsChanged then
    RefreshLines;
  result := FLines;
end;

procedure TFPgtkText.Freeze;
begin
  gtk_Text_Freeze (TheGtkObject);
end;

procedure TFPgtkText.Thaw;
begin
  gtk_Text_Thaw (TheGtkObject);
end;

function TFPgtkText.TextLength : guint;
begin
  result := gtk_Text_get_length (TheGtkObject);
end;

procedure TFPgtkText.Insert (font:PgdkFont; fore:PgdkColor; back:PgdkColor; TheText:string);
begin
  gtk_text_insert (TheGtkObject, font, fore, back, pgchar(TheText), length(TheText));
end;

procedure TFPgtkText.DeleteBackward (number:longword);
begin
  gtk_Text_Backward_Delete (TheGtkObject, number);
end;

procedure TFPgtkText.DeleteForward (number:longword);
begin
  gtk_Text_Forward_Delete (TheGtkObject, number);
end;

function TFPgtkText.GetWordWrap : boolean;
begin
  result := boolean(gtk.word_wrap(TheGtkObject^));
end;

procedure TFPgtkText.SetWordWrap (TheValue:boolean);
begin
  gtk_text_set_word_wrap (TheGtkObject,gint(TheValue));
end;

function TFPgtkText.GetLineWrap : boolean;
begin
  result := boolean(gtk.Line_Wrap(TheGtkObject^));
end;

procedure TFPgtkText.SetLineWrap (TheValue:boolean);
begin
{$IFDEF win32 or go32v2}
  Set_Line_Wrap (TheGtkObject^, gint(TheValue));
{$ELSE}
  gtk_Text_Set_Line_Wrap (TheGtkObject, gint(TheValue));
{$ENDIF}
end;

function TFPgtkText.GetPoint : integer;
begin
  result := gtk_Text_get_Point(TheGtkObject);
end;

procedure TFPgtkText.SetPoint (TheValue:integer);
begin
  gtk_Text_set_Point(TheGtkObject,TheValue);
end;

procedure TFPgtkText.SetAdjustments (hadj:TFPgtkAdjustment; vadj:TFPgtkAdjustment);
begin
  gtk_text_set_adjustments (TheGtkObject, hadj.TheGtkObject, vadj.TheGtkObject);
end;

function TFPgtkText.GetHAdjustment : TFPgtkAdjustment;
begin
  result := GetPascalInstance(PGtkObject(TheGtkObject^.hadj),tfpgtkadjustment) as tfpgtkadjustment;
end;

procedure TFPgtkText.SetHAdjustment (TheValue:TFPgtkAdjustment);
begin
  gtk_Text_Set_Adjustments(TheGtkObject, TheValue.TheGtkObject, TheGtkObject^.vadj);
end;

function TFPgtkText.GetVAdjustment : TFPgtkAdjustment;
begin
  result := GetPascalInstance(PGtkObject(TheGtkObject^.vadj),tfpgtkadjustment) as tfpgtkadjustment;
end;

procedure TFPgtkText.SetVAdjustment (TheValue:TFPgtkAdjustment);
begin
  gtk_Text_Set_Adjustments(TheGtkObject, TheGtkObject^.hadj, TheValue.TheGtkObject);
end;

procedure TFPgtkText.SetText (TheValue:string);
begin
  Freeze;
  {$ifdef gtkwin}
  TheValue := stringreplace (TheValue, #13#10, #10, [rfReplaceAll]);
  {$endif}
  clear;
  Insert (null, null, null, TheValue);
  Thaw;
end;

 { TFPgtkRuler }

function TFPgtkRuler.TheGtkObject : PGtkRuler;
begin
  result := PgtkRuler(FGtkObject);
end;


procedure TFPgtkRuler.SetMetric (aMetric:TGtkMetricType);
begin
  gtk_ruler_set_metric (TheGtkObject, aMetric);
end;

procedure TFPgtkRuler.SetRange (Lower:float; Upper:float; Position:float; MaxSize:float);
begin
  gtk_ruler_set_range (TheGtkObject, Lower, Upper, Position, MaxSize);
end;

 { TFPgtkHRuler }

function TFPgtkHRuler.TheGtkObject : PGtkHRuler;
begin
  result := PgtkHRuler(FGtkObject);
end;

procedure TFPgtkHRuler.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_hruler_new);
end;


 { TFPgtkVRuler }

function TFPgtkVRuler.TheGtkObject : PGtkVRuler;
begin
  result := PgtkVRuler(FGtkObject);
end;

procedure TFPgtkVRuler.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_vruler_new);
end;


 { TFPgtkRange }

function TFPgtkRange.TheGtkObject : PGtkRange;
begin
  result := PgtkRange(FGtkObject);
end;


function TFPgtkRange.GetAdjustment : TFPgtkAdjustment;
begin
  result := GetPascalInstance(PGtkObject(gtk_Range_get_Adjustment(TheGtkObject)),tfpgtkadjustment) as tfpgtkadjustment;
end;

procedure TFPgtkRange.SetAdjustment (TheValue:TFPgtkAdjustment);
begin
  gtk_Range_set_adjustment(TheGtkObject,PGtkadjustment(ConvertToGtkObject(TheValue)));
end;

function TFPgtkRange.GetUpdatePolicy : TgtkUpdateType;
begin
  result := gtk.policy(TheGtkObject^);
end;

procedure TFPgtkRange.SetUpdatePolicy (TheValue:TgtkUpdateType);
begin
  gtk_Range_set_update_policy(TheGtkObject,TheValue);
end;

constructor TFPgtkRange.Create (AnAdjustment:TFPgtkAdjustment);
begin
  FAdj := AnAdjustment;
  inherited create;
end;


procedure TFPgtkRange.DrawBackground;
begin
  gtk_Range_draw_background (TheGtkObject);
end;

procedure TFPgtkRange.DrawTrough;
begin
  gtk_Range_draw_trough (TheGtkObject);
end;

procedure TFPgtkRange.DrawStepForw;
begin
  gtk_Range_draw_step_forw (TheGtkObject);
end;

procedure TFPgtkRange.DrawStepBack;
begin
  gtk_Range_draw_step_back (TheGtkObject);
end;

procedure TFPgtkRange.DrawSlider;
begin
  gtk_Range_draw_slider (TheGtkObject);
end;

procedure TFPgtkRange.SliderUpdate;
begin
  gtk_Range_slider_update (TheGtkObject);
end;

function TFPgtkRange.TroughClick (X:integer; Y:integer; var JumpPerc:gfloat) : integer;
begin
  result := gtk_Range_trough_click (TheGtkObject, X, Y, @JumpPerc);
end;

procedure TFPgtkRange.DefaultHSliderUpdate;
begin
  gtk_Range_default_hslider_update (TheGtkObject);
end;

procedure TFPgtkRange.DefaultVSliderUpdate;
begin
  gtk_Range_default_vslider_update (TheGtkObject);
end;

function TFPgtkRange.DefaultHTroughClick (X:integer; Y:integer; var JumpPerc:gfloat) : integer;
begin
  result := gtk_Range_default_htrough_click (TheGtkObject, X, Y, @JumpPerc);
end;

function TFPgtkRange.DefaultVTroughClick (X:integer; Y:integer; var JumpPerc:gfloat) : integer;
begin
  result := gtk_Range_default_vtrough_click (TheGtkObject, X, Y, @JumpPerc);
end;

procedure TFPgtkRange.defaultHMotion (XDelta:integer; YDelta:integer);
begin
  gtk_Range_default_hmotion (TheGtkObject, XDelta, YDelta);
end;

procedure TFPgtkRange.defaultVMotion (XDelta:integer; YDelta:integer);
begin
  gtk_Range_default_vmotion (TheGtkObject, XDelta, YDelta);
end;

procedure TFPgtkRange.ClearBackground;
begin
  gtk_Range_clear_background (TheGtkObject);
end;

 { TFPgtkScale }

function TFPgtkScale.TheGtkObject : PGtkScale;
begin
  result := PgtkScale(FGtkObject);
end;


procedure TFPgtkScale.SetDigits (TheValue:integer);
begin
  gtk_scale_set_digits (TheGtkObject, TheValue);
end;

function TFPgtkScale.GetDrawValue : boolean;
begin
  result := boolean(gtk.draw_value(TheGtkObject^));
end;

procedure TFPgtkScale.SetDrawValue (TheValue:boolean);
begin
  gtk_scale_set_draw_value(TheGtkObject,TheValue);
end;

function TFPgtkScale.GetValuePos : TGtkPositionType;
begin
  result := gtk.value_pos(TheGtkObject^);
end;

procedure TFPgtkScale.SetValuePos (TheValue:TGtkPositionType);
begin
  gtk_scale_set_value_pos(TheGtkObject,TheValue);
end;

 { TFPgtkHScale }

function TFPgtkHScale.TheGtkObject : PGtkHScale;
begin
  result := PgtkHScale(FGtkObject);
end;

procedure TFPgtkHScale.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_hscale_new (nil));
end;


 { TFPgtkVScale }

function TFPgtkVScale.TheGtkObject : PGtkVScale;
begin
  result := PgtkVScale(FGtkObject);
end;

procedure TFPgtkVScale.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_vscale_new (nil));
end;


 { TFPgtkScrollbar }

function TFPgtkScrollbar.TheGtkObject : PGtkScrollbar;
begin
  result := PgtkScrollbar(FGtkObject);
end;


 { TFPgtkHScrollbar }

function TFPgtkHScrollbar.TheGtkObject : PGtkHScrollbar;
begin
  result := PgtkHScrollbar(FGtkObject);
end;


procedure TFPgtkHScrollbar.CreateGtkObject;
var a : PgtkAdjustment;
begin
  if assigned (FAdj) then
    a := FAdj.TheGtkObject
  else
    a := null;
  FGtkObject := PgtkObject (gtk_hscrollbar_new (a));
  FAdj := nil;
end;

 { TFPgtkVScrollbar }


procedure TFPgtkVScrollbar.CreateGtkObject;
var a : PgtkAdjustment;
begin
  if assigned (FAdj) then
    a := FAdj.TheGtkObject
  else
    a := null;
  FGtkObject := PgtkObject (gtk_vscrollbar_new (a));
  FAdj := nil;
end;

 { TFPgtkSeparator }

function TFPgtkSeparator.TheGtkObject : PGtkSeparator;
begin
  result := PgtkSeparator(FGtkObject);
end;


 { TFPgtkHSeparator }

function TFPgtkHSeparator.TheGtkObject : PGtkHSeparator;
begin
  result := PgtkHSeparator(FGtkObject);
end;

procedure TFPgtkHSeparator.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_HSeparator_new);
end;


 { TFPgtkVSeparator }

function TFPgtkVSeparator.TheGtkObject : PGtkVSeparator;
begin
  result := PgtkVSeparator(FGtkObject);
end;

procedure TFPgtkVSeparator.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_VSeparator_new);
end;


 { TFPgtkPreview }

function TFPgtkPreview.TheGtkObject : PGtkPreview;
begin
  result := PgtkPreview(FGtkObject);
end;

procedure TFPgtkPreview.CreateGtkObject;
begin
  FGtkObject := PGtkObject(gtk_preview_new (GTK_PREVIEW_COLOR));
end;


procedure TFPgtkPreview.Size (aWidth:integer; aHeight:integer);
begin
  gtk_preview_size (TheGtkObject, aWidth, aHeight);
end;

procedure TFPgtkPreview.Put (aWindow:PGdkWindow; gc:PGdkGC; SrcX:integer; SrcY:integer; destX:integer; DestY:integer; aWidth:integer; aHeight:integer);
begin
  gtk_preview_put (TheGtkObject, aWindow, gc, SrcX, SrcY, destX, DestY, aWidth, aHeight);
end;

procedure TFPgtkPreview.DrawRow (data:pguchar; X:integer; Y:integer; W:integer);
begin
  gtk_preview_draw_row (TheGtkObject, data, X, Y, W);
end;

procedure SetGamma (aGamma:double);
begin
  gtk_preview_set_gamma (aGamma);
end;


function TFPgtkPreview.GetExpand : longbool;
begin
  result := longbool(gtk.expand(TheGtkObject^));
end;

procedure TFPgtkPreview.SetExpand (TheValue:longbool);
begin
  gtk_preview_set_expand(TheGtkObject,gint(TheValue));
end;

function TFPgtkPreview.GetDither : TGdkRgbDither;
begin
  result := TheGtkObject^.dither;
end;

procedure TFPgtkPreview.SetDither (TheValue:TGdkRgbDither);
begin
  gtk_preview_set_dither(TheGtkObject,TheValue);
end;

 { TFPgtkProgress }

function TFPgtkProgress.TheGtkObject : PGtkProgress;
begin
  result := PgtkProgress(FGtkObject);
end;


function TFPgtkProgress.GetShowtext : longbool;
begin
  result := longbool(gtk.show_text(TheGtkObject^));
end;

procedure TFPgtkProgress.SetShowtext (TheValue:longbool);
begin
  gtk_progress_set_show_text(TheGtkObject,gint(TheValue));
end;

function TFPgtkProgress.GetTextXAlign : gfloat;
begin
  result := TheGtkObject^.x_align;
end;

procedure TFPgtkProgress.SetTextXAlign (TheValue:gfloat);
begin
  gtk_progress_set_text_alignment (TheGtkObject, TheValue, TextYAlign);
end;

function TFPgtkProgress.GetTextYAlign : gfloat;
begin
  result := TheGtkObject^.y_align;
end;

procedure TFPgtkProgress.SetTextYAlign (TheValue:gfloat);
begin
  gtk_progress_set_text_alignment (TheGtkObject, TextXAlign, TheValue);
end;

procedure TFPgtkProgress.SetTextAlignment (anXalign:gfloat; anYAlign:gfloat);
begin
  gtk_progress_set_text_alignment (TheGtkObject, anXalign, anYAlign);
end;

function TFPgtkProgress.GetCurrentValue : float;
begin
  result := gtk_progress_get_Value(TheGtkObject);
end;

procedure TFPgtkProgress.SetCurrentValue (TheValue:float);
begin
  gtk_progress_Set_value (TheGtkObject, TheValue);
  Draw (nil);
end;

function TFPgtkProgress.GetPercentage : float;
begin
  result := gtk_progress_get_current_percentage(TheGtkObject);
end;

procedure TFPgtkProgress.SetPercentage (TheValue:float);
begin
  gtk_progress_set_percentage(TheGtkObject,TheValue);
end;

function TFPgtkProgress.PercentageFromValue (aValue:gfloat) : gfloat;
begin
  result := gtk_progress_get_percentage_from_value (TheGtkObject, aValue);
end;

function TFPgtkProgress.GetFormatString : string;
begin
  result := TheGtkObject^.format;
end;

procedure TFPgtkProgress.SetFormatString (TheValue:string);
begin
  gtk_progress_set_format_string(TheGtkObject,ConvertToPgchar(TheValue));
end;

function TFPgtkProgress.GetAdjustment : TFPgtkAdjustment;
begin
  result := GetPascalInstance (PGtkObject(TheGtkObject^.adjustment), TFPgtkAdjustment) as TFPgtkAdjustment;
end;

procedure TFPgtkProgress.SetAdjustment (TheValue:TFPgtkAdjustment);
begin
  gtk_progress_set_adjustment(TheGtkObject,PGtkadjustment(ConvertToGtkObject(TheValue)));
end;

function TFPgtkProgress.GetActivityMode : longbool;
begin
  result := longbool(gtk.activity_mode(TheGtkObject^));
end;

procedure TFPgtkProgress.SetActivityMode (TheValue:longbool);
begin
  gtk_progress_set_activity_mode(TheGtkObject,gint(TheValue));
end;

function TFPgtkProgress.CurrentText : string;
begin
  result := gtk_progress_get_current_text (TheGtkObject);
end;

function TFPgtkProgress.TextFromValue (aValue:gfloat) : string;
begin
  result := gtk_progress_get_text_from_value (TheGtkObject, aValue);
end;

procedure TFPgtkProgress.Configure (aValue:gfloat; aMin:gfloat; aMax:gfloat);
begin
  gtk_progress_configure (TheGtkObject, aValue, aMin, aMax);
end;

 { TFPgtkProgressBar }

function TFPgtkProgressBar.TheGtkObject : PGtkProgressBar;
begin
  result := PgtkProgressBar(FGtkObject);
end;


constructor TFPgtkProgressBar.Create (adj:TFPgtkAdjustment);
begin
  FAdj := adj;
  inherited create;
end;


procedure TFPgtkProgressBar.CreateGtkObject;
begin
  if assigned (FAdj) then
    TheGtkWidget := gtk_progress_bar_new_with_adjustment (FAdj.TheGtkObject)
  else
    TheGtkWidget := gtk_progress_bar_new;
end;

function TFPgtkProgressBar.GetBarStyle : TGtkProgressBarStyle;
begin
  result := TheGtkObject^.bar_style;
end;

procedure TFPgtkProgressBar.SetBarStyle (TheValue:TGtkProgressBarStyle);
begin
  gtk_progress_bar_set_bar_style(TheGtkObject,TheValue);
end;

function TFPgtkProgressBar.GetDiscreteBlocks : longword;
begin
  result := TheGtkObject^.blocks;
end;

procedure TFPgtkProgressBar.SetDiscreteBlocks (TheValue:longword);
begin
  gtk_progress_bar_set_discrete_blocks(TheGtkObject,TheValue);
end;

function TFPgtkProgressBar.GetActivityStep : longword;
begin
  result := TheGtkObject^.activity_step;
end;

procedure TFPgtkProgressBar.SetActivityStep (TheValue:longword);
begin
  gtk_progress_bar_set_activity_step(TheGtkObject,TheValue);
end;

function TFPgtkProgressBar.GetActivityBlocks : longword;
begin
  result := TheGtkObject^.activity_blocks;
end;

procedure TFPgtkProgressBar.SetActivityBlocks (TheValue:longword);
begin
  gtk_progress_bar_set_activity_blocks(TheGtkObject,TheValue);
end;

function TFPgtkProgressBar.GetOrientation : TGtkProgressBarOrientation;
begin
  result := TheGtkObject^.orientation;
end;

procedure TFPgtkProgressBar.SetOrientation (TheValue:TGtkProgressBarOrientation);
begin
  gtk_progress_bar_set_orientation(TheGtkObject,TheValue);
end;

 { TFPgtkItemFactory }


INITIALIZATION
ObjectsToFree := TList.Create;

FINALIZATION
ObjectsToFree.Free;
if assigned (TheTooltips) then
  TheTooltips.Free;

End.
