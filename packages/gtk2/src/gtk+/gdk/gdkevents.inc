// included by gdk2.pp

{$IFDEF read_interface_types}
   PGdkEvent = ^TGdkEvent;

   TGdkEventFunc = procedure (event:PGdkEvent; data:gpointer); cdecl;
{ Event filtering  }

   PGdkXEvent = ^TGdkXEvent;
   TGdkXEvent = procedure;
{ Can be cast to window system specific
                             even type, XEvent on X11, MSG on Win32.
                            }
{ Event not handled, continue processesing  }
{ Translated event stored  }
{ Terminate processing, removing event  }

   PGdkFilterReturn = ^TGdkFilterReturn;
   TGdkFilterReturn = (
     GDK_FILTER_CONTINUE,
     GDK_FILTER_TRANSLATE,
     GDK_FILTER_REMOVE
   );

   TGdkFilterFunc = function (xevent:PGdkXEvent; event:PGdkEvent;
                              data:gpointer):TGdkFilterReturn; cdecl;
{ Event types.
     Nothing: No event occurred.
     Delete: A window delete event was sent by the window manager.
             The specified window should be deleted.
     Destroy: A window has been destroyed.
     Expose: Part of a window has been uncovered.
     NoExpose: Same as expose, but no expose event was generated.
     VisibilityNotify: A window has become fully/partially/not obscured.
     MotionNotify: The mouse has moved.
     ButtonPress: A mouse button was pressed.
     ButtonRelease: A mouse button was release.
     KeyPress: A key was pressed.
     KeyRelease: A key was released.
     EnterNotify: A window was entered.
     LeaveNotify: A window was exited.
     FocusChange: The focus window has changed. (The focus window gets
                  keyboard events).
     Resize: A window has been resized.
     Map: A window has been mapped. (It is now visible on the screen).
     Unmap: A window has been unmapped. (It is no longer visible on
            the screen).
     Scroll: A mouse wheel was scrolled either up or down.
  }

   PGdkEventType = ^TGdkEventType;
   TGdkEventType = gint;

{ Event masks. (Used to select what types of events a window
    will receive).
  }

   PGdkEventMask = ^TGdkEventMask;
   TGdkEventMask = gint32;

   PGdkVisibilityState = ^TGdkVisibilityState;
   TGdkVisibilityState = (
     GDK_VISIBILITY_UNOBSCURED,
     GDK_VISIBILITY_PARTIAL,
     GDK_VISIBILITY_FULLY_OBSCURED
   );

   PGdkScrollDirection = ^TGdkScrollDirection;
   TGdkScrollDirection = (
     GDK_SCROLL_UP,
     GDK_SCROLL_DOWN,
     GDK_SCROLL_LEFT,
     GDK_SCROLL_RIGHT
   );

{ Types of enter/leave notifications.
     Ancestor:
     Virtual:
     Inferior:
     Nonlinear:
     NonlinearVirtual:
     Unknown: An unknown type of enter/leave event occurred.
  }

   PGdkNotifyType = ^TGdkNotifyType;
   TGdkNotifyType = integer; // see below for possible values

{ Enter/leave event modes.
     NotifyNormal
     NotifyGrab
     NotifyUngrab
  }

   PGdkCrossingMode = ^TGdkCrossingMode;
   TGdkCrossingMode = (
     GDK_CROSSING_NORMAL,
     GDK_CROSSING_GRAB,
     GDK_CROSSING_UNGRAB
   );

   PGdkPropertyState = ^TGdkPropertyState;
   TGdkPropertyState = (
     GDK_PROPERTY_NEW_VALUE,
     GDK_PROPERTY_STATE_DELETE // was GDK_PROPERTY_DELETE, but there is also a function
   );

   PGdkWindowState = ^TGdkWindowState;
   TGdkWindowState =  gint;


   PGdkSettingAction = ^TGdkSettingAction;
   TGdkSettingAction = (
     GDK_SETTING_ACTION_NEW,
     GDK_SETTING_ACTION_CHANGED,
     GDK_SETTING_ACTION_DELETED
   );

   PGdkOwnerChange = ^TGdkOwnerChange;
   TGdkOwnerChange = (
     GDK_OWNER_CHANGE_NEW_OWNER,
     GDK_OWNER_CHANGE_DESTROY,
     GDK_OWNER_CHANGE_CLOSE
   );

   PGdkEventAny = ^TGdkEventAny;
   TGdkEventAny = record
        _type : TGdkEventType;
        window : PGdkWindow;
        send_event : gint8;
     end;

{ If non-zero, how many more events follow.  }
   PGdkEventExpose = ^TGdkEventExpose;
   TGdkEventExpose = record
        _type : TGdkEventType;
        window : PGdkWindow;
        send_event : gint8;
        area : TGdkRectangle;
        region : PGdkRegion;
        count : gint;
     end;

{ XXX: does anyone need the X major_code or minor_code fields?  }
   PGdkEventNoExpose = ^TGdkEventNoExpose;
   TGdkEventNoExpose = record
        _type : TGdkEventType;
        window : PGdkWindow;
        send_event : gint8;
     end;

   PGdkEventVisibility = ^TGdkEventVisibility;
   TGdkEventVisibility = record
        _type : TGdkEventType;
        window : PGdkWindow;
        send_event : gint8;
        state : TGdkVisibilityState;
     end;

   PGdkEventMotion = ^TGdkEventMotion;
   TGdkEventMotion = record
        _type : TGdkEventType;
        window : PGdkWindow;
        send_event : gint8;
        time : guint32;
        x : gdouble;
        y : gdouble;
        axes : Pgdouble;
        state : guint;
        is_hint : gint16;
        device : PGdkDevice;
        x_root : gdouble;
        y_root : gdouble;
     end;

   PGdkEventButton = ^TGdkEventButton;
   TGdkEventButton = record
        _type : TGdkEventType;
        window : PGdkWindow;
        send_event : gint8;
        time : guint32;
        x : gdouble;
        y : gdouble;
        axes : Pgdouble;
        state : guint;
        button : guint;
        device : PGdkDevice;
        x_root : gdouble;
        y_root : gdouble;
     end;

   PGdkEventScroll = ^TGdkEventScroll;
   TGdkEventScroll = record
        _type : TGdkEventType;
        window : PGdkWindow;
        send_event : gint8;
        time : guint32;
        x : gdouble;
        y : gdouble;
        state : guint;
        direction : TGdkScrollDirection;
        device : PGdkDevice;
        x_root : gdouble;
        y_root : gdouble;
     end;

   PGdkEventKey = ^TGdkEventKey;
   TGdkEventKey = record
        _type : TGdkEventType;
        window : PGdkWindow;
        send_event : gint8;
        time : guint32;
        state : guint;
        keyval : guint;
        length : gint;
        _string : Pgchar;
        hardware_keycode : guint16;
        group : guint8;
     end;

   PGdkEventCrossing = ^TGdkEventCrossing;
   TGdkEventCrossing = record
        _type : TGdkEventType;
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
        _type : TGdkEventType;
        window : PGdkWindow;
        send_event : gint8;
        _in : gint16;
     end;

   PGdkEventConfigure = ^TGdkEventConfigure;
   TGdkEventConfigure = record
        _type : TGdkEventType;
        window : PGdkWindow;
        send_event : gint8;
        x : gint;
        y : gint;
        width : gint;
        height : gint;
     end;

   PGdkEventProperty = ^TGdkEventProperty;
   TGdkEventProperty = record
        _type : TGdkEventType;
        window : PGdkWindow;
        send_event : gint8;
        atom : TGdkAtom;
        time : guint32;
        state : guint;
     end;

   PGdkEventSelection = ^TGdkEventSelection;
   TGdkEventSelection = record
        _type : TGdkEventType;
        window : PGdkWindow;
        send_event : gint8;
        selection : TGdkAtom;
        target : TGdkAtom;
        _property : TGdkAtom;
        time : guint32;
        requestor : TGdkNativeWindow;
     end;

{ This event type will be used pretty rarely. It only is important
   for XInput aware programs that are drawing their own cursor  }
   PGdkEventProximity = ^TGdkEventProximity;
   TGdkEventProximity = record
        _type : TGdkEventType;
        window : PGdkWindow;
        send_event : gint8;
        time : guint32;
        device : PGdkDevice;
     end;

   PmatDUMMY = ^TmatDUMMY;
   TmatDUMMY = record
       case longint of
          0 : ( b : array[0..19] of char );
          1 : ( s : array[0..9] of smallint );
          2 : ( li : array[0..4] of longint );
       end;

   PGdkEventClient = ^TGdkEventClient;
   TGdkEventClient = record
        _type : TGdkEventType;
        window : PGdkWindow;
        send_event : gint8;
        message_type : TGdkAtom;
        data_format : gushort;
        data : record
            case longint of
               0 : ( b : array[0..19] of char );
               1 : ( s : array[0..9] of smallint );
               2 : ( l : array[0..4] of longint );
            end;
     end;

   PGdkEventSetting = ^TGdkEventSetting;
   TGdkEventSetting = record
        _type : TGdkEventType;
        window : PGdkWindow;
        send_event : gint8;
        action : TGdkSettingAction;
        name : Pchar;
     end;

   PGdkEventOwnerChange = ^TGdkEventOwnerChange;
   TGdkEventOwnerChange = record
        _type : TGdkEventType;
        window : PGdkWindow;
        send_event : gint8;
        owner : TGdkNativeWindow;
        reason : TGdkOwnerChange;
        selection : TGdkAtom;
        time : guint32;
        selection_time : guint32;
     end;

   PGdkEventWindowState = ^TGdkEventWindowState;
   TGdkEventWindowState = record
        _type : TGdkEventType;
        window : PGdkWindow;
        send_event : gint8;
        changed_mask : TGdkWindowState;
        new_window_state : TGdkWindowState;
     end;

{ Event types for DND  }
   PGdkEventDND = ^TGdkEventDND;
   TGdkEventDND = record
        _type : TGdkEventType;
        window : PGdkWindow;
        send_event : gint8;
        context : PGdkDragContext;
        time : guint32;
        x_root : gshort;
        y_root : gshort;
     end;

   TGdkEvent = record
       case longint of
          0 : ( _type : TGdkEventType );
          1 : ( any : TGdkEventAny );
          2 : ( expose : TGdkEventExpose );
          3 : ( no_expose : TGdkEventNoExpose );
          4 : ( visibility : TGdkEventVisibility );
          5 : ( motion : TGdkEventMotion );
          6 : ( button : TGdkEventButton );
          7 : ( scroll : TGdkEventScroll );
          8 : ( key : TGdkEventKey );
          9 : ( crossing : TGdkEventCrossing );
          10 : ( focus_change : TGdkEventFocus );
          11 : ( configure : TGdkEventConfigure );
          12 : ( _property : TGdkEventProperty );
          13 : ( selection : TGdkEventSelection );
          14 : ( proximity : TGdkEventProximity );
          15 : ( client : TGdkEventClient );
          16 : ( dnd : TGdkEventDND );
          17 : ( window_state : TGdkEventWindowState );
          18 : ( setting : TGdkEventSetting );
       end;

{$ENDIF read_interface_types}

//------------------------------------------------------------------------------

{$IFDEF read_interface_rest}

const
     GDK_NOTHING = -(1);
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
     GDK_DRAG_MOTION_EVENT = 24; // was GDK_DRAG_MOTION, which bites gdk_drag_motion
     GDK_DRAG_STATUS_EVENT = 25; // was GDK_DRAG_STATUS, which bites gdk_drag_status
     GDK_DROP_START = 26;
     GDK_DROP_FINISHED = 27;
     GDK_CLIENT_EVENT = 28;
     GDK_VISIBILITY_NOTIFY = 29;
     GDK_NO_EXPOSE = 30;
     GDK_SCROLL = 31;
     GDK_WINDOW_STATE = 32;
     GDK_SETTING = 33;

     //  values for TGdkNotifyType
     GDK_NOTIFY_ANCESTOR = 0;
     GDK_NOTIFY_VIRTUAL = 1;
     GDK_NOTIFY_INFERIOR = 2;
     GDK_NOTIFY_NONLINEAR = 3;
     GDK_NOTIFY_NONLINEAR_VIRTUAL = 4;
     GDK_NOTIFY_UNKNOWN = 5;


function GDK_TYPE_EVENT : GType;

const
   GDK_PRIORITY_EVENTS = G_PRIORITY_DEFAULT;
   GDK_PRIORITY_REDRAW = G_PRIORITY_HIGH_IDLE + 20;

   GDK_EXPOSURE_MASK            = 1 shl 1;
   GDK_POINTER_MOTION_MASK      = 1 shl 2;
   GDK_POINTER_MOTION_HINT_MASK = 1 shl 3;
   GDK_BUTTON_MOTION_MASK       = 1 shl 4;
   GDK_BUTTON1_MOTION_MASK      = 1 shl 5;
   GDK_BUTTON2_MOTION_MASK      = 1 shl 6;
   GDK_BUTTON3_MOTION_MASK      = 1 shl 7;
   GDK_BUTTON_PRESS_MASK        = 1 shl 8;
   GDK_BUTTON_RELEASE_MASK      = 1 shl 9;
   GDK_KEY_PRESS_MASK           = 1 shl 10;
   GDK_KEY_RELEASE_MASK         = 1 shl 11;
   GDK_ENTER_NOTIFY_MASK        = 1 shl 12;
   GDK_LEAVE_NOTIFY_MASK        = 1 shl 13;
   GDK_FOCUS_CHANGE_MASK        = 1 shl 14;
   GDK_STRUCTURE_MASK           = 1 shl 15;
   GDK_PROPERTY_CHANGE_MASK     = 1 shl 16;
   GDK_VISIBILITY_NOTIFY_MASK   = 1 shl 17;
   GDK_PROXIMITY_IN_MASK        = 1 shl 18;
   GDK_PROXIMITY_OUT_MASK       = 1 shl 19;
   GDK_SUBSTRUCTURE_MASK        = 1 shl 20;
   GDK_SCROLL_MASK              = 1 shl 21;
   GDK_ALL_EVENTS_MASK          = $3FFFFE;

<<<<<<< HEAD
<<<<<<< HEAD
   GDK_WINDOW_STATE_WITHDRAWN  = 1 shl 0;
   GDK_WINDOW_STATE_ICONIFIED  = 1 shl 1;
   GDK_WINDOW_STATE_MAXIMIZED  = 1 shl 2;
   GDK_WINDOW_STATE_STICKY     = 1 shl 3;
   GDK_WINDOW_STATE_FULLSCREEN = 1 shl 4;
   GDK_WINDOW_STATE_ABOVE      = 1 shl 5;
   GDK_WINDOW_STATE_BELOW      = 1 shl 6;
=======
=======
>>>>>>> origin/fixes_2_2
   GDK_WINDOW_STATE_WITHDRAWN = 1 shl 0;
   GDK_WINDOW_STATE_ICONIFIED = 1 shl 1;
   GDK_WINDOW_STATE_MAXIMIZED = 1 shl 2;
   GDK_WINDOW_STATE_STICKY    = 1 shl 3;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

function gdk_event_get_type:GType; cdecl; external gdklib;
function gdk_events_pending:gboolean; cdecl; external gdklib;
function gdk_event_get:PGdkEvent; cdecl; external gdklib;
function gdk_event_peek:PGdkEvent; cdecl; external gdklib;
function gdk_event_get_graphics_expose(window:PGdkWindow):PGdkEvent; cdecl; external gdklib;
procedure gdk_event_put(event:PGdkEvent); cdecl; external gdklib;
function gdk_event_copy(event:PGdkEvent):PGdkEvent; cdecl; external gdklib;
procedure gdk_event_free(event:PGdkEvent); cdecl; external gdklib;
function gdk_event_get_time(event:PGdkEvent):guint32; cdecl; external gdklib;
function gdk_event_get_state(event:PGdkEvent; state:PGdkModifierType):gboolean; cdecl; external gdklib;
function gdk_event_get_coords(event:PGdkEvent; x_win:Pgdouble; y_win:Pgdouble):gboolean; cdecl; external gdklib;
function gdk_event_get_root_coords(event:PGdkEvent; x_root:Pgdouble; y_root:Pgdouble):gboolean; cdecl; external gdklib;
function gdk_event_get_axis(event:PGdkEvent; axis_use:TGdkAxisUse; value:Pgdouble):gboolean; cdecl; external gdklib;
procedure gdk_event_handler_set(func:TGdkEventFunc; data:gpointer; notify:TGDestroyNotify); cdecl; external gdklib;
procedure gdk_set_show_events(show_events:gboolean); cdecl; external gdklib;
function gdk_get_show_events:gboolean; cdecl; external gdklib;
function gdk_event_get_screen(event:PGdkEvent): PGdkScreen; cdecl; external gdklib;
function gdk_event_new(_type: TGdkEventType): PGdkEvent; cdecl; external gdklib;
function gdk_event_send_client_message_for_display(display:PGdkDisplay; event:PGdkEvent; xid:guint32):gboolean; cdecl; external gdklib;
procedure gdk_event_set_screen(event: PGdkEvent; screen: PGdkScreen); cdecl; external gdklib;

{$ifndef GDK_MULTIHEAD_SAFE}
procedure gdk_add_client_message_filter(message_type:TGdkAtom; func:TGdkFilterFunc; data:gpointer); cdecl; external gdklib;
function gdk_setting_get(name:Pgchar; var value:TGValue):gboolean; cdecl; external gdklib;
{$endif}
{ GDK_MULTIHEAD_SAFE  }

{$endif read_interface_rest}

//------------------------------------------------------------------------------

{$IFDEF read_implementation}
function GDK_TYPE_EVENT : GType;
begin
  GDK_TYPE_EVENT:=gdk_event_get_type;
end;
{$ENDIF}

