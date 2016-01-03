unit xlib;
interface
{$mode objfpc}
uses
  ctypes,x;
{$define MACROS}

{$ifndef os2}
  {$LinkLib c}
  {$LinkLib X11}
const
  libX11='X11';
{$else}
const
  libX11='X11';
{$endif}

type
  PPcint = ^Pcint;
  PPcuchar = ^Pcuchar;
{
  Automatically converted by H2Pas 0.99.15 from xlib.h
  The following command line parameters were used:
    -p
    -T
    -S
    -d
    -c
    xlib.h
}

{$PACKRECORDS C}


const
   XlibSpecificationRelease = 6;
type

   PXPointer = ^TXPointer;
   TXPointer = ^char;
   PBool = ^TBool;
<<<<<<< HEAD
<<<<<<< HEAD
   { We cannot use TBool = LongBool, because Longbool(True)=-1, 
     and that is not handled well by X. So we leave TBool=cint; 
     and make overloaded calls with a boolean parameter. 
     For function results, longbool is OK, since everything <>0 
     is interpreted as true, so we introduce TBoolResult. }
   TBool = cint;
   TBoolResult = longbool;
=======
   TBool = longbool; {cint?}
>>>>>>> graemeg/fixes_2_2
=======
   TBool = longbool; {cint?}
>>>>>>> origin/fixes_2_2
   PStatus = ^TStatus;
   TStatus = cint;

const
   QueuedAlready = 0;
   QueuedAfterReading = 1;
   QueuedAfterFlush = 2;

type

   PPXExtData = ^PXExtData;
   PXExtData = ^TXExtData;
   TXExtData = record
        number : cint;
        next : PXExtData;
        free_private : function (extension:PXExtData):cint;cdecl;
        private_data : TXPointer;
     end;

   PXExtCodes = ^TXExtCodes;
   TXExtCodes = record
        extension : cint;
        major_opcode : cint;
        first_event : cint;
        first_error : cint;
     end;

   PXPixmapFormatValues = ^TXPixmapFormatValues;
   TXPixmapFormatValues = record
        depth : cint;
        bits_per_pixel : cint;
        scanline_pad : cint;
     end;

   PXGCValues = ^TXGCValues;
   TXGCValues = record
        _function : cint;
        plane_mask : culong;
        foreground : culong;
        background : culong;
        line_width : cint;
        line_style : cint;
        cap_style : cint;
        join_style : cint;
        fill_style : cint;
        fill_rule : cint;
        arc_mode : cint;
        tile : TPixmap;
        stipple : TPixmap;
        ts_x_origin : cint;
        ts_y_origin : cint;
        font : TFont;
        subwindow_mode : cint;
        graphics_exposures : TBool;
        clip_x_origin : cint;
        clip_y_origin : cint;
        clip_mask : TPixmap;
        dash_offset : cint;
        dashes : cchar;
     end;

   PXGC = ^TXGC;
   TXGC = record
     end;
   TGC = PXGC;
   PGC = ^TGC;

   PVisual = ^TVisual;
   TVisual = record
        ext_data : PXExtData;
        visualid : TVisualID;
        c_class : cint;
        red_mask, green_mask, blue_mask : culong;
        bits_per_rgb : cint;
        map_entries : cint;
     end;

   PDepth = ^TDepth;
   TDepth = record
        depth : cint;
        nvisuals : cint;
        visuals : PVisual;
     end;
   PXDisplay = ^TXDisplay;
   TXDisplay = record
     end;


   PScreen = ^TScreen;
   TScreen = record
        ext_data : PXExtData;
        display : PXDisplay;
        root : TWindow;
        width, height : cint;
        mwidth, mheight : cint;
        ndepths : cint;
        depths : PDepth;
        root_depth : cint;
        root_visual : PVisual;
        default_gc : TGC;
        cmap : TColormap;
        white_pixel : culong;
        black_pixel : culong;
        max_maps, min_maps : cint;
        backing_store : cint;
        save_unders : TBool;
        root_input_mask : clong;
     end;

   PScreenFormat = ^TScreenFormat;
   TScreenFormat = record
        ext_data : PXExtData;
        depth : cint;
        bits_per_pixel : cint;
        scanline_pad : cint;
     end;

   PXSetWindowAttributes = ^TXSetWindowAttributes;
   TXSetWindowAttributes = record
        background_pixmap : TPixmap;
        background_pixel : culong;
        border_pixmap : TPixmap;
        border_pixel : culong;
        bit_gravity : cint;
        win_gravity : cint;
        backing_store : cint;
        backing_planes : culong;
        backing_pixel : culong;
        save_under : TBool;
        event_mask : clong;
        do_not_propagate_mask : clong;
        override_redirect : TBool;
        colormap : TColormap;
        cursor : TCursor;
     end;

   PXWindowAttributes = ^TXWindowAttributes;
   TXWindowAttributes = record
        x, y : cint;
        width, height : cint;
        border_width : cint;
        depth : cint;
        visual : PVisual;
        root : TWindow;
        c_class : cint;
        bit_gravity : cint;
        win_gravity : cint;
        backing_store : cint;
        backing_planes : culong;
        backing_pixel : culong;
        save_under : TBool;
        colormap : TColormap;
        map_installed : TBool;
        map_state : cint;
        all_event_masks : clong;
        your_event_mask : clong;
        do_not_propagate_mask : clong;
        override_redirect : TBool;
        screen : PScreen;
     end;

   PXHostAddress = ^TXHostAddress;
   TXHostAddress = record
        family : cint;
        length : cint;
        address : Pchar;
     end;

   PXServerInterpretedAddress = ^TXServerInterpretedAddress;
   TXServerInterpretedAddress = record
        typelength : cint;
        valuelength : cint;
        _type : Pchar;
        value : Pchar;
     end;

   PXImage = ^TXImage;
   TXImage = record
        width, height : cint;
        xoffset : cint;
        format : cint;
        data : Pchar;
        byte_order : cint;
        bitmap_unit : cint;
        bitmap_bit_order : cint;
        bitmap_pad : cint;
        depth : cint;
        bytes_per_line : cint;
        bits_per_pixel : cint;
        red_mask : culong;
        green_mask : culong;
        blue_mask : culong;
        obdata : TXPointer;
        f : record
             create_image : function (para1:PXDisplay; para2:PVisual; para3:cuint; para4:cint; para5:cint;
                          para6:Pchar; para7:cuint; para8:cuint; para9:cint; para10:cint):PXImage;cdecl;
             destroy_image : function (para1:PXImage):cint;cdecl;
             get_pixel : function (para1:PXImage; para2:cint; para3:cint):culong;cdecl;
             put_pixel : function (para1:PXImage; para2:cint; para3:cint; para4:culong):cint;cdecl;
             sub_image : function (para1:PXImage; para2:cint; para3:cint; para4:cuint; para5:cuint):PXImage;cdecl;
             add_pixel : function (para1:PXImage; para2:clong):cint;cdecl;
          end;
     end;

   PXWindowChanges = ^TXWindowChanges;
   TXWindowChanges = record
        x, y : cint;
        width, height : cint;
        border_width : cint;
        sibling : TWindow;
        stack_mode : cint;
     end;

   PXColor = ^TXColor;
   TXColor = record
        pixel : culong;
        red, green, blue : cushort;
        flags : cchar;
        pad : cchar;
     end;

   PXSegment = ^TXSegment;
   TXSegment = record
        x1, y1, x2, y2 : cshort;
     end;

   PXPoint = ^TXPoint;
   TXPoint = record
        x, y : cshort;
     end;

   PXRectangle = ^TXRectangle;
   TXRectangle = record
        x, y : cshort;
        width, height : cushort;
     end;

   PXArc = ^TXArc;
   TXArc = record
        x, y : cshort;
        width, height : cushort;
        angle1, angle2 : cshort;
     end;

   PXKeyboardControl = ^TXKeyboardControl;
   TXKeyboardControl = record
        key_click_percent : cint;
        bell_percent : cint;
        bell_pitch : cint;
        bell_duration : cint;
        led : cint;
        led_mode : cint;
        key : cint;
        auto_repeat_mode : cint;
     end;

   PXKeyboardState = ^TXKeyboardState;
   TXKeyboardState = record
        key_click_percent : cint;
        bell_percent : cint;
        bell_pitch, bell_duration : cuint;
        led_mask : culong;
        global_auto_repeat : cint;
        auto_repeats : array[0..31] of cchar;
     end;

   PXTimeCoord = ^TXTimeCoord;
   TXTimeCoord = record
        time : TTime;
        x, y : cshort;
     end;

   PXModifierKeymap = ^TXModifierKeymap;
   TXModifierKeymap = record
        max_keypermod : cint;
        modifiermap : PKeyCode;
     end;

   PDisplay = ^TDisplay;
   TDisplay = TXDisplay;

   PXPrivate = ^TXPrivate;
   TXPrivate = record
     end;

   PXrmHashBucketRec = ^TXrmHashBucketRec;
   TXrmHashBucketRec = record
     end;


   PXPrivDisplay = ^TXPrivDisplay;
   TXPrivDisplay = record
        ext_data : PXExtData;
        private1 : PXPrivate;
        fd : cint;
        private2 : cint;
        proto_major_version : cint;
        proto_minor_version : cint;
        vendor : Pchar;
        private3 : TXID;
        private4 : TXID;
        private5 : TXID;
        private6 : cint;
        resource_alloc : function (para1:PXDisplay):TXID;cdecl;
        byte_order : cint;
        bitmap_unit : cint;
        bitmap_pad : cint;
        bitmap_bit_order : cint;
        nformats : cint;
        pixmap_format : PScreenFormat;
        private8 : cint;
        release : cint;
        private9, private10 : PXPrivate;
        qlen : cint;
        last_request_read : culong;
        request : culong;
        private11 : TXPointer;
        private12 : TXPointer;
        private13 : TXPointer;
        private14 : TXPointer;
        max_request_size : cunsigned;
        db : PXrmHashBucketRec;
        private15 : function (para1:PXDisplay):cint;cdecl;
        display_name : Pchar;
        default_screen : cint;
        nscreens : cint;
        screens : PScreen;
        motion_buffer : culong;
        private16 : culong;
        min_keycode : cint;
        max_keycode : cint;
        private17 : TXPointer;
        private18 : TXPointer;
        private19 : cint;
        xdefaults : Pchar;
     end;

   PXKeyEvent = ^TXKeyEvent;
   TXKeyEvent = record
        _type : cint;
        serial : culong;
        send_event : TBool;
        display : PDisplay;
        window : TWindow;
        root : TWindow;
        subwindow : TWindow;
        time : TTime;
        x, y : cint;
        x_root, y_root : cint;
        state : cuint;
        keycode : cuint;
        same_screen : TBool;
     end;

   PXKeyPressedEvent = ^TXKeyPressedEvent;
   TXKeyPressedEvent = TXKeyEvent;

   PXKeyReleasedEvent = ^TXKeyReleasedEvent;
   TXKeyReleasedEvent = TXKeyEvent;

   PXButtonEvent = ^TXButtonEvent;
   TXButtonEvent = record
        _type : cint;
        serial : culong;
        send_event : TBool;
        display : PDisplay;
        window : TWindow;
        root : TWindow;
        subwindow : TWindow;
        time : TTime;
        x, y : cint;
        x_root, y_root : cint;
        state : cuint;
        button : cuint;
        same_screen : TBool;
     end;

   PXButtonPressedEvent = ^TXButtonPressedEvent;
   TXButtonPressedEvent = TXButtonEvent;

   PXButtonReleasedEvent = ^TXButtonReleasedEvent;
   TXButtonReleasedEvent = TXButtonEvent;

   PXMotionEvent = ^TXMotionEvent;
   TXMotionEvent = record
        _type : cint;
        serial : culong;
        send_event : TBool;
        display : PDisplay;
        window : TWindow;
        root : TWindow;
        subwindow : TWindow;
        time : TTime;
        x, y : cint;
        x_root, y_root : cint;
        state : cuint;
        is_hint : cchar;
        same_screen : TBool;
     end;

   PXPointerMovedEvent = ^TXPointerMovedEvent;
   TXPointerMovedEvent = TXMotionEvent;

   PXCrossingEvent = ^TXCrossingEvent;
   TXCrossingEvent = record
        _type : cint;
        serial : culong;
        send_event : TBool;
        display : PDisplay;
        window : TWindow;
        root : TWindow;
        subwindow : TWindow;
        time : TTime;
        x, y : cint;
        x_root, y_root : cint;
        mode : cint;
        detail : cint;
        same_screen : TBool;
        focus : TBool;
        state : cuint;
     end;

   PXEnterWindowEvent = ^TXEnterWindowEvent;
   TXEnterWindowEvent = TXCrossingEvent;

   PXLeaveWindowEvent = ^TXLeaveWindowEvent;
   TXLeaveWindowEvent = TXCrossingEvent;

   PXFocusChangeEvent = ^TXFocusChangeEvent;
   TXFocusChangeEvent = record
        _type : cint;
        serial : culong;
        send_event : TBool;
        display : PDisplay;
        window : TWindow;
        mode : cint;
        detail : cint;
     end;

   PXFocusInEvent = ^TXFocusInEvent;
   TXFocusInEvent = TXFocusChangeEvent;

   PXFocusOutEvent = ^TXFocusOutEvent;
   TXFocusOutEvent = TXFocusChangeEvent;

   PXKeymapEvent = ^TXKeymapEvent;
   TXKeymapEvent = record
        _type : cint;
        serial : culong;
        send_event : TBool;
        display : PDisplay;
        window : TWindow;
        key_vector : array[0..31] of cchar;
     end;

   PXExposeEvent = ^TXExposeEvent;
   TXExposeEvent = record
        _type : cint;
        serial : culong;
        send_event : TBool;
        display : PDisplay;
        window : TWindow;
        x, y : cint;
        width, height : cint;
        count : cint;
     end;

   PXGraphicsExposeEvent = ^TXGraphicsExposeEvent;
   TXGraphicsExposeEvent = record
        _type : cint;
        serial : culong;
        send_event : TBool;
        display : PDisplay;
        drawable : TDrawable;
        x, y : cint;
        width, height : cint;
        count : cint;
        major_code : cint;
        minor_code : cint;
     end;

   PXNoExposeEvent = ^TXNoExposeEvent;
   TXNoExposeEvent = record
        _type : cint;
        serial : culong;
        send_event : TBool;
        display : PDisplay;
        drawable : TDrawable;
        major_code : cint;
        minor_code : cint;
     end;

   PXVisibilityEvent = ^TXVisibilityEvent;
   TXVisibilityEvent = record
        _type : cint;
        serial : culong;
        send_event : TBool;
        display : PDisplay;
        window : TWindow;
        state : cint;
     end;

   PXCreateWindowEvent = ^TXCreateWindowEvent;
   TXCreateWindowEvent = record
        _type : cint;
        serial : culong;
        send_event : TBool;
        display : PDisplay;
        parent : TWindow;
        window : TWindow;
        x, y : cint;
        width, height : cint;
        border_width : cint;
        override_redirect : TBool;
     end;

   PXDestroyWindowEvent = ^TXDestroyWindowEvent;
   TXDestroyWindowEvent = record
        _type : cint;
        serial : culong;
        send_event : TBool;
        display : PDisplay;
        event : TWindow;
        window : TWindow;
     end;

   PXUnmapEvent = ^TXUnmapEvent;
   TXUnmapEvent = record
        _type : cint;
        serial : culong;
        send_event : TBool;
        display : PDisplay;
        event : TWindow;
        window : TWindow;
        from_configure : TBool;
     end;

   PXMapEvent = ^TXMapEvent;
   TXMapEvent = record
        _type : cint;
        serial : culong;
        send_event : TBool;
        display : PDisplay;
        event : TWindow;
        window : TWindow;
        override_redirect : TBool;
     end;

   PXMapRequestEvent = ^TXMapRequestEvent;
   TXMapRequestEvent = record
        _type : cint;
        serial : culong;
        send_event : TBool;
        display : PDisplay;
        parent : TWindow;
        window : TWindow;
     end;

   PXReparentEvent = ^TXReparentEvent;
   TXReparentEvent = record
        _type : cint;
        serial : culong;
        send_event : TBool;
        display : PDisplay;
        event : TWindow;
        window : TWindow;
        parent : TWindow;
        x, y : cint;
        override_redirect : TBool;
     end;

   PXConfigureEvent = ^TXConfigureEvent;
   TXConfigureEvent = record
        _type : cint;
        serial : culong;
        send_event : TBool;
        display : PDisplay;
        event : TWindow;
        window : TWindow;
        x, y : cint;
        width, height : cint;
        border_width : cint;
        above : TWindow;
        override_redirect : TBool;
     end;

   PXGravityEvent = ^TXGravityEvent;
   TXGravityEvent = record
        _type : cint;
        serial : culong;
        send_event : TBool;
        display : PDisplay;
        event : TWindow;
        window : TWindow;
        x, y : cint;
     end;

   PXResizeRequestEvent = ^TXResizeRequestEvent;
   TXResizeRequestEvent = record
        _type : cint;
        serial : culong;
        send_event : TBool;
        display : PDisplay;
        window : TWindow;
        width, height : cint;
     end;

   PXConfigureRequestEvent = ^TXConfigureRequestEvent;
   TXConfigureRequestEvent = record
        _type : cint;
        serial : culong;
        send_event : TBool;
        display : PDisplay;
        parent : TWindow;
        window : TWindow;
        x, y : cint;
        width, height : cint;
        border_width : cint;
        above : TWindow;
        detail : cint;
        value_mask : culong;
     end;

   PXCirculateEvent = ^TXCirculateEvent;
   TXCirculateEvent = record
        _type : cint;
        serial : culong;
        send_event : TBool;
        display : PDisplay;
        event : TWindow;
        window : TWindow;
        place : cint;
     end;

   PXCirculateRequestEvent = ^TXCirculateRequestEvent;
   TXCirculateRequestEvent = record
        _type : cint;
        serial : culong;
        send_event : TBool;
        display : PDisplay;
        parent : TWindow;
        window : TWindow;
        place : cint;
     end;

   PXPropertyEvent = ^TXPropertyEvent;
   TXPropertyEvent = record
        _type : cint;
        serial : culong;
        send_event : TBool;
        display : PDisplay;
        window : TWindow;
        atom : TAtom;
        time : TTime;
        state : cint;
     end;

   PXSelectionClearEvent = ^TXSelectionClearEvent;
   TXSelectionClearEvent = record
        _type : cint;
        serial : culong;
        send_event : TBool;
        display : PDisplay;
        window : TWindow;
        selection : TAtom;
        time : TTime;
     end;

   PXSelectionRequestEvent = ^TXSelectionRequestEvent;
   TXSelectionRequestEvent = record
        _type : cint;
        serial : culong;
        send_event : TBool;
        display : PDisplay;
        owner : TWindow;
        requestor : TWindow;
        selection : TAtom;
        target : TAtom;
        _property : TAtom;
        time : TTime;
     end;

   PXSelectionEvent = ^TXSelectionEvent;
   TXSelectionEvent = record
        _type : cint;
        serial : culong;
        send_event : TBool;
        display : PDisplay;
        requestor : TWindow;
        selection : TAtom;
        target : TAtom;
        _property : TAtom;
        time : TTime;
     end;

   PXColormapEvent = ^TXColormapEvent;
   TXColormapEvent = record
        _type : cint;
        serial : culong;
        send_event : TBool;
        display : PDisplay;
        window : TWindow;
        colormap : TColormap;
        c_new : TBool;
        state : cint;
     end;

   PXClientMessageEvent = ^TXClientMessageEvent;
   TXClientMessageEvent = record
        _type : cint;
        serial : culong;
        send_event : TBool;
        display : PDisplay;
        window : TWindow;
        message_type : TAtom;
        format : cint;
        data : record
            case longint of
               0 : ( b : array[0..19] of cchar );
               1 : ( s : array[0..9] of cshort );
               2 : ( l : array[0..4] of clong );
            end;
     end;

   PXMappingEvent = ^TXMappingEvent;
   TXMappingEvent = record
        _type : cint;
        serial : culong;
        send_event : TBool;
        display : PDisplay;
        window : TWindow;
        request : cint;
        first_keycode : cint;
        count : cint;
     end;

   PXErrorEvent = ^TXErrorEvent;
   TXErrorEvent = record
        _type : cint;
        display : PDisplay;
        resourceid : TXID;
        serial : culong;
        error_code : cuchar;
        request_code : cuchar;
        minor_code : cuchar;
     end;

   PXAnyEvent = ^TXAnyEvent;
   TXAnyEvent = record
        _type : cint;
        serial : culong;
        send_event : TBool;
        display : PDisplay;
        window : TWindow;
     end;

   PXEvent = ^TXEvent;
   TXEvent = record
       case longint of
          0 : ( _type : cint );
          1 : ( xany : TXAnyEvent );
          2 : ( xkey : TXKeyEvent );
          3 : ( xbutton : TXButtonEvent );
          4 : ( xmotion : TXMotionEvent );
          5 : ( xcrossing : TXCrossingEvent );
          6 : ( xfocus : TXFocusChangeEvent );
          7 : ( xexpose : TXExposeEvent );
          8 : ( xgraphicsexpose : TXGraphicsExposeEvent );
          9 : ( xnoexpose : TXNoExposeEvent );
          10 : ( xvisibility : TXVisibilityEvent );
          11 : ( xcreatewindow : TXCreateWindowEvent );
          12 : ( xdestroywindow : TXDestroyWindowEvent );
          13 : ( xunmap : TXUnmapEvent );
          14 : ( xmap : TXMapEvent );
          15 : ( xmaprequest : TXMapRequestEvent );
          16 : ( xreparent : TXReparentEvent );
          17 : ( xconfigure : TXConfigureEvent );
          18 : ( xgravity : TXGravityEvent );
          19 : ( xresizerequest : TXResizeRequestEvent );
          20 : ( xconfigurerequest : TXConfigureRequestEvent );
          21 : ( xcirculate : TXCirculateEvent );
          22 : ( xcirculaterequest : TXCirculateRequestEvent );
          23 : ( xproperty : TXPropertyEvent );
          24 : ( xselectionclear : TXSelectionClearEvent );
          25 : ( xselectionrequest : TXSelectionRequestEvent );
          26 : ( xselection : TXSelectionEvent );
          27 : ( xcolormap : TXColormapEvent );
          28 : ( xclient : TXClientMessageEvent );
          29 : ( xmapping : TXMappingEvent );
          30 : ( xerror : TXErrorEvent );
          31 : ( xkeymap : TXKeymapEvent );
          32 : ( pad : array[0..23] of clong );
       end;

type

   PXCharStruct = ^TXCharStruct;
   TXCharStruct = record
        lbearing : cshort;
        rbearing : cshort;
        width : cshort;
        ascent : cshort;
        descent : cshort;
        attributes : cushort;
     end;

   PXFontProp = ^TXFontProp;
   TXFontProp = record
        name : TAtom;
        card32 : culong;
     end;

   PPPXFontStruct = ^PPXFontStruct;
   PPXFontStruct = ^PXFontStruct;
   PXFontStruct = ^TXFontStruct;
   TXFontStruct = record
        ext_data : PXExtData;
        fid : TFont;
        direction : cunsigned;
        min_char_or_byte2 : cunsigned;
        max_char_or_byte2 : cunsigned;
        min_byte1 : cunsigned;
        max_byte1 : cunsigned;
        all_chars_exist : TBool;
        default_char : cunsigned;
        n_properties : cint;
        properties : PXFontProp;
        min_bounds : TXCharStruct;
        max_bounds : TXCharStruct;
        per_char : PXCharStruct;
        ascent : cint;
        descent : cint;
     end;

   PXTextItem = ^TXTextItem;
   TXTextItem = record
        chars : Pchar;
        nchars : cint;
        delta : cint;
        font : TFont;
     end;

   PXChar2b = ^TXChar2b;
   TXChar2b = record
        byte1 : cuchar;
        byte2 : cuchar;
     end;

   PXTextItem16 = ^TXTextItem16;
   TXTextItem16 = record
        chars : PXChar2b;
        nchars : cint;
        delta : cint;
        font : TFont;
     end;

   PXEDataObject = ^TXEDataObject;
   TXEDataObject = record
       case longint of
          0 : ( display : PDisplay );
          1 : ( gc : TGC );
          2 : ( visual : PVisual );
          3 : ( screen : PScreen );
          4 : ( pixmap_format : PScreenFormat );
          5 : ( font : PXFontStruct );
       end;

   PXFontSetExtents = ^TXFontSetExtents;
   TXFontSetExtents = record
        max_ink_extent : TXRectangle;
        max_logical_extent : TXRectangle;
     end;

   PXOM = ^TXOM;
   TXOM = record
     end;

   PXOC = ^TXOC;
   TXOC = record
     end;
   TXFontSet = PXOC;
   PXFontSet = ^TXFontSet;

   PXmbTextItem = ^TXmbTextItem;
   TXmbTextItem = record
        chars : Pchar;
        nchars : cint;
        delta : cint;
        font_set : TXFontSet;
     end;

   PXwcTextItem = ^TXwcTextItem;
   TXwcTextItem = record
        chars : PWideChar; {wchar_t*}
        nchars : cint;
        delta : cint;
        font_set : TXFontSet;
     end;

const
   XNRequiredCharSet = 'requiredCharSet';
   XNQueryOrientation = 'queryOrientation';
   XNBaseFontName = 'baseFontName';
   XNOMAutomatic = 'omAutomatic';
   XNMissingCharSet = 'missingCharSet';
   XNDefaultString = 'defaultString';
   XNOrientation = 'orientation';
   XNDirectionalDependentDrawing = 'directionalDependentDrawing';
   XNContextualDrawing = 'contextualDrawing';
   XNFontInfo = 'fontInfo';
type

   PXOMCharSetList = ^TXOMCharSetList;
   TXOMCharSetList = record
        charset_count : cint;
        charset_list : PPChar;
     end;

   PXOrientation = ^TXOrientation;
   TXOrientation = (XOMOrientation_LTR_TTB,XOMOrientation_RTL_TTB,
     XOMOrientation_TTB_LTR,XOMOrientation_TTB_RTL,
     XOMOrientation_Context);

   PXOMOrientation = ^TXOMOrientation;
   TXOMOrientation = record
        num_orientation : cint;
        orientation : PXOrientation;
     end;

   PXOMFontInfo = ^TXOMFontInfo;
   TXOMFontInfo = record
        num_font : cint;
        font_struct_list : ^PXFontStruct;
        font_name_list : PPChar;
     end;

   PXIM = ^TXIM;
   TXIM = record
     end;

   PXIC = ^TXIC;
   TXIC = record
     end;

   TXIMProc = procedure (para1:TXIM; para2:TXPointer; para3:TXPointer);cdecl;

<<<<<<< HEAD
<<<<<<< HEAD
   TXICProc = function (para1:TXIC; para2:TXPointer; para3:TXPointer):TBoolResult;cdecl;
=======
   TXICProc = function (para1:TXIC; para2:TXPointer; para3:TXPointer):TBool;cdecl;
>>>>>>> graemeg/fixes_2_2
=======
   TXICProc = function (para1:TXIC; para2:TXPointer; para3:TXPointer):TBool;cdecl;
>>>>>>> origin/fixes_2_2

   TXIDProc = procedure (para1:PDisplay; para2:TXPointer; para3:TXPointer);cdecl;

   PXIMStyle = ^TXIMStyle;
   TXIMStyle = culong;

   PXIMStyles = ^TXIMStyles;
   TXIMStyles = record
        count_styles : cushort;
        supported_styles : PXIMStyle;
     end;

const
   XIMPreeditArea = $0001;
   XIMPreeditCallbacks = $0002;
   XIMPreeditPosition = $0004;
   XIMPreeditNothing = $0008;
   XIMPreeditNone = $0010;
   XIMStatusArea = $0100;
   XIMStatusCallbacks = $0200;
   XIMStatusNothing = $0400;
   XIMStatusNone = $0800;
   XNVaNestedList = 'XNVaNestedList';
   XNQueryInputStyle = 'queryInputStyle';
   XNClientWindow = 'clientWindow';
   XNInputStyle = 'inputStyle';
   XNFocusWindow = 'focusWindow';
   XNResourceName = 'resourceName';
   XNResourceClass = 'resourceClass';
   XNGeometryCallback = 'geometryCallback';
   XNDestroyCallback = 'destroyCallback';
   XNFilterEvents = 'filterEvents';
   XNPreeditStartCallback = 'preeditStartCallback';
   XNPreeditDoneCallback = 'preeditDoneCallback';
   XNPreeditDrawCallback = 'preeditDrawCallback';
   XNPreeditCaretCallback = 'preeditCaretCallback';
   XNPreeditStateNotifyCallback = 'preeditStateNotifyCallback';
   XNPreeditAttributes = 'preeditAttributes';
   XNStatusStartCallback = 'statusStartCallback';
   XNStatusDoneCallback = 'statusDoneCallback';
   XNStatusDrawCallback = 'statusDrawCallback';
   XNStatusAttributes = 'statusAttributes';
   XNArea = 'area';
   XNAreaNeeded = 'areaNeeded';
   XNSpotLocation = 'spotLocation';
   XNColormap = 'colorMap';
   XNStdColormap = 'stdColorMap';
   XNForeground = 'foreground';
   XNBackground = 'background';
   XNBackgroundPixmap = 'backgroundPixmap';
   XNFontSet = 'fontSet';
   XNLineSpace = 'lineSpace';
   XNCursor = 'cursor';
   XNQueryIMValuesList = 'queryIMValuesList';
   XNQueryICValuesList = 'queryICValuesList';
   XNVisiblePosition = 'visiblePosition';
   XNR6PreeditCallback = 'r6PreeditCallback';
   XNStringConversionCallback = 'stringConversionCallback';
   XNStringConversion = 'stringConversion';
   XNResetState = 'resetState';
   XNHotKey = 'hotKey';
   XNHotKeyState = 'hotKeyState';
   XNPreeditState = 'preeditState';
   XNSeparatorofNestedList = 'separatorofNestedList';
   XBufferOverflow = -(1);
   XLookupNone = 1;
   XLookupChars = 2;
   XLookupKeySymVal = 3;
   XLookupBoth = 4;
type

   PXVaNestedList = ^TXVaNestedList;
   TXVaNestedList = pointer;

   PXIMCallback = ^TXIMCallback;
   TXIMCallback = record
        client_data : TXPointer;
        callback : TXIMProc;
     end;

   PXICCallback = ^TXICCallback;
   TXICCallback = record
        client_data : TXPointer;
        callback : TXICProc;
     end;

   PXIMFeedback = ^TXIMFeedback;
   TXIMFeedback = culong;

const
   XIMReverse = 1;
   XIMUnderline = 1 shl 1;
   XIMHighlight = 1 shl 2;
   XIMPrimary = 1 shl 5;
   XIMSecondary = 1 shl 6;
   XIMTertiary = 1 shl 7;
   XIMVisibleToForward = 1 shl 8;
   XIMVisibleToBackword = 1 shl 9;
   XIMVisibleToCenter = 1 shl 10;
type

   PXIMText = ^TXIMText;
   TXIMText = record
        length : cushort;
        feedback : PXIMFeedback;
        encoding_is_wchar : TBool;
        _string : record
            case longint of
               0 : ( multi_byte : Pchar );
               1 : ( wide_char : PWideChar ); {wchar_t*}
            end;
     end;

   PXIMPreeditState = ^TXIMPreeditState;
   TXIMPreeditState = culong;

const
   XIMPreeditUnKnown = 0;
   XIMPreeditEnable = 1;
   XIMPreeditDisable = 1 shl 1;
type

   PXIMPreeditStateNotifyCallbackStruct = ^TXIMPreeditStateNotifyCallbackStruct;
   TXIMPreeditStateNotifyCallbackStruct = record
        state : TXIMPreeditState;
     end;

   PXIMResetState = ^TXIMResetState;
   TXIMResetState = culong;

const
   XIMInitialState = 1;
   XIMPreserveState = 1 shl 1;
type

   PXIMStringConversionFeedback = ^TXIMStringConversionFeedback;
   TXIMStringConversionFeedback = culong;

const
   XIMStringConversionLeftEdge = $00000001;
   XIMStringConversionRightEdge = $00000002;
   XIMStringConversionTopEdge = $00000004;
   XIMStringConversionBottomEdge = $00000008;
   XIMStringConversionConcealed = $00000010;
   XIMStringConversionWrapped = $00000020;
type

   PXIMStringConversionText = ^TXIMStringConversionText;
   TXIMStringConversionText = record
        length : cushort;
        feedback : PXIMStringConversionFeedback;
        encoding_is_wchar : TBool;
        _string : record
            case longint of
               0 : ( mbs : Pchar );
               1 : ( wcs : PWideChar ); {wchar_t*}
            end;
     end;

   PXIMStringConversionPosition = ^TXIMStringConversionPosition;
   TXIMStringConversionPosition = cushort;

   PXIMStringConversionType = ^TXIMStringConversionType;
   TXIMStringConversionType = cushort;

const
   XIMStringConversionBuffer = $0001;
   XIMStringConversionLine = $0002;
   XIMStringConversionWord = $0003;
   XIMStringConversionChar = $0004;
type

   PXIMStringConversionOperation = ^TXIMStringConversionOperation;
   TXIMStringConversionOperation = cushort;

const
   XIMStringConversionSubstitution = $0001;
   XIMStringConversionRetrieval = $0002;
type

   PXIMCaretDirection = ^TXIMCaretDirection;
   TXIMCaretDirection = (XIMForwardChar,XIMBackwardChar,XIMForwardWord,
     XIMBackwardWord,XIMCaretUp,XIMCaretDown,
     XIMNextLine,XIMPreviousLine,XIMLineStart,
     XIMLineEnd,XIMAbsolutePosition,XIMDontChange
     );

   PXIMStringConversionCallbackStruct = ^TXIMStringConversionCallbackStruct;
   TXIMStringConversionCallbackStruct = record
        position : TXIMStringConversionPosition;
        direction : TXIMCaretDirection;
        operation : TXIMStringConversionOperation;
        factor : cushort;
        text : PXIMStringConversionText;
     end;

   PXIMPreeditDrawCallbackStruct = ^TXIMPreeditDrawCallbackStruct;
   TXIMPreeditDrawCallbackStruct = record
        caret : cint;
        chg_first : cint;
        chg_length : cint;
        text : PXIMText;
     end;

   PXIMCaretStyle = ^TXIMCaretStyle;
   TXIMCaretStyle = (XIMIsInvisible,XIMIsPrimary,XIMIsSecondary
     );

   PXIMPreeditCaretCallbackStruct = ^TXIMPreeditCaretCallbackStruct;
   TXIMPreeditCaretCallbackStruct = record
        position : cint;
        direction : TXIMCaretDirection;
        style : TXIMCaretStyle;
     end;

   PXIMStatusDataType = ^TXIMStatusDataType;
   TXIMStatusDataType = (XIMTextType,XIMBitmapType);

   PXIMStatusDrawCallbackStruct = ^TXIMStatusDrawCallbackStruct;
   TXIMStatusDrawCallbackStruct = record
        _type : TXIMStatusDataType;
        data : record
            case longint of
               0 : ( text : PXIMText );
               1 : ( bitmap : TPixmap );
            end;
     end;

   PXIMHotKeyTrigger = ^TXIMHotKeyTrigger;
   TXIMHotKeyTrigger = record
        keysym : TKeySym;
        modifier : cint;
        modifier_mask : cint;
     end;

   PXIMHotKeyTriggers = ^TXIMHotKeyTriggers;
   TXIMHotKeyTriggers = record
        num_hot_key : cint;
        key : PXIMHotKeyTrigger;
     end;

   PXIMHotKeyState = ^TXIMHotKeyState;
   TXIMHotKeyState = culong;

const
   XIMHotKeyStateON = $0001;
   XIMHotKeyStateOFF = $0002;
type

   PXIMValuesList = ^TXIMValuesList;
   TXIMValuesList = record
        count_values : cushort;
        supported_values : PPChar;
     end;
{$ifndef os2}
  var
     _Xdebug : cint;cvar;external;
{$endif}
type
<<<<<<< HEAD
<<<<<<< HEAD
  funcdisp    = function(display:PDisplay):cint;cdecl;
  funcifevent = function(display:PDisplay; event:PXEvent; p : TXPointer):TBoolResult;cdecl;
  chararr32   = array[0..31] of char;
  pchararr32  = chararr32;
=======
  funcdisp = function(display:PDisplay):cint;cdecl;
  funcifevent = function(display:PDisplay; event:PXEvent; p : TXPointer):TBool;cdecl;
  chararr32 = array[0..31] of char;
>>>>>>> graemeg/fixes_2_2
=======
  funcdisp = function(display:PDisplay):cint;cdecl;
  funcifevent = function(display:PDisplay; event:PXEvent; p : TXPointer):TBool;cdecl;
  chararr32 = array[0..31] of char;
>>>>>>> origin/fixes_2_2

const
  AllPlanes : culong = culong(not 0);

function XLoadQueryFont(para1:PDisplay; para2:Pchar):PXFontStruct;cdecl;external libX11;
function XQueryFont(para1:PDisplay; para2:TXID):PXFontStruct;cdecl;external libX11;
function XGetMotionEvents(para1:PDisplay; para2:TWindow; para3:TTime; para4:TTime; para5:Pcint):PXTimeCoord;cdecl;external libX11;
function XDeleteModifiermapEntry(para1:PXModifierKeymap; para2:TKeyCode; para3:cint):PXModifierKeymap;cdecl;external libX11;
function XGetModifierMapping(para1:PDisplay):PXModifierKeymap;cdecl;external libX11;
function XInsertModifiermapEntry(para1:PXModifierKeymap; para2:TKeyCode; para3:cint):PXModifierKeymap;cdecl;external libX11;
function XNewModifiermap(para1:cint):PXModifierKeymap;cdecl;external libX11;
function XCreateImage(para1:PDisplay; para2:PVisual; para3:cuint; para4:cint; para5:cint;
           para6:Pchar; para7:cuint; para8:cuint; para9:cint; para10:cint):PXImage;cdecl;external libX11;
function XInitImage(para1:PXImage):TStatus;cdecl;external libX11;
function XGetImage(para1:PDisplay; para2:TDrawable; para3:cint; para4:cint; para5:cuint;
           para6:cuint; para7:culong; para8:cint):PXImage;cdecl;external libX11;
function XGetSubImage(para1:PDisplay; para2:TDrawable; para3:cint; para4:cint; para5:cuint;
           para6:cuint; para7:culong; para8:cint; para9:PXImage; para10:cint;
           para11:cint):PXImage;cdecl;external libX11;
function XOpenDisplay(para1:Pchar):PDisplay;cdecl;external libX11;
procedure XrmInitialize;cdecl;external libX11;
function XFetchBytes(para1:PDisplay; para2:Pcint):Pchar;cdecl;external libX11;
function XFetchBuffer(para1:PDisplay; para2:Pcint; para3:cint):Pchar;cdecl;external libX11;
function XGetAtomName(para1:PDisplay; para2:TAtom):Pchar;cdecl;external libX11;
function XGetAtomNames(para1:PDisplay; para2:PAtom; para3:cint; para4:PPchar):TStatus;cdecl;external libX11;
function XGetDefault(para1:PDisplay; para2:Pchar; para3:Pchar):Pchar;cdecl;external libX11;
function XDisplayName(para1:Pchar):Pchar;cdecl;external libX11;
function XKeysymToString(para1:TKeySym):Pchar;cdecl;external libX11;
function XSynchronize(para1:PDisplay; para2:TBool):funcdisp;cdecl;external libX11;
function XSetAfterFunction(para1:PDisplay; para2:funcdisp):funcdisp;cdecl;external libX11;
function XInternAtom(para1:PDisplay; para2:Pchar; para3:TBool):TAtom;cdecl;external libX11;
function XInternAtoms(para1:PDisplay; para2:PPchar; para3:cint; para4:TBool; para5:PAtom):TStatus;cdecl;external libX11;
function XCopyColormapAndFree(para1:PDisplay; para2:TColormap):TColormap;cdecl;external libX11;
function XCreateColormap(para1:PDisplay; para2:TWindow; para3:PVisual; para4:cint):TColormap;cdecl;external libX11;
function XCreatePixmapCursor(ADisplay:PDisplay; ASource:TPixmap; AMask:TPixmap; AForegroundColor:PXColor; ABackgroundColor:PXColor;
           AX:cuint; AY:cuint):TCursor;cdecl;external libX11;
function XCreateGlyphCursor(ADisplay:PDisplay; ASourceFont:TFont; AMaskFont:TFont; ASourceChar:cuint; AMaskChar:cuint;
           AForegroundColor:PXColor; ABackgroundColor:PXColor):TCursor;cdecl;external libX11;
function XCreateFontCursor(ADisplay:PDisplay; AShape:cuint):TCursor;cdecl;external libX11;
function XLoadFont(para1:PDisplay; para2:Pchar):TFont;cdecl;external libX11;
function XCreateGC(para1:PDisplay; para2:TDrawable; para3:culong; para4:PXGCValues):TGC;cdecl;external libX11;
function XGContextFromGC(para1:TGC):TGContext;cdecl;external libX11;
procedure XFlushGC(para1:PDisplay; para2:TGC);cdecl;external libX11;
function XCreatePixmap(ADisplay:PDisplay; ADrawable:TDrawable; AWidth:cuint; AHeight:cuint; ADepth:cuint):TPixmap;cdecl;external libX11;
function XCreateBitmapFromData(ADiplay:PDisplay; ADrawable:TDrawable; AData:Pchar; AWidth:cuint; AHeight:cuint):TPixmap;cdecl;external libX11;
function XCreatePixmapFromBitmapData(para1:PDisplay; para2:TDrawable; para3:Pchar; para4:cuint; para5:cuint;
           para6:culong; para7:culong; para8:cuint):TPixmap;cdecl;external libX11;
function XCreateSimpleWindow(ADisplay:PDisplay; AParent:TWindow; AX:cint; AY:cint; AWidth:cuint;
           AHeight:cuint; ABorderWidth:cuint; ABorder:culong; ABackground:culong):TWindow;cdecl;external libX11;
function XGetSelectionOwner(para1:PDisplay; para2:TAtom):TWindow;cdecl;external libX11;
function XCreateWindow(ADisplay:PDisplay; AParent:TWindow; AX:cint; AY:cint; AWidth:cuint;
           AHeight:cuint; ABorderWidth:cuint; ADepth:cint; AClass:cuint; AVisual:PVisual;
           AValueMask:culong; AAttributes:PXSetWindowAttributes):TWindow;cdecl;external libX11;
function XListInstalledColormaps(para1:PDisplay; para2:TWindow; para3:Pcint):PColormap;cdecl;external libX11;
function XListFonts(para1:PDisplay; para2:Pchar; para3:cint; para4:Pcint):PPChar;cdecl;external libX11;
function XListFontsWithInfo(para1:PDisplay; para2:Pchar; para3:cint; para4:Pcint; para5:PPXFontStruct):PPChar;cdecl;external libX11;
function XGetFontPath(para1:PDisplay; para2:Pcint):PPChar;cdecl;external libX11;
function XListExtensions(para1:PDisplay; para2:Pcint):PPChar;cdecl;external libX11;
function XListProperties(para1:PDisplay; para2:TWindow; para3:Pcint):PAtom;cdecl;external libX11;
function XListHosts(para1:PDisplay; para2:Pcint; para3:PBool):PXHostAddress;cdecl;external libX11;
function XKeycodeToKeysym(para1:PDisplay; para2:TKeyCode; para3:cint):TKeySym;cdecl;external libX11;
function XLookupKeysym(para1:PXKeyEvent; para2:cint):TKeySym;cdecl;external libX11;
function XGetKeyboardMapping(para1:PDisplay; para2:TKeyCode; para3:cint; para4:Pcint):PKeySym;cdecl;external libX11;
function XStringToKeysym(para1:Pchar):TKeySym;cdecl;external libX11;
function XMaxRequestSize(para1:PDisplay):clong;cdecl;external libX11;
function XExtendedMaxRequestSize(para1:PDisplay):clong;cdecl;external libX11;
function XResourceManagerString(para1:PDisplay):Pchar;cdecl;external libX11;
function XScreenResourceString(para1:PScreen):Pchar;cdecl;external libX11;
function XDisplayMotionBufferSize(para1:PDisplay):culong;cdecl;external libX11;
function XVisualIDFromVisual(para1:PVisual):TVisualID;cdecl;external libX11;
function XInitThreads:TStatus;cdecl;external libX11;
procedure XLockDisplay(para1:PDisplay);cdecl;external libX11;
procedure XUnlockDisplay(para1:PDisplay);cdecl;external libX11;
function XInitExtension(para1:PDisplay; para2:Pchar):PXExtCodes;cdecl;external libX11;
function XAddExtension(para1:PDisplay):PXExtCodes;cdecl;external libX11;
function XFindOnExtensionList(para1:PPXExtData; para2:cint):PXExtData;cdecl;external libX11;
function XEHeadOfExtensionList(para1:TXEDataObject):PPXExtData;cdecl;external libX11;
function XRootWindow(ADisplay:PDisplay; AScreenNumber:cint):TWindow;cdecl;external libX11;
function XDefaultRootWindow(ADisplay:PDisplay):TWindow;cdecl;external libX11;
function XRootWindowOfScreen(para1:PScreen):TWindow;cdecl;external libX11;
function XDefaultVisual(para1:PDisplay; para2:cint):PVisual;cdecl;external libX11;
function XDefaultVisualOfScreen(para1:PScreen):PVisual;cdecl;external libX11;
function XDefaultGC(para1:PDisplay; para2:cint):TGC;cdecl;external libX11;
function XDefaultGCOfScreen(para1:PScreen):TGC;cdecl;external libX11;
function XBlackPixel(ADisplay:PDisplay; AScreenNumber:cint):culong;cdecl;external libX11;
function XWhitePixel(ADisplay:PDisplay; AScreenNumber:cint):culong;cdecl;external libX11;
function XAllPlanes:culong;cdecl;external libX11;
function XBlackPixelOfScreen(para1:PScreen):culong;cdecl;external libX11;
function XWhitePixelOfScreen(para1:PScreen):culong;cdecl;external libX11;
function XNextRequest(para1:PDisplay):culong;cdecl;external libX11;
function XLastKnownRequestProcessed(para1:PDisplay):culong;cdecl;external libX11;
function XServerVendor(para1:PDisplay):Pchar;cdecl;external libX11;
function XDisplayString(para1:PDisplay):Pchar;cdecl;external libX11;
function XDefaultColormap(para1:PDisplay; para2:cint):TColormap;cdecl;external libX11;
function XDefaultColormapOfScreen(para1:PScreen):TColormap;cdecl;external libX11;
function XDisplayOfScreen(para1:PScreen):PDisplay;cdecl;external libX11;
function XScreenOfDisplay(para1:PDisplay; para2:cint):PScreen;cdecl;external libX11;
function XDefaultScreenOfDisplay(para1:PDisplay):PScreen;cdecl;external libX11;
function XEventMaskOfScreen(para1:PScreen):clong;cdecl;external libX11;
function XScreenNumberOfScreen(para1:PScreen):cint;cdecl;external libX11;
type

   TXErrorHandler = function (para1:PDisplay; para2:PXErrorEvent):cint;cdecl;

function XSetErrorHandler(para1:TXErrorHandler):TXErrorHandler;cdecl;external libX11;
type

   TXIOErrorHandler = function (para1:PDisplay):cint;cdecl;

function XSetIOErrorHandler(para1:TXIOErrorHandler):TXIOErrorHandler;cdecl;external libX11;
function XListPixmapFormats(para1:PDisplay; para2:Pcint):PXPixmapFormatValues;cdecl;external libX11;
function XListDepths(para1:PDisplay; para2:cint; para3:Pcint):Pcint;cdecl;external libX11;
function XReconfigureWMWindow(para1:PDisplay; para2:TWindow; para3:cint; para4:cuint; para5:PXWindowChanges):TStatus;cdecl;external libX11;
function XGetWMProtocols(para1:PDisplay; para2:TWindow; para3:PPAtom; para4:Pcint):TStatus;cdecl;external libX11;
function XSetWMProtocols(para1:PDisplay; para2:TWindow; para3:PAtom; para4:cint):TStatus;cdecl;external libX11;
function XIconifyWindow(para1:PDisplay; para2:TWindow; para3:cint):TStatus;cdecl;external libX11;
function XWithdrawWindow(para1:PDisplay; para2:TWindow; para3:cint):TStatus;cdecl;external libX11;
function XGetCommand(para1:PDisplay; para2:TWindow; para3:PPPchar; para4:Pcint):TStatus;cdecl;external libX11;
function XGetWMColormapWindows(para1:PDisplay; para2:TWindow; para3:PPWindow; para4:Pcint):TStatus;cdecl;external libX11;
function XSetWMColormapWindows(para1:PDisplay; para2:TWindow; para3:PWindow; para4:cint):TStatus;cdecl;external libX11;
procedure XFreeStringList(para1:PPchar);cdecl;external libX11;
function XSetTransientForHint(ADisplay:PDisplay; AWindow:TWindow; APropWindow:TWindow):cint;cdecl;external libX11;
function XActivateScreenSaver(para1:PDisplay):cint;cdecl;external libX11;
function XAddHost(para1:PDisplay; para2:PXHostAddress):cint;cdecl;external libX11;
function XAddHosts(para1:PDisplay; para2:PXHostAddress; para3:cint):cint;cdecl;external libX11;
function XAddToExtensionList(para1:PPXExtData; para2:PXExtData):cint;cdecl;external libX11;
function XAddToSaveSet(para1:PDisplay; para2:TWindow):cint;cdecl;external libX11;
function XAllocColor(para1:PDisplay; para2:TColormap; para3:PXColor):TStatus;cdecl;external libX11;
function XAllocColorCells(para1:PDisplay; para2:TColormap; para3:TBool; para4:Pculong; para5:cuint;
           para6:Pculong; para7:cuint):TStatus;cdecl;external libX11;
function XAllocColorPlanes(para1:PDisplay; para2:TColormap; para3:TBool; para4:Pculong; para5:cint;
           para6:cint; para7:cint; para8:cint; para9:Pculong; para10:Pculong;
           para11:Pculong):TStatus;cdecl;external libX11;
function XAllocNamedColor(para1:PDisplay; para2:TColormap; para3:Pchar; para4:PXColor; para5:PXColor):TStatus;cdecl;external libX11;
function XAllowEvents(para1:PDisplay; para2:cint; para3:TTime):cint;cdecl;external libX11;
function XAutoRepeatOff(para1:PDisplay):cint;cdecl;external libX11;
function XAutoRepeatOn(para1:PDisplay):cint;cdecl;external libX11;
function XBell(para1:PDisplay; para2:cint):cint;cdecl;external libX11;
function XBitmapBitOrder(para1:PDisplay):cint;cdecl;external libX11;
function XBitmapPad(para1:PDisplay):cint;cdecl;external libX11;
function XBitmapUnit(para1:PDisplay):cint;cdecl;external libX11;
function XCellsOfScreen(para1:PScreen):cint;cdecl;external libX11;
function XChangeActivePointerGrab(para1:PDisplay; para2:cuint; para3:TCursor; para4:TTime):cint;cdecl;external libX11;
function XChangeGC(para1:PDisplay; para2:TGC; para3:culong; para4:PXGCValues):cint;cdecl;external libX11;
function XChangeKeyboardControl(para1:PDisplay; para2:culong; para3:PXKeyboardControl):cint;cdecl;external libX11;
function XChangeKeyboardMapping(para1:PDisplay; para2:cint; para3:cint; para4:PKeySym; para5:cint):cint;cdecl;external libX11;
function XChangePointerControl(para1:PDisplay; para2:TBool; para3:TBool; para4:cint; para5:cint;
           para6:cint):cint;cdecl;external libX11;
function XChangeProperty(para1:PDisplay; para2:TWindow; para3:TAtom; para4:TAtom; para5:cint;
           para6:cint; para7:Pcuchar; para8:cint):cint;cdecl;external libX11;
function XChangeSaveSet(para1:PDisplay; para2:TWindow; para3:cint):cint;cdecl;external libX11;
function XChangeWindowAttributes(para1:PDisplay; para2:TWindow; para3:culong; para4:PXSetWindowAttributes):cint;cdecl;external libX11;
<<<<<<< HEAD
<<<<<<< HEAD
function XCheckIfEvent(para1:PDisplay; para2:PXEvent; para3:funcifevent; para4:TXPointer):TBoolResult;cdecl;external libX11;
function XCheckMaskEvent(para1:PDisplay; para2:clong; para3:PXEvent):TBoolResult;cdecl;external libX11;
function XCheckTypedEvent(para1:PDisplay; para2:cint; para3:PXEvent):TBoolResult;cdecl;external libX11;
function XCheckTypedWindowEvent(para1:PDisplay; para2:TWindow; para3:cint; para4:PXEvent):TBoolResult;cdecl;external libX11;
function XCheckWindowEvent(para1:PDisplay; para2:TWindow; para3:clong; para4:PXEvent):TBoolResult;cdecl;external libX11;
=======
=======
>>>>>>> origin/fixes_2_2
function XCheckIfEvent(para1:PDisplay; para2:PXEvent; para3:funcifevent; para4:TXPointer):TBool;cdecl;external libX11;
function XCheckMaskEvent(para1:PDisplay; para2:clong; para3:PXEvent):TBool;cdecl;external libX11;
function XCheckTypedEvent(para1:PDisplay; para2:cint; para3:PXEvent):TBool;cdecl;external libX11;
function XCheckTypedWindowEvent(para1:PDisplay; para2:TWindow; para3:cint; para4:PXEvent):TBool;cdecl;external libX11;
function XCheckWindowEvent(para1:PDisplay; para2:TWindow; para3:clong; para4:PXEvent):TBool;cdecl;external libX11;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
function XCirculateSubwindows(para1:PDisplay; para2:TWindow; para3:cint):cint;cdecl;external libX11;
function XCirculateSubwindowsDown(para1:PDisplay; para2:TWindow):cint;cdecl;external libX11;
function XCirculateSubwindowsUp(para1:PDisplay; para2:TWindow):cint;cdecl;external libX11;
function XClearArea(para1:PDisplay; para2:TWindow; para3:cint; para4:cint; para5:cuint;
           para6:cuint; para7:TBool):cint;cdecl;external libX11;
function XClearWindow(para1:PDisplay; para2:TWindow):cint;cdecl;external libX11;
function XCloseDisplay(para1:PDisplay):cint;cdecl;external libX11;
function XConfigureWindow(para1:PDisplay; para2:TWindow; para3:cuint; para4:PXWindowChanges):cint;cdecl;external libX11;
function XConnectionNumber(para1:PDisplay):cint;cdecl;external libX11;
function XConvertSelection(para1:PDisplay; para2:TAtom; para3:TAtom; para4:TAtom; para5:TWindow;
           para6:TTime):cint;cdecl;external libX11;
function XCopyArea(para1:PDisplay; para2:TDrawable; para3:TDrawable; para4:TGC; para5:cint;
           para6:cint; para7:cuint; para8:cuint; para9:cint; para10:cint):cint;cdecl;external libX11;
function XCopyGC(para1:PDisplay; para2:TGC; para3:culong; para4:TGC):cint;cdecl;external libX11;
function XCopyPlane(para1:PDisplay; para2:TDrawable; para3:TDrawable; para4:TGC; para5:cint;
           para6:cint; para7:cuint; para8:cuint; para9:cint; para10:cint;
           para11:culong):cint;cdecl;external libX11;
function XDefaultDepth(para1:PDisplay; para2:cint):cint;cdecl;external libX11;
function XDefaultDepthOfScreen(para1:PScreen):cint;cdecl;external libX11;
function XDefaultScreen(para1:PDisplay):cint;cdecl;external libX11;
function XDefineCursor(ADisplay:PDisplay; AWindow:TWindow; ACursor:TCursor):cint;cdecl;external libX11;
function XDeleteProperty(para1:PDisplay; para2:TWindow; para3:TAtom):cint;cdecl;external libX11;
function XDestroyWindow(ADisplay:PDisplay; AWindow:TWindow):cint;cdecl;external libX11;
function XDestroySubwindows(ADisplay:PDisplay; AWindow:TWindow):cint;cdecl;external libX11;
function XDoesBackingStore(para1:PScreen):cint;cdecl;external libX11;
<<<<<<< HEAD
<<<<<<< HEAD
function XDoesSaveUnders(para1:PScreen):TBoolResult;cdecl;external libX11;
=======
function XDoesSaveUnders(para1:PScreen):TBool;cdecl;external libX11;
>>>>>>> graemeg/fixes_2_2
=======
function XDoesSaveUnders(para1:PScreen):TBool;cdecl;external libX11;
>>>>>>> origin/fixes_2_2
function XDisableAccessControl(para1:PDisplay):cint;cdecl;external libX11;
function XDisplayCells(para1:PDisplay; para2:cint):cint;cdecl;external libX11;
function XDisplayHeight(para1:PDisplay; para2:cint):cint;cdecl;external libX11;
function XDisplayHeightMM(para1:PDisplay; para2:cint):cint;cdecl;external libX11;
function XDisplayKeycodes(para1:PDisplay; para2:Pcint; para3:Pcint):cint;cdecl;external libX11;
function XDisplayPlanes(para1:PDisplay; para2:cint):cint;cdecl;external libX11;
function XDisplayWidth(para1:PDisplay; para2:cint):cint;cdecl;external libX11;
function XDisplayWidthMM(para1:PDisplay; para2:cint):cint;cdecl;external libX11;
function XDrawArc(para1:PDisplay; para2:TDrawable; para3:TGC; para4:cint; para5:cint;
           para6:cuint; para7:cuint; para8:cint; para9:cint):cint;cdecl;external libX11;
function XDrawArcs(para1:PDisplay; para2:TDrawable; para3:TGC; para4:PXArc; para5:cint):cint;cdecl;external libX11;
function XDrawImageString(para1:PDisplay; para2:TDrawable; para3:TGC; para4:cint; para5:cint;
           para6:Pchar; para7:cint):cint;cdecl;external libX11;
function XDrawImageString16(para1:PDisplay; para2:TDrawable; para3:TGC; para4:cint; para5:cint;
           para6:PXChar2b; para7:cint):cint;cdecl;external libX11;
function XDrawLine(para1:PDisplay; para2:TDrawable; para3:TGC; para4:cint; para5:cint;
           para6:cint; para7:cint):cint;cdecl;external libX11;
function XDrawLines(para1:PDisplay; para2:TDrawable; para3:TGC; para4:PXPoint; para5:cint;
           para6:cint):cint;cdecl;external libX11;
function XDrawPoint(para1:PDisplay; para2:TDrawable; para3:TGC; para4:cint; para5:cint):cint;cdecl;external libX11;
function XDrawPoints(para1:PDisplay; para2:TDrawable; para3:TGC; para4:PXPoint; para5:cint;
           para6:cint):cint;cdecl;external libX11;
function XDrawRectangle(para1:PDisplay; para2:TDrawable; para3:TGC; para4:cint; para5:cint;
           para6:cuint; para7:cuint):cint;cdecl;external libX11;
function XDrawRectangles(para1:PDisplay; para2:TDrawable; para3:TGC; para4:PXRectangle; para5:cint):cint;cdecl;external libX11;
function XDrawSegments(para1:PDisplay; para2:TDrawable; para3:TGC; para4:PXSegment; para5:cint):cint;cdecl;external libX11;
function XDrawString(para1:PDisplay; para2:TDrawable; para3:TGC; para4:cint; para5:cint;
           para6:Pchar; para7:cint):cint;cdecl;external libX11;
function XDrawString16(para1:PDisplay; para2:TDrawable; para3:TGC; para4:cint; para5:cint;
           para6:PXChar2b; para7:cint):cint;cdecl;external libX11;
function XDrawText(para1:PDisplay; para2:TDrawable; para3:TGC; para4:cint; para5:cint;
           para6:PXTextItem; para7:cint):cint;cdecl;external libX11;
function XDrawText16(para1:PDisplay; para2:TDrawable; para3:TGC; para4:cint; para5:cint;
           para6:PXTextItem16; para7:cint):cint;cdecl;external libX11;
function XEnableAccessControl(para1:PDisplay):cint;cdecl;external libX11;
function XEventsQueued(para1:PDisplay; para2:cint):cint;cdecl;external libX11;
function XFetchName(para1:PDisplay; para2:TWindow; para3:PPchar):TStatus;cdecl;external libX11;
function XFillArc(para1:PDisplay; para2:TDrawable; para3:TGC; para4:cint; para5:cint;
           para6:cuint; para7:cuint; para8:cint; para9:cint):cint;cdecl;external libX11;
function XFillArcs(para1:PDisplay; para2:TDrawable; para3:TGC; para4:PXArc; para5:cint):cint;cdecl;external libX11;
function XFillPolygon(para1:PDisplay; para2:TDrawable; para3:TGC; para4:PXPoint; para5:cint;
           para6:cint; para7:cint):cint;cdecl;external libX11;
function XFillRectangle(para1:PDisplay; para2:TDrawable; para3:TGC; para4:cint; para5:cint;
           para6:cuint; para7:cuint):cint;cdecl;external libX11;
function XFillRectangles(para1:PDisplay; para2:TDrawable; para3:TGC; para4:PXRectangle; para5:cint):cint;cdecl;external libX11;
function XFlush(para1:PDisplay):cint;cdecl;external libX11;
function XForceScreenSaver(para1:PDisplay; para2:cint):cint;cdecl;external libX11;
function XFree(para1:pointer):cint;cdecl;external libX11;
function XFreeColormap(para1:PDisplay; para2:TColormap):cint;cdecl;external libX11;
function XFreeColors(para1:PDisplay; para2:TColormap; para3:Pculong; para4:cint; para5:culong):cint;cdecl;external libX11;
function XFreeCursor(ADisplay:PDisplay; ACursor:TCursor):cint;cdecl;external libX11;
function XFreeExtensionList(para1:PPchar):cint;cdecl;external libX11;
function XFreeFont(para1:PDisplay; para2:PXFontStruct):cint;cdecl;external libX11;
function XFreeFontInfo(para1:PPchar; para2:PXFontStruct; para3:cint):cint;cdecl;external libX11;
function XFreeFontNames(para1:PPchar):cint;cdecl;external libX11;
function XFreeFontPath(para1:PPchar):cint;cdecl;external libX11;
function XFreeGC(para1:PDisplay; para2:TGC):cint;cdecl;external libX11;
function XFreeModifiermap(para1:PXModifierKeymap):cint;cdecl;external libX11;
function XFreePixmap(para1:PDisplay; para2:TPixmap):cint;cdecl;external libX11;
function XGeometry(para1:PDisplay; para2:cint; para3:Pchar; para4:Pchar; para5:cuint;
           para6:cuint; para7:cuint; para8:cint; para9:cint; para10:Pcint;
           para11:Pcint; para12:Pcint; para13:Pcint):cint;cdecl;external libX11;
function XGetErrorDatabaseText(para1:PDisplay; para2:Pchar; para3:Pchar; para4:Pchar; para5:Pchar;
           para6:cint):cint;cdecl;external libX11;
function XGetErrorText(para1:PDisplay; para2:cint; para3:Pchar; para4:cint):cint;cdecl;external libX11;
<<<<<<< HEAD
<<<<<<< HEAD
function XGetFontProperty(para1:PXFontStruct; para2:TAtom; para3:Pculong):TBoolResult;cdecl;external libX11;
=======
function XGetFontProperty(para1:PXFontStruct; para2:TAtom; para3:Pculong):TBool;cdecl;external libX11;
>>>>>>> graemeg/fixes_2_2
=======
function XGetFontProperty(para1:PXFontStruct; para2:TAtom; para3:Pculong):TBool;cdecl;external libX11;
>>>>>>> origin/fixes_2_2
function XGetGCValues(para1:PDisplay; para2:TGC; para3:culong; para4:PXGCValues):TStatus;cdecl;external libX11;
function XGetGeometry(para1:PDisplay; para2:TDrawable; para3:PWindow; para4:Pcint; para5:Pcint;
           para6:Pcuint; para7:Pcuint; para8:Pcuint; para9:Pcuint):TStatus;cdecl;external libX11;
function XGetIconName(para1:PDisplay; para2:TWindow; para3:PPchar):TStatus;cdecl;external libX11;
function XGetInputFocus(para1:PDisplay; para2:PWindow; para3:Pcint):cint;cdecl;external libX11;
function XGetKeyboardControl(para1:PDisplay; para2:PXKeyboardState):cint;cdecl;external libX11;
function XGetPointerControl(para1:PDisplay; para2:Pcint; para3:Pcint; para4:Pcint):cint;cdecl;external libX11;
function XGetPointerMapping(para1:PDisplay; para2:Pcuchar; para3:cint):cint;cdecl;external libX11;
function XGetScreenSaver(para1:PDisplay; para2:Pcint; para3:Pcint; para4:Pcint; para5:Pcint):cint;cdecl;external libX11;
function XGetTransientForHint(para1:PDisplay; para2:TWindow; para3:PWindow):TStatus;cdecl;external libX11;
function XGetWindowProperty(para1:PDisplay; para2:TWindow; para3:TAtom; para4:clong; para5:clong;
           para6:TBool; para7:TAtom; para8:PAtom; para9:Pcint; para10:Pculong;
           para11:Pculong; para12:PPcuchar):cint;cdecl;external libX11;
function XGetWindowAttributes(para1:PDisplay; para2:TWindow; para3:PXWindowAttributes):TStatus;cdecl;external libX11;
function XGrabButton(para1:PDisplay; para2:cuint; para3:cuint; para4:TWindow; para5:TBool;
           para6:cuint; para7:cint; para8:cint; para9:TWindow; para10:TCursor):cint;cdecl;external libX11;
function XGrabKey(para1:PDisplay; para2:cint; para3:cuint; para4:TWindow; para5:TBool;
           para6:cint; para7:cint):cint;cdecl;external libX11;
function XGrabKeyboard(para1:PDisplay; para2:TWindow; para3:TBool; para4:cint; para5:cint;
           para6:TTime):cint;cdecl;external libX11;
function XGrabPointer(para1:PDisplay; para2:TWindow; para3:TBool; para4:cuint; para5:cint;
           para6:cint; para7:TWindow; para8:TCursor; para9:TTime):cint;cdecl;external libX11;
function XGrabServer(para1:PDisplay):cint;cdecl;external libX11;
function XHeightMMOfScreen(para1:PScreen):cint;cdecl;external libX11;
function XHeightOfScreen(para1:PScreen):cint;cdecl;external libX11;
function XIfEvent(para1:PDisplay; para2:PXEvent; para3:funcifevent; para4:TXPointer):cint;cdecl;external libX11;
function XImageByteOrder(para1:PDisplay):cint;cdecl;external libX11;
function XInstallColormap(para1:PDisplay; para2:TColormap):cint;cdecl;external libX11;
function XKeysymToKeycode(para1:PDisplay; para2:TKeySym):TKeyCode;cdecl;external libX11;
function XKillClient(para1:PDisplay; para2:TXID):cint;cdecl;external libX11;
function XLookupColor(para1:PDisplay; para2:TColormap; para3:Pchar; para4:PXColor; para5:PXColor):TStatus;cdecl;external libX11;
function XLowerWindow(ADisplay:PDisplay; AWindow:TWindow):cint;cdecl;external libX11;
function XMapRaised(ADisplay:PDisplay; AWindow:TWindow):cint;cdecl;external libX11;
function XMapSubwindows(ADisplay:PDisplay; AWindow:TWindow):cint;cdecl;external libX11;
function XMapWindow(ADisplay:PDisplay; AWindow:TWindow):cint;cdecl;external libX11;
function XMaskEvent(para1:PDisplay; para2:clong; para3:PXEvent):cint;cdecl;external libX11;
function XMaxCmapsOfScreen(para1:PScreen):cint;cdecl;external libX11;
function XMinCmapsOfScreen(para1:PScreen):cint;cdecl;external libX11;
function XMoveResizeWindow(ADisplay:PDisplay; AWindow:TWindow; AX:cint; AY:cint; AWidth:cuint;
           AHeight:cuint):cint;cdecl;external libX11;
function XMoveWindow(ADisplay:PDisplay; AWindow:TWindow; AX:cint; AY:cint):cint;cdecl;external libX11;
function XNextEvent(ADisplay:PDisplay; AEvent:PXEvent):cint;cdecl;external libX11;
function XNoOp(para1:PDisplay):cint;cdecl;external libX11;
function XParseColor(para1:PDisplay; para2:TColormap; para3:Pchar; para4:PXColor):TStatus;cdecl;external libX11;
function XParseGeometry(para1:Pchar; para2:Pcint; para3:Pcint; para4:Pcuint; para5:Pcuint):cint;cdecl;external libX11;
function XPeekEvent(ADisplay:PDisplay; AEvent:PXEvent):cint;cdecl;external libX11;
function XPeekIfEvent(para1:PDisplay; para2:PXEvent; para3:funcifevent; para4:TXPointer):cint;cdecl;external libX11;
function XPending(para1:PDisplay):cint;cdecl;external libX11;
function XPlanesOfScreen(para1:PScreen):cint;cdecl;external libX11;
function XProtocolRevision(para1:PDisplay):cint;cdecl;external libX11;
function XProtocolVersion(para1:PDisplay):cint;cdecl;external libX11;
function XPutBackEvent(para1:PDisplay; para2:PXEvent):cint;cdecl;external libX11;
function XPutImage(para1:PDisplay; para2:TDrawable; para3:TGC; para4:PXImage; para5:cint;
           para6:cint; para7:cint; para8:cint; para9:cuint; para10:cuint):cint;cdecl;external libX11;
function XQLength(para1:PDisplay):cint;cdecl;external libX11;
function XQueryBestCursor(para1:PDisplay; para2:TDrawable; para3:cuint; para4:cuint; para5:Pcuint;
           para6:Pcuint):TStatus;cdecl;external libX11;
function XQueryBestSize(para1:PDisplay; para2:cint; para3:TDrawable; para4:cuint; para5:cuint;
           para6:Pcuint; para7:Pcuint):TStatus;cdecl;external libX11;
function XQueryBestStipple(para1:PDisplay; para2:TDrawable; para3:cuint; para4:cuint; para5:Pcuint;
           para6:Pcuint):TStatus;cdecl;external libX11;
function XQueryBestTile(para1:PDisplay; para2:TDrawable; para3:cuint; para4:cuint; para5:Pcuint;
           para6:Pcuint):TStatus;cdecl;external libX11;
function XQueryColor(para1:PDisplay; para2:TColormap; para3:PXColor):cint;cdecl;external libX11;
function XQueryColors(para1:PDisplay; para2:TColormap; para3:PXColor; para4:cint):cint;cdecl;external libX11;
<<<<<<< HEAD
<<<<<<< HEAD
function XQueryExtension(para1:PDisplay; para2:Pchar; para3:Pcint; para4:Pcint; para5:Pcint):TBoolResult;cdecl;external libX11;
{?}
function XQueryKeymap(para1:PDisplay; para2:pchararr32):cint;cdecl;external libX11;
function XQueryPointer(para1:PDisplay; para2:TWindow; para3:PWindow; para4:PWindow; para5:Pcint;
           para6:Pcint; para7:Pcint; para8:Pcint; para9:Pcuint):TBoolResult;cdecl;external libX11;
=======
=======
>>>>>>> origin/fixes_2_2
function XQueryExtension(para1:PDisplay; para2:Pchar; para3:Pcint; para4:Pcint; para5:Pcint):TBool;cdecl;external libX11;
{?}
function XQueryKeymap(para1:PDisplay; para2:chararr32):cint;cdecl;external libX11;
function XQueryPointer(para1:PDisplay; para2:TWindow; para3:PWindow; para4:PWindow; para5:Pcint;
           para6:Pcint; para7:Pcint; para8:Pcint; para9:Pcuint):TBool;cdecl;external libX11;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
function XQueryTextExtents(para1:PDisplay; para2:TXID; para3:Pchar; para4:cint; para5:Pcint;
           para6:Pcint; para7:Pcint; para8:PXCharStruct):cint;cdecl;external libX11;
function XQueryTextExtents16(para1:PDisplay; para2:TXID; para3:PXChar2b; para4:cint; para5:Pcint;
           para6:Pcint; para7:Pcint; para8:PXCharStruct):cint;cdecl;external libX11;
function XQueryTree(para1:PDisplay; para2:TWindow; para3:PWindow; para4:PWindow; para5:PPWindow;
           para6:Pcuint):TStatus;cdecl;external libX11;
function XRaiseWindow(para1:PDisplay; para2:TWindow):cint;cdecl;external libX11;
function XReadBitmapFile(para1:PDisplay; para2:TDrawable; para3:Pchar; para4:Pcuint; para5:Pcuint;
           para6:PPixmap; para7:Pcint; para8:Pcint):cint;cdecl;external libX11;
function XReadBitmapFileData(para1:Pchar; para2:Pcuint; para3:Pcuint; para4:PPcuchar; para5:Pcint;
           para6:Pcint):cint;cdecl;external libX11;
function XRebindKeysym(para1:PDisplay; para2:TKeySym; para3:PKeySym; para4:cint; para5:Pcuchar;
           para6:cint):cint;cdecl;external libX11;
function XRecolorCursor(para1:PDisplay; para2:TCursor; para3:PXColor; para4:PXColor):cint;cdecl;external libX11;
function XRefreshKeyboardMapping(para1:PXMappingEvent):cint;cdecl;external libX11;
function XRemoveFromSaveSet(para1:PDisplay; para2:TWindow):cint;cdecl;external libX11;
function XRemoveHost(para1:PDisplay; para2:PXHostAddress):cint;cdecl;external libX11;
function XRemoveHosts(para1:PDisplay; para2:PXHostAddress; para3:cint):cint;cdecl;external libX11;
function XReparentWindow(para1:PDisplay; para2:TWindow; para3:TWindow; para4:cint; para5:cint):cint;cdecl;external libX11;
function XResetScreenSaver(para1:PDisplay):cint;cdecl;external libX11;
function XResizeWindow(para1:PDisplay; para2:TWindow; para3:cuint; para4:cuint):cint;cdecl;external libX11;
function XRestackWindows(para1:PDisplay; para2:PWindow; para3:cint):cint;cdecl;external libX11;
function XRotateBuffers(para1:PDisplay; para2:cint):cint;cdecl;external libX11;
function XRotateWindowProperties(para1:PDisplay; para2:TWindow; para3:PAtom; para4:cint; para5:cint):cint;cdecl;external libX11;
function XScreenCount(para1:PDisplay):cint;cdecl;external libX11;
function XSelectInput(ADisplay:PDisplay; AWindow:TWindow; AEventMask:clong):cint;cdecl;external libX11;
function XSendEvent(para1:PDisplay; para2:TWindow; para3:TBool; para4:clong; para5:PXEvent):TStatus;cdecl;external libX11;
function XSetAccessControl(para1:PDisplay; para2:cint):cint;cdecl;external libX11;
function XSetArcMode(para1:PDisplay; para2:TGC; para3:cint):cint;cdecl;external libX11;
function XSetBackground(para1:PDisplay; para2:TGC; para3:culong):cint;cdecl;external libX11;
function XSetClipMask(para1:PDisplay; para2:TGC; para3:TPixmap):cint;cdecl;external libX11;
function XSetClipOrigin(para1:PDisplay; para2:TGC; para3:cint; para4:cint):cint;cdecl;external libX11;
function XSetClipRectangles(para1:PDisplay; para2:TGC; para3:cint; para4:cint; para5:PXRectangle;
           para6:cint; para7:cint):cint;cdecl;external libX11;
function XSetCloseDownMode(para1:PDisplay; para2:cint):cint;cdecl;external libX11;
function XSetCommand(para1:PDisplay; para2:TWindow; para3:PPchar; para4:cint):cint;cdecl;external libX11;
function XSetDashes(para1:PDisplay; para2:TGC; para3:cint; para4:Pchar; para5:cint):cint;cdecl;external libX11;
function XSetFillRule(para1:PDisplay; para2:TGC; para3:cint):cint;cdecl;external libX11;
function XSetFillStyle(para1:PDisplay; para2:TGC; para3:cint):cint;cdecl;external libX11;
function XSetFont(para1:PDisplay; para2:TGC; para3:TFont):cint;cdecl;external libX11;
function XSetFontPath(para1:PDisplay; para2:PPchar; para3:cint):cint;cdecl;external libX11;
function XSetForeground(para1:PDisplay; para2:TGC; para3:culong):cint;cdecl;external libX11;
function XSetFunction(para1:PDisplay; para2:TGC; para3:cint):cint;cdecl;external libX11;
function XSetGraphicsExposures(para1:PDisplay; para2:TGC; para3:TBool):cint;cdecl;external libX11;
function XSetIconName(para1:PDisplay; para2:TWindow; para3:Pchar):cint;cdecl;external libX11;
function XSetInputFocus(para1:PDisplay; para2:TWindow; para3:cint; para4:TTime):cint;cdecl;external libX11;
function XSetLineAttributes(para1:PDisplay; para2:TGC; para3:cuint; para4:cint; para5:cint;
           para6:cint):cint;cdecl;external libX11;
function XSetModifierMapping(para1:PDisplay; para2:PXModifierKeymap):cint;cdecl;external libX11;
function XSetPlaneMask(para1:PDisplay; para2:TGC; para3:culong):cint;cdecl;external libX11;
function XSetPointerMapping(para1:PDisplay; para2:Pcuchar; para3:cint):cint;cdecl;external libX11;
function XSetScreenSaver(para1:PDisplay; para2:cint; para3:cint; para4:cint; para5:cint):cint;cdecl;external libX11;
function XSetSelectionOwner(para1:PDisplay; para2:TAtom; para3:TWindow; para4:TTime):cint;cdecl;external libX11;
function XSetState(para1:PDisplay; para2:TGC; para3:culong; para4:culong; para5:cint;
           para6:culong):cint;cdecl;external libX11;
function XSetStipple(para1:PDisplay; para2:TGC; para3:TPixmap):cint;cdecl;external libX11;
function XSetSubwindowMode(para1:PDisplay; para2:TGC; para3:cint):cint;cdecl;external libX11;
function XSetTSOrigin(para1:PDisplay; para2:TGC; para3:cint; para4:cint):cint;cdecl;external libX11;
function XSetTile(para1:PDisplay; para2:TGC; para3:TPixmap):cint;cdecl;external libX11;
function XSetWindowBackground(para1:PDisplay; para2:TWindow; para3:culong):cint;cdecl;external libX11;
function XSetWindowBackgroundPixmap(para1:PDisplay; para2:TWindow; para3:TPixmap):cint;cdecl;external libX11;
function XSetWindowBorder(para1:PDisplay; para2:TWindow; para3:culong):cint;cdecl;external libX11;
function XSetWindowBorderPixmap(para1:PDisplay; para2:TWindow; para3:TPixmap):cint;cdecl;external libX11;
function XSetWindowBorderWidth(para1:PDisplay; para2:TWindow; para3:cuint):cint;cdecl;external libX11;
function XSetWindowColormap(para1:PDisplay; para2:TWindow; para3:TColormap):cint;cdecl;external libX11;
function XStoreBuffer(para1:PDisplay; para2:Pchar; para3:cint; para4:cint):cint;cdecl;external libX11;
function XStoreBytes(para1:PDisplay; para2:Pchar; para3:cint):cint;cdecl;external libX11;
function XStoreColor(para1:PDisplay; para2:TColormap; para3:PXColor):cint;cdecl;external libX11;
function XStoreColors(para1:PDisplay; para2:TColormap; para3:PXColor; para4:cint):cint;cdecl;external libX11;
function XStoreName(para1:PDisplay; para2:TWindow; para3:Pchar):cint;cdecl;external libX11;
function XStoreNamedColor(para1:PDisplay; para2:TColormap; para3:Pchar; para4:culong; para5:cint):cint;cdecl;external libX11;
function XSync(para1:PDisplay; para2:TBool):cint;cdecl;external libX11;
function XTextExtents(para1:PXFontStruct; para2:Pchar; para3:cint; para4:Pcint; para5:Pcint;
           para6:Pcint; para7:PXCharStruct):cint;cdecl;external libX11;
function XTextExtents16(para1:PXFontStruct; para2:PXChar2b; para3:cint; para4:Pcint; para5:Pcint;
           para6:Pcint; para7:PXCharStruct):cint;cdecl;external libX11;
function XTextWidth(para1:PXFontStruct; para2:Pchar; para3:cint):cint;cdecl;external libX11;
function XTextWidth16(para1:PXFontStruct; para2:PXChar2b; para3:cint):cint;cdecl;external libX11;
function XTranslateCoordinates(ADisplay:PDisplay; ASrcWindow:TWindow; ADestWindow:TWindow; ASrcX:cint; ASrcY:cint;
           ADestXReturn:Pcint; ADestYReturn:Pcint; AChildReturn:PWindow):TBool;cdecl;external libX11;
function XUndefineCursor(para1:PDisplay; para2:TWindow):cint;cdecl;external libX11;
function XUngrabButton(para1:PDisplay; para2:cuint; para3:cuint; para4:TWindow):cint;cdecl;external libX11;
function XUngrabKey(para1:PDisplay; para2:cint; para3:cuint; para4:TWindow):cint;cdecl;external libX11;
function XUngrabKeyboard(para1:PDisplay; para2:TTime):cint;cdecl;external libX11;
function XUngrabPointer(para1:PDisplay; para2:TTime):cint;cdecl;external libX11;
function XUngrabServer(para1:PDisplay):cint;cdecl;external libX11;
function XUninstallColormap(para1:PDisplay; para2:TColormap):cint;cdecl;external libX11;
function XUnloadFont(para1:PDisplay; para2:TFont):cint;cdecl;external libX11;
function XUnmapSubwindows(ADisplay:PDisplay; AWindow:TWindow):cint;cdecl;external libX11;
function XUnmapWindow(ADisplay:PDisplay; AWindow:TWindow):cint;cdecl;external libX11;
function XVendorRelease(para1:PDisplay):cint;cdecl;external libX11;
function XWarpPointer(para1:PDisplay; para2:TWindow; para3:TWindow; para4:cint; para5:cint;
           para6:cuint; para7:cuint; para8:cint; para9:cint):cint;cdecl;external libX11;
function XWidthMMOfScreen(para1:PScreen):cint;cdecl;external libX11;
function XWidthOfScreen(para1:PScreen):cint;cdecl;external libX11;
function XWindowEvent(para1:PDisplay; para2:TWindow; para3:clong; para4:PXEvent):cint;cdecl;external libX11;
function XWriteBitmapFile(para1:PDisplay; para2:Pchar; para3:TPixmap; para4:cuint; para5:cuint;
           para6:cint; para7:cint):cint;cdecl;external libX11;
function XSupportsLocale:TBool;cdecl;external libX11;
function XSetLocaleModifiers(para1:Pchar):Pchar;cdecl;external libX11;
function XOpenOM(para1:PDisplay; para2:PXrmHashBucketRec; para3:Pchar; para4:Pchar):TXOM;cdecl;external libX11;
function XCloseOM(para1:TXOM):TStatus;cdecl;external libX11;
function XSetOMValues(para1:TXOM; dotdotdot:array of const):Pchar;cdecl;external libX11;
function XGetOMValues(para1:TXOM; dotdotdot:array of const):Pchar;cdecl;external libX11;
function XDisplayOfOM(para1:TXOM):PDisplay;cdecl;external libX11;
function XLocaleOfOM(para1:TXOM):Pchar;cdecl;external libX11;
function XCreateOC(para1:TXOM; dotdotdot:array of const):TXOC;cdecl;external libX11;
procedure XDestroyOC(para1:TXOC);cdecl;external libX11;
function XOMOfOC(para1:TXOC):TXOM;cdecl;external libX11;
function XSetOCValues(para1:TXOC; dotdotdot:array of const):Pchar;cdecl;external libX11;
function XGetOCValues(para1:TXOC; dotdotdot:array of const):Pchar;cdecl;external libX11;
function XCreateFontSet(para1:PDisplay; para2:Pchar; para3:PPPchar; para4:Pcint; para5:PPchar):TXFontSet;cdecl;external libX11;
procedure XFreeFontSet(para1:PDisplay; para2:TXFontSet);cdecl;external libX11;
function XFontsOfFontSet(para1:TXFontSet; para2:PPPXFontStruct; para3:PPPchar):cint;cdecl;external libX11;
function XBaseFontNameListOfFontSet(para1:TXFontSet):Pchar;cdecl;external libX11;
function XLocaleOfFontSet(para1:TXFontSet):Pchar;cdecl;external libX11;
<<<<<<< HEAD
<<<<<<< HEAD
function XContextDependentDrawing(para1:TXFontSet):TBoolResult;cdecl;external libX11;
function XDirectionalDependentDrawing(para1:TXFontSet):TBoolResult;cdecl;external libX11;
function XContextualDrawing(para1:TXFontSet):TBoolResult;cdecl;external libX11;
=======
function XContextDependentDrawing(para1:TXFontSet):TBool;cdecl;external libX11;
function XDirectionalDependentDrawing(para1:TXFontSet):TBool;cdecl;external libX11;
function XContextualDrawing(para1:TXFontSet):TBool;cdecl;external libX11;
>>>>>>> graemeg/fixes_2_2
=======
function XContextDependentDrawing(para1:TXFontSet):TBool;cdecl;external libX11;
function XDirectionalDependentDrawing(para1:TXFontSet):TBool;cdecl;external libX11;
function XContextualDrawing(para1:TXFontSet):TBool;cdecl;external libX11;
>>>>>>> origin/fixes_2_2
function XExtentsOfFontSet(para1:TXFontSet):PXFontSetExtents;cdecl;external libX11;
function XmbTextEscapement(para1:TXFontSet; para2:Pchar; para3:cint):cint;cdecl;external libX11;
function XwcTextEscapement(para1:TXFontSet; para2:PWideChar; para3:cint):cint;cdecl;external libX11;
function Xutf8TextEscapement(para1:TXFontSet; para2:Pchar; para3:cint):cint;cdecl;external libX11;
function XmbTextExtents(para1:TXFontSet; para2:Pchar; para3:cint; para4:PXRectangle; para5:PXRectangle):cint;cdecl;external libX11;
function XwcTextExtents(para1:TXFontSet; para2:PWideChar; para3:cint; para4:PXRectangle; para5:PXRectangle):cint;cdecl;external libX11;
function Xutf8TextExtents(para1:TXFontSet; para2:Pchar; para3:cint; para4:PXRectangle; para5:PXRectangle):cint;cdecl;external libX11;
function XmbTextPerCharExtents(para1:TXFontSet; para2:Pchar; para3:cint; para4:PXRectangle; para5:PXRectangle;
           para6:cint; para7:Pcint; para8:PXRectangle; para9:PXRectangle):TStatus;cdecl;external libX11;
function XwcTextPerCharExtents(para1:TXFontSet; para2:PWideChar; para3:cint; para4:PXRectangle; para5:PXRectangle;
           para6:cint; para7:Pcint; para8:PXRectangle; para9:PXRectangle):TStatus;cdecl;external libX11;
function Xutf8TextPerCharExtents(para1:TXFontSet; para2:Pchar; para3:cint; para4:PXRectangle; para5:PXRectangle;
           para6:cint; para7:Pcint; para8:PXRectangle; para9:PXRectangle):TStatus;cdecl;external libX11;
procedure XmbDrawText(para1:PDisplay; para2:TDrawable; para3:TGC; para4:cint; para5:cint;
            para6:PXmbTextItem; para7:cint);cdecl;external libX11;
procedure XwcDrawText(para1:PDisplay; para2:TDrawable; para3:TGC; para4:cint; para5:cint;
            para6:PXwcTextItem; para7:cint);cdecl;external libX11;
procedure Xutf8DrawText(para1:PDisplay; para2:TDrawable; para3:TGC; para4:cint; para5:cint;
            para6:PXmbTextItem; para7:cint);cdecl;external libX11;
procedure XmbDrawString(para1:PDisplay; para2:TDrawable; para3:TXFontSet; para4:TGC; para5:cint;
            para6:cint; para7:Pchar; para8:cint);cdecl;external libX11;
procedure XwcDrawString(para1:PDisplay; para2:TDrawable; para3:TXFontSet; para4:TGC; para5:cint;
            para6:cint; para7:PWideChar; para8:cint);cdecl;external libX11;
procedure Xutf8DrawString(para1:PDisplay; para2:TDrawable; para3:TXFontSet; para4:TGC; para5:cint;
            para6:cint; para7:Pchar; para8:cint);cdecl;external libX11;
procedure XmbDrawImageString(para1:PDisplay; para2:TDrawable; para3:TXFontSet; para4:TGC; para5:cint;
            para6:cint; para7:Pchar; para8:cint);cdecl;external libX11;
procedure XwcDrawImageString(para1:PDisplay; para2:TDrawable; para3:TXFontSet; para4:TGC; para5:cint;
            para6:cint; para7:PWideChar; para8:cint);cdecl;external libX11;
procedure Xutf8DrawImageString(para1:PDisplay; para2:TDrawable; para3:TXFontSet; para4:TGC; para5:cint;
            para6:cint; para7:Pchar; para8:cint);cdecl;external libX11;
<<<<<<< HEAD
<<<<<<< HEAD
function XOpenIM(para1:PDisplay; para2:PXrmHashBucketRec; para3:Pchar; para4:Pchar):PXIM;cdecl;external libX11;
function XCloseIM(para1:PXIM):TStatus;cdecl;external libX11;
=======
function XOpenIM(para1:PDisplay; para2:PXrmHashBucketRec; para3:Pchar; para4:Pchar):TXIM;cdecl;external libX11;
function XCloseIM(para1:TXIM):TStatus;cdecl;external libX11;
>>>>>>> graemeg/fixes_2_2
=======
function XOpenIM(para1:PDisplay; para2:PXrmHashBucketRec; para3:Pchar; para4:Pchar):TXIM;cdecl;external libX11;
function XCloseIM(para1:TXIM):TStatus;cdecl;external libX11;
>>>>>>> origin/fixes_2_2
function XGetIMValues(para1:TXIM; dotdotdot:array of const):Pchar;cdecl;external libX11;
function XSetIMValues(para1:TXIM; dotdotdot:array of const):Pchar;cdecl;external libX11;
function XDisplayOfIM(para1:TXIM):PDisplay;cdecl;external libX11;
function XLocaleOfIM(para1:TXIM):Pchar;cdecl;external libX11;
<<<<<<< HEAD
<<<<<<< HEAD
function XCreateIC(para1:PXIM; dotdotdot:array of const):PXIC;cdecl;external libX11;
procedure XDestroyIC(para1:PXIC);cdecl;external libX11;
procedure XSetICFocus(para1:PXIC);cdecl;external libX11;
procedure XUnsetICFocus(para1:PXIC);cdecl;external libX11;
function XwcResetIC(para1:TXIC):PWideChar;cdecl;external libX11;
function XmbResetIC(para1:TXIC):Pchar;cdecl;external libX11;
function Xutf8ResetIC(para1:PXIC):Pchar;cdecl;external libX11;
function XSetICValues(para1:TXIC; dotdotdot:array of const):Pchar;cdecl;external libX11;
function XGetICValues(para1:TXIC; dotdotdot:array of const):Pchar;cdecl;external libX11;
function XIMOfIC(para1:TXIC):TXIM;cdecl;external libX11;
function XFilterEvent(para1:PXEvent; para2:TWindow):TBoolResult;cdecl;external libX11;
function XmbLookupString(para1:PXIC; para2:PXKeyPressedEvent; para3:Pchar; para4:cint; para5:PKeySym;
           para6:PStatus):cint;cdecl;external libX11;
function XwcLookupString(para1:PXIC; para2:PXKeyPressedEvent; para3:PWideChar; para4:cint; para5:PKeySym;
           para6:PStatus):cint;cdecl;external libX11;
function Xutf8LookupString(para1:PXIC; para2:PXKeyPressedEvent; para3:Pchar; para4:cint; para5:PKeySym;
           para6:PStatus):cint;cdecl;external libX11;
function XVaCreateNestedList(unused:cint; dotdotdot:array of const):TXVaNestedList;cdecl;external libX11;
function XRegisterIMInstantiateCallback(para1:PDisplay; para2:PXrmHashBucketRec; para3:Pchar; para4:Pchar; para5:TXIDProc;
           para6:TXPointer):TBoolResult;cdecl;external libX11;
function XUnregisterIMInstantiateCallback(para1:PDisplay; para2:PXrmHashBucketRec; para3:Pchar; para4:Pchar; para5:TXIDProc;
           para6:TXPointer):TBoolResult;cdecl;external libX11;
=======
=======
>>>>>>> origin/fixes_2_2
function XCreateIC(para1:TXIM; dotdotdot:array of const):TXIC;cdecl;external libX11;
procedure XDestroyIC(para1:TXIC);cdecl;external libX11;
procedure XSetICFocus(para1:TXIC);cdecl;external libX11;
procedure XUnsetICFocus(para1:TXIC);cdecl;external libX11;
function XwcResetIC(para1:TXIC):PWideChar;cdecl;external libX11;
function XmbResetIC(para1:TXIC):Pchar;cdecl;external libX11;
function Xutf8ResetIC(para1:TXIC):Pchar;cdecl;external libX11;
function XSetICValues(para1:TXIC; dotdotdot:array of const):Pchar;cdecl;external libX11;
function XGetICValues(para1:TXIC; dotdotdot:array of const):Pchar;cdecl;external libX11;
function XIMOfIC(para1:TXIC):TXIM;cdecl;external libX11;
function XFilterEvent(para1:PXEvent; para2:TWindow):TBool;cdecl;external libX11;
function XmbLookupString(para1:TXIC; para2:PXKeyPressedEvent; para3:Pchar; para4:cint; para5:PKeySym;
           para6:PStatus):cint;cdecl;external libX11;
function XwcLookupString(para1:TXIC; para2:PXKeyPressedEvent; para3:PWideChar; para4:cint; para5:PKeySym;
           para6:PStatus):cint;cdecl;external libX11;
function Xutf8LookupString(para1:TXIC; para2:PXKeyPressedEvent; para3:Pchar; para4:cint; para5:PKeySym;
           para6:PStatus):cint;cdecl;external libX11;
function XVaCreateNestedList(unused:cint; dotdotdot:array of const):TXVaNestedList;cdecl;external libX11;
function XRegisterIMInstantiateCallback(para1:PDisplay; para2:PXrmHashBucketRec; para3:Pchar; para4:Pchar; para5:TXIDProc;
           para6:TXPointer):TBool;cdecl;external libX11;
function XUnregisterIMInstantiateCallback(para1:PDisplay; para2:PXrmHashBucketRec; para3:Pchar; para4:Pchar; para5:TXIDProc;
           para6:TXPointer):TBool;cdecl;external libX11;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
type
   TXConnectionWatchProc = procedure (para1:PDisplay; para2:TXPointer; para3:cint; para4:TBool; para5:PXPointer);cdecl;

function XInternalConnectionNumbers(para1:PDisplay; para2:PPcint; para3:Pcint):TStatus;cdecl;external libX11;
procedure XProcessInternalConnection(para1:PDisplay; para2:cint);cdecl;external libX11;
function XAddConnectionWatch(para1:PDisplay; para2:TXConnectionWatchProc; para3:TXPointer):TStatus;cdecl;external libX11;
procedure XRemoveConnectionWatch(para1:PDisplay; para2:TXConnectionWatchProc; para3:TXPointer);cdecl;external libX11;
procedure XSetAuthorization(para1:Pchar; para2:cint; para3:Pchar; para4:cint);cdecl;external libX11;

{
  _Xmbtowc?
  _Xwctomb?
}

{$ifdef MACROS}
function ConnectionNumber(dpy : PDisplay) : cint;
function RootWindow(dpy : PDisplay; scr : cint) : TWindow;
function DefaultScreen(dpy : PDisplay) : cint;
function DefaultRootWindow(dpy : PDisplay) : TWindow;
function DefaultVisual(dpy : PDisplay; scr : cint) : PVisual;
function DefaultGC(dpy : PDisplay; scr : cint) : TGC;
function BlackPixel(dpy : PDisplay; scr : cint) : culong;
function WhitePixel(dpy : PDisplay; scr : cint) : culong;
function QLength(dpy : PDisplay) : cint;
function DisplayWidth(dpy : PDisplay; scr : cint) : cint;
function DisplayHeight(dpy : PDisplay; scr : cint) : cint;
function DisplayWidthMM(dpy : PDisplay; scr : cint) : cint;
function DisplayHeightMM(dpy : PDisplay; scr : cint) : cint;
function DisplayPlanes(dpy : PDisplay; scr : cint) : cint;
function DisplayCells(dpy : PDisplay; scr : cint) : cint;
function ScreenCount(dpy : PDisplay) : cint;
function ServerVendor(dpy : PDisplay) : Pchar;
function ProtocolVersion(dpy : PDisplay) : cint;
function ProtocolRevision(dpy : PDisplay) : cint;
function VendorRelease(dpy : PDisplay) : cint;
function DisplayString(dpy : PDisplay) : Pchar;
function DefaultDepth(dpy : PDisplay; scr : cint) : cint;
function DefaultColormap(dpy : PDisplay; scr : cint) : TColormap;
function BitmapUnit(dpy : PDisplay) : cint;
function BitmapBitOrder(dpy : PDisplay) : cint;
function BitmapPad(dpy : PDisplay) : cint;
function ImageByteOrder(dpy : PDisplay) : cint;
function NextRequest(dpy : PDisplay) : culong;
function LastKnownRequestProcessed(dpy : PDisplay) : culong;
function ScreenOfDisplay(dpy : PDisplay; scr : cint) : PScreen;
function DefaultScreenOfDisplay(dpy : PDisplay) : PScreen;
function DisplayOfScreen(s : PScreen) : PDisplay;
function RootWindowOfScreen(s : PScreen) : TWindow;
function BlackPixelOfScreen(s : PScreen) : culong;
function WhitePixelOfScreen(s : PScreen) : culong;
function DefaultColormapOfScreen(s : PScreen) : TColormap;
function DefaultDepthOfScreen(s : PScreen) : cint;
function DefaultGCOfScreen(s : PScreen) : TGC;
function DefaultVisualOfScreen(s : PScreen) : PVisual;
function WidthOfScreen(s : PScreen) : cint;
function HeightOfScreen(s : PScreen) : cint;
function WidthMMOfScreen(s : PScreen) : cint;
function HeightMMOfScreen(s : PScreen) : cint;
function PlanesOfScreen(s : PScreen) : cint;
function CellsOfScreen(s : PScreen) : cint;
function MinCmapsOfScreen(s : PScreen) : cint;
function MaxCmapsOfScreen(s : PScreen) : cint;
function DoesSaveUnders(s : PScreen) : TBool;
function DoesBackingStore(s : PScreen) : cint;
function EventMaskOfScreen(s : PScreen) : clong;
function XAllocID(dpy : PDisplay) : TXID;
{$endif MACROS}

<<<<<<< HEAD
<<<<<<< HEAD
// Overloaded functions to handle TBool parameters as actual booleans.
function XClearArea(para1:PDisplay; para2:TWindow; para3:cint; para4:cint; para5:cuint; para6:cuint; para7:Boolean):cint;
function XGetWindowProperty(para1:PDisplay; para2:TWindow; para3:TAtom; para4:clong; para5:clong;
           para6:Boolean; para7:TAtom; para8:PAtom; para9:Pcint; para10:Pculong;
           para11:Pculong; para12:PPcuchar):cint;
function XGrabKeyboard(para1:PDisplay; para2:TWindow; para3:Boolean; para4:cint; para5:cint; para6:TTime):cint;
function XGrabPointer(para1:PDisplay; para2:TWindow; para3:Boolean; para4:cuint; para5:cint;
           para6:cint; para7:TWindow; para8:TCursor; para9:TTime):cint;
function XInternAtom(para1:PDisplay; para2:Pchar; para3:Boolean):TAtom;
function XInternAtoms(para1:PDisplay; para2:PPchar; para3:cint; para4:Boolean; para5:PAtom):TStatus;
function XSendEvent(para1:PDisplay; para2:TWindow; para3:Boolean; para4:clong; para5:PXEvent):TStatus;
function XSetGraphicsExposures(para1:PDisplay; para2:TGC; para3:Boolean):cint;
function XSync(para1:PDisplay; para2:Boolean):cint;
function XSynchronize(para1:PDisplay; para2:Boolean):funcdisp;


=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
implementation

{$ifdef MACROS}
function ConnectionNumber(dpy : PDisplay) : cint;
begin
   ConnectionNumber:=(PXPrivDisplay(dpy))^.fd;
end;

function RootWindow(dpy : PDisplay; scr : cint) : TWindow;
begin
   RootWindow:=(ScreenOfDisplay(dpy,scr))^.root;
end;

function DefaultScreen(dpy : PDisplay) : cint;
begin
   DefaultScreen:=(PXPrivDisplay(dpy))^.default_screen;
end;

function DefaultRootWindow(dpy : PDisplay) : TWindow;
begin
   DefaultRootWindow:=(ScreenOfDisplay(dpy,DefaultScreen(dpy)))^.root;
end;

function DefaultVisual(dpy : PDisplay; scr : cint) : PVisual;
begin
   DefaultVisual:=(ScreenOfDisplay(dpy,scr))^.root_visual;
end;

function DefaultGC(dpy : PDisplay; scr : cint) : TGC;
begin
   DefaultGC:=(ScreenOfDisplay(dpy,scr))^.default_gc;
end;

function BlackPixel(dpy : PDisplay; scr : cint) : culong;
begin
   BlackPixel:=(ScreenOfDisplay(dpy,scr))^.black_pixel;
end;

function WhitePixel(dpy : PDisplay; scr : cint) : culong;
begin
   WhitePixel:=(ScreenOfDisplay(dpy,scr))^.white_pixel;
end;

function QLength(dpy : PDisplay) : cint;
begin
   QLength:=(PXPrivDisplay(dpy))^.qlen;
end;

function DisplayWidth(dpy : PDisplay; scr : cint) : cint;
begin
   DisplayWidth:=(ScreenOfDisplay(dpy,scr))^.width;
end;

function DisplayHeight(dpy : PDisplay; scr : cint) : cint;
begin
   DisplayHeight:=(ScreenOfDisplay(dpy,scr))^.height;
end;

function DisplayWidthMM(dpy : PDisplay; scr : cint) : cint;
begin
   DisplayWidthMM:=(ScreenOfDisplay(dpy,scr))^.mwidth;
end;

function DisplayHeightMM(dpy : PDisplay; scr : cint) : cint;
begin
   DisplayHeightMM:=(ScreenOfDisplay(dpy,scr))^.mheight;
end;

function DisplayPlanes(dpy : PDisplay; scr : cint) : cint;
begin
   DisplayPlanes:=(ScreenOfDisplay(dpy,scr))^.root_depth;
end;

function DisplayCells(dpy : PDisplay; scr : cint) : cint;
begin
   DisplayCells:=(DefaultVisual(dpy,scr))^.map_entries;
end;

function ScreenCount(dpy : PDisplay) : cint;
begin
   ScreenCount:=(PXPrivDisplay(dpy))^.nscreens;
end;

function ServerVendor(dpy : PDisplay) : Pchar;
begin
   ServerVendor:=(PXPrivDisplay(dpy))^.vendor;
end;

function ProtocolVersion(dpy : PDisplay) : cint;
begin
   ProtocolVersion:=(PXPrivDisplay(dpy))^.proto_major_version;
end;

function ProtocolRevision(dpy : PDisplay) : cint;
begin
   ProtocolRevision:=(PXPrivDisplay(dpy))^.proto_minor_version;
end;

function VendorRelease(dpy : PDisplay) : cint;
begin
   VendorRelease:=(PXPrivDisplay(dpy))^.release;
end;

function DisplayString(dpy : PDisplay) : Pchar;
begin
   DisplayString:=(PXPrivDisplay(dpy))^.display_name;
end;

function DefaultDepth(dpy : PDisplay; scr : cint) : cint;
begin
   DefaultDepth:=(ScreenOfDisplay(dpy,scr))^.root_depth;
end;

function DefaultColormap(dpy : PDisplay; scr : cint) : TColormap;
begin
   DefaultColormap:=(ScreenOfDisplay(dpy,scr))^.cmap;
end;

function BitmapUnit(dpy : PDisplay) : cint;
begin
   BitmapUnit:=(PXPrivDisplay(dpy))^.bitmap_unit;
end;

function BitmapBitOrder(dpy : PDisplay) : cint;
begin
   BitmapBitOrder:=(PXPrivDisplay(dpy))^.bitmap_bit_order;
end;

function BitmapPad(dpy : PDisplay) : cint;
begin
   BitmapPad:=(PXPrivDisplay(dpy))^.bitmap_pad;
end;

function ImageByteOrder(dpy : PDisplay) : cint;
begin
   ImageByteOrder:=(PXPrivDisplay(dpy))^.byte_order;
end;

function NextRequest(dpy : PDisplay) : culong;
begin
   NextRequest:=((PXPrivDisplay(dpy))^.request) + 1;
end;

function LastKnownRequestProcessed(dpy : PDisplay) : culong;
begin
   LastKnownRequestProcessed:=(PXPrivDisplay(dpy))^.last_request_read;
end;

function ScreenOfDisplay(dpy : PDisplay; scr : cint) : PScreen;
begin
   ScreenOfDisplay:=@(((PXPrivDisplay(dpy))^.screens)[scr]);
end;

function DefaultScreenOfDisplay(dpy : PDisplay) : PScreen;
begin
   DefaultScreenOfDisplay:=ScreenOfDisplay(dpy,DefaultScreen(dpy));
end;

function DisplayOfScreen(s : PScreen) : PDisplay;
begin
   DisplayOfScreen:=s^.display;
end;

function RootWindowOfScreen(s : PScreen) : TWindow;
begin
   RootWindowOfScreen:=s^.root;
end;

function BlackPixelOfScreen(s : PScreen) : culong;
begin
   BlackPixelOfScreen:=s^.black_pixel;
end;

function WhitePixelOfScreen(s : PScreen) : culong;
begin
   WhitePixelOfScreen:=s^.white_pixel;
end;

function DefaultColormapOfScreen(s : PScreen) : TColormap;
begin
   DefaultColormapOfScreen:=s^.cmap;
end;

function DefaultDepthOfScreen(s : PScreen) : cint;
begin
   DefaultDepthOfScreen:=s^.root_depth;
end;

function DefaultGCOfScreen(s : PScreen) : TGC;
begin
   DefaultGCOfScreen:=s^.default_gc;
end;

function DefaultVisualOfScreen(s : PScreen) : PVisual;
begin
   DefaultVisualOfScreen:=s^.root_visual;
end;

function WidthOfScreen(s : PScreen) : cint;
begin
   WidthOfScreen:=s^.width;
end;

function HeightOfScreen(s : PScreen) : cint;
begin
   HeightOfScreen:=s^.height;
end;

function WidthMMOfScreen(s : PScreen) : cint;
begin
   WidthMMOfScreen:=s^.mwidth;
end;

function HeightMMOfScreen(s : PScreen) : cint;
begin
   HeightMMOfScreen:=s^.mheight;
end;

function PlanesOfScreen(s : PScreen) : cint;
begin
   PlanesOfScreen:=s^.root_depth;
end;

function CellsOfScreen(s : PScreen) : cint;
begin
   CellsOfScreen:=(DefaultVisualOfScreen(s))^.map_entries;
end;

function MinCmapsOfScreen(s : PScreen) : cint;
begin
   MinCmapsOfScreen:=s^.min_maps;
end;

function MaxCmapsOfScreen(s : PScreen) : cint;
begin
   MaxCmapsOfScreen:=s^.max_maps;
end;

function DoesSaveUnders(s : PScreen) : TBool;
begin
   DoesSaveUnders:=s^.save_unders;
end;

function DoesBackingStore(s : PScreen) : cint;
begin
   DoesBackingStore:=s^.backing_store;
end;

function EventMaskOfScreen(s : PScreen) : clong;
begin
   EventMaskOfScreen:=s^.root_input_mask;
end;

function XAllocID(dpy : PDisplay) : TXID;
begin
   XAllocID:=(PXPrivDisplay(dpy))^.resource_alloc(dpy);
end;
{$endif MACROS}

<<<<<<< HEAD
<<<<<<< HEAD
function XClearArea(para1:PDisplay; para2:TWindow; para3:cint; para4:cint; para5:cuint; para6:cuint; para7:Boolean):cint;

begin
  Result:=XClearArea(para1,para2,para3,para4,para5,para6,Ord(Para7));
end;

function XGetWindowProperty(para1: PDisplay; para2: TWindow; para3: TAtom;
  para4: clong; para5: clong; para6: Boolean; para7: TAtom; para8: PAtom;
  para9: Pcint; para10: Pculong; para11: Pculong; para12: PPcuchar): cint;
begin
  Result := XGetWindowProperty(para1,para2,para3,para4,para5,ord(para6),para7,para8,para9,para10,para11,para12);
end;

function XGrabKeyboard(para1: PDisplay; para2: TWindow; para3: Boolean;
  para4: cint; para5: cint; para6: TTime): cint;
begin
  Result:=XGrabKeyboard(para1,para2,Ord(para3),para4,para5,para6);
end;

function XGrabPointer(para1: PDisplay; para2: TWindow; para3: Boolean;
  para4: cuint; para5: cint; para6: cint; para7: TWindow; para8: TCursor;
  para9: TTime): cint;
begin
  Result:=XGrabPointer(para1,para2,Ord(para3),para4,para5,para6,para7,para8,para9);
end;

function XInternAtom(para1:PDisplay; para2:Pchar; para3:Boolean):TAtom;

begin
  Result:=XInternAtom(para1,para2,Ord(para3));
end;

function XInternAtoms(para1:PDisplay; para2:PPchar; para3:cint; para4:Boolean; para5:PAtom):TStatus;

begin
  Result:=XInternAtoms(para1,para2,para3,Ord(para4),para5);
end;

function XSendEvent(para1:PDisplay; para2:TWindow; para3:Boolean; para4:clong; para5:PXEvent):TStatus;

begin
  Result:=XSendEvent(para1,para2,ord(Para3),para4,para5);
end;

function XSetGraphicsExposures(para1:PDisplay; para2:TGC; para3:Boolean):cint;

begin
  Result:=XSetGraphicsExposures(Para1,para2,Ord(Para3));
end;

function XSync(para1:PDisplay; para2:Boolean):cint;

begin
  Result:=XSync(Para1,Ord(Para2));
end;

function XSynchronize(para1:PDisplay; para2:boolean):funcdisp;

begin
  Result:=XSynchronize(para1,Ord(para2));
end;


=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

end.
