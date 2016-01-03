{ Parsed from Appkit.framework NSClipView.h }

{$ifdef TYPES}
{$ifndef NSCLIPVIEW_PAS_T}
{$define NSCLIPVIEW_PAS_T}

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSCLIPVIEW_PAS_R}
{$define NSCLIPVIEW_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSCLIPVIEW_PAS_F}
{$define NSCLIPVIEW_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSCLIPVIEW_PAS_S}
{$define NSCLIPVIEW_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSClipView = objcclass;
  NSClipViewPointer = ^NSClipView;
  NSClipViewPtr = NSClipViewPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSCLIPVIEW_PAS_C}
{$define NSCLIPVIEW_PAS_C}

{ NSClipView }
  NSClipView = objcclass external (NSView)
  private
    _backgroundColor: NSColor;
    _docView: NSView;
    _docRect: NSRect;
    _oldDocFrame: NSRect;
    _cursor: NSCursor;
    _scrollAnimationHelper: id;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __cvFlags: record
      case byte of
       0: (_anonbitfield_NSClipView0: cuint);
       1: (data: bitpacked record
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __cvFlags: bitpacked record
      case byte of
       0: (_anonbitfield_NSClipView0: cuint);
       1: (
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        _isFlipped: 0..1;
        onlyUncovered: 0..1;
        reflectScroll: 0..1;
        usedByCell: 0..1;
        scrollClipTo: 0..1;
        noCopyOnScroll: 0..1;
        _drawsBackground: 0..1;
        scrollInProgress: 0..1;
        skipRemoveSuperviewCheck: 0..1;
        animateCurrentScroll: 0..1;
        canAnimateScrolls: 0..1;
        nextScrollRelativeToCurrentPosition: 0..1;
        viewBoundsChangedOverridden: 0..1;
        viewFrameChangedOverridden: 0..1;
        documentViewAlignment: 0..((1 shl 4)-1);
        redrawnWhileScrolling: 0..1;
        RESERVED: 0..((1 shl 13)-1);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
       end;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      );
      end;
    
  public
    procedure setBackgroundColor(color: NSColor); message 'setBackgroundColor:';
    function backgroundColor: NSColor; message 'backgroundColor';
    procedure setDrawsBackground(flag: Boolean); message 'setDrawsBackground:';
    function drawsBackground: Boolean; message 'drawsBackground';
    procedure setDocumentView(aView: NSView); message 'setDocumentView:';
    function documentView: id; message 'documentView';
    function documentRect: NSRect; message 'documentRect';
    procedure setDocumentCursor(anObj: NSCursor); message 'setDocumentCursor:';
    function documentCursor: NSCursor; message 'documentCursor';
    function documentVisibleRect: NSRect; message 'documentVisibleRect';
    procedure viewFrameChanged(notification: NSNotification); message 'viewFrameChanged:';
    procedure viewBoundsChanged(notification: NSNotification); message 'viewBoundsChanged:';
    procedure setCopiesOnScroll(flag: Boolean); message 'setCopiesOnScroll:';
    function copiesOnScroll: Boolean; message 'copiesOnScroll';
    function autoscroll(theEvent: NSEvent): Boolean; message 'autoscroll:';
    function constrainScrollPoint(newOrigin: NSPoint): NSPoint; message 'constrainScrollPoint:';
    procedure scrollToPoint(newOrigin: NSPoint); message 'scrollToPoint:';
  end;

{ NSClipViewSuperviewCategory }
  NSClipViewSuperviewCategory = objccategory external (NSView)
    procedure reflectScrolledClipView(aClipView: NSClipView); message 'reflectScrolledClipView:';
    procedure scrollClipView_toPoint(aClipView: NSClipView; aPoint: NSPoint); message 'scrollClipView:toPoint:';
  end;

{$endif}
{$endif}
