{ Parsed from Appkit.framework NSImageView.h }

{$ifdef TYPES}
{$ifndef NSIMAGEVIEW_PAS_T}
{$define NSIMAGEVIEW_PAS_T}

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSIMAGEVIEW_PAS_R}
{$define NSIMAGEVIEW_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSIMAGEVIEW_PAS_F}
{$define NSIMAGEVIEW_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSIMAGEVIEW_PAS_S}
{$define NSIMAGEVIEW_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSImageView = objcclass;
  NSImageViewPointer = ^NSImageView;
  NSImageViewPtr = NSImageViewPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSIMAGEVIEW_PAS_C}
{$define NSIMAGEVIEW_PAS_C}

{ NSImageView }
  NSImageView = objcclass external (NSControl)
  private
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __ivFlags: record
      case byte of
       0: (_anonbitfield_NSImageView0: cuint);
       1: (data: bitpacked record
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __ivFlags: bitpacked record
      case byte of
       0: (_anonbitfield_NSImageView0: cuint);
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
        _unused: 0..((1 shl 26)-1);
        _rejectsMultiFileDrops: 0..1;
        _compatibleScalingAndAlignment: 0..1;
        _reserved: 0..1;
        _overridesDrawing: 0..1;
        _allowsCutCopyPaste: 0..1;
        _editable: 0..1;
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
    _target: id;
    _action: SEL;
    
  public
    function image: NSImage; message 'image';
    procedure setImage(newImage: NSImage); message 'setImage:';
    function imageAlignment: NSImageAlignment; message 'imageAlignment';
    procedure setImageAlignment(newAlign: NSImageAlignment); message 'setImageAlignment:';
    function imageScaling: NSImageScaling; message 'imageScaling';
    procedure setImageScaling(newScaling: NSImageScaling); message 'setImageScaling:';
    function imageFrameStyle: NSImageFrameStyle; message 'imageFrameStyle';
    procedure setImageFrameStyle(newStyle: NSImageFrameStyle); message 'setImageFrameStyle:';
    procedure setEditable(yn: Boolean); message 'setEditable:';
    function isEditable: Boolean; message 'isEditable';
    procedure setAnimates(flag: Boolean); message 'setAnimates:';
    function animates: Boolean; message 'animates';
    function allowsCutCopyPaste: Boolean; message 'allowsCutCopyPaste';
    procedure setAllowsCutCopyPaste(allow: Boolean); message 'setAllowsCutCopyPaste:';
  end;

{$endif}
{$endif}
