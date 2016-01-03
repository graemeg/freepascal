{ Parsed from Appkit.framework NSColorWell.h }

{$ifdef TYPES}
{$ifndef NSCOLORWELL_PAS_T}
{$define NSCOLORWELL_PAS_T}

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSCOLORWELL_PAS_R}
{$define NSCOLORWELL_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSCOLORWELL_PAS_F}
{$define NSCOLORWELL_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSCOLORWELL_PAS_S}
{$define NSCOLORWELL_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSColorWell = objcclass;
  NSColorWellPointer = ^NSColorWell;
  NSColorWellPtr = NSColorWellPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSCOLORWELL_PAS_C}
{$define NSCOLORWELL_PAS_C}

{ NSColorWell }
  NSColorWell = objcclass external (NSControl)
  private
    _color: NSColor;
    _target: id;
    _action: SEL;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __cwFlags: record
      case byte of
       0: (_anonbitfield_NSColorWell0: cuint);
       1: (data: bitpacked record
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __cwFlags: bitpacked record
      case byte of
       0: (_anonbitfield_NSColorWell0: cuint);
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
        _isActive: 0..1;
        _isBordered: 0..1;
        cantDraw: 0..1;
        isNotContinuous: 0..1;
        reservedColorWell: 0..((1 shl 28)-1);
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
    procedure deactivate; message 'deactivate';
    procedure activate(exclusive: Boolean); message 'activate:';
    function isActive: Boolean; message 'isActive';
    procedure drawWellInside(insideRect: NSRect); message 'drawWellInside:';
    function isBordered: Boolean; message 'isBordered';
    procedure setBordered(flag: Boolean); message 'setBordered:';
    procedure takeColorFrom(sender: id); message 'takeColorFrom:';
    procedure setColor(color_: NSColor); message 'setColor:';
    function color: NSColor; message 'color';
  end;

{$endif}
{$endif}
