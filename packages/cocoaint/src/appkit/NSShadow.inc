{ Parsed from Appkit.framework NSShadow.h }

{$ifdef TYPES}
{$ifndef NSSHADOW_PAS_T}
{$define NSSHADOW_PAS_T}

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSSHADOW_PAS_R}
{$define NSSHADOW_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSSHADOW_PAS_F}
{$define NSSHADOW_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSSHADOW_PAS_S}
{$define NSSHADOW_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSShadow = objcclass;
  NSShadowPointer = ^NSShadow;
  NSShadowPtr = NSShadowPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSSHADOW_PAS_C}
{$define NSSHADOW_PAS_C}

{ NSShadow }
  NSShadow = objcclass external (NSObject, NSCopyingProtocol, NSCodingProtocol)
  private
    _shadowFlags: NSUInteger;
    _shadowOffset: NSSize;
    _shadowBlurRadius: CGFloat;
    _shadowColor: NSColor;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    _reservedFloat: array[0..(3)-1] of CGFloat;
=======
        _reservedFloat: array[0..2] of CGFloat;
>>>>>>> graemeg/cpstrnew
=======
        _reservedFloat: array[0..2] of CGFloat;
>>>>>>> graemeg/cpstrnew
=======
        _reservedFloat: array[0..2] of CGFloat;
>>>>>>> graemeg/cpstrnew
=======
        _reservedFloat: array[0..2] of CGFloat;
>>>>>>> origin/cpstrnew
    _reserved: Pointer;
    
  public
    function init: id; message 'init';
    function shadowOffset: NSSize; message 'shadowOffset';
    procedure setShadowOffset(offset: NSSize); message 'setShadowOffset:';
    function shadowBlurRadius: CGFloat; message 'shadowBlurRadius';
    procedure setShadowBlurRadius(val: CGFloat); message 'setShadowBlurRadius:';
    function shadowColor: NSColor; message 'shadowColor';
    procedure setShadowColor(color: NSColor); message 'setShadowColor:';
    procedure set_; message 'set';

    { Adopted Protocols }
    function copyWithZone(zone_: NSZonePtr): id;
    procedure encodeWithCoder(aCoder: NSCoder);
    function initWithCoder(aDecoder: NSCoder): id;
  end;

{$endif}
{$endif}
