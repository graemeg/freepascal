{ Parsed from Quartzcore.framework CAConstraintLayoutManager.h }

{$ifdef TYPES}
{$ifndef CACONSTRAINTLAYOUTMANAGER_PAS_T}
{$define CACONSTRAINTLAYOUTMANAGER_PAS_T}

{ Constants }

const
  kCAConstraintMinX = 0;
  kCAConstraintMidX = 1;
  kCAConstraintMaxX = 2;
  kCAConstraintWidth = 3;
  kCAConstraintMinY = 4;
  kCAConstraintMidY = 5;
  kCAConstraintMaxY = 6;
  kCAConstraintHeight = 7;

{ Types }
type
  CAConstraintAttribute = cint;
  CAConstraintAttributePtr = ^CAConstraintAttribute;

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef CACONSTRAINTLAYOUTMANAGER_PAS_R}
{$define CACONSTRAINTLAYOUTMANAGER_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef CACONSTRAINTLAYOUTMANAGER_PAS_F}
{$define CACONSTRAINTLAYOUTMANAGER_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef CACONSTRAINTLAYOUTMANAGER_PAS_S}
{$define CACONSTRAINTLAYOUTMANAGER_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  CAConstraintLayoutManager = objcclass;
  CAConstraintLayoutManagerPointer = ^CAConstraintLayoutManager;
  CAConstraintLayoutManagerPtr = CAConstraintLayoutManagerPointer;
  CAConstraint = objcclass;
  CAConstraintPointer = ^CAConstraint;
  CAConstraintPtr = CAConstraintPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef CACONSTRAINTLAYOUTMANAGER_PAS_C}
{$define CACONSTRAINTLAYOUTMANAGER_PAS_C}

{ CAConstraintLayoutManager }
  CAConstraintLayoutManager = objcclass external (NSObject)
    
  public
    class function layoutManager: id; message 'layoutManager';
  end;

{ CAConstraint }
  CAConstraint = objcclass external (NSObject, NSCodingProtocol)
  private
    _srcId: NSString;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    _anoninternstruct_CAConstraintLayoutManager0: record
      case byte of
       0: (_anonbitfield_CAConstraintLayoutManager0: CAConstraintAttribute);
       1: (data: bitpacked record
        _srcAttr: 0..((1 shl 16)-1);
        _attr: 0..((1 shl 16)-1);
       end;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    _anoninternstruct_CAConstraintLayoutManager0: bitpacked record
      case byte of
       0: (_anonbitfield_CAConstraintLayoutManager0: CAConstraintAttribute);
       1: (
        _srcAttr: 0..((1 shl 16)-1);
        _attr: 0..((1 shl 16)-1);
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
      );
      end;
    _scale, _offset: CGFloat;
    
  public
    class function constraintWithAttribute_relativeTo_attribute_scale_offset(attr: CAConstraintAttribute; srcId: NSString; srcAttr: CAConstraintAttribute; m: CGFloat; c: CGFloat): id; message 'constraintWithAttribute:relativeTo:attribute:scale:offset:';
    class function constraintWithAttribute_relativeTo_attribute_offset(attr: CAConstraintAttribute; srcId: NSString; srcAttr: CAConstraintAttribute; c: CGFloat): id; message 'constraintWithAttribute:relativeTo:attribute:offset:';
    class function constraintWithAttribute_relativeTo_attribute(attr: CAConstraintAttribute; srcId: NSString; srcAttr: CAConstraintAttribute): id; message 'constraintWithAttribute:relativeTo:attribute:';
    function initWithAttribute_relativeTo_attribute_scale_offset(attr: CAConstraintAttribute; srcId: NSString; srcAttr: CAConstraintAttribute; m: CGFloat; c: CGFloat): id; message 'initWithAttribute:relativeTo:attribute:scale:offset:';
    function attribute: CAConstraintAttribute; message 'attribute';
    function sourceName: NSString; message 'sourceName';
    function sourceAttribute: CAConstraintAttribute; message 'sourceAttribute';
    function scale: CGFloat; message 'scale';
    function offset: CGFloat; message 'offset';

    { Adopted Protocols }
    procedure encodeWithCoder(aCoder: NSCoder);
    function initWithCoder(aDecoder: NSCoder): id;
  end;

{ CAConstraintLayoutManager_CALayerCategory }
  CAConstraintLayoutManager_CALayerCategory = objccategory external name 'CAConstraintLayoutManager' (CALayer)
    procedure setConstraints (newValue: NSArray); message 'setConstraints:';
    function constraints: NSArray; message 'constraints';
    procedure addConstraint(c: CAConstraint); message 'addConstraint:';
  end;

{$endif}
{$endif}
