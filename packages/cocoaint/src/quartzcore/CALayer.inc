{ Parsed from Quartzcore.framework CALayer.h }

{$ifdef TYPES}
{$ifndef CALAYER_PAS_T}
{$define CALAYER_PAS_T}

{ Constants }

const
  kCALayerNotSizable = 0;
  kCALayerMinXMargin = 1 shl 0;
  kCALayerWidthSizable = 1 shl 1;
  kCALayerMaxXMargin = 1 shl 2;
  kCALayerMinYMargin = 1 shl 3;
  kCALayerHeightSizable = 1 shl 4;
  kCALayerMaxYMargin = 1 shl 5;

const
  kCALayerLeftEdge = 1 shl 0;
  kCALayerRightEdge = 1 shl 1;
  kCALayerBottomEdge = 1 shl 2;
  kCALayerTopEdge = 1 shl 3;

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef CALAYER_PAS_R}
{$define CALAYER_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef CALAYER_PAS_F}
{$define CALAYER_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef CALAYER_PAS_S}
{$define CALAYER_PAS_S}

{ External string constants }
var
  kCAGravityCenter: NSString; cvar; external;
  kCAGravityTop: NSString; cvar; external;
  kCAGravityBottom: NSString; cvar; external;
  kCAGravityLeft: NSString; cvar; external;
  kCAGravityRight: NSString; cvar; external;
  kCAGravityTopLeft: NSString; cvar; external;
  kCAGravityTopRight: NSString; cvar; external;
  kCAGravityBottomLeft: NSString; cvar; external;
  kCAGravityBottomRight: NSString; cvar; external;
  kCAGravityResize: NSString; cvar; external;
  kCAGravityResizeAspect: NSString; cvar; external;
  kCAGravityResizeAspectFill: NSString; cvar; external;
  kCAFilterNearest: NSString; cvar; external;
  kCAFilterLinear: NSString; cvar; external;
  kCAFilterTrilinear: NSString; cvar; external;
  kCAOnOrderIn: NSString; cvar; external;
  kCAOnOrderOut: NSString; cvar; external;
  kCATransition: NSString; cvar; external;

{$endif}
{$endif}

{$ifdef FORWARD}
  CAActionProtocol = objcprotocol;
  CALayer = objcclass;
  CALayerPointer = ^CALayer;
  CALayerPtr = CALayerPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef CALAYER_PAS_C}
{$define CALAYER_PAS_C}

{ CALayer }
  CALayer = objcclass external (NSObject, NSCodingProtocol, CAMediaTimingProtocol)
  private
    __attr: record
        refcount: cint32;
        flags: cuint32;
        parent: culong;
        sublayers: CALayerArray;
        mask: CALayer;
        state: Pointer;
        previous_state: Pointer;
        animations: Pointer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        slots: array[0..(3)-1] of culong;
=======
            slots: array[0..2] of culong;
>>>>>>> graemeg/cpstrnew
=======
            slots: array[0..2] of culong;
>>>>>>> graemeg/cpstrnew
=======
            slots: array[0..2] of culong;
>>>>>>> graemeg/cpstrnew
=======
            slots: array[0..2] of culong;
>>>>>>> origin/cpstrnew
        {$ifdef cpu64}
        reserved: cuint32;
        {$endif}
      end;
    
  public
    class function layer: id; message 'layer';
    function init: id; message 'init';
    function initWithLayer(layer_: id): id; message 'initWithLayer:';
    function presentationLayer: id; message 'presentationLayer';
    function modelLayer: id; message 'modelLayer';
    class function defaultValueForKey(key: NSString): id; message 'defaultValueForKey:';
    class function needsDisplayForKey(key: NSString): Boolean; message 'needsDisplayForKey:';
    function shouldArchiveValueForKey(key: NSString): Boolean; message 'shouldArchiveValueForKey:';
    procedure setBounds (newValue: CGRect); message 'setBounds:';
    function bounds: CGRect; message 'bounds';
    procedure setPosition (newValue: CGPoint); message 'setPosition:';
    function position: CGPoint; message 'position';
    procedure setZPosition (newValue: CGFloat); message 'setZPosition:';
    function zPosition: CGFloat; message 'zPosition';
    procedure setAnchorPoint (newValue: CGPoint); message 'setAnchorPoint:';
    function anchorPoint: CGPoint; message 'anchorPoint';
    procedure setAnchorPointZ (newValue: CGFloat); message 'setAnchorPointZ:';
    function anchorPointZ: CGFloat; message 'anchorPointZ';
    procedure setTransform (newValue: CATransform3D); message 'setTransform:';
    function transform: CATransform3D; message 'transform';
    function affineTransform: CGAffineTransform; message 'affineTransform';
    procedure setAffineTransform(m: CGAffineTransform); message 'setAffineTransform:';
    procedure setFrame (newValue: CGRect); message 'setFrame:';
    function frame: CGRect; message 'frame';
    procedure setHidden (newValue: Boolean); message 'setHidden:';
    function isHidden: Boolean; message 'isHidden';
    procedure setDoubleSided (newValue: Boolean); message 'setDoubleSided:';
    function isDoubleSided: Boolean; message 'isDoubleSided';
    procedure setGeometryFlipped (newValue: Boolean); message 'setGeometryFlipped:';
    function isGeometryFlipped: Boolean; message 'isGeometryFlipped';
    function contentsAreFlipped: Boolean; message 'contentsAreFlipped';
    function superlayer: CALayer; message 'superlayer';
    procedure removeFromSuperlayer; message 'removeFromSuperlayer';
    procedure setSublayers (newValue: NSArray); message 'setSublayers:';
    function sublayers: NSArray; message 'sublayers';
    procedure addSublayer(layer_: CALayer); message 'addSublayer:';
    procedure insertSublayer_atIndex(layer_: CALayer; idx: cuint); message 'insertSublayer:atIndex:';
    procedure insertSublayer_below(layer_: CALayer; sibling: CALayer); message 'insertSublayer:below:';
    procedure insertSublayer_above(layer_: CALayer; sibling: CALayer); message 'insertSublayer:above:';
    procedure replaceSublayer_with(layer_: CALayer; layer_1: CALayer); message 'replaceSublayer:with:';
    procedure setSublayerTransform (newValue: CATransform3D); message 'setSublayerTransform:';
    function sublayerTransform: CATransform3D; message 'sublayerTransform';
    procedure setMask (newValue: CALayer); message 'setMask:';
    function mask: CALayer; message 'mask';
    procedure setMasksToBounds (newValue: Boolean); message 'setMasksToBounds:';
    function masksToBounds: Boolean; message 'masksToBounds';
    function convertPoint_fromLayer(p: CGPoint; l: CALayer): CGPoint; message 'convertPoint:fromLayer:';
    function convertPoint_toLayer(p: CGPoint; l: CALayer): CGPoint; message 'convertPoint:toLayer:';
    function convertRect_fromLayer(r: CGRect; l: CALayer): CGRect; message 'convertRect:fromLayer:';
    function convertRect_toLayer(r: CGRect; l: CALayer): CGRect; message 'convertRect:toLayer:';
    function convertTime_fromLayer(t: CFTimeInterval; l: CALayer): CFTimeInterval; message 'convertTime:fromLayer:';
    function convertTime_toLayer(t: CFTimeInterval; l: CALayer): CFTimeInterval; message 'convertTime:toLayer:';
    function hitTest(p: CGPoint): CALayer; message 'hitTest:';
    function containsPoint(p: CGPoint): Boolean; message 'containsPoint:';
    procedure setContents (newValue: id); message 'setContents:';
    function contents: id; message 'contents';
    procedure setContentsRect (newValue: CGRect); message 'setContentsRect:';
    function contentsRect: CGRect; message 'contentsRect';
    procedure setContentsGravity (newValue: NSString); message 'setContentsGravity:';
    function contentsGravity: NSString; message 'contentsGravity';
    procedure setContentsCenter (newValue: CGRect); message 'setContentsCenter:';
    function contentsCenter: CGRect; message 'contentsCenter';
    procedure setMinificationFilter (newValue: NSString); message 'setMinificationFilter:';
    function minificationFilter: NSString; message 'minificationFilter';
    procedure setMagnificationFilter (newValue: NSString); message 'setMagnificationFilter:';
    function magnificationFilter: NSString; message 'magnificationFilter';
    procedure setMinificationFilterBias (newValue: single); message 'setMinificationFilterBias:';
    function minificationFilterBias: single; message 'minificationFilterBias';
    procedure setOpaque (newValue: Boolean); message 'setOpaque:';
    function isOpaque: Boolean; message 'isOpaque';
    procedure display; message 'display';
    procedure setNeedsDisplay; message 'setNeedsDisplay';
    procedure setNeedsDisplayInRect(r: CGRect); message 'setNeedsDisplayInRect:';
    function needsDisplay: Boolean; message 'needsDisplay';
    procedure displayIfNeeded; message 'displayIfNeeded';
    procedure setNeedsDisplayOnBoundsChange (newValue: Boolean); message 'setNeedsDisplayOnBoundsChange:';
    function needsDisplayOnBoundsChange: Boolean; message 'needsDisplayOnBoundsChange';
    procedure drawInContext(ctx: CGContextRef); message 'drawInContext:';
    procedure renderInContext(ctx: CGContextRef); message 'renderInContext:';
    procedure setEdgeAntialiasingMask (newValue: cuint); message 'setEdgeAntialiasingMask:';
    function edgeAntialiasingMask: cuint; message 'edgeAntialiasingMask';
    procedure setBackgroundColor (newValue: CGColorRef); message 'setBackgroundColor:';
    function backgroundColor: CGColorRef; message 'backgroundColor';
    procedure setCornerRadius (newValue: CGFloat); message 'setCornerRadius:';
    function cornerRadius: CGFloat; message 'cornerRadius';
    procedure setBorderWidth (newValue: CGFloat); message 'setBorderWidth:';
    function borderWidth: CGFloat; message 'borderWidth';
    procedure setBorderColor (newValue: CGColorRef); message 'setBorderColor:';
    function borderColor: CGColorRef; message 'borderColor';
    procedure setOpacity (newValue: single); message 'setOpacity:';
    function opacity: single; message 'opacity';
    procedure setCompositingFilter (newValue: id); message 'setCompositingFilter:';
    function compositingFilter: id; message 'compositingFilter';
    procedure setFilters (newValue: NSArray); message 'setFilters:';
    function filters: NSArray; message 'filters';
    procedure setBackgroundFilters (newValue: NSArray); message 'setBackgroundFilters:';
    function backgroundFilters: NSArray; message 'backgroundFilters';
    procedure setShadowColor (newValue: CGColorRef); message 'setShadowColor:';
    function shadowColor: CGColorRef; message 'shadowColor';
    procedure setShadowOpacity (newValue: single); message 'setShadowOpacity:';
    function shadowOpacity: single; message 'shadowOpacity';
    procedure setShadowOffset (newValue: CGSize); message 'setShadowOffset:';
    function shadowOffset: CGSize; message 'shadowOffset';
    procedure setShadowRadius (newValue: CGFloat); message 'setShadowRadius:';
    function shadowRadius: CGFloat; message 'shadowRadius';
    procedure setAutoresizingMask (newValue: cuint); message 'setAutoresizingMask:';
    function autoresizingMask: cuint; message 'autoresizingMask';
    procedure setLayoutManager (newValue: id); message 'setLayoutManager:';
    function layoutManager: id; message 'layoutManager';
    function preferredFrameSize: CGSize; message 'preferredFrameSize';
    procedure setNeedsLayout; message 'setNeedsLayout';
    function needsLayout: Boolean; message 'needsLayout';
    procedure layoutIfNeeded; message 'layoutIfNeeded';
    procedure layoutSublayers; message 'layoutSublayers';
    procedure resizeSublayersWithOldSize(size: CGSize); message 'resizeSublayersWithOldSize:';
    procedure resizeWithOldSuperlayerSize(size: CGSize); message 'resizeWithOldSuperlayerSize:';
    class function defaultActionForKey(event: NSString): CAActionProtocol; message 'defaultActionForKey:';
    function actionForKey(event: NSString): CAActionProtocol; message 'actionForKey:';
    procedure setActions (newValue: NSDictionary); message 'setActions:';
    function actions: NSDictionary; message 'actions';
    procedure addAnimation_forKey(anim: CAAnimation; key: NSString); message 'addAnimation:forKey:';
    procedure removeAllAnimations; message 'removeAllAnimations';
    procedure removeAnimationForKey(key: NSString); message 'removeAnimationForKey:';
    function animationKeys: NSArray; message 'animationKeys';
    function animationForKey(key: NSString): CAAnimation; message 'animationForKey:';
    procedure setName (newValue: NSString); message 'setName:';
    function name: NSString; message 'name';
    procedure setDelegate (newValue: id); message 'setDelegate:';
    function delegate: id; message 'delegate';
    procedure setStyle (newValue: NSDictionary); message 'setStyle:';
    function style: NSDictionary; message 'style';

    { Adopted Protocols }
    procedure encodeWithCoder(aCoder: NSCoder);
    function initWithCoder(aDecoder: NSCoder): id;
    procedure setBeginTime (newValue: CFTimeInterval);
    function beginTime: CFTimeInterval;
    procedure setDuration (newValue: CFTimeInterval);
    function duration: CFTimeInterval;
    procedure setSpeed (newValue: single);
    function speed: single;
    procedure setTimeOffset (newValue: CFTimeInterval);
    function timeOffset: CFTimeInterval;
    procedure setRepeatCount (newValue: single);
    function repeatCount: single;
    procedure setRepeatDuration (newValue: CFTimeInterval);
    function repeatDuration: CFTimeInterval;
    procedure setAutoreverses (newValue: Boolean);
    function autoreverses: Boolean;
    procedure setFillMode (newValue: NSString);
    function fillMode: NSString;
  end;

{ CALayoutManagerCategory }
  CALayoutManagerCategory = objccategory external (NSObject)
    function preferredSizeOfLayer(layer: CALayer): CGSize; message 'preferredSizeOfLayer:';
    procedure invalidateLayoutOfLayer(layer: CALayer); message 'invalidateLayoutOfLayer:';
    procedure layoutSublayersOfLayer(layer: CALayer); message 'layoutSublayersOfLayer:';
  end;

{ CALayerDelegateCategory }
  CALayerDelegateCategory = objccategory external (NSObject)
    procedure displayLayer(layer: CALayer); message 'displayLayer:';
    procedure drawLayer_inContext(layer: CALayer; ctx: CGContextRef); message 'drawLayer:inContext:';
    function actionForLayer_forKey(layer: CALayer; event: NSString): CAActionProtocol; message 'actionForLayer:forKey:';
  end;

{$endif}
{$endif}
{$ifdef PROTOCOLS}
{$ifndef CALAYER_PAS_P}
{$define CALAYER_PAS_P}
  
{ CAAction Protocol }
  CAActionProtocol = objcprotocol external name 'CAAction'
  end;
{$endif}
{$endif}
