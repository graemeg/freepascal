{ Parsed from Foundation.framework NSInvocation.h }

{$ifdef TYPES}
{$ifndef NSINVOCATION_PAS_T}
{$define NSINVOCATION_PAS_T}

{ Constants }

const
  NSObjCNoType = 0;

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSINVOCATION_PAS_R}
{$define NSINVOCATION_PAS_R}

{ Records }
type
  value = record
    _type: NSInteger;
    charValue: char;
    shortValue: cshort;
    longValue: clong;
    longlongValue: clonglong;
    floatValue: single;
    doubleValue: double;
    boolValue: bool;
    selectorValue: SEL;
    objectValue: id;
    pointerValue: Pointer;
    structLocation: Pointer;
    cStringLocation: PChar;
  end deprecated 'in Mac OS X 10.5 and later';
valuePtr = ^value;


{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSINVOCATION_PAS_F}
{$define NSINVOCATION_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSINVOCATION_PAS_S}
{$define NSINVOCATION_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSInvocation = objcclass;
  NSInvocationPointer = ^NSInvocation;
  NSInvocationPtr = NSInvocationPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSINVOCATION_PAS_C}
{$define NSINVOCATION_PAS_C}

{ NSInvocation }
  NSInvocation = objcclass external (NSObject)
  private
    _frame: Pointer;
    _retdata: Pointer;
    _signature: id;
    _container: id;
    _retainedArgs: cuint8;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    _reserved: array[0..(15)-1] of cuint8;
=======
        _reserved: array[0..14] of cuint8;
>>>>>>> graemeg/cpstrnew
=======
        _reserved: array[0..14] of cuint8;
>>>>>>> graemeg/cpstrnew
=======
        _reserved: array[0..14] of cuint8;
>>>>>>> graemeg/cpstrnew
=======
        _reserved: array[0..14] of cuint8;
>>>>>>> origin/cpstrnew
    
  public
    class function invocationWithMethodSignature(sig: NSMethodSignature): NSInvocation; message 'invocationWithMethodSignature:';
    function methodSignature: NSMethodSignature; message 'methodSignature';
    procedure retainArguments; message 'retainArguments';
    function argumentsRetained: Boolean; message 'argumentsRetained';
    function target: id; message 'target';
    procedure setTarget(target_: id); message 'setTarget:';
    function selector: SEL; message 'selector';
    procedure setSelector(selector_: SEL); message 'setSelector:';
    procedure getReturnValue(retLoc: Pointer); message 'getReturnValue:';
    procedure setReturnValue(retLoc: Pointer); message 'setReturnValue:';
    procedure getArgument_atIndex(argumentLocation: Pointer; idx: NSInteger); message 'getArgument:atIndex:';
    procedure setArgument_atIndex(argumentLocation: Pointer; idx: NSInteger); message 'setArgument:atIndex:';
    procedure invoke; message 'invoke';
    procedure invokeWithTarget(target_: id); message 'invokeWithTarget:';
  end;

{$endif}
{$endif}
