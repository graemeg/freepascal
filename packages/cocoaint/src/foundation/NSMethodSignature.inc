{ Parsed from Foundation.framework NSMethodSignature.h }

{$ifdef TYPES}
{$ifndef NSMETHODSIGNATURE_PAS_T}
{$define NSMETHODSIGNATURE_PAS_T}

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSMETHODSIGNATURE_PAS_R}
{$define NSMETHODSIGNATURE_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSMETHODSIGNATURE_PAS_F}
{$define NSMETHODSIGNATURE_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSMETHODSIGNATURE_PAS_S}
{$define NSMETHODSIGNATURE_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSMethodSignature = objcclass;
  NSMethodSignaturePointer = ^NSMethodSignature;
  NSMethodSignaturePtr = NSMethodSignaturePointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSMETHODSIGNATURE_PAS_C}
{$define NSMETHODSIGNATURE_PAS_C}

{ NSMethodSignature }
  NSMethodSignature = objcclass external (NSObject)
  private
    _private: Pointer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    _reserved: array[0..(6)-1] of Pointer;
=======
        _reserved: array[0..5] of Pointer;
>>>>>>> graemeg/cpstrnew
=======
        _reserved: array[0..5] of Pointer;
>>>>>>> graemeg/cpstrnew
=======
        _reserved: array[0..5] of Pointer;
>>>>>>> graemeg/cpstrnew
=======
        _reserved: array[0..5] of Pointer;
>>>>>>> origin/cpstrnew
    
  public
    class function signatureWithObjCTypes(types: PChar): NSMethodSignature; message 'signatureWithObjCTypes:';
    function numberOfArguments: NSUInteger; message 'numberOfArguments';
    function getArgumentTypeAtIndex(idx: NSUInteger): PChar; message 'getArgumentTypeAtIndex:';
    function frameLength: NSUInteger; message 'frameLength';
    function isOneway: Boolean; message 'isOneway';
    function methodReturnType: PChar; message 'methodReturnType';
    function methodReturnLength: NSUInteger; message 'methodReturnLength';
  end;

{$endif}
{$endif}
