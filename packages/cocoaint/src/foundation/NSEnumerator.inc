{ Parsed from Foundation.framework NSEnumerator.h }

{$ifdef TYPES}
{$ifndef NSENUMERATOR_PAS_T}
{$define NSENUMERATOR_PAS_T}

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSENUMERATOR_PAS_R}
{$define NSENUMERATOR_PAS_R}

{ Records }
type
  NSFastEnumerationState = record
    state: culong;
    itemsPtr: idPtr;
    mutationsPtr: pculong;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    extra: array[0..(5)-1] of culong;
=======
        extra: array[0..4] of culong;
>>>>>>> graemeg/cpstrnew
=======
        extra: array[0..4] of culong;
>>>>>>> graemeg/cpstrnew
=======
        extra: array[0..4] of culong;
>>>>>>> graemeg/cpstrnew
=======
        extra: array[0..4] of culong;
>>>>>>> origin/cpstrnew
  end;
NSFastEnumerationStatePtr = ^NSFastEnumerationState;


{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSENUMERATOR_PAS_F}
{$define NSENUMERATOR_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSENUMERATOR_PAS_S}
{$define NSENUMERATOR_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSFastEnumerationProtocol = objcprotocol;
  NSEnumerator = objcclass;
  NSEnumeratorPointer = ^NSEnumerator;
  NSEnumeratorPtr = NSEnumeratorPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSENUMERATOR_PAS_C}
{$define NSENUMERATOR_PAS_C}

{ NSEnumerator }
  NSEnumerator = objcclass external (NSObject, NSFastEnumerationProtocol)
    
  public
    function nextObject: id; message 'nextObject';

    { Adopted Protocols }
    function countByEnumeratingWithState_objects_count(state: NSFastEnumerationStatePtr; stackbuf: idPtr; len: NSUInteger): NSUInteger;
  end;

{ NSExtendedEnumeratorCategory }
  NSExtendedEnumeratorCategory = objccategory external (NSEnumerator)
    function allObjects: NSArray; message 'allObjects';
  end;

{$endif}
{$endif}
{$ifdef PROTOCOLS}
{$ifndef NSENUMERATOR_PAS_P}
{$define NSENUMERATOR_PAS_P}
  
{ NSFastEnumeration Protocol }
  NSFastEnumerationProtocol = objcprotocol external name 'NSFastEnumeration'
    function countByEnumeratingWithState_objects_count(state: NSFastEnumerationStatePtr; stackbuf: idPtr; len: NSUInteger): NSUInteger; message 'countByEnumeratingWithState:objects:count:';
  end;
{$endif}
{$endif}
