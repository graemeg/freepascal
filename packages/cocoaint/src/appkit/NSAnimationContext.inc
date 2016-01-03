{ Parsed from Appkit.framework NSAnimationContext.h }

{$ifdef TYPES}
{$ifndef NSANIMATIONCONTEXT_PAS_T}
{$define NSANIMATIONCONTEXT_PAS_T}

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSANIMATIONCONTEXT_PAS_R}
{$define NSANIMATIONCONTEXT_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSANIMATIONCONTEXT_PAS_F}
{$define NSANIMATIONCONTEXT_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSANIMATIONCONTEXT_PAS_S}
{$define NSANIMATIONCONTEXT_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSAnimationContext = objcclass;
  NSAnimationContextPointer = ^NSAnimationContext;
  NSAnimationContextPtr = NSAnimationContextPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSANIMATIONCONTEXT_PAS_C}
{$define NSANIMATIONCONTEXT_PAS_C}

{ NSAnimationContext }
  NSAnimationContext = objcclass external (NSObject)
  private
    _duration: NSTimeInterval;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    _reserved: array[0..(5)-1] of id;
=======
        _reserved: array[0..4] of id;
>>>>>>> graemeg/cpstrnew
=======
        _reserved: array[0..4] of id;
>>>>>>> graemeg/cpstrnew
=======
        _reserved: array[0..4] of id;
>>>>>>> graemeg/cpstrnew
=======
        _reserved: array[0..4] of id;
>>>>>>> origin/cpstrnew
    
  public
    class procedure beginGrouping; message 'beginGrouping';
    class procedure endGrouping; message 'endGrouping';
    class function currentContext: NSAnimationContext; message 'currentContext';
    procedure setDuration(duration_: NSTimeInterval); message 'setDuration:';
    function duration: NSTimeInterval; message 'duration';
  end;

{$endif}
{$endif}
