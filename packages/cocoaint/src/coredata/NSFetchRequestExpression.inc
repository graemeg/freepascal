{ Parsed from Coredata.framework NSFetchRequestExpression.h }

{$ifdef TYPES}
{$ifndef NSFETCHREQUESTEXPRESSION_PAS_T}
{$define NSFETCHREQUESTEXPRESSION_PAS_T}

{ Constants }

const
  NSFetchRequestExpressionType = 50;

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef NSFETCHREQUESTEXPRESSION_PAS_R}
{$define NSFETCHREQUESTEXPRESSION_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef NSFETCHREQUESTEXPRESSION_PAS_F}
{$define NSFETCHREQUESTEXPRESSION_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef NSFETCHREQUESTEXPRESSION_PAS_S}
{$define NSFETCHREQUESTEXPRESSION_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  NSFetchRequestExpression = objcclass;
  NSFetchRequestExpressionPointer = ^NSFetchRequestExpression;
  NSFetchRequestExpressionPtr = NSFetchRequestExpressionPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef NSFETCHREQUESTEXPRESSION_PAS_C}
{$define NSFETCHREQUESTEXPRESSION_PAS_C}

{ NSFetchRequestExpression }
  NSFetchRequestExpression = objcclass external (NSExpression)
  private
    _reserved1: Pointer;
    _reserved2: Pointer;
    _reserved3: Pointer;
    _reserved4: Pointer;
    _fetchRequest: NSExpression;
    _managedObjectContext: NSExpression;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    __flags: record
      case byte of
       0: (_anonbitfield_NSFetchRequestExpression0: cuint);
       1: (data: bitpacked record
        isCountOnly: 0..1;
        _RESERVED: 0..((1 shl 31)-1);
       end;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    __flags: bitpacked record
      case byte of
       0: (_anonbitfield_NSFetchRequestExpression0: cuint);
       1: (
        isCountOnly: 0..1;
        _RESERVED: 0..((1 shl 31)-1);
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
    
  public
    class function expressionForFetch_context_countOnly(fetch: NSExpression; context: NSExpression; countFlag: Boolean): NSExpression; message 'expressionForFetch:context:countOnly:';
    function requestExpression: NSExpression; message 'requestExpression';
    function contextExpression: NSExpression; message 'contextExpression';
    function isCountOnlyRequest: Boolean; message 'isCountOnlyRequest';
  end;

{$endif}
{$endif}
