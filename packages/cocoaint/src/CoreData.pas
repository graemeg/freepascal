{$mode delphi}
{$modeswitch cvar}
{$modeswitch objectivec1}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$packrecords c}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
unit CoreData;
interface
uses
	ctypes, CocoaAll;
{$linkframework Cocoa}	{ Cocoa is "umbrella" fremework, includes CoreData }
{$define INTERFACE}

{$include coredata/AnonIncludeClassDefinitionsCoredata.inc}

{$define HEADER}
{$include coredata/CoreData.inc}
{$undef HEADER}

{$define TYPES}
{$include coredata/CoreData.inc}
{$undef TYPES}

{$define RECORDS}
{$include coredata/CoreData.inc}
{$undef RECORDS}

{$define FORWARD}
{$include coredata/CoreData.inc}
{$undef FORWARD}

{$define PROTOCOLS}
{$include coredata/CoreData.inc}
{$undef PROTOCOLS}

{$define CLASSES}
{$include coredata/CoreData.inc}
{$undef CLASSES}

{$define FUNCTIONS}
{$include coredata/CoreData.inc}
{$undef FUNCTIONS}

{$define EXTERNAL_SYMBOLS}
{$include coredata/CoreData.inc}
{$undef EXTERNAL_SYMBOLS}

{$define USER_PATCHES}
{$include coredata/CoreData.inc}
{$undef USER_PATCHES}


{$undef INTERFACE}

implementation

{$define IMPLEMENTATION}

{$undef IMPLEMENTATION}

end.
