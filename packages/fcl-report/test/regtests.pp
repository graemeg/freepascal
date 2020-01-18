unit regtests;
{
  Add this unit to the main program uses clause.
  Add all test units to the uses clause here.
  Avoids messing with the uses clause of the main program(s).
}

// Define USEDEMOS if you want to test & compare rendering of the demos.

{$DEFINE USEDEMOS}

interface

uses
  tcbasereport, tcreportstreamer, tchtmlparser
{$IFDEF USEDEMOS}
  , tcreportgenerator
{$ENDIF}
  ;

implementation
  
end.

