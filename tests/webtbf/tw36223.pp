{ %FAIL }

{$mode objfpc}
{$modeswitch multihelpers}

unit tw36223;
interface
uses
  SysUtils;

type
  // ERROR: Compilation raised exception internally
  TMyHelper = class helper for TSomeObject
  end;

implementation

end.
