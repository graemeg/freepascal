library pas2jslib;

{$mode objfpc}
{$H+}

uses
  SysUtils, Classes, FPPJsSrcMap, Pas2jsFileCache, Pas2jsCompiler, Pas2jsLibCompiler;


exports
  GetPas2JSCompiler,
  FreePas2JSCompiler,
  RunPas2JSCompiler,
  SetPas2JSReadPasCallBack,
  SetPas2JSWriteJSCallBack,
  SetPas2JSReadDirCallBack,
  AddPas2JSDirectoryEntry,
  SetPas2JSUnitAliasCallBack,
  SetPas2JSCompilerLogCallBack,
  GetPas2JSCompilerLastError;

end.

