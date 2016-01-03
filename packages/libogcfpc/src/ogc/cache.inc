
{$IFDEF OGC_INTERFACE}
const
  LC_BASEPREFIX = $e000;
  LC_BASE = ( LC_BASEPREFIX shl 16 );

procedure DCEnable; cdecl; external;

procedure DCDisable; cdecl; external;

procedure DCFreeze; cdecl; external;

procedure DCUnfreeze; cdecl; external;

procedure DCFlashInvalidate; cdecl; external;

procedure DCInvalidateRange(startaddress: pointer; len: cuint32); cdecl; external;

procedure DCFlushRange(startaddress: pointer; len: cuint32); cdecl; external;

procedure DCStoreRange(startaddress: pointer; len: cuint32); cdecl; external;

procedure DCFlushRangeNoSync(startaddress: pointer; len: cuint32); cdecl; external;

procedure DCStoreRangeNoSync(startaddress: pointer; len: cuint32); cdecl; external;

procedure DCZeroRange(startaddress: pointer; len: cuint32); cdecl; external;

procedure DCTouchRange(startaddress: pointer; len: cuint32); cdecl; external;

procedure ICSync; cdecl; external;

procedure ICFlashInvalidate; cdecl; external;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

{$ifdef HW_RVL}
procedure L2Enhance(); cdecl; external;
{$endif}

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure ICEnable; cdecl; external;

procedure ICDisable; cdecl; external;

procedure ICFreeze; cdecl; external;

procedure ICUnfreeze; cdecl; external;

procedure ICBlockInvalidate(startaddress: pointer); cdecl; external;

procedure ICInvalidateRange(startaddress: pointer; len: cuint32); cdecl; external;

procedure LCEnable; cdecl; external;

procedure LCDisable; cdecl; external;

procedure LCLoadBlocks(par0, par1: pointer; par2: cuint32); cdecl; external;

procedure LCStoreBlocks(par0, par1: pointer; par2: cuint32); cdecl; external;

function LCLoadData(par0, par1: pointer; par2: cuint32): cuint32; cdecl; external;

function LCStoreData(par0, par1: pointer; par2: cuint32): cuint32; cdecl; external;

function LCQueueLength: cuint32; cdecl; external;

function LCQueueWait(par0: cuint32): cuint32; cdecl; external;

procedure LCFlushQueue; cdecl; external;

procedure LCAlloc(par0: pointer; par1: cuint32); cdecl; external;

procedure LCAllocNoInvalidate(par0: pointer; par1: cuint32); cdecl; external;

procedure LCAllocOneTag(par0: cBOOL; par1: pointer); cdecl; external;

procedure LCAllocTags(par0: cBOOL; par1: pointer; par2: cuint32); cdecl; external;


function LCGetBase(): cint32; inline;
{$ENDIF OGC_INTERFACE}

{$IFDEF OGC_IMPLEMENTATION}
function LCGetBase(): cint32; inline;
begin
  LCGetBase := cint32(pointer(LC_BASE)^);
end;
{$ENDIF OGC_IMPLEMENTATION}
