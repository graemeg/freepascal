{$IFDEF OGC_INTERFACE}
const
  FEATURE_MEDIUM_CANREAD = $00000001;
  FEATURE_MEDIUM_CANWRITE = $00000002;
  FEATURE_GAMECUBE_SLOTA = $00000010;
  FEATURE_GAMECUBE_SLOTB = $00000020;
  FEATURE_GAMECUBE_DVD = $00000040;
  FEATURE_WII_SD = $00000100;
  FEATURE_WII_USB = $00000200;
  FEATURE_WII_DVD = $00000400;

type
  sec_t = cuint32;
  FN_MEDIUM_STARTUP = function: cbool; cdecl;
  FN_MEDIUM_ISINSERTED = function: cbool; cdecl;
  FN_MEDIUM_READSECTORS = function(sector, numSectors: sec_t; buffer: pointer)
    : cbool; cdecl;
  FN_MEDIUM_WRITESECTORS = function(sector, numSectors: sec_t; buffer: pointer)
    : cbool; cdecl;
  FN_MEDIUM_CLEARSTATUS = function: cbool; cdecl;
  FN_MEDIUM_SHUTDOWN = function: cbool; cdecl;

  DISC_INTERFACE_STRUCT = record
    ioType : culong;
    features : culong;
    startup : FN_MEDIUM_STARTUP;
    isInserted : FN_MEDIUM_ISINSERTED;
    readSectors : FN_MEDIUM_READSECTORS;
    writeSectors : FN_MEDIUM_WRITESECTORS;
    clearStatus : FN_MEDIUM_CLEARSTATUS;
    shutdown : FN_MEDIUM_SHUTDOWN;
  end;

  DISC_INTERFACE = DISC_INTERFACE_STRUCT;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  PDISC_INTERFACE = ^DISC_INTERFACE;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$ENDIF}


