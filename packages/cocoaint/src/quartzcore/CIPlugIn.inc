{ Parsed from Quartzcore.framework CIPlugIn.h }

{$ifdef TYPES}
{$ifndef CIPLUGIN_PAS_T}
{$define CIPLUGIN_PAS_T}

{$endif}
{$endif}

{$ifdef RECORDS}
{$ifndef CIPLUGIN_PAS_R}
{$define CIPLUGIN_PAS_R}

{$endif}
{$endif}

{$ifdef FUNCTIONS}
{$ifndef CIPLUGIN_PAS_F}
{$define CIPLUGIN_PAS_F}

{$endif}
{$endif}

{$ifdef EXTERNAL_SYMBOLS}
{$ifndef CIPLUGIN_PAS_S}
{$define CIPLUGIN_PAS_S}

{$endif}
{$endif}

{$ifdef FORWARD}
  CIPlugIn = objcclass;
  CIPlugInPointer = ^CIPlugIn;
  CIPlugInPtr = CIPlugInPointer;

{$endif}

{$ifdef CLASSES}
{$ifndef CIPLUGIN_PAS_C}
{$define CIPLUGIN_PAS_C}

{ CIPlugIn }
  CIPlugIn = objcclass external (NSObject)
  private
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    _priv: array[0..(8)-1] of Pointer;
=======
        _priv: array[0..7] of Pointer;
>>>>>>> graemeg/cpstrnew
=======
        _priv: array[0..7] of Pointer;
>>>>>>> graemeg/cpstrnew
=======
        _priv: array[0..7] of Pointer;
>>>>>>> graemeg/cpstrnew
=======
        _priv: array[0..7] of Pointer;
>>>>>>> origin/cpstrnew
    
  public
    class procedure loadAllPlugIns; message 'loadAllPlugIns';
    class procedure loadNonExecutablePlugIns; message 'loadNonExecutablePlugIns';
    class procedure loadPlugIn_allowNonExecutable(url: NSURL; allowNonExecutable: Boolean); message 'loadPlugIn:allowNonExecutable:';
  end;

{$endif}
{$endif}
