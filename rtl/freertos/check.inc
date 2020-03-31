{
  This file checks feature dependencies, so the compiler
  exits with a proper error message before a bunch of errors
  confuse the user.
}
{$ifdef FPC_HAS_FEATURE_COMMANDARGS}
  {$ifndef FPC_HAS_FEATURE_HEAP}
    {$fatal Using commandline arguments requires a heap manager }
  {$endif FPC_HAS_FEATURE_HEAP}
{$endif FPC_HAS_FEATURE_COMMANDARGS}


{$ifdef FPC_HAS_FEATURE_OBJECTS}
  {$ifndef FPC_HAS_FEATURE_HEAP}
    {$fatal Using objects requires a heap manager }
  {$endif FPC_HAS_FEATURE_HEAP}
{$endif FPC_HAS_FEATURE_OBJECTS}


{$ifdef FPC_HAS_FEATURE_EXCEPTIONS}
  {$ifndef FPC_HAS_FEATURE_HEAP}
    {$fatal Using exceptions requires a heap manager }
  {$endif FPC_HAS_FEATURE_HEAP}
{$endif FPC_HAS_FEATURE_EXCEPTIONS}


{$ifdef FPC_HAS_FEATURE_WIDESTRINGS}
  {$ifndef FPC_HAS_FEATURE_HEAP}
    {$fatal Using widestrings requires a heap manager }
  {$endif FPC_HAS_FEATURE_HEAP}
  {$ifndef FPC_HAS_FEATURE_RTTI}
    {$fatal Using widestrings requires rtti }
  {$endif FPC_HAS_FEATURE_RTTI}
{$endif FPC_HAS_FEATURE_WIDESTRINGS}


{$ifdef FPC_HAS_FEATURE_ANSISTRINGS}
  {$ifndef FPC_HAS_FEATURE_HEAP}
    {$fatal Using ansistrings requires a heap manager }
  {$endif FPC_HAS_FEATURE_HEAP}
  {$ifndef FPC_HAS_FEATURE_RTTI}
    {$fatal Using ansistrings requires rtti }
  {$endif FPC_HAS_FEATURE_RTTI}
{$endif FPC_HAS_FEATURE_ANSISTRINGS}


{$ifdef FPC_HAS_FEATURE_DYNARRAYS}
  {$ifndef FPC_HAS_FEATURE_HEAP}
    {$fatal Using dyn. arrays requires a heap manager }
  {$endif FPC_HAS_FEATURE_HEAP}
  {$ifndef FPC_HAS_FEATURE_RTTI}
    {$fatal Using dyn. arrays requires rtti }
  {$endif FPC_HAS_FEATURE_RTTI}
{$endif FPC_HAS_FEATURE_DYNARRAYS}


{$ifdef FPC_HAS_FEATURE_CLASSES}
  {$ifndef FPC_HAS_FEATURE_HEAP}
    {$fatal Using classes requires a heap manager }
  {$endif FPC_HAS_FEATURE_HEAP}
  {$ifndef FPC_HAS_FEATURE_RTTI}
    {$fatal Using classes requires rtti }
  {$endif FPC_HAS_FEATURE_RTTI}
  {$ifndef FPC_HAS_FEATURE_EXCEPTIONS}
    {$fatal Using classes requires exceptions }
  {$endif FPC_HAS_FEATURE_EXCEPTIONS}
{$endif FPC_HAS_FEATURE_CLASSES}


{$ifdef FPC_HAS_FEATURE_VARIANTS}
  {$ifndef FPC_HAS_FEATURE_HEAP}
    {$fatal Using variants requires a heap manager }
  {$endif FPC_HAS_FEATURE_HEAP}
  {$ifndef FPC_HAS_FEATURE_ANSISTRINGS}
    {$fatal Using variants requires ansistrings }
  {$endif FPC_HAS_FEATURE_ANSISTRRING}
  {$ifndef FPC_HAS_FEATURE_WIDESTRINGS}
    {$fatal Using variants requires widestrings }
  {$endif FPC_HAS_FEATURE_WIDESTRINGS}
  {$ifndef FPC_HAS_FEATURE_CLASSES}
    {$fatal Using variants requires classes }
  {$endif FPC_HAS_FEATURE_CLASSES}
{$endif FPC_HAS_FEATURE_VARIANTS}
