{
  Translation of the OpenAL headers for FreePascal
  Copyright (C) 2006 by Ivo Steinmann
}

unit openal;

{$mode objfpc}

interface

uses
  ctypes;

{$IFDEF WINDOWS}
  {$DEFINE DYNLINK}
{$ENDIF}

{$IFDEF DYNLINK}
const
{$IF Defined(WINDOWS)}
  openallib = 'openal32.dll';
{$ELSEIF Defined(UNIX)}
  openallib = 'libopenal.so';
{$ELSE}
  {$MESSAGE ERROR 'DYNLINK not supported'}
{$IFEND}
{$ELSE}
  {$LINKLIB openal}
{$ENDIF}

{$include alh.inc}
{$include alch.inc}
{$include alexth.inc}

implementation

end.