{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2004 by Marco van de Voort
    Member of the Free Pascal development team

    Aliases for basic types for C interfacing, for reloading them
    in other units from unit unixtype in a typesafe way.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

type
  cint8                  = UnixType.cint8;       pcint8                 = UnixType.pcint8;
  cuint8                 = UnixType.cuint8;      pcuint8                = UnixType.pcuint8;
  cchar                  = UnixType.cchar;       pcchar                 = UnixType.pcchar;
  cschar                 = UnixType.cschar;      pcschar                = UnixType.pcschar;
  cuchar                 = UnixType.cuchar;      pcuchar                = UnixType.pcuchar;

  cint16                 = UnixType.cint16;      pcint16                = UnixType.pcint16;
  cuint16                = UnixType.cuint16;     pcuint16               = UnixType.pcuint16;
  cshort                 = UnixType.cshort;      pcshort                = UnixType.pcshort;
  csshort                = UnixType.csshort;     pcsshort               = UnixType.pcsshort;
  cushort                = UnixType.cushort;     pcushort               = UnixType.pcushort;

  cint32                 = UnixType.cint32;      pcint32                = UnixType.pcint32;
  cuint32                = UnixType.cuint32;     pcuint32               = UnixType.pcuint32;
  cint                   = UnixType.cint;        pcint                  = UnixType.pcint;
  csint                  = UnixType.csint;       pcsint                 = UnixType.pcsint;
  cuint                  = UnixType.cuint;       pcuint                 = UnixType.pcuint;
  csigned                = UnixType.csigned;     pcsigned               = UnixType.pcsigned;
  cunsigned              = UnixType.cunsigned;   pcunsigned             = UnixType.pcunsigned;

  cint64                 = UnixType.cint64;      pcint64                = UnixType.pcint64;
  cuint64                = UnixType.cuint64;     pcuint64               = UnixType.pcuint64;
  clonglong              = UnixType.clonglong;   pclonglong             = UnixType.pclonglong;
  cslonglong             = UnixType.cslonglong;  pcslonglong            = UnixType.pcslonglong;
  culonglong             = UnixType.culonglong;  pculonglong            = UnixType.pculonglong;

  cbool                  = UnixType.cbool;       pcbool                 = UnixType.pcbool;

  clong                  = UnixType.clong;       pclong                 = UnixType.pclong;
  cslong                 = UnixType.cslong;      pcslong                = UnixType.pcslong;
  culong                 = UnixType.culong;      pculong                = UnixType.pculong;

{$ifndef FPUNONE}
  cfloat                 = UnixType.cfloat;      pcfloat                = UnixType.pcfloat;
  cdouble                = UnixType.cdouble;     pcdouble               = UnixType.pcdouble;
//  clongdouble            = UnixType.clongdouble; pclongdouble           = UnixType.pclongdouble;
{$endif}

  csize_t                = UnixType.size_t;      pcsize_t               = UnixType.psize_t;

  coff_t                 = UnixType.TOff;

  coff_t                 = UnixType.TOff;

  coff_t                 = UnixType.TOff;

