{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2006 by Florian Klaempfl
    member of the Free Pascal development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{$mode objfpc}
{$H+}
{$inline on}
unit comconst;

  interface

    resourcestring
      SNoMethod = 'Method ''%s'' is not supported by automation object';
      SOleError = 'OLE error %.8x';
      SVarNotObject = 'Variant does not reference an automation object';
      SDCOMNotInstalled = 'DCOM not installed';

  implementation

end.

