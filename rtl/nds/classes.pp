<<<<<<< HEAD
{
    This file is part of the Free Component Library (FCL)
    Copyright (c) 1999-2002 by the Free Pascal development team

    Classes unit for Nintendo DS
    Copyright (c) 2006 by Francesco Lombardi
    
    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{$mode objfpc}

unit Classes;

interface

uses
  sysutils,
  rtlconsts,
  types,
{$ifdef FPC_TESTGENERICS}
  fgl,
{$endif}
  typinfo;

{$i classesh.inc}


implementation


{ OS - independent class implementations are in /inc directory. }
{$i classes.inc}


initialization
  CommonInit;

finalization
  CommonCleanup;

end.
=======
{
    This file is part of the Free Component Library (FCL)
    Copyright (c) 1999-2002 by the Free Pascal development team

    Classes unit for Nintendo DS
    Copyright (c) 2006 by Francesco Lombardi
    
    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{$mode objfpc}

unit Classes;

interface

uses
  sysutils,
  rtlconsts,
  types,
{$ifdef FPC_TESTGENERICS}
  fgl,
{$endif}
  typinfo;

{$i classesh.inc}


implementation


{ OS - independent class implementations are in /inc directory. }
{$i classes.inc}


initialization
  CommonInit;

finalization
  CommonCleanup;

end.
>>>>>>> graemeg/fixes_2_2
