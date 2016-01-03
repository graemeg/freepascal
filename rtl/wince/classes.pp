{
    This file is part of the Free Component Library (FCL)
    Copyright (c) 1998 by Michael Van Canneyt and Florian Klaempfl

    Classes unit for wince

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

<<<<<<< HEAD
<<<<<<< HEAD
=======
{$define UNICODE}  //ce is unicode only, needed here for classes.inc

>>>>>>> graemeg/fixes_2_2
=======
{$define UNICODE}  //ce is unicode only, needed here for classes.inc

>>>>>>> origin/fixes_2_2
{$mode objfpc}

{ determine the type of the resource/form file }
{$define Win16Res}

unit Classes;

interface

uses
  rtlconsts,
  sysutils,
  types,
{$ifdef FPC_TESTGENERICS}
  fgl,
{$endif}
  typinfo,
  windows;

type
  TWndMethod = procedure(var msg : TMessage) of object;

{$i classesh.inc}

implementation

uses
  sysconst;

{ OS - independent class implementations are in /inc directory. }
{$i classes.inc}

initialization
  CommonInit;

finalization
  CommonCleanup;
end.
