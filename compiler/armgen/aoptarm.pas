{
    Copyright (c) 1998-2020 by Jonas Maebe and Florian Klaempfl, members of the Free Pascal
    Development Team

    This unit implements an ARM optimizer object used commonly for ARM and AAarch64

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

 ****************************************************************************
}

Unit aoptarm;

{$i fpcdefs.inc}

{ $define DEBUG_PREREGSCHEDULER}
{ $define DEBUG_AOPTCPU}

Interface

uses
  cgbase, cgutils, cpubase, aasmtai, aasmcpu,aopt, aoptobj;

Type
  { while ARM and AAarch64 look not very similar at a first glance,
    several optimizations can be shared between both }
  TARMAsmOptimizer = class(TAsmOptimizer)
  End;

Implementation

  uses
    cutils,verbose,globtype,globals,
    systems,
    cpuinfo,
    cgobj,procinfo,
    aasmbase,aasmdata;

end.

