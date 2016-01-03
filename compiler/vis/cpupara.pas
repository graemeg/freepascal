{
    Copyright (c) 2002 by Florian Klaempfl

    Generates the argument location information for the
    virtual instruction set machine

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
{ Generates the argument location information for 680x0.
}
unit cpupara;

{$i fpcdefs.inc}

  interface

    uses
       cpubase,
       symdef,paramgr;

    type
       { Returns the location for the nr-st 32 Bit int parameter
         if every parameter before is an 32 Bit int parameter as well
         and if the calling conventions for the helper routines of the
         rtl are used.
       }
       tcpuparamanager = class(tparamanager)
          function getintparaloc(nr : longint) : tparalocation;override;
          procedure create_param_loc_info(p : tabstractprocdef);override;
          function getselflocation(p : tabstractprocdef) : tparalocation;override;
       end;

  implementation

    uses
       verbose,
       globals,
       globtype,
       systems,
       cpuinfo,cginfo,cgbase,
       defutil;

    function tcpuparamanager.getintparaloc(nr : longint) : tparalocation;
      begin
      end;

    procedure tcpuparamanager.create_param_loc_info(p : tabstractprocdef);
      var
        param_offset : integer;
        hp : tparaitem;
      begin
      end;

    function tcpuparamanager.getselflocation(p : tabstractprocdef) : tparalocation;
      begin
      end;

begin
   paramanager:=tcpuparamanager.create;
end.
