{
    Copyright (c) 2020 by Jonas Maebe

    Basic Processor information for LLVM

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
{$i fpcdefs.inc}

unit llvmfeatures;

interface

function llvm_constrained_si64tofp_support: boolean;

implementation

uses
  globals,
  llvminfo;

function llvm_constrained_si64tofp_support: boolean;
  begin
{$if defined(i386) or defined(x86_64)}
    { LLVM has a custom implementation for these platforms }
    result:=true;
{$else}
    result:=llvmflag_generic_constrained_si64tofp in llvmversion_properties[current_settings.llvmversion];
{$endif}
  end;


end.

