{
    Copyright (c) 1998-2002 by Florian Klaempfl

    Version/target constants

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
unit version;

{$i fpcdefs.inc}

interface

    const
       { version string }
       version_nr = '3';
       release_nr = '3';
       patch_nr   = '1';
       minorpatch = '';

       { word version for ppu file }
       wordversion = ((ord(version_nr)-ord('0')) shl 14)+
                     ((ord(release_nr)-ord('0')) shl 7)+
                     (ord(patch_nr)-ord('0'));

       { date string }
       date_string = {$I %DATE%};

       { source cpu string }
{$ifdef cpui386}
        source_cpu_string = 'i386';
{$endif cpui386}
{$ifdef cpupowerpc32}
        source_cpu_string = 'powerpc';
{$endif cpupowerpc32}
{$ifdef cpupowerpc64}
        source_cpu_string = 'powerpc64';
{$endif cpupowerpc64}
{$ifdef cpum68k}
        source_cpu_string = 'm68k';
{$endif cpum68k}
{$ifdef cpux86_64}
        source_cpu_string = 'x86_64';
{$endif cpux86_64}
{$ifdef cpusparc}
        source_cpu_string = 'sparc';
{$endif cpusparc}
{$ifdef cpusparc64}
        source_cpu_string = 'sparc64';
{$endif cpusparc64}
{$ifdef cpuarm}
        source_cpu_string = 'arm';
{$endif cpuarm}
{$ifdef cpumipseb}
        source_cpu_string = 'mips'{'mipseb'};
{$endif cpumipseb}
{$ifdef cpumipsel}
        source_cpu_string = 'mipsel';
{$endif cpumipsel}
{$ifdef cpuaarch64}
        source_cpu_string = 'aarch64';
{$endif cpuaarch64}
{$ifdef cpuriscv64}
        source_cpu_string = 'riscv64';
{$endif cpuriscv64}
{$ifdef cpuriscv32}
        source_cpu_string = 'riscv32';
{$endif cpuriscv32}
{$ifdef cpuxtensa}
        source_cpu_string = 'xtensa';
{$endif cpuxtensa}
{$ifdef wasm32}
        source_cpu_string = 'wasm32';
{$endif wasm32}

function version_string:string;
function full_version_string:string;


implementation

function version_string:string;
begin
  version_string := version_nr+'.'+release_nr+'.'+patch_nr;
end;


function full_version_string:string;
begin
  full_version_string := version_nr+'.'+release_nr+'.'+patch_nr+minorpatch
{$ifdef REVINC}
  +'-r'+{$i revision.inc}
{$endif REVINC}
  ;
end;

end.
