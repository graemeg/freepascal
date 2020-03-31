{
    Copyright (c) 2014 by Florian Klaempfl

    Symbol table overrides for Xtensa

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
unit symcpu;

{$i fpcdefs.inc}

interface

uses
  symconst,symtype,symdef,symsym;

type
  { defs }
  tcpufiledef = class(tfiledef)
  end;
  tcpufiledefclass = class of tcpufiledef;

  tcpuvariantdef = class(tvariantdef)
  end;
  tcpuvariantdefclass = class of tcpuvariantdef;

  tcpuformaldef = class(tformaldef)
  end;
  tcpuformaldefclass = class of tcpuformaldef;

  tcpuforwarddef = class(tforwarddef)
  end;
  tcpuforwarddefclass = class of tcpuforwarddef;

  tcpuundefineddef = class(tundefineddef)
  end;
  tcpuundefineddefclass = class of tcpuundefineddef;

  tcpuerrordef = class(terrordef)
  end;
  tcpuerrordefclass = class of tcpuerrordef;

  tcpupointerdef = class(tpointerdef)
  end;
  tcpupointerdefclass = class of tcpupointerdef;

  tcpurecorddef = class(trecorddef)
  end;
  tcpurecorddefclass = class of tcpurecorddef;

  tcpuimplementedinterface = class(timplementedinterface)
  end;
  tcpuimplementedinterfaceclass = class of tcpuimplementedinterface;

  tcpuobjectdef = class(tobjectdef)
  end;
  tcpuobjectdefclass = class of tcpuobjectdef;

  tcpuclassrefdef = class(tclassrefdef)
  end;
  tcpuclassrefdefclass = class of tcpuclassrefdef;

  tcpuarraydef = class(tarraydef)
  end;
  tcpuarraydefclass = class of tcpuarraydef;

  tcpuorddef = class(torddef)
  end;
  tcpuorddefclass = class of tcpuorddef;

  tcpufloatdef = class(tfloatdef)
  end;
  tcpufloatdefclass = class of tcpufloatdef;

  tcpuprocvardef = class(tprocvardef)
  end;
  tcpuprocvardefclass = class of tcpuprocvardef;

  tcpuprocdef = class(tprocdef)
  end;
  tcpuprocdefclass = class of tcpuprocdef;

  tcpustringdef = class(tstringdef)
  end;
  tcpustringdefclass = class of tcpustringdef;

  tcpuenumdef = class(tenumdef)
  end;
  tcpuenumdefclass = class of tcpuenumdef;

  tcpusetdef = class(tsetdef)
  end;
  tcpusetdefclass = class of tcpusetdef;

  { syms }
  tcpulabelsym = class(tlabelsym)
  end;
  tcpulabelsymclass = class of tcpulabelsym;

  tcpuunitsym = class(tunitsym)
  end;
  tcpuunitsymclass = class of tcpuunitsym;

  tcpuprogramparasym = class(tprogramparasym)
  end;
  tcpuprogramparasymclass = class(tprogramparasym);

  tcpunamespacesym = class(tnamespacesym)
  end;
  tcpunamespacesymclass = class of tcpunamespacesym;

  tcpuprocsym = class(tprocsym)
  end;
  tcpuprocsymclass = class of tcpuprocsym;

  tcputypesym = class(ttypesym)
  end;
  tcpuypesymclass = class of tcputypesym;

  tcpufieldvarsym = class(tfieldvarsym)
  end;
  tcpufieldvarsymclass = class of tcpufieldvarsym;

  tcpulocalvarsym = class(tlocalvarsym)
  end;
  tcpulocalvarsymclass = class of tcpulocalvarsym;

  tcpuparavarsym = class(tparavarsym)
  end;
  tcpuparavarsymclass = class of tcpuparavarsym;

  tcpustaticvarsym = class(tstaticvarsym)
  end;
  tcpustaticvarsymclass = class of tcpustaticvarsym;

  tcpuabsolutevarsym = class(tabsolutevarsym)
  end;
  tcpuabsolutevarsymclass = class of tcpuabsolutevarsym;

  tcpupropertysym = class(tpropertysym)
  end;
  tcpupropertysymclass = class of tcpupropertysym;

  tcpuconstsym = class(tconstsym)
  end;
  tcpuconstsymclass = class of tcpuconstsym;

  tcpuenumsym = class(tenumsym)
  end;
  tcpuenumsymclass = class of tcpuenumsym;

  tcpusyssym = class(tsyssym)
  end;
  tcpusyssymclass = class of tcpusyssym;


const
   pbestrealtype : ^tdef = @s64floattype;


implementation

begin
  { used tdef classes }
  cfiledef:=tcpufiledef;
  cvariantdef:=tcpuvariantdef;
  cformaldef:=tcpuformaldef;
  cforwarddef:=tcpuforwarddef;
  cundefineddef:=tcpuundefineddef;
  cerrordef:=tcpuerrordef;
  cpointerdef:=tcpupointerdef;
  crecorddef:=tcpurecorddef;
  cimplementedinterface:=tcpuimplementedinterface;
  cobjectdef:=tcpuobjectdef;
  cclassrefdef:=tcpuclassrefdef;
  carraydef:=tcpuarraydef;
  corddef:=tcpuorddef;
  cfloatdef:=tcpufloatdef;
  cprocvardef:=tcpuprocvardef;
  cprocdef:=tcpuprocdef;
  cstringdef:=tcpustringdef;
  cenumdef:=tcpuenumdef;
  csetdef:=tcpusetdef;

  { used tsym classes }
  clabelsym:=tcpulabelsym;
  cunitsym:=tcpuunitsym;
  cprogramparasym:=tcpuprogramparasym;
  cnamespacesym:=tcpunamespacesym;
  cprocsym:=tcpuprocsym;
  ctypesym:=tcputypesym;
  cfieldvarsym:=tcpufieldvarsym;
  clocalvarsym:=tcpulocalvarsym;
  cparavarsym:=tcpuparavarsym;
  cstaticvarsym:=tcpustaticvarsym;
  cabsolutevarsym:=tcpuabsolutevarsym;
  cpropertysym:=tcpupropertysym;
  cconstsym:=tcpuconstsym;
  cenumsym:=tcpuenumsym;
  csyssym:=tcpusyssym;
end.

