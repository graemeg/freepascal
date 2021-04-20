{ Demo for mustache engine with database context

  Copyright (C) 2021 michael Van Canneyt michael@freepascal.org

  This source is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as
  published by the Free Software Foundation; either version 2 of the License, or (at your option) any later version.

  This code is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

  A copy of the GNU General Public License is available on the World Wide Web at <http://www.gnu.org/copyleft/gpl.html>. You can
  also obtain it by writing to the Free Software Foundation, Inc., 51 Franklin Street - Fifth Floor, Boston, MA 02110-1335, USA.
}

program demo2;

uses csvdataset, fpmustache, fpdbmustache;

Const
  // Mock markdown table
  Template =
     '| name | age | '+sLineBreak+
     '|------|------|'+sLineBreak+
     '{{#family}}| {{name}} | {{age}} |'+sLineBreak+
     '{{/family}}';

Var
  M : TMustache;
  C : TMustacheDBContext;
  D : TCSVDataset;

begin
  M:=TMustache.Create(Nil);
  try
    D:=TCSVDataset.Create(Nil);
    D.CSVOptions.FirstLineAsFieldNames:=True;
    D.LoadFromFile('family.csv');
    C:=TMustacheDBContext.Create(Nil);
    C.AddDataset(D,'family');
    M.Template:=Template;
    Writeln(M.Render(C));
  finally
    M.Free;
    D.Free;
    C.Free;
  end;
end.

