{ Demo for mustache engine with JSON context

  Copyright (C) 2021 michael Van Canneyt michael@freepascal.org

  This source is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as
  published by the Free Software Foundation; either version 2 of the License, or (at your option) any later version.

  This code is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

  A copy of the GNU General Public License is available on the World Wide Web at <http://www.gnu.org/copyleft/gpl.html>. You can
  also obtain it by writing to the Free Software Foundation, Inc., 51 Franklin Street - Fifth Floor, Boston, MA 02110-1335, USA.
}
program demo1;

// jsonparser includes the json parser.

uses jsonparser, fpmustache;

Const
  JSON = '{ "products" : [ {"name" : "BMW" }, {"name" : "Mercedes"}, { "name" : "Audi" }] }';

  // Mock markdown table
  Template =
     '| name |'+sLineBreak+
     '|------|'+sLineBreak+
     '{{#products}}| {{name}} |'+sLineBreak+
     '{{/products}}';

Var
  M : TMustache;

begin
  M:=TMustache.Create(Nil);
  try
   // Json support enabled by default
   Writeln(M.Render(Template,JSON));
  finally
    M.Free;
  end;
end.

