{ %TARGET=win32,win64 }
{ %NOTE=This test requires an installed OpenOffice }
<<<<<<< HEAD
<<<<<<< HEAD
{ %INTERACTIVE }
{ This test does create Open Office crashes.
  So we restrict it to interactive mode }
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
program ttt;

{$ifdef fpc}
{$mode delphi}
{$endif fpc}

uses
  Windows, SysUtils, Classes, ComObj, ActiveX, Variants;

var StarOffice : Variant;
	Document : Variant;

<<<<<<< HEAD
<<<<<<< HEAD
function TSampleCode_Connect(OleName : string) : boolean;
begin
    if  VarIsEmpty(StarOffice) then
      begin
        try
          Writeln('Trying to connect to ',OleName);
          StarOffice := CreateOleObject(OleName);
        except
          on e : exception do
            begin
              StarOffice:=Unassigned;
              Writeln('Connection to ',OleName,' failed: ',e.message);
            end;
          end;
      end;
=======
=======
>>>>>>> origin/fixes_2_2
function TSampleCode_Connect() : boolean;
begin
    if  VarIsEmpty(StarOffice) then
        StarOffice := CreateOleObject('com.sun.star.ServiceManager');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

    Result := not (VarIsEmpty(StarOffice) or VarIsNull(StarOffice));
end;

function TSampleCode_CreateDocument(bReadOnly : boolean) : boolean;
var
    StarDesktop : Variant;
    LoadParams : Variant;
    CoreReflection : Variant;
    PropertyValue : Variant;
    AutoObject : Variant;
    TextObject : Variant;
    Cursor : Variant;
begin
   StarDesktop := StarOffice.createInstance('com.sun.star.frame.Desktop');

   if (bReadOnly) then begin
        LoadParams := VarArrayCreate([0, 0], varVariant);
        CoreReflection := StarOffice.createInstance('com.sun.star.reflection.CoreReflection');

        CoreReflection.forName('com.sun.star.beans.PropertyValue').
			createObject(PropertyValue); // CoreReflection().forName().createObject() bring to "Illegal qualifier"
        AutoObject := CoreReflection.forName('com.sun.star.beans.PropertyValue');
	AutoObject.createObject(PropertyValue);

	PropertyValue.Name := 'ReadOnly'; 	// "Arg cant be assigned" and
        PropertyValue.Value := true;		//	"Incompatimle types: const string, untyped expected"

        LoadParams[0] := PropertyValue;
   end
   else
        LoadParams := VarArrayCreate([0, -1], varVariant);

   Document := StarDesktop.LoadComponentFromURL( 'private:factory/swriter', '_blank', 0,  LoadParams);
   if not bReadOnly then begin
       TextObject := Document.Text;
       Cursor := TextObject.createTextCursor;
       TextObject.insertString(Cursor,'Output of FPC Test tdispvar1.pp',False);
	  // works with D7, but not FPC
   end;

   Result := not (VarIsEmpty(Document) or VarIsNull(Document));
end;

begin
<<<<<<< HEAD
<<<<<<< HEAD
  CoInitialize(nil);
  if TSampleCode_Connect('com.sun.star.ServiceManager') then
    begin
      if TSampleCode_CreateDocument(false) then
        Document.Close(false);

    end;
  StarOffice:=Unassigned;
  if TSampleCode_Connect('com.sun.star.ServiceManager.NonExisting.Variant.Just.To.Test') then
    begin
      if TSampleCode_CreateDocument(false) then
        Document.Close(false);
    end;
  CoUnInitialize;
=======
	CoInitialize(nil);
	TSampleCode_Connect();
        TSampleCode_CreateDocument(false);
>>>>>>> graemeg/fixes_2_2
=======
	CoInitialize(nil);
	TSampleCode_Connect();
        TSampleCode_CreateDocument(false);
>>>>>>> origin/fixes_2_2
end.
