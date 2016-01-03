{
    This file is part of the Free Component Library

    Implementation of TJSONConfig class
    Copyright (c) 2007 Michael Van Canneyt michael@freepascal.org

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{
  TJSONConfig enables applications to use JSON files for storing their
  configuration data
}

{$IFDEF FPC}
{$MODE objfpc}
{$H+}
{$ENDIF}

unit jsonConf;

interface

uses
<<<<<<< HEAD
<<<<<<< HEAD
  SysUtils, Classes, fpjson, jsonscanner, jsonparser;


type
  EJSONConfigError = class(Exception);
=======
=======
>>>>>>> origin/fixes_2_2
  SysUtils, Classes, fpjson, jsonparser;

resourcestring
  SWrongRootName = 'XML file has wrong root element name';

type
  EJSONConfigError = class(Exception);
  TPathFlags = set of (pfHasValue, pfWriteAccess);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

(* ********************************************************************
   "APath" is the path and name of a value: A JSON configuration file 
   is hierachical. "/" is the path delimiter, the part after the last 
   "/" is the name of the value. The path components will be mapped 
   to nested JSON objects, with the name equal to the part. In practice 
   this means that "/my/path/value" will be written as:
   { 
     "my" : {
       "path" : {
         "value" : Value
       }
     }
   }
   ******************************************************************** *)

  { TJSONConfig }

  TJSONConfig = class(TComponent)
  private
    FFilename: String;
<<<<<<< HEAD
<<<<<<< HEAD
    FFormatIndentSize: Integer;
    FFormatoptions: TFormatOptions;
    FFormatted: Boolean;
    FKey: TJSONObject;
    procedure DoSetFilename(const AFilename: String; ForceReload: Boolean);
    procedure SetFilename(const AFilename: String);
    Function StripSlash(Const P : UnicodeString) : UnicodeString;
=======
=======
>>>>>>> origin/fixes_2_2
    FKey: TJSONObject;
    procedure DoSetFilename(const AFilename: String; ForceReload: Boolean);
    procedure SetFilename(const AFilename: String);
    Function StripSlash(P : WideString) : WideString;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  protected
    FJSON: TJSONObject;
    FModified: Boolean;
    procedure Loaded; override;
<<<<<<< HEAD
<<<<<<< HEAD
    function FindPath(Const APath: UnicodeString; AllowCreate : Boolean) : TJSONObject;
    function FindObject(Const APath: UnicodeString; AllowCreate : Boolean) : TJSONObject;
    function FindObject(Const APath: UnicodeString; AllowCreate : Boolean;Out ElName : UnicodeString) : TJSONObject;
    function FindElement(Const APath: UnicodeString; CreateParent : Boolean; AllowObject : Boolean = False) : TJSONData;
    function FindElement(Const APath: UnicodeString; CreateParent : Boolean; out AParent : TJSONObject; Out ElName : UnicodeString; AllowObject : Boolean = False) : TJSONData;
  public
    constructor Create(AOwner: TComponent); override;
    destructor Destroy; override;
    Procedure Reload;
    procedure Clear;
    procedure Flush;    // Writes the JSON file
    procedure OpenKey(const aPath: UnicodeString; AllowCreate : Boolean);
    procedure CloseKey;
    procedure ResetKey;
    Procedure EnumSubKeys(Const APath : UnicodeString; List : TStrings);
    Procedure EnumValues(Const APath : UnicodeString; List : TStrings);

    function  GetValue(const APath: UnicodeString; const ADefault: UnicodeString): UnicodeString; overload;
    function  GetValue(const APath: UnicodeString; ADefault: Integer): Integer; overload;
    function  GetValue(const APath: UnicodeString; ADefault: Int64): Int64; overload;
    function  GetValue(const APath: UnicodeString; ADefault: Boolean): Boolean; overload;
    function  GetValue(const APath: UnicodeString; ADefault: Double): Double; overload;
    Function GetValue(const APath: UnicodeString; AValue: TStrings; Const ADefault: String) : Boolean; overload;
    Function GetValue(const APath: UnicodeString; AValue: TStrings; Const ADefault: TStrings): Boolean; overload;
    procedure SetValue(const APath: UnicodeString; const AValue: UnicodeString); overload;
    procedure SetValue(const APath: UnicodeString; AValue: Integer); overload;
    procedure SetValue(const APath: UnicodeString; AValue: Int64); overload;
    procedure SetValue(const APath: UnicodeString; AValue: Boolean); overload;
    procedure SetValue(const APath: UnicodeString; AValue: Double); overload;
    procedure SetValue(const APath: UnicodeString; AValue: TStrings; AsObject : Boolean = False); overload;

    procedure SetDeleteValue(const APath: UnicodeString; const AValue, DefValue: UnicodeString); overload;
    procedure SetDeleteValue(const APath: UnicodeString; AValue, DefValue: Integer); overload;
    procedure SetDeleteValue(const APath: UnicodeString; AValue, DefValue: Int64); overload;
    procedure SetDeleteValue(const APath: UnicodeString; AValue, DefValue: Boolean); overload;

    procedure DeletePath(const APath: UnicodeString);
    procedure DeleteValue(const APath: UnicodeString);
    property Modified: Boolean read FModified;
  published
    Property Filename: String read FFilename write SetFilename;
    Property Formatted : Boolean Read FFormatted Write FFormatted;
    Property FormatOptions : TFormatOptions Read FFormatoptions Write FFormatOptions Default DefaultFormat;
    Property FormatIndentsize : Integer Read FFormatIndentSize Write FFormatIndentSize Default DefaultIndentSize;
=======
=======
>>>>>>> origin/fixes_2_2
    function FindPath(Const APath: WideString; AllowCreate : Boolean) : TJSONObject;
    function FindObject(Const APath: WideString; AllowCreate : Boolean) : TJSONObject;
    function FindObject(Const APath: WideString; AllowCreate : Boolean;Var ElName : WideString) : TJSONObject;
    function FindElement(Const APath: WideString; CreateParent : Boolean) : TJSONData;
    function FindElement(Const APath: WideString; CreateParent : Boolean; Var AParent : TJSONObject; Var ElName : WideString) : TJSONData;
  public
    constructor Create(AOwner: TComponent); override;
    destructor Destroy; override;
    procedure Clear;
    procedure Flush;    // Writes the JSON file
    procedure OpenKey(const aPath: WideString; AllowCreate : Boolean);
    procedure CloseKey;
    procedure ResetKey;
    Procedure EnumSubKeys(Const APath : String; List : TStrings);
    Procedure EnumValues(Const APath : String; List : TStrings);

    function  GetValue(const APath: WideString; const ADefault: WideString): WideString; overload;
    function  GetValue(const APath: WideString; ADefault: Integer): Integer; overload;
    function  GetValue(const APath: WideString; ADefault: Boolean): Boolean; overload;
    function  GetValue(const APath: WideString; ADefault: Double): Double; overload;
    procedure SetValue(const APath: WideString; const AValue: WideString); overload;
    procedure SetValue(const APath: WideString; AValue: Integer); overload;
    procedure SetValue(const APath: WideString; AValue: Boolean); overload;
    procedure SetValue(const APath: WideString; AValue: Double); overload;

    procedure SetDeleteValue(const APath: WideString; const AValue, DefValue: WideString); overload;
    procedure SetDeleteValue(const APath: WideString; AValue, DefValue: Integer); overload;
    procedure SetDeleteValue(const APath: WideString; AValue, DefValue: Boolean); overload;

    procedure DeletePath(const APath: WideString);
    procedure DeleteValue(const APath: WideString);
    property Modified: Boolean read FModified;
  published
    property Filename: String read FFilename write SetFilename;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  end;


// ===================================================================

implementation

<<<<<<< HEAD
<<<<<<< HEAD
Resourcestring
=======
Const
>>>>>>> graemeg/fixes_2_2
=======
Const
>>>>>>> origin/fixes_2_2
  SErrInvalidJSONFile = '"%s" is not a valid JSON configuration file.';
  SErrCouldNotOpenKey = 'Could not open key "%s".';

constructor TJSONConfig.Create(AOwner: TComponent);
begin
  inherited Create(AOwner);
  FJSON:=TJSONObject.Create;
  FKey:=FJSON;
<<<<<<< HEAD
<<<<<<< HEAD
  FFormatOptions:=DefaultFormat;
  FFormatIndentsize:=DefaultIndentSize;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

destructor TJSONConfig.Destroy;
begin
  if Assigned(FJSON) then
    begin
    Flush;
    FreeANdNil(FJSON);
    end;
  inherited Destroy;
end;

procedure TJSONConfig.Clear;
begin
  FJSON.Clear;
  FKey:=FJSON;
end;

procedure TJSONConfig.Flush;

Var
  F : Text;
<<<<<<< HEAD
<<<<<<< HEAD
  S : TJSONStringType;
  
=======

>>>>>>> graemeg/fixes_2_2
=======

>>>>>>> origin/fixes_2_2
begin
  if Modified then
    begin
    AssignFile(F,FileName);
    Rewrite(F);
    Try
<<<<<<< HEAD
<<<<<<< HEAD
      if Formatted then
        S:=FJSON.FormatJSON(Formatoptions,FormatIndentSize)
      else
        S:=FJSON.AsJSON;
      Writeln(F,S);  
=======
      Writeln(F,FJSON.AsJSON);
>>>>>>> graemeg/fixes_2_2
=======
      Writeln(F,FJSON.AsJSON);
>>>>>>> origin/fixes_2_2
    Finally
      CloseFile(F);
    end;
    FModified := False;
    end;
end;


<<<<<<< HEAD
<<<<<<< HEAD
function TJSONConfig.FindObject(const APath: UnicodeString; AllowCreate: Boolean
  ): TJSONObject;

Var
  Dummy : UnicodeString;
=======
=======
>>>>>>> origin/fixes_2_2
function TJSONConfig.FindObject(Const APath: WideString; AllowCreate : Boolean) : TJSONObject;

Var
  Dummy : WideString;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

begin
  Result:=FindObject(APath,AllowCreate,Dummy);
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TJSONConfig.FindObject(const APath: UnicodeString; AllowCreate: Boolean;
  out ElName: UnicodeString): TJSONObject;

Var
  S,El : UnicodeString;
=======
=======
>>>>>>> origin/fixes_2_2
function TJSONConfig.FindObject(Const APath: WideString; AllowCreate : Boolean;Var ElName : WideString) : TJSONObject;

Var
  S,El : WideString;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  P,I : Integer;
  T : TJSonObject;
  
begin
//  Writeln('Looking for : ', APath);
  S:=APath;
  If Pos('/',S)=1 then
    Result:=FJSON
  else
    Result:=FKey;
  Repeat
    P:=Pos('/',S);
    If (P<>0) then
      begin
      // Only real paths, ignore double slash
      If (P<>1) then
        begin
        El:=Copy(S,1,P-1);
        If (Result.Count=0) then
          I:=-1
        else
<<<<<<< HEAD
<<<<<<< HEAD
          I:=Result.IndexOfName(UTF8Encode(El));
=======
          I:=Result.IndexOfName(El);
>>>>>>> graemeg/fixes_2_2
=======
          I:=Result.IndexOfName(El);
>>>>>>> origin/fixes_2_2
        If (I=-1) then
          // No element with this name.
          begin
          If AllowCreate then
            begin
            // Create new node.
            T:=Result;
            Result:=TJSonObject.Create;
<<<<<<< HEAD
<<<<<<< HEAD
            T.Add(UTF8Encode(El),Result);
=======
            T.Add(El,Result);
>>>>>>> graemeg/fixes_2_2
=======
            T.Add(El,Result);
>>>>>>> origin/fixes_2_2
            end
          else
            Result:=Nil
          end
        else
          // Node found, check if it is an object
          begin
          if (Result.Items[i].JSONtype=jtObject) then
<<<<<<< HEAD
<<<<<<< HEAD
            Result:=Result.Objects[UTF8Encode(el)]
=======
            Result:=Result.Objects[el]
>>>>>>> graemeg/fixes_2_2
=======
            Result:=Result.Objects[el]
>>>>>>> origin/fixes_2_2
          else
            begin
//            Writeln(el,' type wrong');
            If AllowCreate then
              begin
//              Writeln('Creating ',el);
              Result.Delete(I);
              T:=Result;
              Result:=TJSonObject.Create;
<<<<<<< HEAD
<<<<<<< HEAD
              T.Add(UTF8Encode(El),Result);
=======
              T.Add(El,Result);
>>>>>>> graemeg/fixes_2_2
=======
              T.Add(El,Result);
>>>>>>> origin/fixes_2_2
              end
            else
              Result:=Nil
            end;
          end;
        end;
      Delete(S,1,P);
      end;
  Until (P=0) or (Result=Nil);
  ElName:=S;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TJSONConfig.FindElement(const APath: UnicodeString; CreateParent: Boolean; AllowObject : Boolean = False): TJSONData;

Var
  O : TJSONObject;
  ElName : UnicodeString;
  
begin
  Result:=FindElement(APath,CreateParent,O,ElName,AllowObject);
end;

function TJSONConfig.FindElement(const APath: UnicodeString;
  CreateParent: Boolean; out AParent: TJSONObject; out ElName: UnicodeString;
  AllowObject : Boolean = False): TJSONData;
=======
=======
>>>>>>> origin/fixes_2_2
function TJSONConfig.FindElement(Const APath: WideString; CreateParent : Boolean) : TJSONData;

Var
  O : TJSONObject;
  ElName : WideString;
  
begin
  Result:=FindElement(APath,CreateParent,O,ElName);
end;

function TJSONConfig.FindElement(Const APath: WideString; CreateParent : Boolean; Var AParent : TJSONObject; Var ElName : WideString) : TJSONData;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

Var
  I : Integer;

begin
  Result:=Nil;
  Aparent:=FindObject(APath,CreateParent,ElName);
  If Assigned(Aparent) then
    begin
//    Writeln('Found parent, looking for element:',elName);
<<<<<<< HEAD
<<<<<<< HEAD
    I:=AParent.IndexOfName(UTF8Encode(ElName));
//    Writeln('Element index is',I);
    If (I<>-1) And ((AParent.items[I].JSONType<>jtObject) or AllowObject) then
      Result:=AParent.Items[i];
    end;
//  Writeln('Find ',aPath,' in "',FJSON.AsJSOn,'" : ',Elname,' : ',Result<>NIl);
end;


function TJSONConfig.GetValue(const APath: UnicodeString; const ADefault: UnicodeString): UnicodeString;
=======
=======
>>>>>>> origin/fixes_2_2
    I:=AParent.IndexOfName(ElName);
//    Writeln('Element index is',I);
    If (I<>-1) And (AParent.items[I].JSONType<>jtObject) then
      Result:=AParent.Items[i];
    end;
end;


function TJSONConfig.GetValue(const APath: WideString; const ADefault: WideString): WideString;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

var
  El : TJSONData;
  
begin
  El:=FindElement(StripSlash(APath),False);
  If Assigned(El) then
<<<<<<< HEAD
<<<<<<< HEAD
    Result:=El.AsUnicodeString
=======
    Result:=El.AsString
>>>>>>> graemeg/fixes_2_2
=======
    Result:=El.AsString
>>>>>>> origin/fixes_2_2
  else
    Result:=ADefault;
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TJSONConfig.GetValue(const APath: UnicodeString; ADefault: Integer): Integer;
=======
function TJSONConfig.GetValue(const APath: WideString; ADefault: Integer): Integer;
>>>>>>> graemeg/fixes_2_2
=======
function TJSONConfig.GetValue(const APath: WideString; ADefault: Integer): Integer;
>>>>>>> origin/fixes_2_2
var
  El : TJSONData;
  
begin
  El:=FindElement(StripSlash(APath),False);
  If Not Assigned(el) then
    Result:=ADefault
  else if (el is TJSONNumber) then
    Result:=El.AsInteger
  else
    Result:=StrToIntDef(El.AsString,ADefault);
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TJSONConfig.GetValue(const APath: UnicodeString; ADefault: Int64): Int64;
var
  El : TJSONData;

begin
  El:=FindElement(StripSlash(APath),False);
  If Not Assigned(el) then
    Result:=ADefault
  else if (el is TJSONNumber) then
    Result:=El.AsInt64
  else
    Result:=StrToInt64Def(El.AsString,ADefault);
end;

function TJSONConfig.GetValue(const APath: UnicodeString; ADefault: Boolean): Boolean;
=======
function TJSONConfig.GetValue(const APath: WideString; ADefault: Boolean): Boolean;
>>>>>>> graemeg/fixes_2_2
=======
function TJSONConfig.GetValue(const APath: WideString; ADefault: Boolean): Boolean;
>>>>>>> origin/fixes_2_2

var
  El : TJSONData;
  
begin
  El:=FindElement(StripSlash(APath),False);
  If Not Assigned(el) then
    Result:=ADefault
  else if (el is TJSONBoolean) then
    Result:=El.AsBoolean
  else
    Result:=StrToBoolDef(El.AsString,ADefault);
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TJSONConfig.GetValue(const APath: UnicodeString; ADefault: Double): Double;
=======
function TJSONConfig.GetValue(const APath: WideString; ADefault: Double): Double;
>>>>>>> graemeg/fixes_2_2
=======
function TJSONConfig.GetValue(const APath: WideString; ADefault: Double): Double;
>>>>>>> origin/fixes_2_2

var
  El : TJSONData;

begin
  El:=FindElement(StripSlash(APath),False);
  If Not Assigned(el) then
    Result:=ADefault
  else if (el is TJSONNumber) then
    Result:=El.AsFloat
  else
    Result:=StrToFloatDef(El.AsString,ADefault);
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TJSONConfig.GetValue(const APath: UnicodeString; AValue: TStrings;
  const ADefault: String): Boolean;
var
  El : TJSONData;
  D : TJSONEnum;

begin
  AValue.Clear;
  El:=FindElement(StripSlash(APath),False,True);
  Result:=Assigned(el);
  If Not Result then
    begin
    AValue.Text:=ADefault;
    exit;
    end;
  Case El.JSONType of
    jtArray:
      For D in El do
        if D.Value.JSONType in ActualValueJSONTypes then
          AValue.Add(D.Value.AsString);
    jtObject:
      For D in El do
        if D.Value.JSONType in ActualValueJSONTypes then
          AValue.Add(D.Key+'='+D.Value.AsString);
  else
    AValue.Text:=EL.AsString
  end;

end;

function TJSONConfig.GetValue(const APath: UnicodeString; AValue: TStrings;
  const ADefault: TStrings): Boolean;
begin
  Result:=GetValue(APath,AValue,'');
  If Not Result then
    AValue.Assign(ADefault);
end;


procedure TJSONConfig.SetValue(const APath: UnicodeString; const AValue: UnicodeString);

var
  El : TJSONData;
  ElName : UnicodeString;
=======
=======
>>>>>>> origin/fixes_2_2

procedure TJSONConfig.SetValue(const APath: WideString; const AValue: WideString);

var
  El : TJSONData;
  ElName : WideString;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  O : TJSONObject;
  I : integer;
  
begin
  El:=FindElement(StripSlash(APath),True,O,ElName);
  if Assigned(El) and (El.JSONType<>jtString) then
    begin
<<<<<<< HEAD
<<<<<<< HEAD
    I:=O.IndexOfName(UTF8Encode(elName));
=======
    I:=O.IndexOfName(elName);
>>>>>>> graemeg/fixes_2_2
=======
    I:=O.IndexOfName(elName);
>>>>>>> origin/fixes_2_2
    O.Delete(i);
    El:=Nil;
    end;
  If Not Assigned(el) then
    begin
    El:=TJSONString.Create(AValue);
<<<<<<< HEAD
<<<<<<< HEAD
    O.Add(UTF8Encode(ElName),El);
    end
  else
    El.AsUnicodeString:=AValue;
  FModified:=True;
end;

procedure TJSONConfig.SetDeleteValue(const APath: UnicodeString; const AValue, DefValue: UnicodeString);
=======
=======
>>>>>>> origin/fixes_2_2
    O.Add(ElName,El);
    end
  else
    El.AsString:=AVAlue;
  FModified:=True;
end;

procedure TJSONConfig.SetDeleteValue(const APath: WideString; const AValue, DefValue: WideString);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
begin
  if AValue = DefValue then
    DeleteValue(APath)
  else
    SetValue(APath, AValue);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TJSONConfig.SetValue(const APath: UnicodeString; AValue: Integer);

var
  El : TJSONData;
  ElName : UnicodeString;
=======
=======
>>>>>>> origin/fixes_2_2
procedure TJSONConfig.SetValue(const APath: WideString; AValue: Integer);

var
  El : TJSONData;
  ElName : WideString;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  O : TJSONObject;
  I : integer;

begin
  El:=FindElement(StripSlash(APath),True,O,ElName);
  if Assigned(El) and (Not (El is TJSONIntegerNumber)) then
    begin
<<<<<<< HEAD
<<<<<<< HEAD
    I:=O.IndexOfName(UTF8Encode(elName));
=======
    I:=O.IndexOfName(elName);
>>>>>>> graemeg/fixes_2_2
=======
    I:=O.IndexOfName(elName);
>>>>>>> origin/fixes_2_2
    If (I<>-1) then // Normally not needed...
      O.Delete(i);
    El:=Nil;
    end;
  If Not Assigned(el) then
    begin
    El:=TJSONIntegerNumber.Create(AValue);
<<<<<<< HEAD
<<<<<<< HEAD
    O.Add(UTF8Encode(ElName),El);
=======
    O.Add(ElName,El);
>>>>>>> graemeg/fixes_2_2
=======
    O.Add(ElName,El);
>>>>>>> origin/fixes_2_2
    end
  else
    El.AsInteger:=AValue;
  FModified:=True;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TJSONConfig.SetValue(const APath: UnicodeString; AValue: Int64);

var
  El : TJSONData;
  ElName : UnicodeString;
  O : TJSONObject;
  I : integer;

begin
  El:=FindElement(StripSlash(APath),True,O,ElName);
  if Assigned(El) and (Not (El is TJSONInt64Number)) then
    begin
    I:=O.IndexOfName(UTF8Encode(elName));
    If (I<>-1) then // Normally not needed...
      O.Delete(i);
    El:=Nil;
    end;
  If Not Assigned(el) then
    begin
    El:=TJSONInt64Number.Create(AValue);
    O.Add(UTF8Encode(ElName),El);
    end
  else
    El.AsInt64:=AValue;
  FModified:=True;
end;

procedure TJSONConfig.SetDeleteValue(const APath: UnicodeString; AValue,
=======
procedure TJSONConfig.SetDeleteValue(const APath: WideString; AValue,
>>>>>>> graemeg/fixes_2_2
=======
procedure TJSONConfig.SetDeleteValue(const APath: WideString; AValue,
>>>>>>> origin/fixes_2_2
  DefValue: Integer);
begin
  if AValue = DefValue then
    DeleteValue(APath)
  else
    SetValue(APath, AValue);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TJSONConfig.SetDeleteValue(const APath: UnicodeString; AValue,
  DefValue: Int64);
begin
  if AValue = DefValue then
    DeleteValue(APath)
  else
    SetValue(APath, AValue);
end;

procedure TJSONConfig.SetValue(const APath: UnicodeString; AValue: Boolean);

var
  El : TJSONData;
  ElName : UnicodeString;
=======
=======
>>>>>>> origin/fixes_2_2
procedure TJSONConfig.SetValue(const APath: WideString; AValue: Boolean);

var
  El : TJSONData;
  ElName : WideString;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  O : TJSONObject;
  I : integer;

begin
  El:=FindElement(StripSlash(APath),True,O,ElName);
  if Assigned(El) and (el.JSONType<>jtBoolean) then
    begin
<<<<<<< HEAD
<<<<<<< HEAD
    I:=O.IndexOfName(UTF8Encode(elName));
=======
    I:=O.IndexOfName(elName);
>>>>>>> graemeg/fixes_2_2
=======
    I:=O.IndexOfName(elName);
>>>>>>> origin/fixes_2_2
    O.Delete(i);
    El:=Nil;
    end;
  If Not Assigned(el) then
    begin
    El:=TJSONBoolean.Create(AValue);
<<<<<<< HEAD
<<<<<<< HEAD
    O.Add(UTF8Encode(ElName),El);
=======
    O.Add(ElName,El);
>>>>>>> graemeg/fixes_2_2
=======
    O.Add(ElName,El);
>>>>>>> origin/fixes_2_2
    end
  else
    El.AsBoolean:=AValue;
  FModified:=True;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TJSONConfig.SetValue(const APath: UnicodeString; AValue: Double);

var
  El : TJSONData;
  ElName : UnicodeString;
=======
=======
>>>>>>> origin/fixes_2_2
procedure TJSONConfig.SetValue(const APath: WideString; AValue: Double);

var
  El : TJSONData;
  ElName : WideString;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  O : TJSONObject;
  I : integer;

begin
  El:=FindElement(StripSlash(APath),True,O,ElName);
  if Assigned(El) and (Not (El is TJSONFloatNumber)) then
    begin
<<<<<<< HEAD
<<<<<<< HEAD
    I:=O.IndexOfName(UTF8Encode(elName));
=======
    I:=O.IndexOfName(elName);
>>>>>>> graemeg/fixes_2_2
=======
    I:=O.IndexOfName(elName);
>>>>>>> origin/fixes_2_2
    O.Delete(i);
    El:=Nil;
    end;
  If Not Assigned(el) then
    begin
    El:=TJSONFloatNumber.Create(AValue);
<<<<<<< HEAD
<<<<<<< HEAD
    O.Add(UTF8Encode(ElName),El);
=======
    O.Add(ElName,El);
>>>>>>> graemeg/fixes_2_2
=======
    O.Add(ElName,El);
>>>>>>> origin/fixes_2_2
    end
  else
    El.AsFloat:=AValue;
  FModified:=True;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TJSONConfig.SetValue(const APath: UnicodeString; AValue: TStrings; AsObject : Boolean = False);
var
  El : TJSONData;
  ElName : UnicodeString;
  O : TJSONObject;
  I : integer;
  A : TJSONArray;
  N,V : String;
  DoDelete: Boolean;

begin
  El:=FindElement(StripSlash(APath),True,O,ElName,True);
  if Assigned(El) then
    begin
    if AsObject then
      DoDelete:=(Not (El is TJSONObject))
    else
      DoDelete:=(Not (El is TJSONArray));
    if DoDelete then
      begin
      I:=O.IndexOfName(UTF8Encode(elName));
      O.Delete(i);
      El:=Nil;
      end;
    end;
  If Not Assigned(el) then
    begin
    if AsObject then
      El:=TJSONObject.Create
    else
      El:=TJSONArray.Create;
    O.Add(UTF8Encode(ElName),El);
    end;
  if Not AsObject then
    begin
    A:=El as TJSONArray;
    A.Clear;
    For N in Avalue do
      A.Add(N);
    end
  else
    begin
    O:=El as TJSONObject;
    For I:=0 to AValue.Count-1 do
      begin
      AValue.GetNameValue(I,N,V);
      O.Add(N,V);
      end;
    end;
  FModified:=True;
end;

procedure TJSONConfig.SetDeleteValue(const APath: UnicodeString; AValue,
=======
procedure TJSONConfig.SetDeleteValue(const APath: WideString; AValue,
>>>>>>> graemeg/fixes_2_2
=======
procedure TJSONConfig.SetDeleteValue(const APath: WideString; AValue,
>>>>>>> origin/fixes_2_2
  DefValue: Boolean);
begin
  if AValue = DefValue then
    DeleteValue(APath)
  else
    SetValue(APath,AValue);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TJSONConfig.DeletePath(const APath: UnicodeString);

Var
  P : UnicodeString;
  L : integer;
  Node : TJSONObject;
  ElName : UnicodeString;
=======
=======
>>>>>>> origin/fixes_2_2
procedure TJSONConfig.DeletePath(const APath: WideString);

Var
  P : String;
  L : integer;
  Node : TJSONObject;
  ElName : WideString;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  
begin
  P:=StripSlash(APath);
  L:=Length(P);
  If (L>0) then
    begin
    Node := FindObject(P,False,ElName);
    If Assigned(Node) then
      begin
<<<<<<< HEAD
<<<<<<< HEAD
      L:=Node.IndexOfName(UTF8Encode(ElName));
=======
      L:=Node.IndexOfName(ElName);
>>>>>>> graemeg/fixes_2_2
=======
      L:=Node.IndexOfName(ElName);
>>>>>>> origin/fixes_2_2
      If (L<>-1) then
        Node.Delete(L);
      end;
    end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TJSONConfig.DeleteValue(const APath: UnicodeString);
=======
procedure TJSONConfig.DeleteValue(const APath: WideString);
>>>>>>> graemeg/fixes_2_2
=======
procedure TJSONConfig.DeleteValue(const APath: WideString);
>>>>>>> origin/fixes_2_2

begin
  DeletePath(APath);
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TJSONConfig.Reload;

begin
  if Length(Filename) > 0 then
    DoSetFilename(Filename,True);
end;

procedure TJSONConfig.Loaded;
begin
  inherited Loaded;
  Reload;
end;

function TJSONConfig.FindPath(const APath: UnicodeString; AllowCreate: Boolean
  ): TJSONObject;
  
Var
  P : UnicodeString;
=======
=======
>>>>>>> origin/fixes_2_2
procedure TJSONConfig.Loaded;
begin
  inherited Loaded;
  if Length(Filename) > 0 then
    DoSetFilename(Filename,True);
end;

function TJSONConfig.FindPath(const APath: WideString; AllowCreate: Boolean
  ): TJSONObject;
  
Var
  P : WideString;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  L : Integer;
  
begin
  P:=APath;
  L:=Length(P);
  If (L=0) or (P[L]<>'/') then
    P:=P+'/';
  Result:=FindObject(P,AllowCreate);
end;

procedure TJSONConfig.DoSetFilename(const AFilename: String; ForceReload: Boolean);

Var
  P : TJSONParser;
  J : TJSONData;
  F : TFileStream;
  
begin
  if (not ForceReload) and (FFilename = AFilename) then
    exit;
  FFilename := AFilename;

  if csLoading in ComponentState then
    exit;

  Flush;
  If Not FileExists(AFileName) then
    Clear
  else
    begin
    F:=TFileStream.Create(AFileName,fmopenRead);
    try
<<<<<<< HEAD
<<<<<<< HEAD
      P:=TJSONParser.Create(F,[joUTF8,joComments]);
=======
      P:=TJSONParser.Create(F);
>>>>>>> graemeg/fixes_2_2
=======
      P:=TJSONParser.Create(F);
>>>>>>> origin/fixes_2_2
      try
        J:=P.Parse;
        If (J is TJSONObject) then
          begin
          FreeAndNil(FJSON);
          FJSON:=J as TJSONObject;
          FKey:=FJSON;
          end
        else
          Raise EJSONConfigError.CreateFmt(SErrInvalidJSONFile,[AFileName]);
      finally
        P.Free;
      end;
    finally
      F.Free;
    end;
    end;
end;

procedure TJSONConfig.SetFilename(const AFilename: String);
begin
  DoSetFilename(AFilename, False);
end;

<<<<<<< HEAD
<<<<<<< HEAD
function TJSONConfig.StripSlash(const P: UnicodeString): UnicodeString;
=======
function TJSONConfig.StripSlash(P: WideString): WideString;
>>>>>>> graemeg/fixes_2_2
=======
function TJSONConfig.StripSlash(P: WideString): WideString;
>>>>>>> origin/fixes_2_2

Var
  L : Integer;

begin
  L:=Length(P);
  If (L>0) and (P[l]='/') then
    Result:=Copy(P,1,L-1)
  else
    Result:=P;
end;


procedure TJSONConfig.CloseKey;
begin
  ResetKey;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TJSONConfig.OpenKey(const aPath: UnicodeString; AllowCreate: Boolean);

Var
  P : UnicodeString;
  L : Integer;
  
=======
=======
>>>>>>> origin/fixes_2_2
procedure TJSONConfig.OpenKey(const aPath: WideString; AllowCreate: Boolean);

Var
  ElName : WideString;
  P : String;
  L : Integer;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
begin
  P:=APath;
  L:=Length(P);
  If (L=0) then
    FKey:=FJSON
  else
    begin
    if (P[L]<>'/') then
      P:=P+'/';
    FKey:=FindObject(P,AllowCreate);
    If (FKey=Nil) Then
      Raise EJSONConfigError.CreateFmt(SErrCouldNotOpenKey,[APath]);
    end;
end;

procedure TJSONConfig.ResetKey;
begin
  FKey:=FJSON;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TJSONConfig.EnumSubKeys(const APath: UnicodeString; List: TStrings);
=======
procedure TJSONConfig.EnumSubKeys(const APath: String; List: TStrings);
>>>>>>> graemeg/fixes_2_2
=======
procedure TJSONConfig.EnumSubKeys(const APath: String; List: TStrings);
>>>>>>> origin/fixes_2_2

Var
  AKey : TJSONObject;
  I : Integer;
  
begin
  AKey:=FindPath(APath,False);
  If Assigned(AKey) then
    begin
    For I:=0 to AKey.Count-1 do
      If AKey.Items[i] is TJSONObject then
        List.Add(AKey.Names[i]);
    end;
end;

<<<<<<< HEAD
<<<<<<< HEAD
procedure TJSONConfig.EnumValues(const APath: UnicodeString; List: TStrings);
=======
procedure TJSONConfig.EnumValues(const APath: String; List: TStrings);
>>>>>>> graemeg/fixes_2_2
=======
procedure TJSONConfig.EnumValues(const APath: String; List: TStrings);
>>>>>>> origin/fixes_2_2

Var
  AKey : TJSONObject;
  I : Integer;

begin
  AKey:=FindPath(APath,False);
  If Assigned(AKey) then
    begin
    For I:=0 to AKey.Count-1 do
      If Not (AKey.Items[i] is TJSONObject) then
        List.Add(AKey.Names[i]);
    end;
end;


end.
