{
    pas2jni - JNI bridge generator for Pascal.

    Copyright (c) 2013 by Yury Sidorov.

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

 ****************************************************************************}

unit writer;

{$mode objfpc}{$H+}

interface

//{$define DEBUG}

{$ifdef DEBUG}
{$ASSERTIONS ON}
{$endif}

uses
  Classes, SysUtils, def, contnrs, PPUParser;

const
  MaxMethodPointers = 10000;

type
  { TTextOutStream }

  TTextOutStream = class(TFileStream)
  private
    FIndent: integer;
    FIndStr: string;
    procedure SetIndednt(const AValue: integer);
  public
    procedure Write(const s: ansistring); overload;
    procedure WriteLn(const s: ansistring = ''; ExtraIndent: integer = 0);
    procedure IncI;
    procedure DecI;
    property Indent: integer read FIndent write SetIndednt;
    property SIndent: string read FIndStr;
  end;

  { TClassInfo }

  TClassInfo = class
  public
    Def: TDef;
    Funcs: TObjectList;
    IsCommonClass: boolean;
    constructor Create;
    destructor Destroy; override;
  end;

  { TProcInfo }

  TProcInfo = class
  public
    Name: string;
    JniName: string;
    JniSignature: string;
  end;

  { TClassList }

  TClassList = class(TStringList)
  private
    function GetFullName(const AName: string; Def: TDef): string;
  public
    constructor Create;
    function Add(const AName: string; Def: TDef; Info: TClassInfo): integer;
    function IndexOf(const AName: string; Def: TDef): integer; reintroduce;
    function GetClassName(Index: integer): string;
    function GetClassInfo(Index: integer): TClassInfo;
  end;

  TMatchType = (mtNone, mtExact, mtWildcard, mtParams);

  { TWriter }

  TWriter = class
  private
    Fjs, Fps: TTextOutStream;
    FClasses: TClassList;
    FPkgDir: string;
    FUniqueCnt: integer;
    FThisUnit: TUnitDef;
    FIntegerType: TDef;
    FRecords: TObjectList;
    FRealClasses: TObjectList;

    function DoCheckItem(const ItemName: string): TCheckItemResult;
    procedure WriteClassTable;

    procedure WriteFileComment(st: TTextOutStream);
    function FindInStringList(list: TStringList; const s: string): integer;
    function FindInStringListEx(list: TStringList; const s: string; AllMatch: boolean; out MatchType: TMatchType): integer;

    procedure ProcessRules(d: TDef; const Prefix: string = '');
    function GetUniqueNum: integer;
    function DefToJniType(d: TDef; var err: boolean): string;
    function DefToJniSig(d: TDef): string;
    function DefToJavaType(d: TDef): string;
    function GetJavaClassPath(d: TDef; const AClassName: string = ''): string;
    function JniToPasType(d: TDef; const v: string; CheckNil: boolean): string;
    function PasToJniType(d: TDef; const v: string): string;
    function GetTypeInfoVar(ClassDef: TDef): string;
    function GetClassPrefix(ClassDef: TDef; const AClassName: string = ''): string;
    function IsJavaSimpleType(d: TDef): boolean;
    function IsJavaVarParam(ParamDef: TVarDef): boolean;
    function GetProcDeclaration(d: TProcDef; const ProcName: string = ''; FullTypeNames: boolean = False; InternalParaNames: boolean = False): string;
    function GetJavaProcDeclaration(d: TProcDef; const ProcName: string = ''): string;
    function GetJniFuncType(d: TDef): string;
    function GetJavaClassName(cls: TDef; it: TDef): string;
    procedure RegisterPseudoClass(d: TDef);
    function GetPasIntType(Size: integer): string;
    function GetPasType(d: TDef; FullName: boolean): string;
//    procedure AddCustomProc(ParentDef: TDef; const JniName, Name: string; RetType: TBasicType; const Params: array of TBasicType);
    function AddCustomProc(ParentDef: TDef; const JniName, Name: string; RetType: TBasicType; const Params: array of TBasicType): TProcDef;
    procedure AddNativeMethod(ParentDef: TDef; const JniName, Name, Signature: string);
    function GetProcSignature(d: TProcDef): string;
    procedure EHandlerStart;
    procedure EHandlerEnd(const EnvVarName: string; const ExtraCode: string = '');
    procedure UpdateUsedUnits(u: TUnitDef);

    procedure WriteClassInfoVar(d: TDef);
    procedure WriteComment(d: TDef; const AType: string);
    procedure WriteClass(d: TClassDef; PreInfo: boolean);
    procedure WriteProc(d: TProcDef; Variable: TVarDef = nil; AParent: TDef = nil);
    procedure WriteVar(d: TVarDef; AParent: TDef = nil);
    procedure WriteConst(d: TConstDef);
    procedure WriteEnum(d: TDef);
    procedure WriteProcType(d: TProcDef; PreInfo: boolean);
    procedure WriteSet(d: TSetDef);
    procedure WritePointer(d: TPointerDef; PreInfo: boolean);
    procedure WriteClassRef(d: TClassRefDef; PreInfo: boolean);
    procedure WriteUnit(u: TUnitDef);
    procedure WriteOnLoad;
    procedure WriteRecordSizes;
  public
    SearchPath: string;
    LibName: string;
    JavaPackage: string;
    Units: TStringList;
    OutPath: string;
    JavaOutPath: string;
    IncludeList: TStringList;
    ExcludeList: TStringList;
    LibAutoLoad: boolean;

    constructor Create;
    destructor Destroy; override;
    procedure ProcessUnits;
  end;

implementation

const
  JNIType: array[TBasicType] of string =
    ('', 'jshort', 'jbyte', 'jint', 'jshort', 'jlong', 'jint', 'jlong', 'jfloat', 'jdouble', 'jstring',
     'jstring', 'jboolean', 'jchar', 'jchar', 'jint', 'jstring');
  JNITypeSig: array[TBasicType] of string =
    ('V', 'S', 'B', 'I', 'S', 'J', 'I', 'J', 'F', 'D', 'Ljava/lang/String;', 'Ljava/lang/String;',
     'Z', 'C', 'C', 'I', 'Ljava/lang/String;');
  JavaType: array[TBasicType] of string =
    ('void', 'short', 'byte', 'int', 'short', 'long', 'int', 'long', 'float', 'double', 'String',
     'String', 'boolean', 'char', 'char', 'int', 'String');

  TextIndent = 2;

  ExcludeStd: array[1..45] of string = (
    'classes.TStream.ReadComponent', 'classes.TStream.ReadComponentRes', 'classes.TStream.WriteComponent', 'classes.TStream.WriteComponentRes',
    'classes.TStream.WriteDescendent', 'classes.TStream.WriteDescendentRes', 'classes.TStream.WriteResourceHeader', 'classes.TStream.FixupResourceHeader',
    'classes.TStream.ReadResHeader', 'classes.TComponent.WriteState', 'classes.TComponent.ExecuteAction', 'classes.TComponent.UpdateAction',
    'classes.TComponent.GetEnumerator', 'classes.TComponent.VCLComObject', 'classes.TComponent.DesignInfo', 'classes.TComponent.Destroying',
    'classes.TComponent.FreeNotification', 'classes.TComponent.RemoveFreeNotification', 'classes.TComponent.FreeOnRelease', 'classes.TComponent.SetSubComponent',
    'system.TObject.newinstance', 'system.TObject.FreeInstance', 'system.TObject.SafeCallException', 'system.TObject.InitInstance',
    'system.TObject.CleanupInstance', 'system.TObject.ClassInfo', 'system.TObject.AfterConstruction', 'system.TObject.BeforeDestruction',
    'system.TObject.GetInterfaceEntry', 'system.TObject.GetInterfaceTable', 'system.TObject.MethodAddress', 'system.TObject.MethodName',
    'system.TObject.FieldAddress', 'classes.TComponent.ComponentState', 'classes.TComponent.ComponentStyle', 'classes.TList.GetEnumerator',
    'classes.TList.List', 'classes.TList.FPOAttachObserver', 'classes.TList.FPODetachObserver', 'classes.TList.FPONotifyObservers',
    'classes.TPersistent.FPOAttachObserver', 'classes.TPersistent.FPODetachObserver', 'classes.TPersistent.FPONotifyObservers',
    'system.fma', 'system.TExtended80Rec'
  );

  ExcludeDelphi7: array[1..57] of string = (
    'system.TObject.StringMessageTable', 'system.TObject.GetInterfaceEntryByStr', 'system.TObject.UnitName', 'system.TObject.Equals',
    'system.TObject.GetHashCode', 'system.TObject.ToString','system.TObject.QualifiedClassName',
    'sysutils.TEncoding',
    'classes.TStream.ReadByte', 'classes.TStream.ReadWord',
    'classes.TStream.ReadDWord', 'classes.TStream.ReadQWord', 'classes.TStream.ReadAnsiString', 'classes.TStream.WriteByte',
    'classes.TStream.WriteWord', 'classes.TStream.WriteDWord', 'classes.TStream.WriteQWord', 'classes.TStream.WriteAnsiString',
    'classes.TStream.ReadData', 'classes.TStream.ReadBufferData', 'classes.TStream.WriteData', 'classes.TStream.WriteBufferData',
    'classes.TCollection.Exchange', 'classes.TStrings.Equals', 'classes.TStrings.GetNameValue', 'classes.TStrings.ExtractName',
    'classes.TStrings.TextLineBreakStyle', 'classes.TStrings.StrictDelimiter', 'classes.TStrings.GetEnumerator', 'classes.TStringList.OwnsObjects',
    'classes.TStrings.Filter', 'classes.TStrings.ForEach', 'classes.TStrings.Reduce', 'classes.TStrings.Map', 'classes.TStrings.AddPair',
    'classes.TStrings.AddText', 'classes.TStrings.Fill', 'classes.TStrings.LastIndexOf', 'classes.TStrings.Pop', 'classes.TStrings.Reverse',
    'classes.TStrings.Shift', 'classes.TStrings.Slice', 'classes.TStrings.AlwaysQuote', 'classes.TStrings.LineBreak',
    'classes.TStrings.MissingNameValueSeparatorAction', 'classes.TStrings.SkipLastLineBreak', 'classes.TStrings.TrailingLineBreak', 'classes.TStrings.WriteBOM',
    'classes.TStrings.AddStrings#ClearFirst', 'classes.TStrings.IndexOf#aStart', 'classes.TStrings.LoadFromFile#IgnoreEncoding',
    'classes.TStrings.LoadFromStream#IgnoreEncoding',
    'classes.TStringList.SortStyle',
    'classes.TList.AddList', 'classes.TCustomMemoryStream.SizeBoundsSeek', 'classes.TBytesStream',
    'sortbase'
  );

  SUnsupportedType = '<unsupported type>';

function JniCaliing: string;
begin
  Result:='{$ifdef mswindows} stdcall {$else} cdecl {$endif};';
end;

{ TClassList }

function TClassList.IndexOf(const AName: string; Def: TDef): integer;
begin
  Result:=inherited IndexOf(GetFullName(AName, Def));
end;

function TClassList.GetClassName(Index: integer): string;
var
  i: integer;
begin
  Result:=Strings[Index];
  i:=Pos('.', Result);
  if i > 0 then
    System.Delete(Result, 1, i);
end;

function TClassList.GetClassInfo(Index: integer): TClassInfo;
begin
  Result:=TClassInfo(Objects[Index]);
end;

function TClassList.GetFullName(const AName: string; Def: TDef): string;
begin
  if (Def = nil) or (Def.DefType = dtUnit) then
    Result:=AName
  else begin
    while (Def.Parent <> nil) and (Def.DefType <> dtUnit) do
      Def:=Def.Parent;
    Result:=Def.Name + '.' + AName;
  end;
end;

constructor TClassList.Create;
begin
  inherited Create;
  Sorted:=True;
end;

function TClassList.Add(const AName: string; Def: TDef; Info: TClassInfo): integer;
begin
  Result:=AddObject(GetFullName(AName, Def), Info);
end;

{ TTextOutStream }

procedure TTextOutStream.SetIndednt(const AValue: integer);
begin
  if FIndent = AValue then exit;
  FIndent:=AValue;
  SetLength(FIndStr, FIndent*TextIndent);
  if FIndent > 0 then
    FillChar(FIndStr[1], FIndent*TextIndent, ' ');
end;

procedure TTextOutStream.Write(const s: ansistring);
begin
  WriteBuffer(PChar(s)^, Length(s));
end;

procedure TTextOutStream.WriteLn(const s: ansistring; ExtraIndent: integer);
begin
  if s = '' then
    Write(LineEnding)
  else begin
    Indent:=Indent + ExtraIndent;
    try
      Write(FIndStr + s + LineEnding);
    finally
      Indent:=Indent - ExtraIndent;
    end;
  end;
end;

procedure TTextOutStream.IncI;
begin
  Indent:=Indent + 1;
end;

procedure TTextOutStream.DecI;
begin
  if Indent > 0  then
    Indent:=Indent - 1;
end;

{ TClassInfo }

constructor TClassInfo.Create;
begin
  Funcs:=TObjectList.Create(True);
end;

destructor TClassInfo.Destroy;
begin
  Funcs.Free;
  inherited Destroy;
end;

{ TWriter }

function TWriter.DefToJniType(d: TDef; var err: boolean): string;
begin
  if d = nil then begin
    Result:=SUnsupportedType;
    err:=True;
  end
  else begin
    if not d.IsUsed then begin
      Result:='<excluded type> ' + d.Name;
      err:=True;
    end
    else
      case d.DefType of
        dtType:
          Result:=JNIType[TTypeDef(d).BasicType];
        dtClass, dtEnum, dtClassRef:
          Result:='jobject';
        dtProcType:
          if poMethodPtr in TProcDef(d).ProcOpt then
            Result:='jobject'
          else begin
            Result:=SUnsupportedType + ' ' + d.Name;
            err:=True;
          end;
        dtSet:
          if TSetDef(d).Size <= 4 then
            Result:='jobject'
          else begin
            Result:=SUnsupportedType + ' ' + d.Name;
            err:=True;
          end;
        dtPointer:
          if TPointerDef(d).IsObjPtr then
            Result:='jobject'
          else
            Result:='jlong';
        dtJniObject:
          Result:='jobject';
        else begin
          Result:=SUnsupportedType + ' ' + d.Name;
          err:=True;
          d.SetNotUsed;
        end;
    end;
  end;
end;

function TWriter.DoCheckItem(const ItemName: string): TCheckItemResult;
begin
  if FindInStringList(ExcludeList, ItemName) >= 0 then
    Result:=crExclude
  else
    if FindInStringList(IncludeList, ItemName) >= 0 then
      Result:=crInclude
    else
      Result:=crDefault;
end;

procedure TWriter.WriteFileComment(st: TTextOutStream);
begin
  st.WriteLn('// This file was automatically generated by the pas2jni utility.');
  st.WriteLn('// Do not edit this file.');
end;

function TWriter.FindInStringList(list: TStringList; const s: string): integer;
var
  mt: TMatchType;
begin
  Result:=FindInStringListEx(list, s, False, mt);
end;

function TWriter.FindInStringListEx(list: TStringList; const s: string; AllMatch: boolean; out MatchType: TMatchType): integer;
var
  len, cnt: integer;
  ss: string;
begin
  MatchType:=mtNone;
  if list.Find(s, Result) then begin
    MatchType:=mtExact;
    exit;
  end;
  if Result < 0 then
    exit;
  if Result < list.Count then begin
    cnt:=3;
    if Result > 0 then
      Dec(Result)
    else
      Dec(cnt);
    if Result + cnt > list.Count then
      Dec(cnt);
    while cnt > 0 do begin
      ss:=list[Result];
      len:=Length(ss);
      if len > 1 then begin
        if ss[len] = '*' then begin
          Dec(len);
          MatchType:=mtWildcard;
        end
        else
          if AllMatch then begin
            len:=Pos('#', ss) - 1;
            MatchType:=mtParams;
          end
          else
            len:=0;

        if (len > 0) and (AnsiCompareText(Copy(s, 1, len), Copy(ss, 1, len)) = 0) then
          exit;
      end;
      Inc(Result);
      Dec(cnt);
    end;
  end;
  MatchType:=mtNone;
  Result:=-1;
end;

procedure TWriter.ProcessRules(d: TDef; const Prefix: string);
var
  i: integer;
  s, c: string;
  b: boolean;
  mt: TMatchType;
begin
  if d.DefType = dtClass then
    with TClassDef(d) do
      if (AncestorClass = nil) and (CType in [ctClass, ctInterface]) and (CompareText(Parent.Name, 'system') <> 0) then begin
        SetNotUsed;
        exit;
      end;
  s:=Prefix + d.AliasName;
  if FindInStringListEx(ExcludeList, s, (d.DefType = dtProc), mt) >= 0 then begin
    if mt <> mtParams then begin
      if d.DefType = dtParam then
        d.Parent.SetNotUsed
      else
        d.SetNotUsed;
    end;
  end
  else
    if FindInStringList(IncludeList, s) >= 0 then
      d.IsUsed:=True;
  b:=not (d.DefType in [dtUnit, dtClass]);
  // Check exclusion rules for parameters of overloaded procs
  if (d.DefType = dtProc) and (mt = mtParams) then begin
    b:=False;
    c:='#';
  end
  else
    c:='.';

  if b then
    exit;

  s:=s + c;
  for i:=0 to d.Count - 1 do
    ProcessRules(d[i], s);
end;

function TWriter.GetUniqueNum: integer;
begin
  Inc(FUniqueCnt);
  Result:=FUniqueCnt;
end;

function TWriter.DefToJniSig(d: TDef): string;
begin
  if d = nil then
    Result:=SUnsupportedType
  else
    case d.DefType of
      dtType:
        Result:=JNITypeSig[TTypeDef(d).BasicType];
      dtClass, dtProcType, dtSet, dtEnum, dtClassRef:
        Result:='L' + GetJavaClassPath(d) + ';';
      dtPointer:
        if TPointerDef(d).IsObjPtr then
          Result:='L' + GetJavaClassPath(d) + ';'
        else
          Result:='J';
      dtJniObject:
        Result:='Ljava/lang/Object;';
      else
        Result:=SUnsupportedType;
    end;
end;

function TWriter.DefToJavaType(d: TDef): string;
begin
  if d = nil then
    Result:=SUnsupportedType
  else
    if not d.IsUsed and (d.DefType <> dtType) then
      Result:='<excluded type> ' + d.Name
    else
      case d.DefType of
        dtType:
          Result:=JavaType[TTypeDef(d).BasicType];
        dtClass, dtProcType, dtSet, dtEnum, dtClassRef:
          Result:=d.Name;
        dtPointer:
          if TPointerDef(d).IsObjPtr then
            Result:=d.Name
          else
            Result:='long';
        dtJniObject:
          Result:='Object';
        else
          Result:=SUnsupportedType;
      end;
end;

function TWriter.GetJavaClassPath(d: TDef; const AClassName: string): string;
var
  n: string;
begin
  if AClassName = '' then
    n:=d.AliasName
  else
    n:=AClassName;
  Result:=StringReplace(JavaPackage, '.', '/', [rfReplaceAll]);
  if Result <> '' then
    Result:=Result + '/';
  if d.DefType = dtUnit then
    Result:=Result + n
  else
    Result:=Result + d.Parent.AliasName + '$' + n;
end;

procedure TWriter.WriteClass(d: TClassDef; PreInfo: boolean);
var
  WrittenItems: TList;

  procedure _WriteConstructors(c: TClassDef; Written: TStringList);
  var
    i, j: integer;
    p: TProcDef;
    OldRet: TDef;
    s: string;
  begin
    if c = nil then
      exit;
    for i:=0 to c.Count - 1 do
      with c[i] do begin
        if (DefType = dtProc) and not c.IsPrivate and (TProcDef(c[i]).ProcType = ptConstructor) then begin
          p:=TProcDef(c[i]);
          j:=Written.IndexOf(p.Name);
          if (j < 0) or (Written.Objects[j] = c) then begin
            s:=p.Name + ':';
            for j:=0 to p.Count - 1 do
              if p[j].DefType = dtParam then
                s:=s + DefToJniSig(TVarDef(p[j]).VarType);
            if Written.IndexOf(s) < 0 then begin
              OldRet:=p.ReturnType;
              p.ReturnType:=d;
              p.Parent:=d;
              try
                WriteProc(p);
              finally
                p.ReturnType:=OldRet;
                p.Parent:=c;
              end;
              Written.Add(s);
              if not (poOverload in p.ProcOpt) then
                Written.AddObject(p.Name, c);
            end;
          end;
        end;
      end;

    _WriteConstructors(c.AncestorClass, Written);
  end;

  procedure WriteConstructors;
  var
    cc: TStringList;
    i: integer;
  begin
    if not TClassDef(d).HasAbstractMethods then begin
      // Writing all constructors including parent's
      cc:=TStringList.Create;
      try
        cc.Sorted:=True;
        _WriteConstructors(TClassDef(d), cc);
      finally
        cc.Free;
      end;
    end;
    if d.CType = ctClass then begin
      i:=FRealClasses.Add(d);
      Fjs.WriteLn(Format('public static %s Class() { return new %0:s(system.GetClassRef(%d)); }', [d.AliasName, i]));
      Fjs.WriteLn(Format('public static system.TClass TClass() { return system.GetTClass(%d); }', [i]));
    end;
  end;

  procedure _WriteReplacedItems(c: TClassDef);
  var
    i: integer;
    p: TReplDef;
  begin
    c:=c.AncestorClass;
    if c = nil then
      exit;
    if c.HasReplacedItems then begin
      for i:=0 to c.Count - 1 do
        with c[i] do begin
          p:=TReplDef(c[i]);
          if (DefType in ReplDefs) and ((p.IsReplaced) or p.IsReplImpl) then begin
            if p.ReplacedItem <> nil then
              WrittenItems.Add(p.ReplacedItem);
            if WrittenItems.IndexOf(p) >= 0 then
              continue;
            case p.DefType of
              dtProc:
                WriteProc(TProcDef(p), nil, d);
              dtProp, dtField:
                WriteVar(TVarDef(p), d);
              else
                ; // no action
            end;
          end;
        end;
    end;
    _WriteReplacedItems(c);
  end;

  procedure WriteReplacedItems;
  begin
    _WriteReplacedItems(TClassDef(d));
  end;

  procedure WriteItems(Regular, Replaced, ReplImpl: boolean);
  var
    i: integer;
    it: TReplDef;
  begin
    for i:=0 to d.Count - 1 do begin
      it:=TReplDef(d[i]);
      if not (it.DefType in ReplDefs) then
        continue;
      if not (it.IsReplImpl or it.IsReplaced) then begin
        if not Regular then
          continue;
      end
      else
        if (not Replaced and it.IsReplaced) or (not ReplImpl and it.IsReplImpl) then
          continue;
      if it.ReplacedItem <> nil then
        WrittenItems.Add(it.ReplacedItem);
      case it.DefType of
        dtProc:
          if TProcDef(it).ProcType <> ptConstructor  then
            WriteProc(TProcDef(it));
        dtProp, dtField:
          WriteVar(TVarDef(it));
        else
          ; // no action
      end;
    end;
  end;

  procedure WriteTypeCast(const AName: string; SecondPass: boolean);
  var
    s, ss: string;
  begin
    with TClassDef(d) do begin
      if HasReplacedItems and not SecondPass then
        s:='protected'
      else
        s:='public';
      if CType = ctInterface then begin
        Fjs.WriteLn('private native long __AsIntf(long objptr);');
        ss:=IID;
        if ss = '' then
          ss:='null'
        else
          ss:='"' + ss + '"';
        Fjs.WriteLn(Format('%s %s(PascalObject obj) { super(0, true); __TypeCast(obj, %s); }', [s, AName, ss]));
        Fjs.WriteLn(Format('%s %s(long objptr) { super(objptr, true); }', [s, AName]));
      end
      else begin
        Fjs.WriteLn(Format('%s %s(PascalObject obj) { super(obj); }', [s, AName]));
        Fjs.WriteLn(Format('%s %s(long objptr) { super(objptr); }', [s, AName]));
      end;
    end;
  end;

var
  s, ss, n: string;
  RegularClass: boolean;
begin
  if PreInfo then begin
    WriteClassInfoVar(d);

    if d.CType in [ctObject, ctRecord] then begin
      s:=d.Parent.Name + '.' + d.Name;
      Fps.WriteLn;
      Fps.WriteLn(Format('function _%s_CreateObj(env: PJNIEnv; const r: %s): jobject;', [GetClassPrefix(d), s]));
      Fps.WriteLn(Format('var pr: ^%s;', [s]));
      Fps.WriteLn('begin');
      Fps.IncI;
      Fps.WriteLn(Format('pr:=AllocMem(SizeOf(%s)); pr^:=r;', [s]));
      Fps.WriteLn(Format('Result:=_CreateJavaObj(env, pr, %s);', [GetTypeInfoVar(d)]));
      Fps.DecI;
      Fps.WriteLn('end;');

      Fps.WriteLn;
      ss:=Format('_%s_Free', [GetClassPrefix(d)]);
      Fps.WriteLn(Format('procedure %s(env: PJNIEnv; _self: JObject; r: jlong);', [ss]) + JniCaliing);
      Fps.WriteLn(Format('var pr: ^%s;', [s]));
      Fps.WriteLn('begin');
      Fps.WriteLn('pr:=pointer(ptruint(r));', 1);
      Fps.WriteLn('system.Dispose(pr);', 1);
      Fps.WriteLn('end;');

      AddNativeMethod(d, ss, '__Destroy', '(J)V');
    end;
    exit;
  end;

  // Java
  case d.CType of
    ctInterface:
      s:='interface';
    ctObject:
      s:='interface';
    ctRecord:
      s:='record';
    else
      s:='class';
  end;
  WriteComment(d, s);
  n:=GetJavaClassName(d, nil);
  s:='public static class ' + n + ' extends ';
  with d do begin
    if AncestorClass <> nil then begin
      ss:=AncestorClass.Name;
      if ImplementsReplacedItems then
        ss:='__' + ss;
      s:=s + ss;
    end
    else
      if d.CType in [ctObject, ctRecord] then
        s:=s + Format('%s.system.Record', [JavaPackage])
      else
        if d.CType = ctInterface then
          s:=s + 'PascalInterface'
        else
          s:=s + 'PascalObject';
  end;
  Fjs.WriteLn(s + ' {');
  Fjs.IncI;
  case d.CType of
    ctObject, ctRecord:
      begin
        Fjs.WriteLn('private native void __Destroy(long pasobj);');
        if d.AncestorClass = nil then
          s:='__Init'
        else
          s:='super';
        Fjs.WriteLn(Format('protected %s(long objptr, boolean cleanup) { %s(objptr, cleanup); }', [d.Name, s]));
        Fjs.WriteLn(Format('public %s() { %s(0, true); }', [d.Name, s]));
        Fjs.WriteLn(Format('@Override public void __Release() { __Destroy(_pasobj); _pasobj=0; }', [d.Name]));
        Fjs.WriteLn(Format('@Override public int __Size() { return __Size(%d); }', [FRecords.Add(d)]));
      end;
    ctInterface:
      begin
        if d.AncestorClass = nil then begin
          Fjs.WriteLn('@Override public void __Release() { if (_pasobj != 0) _Release(); _pasobj = 0; }');
          Fjs.WriteLn('@Override protected void __Init() { _cleanup=true; if (_pasobj != 0) _AddRef(); }');
        end;
        Fjs.WriteLn(Format('protected %s(long objptr, boolean cleanup) { super(objptr, cleanup); }', [d.Name]));
      end;
    else
      ; // no action
  end;

  WriteTypeCast(n, False);

  WrittenItems:=TList.Create;
  try
    RegularClass:=(d.DefType = dtClass) and not TClassDef(d).HasReplacedItems;
    if RegularClass then
      WriteConstructors;
    // Write regular items
    WriteItems(True, False, RegularClass);
    if RegularClass and TClassDef(d).ImplementsReplacedItems then
      // Write implementation wrappers for replaced mehods
      WriteReplacedItems;

    Fjs.DecI;
    Fjs.WriteLn('}');
    Fjs.WriteLn;

    if (d.DefType = dtClass) and (TClassDef(d).HasReplacedItems) then begin
      // Write replaced items
      Fjs.WriteLn(Format('public static class %s extends __%0:s {', [d.AliasName]));
      Fjs.IncI;

      WriteTypeCast(d.AliasName, True);
      WriteConstructors;
      WriteItems(False, True, True);

      if TClassDef(d).ImplementsReplacedItems then
        // Write implementation wrappers for replaced mehods
        WriteReplacedItems;

      Fjs.DecI;
      Fjs.WriteLn('}');
      Fjs.WriteLn;
    end;
  finally
    WrittenItems.Free;
  end;
end;

procedure TWriter.WriteProc(d: TProcDef; Variable: TVarDef; AParent: TDef);
var
  i, j, ClassIdx: integer;
  s, ss, ps, TempRes, VarFin: string;
  err, tf: boolean;
  pi: TProcInfo;
  ci: TClassInfo;
  IsTObject: boolean;
  tempvars: TStringList;
  vd: TVarDef;
  UseTempObjVar, IsObj, IsProcVar: boolean;
  ItemDef: TDef;
begin
  ASSERT(d.DefType = dtProc);
  if d.IsPrivate or not d.IsUsed then
    exit;
  IsTObject:=(d.Parent.DefType = dtClass) and (TClassDef(d.Parent).AncestorClass = nil);
  if (d.ProcType = ptDestructor) and not IsTObject then
    exit;
  if Variable <> nil then
    ItemDef:=Variable
  else
    ItemDef:=d;
  tempvars:=nil;
  pi:=TProcInfo.Create;
  with d do
  try
    IsObj:=(d.Parent.DefType = dtClass) and (TClassDef(d.Parent).CType = ctObject);
    if not IsObj and (poClassMethod in ProcOpt) and (Name = 'ClassType') then
      ProcOpt:=ProcOpt - [poClassMethod];
    pi.Name:=Name;
    s:=GetClassPrefix(d.Parent) + pi.Name;
    pi.JniName:=s;
    pi.JniSignature:=GetProcSignature(d);
    if AParent = nil then begin
      // Checking duplicate proc name and duplicate param types
      ClassIdx:=FClasses.IndexOf(GetJavaClassName(d.Parent, ItemDef), d.Parent);
      if ClassIdx >= 0 then begin
        ci:=FClasses.GetClassInfo(ClassIdx);
        j:=1;
        ss:=Copy(pi.JniSignature, 1, Pos(')', pi.JniSignature));
        repeat
          err:=False;
          for i:=0 to ci.Funcs.Count - 1 do
            with TProcInfo(ci.Funcs[i]) do
              if CompareText(JniName, pi.JniName) = 0 then begin
                Inc(j);
                pi.JniName:=Format('%s_%d', [s, j]);
                err:=True;
                break;
              end
              else
                if (CompareText(Name, pi.Name) = 0) and (ss = Copy(JniSignature, 1, Pos(')', JniSignature))) then
                  // Duplicate params
                  exit;
        until not err;
      end;

      err:=False;
      if ProcType in [ptFunction, ptConstructor] then
        s:='function'
      else
        s:='procedure';
      s:=s + ' ' + pi.JniName + '(_env: PJNIEnv; _jobj: jobject';

      if IsObj and (ProcType in [ptConstructor, ptDestructor]) then
        TempRes:='__tempres';

      IsProcVar:=(Variable <> nil) and (Variable.VarType <> nil) and (Variable.VarType.DefType = dtProcType);
      UseTempObjVar:=IsProcVar and (ProcType = ptProcedure) and (Variable.Parent.DefType <> dtUnit);

      for j:=0 to Count - 1 do begin
        vd:=TVarDef(Items[j]);
        if vd.DefType <> dtParam then
          continue;
        with vd do begin
          if (VarType <> nil) and (VarType.DefType = dtJniEnv) then
            continue;
          s:=s + '; ' + Name + ': ';
          ss:=DefToJniType(VarType, err);
          if not IsJavaVarParam(vd) then
            s:=s + ss
          else begin
            if not err then
              ss:='jarray';
            s:=s + ss;
            if tempvars = nil then
              tempvars:=TStringList.Create;
            if VarType = nil then
              err:=True
            else
              Tag:=tempvars.AddObject('__tmp_' + Name, d.Items[j]) + 1;
          end;
        end;
      end;
      s:=s + ')';

      if ProcType in [ptFunction, ptConstructor] then
        s:=s + ': ' + DefToJniType(ReturnType, err);
      s:=s + '; ' + JniCaliing;
      if err then begin
        s:='// ' + s;
        Fjs.WriteLn('// NOT PROCESSED: ' + GetJavaProcDeclaration(d));
        d.SetNotUsed;
      end;
      Fps.WriteLn;
      Fps.WriteLn(s);
      if err then
        exit;

      if (poClassMethod in ProcOpt) and not IsObj then begin
        Fps.WriteLn(Format('type _classt = %s;', [Parent.Parent.Name + '.' + Parent.Name]));
        Fps.WriteLn('type _class = class of _classt;');
      end;

      if (tempvars <> nil) or UseTempObjVar or (TempRes <> '') then begin
        s:='';
        Fps.WriteLn('var');
        Fps.IncI;
        if tempvars <> nil then begin
          for i:=0 to tempvars.Count - 1 do begin
            vd:=TVarDef(tempvars.Objects[i]);
            Fps.WriteLn(Format('%s: %s;', [tempvars[i], GetPasType(vd.VarType, True)]));
            if IsJavaSimpleType(vd.VarType) then begin
              Fps.WriteLn(Format('%s_arr: P%s;', [tempvars[i], DefToJniType(vd.VarType, err)]));
              if s = '' then
                s:='__iscopy: JBoolean;';
            end;
          end;
          if s <> '' then
            Fps.WriteLn(s);
        end;
        if UseTempObjVar then
          Fps.WriteLn('__objvar: ' + d.Parent.Name + ';');
        if TempRes <> '' then begin
          s:=TempRes + ': ';
          if IsObj and (ProcType in [ptConstructor, ptDestructor]) then
            s:=s + '^' + GetPasType(d.Parent, True)
          else
            s:=s + GetPasType(d.ReturnType, True);
          Fps.WriteLn(s + ';');
        end;
        Fps.DecI;
      end;

      if IsProcVar and (ProcType = ptProcedure) then
        Fps.WriteLn('var __mvar: TMethod;');

      Fps.WriteLn('begin');
      Fps.IncI;
      EHandlerStart;

      tf:=False;
      // Assign var parameter values to local vars
      if tempvars <> nil then begin
        for i:=0 to tempvars.Count - 1 do begin
          vd:=TVarDef(tempvars.Objects[i]);
          Fps.WriteLn(Format('if _env^^.GetArrayLength(_env, %s) <> 1 then _RaiseVarParamException(''%s'');', [vd.Name, vd.Name]));
          if IsJavaSimpleType(vd.VarType) then begin
            Fps.WriteLn(Format('%s_arr:=_env^^.Get%sArrayElements(_env, %s, __iscopy);', [tempvars[i], GetJniFuncType(vd.VarType), vd.Name]));
            Fps.WriteLn(Format('if %s_arr = nil then _RaiseVarParamException(''%s'');', [tempvars[i], vd.Name]));
            s:=tempvars[i] + '_arr^';
            tf:=True;
          end
          else
            s:=Format('_env^^.GetObjectArrayElement(_env, %s, 0)', [vd.Name]);
          if voVar in vd.VarOpt then
            Fps.WriteLn(tempvars[i] + ':=' + JniToPasType(vd.VarType, s, False) + ';');
        end;
      end;

      if tf then begin
        Fps.WriteLn('try');
        Fps.IncI;
      end;

      s:='';
      if not (IsObj and (ProcType in [ptConstructor, ptDestructor])) then
        if Parent.DefType = dtUnit then
          s:=Parent.Name + '.'
        else
          if ProcType = ptConstructor then
            s:=Parent.Parent.Name + '.' + Parent.Name + '.'
          else
            if (poClassMethod in ProcOpt) and not IsObj then
              s:='_class(_GetClass(_env, _jobj, ' + GetTypeInfoVar(d.Parent) + '))' + '.'
            else
              s:=JniToPasType(d.Parent, '_jobj', True) + '.';

      if Variable = nil then begin
        // Regular proc
        s:=s + pi.Name;
        if Count > 0 then begin
          s:=s + '(';
          ps:='';
          for j:=0 to Count - 1 do begin
            vd:=TVarDef(Items[j]);
            if vd.DefType <> dtParam then
              continue;
            if vd.VarType.DefType = dtJniEnv then
              ss:='_env'
            else
              if vd.Tag <> 0 then
                ss:=tempvars[vd.Tag - 1]
              else begin
                ss:=Items[j].Name;
                ss:=JniToPasType(vd.VarType, ss, False);
              end;
            if ps <> '' then
              ps:=ps + ', ';
            ps:=ps + ss;
          end;
          s:=s + ps + ')';
        end;
      end
      else begin
        // Var access
        if UseTempObjVar then begin
          System.Delete(s, Length(s), 1);
          Fps.WriteLn('__objvar:=' + s + ';');
          s:='__objvar.';
        end;
        s:=s + Variable.Name;
        j:=Count;
        if ProcType = ptProcedure then
          Dec(j);
        if j > 0 then begin
          i:=j;
          ss:='';
          for j:=0 to j - 1 do begin
            if ss <> '' then
              ss:=ss + ', ';
            ss:=ss + JniToPasType(TVarDef(Items[j]).VarType, Items[j].Name, False);
          end;
          s:=Format('%s[%s]', [s, ss]);
        end
        else
          i:=0;
        if ProcType = ptProcedure then begin
          ASSERT(Count = i + 1);
          if Variable.VarType.DefType = dtProcType then begin
            Fps.WriteLn(Format('__mvar:=TMethod(%s);', [s]));
            VarFin:=Format('_RefMethodPtr(_env, TMethod(%s), True); _RefMethodPtr(_env, __mvar, False);', [s]);
          end;
          s:=s + ':=' + JniToPasType(TVarDef(Items[i]).VarType, Items[i].Name, False);
        end;
      end;

      if IsObj and (ProcType = ptConstructor) then begin
        s:=Format('system.New(%s, %s);', [TempRes, s]);
        Fps.WriteLn(s);
        s:=Format('Result:=_CreateJavaObj(_env, %s, %s, False);', [TempRes, GetTypeInfoVar(ReturnType)]);
        Fps.WriteLn(s);
      end
      else
      if IsObj and (ProcType = ptDestructor) then begin
        Fps.WriteLn(TempRes + ':=@' + JniToPasType(d.Parent, '_jobj', True) + ';');
        Fps.WriteLn(Format('%s^.%s;', [TempRes, s]));
        Fps.WriteLn(Format('_env^^.SetLongField(_env, _jobj, %s.ObjFieldId, -jlong(ptruint(%s)));', [GetTypeInfoVar(d.Parent), TempRes]));
      end
      else begin
        if ProcType in [ptFunction, ptConstructor] then
          s:='Result:=' + PasToJniType(ReturnType, s);
        s:=s + ';';
        Fps.WriteLn(s);
      end;

      if VarFin <> '' then
        Fps.WriteLn(VarFin);

      // Return var/out parameters
      if tempvars <> nil then begin
        for i:=0 to tempvars.Count - 1 do begin
          vd:=TVarDef(tempvars.Objects[i]);
          if IsJavaSimpleType(vd.VarType) then
            Fps.WriteLn(Format('%s_arr^:=%s;', [tempvars[i], PasToJniType(vd.VarType, tempvars[i])]))
          else
            Fps.WriteLn(Format('_env^^.SetObjectArrayElement(_env, %s, 0, %s);', [vd.Name, PasToJniType(vd.VarType, tempvars[i])]));
        end;
      end;

      if not IsObj then
        if IsTObject and ( (ProcType = ptDestructor) or (CompareText(Name, 'Free') = 0) ) then
          Fps.WriteLn(Format('_env^^.SetLongField(_env, _jobj, %s.ObjFieldId, 0);', [GetTypeInfoVar(d.Parent)]));

      if tf then begin
        Fps.WriteLn('finally', -1);

        if tempvars <> nil then begin
          for i:=0 to tempvars.Count - 1 do begin
            vd:=TVarDef(tempvars.Objects[i]);
            if IsJavaSimpleType(vd.VarType) then
              Fps.WriteLn(Format('_env^^.Release%sArrayElements(_env, %s, %s_arr, 0);', [GetJniFuncType(vd.VarType), vd.Name, tempvars[i]]));
          end;
        end;

        Fps.DecI;
        Fps.WriteLn('end;');
      end;

      s:='';
      if ProcType in [ptFunction, ptConstructor] then begin
        s:='0';
        if (ReturnType.DefType = dtType) and (TTypeDef(ReturnType).BasicType <= btDouble) then
          s:='0'
        else
          s:=Format('%s(0)', [DefToJniType(ReturnType, err)]);
        s:='Result:=' + s + ';';
      end;
      EHandlerEnd('_env', s);

      Fps.DecI;
      Fps.WriteLn('end;');
      AParent:=d.Parent;
    end
    else
      ClassIdx:=FClasses.IndexOf(GetJavaClassName(AParent, ItemDef), AParent);

    if ClassIdx < 0 then begin
      ci:=TClassInfo.Create;
      ci.Def:=AParent;
      s:=GetJavaClassName(AParent, ItemDef);
      ci.IsCommonClass:=s <> AParent.Name;
      ClassIdx:=FClasses.Add(s, AParent, ci);
    end;
    FClasses.GetClassInfo(ClassIdx).Funcs.Add(pi);
    pi:=nil;

    // Java part
    s:=GetJavaProcDeclaration(d) + ';';
    if (Parent.DefType = dtUnit) or (ProcType = ptConstructor) then
      s:='static ' + s;

    if Variable = nil then
      Fjs.WriteLn('// ' + GetProcDeclaration(d));
    if poPrivate in ProcOpt then
      ss:='private'
    else
      if poProtected in ProcOpt then
        ss:='protected'
      else
        ss:='public';
    Fjs.WriteLn(ss + ' native ' + s);
  finally
    pi.Free;
    tempvars.Free;
  end;
end;

procedure TWriter.WriteVar(d: TVarDef; AParent: TDef);

  function _WriteArrayIndex(pd: TProcDef): TDef;
  var
    ad: TArrayDef;
    i: integer;
  begin
    ad:=TArrayDef(d.VarType);
    i:=1;
    repeat
      with TVarDef.Create(pd, dtParam) do begin
        Name:='Index';
        if i > 1 then
          Name:=Name + IntToStr(i);
        VarType:=ad.RangeType;
        if (VarType.DefType = dtType) and (TTypeDef(VarType).BasicType in [btByte, btShortInt, btSmallInt]) then
          VarType:=FIntegerType;
        VarOpt:=[voRead];
        IsUsed:=True;
      end;
      Result:=ad.ElType;
      ad:=TArrayDef(Result);
      Inc(i);
    until Result.DefType <> dtArray;
  end;

var
  pd: TProcDef;
  vd: TVarDef;
  t: TTypeDef;
  vt: TDef;
  s, ss: string;
  i: integer;
  isarray, isdynarray: boolean;
begin
  if not d.IsUsed then
    exit;

  isarray:=(d.VarType <> nil) and (d.VarType.DefType = dtArray);
  isdynarray:=isarray and (TArrayDef(d.VarType).RangeHigh < TArrayDef(d.VarType).RangeLow);
  if isdynarray then
    if not (voRead in d.VarOpt) then
      exit
    else
      d.VarOpt:=d.VarOpt + [voWrite];

  if d.VarType <> nil then begin
    case d.DefType of
      dtVar:
        s:='var';
      dtProp:
        s:='property';
      else
        s:='';
    end;
    s:=Trim(s + ' ' + d.Name);
    if d.Count > 0 then
      s:=s + '[]';
    ss:=d.VarType.Name;
    if ss = '' then
      if d.VarType.DefType = dtArray then
        ss:='array';
    Fjs.WriteLn(Format('// %s: %s', [s, ss]));
  end;

  if voRead in d.VarOpt then begin
    pd:=TProcDef.Create(nil, dtProc);
    try
      pd.IsUsed:=True;
      pd.Parent:=d.Parent;
      pd.ProcType:=ptFunction;
      pd.Name:='get' + d.Name;
      if isarray then
        // Array var
        pd.ReturnType:=_WriteArrayIndex(pd)
      else begin
        pd.ReturnType:=d.VarType;
        if d.DefType = dtProp then begin
          for i:=0 to d.Count - 1 do begin
            vd:=TVarDef(d.Items[i]);
            with TVarDef.Create(pd, dtParam) do begin
              Name:=vd.Name;
              VarType:=vd.VarType;
              VarOpt:=[voRead];
            end;
          end;
        end;
      end;
      WriteProc(pd, d, AParent);
    finally
      pd.Free;
    end;
  end;

  if voWrite in d.VarOpt then begin
    pd:=TProcDef.Create(nil, dtProc);
    try
      pd.IsUsed:=True;
      pd.Parent:=d.Parent;
      pd.ProcType:=ptProcedure;
      pd.Name:='set' + d.Name;
      vt:=d.VarType;;
      if isarray then begin
        // Array var
        if (d.DefType = dtProp) and not isdynarray then
          exit;
        vt:=_WriteArrayIndex(pd);
      end
      else
        if d.DefType = dtProp then begin
          for i:=0 to d.Count - 1 do begin
            vd:=TVarDef(d.Items[i]);
            with TVarDef.Create(pd, dtParam) do begin
              Name:=vd.Name;
              VarType:=vd.VarType;
              VarOpt:=[voRead];
            end;
          end;
        end;

      s:='Value';
      // Check if the name of value parameter is unique
      i:=0;
      while i < d.Count do begin
        if AnsiCompareText(s, d.Items[i].AliasName) = 0 then begin
          i:=0;
          s:='_' + s;
          continue;
        end;
        Inc(i);
      end;

      with TVarDef.Create(pd, dtParam) do begin
        Name:=s;
        VarType:=vt;
        VarOpt:=[voRead];
      end;
      t:=TTypeDef.Create(nil, dtType);
      try
        t.BasicType:=btVoid;
        pd.ReturnType:=t;
        WriteProc(pd, d, AParent);
      finally
        t.Free;
      end;
    finally
      pd.Free;
    end;
  end;
end;

procedure TWriter.WriteConst(d: TConstDef);
var
  s, v: string;
begin
  if not d.IsUsed then
    exit;
  v:=d.Value;
  if d.VarType = nil then begin
    if Copy(d.Value, 1, 1) = '"' then
      s:='String'
    else
      s:='double';
  end
  else begin
    s:='';
    case d.VarType.DefType of
      dtType:
        case TTypeDef(d.VarType).BasicType of
          btLongWord, btInt64:
            v:=v + 'L';
          btBoolean:
            if v = '1' then
              v:='true'
            else
              v:='false';
          else
            ; // no action
        end;
      dtArray:
        with TArrayDef(d.VarType) do
          if (ElType.DefType = dtType) and (TTypeDef(ElType).BasicType in [btChar, btWideChar]) then
            s:='String';
      else
        ; // no action
    end;
    if s = '' then
      s:=DefToJavaType(d.VarType);
  end;
  v:=Format('public static final %s %s = %s;', [s, d.Name, v]);
  if s = SUnsupportedType then
    v:='// ' + v;
  Fjs.WriteLn(v);
end;

procedure TWriter.WriteEnum(d: TDef);
var
  i: integer;
  s: string;
begin
  if not d.IsUsed then
    exit;

  RegisterPseudoClass(d);

  WriteComment(d, 'enum');
  Fjs.WriteLn(Format('public static class %s extends %s.system.Enum {', [d.Name, JavaPackage]));
  Fjs.IncI;
  for i:=0 to d.Count - 1 do begin
    s:=Format('public final static int %s = %s;', [d[i].Name, TConstDef(d[i]).Value]);
    Fjs.WriteLn(s);
  end;
  Fjs.WriteLn;
  for i:=0 to d.Count - 1 do begin
    s:=Format('public static %s %s() { return new %0:s(%1:s); }', [d.Name, d[i].Name]);
    Fjs.WriteLn(s);
  end;
  Fjs.WriteLn;
  Fjs.WriteLn(Format('public %s(int v) { Value = v; }', [d.Name]));
  Fjs.WriteLn(Format('@Override public boolean equals(Object o) { return ((o instanceof %0:s) && Value == ((%0:s)o).Value) || super.equals(o); }', [d.Name]));
  Fjs.DecI;
  Fjs.WriteLn('}');
  Fjs.WriteLn;
end;

procedure TWriter.WriteProcType(d: TProcDef; PreInfo: boolean);

  procedure _AccessSimpleArray(vd: TVarDef; VarIndex: integer; DoSet: boolean);
  begin
    with vd do begin
      Fps.WriteLn(Format('_tmp_%s:=_env^^.Get%sArrayElements(_env, _args[%d].L, PJBoolean(nil)^);', [Name, GetJniFuncType(VarType), VarIndex]));
      Fps.WriteLn(Format('if _tmp_%s <> nil then', [Name]));
      if DoSet then
        Fps.WriteLn(Format('_tmp_%s^:=%s;', [Name, PasToJniType(VarType, Name)]), 1)
      else
        Fps.WriteLn(Format('%s:=%s;', [Name, JniToPasType(VarType, '_tmp_' + Name + '^', False)]), 1);
      Fps.WriteLn(Format('_env^^.Release%sArrayElements(_env, _args[%d].L, _tmp_%s, 0);', [GetJniFuncType(VarType), VarIndex, Name]));
    end;
  end;

var
  vd: TVarDef;
  i: integer;
  s, ss, hclass: string;
  err: boolean;
begin
  if not d.IsUsed or not (poMethodPtr in d.ProcOpt) then
    exit;

  if PreInfo then begin
    WriteClassInfoVar(d);

    // Handler proc
    hclass:=GetClassPrefix(d) + 'Class';
    Fps.WriteLn;
    Fps.WriteLn(Format('type %s = class', [hclass]));
    Fps.WriteLn(Format('private %s;', [ GetProcDeclaration(d, 'Handler', True, True)]), 1);
    Fps.WriteLn('end;');
    Fps.WriteLn;
    Fps.WriteLn(GetProcDeclaration(d, Format('%s.Handler', [hclass]), True, True) + ';');

    Fps.WriteLn('var');
    Fps.IncI;
    Fps.WriteLn('_env: PJNIEnv;');
    Fps.WriteLn('_new_env: boolean;');
    Fps.WriteLn('_mpi: _TMethodPtrInfo;');
    if d.Count > 0 then begin
      Fps.WriteLn(Format('_args: array[0..%d] of jvalue;', [d.Count - 1]));
      for i:=0 to d.Count - 1 do begin
        vd:=TVarDef(d[i]);
        if vd.DefType <> dtParam then
          continue;
        with vd do
          if IsJavaVarParam(vd) and IsJavaSimpleType(VarType) then
            Fps.WriteLn(Format('_tmp_%s: P%s;', [Name, DefToJniType(VarType, err)]));
      end;
    end;
    Fps.DecI;
    Fps.WriteLn('begin');
    Fps.IncI;
    Fps.WriteLn('CurJavaVM^^.GetEnv(CurJavaVM, @_env, JNI_VERSION_1_6);');
    Fps.WriteLn('_new_env:=_env = nil;');
    Fps.WriteLn('if _new_env then CurJavaVM^^.AttachCurrentThread(CurJavaVM, @_env, nil);');
    Fps.WriteLn('_env^^.PushLocalFrame(_env, 100);');
    Fps.WriteLn('try');
    Fps.IncI;
    Fps.WriteLn('_MethodPointersCS.Enter;');
    Fps.WriteLn('try');
    Fps.WriteLn('_mpi:=_TMethodPtrInfo(_MethodPointers[-integer(ptruint(Self)) - 1]);', 1);
    Fps.WriteLn('finally');
    Fps.WriteLn('_MethodPointersCS.Leave;', 1);
    Fps.WriteLn('end;');

    for i:=0 to d.Count - 1 do begin
      vd:=TVarDef(d[i]);
      if vd.DefType <> dtParam then
        continue;
      with vd do begin
        if not IsJavaVarParam(vd) then begin
          s:='L';
          if VarType.DefType = dtType then
            s:=Copy(JNITypeSig[TTypeDef(VarType).BasicType], 1, 1);
          ss:=PasToJniType(VarType, Name);
        end
        else begin
          s:='L';
          if IsJavaSimpleType(VarType) then
            ss:=Format('_env^^.New%sArray(_env, 1)', [GetJniFuncType(VarType)])
          else begin
            if voVar in VarOpt then
              ss:=PasToJniType(VarType, Name)
            else
              ss:='nil';
            ss:=Format('_env^^.NewObjectArray(_env, 1, %s.ClassRef, %s)', [GetTypeInfoVar(VarType), ss]);
          end;
        end;
        Fps.WriteLn(Format('_args[%d].%s:=%s;', [i, s, ss]));
        if IsJavaVarParam(vd) and (voVar in VarOpt) and IsJavaSimpleType(VarType) then
          _AccessSimpleArray(TVarDef(d[i]), i, True);
      end;
    end;

    if d.Count > 0 then
      s:='@_args'
    else
      s:='nil';
    // Calling Java handler
    s:=Format('_env^^.Call%sMethodA(_env, _mpi.Obj, _mpi.MethodId, %s)', [GetJniFuncType(d.ReturnType), s]);
    if d.ProcType = ptFunction then
      s:=Format('Result:=%s', [JniToPasType(d.ReturnType, s, False)]);
    Fps.WriteLn(s + ';');
    // Java exception check
    Fps.WriteLn('_HandleJavaException(_env);');
    // Processing var/out parameters
    for i:=0 to d.Count - 1 do begin
      vd:=TVarDef(d[i]);
      if vd.DefType <> dtParam then
        continue;
      with vd do
        if IsJavaVarParam(vd) then
          if IsJavaSimpleType(VarType) then
            _AccessSimpleArray(TVarDef(d[i]), i, False)
          else begin
            s:=Format('_env^^.GetObjectArrayElement(_env, _args[%d].L, 0)', [i]);
            Fps.WriteLn(Format('%s:=%s;', [Name, JniToPasType(VarType, s, False)]));
          end;
    end;

    Fps.DecI;
    Fps.WriteLn('finally');
    Fps.WriteLn('_env^^.PopLocalFrame(_env, nil);', 1);
    Fps.WriteLn('if _new_env then CurJavaVM^^.DetachCurrentThread(CurJavaVM);', 1);
    Fps.WriteLn('end;');
    Fps.DecI;
    Fps.WriteLn('end;');

    // Get handler proc
    Fps.WriteLn;
    Fps.WriteLn(Format('function %sGetHandler(env: PJNIEnv; jobj: jobject; const ci: _TJavaClassInfo): %s.%s;',
                       [GetClassPrefix(d), d.Parent.Name, d.Name]));
    Fps.WriteLn('begin');
    Fps.WriteLn(Format('TMethod(Result):=_GetMethodPtrHandler(env, jobj, @%s.Handler, %s);', [hclass, GetTypeInfoVar(d)]), 1);
    Fps.WriteLn('end;');

    exit;
  end;

  err:=False;
  WriteComment(d, 'procedural type');

  RegisterPseudoClass(d);

  Fjs.WriteLn(Format('/* Pascal prototype: %s */', [GetProcDeclaration(d, 'Execute')]));
  Fjs.WriteLn(Format('/* Java prototype: %s */', [GetJavaProcDeclaration(d, 'Execute')]));

  Fjs.WriteLn(Format('public static class %s extends %s.system.MethodPtr {', [d.Name, JavaPackage]));
  Fjs.IncI;
  Fjs.WriteLn(Format('{ mSignature = "%s"; }', [GetProcSignature(d)]));
  Fjs.WriteLn(Format('protected %s(long objptr, boolean cleanup) { _pasobj=objptr; }', [d.Name]));
  Fjs.WriteLn(Format('@Deprecated public %s(Object Obj, String MethodName) { mObject=Obj; mName=MethodName; }', [d.Name]));
  Fjs.WriteLn(Format('public %s() { mObject=this; mName="Execute"; }', [d.Name]));
  Fjs.WriteLn(Format('protected %s throws NoSuchMethodException { throw new NoSuchMethodException(); }', [GetJavaProcDeclaration(d, 'Execute')]));
  Fjs.DecI;
  Fjs.WriteLn('}');
  Fjs.WriteLn;
end;

procedure TWriter.WriteSet(d: TSetDef);
begin
  if not d.IsUsed then
    exit;
  if d.ElType = nil then
    raise Exception.Create('No element type.');

  WriteComment(d, '');
  Fjs.WriteLn(Format('/* set of %s */', [d.ElType.Name]));
  if d.Size > 4 then begin
    Fjs.WriteLn('/* Set size more than 32 bits is not supported */');
    exit;
  end;

  RegisterPseudoClass(d);

  Fjs.WriteLn(Format('public static class %s extends %s.system.Set<%s,%s> {', [d.Name, JavaPackage, d.Name, d.ElType.Name]));
  Fjs.IncI;
  Fjs.WriteLn(Format('@Override protected byte Size() { return %d; }', [d.Size]));
  Fjs.WriteLn(Format('@Override protected int Base() { return %d; }', [d.Base]));
  Fjs.WriteLn(Format('@Override protected int ElMax() { return %d; }', [d.ElMax]));
  Fjs.WriteLn(Format('public %s() { }', [d.Name]));
  Fjs.WriteLn(Format('public %s(%s... Elements) { super(Elements); }', [d.Name, d.ElType.Name]));
  Fjs.WriteLn(Format('public %0:s(%0:s... Elements) { super(Elements); }', [d.Name]));
  Fjs.WriteLn(Format('public static %0:s Exclude(%0:s s1, %0:s s2) { %0:s r = new %0:s(s1); r.Exclude(s2); return r; }', [d.Name]));
  Fjs.WriteLn(Format('public static %0:s Intersect(%0:s s1, %0:s s2) { %0:s r = new %0:s(s1); r.Intersect(s2); return r; }', [d.Name]));
  Fjs.DecI;
  Fjs.WriteLn('}');
  Fjs.WriteLn;
end;

procedure TWriter.WritePointer(d: TPointerDef; PreInfo: boolean);
begin
  if not d.IsUsed or not d.IsObjPtr then
    exit;
  if PreInfo then begin
    RegisterPseudoClass(d);
    WriteClassInfoVar(d);
    exit;
  end;

  WriteComment(d, 'pointer');
  Fjs.WriteLn(Format('public static class %s extends %s {', [d.Name, d.PtrType.Name]));
  Fjs.IncI;
  if TClassDef(d.PtrType).CType in [ctObject, ctRecord] then
    Fjs.WriteLn(Format('protected %s(long objptr, boolean cleanup) { super(objptr, false); }', [d.Name]));
  Fjs.WriteLn(Format('public %s(PascalObject obj) { super(obj); }', [d.Name]));
  Fjs.WriteLn(Format('public %s(long objptr) { super(objptr); }', [d.Name]));
  Fjs.DecI;
  Fjs.WriteLn('}');
  Fjs.WriteLn;
end;

procedure TWriter.WriteClassRef(d: TClassRefDef; PreInfo: boolean);
begin
  if not d.IsUsed then
    exit;
  if PreInfo then begin
    RegisterPseudoClass(d);
    WriteClassInfoVar(d);
    exit;
  end;

  WriteComment(d, 'class ref');
  Fjs.WriteLn(Format('public static class %s extends %s {', [d.Name, d.ClassRef.Name]));
  Fjs.IncI;
  Fjs.WriteLn(Format('public %s(PascalObject obj) { super(obj); }', [d.Name]));
  Fjs.DecI;
  Fjs.WriteLn('}');
  Fjs.WriteLn;
end;

procedure TWriter.WriteUnit(u: TUnitDef);

  procedure _ProcessExcludedProcParams(d: TDef);
  var
    i: integer;
  begin
    if not d.IsUsed then
      exit;
    if d.DefType in [dtProc, dtProcType] then begin
      for i:=0 to d.Count - 1 do
        if d[i].DefType = dtParam then
          with TVarDef(d[i]) do
            if (VarType <> nil) and not VarType.IsUsed then begin
              d.SetNotUsed;
              break;
            end;
    end
    else
      for i:=0 to d.Count - 1 do
        _ProcessExcludedProcParams(d[i]);
  end;

var
  d: TDef;
  i: integer;
  f: boolean;
begin
  if u.Processed then
    exit;
  u.Processed:=True;

  if not u.IsUsed then
    exit;

  _ProcessExcludedProcParams(u);

  for i:=0 to High(u.UsedUnits) do
    WriteUnit(u.UsedUnits[i]);

  Fps.WriteLn;
  Fps.WriteLn(Format('{ Unit %s }', [u.Name]));

  u.Name:=LowerCase(u.Name);
  Fjs:=TTextOutStream.Create(IncludeTrailingPathDelimiter(FPkgDir) + u.Name + '.java', fmCreate);
  try
    WriteFileComment(Fjs);
    Fjs.WriteLn(Format('package %s;', [JavaPackage]));
    if Length(u.UsedUnits) > 0 then begin
      UpdateUsedUnits(u);
      f:=False;
      for i:=0 to High(u.UsedUnits) do
        if u.UsedUnits[i].IsUnitUsed then begin
          if not f then begin
            Fjs.WriteLn;
            f:=True;
          end;
          Fjs.WriteLn(Format('import %s.%s.*;', [JavaPackage, LowerCase(u.UsedUnits[i].Name)]));
        end;
    end;
    if u.Name = 'system' then begin
      Fjs.WriteLn;
      Fjs.WriteLn('import java.util.Date;');
      Fjs.WriteLn('import java.util.TimeZone;');
    end;

    Fjs.WriteLn;
    Fjs.WriteLn('public class ' + u.Name + ' {');
    Fjs.IncI;
    if u.Name = 'system' then begin

      for i:=0 to u.Count - 1 do begin
        d:=u[i];
        if (d.DefType = dtType) and (TTypeDef(d).BasicType = btLongInt) and (Copy(d.Name, 1, 1) <> '$') then begin
          FIntegerType:=d;
          break;
        end;
      end;

      if FIntegerType = nil then
        raise Exception.Create('LongInt type has not been found in the System unit.');

      if LibAutoLoad then begin
        Fjs.WriteLn('static private boolean _JniLibLoaded = false;');
        Fjs.WriteLn('public static void InitJni() {');
        Fjs.WriteLn('if (!_JniLibLoaded) {', 1);
        Fjs.WriteLn('_JniLibLoaded=true;', 2);
        Fjs.WriteLn(Format('System.loadLibrary("%s");', [LibName]), 2);
        Fjs.WriteLn('}', 1);
        Fjs.WriteLn('}');
      end;

      // Public support functions
      Fjs.WriteLn('public native static long AllocMemory(int Size);');
      AddNativeMethod(u, '_AllocMemory', 'AllocMemory', '(I)J');

      Fjs.WriteLn('public native static byte[] GetMemoryAsArray(long SrcBuf, int BufSize);');
      AddNativeMethod(u, '_GetMemoryAsArray', 'GetMemoryAsArray', '(JI)[B');

      Fjs.WriteLn('public native static void SetMemoryFromArray(long DstBuf, byte[] SrcArray);');
      AddNativeMethod(u, '_SetMemoryFromArray', 'SetMemoryFromArray', '(J[B)V');

      // Base object
      Fjs.WriteLn;
      Fjs.WriteLn('public static class PascalObject {');
      Fjs.IncI;
      if LibAutoLoad then
        Fjs.WriteLn(Format('static { %s.system.InitJni(); }', [JavaPackage]));
      Fjs.WriteLn('protected long _pasobj = 0;');
      Fjs.WriteLn('protected PascalObject() { }');
      Fjs.WriteLn('protected PascalObject(PascalObject obj) { if (obj != null) _pasobj=obj._pasobj; }');
      Fjs.WriteLn('protected PascalObject(long objptr) { _pasobj=objptr; }');
      Fjs.WriteLn('@Override protected void finalize() { }');
      Fjs.WriteLn('@Override public boolean equals(Object o) { return ((o instanceof PascalObject) && _pasobj == ((PascalObject)o)._pasobj); }');
      Fjs.WriteLn('@Override public int hashCode() { return (int)_pasobj; }');
      Fjs.DecI;
      Fjs.WriteLn('}');
      Fjs.WriteLn;
      Fjs.WriteLn('public static long Pointer(PascalObject obj) { return (obj == null) ? 0 : obj._pasobj; }');

      // Object with finalization
      Fjs.WriteLn;
      Fjs.WriteLn('public static class PascalObjectEx extends PascalObject {');
      Fjs.IncI;
      Fjs.WriteLn('protected boolean _cleanup = false;');
      Fjs.WriteLn('@Override protected void finalize() { ');
{$ifdef DEBUG}
      Fjs.WriteLn('String s = "finalize(): " + getClass().getName(); if (_cleanup) s=s+". Need __Release(). ptr="+_pasobj; System.out.println(s);', 1);
{$endif DEBUG}
      Fjs.WriteLn('if (_cleanup) __Release();', 1);
      Fjs.WriteLn('super.finalize();', 1);
      Fjs.WriteLn('}');
      Fjs.WriteLn('protected PascalObjectEx() { }');
      Fjs.WriteLn('protected PascalObjectEx(PascalObject obj) { super(obj); }');
      Fjs.WriteLn('protected PascalObjectEx(long objptr) { super(objptr); }');
      Fjs.WriteLn('public void __Release() { _pasobj = 0; }');
      Fjs.DecI;
      Fjs.WriteLn('}');

      // Record
      Fjs.WriteLn;
      Fjs.WriteLn('public static class Record extends PascalObjectEx {');
      Fjs.IncI;
      Fjs.WriteLn('protected PascalObject _objref;');
      Fjs.WriteLn('@Override protected void finalize() { if (_pasobj < 0) { _pasobj=-_pasobj; _cleanup=true; } super.finalize(); }');
      Fjs.WriteLn('protected void __Init(long objptr, boolean cleanup) { _pasobj=objptr; _cleanup=cleanup; if (_pasobj==0 && __Size() != 0) _pasobj=AllocMemory(__Size()); }');
      Fjs.WriteLn('protected Record(PascalObject obj) { super(obj); _objref=obj; }');
      Fjs.WriteLn('protected Record(long objptr) { super(objptr); }');
      Fjs.WriteLn('protected final int __Size(int index) { return GetRecordSize(index); }');
      Fjs.WriteLn('public Record() { }');
      Fjs.WriteLn('public int __Size() { return 0; }');
      Fjs.DecI;
      Fjs.WriteLn('}');

      Fjs.WriteLn;
      Fjs.WriteLn('private native static int GetRecordSize(int index);');
      AddNativeMethod(u, '_GetRecordSize', 'GetRecordSize', '(I)I');

      // Method pointer base class
      d:=TClassDef.Create(FThisUnit, dtClass);
      d.Name:='_TMethodPtrInfo';
      d.AliasName:='MethodPtr';
      WriteClassInfoVar(d);

      // Method pointer support
      Fps.WriteLn;
      Fps.WriteLn('type');
      Fps.IncI;
      Fps.WriteLn('_TMethodPtrInfo = class');
      Fps.IncI;
      Fps.WriteLn('Obj: JObject;');
      Fps.WriteLn('MethodId: JMethodID;');
      Fps.WriteLn('Index, RefCnt: integer;');
      Fps.WriteLn('RealMethod: TMethod;');
      Fps.WriteLn('InlineHandler: boolean;');
      Fps.WriteLn('constructor Create(env: PJNIEnv; JavaObj: JObject; const AMethodName, AMethodSig: ansistring);');
      Fps.WriteLn('procedure Release(env: PJNIEnv);');
      Fps.DecI;
      Fps.WriteLn('end;');
      Fps.DecI;
      Fps.WriteLn;
      Fps.WriteLn('var _MethodPointers: array of _TMethodPtrInfo;');
      Fps.WriteLn('var _MethodPointersCS: TCriticalSection;');
      Fps.WriteLn;

      Fps.WriteLn('constructor _TMethodPtrInfo.Create(env: PJNIEnv; JavaObj: JObject; const AMethodName, AMethodSig: ansistring);');
      Fps.WriteLn('var c: JClass;');
      Fps.WriteLn('begin');
      Fps.IncI;
      Fps.WriteLn('if (JavaObj = nil) or (AMethodName = '''') then exit;');
      Fps.WriteLn('c:=env^^.GetObjectClass(env, JavaObj);');
      Fps.WriteLn('if c = nil then exit;');
      Fps.WriteLn('MethodId:=env^^.GetMethodID(env, c, PAnsiChar(AMethodName), PAnsiChar(AMethodSig));');
      Fps.WriteLn('if MethodId = nil then raise Exception.CreateFmt(''Method "%s" does not exist or has wrong parameters.'', [AMethodName]);');
      Fps.WriteLn('Obj:=env^^.NewGlobalRef(env, JavaObj);');
      Fps.WriteLn('_MethodPointersCS.Enter;');
      Fps.WriteLn('try');
      Fps.IncI;
      Fps.WriteLn('Index:=Length(_MethodPointers) + 1;');
      Fps.WriteLn(Format('if Index > %d then raise Exception.Create(''Too many method pointers.'');', [MaxMethodPointers]));
      Fps.WriteLn('SetLength(_MethodPointers, Index);');
      Fps.WriteLn('_MethodPointers[Index - 1]:=Self;');
      Fps.WriteLn('finally', -1);
      Fps.WriteLn('_MethodPointersCS.Leave;');
      Fps.DecI;
      Fps.WriteLn('end;');
      Fps.DecI;
      Fps.WriteLn('end;');

      Fps.WriteLn;
      Fps.WriteLn('procedure _TMethodPtrInfo.Release(env: PJNIEnv);');
      Fps.WriteLn('var i: integer;');
      Fps.WriteLn('begin');
      Fps.IncI;
      Fps.WriteLn('i:=InterlockedDecrement(RefCnt);');
      {$ifdef DEBUG}
      Fps.WriteLn('writeln(''_TMethodPtrInfo.Release(). RefCnt='',i,'' ptr='',ptruint(Self));');
      {$endif DEBUG}
      Fps.WriteLn('if i <> 0 then exit;');
      Fps.WriteLn('if Index > 0 then begin');
      Fps.IncI;
      Fps.WriteLn('_MethodPointersCS.Enter;');
      Fps.WriteLn('try');
      Fps.IncI;
      Fps.WriteLn('if InlineHandler then begin');
      Fps.IncI;
      {$ifdef DEBUG}
      Fps.WriteLn('writeln(''Finalizing Java inline handler.'');');
      {$endif DEBUG}
      Fps.WriteLn(Format('env^^.SetLongField(env, Obj, %s.ObjFieldId, -1);', [GetTypeInfoVar(d)]));
      Fps.DecI;
      Fps.WriteLn('end;');
      Fps.WriteLn('env^^.DeleteGlobalRef(env, Obj);');
      Fps.WriteLn('_MethodPointers[Index-1]:=nil;');
      Fps.WriteLn('Index:=High(_MethodPointers);');
      Fps.WriteLn('while (Index >= 0) and (_MethodPointers[Index] = nil) do Dec(Index);');
      Fps.WriteLn('SetLength(_MethodPointers, Index + 1);');
      Fps.WriteLn('finally', -1);
      Fps.WriteLn('_MethodPointersCS.Leave;');
      Fps.DecI;
      Fps.WriteLn('end;');
      Fps.DecI;
      Fps.WriteLn('end;');
      Fps.WriteLn('Self.Destroy;');
  {$ifdef DEBUG}
      Fps.WriteLn('writeln(''_TMethodPtrInfo destroyed.'');');
  {$endif DEBUG}
      Fps.DecI;
      Fps.WriteLn('end;');

      Fps.WriteLn;
      Fps.WriteLn('procedure _RefMethodPtr(env: PJNIEnv; const m: TMethod; AddRef: boolean);');
      Fps.WriteLn('var i: integer;');
      Fps.WriteLn('begin');
      Fps.IncI;
      Fps.WriteLn('i:=-integer(ptruint(m.Data));');
  {$ifdef DEBUG}
      Fps.WriteLn('writeln(''_RefMethodPtr. i='',i,''  AddRef='',AddRef);');
  {$endif DEBUG}
      Fps.WriteLn(Format('if (i < 1) or (i > %d) then exit;', [MaxMethodPointers]));
      Fps.WriteLn('_MethodPointersCS.Enter;');
      Fps.WriteLn('try');
      Fps.IncI;
      Fps.WriteLn('with _MethodPointers[i - 1] do');
      Fps.WriteLn('if AddRef then InterlockedIncrement(RefCnt) else Release(env);', 1);
      Fps.WriteLn('finally', -1);
      Fps.WriteLn('_MethodPointersCS.Leave;');
      Fps.DecI;
      Fps.WriteLn('end;');
      Fps.DecI;
      Fps.WriteLn('end;');

      Fps.WriteLn;
      Fps.WriteLn('function _CreateMethodPtrObject(env: PJNIEnv; const m: TMethod; const ci: _TJavaClassInfo): jobject;');
      Fps.WriteLn('var i: integer;');
      Fps.WriteLn('var mpi: _TMethodPtrInfo;');
      Fps.WriteLn('begin');
      Fps.IncI;
      Fps.WriteLn('Result:=nil;');
      Fps.WriteLn('if (m.Data = nil) and (m.Code = nil) then exit;');
      Fps.WriteLn('_MethodPointersCS.Enter;');
      Fps.WriteLn('try');
      Fps.IncI;
      Fps.WriteLn('i:=-integer(ptruint(m.Data));');
      Fps.WriteLn(Format('if (i > 0) and (i <= %d) then begin', [MaxMethodPointers]));
      Fps.WriteLn('mpi:=_MethodPointers[i - 1];', 1);
      Fps.WriteLn('end');
      Fps.WriteLn('else begin');
      Fps.WriteLn('mpi:=_TMethodPtrInfo.Create(env, nil, '''', '''');', 1);
      Fps.WriteLn('mpi.RealMethod:=m;', 1);
      Fps.WriteLn('end;');
      Fps.WriteLn('InterlockedIncrement(mpi.RefCnt);');
      Fps.WriteLn('finally', -1);
      Fps.WriteLn('_MethodPointersCS.Leave;');
      Fps.DecI;
      Fps.WriteLn('end;');
      Fps.WriteLn('Result:=_CreateJavaObj(env, pointer(mpi), ci);');
      Fps.DecI;
      Fps.WriteLn('end;');

      Fps.WriteLn;
      Fps.WriteLn('function _GetMethodPtrHandler(env: PJNIEnv; jobj: jobject; hptr: pointer; const ci: _TJavaClassInfo): TMethod;');
      Fps.WriteLn('var mpi: _TMethodPtrInfo;');
      Fps.WriteLn('begin');
      Fps.IncI;
      Fps.WriteLn(  'Result.Data:=nil; Result.Code:=nil;');
      Fps.WriteLn(  'mpi:=_TMethodPtrInfo(_GetPasObj(env, jobj, ci, False));');
      Fps.WriteLn(  'if mpi = nil then exit;');
      Fps.WriteLn(  'if pointer(mpi) = pointer(ptruint(-1)) then begin');
      Fps.WriteLn(    'env^^.CallVoidMethodA(env, jobj, env^^.GetMethodID(env, ci.ClassRef, ''Init'', ''()V''), nil);', 1);
      Fps.WriteLn(    'Result:=_GetMethodPtrHandler(env, jobj, hptr, ci);', 1);
      Fps.WriteLn(    'exit;', 1);
      Fps.WriteLn(  'end;');
      Fps.WriteLn(  'if mpi.Index = 0 then');
      Fps.WriteLn(    'TMethod(Result):=mpi.RealMethod', 1);
      Fps.WriteLn(  'else');
      Fps.WriteLn(    'with TMethod(Result) do begin', 1);
      Fps.WriteLn(      'Data:=pointer(ptruint(-integer(mpi.Index)));', 2);
      Fps.WriteLn(      'Code:=hptr;', 2);
      Fps.WriteLn(    'end;', 1);
      Fps.DecI;
      Fps.WriteLn('end;');

      Fps.WriteLn;
      Fps.WriteLn('procedure _TMethodPtrInfo_Init(env: PJNIEnv; _self, JavaObj: JObject; AMethodName, AMethodSig: jstring; IncRef: jboolean);' + JniCaliing);
      Fps.WriteLn('var mpi: _TMethodPtrInfo;');
      Fps.WriteLn('begin');
      Fps.IncI;
      EHandlerStart;
      Fps.WriteLn('mpi:=_TMethodPtrInfo.Create(env, JavaObj, ansistring(_StringFromJString(env, AMethodName)), ansistring(_StringFromJString(env, AMethodSig)));');
      Fps.WriteLn('if IncRef <> 0 then');
      Fps.WriteLn('InterlockedIncrement(mpi.RefCnt)', 1);
      Fps.WriteLn('else');
      Fps.WriteLn('mpi.InlineHandler:=True;', 1);
{$ifdef DEBUG}
      Fps.WriteLn('writeln(''_TMethodPtrInfo_Init. RefCnt='',mpi.RefCnt,'' ptr='',ptruint(mpi));');
{$endif DEBUG}
      Fps.WriteLn(Format('env^^.SetLongField(env, _self, %s.ObjFieldId, Int64(ptruint(mpi)));', [GetTypeInfoVar(d)]));
      EHandlerEnd('env');
      Fps.DecI;
      Fps.WriteLn('end;');

      AddNativeMethod(d, '_TMethodPtrInfo_Init', '__Init', Format('(Ljava/lang/Object;%s%sZ)V', [JNITypeSig[btString], JNITypeSig[btString]]));

      Fps.WriteLn;
      Fps.WriteLn('procedure _TMethodPtrInfo_Release(env: PJNIEnv; _self: JObject);' + JniCaliing);
      Fps.WriteLn('begin');
      Fps.IncI;
      EHandlerStart;
      Fps.WriteLn(Format('_TMethodPtrInfo(_GetPasObj(env, _self, %s, True)).Release(env);', [GetTypeInfoVar(d)]));
      EHandlerEnd('env');
      Fps.DecI;
      Fps.WriteLn('end;');

      AddNativeMethod(d, '_TMethodPtrInfo_Release', '__Destroy', '()V');

      Fjs.WriteLn;
      Fjs.WriteLn('public static class MethodPtr extends PascalObjectEx {');
      Fjs.IncI;
      Fjs.WriteLn('private native void __Init(Object Obj, String MethodName, String MethodSignature, boolean IncRef);');
      Fjs.WriteLn('private native void __Destroy();');
      Fjs.WriteLn('protected Object mObject;');
      Fjs.WriteLn('protected String mName;');
      Fjs.WriteLn('protected String mSignature;');
      Fjs.WriteLn('protected void Init() { __Init(mObject, mName, mSignature, this != mObject); }');
      Fjs.WriteLn('protected MethodPtr() { _cleanup=true; _pasobj=-1; }');
      Fjs.WriteLn('public void __Release() { if (_pasobj > 0) __Destroy(); }');
      Fjs.DecI;
      Fjs.WriteLn('}');
      Fjs.WriteLn;

      // Base class for Enum
      Fjs.WriteLn('public static class Enum {');
      Fjs.IncI;
      Fjs.WriteLn('public int Value;');
      Fjs.WriteLn('public int Ord() { return Value; }');
      Fjs.WriteLn('@Override public boolean equals(Object o) { return (o instanceof Integer) && Value == (Integer)o; }');
      Fjs.WriteLn('public boolean equals(int v) { return Value == v; }');
      Fjs.WriteLn('@Override public int hashCode() { return Value; }');
      Fjs.DecI;
      Fjs.WriteLn('}');
      Fjs.WriteLn;

      // Base class for Set
      Fjs.WriteLn('private static abstract class BaseSet {');
      Fjs.IncI;
      Fjs.WriteLn('protected int Value = 0;');
      Fjs.WriteLn('protected abstract byte Size();');
      Fjs.WriteLn('protected abstract int Base();');
      Fjs.WriteLn('protected abstract int ElMax();');
      Fjs.WriteLn('public BaseSet() { }');
      Fjs.DecI;
      Fjs.WriteLn('}');

      Fjs.WriteLn('public static abstract class Set<TS extends BaseSet,TE extends Enum> extends BaseSet {');
      Fjs.IncI;
      Fjs.WriteLn('protected int GetMask(TE Element) { return 1 << (Element.Ord() - Base()); }');
      Fjs.WriteLn('public Set() { }');
      Fjs.WriteLn('@SuppressWarnings({"unchecked", "varargs"})');
      Fjs.WriteLn('public Set(TE... Elements) { Include(Elements); }');
      Fjs.WriteLn('@SuppressWarnings({"unchecked", "varargs"})');
      Fjs.WriteLn('public Set(TS... Elements) { for (TS e : Elements) Include(e); }');
      Fjs.WriteLn('@SuppressWarnings({"unchecked", "varargs"})');
      Fjs.WriteLn('public void Include(TE... Elements) { for (TE e: Elements) Value = Value | GetMask(e); }');
      Fjs.WriteLn('public void Include(TS s) { Value=Value | s.Value; }');
      Fjs.WriteLn('@SuppressWarnings({"unchecked", "varargs"})');
      Fjs.WriteLn('public void Exclude(TE... Elements) { for (TE e: Elements) Value = Value & ~GetMask(e); }');
      Fjs.WriteLn('public void Exclude(TS s) { Value=Value & ~s.Value; }');
      Fjs.WriteLn('public void Assign(TS s) { Value=s.Value; }');
      Fjs.WriteLn('public void Intersect(TS s) { Value=Value & s.Value; }');
      Fjs.WriteLn('public boolean Has(TE Element) { return (Value & GetMask(Element)) != 0; }');
      Fjs.WriteLn('public boolean IsEmpty() { return Value == 0; }');
      Fjs.WriteLn('public boolean equals(TS s) { return Value == s.Value; }');
      Fjs.WriteLn('public boolean equals(TE Element) { return Value == Element.Ord(); }');
      Fjs.WriteLn('public boolean equals(int Element) { return Value == Element; }');
      Fjs.DecI;
      Fjs.WriteLn('}');
      Fjs.WriteLn;

      // TDateTime support
      Fjs.WriteLn('public static class TDateTime {');
      Fjs.IncI;
      Fjs.WriteLn('public static Date toDateUTC(double d) {');
      Fjs.WriteLn('return new Date(Math.round((d - 25569)*86400000.0));', 1);
      Fjs.WriteLn('}');
      Fjs.WriteLn('public static Date toDate(double d) {');
      Fjs.WriteLn('long t = Math.round((d - 25569)*86400000.0); return new Date(t - TimeZone.getDefault().getOffset(t));', 1);
      Fjs.WriteLn('}');
      Fjs.WriteLn('public static double getUTC(Date d) {');
      Fjs.WriteLn('return d.getTime()/86400000.0 + 25569;', 1);
      Fjs.WriteLn('}');
      Fjs.WriteLn('public static double get(Date d) {');
      Fjs.WriteLn('return (d.getTime() + TimeZone.getDefault().getOffset(d.getTime()))/86400000.0 + 25569;', 1);
      Fjs.WriteLn('}');
      Fjs.DecI;
      Fjs.WriteLn('}');
      Fjs.WriteLn;

      // Interface support
      Fps.WriteLn;
      Fps.WriteLn('function _IntfCast(env: PJNIEnv; _self: JObject; objptr: jlong; objid: jstring): jlong;' + JniCaliing);
      Fps.WriteLn('var');
      Fps.WriteLn('obj: system.TObject;', 1);
      Fps.WriteLn('intf: IUnknown;', 1);
      Fps.WriteLn('begin');
      Fps.IncI;
      Fps.WriteLn('Result:=0;');
      EHandlerStart;
      Fps.WriteLn('if objptr = 0 then exit;');
      Fps.WriteLn('if objid = nil then');
      Fps.WriteLn('raise Exception.Create(''A GUID must be assigned for the interface to allow a type cast.'');', 1);
      Fps.WriteLn('obj:=system.TObject(pointer(ptruint(objptr)));');
      Fps.WriteLn('if not (obj is system.TInterfacedObject) then');
      Fps.WriteLn('raise Exception.Create(''Object must be inherited from TInterfacedObject.'');', 1);
      Fps.WriteLn('if (system.TInterfacedObject(obj) as IUnknown).QueryInterface(StringToGUID(ansistring(_StringFromJString(env, objid))), intf) <> 0 then');
      Fps.WriteLn('raise Exception.Create(''Invalid type cast.'');', 1);
      Fps.WriteLn('intf._AddRef;');
      Fps.WriteLn('Result:=ptruint(intf);');
      EHandlerEnd('env');
      Fps.DecI;
      Fps.WriteLn('end;');

      AddNativeMethod(u, '_IntfCast', 'InterfaceCast', '(JLjava/lang/String;)J');

      Fjs.WriteLn('private native static long InterfaceCast(long objptr, String objid);');
      Fjs.WriteLn;
      Fjs.WriteLn('public static abstract class PascalInterface extends PascalObjectEx {');
      Fjs.IncI;
      Fjs.WriteLn('abstract protected void __Init();');
      Fjs.WriteLn('public void __TypeCast(PascalObject obj, String intfId) {');
      Fjs.WriteLn('if (obj != null) {', 1);
      Fjs.WriteLn('if (obj instanceof PascalInterface) {', 2);
      Fjs.WriteLn('_pasobj=obj._pasobj;',3);
      Fjs.WriteLn('__Init();',3);
      Fjs.WriteLn('} else',2);
      Fjs.WriteLn('_pasobj=InterfaceCast(obj._pasobj, intfId);', 3);
      Fjs.WriteLn('}', 1);
      Fjs.WriteLn('}');
      Fjs.WriteLn('protected PascalInterface(long objptr, boolean cleanup) { _pasobj=objptr; __Init(); }');
      Fjs.DecI;
      Fjs.WriteLn('}');
      Fjs.WriteLn;

    end;
    if LibAutoLoad then begin
      Fjs.WriteLn(Format('static { %s.system.InitJni(); }', [JavaPackage]));
      Fjs.WriteLn;
    end;

    // First pass
    for i:=0 to u.Count - 1 do begin
      d:=u[i];
      if not d.IsUsed then
        continue;
      case d.DefType of
        dtSet, dtEnum:
          WriteClassInfoVar(d);
        dtClass:
          WriteClass(TClassDef(d), True);
        dtProcType:
          WriteProcType(TProcDef(d), True);
        dtPointer:
          WritePointer(TPointerDef(d), True);
        dtClassRef:
          WriteClassRef(TClassRefDef(d), True);
        else
          ; // no action
      end;
    end;

    // Second pass
    for i:=0 to u.Count - 1 do begin
      d:=u[i];
      if not d.IsUsed then
        continue;
      case d.DefType of
        dtClass:
          WriteClass(TClassDef(d), False);
        dtProc:
          WriteProc(TProcDef(d));
        dtVar, dtProp:
          WriteVar(TVarDef(d));
        dtEnum:
          WriteEnum(d);
        dtProcType:
          WriteProcType(TProcDef(d), False);
        dtSet:
          WriteSet(TSetDef(d));
        dtConst:
          WriteConst(TConstDef(d));
        dtPointer:
          WritePointer(TPointerDef(d), False);
        dtClassRef:
          WriteClassRef(TClassRefDef(d), False);
        else
          ; // no action
      end;
    end;

    // Class ref helpers
    if (u.Name = 'system') and (FClasses.IndexOf('system.TClass', nil) >= 0) then begin
      Fjs.WriteLn('native static long GetClassRef(int index);');
      AddNativeMethod(u, '_GetClassRef', 'GetClassRef', '(I)J');
      Fjs.WriteLn('static TClass GetTClass(int index) { TClass c = new TClass(null); c._pasobj=GetClassRef(index); return c; }');
      Fjs.WriteLn;
    end;

    Fjs.DecI;
    Fjs.WriteLn('}');
  finally
    Fjs.Free;
  end;
end;

procedure TWriter.WriteOnLoad;
var
  i, j: integer;
  ci: TClassInfo;
  s, ss, fn: string;
  d: TTypeDef;
begin
  if FClasses.Count = 0 then
    exit;
  Fps.WriteLn;
  Fps.WriteLn('function JNI_OnLoad(vm: PJavaVM; reserved: pointer): jint;' + JniCaliing);

  Fps.WriteLn('const');
  for i:=0 to FClasses.Count - 1 do begin
    ci:=FClasses.GetClassInfo(i);
    if ci.Funcs.Count = 0 then
      continue;
    Fps.WriteLn(Format('  _%sNativeMethods: array[0..%d] of JNINativeMethod = (', [GetClassPrefix(ci.Def, FClasses.GetClassName(i)), ci.Funcs.Count - 1]));
    for j:=0 to ci.Funcs.Count - 1 do begin
      with TProcInfo(ci.Funcs[j]) do
        Fps.Write(Format('    (name: ''%s''; signature: ''%s''; fnPtr: @%s)', [Name, JniSignature, JniName]));
      if j < ci.Funcs.Count - 1 then
        Fps.Write(',');
      Fps.WriteLn;
    end;
    Fps.WriteLn('  );');
  end;

  Fps.WriteLn;
  Fps.WriteLn('var');
  Fps.IncI;
  Fps.WriteLn('env: PJNIEnv;');
  Fps.WriteLn;
  Fps.WriteLn('function _Reg(ClassName: PAnsiChar; Methods: PJNINativeMethod; Count: integer; ci: _PJavaClassInfo; const FieldName: ansistring = ''_pasobj''; const FieldSig: ansistring = ''J''): boolean;');
  Fps.WriteLn('var');
  Fps.WriteLn('c: jclass;', 1);
  Fps.WriteLn('begin');
  Fps.IncI;
  Fps.WriteLn('Result:=False;');
  Fps.WriteLn('c:=env^^.FindClass(env, ClassName);');
  Fps.WriteLn('if c = nil then exit;');
  Fps.WriteLn('Result:=(Count = 0) or (env^^.RegisterNatives(env, c, Methods, Count) = 0);');
  Fps.WriteLn('if Result and (ci <> nil) then begin');
  Fps.IncI;
  Fps.WriteLn('ci^.ClassRef:=env^^.NewGlobalRef(env, c);');
  Fps.WriteLn('Result:=ci^.ClassRef <> nil;');
  Fps.WriteLn('if Result and (env^^.ExceptionCheck(env) = 0) then begin');
  Fps.WriteLn('ci^.ConstrId:=env^^.GetMethodID(env, ci^.ClassRef, ''<init>'', ''(JZ)V'');', 1);
  Fps.WriteLn('env^^.ExceptionClear(env);', 1);
  Fps.WriteLn('end;');
  Fps.WriteLn('if Result and (FieldName <> '''') then begin');
  Fps.WriteLn('ci^.ObjFieldId:=env^^.GetFieldID(env, ci^.ClassRef, PAnsiChar(FieldName), PAnsiChar(FieldSig));', 1);
  Fps.WriteLn('Result:=ci^.ObjFieldId <> nil;', 1);
  Fps.WriteLn('end;');
  Fps.DecI;
  Fps.WriteLn('end;');
  Fps.DecI;
  Fps.WriteLn('end;');
  Fps.WriteLn;
  Fps.WriteLn('begin', -1);
  Fps.WriteLn('Result:=JNI_ERR;');
  Fps.WriteLn('if vm^^.GetEnv(vm, @env, JNI_VERSION_1_6) <> JNI_OK then exit;');
  Fps.WriteLn('CurJavaVM:=vm;');

  d:=TTypeDef.Create(nil, dtType);
  try
    d.BasicType:=btString;
    s:=JNITypeSig[d.BasicType];
    s:=Copy(s, 2, Length(s) - 2);
    Fps.WriteLn(Format('if not _Reg(''%s'', nil, 0, @%s, '''', '''') then exit;',
                       [s, GetTypeInfoVar(d)]));
  finally
    d.Free;
  end;

  for i:=0 to FClasses.Count - 1 do begin
    ci:=FClasses.GetClassInfo(i);
    s:=GetTypeInfoVar(ci.Def);
    if (s = '') or (ci.IsCommonClass) then
      s:='nil'
    else
      s:='@' + s;
    if ci.Funcs.Count = 0 then
      ss:='nil'
    else
      ss:=Format('@_%sNativeMethods', [GetClassPrefix(ci.Def, FClasses.GetClassName(i))]);
    fn:='';
    if ci.Def <> nil then
      if ci.Def.DefType in [dtSet, dtEnum] then
        fn:=', ''Value'', ''I''';
    Fps.WriteLn(Format('if not _Reg(''%s'', %s, %d, %s%s) then exit;',
                       [GetJavaClassPath(ci.Def, FClasses.GetClassName(i)), ss, ci.Funcs.Count, s, fn]));
  end;

  Fps.WriteLn('Result:=JNI_VERSION_1_6;');
  Fps.DecI;
  Fps.WriteLn('end;');
  Fps.WriteLn;
  Fps.WriteLn('exports JNI_OnLoad;');
end;

procedure TWriter.WriteRecordSizes;
var
  i, j: integer;
  s: string;
begin
  Fps.WriteLn;
  Fps.WriteLn('function _GetRecordSize(env: PJNIEnv; jobj: jobject; index: jint): jint;' + JniCaliing);
  if FRecords.Count > 0 then begin
    Fps.WriteLn(Format('const sizes: array[0..%d] of longint =', [FRecords.Count - 1]));
    Fps.IncI;
    s:='(';
    j:=0;
    for i:=0 to FRecords.Count - 1 do begin
      if i > 0 then
        s:=s + ',';
      Inc(j);
      if j > 20 then begin
        Fps.WriteLn(s);
        s:='';
        j:=0;
      end;
      s:=s + IntToStr(TClassDef(FRecords[i]).Size);
    end;
    Fps.WriteLn(s + ');');
    Fps.DecI;
  end;
  Fps.WriteLn('begin');
  if FRecords.Count > 0 then
    s:='sizes[index]'
  else
    s:='0';
  Fps.WriteLn('Result:=' + s + ';', 1);
  Fps.WriteLn('end;');
end;

procedure TWriter.WriteClassTable;
var
  i: integer;
  s,ss: string;
begin
  Fps.WriteLn;
  Fps.WriteLn('function _GetClassRef(env: PJNIEnv; jobj: jobject; index: jint): jlong;' + JniCaliing);
  if FRealClasses.Count > 0 then begin
    Fps.WriteLn(Format('const cls: array[0..%d] of TClass =', [FRealClasses.Count - 1]));
    Fps.IncI;
    s:='(';
    for i:=0 to FRealClasses.Count - 1 do begin
      if i > 0 then
        s:=s + ',';
      if Length(s) > 100 then begin
        Fps.WriteLn(s);
        s:='';
      end;
      with TClassDef(FRealClasses[i]) do
        ss:=Parent.Name + '.' + Name;
      s:=s + ss;
    end;
    Fps.WriteLn(s + ');');
    Fps.DecI;
  end;
  Fps.WriteLn('begin');
  if FRealClasses.Count > 0 then
    s:='cls[index]'
  else
    s:='nil';
  Fps.WriteLn('Result:=-jlong(ptruint(pointer(' + s + ')));', 1);
  Fps.WriteLn('end;');
end;

function TWriter.JniToPasType(d: TDef; const v: string; CheckNil: boolean): string;

  function _GetFullName(d: TDef): string;
  begin
    Result:=Format('%s.%s', [d.Parent.Name, d.Name]);
    if Result = 'types.TDuplicates' then
      Result:='classes.TDuplicates';  // Hack for Delphi 7 compatibility
  end;

var
  n: string;
begin
  Result:=v;
  if d = nil then
    exit;
  case d.DefType of
    dtType:
      with TTypeDef(d) do
        case BasicType of
          btString, btWideString:
            begin
              Result:=Format('_StringFromJString(_env, %s)', [Result]);
              if BasicType <> btWideString then
                Result:=Format('%s(%s)', [d.Name, Result]);
            end;
          btBoolean:
            Result:=Format('LongBool(%s)', [Result]);
          btChar:
            Result:=Format('char(widechar(%s))', [Result]);
          btWideChar:
            Result:=Format('widechar(%s)', [Result]);
          btGuid:
            Result:=Format('StringToGUID(ansistring(_StringFromJString(_env, %s)))', [Result]);
          else
            Result:=Format('%s.%s(%s)', [d.Parent.Name, d.Name, Result]);
        end;
    dtClass:
      begin
        if TClassDef(d).CType = ctRecord then
          n:='True'
        else
          if CheckNil then
            n:='True'
          else
            n:='False';
        Result:=Format('_GetPasObj(_env, %s, %s, %s)', [Result, GetTypeInfoVar(d), n]);
        if TClassDef(d).CType in [ctObject, ctRecord] then
          Result:=Result + '^';
        Result:=Format('%s.%s(%s)', [d.Parent.Name, d.Name, Result]);
      end;
    dtProcType:
      Result:=Format('%sGetHandler(_env, %s, %s)', [GetClassPrefix(d), Result, GetTypeInfoVar(d)]);
    dtEnum:
      Result:=Format('%s(_GetIntObjValue(_env, %s, %s))', [_GetFullName(d), Result, GetTypeInfoVar(d)]);
    dtSet:
      Result:=Format('%s.%s(%s(_GetIntObjValue(_env, %s, %s)))', [d.Parent.Name, d.Name, GetPasIntType(TSetDef(d).Size), Result, GetTypeInfoVar(d)]);
    dtPointer:
      begin
        if TPointerDef(d).IsObjPtr then
          Result:=Format('%s.%s(_GetPasObj(_env, %s, %s, False))', [d.Parent.Name, d.Name, Result, GetTypeInfoVar(d)])
        else
          Result:=Format('pointer(ptruint(%s))', [Result]);
      end;
    dtClassRef:
      begin
        Result:=Format('_GetClass(_env, %s, %s)', [Result, GetTypeInfoVar(d)]);
        Result:=Format('%s.%s(%s)', [d.Parent.Name, d.Name, Result]);
      end;
    else
      ; // no action
  end;
end;

function TWriter.PasToJniType(d: TDef; const v: string): string;
begin
  Result:=v;
  if d = nil then
    exit;
  case d.DefType of
    dtType:
      with TTypeDef(d) do
        case BasicType of
          btString, btWideString:
            Result:=Format('_StringToJString(_env, _JNIString(%s))', [Result]);
          btBoolean:
            Result:=Format('(jboolean(%s) and 1)', [Result]);
          btChar:
            Result:=Format('jchar(widechar(%s))', [Result]);
          btWideChar:
            Result:=Format('jchar(%s)', [Result]);
          btEnum:
            Result:=Format('jint(%s)', [Result]);
          btGuid:
            Result:=Format('_StringToJString(_env, _JNIString(GUIDToString(%s)))', [Result]);
          else
            ; // no action
        end;
    dtClass:
      case TClassDef(d).CType of
        ctObject, ctRecord:
          Result:=Format('_%s_CreateObj(_env, %s)', [GetClassPrefix(d), Result]);
        ctInterface:
          Result:=Format('_CreateJavaObj(_env, pointer(%s), %s)', [Result, GetTypeInfoVar(d)]);
        else
          Result:=Format('_CreateJavaObj(_env, %s, %s)', [Result, GetTypeInfoVar(d)]);
      end;
    dtProcType:
      Result:=Format('_CreateMethodPtrObject(_env, TMethod(%s), %s)', [Result, GetTypeInfoVar(d)]);
    dtEnum:
      Result:=Format('_CreateIntObj(_env, longint(%s), %s)', [Result, GetTypeInfoVar(d)]);
    dtSet:
      Result:=Format('_CreateIntObj(_env, %s(%s), %s)', [GetPasIntType(TSetDef(d).Size), Result, GetTypeInfoVar(d)]);
    dtPointer:
      if TPointerDef(d).IsObjPtr then
        Result:=Format('_CreateJavaObj(_env, %s, %s)', [Result, GetTypeInfoVar(d)])
      else
        Result:=Format('ptruint(pointer(%s))', [Result]);
    dtClassRef:
      Result:=Format('_CreateJavaObj(_env, -jlong(ptruint(pointer(%s))), %s)', [Result, GetTypeInfoVar(d)])
    else
      ; // no action
  end;
end;

function TWriter.GetTypeInfoVar(ClassDef: TDef): string;
begin
  if ClassDef.DefType = dtUnit then
    Result:=''
  else
    if ClassDef.DefType = dtType then
      Result:='_Java_' + JavaType[TTypeDef(ClassDef).BasicType] + '_Info'
    else
      Result:='_JNI_' + ClassDef.Parent.Name + '_' + ClassDef.Name + '_Info';
end;

function TWriter.GetClassPrefix(ClassDef: TDef; const AClassName: string): string;
begin
  if AClassName = '' then
    Result:=ClassDef.Name
  else
    Result:=AClassName;
  Result:=Result + '_';
  if ClassDef.DefType <> dtUnit then
    Result:=ClassDef.Parent.Name + '_' + Result;
  Result:='JNI_' + Result;
end;

function TWriter.IsJavaSimpleType(d: TDef): boolean;
begin
  Result:=d <> nil;
  if Result then
    case d.DefType of
      dtType:
        Result:=Length(JNITypeSig[TTypeDef(d).BasicType]) = 1;
      dtPointer:
        Result:=not TPointerDef(d).IsObjPtr;
      else
        Result:=False;
    end;
end;

function TWriter.IsJavaVarParam(ParamDef: TVarDef): boolean;
begin
  with ParamDef do
    Result:=VarOpt * [voVar, voOut] <> [];
end;

function TWriter.GetProcDeclaration(d: TProcDef; const ProcName: string; FullTypeNames: boolean; InternalParaNames: boolean): string;
var
  s, ss: string;
  j: integer;
begin
  with d do begin
    s:='';
    for j:=0 to Count - 1 do
      with TVarDef(Items[j]) do begin
        if DefType <> dtParam then
          continue;
        if s <> '' then
          s:=s + '; ';
        if voVar in VarOpt then
          s:=s + 'var '
        else
        if voOut in VarOpt then
          s:=s + 'out '
        else
        if voConst in VarOpt then
          s:=s + 'const ';
        if InternalParaNames then
          s:=s + Name
        else
          s:=s + AliasName;
        s:=s + ': ' + GetPasType(VarType, FullTypeNames);
      end;

    if s <> '' then
      s:='(' + s + ')';
    ss:='';
    case ProcType of
      ptConstructor:
        ss:='constructor';
      ptDestructor:
        ss:='destructor';
      ptProcedure:
        ss:='procedure';
      ptFunction:
        ss:='function';
    end;
    if ProcType in [ptConstructor, ptFunction] then
      s:=s + ': ' + GetPasType(ReturnType, FullTypeNames);
    ss:=ss + ' ';
    if ProcName <> '' then
      ss:=ss + ProcName
    else
      ss:=ss + Name;
    Result:=ss + s;
  end;
end;

function TWriter.GetJavaProcDeclaration(d: TProcDef; const ProcName: string): string;
var
  s, ss: string;
  j: integer;
  vd: TVarDef;
begin
  with d do begin
    if ProcName <> '' then
      ss:=ProcName
    else
      ss:=AliasName;
    ss:=DefToJavaType(ReturnType) + ' ' + ss + '(';
    s:='';
    for j:=0 to Count - 1 do begin
      vd:=TVarDef(Items[j]);
      if vd.DefType <> dtParam then
        continue;
      with vd do begin
        if (VarType <> nil) and (VarType.DefType = dtJniEnv) then
          continue;
        if s <> '' then
          s:=s + ', ';
        s:=s + DefToJavaType(VarType);
        if IsJavaVarParam(vd) then
          s:=s + '[]';
        s:=s + ' ' + AliasName;
      end;
    end;
    ss:=ss + s + ')';
  end;
  Result:=ss;
end;

function TWriter.GetJniFuncType(d: TDef): string;
begin
  if IsJavaSimpleType(d) then begin
    if d.DefType = dtPointer then
      Result:='Long'
    else begin
      Result:=JavaType[TTypeDef(d).BasicType];
      Result[1]:=UpCase(Result[1]);
    end;
  end
  else
    Result:='Object';
end;

function TWriter.GetJavaClassName(cls: TDef; it: TDef): string;
begin
  Result:=cls.AliasName;
  if (cls.DefType <> dtClass) or ((it <> nil) and not (it.DefType in ReplDefs)) then
    exit;
  with TClassDef(cls) do begin
    if not (HasReplacedItems or ImplementsReplacedItems) then
      exit;
    if ImplementsReplacedItems and not HasReplacedItems then
      exit;
    if it <> nil then
      with TReplDef(it) do begin
        if (it.DefType = dtProc) and (TProcDef(it).ProcType = ptConstructor) then
          exit;
        if IsReplaced or IsReplImpl then
          exit;
      end;
  end;
  Result:='__' + Result;
end;

procedure TWriter.RegisterPseudoClass(d: TDef);
var
  ci: TClassInfo;
begin
  if FClasses.IndexOf(d.Name, d) < 0 then begin
    ci:=TClassInfo.Create;
    ci.Def:=d;
    FClasses.Add(d.Name, d, ci);
  end;
end;

function TWriter.GetPasIntType(Size: integer): string;
begin
  case Size of
    1: Result:='byte';
    2: Result:='word';
    else
       Result:='cardinal';
  end;
end;

function TWriter.GetPasType(d: TDef; FullName: boolean): string;
begin
  Result:=d.Name;
  if FullName and (d.DefType <> dtType) then
    Result:=d.Parent.Name + '.' + Result;
end;

function TWriter.AddCustomProc(ParentDef: TDef; const JniName, Name: string; RetType: TBasicType; const Params: array of TBasicType): TProcDef;
var
  i: integer;
  vd: TVarDef;
begin
  Result:=TProcDef.Create(ParentDef, dtProc);
  Result.Name:=JniName;
  Result.AliasName:=Name;
  if RetType = btVoid then
    Result.ProcType:=ptProcedure
  else
    Result.ProcType:=ptFunction;
  for i:=0 to High(Params) do begin
    vd:=TVarDef.Create(Result, dtParam);
    vd.Name:=Format('p%d', [i + 1]);
    vd.VarType:=TTypeDef.Create(vd, dtType);
    TTypeDef(vd.VarType).BasicType:=Params[i];
  end;
  Result.ReturnType:=TTypeDef.Create(ParentDef, dtType);
  TTypeDef(Result.ReturnType).BasicType:=RetType;
end;

procedure TWriter.AddNativeMethod(ParentDef: TDef; const JniName, Name, Signature: string);
var
  i: integer;
  ci: TClassInfo;
  pi: TProcInfo;
begin
  pi:=TProcInfo.Create;
  pi.Name:=Name;
  pi.JniName:=JniName;
  pi.JniSignature:=Signature;
  i:=FClasses.IndexOf(ParentDef.AliasName, ParentDef);
  if i < 0 then begin
    ci:=TClassInfo.Create;
    ci.Def:=ParentDef;
    i:=FClasses.Add(ParentDef.AliasName, ParentDef, ci);
  end;
  FClasses.GetClassInfo(i).Funcs.Add(pi);
end;

function TWriter.GetProcSignature(d: TProcDef): string;
var
  j: integer;
  vd: TVarDef;
begin
  Result:='(';
  for j:=0 to d.Count - 1 do begin
    vd:=TVarDef(d[j]);
    if vd.DefType <> dtParam then
      continue;
    with vd do begin
      if (VarType <> nil) and (VarType.DefType = dtJniEnv) then
        continue;
      if IsJavaVarParam(vd) then
        Result:=Result + '[';
      Result:=Result + DefToJniSig(VarType);
    end;
  end;
  Result:=Result + ')' + DefToJniSig(d.ReturnType);
end;

procedure TWriter.EHandlerStart;
begin
  Fps.WriteLn('try');
  Fps.IncI;
end;

procedure TWriter.EHandlerEnd(const EnvVarName: string; const ExtraCode: string);
begin
  Fps.WriteLn('except', -1);
  Fps.WriteLn(Format('_HandleJNIException(%s);', [EnvVarName]));
  if ExtraCode <> '' then
    Fps.WriteLn(ExtraCode);
  Fps.DecI;
  Fps.WriteLn('end;');
end;

procedure TWriter.UpdateUsedUnits(u: TUnitDef);

  procedure _CheckDef(d: TDef);
  begin
    if (d = nil) or not d.IsUsed then
      exit;
    d:=d.Parent;
    if (d <> nil) and (d.DefType = dtUnit) then
      with TUnitDef(d) do
        if not IsUnitUsed and IsUsed then
          IsUnitUsed:=True;
  end;

  procedure _ScanDef(def: TDef);
  var
    i: integer;
    d: TDef;
  begin
    for i:=0 to def.Count - 1 do begin
      d:=def[i];
      if not d.IsUsed then
        continue;
      _CheckDef(d.GetRefDef);
      _CheckDef(d.GetRefDef2);
      _ScanDef(d);
    end;
  end;

var
  i: integer;
begin
  for i:=0 to High(u.UsedUnits) do
    u.UsedUnits[i].IsUnitUsed:=False;
  _ScanDef(u);
end;

procedure TWriter.WriteClassInfoVar(d: TDef);
begin
  Fps.WriteLn;
  Fps.WriteLn(Format('var %s: _TJavaClassInfo;', [GetTypeInfoVar(d)]));
end;

procedure TWriter.WriteComment(d: TDef; const AType: string);
begin
  Fps.WriteLn;
  Fps.WriteLn(Format('{ %s }', [d.Name]));

  Fjs.WriteLn(Format('/* %s */', [Trim(AType + ' ' + d.Name)]));
{$ifdef DEBUG}
  Fjs.WriteLn(Format('/* Ref count: %d */', [d.RefCnt]));
{$endif}
end;

{
procedure TWriter.AddCustomProc(ParentDef: TDef; const JniName, Name: string; RetType: TBasicType; const Params: array of TBasicType);
var
  i: integer;
  ci: TClassInfo;
  pi: TProcInfo;
begin
  pi:=TProcInfo.Create;
  pi.Name:=Name;
  pi.JniName:=JniName;
  pi.JniSignature:='(';
  for i:=0 to High(Params) do
    pi.JniSignature:=pi.JniSignature + JNITypeSig[Params[i]];
  pi.JniSignature:=pi.JniSignature + ')';
  pi.JniSignature:=pi.JniSignature + JNITypeSig[RetType];

  i:=FClasses.IndexOf(ParentDef.Name);
  if i < 0 then begin
    ci:=TClassInfo.Create;
    ci.Def:=ParentDef;
    i:=FClasses.AddObject(ParentDef.Name, ci);
  end;
  TClassInfo(FClasses.Objects[i]).Funcs.Add(pi);
end;
}
constructor TWriter.Create;
begin
  Units:=TStringList.Create;
  FClasses:=TClassList.Create;
  JavaPackage:='pas';
  IncludeList:=TStringList.Create;
  IncludeList.Duplicates:=dupIgnore;
  ExcludeList:=TStringList.Create;
  ExcludeList.Duplicates:=dupIgnore;
  FThisUnit:=TUnitDef.Create(nil, dtUnit);
  FRecords:=TObjectList.Create(False);
  FRealClasses:=TObjectList.Create(False);
  LibAutoLoad:=True;
end;

function DoCanUseDef(def, refdef: TDef): boolean;
begin
  Result:=True;
  if (def.DefType = dtArray) and (refdef is TVarDef) then begin
    // Arrays are supported only for variables, fields, properties and constants
    Result:=refdef.DefType in [dtVar, dtProp, dtField, dtConst];
  end;
end;

destructor TWriter.Destroy;
var
  i: integer;
begin
  for i:=0 to FClasses.Count - 1 do
    FClasses.Objects[i].Free;
  FClasses.Free;
  Units.Free;
  IncludeList.Free;
  ExcludeList.Free;
  FThisUnit.Free;
  FRecords.Free;
  FRealClasses.Free;
  inherited Destroy;
end;

procedure TWriter.ProcessUnits;

  procedure _ExcludeClasses(u: TDef; AAncestorClass: TClassDef);
  var
    i: integer;
    d: TDef;
    s: string;
    excl: boolean;
  begin
    for i:=0 to u.Count - 1 do begin
      d:=u[i];
      if d.DefType = dtClass then begin
        s:=u.Name + '.' + d.Name;
        if AAncestorClass = nil then begin
          excl:=DoCheckItem(s) = crExclude;
          if not excl and (TClassDef(d).AncestorClass <> nil) then
            with TClassDef(d).AncestorClass do
              excl:=DoCheckItem(Parent.Name + '.' + Name) = crExclude;
        end
        else
          excl:=TClassDef(d).AncestorClass = AAncestorClass;

        if excl then begin
          d.SetNotUsed;
          ExcludeList.Add(s);
          _ExcludeClasses(u, TClassDef(d));
        end;
      end;
    end;
  end;

var
  p: TPPUParser;
  i: integer;
  s, ss: string;
  d: TDef;
begin
  if Units.Count = 0 then
    raise Exception.Create('No unit name specified.');
  if (OutPath <> '') and not DirectoryExists(OutPath) then
    raise Exception.CreateFmt('Output path "%s" does not exist.', [OutPath]);
  if (JavaOutPath <> '') and not DirectoryExists(JavaOutPath) then
    raise Exception.CreateFmt('Output path "%s" does not exist.', [JavaOutPath]);
  if LibName = '' then
    LibName:=AnsiLowerCase(ChangeFileExt(Units[0], '')) + 'jni';

  for i:=0 to IncludeList.Count - 1 do
    IncludeList[i]:=Trim(IncludeList[i]);
  IncludeList.Sorted:=True;
  for i:=0 to ExcludeList.Count - 1 do
    ExcludeList[i]:=Trim(ExcludeList[i]);
  ExcludeList.Sorted:=True;

  for i:=Low(ExcludeStd) to High(ExcludeStd) do
    if IncludeList.IndexOf(ExcludeStd[i]) < 0 then
      ExcludeList.Add(ExcludeStd[i]);
  for i:=Low(ExcludeDelphi7) to High(ExcludeDelphi7) do
    if IncludeList.IndexOf(ExcludeDelphi7[i]) < 0 then
      ExcludeList.Add(ExcludeDelphi7[i]);

  FThisUnit.Name:=LibName;
  FThisUnit.AliasName:='system';

  p:=TPPUParser.Create(SearchPath);
  try
    p.OnCheckItem:=@DoCheckItem;
    OnCanUseDef:=@DoCanUseDef;
    for i:=0 to Units.Count - 1 do
      IncludeList.Add(ChangeFileExt(ExtractFileName(Units[i]), ''));
    for i:=0 to Units.Count - 1 do
      p.Parse(ChangeFileExt(ExtractFileName(Units[i]), ''));

    if OutPath <> '' then
      OutPath:=IncludeTrailingPathDelimiter(OutPath);
    if JavaOutPath <> '' then
      JavaOutPath:=IncludeTrailingPathDelimiter(JavaOutPath);

    FPkgDir:=JavaOutPath + StringReplace(JavaPackage, '.', DirectorySeparator, [rfReplaceAll]);
    ForceDirectories(FPkgDir);
    Fps:=TTextOutStream.Create(OutPath + LibName + '.pas', fmCreate);

    WriteFileComment(Fps);
    Fps.WriteLn('library '+ LibName + ';');
    Fps.WriteLn('{$ifdef fpc} {$mode objfpc} {$H+} {$endif}');

    Fps.WriteLn;
    Fps.WriteLn('uses');
    Fps.WriteLn('{$ifdef unix} cthreads, {$endif}', 1);
    s:='';
    for i:=0 to p.Units.Count - 1 do begin
      ProcessRules(p.Units[i]);
      ss:=LowerCase(p.Units[i].Name);
      if (ss ='system') or (ss = 'objpas') or (ss = 'sysutils') or (ss = 'syncobjs') or (ss = 'jni')
         or (ss = 'cthreads') or (ss = 'windows')
      then
        continue;
      if s <> '' then
        s:=s + ', ';
      if Length(s) >= 100 then begin
        Fps.WriteLn(s, 1);
        s:='';
      end;
      s:=s + p.Units[i].Name;
    end;
    if s <> '' then
      Fps.WriteLn(s + ',', 1);
    Fps.WriteLn('{$ifndef FPC} Windows, {$endif} SysUtils, SyncObjs, jni;', 1);

    // Types
    Fps.WriteLn;
    Fps.WriteLn('type');
    Fps.IncI;
    Fps.WriteLn('_JNIString = {$ifdef FPC} unicodestring {$else} widestring {$endif};');
    Fps.WriteLn('{$ifndef FPC} ptruint = cardinal; {$endif}');
    Fps.WriteLn;
    Fps.WriteLn('_TJavaClassInfo = record');
    Fps.WriteLn('ClassRef: JClass;', 1);
    Fps.WriteLn('ConstrId: JMethodId;', 1);
    Fps.WriteLn('ObjFieldId: JFieldId;', 1);
    Fps.WriteLn('end;');
    Fps.WriteLn('_PJavaClassInfo = ^_TJavaClassInfo;');
    Fps.DecI;

    Fps.WriteLn;
    d:=TtypeDef.Create(nil, dtType);
    TtypeDef(d).BasicType:=btString;
    Fps.WriteLn(Format('var %s: _TJavaClassInfo;', [GetTypeInfoVar(d)]));
    d.Free;

    // Support functions
    Fps.WriteLn;
    Fps.WriteLn('function _StringFromJString(env: PJNIEnv; s: jstring): _JNIString;');
    Fps.WriteLn('var');
    Fps.WriteLn('p: PJChar;', 1);
    Fps.WriteLn('c: JBoolean;', 1);
    Fps.WriteLn('begin');
    Fps.WriteLn('if s = nil then begin', 1);
    Fps.WriteLn('Result:='''';', 2);
    Fps.WriteLn('exit;', 2);
    Fps.WriteLn('end;', 1);
    Fps.WriteLn('p:=env^^.GetStringChars(env, s, c);', 1);
    Fps.WriteLn('SetString(Result, PWideChar(p), env^^.GetStringLength(env, s));', 1);
    Fps.WriteLn('env^^.ReleaseStringChars(env, s, p);', 1);
    Fps.WriteLn('end;');

    Fps.WriteLn;
    Fps.WriteLn('function _StringToJString(env: PJNIEnv; const s: _JNIString): jstring;');
    Fps.WriteLn('begin');
    Fps.WriteLn('Result:=env^^.NewString(env, PJChar(PWideChar(s)), Length(s));', 1);
    Fps.WriteLn('end;');

    Fps.WriteLn;
    Fps.WriteLn('function _CreateJavaObj(env: PJNIEnv; PasObj: jlong; const ci: _TJavaClassInfo; cleanup: boolean = True): jobject; overload;');
    Fps.WriteLn('var v: array [0..1] of jvalue;');
    Fps.WriteLn('begin');
    Fps.IncI;
    Fps.WriteLn('Result:=nil;');
    Fps.WriteLn('if PasObj = 0 then exit;');
    Fps.WriteLn('v[0].J:=PasObj;');
    Fps.WriteLn('if ci.ConstrId = nil then begin');
    Fps.WriteLn('Result:=env^^.AllocObject(env, ci.ClassRef);', 1);
    Fps.WriteLn('if Result = nil then exit;', 1);
    Fps.WriteLn('env^^.SetLongField(env, Result, ci.ObjFieldId, v[0].J);', 1);
    Fps.WriteLn('end else begin');
    Fps.WriteLn('v[1].Z:=byte(cleanup) and 1;', 1);
    Fps.WriteLn('Result:=env^^.NewObjectA(env, ci.ClassRef, ci.ConstrId, @v);', 1);
    Fps.WriteLn('end;');
    Fps.DecI;
    Fps.WriteLn('end;');
    Fps.WriteLn;
    Fps.WriteLn('function _CreateJavaObj(env: PJNIEnv; PasObj: pointer; const ci: _TJavaClassInfo; cleanup: boolean = True): jobject; overload;');
    Fps.WriteLn('begin');
    Fps.WriteLn('Result:=_CreateJavaObj(env, jlong(ptruint(PasObj)), ci, cleanup)', 1);
    Fps.WriteLn('end;');

    Fps.WriteLn;
    Fps.WriteLn('function _GetPasObj(env: PJNIEnv; jobj: jobject; const ci: _TJavaClassInfo; CheckNil: boolean): pointer;');
    Fps.WriteLn('var pasobj: jlong;');
    Fps.WriteLn('begin');
    Fps.IncI;
    Fps.WriteLn('if jobj <> nil then');
    Fps.WriteLn('pasobj:=env^^.GetLongField(env, jobj, ci.ObjFieldId)', 1);
    Fps.WriteLn('else');
    Fps.WriteLn('pasobj:=0;', 1);
    Fps.WriteLn('if CheckNil and (pasobj <= 0) then');
    Fps.WriteLn('raise Exception.Create(''Attempt to access a released Pascal object.'');', 1);
    Fps.WriteLn('Result:=pointer(ptruint(pasobj));');
    Fps.DecI;
    Fps.WriteLn('end;');

    Fps.WriteLn;
    Fps.WriteLn('function _GetClass(env: PJNIEnv; jobj: jobject; const ci: _TJavaClassInfo): TClass;');
    Fps.WriteLn('var pasobj: jlong;');
    Fps.WriteLn('begin');
    Fps.IncI;
    Fps.WriteLn('if jobj <> nil then');
    Fps.WriteLn('pasobj:=env^^.GetLongField(env, jobj, ci.ObjFieldId)', 1);
    Fps.WriteLn('else');
    Fps.WriteLn('pasobj:=0;', 1);
    Fps.WriteLn('if pasobj > 0 then');
    Fps.WriteLn('Result:=TObject(ptruint(pasobj)).ClassType', 1);
    Fps.WriteLn('else');
    Fps.WriteLn('Result:=TClass(ptruint(-pasobj));', 1);
    Fps.DecI;
    Fps.WriteLn('end;');

    Fps.WriteLn;
    Fps.WriteLn('procedure _HandleJNIException(env: PJNIEnv);');
    Fps.WriteLn('begin');
    Fps.WriteLn('if env^^.ExceptionCheck(env) <> 0 then exit;', 1);
    if p.OnExceptionProc <> nil then begin
      Fps.WriteLn(Format('%s.%s;', [p.OnExceptionProc.Parent.Name, p.OnExceptionProc.Name]), 1);
      p.OnExceptionProc.SetNotUsed;
    end;
    Fps.WriteLn('env^^.ThrowNew(env, env^^.FindClass(env, ''java/lang/Exception''), PAnsiChar(Utf8Encode(Exception(ExceptObject).Message)));', 1);
    Fps.WriteLn('end;');

    Fps.WriteLn;
    Fps.WriteLn('procedure _HandleJavaException(env: PJNIEnv);');
    Fps.WriteLn('begin');
    Fps.WriteLn('if env^^.ExceptionCheck(env) <> 0 then raise Exception.Create(''Java exception.'');', 1);
    Fps.WriteLn('end;');

    Fps.WriteLn;
    Fps.WriteLn('procedure _RaiseVarParamException(const VarName: string);');
    Fps.WriteLn('begin');
    Fps.WriteLn('raise Exception.CreateFmt(''An array with only single element must be passed as parameter "%s".'', [VarName]);', 1);
    Fps.WriteLn('end;');

    // Public support functions

    Fps.WriteLn;
    Fps.WriteLn('function _AllocMemory(env: PJNIEnv; jobj: jobject; size: jint): jlong;' + JniCaliing);
    Fps.WriteLn('var p: pointer;');
    Fps.WriteLn('begin');
    Fps.WriteLn('GetMem(p, size);', 1);
    Fps.WriteLn('FillChar(p^, size, 0);', 1);
    Fps.WriteLn('Result:=ptruint(p);', 1);
    Fps.WriteLn('end;');

    Fps.WriteLn;
    Fps.WriteLn('function _GetMemoryAsArray(env: PJNIEnv; jobj: jobject; SrcBuf: jlong; BufSize: jint): jarray;' + JniCaliing);
    Fps.WriteLn('begin');
    Fps.WriteLn('Result:=env^^.NewByteArray(env, BufSize);', 1);
    Fps.WriteLn('env^^.SetByteArrayRegion(env, Result, 0, BufSize, pointer(ptruint(SrcBuf)));', 1);
    Fps.WriteLn('end;');

    Fps.WriteLn;
    Fps.WriteLn('procedure _SetMemoryFromArray(env: PJNIEnv; jobj: jobject; DstBuf: jlong; SrcArray: jarray);' + JniCaliing);
    Fps.WriteLn('begin');
    Fps.WriteLn('env^^.GetByteArrayRegion(env, SrcArray, 0, env^^.GetArrayLength(env, SrcArray), pointer(ptruint(DstBuf)));', 1);
    Fps.WriteLn('end;');

    // Set support
    Fps.WriteLn;
    Fps.WriteLn('function _GetIntObjValue(env: PJNIEnv; jobj: jobject; const ci: _TJavaClassInfo): longint;');
    Fps.WriteLn('begin');
    Fps.IncI;
    Fps.WriteLn('if jobj = nil then raise Exception.Create(''Attempt to access a NULL set.'');');
    Fps.WriteLn('Result:=env^^.GetIntField(env, jobj, ci.ObjFieldId);');
    Fps.DecI;
    Fps.WriteLn('end;');
    Fps.WriteLn;
    Fps.WriteLn('function _CreateIntObj(env: PJNIEnv; Value: longint; const ci: _TJavaClassInfo): jobject;');
    Fps.WriteLn('begin');
    Fps.IncI;
    Fps.WriteLn('Result:=nil;');
    Fps.WriteLn('Result:=env^^.AllocObject(env, ci.ClassRef);');
    Fps.WriteLn('if Result = nil then exit;');
    Fps.WriteLn('env^^.SetIntField(env, Result, ci.ObjFieldId, Value);');
    Fps.DecI;
    Fps.WriteLn('end;');

    // Preprocess units
    for i:=0 to p.Units.Count - 1 do begin
      if AnsiCompareText(p.Units[i].Name, 'system') <> 0 then
        _ExcludeClasses(p.Units[i], nil);
    end;

    // Write units
    for i:=0 to p.Units.Count - 1 do
      with TUnitDef(p.Units[i]) do begin
        WriteUnit(TUnitDef(p.Units[i]));
      end;

    WriteRecordSizes;
    WriteClassTable;

    WriteOnLoad;

    Fps.WriteLn;
    Fps.WriteLn('procedure ___doexit;');
    Fps.WriteLn('begin');
    Fps.WriteLn('_MethodPointersCS.Free;', 1);
    Fps.WriteLn('end;');
    Fps.WriteLn;
    Fps.WriteLn('begin');
    Fps.WriteLn('ExitProc:=@___doexit;', 1);
    Fps.WriteLn('IsMultiThread:=True;', 1);
    Fps.WriteLn('_MethodPointersCS:=TCriticalSection.Create;', 1);
    Fps.WriteLn('end.');
  finally
    Fps.Free;
    p.Free;
  end;
end;

end.

