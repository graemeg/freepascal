{
    $Id: header,v 1.1 2000/07/13 06:33:45 michael Exp $
    This file is part of the Free Component Library (FCL)
    Copyright (c) 1999-2000 by the Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{$mode objfpc}
{$H+}
{$define NOCONTNRS}
unit fpTemplate;


interface

uses
  SysUtils,
  Classes;

Const
  DefaultParseDepth = 100;
  MaxDelimLength    = 5;
  
Type
  TParseDelimiter = String[MaxDelimLength];
  
Var
  DefaultStartDelimiter : TParseDelimiter = '{';           //Template tag start                  |If you want Delphi-like, set it to '<#'
  DefaultEndDelimiter  : TParseDelimiter = '}';            //Template tag end                    |                                   '>'
  DefaultParamStartDelimiter  : TParseDelimiter = '[-';    //Tag parameter start                 |                                   ' '
  DefaultParamEndDelimiter    : TParseDelimiter = '-]';    //Tag parameter end                   |                                   '"'
  DefaultParamValueSeparator  : TParseDelimiter = '=';     //Tag parameter name/value separator  |                                   '="'
                                                           //                                    |for tags like <#TagName paramname1="paramvalue1" paramname2="paramvalue2">

Type
  TGetParamEvent = Procedure(Sender : TObject; Const ParamName : String; Out AValue : String) Of Object;                              //for simple template tag support only (ex: {Name})
  TReplaceTagEvent = Procedure(Sender : TObject; Const TagString : String; TagParams:TStringList; Out ReplaceText : String) Of Object;//for tags with parameters support


  { TTemplateParser }

  TTemplateParser = Class(TObject)
  Private
    FParseLevel : Integer;
    FMaxParseDepth : Integer;
    FEndDelimiter: TParseDelimiter;
    FStartDelimiter: TParseDelimiter;
    FParamStartDelimiter: TParseDelimiter;
    FParamEndDelimiter: TParseDelimiter;
    FParamValueSeparator: TParseDelimiter;
    FAllowTagParams: Boolean; //default is false -> simple template tags allowed only [FValues, FOnGetParam (optional) used];
                              //if true -> template tags with parameters allowed, [FOnReplaceTag] is used for all tag replacements
    FRecursive: Boolean;                                   //when only simple tags are used in a template (AllowTagParams=false), the replacement can
    FValues : TStringList;                                 //contain further tags for recursive processing (only used when no tag params are allowed)
    FOnGetParam: TGetParamEvent;                           //Event handler to use for templates containing simple tags only (ex: {Name})
    FOnReplaceTag: TReplaceTagEvent;                       //Event handler to use for templates containing tags with parameters (ex: <#TagName paramname1="paramvalue1" paramname2="paramvalue2">)
    function GetDelimiter(Index: integer): TParseDelimiter;
    function GetValue(Key : String): String;
    procedure SetDelimiter(Index: integer; const AValue: TParseDelimiter);
    procedure SetValue(Key : String; const AValue: String);
  Public
    Constructor Create;
    Destructor Destroy; override;
    Procedure Clear;
    Function ReplaceTag(const Key: String; TagParams:TStringList; out ReplaceWith: String): Boolean;//used only when AllowTagParams = true
    Function GetParam(Const Key : String; Out AValue : String) : Boolean;                           //used only when AllowTagParams = false
    Procedure GetTagParams(var TagName:String; var TagParams : TStringList) ;
    Function ParseString(Src : String) : String;
    Function ParseStream(Src : TStream; Dest : TStream) : Integer; // Wrapper, Returns number of bytes written.
    Procedure ParseStrings(Src : TStrings; Dest : TStrings) ;      // Wrapper
    Property OnGetParam : TGetParamEvent Read FOnGetParam Write FOnGetParam;               // Called if not found in values  //used only when AllowTagParams = false
    Property OnReplaceTag : TReplaceTagEvent Read FOnReplaceTag Write FOnReplaceTag;       // Called if a tag found          //used only when AllowTagParams = true
    Property StartDelimiter : TParseDelimiter Index 1 Read GetDelimiter Write SetDelimiter;// Start char/string, default '}'
    Property EndDelimiter : TParseDelimiter Index 2 Read GetDelimiter Write SetDelimiter;  // end char/string, default '{'
    Property ParamStartDelimiter : TParseDelimiter Index 3 Read GetDelimiter Write SetDelimiter;
    Property ParamEndDelimiter : TParseDelimiter Index 4 Read GetDelimiter Write SetDelimiter;
    Property ParamValueSeparator : TParseDelimiter Index 5 Read GetDelimiter Write SetDelimiter;
    Property Values[Key : String] : String Read GetValue Write SetValue; // Contains static values.                          //used only when AllowTagParams = false
    Property Recursive : Boolean Read FRecursive Write FRecursive;                                                           //used only when AllowTagParams = false
    Property AllowTagParams : Boolean Read FAllowTagParams Write FAllowTagParams;
  end;

  { TFPCustomTemplate }

  TFPCustomTemplate = Class(TPersistent)
  private
    FEndDelimiter: TParseDelimiter;
    FStartDelimiter: TParseDelimiter;
    FParamStartDelimiter: TParseDelimiter;
    FParamEndDelimiter: TParseDelimiter;
    FParamValueSeparator: TParseDelimiter;
    FFileName: String;
    FTemplate: String;
    FOnGetParam: TGetParamEvent;                                                                                             //used only when AllowTagParams = false
    FOnReplaceTag: TReplaceTagEvent;                                                                                         //used only when AllowTagParams = true
    FAllowTagParams: Boolean;
  Protected
    Procedure GetParam(Sender : TObject; Const ParamName : String; Out AValue : String);virtual;                             //used only when AllowTagParams = false
    Procedure ReplaceTag(Sender : TObject; Const TagName: String; TagParams:TStringList; Out AValue: String);virtual;        //used only when AllowTagParams = true
    Function CreateParser : TTemplateParser; virtual;
  Public
    Function HasContent : Boolean;
    Function GetContent : String;
    Procedure Assign(Source : TPersistent); override;
    Property StartDelimiter : TParseDelimiter Read FStartDelimiter Write FStartDelimiter;
    Property EndDelimiter : TParseDelimiter Read FEndDelimiter Write FEndDelimiter;
    Property ParamStartDelimiter : TParseDelimiter Read FParamStartDelimiter Write FParamStartDelimiter;
    Property ParamEndDelimiter : TParseDelimiter Read FParamEndDelimiter Write FParamEndDelimiter;
    Property ParamValueSeparator : TParseDelimiter Read FParamValueSeparator Write FParamValueSeparator;
    Property FileName : String Read FFileName Write FFileName;
    Property Template : String Read FTemplate Write FTemplate;
    Property OnGetParam : TGetParamEvent Read FOnGetParam Write FOnGetParam;
    Property OnReplaceTag : TReplaceTagEvent Read FOnReplaceTag Write FOnReplaceTag;
    Property AllowTagParams : Boolean Read FAllowTagParams Write FAllowTagParams;
  end;
  
  TFPTemplate = Class(TFPCustomTemplate)
  Published
    Property FileName;
    Property Template;
//    Property AllowTagParams;
//    Property OnReplaceTag;
  end;
  
  ETemplateParser = Class(Exception);

Var
  MaxParseDepth : Integer = DefaultParseDepth;


implementation

Resourcestring
  SErrParseDepthExceeded = 'Maximum parse level (%d) exceeded.';
  SErrNoEmptyDelimiters = 'Delimiters cannot be empty';
  
{ TTemplateParser }
Type

  { TStringItem }

  TStringItem = Class(TObject)
  Private
    FValue : String;
  Public
    Constructor Create(AValue : String);
    Property Value : String Read FValue Write FValue;
  end;

{ TStringItem }

constructor TStringItem.Create(AValue: String);
begin
  FValue:=AValue;
end;

{ TTemplateParser }

function TTemplateParser.GetValue(Key : String): String;

Var
  I : Integer;

begin
  Result:='';
  If Assigned(FValues) then
    begin
    I:=FValues.IndexOf(Key);
    If (I<>-1) then
      Result:=TStringItem(FValues.Objects[i]).Value;
    end;
end;

function TTemplateParser.GetDelimiter(Index: integer): TParseDelimiter;
begin
  case Index of
  1: Result:=FStartDelimiter;
  2: Result:=FEndDelimiter;
  3: Result:=FParamStartDelimiter;
  4: Result:=FParamEndDelimiter;
    else
     Result:=FParamValueSeparator;
  end;
end;

procedure TTemplateParser.SetDelimiter(Index: integer;
  const AValue: TParseDelimiter);
begin
  If Length(AValue)=0 then
    Raise ETemplateParser.Create(SErrNoEmptyDelimiters);
  case Index of
    1: FStartDelimiter:=AValue;
    2: FEndDelimiter:=AValue;
    3: FParamStartDelimiter:=AValue;
    4: FParamEndDelimiter:=AValue;
      else
       FParamValueSeparator:=AValue;
  end;

end;

procedure TTemplateParser.SetValue(Key : String; const AValue: String);

Var
  I : Integer;

begin
  If (AValue='') then
    begin
    If Assigned(FValues) then
      begin
      I:=FValues.IndexOf(Key);
      If (I<>-1) then
        begin
        FValues.Objects[i].Free;
        FValues.Delete(I);
        end;
      end;
    end
  else
    begin
    if Not Assigned(FValues) then
      begin
      FVAlues:=TStringList.Create;
      FValues.Sorted:=True;
      end;
    I:=FValues.IndexOf(Key);
    If (I=-1) then
      FValues.AddObject(Key,TStringItem.Create(AValue))
    else
      TStringItem(FValues.Objects[I]).Value:=AValue;
    end;
end;

constructor TTemplateParser.Create;

begin
  FParseLevel:=0;
  FMaxParseDepth:=MaxParseDepth;
  FStartDelimiter:=DefaultStartDelimiter;
  FEndDelimiter:=DefaultEndDelimiter;
  FParamStartDelimiter:=DefaultParamStartDelimiter;
  FParamEndDelimiter:=DefaultParamEndDelimiter;
  FParamValueSeparator:=DefaultParamValueSeparator;
  FAllowTagParams := false;
end;

destructor TTemplateParser.Destroy;

begin
  Clear;
  inherited Destroy;
end;

procedure TTemplateParser.Clear;

Var
  I : Integer;
  
begin
  If Assigned(FValues) then
    For I:=0 to FValues.Count-1 do
      FValues.Objects[i].Free;
  FreeAndNil(FValues);
end;

function TTemplateParser.GetParam(const Key: String; out AValue: String): Boolean;
  
Var
  I : Integer;
  
begin
  If Assigned(FValues) then
    I:=FValues.IndexOf(Key)
  else
    I:=-1;
  Result:=(I<>-1);
  If Result then
    AValue:=TStringItem(FValues.Objects[i]).Value
  else
    begin
    Result:=Assigned(FOnGetParam);
    If Result then
      FOnGetParam(Self,Key,AValue);
    end;
  If Result and Recursive then
    AValue:=ParseString(AValue);
end;

function TTemplateParser.ReplaceTag(const Key: String; TagParams:TStringList; out ReplaceWith: String): Boolean;
begin
  Result:=Assigned(FOnReplaceTag);
  If Result then
    FOnReplaceTag(Self,Key,TagParams,ReplaceWith);
end;

Function FindDelimiter(SP : PChar; D : TParseDelimiter; MaxLen : Integer) : PChar; Inline;

Var
  P,P2 : PChar;
  I,DLen : Integer;

begin
  Result:=Nil;
  DLen:=Length(D);
  Dec(MaxLen,(DLen-1));
  If MaxLen<=0 then
   exit;
  P:=SP;
  While (Result=Nil) and (P-SP<=MaxLen) do
    begin
    While (P-SP<=MaxLen) and (P^<>D[1]) do
      Inc(P);
    If ((P-SP)<=MaxLen) then
      begin
      Result:=P;
      P2:=P+1;
      // Check Other characters
      I:=2;
      While (I<=DLen) and (Result<>Nil) do
        If (P2^=D[i]) then
          begin
          inc(i);
          Inc(p2);
          end
        else
          begin
          P:=Result;
          Result:=Nil;
          end;
      // Either result<>Nil -> match or result=nil -> no match
      inc(P);
      end;
    end;
end;

Procedure AddToString(Var S : String; P : PChar; NChars : Integer);inline;

Var
  SLen : Integer;

begin
  SLen:=Length(S);
  SetLength(S,SLen+NChars);
  Move(P^,S[Slen+1],NChars);
end;

procedure TTemplateParser.GetTagParams(var TagName:String; var TagParams : TStringList) ;
var
  I,SLen:Integer;
  TS,TM,TE,SP,P : PChar;
  PName, PValue, TP : String;
  IsFirst:Boolean;
begin
  SLen:=Length(TagName);
  if SLen=0 then exit;

  IsFirst := true;
  SP:=PChar(TagName);
  TP := TagName;
  P:=SP;
  while (P-SP<SLen) do
  begin
    TS:=FindDelimiter(P,FParamStartDelimiter,SLen-(P-SP));
    if (TS<>Nil) then
    begin//Found param start delimiter
      if IsFirst then
      begin//Get the real Tag name
        IsFirst := false;
        I := 1;
        while not (P[I] in [#0..' ']) do Inc(I);
        SetLength(TP, I);
        Move(P^, TP[1], I);
      end;
      inc(TS, Length(FParamStartDelimiter));
      I:=TS-P;//index of param name
      TM:=FindDelimiter(TS,FParamValueSeparator,SLen-I+1);
      if (TM<>Nil) then
      begin//Found param value separator
        I:=TM-TS;//lenght of param name
        SetLength(PName, I);
        Move(TS^, PName[1], I);//param name
        inc(TS, Length(FParamValueSeparator) + I);
        I := TS - P;//index of param value
        TE:=FindDelimiter(TS,FParamEndDelimiter, SLen-I+1);
        if (TE<>Nil) then
        begin//Found param end
          I:=TE-TS;//Param length
          Setlength(PValue,I);
          Move(TS^,PValue[1],I);//Param value
          TagParams.Add(Trim(PName) + '=' + PValue);//Param names cannot contain '='
          P:=TE+Length(FParamEndDelimiter);
          TS:=P;
        end else break;
      end else break;
    end else break;
  end;
  TagName := Trim(TP);
end;

function TTemplateParser.ParseString(Src: String): String;

Var
  PN,PV,ReplaceWith : String;
  i,SLen : Integer;
  TS,TE,SP,P : PChar;
  TagParams:TStringList;
begin
  if FAllowTagParams then
  begin//template tags with parameters are allowed
    SLen:=Length(Src);
    If SLen=0 then
      exit;
    Result:='';
    SP:=PChar(Src);
    P:=SP;
    While (P-SP<SLen) do
      begin
      TS:=FindDelimiter(P,FStartDelimiter,SLen-(P-SP));
      If (TS=Nil) then
        begin//Tag Start Delimiter not found
        TS:=P;
        P:=SP+SLen;
        end
      else
        begin
        I:=TS-P;
        TE:=FindDelimiter(TS,FEndDelimiter,SLen-I+1);
        If (TE=Nil) then
          begin//Tag End Delimiter not found
          TS:=P;
          P:=SP+SLen;
          end
        else//Found start and end delimiters for the Tag
          begin
          // Add text prior to template tag to result
          AddToString(Result,P,I);
          // Retrieve the full template tag (only tag name if no params specified)
          inc(TS,Length(FStartDelimiter));//points to first char of Tag name now
          I:=TE-TS;//full Tag length
          Setlength(PN,I);
          Move(TS^,PN[1],I);//full Tag string (only tag name if no params specified)
          TagParams := TStringList.Create;
          try
            TagParams.Sorted := True;
            GetTagParams(PN, Tagparams);
            If ReplaceTag(PN,TagParams,ReplaceWith) then
              Result:=Result+ReplaceWith;
          finally
            TagParams.Free;
          end;
          P:=TE+Length(FEndDelimiter);
          TS:=P;
          end;
        end
      end;
    I:=P-TS;
    If (I>0) then
      AddToString(Result,TS,I);
  end else begin//template tags with parameters are not allowed
    Inc(FParseLevel);
    If FParseLevel>FMaxParseDepth then
      Raise ETemplateParser.CreateFmt(SErrParseDepthExceeded,[FMaxParseDepth]);
    SLen:=Length(Src); // Minimum
    If SLen=0 then
      exit;
//    STLen:=Length(FStartDelimiter);
    Result:='';
    SP:=PChar(Src);
    P:=SP;
    While (P-SP<SLen) do
      begin
      TS:=FindDelimiter(P,FStartDelimiter,SLen-(P-SP));
      If (TS=Nil) then
        begin
        TS:=P;
        P:=SP+SLen
        end
      else
        begin
        I:=TS-P;
        TE:=FindDelimiter(TS,FEndDelimiter,SLen-I+1);
        If (TE=Nil) then
          begin
          TS:=P;
          P:=SP+SLen;
          end
        else
          begin
          // Add text prior to template to result
          AddToString(Result,P,I);
          // retrieve template name
          inc(TS,Length(FStartDelimiter));
          I:=TE-TS;
          Setlength(PN,I);
          Move(TS^,PN[1],I);
          If GetParam(PN,PV) then
            begin
            Result:=Result+PV;
            end;
          P:=TE+Length(FEndDelimiter);
          TS:=P;
          end;
        end
      end;
    I:=P-TS;
    If (I>0) then
      AddToString(Result,TS,I);
  end;
end;

function TTemplateParser.ParseStream(Src: TStream; Dest: TStream): Integer;

Var
  SS : TStringStream;
  S,R : String;
  
begin
  SS:=TStringStream.Create('');
  Try
    SS.CopyFrom(Src,0);
    S:=SS.DataString;
  Finally
    SS.Free;
  end;
  FParseLevel := 0;
  R:=ParseString(S);
  Result:=Length(R);
  If (Result>0) then
    Dest.Write(R[1],Result);
end;

procedure TTemplateParser.ParseStrings(Src: TStrings; Dest: TStrings);

Var
  I : Integer;

begin
  For I:=0 to Src.Count-1 do
  begin
    FParseLevel := 0;
    Dest.Add(ParseString(Src[i]));
  end;
end;

{ TFPCustomTemplate }

procedure TFPCustomTemplate.GetParam(Sender: TObject; const ParamName: String; out AValue: String);
  
begin
  If Assigned(FOnGetParam) then
   FOnGetParam(Self,ParamName,AValue);
end;

procedure TFPCustomTemplate.ReplaceTag(Sender: TObject; const TagName: String; TagParams:TStringList; Out AValue: String);

begin
  If Assigned(FOnReplaceTag) then
  begin
    FOnReplaceTag(Self,TagName,TagParams,AValue);
  end;
end;

function TFPCustomTemplate.CreateParser: TTemplateParser;

begin
  Result:=TTemplateParser.Create;
  Result.FParseLevel := 0;
  If (FStartDelimiter<>'') then
    Result.StartDelimiter:=FStartDelimiter;
  If (FEndDelimiter<>'') then
    Result.EndDelimiter:=FEndDelimiter;
  If (FParamStartDelimiter<>'') then
    Result.ParamStartDelimiter:=FParamStartDelimiter;
  If (FParamEndDelimiter<>'') then
    Result.ParamEndDelimiter:=FParamEndDelimiter;
  If (FParamValueSeparator<>'') then
    Result.ParamValueSeparator:=FParamValueSeparator;
  Result.OnGetParam:=@GetParam;
  Result.OnReplaceTag:=@ReplaceTag;
  Result.AllowTagParams:=FAllowTagParams;
end;

function TFPCustomTemplate.HasContent: Boolean;

begin
  Result:=(FTemplate<>'') or (FFileName<>'');
end;

function TFPCustomTemplate.GetContent: String;

Var
  P : TTemplateParser;
  S : TStringStream;
  F : TFileStream;
  
begin
  F:=Nil;
  S:=Nil;
  If HasContent then
    begin
    if (FFileName<>'') then
      begin
      F:=TFileStream.Create(FFileName,fmOpenRead);
      S:=TStringStream.Create('');
      end;
    Try
      P:=CreateParser;
      Try
        If (F<>Nil) then
          begin
          P.ParseStream(F,S);
          Result:=S.DataString;
          end
        else
          Result:=P.ParseString(FTemplate);
      Finally
        P.Free;
      end;
    Finally
      F.Free;
      S.Free;
    end;
    end;
end;

procedure TFPCustomTemplate.Assign(Source: TPersistent);

Var
  T : TFPCustomTemplate;

begin
  If Source is TFPCustomTemplate then
    begin
    T:=Source as TFPCustomTemplate;
    FEndDelimiter:=T.EndDelimiter;
    FStartDelimiter:=T.StartDelimiter;
    FParamEndDelimiter:=T.ParamEndDelimiter;
    FParamStartDelimiter:=T.ParamStartDelimiter;
    FParamValueSeparator:=T.ParamValueSeparator;
    FFileName:=T.FileName;
    FTemplate:=T.Template;
    FOnGetParam:=T.OnGetParam;
    FOnReplaceTag:=T.OnReplaceTag;
    FAllowTagParams := T.AllowTagParams;
    end
  else
    inherited Assign(Source);
end;

end.

