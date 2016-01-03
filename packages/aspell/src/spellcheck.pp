unit SpellCheck;

{ Simple unit to simplify/OOP-ize pascal-style the aspell interface. Currently
  very limited, will be expanded eventually. Use like you wish. }

{$mode objfpc}{$H+}

interface

uses
  SysUtils, Classes, Aspell;

type
  TSuggestionArray = array of string;
  
  TWordError = record
    Word: string; // the word itself
    Pos: LongWord; // word position in line
    Length: LongWord; // word length
    Suggestions: TSuggestionArray; // suggestions for the given word
  end;
  
  TLineErrors = array of TWordError;
  TLineErrorsArray = array of TLineErrors;

<<<<<<< HEAD
<<<<<<< HEAD
  { TSpellCheck }
=======
  { TSpeller }
  { Abstract ancestor, don't use directly }
>>>>>>> graemeg/fixes_2_2
=======
  { TSpeller }
  { Abstract ancestor, don't use directly }
>>>>>>> origin/fixes_2_2

  TSpeller = class // abstract class, basis for all checkers
   protected
    FMode: string;
    FEncoding: string;
    FLanguage: string;
    procedure SetEncoding(const AValue: string);
    procedure SetLanguage(const AValue: string);
    procedure SetMode(const AValue: string);
    procedure CreateSpeller; virtual; abstract;
    procedure FreeSpeller; virtual; abstract;
   public
    constructor Create;
    destructor Destroy; override;
   public
    property Mode: string read FMode write SetMode;
    property Encoding: string read FEncoding write SetEncoding;
    property Language: string read FLanguage write SetLanguage;
  end;
<<<<<<< HEAD
<<<<<<< HEAD
  
  { TWordSpeller }

  TWordSpeller = class(TSpeller) // class for simple per-word checking
   private
    FSpeller: PAspellSpeller;
    FLastError: string;
    function DoCreateSpeller(Lang, Enc, aMode: pChar): PAspellSpeller;
=======
=======
>>>>>>> origin/fixes_2_2

  { TWordSpeller }
  { Basic spelling class for spelling single words without context }
  
  TWordSpeller = class(TSpeller) // class for simple per-word checking
   private
    FSpeller: PAspellSpeller;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
   protected
    procedure CreateSpeller; override;
    procedure FreeSpeller; override;
   public
    function SpellCheck(const Word: string): TSuggestionArray; // use to check single words, parsed out by you
  end;
  
  { TDocumentSpeller }
<<<<<<< HEAD
<<<<<<< HEAD
=======
  { This speller is used to spellcheck lines or even whole documents.
    It is usefull when different mode (like "tex") is used so you can pass
    everything to aspell and let it take care of the context }
>>>>>>> graemeg/fixes_2_2
=======
  { This speller is used to spellcheck lines or even whole documents.
    It is usefull when different mode (like "tex") is used so you can pass
    everything to aspell and let it take care of the context }
>>>>>>> origin/fixes_2_2

  TDocumentSpeller = class(TWordSpeller)
   private
    FChecker: PAspellDocumentChecker;
    FLineErrors: TLineErrorsArray;
    FNameSuggestions: Boolean;
    function GetLineErrors(i: Integer): TLineErrors;
    function GetLineErrorsCount: Integer;
   protected
    procedure CreateSpeller; override;
    procedure FreeSpeller; override;
    procedure DoNameSuggestions(const Word: string; var aWordError: TWordError);
   public
    constructor Create;
    function CheckLine(const aLine: string): TLineErrors;
    function CheckDocument(const FileName: string): Integer; // returns number of spelling errors found or -1 for error
    function CheckDocument(aStringList: TStringList): Integer; // returns number of spelling errors found or -1 for error
    procedure Reset;
   public
    property LineErrors[i: Integer]: TLineErrors read GetLineErrors;
    property LineErrorsCount: Integer read GetLineErrorsCount;
    property NameSuggestions: Boolean read FNameSuggestions write FNameSuggestions;
  end;

implementation

const
  DEFAULT_ENCODING = 'utf-8';
  DEFAULT_LANGUAGE = 'en';
  DEFAULT_MODE     = '';

function GetDefaultLanguage: string;
begin
  Result := GetEnvironmentVariable('LANG');
  if Length(Result) = 0 then
    Result := DEFAULT_LANGUAGE;
end;

{ TSpeller }

procedure TSpeller.SetEncoding(const AValue: string);
begin
  FEncoding := aValue;
  CreateSpeller;
end;

procedure TSpeller.SetLanguage(const AValue: string);
begin
  FLanguage := aValue;
  CreateSpeller;
end;

procedure TSpeller.SetMode(const AValue: string);
begin
  FMode := aValue;
  CreateSpeller;
end;

constructor TSpeller.Create;
begin
  FEncoding := DEFAULT_ENCODING;
  FLanguage := GetDefaultLanguage;
  FMode := DEFAULT_MODE;

  CreateSpeller;
end;

destructor TSpeller.Destroy;
begin
  FreeSpeller;
end;

{ TWordSpeller }

<<<<<<< HEAD
<<<<<<< HEAD
function TWordSpeller.DoCreateSpeller(Lang, Enc, aMode: pChar): PAspellSpeller;
var
  Error: Paspellcanhaveerror;
begin
  Result := new_aspell_config();

  if Length(FLanguage) > 0 then
    aspell_config_replace(Result, 'lang', Lang);
  if Length(FEncoding) > 0 then
    aspell_config_replace(Result, 'encoding', Enc);
  if Length(FMode) > 0 then
    aspell_config_replace(Result, 'mode', aMode);

  Error := new_aspell_speller(Result);

  delete_aspell_config(Result);

  if aspell_error_number(Error) <> 0 then begin
    FLastError := aspell_error_message(Error);
    delete_aspell_can_have_error(Error);
    Result := nil;
  end else
    Result := to_aspell_speller(Error);
end;

procedure TWordSpeller.CreateSpeller;
begin
  FLastError := '';
  FreeSpeller;

  FSpeller := DoCreateSpeller(pChar(FLanguage), pChar(FEncoding), pChar(FMode));
  if not Assigned(FSpeller) then
    FSpeller := DoCreateSpeller(nil, pChar(FEncoding), pChar(FMode));
  if not Assigned(FSpeller) then
    FSpeller := DoCreateSpeller(nil, pChar(FEncoding), nil);
  if not Assigned(FSpeller) then
    FSpeller := DoCreateSpeller(nil, nil, pChar(FMode));
  if not Assigned(FSpeller) then
    FSpeller := DoCreateSpeller(nil, nil, nil);

  if not Assigned(FSpeller) then
    raise Exception.Create('Error on speller creation: ' + FLastError);
=======
=======
>>>>>>> origin/fixes_2_2
procedure TWordSpeller.CreateSpeller;
var
  Config: Paspellconfig;
  Error: Paspellcanhaveerror;
begin
  Config := new_aspell_config();

  if Length(FLanguage) > 0 then
    aspell_config_replace(Config, 'lang', pChar(FLanguage));
  if Length(FEncoding) > 0 then
    aspell_config_replace(Config, 'encoding', pChar(FEncoding));
  if Length(FMode) > 0 then
    aspell_config_replace(Config, 'mode', pChar(FMode));

  Error := new_aspell_speller(Config);

  delete_aspell_config(Config);
  FreeSpeller;

  if aspell_error_number(Error) <> 0 then
    raise Exception.Create('Error on speller creation: ' + aspell_error_message(Error))
  else
    FSpeller := to_aspell_speller(Error);
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
end;

procedure TWordSpeller.FreeSpeller;
begin
  if Assigned(FSpeller) then begin
    delete_aspell_speller(FSpeller);
    FSpeller := nil;
  end;
end;

function TWordSpeller.SpellCheck(const Word: string): TSuggestionArray;
var
  sgs: Paspellwordlist;
  elm: Paspellstringenumeration;
  tmp: pChar;
  i: Integer = 0;
begin
  SetLength(Result, 0);

  if aspell_speller_check(FSpeller, pChar(Word), Length(Word)) = 0 then begin
    sgs := aspell_speller_suggest(FSpeller, pChar(Word), Length(Word));
    elm := aspell_word_list_elements(sgs);

    repeat
      if i >= Length(Result) then
        SetLength(Result, Length(Result) + 10);

      tmp := aspell_string_enumeration_next(elm);

      if tmp <> nil then begin
        Result[i] := tmp;
        Inc(i);
      end;
    until tmp = nil;

    SetLength(Result, i);

    delete_aspell_string_enumeration(elm);
  end;
end;

{ TDocumentSpeller }

function TDocumentSpeller.GetLineErrors(i: Integer): TLineErrors;
begin
  Result := FLineErrors[i];
end;

function TDocumentSpeller.GetLineErrorsCount: Integer;
begin
  Result := Length(FLineErrors);
end;

procedure TDocumentSpeller.CreateSpeller;
var
  Error: PAspellCanHaveError;
begin
  inherited CreateSpeller;
  
  Error := new_aspell_document_checker(FSpeller);

  if aspell_error_number(Error) <> 0 then
    raise Exception.Create('Error on checker creation: ' + aspell_error_message(Error))
  else
    FChecker := to_aspell_document_checker(Error);
end;

procedure TDocumentSpeller.FreeSpeller;
begin
  if Assigned(FChecker) then begin
    delete_aspell_document_checker(FChecker);
    FChecker := nil;
  end;

  inherited FreeSpeller;
end;

procedure TDocumentSpeller.DoNameSuggestions(const Word: string;
  var aWordError: TWordError);
begin
  aWordError.Suggestions := SpellCheck(Word);
end;

constructor TDocumentSpeller.Create;
begin
  inherited Create;
  
  FNameSuggestions := True;
end;

function TDocumentSpeller.CheckLine(const aLine: string): TLineErrors;
const
  CHUNK_SIZE = 10;
var
  i, Count: Integer;
  Token: AspellToken;
begin
  aspell_document_checker_process(FChecker, pChar(aLine), Length(aLine));

  SetLength(Result, CHUNK_SIZE);
  i := 0;
  Count := 0;
  repeat
    Token := aspell_document_checker_next_misspelling(FChecker);

    if Token.len > 0 then begin
      if Length(Result) <= i then
        SetLength(Result, Length(Result) + CHUNK_SIZE);

      Result[i].Word := Copy(aLine, Token.offset + 1, Token.len);
      Result[i].Pos := Token.offset + 1; // C goes from 0, we go from 1
      Result[i].Length := Token.len;

      if FNameSuggestions then
        DoNameSuggestions(Copy(aLine, Token.offset + 1, Token.len), Result[i]);
        
      Inc(Count);
    end;

    Inc(i);
  until Token.len = 0;
  
  SetLength(Result, Count);
end;

function TDocumentSpeller.CheckDocument(const FileName: string): Integer;
var
  s: TStringList;
begin
  Result := 0;
  if FileExists(FileName) then try
    s := TStringList.Create;
    s.LoadFromFile(FileName);
    Result := CheckDocument(s);
  finally
    s.Free;
  end;
end;

function TDocumentSpeller.CheckDocument(aStringList: TStringList): Integer;
var
  i: Integer;
begin
  Result := 0;
  SetLength(FLineErrors, aStringList.Count);

  for i := 0 to aStringList.Count - 1 do begin
    FLineErrors[i] := CheckLine(aStringList[i]);
    Inc(Result, Length(FLineErrors[i]));
  end;
end;

procedure TDocumentSpeller.Reset;
begin
  aspell_document_checker_reset(FChecker);
end;

end.

