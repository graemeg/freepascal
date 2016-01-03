program Example;

{$mode objfpc}{$H+}

uses
<<<<<<< HEAD
<<<<<<< HEAD
  SpellCheck;

var
  i, j: Integer;
  s: TSuggestionArray; { in case the word is wrong, this array contains
                         a list of suggestions }
  Speller: TWordSpeller;
begin
  if Paramcount < 2 then // check if user has used valid input
    Writeln('Usage: ', ParamStr(0), ' <lang> <word1> <word2> ...')
  else begin
    Speller := TWordSpeller.Create;
    Speller.Language := ParamStr(1);

    for i := 2 to ParamCount do begin // go for each word specified
      s := Speller.SpellCheck(ParamStr(i)); // spellcheck each word
      if Length(s) > 0 then begin // we need to write suggestions
        Write(ParamStr(i), ' is wrong. Here are some suggestions: ');
        for j := 0 to High(s) do
          Write(s[j], ' '); // write out the suggestions
        Writeln; // to keep format
      end else
        Writeln(ParamStr(i), ' is spelled correctly!');
    end;

    Speller.Free;
=======
=======
>>>>>>> origin/fixes_2_2
  sCheck;

var
  i, j, n: Integer;
  s: TSuggestionArray; { in case the word is wrong, this array contains
                         a list of suggestions }
begin
  if Paramcount < 2 then // check if user has used valid input
    Writeln('Usage: ', ParamStr(0), ' <lang> <word1> <word2> ...')
  else for i := 2 to ParamCount do begin // go for each word specified
    n := SpellCheck(ParamStr(i), ParamStr(1), s); // spellcheck each word
    if n > 0 then begin // if n > 0 then the word is wrong and we need to write suggestions
      Write(ParamStr(i), ' is wrong. Here are some suggestions: ');
      for j := 0 to High(s) do
        Write(s[j], ' '); // write out the suggestions
      Writeln; // to keep format
    end else
      Writeln(ParamStr(i), ' is spelled correctly!');
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  end;
end.

