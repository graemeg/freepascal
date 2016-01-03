{ $DEFINE DEBUG}

{
    This unit implements basic regular expression support

    This file is part of the Free Pascal run time library.
    Copyright (c) 2000-2006 by Florian Klaempfland Carl Eric Codere

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{.$ define DEBUG}

(*
  - newline handling (uses all known formats of ASCII, #10,#13,#13#10 and #$85

  TODO:
     - correct backtracking, for example in (...)*
     - full | support (currently requires to put all items with | operator
        between parenthesis (in a group) to take care over order priority.
          Therefore the following would work: (foo)|(nofoo) but not
          foo|nofoo
     - getting substrings and using substrings with \1 etc.
     - do we treat several multiline characters in a row as a single
        newline character for $ and ^?
*)

{$IFDEF FPC}
{$mode objfpc}
{$ENDIF}

{** @abstract(Regular expression unit)

    This unit implements a basic regular expression parser that mostly conforms
    to the POSIX extended-regular expression syntax. It also supports standard
    escape characters for patterns (as defined in PERL).
}
unit regexpr;

  interface

    { the following declarions are only in the interface because }
    { some procedures return pregexprentry but programs which   }
    { use this unit shouldn't access this data structures        }
    type
       tcharset = set of char;
       tregexprentrytype = (ret_charset,ret_or,
          ret_illegalend,ret_backtrace,ret_startline,
          ret_endline,ret_pattern);

       pregexprentry = ^tregexprentry;
       tregexprentry = record
          next,nextdestroy : pregexprentry;
          case typ : tregexprentrytype of
             ret_charset : (chars : tcharset; elsepath : pregexprentry);
             {** This is a complete pattern path ()+ , ()* or ()?, n,m }
             ret_pattern: (pattern: pregexprentry; minoccurs: integer; maxoccurs: integer; alternative : pregexprentry);
       end;

       tregexprflag = (
         ref_singleline,
         {** This indicates that a start of line is either the
             start of the pattern or a linebreak. }
         ref_multiline,
         {** The match will be done in a case-insensitive way
              according to US-ASCII character set. }
         ref_caseinsensitive);
       tregexprflags = set of tregexprflag;

       TRegExprEngine = record
          Data : pregexprentry;
          DestroyList : pregexprentry;
          Flags : TRegExprFlags;
       end;

     const
        cs_allchars : tcharset = [#0..#255];
        cs_wordchars : tcharset = ['A'..'Z','a'..'z','_','0'..'9'];
        cs_newline : tcharset = [#10];
        cs_digits : tcharset = ['0'..'9'];
        cs_whitespace : tcharset = [' ',#9];

     var
        { these are initilized in the init section of the unit }
        cs_nonwordchars : tcharset;
        cs_nondigits : tcharset;
        cs_nonwhitespace : tcharset;

     { the following procedures can be used by units basing }
     { on the regexpr unit                                  }

     {** From a regular expression, compile an encoded version of the regular expression.

         @param(regexpr Regular expression to compile)
         @param(flags Flags relating to the type of parsing that will occur)
         @param(RegExprEngine The actual encoded version of the regular expression)
         @returns(true if success, otherwise syntax error in compiling regular expression)
     }
     function GenerateRegExprEngine(regexpr : pchar;flags : tregexprflags;var RegExprEngine: TRegExprEngine): boolean;

{$IFDEF FPC}
    {** Backward compatibility routine }
     function GenerateRegExprEngine(regexpr : pchar;flags : tregexprflags): TREGExprEngine;
{$ENDIF}

     {** Frees all up resources used for the encoded regular expression }
     procedure DestroyRegExprEngine(var regexpr : TRegExprEngine);

     {** @abstract(Matches a regular expression)

        @param(RegExprEngine The actual compiled regular expression to match against)
        @param(p The text to search for for a match)
        @param(index zero-based index to the start of the match -1 if no match in p)
        @param(len length of the match)
        @returns(true if there was a match, otherwise false)
     }
     function RegExprPos(RegExprEngine : TRegExprEngine;p : pchar;var index,len : longint) : boolean;

     function RegExprReplaceAll(RegExprEngine : TRegExprEngine;const src,newstr : ansistring;var dest : ansistring) : sizeint;

     { This function Escape known regex chars and place the result on Return. If something went wrong the
       function will return false. }
     function RegExprEscapeStr (const S : string) : string;

  implementation

{$ifdef DEBUG}
     procedure writecharset(c : tcharset);

       var
          b : byte;

       begin
          for b:=20 to 255 do
            if chr(b) in c then
              write(chr(b));
          writeln;
       end;


    const

      typ2str : array[tregexprentrytype] of string =
      (
        'ret_charset',
        'ret_or',
        'ret_illegalend',
        'ret_backtrace',
        'ret_startline',
        'ret_endline',
        'ret_pattern'
      );


     { Dumps all the next elements of a tree }
     procedure dumptree(space: string; regentry: pregexprentry);
      begin
        while assigned(regentry) do
          begin
            WriteLn(space+'------- Node Type ',typ2str[regentry^.typ]);
            if (regentry^.typ = ret_charset) then
              WriteCharSet(regentry^.chars);
            { dump embedded pattern information }
            if regentry^.typ = ret_pattern then
               begin
                 dumptree(space+#9,regentry^.pattern);
                 WriteLn(space+#9,' --- Alternative nodes ');
                 if assigned(regentry^.alternative) then
                   dumptree(space+#9#9,regentry^.alternative);
               end;
            if regentry^.typ = ret_startline then
               dumptree(space+#9,regentry^.pattern);
            regentry:=regentry^.next;
          end;
      end;
{$endif DEBUG}


     {** Determines the length of a pattern, including sub-patterns.

         It goes through the nodes and returns the pattern length
         between the two, using minOccurs as required.

         Called recursively.
     }
     function patlength(hp1: pregexprentry): integer;
       var
        count: integer;
        hp: pregexprentry;
       begin
        count:=0;
        if hp1^.typ=ret_pattern then
            hp:=hp1^.pattern
        else
            hp:=hp1;
        { now go through all chars and get the length
          does not currently take care of embedded patterns
        }
        while assigned(hp) do
          begin
            if hp^.typ = ret_pattern then
              begin
                inc(count,patlength(hp));
              end
            else
            if hp^.typ = ret_charset then
               inc(count);
            hp:=hp^.next;
          end;
        if hp1^.typ=ret_pattern then
          begin
            count:=hp1^.minOccurs*count;
          end;
         patlength:=count;
       end;

     function GenerateRegExprEngine(regexpr : pchar;flags : tregexprflags; var RegExprEngine:TRegExprEngine) : boolean;

       var
          first : pregexprentry;

       procedure doregister(p : pregexprentry);

         begin
           p^.nextdestroy:=first;
           first:=p;
         end;

       var
          currentpos : pchar;
          error : boolean;

       procedure readchars(var chars: tcharset);

         var
            c1 : char;

         begin
            chars:=[];
            case currentpos^ of
               #0:
                  exit;
               '.':
                  begin
                     inc(currentpos);
                     chars:=cs_allchars-cs_newline;
                  end;
               '\':
                  begin
                     inc(currentpos);
                     case currentpos^ of
                        #0:
                          begin
                             error:=true;
                             exit;
                          end;
                        't':
                           begin
                              inc(currentpos);
                              chars:=[#9];
                           end;
                        'n':
                           begin
                              inc(currentpos);
                              chars:=[#10];
                           end;
                        'r':
                           begin
                              inc(currentpos);
                              chars:=[#13];
                           end;
                        'd':
                          begin
                             inc(currentpos);
                             chars:=cs_digits;
                          end;
                        'D':
                          begin
                             inc(currentpos);
                             chars:=cs_nondigits;
                          end;
                        's':
                          begin
                             inc(currentpos);
                             chars:=cs_whitespace;
                          end;
                        'S':
                          begin
                             inc(currentpos);
                             chars:=cs_nonwhitespace;
                          end;
                        'w':
                           begin
                              inc(currentpos);
                              chars:=cs_wordchars;
                           end;
                        'W':
                           begin
                              inc(currentpos);
                              chars:=cs_nonwordchars;
                           end;
                        'f' :
                            begin
                              inc(currentpos);
                              chars:= [#12];
                            end;
                        'a' :
                            begin
                              inc(currentpos);
                              chars:= [#7];
                            end;
                         else
                           begin { Some basic escaping...}
                              chars := [currentpos^];
                              inc (currentpos);
                              {error:=true;
                              exit;}
                           end;
                     end;
                  end;
               else
                 begin
                    if ref_caseinsensitive in flags then
                       c1:=upcase(currentpos^)
                    else
                       c1:=currentpos^;

                    inc(currentpos);
                    if currentpos^='-' then
                      begin
                         inc(currentpos);
                         if currentpos^=#0 then
                           begin
                              error:=true;
                              exit;
                           end;
                         if ref_caseinsensitive in flags then
                           chars:=[c1..upcase(currentpos^)]
                         else
                           chars:=[c1..currentpos^];
                         inc(currentpos);
                      end
                    else
                      chars:=[c1];
                 end;
            end;
         end;


       procedure readcharset(var charset: tcharset);

         var
           chars: tcharset;
         begin
            charset:=[];
            case currentpos^ of
               #0:
                  exit;
               '[':
                  begin
                     inc(currentpos);
                     while currentpos^<>']' do
                       begin
                          if currentpos^='^' then
                            begin
                               inc(currentpos);
                               readchars(chars);
                               charset:=charset+(cs_allchars-chars);
                            end
                          else
                            begin
                              readchars(chars);
                              charset:=charset+chars;
                            end;
                          if error or (currentpos^=#0) then
                            begin
                               error:=true;
                               exit;
                            end;
                       end;
                     inc(currentpos);
                  end;
               '^':
                  begin
                     inc(currentpos);
                     readchars(chars);
                     charset:=cs_allchars-chars;
                  end;
               else
                  begin
                    readchars(chars);
                    charset:=chars;
                  end;
            end;
         end;


       (* takes care of parsing the {n}, {n,} and {n,m} regular expression
          elements. In case of error, sets error to true and returns false,
          otherwise returns true and set minoccurs and maxoccurs accordingly
          (-1 if not present). *)
       function parseoccurences(var currentpos: pchar; var minoccurs,maxoccurs: integer): boolean;
         var
          minOccursString: string;
          maxOccursString: string;
         begin
           parseoccurences:=false;
           minOccurs:=-1;
           maxOccurs:=-1;
           inc(currentpos);
           minOccursString:='';
           if currentPos^ = #0 then
             begin
               error:=true;
               exit;
             end;
            while (currentpos^<>#0) and (currentpos^ in ['0'..'9']) do
                begin
                   minOccursString:=minOccursString+currentPos^;
                   inc(currentpos);
                end;
            if length(minOccursString) = 0 then
                begin
                  error:=true;
                  exit;
                end;
            Val(minOccursString,minOccurs);
            { possible cases here: commad or end bracket }
            if currentpos^= '}' then
              begin
                inc(currentpos);
                maxOccurs:=minOccurs;
                parseoccurences:=true;
                exit;
              end;
            if currentpos^= ',' then
              begin
                maxOccursString:='';
                inc(currentpos);
                while (currentpos^<>#0) and (currentpos^ in ['0'..'9']) do
                begin
                   maxOccursString:=maxOccursString+currentPos^;
                   inc(currentpos);
                end;
                if currentpos^= '}' then
                 begin
                   { If the length of the string is zero, then there is
                     no upper bound. }
                   if length(maxOccursString) > 0 then
                      Val(maxOccursString,maxOccurs)
                   else
                      maxOccurs:=high(integer);
                   inc(currentpos);
                   parseoccurences:=true;
                   exit;
                 end;
              end;
              error:=true;
         end;


       function parseregexpr(next,elsepath : pregexprentry) : pregexprentry;

         var
            hp : pregexprentry;
            minOccurs,maxOccurs: integer;
            hp3: pregexprentry;
            cs : tcharset;
            chaining : ^pregexprentry;

         begin
            chaining:=nil;
            parseregexpr:=nil;
            elsepath:=nil;
            if error then
              exit;
            { this dummy allows us to redirect the elsepath later }
{            new(ep);
            doregister(ep);
            ep^.typ:=ret_charset;
            ep^.chars:=[];
            ep^.elsepath:=elsepath;
            elsepath:=ep;}
            while true do
              begin
                 if error then
                   exit;
                 case currentpos^ of
                    '(':
                       begin
                          inc(currentpos);
                          hp:=parseregexpr(nil,nil);
                          { Special characters after the bracket }
                           if error then
                              exit;
                          if currentpos^<>')' then
                            begin
                               error:=true;
                               exit;
                            end;
                          inc(currentpos);
                            case currentpos^ of
                            '*':
                               begin
                                  inc(currentpos);
                                  new(hp3);
                                  doregister(hp3);
                                  hp3^.typ:=ret_pattern;
                                  hp3^.alternative:=nil;
                                  hp3^.pattern:=hp;
                                  hp3^.elsepath:=elsepath;
                                  hp3^.minoccurs:=0;
                                  hp3^.maxoccurs:=high(integer);
                                  hp3^.next:=nil;
                                  if assigned(chaining) then
                                    chaining^:=hp3
                                  else
                                    parseregexpr:=hp3;
                                  chaining:=@hp3^.next;
                               end;
                            '+':
                               begin
                                  inc(currentpos);
                                  new(hp3);
                                  doregister(hp3);
                                  hp3^.typ:=ret_pattern;
                                  hp3^.alternative:=nil;
                                  hp3^.pattern:=hp;
                                  hp3^.elsepath:=elsepath;
                                  hp3^.minoccurs:=1;
                                  hp3^.maxoccurs:=high(integer);
                                  hp3^.next:=nil;
                                  if assigned(chaining) then
                                    chaining^:=hp3
                                  else
                                    parseregexpr:=hp3;
                                  chaining:=@hp3^.next;
                               end;

                            '?':
                               begin
                                  inc(currentpos);
                                  new(hp3);
                                  doregister(hp3);
                                  hp3^.typ:=ret_pattern;
                                  hp3^.alternative:=nil;
                                  hp3^.pattern:=hp;
                                  hp3^.elsepath:=elsepath;
                                  hp3^.minoccurs:=0;
                                  hp3^.maxoccurs:=1;
                                  hp3^.next:=nil;
                                  if assigned(chaining) then
                                    chaining^:=hp3
                                  else
                                    parseregexpr:=hp3;
                                  chaining:=@hp3^.next;
                               end;
                            '{':
                               begin
                                 if not parseOccurences(currentPos,minOccurs,maxOccurs) then
                                   exit;
<<<<<<< HEAD
<<<<<<< HEAD
                                  // currentpos is increased by parseOccurences
=======
                                  inc(currentpos);
>>>>>>> graemeg/fixes_2_2
=======
                                  inc(currentpos);
>>>>>>> origin/fixes_2_2
                                  new(hp3);
                                  doregister(hp3);
                                  hp3^.typ:=ret_pattern;
                                  hp3^.alternative:=nil;
                                  hp3^.pattern:=hp;
                                  hp3^.elsepath:=elsepath;
                                  hp3^.minoccurs:=minOccurs;
                                  hp3^.maxoccurs:=maxOccurs;
                                  hp3^.next:=nil;
                                  if assigned(chaining) then
                                    chaining^:=hp3
                                  else
                                    parseregexpr:=hp3;
                                  chaining:=@hp3^.next;
                               end;
                            else
                              begin
                                { go to end of this list - always the
                                  last next used }
(*
                                hp3:=hp;
                                while assigned(hp3^.next) do
                                  begin
                                    hp3:=hp3^.next;
                                  end;
                                if assigned(chaining) then
                                   chaining^:=hp
                                else
                                   parseregexpr:=hp;
                                chaining:=@hp3^.next;*)
                                  new(hp3);
                                  doregister(hp3);
                                  hp3^.typ:=ret_pattern;
                                  hp3^.alternative:=nil;
                                  hp3^.pattern:=hp;
                                  hp3^.elsepath:=elsepath;
                                  hp3^.minoccurs:=1;
                                  hp3^.maxoccurs:=1;
                                  hp3^.next:=nil;
                                  if assigned(chaining) then
                                    chaining^:=hp3
                                  else
                                    parseregexpr:=hp3;
                                  chaining:=@hp3^.next;

                              end;
                          end;
                       end;
{ This is only partially implemented currently, as the terms before
  the | character must be grouped together with parenthesis, which
  is also compatible with other regular expressions.
}
                    '|':
                       begin
{$ifdef DEBUG}
                          writeln('Creating or entry');
{$endif DEBUG}
                          if (not assigned (hp3)) then
                            begin
                              error:=true;
                              exit;
                            end;
                          if (hp3^.typ <> ret_pattern) then
                            begin
                              error:=true;
                              exit;
                            end;
                          while currentpos^='|' do
                            begin
                              inc(currentpos);
                              if currentpos^=#0 then
                                begin
                                   error:=true;
                                   exit;
                                end;
                              { always put the longest pattern first, so
                                swap the trees as necessary.
                              }
                              hp := parseregexpr (next, elsepath);
                              if patlength(hp) > patlength(hp3^.pattern) then
                                begin
                                  hp3^.alternative:=hp3^.pattern;
                                  hp3^.pattern:=hp;
                                end
                              else
                                 hp3^.alternative:=hp;
                            end;
                       end;
                    ')':
                       exit;
                    '^':
                       begin
                          inc(currentpos);
                          hp:=parseregexpr(nil,nil);
                          { Special characters after the bracket }
                           if error then
                              exit;
                           new(hp3);
                           doregister(hp3);
                           hp3^.typ:=ret_startline;
                           hp3^.pattern:=hp;
                           hp3^.elsepath:=elsepath;
                           hp3^.next:=nil;
                           if assigned(chaining) then
                              chaining^:=hp3
                           else
                              parseregexpr:=hp3;
                           chaining:=@hp3^.next;
                       end;
                    '$':
                       begin
                          inc(currentpos);
                          new(hp);
                          doregister(hp);
                          hp^.typ:=ret_endline;
                          hp^.elsepath:=elsepath;
                          hp^.next:=nil;
                          if assigned(chaining) then
                            chaining^:=hp
                          else
                            parseregexpr:=hp;
                          chaining:=@hp^.next;
                       end;
                    #0:
                       exit;
                    else
                      begin
                         readcharset(cs);
                         if error then
                           exit;
                         case currentpos^ of
                            '*':
                               begin
                                  inc(currentpos);
                                  new(hp);
                                  doregister(hp);
                                  hp^.typ:=ret_charset;
                                  hp^.chars:=cs;
                                  hp^.elsepath:=nil;
                                  hp^.next:=nil;
                                  new(hp3);
                                  doregister(hp3);
                                  hp3^.typ:=ret_pattern;
                                  hp3^.alternative:=nil;
                                  hp3^.pattern:=hp;
                                  hp3^.elsepath:=elsepath;
                                  hp3^.minoccurs:=0;
                                  hp3^.maxoccurs:=high(integer);
                                  hp3^.next:=nil;
                                  if assigned(chaining) then
                                    chaining^:=hp3
                                  else
                                    parseregexpr:=hp3;
                                  chaining:=@hp3^.next;
                               end;
                            '+':
                               begin
                                  inc(currentpos);
                                  new(hp);
                                  doregister(hp);
                                  hp^.typ:=ret_charset;
                                  hp^.chars:=cs;
                                  hp^.elsepath:=nil;
                                  hp^.next:=nil;

                                  new(hp3);
                                  doregister(hp3);
                                  hp3^.typ:=ret_pattern;
                                  hp3^.alternative:=nil;
                                  hp3^.pattern:=hp;
                                  hp3^.elsepath:=elsepath;
                                  hp3^.minoccurs:=1;
                                  hp3^.maxoccurs:=high(integer);
                                  hp3^.next:=nil;
                                  if assigned(chaining) then
                                    chaining^:=hp3
                                  else
                                    parseregexpr:=hp3;
                                  chaining:=@hp3^.next;
                               end;
                            '?':
                               begin
                                  inc(currentpos);
                                  new(hp);
                                  doregister(hp);
                                  hp^.typ:=ret_charset;
                                  hp^.chars:=cs;
                                  hp^.elsepath:=nil;
                                  hp^.next:=nil;

                                  new(hp3);
                                  doregister(hp3);
                                  hp3^.typ:=ret_pattern;
                                  hp3^.pattern:=hp;
                                  hp3^.alternative:=nil;
                                  hp3^.elsepath:=elsepath;
                                  hp3^.minoccurs:=0;
                                  hp3^.maxoccurs:=1;
                                  hp3^.next:=nil;
                                  if assigned(chaining) then
                                    chaining^:=hp3
                                  else
                                    parseregexpr:=hp3;
                                  chaining:=@hp3^.next;
                               end;
                            '{':
                               begin
                                 if not parseOccurences(currentPos,minOccurs,maxOccurs) then
                                   exit;
                                  new(hp);
                                  doregister(hp);
                                  hp^.typ:=ret_charset;
                                  hp^.chars:=cs;
                                  hp^.elsepath:=nil;
                                  hp^.next:=nil;

                                  new(hp3);
                                  doregister(hp3);
                                  hp3^.typ:=ret_pattern;
                                  hp3^.alternative:=nil;
                                  hp3^.pattern:=hp;
                                  hp3^.elsepath:=elsepath;
                                  hp3^.minoccurs:=minOccurs;
                                  hp3^.maxoccurs:=maxOccurs;
                                  hp3^.next:=nil;
                                  if assigned(chaining) then
                                    chaining^:=hp3
                                  else
                                    parseregexpr:=hp3;
                                  chaining:=@hp3^.next;

                                end;
                            else
                               { Normal character }
                               begin
                                  new(hp);
                                  doregister(hp);
                                  hp^.typ:=ret_charset;
                                  hp^.chars:=cs;
                                  hp^.elsepath:=elsepath;
                                  hp^.next:=next;
                                  if assigned(chaining) then
                                    chaining^:=hp
                                  else
                                    parseregexpr:=hp;
                                  chaining:=@hp^.next;
                                  continue;
                               end;
                           { This was a pattern }
                         end; { END CASE }
                      end;
                 end;
              end;
         end;

       var
          endp : pregexprentry;

       begin
          GenerateRegExprEngine:=false;
          RegExprEngine.Data:=nil;
          RegExprEngine.DestroyList:=nil;
          if regexpr=nil then
            exit;
          first:=nil;
          if (ref_singleline in flags) and (ref_multiline in flags) then
            exit;
          currentpos:=regexpr;
          GenerateRegExprEngine:=true;
          error:=false;
          new(endp);
          doregister(endp);
          endp^.typ:=ret_illegalend;
          RegExprEngine.flags:=flags;
          RegExprEngine.Data:=parseregexpr(nil,endp);
{$IFDEF DEBUG}
          writeln('========== Generating tree ============');
          dumptree('',RegExprEngine.Data);
{$ENDIF}
          RegExprEngine.DestroyList:=first;
          if error or (currentpos^<>#0) then
            begin
              GenerateRegExprEngine:=false;
              DestroyRegExprEngine(RegExprEngine);
            end;
       end;


{$IFDEF FPC}
    function GenerateRegExprEngine(regexpr : pchar;flags : tregexprflags): TREGExprEngine;
    var
     r: TRegExprEngine;
    begin
      GenerateRegExprEngine(regexpr,flags,r);
      GenerateRegExprEngine:=r;
    end;
{$ENDIF}

    procedure DestroyRegExprEngine(var regexpr : TRegExprEngine);

       var
          hp : pregexprentry;

       begin
          hp:=regexpr.DestroyList;
          while assigned(hp) do
            begin
               regexpr.DestroyList:=hp^.nextdestroy;
               dispose(hp);
               hp:=regexpr.DestroyList;
            end;
          regexpr.Data:=nil;
          regexpr.DestroyList:=nil;
       end;

     function RegExprPos(regexprengine : TRegExprEngine;p : pchar;var index,len : longint) : boolean;

       var
          lastpos : pchar;
          firstpos: pchar;

       { Does the actual search of the data - return true if the term was found }
       function dosearch(regexprentry : pregexprentry;pos : pchar) : boolean;
       var
          found: boolean;
          checkvalue: boolean;
          savedpos: pchar;
          counter: word;

         begin
            dosearch:=false;
            while true do
              begin
                 {$IFDEF Debug}
                 writeln('Entering ',typ2str[regexprentry^.typ]);
                 writeln('Pattern length ',patlength(regexprentry));
                 {$ENDIF Debug}
                 case regexprentry^.typ of
                    ret_endline:
                      begin
                         { automatically a match! }
                         if pos^ = #0 then
                            begin
                              dosearch:=true;
                              exit;
                            end;
                         if ref_multiline in regexprengine.flags then
                            begin
                              { Supports DOS/Commodore/UNIX/IBM Mainframe line formats }
                              { avoid reading invalid memory also }
                                  if (pos^=#13) and ((pos+1)^=#10) then
                                    begin
                                      regexprentry:=regexprentry^.next;
                                    end
                                  else
                                  if (pos^=#$85) or (pos^=#10) or ((pos^=#13) and ((pos-1) >= firstpos) and ((pos-1)^ <> #10)) then
                                    begin
                                       regexprentry:=regexprentry^.next;
                                    end
                                  else
                                    begin
                                       dosearch:=false;
                                       exit;
                                    end;
                             end
                           else
                             exit;
                      end;
                    ret_pattern:
                      begin
                         found:=false;
                         { Take care of occurences here }
                         savedpos:=pos;
                         counter:=0;
                         repeat
                           found:=dosearch(regexprentry^.pattern,pos);
                           if not found then
                            break;
                           pos:=lastpos;
                           inc(counter);
                         until (not found) or (counter >= regexprentry^.maxoccurs) or (pos^= #0);

                         if counter = 0 then
                           begin
                             { If there was no occurence and the minimum occurence is > 0 then
                               problem.
                             }
                             if (regexprentry^.minoccurs > 0) then
                              begin
                                dosearch:=false;
                                { verify alternative path as required }
                                if assigned(regexprentry^.alternative) then
                                  begin
                                     dosearch:=dosearch(regexprentry^.alternative,savedpos);
                                     exit;
                                  end;
                                exit;
                              end;
                             dosearch:=true;
                             lastpos:=savedpos;
                           end
                         else
                           { found }
                           begin
                              { Possible choices :
                                 - found and (counter >= minOccurences) and (counter =< maxOccurences) = true
                                 - found and (counter < minOccurences) or (counter > maxOccurences) = false
                              }
                              if (counter < regexprentry^.minoccurs) or (counter > regexprentry^.maxoccurs) then
                                begin
                                  dosearch:=false;
                                  exit;
                                end;
                              dosearch:=true;
                              { if all matches were found, and the current position
                                points to zero (processed all characters) }
                              if pos^=#0 then
                                begin
                                  dosearch:=true;
                                  exit;
                                end;
                           end;
                         { If we are that means the matches were valid, go to next element to match
                         }
                         regexprentry:=regexprentry^.next;
                         if (counter = 0) and not assigned(regexprentry) then
                           exit;
                      end;
                    ret_startline:
                      begin
                         checkvalue:=pos=firstpos;
                         if ref_multiline in regexprengine.flags then
                           begin
                             { Supports DOS/Commodore/UNIX/IBM Mainframe line formats }
                             { avoid reading invalid memory also }
                             if
                                 (
                                   ((pos-1) >= firstpos) and ((pos-1)^=#$85)
                                  )
                              or
                                 (
                                   ((pos-1) >= firstpos) and ((pos-1)^=#10)
                                  )
                              or
                                (
                                 ((pos-1) >= firstpos) and ((pos-1)^=#13) and
                                 ((pos)^ <> #10)
                                )
                             then
                               begin
                                 checkvalue:=true;
                               end;
                           end;
                          if checkvalue then
                            begin
                              dosearch:=dosearch(regexprentry^.pattern,pos);
                              regexprentry:=regexprentry^.next;
                              if not dosearch then
                                exit;
                              pos:=lastpos;
                            end
                          else
                            begin
                              dosearch:=false;
                              exit;
                            end;
                      end;
                    ret_charset:
                      begin
                         if (pos^ in regexprentry^.chars) or
                           ((ref_caseinsensitive in regexprengine.flags) and
                            (upcase(pos^) in regexprentry^.chars)) then
                           begin
{$ifdef DEBUG}
                              writeln('Found matching: ',pos^);
{$endif DEBUG}
                              regexprentry:=regexprentry^.next;
                              inc(pos);
                           end
                         else
                           begin
{$ifdef DEBUG}
                              writeln('Found unmatching: ',pos^);
{$endif DEBUG}
                              exit;
                           end;
                      end;
                    ret_backtrace:
                      begin
{$ifdef DEBUG}
                         writeln('Starting backtrace');
{$endif DEBUG}
                         if dosearch(regexprentry^.next,pos) then
                           begin
                              dosearch:=true;
                              exit;
                           end
                         else if dosearch(regexprentry^.elsepath,pos) then
                           begin
                              dosearch:=true;
                              exit;
                           end
                         else
                           exit;
                      end;
                 end;
                 lastpos:=pos;
                 if regexprentry=nil then
                   begin
                      dosearch:=true;
                      exit;
                   end;
                 if regexprentry^.typ=ret_illegalend then
                   exit;
                 { end of string, and we were expecting an end of string }
                 if (pos^=#0) and (regexprentry^.typ = ret_endline) and
                    (not assigned(regexprentry^.next)) then
                   begin
                     dosearch:=true;
                     exit;
                   end;
                 if pos^=#0 then
                   exit;
              end;
         end;

       begin
          RegExprPos:=false;
          index:=0;
          len:=0;
          firstpos:=p;
          if regexprengine.Data=nil then
            exit;
          while p^<>#0 do
            begin
               if dosearch(regexprengine.Data,p) then
                 begin
                    len:=lastpos-p;
                    RegExprPos:=true;
                    exit;
                 end
               else
                 begin
                    inc(p);
                    inc(index);
                 end;
            end;
          index:=-1;
       end;


  function RegExprReplaceAll(RegExprEngine : TRegExprEngine;const src,newstr : ansistring;var dest : ansistring) : sizeint;
    var
      index,len : longint;
      pos,lastpos : pchar;
      first : boolean;
      oldlength : PtrInt;
    begin
      pos:=pchar(src);
      lastpos:=nil;
      first:=true;
      Result:=0;
      { estimate some length }
      SetLength(dest,length(src)+((length(src) div 10)*length(newstr)));
      while RegExprPos(RegExprEngine,pos,index,len) do
        begin
          inc(pos,index);
          if (lastpos = nil) or (pos>lastpos) then
            begin
              if lastpos = nil then lastpos := pchar(src);
              { copy skipped part }

              { because we cheat with SetLength a SetLength(...,0) isn't what we want
                so we've to trick at the first SetLength call
              }
              if first then
                begin
                  SetLength(dest,(pos-lastpos));
                  { cast dest here because it is already unified }
                  move(lastpos^,char(dest[1]),pos-lastpos);
                end
              else
                begin
                  oldlength:=Length(dest);
                  SetLength(dest,oldlength+(pos-lastpos));
                  move(lastpos^,char(dest[oldlength+1]),pos-lastpos);
                end;
              first:=false;
            end;
          { found }
          inc(Result);
          dest:=dest+newstr;
          inc(pos,len);
          lastpos:=pos;
        end;
      { copy remainder }
      len:=strlen(pos);
      if first then
        begin
          SetLength(dest,len);
          move(pos^,char(dest[length(dest)+1]),len);
        end
      else
        begin
          oldlength:=Length(dest);
          SetLength(dest,oldlength+len);
          move(pos^,char(dest[oldlength+1]),len);
        end
    end;


  function RegExprEscapeStr (const S : string) : string;
    var
     i, len   : integer;
     s1: string;
    begin
      RegExprEscapeStr:= '';
      s1:='';
      if (S = '') then
       exit;

      len := Length (S);

      for i := 1 to len do
        begin
          if (S [i] in ['(','|', '.', '*', '?', '^', '$', '-', '[', '{', '}', ']', ')', '\']) then
            begin
              s1 := s1 + '\';
            end;

          s1 := s1 + S[i];
        end;
      RegExprEscapeStr:=s1;
    end;

begin
   cs_nonwordchars:=cs_allchars-cs_wordchars;
   cs_nondigits:=cs_allchars-cs_digits;
   cs_nonwhitespace:=cs_allchars-cs_whitespace;
end.
