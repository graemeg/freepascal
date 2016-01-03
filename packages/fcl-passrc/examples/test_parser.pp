{   This is a test-program for the fcl-passrc package (except writer-class).

    Please notice that i have done this to find out how good the parser workes,
    it is not thought to be a good example to use the fcl-passrc package but
    may give you hints on using it.

    It is done to test the source of these units for usability, completeness and
    bugs. It is base on the fcl-passrc exampe.
    It workes like a pretty-printer to compare the output of this program with
    the original code, but is not thought to be a real pretty-printer as
    e.g. the semicolons can sometimes not be set at the place they sould be
    (this imformation is not available from the parsing-engine, as a parser
    should only give you a positiv result if the source is valid, otherwise
    you get a negative result).
    Also the output is not always in the same order as in input as this
    information is not available easily.
    
    !!!Do not expect this program to produce executeable output!!!

    Status: -workes with one Unit or Program
            -Some type declarations missing
            -string[n] the [n] part missing -> missing in parser
            -array of const -> missing in parser
            -Hints deprecated, etc. missing sometimes
            -the parser splits x,y:atype
              x:atype
              y:atype
             i tryed to put them together again
            - () missing in statements: () expression and typecast
            -missing forward class declaration like x=class
            -incomplete !

            parser: -ugly ''' quotation from scanner, why not #39 ?
                    -see comments in the program for hints
                    -incomplete !

    Usage: call with one complete filename of a Unit or Program
           defaults for the parser are 'linux' and 'i386'

    Output: is 'pretty-printed' to stdout or unformated
            The unformated output is thought to be diffed with the original
            source to see differences caused by the parser (a tool to unformat
            a souce file is in progress but not finished jet).

    Bugs: 1. In case of unimplemented statements (like up to now asm) the parser
             cause a excemtion to abort the program hard.
          2. Missing implementaion in this program should not print out anything
             or result in not pascal conform output.

    Hit: The parser uses directives given in the source file.

   Hints to read the code:
    There are comments in the code with hints and marks of possible bugs.
    During development some code was modified for true order output but the
    old code is still available as a comment as it is easier to understand.
    This is programmed using 'recursive' calls. Most options in functions are
    for printing the output.
    There is no writer-class used to keep it simple and see what is done.
    All output is produced by direct writing to stdout, this cause problems in
    furter development; a function result as string may be more usable.

    The parser was written to be used for unit interface and was expanded to
    work with program and implementation too. It does nearly no seperate
    things for programs, they are adapted to the unit scheme (see main).

    The order will change in following case:
     -function with forward declaration (also overloading etc.)


  Inheritance (only the important ones):

    TInterfaceSection, TImplementationSection, TProgramSection
     |
    TPasSection
     |
    TPasDeclarations
     |
    TPasElement
     |
    TPasElementBase
     |
    TObject

    TInitializationSection, TFinalizationSection
     |
    TPasImplBlock
     |
    TPasImplElement
     |
    TPasElement
     |
    TPasElementBase
     |
    TObject

    TPasProgram
     |
    TPasModule
     |
    TPasElement
     |
    TPasElementBase
     |
    TObject

  Dependance Structure :

    TPasPackage = class(TPasElement)
      |
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Modules: TFPList;
=======
    Modules: TList;
>>>>>>> graemeg/cpstrnew
=======
    Modules: TList;
>>>>>>> graemeg/cpstrnew
=======
    Modules: TList;
>>>>>>> graemeg/cpstrnew
=======
    Modules: TList;
>>>>>>> origin/cpstrnew

    TPasModule = class(TPasElement)
      |-InterfaceSection: TInterfaceSection;
      |  |-Declarations -> forward part, unit only
      |
      |-ImplementationSection: TImplementationSection;
      |  |-Declarations -> full declaration, unit and program
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      |     |-Functions: TFPList;
=======
      |     |-Functions: TList;
>>>>>>> graemeg/cpstrnew
=======
      |     |-Functions: TList;
>>>>>>> graemeg/cpstrnew
=======
      |     |-Functions: TList;
>>>>>>> graemeg/cpstrnew
=======
      |     |-Functions: TList;
>>>>>>> origin/cpstrnew
      |        |-TPasFunction = class(TPasProcedureBase)
      |           |-Body: TProcedureBody;
      |              |-Declarations -> declaration and sub function
      |              |-Body: TPasImplBlock; -> procedure block
      |
      |-InitializationSection: TInitializationSection;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      |  |-TPasImplBlock.Elements: TFPList; -> main block
      |
      |-FinalizationSection: TFinalizationSection;
         |-TPasImplBlock.Elements: TFPList; -> unit only

    Declarations = class(TPasElement)
      |-Declarations: TFPList; -> the following are all in here
      |-ResStrings: TFPList;
      |-Types: TFPList;
      |-Consts: TFPList;
      |-Classes: TFPList;
      |-Functions: TFPList;
      |-Variables: TFPList;
      |-Properties: TFPList;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      |  |-TPasImplBlock.Elements: TList; -> main block
      |
      |-FinalizationSection: TFinalizationSection;
         |-TPasImplBlock.Elements: TList; -> unit only

    Declarations = class(TPasElement)
      |-Declarations: TList; -> the following are all in here
      |-ResStrings: TList;
      |-Types: TList;
      |-Consts: TList;
      |-Classes: TList;
      |-Functions: TList;
      |-Variables: TList;
      |-Properties: TList;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    }


program test_parser1;
{$mode objfpc}{$H+}

uses SysUtils, Classes, PParser, PasTree;

//# types the parser needs

type
  { We have to override abstract TPasTreeContainer methods.
    See utils/fpdoc/dglobals.pp for an implementation of TFPDocEngine,
    a "real" engine. }
  TSimpleEngine = class(TPasTreeContainer)
  public
    function CreateElement(AClass: TPTreeElement; const AName: String;
      AParent: TPasElement; AVisibility: TPasMemberVisibility;
      const ASourceFilename: String; ASourceLinenumber: Integer): TPasElement;
      override;
    function FindElement(const AName: String): TPasElement; override;
  end;

function TSimpleEngine.CreateElement(AClass: TPTreeElement; const AName: String;
  AParent: TPasElement; AVisibility: TPasMemberVisibility;
  const ASourceFilename: String; ASourceLinenumber: Integer): TPasElement;
begin
  Result := AClass.Create(AName, AParent);
  Result.Visibility := AVisibility;
  Result.SourceFilename := ASourceFilename;
  Result.SourceLinenumber := ASourceLinenumber;
end;

function TSimpleEngine.FindElement(const AName: String): TPasElement;
begin
  { dummy implementation, see TFPDocEngine.FindElement for a real example }
  Result := nil;
end;


//# main var
var
  M: TPasModule;
  E: TPasTreeContainer;
  I: Integer;
  cmdl, TargetOS, TargetCPU : string;
  isim, //is Impleamentation, only for GetTPasProcedureBody
  Unformated:boolean; // no Formating in output
  

//# tools

 function GetIndent(indent:integer):String;
   var i:integer;
  begin
   Result:='';
   if not Unformated then 
      for i:=1 to indent do Result:=Result+' ';
  end;

 //delete ugly quoting '''STRING'''
 function DelQuot(s:String):String;
    var i:integer;
    const s1=#39#39#39;
   begin
    Result:='';
    i:=pos(s1,s);
    while i > 0 do
     begin
      if i > 0 then delete(s,i,2);
      i:=pos(s1,s);
     end; 
    //if i > 0 then delete(s,i,2);
    Result:=s;
   end;

 //LeadingSpace only valid if Formated output (as this will be one line in output)
 //UnFormated: all is printed in a new line
 procedure WriteFmt(LeadingSpace:boolean; s:String; Semicolon:boolean);
  begin
   if Semicolon then s:=s+';';
   if Unformated then writeln(s)
    else if LeadingSpace then write(' ',s)
     else write(s);
  end;

//# parsing output

function GetTPasImplBlock(lb:TPasImplBlock; indent,declistby:integer;
                           LastNoSem,NoFirstIndent:boolean):boolean; forward;

function GetTPasImplElement(le:TPasImplElement; lindent:integer;
                             lLastNoSem,NoFirstIndent:boolean):boolean; forward;

procedure GetDecls(Decl:TPasDeclarations; indent:integer); forward;
//procedure PrintDecls(Decl:TPasDeclarations; indent:integer); forward;

//# most is for implementation or implblocks except the expr things

function ReturnTPasMemberHints(h:TPasMemberHints):String;
 begin
  Result:='';
  if hDeprecated    in h then Result:=' deprecated'; 
  if hLibrary       in h then Result:=Result+' library';
  if hPlatform      in h then Result:=Result+' platform';
  if hExperimental  in h then Result:=Result+' experimental';
  if hUnimplemented in h then Result:=Result+' unimplemented';
 end;      
   
function GetTPasMemberHints(h:TPasMemberHints):Boolean;
 begin
  Result:=false;
  if hDeprecated in h then begin write(' deprecated'); Result:=true; end;
  if hLibrary in h then begin write(' library'); Result:=true; end;
  if hPlatform in h then begin write(' platform'); Result:=true; end;
  if hExperimental in h then begin write(' experimental'); Result:=true; end;
  if hUnimplemented in h then begin write(' unimplemented'); Result:=true; end;
 end;   




function GetTPasExprKind(lpek:TPasExprKind):String;
 begin
  Result:='';
  case lpek of
    pekIdent:Result:='ID';
    pekNumber:Result:='NUMBER';
    pekString:Result:='STRING';
    pekSet:Result:='SET';
    pekNil:Result:='NIL';
    pekBoolConst:Result:='BOOL';
    pekRange:Result:='RANGE';
    pekUnary:Result:='UNARY';
    pekBinary:Result:='BINARY';
    pekFuncParams:Result:='FUNCPAR';
    pekArrayParams:Result:='ARRAYPAR';
    pekListOfExp:Result:='EXPLIST';
  end;
 end;

procedure GetTPasExpr(lex:TPasExpr);
 var lex1:TpasExpr;
     lpe:TParamsExpr;
     l:integer;
     lbk,rbk,sep:string;
     lav:TArrayValues;
     lrv:TRecordValues;
     rvi:TRecordValuesItem;


 function GetExpKind(ek:TPasExprKind; var lbrak,rbrak:string):string;
  begin
   lbrak:='';
   rbrak:='';
   Result:='';
   case ek of
    pekIdent:Result:='ID';
    pekNumber:Result:='NU';
    pekString:begin lbrak:=#39; rbrak:=#39; Result:=#39; end;
    pekSet:begin lbrak:='['; rbrak:=']'; Result:=','; end;
    pekNil:Result:='NIL';
    pekBoolConst:Result:='';
    pekRange:Result:='..';
    pekUnary:Result:='';
    pekBinary:Result:='';
    pekFuncParams:begin lbrak:='('; rbrak:=')'; Result:=','; end;
    pekArrayParams:begin lbrak:='['; rbrak:=']'; Result:=','; end;
    pekListOfExp:Result:=',';
    pekInherited:Result:=' InheriteD';
    pekSelf:Result:=' SelF';
   end;
  end;

 function GetOp(lop:TExprOpCode):string;
  begin
   Result:='';
   case lop of
    eopNone:Result:='';
    eopAdd:Result:='+';
    eopSubtract:Result:='-';
    eopMultiply:Result:='*';
    eopDivide:Result:='/';
    eopDiv:Result:=' div ';
    eopMod:Result:=' mod ';
    eopPower:Result:='^';
    eopShr:Result:=' shr ';
    eopSHl:Result:=' shl ';
    eopNot:Result:=' not ';
    eopAnd:Result:=' and ';
    eopOr:Result:=' or ';
    eopXor:Result:=' xor ';
    eopEqual:Result:='=';
    eopNotEqual:Result:='<>';
    eopLessThan:Result:='<';
    eopGreaterThan:Result:='>';
    eopLessthanEqual:Result:='<=';
    eopGreaterThanEqual:Result:='>=';
    eopIn:Result:=' in ';
    eopIs:Result:=' is ';
    eopAs:Result:=' as ';
    eopSymmetricaldifference:Result:='><';
    eopAddress:Result:='@';
    eopDeref:Result:='^';
    eopSubIdent:Result:='.';
   end;
  end;

 begin
  if lex is TBinaryExpr then //compined constants
   begin
    sep:=GetExpKind(lex.Kind,lbk,rbk);
    //write('|');
    write(lbk);
    GetTPasExpr(TBinaryExpr(lex).left);
    write(GetOp(TBinaryExpr(lex).OpCode));
    write(sep);
    GetTPasExpr(TBinaryExpr(lex).right);
    write(rbk);
    //write('|');
    //write(' [',lex.Name,' ',GetTPasExprKind(lex.Kind),']');
   end
    else
     begin
      //write('UNARY');
      if lex is TUnaryExpr then
       begin
        lex1:=TUnaryExpr(lex).Operand;
        if lex.OpCode = eopDeref then
         begin
          GetTPasExpr(lex1);
          write(GetOp(lex.OpCode)); //unary last, only: p^
         end
        else
         begin
          write(GetOp(lex.OpCode)); //unary first: -1
          GetTPasExpr(lex1);
         end;
       end;
      if lex is TPrimitiveExpr then write(TPrimitiveExpr(lex).Value) //simple constant
      else
      if lex is TBoolConstExpr then write(TBoolConstExpr(lex).Value)
      else
      if lex is TNilExpr then write('nil')
      else
      if lex is TInheritedExpr then write('Inherited ')
      else
      if lex is TSelfExpr then write('Self')
      else
      if lex is TParamsExpr then //writeln(param1,param2,..,paramn);
        begin
         //write(' PAREX ');
         lpe:=TParamsExpr(lex);
         GetTPasExpr(lpe.Value);
         if length(lpe.Params) >0 then
          begin
           sep:=GetExpKind(lpe.Kind,lbk,rbk);
           write(lbk); //write('(');
           for l:=0 to High(lpe.Params)-1 do
            begin
             GetTPasExpr(lpe.Params[l]);
             write(sep); //seperator
            end;
           GetTPasExpr(lpe.Params[High(lpe.Params)]);
           write(rbk);//write(')');
          end
         else
          begin //funcion()
           sep:=GetExpKind(lpe.Kind,lbk,rbk);
           write(lbk,rbk);
          end;
        end
      else if lex is TArrayValues then  //const AnArrayConst: Array[1..3] of Integer = (1,2,3);
        begin
         write('(');
         lav:=TArrayValues(lex);
         if length(lav.Values) > 0 then
          begin
           for l:=0 to high(lav.Values)-1 do
            begin
             GetTPasExpr(TPasExpr(lav.Values[l]));
             write(',');
            end;
           GetTPasExpr(TPasExpr(lav.Values[high(lav.Values)]));
          end;
         write(')');
        end
      else if lex is TRecordValues then
        begin
         write('(');
         lrv:=TRecordValues(lex);
         if length(lrv.Fields) > 0 then
          begin
           for l:=0 to high(lrv.Fields)-1 do
            begin
             rvi:=TRecordValuesItem(lrv.Fields[l]);
             write(rvi.Name,':');
             GetTPasExpr(rvi.ValueExp);
             write(';');
            end;
           rvi:=TRecordValuesItem(lrv.Fields[high(lrv.Fields)]);
           write(rvi.Name,':');
           GetTPasExpr(rvi.ValueExp);
          end;
         write(')');
        end
      else
       begin
        //?
        //writeln('{ Unknown Expression: ');
        //if assigned(lex) then GetTPasExprKind(lex.Kind);
        //writeln('}');
       end;
     end;
 end;


//NoFirstIndent only for block in case:
procedure GetTPasSmt(lsmt:TPasImplStatement; lindent:integer; DoNoSem,NoFirstIndent:boolean);
 var l:integer;
     lics:TPasImplCaseStatement;
     DoSem:boolean;
     liwd:TPasImplWithDo;
     liwhd:TPasImplWhileDo;
     lieo:TPasImplExceptOn;
     lifl:TPasImplForLoop;
     lir:TPasImplRaise;
      s,s1:String;//s1 only first line of block statement

begin
  DoSem:=true;
  s:=GetIndent(lindent);
  if NoFirstIndent then s1:=' ' else s1:=s;
  if lsmt is TPasImplSimple then
    begin
     write(s1); GetTPasExpr(TPasImplSimple(lsmt).expr);
     //DoSem:=true;
    end
   else if lsmt is TPasImplAssign then
    begin
     write(s1); GetTPasExpr(TPasImplAssign(lsmt).left);
     write(':= ');
     GetTPasExpr(TPasImplAssign(lsmt).right);
     //DoSem:=true;
    end
   else if lsmt is TPasImplCaseStatement then
    begin
     lics:=TPasImplCaseStatement(lsmt);
     if lics.Expressions.Count>0 then
      begin
       write(s);
       for l:=0 to lics.Expressions.Count-2 do
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          write(DelQuot(TPasExpr(lics.Expressions[l]).GetDeclaration(True)),',');
       write(DelQuot(TPasExpr(lics.Expressions[lics.Expressions.Count-1]).GetDeclaration(True)),': '); // !!bug too much ' in expression
=======
          write(DelQuot(lics.Expressions[l]),',');
       write(DelQuot(lics.Expressions[lics.Expressions.Count-1]),': '); // !!bug too much ' in expression
>>>>>>> graemeg/cpstrnew
=======
          write(DelQuot(lics.Expressions[l]),',');
       write(DelQuot(lics.Expressions[lics.Expressions.Count-1]),': '); // !!bug too much ' in expression
>>>>>>> graemeg/cpstrnew
=======
          write(DelQuot(lics.Expressions[l]),',');
       write(DelQuot(lics.Expressions[lics.Expressions.Count-1]),': '); // !!bug too much ' in expression
>>>>>>> graemeg/cpstrnew
=======
          write(DelQuot(lics.Expressions[l]),',');
       write(DelQuot(lics.Expressions[lics.Expressions.Count-1]),': '); // !!bug too much ' in expression
>>>>>>> origin/cpstrnew
       //if not assigned(lics.Body) then writeln('TPasImplCaseStatement missing BODY');
       //if assigned(lics.Body) and (TPasImplBlock(lics.Body).Elements.Count >0) then
       //  GetTPasImplBlock(TPasImplBlock(lics.Body),lindent+1,0,false,true)
       //    else GetTPasImplBlock(TPasImplBlock(lics),lindent+1,0,false,true);  // !!bug missing body, assigned but empty
        if assigned(lics.Body) then
         begin
          if not GetTPasImplElement(lics.Body,lindent+1,false,true) then ;//writeln(';');
         end
          else writeln(';');
      end;
     DoSem:=false;
    end
   else if lsmt is TPasImplWithDo then
    begin
     liwd:=TPasImplWithDo(lsmt);   // !!Bug: missing with do at following with do !solved see Bug
     write(s1,'with ',liwd.Name);
     if liwd.Expressions.Count>0 then
      begin
       for l:=0 to liwd.Expressions.Count-2 do
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
         write(TPasExpr(liwd.Expressions[l]).GetDeclaration(true),',');
       write(TPasExpr(liwd.Expressions[liwd.Expressions.Count-1]).GetDeclaration(true));
=======
         write(liwd.Expressions[l],',');
       write(liwd.Expressions[liwd.Expressions.Count-1]);
>>>>>>> graemeg/cpstrnew
=======
         write(liwd.Expressions[l],',');
       write(liwd.Expressions[liwd.Expressions.Count-1]);
>>>>>>> graemeg/cpstrnew
=======
         write(liwd.Expressions[l],',');
       write(liwd.Expressions[liwd.Expressions.Count-1]);
>>>>>>> graemeg/cpstrnew
=======
         write(liwd.Expressions[l],',');
       write(liwd.Expressions[liwd.Expressions.Count-1]);
>>>>>>> origin/cpstrnew
      end;
     writeln(' do');
     //if TPasImplBlock(liwd.Body).Elements.Count >0  then
       //GetTPasImplBlock(TPasImplBlock(liwd.Body),0); // !!Bug: BODY Not used
       //else
     GetTPasImplBlock(TPasImplBlock(liwd),lindent+1,0,false,false);
     DoSem:=false;
    end
   else if lsmt is TPasImplWhileDo then
    begin
     liwhd:=TPasImplWhileDo(lsmt);
     writeln(s1,'while ',DelQuot(liwhd.Condition),' do');
     //if not GetTPasImplBlock(TPasImplBlock(liwhd.Body),0) then // !!Bug: BODY Not used
     GetTPasImplBlock(TPasImplBlock(liwhd),lindent,0,DoNoSem,false); //OK for all constructs
     DoNoSem:=false; //?
     DoSem:=false;
    end
   else if lsmt is TPasImplExceptOn then
    begin
     lieo:=TPasImplExceptOn(lsmt);
     writeln(s,'on ',lieo.VariableName,': ',lieo.TypeName,' do');
     if TPasImplBlock(lieo.Body) is TPasImplRaise then
      begin
       write(s,'raise ');//raise is in TPasImplBlock in this case
       GetTPasImplBlock(TPasImplBlock(lieo.Body),lindent+1,0,false,true);
      end
       else GetTPasImplBlock(TPasImplBlock(lieo.Body),lindent+1,0,false,false);
     DoSem:=false;
    end
   else if lsmt is TPasImplForLoop then
    begin
     lifl:=TPasImplForLoop(lsmt);
     //TODO variable
     write(s1,'for ',lifl.VariableName,':= ',lifl.StartValue,' ');
     if lifl.Down then write('down');
     writeln('to ',lifl.EndValue,' do');
     GetTPasImplBlock(TPasImplBlock(lifl),lindent+1,0,false,false);
     DoSem:=false;
    end
   else if lsmt is TPasImplRaise then
    begin
     write(s1,'raise ');
     lir:=TPasImplRaise(lsmt);
     if not GetTPasImplBlock(TPasImplBlock(lir),lindent,0,DoNoSem,true) then
      writeln(';');
     DoNoSem:=false;
     DoSem:=false;
    end
   else
    begin
     if assigned(lsmt.Elements) then
      begin
       writeln('{ Unknown SMT(s): '); //,lsmt.Name,' ',lsmt.ElementTypeName);
       for l:=0 to lsmt.Elements.Count-1 do
         write(s,' SMT ',l,' ',TPasElement(lsmt.Elements[l]).Name);
       writeln('}');
      end;
     DoSem:=false;
    end;
  if not DoNoSem then
   begin
    if DoSem then writeln(';');
   end
    else writeln;
end;


 //result: result of TPasImplBlock or valid element
 //NoFirstIndent only for block in case:
 function GetTPasImplElement(le:TPasImplElement; lindent:integer;
                             lLastNoSem,NoFirstIndent:boolean):boolean;
  var liie:TPasImplIfElse;
      lico:TPasImplCaseOf;
      lice:TPasImplCaseElse;
      liru:TPasImplRepeatUntil;
      lit:TPasImplTry;
      //lic:TPasImplCommand;
      s,s1:String;//s1 only first line of block statement

begin
  Result:=true;
  s:=GetIndent(lindent);
  if NoFirstIndent then s1:=' ' else s1:=s;
    if le is TPasImplStatement then
      begin
       if NoFirstIndent then lindent:=0;
       GetTPasSmt(TPasImplStatement(le),lindent+1,lLastNoSem,NoFirstIndent);
      end
     else if le is TPasImplIfElse then
      begin
       liie:=TPasImplIfElse(le);
       write(s1,'if ',DelQuot(liie.Condition),' then ');
       if assigned(liie.ElseBranch) then
        begin
         writeln;
         GetTPasImplElement(liie.IfBranch,lindent+1,true,false);
         writeln(s,'else');// {if}');
         GetTPasImplElement(liie.ElseBranch,lindent+1,lLastNoSem,false);
        end
       else
        begin //no else part
         if assigned(liie.IfBranch) then
          begin
           writeln;
           if not GetTPasImplElement(liie.IfBranch,lindent+1,false,false) then
             writeln(';');
          end
           else writeln(';'); //empty if then;
        end;
      end
     else if le is TPasImplCaseOf then
      begin
       lico:=TPasImplCaseOf(le);
       writeln(s1,'case ',lico.Expression,' of ');
       if assigned(lico.ElseBranch) then //workaround duplicate bug
        begin                            //reduce count of CaseOf as CaseElse is in there
         lice:=lico.ElseBranch;
         GetTPasImplBlock(TPasImplBlock(lico),lindent+1,1,false,false);
        end
          else GetTPasImplBlock(TPasImplBlock(lico),lindent+1,0,false,false); // !! else duplicate in here
       if assigned(lico.ElseBranch) then
        begin
         writeln(s,'else');//' {case}');
         lice:=lico.ElseBranch;
         GetTPasImplBlock(TPasImplBlock(lice),lindent+1,0,false,false);
        end;
       if lLastNoSem then writeln(s,'end')//' {case}')
        else writeln(s,'end;');// {case}');
       //Result:=false; ??? GetTPasImplBlock
      end
     else if le is TPasImplRepeatUntil then
      begin
        liru:=TPasImplRepeatUntil(le);
        writeln(s1,'repeat');
        GetTPasImplBlock(TPasImplBlock(liru),lindent+1,0,false,false);
        write(s,'until ',DelQuot(liru.Condition));
        if lLastNoSem then writeln
         else writeln(';');
      end
     else if le is TPasImplTryFinally then
      begin
       writeln(s,'finally');
       GetTPasImplBlock(TPasImplBlock(le),lindent+1,0,false,false);
      end
     else if le is TPasImplTryExcept then
      begin
       writeln(s,'except');
       GetTPasImplBlock(TPasImplBlock(le),lindent+1,0,false,false);
      end
     else if le is TPasImplTryExceptElse then
      begin
       writeln(s,'else');// {try}');
       GetTPasImplBlock(TPasImplBlock(le),lindent+1,0,false,false);
      end
     else if le is TPasImplTry then
      begin
       lit:=TPasImplTry(le);
       writeln(s1,'try');
       GetTPasImplBlock(TPasImplBlock(le),lindent+1,0,false,false);
       if assigned(lit.FinallyExcept) then
          GetTPasImplElement(TPasImplElement(lit.FinallyExcept),lindent+1,false,false);
       if assigned(lit.ElseBranch) then
          GetTPasImplElement(TPasImplElement(lit.ElseBranch),lindent+1,false,false);
       if lLastNoSem then writeln(s,'end')// {try} ') //there is no ImplBeginBlock
        else writeln(s,'end;');// {try} ');
      end
     else if le is TPasImplCommand then
      begin
       //ignore because empty
       // lic:=TPasImplCommand(le);
       // writeln(' CMD ',lic.Command,' ',lic.Name,' ',lic.ElementTypeName);
      end
     else if le is TPasImplLabelMark then
      begin
       writeln(s1,'label ',TPasImplLabelMark(le).LabelId,';');
      end
     else if le is TPasImplBlock then
      begin
       //IfElse, case:
       Result:=GetTPasImplBlock(TPasImplBlock(le),lindent+1,0,lLastNoSem,NoFirstIndent);
      end
     else
      begin
       Result:=false;
       //writeln(s,';');
       //writeln(' EL ',l);//,' ',le.Name)//,' ',le.ElementTypeName,' ',le.FullName);
      end;
 end;
     
// indent: indent from page left side
// DecListBy: dec(elements.count) because of case duplicate else bug
// LastNoSem: only true on last expr before else in a if clause
// NoFirstIndent: if line was started by other block like in case at -> 1:Noindent;
// Result: true if elements not empty
function GetTPasImplBlock(lb:TPasImplBlock; indent,declistby:integer;
                           LastNoSem,NoFirstIndent:boolean):boolean;
   var l,n:integer;
       lbe:TPasImplElement;
       NoSem:boolean;
       ls:String;     

begin
  Result:=false;
  NoSem:=false;
  ls:=GetIndent(indent);
  if not assigned(lb) then exit;
  //if lb is TPasImplRaise then writeln('RAISE');
  if assigned(lb.Elements) then
   begin
    if lb is TPasImplBeginBlock then
     begin
      NoSem:=LastNoSem;
      LastNoSem:=false;
      if NoFirstIndent then
       begin
        writeln('begin');////NFI');
        NoFirstIndent:=false;
       end
        else writeln(ls,'begin');
      inc(indent);
     end;

    if lb.Elements.Count >0 then
     begin
      Result:=true;
      n:=lb.Elements.Count-1;
      //workaround CaseOf duplicate bug
      if (declistby >0)and(lb.Elements.Count >declistby) then dec(n,declistby);
      for l:=0 to n do
       begin
        lbe:=TPasImplElement(lb.Elements[l]);
        //write(l:2,'/',n:2,' '); //No of curent element, max element
        if ((l = 0)and NoFirstIndent) then
         begin //index0
          if l=n then GetTPasImplElement(lbe,0,LastNoSem,false)
           else GetTPasImplElement(lbe,0,false,false)
         end
        else if l<>n then GetTPasImplElement(lbe,indent,false,false) //other index
        else GetTPasImplElement(lbe,indent,LastNoSem,false); //indexn
       end;
     end
    else
     begin //block is empty
      //write(ls,' {!EMPTY!}');
       {if not NoSem then
        begin
         if lb is TPasImplBeginBlock then writeln //empty compound need no ;
          else writeln(';')
        end
         else
          writeln;}
     end;
    if lb is TPasImplBeginBlock then
      if not NoSem then writeln(ls,'end;')// {Block}') 
        else writeln(ls,'end');// {Block}');
   end
    else
      writeln(';'); //writeln(' {!empty!};')
end;


//# Declarations (type,var,const,..)

procedure GetTPasArrayType(lpat:TPasArrayType);
 begin
  if lpat.IsPacked then write('packed ');
  write('Array');
  if lpat.IndexRange <> '' then write('[',lpat.IndexRange,']');
  if assigned(lpat.ElType) then write(' of ',lpat.ElType.Name);
   // BUG: of const missing in Procedure ConstArrayArgProc(A: Array of const); pparser: 643
 end;

//write out one variable or constant declaration, also used in types
//In spite of the use of GetPasVariables this is still used !
procedure GetTPasVar(lpv:TPasVariable; lindent:integer; NoLF:boolean);//BUG string[] pparser: 482
   var i,j:integer;
       //lppt:TPasProcedureType;
       //lpa:TPasArgument;
       //lpat:TPasArrayType;
       s,s1:string;
       prct:TPasRecordType;

  begin
   if not Assigned(lpv) then exit;
   s:=GetIndent(lindent);
   write(s,lpv.Name);//,'  ',lpv.value,' ',lpv.Modifiers,' ',lpv.AbsoluteLocation);
   if assigned(lpv.VarType) then
     begin
       //if TPasType(lpa.ArgType).ElementTypeName <>'unresolved type reference' then
       //,TPasType(lpa.ArgType).Name,' ');//,TPasType(lpa.ArgType).FullName,TPasType(lpa.ArgType).ElementTypeName)
       // PParser 2099: ArgType := nil; if IsUntyped then => Arg.ArgType := ArgType;
       //     else write(':? ');
       write(': ');
       if lpv.VarType is TPasArrayType then
        begin
         GetTPasArrayType(TPasArrayType(lpv.VarType));
        end
       else if lpv.VarType is TPasSetType then
        begin
         write('set of ',TPasSetType(lpv.VarType).EnumType.Name);
        end
       else
        begin
          if lpv.VarType is TPasPointerType then
                write('^',TPasPointerType(lpv.VarType).DestType.Name)
          else if lpv.VarType is TPasRecordType then //var record
           begin
            j:=lindent+Length(lpv.Name)+4;
            s1:=GetIndent(j);
            prct:=TPasRecordType(lpv.VarType);
            if prct.IsBitPacked then write('bitpacked ');
            if prct.IsPacked then write('packed ');
            writeln('Record');
            for i:=0 to prct.Members.Count-1 do
             begin
              GetTPasVar(TPasVariable(prct.Members[i]),j+1,false);
             end;
            write(s1,'end');
           end
          else
           begin
            write(TPasType(lpv.VarType).Name);
            //if TPasType(lpv.VarType) is TPasAliasType then write(TPasAliasType(lpv.VarType).Name);
           end;
        end;
     end;
   if lpv.Value <> '' then write('=',lpv.Value);
   if assigned(lpv.Expr) then // var ?, const AnArrayConst : Array[1..3] of Integer = (1,2,3);
     begin
      write('=');
      GetTPasExpr(lpv.Expr);
     end;
     
   if lpv.Modifiers <>'' then //Modifiers starts with ;
    begin
     write(' ',lpv.Modifiers,';');
     if GetTPasMemberHints(lpv.Hints) then write(';');
    end
   else
    begin
     GetTPasMemberHints(lpv.Hints);
     write(';');
    end;
   if not NoLF then writeln;
  end;
  
//write out a list of variables only
//more compact than the output of seperate calls of GetTPasVar
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure GetPasVariables(vl:TFPList; lindent:integer; NoLF,NoSEM:boolean);
=======
procedure GetPasVariables(vl:TList; lindent:integer; NoLF,NoSEM:boolean);
>>>>>>> graemeg/cpstrnew
=======
procedure GetPasVariables(vl:TList; lindent:integer; NoLF,NoSEM:boolean);
>>>>>>> graemeg/cpstrnew
=======
procedure GetPasVariables(vl:TList; lindent:integer; NoLF,NoSEM:boolean);
>>>>>>> graemeg/cpstrnew
=======
procedure GetPasVariables(vl:TList; lindent:integer; NoLF,NoSEM:boolean);
>>>>>>> origin/cpstrnew
   var v,i,j:integer;
       s,s1:string;
       prct:TPasRecordType;
       lpv:TPasVariable;
       
       same:boolean;
       samestr,tmpstr:Ansistring;
       samevar:array of integer;
       svi:integer;

  begin
   if vl.Count <= 0 then exit; 
   s:=GetIndent(lindent);
   //> compare all variable types as string to find the ones with same type
   samestr:='';
   svi:=0;
   SetLength(samevar,vl.count);
   for v:=0 to vl.count-1 do
    begin
     tmpstr:=''; 
     same:=true;   
     lpv:=TPasVariable(vl[v]);
     //write(s,lpv.Name);
     if assigned(lpv.VarType) then
      begin
       tmpstr:=tmpstr+': ';
       if lpv.VarType is TPasArrayType then
         begin
          //GetTPasArrayType(TPasArrayType(lpv.VarType));
          tmpstr:=tmpstr+'array'+TPasArrayType(lpv.VarType).IndexRange;
          if assigned(TPasArrayType(lpv.VarType).ElType) then
           tmpstr:=tmpstr+TPasArrayType(lpv.VarType).ElType.Name;
         end
       else if lpv.VarType is TPasSetType then
         begin
          tmpstr:=tmpstr+'set of '+TPasSetType(lpv.VarType).EnumType.Name;
         end
       else
        begin
         if lpv.VarType is TPasPointerType then
            tmpstr:=tmpstr+'^'+TPasPointerType(lpv.VarType).DestType.Name
         else if lpv.VarType is TPasRecordType then //var record
           begin
            prct:=TPasRecordType(lpv.VarType);
            if prct.IsBitPacked then tmpstr:=tmpstr+'bitpacked ';
            if prct.IsPacked then tmpstr:=tmpstr+'packed ';
            tmpstr:=tmpstr+'Record ';
            for i:=0 to prct.Members.Count-1 do
             begin
              //todo
              //GetTPasVar(TPasVariable(prct.Members[i]),j+1,false);
             end;
            tmpstr:=tmpstr+'end';
           end
         else
          begin
            tmpstr:=tmpstr+TPasType(lpv.VarType).Name;
          end;
        end;
      end
        else same:=false;
     if lpv.Value <> '' then same:=false;//=
     if assigned(lpv.Expr) then // var ?, const AnArrayConst : Array[1..3] of Integer = (1,2,3);
      begin
       same:=false;//=
      end;
     if lpv.Modifiers <>'' then //Modifiers starts with ;
      begin
       tmpstr:=tmpstr+' '+lpv.Modifiers+';';
       tmpstr:=tmpstr+ReturnTPasMemberHints(lpv.Hints);
      end
     else
      begin
       tmpstr:=tmpstr+ReturnTPasMemberHints(lpv.Hints);
      end;
  //if v = 0 then begin samestr:=tmpstr; end;
     if (not same)or(samestr <> tmpstr) then
       begin
        samestr:=tmpstr;
        inc(svi);
       end;
     samevar[v]:=svi;
   end;
   //compare <
   //now print them
   svi:=-1; 
   for v:=0 to vl.count-1 do
    begin
     lpv:=TPasVariable(vl[v]);
     if not Assigned(lpv) then continue;
     if svi <> samevar[v] then
       begin
        svi:=samevar[v];
        if v>0 then writeln;
        write(s,lpv.Name);//variblenname
       end
        else write(lpv.Name);
     if (v < vl.Count-1)and(samevar[v+1]=svi) then write(',')
      else
       begin
        if assigned(lpv.VarType) then
         begin
           write(': ');
           if lpv.VarType is TPasArrayType then
            begin
             GetTPasArrayType(TPasArrayType(lpv.VarType));
            end
           else if lpv.VarType is TPasSetType then
            begin
             write('set of ',TPasSetType(lpv.VarType).EnumType.Name);
            end
           else
            begin
             if lpv.VarType is TPasPointerType then
                write('^',TPasPointerType(lpv.VarType).DestType.Name)
             else if lpv.VarType is TPasRecordType then //var record
              begin
               j:=lindent+Length(lpv.Name)+4;
               s1:=GetIndent(j);
               prct:=TPasRecordType(lpv.VarType);
               if prct.IsBitPacked then write('bitpacked ');
               if prct.IsPacked then write('packed ');
               writeln('Record');
               {for i:=0 to prct.Members.Count-1 do
                 begin
                  GetTPasVar(TPasVariable(prct.Members[i]),j+1,false);
                 end;}
               if prct.Members.Count > 0 then
                 GetPasVariables(prct.Members,j+1,false,false);
               write(s1,'end');
              end
             else
              begin
               write(TPasType(lpv.VarType).Name);
              end;
            end;
         end;
        if lpv.Value <> '' then write('=',lpv.Value);
        if assigned(lpv.Expr) then // var ?, const AnArrayConst : Array[1..3] of Integer = (1,2,3);
         begin
          write('=');
          GetTPasExpr(lpv.Expr);
         end;

        if lpv.Modifiers <>'' then //Modifiers starts with ;
         begin
          write(' ',lpv.Modifiers,';');
          if GetTPasMemberHints(lpv.Hints) then write(';');
         end
        else
         begin
          GetTPasMemberHints(lpv.Hints);
          if (v < vl.Count-1) then write(';')
            else if (not NoSEM) then write(';');
         end;
	   //if not NoLF then writeln;
       end;
     end;
    if not NoLF then writeln;
  end;  
  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function GetTPasArgumentAccess(acc:TArgumentAccess):String;

begin
  Result:='';
  case acc of
    //argDefault:Result:='default'; //normal proccall is default
    argConst:Result:='const';
    argVar:Result:='var';
    argOut:Result:='out';
  end;
end;

procedure GetTPasProcedureType(lppt:TPasProcedureType; indent:integer);

Var
  l : integer;
  lpa:TPasArgument;
  samevar:array of integer;//same index same type
  aktaa:TArgumentAccess;
  svi:integer;
  same:boolean;
  aktname,tmpname:String;

begin
  if assigned(lppt.Args) and (lppt.Args.Count > 0) then
    begin
    write('(');
    if lppt.Args.Count > 0 then
     begin
      //produce more compact output than the commented block below
      //>find same declaration
      //look ahead what is the same
      SetLength(samevar,lppt.Args.Count);
      svi:=0;
      aktname:='';
      for l:=0 to lppt.Args.Count-1 do
       begin
        same:=true;
        tmpname:='';
        lpa:=TPasArgument(lppt.Args.Items[l]);
        if assigned(lpa.ArgType) then
         begin
          if lpa.ArgType is TPasArrayType then
           begin
             if assigned(TPasArrayType(lpa.ArgType).ElType) then tmpname:=TPasArrayType(lpa.ArgType).ElType.Name;
           end
            else tmpname:=TPasType(lpa.ArgType).Name;
         end;
        if l=0 then begin aktaa:=lpa.Access; aktname:=tmpname; end;
        if lpa.Access <> aktaa then begin same:=false; aktaa:=lpa.Access; end;//access type
        if (tmpname = '')or(tmpname <> aktname) then begin same:=false; aktname:=tmpname; end;//type name
        if lpa.Value <> '' then same:=false;//var=value
        if not same then inc(svi);
        samevar[l]:=svi;
       end;
     //find same declaration<
     svi:=-1;
     same:=false;
     for l:=0 to lppt.Args.Count-1 do
      begin
       lpa:=TPasArgument(lppt.Args.Items[l]);
       if svi <> samevar[l] then
        begin
         svi:=samevar[l];
         if lpa.Access <> argDefault then write(GetTPasArgumentAccess(lpa.Access),' ');
         write(lpa.Name);//variblenname
        end
          else write(lpa.Name);
       if (l < lppt.Args.Count-1)and(samevar[l+1]=svi) then write(',')
        else
         begin
          if assigned(lpa.ArgType) then
           begin
            write(': ');
            if lpa.ArgType is TPasArrayType then
             GetTPasArrayType(TPasArrayType(lpa.ArgType))
              else write(TPasType(lpa.ArgType).Name);
           end;
          if lpa.Value <> '' then write('=',lpa.Value);
          if l< lppt.Args.Count-1 then write('; ');
        end;
      end;
    write(')');
    end;
    end;
  if (lppt is TPasFunctionType) then
      write(': ',TPasFunctionType(lppt).ResultEl.ResultType.Name);
  if lppt.IsOfObject then
    write(' of Object');
end;
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

procedure GetTypes(pe:TPasElement; lindent:integer);
  var i,j,k:integer;
      s,s1,s2:string;
      pet:TPasEnumType;
      pev:TPasEnumValue;

      prt:TPasRangeType;
      prct:TPasRecordType;
      pv:TPasVariant;
      pst:TPasSetType;


  function GetVariantRecord(pe:TPasElement; lindent:integer):boolean;
    var i,j,k:integer;
        prct:TPasRecordType;
        pv:TPasVariant;
        s,s1:string;

   begin
    Result:=false;
    j:=lindent+Length(pe.Name)+2;
    s:=GetIndent(lindent);
    s1:=GetIndent(lindent+2);
    prct:=TPasRecordType(pe);
    {Now i use GetPasVariables for more compact output
     for i:=0 to prct.Members.Count-1 do
     begin
      GetTPasVar(TPasVariable(prct.Members[i]),1,true);
     end;}
    if prct.Members.Count > 0 then GetPasVariables(prct.Members,1,true,true);  
    if assigned(prct.Variants) then
     begin
      Result:=true;
      writeln(';');
      write(s,'case ');
      if prct.VariantName <>'' then write(prct.VariantName,'=');
      write(TPasType(prct.VariantType).Name);
      writeln(' of');
      if assigned(prct.Variants)then
       if prct.Variants.Count >0 then
        begin
         for i:=0 to prct.Variants.Count-1 do
          begin
           pv:=TPasVariant(prct.Variants[i]);
           write(s1,pv.Name);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
           for k:=0 to pv.Values.Count-1 do write(TPasElement(pv.Values[k]).GetDeclaration(true));
=======
           for k:=0 to pv.Values.Count-1 do write(pv.Values[k]);
>>>>>>> graemeg/cpstrnew
=======
           for k:=0 to pv.Values.Count-1 do write(pv.Values[k]);
>>>>>>> graemeg/cpstrnew
=======
           for k:=0 to pv.Values.Count-1 do write(pv.Values[k]);
>>>>>>> graemeg/cpstrnew
=======
           for k:=0 to pv.Values.Count-1 do write(pv.Values[k]);
>>>>>>> origin/cpstrnew
           write(': (');
           if GetVariantRecord(TPasElement(pv.Members),j+1) then
             writeln(s1,');')
              else writeln(');');
          end;
        end;
     end;
   end;

 begin
  s:=GetIndent(lindent);
  write(s,pe.Name,'=');
  if pe is TPasArrayType then
   begin
     GetTPasArrayType(TPasArrayType(pe));
     writeln(';');
   end
  else if pe is TPasEnumType then
   begin
    pet:=TPasEnumType(pe);
    write('(');
    if pet.Values.Count > 0 then
     begin
      for j:=0 to pet.Values.Count-2 do
       begin
        pev:=TPasEnumValue(pet.Values[j]);
        write(pev.name,',');
        //pev.Value ?
        //pev.AssignedValue ?
        //pev.IsValueUsed ?
       end;
      pev:=TPasEnumValue(pet.Values[pet.Values.Count-1]);
      write(pev.name);
     end;
    writeln(');');
   end
  else if pe is TPasFileType then
   begin
    writeln('file of ',TPasFileType(pe).ElType.Name,';');
   end
  else if pe is TPasProcedureType then
   begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   if pe is TPasFunctionType then
     Write('function ')
   else
     Write('procedure ');
   GetTPasProcedureType(TPasProcedureType(pe), lindent);
   Writeln(';');
=======
    writeln('procedure');
>>>>>>> graemeg/cpstrnew
=======
    writeln('procedure');
>>>>>>> graemeg/cpstrnew
=======
    writeln('procedure');
>>>>>>> graemeg/cpstrnew
=======
    writeln('procedure');
>>>>>>> origin/cpstrnew
   end
  else if pe is TPasPointerType then
   begin
    //writeln('pointer');
    writeln('^',TPasPointerType(pe).DestType.Name,';');
   end
  else if pe is TPasRangeType then
   begin
    prt:=TPasRangeType(pe);
    writeln(prt.RangeStart,'..',prt.RangeEnd,';');
   end
  else if pe is TPasRecordType then
   begin
    j:=lindent+Length(pe.Name)+2;
    s1:=GetIndent(j);
    s2:=GetIndent(j+1);
    prct:=TPasRecordType(pe);
    if prct.IsBitPacked then write('bitpacket ');
    if prct.IsPacked then write('packet');
    writeln('record');
    {Now i use GetPasVariables for more compact output
     for i:=0 to prct.Members.Count-1 do
     begin
      GetTPasVar(TPasVariable(prct.Members[i]),j+1,false);
     end;}
    GetPasVariables(prct.Members,j+2,false,false);
    if assigned(prct.Variants) then
     begin
      write(s1,'case ');
      if prct.VariantName <>'' then write(prct.VariantName,'=');
        write(TPasType(prct.VariantType).Name);
      writeln(' of');
      if assigned(prct.Variants)then
       if prct.Variants.Count >0 then
        begin
         for i:=0 to prct.Variants.Count-1 do
          begin
           pv:=TPasVariant(prct.Variants[i]);
           write(s2,pv.Name);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
           for k:=0 to pv.Values.Count-1 do write(TPasElement(pv.Values[k]).GetDeclaration(true));
=======
           for k:=0 to pv.Values.Count-1 do write(pv.Values[k]);
>>>>>>> graemeg/cpstrnew
=======
           for k:=0 to pv.Values.Count-1 do write(pv.Values[k]);
>>>>>>> graemeg/cpstrnew
=======
           for k:=0 to pv.Values.Count-1 do write(pv.Values[k]);
>>>>>>> graemeg/cpstrnew
=======
           for k:=0 to pv.Values.Count-1 do write(pv.Values[k]);
>>>>>>> origin/cpstrnew
           write(': (');
           if GetVariantRecord(TPasElement(pv.Members),j+2) then
             writeln(s2,');')
              else writeln(');');
          end;
        end;
     end;
    writeln(s1,'end;');
   end
  else if pe is TPasSetType then
   begin
    pst:=TPasSetType(pe);
    writeln('set of ',pst.EnumType.Name,';');
   end
  else if pe is TPasClassOfType then writeln('Class of ',TPasClassOfType(pe).DestType.Name,';')
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  else if pe is tPasAliasType then
    begin
    pe:=tPasAliasType(PE).DestType;
    write(PE.name);
    if pe is tPasStringType then
      begin
      if (TPasStringType(PE).LengthExpr<>'') then
        Write('[',TPasStringType(PE).LengthExpr,']');
      end;
    Writeln(';');
    end
  else if pe is tPasUnresolvedTypeRef then writeln(TPasUnresolvedTypeRef(PE).name,';')
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  else
   begin
    
    writeln('{ Unknown TYPE(s): ');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    writeln(s,pe.Name,' ',pe.classname);
=======
    writeln(s,pe.Name);
>>>>>>> graemeg/cpstrnew
=======
    writeln(s,pe.Name);
>>>>>>> graemeg/cpstrnew
=======
    writeln(s,pe.Name);
>>>>>>> graemeg/cpstrnew
=======
    writeln(s,pe.Name);
>>>>>>> origin/cpstrnew
    writeln('}');
    writeln;
   end;
 end;


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
 function GetTPasArgumentAccess(acc:TArgumentAccess):String;
  begin
   Result:='';
   case acc of
     //argDefault:Result:='default'; //normal proccall is default
     argConst:Result:='const';
     argVar:Result:='var';
     argOut:Result:='out';
   end;
  end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

 procedure GetTCallingConvention(cc:TCallingConvention);  //TODO: test it
  begin
   case cc of
     //ccDefault:write(' default;'); //normal proccall is default
     ccRegister:WriteFmt(true,'Register;',false);
     ccPascal  :WriteFmt(true,'Pascal;',false);
     ccCDecl   :WriteFmt(true,'CDecl;',false);
     ccStdCall :WriteFmt(true,'StdCall;',false);
     ccOldFPCCall:WriteFmt(true,'OldFPCall;',false);
     ccSafeCall:WriteFmt(true,'SaveCall;',false);
   end;
  end;
  
 procedure GetHiddenModifiers(Mfs:TProcedureModifiers);
  begin
   if pmInline in Mfs then WriteFmt(true,'inline;',false);
   if pmAssembler in Mfs then WriteFmt(true,'assembler;',false);
   if pmVarargs in Mfs then WriteFmt(true,'varargs;',false);
   if pmCompilerProc in Mfs then WriteFmt(true,'compilerproc;',false);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
   if pmExtdecl in Mfs then WriteFmt(true,'extdecl;',false);
>>>>>>> graemeg/cpstrnew
=======
   if pmExtdecl in Mfs then WriteFmt(true,'extdecl;',false);
>>>>>>> graemeg/cpstrnew
=======
   if pmExtdecl in Mfs then WriteFmt(true,'extdecl;',false);
>>>>>>> graemeg/cpstrnew
=======
   if pmExtdecl in Mfs then WriteFmt(true,'extdecl;',false);
>>>>>>> origin/cpstrnew
  end; 

  procedure GetTPasProcedure(lpp:TPasProcedure; indent:integer);
   var l:integer;
       lppt:TPasProcedureType;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
       s:String;
       
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
       lpa:TPasArgument;
       s:String;
       
       same:boolean;
       samevar:array of integer;//same index same type
       aktaa:TArgumentAccess;
       aktname,tmpname:String;
       svi:integer;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

  begin
   if not Assigned(lpp) then exit;
   s:=GetIndent(indent);
   if lpp is TPasConstructor then write(s,'Constructor ')
    else if TPasElement(lpp) is TPasConstructorImpl then write(s,'Constructor ')
    else if lpp is TPasDestructor then write(s,'Destructor ')
    else if TPasElement(lpp) is TPasDestructorImpl then write(s,'Destructor ')
    else if lpp is TPasClassProcedure then write(s,'Class Procedure ') //pparser.pp: 3221
    else if lpp is TPasClassFunction then write(s,'Class Function ')
    else if lpp is TPasFunction then write(s,'Function ')
      else write(s,'Procedure ');
   write(lpp.Name);//,' ',lpp.TypeName);
   if assigned(lpp.ProcType) then
    begin
     lppt:=lpp.ProcType;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     GetTPasProcedureType(lppt,Indent);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
     if assigned(lppt.Args) and (lppt.Args.Count > 0) then
      begin
       write('(');
       if lppt.Args.Count > 0 then 
        begin
         //produce more compact output than the commented block below
         //>find same declaration
         //look ahead what is the same
         SetLength(samevar,lppt.Args.Count);
         svi:=0;
         aktname:='';
         for l:=0 to lppt.Args.Count-1 do
          begin
           same:=true;
           tmpname:='';
           lpa:=TPasArgument(lppt.Args.Items[l]);
           if assigned(lpa.ArgType) then
            begin
             if lpa.ArgType is TPasArrayType then
              begin
                if assigned(TPasArrayType(lpa.ArgType).ElType) then tmpname:=TPasArrayType(lpa.ArgType).ElType.Name;
              end
               else tmpname:=TPasType(lpa.ArgType).Name;
            end;
           if l=0 then begin aktaa:=lpa.Access; aktname:=tmpname; end;   
           if lpa.Access <> aktaa then begin same:=false; aktaa:=lpa.Access; end;//access type 
           if (tmpname = '')or(tmpname <> aktname) then begin same:=false; aktname:=tmpname; end;//type name
           if lpa.Value <> '' then same:=false;//var=value
           if not same then inc(svi); 
           samevar[l]:=svi;
          end; 
        //find same declaration<  
        svi:=-1;
        same:=false;
        for l:=0 to lppt.Args.Count-1 do
         begin
          lpa:=TPasArgument(lppt.Args.Items[l]);
          if svi <> samevar[l] then
           begin
            svi:=samevar[l];
            if lpa.Access <> argDefault then write(GetTPasArgumentAccess(lpa.Access),' '); 
            write(lpa.Name);//variblenname
           end
             else write(lpa.Name); 
          if (l < lppt.Args.Count-1)and(samevar[l+1]=svi) then write(',')
           else
            begin
             if assigned(lpa.ArgType) then
              begin
               write(': ');
               if lpa.ArgType is TPasArrayType then
                GetTPasArrayType(TPasArrayType(lpa.ArgType))
                 else write(TPasType(lpa.ArgType).Name);
              end;
             if lpa.Value <> '' then write('=',lpa.Value);
             if l< lppt.Args.Count-1 then write('; ');
           end;    
         end; 
       {//simple version duplicates declarations of same type
        for l:=0 to lppt.Args.Count-1 do
        begin
         lpa:=TPasArgument(lppt.Args.Items[l]);
          if lpa.Access <> argDefault then write(GetTPasArgumentAccess(lpa.Access),' '); 
         write(lpa.Name);//variblenname
         if assigned(lpa.ArgType) then
          begin
           //if TPasType(lpa.ArgType).ElementTypeName <>'unresolved type reference' then
           //,TPasType(lpa.ArgType).Name,' ');
           //,TPasType(lpa.ArgType).FullName,TPasType(lpa.ArgType).ElementTypeName)
           // PParser 2099: ArgType := nil; if IsUntyped then => Arg.ArgType := ArgType;
           //     else write(':? ');
           write(': ');
           if lpa.ArgType is TPasArrayType then
            begin
             GetTPasArrayType(TPasArrayType(lpa.ArgType));
            end
             else  write(TPasType(lpa.ArgType).Name);
          end;
         if lpa.Value <> '' then write('=',lpa.Value);
         if l< lppt.Args.Count-1 then write('; ');
        end;}
        end;
       write(')');
      end;
     if lppt.IsOfObject then write(' of Object'); 
     if (TPasElement(lpp) is TPasFunction)or(TPasElement(lpp) is TPasClassFunction) then 
         write(': ',TPasFunctionType(lpp.ProcType).ResultEl.ResultType.Name);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    end;
   //writeln(';');
   WriteFmt(false,'',true);
   if lpp.IsVirtual then WriteFmt(true,'virtual;',false);
   if lpp.IsOverload then WriteFmt(true,'overload;',false);
   if lpp.IsAbstract then WriteFmt(true,'abstract;',false);
   if lpp.IsDynamic then WriteFmt(true,'dynamic;',false);
   if lpp.IsOverride then WriteFmt(true,'override;',false);
   if lpp.IsExported then WriteFmt(true,'exported;',false);
   if lpp.IsExternal then WriteFmt(true,'external;',false);
   //pparser 2360: everyting behind external is ignored !!!
   if lpp.IsMessage then
    begin
      write('message ');
      if lpp.MessageType = pmtString then writeln(false,lpp.MessageName,true)
       else WriteFmt(false,lpp.MessageName,true);//pmtInteger
    end;
   if lpp.IsReintroduced then WriteFmt(true,'reintroduce;',false);
   if lpp.IsStatic then WriteFmt(true,'static;',false);
   if lpp.IsForward then WriteFmt(true,'forward;',false);
   GetHiddenModifiers(lpp.Modifiers);
   GetTCallingConvention(lpp.CallingConvention);
   if GetTPasMemberHints(TPasElement(lpp).Hints) then WriteFmt(false,'',true); //BUG ? missing hints
   if not Unformated then writeln;
  end;

 procedure GetTPasProcedureBody(pb:TProcedureBody; indent:integer);
   var j:integer;
       pd:TPasDeclarations;
       pib:TPasImplBlock;
  begin
   if assigned(pb) then
    begin
     if assigned(pb.Body)then
      begin
       if assigned(TPasDeclarations(pb).Functions)then
        begin
         pd:=TPasDeclarations(pb);
         if isim then
          begin
           //writeln;
           GetDecls(pd,indent+1);     //~recursion
           //PrintDecls(pd,indent+1); //~recursion
          end
         else
          if pd.Functions.Count >0 then //sub-functions
           begin
            for j:=0 to pd.Functions.Count-1 do
              GetTPasProcedure(TPasProcedure(pd.Functions[j]),indent+1);
           end;
        end;
       pib:=TPasImplBlock(pb.Body);
       if assigned(pib) then
        begin
          GetTPasImplBlock(pib,indent,0,false,false); //indent depend on sub function level
          if not Unformated then writeln; //('//block');
        end;
      end;
    end;
  end;

 procedure GetTpasOverloadedProc(pop:TPasOverloadedProc; indent:integer);
   var pp:TPasProcedure;
       j:integer;
  begin
   if assigned(pop) then
    begin
     if pop.Overloads.Count >0 then
      begin
       for j:=0  to pop.Overloads.Count-1 do
        begin
         pp:=TPasProcedure(pop.Overloads[j]);
         GetTPasProcedure(pp,indent);
         GetTPasProcedureBody(pp.Body,indent);
        end;
      end;
    end;
  end;

 function GetVisibility(v:TPasMemberVisibility):String;
  begin
   Result:='';
   case v of
    //visDefault:Result:='default';
    visPrivate:Result:='private';
    visProtected:Result:='protected';
    visPublic:Result:='public';
    visPublished:Result:='published';
    visAutomated:Result:='automated';
    visStrictPrivate:Result:='strictprivate';
    visStrictProtected:Result:='strictprotected';
   end;
  end;

 procedure GetTPasClass(pc:TPasClassType; indent:integer);
   var j,l:integer;
       s,s1,s2:String;
       lpe:TPasElement;
       lpp:TPasProperty;
       lpa:TPasArgument;
       vis:TPasMemberVisibility;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
       vars:TFPList;
=======
       vars:TList;
>>>>>>> graemeg/cpstrnew
=======
       vars:TList;
>>>>>>> graemeg/cpstrnew
=======
       vars:TList;
>>>>>>> graemeg/cpstrnew
=======
       vars:TList;
>>>>>>> origin/cpstrnew
       IsVar:boolean;

  procedure PrintVars;
   begin
    if vars.Count > 0 then GetPasVariables(vars,indent+1,false,false);
    IsVar:=False;
    vars.Clear;
   end;

  begin
   if assigned(pc) then
    begin
     s:=GetIndent(indent);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     if (pc.ObjKind=okGeneric) then
       begin
       write(s,'generic ',pc.Name);
       for l:=0 to pc.GenericTemplateTypes.Count-1 do
          begin
          if l=0 then
           Write('<')
          else
           Write(',');
          Write(TPasGenericTemplateType(pc.GenericTemplateTypes[l]).Name);
          end;
       Write('> = ');
       end
     else
       write(s,pc.Name,' = ');
=======
     write(s,pc.Name,'=');
>>>>>>> graemeg/cpstrnew
=======
     write(s,pc.Name,'=');
>>>>>>> graemeg/cpstrnew
=======
     write(s,pc.Name,'=');
>>>>>>> graemeg/cpstrnew
=======
     write(s,pc.Name,'=');
>>>>>>> origin/cpstrnew
     if pc.IsPacked then write('packed ');
     case pc.ObjKind of
      okObject:write('Object');
      okClass:write('Class');
      okInterface:write('Interface');
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      okGeneric:write('class');
      okspecialize : write('specialize');
     end;
     if assigned(pc.AncestorType) and (pc.AncestorType.ElementTypeName <> '') then
        begin
        if pc.ObjKind<>okspecialize then
          write('(',pc.AncestorType.Name,')')
        else
          begin
          write(' ',pc.AncestorType.Name);
          for l:=0 to pc.GenericTemplateTypes.Count-1 do
           begin
           if l=0 then
            Write('<')
           else
            Write(',');
           Write(TPasGenericTemplateType(pc.GenericTemplateTypes[l]).Name);
           end;
          Write('>');
          end;
        end;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
     end;
     if assigned(pc.AncestorType) and (pc.AncestorType.ElementTypeName <> '') then
        write('(',pc.AncestorType.Name,')');

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
     if pc.IsForward or pc.IsShortDefinition then //pparser.pp: 3417 :class(anchestor); is allowed !
      begin
       writeln(';');
       exit;
      end;  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    //Members: TFPList;
    //InterfaceGUID: String;
    //ClassVars: TFPList; //is this always empty ?
    //Modifiers: TStringList;
    //Interfaces: TFPList;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    //Members: TList;
    //InterfaceGUID: String;
    //ClassVars: TList; //is this always empty ?
    //Modifiers: TStringList;
    //Interfaces: TList;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
      s1:=GetIndent(indent+1);
      s2:=GetIndent(indent+2);
      if pc.Members.Count > 0 then
       begin
        writeln;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        vars:=TFPList.Create;
=======
        vars:=TList.Create;
>>>>>>> graemeg/cpstrnew
=======
        vars:=TList.Create;
>>>>>>> graemeg/cpstrnew
=======
        vars:=TList.Create;
>>>>>>> graemeg/cpstrnew
=======
        vars:=TList.Create;
>>>>>>> origin/cpstrnew
        IsVar:=false;
        for j:=0 to pc.Members.Count-1 do
         begin
           lpe:=TPasElement(pc.Members[j]);

           //Class visibility, written on change
           if j=0 then
            begin
             vis:=lpe.Visibility;
             if GetVisibility(vis) <> '' then writeln(s1,GetVisibility(vis));
            end
           else
            if vis <> lpe.Visibility then
             begin
              if IsVar then PrintVars;
              if lpe.Visibility <> visDefault then //Class Function = visDefault
               begin
                vis:=lpe.Visibility;
                if GetVisibility(vis) <> '' then writeln(s1,GetVisibility(vis));
               end;
             end;

           if lpe is TPasOverloadedProc then
            begin
             if IsVar then PrintVars;
             GetTPasOverloadedProc(TPasOverloadedProc(lpe),indent+2);
            end
           else if lpe is TPasProcedure then //TPasClassProcedure and
            begin         //TPasClassFunction are both child of TPasProcedure
             if IsVar then PrintVars;
             GetTPasProcedure(TPasProcedure(lpe),indent+2);
            end
           else if lpe is TPasProperty then
            begin
             if IsVar then PrintVars;
             lpp:=TPasProperty(lpe);
             write(s2,'property ',lpp.Name);
             if lpp.Args.Count >0 then
              begin
               for l:=0 to lpp.Args.Count-1 do
                begin
                 lpa:=TPasArgument(lpp.Args.Items[l]);
                 if GetTPasArgumentAccess(lpa.Access) <> '' then
                   write('[',GetTPasArgumentAccess(lpa.Access),' ',lpa.Name)
                    else write('[',lpa.Name); //variblename
                 if assigned(lpa.ArgType) then
                  begin
                   //if TPasType(lpa.ArgType).ElementTypeName <>'unresolved type reference' then
                   //,TPasType(lpa.ArgType).Name,' ');
                   //,TPasType(lpa.ArgType).FullName,TPasType(lpa.ArgType).ElementTypeName)
                   // PParser 2099: ArgType := nil; if IsUntyped then => Arg.ArgType := ArgType;
                   //     else write(':? ');
                   write(': ');
                   if lpa.ArgType is TPasArrayType then
                    begin
                     GetTPasArrayType(TPasArrayType(lpa.ArgType));
                    end
                     else  write(TPasType(lpa.ArgType).Name);
                  end;
                 if lpa.Value <> '' then write('=',lpa.Value);
                 write(']');
                end;
              end;//args
             if assigned(lpp.VarType) then
              begin
               write(': ',TPasType(lpp.VarType).Name);
              end;
             if lpp.IndexValue <> '' then write(' Index ',lpp.IndexValue);
             if lpp.ReadAccessorName <> '' then write(' Read ',lpp.ReadAccessorName);
             if lpp.WriteAccessorName <> '' then write(' Write ',lpp.WriteAccessorName);
             if lpp.ImplementsName <> '' then write(' Implements ',lpp.ImplementsName);
             if lpp.IsDefault then write(' Default ',lpp.DefaultValue);
             if lpp.IsNodefault then write(' NoDefault');
             if lpp.StoredAccessorName <> '' then write(' Stored ',lpp.StoredAccessorName);
             GetTPasMemberHints(lpp.Hints);
             writeln(';');
            end
           else if lpe is TPasVariable then
            begin
             //this is done with printvars
             //GetTPasVar(TPasVariable(lpe),indent+1,false);
             IsVar:=true;
             vars.add(lpe);
            end
           else
            begin
             if IsVar then PrintVars;
             writeln('{ Unknown Declaration(s) in Class/Object/Interface: ');
             writeln(s,lpe.Name);
             writeln('}');
            end;
         end;
        //writeln(s,'end;');//'//class');
         if IsVar then PrintVars;
         vars.free;
       end
        else  writeln;//(';'); //x=class(y);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
     writeln(s,'end;');
    end;
  end;


procedure GetDecls(Decl:TPasDeclarations; indent:integer);
 var i,j:integer;
     pe:TPasElement;
     pp:TPasProcedure;
     ps:TPasSection;
     s:string;
     x:(None,ResStrings,Types,Consts,Classes,Functions,Variables,Properties);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
     l:TFPList;
=======
     l:TList;
>>>>>>> graemeg/cpstrnew
=======
     l:TList;
>>>>>>> graemeg/cpstrnew
=======
     l:TList;
>>>>>>> graemeg/cpstrnew
=======
     l:TList;
>>>>>>> origin/cpstrnew

  procedure PrintVars;
   begin
    if l.Count > 0 then GetPasVariables(l,indent+1,false,false);
   end;

begin
 s:=GetIndent(indent);
 x:=None;
 if assigned(Decl)then
  begin
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
   l:=TFPList.Create;
=======
   l:=TList.Create;
>>>>>>> graemeg/cpstrnew
=======
   l:=TList.Create;
>>>>>>> graemeg/cpstrnew
=======
   l:=TList.Create;
>>>>>>> graemeg/cpstrnew
=======
   l:=TList.Create;
>>>>>>> origin/cpstrnew
   pe:=TPasElement(Decl);
   if pe is TPasSection then
    begin
     {(Decl is TInterfaceSection)or(Decl is TImplementationSection) or
     (Decl is TProgramSection}
     ps:=TPasSection(pe);
     if ps.UsesList.Count >0 then
      begin
       write(s,'uses ');
       ps:=TPasSection(Decl);
       if not Unformated then begin writeln; write(s,'  '); end;
       for i:=0 to ps.UsesList.Count-2 do
        if UpCase(TPasElement(ps.UsesList[i]).Name) = 'SYSTEM' then continue //do not print system
         else write(TPasElement(ps.UsesList[i]).Name,',');                   //as it is added by parser
       writeln(TPasElement(ps.UsesList[ps.UsesList.Count-1]).Name,';');
       if not Unformated then writeln;
      end;
    end;
   if assigned(Decl.Declarations)and(Decl.Declarations.Count > 0) then
   for j:=0 to Decl.Declarations.Count-1 do
    begin
     pe:=TPasElement(Decl.Declarations[j]);
     if pe is TPasResString then
       begin
        if x = Variables then PrintVars;
        if x <> ResStrings then
         begin
          if not Unformated then writeln;
          writeln(s,'ResourceString');
          x:=ResStrings;
         end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        writeln(s,pe.Name,'=',DelQuot(TPasResString(pe).Expr.GetDeclaration(false)),';'); //too much '''
=======
        writeln(s,pe.Name,'=',DelQuot(TPasResString(pe).Value),';'); //too much '''
>>>>>>> graemeg/cpstrnew
=======
        writeln(s,pe.Name,'=',DelQuot(TPasResString(pe).Value),';'); //too much '''
>>>>>>> graemeg/cpstrnew
=======
        writeln(s,pe.Name,'=',DelQuot(TPasResString(pe).Value),';'); //too much '''
>>>>>>> graemeg/cpstrnew
=======
        writeln(s,pe.Name,'=',DelQuot(TPasResString(pe).Value),';'); //too much '''
>>>>>>> origin/cpstrnew
       end
     else if pe is TPasConst then
       begin
        if x = Variables then PrintVars;
        if x <> Consts then
         begin
          if not Unformated then writeln;
          writeln(s,'const');
          x:=Consts;
         end;
        GetTPasVar(TPasVariable(pe),indent+1,false);
       end
     else if pe is TPasVariable then
       begin
        if x <> Variables then
         begin
          if not Unformated then writeln;
          writeln(s,'var');
          x:=Variables;
          l.Clear;
         end;
        l.Add(pe);
        //this is done with printvars
        //GetTPasVar(TPasVariable(pe),indent+1,false);
       end
     else if pe is TPasClassType then
       begin
        if x = Variables then PrintVars;
        if x <> Types then
         begin
          if not Unformated then writeln;
          writeln(s,'Type');
          x:=Types;
         end;
        GetTPasClass(TPasClassType(pe),indent+1);
       end
     else if pe is TPasType then
       begin
        if x = Variables then PrintVars;
        if x <> Types then
         begin
          if not Unformated then writeln;
          writeln(s,'Type');
          x:=Types;
         end;
        GetTypes(TPasElement(pe),indent+1);
       end
     else if pe is TPasProcedureBase then
       begin
        if x = Variables then PrintVars;
        if (x <> Functions)and not Unformated then writeln;
        x:=Functions;
        if pe is TPasOverloadedProc then
          begin
           GetTpasOverloadedProc(TPasOverloadedProc(pe),indent);
          end
         else
          begin
           pp:=TPasProcedure(pe);
           GetTPasProcedure(pp,indent);
           GetTPasProcedureBody(pp.Body,indent);
          end;
       end
      else
       begin
        if x = Variables then PrintVars;
        x:=None;
        writeln('{ Unknown Declaration: ',pe.Name,' }');
       end;
    end;
   if x = Variables then PrintVars;
   l.Free;
  end;
end;

{replaced by GetDecls
 this does the same but not in true order

procedure PrintDecls(Decl:TPasDeclarations; indent:integer);
 var i:integer;
     pe:TPasElement;
     pp:TPasProcedure;
     ps:TPasSection;
     s:string;
     istype:boolean;

begin
 istype:=false;
 s:=GetIndent(indent);
 if (Decl is TInterfaceSection)or(Decl is TImplementationSection) or
     (Decl is TProgramSection) then
  if TPasSection(Decl).UsesList.Count >0 then
    begin
     write(s,'uses ');
     ps:=TPasSection(Decl);
     if not Unformated then begin writeln; write(s,'  '); end;
     for i:=0 to ps.UsesList.Count-2 do
      if UpCase(TPasElement(ps.UsesList[i]).Name) = 'SYSTEM' then continue //do not print system
       else write(TPasElement(ps.UsesList[i]).Name,',');                   //as it is added by parser
     writeln(TPasElement(ps.UsesList[ps.UsesList.Count-1]).Name,';');
     if not Unformated then writeln;
    end;

  if assigned(Decl.ResStrings) then
    if Decl.ResStrings.Count >0 then
     begin
      writeln('ResourceString');
      for i := 0 to Decl.ResStrings.Count - 1 do
       begin
        pe:=TPasElement(Decl.ResStrings[i]);
        writeln(s,pe.Name,'=',DelQuot(TPasResString(pe).Value),';'); //too much '''
       end;
      if not Unformated then writeln;
     end;

  if assigned(Decl.Consts)then
    if Decl.Consts.Count >0 then
     begin
      writeln(s,'const');
      for i:=0 to Decl.Consts.Count-1 do GetTPasVar(TPasVariable(Decl.Consts[i]),indent+1,false);
      if not Unformated then writeln;
     end;

  if assigned(Decl.Types) then
    if Decl.Types.Count >0 then
     begin
      writeln(s,'Type');
      for i := 0 to Decl.Types.Count - 1 do
       begin
        GetTypes(TPasElement(Decl.Types[i]),indent+1);
       end;
      if not Unformated then writeln;
      istype:=true; 
     end;

  if assigned(Decl.Classes) then
    if Decl.Classes.Count >0 then
     begin
      if not istype then writeln('Type');
      for i := 0 to Decl.Classes.Count - 1 do
       begin
        pe:=TPasElement(Decl.Classes[i]);
        GetTPasClass(TPasClassType(pe),indent+1);
        if not Unformated then writeln;
       end;
     end;

  if assigned(Decl.Variables)then
    if Decl.Variables.Count >0 then
     begin
       writeln(s,'var');
       //Now i use GetPasVariables for more compact output
       //for i:=0 to Decl.Variables.Count-1 do GetTPasVar(TPasVariable(Decl.Variables[i]),indent+1,false);
       GetPasVariables(Decl.Variables,indent+1,false,false);
       if not Unformated then writeln;
     end;

  if assigned(Decl.Functions) then
   begin
    for i := 0 to Decl.Functions.Count - 1 do
     begin
       pe:=TPasElement(Decl.Functions[i]);
       if pe is TPasOverloadedProc then
        begin
         GetTpasOverloadedProc(TPasOverloadedProc(pe),indent);
        end
       else
        begin
         pp:=TPasProcedure(pe);
         GetTPasProcedure(pp,indent);
         GetTPasProcedureBody(pp.Body,indent);
        end;
     end;
   end;
end;   }

//# parameter

 procedure PrintUsage;
  begin
   writeln('usage: test_parser1 <Options> <Commandline> File');
   writeln;
   writeln(' <Options> : Options for test_parser1');
   writeln('  -u  : Unformated output');
   writeln('  -OS <os>   : <os>  = WINDOWS, LINUX (default), FREEBSD, NETBSD,');
   writeln('                        SUNOS, BEOS, QNX, GO32V2');
   writeln('  -CPU <cpu> : <cpu> = i386 (default), x86_64');
   writeln(' <Commandline> : is the commandline for the parser');
   writeln('  -d<define>        : <define> = Directive');
   writeln('  -Fi<include_path> : <include_path> = ?');
   writeln('  -I<include_path>  : <include_path> = ?');
   writeln('  -Sd               : mode delphi');
   writeln(' File : a pascal source file (Program or Unit)');
  end;

 procedure GetParam;
  begin
    if paramcount>0 then
     begin
      cmdl:='';
      i:=1;
      repeat
        if paramstr(i) = '-h' then
         begin
          PrintUsage;
          halt(0);
         end
        else if paramstr(i) = '-u' then Unformated:= true
        else if paramstr(i) = '-OS' then
         begin
          if i < paramcount then
           begin
            inc(i);
            TargetOS:=paramstr(i);
            if (TargetOS = '')or(TargetOS[1] = '-') then halt(1);
           end
            else halt(1);
         end
        else if paramstr(i) = '-CPU' then
         begin
          if i < paramcount then
           begin
            inc(i);
            TargetCPU:=paramstr(i);
            if (TargetCPU = '')or(TargetCPU[1] = '-') then halt(1);
           end
            else halt(1);
         end
        else
          cmdl:=cmdl+' '+paramstr(i);
       inc(i);
      until i > paramcount;
     end;
    if (Paramcount < 1)or(cmdl = '') then
     begin
      // remember to put the whole cmdline in quotes, and
      // to always add some path options. Even if only -Fu. -Fi.
       writeln('Error: No file for input given !');
       PrintUsage;
       halt(1);
     end;
  end;

//# ***    main    ***

begin
  isim:=false;
  Unformated:=false;//false to format output to be human readable
  TargetOS:='linux';
  TargetCPU:='i386';
  GetParam;
  //writeln(TargetOS,' ',TargetCPU,' ',cmdl);halt;
  E := TSimpleEngine.Create;
  try
    try
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
      M := ParseSource(E, cmdl ,TargetOS ,TargetCPU,False);
=======
      M := ParseSource(E, cmdl ,TargetOS ,TargetCPU);
>>>>>>> graemeg/cpstrnew
=======
      M := ParseSource(E, cmdl ,TargetOS ,TargetCPU);
>>>>>>> graemeg/cpstrnew
=======
      M := ParseSource(E, cmdl ,TargetOS ,TargetCPU);
>>>>>>> graemeg/cpstrnew
=======
      M := ParseSource(E, cmdl ,TargetOS ,TargetCPU);
>>>>>>> origin/cpstrnew
    except
      on excep:EParserError do
        begin
          writeln(excep.message,' line:',excep.row,' column:',excep.column,' file:',excep.filename);
          raise;
       end;
    end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> graemeg/cpstrnew
=======

>>>>>>> origin/cpstrnew
   if M is TPasProgram then
    begin
     writeln('Program ',M.Name,';');
     if not Unformated then writeln;
     if assigned(M.ImplementationSection) then
       begin
        isim:=true;
        if not Unformated then writeln;
        GetDecls(M.ImplementationSection as TPasDeclarations,0);
        //PrintDecls(M.ImplementationSection as TPasDeclarations,0);
       end;
     if assigned(M.InitializationSection) then // MAIN BLOCK
       begin
        isim:=false;
        if not Unformated then writeln;
        writeln('begin');//writeln('begin {Begin MAIN Program}')
        GetTPasImplBlock(M.InitializationSection as TPasImplBlock,1,0,false,false);
       end;  
    end
   else
    begin
      { Cool, we successfully parsed the unit.
        Now output some info about it. }
      writeln('Unit ',M.Name,';');
      if not Unformated then writeln;
      Writeln('Interface');
      if not Unformated then writeln;
      GetDecls(M.InterfaceSection as TPasDeclarations,0);
      //PrintDecls(M.InterfaceSection as TPasDeclarations,0);

      if assigned(M.ImplementationSection) then
       begin
        isim:=true;
        if not Unformated then writeln;
        Writeln('Implementation');
        if not Unformated then writeln;
        GetDecls(M.ImplementationSection as TPasDeclarations,0);
        //PrintDecls(M.ImplementationSection as TPasDeclarations,0);
        if TPasElement(M.ImplementationSection) is TPasImplElement then writeln('{MAIN}');
       end;
      if assigned(M.InitializationSection) then //is this begin .. end. of a unit too ?
       begin
        isim:=true;
        if not Unformated then writeln;
        Writeln('Initialization');
        if not Unformated then writeln;
        GetTPasImplBlock(M.InitializationSection as TPasImplBlock,1,0,false,false);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
       end;
      
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
        if assigned(M.FinalizationSection) then
         begin
          isim:=true;
          if not Unformated then writeln;
          Writeln('Finalization');
          if not Unformated then writeln;
          GetTPasImplBlock(M.FinalizationSection as TPasImplBlock,1,0,false,false);
         end;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
       end;
>>>>>>> graemeg/cpstrnew
=======
       end;
>>>>>>> graemeg/cpstrnew
=======
       end;
>>>>>>> graemeg/cpstrnew
=======
       end;
>>>>>>> origin/cpstrnew
    end;
    if not Unformated then writeln('end.')
     else
      begin
       writeln('end');
       writeln('.');
      end;
    FreeAndNil(M);
  finally
    FreeAndNil(E);
  end;
end.
