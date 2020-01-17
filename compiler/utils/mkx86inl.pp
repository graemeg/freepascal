program mkx86inl;

{$mode objfpc}
{$H+}

uses
  sysutils, classes,
  strutils;

type
  TOperDirection = (operIn, operVar, operOut);

  TOperand = record
    name,
    typ: string;
    direction: TOperDirection;
  end;

const
  DirLUT: array[TOperDirection] of string = ('','var ','out ');

function GetPascalType(const ATyp: string): string;
  begin
    case ATyp of
      'r32':   exit('longword');
      'rs32':  exit('longint');
      'r64':   exit('qword');
      'rs64':  exit('int64');
      'f32':   exit('single');
      'mm':    exit('__m64');
      'xmm':   exit('__m128');
      'i32':   exit('longint');
      'ptr32': exit('pointer');
    else
      exit(ATyp);
    end;
  end;

function GetTypeDef(const ATyp: string): string;
  begin
    case ATyp of
      'r32':   exit('u32inttype');
      'rs32':  exit('s32inttype');
      'r64':   exit('u64inttype');
      'rs64':  exit('s64inttype');
      'f32':   exit('s32floattype');
      'mm':    exit('x86_m64type');
      'xmm':   exit('x86_m128type');
      'i32':   exit('s32inttype');
      'ptr32': exit('voidpointertype');
    else
      exit(ATyp);
    end;
  end;

function GetOper(const ATyp: string): string;
  begin
    case ATyp of
      'r32':   exit('_reg');
      'rs32':  exit('_reg');
      'r64':   exit('_reg_reg');
      'rs64':  exit('_reg_reg');
      'f32':   exit('_reg');
      'mm':    exit('_reg');
      'xmm':   exit('_reg');
      'i32':   exit('_const');
      'ptr32': exit('_ref');
    else
      exit('');
    end;
  end;

function GetOperand(const ATyp: string; AIndex: longint): string;
  begin
    case ATyp of
      'r32':   exit(format(',paraarray[%d].location.register', [AIndex]));
      'rs32':  exit(format(',paraarray[%d].location.register', [AIndex]));
      'r64':   exit(format(',paraarray[%d].location.register64.reglo,paraarray[%d].location.register64.reghi', [AIndex,AIndex]));
      'rs64':  exit(format(',paraarray[%d].location.register64.reglo,paraarray[%d].location.register64.reghi', [AIndex,AIndex]));
      'f32':   exit(format(',paraarray[%d].location.register', [AIndex]));
      'mm':    exit(format(',paraarray[%d].location.register', [AIndex]));
      'xmm':   exit(format(',paraarray[%d].location.register', [AIndex]));
      'i32':   exit(format(',GetConstInt(paraarray[%d])',[AIndex]));
      'ptr32': exit(format(',paraarray[%d].location.reference', [AIndex]));
    else
      exit(ATyp);
    end;
  end;

function GetOperandLoc(const ATyp: string): string;
  begin
    result:='';
    case ATyp of
      'r32':  exit(',location.register');
      'rs32': exit(',location.register');
      'r64':  exit(',location.register64.reglo,location.register64.reghi');
      'rs64': exit(',location.register64.reglo,location.register64.reghi');
      'f32':  exit(',location.register');
      'mm':   exit(',location.register');
      'xmm':  exit(',location.register');
    end;
  end;

function GetLocStatement(AIndex: longint; const ATyp: string; AConst: boolean): string;
  begin
    result:='';
    case ATyp of
      'r32':   exit(format('hlcg.location_force_reg(current_asmdata.CurrAsmList, paraarray[%d].location, paraarray[%d].resultdef,u32inttype,%s);', [AIndex+1, AIndex+1, BoolToStr(aconst,'true','false')]));
      'rs32':  exit(format('hlcg.location_force_reg(current_asmdata.CurrAsmList, paraarray[%d].location, paraarray[%d].resultdef,u32inttype,%s);', [AIndex+1, AIndex+1, BoolToStr(aconst,'true','false')]));
      'r64':   exit(format('hlcg.location_force_reg(current_asmdata.CurrAsmList, paraarray[%d].location, paraarray[%d].resultdef,u64inttype,%s);', [AIndex+1, AIndex+1, BoolToStr(aconst,'true','false')]));
      'rs64':  exit(format('hlcg.location_force_reg(current_asmdata.CurrAsmList, paraarray[%d].location, paraarray[%d].resultdef,u64inttype,%s);', [AIndex+1, AIndex+1, BoolToStr(aconst,'true','false')]));
      'f32':   exit(format('location_force_mmreg(current_asmdata.CurrAsmList, paraarray[%d].location, %s);', [AIndex+1, BoolToStr(aconst,'true','false')]));
      'mm':    exit(format('location_force_mmxreg(current_asmdata.CurrAsmList, paraarray[%d].location, %s);', [AIndex+1, BoolToStr(aconst,'true','false')]));
      'xmm':   exit(format('location_force_mmreg(current_asmdata.CurrAsmList, paraarray[%d].location, %s);', [AIndex+1, BoolToStr(aconst,'true','false')]));
      'ptr32': exit(format('location_make_ref(paraarray[%d].location);', [AIndex+1]));
    end;
  end;

function GetLoc(const ATyp: string; AWithSize: boolean = true): string;
  begin
    result:='';
    if AWithSize then
      case ATyp of
        'r32':   exit('LOC_REGISTER,OS_32');
        'rs32':  exit('LOC_REGISTER,OS_S32');
        'r64':   exit('LOC_REGISTER,OS_64');
        'rs64':  exit('LOC_REGISTER,OS_S64');
        'f32':   exit('LOC_MMREGISTER,OS_M128');
        'mm':    exit('LOC_MMXREGISTER,OS_M64');
        'xmm':   exit('LOC_MMREGISTER,OS_M128');
        'ptr32': exit('LOC_MEM,OS_32');
      end
    else
      case ATyp of
        'r32':   exit('LOC_REGISTER');
        'rs32':  exit('LOC_REGISTER');
        'r64':   exit('LOC_REGISTER');
        'rs64':  exit('LOC_REGISTER');
        'f32':   exit('LOC_MMREGISTER');
        'mm':    exit('LOC_MMXREGISTER');
        'xmm':   exit('LOC_MMREGISTER');
        'ptr32': exit('LOC_MEM');
      end;
  end;

function GetLocAllocation(const ATyp: string): string;
  begin
    result:='';
    case ATyp of
      'r32':  exit('location.register:=cg.getintregister(current_asmdata.CurrAsmList, OS_32);');
      'rs32': exit('location.register:=cg.getintregister(current_asmdata.CurrAsmList, OS_32);');
      'r64':  exit('location.register64.reglo:=cg.getintregister(current_asmdata.CurrAsmList, OS_32); location.register64.reghi:=cg.getintregister(current_asmdata.CurrAsmList, OS_32);');
      'rs64': exit('location.register64.reglo:=cg.getintregister(current_asmdata.CurrAsmList, OS_32); location.register64.reghi:=cg.getintregister(current_asmdata.CurrAsmList, OS_32);');
      'f32':  exit('location.register:=cg.getmmregister(current_asmdata.CurrAsmList, OS_M128);');
      'mm':   exit('location.register:=tcgx86(cg).getmmxregister(current_asmdata.CurrAsmList);');
      'xmm':  exit('location.register:=cg.getmmregister(current_asmdata.CurrAsmList, OS_M128);');
    end;
  end;

function GetPostFix(const APF: string): string;
  begin
    if APF<>'' then
      result:='PF_'+APF
    else
      result:='PF_None';
  end;

procedure ParseList(const APrefix, AFilename: string);
  var
    f: TextFile;

    fprocs,
    fcinnr, fcpumminnr: TextFile;
    ftypechk, ffirst, fsecond: TStringList;

    str,
    instrPart,postfix,_alias,
    params, operline: String;

    opers: array[0..7] of TOperand;
    opercnt: longint;

    hasOutput: boolean;
    outputType: string;
    cnt,
    i, intrnum: longint;
    tmp: String;

  function ParseOperands(AIndex: longint = -1): string;
    var
      idx: LongInt;
      pt: Integer;
      c: Char;
    begin
      idx:=opercnt;

      params:=trim(params);
      if params='' then
        exit('');

      inc(opercnt);

      if pos('var ', params)=1 then
        begin
          opers[idx].direction:=operVar;
          Delete(params,1,4);
          params:=trim(params);
          hasOutput:=true;
        end
      else if pos('out ', params)=1 then
        begin
          opers[idx].direction:=operOut;
          Delete(params,1,4);
          params:=trim(params);
          hasOutput:=true;
        end
      else
        begin
          if AIndex<>-1 then
            opers[idx].direction:=opers[AIndex].direction
          else
            opers[idx].direction:=operIn;
        end;

          pt:=PosSet([',',':'], params);

      c:=params[pt];
      opers[idx].name:=Copy2SymbDel(params, c);
      params:=trim(params);

      if c = ':' then
        begin
          opers[idx].typ:=Copy2SymbDel(params, ';');
          result:=opers[idx].typ;
        end
      else
        begin
          opers[idx].typ:=ParseOperands(idx);
          result:=opers[idx].typ;
        end;

      if opers[idx].direction<>operIn then
        outputType:=opers[idx].typ;
    end;

  function GetOperLine: string;
    var
      i: longint;
    begin
      result:='';
      for i := 0 to opercnt-1 do
        result:=result+DirLUT[opers[i].direction]+opers[i].name+':'+opers[i].typ+';';
    end;

  function GetParams: longint;
    var
      i: longint;
    begin
      result:=0;
      for i := 0 to opercnt-1 do
        if opers[i].direction in [operIn,operVar] then
          inc(result);
    end;

  function FindOperIdx(const AOper: string): longint;
    var
      i,cnt: longint;
    begin
      cnt:=0;
      result:=0;
      for i := 0 to opercnt-1 do
        if (opers[i].direction in [operIn,operVar]) then
          begin
            if opers[i].name=AOper then
              exit(cnt);
            inc(cnt);
          end;
    end;

  begin
    intrnum:=0;

    assignfile(f, AFilename);
    reset(f);

    assignfile(fprocs, 'cpummprocs.inc'); rewrite(fprocs);
    assignfile(fcinnr, 'c'+APrefix+'mminnr.inc'); rewrite(fcinnr);
    assignfile(fcpumminnr, 'cpumminnr.inc'); rewrite(fcpumminnr);

//    writeln(finnr,'const');

    ftypechk:=TStringList.Create;
    ffirst:=TStringList.Create;
    fsecond:=TStringList.Create;

//    writeln(finnr, '  fpc_in_', APrefix,'_first = fpc_in_',APrefix,'_base;');

    while not EOF(f) do
      begin
        readln(f, str);

        str:=trim(str);

        if (str='') or (Pos(';',str)=1) then
          continue;

        instrPart:=Copy2SymbDel(str, '(');

        // Check for postfix
        if pos('{',instrPart)>0 then
          begin
            postfix:=instrPart;
            instrPart:=Copy2SymbDel(postfix, '{');
            postfix:=TrimRightSet(postfix,['}']);
          end
        else
          postfix:='';

        // Check for alias
        if pos('[',instrPart)>0 then
          begin
            _alias:=instrPart;
            instrPart:=Copy2SymbDel(_alias, '[');
            _alias:='_'+TrimRightSet(_alias,[']']);
          end
        else
          _alias:='';

        // Get parameters
        params:=trim(Copy2SymbDel(str,')'));
        str:=trim(str);

        hasOutput:=false;
        opercnt:=0;
        outputType:='';

        while params<>'' do
          ParseOperands;

        operline:=GetOperLine;
        // Write typecheck code
        i:=ftypechk.IndexOf(': //'+operline);
        if i>=0 then
          ftypechk.Insert(i,',in_'+APrefix+'_'+instrPart+postfix+_alias)
        else
          begin
            ftypechk.Add('in_'+APrefix+'_'+instrPart+postfix+_alias);
            ftypechk.Add(': //'+operline);
            ftypechk.Add('  begin');
            ftypechk.Add('    CheckParameters('+inttostr(GetParams())+');');
            if hasOutput then
              ftypechk.Add('    resultdef:='+GetTypeDef(outputType)+';')
            else
              ftypechk.Add('    resultdef:=voidtype;');
            ftypechk.Add('  end;')
          end;

        // Write firstpass code
        i:=ffirst.IndexOf(': //'+operline);
        if i>=0 then
          ffirst.Insert(i,',in_'+APrefix+'_'+instrPart+postfix+_alias)
        else
          begin
            ffirst.Add('in_'+APrefix+'_'+instrPart+postfix+_alias);
            ffirst.Add(': //'+operline);
            ffirst.Add('  begin');
            if hasOutput then
              ffirst.Add('    expectloc:='+GetLoc(outputType,false)+';')
            else
              ffirst.Add('    expectloc:=LOC_VOID;');
            ffirst.Add('    result:=nil;');
            ffirst.Add('  end;')
          end;

        // Write secondpass code
        i:=fsecond.IndexOf(': //'+operline);
        if i>=0 then
          begin
            fsecond.Insert(i+3,'      in_'+APrefix+'_'+instrPart+postfix+_alias+': begin op:=A_'+instrPart+' end;');
            fsecond.Insert(i,',in_'+APrefix+'_'+instrPart+postfix+_alias);
          end
        else
          begin
            fsecond.Add('in_'+APrefix+'_'+instrPart+postfix+_alias);
            fsecond.Add(': //'+operline);
            fsecond.Add('  begin');
            fsecond.Add('    case inlinenumber of');
            fsecond.Add('      in_'+APrefix+'_'+instrPart+postfix+_alias+': begin op:=A_'+instrPart+'; end;');
            fsecond.Add('      else');
            fsecond.Add('        Internalerror(2020010201);');
            fsecond.Add('    end;');
            fsecond.Add('');

            i:=GetParams;
            fsecond.Add('    GetParameters('+inttostr(i)+');');
            fsecond.Add('');

            fsecond.Add('    for i := 1 to '+inttostr(i)+' do secondpass(paraarray[i]);');
            fsecond.Add('');

            // Force inputs
            cnt:=0;
            for i := 0 to opercnt-1 do
              begin
                case opers[i].direction of
                  operIn:
                    begin
                      tmp:=GetLocStatement(cnt, opers[i].typ, true);
                      if tmp<>'' then
                        fsecond.add('    '+tmp);
                      inc(cnt);
                    end;
                  operVar:
                    begin
                      tmp:=GetLocStatement(cnt, opers[i].typ, false);
                      if tmp<>'' then
                        fsecond.add('    '+tmp);
                      inc(cnt);
                    end;
                end;
              end;

            // Allocate output
            cnt:=0;
            for i := 0 to opercnt-1 do
              begin
                case opers[i].direction of
                  operOut:
                    begin
                      fsecond.add('    location_reset(location,'+GetLoc(opers[i].typ)+');');
                      fsecond.Add('    '+GetLocAllocation(opers[i].typ));
                    end;
                  operVar:
                    begin
                      fsecond.Add('    location:=paraarray['+inttostr(cnt+1)+'].location;');
                      inc(cnt);
                    end;
                  operIn:
                    inc(cnt);
                end;
              end;

            operline:='taicpu.op';
            //for i := 0 to opercnt-1 do
            for i := opercnt-1 downto 0 do
              begin
                case opers[i].direction of
                  operOut:
                    operline:=operline+GetOper(opers[i].typ);
                  operVar:
                    operline:=operline+GetOper(opers[i].typ);
                  operIn:
                    operline:=operline+GetOper(opers[i].typ);
                end;
              end;

            if operline='taicpu.op' then
              operline:='taicpu.op_none(op,S_NO'
            else
              operline:=operline+'(op,S_NO';

            //for i := 0 to opercnt-1 do
            for i := opercnt-1 downto 0 do
              begin
                case opers[i].direction of
                  operOut:
                    operline:=operline+GetOperandLoc(opers[i].typ);
                  operIn,
                  operVar:
                    begin
                      dec(cnt);
                      operline:=operline+GetOperand(opers[i].typ, cnt+1);
                    end;
                end;
              end;

            operline:=operline+')';

            fsecond.Add('    current_asmdata.CurrAsmList.concat('+operline+');');

            fsecond.Add('  end;')
          end;

        // Write innr
        writeln(fcinnr, '  in_', APrefix,'_',instrPart,postfix+_alias,' = in_',APrefix,'_mm_first+',intrnum,',');
        writeln(fcpumminnr, '  fpc_in_', APrefix,'_',instrPart,postfix+_alias,' = fpc_in_',APrefix,'_mm_first+',intrnum,';');

        // Write function
        if hasOutput then write(fprocs,'function ') else write(fprocs,'procedure ');
        write(fprocs,APrefix,'_',instrPart,postfix,'(');

        cnt:=0;
        for i:=0 to opercnt-1 do
          begin
            if opers[i].direction=operOut then
              Continue;

            if cnt>0 then
              begin
                if opers[i].typ<>opers[i-1].typ then
                  write(fprocs,': ',GetPascalType(opers[i-1].typ),'; ')
                else
                  write(fprocs,', ');
              end;

            write(fprocs,opers[i].name);
            if i=opercnt-1 then
              write(fprocs,': ',GetPascalType(opers[i].typ));

            inc(cnt);
          end;

        write(fprocs,')');

        if hasOutput then write(fprocs,': ',GetPascalType(outputType));
        writeln(fprocs,'; [INTERNPROC: fpc_in_',APrefix,'_',instrPart,postfix+_alias,'];');

        // Str now contains conditionals

        inc(intrnum);
      end;

    writeln(fcinnr, '  in_', APrefix,'mm_last = in_',APrefix,'_mm_first+',intrnum-1);

    ftypechk.SaveToFile(APrefix+'mmtype.inc');
    ffirst.SaveToFile(APrefix+'mmfirst.inc');
    fsecond.SaveToFile(APrefix+'mmsecond.inc');

    ftypechk.Free;
    ffirst.Free;
    fsecond.Free;

    CloseFile(fprocs);
    CloseFile(fcinnr);
    CloseFile(fcpumminnr);

    closefile(f);
  end;

begin
  ParseList('x86', 'x86intr.dat');
end.

