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
      'r8':    exit('byte');
      'rs8':   exit('shortint');
      'r16':   exit('word');
      'rs16':  exit('smallint');
      'r32':   exit('longword');
      'rs32':  exit('longint');
      'r64':   exit('qword');
      'rs64':  exit('int64');
      'reg':   exit('NativeUInt');
      'sreg':  exit('NativeInt');
      'f32':   exit('single');
      'f64':   exit('double');
      'mm':    exit('__m64');
      'implicit_xmm0',
      'xmm':   exit('__m128');
      'i32':   exit('longint');

      'edi_ptr':   exit('pointer');

      'ptr8',
      'ptr16',
      'ptr32',
      'ptr64',
      'ptr128': exit('pointer');
    else
      exit(ATyp);
    end;
  end;

function GetTypeDef(const ATyp: string): string;
  begin
    case ATyp of
      'r8':    exit('u8inttype');
      'rs8':   exit('s8inttype');
      'r16':   exit('u16inttype');
      'rs16':  exit('s16inttype');
      'r32':   exit('u32inttype');
      'rs32':  exit('s32inttype');
      'r64':   exit('u64inttype');
      'rs64':  exit('s64inttype');
      'reg':   exit('uinttype');
      'sreg':  exit('sinttype');
      'f32':   exit('s32floattype');
      'f64':   exit('s64floattype');
      'mm':    exit('x86_m64type');
      'implicit_xmm0',
      'xmm':   exit('x86_m128type');
      'i32':   exit('s32inttype');

      'edi_ptr':   exit('voidpointertype');

      'ptr8',
      'ptr16',
      'ptr32',
      'ptr64',
      'ptr128': exit('voidpointertype');
    else
      exit(ATyp);
    end;
  end;

function GetOper(const ATyp: string): string;
  begin
    case ATyp of
      'r8':    exit('_reg');
      'rs8':   exit('_reg');
      'r16':   exit('_reg');
      'rs16':  exit('_reg');
      'r32':   exit('_reg');
      'rs32':  exit('_reg');
      'r64':   exit('_reg_reg');
      'rs64':  exit('_reg_reg');
      'reg':   exit('_reg');
      'sreg':  exit('_reg');
      'f32':   exit('_reg');
      'f64':   exit('_reg');
      'mm':    exit('_reg');
      'xmm':   exit('_reg');
      'i32':   exit('_const');

      'implicit_xmm0',
      'edi_ptr':   exit('');

      'ptr8',
      'ptr16',
      'ptr32',
      'ptr64',
      'ptr128': exit('_ref');
    else
      exit('');
    end;
  end;

function GetOperand(const ATyp: string; AIndex: longint): string;
  begin
    case ATyp of
      'r8':    exit(format(',paraarray[%d].location.register', [AIndex]));
      'rs8':   exit(format(',paraarray[%d].location.register', [AIndex]));
      'r16':   exit(format(',paraarray[%d].location.register', [AIndex]));
      'rs16':  exit(format(',paraarray[%d].location.register', [AIndex]));
      'r32':   exit(format(',paraarray[%d].location.register', [AIndex]));
      'rs32':  exit(format(',paraarray[%d].location.register', [AIndex]));
      'r64':   exit(format(',paraarray[%d].location.register64.reglo,paraarray[%d].location.register64.reghi', [AIndex,AIndex]));
      'rs64':  exit(format(',paraarray[%d].location.register64.reglo,paraarray[%d].location.register64.reghi', [AIndex,AIndex]));
      'reg':   exit(format(',paraarray[%d].location.register', [AIndex]));
      'sreg':  exit(format(',paraarray[%d].location.register', [AIndex]));
      'f32':   exit(format(',paraarray[%d].location.register', [AIndex]));
      'f64':   exit(format(',paraarray[%d].location.register', [AIndex]));
      'mm':    exit(format(',paraarray[%d].location.register', [AIndex]));
      'xmm':   exit(format(',paraarray[%d].location.register', [AIndex]));
      'i32':   exit(format(',GetConstInt(paraarray[%d])',[AIndex]));

      'implicit_xmm0',
      'edi_ptr': exit('');

      'ptr8',
      'ptr16',
      'ptr32',
      'ptr64',
      'ptr128': exit(format(',paraarray[%d].location.reference', [AIndex]));
    else
      exit(ATyp);
    end;
  end;

function GetOperandLoc(const ATyp: string): string;
  begin
    result:='';
    case ATyp of
      'r8':   exit(',location.register');
      'rs8':  exit(',location.register');
      'r16':  exit(',location.register');
      'rs16': exit(',location.register');
      'r32':  exit(',location.register');
      'rs32': exit(',location.register');
      'r64':  exit(',location.register64.reglo,location.register64.reghi');
      'rs64': exit(',location.register64.reglo,location.register64.reghi');
      'reg':  exit(',location.register');
      'sreg': exit(',location.register');
      'f32':  exit(',location.register');
      'f64':  exit(',location.register');
      'mm':   exit(',location.register');
      'implicit_xmm0',
      'xmm':  exit(',location.register');

      'edi_ptr': exit(',location.register');

      'ptr8',
      'ptr16',
      'ptr32',
      'ptr64',
      'ptr128': exit(',location.register');
    end;
  end;

function GetLocStatement(AIndex: longint; const ATyp: string; AConst: boolean): string;
  begin
    result:='';
    case ATyp of
      'r8':    exit(format('hlcg.location_force_reg(current_asmdata.CurrAsmList, paraarray[%d].location, paraarray[%d].resultdef,u8inttype,%s);', [AIndex+1, AIndex+1, BoolToStr(aconst,'true','false')]));
      'rs8':   exit(format('hlcg.location_force_reg(current_asmdata.CurrAsmList, paraarray[%d].location, paraarray[%d].resultdef,u8inttype,%s);', [AIndex+1, AIndex+1, BoolToStr(aconst,'true','false')]));
      'r16':   exit(format('hlcg.location_force_reg(current_asmdata.CurrAsmList, paraarray[%d].location, paraarray[%d].resultdef,u16inttype,%s);', [AIndex+1, AIndex+1, BoolToStr(aconst,'true','false')]));
      'rs16':  exit(format('hlcg.location_force_reg(current_asmdata.CurrAsmList, paraarray[%d].location, paraarray[%d].resultdef,u16inttype,%s);', [AIndex+1, AIndex+1, BoolToStr(aconst,'true','false')]));
      'r32':   exit(format('hlcg.location_force_reg(current_asmdata.CurrAsmList, paraarray[%d].location, paraarray[%d].resultdef,u32inttype,%s);', [AIndex+1, AIndex+1, BoolToStr(aconst,'true','false')]));
      'rs32':  exit(format('hlcg.location_force_reg(current_asmdata.CurrAsmList, paraarray[%d].location, paraarray[%d].resultdef,u32inttype,%s);', [AIndex+1, AIndex+1, BoolToStr(aconst,'true','false')]));
      'r64':   exit(format('hlcg.location_force_reg(current_asmdata.CurrAsmList, paraarray[%d].location, paraarray[%d].resultdef,u64inttype,%s);', [AIndex+1, AIndex+1, BoolToStr(aconst,'true','false')]));
      'rs64':  exit(format('hlcg.location_force_reg(current_asmdata.CurrAsmList, paraarray[%d].location, paraarray[%d].resultdef,u64inttype,%s);', [AIndex+1, AIndex+1, BoolToStr(aconst,'true','false')]));
      'reg':   exit(format('hlcg.location_force_reg(current_asmdata.CurrAsmList, paraarray[%d].location, paraarray[%d].resultdef,uinttype,%s);', [AIndex+1, AIndex+1, BoolToStr(aconst,'true','false')]));
      'sreg':  exit(format('hlcg.location_force_reg(current_asmdata.CurrAsmList, paraarray[%d].location, paraarray[%d].resultdef,sinttype,%s);', [AIndex+1, AIndex+1, BoolToStr(aconst,'true','false')]));
      'f32':   exit(format('location_force_mmreg(current_asmdata.CurrAsmList, paraarray[%d].location, %s);', [AIndex+1, BoolToStr(aconst,'true','false')]));
      'f64':   exit(format('location_force_mmreg(current_asmdata.CurrAsmList, paraarray[%d].location, %s);', [AIndex+1, BoolToStr(aconst,'true','false')]));
      'mm':    exit(format('location_force_mmxreg(current_asmdata.CurrAsmList, paraarray[%d].location, %s);', [AIndex+1, BoolToStr(aconst,'true','false')]));
      'xmm':   exit(format('location_force_mmreg(current_asmdata.CurrAsmList, paraarray[%d].location, %s);', [AIndex+1, BoolToStr(aconst,'true','false')]));

      'implicit_xmm0':
        exit(format('location_force_mmreg(current_asmdata.CurrAsmList, paraarray[%d].location, %s);'+LineEnding+
                    '    hlcg.getcpuregister(current_asmdata.CurrAsmList,NR_XMM0);'+LineEnding+
                    '    hlcg.a_loadmm_loc_reg(current_asmdata.CurrAsmList,paraarray[%d].resultdef,x86_m128type,paraarray[%d].location,NR_XMM0,nil);',
                      [AIndex+1, BoolToStr(aconst,'true','false'), AIndex+1, AIndex+1]));
      'edi_ptr':
        exit(format('hlcg.getcpuregister(current_asmdata.CurrAsmList,{$if defined(cpu64bitalu)}NR_RDI{$else}NR_EDI{$endif});'+LineEnding+
                    '    hlcg.a_load_loc_reg(current_asmdata.CurrAsmList,paraarray[%d].resultdef,voidpointertype,paraarray[%d].location,{$if defined(cpu64bitalu)}NR_RDI{$else}NR_EDI{$endif});',
                      [AIndex+1, AIndex+1]));

      'ptr8',
      'ptr16',
      'ptr32',
      'ptr64',
      'ptr128':exit(format('location_make_ref(paraarray[%d].location);', [AIndex+1]));
    end;
  end;

function GetDeallocStatement(AIndex: longint; const ATyp: string): string;
  begin
    result:='';
    case ATyp of
      'implicit_xmm0':
        exit('hlcg.ungetcpuregister(current_asmdata.CurrAsmList,NR_XMM0);');
      'edi_ptr':
        exit('hlcg.ungetcpuregister(current_asmdata.CurrAsmList,{$if defined(cpu64bitalu)}NR_RDI{$else}NR_EDI{$endif});');
    end;
  end;

function GetLoc(const ATyp: string; AWithSize: boolean = true): string;
  begin
    result:='';
    if AWithSize then
      case ATyp of
        'r8':    exit('LOC_REGISTER,OS_8');
        'rs8':   exit('LOC_REGISTER,OS_S8');
        'r16':   exit('LOC_REGISTER,OS_16');
        'rs16':  exit('LOC_REGISTER,OS_S16');
        'r32':   exit('LOC_REGISTER,OS_32');
        'rs32':  exit('LOC_REGISTER,OS_S32');
        'r64':   exit('LOC_REGISTER,OS_64');
        'rs64':  exit('LOC_REGISTER,OS_S64');
        'reg':   exit('LOC_REGISTER,OS_INT');
        'sreg':  exit('LOC_REGISTER,OS_SINT');
        'f32':   exit('LOC_MMREGISTER,OS_M128');
        'f64':   exit('LOC_MMREGISTER,OS_M128');
        'mm':    exit('LOC_MMXREGISTER,OS_M64');
        'implicit_xmm0',
        'xmm':   exit('LOC_MMREGISTER,OS_M128');

        'edi_ptr':   exit('LOC_REGISTER,OS_INT');

        'ptr8':  exit('LOC_MEM,OS_8');
        'ptr16': exit('LOC_MEM,OS_16');
        'ptr32': exit('LOC_MEM,OS_32');
        'ptr64': exit('LOC_MEM,OS_64');
        'ptr128':exit('LOC_MEM,OS_128');
      end
    else
      case ATyp of
        'r8':    exit('LOC_REGISTER');
        'rs8':   exit('LOC_REGISTER');
        'r16':   exit('LOC_REGISTER');
        'rs16':  exit('LOC_REGISTER');
        'r32':   exit('LOC_REGISTER');
        'rs32':  exit('LOC_REGISTER');
        'r64':   exit('LOC_REGISTER');
        'rs64':  exit('LOC_REGISTER');
        'reg':   exit('LOC_REGISTER');
        'sreg':  exit('LOC_REGISTER');
        'f32':   exit('LOC_MMREGISTER');
        'f64':   exit('LOC_MMREGISTER');
        'mm':    exit('LOC_MMXREGISTER');
        'implicit_xmm0',
        'xmm':   exit('LOC_MMREGISTER');

        'edi_ptr':   exit('LOC_REGISTER');

        'ptr8',
        'ptr16',
        'ptr32',
        'ptr64',
        'ptr128':exit('LOC_MEM');
      end;
  end;

function GetLocAllocation(const ATyp: string): string;
  begin
    result:='';
    case ATyp of
      'r8':  exit('location.register:=cg.getintregister(current_asmdata.CurrAsmList, OS_8);');
      'rs8': exit('location.register:=cg.getintregister(current_asmdata.CurrAsmList, OS_8);');
      'r16':  exit('location.register:=cg.getintregister(current_asmdata.CurrAsmList, OS_16);');
      'rs16': exit('location.register:=cg.getintregister(current_asmdata.CurrAsmList, OS_16);');
      'r32':  exit('location.register:=cg.getintregister(current_asmdata.CurrAsmList, OS_32);');
      'rs32': exit('location.register:=cg.getintregister(current_asmdata.CurrAsmList, OS_32);');
      'r64':  exit('location.register64.reglo:=cg.getintregister(current_asmdata.CurrAsmList, OS_32); location.register64.reghi:=cg.getintregister(current_asmdata.CurrAsmList, OS_32);');
      'rs64': exit('location.register64.reglo:=cg.getintregister(current_asmdata.CurrAsmList, OS_32); location.register64.reghi:=cg.getintregister(current_asmdata.CurrAsmList, OS_32);');
      'reg':  exit('location.register:=cg.getintregister(current_asmdata.CurrAsmList, OS_INT);');
      'sreg': exit('location.register:=cg.getintregister(current_asmdata.CurrAsmList, OS_INT);');
      'f32':  exit('location.register:=cg.getmmregister(current_asmdata.CurrAsmList, OS_M128);');
      'f64':  exit('location.register:=cg.getmmregister(current_asmdata.CurrAsmList, OS_M128);');
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
    tmp, condition, target: String;

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

        // Parse condition and target
        if pos('|', str)>0 then
        begin
          condition:=trim(Copy2SymbDel(str, '|'));
          target:=trim(str);
        end
        else
        begin
          condition:=str;
          target:='';
        end;

        hasOutput:=false;
        opercnt:=0;
        outputType:='';

        while params<>'' do
          ParseOperands;

        operline:=GetOperLine;
        // Write typecheck code
        i:=ftypechk.IndexOf(': //'+operline);
        if (i>=0) and (target='') then
          ftypechk.Insert(i,',in_'+APrefix+'_'+instrPart+postfix+_alias)
        else
          begin
            if target<>'' then
              ftypechk.add(format('{$ifdef %s}', [target]));

            ftypechk.Add('in_'+APrefix+'_'+instrPart+postfix+_alias);
            ftypechk.Add(': //'+operline);
            ftypechk.Add('  begin');
            ftypechk.Add('    CheckParameters('+inttostr(GetParams())+');');
            if hasOutput then
              ftypechk.Add('    resultdef:='+GetTypeDef(outputType)+';')
            else
              ftypechk.Add('    resultdef:=voidtype;');
            ftypechk.Add('  end;');

            if target<>'' then
              ftypechk.add('{$endif}');
          end;

        // Write firstpass code
        i:=ffirst.IndexOf(': //'+operline);
        if (i>=0) and (target='') then
          ffirst.Insert(i,',in_'+APrefix+'_'+instrPart+postfix+_alias)
        else
          begin
            if target<>'' then
              ffirst.add(format('{$ifdef %s}', [target]));

            ffirst.Add('in_'+APrefix+'_'+instrPart+postfix+_alias);
            ffirst.Add(': //'+operline);
            ffirst.Add('  begin');
            if hasOutput then
              ffirst.Add('    expectloc:='+GetLoc(outputType,false)+';')
            else
              ffirst.Add('    expectloc:=LOC_VOID;');
            ffirst.Add('    result:=nil;');
            ffirst.Add('  end;');

            if target<>'' then
              ffirst.add('{$endif}');
          end;

        // Write secondpass code
        i:=fsecond.IndexOf(': //'+operline);
        if (i>=0) and (target='') then
          begin
            fsecond.Insert(i+3,'      in_'+APrefix+'_'+instrPart+postfix+_alias+': begin op:=A_'+instrPart+' end;');
            fsecond.Insert(i,',in_'+APrefix+'_'+instrPart+postfix+_alias);
          end
        else
          begin
            if target<>'' then
              fsecond.add(format('{$ifdef %s}', [target]));

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

            // Deallocate CPU registers
            for i := 0 to opercnt-1 do
              begin
                tmp:=GetDeallocStatement(cnt, opers[i].typ);
                if tmp<>'' then
                  fsecond.add('    '+tmp);
              end;

            fsecond.Add('  end;');

            if target<>'' then
              fsecond.add('{$endif}');
          end;

        // Write innr
        writeln(fcinnr, '  in_', APrefix,'_',instrPart,postfix+_alias,' = in_',APrefix,'_mm_first+',intrnum,',');
        writeln(fcpumminnr, '  fpc_in_', APrefix,'_',instrPart,postfix+_alias,' = fpc_in_',APrefix,'_mm_first+',intrnum,';');

        // Write function
        if target<>'' then
          writeln(fprocs, '{$ifdef ',target,'}');

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

        if target<>'' then
          writeln(fprocs, '{$endif}');

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

