unit wasmdef;

{$i fpcdefs.inc}

interface

uses
  symtype, symsym, symdef, symconst, constexp
  ,defutil, procdefutil, cclasses;

type

    { TWasmTypeEntry }

    TWasmTypeEntry = class(Tobject)
      name : string; // always empty
      idx  : integer;
      constructor Create(aidx: integer; aname: string);
    end;

    { TWasmProcTypeLookup }

    TWasmProcTypeLookup = class(TObject)
      list: TFPHashObjectList;
      idx: integer;
      constructor Create(astartIndex: integer = 0);
      destructor Destroy; override;
      function GetTypeIndex(const typecode: string): Integer;
    end;


    // encodes procedure definition to a code used for the proc type lookup
    // it's case-sensitive!!!
    // i = i32, I = i64, f = f32, F = f32
    function WasmGetTypeCodeForDef(def: tdef; var ch: char): Boolean;

    function WasmGetTypeCode(aprocdef: tabstractprocdef): string;

    { returns whether a def always resides in memory,
      rather than in wasm local variables...) }
    function wasmAlwayInMem(def: tdef): boolean;

    function get_para_push_size(def: tdef): tdef;

implementation

  function get_para_push_size(def: tdef): tdef;
    begin
      result:=def;
      if def.typ=orddef then
        case torddef(def).ordtype of
          u8bit,uchar:
            if torddef(def).high>127 then
              result:=s8inttype;
          u16bit:
            begin
              if torddef(def).high>32767 then
                result:=s16inttype;
            end
          else
            ;
        end;
    end;

  function wasmAlwayInMem(def: tdef): boolean;
    begin
      case def.typ of
        arraydef,
        filedef,
        recorddef,
        objectdef,
        stringdef:
          result:=true;
        else
          result:=false;
      end;
    end;

  function WasmGetTypeCodeForDef(def: tdef; var ch: char): Boolean;
  begin
    Result := assigned(def);
    if not Result then Exit;

    case def.typ of
      floatdef:
        if def.size = 4 then ch :='f'
        else ch :='F';
      orddef:
        if def.size = 8 then ch :='I'
        else ch := 'i';
      // todo: set can be bigger
    else
      ch:='i'; // by address
    end;
  end;

  function WasmGetTypeCode(aprocdef: tabstractprocdef): string;
    var
      ch : char;
      i  : integer;
    begin
      Result := '';
      if not Assigned(aprocdef) then exit;

      for i:=0 to aprocdef.paras.Count-1 do begin
        WasmGetTypeCodeForDef( tparavarsym(aprocdef.paras[i]).paraloc[callerside].Def, ch);
        result:=result+ch;
      end;

      if assigned(aprocdef) then begin
        result:=result+':';
        WasmGetTypeCodeForDef(aprocdef.returndef, ch);
        result:=result+ch;
      end;

    end;

    { TWasmTypeEntry }

     constructor TWasmTypeEntry.Create(aidx: integer; aname: string);
      begin
        idx := aidx;
        name := aname;
      end;

  { TWasmProcTypeLookup }

    constructor TWasmProcTypeLookup.Create(astartIndex: integer = 0);
      begin
        inherited Create;
        list := TFPHashObjectList.Create(true);
        idx := astartIndex;
      end;

    destructor TWasmProcTypeLookup.Destroy;
      begin
        list.Free;
        inherited Destroy;
      end;

    function TWasmProcTypeLookup.GetTypeIndex(const typecode: string): Integer;
      var
        en : TWasmTypeEntry;
      begin
        en := TWasmTypeEntry(list.Find(typecode));
        if not Assigned(en) then begin
          en := TWasmTypeEntry.Create(idx, ''); // no need to copy
          inc(idx);
          list.Add(typecode, en);
        end;
        Result := en.idx;
      end;


end.
