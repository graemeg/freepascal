{$MODE FPC}
{
    This file is part of Free Pascal build tools
    Copyright (c) 2014-2015 by Tomas Hajny, member of the FPC core team.

    This program processes one or more listing files created with fpmake
    (e.g. using 'fpmake pkglist --target=<FPC_target> -zp units-' for
    unit packages or without the '-zp <prefix>' for utils), compares
    them to the text-mode installer configuration file install.dat and
    creates file install.add which provides information about packages
    missing in install.dat in a form allowing copy&paste of individual
    lines into install.dat.

    If the original description of a certain package as found in fpmake.pp
    is too long for install.dat, the maximum length is marked
    in the respective line in install.add using a pipe character ('|').

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

program mkinsadd;

uses
 dos, objects;


const
 MaxTarget = 5;
 TargetListShort: array [1..MaxTarget] of string [3] = ('dos', 'emx', 'os2', 'w32', 'src');
 TargetListLong: array [1..MaxTarget] of string = ('dos', 'emx', 'os2', '.i386-win32', '.source');
 DefDiffFN = 'install.add';
 PackageStr = 'package=';
 UnitsStr = 'units-';
 ZipExt = '.zip';


type
 PPackageRec = ^TPackageRec;
 TPackageRec = object (TObject)
  Name, ShortName, Desc: PString;
  Target: byte;
  constructor Init (ALine: string);
  function GetKeyStr: string;
  function GetLine: string;
  function GetSrcLine: string;
  destructor Done; virtual;
 end;

 PPackageCollection = ^TPackageCollection;
 TPackageCollection = object (TSortedCollection)
  constructor Load (FN: string);
  function LoadFile (FN: string; DupSrc: PPackageCollection): boolean;
  function WriteFile (FN: string): boolean;
  function Compare (Key1, Key2: pointer): sw_integer; virtual;
 end;

 PDatFile = ^TDatFile;
 TDatFile = object (TObject)
  DatCollection, LstCollection: PPackageCollection;
  constructor LoadDat (FN: string);
  function ReadLstFile (FN: string): boolean;
  function WriteNew (FN: string): boolean;
  destructor Done; virtual;
 end;


function LoCase (S: string): string;
var
 I: longint;
begin
 for I := 1 to Length (S) do
  if S [I] in ['A'..'Z'] then
   S [I] := char (Ord (S [I]) + 32);
 LoCase := S;
end;


constructor TPackageRec.Init (ALine: string);
var
 I: longint;
 J: byte;
 N, SN, D, TS: string;
 ALine2: string;
begin
 inherited Init;
 N := '';
 SN := '';
 D := '';
 TS := '';
 ALine2 := LoCase (ALine);
 if Copy (ALine2, 1, Length (PackageStr)) = PackageStr then
  begin
   Delete (ALine, 1, Length (PackageStr));
   I := Pos ('[', ALine);
   if I = 0 then
    begin
     I := Pos (',', ALine);
     if I = 0 then
      I := Succ (Length (ALine));
    end
   else
    begin
     SN := Copy (ALine, Succ (I), Pos (',', ALine) - I - 2);
     Delete (ALine, I, Length (SN) + 2);
    end;
   N := Copy (ALine, 1, Pred (I));
   if Length (N) <= 12 then
    SN := N
   else if (Copy (N, 1, Length (UnitsStr)) = UnitsStr) and
                                    (Length (N) - Length (UnitsStr) <= 11) then
    SN := 'u' + Copy (N, Succ (Length (UnitsStr)),
                                               Length (N) - Length (UnitsStr));
   D := Copy (ALine, Succ (I), Length (ALine) - I);
  end;

 Name := NewStr (N);
 if SN <> '' then
  ShortName := NewStr (SN)
 else
  ShortName := nil;
 Desc := NewStr (D);
 Target := 0;

 if SN <> '' then
  begin
   TS := LoCase (Copy (SN, Length (SN) - Length (ZipExt) - 2, 3));
   if Length (TS) <> 3 then
    TS := ''
   else
    for J := 1 to MaxTarget do
     if TS = TargetListShort [J] then
      begin
       Target := J;
       Break;
      end;
  end
 else
  begin
   I := Length (N) - Length (ZipExt);
   while (I > 0) and (N [I] <> '.') do
    Dec (I);
   if I = 0 then
    TS := LoCase (Copy (N, Length (SN) - Length (ZipExt) - 2, 3))
   else
    TS := LoCase (Copy (N, I, Length (N) - Length (ZipExt) - I + 1));
   for J := 1 to MaxTarget do
    if TS = TargetListLong [J] then
     begin
      Target := J;
      Break;
     end;
  end;
 if N = '' then
  begin
   WriteLn ('Err: Init failed (', ALine, ')!');
   Fail;
  end;
end;


destructor TPackageRec.Done;
begin
 DisposeStr (Name);
 if ShortName <> nil then
  DisposeStr (ShortName);
 DisposeStr (Desc);
 inherited Done;
end;


function TPackageRec.GetKeyStr: string;
var
 G: string;
begin
 if ShortName <> nil then
  begin
   if Target = 0 then
    G := LoCase (Copy (ShortName^, 1, Length (ShortName^) - Length (ZipExt)))
   else
    G := LoCase (Copy (ShortName^, 1, Length (ShortName^) - Length (ZipExt) - 3));
  end
 else
  begin
   if Name = nil then
    begin
     GetKeyStr := '';
     WriteLn ('Err - GetKeyStr (nil)!');
     Exit;
    end;
   if Target = 0 then
    G := LoCase (Copy (Name^, 1, Length (Name^) - Length (ZipExt)))
   else
    begin
     if Copy (LoCase (Name^), 1, Length (UnitsStr)) = UnitsStr then
      G := 'u' + LoCase (Copy (Name^, Succ (Length (UnitsStr)),
         Length (Name^) - Length (UnitsStr) - Length (TargetListLong [Target])
                                                            - Length (ZipExt)))
     else
      G := LoCase (Copy (Name^, 1,
         Length (Name^) - Length (TargetListLong [Target]) - Length (ZipExt)));
    end;
  end;

 G := G + '.';
 if Target <> 0 then
  G := G + TargetListShort [Target];
 GetKeyStr := G;
end;


function TPackageRec.GetLine: string;
var
 G: string;
begin
 G := PackageStr + Name^;
 if ShortName <> nil then
  G := G + '[' + ShortName^ + ']';
 if Length (Desc^) <= 45 then
  G := G + ',' + Desc^
 else
  G := G + ',' + Copy (Desc^, 1, 45) + '|' +
                                         Copy (Desc^, 46, Length (Desc^) - 45);
 GetLine := G;
end;


function TPackageRec.GetSrcLine: string;
var
 GS: string;
begin
 if Target = 0 then
  GS := ''
 else
  begin
   GS := PackageStr + Copy (Name^, 1,
     Length (Name^) - Length (TargetListLong [Target]) - Length (ZipExt)) +
                                           TargetListLong [MaxTarget] + ZipExt;
   if ShortName <> nil then
    GS := GS + '[' + Copy (ShortName^, 1, Length (ShortName^)
                    - Length (TargetListShort [Target]) - Length (ZipExt)) +
                                    TargetListShort [MaxTarget] + ZipExt + ']';
   GS := GS + ',' + Desc^;
  end;
 GetSrcLine := GS;
end;


constructor TDatFile.LoadDat (FN: string);
begin
 Init;
 New (DatCollection, Load (FN));
 New (LstCollection, Init (100, 50)); (* false? *)
end;


function TDatFile.ReadLstFile (FN: string): boolean;
begin
 ReadLstFile := LstCollection^.LoadFile (FN, DatCollection);
end;


function TDatFile.WriteNew (FN: string): boolean;
begin
 WriteNew := LstCollection^.WriteFile (FN);
end;


destructor TDatFile.Done;
begin
 Dispose (DatCollection, Done);
 Dispose (LstCollection, Done);
 inherited Done;
end;


constructor TPackageCollection.Load (FN: string);
begin
 Init (100, 50);
 if not (LoadFile (FN, nil)) then
  Fail;
end;


function TPackageCollection.LoadFile (FN: string; DupSrc: PPackageCollection): boolean;
var
 F: text;
 S: ansistring;
 S2: string;
 P, Q: PPackageRec;
 I: SW_Integer;
begin
{$I-}
 Assign (F, FN);
 Reset (F);
 while not (Eof (F)) {and (LastErr = 0)} do
  begin
   S := '';
   ReadLn (F, S);
   if (Length (S) > 255) then
    begin
     WriteLn ('Error: Line too long!');
     WriteLn (S);
     Halt (255); (* Change error handling *)
    end;
   if Copy (LoCase (S), 1, Length (PackageStr)) = PackageStr then
    begin
     New (P, Init (S));
     if DupSrc = nil then
      S2 := ''
     else
      S2 := P^.GetSrcLine;
     if (DupSrc = nil) or not (DupSrc^.Search (P, I)) then
      Insert (P)
     else
      Dispose (P, Done);
     if S2 <> '' then
      begin
       New (Q, Init (S2));
       if (Q <> nil) and not (Search (Q, I)) and
                           ((DupSrc = nil) or not (DupSrc^.Search (Q, I))) then
        Insert (Q)
       else
        Dispose (Q, Done);
      end;
    end;
  end;
 Close (F);
 LoadFile := IOResult = 0;
{
 if P = nil then Fail else
 begin
  if P^.LastErr <> 0 then
  begin
   Dispose (P, Done);
   Fail;
  end else
  begin
   P^.ReadIni (@Self);
   Dispose (P, Done);
  end;
 end;
}
end;


function TPackageCollection.WriteFile (FN: string): boolean;
var
 F: text;
 S: string;
 P: PPackageRec;
 I: SW_Integer;
 J: byte;
begin
 Assign (F, FN);
 Rewrite (F);
 for J := 0 to MaxTarget do
  for I := 0 to Count - 1 do
   begin
    P := At (I);
    if (P <> nil) and (P^.Target = J) then
     begin
{ Write (P^.Name^, '|');
   if P^.ShortName <> nil then
    Write (P^.ShortName^, '|')
   else
    Write ('x|');
   WriteLn (P^.Desc^, '|', P^.Target);
 WriteLn (P^.GetKeyStr);
}
      S := P^.GetLine;
(* Signalize too long description *)
      WriteLn (F, S);
     end;
   end;
 Close (F);
 WriteFile := IOResult = 0;
end;


function TPackageCollection.Compare (Key1, Key2: pointer): SW_Integer;
var
 S1, S2: string;
begin
 S1 := LoCase (PPackageRec (Key1)^.GetKeyStr);
 S2 := LoCase (PPackageRec (Key2)^.GetKeyStr);
 if S1 < S2 then
  Compare := -1
 else if S1 > S2 then
  Compare := 1
 else
  Compare := 0;
end;


function Base (const S: string): string;
var
 D: DirStr;
 N: NameStr;
 E: ExtStr;
begin
 FSplit (S, D, N, E);
 Base := N;
end;


procedure Error (const S: string; B: byte);
begin
 WriteLn;
 WriteLn ('Error: ', S, '!!');
 Halt (B);
end;


procedure Syntax;
begin
 WriteLn;
 WriteLn ('Syntax: ', Base (ParamStr (0)),
                          ' <path_to_install.dat> <LstFile1> [<LstFile2>...]');
 WriteLn;
 WriteLn ('<LstFileN> files are expected to be in the format produced by fpmake');
 WriteLn ('(e.g. using ''fpmake pkglist --target=<FPC_target> -zp units-''');
 WriteLn ('for unit packages or without the ''-zp <prefix>'' parameter for utils).');
 WriteLn;
 WriteLn ('Program compares their content to the list of packages in the text-mode');
 WriteLn ('installer configuration file install.dat and creates file install.add');
 WriteLn ('with information about packages missing in install.dat in a form allowing');
 WriteLn ('copy&paste of individual lines into install.dat.');
 WriteLn;
 WriteLn ('If the original description of a certain package as found in fpmake.pp is');
 WriteLn ('too long for install.dat, the maximum length is marked in the respective line');
 WriteLn ('in install.add using a pipe character (''|'') to give hint for manual editing.');
 Halt;
end;

var
 I, J, K: byte;
 DAT: TDatFile;
 PrevCount: SW_Integer;
 SR: SearchRec;
 D: DirStr;
 N: NameStr;
 E: ExtStr;

begin
 J := ParamCount;
 if J < 2 then
  begin
   WriteLn;
   WriteLn ('Error: Too few parameters!!');
   Syntax;
  end;
 DAT.LoadDat (ParamStr (1));
 if DAT.DatCollection <> nil then
  WriteLn (LineEnding +
            'Source install.dat file (', ParamStr (1), ') loaded correctly: ',
                                          DAT.DatCollection^.Count, ' records')
 else
  Error ('Failure while loading source install.dat file (' + ParamStr (1) +
                                                                       ')', 1);
 K := 0;
 for I := 2 to J do
  begin
   FSplit (ParamStr (I), D, N, E);
   FindFirst (ParamStr (I), AnyFile - Directory, SR);
   if DosError <> 0 then
    Error ('No package listing file found for "' + ParamStr (I) + '"', I)
   else
    begin
     while (DosError = 0) do
      begin
       Inc (K);
       PrevCount := DAT.LstCollection^.Count;
       if DAT.ReadLstFile (D + SR.Name) then
        WriteLn ('Package listing #', K, ' (', D + SR.Name,
          ') loaded correctly: ', DAT.LstCollection^.Count - PrevCount,
                                                                ' new records')
       else
        Error ('Failure while loading package listing (' + D + SR.Name + ')',
                                                                        J + K);
       FindNext (SR);
      end;
     FindClose (SR);
    end;
  end;
 WriteLn ('Total: ', DAT.LstCollection^.Count, ' new records');
 if DAT.WriteNew (DefDiffFN) then
  WriteLn ('Output file (' + DefDiffFN + ') created successfully.')
 else
  Error ('Failure while trying to write records to the output file (' +
                                                DefDiffFN + ')', Succ (J) + K);
 DAT.Done;
end.
