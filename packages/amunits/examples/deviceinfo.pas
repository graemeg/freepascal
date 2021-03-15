
{ *************************************************************************
  **                                                                     **
  **  DeviceInfo  -  Version 1.0,         (C)1992  by  Thomas Schmid     **
  **                                                 Im Grenzacherhof 12 **
  **   This programm is Public Domain,               CH- 4058 Basel      **
  **   coded 18.02.1992 in PCQ-Pascal(1.2b).                             **
  **   Gibt Info über angegebenes Device aus.                            **
  **                                             Usage :                 **
  **                                                    DeviceInfo Dfx:  **
  **                                                                     **
  *************************************************************************
}

{
   Translated to fpc pascal.
   24 Mar 2001.

   nils.sjoholm@mailbox.swipnet.se
}

Program DeviceInfo;

uses exec,amigados;

Const
  MaxSize = 80;

Var
  MyLock          : longint;
  Inf             : pInfoData;
  Ok              : Boolean;
  Myfile          : string;
  S, S1           : String;
  Size, Used, Bpb : Integer;

Procedure Cls;

Begin
  WriteLn('   DeviceInfo V1.0 © 1992, by T.Schmid, Written in PCQ V1.2b',#10);

End;

Procedure AsdaLaVista(warum : String ; code : longint);

Begin
  If Inf   <> Nil Then FreeMem(Inf);
  If warum <> '' Then WriteLn('[3;32m',warum,'[0;31m');
  halt(code);
End;


Begin


  If ParamCount = 0 Then AsdaLaVista(' DiskInfo V1.0, © 1992 T.Schmid - Usage : DiskInfo Dfx:',0);
  MyFile := ParamStr(1) + #0;

  Inf:=pInfoData(AllocMem( SizeOf(tInfoData) ) );
  If Inf=Nil Then AsdaLaVista('No memory',5);

  s:= 'Writeenabled';
  s1:= 'Dos';

  MyLock:=Lock(@Myfile[1],ACCESS_READ);
  If MyLock = 0 Then AsdaLaVista('Can''t get a lock.',5);

  Ok:=Info(MyLock,Inf);
  Unlock(MyLock);               { ------- Wichtig !! -------- }

  If Ok = FALSE Then AsdaLaVista('Can''t get info on this Device.',10);

  Bpb  := Inf^.id_BytesPerBlock;
  Size := Inf^.id_NumBlocks     * Bpb DIV 1024;
  Used := Inf^.id_NumBlocksUsed * Bpb DIV 1024;
  Cls;

  WriteLn('   Info about Device          :  [0;33m', Myfile, '[0;31m');
  WriteLn('   Size                       :  [0;33m', Size, ' KBytes  ','[0;31m');
  WriteLn('   Size used                  :  [0;33m', Used, ' KBytes  ','[0;31m');
  WriteLn('   Free                       :  [0;33m', Size-Used, ' KBytes  ','[0;31m');
  WriteLn('   Number of Bytes per Block  :  [0;33m', Inf^.id_BytesPerBlock, '[0;31m');

  Case Inf^.id_DiskType of
        ID_NO_DISK_PRESENT : S1:='No Disk';
        ID_UNREADABLE_DISK : S1:='Can''t read Disk';
        ID_NOT_REALLY_DOS  : S1:='No Dos-Disk';
        ID_KICKSTART_DISK  : S1:='Kickstart-Disk';
  End;

  WriteLn('   Disk Type                  :  [0;33m',S1,'[0;31m');
  WriteLn('   Type of error              :  [0;33m',Inf^.id_NumSoftErrors,'[0;31m');

  Case Inf^.id_DiskState of
        ID_WRITE_PROTECTED : S:='Writeprotected';
        ID_VALIDATING      : S:='Is Validated';
  End;
  WriteLn('   Device Status              :  [0;33m',S,'[0;31m');

  { wichtig : }
  AsdaLaVista(#10 + '  C U in next CF !' + #10,0);

End.
