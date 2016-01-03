<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> origin/fixes_2.4
{*****************************************************************************
      OS Memory allocation / deallocation
 ****************************************************************************}

function ___sbrk(size:longint):pointer;cdecl; external name '___sbrk';

function SysOSAlloc(size: ptrint): pointer;assembler;
asm
{$ifdef SYSTEMDEBUG}
        cmpb    $1,accept_sbrk
        je      .Lsbrk
        movl    $0,%eax
        jmp     .Lsbrk_fail
      .Lsbrk:
{$endif}
        movl    size,%eax
        pushl   %eax
        call    ___sbrk
        addl    $4,%esp
{$ifdef SYSTEMDEBUG}
      .Lsbrk_fail:
{$endif}
end;

{ define HAS_SYSOSFREE}

procedure SysOSFree(p: pointer; size: ptrint);
begin
end;

<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
{*****************************************************************************
      OS Memory allocation / deallocation
 ****************************************************************************}

function ___sbrk(size:longint):pointer;cdecl; external name '___sbrk';

function SysOSAlloc(size: ptrint): pointer;assembler;
asm
{$ifdef SYSTEMDEBUG}
        cmpb    $1,accept_sbrk
        je      .Lsbrk
        movl    $0,%eax
        jmp     .Lsbrk_fail
      .Lsbrk:
{$endif}
        movl    size,%eax
        pushl   %eax
        call    ___sbrk
        addl    $4,%esp
{$ifdef SYSTEMDEBUG}
      .Lsbrk_fail:
{$endif}
end;

{ define HAS_SYSOSFREE}

procedure SysOSFree(p: pointer; size: ptrint);
begin
end;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
