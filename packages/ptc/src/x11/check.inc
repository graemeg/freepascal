{
    Free Pascal port of the OpenPTC C++ library.
    Copyright (C) 2001-2003  Nikolay Nikolov (nickysn@users.sourceforge.net)
    Original C++ version by Glenn Fiedler (ptc@gaffer.org)

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
<<<<<<< HEAD
<<<<<<< HEAD
    version 2.1 of the License, or (at your option) any later version
    with the following modification:

    As a special exception, the copyright holders of this library give you
    permission to link this library with independent modules to produce an
    executable, regardless of the license terms of these independent modules,and
    to copy and distribute the resulting executable under terms of your choice,
    provided that you also meet, for each linked independent module, the terms
    and conditions of the license of that module. An independent module is a
    module which is not derived from or based on this library. If you modify
    this library, you may extend this exception to your version of the library,
    but you are not obligated to do so. If you do not wish to do so, delete this
    exception statement from your version.
=======
    version 2.1 of the License, or (at your option) any later version.
>>>>>>> graemeg/fixes_2_2
=======
    version 2.1 of the License, or (at your option) any later version.
>>>>>>> origin/fixes_2_2

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
}

<<<<<<< HEAD
<<<<<<< HEAD
procedure X11Check(AStatus: TStatus);
begin
  if AStatus <> 0 then
    exit;

  raise TPTCError.Create('X11 Error');
end;

procedure X11Check(AStatus: TStatus; const AMessage: string);
begin
  try
    X11Check(AStatus);
  except
    on error: TPTCError do
      raise TPTCError.Create(AMessage, error);
  end;
end;

procedure X11CheckSuccess(AStatus: TStatus);
var
  ErrStr: string;
begin
  if AStatus = Success then
    exit;
  case AStatus of
    BadRequest  : ErrStr := 'BadRequest';
    BadValue    : ErrStr := 'BadValue';
    BadWindow   : ErrStr := 'BadWindow';
    BadPixmap   : ErrStr := 'BadPixmap';
    BadAtom     : ErrStr := 'BadAtom';
    BadCursor   : ErrStr := 'BadCursor';
    BadFont     : ErrStr := 'BadFont';
    BadMatch    : ErrStr := 'BadMatch';
    BadDrawable : ErrStr := 'BadDrawable';
    BadAccess   : ErrStr := 'BadAccess';
    BadAlloc    : ErrStr := 'BadAlloc';
    BadColor    : ErrStr := 'BadColor';
    BadGC       : ErrStr := 'BadGC';
    BadIDChoice : ErrStr := 'BadIDChoice';
    BadName     : ErrStr := 'BadName';
    BadLength   : ErrStr := 'BadLength';
    BadImplementation: ErrStr := 'BadImplementation';
    else
      ErrStr := IntToStr(AStatus);
  end;
  raise TPTCError.Create('X11 Error: ' + ErrStr);
end;

procedure X11CheckSuccess(AStatus: TStatus; const AMessage: String);
begin
  try
    X11CheckSuccess(AStatus);
  except
    on error: TPTCError do
      raise TPTCError.Create(AMessage, error);
  end;
end;
=======
=======
>>>>>>> origin/fixes_2_2
Procedure X11Check(result : TStatus);

{Var
  ErrStr : String;}

Begin
  {todo: fix X11 error handling}
{  If result = Success Then
    Exit;
  Case result Of
    BadRequest        : ErrStr := 'BadRequest';
    BadValue          : ErrStr := 'BadValue';
    BadWindow         : ErrStr := 'BadWindow';
    BadPixmap         : ErrStr := 'BadPixmap';
    BadAtom           : ErrStr := 'BadAtom';
    BadCursor         : ErrStr := 'BadCursor';
    BadFont           : ErrStr := 'BadFont';
    BadMatch          : ErrStr := 'BadMatch';
    BadDrawable       : ErrStr := 'BadDrawable';
    BadAccess         : ErrStr := 'BadAccess';
    BadAlloc          : ErrStr := 'BadAlloc';
    BadColor          : ErrStr := 'BadColor';
    BadGC             : ErrStr := 'BadGC';
    BadIDChoice       : ErrStr := 'BadIDChoice';
    BadName           : ErrStr := 'BadName';
    BadLength         : ErrStr := 'BadLength';
    BadImplementation : ErrStr := 'BadImplementation';
    Else
      Str(result, ErrStr);
  End;
  Raise TPTCError.Create('X11 Error: ' + ErrStr);}
End;

Procedure X11Check(result : TStatus; Const message : String);

Begin
  Try
    X11Check(result);
  Except
    On error : TPTCError Do
      Raise TPTCError.Create(message, error);
  End;
End;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
