{
    Free Pascal port of the OpenPTC C++ library.
    Copyright (C) 2001-2003  Nikolay Nikolov (nickysn@users.sourceforge.net)
    Original C++ version by Glenn Fiedler (ptc@gaffer.org)

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
}

Type
  TPTCTimer=Class(TObject)
  Private
    m_old : Double;
    m_time : Double;
    m_start : Double;
    m_current : Double;
    m_running : Boolean;
    {$IFDEF WIN32}
    m_frequency : QWord;
    {$ENDIF WIN32}
    Function clock : Double;
    Procedure internal_init_timer;
  Public
    Constructor Create;
    Constructor Create(_time : Double);
    Constructor Create(Const timer : TPTCTimer);
    Destructor Destroy; Override;
    Procedure Assign(Const timer : TPTCTimer);
    Function Equals(Const timer : TPTCTimer) : Boolean;
    Procedure settime(_time : Double); {was 'set' in the C++ version}
    Procedure start;
    Procedure stop;
    Function time : Double;
    Function delta : Double;
    Function resolution : Double;
  End;
