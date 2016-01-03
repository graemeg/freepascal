{
    Free Pascal port of the OpenPTC C++ library.
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    Copyright (C) 2001-2003, 2006, 2007, 2009-2011  Nikolay Nikolov (nickysn@users.sourceforge.net)
=======
    Copyright (C) 2001-2003  Nikolay Nikolov (nickysn@users.sourceforge.net)
>>>>>>> graemeg/cpstrnew
=======
    Copyright (C) 2001-2003  Nikolay Nikolov (nickysn@users.sourceforge.net)
>>>>>>> graemeg/cpstrnew
=======
    Copyright (C) 2001-2003  Nikolay Nikolov (nickysn@users.sourceforge.net)
>>>>>>> graemeg/cpstrnew
=======
    Copyright (C) 2001-2003  Nikolay Nikolov (nickysn@users.sourceforge.net)
>>>>>>> origin/cpstrnew
    Original C++ version by Glenn Fiedler (ptc@gaffer.org)

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
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

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
}

type
  TPTCClipper = class
  public
    { clip a single area against clip area }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
    class function Clip(AArea, AClip: IPTCArea): IPTCArea;
    { clip source and destination areas against source and destination clip areas }
    class procedure Clip(ASource, AClipSource: IPTCArea;
                         out AClippedSource: IPTCArea;
                         ADestination, AClipDestination: IPTCArea;
                         out AClippedDestination: IPTCArea);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
    class function Clip(const AArea, AClip: TPTCArea): TPTCArea;
    { clip source and destination areas against source and destination clip areas }
    class procedure Clip(const ASource, AClipSource, AClippedSource,
                         ADestination, AClipDestination,
                         AClippedDestination: TPTCArea);
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
