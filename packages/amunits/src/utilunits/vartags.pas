{
    This file is part of the Free Pascal run time library.

    A file in Amiga system run time library.
    Copyright (c) 2000-2003 by Nils Sjoholm
    member of the Amiga RTL development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

unit vartags;


{

  This is I hope, a unit that will go away when fpc
  have array of const. For now it will help to create
  taglists for the functions that need it.
  25 Jan 2000.

  Added const ltrue and lfalse, for work with taglists.
  16 Jul 2000.

  Moved SetTags and TagItem from tagutils.inc to this
  unit.
  Removed tagutils.inc from amigainc.
  23 Jul 2000.

  Use tagsarray instead.
  09 Nov 2002.

  Added the define use_amiga_smartlink.
  13 Jan 2003.

  nils.sjoholm@mailbox.swipnet.se

}


{
  Here is an example on how to use TAGS.

    win := OpenWindowTagList(NIL, TAGS(
                             WA_Width,  400,
                             WA_Activate,    ltrue,
                             WA_Height, 100,
                             WA_CloseGadget, ltrue,
                             WA_Title,  Longstr('Menu Test Window'),
                             WA_IDCMP,  IDCMP_CLOSEWINDOW or IDCMP_MENUPICK,
                             TAG_END));

}

interface

uses utility;

type long = longint;

const
{
   This consts is for taglists, no need to cast
   longint(byte(true)). Just use ltrue instead.
}
   ltrue : longint =  1;
   lfalse : longint = 0;

var
   argbuff : array[0..30] of tTagItem;

function LongStr(const s : string) : Longint;

procedure SetTags(ti : pTagItem; item, data : Longint);
function TagItem(item, data : Longint): tTagItem;

function TAGS(a,b,c:long):pointer;
function TAGS(a,b,c,d,e:long):pointer;
function TAGS(a,b,c,d,e,f,g:long):pointer;
function TAGS(a,b,c,d,e,f,g,h,i:long):pointer;
function TAGS(a,b,c,d,e,f,g,h,i,j,k:long):pointer;
function TAGS(a,b,c,d,e,f,g,h,i,j,k,l,m:long):pointer;
function TAGS(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o:long):pointer;
function TAGS(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q:long):pointer;
function TAGS(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s:long):pointer;
function TAGS(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t,u:long):pointer;
function TAGS(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t,u,v,w:long):pointer;
function TAGS(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t,u,v,w,x,z:long):pointer;
function TAGS(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t,v,w,x,z,aa,bb,cc:long):pointer;
function TAGS(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t,v,w,x,z,aa,bb,cc,
              dd,ee:long):pointer;
function TAGS(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t,v,w,x,z,aa,bb,cc,
              dd,ee,ff,gg:long):pointer;
function TAGS(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t,v,w,x,z,aa,bb,cc,
              dd,ee,ff,gg,hh,ii:long):pointer;
function TAGS(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t,v,w,x,z,aa,bb,cc,
              dd,ee,ff,gg,hh,ii,jj,kk:long):pointer;
function TAGS(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t,v,w,x,z,aa,bb,cc,
              dd,ee,ff,gg,hh,ii,jj,kk,ll,mm:long):pointer;
function TAGS(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t,v,w,x,z,aa,bb,cc,
              dd,ee,ff,gg,hh,ii,jj,kk,ll,mm,nn,oo:long):pointer;
function TAGS(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t,v,w,x,z,aa,bb,cc,
              dd,ee,ff,gg,hh,ii,jj,kk,ll,mm,nn,oo,pp,qq:long):pointer;
function TAGS(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t,v,w,x,z,aa,bb,cc,
              dd,ee,ff,gg,hh,ii,jj,kk,ll,mm,nn,oo,pp,qq,rr,ss:long):pointer;
function TAGS(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t,v,w,x,z,aa,bb,cc,
              dd,ee,ff,gg,hh,ii,jj,kk,ll,mm,nn,oo,pp,qq,rr,ss,tt,uu:long):pointer;
function TAGS(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t,v,w,x,z,aa,bb,cc,
              dd,ee,ff,gg,hh,ii,jj,kk,ll,mm,nn,oo,pp,qq,rr,ss,tt,uu,vv,ww:long):pointer;

implementation

uses pastoc;

procedure SetTags(ti : pTagItem; item, data : Longint);
begin
    with ti^ do begin
       ti_Tag := item;
       ti_Data := data;
    end;
end;

function TagItem(item, data : Longint): tTagItem;
var
   temp : tTagItem;
begin
   with temp do begin
      ti_Tag := item;
      ti_Data := data;
   end;
   TagItem := temp;
end;

function LongStr(const s : string) : Longint;
begin
   LongStr := Longint(Pas2C(s));
end;

function TAGS(a,b,c:long):pointer;
begin
    argbuff[0] := TagItem(a,b);
    argbuff[1].ti_Tag := c;
    TAGS := @argbuff;
end;

function TAGS(a,b,c,d,e:long):pointer;
begin
    argbuff[0] := TagItem(a,b);
    argbuff[1] := TagItem(c,d);
    argbuff[2].ti_Tag := e;
    TAGS := @argbuff;
end;

function TAGS(a,b,c,d,e,f,g:long):pointer;
begin
    argbuff[0] := TagItem(a,b);
    argbuff[1] := TagItem(c,d);
    argbuff[2] := TagItem(e,f);
    argbuff[3].ti_Tag := g;
    TAGS := @argbuff;
end;

function TAGS(a,b,c,d,e,f,g,h,i:long):pointer;
begin
    argbuff[0] := TagItem(a,b);
    argbuff[1] := TagItem(c,d);
    argbuff[2] := TagItem(e,f);
    argbuff[3] := TagItem(g,h);
    argbuff[4].ti_Tag := i;
    TAGS := @argbuff;
end;

function TAGS(a,b,c,d,e,f,g,h,i,j,k:long):pointer;
begin
    argbuff[0] := TagItem(a,b);
    argbuff[1] := TagItem(c,d);
    argbuff[2] := TagItem(e,f);
    argbuff[3] := TagItem(g,h);
    argbuff[4] := TagItem(i,j);
    argbuff[5].ti_Tag := k;
    TAGS := @argbuff;
end;

function TAGS(a,b,c,d,e,f,g,h,i,j,k,l,m:long):pointer;
begin
    argbuff[0] := TagItem(a,b);
    argbuff[1] := TagItem(c,d);
    argbuff[2] := TagItem(e,f);
    argbuff[3] := TagItem(g,h);
    argbuff[4] := TagItem(i,j);
    argbuff[5] := TagItem(k,l);
    argbuff[6].ti_Tag := m;
    TAGS := @argbuff;
end;

function TAGS(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o:long):pointer;
begin
    argbuff[0] := TagItem(a,b);
    argbuff[1] := TagItem(c,d);
    argbuff[2] := TagItem(e,f);
    argbuff[3] := TagItem(g,h);
    argbuff[4] := TagItem(i,j);
    argbuff[5] := TagItem(k,l);
    argbuff[6] := TagItem(m,n);
    argbuff[7].ti_Tag := o;
    TAGS := @argbuff;
end;

function TAGS(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q:long):pointer;
begin
    argbuff[0] := TagItem(a,b);
    argbuff[1] := TagItem(c,d);
    argbuff[2] := TagItem(e,f);
    argbuff[3] := TagItem(g,h);
    argbuff[4] := TagItem(i,j);
    argbuff[5] := TagItem(k,l);
    argbuff[6] := TagItem(m,n);
    argbuff[7] := TagItem(o,p);
    argbuff[8].ti_Tag := q;
    TAGS := @argbuff;
end;

function TAGS(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s:long):pointer;
begin
    argbuff[0] := TagItem(a,b);
    argbuff[1] := TagItem(c,d);
    argbuff[2] := TagItem(e,f);
    argbuff[3] := TagItem(g,h);
    argbuff[4] := TagItem(i,j);
    argbuff[5] := TagItem(k,l);
    argbuff[6] := TagItem(m,n);
    argbuff[7] := TagItem(o,p);
    argbuff[8] := TagItem(q,r);
    argbuff[9].ti_Tag := s;
    TAGS := @argbuff;
end;

function TAGS(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t,u:long):pointer;
begin
    argbuff[0] := TagItem(a,b);
    argbuff[1] := TagItem(c,d);
    argbuff[2] := TagItem(e,f);
    argbuff[3] := TagItem(g,h);
    argbuff[4] := TagItem(i,j);
    argbuff[5] := TagItem(k,l);
    argbuff[6] := TagItem(m,n);
    argbuff[7] := TagItem(o,p);
    argbuff[8] := TagItem(q,r);
    argbuff[9] := TagItem(s,t);
    argbuff[10].ti_Tag := u;
    TAGS := @argbuff;
end;

function TAGS(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t,u,v,w:long):pointer;
begin
    argbuff[0] := TagItem(a,b);
    argbuff[1] := TagItem(c,d);
    argbuff[2] := TagItem(e,f);
    argbuff[3] := TagItem(g,h);
    argbuff[4] := TagItem(i,j);
    argbuff[5] := TagItem(k,l);
    argbuff[6] := TagItem(m,n);
    argbuff[7] := TagItem(o,p);
    argbuff[8] := TagItem(q,r);
    argbuff[9] := TagItem(s,t);
    argbuff[10] := TagItem(u,v);
    argbuff[11].ti_Tag := w;
    TAGS := @argbuff;
end;

function TAGS(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t,u,v,w,x,z:long):pointer;
begin
    argbuff[0] := TagItem(a,b);
    argbuff[1] := TagItem(c,d);
    argbuff[2] := TagItem(e,f);
    argbuff[3] := TagItem(g,h);
    argbuff[4] := TagItem(i,j);
    argbuff[5] := TagItem(k,l);
    argbuff[6] := TagItem(m,n);
    argbuff[7] := TagItem(o,p);
    argbuff[8] := TagItem(q,r);
    argbuff[9] := TagItem(s,t);
    argbuff[10] := TagItem(u,v);
    argbuff[11] := TagItem(w,x);
    argbuff[12].ti_Tag := z;
    TAGS := @argbuff;
end;

function TAGS(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t,v,w,x,z,aa,bb,cc:long):pointer;
begin
    argbuff[0] := TagItem(a,b);
    argbuff[1] := TagItem(c,d);
    argbuff[2] := TagItem(e,f);
    argbuff[3] := TagItem(g,h);
    argbuff[4] := TagItem(i,j);
    argbuff[5] := TagItem(k,l);
    argbuff[6] := TagItem(m,n);
    argbuff[7] := TagItem(o,p);
    argbuff[8] := TagItem(q,r);
    argbuff[9] := TagItem(s,t);
    argbuff[10] := TagItem(v,w);
    argbuff[11] := TagItem(x,z);
    argbuff[12] := TagItem(aa,bb);
    argbuff[13].ti_Tag := cc;
    TAGS := @argbuff;
end;

function TAGS(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t,v,w,x,z,aa,bb,cc,
              dd,ee:long):pointer;
begin
    argbuff[0] := TagItem(a,b);
    argbuff[1] := TagItem(c,d);
    argbuff[2] := TagItem(e,f);
    argbuff[3] := TagItem(g,h);
    argbuff[4] := TagItem(i,j);
    argbuff[5] := TagItem(k,l);
    argbuff[6] := TagItem(m,n);
    argbuff[7] := TagItem(o,p);
    argbuff[8] := TagItem(q,r);
    argbuff[9] := TagItem(s,t);
    argbuff[10] := TagItem(v,w);
    argbuff[11] := TagItem(x,z);
    argbuff[12] := TagItem(aa,bb);
    argbuff[13] := TagItem(cc,dd);
    argbuff[14].ti_Tag := ee;
    TAGS := @argbuff;
end;

function TAGS(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t,v,w,x,z,aa,bb,cc,
              dd,ee,ff,gg:long):pointer;
begin
    argbuff[0] := TagItem(a,b);
    argbuff[1] := TagItem(c,d);
    argbuff[2] := TagItem(e,f);
    argbuff[3] := TagItem(g,h);
    argbuff[4] := TagItem(i,j);
    argbuff[5] := TagItem(k,l);
    argbuff[6] := TagItem(m,n);
    argbuff[7] := TagItem(o,p);
    argbuff[8] := TagItem(q,r);
    argbuff[9] := TagItem(s,t);
    argbuff[10] := TagItem(v,w);
    argbuff[11] := TagItem(x,z);
    argbuff[12] := TagItem(aa,bb);
    argbuff[13] := TagItem(cc,dd);
    argbuff[14] := TagItem(ee,ff);
    argbuff[14].ti_Tag := gg;
    TAGS := @argbuff;
end;

function TAGS(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t,v,w,x,z,aa,bb,cc,
              dd,ee,ff,gg,hh,ii:long):pointer;
begin
    argbuff[0] := TagItem(a,b);
    argbuff[1] := TagItem(c,d);
    argbuff[2] := TagItem(e,f);
    argbuff[3] := TagItem(g,h);
    argbuff[4] := TagItem(i,j);
    argbuff[5] := TagItem(k,l);
    argbuff[6] := TagItem(m,n);
    argbuff[7] := TagItem(o,p);
    argbuff[8] := TagItem(q,r);
    argbuff[9] := TagItem(s,t);
    argbuff[10] := TagItem(v,w);
    argbuff[11] := TagItem(x,z);
    argbuff[12] := TagItem(aa,bb);
    argbuff[13] := TagItem(cc,dd);
    argbuff[14] := TagItem(ee,ff);
    argbuff[15] := TagItem(gg,hh);
    argbuff[16].ti_Tag := ii;
    TAGS := @argbuff;
end;

function TAGS(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t,v,w,x,z,aa,bb,cc,
              dd,ee,ff,gg,hh,ii,jj,kk:long):pointer;
begin
    argbuff[0] := TagItem(a,b);
    argbuff[1] := TagItem(c,d);
    argbuff[2] := TagItem(e,f);
    argbuff[3] := TagItem(g,h);
    argbuff[4] := TagItem(i,j);
    argbuff[5] := TagItem(k,l);
    argbuff[6] := TagItem(m,n);
    argbuff[7] := TagItem(o,p);
    argbuff[8] := TagItem(q,r);
    argbuff[9] := TagItem(s,t);
    argbuff[10] := TagItem(v,w);
    argbuff[11] := TagItem(x,z);
    argbuff[12] := TagItem(aa,bb);
    argbuff[13] := TagItem(cc,dd);
    argbuff[14] := TagItem(ee,ff);
    argbuff[15] := TagItem(gg,hh);
    argbuff[16] := TagItem(ii,jj);
    argbuff[17].ti_Tag := kk;
    TAGS := @argbuff;
end;

function TAGS(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t,v,w,x,z,aa,bb,cc,
              dd,ee,ff,gg,hh,ii,jj,kk,ll,mm:long):pointer;
begin
    argbuff[0] := TagItem(a,b);
    argbuff[1] := TagItem(c,d);
    argbuff[2] := TagItem(e,f);
    argbuff[3] := TagItem(g,h);
    argbuff[4] := TagItem(i,j);
    argbuff[5] := TagItem(k,l);
    argbuff[6] := TagItem(m,n);
    argbuff[7] := TagItem(o,p);
    argbuff[8] := TagItem(q,r);
    argbuff[9] := TagItem(s,t);
    argbuff[10] := TagItem(v,w);
    argbuff[11] := TagItem(x,z);
    argbuff[12] := TagItem(aa,bb);
    argbuff[13] := TagItem(cc,dd);
    argbuff[14] := TagItem(ee,ff);
    argbuff[15] := TagItem(gg,hh);
    argbuff[16] := TagItem(ii,jj);
    argbuff[17] := TagItem(kk,ll);
    argbuff[18].ti_Tag := mm;
    TAGS := @argbuff;
end;

function TAGS(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t,v,w,x,z,aa,bb,cc,
              dd,ee,ff,gg,hh,ii,jj,kk,ll,mm,nn,oo:long):pointer;
begin
    argbuff[0] := TagItem(a,b);
    argbuff[1] := TagItem(c,d);
    argbuff[2] := TagItem(e,f);
    argbuff[3] := TagItem(g,h);
    argbuff[4] := TagItem(i,j);
    argbuff[5] := TagItem(k,l);
    argbuff[6] := TagItem(m,n);
    argbuff[7] := TagItem(o,p);
    argbuff[8] := TagItem(q,r);
    argbuff[9] := TagItem(s,t);
    argbuff[10] := TagItem(v,w);
    argbuff[11] := TagItem(x,z);
    argbuff[12] := TagItem(aa,bb);
    argbuff[13] := TagItem(cc,dd);
    argbuff[14] := TagItem(ee,ff);
    argbuff[15] := TagItem(gg,hh);
    argbuff[16] := TagItem(ii,jj);
    argbuff[17] := TagItem(kk,ll);
    argbuff[18] := TagItem(mm,nn);
    argbuff[19].ti_Tag := oo;
    TAGS := @argbuff;
end;

function TAGS(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t,v,w,x,z,aa,bb,cc,
              dd,ee,ff,gg,hh,ii,jj,kk,ll,mm,nn,oo,pp,qq:long):pointer;
begin
    argbuff[0] := TagItem(a,b);
    argbuff[1] := TagItem(c,d);
    argbuff[2] := TagItem(e,f);
    argbuff[3] := TagItem(g,h);
    argbuff[4] := TagItem(i,j);
    argbuff[5] := TagItem(k,l);
    argbuff[6] := TagItem(m,n);
    argbuff[7] := TagItem(o,p);
    argbuff[8] := TagItem(q,r);
    argbuff[9] := TagItem(s,t);
    argbuff[10] := TagItem(v,w);
    argbuff[11] := TagItem(x,z);
    argbuff[12] := TagItem(aa,bb);
    argbuff[13] := TagItem(cc,dd);
    argbuff[14] := TagItem(ee,ff);
    argbuff[15] := TagItem(gg,hh);
    argbuff[16] := TagItem(ii,jj);
    argbuff[17] := TagItem(kk,ll);
    argbuff[18] := TagItem(mm,nn);
    argbuff[19] := TagItem(oo,pp);
    argbuff[20].ti_Tag := qq;
    TAGS := @argbuff;
end;

function TAGS(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t,v,w,x,z,aa,bb,cc,
              dd,ee,ff,gg,hh,ii,jj,kk,ll,mm,nn,oo,pp,qq,rr,ss:long):pointer;
begin
    argbuff[0] := TagItem(a,b);
    argbuff[1] := TagItem(c,d);
    argbuff[2] := TagItem(e,f);
    argbuff[3] := TagItem(g,h);
    argbuff[4] := TagItem(i,j);
    argbuff[5] := TagItem(k,l);
    argbuff[6] := TagItem(m,n);
    argbuff[7] := TagItem(o,p);
    argbuff[8] := TagItem(q,r);
    argbuff[9] := TagItem(s,t);
    argbuff[10] := TagItem(v,w);
    argbuff[11] := TagItem(x,z);
    argbuff[12] := TagItem(aa,bb);
    argbuff[13] := TagItem(cc,dd);
    argbuff[14] := TagItem(ee,ff);
    argbuff[15] := TagItem(gg,hh);
    argbuff[16] := TagItem(ii,jj);
    argbuff[17] := TagItem(kk,ll);
    argbuff[18] := TagItem(mm,nn);
    argbuff[19] := TagItem(oo,pp);
    argbuff[20] := TagItem(qq,rr);
    argbuff[21].ti_Tag := ss;
    TAGS := @argbuff;
end;

function TAGS(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t,v,w,x,z,aa,bb,cc,
              dd,ee,ff,gg,hh,ii,jj,kk,ll,mm,nn,oo,pp,qq,rr,ss,tt,uu:long):pointer;
begin
    argbuff[0] := TagItem(a,b);
    argbuff[1] := TagItem(c,d);
    argbuff[2] := TagItem(e,f);
    argbuff[3] := TagItem(g,h);
    argbuff[4] := TagItem(i,j);
    argbuff[5] := TagItem(k,l);
    argbuff[6] := TagItem(m,n);
    argbuff[7] := TagItem(o,p);
    argbuff[8] := TagItem(q,r);
    argbuff[9] := TagItem(s,t);
    argbuff[10] := TagItem(v,w);
    argbuff[11] := TagItem(x,z);
    argbuff[12] := TagItem(aa,bb);
    argbuff[13] := TagItem(cc,dd);
    argbuff[14] := TagItem(ee,ff);
    argbuff[15] := TagItem(gg,hh);
    argbuff[16] := TagItem(ii,jj);
    argbuff[17] := TagItem(kk,ll);
    argbuff[18] := TagItem(mm,nn);
    argbuff[19] := TagItem(oo,pp);
    argbuff[20] := TagItem(qq,rr);
    argbuff[21] := TagItem(ss,tt);
    argbuff[22].ti_Tag := uu;
    TAGS := @argbuff;
end;

function TAGS(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p,q,r,s,t,v,w,x,z,aa,bb,cc,
              dd,ee,ff,gg,hh,ii,jj,kk,ll,mm,nn,oo,pp,qq,rr,ss,tt,uu,vv,ww:long):pointer;
begin
    argbuff[0] := TagItem(a,b);
    argbuff[1] := TagItem(c,d);
    argbuff[2] := TagItem(e,f);
    argbuff[3] := TagItem(g,h);
    argbuff[4] := TagItem(i,j);
    argbuff[5] := TagItem(k,l);
    argbuff[6] := TagItem(m,n);
    argbuff[7] := TagItem(o,p);
    argbuff[8] := TagItem(q,r);
    argbuff[9] := TagItem(s,t);
    argbuff[10] := TagItem(v,w);
    argbuff[11] := TagItem(x,z);
    argbuff[12] := TagItem(aa,bb);
    argbuff[13] := TagItem(cc,dd);
    argbuff[14] := TagItem(ee,ff);
    argbuff[15] := TagItem(gg,hh);
    argbuff[16] := TagItem(ii,jj);
    argbuff[17] := TagItem(kk,ll);
    argbuff[18] := TagItem(mm,nn);
    argbuff[19] := TagItem(oo,pp);
    argbuff[20] := TagItem(qq,rr);
    argbuff[21] := TagItem(ss,tt);
    argbuff[22] := TagItem(uu,vv);
    argbuff[23].ti_Tag := ww;
    TAGS := @argbuff;
end;


end.
