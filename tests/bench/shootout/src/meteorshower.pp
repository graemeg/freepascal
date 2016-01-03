program meteorshower;

{Shootout Meteor puzzle implementation

 by Daniel Mantione

 mostly based on Ben St. John's implementation.}

<<<<<<< HEAD
<<<<<<< HEAD
=======
{$Q-}

>>>>>>> graemeg/fixes_2_2
=======
{$Q-}

>>>>>>> origin/fixes_2_2
uses dos;

const N_COL = 5;
      N_ROW = 10;
      N_CELL = N_COL * N_ROW;
      N_PIECE_TYPE = 10;

      N_ELEM=5;
      N_ORIENT=12;
      ALL_PIECE_MASK=[0..N_PIECE_TYPE-1];
      SKIP_PIECE=5;

      no_piece=high(byte);

      L_EDGE_MASK=[0,5,10,15,20,25,30];
      R_EDGE_MASK=[4,9,14,19,24,29];
      TOP_ROW    = [0*N_COL..1*N_COL-1];
      SECOND_ROW = [1*N_COL..2*N_COL-1];
      THIRD_ROW  = [2*N_COL..3*N_COL-1];
      FOURTH_ROW = [3*N_COL..4*N_COL-1];
      FIFTH_ROW  = [4*N_COL..5*N_COL-1];
      SIXTH_ROW  = [5*N_COL..6*N_COL-1];
      LAST_ROW   = SIXTH_ROW;
      ROW_0_MASK=[0..N_COL-1,10..N_COL+10-1,20..N_COL+20-1,30,31];
      ROW_1_MASK=[5..N_COL+5-1,15..N_COL+15-1,25..N_COL+25-1];
      BOARD_MASK=[0..29];


type  bitvec=set of 0..31;
      dimensions=(dimx,dimy);
      parity=(even,odd);
      goodbad=(good,bad,always_bad);
      piecenr=type 0..N_PIECE_TYPE-1;
      orientation=type 0..N_ORIENT-1;

      piece_placement=record
        vec:bitvec;
        ipiece:piecenr;
        row:byte;
      end;

type Soln=object
       m_pieces:array[piecenr] of piece_placement;
       m_npiece:byte;
       m_cells:array[0..N_ROW-1,0..N_COL-1] of piecenr;
       m_synched:boolean;
       constructor init(fillval:byte);
       procedure setCells;
       function lessThan(var r:Soln):boolean;
       procedure write(var f:text);
       procedure fill(value:byte);
       procedure spin(var spun:Soln);

       function isEmpty:boolean;
       procedure popPiece;inline;
       procedure pushPiece(Avec:bitvec;AiPiece:piecenr;Arow:byte);
     end;

     instance=record
       m_allowed:set of byte;
       m_vec:bitvec;
       m_offset:longint;
     end;

     TPts=array[0..N_ELEM-1,dimensions] of shortint;
     piece=object
       m_instance:array[parity] of instance;
       procedure set_ok_positions(isOdd:parity;w,h:longint);
     end;

     OkPieces=record
       nPieces:array[piecenr] of byte;
       pieceVec:array[piecenr,orientation] of bitvec;
     end;


type  fixed=(OPEN, CLOSED);
      islandinfo=record
        has_bad:array[fixed,parity] of bitvec;
        is_known:array[fixed,parity] of bitvec;
        alwaysBad:array[parity] of bitvec;
      end;

const MAX_ISLAND_OFFSET=1024;

type cacherec=record
       krow,kpiecevec:word;
       kboardvec:bitvec;
     end;

var s_basePiece:array[piecenr,orientation] of piece;
    g_okPieces:array[0..N_ROW-1,0..N_COL-1] of OkPieces;
    g_islandInfo:array[0..MAX_ISLAND_OFFSET-1] of islandinfo;
    g_nIslandInfo:cardinal=0;
    cache:array[0..1024*128-1] of cacherec;

    m_curSoln,m_minSoln,m_maxSoln:Soln;
    m_nSoln:cardinal;

const basevecs:array [0..9] of bitvec= (
        [0,1,2,3,8],
        [0,1,3,6,7],
        [0,1,2,7,12],
        [0,1,2,5,10],
        [0,2,5,6,10],
        [0,1,2,6,7],
        [0,1,5,10,15],
        [0,1,2,5,7],
        [0,1,2,7,8],
        [0,1,2,3,7]
      );


constructor soln.init(fillval:byte);

begin
  fill(fillval);
end;

procedure Soln.fill(value:byte);

begin
   m_synched:=false;
   fillchar(m_cells,N_CELL,value);
end;

function soln.isEmpty:boolean;

begin
  isempty:=m_nPiece=0;
end;

procedure soln.pushPiece(Avec:bitvec;AiPiece:piecenr;Arow:byte);

begin
  with m_pieces[m_npiece] do
    begin
      vec:=Avec;
      iPiece:=AiPiece;
      row:=Arow;
    end;
  inc(m_npiece);
end;

procedure soln.popPiece;inline;

begin
  dec(m_nPiece);
  m_synched := false;
end;

procedure soln.write(var f:text);

var x,y:byte;

begin
  for y:=0 to N_ROW-1 do
    begin
      {indent every second line}
      if y mod 2=1 then
        system.write(f,' ');
      for x:=0 to N_COL-1 do
        if m_cells[y,x]=no_piece then
          system.write(f,'. ')
        else
          system.write(f,char(byte('0')+m_cells[y,x]),' ');
      writeln(f);
    end;
end;


procedure Soln.setCells;

var c,i,x,y,newcells:byte;

begin
   if m_synched then
     exit;
   for i:=1 to m_nPiece do
     with m_pieces[i-1] do
       begin
         newcells:=0;
         c:=0;
         for y:=row to N_ROW do
           begin
             for x:=0 to N_COL-1 do
               begin
                 if c in vec then
                   begin
                     m_cells[y,x]:=ipiece;
                     inc(NewCells);
                   end;
                 inc(c);
               end;
             if NewCells=N_ELEM then
               break;
           end;
       end;
   m_synched:=true;
end;

function Soln.lessThan(var r:Soln):boolean;

var x,y,lval,rval:byte;

begin
   if m_pieces[0].iPiece<>r.m_pieces[0].iPiece then
     begin
       lessthan:=m_pieces[0].iPiece < r.m_pieces[0].iPiece;
       exit;
     end;

   setCells();
   r.setCells();

   for y:=0 to N_ROW-1 do
      for x:=0 to N_COL-1 do
        begin
         lval:=m_cells[y,x];
         rval:=r.m_cells[y,x];

         if lval <> rval then
           begin
             lessthan:=lval<rval;
             exit;
           end;
        end;

   lessthan:=false; {solutions are equal}
end;

procedure Soln.spin(var spun:Soln);

var x,y:byte;

begin
   setCells;
   {swap cells}
   for y:=0 to N_ROW-1 do
      for x:=0 to N_COL-1 do
        spun.m_cells[y,x]:=m_cells[N_ROW-y-1,N_COL-x-1];

   {swap first and last pieces (the rest aren't used)}
   spun.m_pieces[0].iPiece:=m_pieces[N_PIECE_TYPE-1].iPiece;
   spun.m_synched:=true;
end;

function floor(top,bot:longint):longint;

begin
   floor:=top div bot;
   {negative numbers should be rounded down, not towards zero}
   if (floor*bot<>top) and ((top<0) xor (bot<=0)) then
      dec(floor);
end;

const s_firstOne:array[0..31] of byte=(
   0, 0, 1, 0,   2, 0, 1, 0,
   3, 0, 1, 0,   2, 0, 1, 0,

   4, 0, 1, 0,   2, 0, 1, 0,
   3, 0, 1, 0,   2, 0, 1, 0
);

function first_set_bit(v:bitvec):cardinal;inline;

{$ifdef endian_little}
const l=0;
      h=1;
{$else}
const l=1;
      h=0;
{$endif}

var d:double;
    u:array[0..1] of bitvec absolute d;

begin
  first_set_bit:=0;
  if v<>[] then
    begin
      u[l]:=v;
      u[h]:=[30,25,24,21,20];
      d:=d-4503599627370496;
      first_set_bit:=cardinal(u[h]) shr 20-$3ff;
    end;
end;

function count_ones(v:bitvec):cardinal;inline;

begin
   count_ones:=0;
   while v<>[] do
     begin
       inc(count_ones);
       cardinal(v):=cardinal(v) and (cardinal(v)-1);
     end;
end;

procedure setCoordList(vec:bitvec;var pts:Tpts);

var iPt,n:longint;
    x,y:byte;

begin
   iPt:=0;
   n:=0;
   for y:=0 to N_ROW-1 do
     for x:=0 to N_COL-1 do
       begin
         if n in vec then
           begin
             pts[iPt,dimx]:=x;
             pts[iPt,dimy]:=y;
             inc(iPt);
           end;
         inc(n);
         if n=32 then
           exit;
       end;
end;

function toBitVector(const pts:Tpts):bitvec;

var x,y,iPt:byte;

begin
   tobitvector:=[];
   for iPt:=low(pts) to high(pts) do
     begin
       x:=pts[iPt,dimx];
       y:=pts[iPt,dimy];
       include(tobitvector,y*N_COL+x);
     end;
end;

procedure shiftUpLines(var pts:Tpts;shift:longint);

var iPt:byte;

begin
   {vertical shifts have a twist}
   for iPt:=low(pts) to high(pts) do
     begin
       if pts[iPt,dimy] and shift and 1<>0 then
         inc(pts[iPt,dimx]);
       dec(pts[iPt,dimy],shift);
     end;
end;

function shiftToX0(var pts:Tpts;var Ainstance:instance;offsetRow:longint):shortint;

var x,y,xmin,xmax,iPt,offset:shortint;

begin
   { .. determine shift}
   xMin:=pts[0,dimx];
   xMax:=xMin;
   for iPt:=low(pts)+1 to high(pts) do
     begin
       x:=pts[iPt,dimx];
       y:=pts[iPt,dimy];
       if x<xMin then
         xMin:=x
       else if x > xMax then
         xMax:=x;
     end;

   offset:=N_ELEM;
   for iPt:=low(pts) to high(pts) do
     begin
      dec(pts[iPt,dimx],xMin);
      {check offset -- leftmost cell on top line}
      if (pts[iPt,dimy]=offsetRow) and (pts[iPt,dimx]<offset) then
         offset:=pts[iPt,dimx];
   end;

   Ainstance.m_offset := offset;
   Ainstance.m_vec := toBitVector(pts);
   shifttox0:=xMax - xMin;
end;

function badregion(var to_fill:bitvec;rnew:bitvec):boolean;

var region,even_region,odd_region:bitvec;
    cell_count:cardinal;

begin
   {Grow empty region, until it doesn't change any more.}
   repeat
      region:=rnew;
      even_region:=region*(ROW_0_MASK*([0..31]-L_EDGE_MASK));
      odd_region:=region*(ROW_1_MASK*([0..31]-R_EDGE_MASK));

      rnew:=to_fill*(rnew
                    {simple grow up/down}
                    +bitvec(cardinal(region) shr N_COL)
                    +bitvec(cardinal(region) shl N_COL)
                    {grow right/left}
                    +bitvec(cardinal(region) and not cardinal(L_EDGE_MASK) shr 1)
                    +bitvec(cardinal(region) and not cardinal(R_EDGE_MASK) shl 1)
                    {tricky growth}
                    +bitvec(cardinal(even_Region) shr (N_COL+1))
                    +bitvec(cardinal(even_Region) shl (N_COL-1))
                    +bitvec(cardinal(odd_Region) shr (N_COL-1))
                    +bitvec(cardinal(odd_Region) shl (N_COL+1))
                    );
   until (rnew=to_fill) or (rnew=region);

   {Subtract empty region from board.}
   to_fill:=to_fill-rnew;

   cell_count:=count_ones(to_fill);
   {Optimize 'cell_count mod 5<>0' by hand...}
   badregion:=cell_count<>((cell_count*$cccd) shr 18)*5;
end;

function has_bad_islands_single(boardVec:bitvec;row:longint):boolean;

var tofill,startregion,bmask:bitvec;
    isodd:boolean;

begin
   tofill:=[0..31]-boardvec;
   isOdd:=row and 1<>0;
   if isOdd then
     begin
       dec(row);
       toFill:=bitvec(cardinal(tofill) shl N_COL); {shift to even aligned}
       toFill:= tofill + TOP_ROW;
     end;

   startRegion := TOP_ROW;
   bMask := BOARD_MASK; {all but the first two bits}
   if row>=4 then
      cardinal(bMask):=cardinal(bmask) shr ((row-4)*N_COL)
   else if isOdd or (row = 0) then
      startRegion := LAST_ROW;

   toFill:=tofill*bMask;
   startRegion:=startregion*toFill;

   has_bad_islands_single:=true;
   while toFill<>[] do
     begin
       if badRegion(toFill, startRegion) then
         exit;
       startRegion:=[first_set_bit(toFill)];
     end;
   has_bad_islands_single:=false;
end;


procedure piece.set_ok_positions(isOdd:parity;w,h:longint);

var x,y,xpos,pos:byte;

begin
   pos:=byte(isodd)*N_COL;
   with m_instance[isOdd] do
     begin
       m_allowed:=[];
       y:=byte(isOdd);
       while y<N_ROW-h do
         begin
           if m_offset<>0 then
             inc(pos,m_offset);
           for xPos:=0 to N_COL-1-m_offset do
             begin
               {check if the new position is on the board}
               if (xPos<N_COL-w) and not has_bad_islands_single(bitvec(cardinal(m_vec) shl xPos),y) then
                 begin
                   {position is allowed}
                   include(m_allowed,pos);
                 end;
               inc(pos);
             end;
           y:=y+2;
           {Skip row with wrong parity:}
           inc(pos,N_COL);
         end;
   end;
end;

procedure gen_orientation(vec:bitvec;iOrient:cardinal;var target:Piece);

var pts:Tpts;
    x,y,ymin,ymax,h,w:shortint;
    rot,iPt:byte;
    flip:boolean;

begin
   {get (x,y) coordinates}
   setCoordList(vec, pts);

   rot := iOrient mod 6;
   flip := iOrient >= 6;
   if flip then
     for iPt:=0 to N_ELEM-1 do
       pts[iPt,dimy]:=-pts[iPt,dimy];

   {rotate as necessary}
   while rot>0 do
     begin
       for iPt:=0 to N_ELEM-1 do
         begin
           x:=pts[iPt,dimx];
           y:=pts[iPt,dimy];
           pts[iPt,dimx]:=floor(2*x-3*y+1,4);
           pts[iPt,dimy]:=floor(2*x+y+1,2);
         end;
      dec(rot);
   end;

   {determine vertical shift}
   yMin := pts[0,dimy];
   yMax := yMin;
   for iPt:= 1 to N_ELEM-1 do
     begin
       y := pts[iPt,dimy];

       if y < yMin then
         yMin := y
       else if y > yMax then
         yMax := y;
     end;
   h:=yMax-yMin;

   shiftUpLines(pts, yMin);
   w := shiftToX0(pts, target.m_instance[EVEN], 0);
   target.set_ok_positions(EVEN, w, h);
   cardinal(target.m_instance[EVEN].m_vec) := cardinal(target.m_instance[EVEN].m_vec) shr target.m_instance[EVEN].m_offset;

   {shift down one line}
   shiftUpLines(pts, -1);
   w := shiftToX0(pts, target.m_instance[ODD], 1);
   {shift the bitmask back one line}
   cardinal(target.m_instance[ODD].m_vec) :=cardinal(target.m_instance[ODD].m_vec) shr N_COL;
   target.set_ok_positions(ODD, w, h);
   cardinal(target.m_instance[ODD].m_vec):= cardinal(target.m_instance[ODD].m_vec) shr target.m_instance[ODD].m_offset;
end;

function getPiece(iPiece,iOrient:cardinal;iParity:parity):instance;inline;

begin
  getpiece:=s_basePiece[iPiece][iOrient].m_instance[iParity];
end;

procedure gen_all_orientations;

var ipiece:piecenr;
    iorient:orientation;
    irow,icol:byte;
    refpiece:bitvec;
    n,npiece:byte;

begin
   for iPiece:=low(ipiece) to high(ipiece) do
     begin
       refPiece:=BaseVecs[iPiece];
       for iOrient:=low(iorient) to high(iorient) do
         begin
           gen_orientation(refPiece, iOrient, s_basePiece[iPiece,iOrient]);
           with s_basePiece[iPiece,iOrient] do
             begin
               if (iPiece=SKIP_PIECE) and (iOrient in [3..5,9..11]) then
                 begin
                   m_instance[odd].m_allowed := [];
                   m_instance[even].m_allowed := [];
                 end;
             end;
         end;
     end;

   for iPiece:=low(ipiece) to high(ipiece) do
     begin
      for iOrient:=low(iorient) to high(iorient) do
        begin
         n:=0;
         for iRow:=0 to N_ROW-1 do
           begin
            with getPiece(iPiece, iOrient, parity(iRow and 1)) do
              for iCol:=0 to N_COL-1 do
                begin
                  if n in m_allowed then
                    begin
                      nPiece:=g_okPieces[iRow,iCol].nPieces[iPiece];
                      g_okPieces[iRow,iCol].pieceVec[iPiece,nPiece]:=bitvec(cardinal(m_vec) shl iCol);
                      inc(g_okPieces[iRow,iCol].nPieces[iPiece]);
                    end;
                  inc(n);
                end;
          end
      end
   end
end;

procedure init_board;

begin
  m_cursoln.init(NO_PIECE);
  m_minsoln.init(NO_PIECE);
  m_maxsoln.init(NO_PIECE);
  m_nsoln:=0;
end;

const g_firstRegion:array[0..31] of bitvec=(
        [],      [0],      [1],       [0,1],
        [2],     [0],      [1,2],     [0,1,2],
        [3],     [0],      [1],       [0,1],
        [2,3],   [0],      [1,2,3],   [0,1,2,3],
        [4],     [0],      [1],       [0,1],
        [2],     [0],      [1,2],     [0,1,2],
        [3,4],   [0],      [1],       [1,2],
        [2,3,4], [0],      [1,2,3,4], [0,1,2,3,4]
);

function calc_bad_islands(boardVec:bitvec;row:longint):goodbad;

var tofill,boardmask,bottom,startregion:bitvec;
    filled:boolean;

begin
   toFill:=[0..31]-boardVec;
   {Compensate for odd rows.}
   if row and 1<>0 then
     begin
       dec(row);
       cardinal(toFill):=cardinal(tofill) shl N_COL;
     end;

   boardMask := BOARD_MASK; {all but the first two bits}
   if row>4 then
      cardinal(boardMask):=cardinal(boardmask) shr ((row-4)*N_COL);
   toFill:=tofill*boardMask;

   {a little pre-work to speed things up}
   filled:=toFill*LAST_ROW=LAST_ROW;
   bottom:=LAST_ROW;
   while bottom*toFill=bottom do
     begin
       toFill:=tofill-bottom;
       cardinal(bottom):=cardinal(bottom) shr N_COL;
     end;

   if filled or (row<4) then
      startRegion := bottom * toFill
   else
     begin
       startRegion := g_firstRegion[cardinal(toFill*TOP_ROW)];
       if startRegion=[] then
          begin
            startRegion := bitvec(cardinal(toFill) shr N_COL)*TOP_ROW;
            startRegion := g_firstRegion[cardinal(startRegion)];
            cardinal(startRegion) := cardinal(startregion) shl N_COL;
          end;
        startRegion:=startregion+bitvec(cardinal(startRegion) shl N_COL)*toFill;
     end;

   while toFill<>[] do
     begin
       if badRegion(toFill, startRegion) then
          begin
            if toFill<>[] then
              calc_bad_islands:=ALWAYS_BAD
            else
              calc_bad_islands:=BAD;
            exit;
          end;
       startRegion := [first_set_bit(toFill)];
     end;

   calc_bad_islands:=GOOD;
end;


function has_bad_islands(boardvec:bitvec;row:longint):goodbad;

var last_row:bitvec;
    isodd:parity;
    isclosed:fixed;

begin
   {skip over any filled rows}
   while boardVec*TOP_ROW=TOP_ROW do
     begin
       cardinal(boardVec):=cardinal(boardvec) shr N_COL;
       inc(row);
     end;

   has_bad_islands:=bad;
   with g_islandInfo[cardinal(boardvec*(TOP_ROW+SECOND_ROW))] do
     begin
       last_row:=bitvec(cardinal(boardvec) shr (2*N_COL))*TOP_ROW;
       isOdd:=parity(row and 1);

       if not(cardinal(last_row) in alwaysBad[parity(row and 1)]) then
<<<<<<< HEAD
<<<<<<< HEAD
         if boardVec*bitvec(cardinal(cardinal(TOP_ROW) shl N_COL*3))=[] then
=======
         if boardVec*bitvec(cardinal(TOP_ROW) shl N_COL*3)=[] then
>>>>>>> graemeg/fixes_2_2
=======
         if boardVec*bitvec(cardinal(TOP_ROW) shl N_COL*3)=[] then
>>>>>>> origin/fixes_2_2
           begin
             isClosed:=fixed(row>6); {because we track 3 rows}
               if not(cardinal(last_row) in is_known[isClosed,isOdd]) then
                 if boardVec<>[] then
                   begin
                     has_bad_islands:=calc_bad_islands(boardvec,row);
                     include(is_known[isClosed,isOdd],cardinal(last_row));
                     if has_bad_islands<>good then
                       include(is_known[isClosed,isOdd],cardinal(last_row));
                   end
                 else
                   has_bad_islands:=good
               else
                 if not(cardinal(last_row) in has_bad[isClosed,isOdd]) then
                   has_bad_islands:=good;
           end
         else
           has_bad_islands:=calc_bad_islands(boardvec,row);
     end;
end;

const g_flip:array[0..31] of bitvec=(
        [],        [4],        [3],        [3,4],
        [2],       [2,4],      [2,3],      [2,3,4],
        [1],       [1,4],      [1,3],      [1,3,4],
        [1,2],     [1,2,4],    [1,2,3],    [1,2,3,4],
        [0],       [0,4],      [0,3],      [0,3,4],
        [0,2],     [0,2,4],    [0,2,3],    [0,2,3,4],
        [0,1],     [0,1,4],    [0,1,3],    [0,1,3,4],
        [0,1,2],   [0,1,2,4],  [0,1,2,3],  [0,1,2,3,4]
);

function flipTwoRows(bits:bitvec):bitvec;inline;

var flipped:cardinal;

begin
   flipped:=cardinal(g_flip[cardinal(bits) shr N_COL]) shl N_COL;
   fliptworows:=bitvec(flipped or cardinal(g_flip[cardinal(bits*TOP_ROW)]));
end;

procedure mark_bad(var info:IslandInfo;n:byte;eo:parity;always:boolean);inline;

begin
  with info do
   begin
     include(has_bad[OPEN,eo],n);
     include(has_bad[CLOSED,eo],n);

     if always then
       include(alwaysBad[eo],n);
   end;
end;

procedure calc_always_bad;

var i,iWord:cardinal;
    boardvec:bitvec;
    hasbad:goodbad;
    always:boolean;
    flipped:^islandinfo;

begin
   for iWord:=1 to MAX_ISLAND_OFFSET-1 do
     begin
      flipped := @g_islandInfo[cardinal(flipTwoRows(bitvec(iWord)))];
      for i:=0 to 31 do
        begin
          boardvec:=bitvec((i shl (2*N_COL)) or iWord);
          if not(i in g_islandInfo[iWord].is_known[OPEN,EVEN]) then
            begin
              hasBad:=calc_bad_islands(boardvec,0);
              if hasBad<>good then
                begin
                 always:=hasBad=ALWAYS_BAD;
                 mark_bad(g_islandInfo[iWord], i, EVEN, always);
                 mark_bad(flipped^,cardinal(g_flip[i]), ODD, always);
              end;
            end;
      end;
      flipped^.is_known[OPEN,odd]:=[0..31];
      g_islandInfo[iWord].is_known[OPEN,even]:=[0..31];
   end
end;

procedure record_solution(var s:Soln);

var spun:soln;

begin
   s.setcells;
   inc(m_nSoln,2); {add solution and its rotation}

   if m_minSoln.isEmpty then
     begin
       m_minSoln := s;
       m_maxSoln := s;
       exit;
     end;

   if s.lessThan(m_minSoln) then
      m_minSoln := s
   else if m_maxSoln.lessThan(s) then
      m_maxSoln := s;

   s.spin(spun);
   if spun.lessThan(m_minSoln) then
      m_minSoln := spun
   else if m_maxSoln.lessThan(spun) then
      m_maxSoln := spun;
end;

function gen_all_solutions(boardVec,placedPieces:bitvec;row:byte):cardinal;

var ipiece:piecenr;
    iorient:byte;
    piece:bitvec;

begin
   while boardVec*TOP_ROW=TOP_ROW do
     begin
       cardinal(boardVec):=cardinal(boardvec) shr N_COL;
       inc(row);
     end;
   gen_all_solutions:=0;
   with cache[((cardinal(boardvec)*
              (cardinal(placedpieces) {shl 3} + 1)
              xor row shl 5)) mod 131071] do
     if (krow<>row) or (bitvec(cardinal(kpiecevec))<>placedpieces) or (kboardvec<>boardvec) then
       begin
         with g_okpieces[row,s_firstOne[cardinal([0..N_COL-1]-boardVec)]] do
           for ipiece:=0 to N_PIECE_TYPE-1 do
             if not(ipiece in placedpieces) then
               for iorient:=1 to npieces[ipiece] do {start with 1, npieces[x] can be zero}
                 begin
                   piece:=pieceVec[iPiece,iOrient-1];
                   {check if piece conflicts with other pieces or if we get a bad island.}
                   if (piece*boardVec=[]) and (has_bad_islands(boardVec+piece,row)=good) then
                     begin
                       m_curSoln.pushPiece(piece,iPiece,row);
                       {recurse or record solution}
                       if placedPieces+[ipiece]<>ALL_PIECE_MASK then
                         inc(gen_all_solutions,gen_all_solutions(boardVec+piece,placedPieces+[ipiece],row))
                       else
                         begin
                           record_solution(m_curSoln);
                           inc(gen_all_solutions);
                         end;
                       m_curSoln.popPiece();
                     end;
                end;
         if gen_all_solutions=0 then
           begin
             krow:=row;
             kpiecevec:=word(cardinal(placedpieces));
             kboardvec:=boardvec;
           end;
      end;
end;

begin
   if paramcount > 2 then
     halt(1); {spec says this is an error}

   textrec(output).flushfunc:=nil;

   gen_all_orientations;
   calc_always_bad;
   init_board;
   filldword(cache,sizeof(cache) shr 2,$ffffffff);
   gen_all_solutions([], [], 0);

   writeln(m_nSoln,' solutions found');
   writeln;
   m_minSoln.write(output);
   writeln;
   m_maxSoln.write(output);
   writeln;
end.
