

{$IFDEF DetectConstFlagCorruption}
TYPE
   EConstCorruption=Class(Exception);
{$ENDIF}

PROCEDURE TExpression.UpdateConstants;

function InternalUpdateConstants(expr:pnode):boolean;
{Shouldn't be necessary. Detects both corruption of flags if
DetectConstFlagCorruption is defined
and rebuilds them.}

begin
 if Expr<>NIL THEN
  case Expr^.NodeType of
   VarNode  :  begin  {A symbol is not a constant}
                {$IFDEF DetectConstFlagCorruption}
                 if (ExprIsConstant IN Expr^.Flags) then
                  Raise EConstCorruption.Create('Corrupt Varnode');
                {$ENDIF}
                Exclude(Expr^.flags,ExprIsConstant);
                Result:=false;
               end;
   IConstNode,
   ConstNode:  begin
                {$IFDEF DetectConstFlagCorruption}
                 if NOT (ExprIsConstant IN Expr^.Flags) then
                  Raise EConstCorruption.Create('Corrupt (I)constnode');
                {$ENDIF}
                Include(Expr^.flags,ExprIsConstant);
                Result:=TRUE;
               end;
   calcnode:  begin
               Result:=InternalUpdateConstants(Expr^.Left) AND InternalUpdateConstants(Expr^.Right);
               {$IFDEF DetectConstFlagCorruption}
                 if (ExprIsConstant IN Expr^.Flags)<>Result then
                  Raise EConstCorruption.Create('Corrupt calcnode');
                {$ENDIF}
               IF Result THEN
                Include(Expr^.flags,ExprIsConstant)
               else
                Exclude(Expr^.flags,ExprIsConstant)
              end;
   funcnode:  begin
               Result:=InternalUpdateConstants(Expr^.Son);
               {$IFDEF DetectConstFlagCorruption}
                 if (ExprIsConstant IN Expr^.Flags)<>Result then
                  Raise EConstCorruption.Create('Corrupt funcnode');
                {$ENDIF}
               IF Result THEN
                Include(Expr^.flags,ExprIsConstant)
               else
                Exclude(Expr^.flags,ExprIsConstant)
              end;
   func2node: begin
               Result:=InternalUpdateConstants(Expr^.Son2Left) and InternalUpdateConstants(Expr^.Son2Right);
               {$IFDEF DetectConstFlagCorruption}
                 if (ExprIsConstant IN Expr^.Flags)<>Result then
                  Raise EConstCorruption.Create('Corrupt func2node');
                {$ENDIF}
               IF Result THEN
                Include(Expr^.flags,ExprIsConstant)
               else
                Exclude(Expr^.flags,ExprIsConstant)
              end;
            end;
end;

begin
 InternalUpdateConstants(ExprTree);
end;

{














}