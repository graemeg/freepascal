{
    $ id:                                                       $
    Copyright (c) 2000 by Marco van de Voort(marco@freepascal.org)
     member of the Free Pascal development team

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright. (LGPL)

    Some resourcestrings.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

resourcestring
   SEvalUnknownParameterType='QuickEvaluate: unknown variable type for parameter "%s".';
   SEvalUndefinedVar	    ='QuickEvaluate: No value for parameter "%s".';
   SEvalIE                  ='TEvaluator: Internal Error: ';
   SEvalStackDepthExceeded  ='TEvaluator: Stack depth Evaluate exceeded';
   SEvalBadNumberOfVars     ='TEvaluator: Invalid number of variables';
   SParsIE                  ='TBaseExprParser: Internal Error:';
   SParseRPNOverflow        ='TBaseExprParser: RPN stack overflow';
   SParseRPNUnderflow       ='TBaseExprParser: RPN stack underflow';
   SParsDiv0                ='TBaseExprParser: RPN Division by zero in parser';
   SParsOpOverflow          ='TBaseExprParser: Operator stack Overflow';
   SParsOpUnderflow         ='TBaseExprParser: Operator stack Underflow';
   SNILDeref                ='NIL dereference';
   SExprIE                  ='TExpression: Internal error: ';
   SExprNotInt              ='TExpression: This is not an integer';
   SExprNotFloat            ='TExpression: This is not a float';
   SExprInvmsg              ='TExpression: Inv(x) evaluates to 1/0';
   SExprInvSimp             ='TExpression: Division by 0 encountered while simplifying';

 {
  $Log$
  Revision 1.1  2002/12/15 21:01:22  marco
  Initial revision

}
