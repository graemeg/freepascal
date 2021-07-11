{ this program just links all externals, declared in the shape unit }
program shape_linktest;
uses
  shape;
begin
  halt(0);
  XShapeQueryExtension(nil,nil,nil);
  XShapeQueryVersion(nil,nil,nil);
  XShapeCombineRegion(nil,0,0,0,0,nil,0);
  XShapeCombineRectangles(nil,0,0,0,0,nil,0,0,0);
  XShapeCombineMask(nil,0,0,0,0,0,0);
  XShapeCombineShape(nil,0,0,0,0,0,0,0);
  XShapeOffsetShape(nil,0,0,0,0);
  XShapeQueryExtents(nil,0,nil,nil,nil,nil,nil,nil,nil,nil,nil,nil);
  XShapeSelectInput(nil,0,0);
  XShapeInputSelected(nil,0);
  XShapeGetRectangles(nil,0,0,nil,nil);
end.
