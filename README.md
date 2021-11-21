# Triangle clipper

C++ simple application.

Given two coplanar triangles in overlap on 3D space, calculates the area of the intersection.

1. Tranform the triangles vertices to the xy plane, keeping areas
2. Calculate the overlap area by using the polygon clipper algorithm
  > Ivan Sutherland, Gary W. Hodgman: Reentrant Polygon Clipping. Communications of the ACM, vol. 17, pp. 32–42, 1974
3. Using the resulting polygon, calculate the area

Adapted from https://github.com/mdabdk/sutherland-hodgman