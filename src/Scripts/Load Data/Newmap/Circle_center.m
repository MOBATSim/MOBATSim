function [x0,y0] = Circle_center(A,B,C)
% just use three points on the arc to calculate circle center
% A and B are start and end points
% C is the middle point of the arc
syms x y r
eqn1 = r==(x-A(1))^2+(y-A(2))^2;
eqn2 = r==(x-B(1))^2+(y-B(2))^2;
eqn3 = r==(x-C(1))^2+(y-C(2))^2;

[x,y,r]=solve(eqn1, eqn2, eqn3, x, y, r);
x = double(x);
y = double(y);
r = double(r);

x0 = roundn(x, -2);
y0 = roundn(y, -2);
end