function [x0, y0, r] = Circle_center(A,B,C)% this function use three points on the arc to calculate the circle center
% just use three points on the arc
% A and B are start and end points
% C is the middle point of the arc
syms x y r % three unknown parameters: the coordinate of the circle center and the radius
eqn1 = r==(x-A(1))^2+(y-A(2))^2; %according to the stantard equation of the circle :(x-a)^2+(y-b)^2 = r^2
eqn2 = r==(x-B(1))^2+(y-B(2))^2;
eqn3 = r==(x-C(1))^2+(y-C(2))^2;

[x,y,r]=solve(eqn1, eqn2, eqn3, x, y, r);% solve three equation to get three unknown parameters 
x = double(x);
y = double(y);
r = double(r);
r = sqrt(r);

x0 = roundn(x, -2);% rounded to 2 digits
y0 = roundn(y, -2);
r  = roundn(r,-2); 
end