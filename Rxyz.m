function R_xyz = Rxyz(eular)
%RXYZ
%    R_XYZ = RXYZ(A,B,R)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    28-May-2021 11:55:57
a = eular(1);
b = eular(2);
r = eular(3);

t2 = cos(a);
t3 = cos(b);
t4 = sin(a);
t5 = sin(b);
t6 = cos(r);
t7 = sin(r);
R_xyz = reshape([t3.*t6,t2.*t7+t4.*t5.*t6,t4.*t7-t2.*t5.*t6,-t3.*t7,t2.*t6-t4.*t5.*t7,t4.*t6+t2.*t5.*t7,t5,-t3.*t4,t2.*t3],[3,3]);