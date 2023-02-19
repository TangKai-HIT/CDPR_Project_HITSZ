function R_zyx = Rzyx(eular)
%RZYX_MF
%    R_ZYX = RZYX_MF(A,B,R)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    23-Mar-2021 10:08:03
a = eular(1);
b = eular(2);
r = eular(3);

t2 = cos(a);
t3 = cos(b);
t4 = sin(a);
t5 = sin(b);
t6 = cos(r);
t7 = sin(r);
R_zyx = reshape([t2.*t3,t3.*t4,-t5,-t4.*t6+t2.*t5.*t7,t2.*t6+t4.*t5.*t7,t3.*t7,t4.*t7+t2.*t5.*t6,-t2.*t7+t4.*t5.*t6,t3.*t6],[3,3]);
