%% Build Cpp MEX File with CGAL library
% Build a single Cpp program into a MEX file. 
clc; clear;

%% specify path of headers, libs
cgalLibLoc = 'D:\dev\CGAL-5.1.2'; %CGAL库根目录
boostLoc = 'D:\dev\boost_1_71_0'; %boost根目录
headerFiles = 'CDPR_CollisionDetection.hpp'; %头文件
% lib文件的目录
libFolder1 = ['-L' fullfile(cgalLibLoc, 'build\lib')];
libFolder2 = ['-L' fullfile(cgalLibLoc, 'auxiliary\gmp\lib')];
libFolder3 = ['-L' fullfile(boostLoc, 'libs')];
%libFolder4 = fullfile(boostLoc, "lib64-msvc-14.1");
%libraries = [lib1, lib2, lib3];

% libname
lib1 = ['-l' 'CGAL_Core-vc141-mt-gd-5.1.2']; %libFolder1
lib2 = ['-l' 'CGAL_ImageIO-vc141-mt-gd-5.1.2']; %libFolder1
lib3 = ['-l' 'CGAL-vc141-mt-gd-5.1.2']; %libFolder1
lib4 = ['-l' 'libgmp-10']; %libFolder2
lib5 = ['-l' 'libmpfr-4']; %libFolder2

% 包含目录
include1 = ['-I' fullfile(cgalLibLoc, 'include')];
include2 = ['-I' fullfile(cgalLibLoc, 'build\include')];
include3 = ['-I' fullfile(cgalLibLoc, 'auxiliary\gmp\include')];
include4 = ['-I' boostLoc];
include5 = ['-I' pwd]; %current folder
%includePath = [include1, include2, include3, include4];

%% build calCableSqrDis_CGAL.mexw64
% Build the MEX file. The output displays information specific to your compiler.
mex('-v', include1, include2, include3, include4, include5, ...
    libFolder1, lib1, libFolder1, lib2, libFolder1, lib3, libFolder2, lib4, libFolder2, lib5, ...
    'calCableSqrDis_CGAL.cpp');
% Test1.
seg1 = [1 0 0; 1 -5 5]'; %3X2 mat
seg2 = [0 0 0; -5 5 5]'; %3X2 mat
test1 = calCableSqrDis_CGAL(seg1, seg2);

%% build doCableIntersect_CGAL.mexw64
% Build the MEX file. The output displays information specific to your compiler.
mex('-v', include1, include2, include3, include4, include5, ...
    libFolder1, lib1, libFolder1, lib2, libFolder1, lib3, libFolder2, lib4, libFolder2, lib5, ...
    'doIntersect_CGAL.cpp');
% Test1.
tetra1=[1 2 3; 4 5 6 ; -1 -4 -6 ; -7 -8 -9]'; %3X4 mat
tetra2=[1 5 3; 4 9 10 ; -5 -7 -8 ; -10 -12 -9]'; %3X4 mat
test2 = doIntersect_CGAL(tetra1, tetra2);
if test2
    disp("intersection detected!");
else
    disp("no intersection detected.");
end
s1=alphaShape(tetra1(1,:)', tetra1(2,:)', tetra1(3,:)');
s2=alphaShape(tetra2(1,:)', tetra2(2,:)', tetra2(3,:)');
plot(s1);
hold on;
plot(s2);

%% build boxIntersect2D_CGAL.mexw64
% Build the MEX file. The output displays information specific to your compiler.
mex('-v', include1, include2, include3, include4, include5, ...
    libFolder1, lib1, libFolder1, lib2, libFolder1, lib3, libFolder2, lib4, libFolder2, lib5, ...
    'boxIntersect2D_CGAL.cpp');
%% 
% Copyright 2021 汤凯 HITSZ