%% buildInterfaceToCGAL.m
% 为所定义的与CGAL库相关的函数创建MATLAB接口
% 注：本文件运行时，与boost相关头文件解析失败！改用MEX做接口。2021.2.18
% 已改用MexBuildCppWithCGAL.m来创建接口
clc; clear;
cgalLibLoc = "D:\dev\CGAL-5.1.2"; %CGAL库根目录
boostLoc = "D:\dev\boost_1_71_0"; %boost根目录
headerFiles = "CDPR_CollisionDetection.hpp"; %头文件
% lib文件的目录
lib1 = fullfile(cgalLibLoc, "build\lib");
lib2 = fullfile(cgalLibLoc, "auxiliary\gmp\lib");
lib3 = fullfile(boostLoc, "libs");
lib4 = fullfile(boostLoc, "lib64-msvc-14.1");
libraries = [lib1, lib2, lib3, lib4];
% 包含目录
include1 = fullfile(cgalLibLoc, "include");
include2 = fullfile(cgalLibLoc, "build\include");
include3 = fullfile(cgalLibLoc, "auxiliary\gmp\include");
include4 = boostLoc;
includePath = [include1, include2, include3, include4];
% 创建库的定义文件
clibgen.generateLibraryDefinition(headerFiles, "IncludePath", includePath, ...
    'Libraries', libraries,  'PackageName', 'crypto', 'verbose', true);