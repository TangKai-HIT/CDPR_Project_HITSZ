classdef Face
    %	面拓扑信息类
    %   Detailed explanation goes here
    
    properties (Access = public)
        n_verticesId; %相邻点的index
        n_edgesId; %相邻边的index
        n_facesId; %相邻面的index
        v_edgesId; %点连接边的index
        v_facesId; %点连接面的index
        normal_vec; %平面外单位法向量
    end
    
    methods (Access = public)
        %Constructor
        function obj = Face(face)
            obj.n_verticesId = face.n_verticesId;
            obj.n_edgesId = face.n_edgesId;
            obj.n_facesId = face.n_facesId;
            obj.v_facesId = face.v_facesId;
            obj.normal_vec = face.normal_vec;
            obj.v_edgesId = face.v_edgesId;
        end
        
    end
end