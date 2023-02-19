classdef Edge
    %	边拓扑信息类
    %   Detailed explanation goes here
    
    properties (Access = public)
        n_verticesId; %相邻点的index
        n_edgesId; %相邻边的index
        n_facesId; %相邻面的index
        v_facesId; %点连接面的index
    end
    
    methods (Access = public)
        %Constructor
        function obj = Edge(edge)     
            obj.n_verticesId = edge.n_verticesId;
            obj.n_edgesId = edge.n_edgesId;
            obj.v_facesId = edge.v_facesId;
            obj.n_facesId = edge.n_facesId;
        end
        
    end
end

