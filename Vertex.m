classdef Vertex
    %	点拓扑信息类
    %   Detailed explanation goes here
    
    properties (Access = public)
        n_verticesId; %相邻点的index
        n_edgesId; %相邻边的index
        n_facesId; %相邻面的index
        localEdgeVector; %3X4,局部单元中边的单位向量
        coordinate; %坐标
    end
    
    methods (Access = public)
        %Constructor
        function obj = Vertex(vertex)
            obj.n_verticesId = vertex.n_verticesId;
            obj.n_edgesId = vertex.n_edgesId;
            obj.n_facesId = vertex.n_facesId;
            obj.localEdgeVector = vertex.localEdgeVector;
            obj.coordinate = vertex.coordinate;
        end
        
    end
end

