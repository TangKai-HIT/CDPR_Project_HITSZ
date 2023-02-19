function plotCircle3D(axi, center,normal,radius)

    theta=0:0.02:2*pi;
      if size(normal,1) ~= 1
        normal = normal';
    end
    v=null(normal);
    if size(center,1) == 1
        center = center';
    end
    points=repmat(center,1,size(theta,2))+radius*(v(:,1)*cos(theta)+v(:,2)*sin(theta));
    plot3(axi, points(1,:), points(2,:), points(3,:), 'r-', 'LineWidth', 1.5);
    
end