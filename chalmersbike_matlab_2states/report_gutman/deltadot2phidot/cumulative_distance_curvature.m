% Calculate cumulative distance along path. 
% Adapted from https://blogs.mathworks.com/steve/2012/07/06/walking-along-a-path/

xy = [path_x path_y];
d = diff(xy,1);
dist_from_vertex_to_vertex = hypot(d(:,1), d(:,2)); % C = hypot(A,B) -> C = sqrt(abs(A).^2 + abs(B).^2)
cumulative_dist_along_path = [0;cumsum(dist_from_vertex_to_vertex,1)];


% Radius of Curvature Calculation
for i=1:(length(path_x)-2)
    
    length_a=sqrt( (path_x(i+1) - path_x(i) )^2 + ( path_y(i+1) - path_y(i) )^2 );
    length_b=sqrt( (path_x(i+2) - path_x(i) )^2 + ( path_y(i+2) - path_y(i) )^2 );
    length_c=sqrt( (path_x(i+2) - path_x(i+1) )^2 + (path_y(i+2) - path_y(i+1) )^2 );
    per=(length_a+length_b+length_c)/2;
    area=sqrt( per * (per-length_a) * (per-length_b) * (per-length_c) );
    
    curvature(i+1)=(4*area)/(length_a*length_b*length_c);
    
    if curvature(i+1)<0.01
        curvature(i+1)=0;
    end
    
    curvature(i+1)=abs(curvature(i+1));
    
end

curvature(1)=curvature(2);
curvature(length(path_x))=curvature(length(path_x)-1);