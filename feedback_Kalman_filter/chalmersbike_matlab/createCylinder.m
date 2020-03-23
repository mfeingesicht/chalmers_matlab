function [f1, f2, v] = createCylinder(w,d,n)
t = linspace(0,2*pi,n);
x = w*cos(t);
y = w*sin(t);
z = -d*ones(1,n);

% x = [0 x];
% y = [0 y];

f1 = [
    1:n
    n+1:2*n
];

f2 = [
    (1:n-2)' (2:n-1)' (n+2:2*n-1)' (n+1:2*n-2)';
    1 n-1 2*n-1 n+1
];



v = [
    x' y' z'
    x' y' -z'
];

% close all
% g = hgtransform;     
% patch('Vertices',v,'Faces',f1,'FaceColor',[.75 .75 .75],'Parent',g)
% patch('Vertices',v,'Faces',f2,'FaceColor',[.75 .75 .75],'Parent',g)

