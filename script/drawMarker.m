function drawMarker( pose )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

hslice = surf(linspace(xmin,xmax,100),...
   linspace(ymin,ymax,100),...
   zeros(100));
rotate(hslice,[-1,0,0],-45);
figure
colormap(jet)

end

