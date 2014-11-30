P0 = csvread('/home/parallels/catkin_ws/src/more_t2/posedata/pose_cam_1.csv');


	xd = zeros(size(P0(:,5)));
    zd= zeros(size(P0(:,5)));
	% Careful recovery of direction from tan
    mask1 = P0(:,5) >= 0;
    mask2 = P0(:,5) < pi/2;
    mask3 = mask1&mask2;
    xd(mask3) = abs(tan(P0(mask3,5)));
    zd(mask3) = 1;
    
    mask1 = P0(:,5) >= pi/2;
    mask2 = P0(:,5) < pi;
    mask3 = mask1&mask2;
    xd(mask3) = abs(tan(P0(mask3,5)));
    zd(mask3) = -1;
    
    mask1 = P0(:,5) >= -pi;
    mask2 = P0(:,5) < -pi/2;
    mask3 = mask1&mask2;
    xd(mask3) = -abs(tan(P0(mask3,5)));
    zd(mask3) = -1;
    
    mask1 = P0(:,5) >= -pi/2;
    mask2 = P0(:,5) < 0;
    mask3 = mask1&mask2;
    xd(mask3) = -abs(tan(P0(mask3,5)));
    zd(mask3) = 1;

    yd = sin(P0(:,4));
    

% y-axis
Zm = [xd yd zd];
X0 = [zd zeros(size(zd)) -xd];
Y0=cross(Zm', X0');
Y0 = Y0';


rod = @rodrigues;
zm = zeros(size(Y0));
Ym = zeros(size(Y0));
ym = zeros(size(Y0));
for i = 1:size(Y0,1)
    zm(i,:) = Zm(i,:)/norm(Zm(i,:));
    disp(zm(i,:));
    Ym(i,:) = rod( zm(i,:) ,Y0(i,:), P0(i,6));
    ym(i,:) = Ym(i,:)/norm(Ym(i,:));
end



%x-axis
quiver3(P0(1:10:end,1), P0(1:10:end,2), P0(1:10:end,3), zm(1:10:end,1), zm(1:10:end,2), zm(1:10:end,3));
hold on
quiver3(P0(1:10:end,1), P0(1:10:end,2), P0(1:10:end,3), ym(1:10:end,1), ym(1:10:end,2), ym(1:10:end,3));
	xm = cross(ym,zm);
    quiver3(P0(1:10:end,1), P0(1:10:end,2), P0(1:10:end,3), xm(1:10:end,1), xm(1:10:end,2), xm(1:10:end,3))
xlabel('x')
ylabel('y')
zlabel('z')
axis equal;