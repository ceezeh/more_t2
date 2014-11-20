function plotpose
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
P0 = csvread('/home/parallels/catkin_ws/src/more_t2/posedata/pose_cam_0.csv');
P1 = csvread('/home/parallels/catkin_ws/src/more_t2/posedata/pose_cam_1.csv');
% plot3(P0(:,1), P0(:,2),P0(:,3), '-.r*');

figure;
hold on;
axis equal;

plot3(P0(:,1), P0(:,2),P0(:,3), '-.r*');

plot3(P1(:,1), P1(:,2),P1(:,3), '-.bo');
% hold off;
axis equal;

% ptrs = [1, 1];
% 
% length = max(size(P0,1), size(P1,1));
% while(any(ptrs <= length))
% %     Compare times
%     if (P0(ptrs(1),5) < P1(ptrs(2),5))
%         plot3(P0(ptrs(1) ,1), P0(ptrs(1) ,2),P0(ptrs(1) ,3), '-.r*');
%         ptrs(1) = ptrs(1) + 1;
%     else
%         plot3(P1(ptrs(2) ,1), P1(ptrs(2) ,2),P0(ptrs(2) ,3), '-.bo');
%         ptrs(2) = ptrs(2) + 1;
%     end
% end

hold off;
end

