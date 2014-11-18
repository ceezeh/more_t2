function plotpose
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
P0 = csvread('/home/parallels/catkin_ws/src/more_t2/posedata/pose_cam_0.csv');
P1 = csvread('/home/parallels/catkin_ws/src/more_t2/posedata/pose_cam_1.csv');
plot3(P0(:,1), P0(:,2),P0(:,3));
axis equal;
% hold on;
figure;
plot3(P1(:,1), P1(:,2),P1(:,3));
% hold off;
axis equal;
end

