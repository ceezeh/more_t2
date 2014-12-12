%estimate marker jitter across cameras.

P0 = csvread('/home/parallels/catkin_ws/src/more_t2/posedata/pose_cam_0.csv');
P1 = csvread('/home/parallels/catkin_ws/src/more_t2/posedata/pose_cam_1.csv');

ptr0 =1;
ptr1 =1; 
noCams = 2;
currTime = 0;
index =1;
data = [];
while 1
    if (P0(ptr0, 7) < P1(ptr1, 7))
        ptr0 = ptr0 + 1;
        if ptr0 > size(P0,1)
            break;
        end
    elseif (P0(ptr0, 7) > P1(ptr1, 7)) 
        ptr1 = ptr1 + 1;
        if ptr1 > size(P1,1)
            break;
        end
    else
        data = vertcat(data, P0(ptr0, 1:6)-P1(ptr1, 1:6));
        ptr0 = ptr0 + 1;
        ptr1 = ptr1 + 1;
        if ptr0 > size(P0,1)
            break;
        end
        if ptr1 > size(P1,1)
            break;
        end
    end
    
end

disp(std(data))