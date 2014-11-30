function out = rodrigues( z,y,roll )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
out =  y * cos(roll) + cross(z, y) * sin(-roll)...
			+ z * dot(z,y,2) * (1 - cos(roll));

end

