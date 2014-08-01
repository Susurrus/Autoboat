function [ out ] = rotmat2( theta )
%ROTMAT Generate 2x2 rotationmatrix based on input angle
    out = [cos(theta) -sin(theta); sin(theta) cos(theta)];
end

