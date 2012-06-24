function [ R ] = rotmat( angle )
%ROTMAT Summary of this function goes here
%   Detailed explanation goes here

R = [cos(angle) -sin(angle); sin(angle) cos(angle)];

end

