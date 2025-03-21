function [b] = range_angle(a)
b = a+floor((pi-a)/(2*pi))*2*pi;
end