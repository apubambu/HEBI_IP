function [Rw,l1,l2,alpha] = MecanumPendulum()
% wheel offset
l1 = 0.06;
l2 = 0.15;
% wheel radius
Rw = 0.04;
% wheel roller angle
alpha = [45 -45 -45 45]*pi/180;
end

