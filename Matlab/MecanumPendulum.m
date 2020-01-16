function [Rw,l1,l2,alpha,lv] = MecanumPendulum()
% wheel offset

l1 = 0.025;%0.06;
l2 = 0.20;%0.15;
% wheel radius
Rw = 0.045;%0.04;
% distance wheel axis to velocity controlled ponit on the robot (estimation 10 cm)
lv = 0.12;
% wheel roller angle
alpha = [-45 45 -45 45]*pi/180;

end

