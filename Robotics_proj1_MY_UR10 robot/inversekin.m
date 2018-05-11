%% UR10 DH parameters
function [q] = inversekin(x1,y1,z1,roll1,pitch1,yaw1)

L1 = 0.1273;   L2 = -0.612; L3 = -0.5723;
L4 = 0.163941; L5 = 0.1157; L6 = 0.0922;
%qz = ([ 0 0 0 0 0 0]);

%syms x1 y1 z1 r1 p1 y1

% using peter corke robotics tool box
%serial link
% DH = [THETA D A ALPHA SIGMA OFFSET] p for presmatic

L(1) = Link ([ 0 L1 0 pi/2 0 0]);

L(2) = Link ([ 0 0 L2 0 0 0]);

L(3) = Link ([ 0 0 L3 0 0 0 ]);

L(4) = Link ([ 0 L4 0 pi/2 0 0]);

L(5) = Link ([ 0 L5 0 -pi/2 0 0]);

L(6) = Link ([ 0 L6 0 0 0 0 ]) ;

ur10robot = SerialLink(L);
ur10robot.name = 'YehiaUR10';

obj = transl( -y1 , x1 , z1) *...
    rpy2tr(roll1,pitch1,yaw1);

q = ur10robot.ikine(obj , [0 0 0 0 0 0], [1 1 1 0 0 0]);

end