% function iscollision = collisionCheck(trans1,trans2,robot1,robot2)
% 
% r1jointpos = robot1.model.getpos;
% r2jointpos = robot2.model.getpos;
% 
% r1jointpos(1:3) = r1jointpos(6:8);
% r2jointpos(1:3) = r2jointpos(9:11);
% j = 1;
% 
% 
% 
% 
% tr1 = T(robot1.model.fkine(trans1));
% tr2 = T(robot2.model.fkine(trans2));
% distance = sqrt(sum((tr1(1:3,4)-tr2(1:3.4)').^2));
% if distance == 0 
%     disp("stop")
%     pause
%     iscollision = 1;
% else
%     disp('safe')
%     iscollision = 0;
% end
% 
% 
% end
%% UR3
clf
clear
close all
r = ModifiedUR3;

centerPoint = [0 0 0];
% radii = [0.328,0.091,0.123];
radii = [0.128 0.163 0.261;
            0.328,0.091,0.123;
            0.283,0.075,0.971;
            0.0644,0.0768,0.0877;
            0.0644,0.0768,0.0877;
            0.063,0.0696,0.0398;
            0.0614,0.0923,0.126;
            0.197,0.045,0.1
            0,0,0];

% [X,Y,Z] = ellipsoid(centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3));
k = 1;
m = 21;
X = zeros(21,168);
Y = zeros(21,168);
Z = zeros(21,168);

for j = 1:21:168
    [X(1:21,j:m),Y(1:21,j:m),Z(1:21,j:m)] = ellipsoid(centerPoint(1), centerPoint(2), centerPoint(3), radii(k,1), radii(k,2), radii(k,3));
    m = m + 21;
end
X = X(:);
Y = Y(:);
Z = Z(:);


b = 1;
n = 422;
for i = 1:8
    r.model.points{i} = [X(b:n), Y(b:n), Z(b:n)];
    warning off
    r.model.faces{i} = delaunay(r.model.points{i});
    warning on;
    b = b + 422;
    n = n + 422;
end
% 
% for i = 1:9
%     r.model.points{i} = [X(:), Y(:), Z(:)];
%     warning off
%     r.model.faces{i} = delaunay(r.model.points{i});
%     warning on;
% 
% end
% 

r.model.animate([0,0,0,0,0,0,0,0,0]);

% xlim('auto')
% ylim('auto')
% zlim('auto')
% alpha(0.1)
axis equal
camlight

%% KUKA
clf
clear
r = KUKA;
numj = 12;
centerPoint = [0 0 0];
% radii = [1.12,0.5,0.11067];
radii = [0.5,0.216,1.12;
            0.217005,0.207952,0.157513;
            0.135983,0.250555,0.182625;
            0.136013,0.182614,0.284818;
            0.136095,0.251215,0.182625;
            0.136016,0.182597,0.285527;
            0.135971, 0.234071, 0.167121 ;
            0.132334, 0.129053,  0.167061;
            0.104072, 0.104033, 0.030367;
            0.061356, 0.092336, 0.125051;
            0.20799, 0.046, 0.058 
            0.01, 0.01, 0.01];

% [Z,X,Y] = ellipsoid(centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3));
k = 1;
m = 21;
X = zeros(21,21*numj);
Y = zeros(21,21*numj);
Z = zeros(21,21*numj);

for j = 1:21:21*numj
    [Z(1:21,j:m),X(1:21,j:m),Y(1:21,j:m)] = ellipsoid(centerPoint(1), centerPoint(2), centerPoint(3), radii(k,1), radii(k,2), radii(k,3));
    m = m + 21;
end
X = X(:);
Y = Y(:);
Z = Z(:);


b = 1;
n = 422;
for i = 1:12
    r.model.points{i} = [X(b:n), Y(b:n), Z(b:n)];
    warning off
    r.model.faces{i} = delaunay(r.model.points{i});
    warning on;
    b = b + 422;
    n = n + 422;
end
% 
% for i = 1
%     r.model.points{i} = [X(:), Y(:), Z(:)];
%     warning off
%     r.model.faces{i} = delaunay(r.model.points{i});
%     warning on;
% 
% end


r.model.animate([-0.5854    0.0009    0.0727    0.7652-0.3491    0.2359    2.1368+0.3491   -3.7208   -0.3004    0.7854         0         0   0],'workspace',[-1 1 -1 1 0 ]);
% alpha(0.1)
axis equal
camlight
