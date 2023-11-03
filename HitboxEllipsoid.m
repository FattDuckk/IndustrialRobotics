function [EptsUR3,EptsKUKA] = HitboxEllipsoid(r1,r2)
%% UR3
clf
clear
% r1 = ModifiedUR3;

centerPoint = [0 0 0];
% radii = [0.328,0.091,0.123];
radiiUR3 = [0.128 0.163 0.261;
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
X1 = zeros(21,168);
Y1 = zeros(21,168);
Z1 = zeros(21,168);

for j = 1:21:168
    [X1(1:21,j:m),Y1(1:21,j:m),Z1(1:21,j:m)] = ellipsoid(centerPoint(1), centerPoint(2), centerPoint(3), radiiUR3(k,1), radiiUR3(k,2), radiiUR3(k,3));
    m = m + 21;
end
X1 = X1(:);
Y1 = Y1(:);
Z1 = Z1(:);

EptsUR3(1:length(X1),1) = X1;
EptsUR3(1:length(X1),2) = Y1;
EptsUR3(1:length(X1),3) = Z1;

b = 1;
n = 422;
for i = 1:8
    r1.model.points{i} = [X1(b:n), Y1(b:n), Z1(b:n)];
    warning off
    r1.model.faces{i} = delaunay(r1.model.points{i});
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
% hold on
r1.model.plot3d([0,0,0,0,0,0,0,0,0]);
% hold off
% alpha(0.01)
axis equal
camlight

%% KUKA
clf
clear
% r2 = KUKA;
% numj = 12;
centerPoint = [0 0 0];
radiiKUKA = [ 0.20799, 0.046, 0.058 ];
% radii = [0.5,0.216,1.12;
%             0.217005,0.207952,0.157513;
%             0.135983,0.250555,0.182625;
%             0.136013,0.182614,0.284818;
%             0.136095,0.251215,0.182625;
%             0.136016,0.182597,0.285527;
%             0.135971, 0.234071, 0.167121 ;
%             0.132334, 0.129053,  0.167061;
%             0.104072, 0.104033, 0.030367;
%             0.061356, 0.092336, 0.125051;
%             0.20799, 0.046, 0.058 
%             0.01, 0.01, 0.01];

[X2,Y2,Z2] = ellipsoid(centerPoint(1), centerPoint(2), centerPoint(3), radiiKUKA(1), radiiKUKA(2), radiiKUKA(3));
% k = 1;
% m = 21;
% X2 = zeros(21,21*numj);
% Y2 = zeros(21,21*numj);
% Z2 = zeros(21,21*numj);
% 
% for j = 1:21:21*numj
%     [Z2(1:21,j:m),X2(1:21,j:m),Y2(1:21,j:m)] = ellipsoid(centerPoint(1), centerPoint(2), centerPoint(3), radii(k,1), radii(k,2), radii(k,3));
%     m = m + 21;
% end
% X2 = X2(:);
% Y2 = Y2(:);
% Z2 = Z2(:);


% b = 1;
% n = 422;
% for i = 1:12
%     r2.model.points{i} = [X2(b:n), Y2(b:n), Z2(b:n)];
%     warning off
%     r2.model.faces{i} = delaunay(r2.model.points{i});
%     warning on;
%     b = b + 422;
%     n = n + 422;
% end
% 
for i = 11
    r2.model.points{i} = [X2(:), Y2(:), Z2(:)];
    warning off
    r2.model.faces{i} = delaunay(r2.model.points{i});
    warning on;

end
EptsKUKA(1:441,1) = X2(:);
EptsKUKA(1:441,2) = Y2(:);
EptsKUKA(1:441,3) = Z2(:);
r2.model.plot3d([-0.01,0,0,0,0,0,0,0,0,0,0,0],'workspace',[-2 2 -2 2 -2 2]);
% alpha(0.1)

axis equal
camlight
end
