function iscollision = collisionCheck(robot1,robot2,EptsUR3,EptsKUKA,radii)
centerPoint = [0 0 0]; 

%% ModifiedUR3
r1jointpos = robot1.model.getpos;
% tr1 = T(robot1.model.fkine(r1jointpos));
tr1 = zeros(4,4,robot1.model.n+1);
tr1(:,:,1) = robot1.model.base;
L1 = robot1.model.links;
for  i = 1:robot1.model.n
    tr1(:,:,i+1) = tr1(:,:,i) * trotz(r1jointpos(i)) * transl(0,0,L1(i).d) * transl(L1(i).a,0,0) * trotx(L1(i).alpha);
end

% % Modify Later----------------------------
% cubePointsAndOnes = [inv(tr) * [cubePoints,ones(size(cubePoints,1),1)]']';
% updatedCubePoints = cubePointsAndOnes(:,1:3);
% algebraicDist = GetAlgebraicDist(updatedCubePoints, centerPoint, radii);
% pointsInside = find(algebraicDist < 1);
% disp(['2.9: There are now ', num2str(size(pointsInside,1)),' points inside']);
% %-----------------------------------------



%% KUKA
% r2jointpos = robot2.model.getpos;
% tr2 = T(robot2.model.fkine(r2jointpos));

%% Going through each ellipsoid
for i = 8
    KUKAEpts = [inv(tr1(:,:,i)) * [EptsKUKA,ones(size(EptsKUKA,1),1)]']';
    updatedKUKAPoints = KUKAEpts(:,1:3);
    algebraicDist = GetAlgebraicDist(updatedKUKAPoints, centerPoint, radii);
    pointsInside = find(algebraicDist < 33.15);
%     ptcheck = find(algebraicDist<=30)
    disp(min(algebraicDist))
    
%     disp(['2.10: There are ', num2str(size(pointsInside,1)),' points inside the ',num2str(i),'th ellipsoid']);
    if isempty(pointsInside) == false
        disp("Collision!")
        iscollision = 1;
    else 
        disp("Safe")
        iscollision = 0;
    end

end

% 
end