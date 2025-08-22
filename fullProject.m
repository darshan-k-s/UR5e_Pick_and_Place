% Project2.m
% MTRN4230 Project 2 25T2
% Name: Darshan Komala Sreeramu
% Zid: z5610741

clc;
clear; 

startup_rvc;

global RUN_MODE
% =========================================================
% RUN_MODE = 'sim';
RUN_MODE = 'real';
% =========================================================
% -----------------------------------------------------------------------
% Params FOR part A
global groundTruth gtOffsets SMALL_ITEM_SLOT_OFFSET BIG_ITEM_SLOT_OFFSET_HORIZ BIG_ITEM_SLOT_OFFSET_DOWN IMAGE_TYPE

% ================================
% 'camera' OR 'file'
IMAGE_TYPE= 'camera';
% =================================================================
BIG_ITEM_SLOT_OFFSET_HORIZ = 35;
BIG_ITEM_SLOT_OFFSET_DOWN = 100;
SMALL_ITEM_SLOT_OFFSET = 87;
% =============================================== 

groundTruth= [-990,60; -230, 60; -990,-520; -230, -520];
gtOffsets =[100, -50; -50, -50; 190, 120; -50, 50;];

% For part B
global K_ATTR K_REP PARABOLIC_LIMIT REP_DIST STEP_SIZE_APF STEP_SIZE PATH_TOLERANCE

STEP_SIZE_APF= 20; % grid resolution 
K_ATTR= 20;
K_REP = 3.5e9;
REP_DIST = 170;  % obst distance of influence
STEP_SIZE = 5; % step size for path
PATH_TOLERANCE = 3;  % tolerance for path planning
PARABOLIC_LIMIT = 100;  % threshold for parabolic vs conic

% For part C
global ACC VEL BLEND MOVE_OFFSET MOVE_HEIGHT VACUUM_PAUSE HIGH_HOME JOINT_HOME HOME_POS

function host = getHostAddr()
    global RUN_MODE HOME_POS
    if strcmp('sim', RUN_MODE)
        host = '127.0.0.1';
    else
        host = '192.168.0.100';
    end
end

JOINT_HOME = deg2rad([0, -75, 90, -105, -90, 0]); 
HOME_POS= [-588.53, -133.30, 227.00, 2.221, 2.221, 0];
HIGH_HOME= [-588.53, -133.30, 350.00, 2.221, 2.221, 0]; 

function height = PICKUP_HEIGHT()
    global RUN_MODE
    if strcmp('sim', RUN_MODE)
        height = 20; % in sim
    else
        height = -5; % in real 
    end
end  

% Movement params
MOVE_OFFSET = 15;
MOVE_HEIGHT = PICKUP_HEIGHT() + MOVE_OFFSET;
VACUUM_PAUSE = 3; % wait after vacuum

ACC = 0.8;
VEL = 0.3;
BLEND = 0.000;

% ========================== RUN START =======================================

fprintf('Which part(B, C, D, E)?: ');
selection = input('','s');

RTDE_PORT = 30003;
VACC_PORT = 63352;

switch selection
    case 'B'
        [Img, Img2, Img3,worldCoords, obstCoordsList, smallItemCoords, smallYaw, goalCoords, smallGoalCoords] =Transf();

        [path] = pathGen(Img3, worldCoords, obstCoordsList, smallItemCoords, smallGoalCoords);

    case 'C'
        
        rtde = rtde(getHostAddr(), RTDE_PORT);
                
        if ~strcmp(RUN_MODE, 'sim')
            vacuum = vacuum(getHostAddr(), VACC_PORT);
            vacuum.release();
        else
           vacuum = [];
        end

        [Img, Img2, Img3, worldCoords, obstCoordsList, smallItemCoords, smallYaw, goalCoords, smallGoalCoords, smallItemGoalOrientation] =Transf();

        [path] = pathGen(Img3, worldCoords, obstCoordsList, smallItemCoords, smallGoalCoords);
        
        partC(rtde, vacuum, smallItemCoords, smallGoalCoords, smallYaw, smallItemGoalOrientation, path);

    case 'D'
        rtde = rtde(getHostAddr(), RTDE_PORT);

        if ~strcmp(RUN_MODE, 'sim')
            vacuum = vacuum(getHostAddr(), VACC_PORT);
            vacuum.release();
        else
           vacuum = [];
        end        

        [Img, Img2, Img3, worldCoords, obstCoordsList, smallItemCoords, smallYaw, goalCoords, smallGoalCoords, smallItemGoalOrientation, ...
          bigItemWorldCoords, bigItemOrientation, bigItemGoalWorldCoords, bigItemGoalOrientation] =Transf();
        
        partD(Img3, worldCoords, rtde, vacuum, bigItemWorldCoords, bigItemGoalWorldCoords, bigItemOrientation, bigItemGoalOrientation, obstCoordsList);

    case 'E'
        rtde = rtde(getHostAddr(), RTDE_PORT);
        
        if ~strcmp(RUN_MODE, 'sim')
            vacuum = vacuum(getHostAddr(), VACC_PORT);
            vacuum.release();
        else
           vacuum = [];
        end

        [Img, Img2, Img3, worldCoords, obstCoordsList, smallItemCoords, smallYaw, goalCoords, smallGoalCoords, smallItemGoalOrientation] =Transf();

        [path] = pathGen(Img3, worldCoords, obstCoordsList, smallItemCoords, smallGoalCoords);
        
        partE(rtde, vacuum, smallItemCoords, smallGoalCoords, smallYaw, smallItemGoalOrientation, path);

    otherwise
        disp('Selection INVALID');
end

% HELPER FUNCS
function waitToMove(rtde)
    pause(0.5); 
    err = inf;
    while err > 0.1
        delta = rtde.targetJointPositions() - rtde.actualJointPositions();
        err = sum(abs(delta));
    end
end

% Get Image
function Img = getImage()
    global IMAGE_TYPE
    mode = lower(string(IMAGE_TYPE));  % normalize once

    switch mode
        case "file"
            disp('Enter image path:');
            imagePath = input('', 's');
            if ~exist(imagePath, 'file')
                error('File does not exist');
            end
            Img = imread(imagePath);

        case "camera"
            cam = webcam(2);     % adjust index ------------------------------------------------
            Img = snapshot(cam);
            clear cam
    end
end

function imageCoords = transformWorldCoordsToWarpedImageCoords(worldCoords, minX, minY)
    imageCoords = worldCoords - [minX, minY];
end


% --------------- PART A
function [Img, Img2, Img3, worldCoords, obstCoordsList, smallItemCoords, smallYaw, goalCoords, smallGoalCoords, smallItemGoalOrientation, ...
          bigItemWorldCoords, bigItemOrientation, bigItemGoalWorldCoords, bigItemGoalOrientation] = Transf()
    

    global groundTruth SMALL_ITEM_SLOT_OFFSET BIG_ITEM_SLOT_OFFSET_HORIZ BIG_ITEM_SLOT_OFFSET_DOWN

    % HELPERS
    function [X, Y, topLeft, topRight, botLeft, botRight] = cornersFind(ids, locs)
        botLeft = mean(locs(:, :, ids == 2));
        botRight = mean(locs(:, :, ids == 3));
        topLeft = mean(locs(:, :, ids == 0));
        topRight = mean(locs(:, :, ids == 1));
    
        X = [topLeft(1), topRight(1), botLeft(1), botRight(1)];
        Y = [topLeft(2), topRight(2), botLeft(2), botRight(2)];
    end


    while true
        Img = getImage();
    
        [ids, locs] = readArucoMarker(Img, "DICT_ARUCO_ORIGINAL");
        
        % Check if all markers detected
        if ~((size(find(ids == 6), 2)==2) && all(ismember([0,1, 2, 3, 5, 6, 7],ids)))
           Img = imsharpen(Img, 'Radius', 1, 'Amount', 0.8); 
           Img = rgb2gray(Img);
           Img = imadjust(Img, [0.3 1]);% contrast
           figure(1);
           imshow(Img);
           [ids, locs] = readArucoMarker(Img, "DICT_ARUCO_ORIGINAL");
        else
            break;
        end
        

        warning('Found: %s', mat2str(ids));
        warning('Not all ArUco markers were detected.');
    end

    [xBounds, yBounds] = cornersFind(ids, locs);
    % Real world
    worldCoords = groundTruth;

    % Find small item props
    idxSmall = find(ids == 5);
    if ~isempty(idxSmall)
        smallItemCornersImageCoords = locs(:, :, idxSmall (1));
        smallItemImageCoords = mean(smallItemCornersImageCoords);
    else
        error('Small item not found');
    end

    % Find big item props
    bigItemIndices = find(ids == 6);
    if size(bigItemIndices, 2) == 2
        bigItem1CornersImageCoords = locs(:, :, bigItemIndices(1));
        bigItem1ImageCoords = mean(bigItem1CornersImageCoords);
        
        bigItem2CornersImageCoords = locs(:, :, bigItemIndices(2));
        bigItem2ImageCoords = mean(bigItem2CornersImageCoords);

        bigItemImageCoords = mean([bigItem1ImageCoords; bigItem2ImageCoords]);
        bigItemCornersImageCoords = bigItem1CornersImageCoords;
    else
        error('Big item not found');
    end

    % Find obstacles props
    obstacleIndices = find(ids == 4);
    obstacleImageCoordsList = [];
    for i = 1:length(obstacleIndices)
        obstacleCenter = mean(locs(:,:, obstacleIndices(i)));
        obstacleImageCoordsList = [obstacleImageCoordsList; obstacleCenter];
    end

    % Find obstacles props
    goalIndices = find(ids == 7);
    if ~isempty(goalIndices)
        goalCornersImageCoords = locs(:, :, goalIndices(1));
        goalImageCoords = mean(goalCornersImageCoords);
    else
        error('Goal not found');
    end

    maxX = max(worldCoords(:, 1));
    maxY = max(worldCoords(:, 2));
    minX = min(worldCoords(:, 1));
    minY = min(worldCoords(:, 2));
    
    outputSize = [maxY - minY, maxX - minX];
    outputRef = imref2d(outputSize, [minX, maxX], [minY, maxY]);

    % Transformation
    tform = fitgeotrans([xBounds', yBounds'], worldCoords, "projective");
    Img2 = imwarp(Img, tform, 'OutputView', outputRef);
    
    % Image to world coords using tform
    obstCoordsList = tform.transformPointsForward(obstacleImageCoordsList);

    smallItemCoords= tform.transformPointsForward(smallItemImageCoords);
    smallItemCornersWorldCoords = tform.transformPointsForward(smallItemCornersImageCoords);
    goalCoords = tform.transformPointsForward(goalImageCoords);
    goalCornersWorldCoords = tform.transformPointsForward(goalCornersImageCoords);
    bigItemWorldCoords = tform.transformPointsForward(bigItemImageCoords);
    bigItemCornersWorldCoords = tform.transformPointsForward(bigItemCornersImageCoords);

    % Orientation of small & big items
    smallYaw = smallItemCornersWorldCoords(1, :) - smallItemCornersWorldCoords(2, :);
    smallYaw = smallYaw /norm(smallYaw);
    bigItemOrientation = bigItemCornersWorldCoords(1, :) - bigItemCornersWorldCoords(2, :);
    bigItemOrientation = bigItemOrientation / norm(bigItemOrientation);

    % Small goal offset correction
    goalTopLeft = goalCornersWorldCoords(1, :);
    goalTopRight = goalCornersWorldCoords(2, :);
    smallItemGoalOrientation = goalTopRight - goalTopLeft;
    smallItemGoalOrientation = smallItemGoalOrientation / norm(smallItemGoalOrientation);
    smallGoalCoords = goalCoords - SMALL_ITEM_SLOT_OFFSET * smallItemGoalOrientation;

    % big item goal
    goalTopLeft = goalCornersWorldCoords(1, :);
    goalTopRight = goalCornersWorldCoords(2, :);
    goalBottomRight = goalCornersWorldCoords(3, :);

    down = goalBottomRight - goalTopRight;
    down = down / norm(down);

    bigItemGoalOrientation = goalTopRight - goalTopLeft;
    bigItemGoalOrientation = bigItemGoalOrientation / norm(bigItemGoalOrientation);
    bigItemGoalWorldCoords = goalCoords - BIG_ITEM_SLOT_OFFSET_HORIZ() * bigItemGoalOrientation + BIG_ITEM_SLOT_OFFSET_DOWN() * down;

    % Draw dots
    Img3 = Img2;
    for i = 1:size(obstCoordsList, 1)
        Img3 = insertShape(Img3, "FilledCircle", [transformWorldCoordsToWarpedImageCoords(obstCoordsList(i, :), minX, minY), 10], 'Color', 'red', 'Opacity', 1);
    end
    Img3 = insertShape(Img3, "FilledCircle", [transformWorldCoordsToWarpedImageCoords(smallItemCoords, minX, minY), 10], 'Color', 'blue', 'Opacity', 1);
    Img3 = insertShape(Img3, "FilledCircle", [transformWorldCoordsToWarpedImageCoords(smallGoalCoords, minX, minY), 10], 'Color', 'green', 'Opacity', 1);
    Img3 = insertShape(Img3, "FilledCircle", [transformWorldCoordsToWarpedImageCoords(bigItemWorldCoords, minX, minY), 10], 'Color', 'black', 'Opacity', 1);
    Img3 = insertShape(Img3, "FilledCircle", [transformWorldCoordsToWarpedImageCoords(bigItemGoalWorldCoords, minX, minY), 10], 'Color', 'white', 'Opacity', 1);
    

    disp('PART A:');
    fprintf('Small item: %s\n', mat2str(smallItemCoords, 3));
    fprintf('Goal: %s\n', mat2str(smallGoalCoords, 3));
    for i = 1:size(obstCoordsList, 1)
        fprintf('Obstacle %d: %s\n', i, mat2str(obstCoordsList(i, :), 3));
    end
    fprintf('Small item orientation: %s\n', mat2str(smallYaw, 3));
    fprintf('Small goal orientation: %s\n', mat2str(smallItemGoalOrientation, 3));
    disp('---------------------------------------------------------------');
end

% HELPERS FOR part B
function [imageX, imageY] = transformWorldCoordsToWarpedImageCoords2(X, Y, minX, minY)
    % Transform world coordinates to image coordinates
    imageX = X - minX;
    imageY = Y - minY;
end

% --------------- PART B --------------- %
function [path] = pathGen(Img, worldCoords, obstCoordsList, ...
          smallItemCoords, smallGoalCoords)
    global STEP_SIZE_APF

    maxX = max(worldCoords(:, 1));
    maxY = max(worldCoords(:, 2));
    minX = min(worldCoords(:, 1));
    minY = min(worldCoords(:, 2));
    
    [X, Y] = meshgrid(minX:STEP_SIZE_APF:maxX, minY:STEP_SIZE_APF:maxY);

    Fx = zeros(size(X));
    Fy = zeros(size(Y));
    
    for i = 1:size(X, 1)
        for j = 1:size(X, 2)
            currentPos = [X(i,j), Y(i,j)];
            
            [fxAtt, fyAtt] = calculateAttractiveForce(currentPos, smallGoalCoords);
            [fxRep, fyRep] = calculateRepulsiveForce(currentPos, obstCoordsList, smallGoalCoords);
            
            Fx(i,j) = fxAtt + fxRep;
            Fy(i,j) = fyAtt + fyRep;
        end
    end
    
    displayPotentialField(Img, worldCoords, X, Y, Fx, Fy);

    path = planPathWithAPF(smallItemCoords, smallGoalCoords, obstCoordsList);
    displayAPFPath(Img, worldCoords, path);

    displayAllPartAandB(Img, worldCoords, X, Y, Fx, Fy, path);
end

% Calculate attractive force using parabolic when dist <= PARABOLIC_THRESHOLD 
% and conic well when dist > PARABOLIC_THRESHOLD
function [fx, fy] = calculateAttractiveForce(currentPos, goalPos)
global groundTruth K_ATTR PARABOLIC_LIMIT
    currToGoal = goalPos - currentPos;
    distanceToGoal = norm(currToGoal);
    
    if distanceToGoal <= PARABOLIC_LIMIT
        % Parabolic well: F = k_att * distance
        if distanceToGoal > 0
            fx = K_ATTR * currToGoal(1);
            fy = K_ATTR * currToGoal(2);
        else
            fx = 0;
            fy = 0;
        end
    else
        % Conic well: F = k_att * d * (unit vector toward goal)
        fx = K_ATTR * PARABOLIC_LIMIT * currToGoal(1) / distanceToGoal;
        fy = K_ATTR * PARABOLIC_LIMIT * currToGoal(2) / distanceToGoal;
    end

    % worldCoords = groundTruth;
    % [minX, maxX, minY, maxY] = findWorldBoundaries(worldCoords);
    % if abs(currentPos(1) - minX) < 10
    %     fx = max(0, fx);
    % elseif abs(currentPos(1) - maxX) < 10
    %     fx = min(0, fx);
    % end
    % if abs(currentPos(2) - minY) < 10 
    %     fy = max(0, fy);
    % elseif abs(currentPos(2) - maxY) < 10
    %     fy = min(0, fy);
    % end
end

% Calculate repulsive force from all obstacles
function [fxTotal, fyTotal] = calculateRepulsiveForce(currentPos, obstacleList, goalPos)
    global K_REP REP_DIST
    fxTotal = 0;
    fyTotal = 0;

    % no repulsive forces if close to goal
    if norm(currentPos - goalPos) < 45
        return
    end
    
    for i = 1:size(obstacleList, 1)
        obstacle = obstacleList(i, :);
        
        currToObstacle = obstacle - currentPos;
        distanceToObstacle = norm(currToObstacle);

        if distanceToObstacle <= REP_DIST && distanceToObstacle > 0
            % Repulsive force: F = k_rep * (1/rho - 1/rho0) * (1/rho^2) * gradientRho
            rho = distanceToObstacle;
            gradientRho = -currToObstacle / distanceToObstacle;
            force = double(K_REP * (-0.01/rho + 1/REP_DIST) * 1/rho^2 * gradientRho);

            fxTotal = fxTotal + force(1);
            fyTotal = fyTotal + force(2);
        end
    end
end

% Plan path using gradient descent on the potential field
function path = planPathWithAPF(startPos, goalPos, obstacleList, depth)
    if nargin < 4
        depth = 1; % default depth for recursion
    end

    global groundTruth gtOffsets STEP_SIZE PATH_TOLERANCE

    worldCoords = groundTruth;
    maxX = max(worldCoords(:, 1));
    maxY = max(worldCoords(:, 2));
    minX = min(worldCoords(:, 1));
    minY = min(worldCoords(:, 2));

    path = startPos;
    currentPos = startPos;
    maxIterations = 1000;
    
    for iter = 1:maxIterations
        % calculate forces at current position
        [fxAtt, fyAtt] = calculateAttractiveForce(currentPos, goalPos);
        [fxRep, fyRep] = calculateRepulsiveForce(currentPos, obstacleList, goalPos);
        
        fxTotal = double(fxAtt) + double(fxRep);
        fyTotal = double(fyAtt) + double(fyRep);

        % get unit vector in force direction
        force = sqrt(fxTotal^2 + fyTotal^2);
        if force > 0
            fxTotal = fxTotal / force;
            fyTotal = fyTotal / force;
        else
            error('Zero force encountered, cannot proceed.');
        end
        
        nextPos = currentPos + min(STEP_SIZE, force) * [fxTotal, fyTotal];

        if nextPos(1) < minX || nextPos(1) > maxX || nextPos(2) < minY || nextPos(2) > maxY
            warning('Next position out of bounds, stopping path planning.');
            break; % stop if next position is out of bounds
        end
        
        % check if reached goal
        distanceToGoal = norm(nextPos - goalPos);
        if distanceToGoal < PATH_TOLERANCE
            path = [path; goalPos];
            break;
        end
        
        % Add to path and update current position
        path = [path; nextPos];
        currentPos = nextPos;
    end

    if norm(currentPos - goalPos) > 45
        if depth > 4
            warning('Path planning failed to reach the goal within tolerance after multiple attempts.');
            return; % exit if too many retries
        end
        worldCoords = groundTruth;
        worldCornerOffsets = gtOffsets;
        warning('Path planning did not reach the goal within tolerance.');
        newGoal = worldCoords(depth, :) + worldCornerOffsets(depth, :); % adjust goal position for retry
        path = planPathWithAPF(startPos, newGoal, obstacleList, depth + 1); % retry path planning
        path2 = planPathWithAPF(newGoal, goalPos, obstacleList, depth + 1); % plan path to original goal
        path = [path; path2]; % combine paths, avoiding
    end
end

% Display the potential field and planned path
function displayPotentialField(Img, worldCoords, X, Y, Fx, Fy)
    minX = min(worldCoords(:, 1));
    minY = min(worldCoords(:, 2));
    
    
    Fx = capForcesForDisplay(Fx, X);
    Fy = capForcesForDisplay(Fy, Y);

    [imageX, imageY] = transformWorldCoordsToWarpedImageCoords2(X, Y, minX, minY);

    figure(4);
    imshow(Img);
    hold on;
    quiver(imageX, imageY, Fx, Fy, 0.5, 'Color', 'y', 'LineWidth', 1);
    title('Artificial Potential Field');
    axis("equal");
    hold off;
end

% keep forces between Q1-1.5*IQR and Q3+1.5*IQR
function newFx = capForcesForDisplay(Fx, X) 
    Fx = Fx(:);

    Q1 = quantile(Fx, 0.25);
    Q3 = quantile(Fx, 0.75);

    IQR = Q3 - Q1;

    Fx(Fx < Q1 - 1.5 * IQR) = Q1 - 1.5 * IQR;
    Fx(Fx > Q3 + 1.5 * IQR) = Q3 + 1.5 * IQR;

    newFx = reshape(Fx, size(X));
end

function displayAPFPath(Img, worldCoords, path)
    minX = min(worldCoords(:, 1));
    minY = min(worldCoords(:, 2));

    imagePath = transformWorldCoordsToWarpedImageCoords(path, minX, minY);
    
    figure(5);
    imshow(Img);
    hold on;
    plot(imagePath(:, 1), imagePath(:, 2), 'Color', [0.7 0 1], 'LineWidth', 2);
    title('Trajectory from Start to Goal via APF');
    hold off;
end

function displayAllPartAandB(Img, worldCoords, X, Y, Fx, Fy, path)
    minX = min(worldCoords(:, 1));
    minY = min(worldCoords(:, 2));

    
    Fx = capForcesForDisplay(Fx, X);
    Fy = capForcesForDisplay(Fy, Y);

    [imageX, imageY] = transformWorldCoordsToWarpedImageCoords2(X, Y, minX, minY);
    imagePath = transformWorldCoordsToWarpedImageCoords(path, minX, minY);
    
    figure(6);
    imshow(Img);
    hold on;
    plot(imagePath(:, 1), imagePath(:, 2), 'Color', [0.7 0 1], 'LineWidth', 2);
    quiver(imageX, imageY, Fx, Fy, 1, 'Color', 'y', 'LineWidth', 1);
    title('All Part A and B Results');
    hold off;

    fprintf('PART B RESULTS:\n');
    fprintf('------------------------------\n');
    fprintf('Final path:\n');
    disp(path');
    fprintf('==============================\n');
end

% ====================================== %
% --------------- PART C --------------- %
% ====================================== %
function partC(rtde, vacuum, smallItemCoords, smallGoalCoords, ...
    smallYaw, smallItemGoalOrientation, path)

    global RUN_MODE HOME_POS ACC VEL BLEND MOVE_HEIGHT VACUUM_PAUSE JOINT_HOME

    % move to home position
    rtde.movej(HOME_POS, 'pose', ACC, VEL, 0, BLEND);
    waitToMove(rtde);

    % move above small item
    rtde.movej([smallItemCoords(1), smallItemCoords(2), MOVE_HEIGHT, 2.221, 2.221, 0], 'pose', ACC, VEL, 0, BLEND);

    %pause(2);
    waitToMove(rtde);

    % move down to pick up small item
    rtde.movel([smallItemCoords(1), smallItemCoords(2), PICKUP_HEIGHT(), 2.221, 2.221, 0], 'pose', ACC, VEL, 0, BLEND);

    %pause(2);
    waitToMove(rtde);

    % activate vacuum if using real robot
    if ~strcmp(RUN_MODE, 'sim')
        vacuum.grip();
    end

    % wait for vacuum to pick up the item
    pause(VACUUM_PAUSE);

    % move up with item
    rtde.movel([smallItemCoords(1), smallItemCoords(2), MOVE_HEIGHT, 2.221, 2.221, 0], 'pose', ACC, VEL, 0, BLEND);

    %pause(2);
    waitToMove(rtde);

    % follow the planned path
    for i = 1:size(path, 1)
        rtde.movel([path(i, 1), path(i, 2), MOVE_HEIGHT, 2.221, 2.221, 0], 'pose', ACC, VEL, 0, BLEND);
    end

    % move to goal position
    rtde.movel([smallGoalCoords(1), smallGoalCoords(2), MOVE_HEIGHT, 2.221, 2.221, 0], 'pose', ACC, VEL, 0, BLEND);

    %pause(2);
    waitToMove(rtde);

    % rotate based on small item orientation vs. goal orientation
    jointPos = rtde.targetJointPositions();
    angle = atan2(smallItemGoalOrientation(2), smallItemGoalOrientation(1)) - atan2(smallYaw(2), smallYaw(1));
    angle = adjustAngle(jointPos(:,6) - angle + pi);

    rtde.movej([jointPos(:,1), jointPos(:,2), jointPos(:,3), jointPos(:,4), jointPos(:,5), angle], 'joint', ACC, VEL, 0, BLEND);
    
    %pause(2);
    waitToMove(rtde);

    % deactivate vacuum if using real robot
    if ~strcmp(RUN_MODE, 'sim')
        vacuum.release();
    end
    
    % wait for vacuum to release the item
    pause(VACUUM_PAUSE);

    % return to home position
    rtde.movej(HOME_POS, 'pose', ACC, VEL, 0, BLEND);

    waitToMove(rtde);
end

function newAngle = adjustAngle(angle)
    % Adjust angle to be within [0, 2*pi]
    newAngle = mod(angle, 2 * pi);
    if newAngle < 0
        newAngle = newAngle + 2 * pi; % ensure angle is positive
    end
end

% ====================================== %
% --------------- PART D --------------- %
% ====================================== %
function partD(Img, worldCoords, rtde, vacuum, bigItemWorldCoords, bigItemGoalWorldCoords, ...
    bigItemOrientation, bigItemGoalOrientation, obstCoordsList)

    global RUN_MODE HOME_POS ACC VEL BLEND MOVE_HEIGHT VACUUM_PAUSE

    itemYaw = atan2(bigItemOrientation(2), bigItemOrientation(1)); % TODO: unused
    goalYaw = atan2(bigItemGoalOrientation(2), bigItemGoalOrientation(1));

    % plan path for bigItem
    path = planPathWithAPF(bigItemWorldCoords, bigItemGoalWorldCoords, obstCoordsList);

    displayAPFPath(Img, worldCoords, path);

    dirOnPath = diff(path, 1, 1);
    headings = atan2(dirOnPath(:,2), dirOnPath(:,1));
    headings = [headings(1); headings];

    disp(headings);
    disp(itemYaw);
    disp(goalYaw);

    % move to home position
    rtde.movej(HOME_POS, 'pose', ACC, VEL, 0, BLEND);

    waitToMove(rtde);

    % move above big item
    rtde.movej([bigItemWorldCoords(1), bigItemWorldCoords(2), MOVE_HEIGHT, 2.221, 2.221, 0], 'pose', ACC, VEL, 0, BLEND);

    waitToMove(rtde);

    % move down to pick up big item
    rtde.movel([bigItemWorldCoords(1), bigItemWorldCoords(2), PICKUP_HEIGHT(), 2.221, 2.221, 0], 'pose', ACC, VEL, 0, BLEND);

    waitToMove(rtde);

    % activate vacuum if using real robot
    if ~strcmp(RUN_MODE, 'sim')
        vacuum.grip();
    end

    % wait for vacuum to pick up the item
    pause(VACUUM_PAUSE);

    % move up with item
    rtde.movel([bigItemWorldCoords(1), bigItemWorldCoords(2), MOVE_HEIGHT, 2.221, 2.221, 0], 'pose', ACC, VEL, 0, BLEND);

    waitToMove(rtde);

    % follow the planned path
    for i = 1:size(path, 1)
        newYaw = mod(headings(i) - itemYaw, 2*pi); % find yaw we want to rotate to
        rvec = yawToRotAngles(newYaw, 2.221, 2.221, 0);
        rtde.movel([path(i, 1), path(i, 2), MOVE_HEIGHT, rvec(1), rvec(2), rvec(3)], 'pose', ACC, VEL, 0, BLEND);
    end 

    waitToMove(rtde);

    % move to goal position
    finalRvec = yawToRotAngles(mod(goalYaw - itemYaw, 2*pi), 2.221, 2.221, 0);
    rtde.movej([bigItemGoalWorldCoords(1), bigItemGoalWorldCoords(2), MOVE_HEIGHT, finalRvec(1), finalRvec(2), finalRvec(3)], 'pose', ACC, VEL, 0, BLEND);

    waitToMove(rtde);

    % deactivate vacuum if using real robot
    if ~strcmp(RUN_MODE, 'sim')
        vacuum.release();
    end

    % wait for vacuum to release the item
    pause(VACUUM_PAUSE);

    % return to home position
    rtde.movej(HOME_POS, 'pose', ACC, VEL, 0, BLEND);

    waitToMove(rtde);
end

function rvec = yawToRotAngles(deltaYaw, roll, pitch, yaw)
    rvec = [roll, pitch, yaw];
    ang = norm(rvec);

    if ang < 1e-9
        rHome = eye(3);
    else
        k = rvec / ang;
        rHome = angvec2r(ang, k);        
    end
    
    T = trotz(deltaYaw);
    rNew = T(1:3, 1:3) * rHome;

    % R -> axis–angle
    [ang2, k2] = tr2angvec(rNew);
    rvec = k2 * ang2;
end



% ====================================== %
% --------------- PART E --------------- %
% ====================================== %
function partE(rtde, vacuum, smallItemCoords, smallGoalCoords, smallYaw, smallItemGoalOrientation, path)

    global RUN_MODE HOME_POS JOINT_HOME ACC VEL BLEND MOVE_HEIGHT VACUUM_PAUSE
    % move to home position
    homeQ= deg2rad([0 -75 90 -105 -90 0]);
    rtde.movej(homeQ, 'joint', ACC, VEL, 0, BLEND);

    d = [0.1625, 0, 0, 0.1333, 0.0997, 0.0996];
    a = [0, -0.425, -0.3922, 0, 0, 0];
    al= [pi/2, 0, 0, pi/2, -pi/2, 0];
    
    L(1) = Link('d',d(1),'a',a(1),'alpha',al(1));
    L(2) = Link('d',d(2),'a',a(2),'alpha',al(2));
    L(3) = Link('d',d(3),'a',a(3),'alpha',al(3));
    L(4) = Link('d',d(4),'a',a(4),'alpha',al(4));
    L(5) = Link('d',d(5),'a',a(5),'alpha',al(5));
    L(6) = Link('d',d(6),'a',a(6),'alpha',al(6));
    
    ur = SerialLink(L, 'name', 'UR5e');

    function T = makeT_xyzyaw_mm(x_mm, y_mm, z_mm, yaw_rad)
        % Tool Z-down + yaw about world Z
        mm2m = 1e-3;
        T = transl(x_mm*mm2m, y_mm*mm2m, z_mm*mm2m) * trotz(yaw_rad) * trotx(pi);
    end

    function Tseq = buildTargetTransforms(path_mm, yaw_path_rad, Z_TRANSPORT_MM)
        N = size(path_mm,1);
        Tseq = repmat(eye(4), 1,1, N);     % 3D array of 4x4s
        for j = 1:N
            Tseq(:,:,j) = makeT_xyzyaw_mm(path_mm(j,1), path_mm(j,2), Z_TRANSPORT_MM, yaw_path_rad(j));
        end
    end

    yaw_const = 0;                                % radians
    yaw_path_rad = yaw_const*ones(size(path,1),1);
    Tseq = buildTargetTransforms(path, yaw_path_rad, MOVE_HEIGHT);   % MOVE_HEIGHT in mm
    T_above_pick = makeT_xyzyaw_mm(smallItemCoords(1), smallItemCoords(2), ...
                                   MOVE_HEIGHT, atan2(smallYaw(2),smallYaw(1)));
    T_pick       = makeT_xyzyaw_mm(smallItemCoords(1), smallItemCoords(2), ...
                                   PICKUP_HEIGHT(), atan2(smallYaw(2),smallYaw(1)));
    T_above_goal = makeT_xyzyaw_mm(smallGoalCoords(1), smallGoalCoords(2), ...
                                   MOVE_HEIGHT, atan2(smallItemGoalOrientation(2),smallItemGoalOrientation(1)));
    T_place      = makeT_xyzyaw_mm(smallGoalCoords(1), smallGoalCoords(2), ...
                                   PICKUP_HEIGHT(), atan2(smallItemGoalOrientation(2),smallItemGoalOrientation(1)));


    N  = size(Tseq,3);
    Q  = zeros(N,6);
    
    q_prev = rtde.targetJointPositions();
   

    waitToMove(rtde);


    % move above small item
    rtde.movej(ur.ikcon(T_above_pick, q_prev), 'joint', ACC, VEL, 0, BLEND);

    waitToMove(rtde);

    % move down to pick up small item
    q_prev = rtde.targetJointPositions();
    rtde.movej(ur.ikcon(T_pick, q_prev), 'joint', ACC, VEL, 0, BLEND);
    

    waitToMove(rtde);

    % activate vacuum if using real robot
    if ~strcmp(RUN_MODE, 'sim')
        vacuum.grip();
    end

    % wait for vacuum to pick up the item
    pause(VACUUM_PAUSE);
    
    % move up with item
    q_prev = rtde.targetJointPositions();
    rtde.movej(ur.ikcon(T_above_pick, q_prev), 'joint', ACC, VEL, 0, BLEND);

    waitToMove(rtde);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
    % follow the planned path
    q_prev = rtde.targetJointPositions();
    Q(1,:) = ur.ikcon(Tseq(:,:,1), q_prev);
    for i = 2:N
        Q(i,:) = ur.ikcon(Tseq(:,:,i), Q(i-1,:));  % seed with previous for smoothness
    end

    for i = 1:size(Q,1)
        rtde.movej(Q(i,:), 'joint', ACC, VEL, 0, BLEND);
    end

    % move to goal position
    q_prev = rtde.targetJointPositions();
    rtde.movej(ur.ikcon(T_above_goal, q_prev), 'joint', ACC, VEL, 0, BLEND);

    %pause(2);
    waitToMove(rtde);

    jointPos = rtde.targetJointPositions();
    % rotate based on small item orientation vs. goal orientation
    angle = atan2(smallItemGoalOrientation(2), smallItemGoalOrientation(1)) - atan2(smallYaw(2), smallYaw(1));
    angle = adjustAngle(jointPos(:,6) - angle + pi);

    % rtde.movej([jointPos(:,1), jointPos(:,2), jointPos(:,3), jointPos(:,4), jointPos(:,5), angle], 'joint', ACC, VEL, 0, BLEND);
    q_prev = rtde.targetJointPositions();
    placeQ = ur.ikcon(T_place, q_prev);
    placeQ(:,6) = angle;
    rtde.movej(placeQ, 'joint', ACC, VEL, 0, BLEND);
    waitToMove(rtde);

    % deactivate vacuum if using real robot
    if ~strcmp(RUN_MODE, 'sim')
        vacuum.release();
    end
    
    % wait for vacuum to release the item
    pause(VACUUM_PAUSE);

    % return to home position
    rtde.movej(homeQ, 'joint', ACC, VEL, 0, BLEND);
    

    waitToMove(rtde);
end


% ---------- helpers ----------

function [q_best, ok] = ik6s_or_num(ur, T, q_prev)
% Try all ikine6s branches, pick nearest to q_prev; fall back to numerical IK.
ok = false; q_best = [];
configs = {'lun','luf','ldn','ldf','run','ruf','rdn','rdf'};
cands = [];

for k = 1:numel(configs)
    try
        qk = ur.ikine6s(T, configs{k});
        if ~isempty(qk)
            if size(qk,1) > 1, qk = qk(1,:); end
            cands = [cands; qk];
        end
    catch
        % ignore failed branch
    end
end

if ~isempty(cands)
    [~,idx] = min(vecnorm(wrapToPi(cands - q_prev), 2, 2));
    q_best = cands(idx,:);
    ok = true; return;
end

% Fallback numerical IK (warm start)
try
    q_best = ur.ikine(T, q_prev, 'mask', [1 1 1 1 1 1], 'tol', 1e-6, 'ilimit', 1000, 'rlimit', 1000);
    ok = true;
catch
    ok = false; q_best = [];
end
end

function wait_rtde(rtde)
% Simple wait loop; replace with your existing waitForRtde if preferred
pause(0.2);
try
    while sum(abs(rtde.targetJointPositions() - rtde.actualJointPositions())) > 1e-2
        pause(0.05);
    end
catch
    pause(0.2);
end
end
