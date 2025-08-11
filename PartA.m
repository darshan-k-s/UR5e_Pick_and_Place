function out = run_partA(imgFile)
% MTRN4230 Project 2 — Part A (geometry goal + console poses)
% Detect tags, resolve duplicate ID=0, warp to robot mm,
% compute goal slot centre from goal tag geometry,
% and print poses for small item, goal, and obstacles.

assert(exist(imgFile,'file')==2, 'Image not found: %s', imgFile);
Iraw = imread(imgFile);

%% ---------------- CONFIG ----------------
ID_TABLE      = [0 1 2 3];
ID_SMALL_ITEM = [5];
ID_GOAL       = [7];
ID_OBSTACLES  = [4];

GT = containers.Map('KeyType','double','ValueType','any');
GT(0) = [-990,   60];
GT(1) = [-230,   60];
GT(2) = [-990, -520];
GT(3) = [-230, -520];

GOAL_W = 90;
ideal_offset_tag_mm = [-GOAL_W; 0];  % kept as per your working setup
FINE_TUNE_mm        = [0; 0];

%% ---------------- 1) Detect all markers ----------------
[ids, locs] = readArucoMarker(Iraw);     % locs: 4x2xN (px)
ids = double(ids(:));
N   = numel(ids);
assert(N>0, 'No ArUco markers detected.');

% Marker centres (Nx2)
centers_px = squeeze(mean(locs,1)).';
if size(centers_px,2)~=2, centers_px = centers_px.'; end  % guard N==1

%% ---------------- 2) Pick table markers ----------------
imgPts   = nan(4,2);   % rows -> IDs [0 1 2 3]
worldPts = nan(4,2);

% IDs 1,2,3 (if duplicates, pick by vertical placement)
for t = [1 2 3]
    idxs = find(ids==t);
    assert(~isempty(idxs), 'Table marker ID %d not detected.', t);
    if numel(idxs) > 1
        v = centers_px(idxs,2);
        if t==1, [~,k] = min(v); else, [~,k] = max(v); end
        idxs = idxs(k);
    end
    imgPts(t+1,:)   = centers_px(idxs,:);
    worldPts(t+1,:) = GT(t);
end

% ID 0 — choose candidate that minimizes reprojection error
idx0s = find(ids==0);
assert(~isempty(idx0s), 'Table marker ID 0 not detected.');
bestErr = inf; bestImg0 = [];
for j = 1:numel(idx0s)
    img0 = centers_px(idx0s(j),:);
    imgTry   = imgPts;   imgTry(1,:)   = img0;
    worldTry = worldPts; worldTry(1,:) = GT(0);
    try
        H = fitgeotrans(imgTry, worldTry, 'projective');
        proj = transformPointsForward(H, imgTry);
        err  = mean(sum((proj - worldTry).^2, 2));
        if err < bestErr, bestErr = err; bestImg0 = img0; end
    catch
        % skip ill-conditioned candidate
    end
end
assert(~isempty(bestImg0), 'Could not disambiguate ID=0.');
imgPts(1,:)   = bestImg0;
worldPts(1,:) = GT(0);

%% ---------------- 3) Homography & warp ----------------
tform = fitgeotrans(imgPts, worldPts, 'projective');
xlims = [min(worldPts(:,1)) max(worldPts(:,1))];
ylims = [min(worldPts(:,2)) max(worldPts(:,2))];
outW  = round(diff(xlims));
outH  = round(diff(ylims));
RA    = imref2d([outH outW], xlims, ylims);
Iwarp = imwarp(Iraw, tform, 'OutputView', RA, 'FillValues', 220);

all_xy_mm = transformPointsForward(tform, centers_px);

%% ---------------- 4) Goal from tag pose + geometry ----------------
goal_xy_mm = [];
goal_tag_pose = [];
isGoalTag = ismember(ids, ID_GOAL);
if any(isGoalTag)
    k = find(isGoalTag,1,'first');
    ctr_px = centers_px(k,:);
    crn_px = squeeze(locs(:,:,k));

    ctr_mm = transformPointsForward(tform, ctr_px);
    crn_mm = transformPointsForward(tform, crn_px);

    % TL->TR vector (corner order TL,TR,BR,BL)
    xvec = crn_mm(2,:) - crn_mm(1,:); xvec = xvec / norm(xvec);
    yvec = [-xvec(2), xvec(1)];
    Rtag = [xvec(:) yvec(:)];

    slot_offset_tag = ideal_offset_tag_mm + FINE_TUNE_mm;
    goal_xy_mm = ctr_mm + (Rtag * slot_offset_tag).';

    yaw_deg = wrapTo180(rad2deg(atan2(xvec(2), xvec(1))));
    goal_tag_pose = [ctr_mm(:).', yaw_deg];
end

%% ---------------- 5) Classify ----------------
isTable = ismember(ids, ID_TABLE);
isSmall = ismember(ids, ID_SMALL_ITEM);
isObs   = ismember(ids, ID_OBSTACLES);

small_xy_mm = all_xy_mm(isSmall,:);
obs_xy_mm   = all_xy_mm(isObs,:);

%% ---------------- 6) Plot ----------------
figure('Name','Part A — Warped & Labeled','Color','w');
imshow(Iwarp, RA); hold on; axis on; axis xy;
xlabel('X (mm)'); ylabel('Y (mm)');
title('Part A: Top-down (robot frame), cropped to field');
if ~isempty(obs_xy_mm),   plot(obs_xy_mm(:,1),   obs_xy_mm(:,2),   'r.', 'MarkerSize',28); end
if ~isempty(goal_xy_mm),  plot(goal_xy_mm(1),    goal_xy_mm(2),    'g.', 'MarkerSize',28); end
if ~isempty(small_xy_mm), plot(small_xy_mm(:,1), small_xy_mm(:,2), 'b.', 'MarkerSize',28); end
plot(all_xy_mm(isTable,1), all_xy_mm(isTable,2), '.', 'Color',[.4 .4 .4], 'MarkerSize',18);
% legend({'Obstacle(s) (red)','Goal (green)','Small (blue)'}, 'Location','best');

%% ---------------- 7) Print poses ----------------
fprintf('\n==== Poses in robot base frame (mm, deg) ====\n');
if ~isempty(small_xy_mm)
    kS = find(isSmall,1,'first');
    yawS = tagYawDegFromCorners(tform, squeeze(locs(:,:,kS)));
    fprintf('Small item:  X = %+7.1f  Y = %+7.1f   Yaw = %+6.1f°\n', ...
        small_xy_mm(1,1), small_xy_mm(1,2), yawS);
else
    fprintf('Small item:  NOT FOUND\n');
end
if ~isempty(goal_xy_mm)
    fprintf('Goal slot :  X = %+7.1f  Y = %+7.1f\n', goal_xy_mm(1), goal_xy_mm(2));
end
if ~isempty(goal_tag_pose)
    fprintf('Goal tag  :  X = %+7.1f  Y = %+7.1f   Yaw = %+6.1f°\n', ...
        goal_tag_pose(1), goal_tag_pose(2), goal_tag_pose(3));
end
if ~isempty(obs_xy_mm)
    obsIDs = ids(isObs);
    idxObs = find(isObs);
    for i = 1:size(obs_xy_mm,1)
        yawO = tagYawDegFromCorners(tform, squeeze(locs(:,:,idxObs(i))));
        fprintf('Obstacle  :  X = %+7.1f  Y = %+7.1f   Yaw = %+6.1f°\n', ...
            obs_xy_mm(i,1), obs_xy_mm(i,2), yawO);
    end
else
    fprintf('Obstacle  :  NONE\n');
end
if ~isempty(small_xy_mm) && ~isempty(goal_xy_mm)
    d = hypot(small_xy_mm(1,1)-goal_xy_mm(1), small_xy_mm(1,2)-goal_xy_mm(2));
    fprintf('Distance small→goal: %.1f mm\n', d);
end
fprintf('============================================\n\n');

%% ---------------- 8) Outputs ----------------
out.tform_img2robot = tform;
out.field_xlim_mm   = xlims;
out.field_ylim_mm   = ylims;
out.all_ids         = ids(:);
out.all_xy_mm       = all_xy_mm;
out.small_xy_mm     = small_xy_mm;
out.goal_xy_mm      = goal_xy_mm;
out.obs_xy_mm       = obs_xy_mm;
end

function yaw_deg = tagYawDegFromCorners(tform, corners_px4x2)
% Assumes corner order TL, TR, BR, BL from readArucoMarker
crn = transformPointsForward(tform, corners_px4x2);
xvec = crn(2,:) - crn(1,:);  % TL->TR
yaw_deg = wrapTo180(rad2deg(atan2(xvec(2), xvec(1))));
end


run_partA("./example_1.jpg");