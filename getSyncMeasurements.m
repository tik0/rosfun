function [measure, Ids] = getSyncMeasurements(marker_odom_cam, camerainfo_cam)

%% Preallocation
num_cam = numel(marker_odom_cam);
num_edges = int64((num_cam*(num_cam-1))/2);
camerainfo_cam_time = cell(size(marker_odom_cam));
marker_odom_cam_time = cell(size(marker_odom_cam));
Ids = zeros(num_edges,2);
measure = repmat(struct('m1',nan, 'm2', nan), num_edges, 1);

%% Rewrite the sequence numbers in the trackings
for idx = 1 : num_cam
    % Get timestamps from the camerainfo and odometry
    camerainfo_cam_time{idx} = header2time(camerainfo_cam{idx});
    marker_odom_cam_time{idx} = header2time(marker_odom_cam{idx});
    % overwrite the sequence numbers of odom with the consecutive
    % framenumber of camerainfo
    marker_odom_cam{idx}.header_seq = ...
    getCameraSequence(marker_odom_cam_time{idx}, camerainfo_cam_time{idx}, ...
        camerainfo_cam{idx}.header_seq) - camerainfo_cam{idx}.header_seq(1);
end

%% Compare the sequence numbers to find matches
id_pair = 1;
for m1 = 1 : num_cam-1
    for m2 = m1+1 : num_cam
        [measure(id_pair).m1, measure(id_pair).m2] = syncMeasurements(marker_odom_cam{m1}, marker_odom_cam{m2});
        Ids(id_pair,:) = [m1, m2];
        id_pair = id_pair + 1;
    end
end
end

function camerainfo_seq_new = getCameraSequence(marker_time, camerainfo_time, camerainfo_seq) 
camerainfo_seq_new = int64(zeros(size(marker_time)));
for idx_marker = 1 : numel(marker_time)
    
    idx_camerainfo = find(camerainfo_time == marker_time(idx_marker));
    if ~any(idx_camerainfo)
        error("Missing frame for track");
    end
    camerainfo_seq_new(idx_marker) = int64(camerainfo_seq(idx_camerainfo));
end
end

function [m1_sync, m2_sync] = syncMeasurements(m1, m2)
%syncMeasurements Get the measurements which can be seen from m1 and m2

max_measurements = max(numel(m1.header_seq), numel(m2.header_seq));
matchIdx.m1 = zeros(max_measurements,1);
matchIdx.m2 = zeros(max_measurements,1);
idxMatch = 1;
for idx = 1 : numel(m1.header_seq)
    matchIdx_tmp = find(m2.header_seq == m1.header_seq(idx));
    if any(matchIdx_tmp)
        matchIdx.m1(idxMatch) = idx;
        matchIdx.m2(idxMatch) = matchIdx_tmp;
        idxMatch = idxMatch + 1;
    end
end
matchIdx.m1 = matchIdx.m1(1:idxMatch-1);
matchIdx.m2 = matchIdx.m2(1:idxMatch-1);

m1_sync = getTxyzQwxyz(m1, matchIdx.m1);
m2_sync = getTxyzQwxyz(m2, matchIdx.m2);
end



