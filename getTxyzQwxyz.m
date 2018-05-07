function measurements = getTxyzQwxyz(var, idx)
measurements = [ ...
    var.pose_pose_position_x(idx), ...
    var.pose_pose_position_y(idx), ...
    var.pose_pose_position_z(idx), ...
    var.pose_pose_orientation_w(idx), ...
    var.pose_pose_orientation_x(idx), ...
    var.pose_pose_orientation_y(idx), ...
    var.pose_pose_orientation_z(idx)];
end
