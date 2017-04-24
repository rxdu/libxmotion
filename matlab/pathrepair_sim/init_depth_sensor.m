function sensor_model = init_depth_sensor(sensor_info)
    sensor_model.sensor_info = sensor_info;
    
    % calculate horizontal optical plane boundary points 
    beta = sensor_info.sensor_angle;
    alpha1 = sensor_info.sensor_angle + sensor_info.aov_h/2;
    alpha2 = sensor_info.sensor_angle - sensor_info.aov_h/2;
    left_limit = sensor_info.sensor_pos(1:2) + [sin(alpha1) cos(alpha1)] * sensor_info.range;
    right_limit = sensor_info.sensor_pos(1:2) + [sin(alpha2) cos(alpha2)] * sensor_info.range;
    ll_idx = get_voxel_free_index([left_limit sensor_info.sensor_pos(3)]);
    rl_idx = get_voxel_free_index([right_limit sensor_info.sensor_pos(3)]);

    pp_half_layer_num = floor(sensor_info.range*tan(sensor_info.aov_v/2));
    llu_idx = [ll_idx(1) ll_idx(2) ll_idx(3) + pp_half_layer_num];
    rlu_idx = [rl_idx(1) rl_idx(2) rl_idx(3) + pp_half_layer_num];
    lll_idx = [ll_idx(1) ll_idx(2) ll_idx(3) - pp_half_layer_num];
    rll_idx = [rl_idx(1) rl_idx(2) rl_idx(3) - pp_half_layer_num];

    % calculate boundary points
    ll_pos = calc_center_pos([ll_idx(1),ll_idx(2), ll_idx(3)]);
    rl_pos = calc_center_pos([rl_idx(1),rl_idx(2), rl_idx(3)]);

    llu_pos = calc_center_pos([llu_idx(1),llu_idx(2), llu_idx(3)]);
    rlu_pos = calc_center_pos([rlu_idx(1),rlu_idx(2), rlu_idx(3)]);

    lll_pos = calc_center_pos([lll_idx(1),lll_idx(2), lll_idx(3)]);
    rll_pos = calc_center_pos([rll_idx(1),rll_idx(2), rll_idx(3)]);

    sensor_model.projection_plane_points = [ll_pos'; rl_pos'; llu_pos'; rlu_pos'; lll_pos'; rll_pos'];
end