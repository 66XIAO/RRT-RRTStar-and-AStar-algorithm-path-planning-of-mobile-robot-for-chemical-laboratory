function smoothed_path = smooth_path(original_path, obstacles, robot_radius)
%% 路径平滑处理
% 使用圆弧平滑转角，使路径更加流畅
% 输入：
%   original_path - 原始路径点
%   obstacles - 障碍物矩阵
%   robot_radius - 机器人半径
% 输出：
%   smoothed_path - 平滑后的路径

    if size(original_path, 1) < 3
        smoothed_path = original_path;
        return;
    end
    
    % 首先使用Ramer-Douglas-Peucker算法简化路径
    simplified_path = rdp_simplify(original_path, 5);
    
    % 然后使用贝塞尔曲线或圆弧进行平滑
    smoothed_path = [];
    
    for i = 1:size(simplified_path, 1)
        if i == 1 || i == size(simplified_path, 1)
            % 起点和终点直接保留
            smoothed_path = [smoothed_path; simplified_path(i, :)];
        else
            % 中间点使用圆弧平滑
            p1 = simplified_path(i-1, :);
            p2 = simplified_path(i, :);
            p3 = simplified_path(i+1, :);
            
            % 计算圆弧插值点
            arc_points = create_arc_points(p1, p2, p3, 5);
            
            % 验证圆弧点是否有效
            valid_arc = true;
            for j = 1:size(arc_points, 1)
                if ~is_valid_point(arc_points(j, :), obstacles, robot_radius)
                    valid_arc = false;
                    break;
                end
            end
            
            if valid_arc
                % 添加圆弧点（排除第一个点，避免重复）
                smoothed_path = [smoothed_path; arc_points(2:end, :)];
            else
                % 如果圆弧无效，直接添加原始点
                smoothed_path = [smoothed_path; p2];
            end
        end
    end
    
    % 最后进行路径优化（移除冗余点）
    smoothed_path = optimize_path(smoothed_path, obstacles, robot_radius);
    
end

%% Ramer-Douglas-Peucker算法简化路径
function simplified = rdp_simplify(points, epsilon)
    if size(points, 1) <= 2
        simplified = points;
        return;
    end
    
    % 找到距离起点-终点连线最远的点
    start_point = points(1, :);
    end_point = points(end, :);
    
    max_dist = 0;
    max_index = 1;
    
    for i = 2:size(points, 1) - 1
        dist = point_to_line_distance(points(i, :), start_point, end_point);
        if dist > max_dist
            max_dist = dist;
            max_index = i;
        end
    end
    
    % 如果最大距离大于阈值，递归处理
    if max_dist > epsilon
        % 递归处理前半部分
        left_simplified = rdp_simplify(points(1:max_index, :), epsilon);
        % 递归处理后半部分
        right_simplified = rdp_simplify(points(max_index:end, :), epsilon);
        % 合并结果
        simplified = [left_simplified(1:end-1, :); right_simplified];
    else
        simplified = [start_point; end_point];
    end
end

%% 计算点到线段的距离
function dist = point_to_line_distance(point, line_start, line_end)
    % 向量计算
    line_vec = line_end - line_start;
    point_vec = point - line_start;
    
    % 投影长度
    line_len_sq = sum(line_vec.^2);
    if line_len_sq == 0
        dist = norm(point - line_start);
        return;
    end
    
    t = max(0, min(1, sum(point_vec .* line_vec) / line_len_sq));
    projection = line_start + t * line_vec;
    dist = norm(point - projection);
end

%% 创建圆弧插值点
function arc_points = create_arc_points(p1, p2, p3, num_points)
    % 计算转角处的圆弧
    % p1: 前一个点, p2: 转角点, p3: 后一个点
    
    % 计算方向向量
    v1 = p1 - p2;
    v2 = p3 - p2;
    
    % 归一化
    v1 = v1 / norm(v1);
    v2 = v2 / norm(v2);
    
    % 计算圆弧控制点
    % 使用二次贝塞尔曲线创建平滑转角
    arc_points = [];
    
    for t = linspace(0, 1, num_points)
        % 二次贝塞尔曲线
        % B(t) = (1-t)^2 * P1 + 2(1-t)t * P2 + t^2 * P3
        % 这里我们使用控制点来创建平滑曲线
        
        % 计算控制点（在转角处稍微向内收缩）
        control_point = p2 + (v1 + v2) * 3;  % 控制点偏移量
        
        % 使用德卡斯特里奥算法
        q0 = p1 + t * (control_point - p1);
        q1 = control_point + t * (p3 - control_point);
        point = q0 + t * (q1 - q0);
        
        arc_points = [arc_points; point];
    end
end

%% 检查点是否有效
function valid = is_valid_point(point, obstacles, robot_radius)
    valid = true;
    
    % 检查边界
    if point(1) < robot_radius || point(1) > 1000 - robot_radius || ...
       point(2) < robot_radius || point(2) > 800 - robot_radius
        valid = false;
        return;
    end
    
    % 检查障碍物碰撞
    for i = 1:size(obstacles, 1)
        x = obstacles(i, 1) - robot_radius;
        y = obstacles(i, 2) - robot_radius;
        w = obstacles(i, 3) + 2 * robot_radius;
        h = obstacles(i, 4) + 2 * robot_radius;
        
        if point(1) >= x && point(1) <= x + w && ...
           point(2) >= y && point(2) <= y + h
            valid = false;
            return;
        end
    end
end

%% 优化路径（移除冗余点）
function optimized = optimize_path(path, obstacles, robot_radius)
    if size(path, 1) <= 2
        optimized = path;
        return;
    end
    
    optimized = path(1, :);  % 保留起点
    i = 1;
    
    while i < size(path, 1)
        % 尝试找到可以直接到达的最远点
        j = size(path, 1);
        while j > i
            if is_line_valid(path(i, :), path(j, :), obstacles, robot_radius)
                optimized = [optimized; path(j, :)];
                i = j;
                break;
            end
            j = j - 1;
        end
        
        if j == i
            i = i + 1;
            if i <= size(path, 1)
                optimized = [optimized; path(i, :)];
            end
        end
    end
end

%% 检查线段是否有效
function valid = is_line_valid(p1, p2, obstacles, robot_radius)
    valid = true;
    
    % 在线段上采样多个点
    num_samples = max(ceil(norm(p2 - p1) / 5), 5);
    
    for t = linspace(0, 1, num_samples)
        point = p1 + t * (p2 - p1);
        if ~is_valid_point(point, obstacles, robot_radius)
            valid = false;
            return;
        end
    end
end
