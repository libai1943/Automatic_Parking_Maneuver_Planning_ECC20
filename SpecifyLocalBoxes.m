function [BVr, BVf, xr, yr, xf, yf] = SpecifyLocalBoxes(x, y, theta)
global vehicle_geometrics_
xr = x + vehicle_geometrics_.r2x .* cos(theta);
yr = y + vehicle_geometrics_.r2x .* sin(theta);
xf = x + vehicle_geometrics_.f2x .* cos(theta);
yf = y + vehicle_geometrics_.f2x .* sin(theta);
BVr = zeros(length(x),4); % xmin, xmax, ymin, ymax
BVf = BVr;

delete('CC');
fid = fopen('CC', 'w');
for ii = 1 : length(xr)
    x = xr(ii); y = yr(ii);
    lb = GetBoxVertexes(x, y);
    if (~any(lb))
        counter = 0;
        is_lb_nonzero = 0;
        while (~is_lb_nonzero)
            counter = counter + 1;
            for jj = 1 : 4
                switch jj
                    case 1
                        x_nudge = x + counter * 0.01;
                        y_nudge = y;
                    case 2
                        x_nudge = x - counter * 0.01;
                        y_nudge = y;
                    case 3
                        x_nudge = x;
                        y_nudge = y + counter * 0.01;
                    case 4
                        x_nudge = x;
                        y_nudge = y - counter * 0.01;
                end
                lb = GetBoxVertexes(x_nudge, y_nudge);
                if (any(lb))
                    is_lb_nonzero = 1;
                    x = x_nudge;
                    y = y_nudge;
                    break;
                end
            end
        end
    end
    BVr(ii,:) = [x - lb(2), x + lb(4), y - lb(3), y + lb(1)];
    xr(ii) = x; yr(ii) = y;
    
    x = xf(ii); y = yf(ii);
    lb = GetBoxVertexes(x, y);
    if (~any(lb))
        counter = 0;
        is_lb_nonzero = 0;
        while (~is_lb_nonzero)
            counter = counter + 1;
            for jj = 1 : 4
                switch jj
                    case 1
                        x_nudge = x + counter * 0.01;
                        y_nudge = y;
                    case 2
                        x_nudge = x - counter * 0.01;
                        y_nudge = y;
                    case 3
                        x_nudge = x;
                        y_nudge = y + counter * 0.01;
                    case 4
                        x_nudge = x;
                        y_nudge = y - counter * 0.01;
                end
                lb = GetBoxVertexes(x_nudge, y_nudge);
                if (any(lb))
                    is_lb_nonzero = 1;
                    x = x_nudge;
                    y = y_nudge;
                    break;
                end
            end
        end
    end
    BVf(ii,:) = [x - lb(2), x + lb(4), y - lb(3), y + lb(1)];
    xf(ii) = x; yf(ii) = y;
    
    fprintf(fid, '%g 1 %f \r\n', ii, BVr(ii,1));
    fprintf(fid, '%g 2 %f \r\n', ii, BVr(ii,2));
    fprintf(fid, '%g 3 %f \r\n', ii, BVr(ii,3));
    fprintf(fid, '%g 4 %f \r\n', ii, BVr(ii,4));
    fprintf(fid, '%g 5 %f \r\n', ii, BVf(ii,1));
    fprintf(fid, '%g 6 %f \r\n', ii, BVf(ii,2));
    fprintf(fid, '%g 7 %f \r\n', ii, BVf(ii,3));
    fprintf(fid, '%g 8 %f \r\n', ii, BVf(ii,4));
end
fclose(fid);
end

function lb = GetBoxVertexes(x,y)
global optimization_
unit_step = optimization_.unit_step;
max_step = optimization_.max_step;
% up left down right
lb = zeros(1,4);
is_completed = zeros(1,4);
while (sum(is_completed) < 4)
    for ind = 1 : 4
        if (is_completed(ind))
            continue;
        end
        test = lb;
        if (test(ind) + unit_step > max_step)
            is_completed(ind) = 1;
            continue;
        end
        test(ind) = test(ind) + unit_step;
        if (IsCurrentEnlargementValid(x, y, test, lb, ind))
            lb = test;
        else
            is_completed(ind) = 1;
        end
    end
end
end

function is_valid = IsCurrentEnlargementValid(x, y, test, lb, ind)
switch ind
    case 1
        A = [x - lb(2), y + lb(1)];
        B = [x + lb(4), y + lb(1)];
        EA = [x - test(2), y + test(1)];
        EB = [x + test(4), y + test(1)];
        V_check = [A; B; EB; EA];
    case 2
        A = [x - lb(2), y + lb(1)];
        D = [x - lb(2), y - lb(3)];
        EA = [x - test(2), y + test(1)];
        ED = [x - test(2), y - test(3)];
        V_check = [A; D; ED; EA];
    case 3
        C = [x + lb(4), y - lb(3)];
        D = [x - lb(2), y - lb(3)];
        EC = [x + test(4), y - test(3)];
        ED = [x - test(2), y - test(3)];
        V_check = [C; D; ED; EC];
    case 4
        B = [x + lb(4), y + lb(1)];
        C = [x + lb(4), y - lb(3)];
        EB = [x + test(4), y + test(1)];
        EC = [x + test(4), y - test(3)];
        V_check = [C; B; EB; EC];
    otherwise
        is_valid = 0;
        return;
end
x_min = min(V_check(:,1));
x_max = max(V_check(:,1));
y_min = min(V_check(:,2));
y_max = max(V_check(:,2));

global environment_scale_
if ((x_min < environment_scale_.environment_x_min)||(x_max > environment_scale_.environment_x_max)||...
        (y_min < environment_scale_.environment_y_min)||(y_max > environment_scale_.environment_y_max))
    is_valid = 0;
    return;
end

global hybrid_astar_
ind_x_min = ceil((x_min - environment_scale_.environment_x_min) / hybrid_astar_.resolution_x) + 1;
ind_y_min = ceil((y_min - environment_scale_.environment_y_min) / hybrid_astar_.resolution_y) + 1;
ind_x_max = ceil((x_max - environment_scale_.environment_x_min) / hybrid_astar_.resolution_x) + 1;
ind_y_max = ceil((y_max - environment_scale_.environment_y_min) / hybrid_astar_.resolution_y) + 1;
ind_x = ind_x_min : ind_x_max;
ind_y = ind_y_min : ind_y_max;

global costmap_
if (any(any(costmap_(ind_x, ind_y))))
    is_valid = 0;
else
    is_valid = 1;
end
end