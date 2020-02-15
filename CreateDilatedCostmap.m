function costmap = CreateDilatedCostmap()
global environment_scale_ hybrid_astar_ vehicle_geometrics_
xmin = environment_scale_.environment_x_min;
ymin = environment_scale_.environment_y_min;
resolution_x = hybrid_astar_.resolution_x;
resolution_y = hybrid_astar_.resolution_y;
costmap = zeros(hybrid_astar_.num_nodes_x, hybrid_astar_.num_nodes_y);
global obstacle_vertexes_

for ii = 1 : size(obstacle_vertexes_,2)
    vx = obstacle_vertexes_{ii}.x;
    vy = obstacle_vertexes_{ii}.y;
    x_lb = min(vx); x_ub = max(vx); y_lb = min(vy); y_ub = max(vy);
    [Nmin_x,Nmin_y] = ConvertXYToIndex(x_lb,y_lb);
    [Nmax_x,Nmax_y] = ConvertXYToIndex(x_ub,y_ub);
    for jj = Nmin_x : Nmax_x
        for kk = Nmin_y : Nmax_y
            if (costmap(jj,kk) == 1)
                continue;
            end
            cur_x = xmin + (jj - 1) * resolution_x;
            cur_y = ymin + (kk - 1) * resolution_y;
            if (inpolygon(cur_x, cur_y, obstacle_vertexes_{ii}.x, obstacle_vertexes_{ii}.y) == 1)
                costmap(jj,kk) = 1;
            end
        end
    end
end
length_unit = 0.5 * (resolution_x + resolution_y);
basic_elem = strel('disk', 1 + ceil(vehicle_geometrics_.radius / length_unit));
costmap = imdilate(costmap, basic_elem);
end

function [ind1,ind2] = ConvertXYToIndex(x,y)
global hybrid_astar_ environment_scale_
ind1 = ceil((x - environment_scale_.environment_x_min) / hybrid_astar_.resolution_x) + 1;
ind2 = ceil((y - environment_scale_.environment_y_min) / hybrid_astar_.resolution_y) + 1;
if ((ind1 <= hybrid_astar_.num_nodes_x)&&(ind1 >= 1)&&(ind2 <= hybrid_astar_.num_nodes_y)&&(ind2 >= 1))
    return;
end
if (ind1 > hybrid_astar_.num_nodes_x)
    ind1 = hybrid_astar_.num_nodes_x;
elseif (ind1 < 1)
    ind1 = 1;
end
if (ind2 > hybrid_astar_.num_nodes_y)
    ind2 = hybrid_astar_.num_nodes_y;
elseif (ind2 < 1)
    ind2 = 1;
end
end