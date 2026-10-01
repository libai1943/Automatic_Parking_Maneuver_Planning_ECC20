function PlotParkingResult(result)
%PLOTPARKINGRESULT Static trajectory/footprints and state/control profiles.
global obstacle_vertexes_ vehicle_geometrics_
figure('Color','w','Name','ECC 2020 parking trajectory');hold on;axis equal;box on;
for obstacle=obstacle_vertexes_
    patch(obstacle{1}.x,obstacle{1}.y,[.7 .7 .7],'EdgeColor','none','HandleVisibility','off');
end
g=vehicle_geometrics_;local=[g.vehicle_wheelbase+g.vehicle_front_hang,g.vehicle_width/2; ...
    g.vehicle_wheelbase+g.vehicle_front_hang,-g.vehicle_width/2; ...
    -g.vehicle_rear_hang,-g.vehicle_width/2;-g.vehicle_rear_hang,g.vehicle_width/2];
for index=unique(round(linspace(1,numel(result.x),35)))
    theta=result.theta(index);rotation=[cos(theta),-sin(theta);sin(theta),cos(theta)];
    body=local*rotation.'+[result.x(index),result.y(index)];
    patch(body(:,1),body(:,2),[0 .5 .75],'FaceAlpha',.08,'EdgeColor',[0 .5 .75],'HandleVisibility','off');
end
plot(result.coarse_path.x,result.coarse_path.y,'--','Color',[.85 .45 .1],'LineWidth',1.3);
plot(result.x,result.y,'k','LineWidth',1.8);
legend('Hybrid A*','Optimized trajectory','Location','best');xlabel('x (m)');ylabel('y (m)');
title(sprintf('Safe Travel Corridors | T = %.2f s',result.duration));
figure('Color','w','Name','ECC 2020 states and controls');t=(0:numel(result.x)-1)*result.duration/numel(result.x);
names={'v','theta','phi','a','omega'};labels={'Speed (m/s)','Heading (rad)','Steering (rad)','Acceleration (m/s^2)','Steering rate (rad/s)'};
for index=1:numel(names),subplot(2,3,index);plot(t,result.(names{index}),'LineWidth',1.5);grid on;xlabel('Time (s)');ylabel(labels{index});end
end
