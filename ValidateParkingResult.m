function report=ValidateParkingResult(result)
%VALIDATEPARKINGRESULT Independently check the original backward-Euler NLP.
global vehicle_TPBV_ vehicle_kinematics_ optimization_
r=result;dt=r.duration/optimization_.Nfe;
residual=[diff(r.x)-dt*r.v(2:end).*cos(r.theta(2:end)); ...
    diff(r.y)-dt*r.v(2:end).*sin(r.theta(2:end)); ...
    diff(r.theta)-dt*r.v(2:end).*tan(r.phi(2:end))/2.8; ...
    diff(r.v)-dt*r.a(2:end);diff(r.phi)-dt*r.omega(2:end)];
report.dynamics_residual=max(abs(residual));
boundary=[r.x(1)-vehicle_TPBV_.x0;r.y(1)-vehicle_TPBV_.y0; ...
    sin(r.theta(1))-sin(vehicle_TPBV_.theta0);cos(r.theta(1))-cos(vehicle_TPBV_.theta0); ...
    r.x(end)-vehicle_TPBV_.xtf;r.y(end)-vehicle_TPBV_.ytf; ...
    sin(r.theta(end))-sin(vehicle_TPBV_.thetatf);cos(r.theta(end))-cos(vehicle_TPBV_.thetatf); ...
    r.v([1,end]);r.a([1,end]);r.phi([1,end]);r.omega([1,end])];
report.boundary_residual=max(abs(boundary));
kin=vehicle_kinematics_;
report.limit_violation=max([0;abs(r.v)-kin.vehicle_v_max;abs(r.a)-kin.vehicle_a_max; ...
    abs(r.phi)-kin.vehicle_phy_max;abs(r.omega)-kin.vehicle_w_max]);
report.corridor_violation=0;
for part=1:2
    if part==1,offset=.2432;bounds=r.rear_boxes;else,offset=2.5877;bounds=r.front_boxes;end
    x=r.x+offset*cos(r.theta);y=r.y+offset*sin(r.theta);
    report.corridor_violation=max([report.corridor_violation;bounds(:,1)-x;x-bounds(:,2);bounds(:,3)-y;y-bounds(:,4)]);
end
report.passed=max([report.dynamics_residual,report.boundary_residual,report.limit_violation,report.corridor_violation])<1e-5;
disp(report);
end
