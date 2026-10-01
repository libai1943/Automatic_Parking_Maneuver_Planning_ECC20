function WriteBoundaryValues()
global vehicle_TPBV_
if isfile('BV'), delete('BV'); end
fid = fopen('BV', 'w');
fprintf(fid, '1  %.17g\r\n', vehicle_TPBV_.x0);
fprintf(fid, '2  %.17g\r\n', vehicle_TPBV_.y0);
fprintf(fid, '3  %.17g\r\n', vehicle_TPBV_.theta0);
fprintf(fid, '4  %.17g\r\n', vehicle_TPBV_.xtf);
fprintf(fid, '5  %.17g\r\n', vehicle_TPBV_.ytf);
fprintf(fid, '6  %.17g\r\n', vehicle_TPBV_.thetatf);
fclose(fid);