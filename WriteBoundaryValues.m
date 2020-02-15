function WriteBoundaryValues()
global vehicle_TPBV_
delete('BV');
fid = fopen('BV', 'w');
fprintf(fid, '1  %f\r\n', vehicle_TPBV_.x0);
fprintf(fid, '2  %f\r\n', vehicle_TPBV_.y0);
fprintf(fid, '3  %f\r\n', vehicle_TPBV_.theta0);
fprintf(fid, '4  %f\r\n', vehicle_TPBV_.xtf);
fprintf(fid, '5  %f\r\n', vehicle_TPBV_.ytf);
fprintf(fid, '6  %f\r\n', vehicle_TPBV_.thetatf);
fclose(fid);