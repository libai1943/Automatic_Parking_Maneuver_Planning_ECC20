function WriteInitialGuess(x, y, theta, xr, yr, xf, yf, v, a, phy, w, tf)
delete('initial_guess0.INIVAL');
fid = fopen('initial_guess0.INIVAL', 'w');
for ii = 1 : length(x)
    fprintf(fid, 'let x[%g] := %f;\r\n', ii, x(ii));
    fprintf(fid, 'let y[%g] := %f;\r\n', ii, y(ii));
    fprintf(fid, 'let theta[%g] := %f;\r\n', ii, theta(ii));
    fprintf(fid, 'let v[%g] := %f;\r\n', ii, v(ii));
    fprintf(fid, 'let a[%g] := %f;\r\n', ii, a(ii));
    fprintf(fid, 'let phy[%g] := %f;\r\n', ii, phy(ii));
    fprintf(fid, 'let w[%g] := %f;\r\n', ii, w(ii));
    fprintf(fid, 'let xr[%g] := %f;\r\n', ii, xr(ii));
    fprintf(fid, 'let yr[%g] := %f;\r\n', ii, yr(ii));
    fprintf(fid, 'let xf[%g] := %f;\r\n', ii, xf(ii));
    fprintf(fid, 'let yf[%g] := %f;\r\n', ii, yf(ii));
end
fprintf(fid, 'let tf := %f;\r\n', tf);
fclose(fid);