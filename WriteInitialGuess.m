function WriteInitialGuess(x, y, theta, xr, yr, xf, yf, v, a, phy, w, tf)
if isfile('initial_guess0.INIVAL'), delete('initial_guess0.INIVAL'); end
fid = fopen('initial_guess0.INIVAL', 'w');
for ii = 1 : length(x)
    fprintf(fid, 'let x[%g] := %.17g;\r\n', ii, x(ii));
    fprintf(fid, 'let y[%g] := %.17g;\r\n', ii, y(ii));
    fprintf(fid, 'let theta[%g] := %.17g;\r\n', ii, theta(ii));
    fprintf(fid, 'let v[%g] := %.17g;\r\n', ii, v(ii));
    fprintf(fid, 'let a[%g] := %.17g;\r\n', ii, a(ii));
    fprintf(fid, 'let phy[%g] := %.17g;\r\n', ii, phy(ii));
    fprintf(fid, 'let w[%g] := %.17g;\r\n', ii, w(ii));
    fprintf(fid, 'let xr[%g] := %.17g;\r\n', ii, xr(ii));
    fprintf(fid, 'let yr[%g] := %.17g;\r\n', ii, yr(ii));
    fprintf(fid, 'let xf[%g] := %.17g;\r\n', ii, xf(ii));
    fprintf(fid, 'let yf[%g] := %.17g;\r\n', ii, yf(ii));
end
fprintf(fid, 'let tf := %.17g;\r\n', tf);
fclose(fid);