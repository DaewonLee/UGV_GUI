function xy = llToMeters(lon, lat)
% xy = llToMeters(lon, lat)
    originShift = 2 * pi * 6378137 / 2.0; % 20037508.342789244
    x = lon * originShift / 180;
    y = log(tan((90 + lat) * pi / 360 )) / (pi / 180);
    y = y * originShift / 180;
    xy = [x,y];

end