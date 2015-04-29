% function to obtain depth of interest points

function P = get3dPoint(dMap,loc)

 %Interest Point Location
 u = round(loc(2)); % col
 v = round(loc(1)); % row
 d = double(dMap(u,v));
 
 % Camera Parameters
 f = 525.0;   % Focal length
 cX = 319.5;  % Center X
 cY = 239.5;  % Center Y
 sf = 5000.0; % Scaling Factor
 
 % Compute 3D point
 Z = d/sf;
 X =  ((u-cX)*Z)/f;
 Y =  ((v-cY)*Z)/f;
 
 P = [X,Y,Z];

end