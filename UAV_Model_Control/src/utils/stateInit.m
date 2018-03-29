function quads = stateInit()
quads.ax(1) = 0;
quads.ay(1) = 0;
quads.az(1) = 0;

quads.vx(1) = 0;
quads.vy(1) = 0;
quads.vz(1) = 0;

quads.px(1) = 0;
quads.py(1) = 0;
quads.pz(1) = 0;

quads.R = zeros(3,3,1);
quads.R(:,:,1) = eye(3);


end