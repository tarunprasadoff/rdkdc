g = [ [ROTX(pi/2),[3;2;0]];[0,0,0,1]];

% Computes the twist vector from a transformation matrix
R = g(1:3, 1:3);
p = g(1:3, 4);
w = zeros(3, 1); % Initialize angular velocity

disp( abs(sum( sum( abs( ( R ) - eye(3) ) ))) < 0.001 )