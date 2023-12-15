function EI = FINV(E)

    R = E(1:3,1:3);
    t = E(1:3,4);
    
    if ( abs(det(R) - 1) < 0.001 ) && ( abs(sum( sum( abs( ( R * R' ) - eye(3) ) ))) < 0.001 ) && ( all( E(4,:) == [0,0,0,1] ) )

        EI = [
          [ R', -R'*t ];
          [0,0,0,1]
        ];

    else

        disp("E does not belong to SE(3)");
        EI = zeros(4,4);

    end            

end