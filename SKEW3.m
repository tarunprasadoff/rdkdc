function V = SKEW3(v)
    
    if ~all(size(v) == [3,1])

        disp("Incorrect Size for Input Vector");
        V = zeros(3,3);
    
    else
    
        V = [
          [0, -v(3), v(2)];
          [v(3), 0, -v(1)];
          [-v(2), v(1), 0]
        ];

    end

end

