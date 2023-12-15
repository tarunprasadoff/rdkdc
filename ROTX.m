function r = ROTX(rad)
    
    r = [
      [1, 0, 0];
      [0, cos(rad), -sin(rad)];
      [0, sin(rad), cos(rad)]
    ];

end

