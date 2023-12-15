function r = ROTY(rad)
    
    r = [
      [cos(rad), 0, sin(rad)];
      [0, 1, 0];
      [-sin(rad), 0, cos(rad)]
    ];

end

