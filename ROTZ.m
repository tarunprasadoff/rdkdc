function r = ROTZ(rad)
       
    r = [
      [cos(rad), -sin(rad), 0];
      [sin(rad), cos(rad), 0];
      [0, 0, 1]
    ];

end