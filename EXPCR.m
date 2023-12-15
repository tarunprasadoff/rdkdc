function Q = EXPCR(x,angle)
    %invoke rodrigues and skew3 here
    Q = eye(3) + sin(norm(x)*angle)*SKEW3(x)/norm(x) + ((1-cos(norm(x)*angle))/norm(x)^2)*(SKEW3(x))^2;
end
