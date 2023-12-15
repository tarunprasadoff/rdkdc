function twist = createtwist(w,q)
    component1 = cross(-w, q);
    twist = [component1(1);component1(2);component1(3);w(1);w(2);w(3)];
end