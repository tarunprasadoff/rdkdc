function anglenumber = determineangle(solutions,ur5)
sum = 0;
lowestsum = 100000;
currentselection = 0;
currentangles = ur5.get_current_joints();
for i = 1:8
    for j = 1:6
        term1 = solutions(j,i);
        term2 = currentangles(j);
        sum = sum+abs(term1+term2);
    end
    if(sum < lowestsum)
        lowestsum = sum;
        currentselection = i;
    end
end
anglenumber = currentselection;
end