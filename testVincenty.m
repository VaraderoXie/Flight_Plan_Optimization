% % =====================================================================
% % Code to test the Mapping Toolbox spherical earth distance against
% % Vincenty's algorithm using random test points:
format short g
errmax=0;
abserrmax=0;
for i = 1:10000
    llat = rand * 184-92;
    tlat = rand * 184-92;
    llon = rand * 364 - 2;
    tlon = rand * 364 - 2;
    llat = max(-90,min(90,llat)); % round to include occasional exact poles
    tlat = max(-90,min(90,tlat));
    llon = max(0,min(360,llon));
    tlon = max(0,min(360,tlon));
    % randomly test exact equator
    if rand < .01
        llat = 0;
        llon = 0;
    else
        if rand < .01
            llat = 0;
        end
        if rand < .01
            tlat = 0;
        end
    end
    dm = 1000*deg2km(distance(llat,llon,tlat,tlon));
    dv = vdist(llat,llon,tlat,tlon);
    abserr = abs(dm-dv);
    if abserr < 1e-2 % disagreement less than a centimeter
        err = 0;
    else
        err = abs(dv-dm)/dv;
    end
    errmax = max(err,errmax);
    abserrmax = max(abserr,abserrmax);
    %     if i==1 | rand > .99
    disp([i dv dm err errmax abserrmax])
    %     end
    if err > .01
        break
    end
end