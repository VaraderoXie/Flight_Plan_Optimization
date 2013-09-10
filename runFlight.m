function [fuelburn, TIME] = runFlight(climbAltitudeMeters, cruiseAltitudeMeters, totalDistance, descentTime)
%% Calculate fuel burn and flight time for projected route
% We run these calculations using the following functions
% [fuelburn, planeWeight, fuelWeight, machCruise,SpeedOfSound,liftCoef,planeDrag,horizClimbDist,Tmax,Vmean1] = planeClimb(planeNumber, descentTime, climbAltitude)
% [fuelburn, planeWeight, fuelWeight, cruiseTime, horizCruiseDist] = planeCruise(planeNumber, cruiseAltitude, deltaAltitude, liftCoef, totalDistance, SpeedOfSound, machCruise, descentTime, fuelweightCL, horizClimbDist,Vmean1,planeDrag)    
% [fuelburn, TIME] = planeLand(planeNumber,descentTime,planeWeight,fuelWeight, fuelburnCL, fuelburnCR, Timedescent,Tmax)

planeNumber = 1;
deltaAltitude = 0;

[fuelburnClimb, planeWeightClimb, fuelWeightClimb, machCruise, SpeedOfSound, liftCoef, planeDrag, horizClimbDist, Tmax, Vmean1, Timeclimb] = planeClimb(planeNumber, descentTime, climbAltitudeMeters);
[fuelburnCruise, planeWeightCruise, fuelWeightCruise, cruiseTime, horizCruiseDist, vmean3, Timedescent] = planeCruise(planeNumber, cruiseAltitudeMeters, deltaAltitude, liftCoef, totalDistance, SpeedOfSound, machCruise, descentTime, fuelWeightClimb, horizClimbDist, Vmean1, planeDrag);
[fuelburnTotal, totalTime] = planeLand(planeNumber, descentTime, planeWeightCruise, fuelWeightCruise, fuelburnClimb, fuelburnCruise, Timedescent, Tmax, climbAltitudeMeters, vmean3, Timeclimb, cruiseTime);

disp('Plane Climb Output');
outputString = sprintf('fuelburnClimb = %f\nplaneWeightClimb = %f\nfuelWeightClimb = %f\nmachCruise = %f\nSpeedOfSound = %f\nliftCoef = %f\nplaneDrag = %f\nhorizClimbDist = %f\nTamx = %f\nVmean1 = %f\n', ...
    fuelburnClimb, planeWeightClimb, fuelWeightClimb, machCruise, SpeedOfSound, liftCoef, planeDrag, horizClimbDist, Tmax, Vmean1);
disp(outputString);

disp('Plane Cruise Output');
outputString = sprintf('fuelburnCruise = %f\nplaneWeightCruise = %f\nfuelWeightCruise = %f\ncruiseTime = %f\nhorizCruiseDist = %f\n', ...
    fuelburnCruise, planeWeightCruise, fuelWeightCruise, cruiseTime, horizCruiseDist);
disp(outputString);

disp('Plane Land Output');
outputString = sprintf('fuelburnTotal = %f\ntotalTime = %f\n', fuelburnTotal, totalTime);
disp(outputString);

fuelburn = fuelburnTotal;
TIME = totalTime;

