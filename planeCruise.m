function [fuelburn, planeWeight, fuelWeight, cruiseTime, horizCruiseDist] = planeCruise(planeNumber, cruiseAltitude, deltaAltitude, liftCoef, totalDistance, SpeedOfSound, machCruise, descentTime, fuelweightCL, horizClimbDist)
% PLANECRUISE(planeNumber, cruiseSpeed,cruiseAltitude) calculates the fuel 
% consumption during the cruising phase of the flight. 

if (nargin < 10)
    error('on','Usage: planeCruise(planeNumber, cruiseAltitude, deltaAltitude, liftCoef, totalDistance, SpeedOfSound, machCruise, descentTime, fuelweightCL, horizClimbDist)');
end

% planeType returns S, m0, lf, alpha, SFC
[S, ~, ~, ~, SFC] = planeType(planeNumber);

%% Parameters and constants

beta = 60000;   % Velocity/altitude parameter
if (descentTime >= 150 && descentTime <= 300) 
    gamma = descentTime; % A number between 150-300
else
    warning('Invalid descentTime. Input range is 150 - 300');
end
zc = climbAltitude;   % Cruise altitude [m]
rho0 = 1.225;   % Air density at sea level [kg/m^3]
epsilon = 0.0001;   % Constant in the density function [l/m]
k = 0.045;  % Constant selected for the included drag coefficient
CDo = 0.015;    % Constant selected for the zero-lift drag coefficient

%% Simulate Cruise
nz = length(0.01:0.001:0.99);
V2 = SpeedOfSound * machCruise;   % Cruise velocity [m/s] (constant)

if (totalDistance < 5851000) 
    % Calculate the air density and the aircraft weight at zc (after climb)
    zc = cruiseAltitude;
    rhoc = rho0*exp(-epsilon*zc);   % Air density [kg/m^3]
    mc = ((liftCoef*S*V2^2)/(2*9.81))*rhoc; % Weight of the aircraft [kg]


    % Cruise time and distance
    % Define the time and the distance for the descent
    if (descentTime >= 100 && descentTime <= 200) 
        gamma = descentTime;          % Pick a number between 100 and 200
    else
        warning('Invalid descentTime. Input range is 100 - 200');
    end
    vmean3 = 0.0333*gamma;  % Mean vertical speed [m/s]
    vhmean3 = sqrt(Vmean1^2 - vmean3^2);    % Mean horizontal velocity [m/s]
    Timedescent = (zc * 0.99)/vmean3/3600;  % Time of descent [hours]
    dHdescent = vhmean3 * Timedescent * 36000;  % Horizontal descent distance [m]

    % Time and distance for the cruise phase
    dHcruise = totalDistance - dHdescent - horizClimbDist;     % Horizontal cruise distance [m]
    Timecruise = dHcruise / V2 / 3600;  % Time of cruise [hours]

    % Thrust force [N]
    T2 = planeDrag;   % Thrust force is constant [N]

    % Fuel burn
    fuelburnCR = SFC * Timecruise*T2;   % Fuel burn during the cruise [kg]
    m2 = mc - fuelburnCR;   % Weight of aircraft after the cruise [kg]
    mf2 = fuelweightCL - fuelburnCR;    % Weight of fuel [kg]

    fuelburn = fuelburnCR;
    planeWeight = m2;
    fuelWeight = mf2;
    cruiseTime = Timecruise;
    horizCruiseDist = dHcruise;
else
    zc1 = cruiseAltitude;
    rhoc = rho0 * exp(-epsilon*zc1);                % air density [kg/m^3]
    mc = ((liftCoef * S * V2^2)/(2*9.81))*rhoc;     % Weight of the aircraft [kg]
    deltazc = deltaAltitude;
    zc2 = zc1 + deltazc;
    ncr = length(zc1:1:zc1+deltazc);                % Number of values in the vector
    
    % Cruise time and distance
    % Define the time and the distance for the descent
     if (descentTime >= 100 && descentTime <= 200) 
        gamma = descentTime;          % Pick a number between 100 and 200
    else
        warning('Invalid descentTime. Input range is 100 - 200');
    end
    vmean3 = 0.0333*gamma;                          % Mean vertical velocity [m/s]
    vhmean3 = sqrt(Vmean1^2 - vmean3^2);            % Mean horizontal descent velocity [m/s]
    Timedescent = (zc2*0.99)/vmean3/3600;           % Time of descent [hours]
    dHdescent = vhmean3 * Timedescent * 3600;       % Horizontal descent distance [m]
    % Time and distance for the cruise phase
    dHcruise = totalDistance - dHdescent - horizClimbDist;   % Horizontal cruise distance [m]
    Timecruise = dHcruise/V2/3600;                  % Time of cruise [hours]
    tcruise = Timecruise/ncr;                       % Increment time [hours]
    
    % Initialization at cruise altitude z1
    v2 = zeros(ncr, 1); theta2 = zeros(ncr,1);  rho2 = zeros(ncr,1);
    D2 = zeros(ncr,1);  T2 = zeros(nz, 1);  fuelburnCR = zeros(nz,1);
    
    % Fuel capacity in the airplane after the climb
    fuelCR = zeros(nz, 1);      % Fuel burn during the cruise [kg]
    mf2 = zeros(ncr+1,1);       % Weight of fuel vector
    mf2(1) = fuelweightCL;      % Weight of fuel during the cruise [kg]
    
    % Maximum take off weight [kg]
    m2 = zeros(nz+1,1);         % Weight of the aircraft vector
    m2(1) = mc;                 % Weight of the aircraft [kg]
    
    % Cruise trajectory
    i = 1;
    for z = zc1:1:zc2;
        rho2(i) = rho0*exp(-epsilon*z);     % Air density [kg/m^3]
        D2(i) = 0.5 * S * rho2(i)*V2^2*(CDo+k*liftCoef^2);  % Aerodynamic drag D [N]
        T2(i) = D2(i);                      % Thrust force [N]
        fuelCR(i) = SFC*T2(i)*tcruise;      % Fuel burn at each increment [kg]
        fuelburnCR(i+1) = fuelburnCR(i) + fuelCR(i);       % Total fuel burn [kg]
        mf2(i+1) = mf2(i) - fuelCR(i);      % Weight of fuel [kg]
        m2(i+1) = m2(i) - fuelCR(i);        % Weight of the airplane [kg]
        i = i + 1;
    end
    fuelburn = fuelburnCR;
    planeWeight = m2(end);
    fuelWeight = mf2(end);
    cruiseTime = Timecruise;
    horizCruiseDist = dHcruise;
end

    