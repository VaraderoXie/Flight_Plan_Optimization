%% STEP 1: Choose the trajectory and the aircraft
clear all, clc

desti_number=menu('Choose the trajectory','Trondheim-Oslo','Trondheim-Nice','Paris-New York');

% Two cruise trajectory configurations possible, depends on the trajectory
% chosen.

% STEP 2: Choose the aircraft in function of the travel chosen
% Configuration 1: the cruise trajectory is horizontal

if desti_number==1 % Destination: Trondheim-Oslo
    plane_number = menu('Plane','737-300','737-800');
    d=390000; % Distance Trondheim-Oslo [m]
    if plane_number==1
        disp('Torndheim-Oslo and plane 737-300');
        % Characteristics of the aircraft 737-300
        S = 105.4;  % Wing area [m^2]
        m0 = 62800.0;   % Maximum takeoff weight [kg]
        lf = 20100; % Maximum fuel capacity [l]
        alpha = 0.0735; % Parameter defining the dimensional velocity
        SFC = 0.03977;  % Coefficient fuel burn
    else
        disp('Trondheim-Oslo and plane 737-800');
        % Characteristics of the aircraft 737-800
        S = 125.58; % Wing area [m^2]
        m0 = 79100; % maximum takeoff weight [kg]
        lf = 26020; % Maximum fuel capacity [l]
        alpha = 0.0697; % Parameter defining the dimensional velocity
        SFC = 0.03875;  % Coefficient fuel burn
    end
end

if desti_number == 2    % Destination Trondheim-Nice
    plane_number = menu('Plane','A320','737-800');
    d = 2204000;    % Distance Trondheim-Nice [m]
    if plane_number == 1
        disp('Trondheim-Nice and A320');
        % Characteristics of the aircraft A320
        S = 125;    % Wing area [m^2]
        m0 = 70000; % Maximum takeoff weight [kg]
        lf = 24050; % Maximum fuel capacity [l]
        alpha = 0.07;   % Parameter defining the dimensional velocity
        SFC = 0.03467;  % Coefficient fuel burn
    else
        disp('Trondheim-Nice and plane 737-800');
        % Characterisitcs of the aircraft 737-800
        S = 125.58; % Wing area [m^2]
        m0 = 79100; % Maximum takeoff weight [kg]
        lf = 26020; % Maximum fuel capacity [l]
        alpha = 0.0697; % Parameter defining the dimensional velocity
        SFC = 0.03875;  % Coefficient fuel burn
    end
end

% Configuration 2: the cruise trajectory is not horizontal 
% theta cruise and delta altitude

if desti_number == 3
    plane_number = menu('Plane','A340','777-200');
    d = 5851000; % Distance Paris-New York
    if plane_number == 1
        disp('Paris-New York and plane A340');
        % Characteristics of the aircraft A340
        S = 361.6; % Wing area [m^2]
        m0 = 275000; % Maximum takeoff weight [kg]
        lf = 155040;    % Maximum fuel capacity [l]
        alpha = 0.0663; % Parameter defining the dimensional velocity
        SFC = 0.03263;  % Coefficient fuel burn
    else
        disp('Paris-New York and plane 777-200');
        % Characteristics of the aircraft 777-200
        S = 427.8;  % Wing area [m^2]
        m0 = 247200;    % Maximum takeoff weight [kg]
        lf = 117348;    % Maximum fuel capacity [l]
        alpha = 0.0648; % Parameter defining the dimensional velocity
        SFC = 0.03365;  % Coefficient fuel burn
    end
end

%% Parameters and constants

beta = 60000;   % Velocity/altitude parameter
gamma = input('Parameter for climb time determination (between 150 and 300)')
zc = input('The cruise altitude [m]')   % Cruise altitude [m]
rho0 = 1.225;   % Air density at sea level [kg/m^3]
epsilon = 0.0001;   % Constant in the density function [l/m]
k = 0.045;  % Constant selected for the included drag coefficient
CDo = 0.015;    % Constant selected for the zero-lift drag coefficient

%% STEP 3: CLIMB

n = length(0.01:0.001:0.99);    % Number of values in the vector
nz = n;

% Initialization at t = 0 (takeoff)
% The index 1 is for the climb values
V1 = zeros(nz, 1); v1 = zeros(nz, 1); theta1 = zeros(nz, 1); c1 = zeros(nz, 1);
M1 = zeros(nz, 1); rho1 = zeros(nz,1); CL1 = zeros(nz, 1); F1 = zeros(nz, 1);
G1 = zeros(nz, 1); D1 = zeros(nz, 1); T1 = zeros(nz, 1);
fuelburnCL = zeros(nz+1,1); fuelburnCL(1) = 0;

% Fuel capacity in the aircraft at takeoff
rhof = 817.15;  % Fuel density [kg/m3]
fuelCL = zeros(nz, 1);  % Fuel burn vector [kg]
mf1 = zeros(nz+1,1);    % Weight of fuel vector
mf1(1) = lf*10^-3*rhof; % Weight of fuel at t=0 [kg]

% Maximum takeoff weight [kg]
m1 = zeros(nz+1,1);     % Weight of the aircraft vector
m1(1) = m0;             % Maximum takeoff weight [kg]

% Calculation of the time of climb
vmean1 = 0.0333 * gamma;    % Mean vertical speed [m/s]
Timeclimb = (0.99*zc)/vmean1/3600;  % Time of climb [hours]
tclimb = Timeclimb/n;   % Time increment [hours]

% Climb trajectory
% Calculate the velocities, forces, weight of the fuel burn and aircraft
% for each increment until the cruise altitude

i = 1;
for Z = 0.01:0.001:0.99;
    V1(i)=(-1/alpha)*log((1/beta)*((1/Z)-1));   % Velocity along flight path
    v1(i) = gamma*Z^2*(1-Z)^2;  % Vertical velocity (m/s)
    % v1(i) = -6*vm1*Z^2+6*vm1*Z;   % the parabolic vertical speed
    theta1(i) = asin(v1(i)/V1(i));  % Climb angle [rad]
    c1(i) = 340.245 - 54.15*Z;  % Speed of sound [m/s]
    M1(i) = V1(i) / c1(i);      % Mach number [-]
    rho1(i) = rho0*exp(-epsilon*zc*Z);  % Air density [kg/m^3]
    CL1(i) = (m1(i)*9.81*cos(theta1(i)))/(0.5*S*rho1(i)*V1(i)^2);   % Lift coef.
    F1(i) = (m1(i)*gamma*Z*(1-Z))/(alpha*zc);   % Acceleration force F[N]
    G1(i) = m1(i)*9.81*sin(theta1(i));  % Drag due to gravity G[N]
    D1(i) = 0.5*S*rho1(i)*V1(i)^2*(CDo+k*CL1(i)^2); % Aerodynamic drag D[N]
    T1(i) = F1(i)+G1(i)+D1(i);  % Thrust force T[N]
    fuelCL(i) = SFC*T1(i)*tclimb;   % Fuel burn at each increment [kg]
    fuelburnCL(i+1) = fuelburnCL(i) + fuelCL(i);    % Total fuel burn [kg]
    mf1(i+1) = mf1(i) - fuelCL(i);  % Weight of fuel [kg]
    m1(i+1) = m1(i) - fuelCL(i);    % Weight of the aircraft [kg]
    i = i + 1;
end

% Maximum thrust required during the climb
Tmax = max(T1);

% Distance climb
Vmean1 = mean(V1);  % Mean velocity along the flight path [m/s]
vhmean1 = sqrt(Vmean1^2 - vmean1^2);    % Mean horizontal velocity [m/s]
dHclimb = vhmean1 * Timeclimb * 3600;   % Horizontal climb distance [m]
% Plot

% Velocity along the flight path
figure(1);
subplot(2,1,1), plot(V1,0.01:0.001:0.99)
title('Climb velocity along the flight path')
xlabel('Velocity along the flight path V [m/s]')
ylabel('Dimensionless altitude Z [-]')

% Vertical velocity and climb angle
subplot(2,1,2), plot(v1,0.01:0.001:0.99,theta1*(180/pi),0.01:0.001:0.99)
title('Climb vertical velocity and climb angle')
legend('Vertical velocity','Climb angle')
xlabel('Vertical velocity v [m/s] and climb angle theta [o]')
ylabel('Dimensionless altitude Z [-]')

% Forces
%figure(2);
subplot(2,2,1),
plot(F1,0.01:0.001:0.99,G1,0.01:0.001:0.99,D1,0.01:0.001:0.99,T1,0.01:0.001:0.99)
title('Climb forces')
legend('Acceleration force F','Drag due to gravity G', 'Aerodynamic drag D','Thrust force T')
xlabel('Forces and required thrust [N]')
ylabel('Dimensionless altitude Z [-]')

% Fuel burn at each increment
subplot(2,2,2), plot(fuelCL,0.01:0.001:0.99)
xlabel('Fuel burn [kg]')
ylabel('Dimensionless altitude Z [-]')
title('Fuel consumption at each step')

% Fuel consumption
subplot(2,2,3), plot(fuelburnCL, 0.009:0.001:0.99)
xlabel('Fuel burn [kg]')
ylabel('Dimensionless altitude Z [-]')
title('Fuel consumption during the climb')

% Values for the next step: the cruise

step1=('CLIMB')

disp(['Fuel burn after the climb [kg]: ', num2str(fuelburnCL(end))]);
disp(['Weight of the aircraft m [kg]: ', num2str(m1(end))]);
disp(['Weight of fuel mf [kg]: ', num2str(mf1(end))]);
disp(['Mach number M [-] (for the cruise phase): ', num2str(M1(end))]);
disp(['Speed of sound c [m/s]: ', num2str(c1(end))]);
disp(['Value of CL: ', num2str(CL1(end))]);
disp(['Time of climb [h]: ', num2str(Timeclimb)]);
disp(['Horizontal climb distance [m]: ', num2str(dHclimb)]);

%% STEP 4: CRUISE
% The cruise trajectory for the short and medium flight is horizontal
% Use for the flight Trondheim - Oslo and Trondheim - Nice
% The index c and 2 are for the cruise values

% Initialization for the cruise trajectory
step2 = ('CRUISE')

V2 = M1(end)*c1(end);   % Cruise velocity [m/s] (constant)
disp(['Cruise velocity Vc [m/s]: ', num2str(V2)]);

% Calculate the air density and the aircraft weight at zc (after climb)
zc = Z(end)*zc;
rhoc = rho0*exp(-epsilon*zc);   % Air density [kg/m^3]
mc = ((CL1(end)*S*V2^2)/(2*9.81))*rhoc; % Weight of the aircraft [kg]
disp(['Weight of the aircraft at cruise altitude 1 [kg]: ', num2str(mc)]);

% Cruise time and distance
% Define the time and the distance for the descent
gamma = input('Parameter for descent time determination (between 100 and 200)')
vmean3 = 0.0333*gamma;  % Mean vertical speed [m/s]
vhmean3 = sqrt(Vmean1^2 - vmean3^2);    % Mean horizontal velocity [m/s]
Timedescent = (zc * 0.99)/vmean3/3600;  % Time of descent [hours]
dHdescent = vhmean3 * Timedescent * 36000;  % Horizontal descent distance [m]

% Time and distance for the cruise phase
dHcruise = d - dHdescent - dHclimb;     % Horizontal cruise distance [m]
Timecruise = dHcruise / V2 / 3600;  % Time of cruise [hours]

% Thrust force [N]
T2 = D1(end);   % Thrust force is constant [N]

% Fuel burn
fuelburnCR = SFC * Timecruise*T2;   % Fuel burn during the cruise [kg]
m2 = mc - fuelburnCR;   % Weight of aircraft after the cruise [kg]
mf2 = mf1(end) - fuelburnCR;    % Weight of fuel [kg]

disp(['Fuel burn during the cruise [kg]: ', num2str(fuelburnCR)]);
disp(['Weight of the aircraft m after the cruise [kg]: ', num2str(m2)]);
disp(['Weight of fuel mf after the cruise[kg]: ', num2str(mf2)]);
disp(['Time of cruise [h]: ', num2str(Timecruise)]);
disp(['Horizontal cruise distance [m]: ', num2str(dHcruise)]);

%% STEP 4-bis: CRUISE
% The cruise trajectory for the long flight is linear between zc1 and zc2
% Use for the flight Paris - New York

% Calculate the air density and the aircraft weight at zc (after climb)
zc1 = Z(end)*zc;
rhoc = rho0 * exp(-epsilon*zc1);    % air density [kg/m^3]
mc = ((CL1(end)*S*V2^2)/(2*9.81))*rhoc;     % Weight of the aircraft [kg]
disp(['Weight of the aircraft at cruise altitude 1 [kg]: ', num2str(mc)]);

% Delta cruise altitude [m]
deltazc = input('Value of the delta cruise altitude [m]')
% Altitude at the end
zc2 = zc1 + deltazc;
ncr = length(zc1:1:zc1+deltazc);    % Number of values in the vector

% Cruise time and distance
% Define the time and the distance for the descent
gamma = input('Parameter descent time determination (between 100 and 200)')
vmean3 = 0.0333*gamma;  % Mean vertical velocity [m/s]
vhmean3 = sqrt(Vmean1^2 - vmean3^2);    % Mean horizontal descent velocity [m/s]
Timedescent = (zc2*0.99)/vmean3/3600;   % Time of descent [hours]
dHdescent = vhmean3*Timedescent*3600;   % Horizontal descent distance [m]
% Time and distance for the cruise phase
dHcruise = d - dHdescent - dHclimb; % Horizontal cruise distance [m]
Timecruise = dHcruise/V2/3600;  % Time of cruise [hours]
tcruise = Timecruise/ncr;   % Increment time [hours]

% Initialization at cruise altitude z1
v2 = zeros(ncr,1); theta2 = zeros(ncr,1); rho2 = zeros(ncr,1);
D2 = zeros(ncr,1); T2 = zeros(nz,1); fuelburnCR = zeros(nz,1);

% Fuel capacity in the airplane after the climb
fuelCR = zeros(nz,1);   % Fuel burn during the cruise [kg]
mf2 = zeros(ncr+1,1);   % Weight of fuel vector
mf2(1) = mf1(end);      % Weight of fuel during the cruise [kg]

% Maximum take off weight [kg]
m2 = zeros(nz+1,1); % Weight of the aircraft vector
m2(1) = mc; % Weight of the aircraft [kg]

% Cruise trajectory
i = 1;
for z = zc1:1:zc2;
    rho2(i) = rho0*exp(-epsilon*z); % Air density [kg/m^3]
    D2(i) = 0.5 * S * rho2(i)*V2^2*(CDo+k*CL1(end)^2);  % Aerodynamic drag D [N]
    T2(i) = D2(i);  % Thrust force [N]
    fuelCR(i) = SFC*T2(i)*tcruise;  % Fuel burn at each increment [kg]
    fuelburnCR(i+1) = fuelburnCR(i) + fuelCR(i);    % Total fuel burn [kg]
    mf2(i+1) = mf2(i) - fuelCR(i);  % Weight of fuel [kg]
    m2(i+1) = m2(i) - fuelCR(i);    % Weight of the airplane [kg]
    i = i + 1;
end
%% STEP 5: DESCENT

% The index 3 is for the descent values

% Initialization
% Fuel burn before the descent [kg]
fuelD = zeros(nz, 1);
fuelburnD = zeros(nz+1,1);  fuelburnD(1) = 0;

% Weight aircraft and fuel before the descent [kg]

% Maximum take off weight [kg]

m3 = zeros(nz+1,1); m3(1) = m2(end);
mf3 = zeros(nz+1,1); mf3(1) = mf2(end);

V3 = zeros(nz,1); v3 = zeros(nz,1); theta3 = zeros(nz,1); rho3 = zeros(nz,1);
c3 = zeros(nz,1); M3 = zeros(nz,1);
CL3 = zeros(nz,1); F3 = zeros(nz,1); G3 = zeros(nz,1); D3 = zeros(nz,1);
T3 = zeros(nz, 1);

% Calculation of the time increment
tdescent = Timedescent/n;   % Time increment [hours]

% Descent trajectory 
% Calculate teh velocities, forces, weight of the fuel burn and airplane
% for each incremnet until the landing

i = 1;
for Z = 0.01:0.001:0.99;
    Zd = 1 - Z;     % dimensionless altitude
    V3(i) = (-1/alpha)*log((1/beta)*((1/Zd)-1));    % Velocity along flight path
    v3(i) = -gamma*Zd^2*(1-Zd)^2;   % Vertical speed (m/s)
    % v3(i) = -(-6*vm1*Z^2+6*vm1*Z);    % the parabolic vertical speed
    theta3(i) = asin(v3(i)/V3(i));  % Climb angle [rad]
    c3(i) = 340.254 - 45.15*Zd;     % Speed of sound [m/s]
    M3(i) = V3(i) / c3(i);          % Mach number [-]
    rho3(i) = rho0 * exp(-epsilon*zc*Zd);   % Air density [kg/m^3]
    CL3(i) = (m3(i) * 9.81 * cos(-theta3(i)))/(0.5*S*rho3(i)*V3(i)^2);  % Lift coef.
    F3(i) = (m3(i)*gamma*Zd*(1-Zd))/(alpha*zc); % Acceleration force F [N]
    G3(i) = m3(i) * 9.81 * sin(-theta3(i)); % Drag due to gravity Gd [N]
    D3(i) = 0.5 * S * rho3(i) * V3(i)^2 * (CDo + k * CL3(i)^2); % Aerodynamic drag D [N]
    T3(i) = D3(i) - G3(i) - F3(i);  % Thrust force [N] can't be negative
    if T3(i) <= (10/100)*Tmax
        T3(i) = (10/100)*Tmax;
    elseif T3(i) > (10/100)*Tmax
        T3(i) = T3(i);
    end
    fuelD(i) = SFC*T3(i)*tdescent;  % Fuel burn at each increment [kg]
    fuelburnD(i+1) = fuelburnD(i) + fuelD(i);   % Total fuel burn [kg]
    mf3(i+1) = mf3(i) - fuelD(i);   % Weight of fuel [kg]
    m3(i+1) = m3(i) - fuelD(i); % Weight of the aircraft [kg]
    i = i + 1;
end

Tmean = mean(T3)

% Horizontal descent distance
Vmean3 = mean(V3);  % Mean velocity along the flight path [m/s]
vhmean3 = sqrt(Vmean3^2 - vmean3^2);    % Mean horizontal descent velocity [m/s]
dHdescent = vhmean3 * Timedescent * 3600;   % Horizontal descent distance [m]

% Plot

% Forces
figure(3);
subplot(2,2,1),
plot(F3, 0.01:0.001:0.99,G3,0.01:0.001:0.99,D3,0.01:0.001:0.99,T3,0.01:0.001:0.99)
title('Descent forces')
legend('Acceleration force F', 'Drag due to gravity G', 'Aerodynamic drag D', 'Thrust force T')
xlabel('Forces and required thrust [N]')
ylabel('Dimensionless altitude Z [-]')

% Vertical velocity and descent angle
subplot(2,2,2), plot(v3, 0.01:0.001:0.99, theta3*(180/pi),0.01:0.001:0.99)
title('Descent vertical velocity and descent angle')
legend('Vertical velocity', 'Descent angle')
xlabel('Vertical velocity v [m/s] and descent angle theta [o]')
ylabel('Dimensionless altitude Z [-]')

% Fuel burn at each increment
subplot(2,2,3), plot(fuelD, 0.01:0.001:0.99)
xlabel('Fuel burn [kg]')
ylabel('Dimensionless altitude Z [-]')
title('Fuel consumption at each step')

% Fuel burn
subplot(2,2,4), plot(fuelburnD, 0.99:-0.001:0.009)
xlabel('Fuel consumption [kg]')
ylabel('Dimensionless altitude Z [-]')
title('Fuel consumption during the descent')

% Values of the descent
step3 = ('DESCENT')
disp(['Value of fuel burn during the descent [kg]: ', num2str(fuelburnD(end))]);
disp(['Time of descent [hr]: ', num2str(Timedescent)]);
disp(['Horizontal descent distance [m]: ', num2str(dHdescent)]);

%% Results

disp('RESULTS')

% Time of the trajectory [hours]
TIME = Timeclimb + Timecruise + Timedescent;
disp(['Time trajectory [h]: ', num2str(TIME)]);

% The fuel burn [kg]
fuelburn = fuelburnCL(end) + fuelburnCR(end) + fuelburnD(end);
disp(['Fuel burn during the trajectory [kg]: ', num2str(fuelburn)]);

% The remaining fuel [kg]
disp(['Weight of remaining fuel [kg]: ', num2str(mf3(end))]);
% The weight of the airplane [kg]
disp(['Weight of airplane after the trajectory [kg]: ', num2str(m3(end))]);







