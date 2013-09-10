function [totalFuelBurn, totalFlightTime] = runFullFlight()
%% runFullFlight simulates the flight to calculate fuel consumption and time.
planeType = 1;
if (planeType == 1) 
    ServiceCeilingFeet = 41000.0;
    OperatingEmptyWeightPounds = 91108.0;
    FuelCapacityGallons = 7837.0;
    MaximumTakeOffWeightPounds = 174200.0;
    MaximumMachSpeed = 0.82;
    MinimumAirSpeedKnots = 150.0;
    TaxiFuelBurnPoundsPerHour = 1984.158;
end
    
