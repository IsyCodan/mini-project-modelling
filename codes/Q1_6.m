rho = 1240; %Density of filament
cp = 1800; %Specific heat capacity of filament
k = 0.2; %Thermal Conductivity
h = 15; %convection coeff to air
hp = 500; %convection coeff to hot plate
a = 0.0004; %length of cross-section of filament

Tdep = 220; %nozzle temp
Tinf = 22; %ambient temp
Tp = 60; %Hot plate temp

vp = 0.04; %printing speed

r = 0.02; %radius of circle to be printed

Ltotal = 2*pi*r; %distance to print

tprint = Ltotal/vp; %minimum time needed to print

%Discretizationnnnn

%spatial discretization
N = 1001; %more cells = better results = bad computation time
s = linspace(0, Ltotal, N)';
ds = s(2)-s(1);

alpha = k/(rho*cp);
betaAir = 3*h/(rho*cp*a);
betaPlate = hp/(rho*cp*a);

%time discretization
dtMax = 1/(2*alpha/ds^2 + betaAir + betaPlate); %explicit scheme
dt = 0.8*dtMax;

tEnd = tprint + 8; %change number to continue simulation for time beyond printing
Nt = ceil(tEnd/dt) + 1;
dt = tEnd/(Nt-1); %adjusting time step based on end time.
time = linspace(0, tEnd,Nt);

%%Temp field
T = nan(N,Nt); %Temp at node i at j time [T(i,j)]
T(:,1) = nan;

T(1,1) = Tdep;

for j = 2:Nt
    tNow = time(j);

    %nodes where material is deposited
    nActive = min(N, floor(vp*tNow/ds)+1); %so that we do not exceed the size of matrix
    nActive = max(nActive,1); %at least one node is active

    T(:,j) = T(:,j-1);

    %assign initial temp to all freshly deposited nodes
    for q = 1:nActive
        if isnan(T(q,j-1))
            T(q,j-1) = Tdep;
        end
    end

    %temp of active nodes at prev time
    Told = T(1:nActive,j-1);

    %boundry flux approximation
    Tleft = Told;
    Tright = Told;

    if nActive > 1
        Tleft(2:end) = Told(1:end-1);
        Tright(1:end-1) = Told(2:end);
    end

    lapT = (Tleft - 2*Told + Tright)/ds^2; %fourier law

    dTdt = alpha*lapT - betaAir*(Told-Tinf) - betaPlate*(Told-Tp);

    T(1:nActive,j) = Told + dt*dTdt;
end

%%plot of spatial and temporal distribution of temperature

figure;
imagesc(time,s,T);
set(gca,'YDir','normal');
xlabel('Time (s)');
ylabel('Distance along filament (m)');
title('Temperature Evolution During Printing and Cooling');
cb = colorbar;
ylabel(cb,'Temperature (deg C)');

idxPrint = find(time >= tprint,1);
uniformTolerance = 1;

coolingTime = 0;
uniformTime = 0;

for j = idxPrint:Nt
    activeT = T(:,j);
    activeT = activeT(~isnan(activeT));
    
    if isempty(activeT)
        continue;
    end

    tempRange = max(activeT) - min(activeT);

    if tempRange <= uniformTolerance
        uniformTime = time(j);
        coolingTime = uniformTime - tprint;
        break;
    end
end

if uniformTime == 0
    fprintf('Filament did not reach the uniformity threshold. Increase simulation time\n');
else
    fprintf('Uniformity reached at %.3f s (%.3f s after printing).\n', ...
        uniformTime, coolingTime);
    fprintf('Total printing time       : %.3f s\n', tprint);
    fprintf('Additional cooling time   : %.3f s\n', coolingTime);
    fprintf('Total time (print + cool) : %.3f s\n', (tprint + coolingTime));
end
