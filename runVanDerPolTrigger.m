


%% Define model and system parameters
A=[0 1;-1 -1]; B=[0;1];
Ar=A; Br=B;
x0=[2,3]; xr0=[2,2];


%% Initial controller parameters
rmax = norm(xr0);
gf = 1e4;
Tswitch_min=5;
noisy=true;     % Decide if unmatched uncertainty should be added in

% Defines the approximation region
Ri = 2*sqrt(2)-0.3; 
Ro = 2*sqrt(2)+0.3;


delta = 0; % for used in State limiting mode control
triggered = 1; % used in Simulink
del = 0;




%% verify controllability
if  rank(ctrb(A,B)) == size(A,1)
    fprintf('System controllable\n')
    pDes = [-0.5000 + 0.8660i;-0.5000 - 0.8660i];
    kx = place(A,B,pDes);
else
    fprintf('System uncontrollable\n')
end
ga = 0; 
gb = 0;

Q = eye(2);

P = lyap(A',Q);

mu = 3;


%% Defines target hyperparameters

type = 'g';
%type = 'ms';
scale=0.3;
par = 6;

%% Initial simulation and Simunlink parametrs

runTimes = 200;
Newt = 1;
model = 'NewtTriggerVanDerPol';
open(model)
trackError0 = abs(x0-xr0);
maxN = 500;

ssErrorNB = [];


cs = getActiveConfigSet(model);
csNew = copy(cs);
set_param(csNew,"StopTime",num2str(runTimes))
StartMeasureTime = runTimes-Tswitch_min/2;
startTime = 0;
h = 1e-1;
N0 = 0;
lambda0 = [];
triggerErrors = [];
triggerTimes = [];
trackingError = [];
Epsilons = [];
totTime = [];
rTot = [];
xTot = [];
xrTot = [];
input = [];
vn = [];
uSL = [];
nonCo = [];
fhat = [];
fNon = [];
Ns = [];
condK = [];
condN =[];
C = 0.7;

Omega = omegaCover2(Ri,Ro,h);
%% Determines the vanderpol norm
[norm_fs,condT,Ntest] = getVDP_HNorm(type,par,scale,mu,Ri,Ro);

norm_f = max(norm_fs(condT < 1e12))
format short g;
condT
tol = C*norm(x0-xr0);
R = norm_f+0.1;
triggerTime = 0;

%%
while norm(trackError0) > 1e-7 && N0 < maxN && triggerTime < runTimes
    set_param(csNew,"StartTime",num2str(triggerTime))
    
    
    [gCenters,offlineError,powBound,condNum,condNumNewt,gV] = greedySet2(tol,Omega,maxN,type,par,scale,norm_f);
    
    N = size(gCenters,1)
    Ns = [Ns;N];
    
    if N > N0
        
        N = size(gCenters,1);
        lambda0 = [lambda0;zeros(N-N0,1)];
        
        if N > 1 && N ~= maxN
            PowerBound = powBound(N+1);
        else
            PowerBound = powBound(N);
        end
        N0 = N;
        gK = real(kermat(gCenters,gCenters,type,par,scale));
        
        
        
        gV = gV(1:N,1:N);
        z = diag(gK);
        P = sqrt(gK(1,1));
        
        
        V = zeros(N,N);
        
        V(:,1) = gK(:,1)/P;
        if size(V,1) > 1
            w = V(:,1).^2;
            P = sqrt(z(2)-w(2));
            
            for jj = 1:N-1
                e = zeros(N,1);
                e(jj+1) = 1;
                Vm = V(:,1:jj);
                uSL = gK*e-Vm*Vm'*e;
                V(:,jj+1) = uSL/P; % Newton Basis evaluated at the centers
                w = V(:,jj+1).^2 + w; % Updating sum of Newton basis squared at the centers
                if jj ~=N-1
                    P = sqrt(z(jj+2)-w(jj+2));
                end
            end
        end
    end
    tol = C*PowerBound*R
    condN = [condN;cond(V)];
    epsilon = PowerBound*R+delta
    
    Epsilons = [Epsilons;epsilon];
    
    %% RKHS Preparation
    centers=gCenters;
    K = real(kermat(centers,centers,type,par,scale));
    condK = [condK;cond(K)];
    %% Runs Simulink
    % 
    simresult=sim(model,csNew);
    
    %% collects data
    x = simresult.x.Data;
    time = simresult.x.time;
    triggerTime = time(end)
    
    xr = simresult.xr.Data;
    
    fNon = [fNon;simresult.fNon.Data];
    fhat = [fhat;-simresult.fhat.Data];
    
    nonCo = [nonCo;simresult.nonCo.Data];
    vn = [vn;simresult.vn.Data];
    uSL = [uSL;simresult.uSL.Data];
    
    input = [input;simresult.u.Data];
    lambda = simresult.lambdaHat.Data;
    
    r = simresult.reference.Data;
    x0 = x(end,:);
    xr0 = xr(end,:);
    rTot = [rTot;r];
    xrTot = [xrTot;xr];
    xTot = [xTot;x];
    lambda0 = lambda(end,:)';
    trackError0 = norm(simresult.trackingError.Data(end,:))
    trackingError = [trackingError;abs(simresult.trackingError.Data)];
    totTime = [totTime;time];
    if triggerTime < runTimes && N0 < maxN && trackError0 > 1e-7
        triggerErrors = [triggerErrors;trackError0];
        triggerTimes = [triggerTimes;triggerTime];
        
    end
    if N0 == maxN
        triggered = 0;
    end
end

%% no more triggers Simulation runs until the final time is met
triggered = 0;
if triggerTime < runTimes
    set_param(csNew,"StartTime",num2str(triggerTime))
    % 
    % 
    
    
    %% Run Simulink
    % 
    simresult=sim(model,csNew);
    
    %%
    x = simresult.x.Data;
    time = simresult.x.time;
    
    
    xr = simresult.xr.Data;
    
    fNon = [fNon;simresult.fNon.Data];
    fhat = [fhat;-simresult.fhat.Data];
    
    nonCo = [nonCo;simresult.nonCo.Data];
    vn = [vn;simresult.vn.Data];
    uSL = [uSL;simresult.vn.Data];
    
    input = [input;simresult.u.Data];
    lambda = simresult.lambdaHat.Data;
    
    r = simresult.reference.Data;
    
    trackingError = [trackingError;abs(simresult.trackingError.Data)];
    totTime = [totTime;time];
    xTot = [xTot;x];
    xrTot = [xrTot;xr];
end

%%
data.Ns = Ns;
data.trackingError = trackingError;
data.totTime = totTime;
data.triggerTimes = triggerTimes ;
data.xTot = xTot;
data.condK = condK;
data.condN = condN;
data.gCenters = gCenters;
data.Epsilons = Epsilons;
data.runTimes = runTimes;
data.triggerErrors = triggerErrors;


params.B = B;
params.P = P;
params.rmax = rmax;

plotSimulinkResults(data,params)