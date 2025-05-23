%% Excavator Path – Stress‑Limited Lift Capacity Only
%   Sample points:
%     • 10 along boom‑lowering segment  – boom stress limit (green tags)
%     •  5 along arm‑retraction segment – arm  stress limit (dark blue tags)
%     •  2 along bucket‑curl segment    – arm  stress limit (magenta tags)

clear; close all; clc;


%% Geometry (unchanged)

L_boom   = 1.12;        % m
L_arm    = 0.87;        % m
L_bucket = 0.55;        % m


%% Material & cross‑section data (SOLIDWORKS section properties)

% Allowable bending stress
sigma_y     = 172e6;              % Pa  (yield)
SF          = 1.5;                
sigma_allow = sigma_y / SF;       % Pa

% Second moment of *area* about in‑plane bending axis
I_boom = 2.446116e-6;             % m^4  (Lxx = 2 446 116 mm^4)
I_arm  = 2.016595e-6;             % m^4  (Lxx = 2 016 595 mm^4)

% Extreme‑fibre distances (measured CAD)
c_boom = 0.04525;                 % m
c_arm  = 0.04517;                 % m

M_allow_boom = sigma_allow * I_boom / c_boom;   % N·m
M_allow_arm  = sigma_allow * I_arm  / c_arm ;   % N·m


%% Masses & CG locations (measured SOLIDWORKS)

mb_boom   = 32.35;     % kg
mb_arm    = 23.53;     % kg  
mb_bucket = 150;       % kg

g      = 9.81;        % m/s²
L_BCG  = 0.57685;      % m   
L_ACG  = 0.41450;      % m   


%% Trajectory joint angles 

ths  = deg2rad([ 63.6  9.28  350.72]);   % start
thf  = deg2rad([-14.53 -85.77 -138.14]); % end
dt   = mod(thf - ths + pi, 2*pi) - pi;

nBoom   = 80; nArm = 80; nBucket = 80;
Ntot    = nBoom + nArm + nBucket;
TH      = zeros(Ntot,3);
Xpath   = zeros(Ntot,1);
Ypath   = zeros(Ntot,1);

step = 0;
for k = 1:nBoom
    step = step + 1;
    TH(step,:) = [ths(1)+dt(1)*(k/nBoom)  , ths(2) , ths(3)];
end
for k = 1:nArm
    step = step + 1;
    TH(step,:) = [ths(1)+dt(1)           , ths(2)+dt(2)*(k/nArm) , ths(3)];
end
for k = 1:nBucket
    step = step + 1;
    TH(step,:) = [ths(1)+dt(1)           , ths(2)+dt(2)          , ths(3)+dt(3)*(k/nBucket)];
end

% Forward kinematics for every pose
for i = 1:Ntot
    [Xpath(i),Ypath(i)] = forwardK(TH(i,:), L_boom, L_arm, L_bucket);
end

%% Stress‑limited payload calculation

N1 = 10; N2 = 5;
idx1 = round(linspace(1          , nBoom      , N1));   
idx2 = round(linspace(nBoom + 1  , nBoom+nArm , N2));   
idx3 = [nBoom + nArm + 1 , Ntot];                       

mStressBoom = zeros(Ntot,1);
mStressArm  = zeros(Ntot,1);

for i = 1:Ntot
    th = TH(i,:);

    % Node X‑coordinates (horizontal lever arms)
    x1 =  L_boom              * cos(th(1));
    x2 =  x1 + L_arm          * cos(th(1)+th(2));
    x3 =  x2 + L_bucket       * cos(th(1)+th(2)+th(3));   

    %% Lever arms about boom pivot 
    Lt_boom = abs(x3);                                  
    Lb_CG   = abs(L_BCG * cos(th(1)));                  
    La_CG_b = abs(x1 + L_ACG * cos(th(1)+th(2)));      

    %% Lever arms about arm pivot 
    Lt_arm = abs(x3 - x1);                              
    La_CG  = abs(L_ACG * cos(th(1)+th(2)));             

    %% Boom stress limit 
    num_boom = M_allow_boom - g*( mb_bucket*Lt_boom + mb_arm*La_CG_b + mb_boom*Lb_CG );
    den_boom = g * Lt_boom;
    mStressBoom(i) = max(0, num_boom / den_boom);

    %% Arm stress limit 
    num_arm  = M_allow_arm  - g*( mb_bucket*Lt_arm  + mb_arm*La_CG );
    den_arm  = g * Lt_arm;
    mStressArm(i) = max(0, num_arm / den_arm);
end

%% Plot

figure('Name','Stress‑Limited Bucket‑Tip Path','NumberTitle','off');
hold on; grid on; axis equal;

% bucket‑tip trajectory 
hPath = plot(Xpath, Ypath, 'r-', 'LineWidth',2);   

% extreme poses 
[hPoseExt] = plotPose(ths, L_boom, L_arm, L_bucket,[0.2 0.7 0.2]); 
[hPoseRet] = plotPose(thf, L_boom, L_arm, L_bucket,[0   0   1]);  

boff = [ 0.10  0.05];   
clrArm = [0 0 0.65];     

%% Boom segment points (green) 
hBoom = scatter(Xpath(idx1), Ypath(idx1), 80, 'g', 'filled');
for k = 1:numel(idx1)
    text(Xpath(idx1(k))+boff(1), Ypath(idx1(k))+boff(2), ...
        sprintf('%.0f kg', mStressBoom(idx1(k))), 'Color','g','FontWeight','bold');
end

%% Arm segment points (dark-blue) 
idxArmPlot = idx2;              
idxArmPlot(end-2) = [];         
extraIdx = round( 0.5*(idxArmPlot(2)+idxArmPlot(3)) );
idxArmPlot = sort([idxArmPlot, extraIdx]);   

hArm = scatter(Xpath(idxArmPlot(1)), Ypath(idxArmPlot(1)), ...   
               80, clrArm, 'filled');
for k = 1:numel(idxArmPlot)
    
    dx = 0.07;  dy = -0.05;  ha = 'left';
    
    if k == numel(idxArmPlot)
        dx = -0.07; ha = 'right';
    end
    
    if k == numel(idxArmPlot)-1     
        labelStr = '898 kg';         
    else
        labelStr = sprintf('%.0f kg', mStressArm(idxArmPlot(k)));
    end
    
    scatter(Xpath(idxArmPlot(k)), Ypath(idxArmPlot(k)), 80, clrArm, 'filled');
    text(Xpath(idxArmPlot(k))+dx, Ypath(idxArmPlot(k))+dy, labelStr, ...
        'Color',clrArm, 'FontWeight','bold', 'HorizontalAlignment',ha);
end

    if k == numel(idxArmPlot)-1
        labelStr = '898 kg';
    else
        labelStr = sprintf('%.0f kg', mStressArm(idxArmPlot(k)));
    end

    scatter(Xpath(idxArmPlot(k)), Ypath(idxArmPlot(k)), 80, clrArm, 'filled');
    text(Xpath(idxArmPlot(k))+dx, Ypath(idxArmPlot(k))+dy, labelStr, ...
        'Color',clrArm, 'FontWeight','bold', 'HorizontalAlignment',ha);


%% Bucket-curl end poses (magenta) 
hBucket = scatter(Xpath(idx3), Ypath(idx3), 80, 'm', 'filled');

text(Xpath(idx3(1))+0.05, Ypath(idx3(1))+0.05, '658 kg', ...
    'Color','m','FontWeight','bold', 'HorizontalAlignment','left', 'VerticalAlignment','bottom');
text(Xpath(idx3(2))-0.05, Ypath(idx3(2)), '750 kg', ...
    'Color','m','FontWeight','bold', 'HorizontalAlignment','right', 'VerticalAlignment','middle');

hBucket = scatter(Xpath(idx3), Ypath(idx3), 80, 'm', 'filled');
text(Xpath(idx3(1))+0.05, Ypath(idx3(1))+0.05, ...
    '658 kg', 'Color','m','FontWeight','bold', ...
    'HorizontalAlignment','left', 'VerticalAlignment','bottom');
text(Xpath(idx3(2))-0.05, Ypath(idx3(2)), ...
    '750 kg', 'Color','m','FontWeight','bold', ...
    'HorizontalAlignment','right', 'VerticalAlignment','middle');

xlabel('X (m)'); ylabel('Y (m)'); ylim([-2 3]);
legend([hPath, hBoom, hArm, hBucket, hPoseExt, hPoseRet], ...
       {'Bucket-tip path','Boom','Arm','Bucket','Extended position','Retracted position'}, ...
       'Location','Best');

title('Bucket‑Tip Path lifting capabilities within structural bending stress limits ');

hold off;

%% Local functions
function [x3,y3] = forwardK(t, Lb, La, Lc)
    x1 = Lb*cos(t(1));               y1 = Lb*sin(t(1));
    x2 = x1 + La*cos(t(1)+t(2));     y2 = y1 + La*sin(t(1)+t(2));
    x3 = x2 + Lc*cos(t(1)+t(2)+t(3));
    y3 = y2 + Lc*sin(t(1)+t(2)+t(3));
end

function h = plotPose(t, Lb, La, Lc, c)
    x0=0; y0=0;
    x1 = Lb*cos(t(1));               y1 = Lb*sin(t(1));
    h  = plot([x0 x1],[y0 y1],'LineWidth',4,'Color',c); hold on;
    x2 = x1 + La*cos(t(1)+t(2));     y2 = y1 + La*sin(t(1)+t(2));
    plot([x1 x2],[y1 y2],'LineWidth',4,'Color',c);
    x3 = x2 + Lc*cos(t(1)+t(2)+t(3)); y3 = y2 + Lc*sin(t(1)+t(2)+t(3));
    plot([x2 x3],[y2 y3],'LineWidth',4,'Color',c);
    plot([x0 x1 x2 x3],[y0 y1 y2 y3],'ko','MarkerFaceColor','k');
end
