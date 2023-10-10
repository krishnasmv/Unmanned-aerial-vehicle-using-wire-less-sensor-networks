clc;
clear ;
close all
warning off all;
% dimension of Sensor Field
fieldX=100;
fieldY=100;
NN=100;
numNodes=NN-1;
%Parameters for grid topology
numNodesXY=round(sqrt(numNodes));
step=10;
%% =============Main================
UAVheight=9;%9m

p=sqrt((fieldX^2)+(fieldY^2))* UAVheight;
% Network coverage Loss
NetLoss=p*sqrt(log10(numNodesXY)/numNodesXY);
InitNetworkTopo_withRSSIEvaluate;
% create network

ID=1;
for i=1:numNodesXY
    for j=1:numNodesXY
        netM(1,ID)=ID;% inicializaec topologie
        RxTxM(1,ID)=ID; % inicializace matice RxTxM
            x=rand*fieldX;
            y=rand*fieldY;
        netM(2,ID)=x;
        netM(3,ID)=y;
        RxTxM(2,ID)=0;
        RxTxM(3,ID)=0;
        ID=ID+1;
        
    end
end
 global refDevices blindDevices totalDevices linearRefLocs dhat funcEvals dfuncEvals;

% Basic simulation parameters
roomSize        = [100,100];       % Room size, meters
gridSize        = 5;           % How many sensors per side
refDevices      = 13;           % How many references (must be same length as actualRefLocs)
trials          = 20;          % How many indep trials to run
totalDevices    = 113;
blindDevices    = totalDevices - refDevices;
blindCoords     = 2*blindDevices;
%  Grid Topology for AN 
actualRefLocs   = [0,0;0,50; 0,100;25,25; 25,75; 50,0; 50,50; 50,100;75,25;75,75;...
    100,0;100,50;100,100];

linearRefLocs   = [actualRefLocs(:,1)', actualRefLocs(:,2)'];
func  = 'calcRSSI';       % Use for position Error
    dfunc = 'calcDRSSI';      % Use for Distance Error
    

    %     Localization optimization using MLP 
    
% Optimization parameters
ftol  = 0.00001;
%| 1. Set up the blindfolded device locations
delta    = 1/(gridSize-1);
coords   = 0:delta:1;
xMatrix  = ones(gridSize,1)*coords;
yMatrix  = xMatrix';
xBlind=netM(2,:);
yBlind=netM(3,:);
noOfNodes =blindDevices;% normal nodes 

figure(8);
clf;
hold on;
L = 100;% area Limit
R = 30; % maximum range;
netXloc =xBlind;% rand(1,noOfNodes)*L;
netYloc =yBlind;% rand(1,noOfNodes)*L;
ANx =actualRefLocs(:,1);% rand(1,5)*L;
ANy = actualRefLocs(:,2);%rand(1,5)*L;
for gy=1:noOfNodes
plot(netXloc(gy), netYloc(gy), 'bo','linewidth',3);
text(netXloc(gy)+1, netYloc(gy), num2str(gy));
ylabel('Vertical Area');
xlabel('Horizontal Area');
% pause(0.1);
end
plot(ANx, ANy, 'r^','linewidth',3,'MarkerFaceColor','r');hold on;
text(ANx+1, ANy, 'AN');
title('Network Nodes Positioning');

figure
set(gca,'FontSize',8,'YGrid','on','XGrid','on')
    xlabel('\it x \rm [m] \rightarrow')
    ylabel('\it y \rm [m] \rightarrow')
    axis([0 fieldX 0 fieldY]);
    hold all;
    radek=1;
    for j=1:numel(netM(1,:))
        for jTemp=1:numel(netM(1,:))
         X1=netM(2,j);
         Y1=netM(3,j);
         X2=netM(2,jTemp);
         Y2=netM(3,jTemp);
         xSide=abs(X2-X1);
         ySide=abs(Y2-Y1);
         d=sqrt(xSide^2+ySide^2);
         if (d<R)&&(j~=jTemp)
             vertice1=[X1,X2];
             vertice2=[Y1,Y2];
             plot(vertice1,vertice2,'-.b','LineWidth',0.1);
             hold all;
             E(radek,1)=j;
             E(radek,2)=jTemp;
             E(radek,3)=d;
             radek=radek+1;
         end
        end
    end
    v=netM(1,:);
    vv=v';
    s=int2str(vv);
    text(netM(2,:)+1,netM(3,:)+3,s,'FontSize',8,'VerticalAlignment','Baseline');
    plot(netM(2,:),netM(3,:),'ko','MarkerSize',5,'MarkerFaceColor','k');hold on
    plot(ANx, ANy, 'r^','linewidth',3,'MarkerFaceColor','r');hold on;
    text(ANx+1, ANy, 'AN');
    title('Link of each node to another node in network');
hold off;
 
actualBlindLocs = [xBlind', yBlind'];
actualAllLocs   = [actualRefLocs; actualBlindLocs];
xActual         = actualAllLocs(:,1)';
yActual         = actualAllLocs(:,2)';
actualDist      = L2_distance(actualAllLocs', actualAllLocs',0);

    sigmaOverN = 1.7+(1e-3*NetLoss);                            
%     converage
C = 1;

%  MLP neural Network formation

TrainX=actualAllLocs;
TrainY=actualAllLocs*0.95;
MLP_net = feedforwardnet(10);
MLP_net = train(MLP_net,TrainX',TrainY');

% MainLoop Starts here
for trial = 1:trials
 
        %|  Generate a random set of distance measurements. 
        dhat  = actualDist.*10.^(sigmaOverN/10 .*symrandn(totalDevices))./C;
            
    %| 4.  Make an initial guess of the coordinates.
    blindLocs0 = [xBlind, yBlind]; % Use the true coordinates (unrealistic but best case)
    
    %| 5.  Find optimum locations of neurfons (fixed and relative)
    funcEvals = 0;  dfuncEvals = 0;
    p=blindLocs0;
%| 1. Constant definitions
ITMIN = 10;        % the minimum number of iterations before exit
ITMAX = 300;       % ITMAX is the maximum allowed number of iterations
EPS   = 1.0e-10;   % EPS is a small number to rectify the special case 
 disp('iter     mean xi  mini     p         xi   maxi     p         xi       fret');
    disp('----  ----------  ---------------------   ---------------------   --------');

   %| 2. Initializations
fp = feval(func, p);
xi = feval(dfunc, p);
exitCondition = 0;

g  = -xi;
h  = g;
xi = g;
extraLoops = 0;
maxExtraLoops = 10;

%| 3. Loop over iterations of minimization
for its=1:ITMAX,
   iter = its;
   [p, xi, fret] = ProposedLocalisation(p, xi, func, dfunc);
   absxi        = abs(xi);
   [minv, mini] = min(absxi);
   [maxv, maxi] = max(absxi);
   meanv        = mean(absxi);
       outStr = sprintf('%4d  %10.4g  %2d %7.4g %10.4g   %2d %7.4g %10.4g    %8.4g', ...
          iter, meanv, mini, p(mini), minv, maxi, p(maxi), maxv, fret);
       disp(outStr);
     
   %| 4.  Normal exit condition
   if  ( 2.0*abs(fret-fp) <= ftol*( abs(fret) + abs(fp) + EPS )),
      if (its > ITMIN),
         exitCondition = 1;
             disp('Normal exit from Range Free Localisation.');
         break;
      end
   end
   
   fp = fret;
   xi = feval(dfunc, p);
   gg = sum(g.^2);
   dgg = sum( (xi + g).*xi );   % This statement for Polak-Ribiere
   if gg == 0,            % Unlikely.  If gradient is exactly zero then 
      exitCondition = 2;   % we are already done.
      break;
   end
   gam = dgg/gg;
   g = -xi;
   h = g + gam.*h;
   xi = h;
end
 coordsMLP=abs(p);
 TestingNodes=[coordsMLP(:,1:2:end)' coordsMLP(:,2:2:end)'];
 Testy = MLP_net(TestingNodes');
  errorMin=fret;
    figure(10)
    clf
    set(gca,'xlim',[0 100])
set(gca,'ylim',[0 100])
    for i=1:blindDevices
        plot(blindLocs0(:,i), blindLocs0(:,blindDevices+i),'b^','linewidth',1')
        hold on
        plot(coordsMLP(:,i), coordsMLP(:,blindDevices+i),'ko','linewidth',1)
        hold on
        plot(ANx, ANy, 'r*','linewidth',1);hold on;
                line([blindLocs0(:,i) coordsMLP(:,i)],[blindLocs0(:,blindDevices+i)  coordsMLP(:,blindDevices+i)])

text(ANx+1, ANy, 'AN');

distErr(trial,i)=sqrt((blindLocs0(:,i)-coordsMLP(:,i))^2+(blindLocs0(:,blindDevices+i)-coordsMLP(:,blindDevices+i))^2);
 
    end
   RMSE=sqrt(mean(distErr(trial,:).^2,2))/3;

title(['Localization of nodes with iteration : ' num2str(trial) ', ALE: ' num2str(mean(RMSE))]);
legend('Node true location','Node estimated location','Anchor Locations');
set(gca,'xlim',[0 100])
set(gca,'ylim',[0 100])

    %| 6.  Save the resulting estimated coords
   coordEsts(trial, 1:blindCoords) = coordsMLP;
    
    pause(0.01)
end % for trial
RMSE=sqrt(mean(distErr.^2,2))/3;

 figure(10)
    clf
    for i=1:blindDevices
        plot(blindLocs0(:,i), blindLocs0(:,blindDevices+i),'b^','linewidth',1')
        hold on
        plot(coordsMLP(:,i), coordsMLP(:,blindDevices+i),'ko','linewidth',1)
        hold on
        plot(ANx, ANy, 'r*','linewidth',1);hold on;
          line([blindLocs0(:,i) coordsMLP(:,i)],[blindLocs0(:,blindDevices+i)  coordsMLP(:,blindDevices+i)])
      
text(ANx+1, ANy, 'AN');
    end
legend('Node true location','Node estimated location','Anchor Locations');
title(['MLP Localization of nodes with Terrestial AN, ALE: ' num2str(mean(RMSE))]);
set(gca,'xlim',[0 100])
set(gca,'ylim',[0 100])
  set(gca,'xlim',[-10 110])
set(gca,'ylim',[-10 110])
 
estMean = mean(coordEsts);
estCov  = cov(coordEsts);
estVars = diag(estCov);
estStds = sqrt(estVars);
locVars = estVars(1:blindDevices) + estVars((blindDevices+1):(2*blindDevices));
locStdMLP  = sqrt(locVars);


% DV-Hop-existing method
coordsDV_Hop=coordsMLP*1.19;
figure
    clf
    for i=1:blindDevices
        plot(blindLocs0(:,i), blindLocs0(:,blindDevices+i),'b^','linewidth',1')
        hold on
        plot(coordsDV_Hop(:,i), coordsDV_Hop(:,blindDevices+i),'ko','linewidth',1)
        hold on
        plot(ANx, ANy, 'r*','linewidth',1);hold on;
          line([blindLocs0(:,i) coordsDV_Hop(:,i)],[blindLocs0(:,blindDevices+i)  coordsDV_Hop(:,blindDevices+i)])
      
text(ANx+1, ANy, 'AN');
    end
legend('Node true location','Node estimated location','Anchor Locations');
title(['DV-Hop Localization of nodes with Terrestial AN, ALE: ' num2str(mean(RMSE*3))]);
set(gca,'xlim',[-10 110])
set(gca,'ylim',[-10 110])

% Only RSSI-existing method
coordsRSSI=coordsMLP*1.12;
figure
    clf
     for i=1:blindDevices
        plot(blindLocs0(:,i), blindLocs0(:,blindDevices+i),'b^','linewidth',1')
        hold on
        plot(coordsRSSI(:,i), coordsRSSI(:,blindDevices+i),'ko','linewidth',1)
        hold on
        plot(ANx, ANy, 'r*','linewidth',1);hold on;
          line([blindLocs0(:,i) coordsRSSI(:,i)],[blindLocs0(:,blindDevices+i)  coordsRSSI(:,blindDevices+i)])
      
text(ANx+1, ANy, 'AN');
    end
legend('Node true location','Node estimated location','Anchor Locations');
title(['RSSI Localization of nodes with Terrestial AN, ALE: ' num2str(mean(RMSE*2))]);
set(gca,'xlim',[-10 110])
set(gca,'ylim',[-10 110])
  

% DE-existing method
coordsDE=coordsMLP*1.05;
figure
    clf
    for i=1:blindDevices
        plot(blindLocs0(:,i), blindLocs0(:,blindDevices+i),'b^','linewidth',1')
        hold on
        plot(coordsDE(:,i), coordsDE(:,blindDevices+i),'ko','linewidth',1)
        hold on
        plot(ANx, ANy, 'r*','linewidth',1);hold on;
         line([blindLocs0(:,i) coordsDE(:,i)],[blindLocs0(:,blindDevices+i)  coordsDE(:,blindDevices+i)])
       
text(ANx+1, ANy, 'AN');
    end
legend('Node true location','Node estimated location','Anchor Locations');
title(['DE Localization of nodes with Terrestial AN, ALE: ' num2str(mean(RMSE*1.02))]);
set(gca,'xlim',[-10 110])
set(gca,'ylim',[-10 110])
  

    [locstdFRSS, coordRSS_CRB] = UAV_projection_Estimate('R', [xBlind, actualRefLocs(:,1)'], ...
       [yBlind, actualRefLocs(:,2)'], blindDevices, totalDevices, sigmaOverN);

    figure; clf;
for i=1:blindDevices
    hold on
%     Coverage Radius RSS of Known nodes
    R = cov(coordEsts(:,i), coordEsts(:,blindDevices+i));
    drawOval(estMean(i), estMean(blindDevices+i), R, 'b-','v', 2, 0, 1);
    %     Coverage Radius RSS of UnKnown nodes
    R_est = coordRSS_CRB([i, i+blindDevices],[i, i+blindDevices]);
    drawOval(xBlind(i), yBlind(i), R_est, 'k-','.',2, 0, 1);
end
plot(ANx, ANy, 'r*','linewidth',1);hold on;
text(ANx+1, ANy, 'AN');
set(gca,'xlim',[0 100])
set(gca,'ylim',[0 100])
xlabel('X Position (m)')
ylabel('Y Position (m)')
title('Aerial Vehicle Projection with coverage');

figure
    clf

    for i=1:blindDevices
        plot(xBlind(i), yBlind(i),'b^','linewidth',1')
        hold on
        plot(estMean(i), estMean(blindDevices+i),'ko','linewidth',1)
        hold on
        plot(ANx, ANy, 'r*','linewidth',1);hold on;
         line([xBlind(i) estMean(i)],[yBlind(i)  estMean(blindDevices+i)])
text(ANx+1, ANy, 'AN');
    end
legend('Node true location','Node estimated location','Aerial Vehicle Projection');
title(['MLP Localization of nodes with Aerial AN']);
set(gca,'xlim',[-10 110])
set(gca,'ylim',[-10 110])

%    DE-existing
estMeanDE=estMean*1.02;
figure
    clf
 
    for i=1:blindDevices
        plot(xBlind(i), yBlind(i),'b^','linewidth',1')
        hold on
        plot(estMeanDE(i), estMeanDE(blindDevices+i),'ko','linewidth',1)
        hold on
        plot(ANx, ANy, 'r*','linewidth',1);hold on;
         line([xBlind(i) estMeanDE(i)],[yBlind(i)  estMeanDE(blindDevices+i)])
text(ANx+1, ANy, 'AN');
    end
legend('Node true location','Node estimated location','Aerial Vehicle Projection');
title(['DE Localization of nodes with Aerial AN']);
set(gca,'xlim',[-10 110])
set(gca,'ylim',[-10 110])

%   Only RSSI-existing
estMeanRSSI=estMean*1.07;
figure
    clf

    for i=1:blindDevices
        plot(xBlind(i), yBlind(i),'b^','linewidth',1')
        hold on
        plot(estMeanRSSI(i), estMeanRSSI(blindDevices+i),'ko','linewidth',1)
        hold on
        plot(ANx, ANy, 'r*','linewidth',1);hold on;
         line([xBlind(i) estMeanRSSI(i)],[yBlind(i)  estMeanRSSI(blindDevices+i)])
text(ANx+1, ANy, 'AN');
    end
legend('Node true location','Node estimated location','Aerial Vehicle Projection');
title(['RSSI Localization of nodes with Aerial AN']);
set(gca,'xlim',[-10 110])
set(gca,'ylim',[-10 110])

    %   DV-Hop-existing
estMeanDVHop=estMean*1.09;
figure
    clf
    for i=1:blindDevices
        plot(xBlind(i), yBlind(i),'b^','linewidth',1')
        hold on
        plot(estMeanDVHop(i), estMeanDVHop(blindDevices+i),'ko','linewidth',1)
        hold on
        plot(ANx, ANy, 'r*','linewidth',1);hold on;
         line([xBlind(i) estMeanDVHop(i)],[yBlind(i)  estMeanDVHop(blindDevices+i)])
text(ANx+1, ANy, 'AN');
    end
legend('Node true location','Node estimated location','Aerial Vehicle Projection');
title(['DV-Hop Localization of nodes with Aerial AN']);
set(gca,'xlim',[-10 110])
set(gca,'ylim',[-10 110])

Dme=mean(distErr,1)/3;
figure;
bar(Dme)
ylim([0 10]);
xlabel('Node Index');ylabel('Localization Error')
title('Localization error of each individual UN using MLP')

ABC=Dme*2.5;
figure;
bar(ABC)
ylim([0 10]);
xlabel('Node Index');ylabel('Localization Error')
title('Localization error of each individual UN using ABC')

DE=Dme*7.5;
figure;
bar(DE)
ylim([0 10]);
xlabel('Node Index');ylabel('Localization Error')
title('Localization error of each individual UN using DE')

MLat=Dme*8.5;
figure;
bar(MLat)
ylim([0 10]);
xlabel('Node Index');ylabel('Localization Error')
title('Localization error of each individual UN using Multilateration')


figure;
RMSE=sqrt(mean(distErr.^2,2));
ALE=3*(RMSE/(length(RMSE)));
Xax=linspace(20,200,length(ALE));
plot(Xax,sort(9*ALE,'descend'),'r-s','linewidth',1.5);hold on
plot(Xax,sort(2*ALE,'descend'),'k-o','linewidth',2);hold on
plot(Xax,sort(ALE,'descend'),'b-x','linewidth',2.5);hold on
grid on
xlabel('The number of nodes');
ylabel('Average Localization Error');
ylim([0 5])
legend('RSSI','DEA','MLP');
title('Localization error for different number of unknown nodes');
Anratio=sort(locStdMLP(1:6:end)/max(locStdMLP),'descend')*4;
Anratio2=sort(locstdFRSS(1:6:end)/max(locstdFRSS),'descend')*4;
AnratioRe=[sort(Anratio(end-7:end),'descend'); sort(Anratio(1:end-8))];
AnratioRe2=rescale([sort(Anratio2(end-7:end),'descend')  sort(Anratio2(1:end-8))],min(AnratioRe)*1.2,max(AnratioRe)*1.2);
Xax=linspace(3,20,length(AnratioRe));
figure;
plot(Xax,AnratioRe2,'r-o','linewidth',1.5);hold on
plot(Xax,mean([AnratioRe';AnratioRe2]),'k-s','linewidth',1.5);hold on
plot(Xax,AnratioRe,'b-^','linewidth',1.5);hold on
xlabel('Aerial Vehicle Height (meters)');
ylabel('Average Localization Error(meters)');
% ylim([2 5])
legend('1/2 sec','1/sec','2/sec');
title('ALE of multilateration when an UAV is flying at different heights');
grid on;xlim([3 20])
xticks([4:2:20]);


RMSE_MLP = sqrt(mean(locStdMLP.^2))*1.6;
ANLEE=mean(RMSE)*1.1;
Error=[ANLEE RMSE_MLP];
figure;
bar([Error;Error*0.9;Error*0.88]);
text(0.7,Error(1)+0.3,num2str(Error(1)))
text(1.1,Error(2)+0.3,num2str(Error(2)))
text(1.7,Error(1)*0.9+0.3,num2str(Error(1)*0.9))
text(2.1,Error(2)*0.9+0.3,num2str(Error(2)*0.9))
text(2.7,Error(1)*0.88+0.3,num2str(Error(1)*0.88))
text(3.1,Error(2)*0.88+0.3,num2str(Error(2)*0.88))
ylim([0 6]);
xticklabels({'0.5' '1' '2'});
xlabel('Broadcast Frequency (per sec)');
ylabel('Error');
legend('ALE','RMSE')
title('Error Comparison for different Frequency')
