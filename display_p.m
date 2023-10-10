tic
clear all
clc
close all
r=10;% maximum range
N=10;% 

%% step 1 --4.1.1 Deployment of beacon points
%number of sensor nods
S=100;                                            
%finding the coordinates for S sensor nodes
x(1)=1;
y(1)=1;
p(1,:)=[x(1),y(1)];
for i=2:N
    x(i)=(i-1)*1.5*r;
    h(i)=(i-1)*sqrt(3)*r;
    y(i)=h(i)^2-x(i)^2;
    p(i,:)=[x(i),y(i)];
end  
for i=1:S
    k=randi([1 N],1,1);
    P(i,:)=p(k,:);
end
disp(P)
% % % step 2-----    4.1.2 Energy consumption

%finding the Energy captured by all sensor nodes
theta=2*pi*rand;
X=r*cos(theta);
Y=r*sin(theta);
M=[X,Y];
sum=0;
for i=1:S
    for j=1:2
        a=sum+(M(1,j)-P(i,j))^2;
    end
    d(i)=sqrt(a);
end
d0=10;
n=4;
MEnergy=-10*n*log(d0);
for i=1:S
    Energy(i)=(MEnergy-10*n*log(d(i)/d0))/5;
end

%finding stongest energy and generating mapping circle
for i=1:S
    d(i)=d0*10^((MEnergy-Energy(i))/(10*n));
end
for i=1:S
    for j=1:2
        t(i,j)=(d(i))^2+M(1,j);
    end  
end
[v p]=min(d);
xth =x(p)+d(p)*cos(theta);
yth=y(p)+d(p)*sin(theta);

%% step 3------ anchor node location changing

%generate the cost function
AnchorNode

%finding minimum distance and angle
for k=1:359
    if(p==1)
        cf(k)=sqrt(abs(e3(k)-e2(k)));
    elseif(p==2)
        cf(k)=sqrt(abs(e3(k)-e1(k)));
    else
        cf(k)=sqrt(abs(e2(k)-e1(k)));
    end
      subplot(3,1,3);
    plot(k,cf(k),'r.');
    hold on
    grid on
    xlabel('angle in degrees')
    ylabel('Distance')
    title('Distance with respect to change in angle')
end

[CF ang]=min(cf);
plot(ang,CF,'go');
legend('Distance','minimum Distance')
theta=ang;
D(1,:)=[x(p)+d(p)*cos(theta),y(p)+d(p)*sin(theta)];
%results
fprintf('EnergyIs collected from all sensor nodes=')
disp(Energy)
fprintf('Distance of destination from all sensor nodes=')
disp(d)
fprintf('Minimum distance and angle=')
disp([CF theta])
fprintf('co-ordinates of the new location of the mobile user are=')
disp(D)
toc
clear
xmin=[0; 0];         % Set edges of region want to search in
xmax=[30;30];

Nsteps=500;            % Maximum number of steps to produce

% Next set the parameters of the SN:

lambda=0.1;  % Step size to take in chosen direction at each move
Ns=16;        % Number of points on circular pattern to sense
r=1;          % Sensing radius
xs=0*ones(2,Ns); % Initialize
Jo(:,1)=0*ones(Ns,1);
Jg(:,1)=0*ones(Ns,1);
J(:,1)=0*ones(Ns,1);
theta(:,1)=0*ones(Ns,1);
for m=2:Ns  % Compute the angles to be used around the circle
	theta(m,1)=theta(m-1,1)+(pi/180)*(360/Ns); 
end

% % % step 4------   4.1.3  free path planning


% Goal position of node
xgoal=[25; 25];

% source node position
x=[5; 5];

% Weighting parameters for planning 
w1=1;
w2=1.0000e-04;

% Allocate memory 

x(:,2:Nsteps)=0*ones(2,Nsteps-1);

% The obstacles:
figure(2)
clf
% Plot initial and final positions
plot(5,5,'s',25,25,'p','linewidth',5)
axis([0 30 0 30])
hold on
xlabel('x');
ylabel('y');
title('source (square) and destination (pentagon) positions');
hold on
% Plot obstacle positions (sets obstaclefunction)
plot(20,15,'o',8,10,'o',10,10,'o',12,10,'o',24,20,'o',18,20,'o')
hold off

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot the functions:

xx=0:31/100:30;   % For our function the range of values we are considering
yy=xx;

% Compute the obstacle and goal functions

for jj=1:length(xx)
	for ii=1:length(yy)
		zz(ii,jj)=obstaclefunction([xx(jj);yy(ii)],w1);
	end
end
for jj=1:length(xx)
	for ii=1:length(yy)
		zzz(ii,jj)=goalfunction([xx(jj);yy(ii)],xgoal,w2);
	end
end



figure(6)
clf
contour(xx,yy,zzz,25)
colormap(jet)
% Use next line for generating plots to put in black and white documents.
%colormap(gray);
xlabel('x');
ylabel('y');
title('Path estimation');
hold on
% Plot initial and final positions
plot(5,5,'s',25,25,'p','linewidth',5)
hold off

figure(7)
clf
contour(xx,yy,zz+zzz,50)
colormap(jet)
% Use next line for generating plots to put in black and white documents.
%colormap(gray);
xlabel('x');
ylabel('y');
title('Path along With RSSI signal');
hold on

% Plot initial and final positions
plot(5,5,'s',25,25,'p','linewidth',5)

hold off




