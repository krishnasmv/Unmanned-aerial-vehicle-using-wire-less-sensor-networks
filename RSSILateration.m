clc;
close all;
clear all;
NS = 100;                           % networkSize
M = 100;                            % number of unknown nodes
N = 10;                             % number of anchor nodes
U = NS*rand(M,2);                   % Positionns of unknown nodes
A = NS*rand(N,2);                   % Positionns ofanchor nodes
Dist = zeros(N, M);                 % True Distance
DistN = zeros(N, M);                % Noisy Distance
% Distance Calculation 
P0 = -30.446;
alfa = 2.835;
SD = 5.3;
Ntrail = 100;       % No of trails for computing Average RSSI
for m = 1 : M
    for n = 1 : N
            Dist(n,m) = sqrt((A(n,1)-U(m,1)).^2 + (A(n,2)-U(m,2)).^2);
            Pnm = P0-10*alfa*log10(Dist(n,m))-mean(random('norm',0,SD,Ntrail,1));
            DistN(n,m)=10^((P0-Pnm)/(10*alfa));
    end
end
%~~~~~~~~~~~~Least squares method for determination of unknown point coordinates~~~~~~~~~~~

p = zeros(N-1,2);
An = A(N,:);
for i=1:N-1
    p(i,:) = A(i,:) - An;
end
P=2*(p);

Xi = zeros(M,2);
Nt = 100; % Number of Trails
UN = zeros(M,2,Nt);
for j=1:Nt
    j
     for m = 1:M
         d = DistN(:,m);
         q = zeros(N-1,1);
        for i=1:(N-1)
            q(i)=d(N)^2-d(i)^2+A(i,1)^2-An(1)^2+A(i,2)^2-An(2)^2;
        end
            Xi(m,:)=((P'*P)\(P'*q))'; % Least Square method
     end
 UN (:,:,j)=Xi;
end

X = mean(UN,3);
LE = zeros(1,M);
SE = zeros(1,M);
for m=1:M
    LE(m)= sqrt(sum((X(m,:)-U(m,:)).^2));
    SE(m)= sum((X(m,:)-U(m,:)).^2);
end
ALE = mean(LE);
RMSE = sqrt(mean(SE));
[ALE RMSE]
plot(A(:,1),A(:,2),'r*','MarkerSize',8,'linewidth',2);
hold on;
plot(U(:,1),U(:,2),'ko','MarkerSize',6,'linewidth',2);
hold on;
plot(X(:,1),X(:,2),'b^','MarkerSize',6,'linewidth',2);
hold on;
plot([X(:,1) U(:,1)]',[X(:,2) U(:,2)]','-b','linewidth',1.5);
legend('Anchor locations','Node true location','Node estimated location')
axis([-0.1 1.1 -0.1 1.1]*NS)
xlabel('X-Position (in meters)');
ylabel('Y-Position (in meters)');
figure;
% % stem(1:M,LE,'-k','linewidth',2);
bar(LE);
axis([0 100 0 20])
xlabel('Node Index');
ylabel('Localization Error');

% hold on;
% plot(X(:,1),U(:,1),'-b','linewidth',1);
