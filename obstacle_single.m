%% Simulation of reach-avoid problem with single obstacle
clear; clc;

%% Problem setup
% Setup and parameters
% geometry
par.w=[0.4;0.6];	% agent half-size, m
par.wo=[1.5;1];     % obstacle half-size, m
par.xo=[4;4];       % obstacle location, m
% desired controller
par.xgoal=[7;7];	% goal point, m
par.Kp=0.5;         % gain of desired controller, 1/s
par.umax=1;         % saturation limit of desired controller, m/s
% safety
par.filter='on';	% safety filter 'on', 'off'
par.kappa=2;        % smoothing parameter (inf for nonsmooth limit), 1/m
par.b=log(2);       % buffer
par.alpha=@(h)h;	% class-K function

% Simulation settings
t0=0;               % start time, s
tend=20;            % end time, s
dt=0.01;            % time step, s
t=t0:dt:tend;       % time, s

% Initial conditions
x0=[0;0];           % start point, m

% Plot settings
It=[t0,tend];	Lt='time, $t$ (s)';
Ix1=[0,8];      Lx1='position, $x_1$ (m)';
Ix2=[0,8];      Lx2='position, $x_2$ (m)';
Ih=[-1,3];      Lh='CBF, $h$ (m)';
Iu=[-0.2,1];    Lu='input, $u$ (m/s)';
purple=[170,0,170]/256;
orange=[255,170,0]/256;
black=[0,0,0];
blue=[0,0,1];
darkgreen=[0,170,0]/256;
darkred=[230,0,0]/256;

% Animation settings
animate=1;          % whether animation is done
Nt=10;              % one out of Nt frames is plotted

%% Simulation
% Simulate system
sol=ode45(@(t,x)rhs(x,par),[t0,tend],x0,odeset('RelTol',1e-6));
x=deval(sol,t);

% Evaluate solution
u=zeros(size(x));
ud=zeros(size(u));
h=zeros(1,length(t));
hi=zeros(4,length(t));
for kt=1:length(t)
    [u(:,kt),ud(:,kt),h(kt)]=k(x(:,kt),par);
    hi(:,kt)=CBFs(x(:,kt),par);
end
hc=splitapply(@(hi)hlogic(hi,'nonsmooth'),hi,1:size(hi,2));

% Evaluate superlevel sets
[xx1,xx2]=meshgrid(linspace(Ix1(1),Ix1(2),201),linspace(Ix2(1),Ix2(2),201));
hhc=arrayfun(@(x1,x2)hlogic(CBFs([x1;x2],par),'nonsmooth'),xx1,xx2);
hh=arrayfun(@(x1,x2)CBF([x1;x2],par),xx1,xx2);

%% Plot results
figure(1); clf;

% Trajectory
subplot(2,2,[1,3]); hold on; box on;
% start and goal
plot(x0(1),x0(2),'.','Color',blue,'Markersize',20,'DisplayName','Start');
plot(par.xgoal(1),par.xgoal(2),'.','Color',darkgreen,'Markersize',20,'DisplayName','Goal');
% CBF contours
contour(xx1,xx2,hhc,[0,0],'Color',black,'HandleVisibility','off');
contour(xx1,xx2,hh,[0,0],'Color',darkred,'HandleVisibility','off');
% obstacle and agent
rectangle('Position',[par.xo-par.wo;2*par.wo(1);2*par.wo(2)],'LineWidth',2);
rectangle('Position',[x(:,end)-par.w;2*par.w(1);2*par.w(2)],'LineWidth',2);
% trajectory
plot(x(1,:),x(2,:),'Color',purple,'LineWidth',2,'HandleVisibility','off');
PlotFinalize({Lx1,Lx2},[Ix1,Ix2]);
legend('Location','NW');

% CBF
subplot(2,2,2); hold on; box on;
plot(It,[0,0],'k','LineWidth',1,'HandleVisibility','off');
% individual CBFs
plot(t,hi,'--','Color',black,'LineWidth',1,'HandleVisibility','off');
% nonsmooth combined CBF
plot(t,hc,'Color',black,'LineWidth',2,'DisplayName','$h_{\rm c}$');
% smooth combined CBF
plot(t,h,'Color',darkred,'LineWidth',2,'DisplayName','$h$');
PlotFinalize({Lt,Lh},[It,Ih]);
pbaspect([2,1,1]);

% Control input
subplot(2,2,4); hold on; box on;
% desired input
plot(t,ud(1,:),'Color',darkgreen,'LineWidth',2,'DisplayName','$u_{1,{\rm d}}$');
plot(t,ud(2,:),'Color',orange,'LineWidth',2,'DisplayName','$u_{2,{\rm d}}$');
% actual input
plot(t,u(1,:),'Color',darkred,'LineWidth',2,'DisplayName','$u_1$');
plot(t,u(2,:),'Color',purple,'LineWidth',2,'DisplayName','$u_2$');
PlotFinalize({Lt,Lu},[It,Iu]);
pbaspect([2,1,1]);

%% Animate motion
if animate
% % Create a video file for animation
% videoname='obstacle_single';
% v=VideoWriter(videoname,'MPEG-4');
% v.Quality=100;
% open(v);

% Make the animation
figure(2); clf;
for kt=1:Nt:length(t)
    tic;
    drawnow;
    % start and goal
    plot(x0(1),x0(2),'.','Color',blue,'Markersize',20,'DisplayName','Start');
    hold on;
    plot(par.xgoal(1),par.xgoal(2),'.','Color',darkgreen,'Markersize',20,'DisplayName','Goal');
    % CBF contours
    contour(xx1,xx2,hhc,[0,0],'Color',black,'HandleVisibility','off');
    contour(xx1,xx2,hh,[0,0],'Color',darkred,'HandleVisibility','off');
    % obstacle and agent
    rectangle('Position',[par.xo-par.wo;2*par.wo(1);2*par.wo(2)],'LineWidth',2);
    rectangle('Position',[x(:,kt)-par.w;2*par.w(1);2*par.w(2)],'LineWidth',2);
    % trajectory
    plot(x(1,1:kt),x(2,1:kt),'Color',purple,'LineWidth',2,'HandleVisibility','off');
    plot(x(1,kt),x(2,kt),'.','Color',purple,'Markersize',20,'HandleVisibility','off');
    hold off;
    PlotFinalize({Lx1,Lx2},[Ix1,Ix2]);
    legend('Location','NW');

%     % Save the animation
%     frame=getframe(gcf);
%     for kframe=1:v.FrameRate/Nt
%         writeVideo(v,frame);
%     end

    T=toc;
    pause(Nt*dt-T); % time the plots according to the frame rate
end

% close(v);

end

%% Save results
resultname = ['obstacle_single_kappa_',num2str(par.kappa)];
% saveas(gcf,[resultname,'.fig']);
% saveas(gcf,[resultname,'.svg']);

%% Functions for dynamics
% System model
function [f,g] = sys(~,~)
    f = [0;0];
    g = eye(2);
end

% Right-hand side
function dxdt = rhs(x,par)
    [f,g] = sys(x,par);
    u = k(x,par);
    dxdt = f+g*u;
end

%% Functions for control
% Desired controller
function ud = kd(x,par)
    ud = par.Kp*(par.xgoal-x);
    if norm(ud)>par.umax
        ud = ud/norm(ud)*par.umax;
    end
end

% Individual CBF evaluation
function [hi,gradhi] = CBFs(x,par)
    hi = [[1,0]*(x-par.xo)-par.w(1)-par.wo(1);
          [0,1]*(x-par.xo)-par.w(2)-par.wo(2);
          [-1,0]*(x-par.xo)-par.w(1)-par.wo(1);
          [0,-1]*(x-par.xo)-par.w(2)-par.wo(2)];
	gradhi = [[1,0];
              [0,1];
              [-1,0];
              [0,-1]];
end

% Operations to combine CBFs
function h = rsum(hi)   % reciprocal sum
    h = 1/sum(1./hi);   
end
function [h,gradh] = gradrsum(hi,gradhi)	% gradient of reciprocal sum
    h = rsum(hi);
    gradh = (h^2./hi.^2).'*gradhi;
end

% Logic to combine CBFs
function h = hlogic(hi,type)
    if strcmp(type,'smooth')
        h_or=@sum;
        h_and=@rsum;
    else
        h_or=@max;
        h_and=@min;
    end
    h = h_or(hi);
end
function [h,gradh] = gradhlogic(hi,gradhi,type)
    h = hlogic(hi,type);
    if strcmp(type,'smooth')
        gradh = sum(gradhi);
    else
        idx = find(h==hi,1,'first');
        gradh = gradhi(idx,:);
    end
end

% Combined CBF evaluation
function [h,gradh] = CBF(x,par)
    [hi,gradhi] = CBFs(x,par);
    % nonsmooth CBF solution for reference
    if par.kappa==inf	
        [h,gradh] = gradhlogic(hi,gradhi,'nonsmooth');
    % smooth CBF combination
    else          
        Hi = exp(par.kappa*hi);
        gradHi = par.kappa*Hi.*gradhi;
        [H,gradH] = gradhlogic(Hi,gradHi,'smooth');
        h = log(H)/par.kappa - par.b/par.kappa;
        gradh = gradH/par.kappa/H;
    end
end

% Controller
function [u,ud,h] = k(x,par)
    % desired controller
    ud = kd(x,par);
    % safety filter
    switch par.filter
        case 'off'
            h = CBF(x,par);
            u = ud;
        case 'on'
            [h,gradh] = CBF(x,par);
            [f,g] = sys(x,par);
            Lfh = gradh*f;
            Lgh = gradh*g;
            u = ud + max(0,-Lfh-Lgh*ud-par.alpha(h))*Lgh.'/(Lgh*Lgh.');
    end
end

%% Finalize plot with axis labels, limits, legends
function PlotFinalize(axislabels,axislimits)
    axis(axislimits);
    pbaspect([1,1,1]);
    xlabel(axislabels{1},'Interpreter','latex');
    ylabel(axislabels{2},'Interpreter','latex');
    if length(axislabels)>2
        zlabel(axislabels{3},'Interpreter','latex');
    end
    set(gca,'TickLabelInterpreter','latex','FontSize',12);
    legend('Location','NE','Interpreter','latex','FontSize',14);
    if isempty(get(get(gca,'Legend'),'String'))
        legend off;
    end
end