function [t,X] = slip(xd0,tdang,hObject)
  % matlab settings
  format compact
  % eom parameters
  p.mb    = 1;
  p.omega   = 50;
  p.rho = 0.12;
  p.g    = 9.81;
  p.b    = 20;
  % PARAMS TO CHANGE -------------------
  p.tdang = tdang;
  p.xd0 =xd0;
  % NO MORE CHANGE ---------------------
  % calculated
  p.ttd1 = 0;

  % initial conditions
  ic = [0, ... % first entry is mode
    p.rho, p.tdang, -0.8, 0.2, ... % q = (th1, r1, x, z)
    0, 0, p.xd0, 0];
  % modes: 0=aerial, 1=stance

  % simulation parameters
  tmax = 2;
  [t,X] = runSim(tmax, ic, p);
  % get rid of duplicate t's
  [t, index] = unique(t);
  X = X(index,:);

  % visualize data
  [r1, th1, x, z, dr1, dth1, dx, dz] = unpackX(X);
  
  handles = guidata(hObject);
  axes(handles.axes2);
  cla();
  %for k = 1:2
  %  h(k) = subplot(2,1,k);
  %end

  %subplot(h(1))
  plot(t, dx,'-')
  ylabel('xdot')

  %subplot(h(2))
  
  handles = guidata(hObject);
  axes(handles.axes3);
  cla();
  hold on
  plot(t, z,'-')
  plot([0,t(end)],p.rho*[1,1],'k--')
  ylabel('z')
  hold off

  % animate
  framedel = 0.02;
  slowmo = 0.1;
  tdraw = 0:framedel:t(end);
  Xdraw = interp1(t, X,tdraw);
  size(Xdraw)
  for i=1:numel(tdraw)
    handles = guidata(hObject);
    axes(handles.axes1);
    cla();
    hold all
    [r1, th1, x, z, dr1, dth1, dx, dz] = unpackX(Xdraw(i,:));
    % hip loc
    hip1 = [x,z];
    % toes
    toes1 = hip1 - r1 * [-sin(th1), cos(th1)];
    % body
    plot(hip1(1),hip1(2),'o','Color','black','markers',8,'MarkerFaceColor',[.5,.5,.5])
    % legs
    line([hip1(1),toes1(1)],[hip1(2),toes1(2)],'Color','black','Linewidth',2)
    % 
    line([-1,1],[0,0],'Color','black')
    % 
    text(-0.1,-0.3,['t = ',num2str(tdraw(i))])
    hold off
    axis equal
    xlim([-1,1]);
    ylim([-0.5,1]);
    drawnow
    pause(slowmo*framedel)
  end
end


function [r1, th1, x, z, dr1, dth1, dx, dz] = unpackX(X)
  if (numel(X) > 9)
    r1 = X(:,2); th1 = X(:,3); x = X(:,4); z = X(:,5);
    dr1 = X(:,6); dth1 = X(:,7); dx = X(:,8); dz = X(:,9);
  else
    r1 = X(2); th1 = X(3); x = X(4); z = X(5);
    dr1 = X(6); dth1 = X(7); dx = X(8); dz = X(9);
  end
end
% ==============================================================
% ENTER EOM
% ==============================================================
function Xd = dynamics(t,X,p)
  % account for hybrid state
  Xd = X;
  Xd(1) = 0;% \dot(mode) := 0
  Xd(2:5) = X(6:end);
  [r1, th1, x, z, dr1, dth1, dx, dz] = unpackX(X);
  mb = p.mb;g = p.g;

  % controls
  u = 0;

  % angs
  ang1 = atan2(-(r1 - p.rho)*p.omega, -dr1);

  % massless limbs
  if X(1) == 0
    % vh1 is in flight. rdot=0, thetadot
    Xd(7) = 0;%-10000*(th1) - 100*dth1;%ddth1
    Xd(6) = 0;%ddr1
  else
    % active damping controller
    u = -p.omega^2 *(r1 - p.rho);
  end
  % non-degenerate EoM from MMA
  if X(1) == 0 % cntF
    Xd(8:9) = [0,-p.g];

  elseif X(1) == 1 % stance
    % from MMA
    rhs = [dth1.^2.*r1+mb.^(-1).*u+(-1).*g.*cos(th1),r1.^(-1).*((-2).*dr1.* ...
  dth1+g.*sin(th1)),(-1).*mb.^(-1).*u.*sin(th1),(-1).*g+mb.^(-1).* ...
  u.*cos(th1),u.*sin(th1)];

    Xd(6:9) = rhs(1:4);
  end
end


% EDIT
% --------------------------------------------------------------
function [zeroCrossing,isterminal,direction] = events(~,X,p)
  [r1, th1, x, z, dr1, dth1, dx, dz] = unpackX(X);

  % hips touching ground
  hip1 = z;%1
  % toes touching ground
  td1 = hip1 - r1 * cos(th1);%3
  lo1 = r1 - p.rho;%5
  
  zeroCrossing = [hip1, td1, lo1];
  direction    = [-1, -1, 1];
  isterminal   = ones(size(zeroCrossing));
end

% --------------------------------------------------------------
function Xnew = reset(ev,t,X,p) 
  Xnew = X;
  % modes: 0=aerial, 1=stance
  [r1, th1, x, z, dr1, dth1, dx, dz] = unpackX(X);
  mb = p.mb;g = p.g;

  if ev==1 % fall
    Xnew(1) = -1; % indicate that sim should stop!
  elseif(ev == 2) % td1
    p.ttd1 = t;
    Xnew(1) = 1;
    % resolve velocity constraint
    velproj=[0,0,(-1).*sin(th1),cos(th1);0,0,(-1).*r1.^(-1).*cos(th1),(-1).* ...
  r1.^(-1).*sin(th1);0,0,1,0;0,0,0,1];
    Xnew(6:9) = (velproj * X(6:9)')';
  elseif(ev == 3) % lo1
    Xnew(1) = 0;
    % massless limbs: set rdot=0
    Xnew(6) = 0;%dr1
    Xnew(7) = 0;%dth1
    Xnew(2) = p.rho;
    % leg angle reset?
    Xnew(3) = p.tdang;
  end
end


% ==============================================================
% Simulate hybrid dynamical system
% ==============================================================
function [t,y] = runSim(tmax, ic, p)

  t = [];
  y = [];
  tLatest = 0;

  while(tLatest < tmax)

    tspan = [tLatest, tmax];
    
    % set up event detection
    options = odeset('Events', ...
      @(t,y) events(t,y,p), ...
      'MaxStep', 1e-2);
     
    % Run simulation
    % t  is time
    % y  is soln 
    % tE is time events occured
    % yE is soln at event time
    % iE is index of vanishing event fxn
    [tChart,yChart,tE,yE,iE]= ode45(...
      @(t,y) dynamics(t,y,p), ...
      tspan, ic, options); %#ok<ASGLU>
    
     % Record data
    t = [t; tChart]; %#ok<AGROW>
    y = [y; yChart]; %#ok<AGROW>  
    tLatest = t(end);
    
    if(~isempty(iE))
      % lots of time the ic is an event, in which case look at last
      % record initial conditions for next time
      ic2 = reset(iE(end),tE(end),y(end,:),p);
      % apply reset if this next segment is different from the current one
      if ic2(1) ~= y(end,1)
        ic = ic2;
        fprintf('event ')
        if iE(end)==1
          fprintf('FALL')
        elseif iE(end)==2
          fprintf('TD')
        elseif iE(end)==3
          fprintf('LO')
        end
        fprintf(' at %f\n', tE(end))
        if (ic(1) < 0)
          break;
        end
      else
        ic = y(end,:);
      end
    end
  end
end


