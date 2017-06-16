function [t,X] = vh2body(pk,pd,tmax,hObject)
  % matlab settings
  format compact
  % eom parameters
  p.mb    = 5;
  p.omega   = 50;
  p.rho = 0.2;
  p.g    = 9.81;
  p.b    = 20;
  % PARAMS TO CHANGE -------------------
  p.d    = pd; % half of body length
  p.k =  pk;
  % NO MORE CHANGE ---------------------
  % calculated
  p.ib    = 1/8 * p.mb * p.d^2; % somewhere between 1/12 (uniform) and 1/4 (mass at ends)
  p.ttd1 = 0;
  p.ttd2 = 0;

  % initial conditions
  ic = [0, ... % first entry is mode
    0.1, p.rho, 0.1, p.rho, 0, -0.3, 0.2, ... % q = (th1, r1, th2, r2, x, z, phi)
    0, 0, 0, 0, 0, 0, 0];
  % modes: 0=aerial, 1=stance1, 2=stance2, 3=double stance

  % simulation parameters
  %tmax = 2;
  [t,X] = runSim(tmax, ic, p);
  % get rid of duplicate t's
  [t, index] = unique(t);
  X = X(index,:);

  % visualize data
  [th1, r1, th2, r2, x, z, phi, dth1, dr1, dth2, dr2, dx, dz, dphi] = unpackX(X);
  
  
%   figure(1)
%   %get(handles)
% 
%   clf
%   for k = 1:6
%     h(k) = subplot(3,2,k);
%   end
% 
%   subplot(h(1))
%   
    handles = guidata(hObject);
    axes(handles.axes4);
    cla();
    plot(t, X(:,1))
    title('Hybrid Mode');
% 
%   subplot(h(2))
%   hold on
%   plot(t, r1,'-')
%   plot(t, r2,'-')
%   plot([0,t(end)],p.rho*[1,1],'k--')
%   ylabel('r')
%   hold off
% 
%   subplot(h(3))
%   hold on
%   plot(t, atan2(-(r1 - p.rho)/p.omega, -dr1),'-')
%   plot(t, atan2(-(r2 - p.rho)/p.omega, -dr2),'-')
%   ylabel('ang')
%   hold off
% 
%   subplot(h(4))
%   plot(t, x,'-')
%   ylabel('x')

  %subplot(h(5))
  handles = guidata(hObject);
  axes(handles.axes2);
  cla();
  hold on
  plot(t, -z,'-')
  plot([0,t(end)],p.rho*[1,1],'k--')
  title('Z');
  ylabel('meters');
  hold off

%   subplot(h(6))
%   hold on
   handles = guidata(hObject);
   axes(handles.axes3);
   cla();
   hold on;
   plot(t, phi,'-')
   plot([0,t(end)],[0,0],'k--')
   hold off
   title('Pitch');
   ylabel('rads')

  % animate
  framedel = 0.01;
  slowmo = 0.5;
  tdraw = 0:framedel:t(end);
  Xdraw = interp1(t, X,tdraw);
  size(Xdraw)
  for i=1:numel(tdraw)
    
    %figure(2)

    %clf
    hold all
    [th1, r1, th2, r2, x, z, phi, dth1, dr1, dth2, dr2, dx, dz, dphi] = unpackX(Xdraw(i,:));
    % hip loc
    hip1 = [-x,-z] + p.d * [cos(phi),-sin(phi)];
    hip2 = [-x,-z] - p.d * [cos(phi),-sin(phi)];
    % toes
    toes1 = hip1 - r1 * [-sin(th1 - phi), cos(th1 - phi)];
    toes2 = hip2 - r2 * [-sin(th2 - phi), cos(th2 - phi)];
    % body
    handles = guidata(hObject);
    axes(handles.axes1);
    cla();
    line([hip1(1),hip2(1)],[hip1(2),hip2(2)],'Color',[.5,.5,.5],'Linewidth',12);
    
    % legs
    line([hip1(1),toes1(1)],[hip1(2),toes1(2)],'Color','black','Linewidth',2);
    line([hip2(1),toes2(1)],[hip2(2),toes2(2)],'Color','black','Linewidth',2);
    % 
    line([-1,1],[0,0])
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


function [th1, r1, th2, r2, x, z, phi, dth1, dr1, dth2, dr2, dx, dz, dphi] = unpackX(X)
  if (numel(X) > 15)
    th1 = X(:,2); r1 = X(:,3); th2 = X(:,4); r2 = X(:,5); x = X(:,6); z = X(:,7); phi = X(:,8);
    dth1 = X(:,9); dr1 = X(:,10); dth2 = X(:,11); dr2 = X(:,12); dx = X(:,13); dz = X(:,14); dphi = X(:,15);
  else
    th1 = X(2); r1 = X(3); th2 = X(4); r2 = X(5); x = X(6); z = X(7); phi = X(8);
    dth1 = X(9); dr1 = X(10); dth2 = X(11); dr2 = X(12); dx = X(13); dz = X(14); dphi = X(15);
  end
end
% ==============================================================
% ENTER EOM
% ==============================================================
function Xd = dynamics(t,X,p)
  % account for hybrid state
  Xd = X;
  Xd(1) = 0;% \dot(mode) := 0
  Xd(2:8) = X(9:end);
  [th1, r1, th2, r2, x, z, phi, dth1, dr1, dth2, dr2, dx, dz, dphi] = unpackX(X);
  mb = p.mb;ib = p.ib;d = p.d;g = p.g;

  % controls
  uth1 = 0;
  uth2 = 0;
  u1 = 0;
  u2 = 0;

  % angs
  ang1 = atan2(-(r1 - p.rho)*p.omega, -dr1);
  ang2 = atan2(-(r2 - p.rho)*p.omega, -dr2);

  % massless limbs
  if X(1) == 0 || X(1) == 2
    % vh1 is in flight. rdot=0, thetadot
    Xd(9) = -10000*(th1 - phi) - 100*dth1;%ddth1
    Xd(10) = 0;%ddr1
  else
    % active damping controller
    u1 = -p.omega^2 *(r1 - p.rho) - p.b * dr1 - p.k * cos(ang1);
    if (t - p.ttd1 > 0.001)
      % small restoring force to keep vertical (not implementing fore aft control)
      uth1 = -2*(th1-phi) - 1*(dth1 - dphi);
    end
  end
  if X(1) == 0 || X(1) == 1
    % vh2 is in flight
    Xd(11) = -10000*(th2 - phi) - 100*dth2;%ddth2
    Xd(12) = 0;%ddr2
  else
    % active damping controller
    u2 = -p.omega^2 *(r2 - p.rho) - p.b * dr2 - p.k * cos(ang2);
    if (t - p.ttd2 > 0.001)
      % small restoring force to keep vertical (not implementing fore aft control)
      uth2 = -2*(th2-phi) - 1*(dth2 - dphi);
    end
  end
  % non-degenerate EoM from MMA
  if X(1) == 0 % cntF
    Xd(13:15) = [0,p.g,0];

  elseif X(1) == 1 % cntS1
    % from MMA
    rhs = [(1/2).*ib.^(-1).*mb.^(-1).*r1.^(-2).*(4.*dphi.*dr1.*ib.*mb.*r1+( ...
  -4).*dr1.*dth1.*ib.*mb.*r1+2.*ib.*uth1+d.^2.*mb.*uth1+2.*mb.* ...
  r1.^2.*uth1+(-1).*d.^2.*mb.*uth1.*cos(2.*th1)+(-2).*g.*ib.*mb.* ...
  r1.*sin(phi+(-1).*th1)+4.*d.*mb.*r1.*uth1.*sin(th1)+(-2).*d.*mb.* ...
  r1.*cos(th1).*((-1).*dphi.^2.*ib+r1.*u1+d.*u1.*sin(th1))),(-2).* ...
  dphi.*dth1.*r1+dth1.^2.*r1+(-1).*g.*cos(phi+(-1).*th1)+(1/2).* ...
  ib.^(-1).*mb.^(-1).*u1.*(2.*ib+d.^2.*mb+d.^2.*mb.*cos(2.*th1))+ ...
  dphi.^2.*(r1+d.*sin(th1))+(-1).*d.*ib.^(-1).*r1.^(-1).*uth1.*cos( ...
  th1).*(r1+d.*sin(th1)),mb.^(-1).*r1.^(-1).*(uth1.*cos(phi+(-1).* ...
  th1)+(-1).*r1.*u1.*sin(phi+(-1).*th1)),g+(-1).*mb.^(-1).*u1.*cos( ...
  phi+(-1).*th1)+(-1).*mb.^(-1).*r1.^(-1).*uth1.*sin(phi+(-1).*th1), ...
  ib.^(-1).*r1.^(-1).*(r1.*uth1+(-1).*d.*r1.*u1.*cos(th1)+d.*uth1.* ...
  sin(th1)),r1.^(-1).*uth1.*cos(phi+(-1).*th1)+(-1).*u1.*sin(phi+( ...
  -1).*th1),(-1).*r1.^(-1).*(r1.*u1.*cos(phi+(-1).*th1)+uth1.*sin( ...
  phi+(-1).*th1))];

    Xd(9:10) = rhs(1:2);
    Xd(13:15) = rhs(3:5);

  elseif X(1) == 2 % cntS2
    rhs = [r2.^(-2).*((-1).*r2.*(2.*dr2.*((-1).*dphi+dth2)+d.*dphi.^2.*cos( ...
  th2))+(-1).*g.*r2.*sin(phi+(-1).*th2)+d.*ib.^(-1).*r2.*u2.*cos( ...
  th2).*(r2+(-1).*d.*sin(th2))+ib.^(-1).*mb.^(-1).*uth2.*(ib+(1/2).* ...
  d.^2.*mb+mb.*r2.^2+(-1/2).*d.^2.*mb.*cos(2.*th2)+(-2).*d.*mb.*r2.* ...
  sin(th2))),(-2).*dphi.*dth2.*r2+dth2.^2.*r2+(-1).*g.*cos(phi+(-1) ...
  .*th2)+(1/2).*ib.^(-1).*mb.^(-1).*u2.*(2.*ib+d.^2.*mb+d.^2.*mb.* ...
  cos(2.*th2))+dphi.^2.*(r2+(-1).*d.*sin(th2))+d.*ib.^(-1).*r2.^(-1) ...
  .*uth2.*cos(th2).*(r2+(-1).*d.*sin(th2)),mb.^(-1).*r2.^(-1).*( ...
  uth2.*cos(phi+(-1).*th2)+(-1).*r2.*u2.*sin(phi+(-1).*th2)),g+(-1) ...
  .*mb.^(-1).*u2.*cos(phi+(-1).*th2)+(-1).*mb.^(-1).*r2.^(-1).* ...
  uth2.*sin(phi+(-1).*th2),ib.^(-1).*r2.^(-1).*(r2.*uth2+d.*r2.*u2.* ...
  cos(th2)+(-1).*d.*uth2.*sin(th2)),r2.^(-1).*uth2.*cos(phi+(-1).* ...
  th2)+(-1).*u2.*sin(phi+(-1).*th2),(-1).*r2.^(-1).*(r2.*u2.*cos( ...
  phi+(-1).*th2)+uth2.*sin(phi+(-1).*th2))];

    Xd(11:15) = rhs(1:5);

  elseif X(1) == 3 % cntSD
    rhs = [r1.^(-2).*(r1.*(2.*dr1.*(dphi+(-1).*dth1)+d.*dphi.^2.*cos(th1))+( ...
  -1).*g.*r1.*sin(phi+(-1).*th1)+(-1).*d.*ib.^(-1).*r1.*u1.*cos(th1) ...
  .*(r1+d.*sin(th1))+ib.^(-1).*mb.^(-1).*uth1.*(ib+(1/2).*d.^2.*mb+ ...
  mb.*r1.^2+(-1/2).*d.^2.*mb.*cos(2.*th1)+2.*d.*mb.*r1.*sin(th1))+ ...
  ib.^(-1).*mb.^(-1).*r1.*u2.*(cos(th2).*(d.*mb.*r1+((-1).*ib+d.^2.* ...
  mb).*sin(th1))+ib.*cos(th1).*sin(th2))+ib.^(-1).*mb.^(-1).*r1.* ...
  r2.^(-1).*uth2.*(ib.*cos(th1).*cos(th2)+mb.*r1.*(r2+(-1).*d.*sin( ...
  th2))+sin(th1).*(d.*mb.*r2+(ib+(-1).*d.^2.*mb).*sin(th2)))),(-2).* ...
  dphi.*dth1.*r1+dth1.^2.*r1+dphi.^2.*(r1+d.*sin(th1))+(1/2).*ib.^( ...
  -1).*mb.^(-1).*r1.^(-1).*r2.^(-1).*(r1.*(2.*ib.*r2.*u1+d.^2.*mb.* ...
  r2.*u1+(-2).*g.*ib.*mb.*r2.*cos(phi+(-1).*th1)+d.^2.*mb.*r2.*u1.* ...
  cos(2.*th1)+2.*ib.*uth2.*cos(th2).*sin(th1)+2.*ib.*r2.*u2.*sin( ...
  th1).*sin(th2))+(-2).*cos(th1).*((-1).*(ib+(-1).*d.^2.*mb).*r1.* ...
  r2.*u2.*cos(th2)+d.^2.*mb.*r2.*uth1.*sin(th1)+r1.*(d.*mb.*r2.*( ...
  uth1+uth2)+(ib+(-1).*d.^2.*mb).*uth2.*sin(th2)))),r2.^(-2).*((-1) ...
  .*r2.*(2.*dr2.*((-1).*dphi+dth2)+d.*dphi.^2.*cos(th2))+(-1).*g.* ...
  r2.*sin(phi+(-1).*th2)+d.*ib.^(-1).*r2.*u2.*cos(th2).*(r2+(-1).* ...
  d.*sin(th2))+ib.^(-1).*mb.^(-1).*uth2.*(ib+(1/2).*d.^2.*mb+mb.* ...
  r2.^2+(-1/2).*d.^2.*mb.*cos(2.*th2)+(-2).*d.*mb.*r2.*sin(th2))+ ...
  ib.^(-1).*mb.^(-1).*r1.^(-1).*r2.*uth1.*(ib.*cos(th1).*cos(th2)+ ...
  mb.*r1.*(r2+(-1).*d.*sin(th2))+sin(th1).*(d.*mb.*r2+(ib+(-1).* ...
  d.^2.*mb).*sin(th2)))+ib.^(-1).*mb.^(-1).*r2.*u1.*(ib.*cos(th2).* ...
  sin(th1)+cos(th1).*((-1).*d.*mb.*r2+((-1).*ib+d.^2.*mb).*sin(th2)) ...
  )),(-2).*dphi.*dth2.*r2+dth2.^2.*r2+dphi.^2.*(r2+(-1).*d.*sin(th2) ...
  )+(1/2).*ib.^(-1).*mb.^(-1).*r1.^(-1).*r2.^(-1).*(r2.*(2.*ib.*r1.* ...
  u2+d.^2.*mb.*r1.*u2+(-2).*g.*ib.*mb.*r1.*cos(phi+(-1).*th2)+d.^2.* ...
  mb.*r1.*u2.*cos(2.*th2)+2.*ib.*uth1.*cos(th1).*sin(th2)+2.*ib.* ...
  r1.*u1.*sin(th1).*sin(th2))+(-2).*cos(th2).*((-1).*(ib+(-1).* ...
  d.^2.*mb).*r1.*r2.*u1.*cos(th1)+(ib+(-1).*d.^2.*mb).*r2.*uth1.* ...
  sin(th1)+d.*mb.*r1.*((-1).*r2.*(uth1+uth2)+d.*uth2.*sin(th2)))), ...
  mb.^(-1).*r1.^(-1).*r2.^(-1).*(r2.*uth1.*cos(phi+(-1).*th1)+r1.*( ...
  uth2.*cos(phi+(-1).*th2)+(-1).*r2.*(u1.*sin(phi+(-1).*th1)+u2.* ...
  sin(phi+(-1).*th2)))),(-1).*mb.^(-1).*r1.^(-1).*r2.^(-1).*((-1).* ...
  g.*mb.*r1.*r2+r1.*r2.*u1.*cos(phi+(-1).*th1)+r1.*r2.*u2.*cos(phi+( ...
  -1).*th2)+r2.*uth1.*sin(phi+(-1).*th1)+r1.*uth2.*sin(phi+(-1).* ...
  th2)),ib.^(-1).*r1.^(-1).*r2.^(-1).*(r1.*r2.*uth1+r1.*r2.*uth2+( ...
  -1).*d.*r1.*r2.*u1.*cos(th1)+d.*r1.*r2.*u2.*cos(th2)+d.*r2.*uth1.* ...
  sin(th1)+(-1).*d.*r1.*uth2.*sin(th2)),r1.^(-1).*uth1.*cos(phi+(-1) ...
  .*th1)+(-1).*u1.*sin(phi+(-1).*th1),(-1).*r1.^(-1).*(r1.*u1.*cos( ...
  phi+(-1).*th1)+uth1.*sin(phi+(-1).*th1)),r2.^(-1).*uth2.*cos(phi+( ...
  -1).*th2)+(-1).*u2.*sin(phi+(-1).*th2),(-1).*r2.^(-1).*(r2.*u2.* ...
  cos(phi+(-1).*th2)+uth2.*sin(phi+(-1).*th2))];

    Xd(9:15) = rhs(1:7);
  end
end


% EDIT
% --------------------------------------------------------------
function [zeroCrossing,isterminal,direction] = events(~,X,p)
  [th1, r1, th2, r2, x, z, phi, dth1, dr1, dth2, dr2, dx, dz, dphi] = unpackX(X);

  % hips touching ground
  hip1 = -z - p.d * sin(phi);%1
  hip2 = -z + p.d * sin(phi);%2
  % toes touching ground
  td1 = hip1 - r1 * cos(th1 - phi);%3
  td2 = hip2 - r2 * cos(th2 - phi);%4
  lo1 = r1 - p.rho;%5
  lo2 = r2 - p.rho;%6
  
  zeroCrossing = [hip1, hip2, td1, td2, lo1, lo2];
  direction    = [-1, -1, -1, -1, 1, 1];
  isterminal   = ones(size(zeroCrossing));
end

% --------------------------------------------------------------
function Xnew = reset(ev,t,X,p) 
  Xnew = X;
  % modes: 0=aerial, 1=stance1, 2=stance2, 3=double stance
  [th1, r1, th2, r2, x, z, phi, dth1, dr1, dth2, dr2, dx, dz, dphi] = unpackX(X);
  mb = p.mb;ib = p.ib;d = p.d;g = p.g;

  % th1 = X(2); r1 = X(3); th2 = X(4); r2 = X(5); x = X(6); z = X(7); phi = X(8);
    % dth1 = X(9); dr1 = X(10); dth2 = X(11); dr2 = X(12); dx = X(13); dz = X(14); dphi = X(15);
  if (ev==1 || ev==2) % fall
    Xnew(1) = -1; % indicate that sim should stop!
  elseif(ev == 3) % td1
    p.ttd1 = t;
    if X(1)==0
      Xnew(1) = 1;
    elseif X(1)==2
      Xnew(1) = 3;
    end
  elseif(ev == 4) % td2
    p.ttd2 = t;
    if X(1)==0
      Xnew(1) = 2;
    elseif X(1)==1
      Xnew(1) = 3;
    end
  elseif(ev == 5) % lo1
    if X(1)==1
      Xnew(1) = 0;
    elseif X(1)==3
      Xnew(1) = 2;
    end
    % massless limbs: set rdot=0
    Xnew(10) = 0;%dr1
    % Xnew(3) = p.rho;
    % leg angle reset?
  elseif(ev == 6) % lo2
    if X(1)==2
      Xnew(1) = 0;
    elseif X(1)==3
      Xnew(1) = 1;
    end
    % massless limbs: set rdot=0
    Xnew(12) = 0;%dr2
    % Xnew(5) = p.rho;
    % leg angle reset?
  end

  % resolve velocity constraint
  if (ev==3 || ev==4) % touchdown?
    if Xnew(1)==1 % cntS1
      Xnew(9:10) = [r1.^(-1).*(dx.*cos(phi+(-1).*th1)+(-1).* ...
  dz.*sin(phi+(-1).*th1)+dphi.*(r1+d.*sin(th1))),(-1).*dz.*cos(phi+( ...
  -1).*th1)+(-1).*d.*dphi.*cos(th1)+(-1).*dx.*sin(phi+(-1).*th1)];

    elseif Xnew(1)==2 % cntS2
      Xnew(11:12) = [r2.^(-1).*(dx.*cos(phi+(-1).*th2)+(-1) ...
  .*dz.*sin(phi+(-1).*th2)+dphi.*(r2+(-1).*d.*sin(th2))),(-1).*dz.* ...
  cos(phi+(-1).*th2)+d.*dphi.*cos(th2)+(-1).*dx.*sin(phi+(-1).*th2)];
    elseif Xnew(1)==3 % cntSD
      Xnew(9:12) = [r1.^(-1).*(dx.*cos(phi+(-1).*th1)+(-1).* ...
  dz.*sin(phi+(-1).*th1)+dphi.*(r1+d.*sin(th1))),(-1).*dz.*cos(phi+( ...
  -1).*th1)+(-1).*d.*dphi.*cos(th1)+(-1).*dx.*sin(phi+(-1).*th1), ...
  r2.^(-1).*(dx.*cos(phi+(-1).*th2)+(-1).*dz.*sin(phi+(-1).*th2)+ ...
  dphi.*(r2+(-1).*d.*sin(th2))),(-1).*dz.*cos(phi+(-1).*th2)+d.* ...
  dphi.*cos(th2)+(-1).*dx.*sin(phi+(-1).*th2)];
    end
    % Xnew
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
        if iE(end)==1 || iE(end)==2
          fprintf('FALL%d', iE(end))
        elseif iE(end)==3 || iE(end)==4
          fprintf('TD%d', iE(end)-2)
        elseif iE(end)==5 || iE(end)==6
          fprintf('LO%d', iE(end)-4)
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


