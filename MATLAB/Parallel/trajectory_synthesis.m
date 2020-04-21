nsteps=301;
dt=0.05;

%z=[x0 y0 th0 x1 y1 th1 ... xn yn thn v0 w0 ... v(n-1) w(n-1)]';



options = optimoptions('fmincon','SpecifyConstraintGradient',true,...
                       'SpecifyObjectiveGradient',true,'Display','iter-detailed') ;
                   
ub = zeros(5*nsteps-2,1);
ub(1:3:3*nsteps) = 10 ;
ub(2:3:3*nsteps) = 10 ;
ub(3:3:3*nsteps) = pi ;
ub(3*nsteps+1:2:5*nsteps-2) = 1 ;
ub(3*nsteps+2:2:5*nsteps-2) = 0.8;

lb = zeros(5*nsteps-2,1);
lb(1:3:3*nsteps) = 0 ;
lb(2:3:3*nsteps) = 0 ;
lb(3:3:3*nsteps) = -pi;
lb(3*nsteps+1:2:5*nsteps-2) = -1;
lb(3*nsteps+2:2:5*nsteps-2) = -0.8;



x0=zeros(1,5*nsteps-2);
cf=@costfun;
nc=@nonlcon;

z=fmincon(cf,x0,[],[],[],[],lb',ub',nc,options);

Y0=reshape(z(1:3*nsteps),3,nsteps)';
U=reshape(z(3*nsteps+1:end),2,nsteps-1);
u=@(t) [interp1(0:dt:299*dt,U(1,:),t,'previous','extrap');...
       interp1(0:dt:299*dt,U(2,:),t,'previous','extrap')];
[T1,Y1]=ode45(@(t,x) odefun(x,u(t)),[0:dt:300*dt],[0 0 0]);
plot(Y0(:,1),Y0(:,2),Y1(:,1),Y1(:,2))
theta = 0:0.01:2*pi;
hold on
plot((0.75*cos(theta)+7),(0.75*sin(theta)+5))
plot((0.75*cos(theta)+4),(0.75*sin(theta)+5))
plot((0.75*cos(theta)+4),(0.75*sin(theta)+2))
hold on
plot(0,0,'x');
legend('fmincon trajectory','ode45 trajectory using x0 = [0;0;0]',...
    'Buffered Obstacle','Start');
ylim([-2,11]);
xlim([-1,11]);
xlabel('x');
ylabel('y');

function [J, dJ] = costfun(z)
    if size(z,2)>size(z,1)
        z = z' ;
    end
    nsteps = (size(z,1)+2)/5 ;

    zx = z(1:3*nsteps) ;
    zu = z(3*nsteps+1:end) ;
    R=eye(2*nsteps-2);
   %{
 for i = 1:nsteps-1
        R(2*i,2*i)= 50; 
    end
   %}
    nom=zeros(3*nsteps,1) ;
    nom(1:3:3*nsteps) = 8.3;
    nom(2:3:3*nsteps) = 4.5 ;
    nom(3:3:3*nsteps) = pi/2 ;
    Q=eye(3*nsteps);
    
    J = zu'*R*zu+(zx-nom)'*Q*(zx-nom) ;
    dJ = [2*Q*zx-2*Q*nom;...
          2*R*zu]' ;
end
function [g,h,dg,dh]=nonlcon(z)
    if size(z,2)>size(z,1)
        z = z' ;
    end
    nsteps = (size(z,1)+2)/5 ;
    dt = 0.05 ;

    zx=z(1:nsteps*3);
    zu=z(nsteps*3+1:end);

    g = zeros(3*nsteps,1) ;
    dg = zeros(3*nsteps,5*nsteps-2) ;

    h = zeros(3*nsteps,1) ;
    dh = zeros(3*nsteps,5*nsteps-2);

    h(1:3) = z(1:3,:) ;
    dh(1:3,1:3)=eye(3);

    for i=1:nsteps
        if i==1
            h(1:3) = z(1:3,:) ;
            dh(1:3,1:3)=eye(3); 
        else
            h(3*i-2:3*i) = zx(3*i-2:3*i)-zx(3*i-5:3*i-3)-...
                               dt*odefun(zx(3*i-5:3*i-3),zu(2*i-3:2*i-2)) ;
            dh(3*i-2:3*i,3*i-5:3*i) = [-eye(3)-dt*statepart(zx(3*i-5:3*i-3),zu(2*i-3:2*i-2)),eye(3)] ;
            dh(3*i-2:3*i,3*nsteps+2*i-3:3*nsteps+2*i-2) = -dt*inputpart(zx(3*i-5:3*i-3),zu(2*i-3:2*i-2)) ;
        end
        g(3*i-2) = (1.25^2)-((z(3*i-2)-7)^2+(z(3*i-1)-5)^2) ;
        g(3*i-1) = (1.25^2)-((z(3*i-2)-4)^2+(z(3*i-1)-2)^2) ;
        g(3*i) = (1.25^2)-((z(3*i-2)-4)^2+(z(3*i-1)-5)^2) ;
        dg(3*i-2,3*i-2:3*i-1) = -2*[z(3*i-2)-7, z(3*i-1)-5] ;
        dg(3*i-1,3*i-2:3*i-1) = -2*[z(3*i-2)-4, z(3*i-1)-2] ;
        dg(3*i,3*i-2:3*i-1) = -2*[z(3*i-2)-4, z(3*i-1)-5] ;
    end

    dg = dg' ;
    dh= dh' ;
end

function [dx] = odefun(x,u)
    dx = [u(1)*cos(x(3));...
          u(1)*sin(x(3));...
          u(2)] ;
end

function [pd] = statepart(x,u)
    pd = zeros(3,3) ;
    pd(:,3) = [-u(1)*sin(x(3));...
                u(1)*cos(x(3));...
                0] ;
end

function [ip] = inputpart(x,u)
    ip = [cos(x(3)) 0;...
          sin(x(3))  0 ;...
          0         1];
end
