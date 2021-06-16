function new_states = QuarterCar(states,torque,stiffness,hamp3)

    b.R = 0.2;                          % Tire radius [m]
    b.M = 300;                       % Sprung Mass
    b.m = 75;                          % Unsprung mass
    b.I = 0.5 * b.m * b.R^2;           % Inertia
    %b.T = -200;                         % Motor Torque
    b.T = torque;
    %b.K = 15000;                       % Stiffness
    b.K = 15000 + stiffness;
    b.C = 1e3;                          % Damping
    b.g = 9.81;                         % Gravity
    b.c = 5;
    b.L0 =0.5;
    b.cr = 0.6;
    b.Fy = 0;

    w1 = 0.4;                             % frequency / wavenumber
    hamp1 = 0.1;                          % amplitude of road height
    w2 = 0.752;                             % frequency / wavenumber
    hamp2 = 0.1*0;                          % amplitude of road height
    %hamp3 = hamp3;
    
    
    w3 = 0.503;                             % frequency / wavenumber
    w4 = 0.8164;
    hamp4 = 0.2*0;
    %h2 = 0.5*cos(0.05*X);
    
    %b.h = @(x) hamp1*cos(w1*x)  ;            % Road height
    %b.dh = @(x) -w1*hamp1*sin(w1*x);        % Road height derivative
    %b.ddh = @(x) -w1^2*hamp1*cos(w1*x);     % Road height 2nd derivative
    
    b.h = @(x) hamp1*cos(w1*x) + hamp2*cos(w2*x) +hamp3*cos(w3*x) + hamp4*cos(w4*x);            % Road height
    b.dh = @(x) -w1*hamp1*sin(w1*x) -w2*hamp2*sin(w2*x) -w3*hamp3*sin(w3*x) -w4*hamp4*sin(w4*x);        % Road height derivative
    b.ddh = @(x) -w1^2*hamp1*cos(w1*x) -w2^2*hamp2*cos(w2*x) -w3^2*hamp3*cos(w3*x) -w4^2*hamp4*cos(w4*x);     % Road height 2nd derivative
    
    b.i=0;
    d0 = states;
    %X = states;
    b.x=0;
    %options = odeset('RelTol',5.421011e-6,'AbsTol',5.421011e-6);
    [~,X] = ode45(@(t,X) QCRolling(X,b),[0 0.1],d0);

    function dXdt = QCRolling(X,b)
        %x = b.x + 0.01*b.i;
        %b.x =x;
        x=X(1);
        x_dot = X(2);
        y = X(3);
        y_dot = X(4);
        
        
        
        
        %if b.i>2
         %   b.T = 0;
        %end

        a = -1*(b.C * b.dh(x)^2 -b.m*b.dh(x)*b.ddh(x))/(b.M + b.m + (b.I/(b.R^2)) + b.dh(x)^2) ;
        p = -1*(b.C * b.h(x))/(b.M + b.m + (b.I/(b.R^2)) + b.dh(x)^2);
        d = (b.K * b.dh(x))/(b.M + b.m + (b.I/(b.R^2)) + b.dh(x)^2);
        e = (-b.c*(x_dot).^2 - b.cr*(x_dot) +(b.T/b.R)- (b.L0*b.K*b.dh(x)) - 2*b.m*b.dh(x)*b.ddh(x)*(x_dot^2) - b.m*b.g*b.dh(x) - b.K*b.h(x)*b.dh(x))/(b.M + b.m + (b.I/(b.R^2)) + b.dh(x)^2);

        A = [0 1 0 0;0 a d p;0 0 0 1;0 (b.C*b.dh(x)/b.M) (-b.K/b.M) (-b.C/b.M)];
        B = [0;e;0;(-b.M*b.g + b.K*b.h(x) +b.K*b.L0 +b.Fy)/b.M];

        dXdt = A*X +B;
        b.i = b.i+1;
        
    end

    new_states = X;
end
    
    
         
