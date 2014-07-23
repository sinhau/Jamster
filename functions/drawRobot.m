% Daws the robot according to forward kinematics from theta
% drawRobot.m
%
% drawRobot(robot,theta)
%
% purpose: draws the robot according to forward kinematics from theta
%
% requires: FwdKin.m, hat.m, rot.m
%
% input:
% robot struct with parts:
%   theta: n-vector of rotational angle / translational displacement
%   type: 0 = rotational  nonzero = prismatic
%   H = [ h1 h2 ... hn ] axis of rotation or translation
%   P = [p01 p12 p23 .. p_{n-1}n ] inter-link vectors
%   n: # of links (>1)
% 
% output:
% no output
%
function drawRobot(robot,theta)
    type = robot.type;
    H = robot.H;
    P = robot.P;
    n = robot.n;
    
    p_all = zeros(3,n+2);
    h_all = zeros(3,n);
     
    for i=1:n
        sub_robot.type = type(1:i);
        sub_robot.H = H(:,1:i);
        sub_robot.P = P(:,1:i);
        sub_robot.n = i;
        [R,p] =fwdKin(sub_robot,theta(1:i));
        p_all(:,i+1) = p;
        h_all(:,i) = R*H(:,i);
    end
    
    if size(P,2) > n
        [~,p] = fwdKin(robot,theta);
        p_all(:,n+2) = p;
    end
    
    plot3(p_all(1,:),p_all(2,:),p_all(3,:),'-ro');
    hold on;
    for i=1:n
        if type(i)==0
            drawRotJoint(h_all(:,i),p_all(:,i+1),0.05,0.05);
        else
            drawPrismJoint(h_all(:,i),p_all(:,i+1),0.04,theta(i));
        end
    end
    axis equal;
    max_x = max(abs(p_all(1,:)));
    if max_x == 0, max_x = 1; end
    max_y = max(abs(p_all(2,:)));
    if max_y == 0, max_y = 1; end
    max_z = max(abs(p_all(3,:)));
    if max_z == 0, max_z = 1; end
    xlabel('X');ylabel('Y');zlabel('Z');
    xlim([-max_x max_x]);
    ylim([-max_y max_y]);
    zlim([-max_z max_z]);
end
    
    function drawRotJoint(h,p,radius,height)
        [X Y Z] = cylinder(radius);
        Z = height*Z-0.5*height;
        h = h / norm(h);
        z = [0;0;1]; % cylinder defaults to draw about z
        if ~(dot(h,z)==1)
            phi = subproblem1(hat(h)*z,h,z);
            Rcyl = rot(hat(h)*z,-phi);
            top = Rcyl*[X(1,:);Y(1,:);Z(1,:)];
            bottom = Rcyl*[X(2,:);Y(2,:);Z(2,:)];
            X_rot = [top(1,:);bottom(1,:)];
            Y_rot = [top(2,:);bottom(2,:)];
            Z_rot = [top(3,:);bottom(3,:)];
        else
            X_rot = X;
            Y_rot = Y;
            Z_rot = Z;
        end

        surf(p(1) + X_rot,p(2) + Y_rot,p(3) + Z_rot);
    end

    function drawPrismJoint(h,p,width,theta)
        [X Y Z] = cuboid(width);
        Z = theta*Z;
        h = h / norm(h);
        z = [0;0;1]; % cuboid defaults to draw about z
        if ~(dot(h,z)==1)
            phi = subproblem1(hat(h)*z,h,z);
            Rcyl = rot(hat(h)*z,-phi);
            top = Rcyl*[X(1,:);Y(1,:);Z(1,:)];
            bottom = Rcyl*[X(2,:);Y(2,:);Z(2,:)];
            X_rot = [top(1,:);bottom(1,:)];
            Y_rot = [top(2,:);bottom(2,:)];
            Z_rot = [top(3,:);bottom(3,:)];
        else
            X_rot = X;
            Y_rot = Y;
            Z_rot = Z;
        end

        surf(p(1) + X_rot,p(2) + Y_rot,p(3) + Z_rot);
    end

    function [X Y Z] = cuboid(w)
        X = [w/2 w/2 -w/2 -w/2 w/2; w/2 w/2 -w/2 -w/2 w/2];
        Y = [w/2 -w/2 -w/2 w/2 w/2; w/2 -w/2 -w/2 w/2 w/2];
        Z = [0 0 0 0 0; 1 1 1 1 1];
    end

    function [theta]=subproblem1(k,p,q)

        if norm(p-q)<sqrt(eps);theta=0;return;end

        k=k/norm(k);
        pp=p-(p'*k)*k;
        qp=q-(q'*k)*k;
        epp=pp/norm(pp);
        eqp=qp/norm(qp);

        theta=subproblem0(epp,eqp,k);

        tol=1e-2;
        if (abs(norm(p)-norm(q))>tol);
          disp('*** Warning *** ||p|| and ||q|| must be the same!!!');
        end
    end
    
    function [theta]=subproblem0(p,q,k)

        if ((k'*p)>sqrt(eps)||(k'*q)>sqrt(eps))
          error('k must be perpendicular to p and q');
        end

        ep=p/norm(p);
        eq=q/norm(q);

        theta=2*atan2(norm(ep-eq),norm(ep+eq));

        if k'*(cross(p,q))<0
          theta=-theta;
        end
    end
