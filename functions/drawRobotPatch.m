% Draws the robot according to forward kinematics defined in robot and theta

% drawRobotPatch.m
%
% h_p = drawRobotPatch(robot,theta,...)
%
% purpose: draws the robot according to forward kinematics defined in
%           robot and theta
%
% requires: FwdKin.m, hat.m, rot.m, 
%           drawCuboidPatch.m, drawCylinderPatch.m
%
% input:
% robot struct with parts:
%       theta: n-vector of rotational angle / translational displacement
%       type: 0 = rotational  nonzero = prismatic
%       H = [ h1 h2 ... hn ] axis of rotation or translation
%       P = [p01 p12 p23 .. p_{n-1}n ] inter-link vectors
%       n: # of links (>1)
% theta: joint angles
% Optional Additional Parameters:
%       'LinkColor'             default: [1;0;0]
%       'JointColor'            default: [0;1;0]
%       'LinkWidth'             default: 0.04
%       'RotJointDimension'     default: [0.05 0.05]
%       'PrisJointDimension'    default: [0.05 0.05 0]
%       'JointAlpha'            default: 1
%       'LinkAlpha'             default: 1
%       'JointEdgeAlpha'        default: 1
%       'LinkEdgeAlpha'         default: 1
%       'DrawFrame'             default: 'off'
%       'FrameScale'            default: 0.1
%       'Origin'                default: [eye(3) [0;0;0]; 
%                                        [0 0 0]  1]
% 
% can accept existing handle to robot struct using 'Handle'
% 
% output:
% h_p is a struct containing drawing handles to drawing objects
%       h_p.h_joints    joint handles
%       h_p.h_links     link handles
%       h_p.h_frames    frames at each joint (optional)
%
function h_p = drawRobotPatch(robot,theta,varargin)
    % Walk through varargin
    for i=1:2:(nargin-3)
        if strcmp(varargin{i},'LinkColor')
            lc = varargin{i+1};
        elseif strcmp(varargin{i},'JointColor')
            jc = varargin{i+1};
        elseif strcmp(varargin{i},'LinkWidth')
            lw = varargin{i+1};
            if length(lw)==1
                lw = lw*ones(1,robot.n+1);
            end
        elseif strcmp(varargin{i},'RotJointDimension')
            rjd = varargin{i+1};
            if size(rjd,1)==1
                rjd = ones(robot.n,1)*rjd;
            end
        elseif strcmp(varargin{i},'PrisJointDimension')
            pjd = varargin{i+1};
            if size(pjd,1)==1
                pjd = ones(robot.n,1)*pjd;
            end
        elseif strcmp(varargin{i},'LinkAlpha')
            la = varargin{i+1};
        elseif strcmp(varargin{i},'JointAlpha')
            ja = varargin{i+1};
        elseif strcmp(varargin{i},'LinkEdgeAlpha')
            lea = varargin{i+1};
        elseif strcmp(varargin{i},'JointEdgeAlpha')
            jea = varargin{i+1};
        elseif strcmp(varargin{i},'DrawFrame')
            df = varargin{i+1};
        elseif strcmp(varargin{i},'FrameScale')
            fs = varargin{i+1};
        elseif strcmp(varargin{i},'Origin')
            origin = varargin{i+1};
        elseif strcmp(varargin{i},'Handle')
            h_p = varargin{i+1};
        else
            error(['Parameter not recognized: ' varargin{i}]);
        end
    end
    
    if ~exist('lc','var')
        lc = [1;0;0];
    end
    if ~exist('jc','var')
        jc = [0;1;0];
    end
    if ~exist('lw','var')
        lw = 0.04*ones(1,robot.n+1);
    end
    if ~exist('rjd','var')
        rjd = ones(robot.n,1)*[0.05 0.05];
    end
    if ~exist('pjd','var')
        pjd = [0.05;0.05;0]*ones(1,robot.n);
    end
    if ~exist('ja','var')
        ja = 1;
    end
    if ~exist('la','var')
        la = 1;
    end
    if ~exist('jea','var')
        jea = 1;
    end
    if ~exist('lea','var')
        lea = 1;
    end
    if ~exist('df','var')
        df = 'off';
    end
    if ~exist('fs','var')
        fs = 0.1;
    end
    if ~exist('origin','var')
        origin = [eye(3) [0;0;0]; [0 0 0] 1];
    end
    z = [0;0;1];
    
    type = robot.type;
    H = robot.H;
    P = robot.P;
    n = robot.n;
    
    if exist('h_p','var')
        if isfield(h_p,'h_links')
            h_links_p = h_p.h_links;
        end
        if isfield(h_p,'h_joints')
            h_joints_p = h_p.h_joints;
        end
        if isfield(h_p,'h_frames')
            h_frames_p = h_p.h_frames;
        end
    end
    
    p_last = origin(1:3,4);
    R_last = origin(1:3,1:3);
    h_frames = struct('x',0,'y',0,'z',0);
    h_links = struct('h_sides',0,'h_caps',0);
    h_joints = struct('h_sides',0,'h_caps',0);
    for i=1:n
        sub_robot.type = type(1:i);
        sub_robot.H = H(:,1:i);
        sub_robot.P = P(:,1:i);
        sub_robot.n = i;
        [R,p] =FwdKin(sub_robot,theta(1:i));
        R = origin(1:3,1:3)*R;
        p = origin(1:3,4) + origin(1:3,1:3)*p;

        p_c = p_last + 1/2*(p-p_last);
        if norm(P(:,i)) ~= 0
            ljd = [lw(i) norm(p-p_last)];
            h_l = R_last*P(:,i)/norm(P(:,i));
            if abs(dot(h_l,z))==1
%                 Rl = R_last;
                Rl = eye(3);
            else
                phi = subproblem1(hat(h_l)*z,h_l,z);
                Rl = rot(hat(h_l)*z,-phi);
            end
        else
            ljd = [0 0];
            Rl = R_last;
        end            
        if exist('h_links_p','var')
            h_links(i) = drawCylinderPatch(Rl,p_c,ljd, ...
                'Handle',h_links_p(i));
        else
            h_links(i) = drawCylinderPatch(Rl,p_c,ljd, ...
                'FaceColor',lc,'FaceAlpha',la,'EdgeAlpha',lea);
        end
        p_last = p;
        R_last = R;

        if strcmpi(df,'on')
            if exist('h_frames_p','var')
                h_frames(i) = drawFramePatch(R,p,fs, ...
                    'Handle',h_frames_p(i));
            else
                h_frames(i) = drawFramePatch(R,p,fs);
            end
        end
        
        h = R*H(:,i);
        if dot(h,z)==1
            Rj = eye(3);
        else
            phi = subproblem1(hat(h)*z,h,z);
            Rj = rot(hat(h)*z,-phi);
        end
        if type(i) == 0
            if exist('h_joints_p','var')
                h_joints(i) = drawCylinderPatch(Rj,p,rjd(i,:), ...
                    'Handle',h_joints_p(i));
            else
                h_joints(i) = drawCylinderPatch(Rj,p,rjd(i,:), ...
                    'FaceColor',jc,'FaceAlpha',ja,'EdgeAlpha',jea);
            end
        elseif type(i) == 1
            if exist('h_joints_p','var')
                h_joints(i) = drawCuboidPatch(Rj,p, pjd(:,i)+theta(i)/2*h, ...
                    'Handle',h_joints_p(i));
            else
                h_joints(i) = drawCuboidPatch(Rj,p, pjd(:,i)+theta(i)/2*h, ...
                     'FaceColor',jc,'FaceAlpha',ja,'EdgeAlpha',jea);
            end
        end
    end
    
    if size(P,2) > n
        [R,p] = FwdKin(robot,theta);
        R = origin(1:3,1:3)*R;
        p = origin(1:3,4) + origin(1:3,1:3)*p;
        p_c = p_last + 1/2*(p-p_last);
        if norm(P(:,n+1)) ~= 0
            ljd = [lw(n+1) norm(p-p_last)];
            h_l = R*P(:,n+1)/norm(P(:,n+1));
            if dot(h_l,z)==1
                Rl = R;
            else
                phi = subproblem1(hat(h_l)*z,h_l,z);
                Rl = rot(hat(h_l)*z,-phi);
            end
        else
            ljd = [0 0];
            Rl = R;
        end 
        if exist('h_links_p','var')
            h_links(n+1) = drawCylinderPatch(Rl,p_c,ljd, ...
                'Handle',h_links_p(n+1));
        else
            h_links(n+1) = drawCylinderPatch(Rl,p_c,ljd, ...
                'FaceColor',lc,'FaceAlpha',la,'EdgeAlpha',lea);
        end
    end
        
    h_p.h_links = h_links;
    h_p.h_joints = h_joints;
    if strcmpi(df,'on')
        h_p.h_frames = h_frames;
    end
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