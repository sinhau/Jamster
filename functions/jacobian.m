function J = jacobian(robot,theta)
% Calculate jacobian given robot constants and joint angles

    P=robot.P;
    type=robot.type;
    H=robot.H;
    n=robot.n;

    J=zeros(6,n);

    for j=1:n

        R=eye(3);
        p=[0;0;0];
        h=H(:,j);

        for i=n:-1:1

            if type(i)==0
                hi=H(:,i);

                Ri=rot(hi,theta(i));

                if i>= j
                    p=Ri*(p+P(:,i+1));
                    R=Ri*R;
                else
                    p=Ri*p;
                    R=Ri*R;
                    h=Ri*h;

                end

            else
                if i >= j
                    p=P(:,i+1)+H(:,i)*theta(i)+p;
                end
            end

        end

        if type(j)==0
            Jj=[h; cross(h,p)];
        else
            Jj=[0;0;0; h];
        end

        J(:,j)=Jj;

    end

end

   
   
    
    
