function R=rot(k,theta)
% Find 3x3 rotation matrix given axis-angle representation
  
  k=k/norm(k);
  R=eye(3,3)+sin(theta)*hat(k)+(1-cos(theta))*hat(k)*hat(k);
  
end
  