function khat = hat(k)
% Converts a 3x1 vector in to 3x3 skew-symmetric matrix
  
    khat=[0 -k(3) k(2); k(3) 0 -k(1); -k(2) k(1) 0];
  
end
  