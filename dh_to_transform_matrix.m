function transform_matrix = dh_to_transform_matrix(a, alpha, d, theta)
    
    transform_matrix = eye(4);

    for i = 1:6
        A = [cos(theta(i)),-sin(theta(i)),0,a(i);
             sin(theta(i))*cos(alpha(i)),cos(theta(i))*cos(alpha(i)),-sin(alpha(i)),-sin(alpha(i))*d(i);
             sin(theta(i))*sin(alpha(i)),cos(theta(i))*sin(alpha(i)),cos(alpha(i)),cos(alpha(i))*d(i);
             0,0,0,1];

        transform_matrix = transform_matrix * A;
    end
    
end