function [T0Tn,entities] = DenaHart(alpha, d, theta, r)

%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
entities = struct();

n = length(alpha);


    for i = 1:(n-1)
        if(i==1)
            T1= DHMatrix(i);
            T2= DHMatrix(i+1);
            T0Tn= simplify(T1*T2);
            entities(i).ele= T1;
            entities(i+1).ele= T0Tn;
        else
            T2= DHMatrix(i+1);
            T0Tn= simplify(T0Tn*T2);
            entities(i+1).ele= T0Tn;
        end
    end


    function T = DHMatrix(j)

        Ct = cos(theta(j));
        St = sin(theta(j));
        Ca = cos(alpha(j));
        Sa = sin(alpha(j));
        
        T = [Ct, -St, 0, d(j);
            Ca*St, Ca*Ct, -Sa, -r(j)*Sa;
            Sa*St, Sa*Ct, Ca, r(j)*Ca; 
            0, 0, 0, 1];

    end

end



