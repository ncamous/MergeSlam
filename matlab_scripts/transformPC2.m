function X_hat = transformPC2(X,T)

rows = size(X,1);
cols = size(X,2);

X_hat = zeros(rows,cols,3);

for i = 1:rows
    for j = 1: cols
        
        Xh  = [reshape(X(i,j,:),3,1) ; 1 ];
        v_hat = T*Xh; 
        X_hat(i,j,1) = v_hat(1);
        X_hat(i,j,2) = v_hat(2);
        X_hat(i,j,3) = v_hat(3);
    end
end



end