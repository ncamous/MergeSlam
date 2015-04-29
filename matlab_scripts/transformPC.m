function X_hat = transformPC(X,T)

rows = size(X,1);
cols = size(X,2);

Xh = cat(3,X,ones(rows,cols)); % Add  to all coordinates (Convert to Homogeneous coordinates)
Xh_hat = zeros(rows,cols,4);

for i = 1:rows
    for j = 1: cols
        
        Xh_hat(i,j,:) = reshape(T*reshape(Xh(i,j,:),4,1),1,1,4); 
    
    end
end

X_hat = Xh_hat(:,:,1:3);


end