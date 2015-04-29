function X_list = getPCPoints(X)

rows = size(X,1);
cols = size(X,2);
X_list = zeros(rows*cols,3);

for i = 1:rows
    for j = 1:cols

        X_list(i*j,:) = reshape(X(i,j,:),3,1);
        
    end
end



end