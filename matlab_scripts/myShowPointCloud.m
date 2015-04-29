% Input : X = rows*cols,3

function myShowPointCloud(X,im)

rows = size(im,1);
cols = size(im,2);
X3C = zeros(rows,cols,3);

for i = 1:rows
    for j = 1:cols
        
        X3C(i,j,1) = X((i-1)*cols + j,1); 
        X3C(i,j,2) = X((i-1)*cols + j,2);
        X3C(i,j,3) = X((i-1)*cols + j,3);
        
    end
end


showPointCloud(X3C,im); 

end