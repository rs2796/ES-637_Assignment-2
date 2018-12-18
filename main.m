% Reading the images
Image1 = imread('RGBD_dataset/467.png');
Image2 = imread('RGBD_dataset/472.png');
% Reading the images and converting them into grayscale

I1 = rgb2gray(imread('RGBD_dataset/467.png'));
I2 = rgb2gray(imread('RGBD_dataset/472.png'));
% Reading the depth map images
D1 = imread('RGBD_dataset/467_D.png');
D2 = imread('RGBD_dataset/472_D.png');
points1 = detectSURFFeatures(I1);
points2 = detectSURFFeatures(I2);
[f1,vpts1] = extractFeatures(I1,points1);
[f2,vpts2] = extractFeatures(I2,points2);
indexPairs = matchFeatures(f1,f2) ;
matchedPoints1 = vpts1(indexPairs(:,1));
matchedPoints2 = vpts2(indexPairs(:,2));


a = matchedPoints1(:).Location;
b = matchedPoints2(:).Location;
a1 = a;
a = single(int16(a));
b = single(int16(b));
a = a.';
b = b.';
p = single(zeros(3,size(a,2)));
q = single(zeros(3,size(b,2)));
p(1:2,:) = a;
q(1:2,:) = b;
% Computing the data set p and q for ICP
for i=1:size(a,2)
    p(3,i) = D1(a(2,i),a(1,i));
    q(3,i) = D2(b(2,i),b(1,i));
end
    
    
    
w = zeros(1,size(indexPairs,1));
%Computing the weight vector, First Iteration. 
for i=1:size(a,2);
    nrm = norm(f1(indexPairs(i,1),:)-f2(indexPairs(i,2),:));
    w(i) = exp(-(nrm^2)/(2*(.15)^2));
    
end


I1n = uint8(zeros(size(I1,1),size(I1,2),3));
D1n  = uint8(zeros(size(I1,1),size(I1,2)));
[p,q,A,R,t] = RotationTranslation(p,q,w);


 f2n  = single(zeros(size(indexPairs,1),size(f2,2)));
 
 
 %Other three iterations
 for i = 1:0
     f2n  = single(zeros(size(indexPairs,1),size(f2,2)));
     for j = 1:size(indexPairs,1)
         f2n(j,:) = f2(indexPairs(A(j),2),:);
     end
     for k=1:size(a,1);
         w(k) = exp((-norm(f1(indexPairs(k,1),:)-f2n(k,:)))/2*(.15)^2);
     end
     [p,q,A,R,t] = RotationTranslation(p,q,w);
 end
 % Applying the rotation matrix and translation vector got in the fourth
 % iteration.
 for i= 1: size(I1,2)
    for j = 1:size(I1,1)
        dd = single(D1(j,i));
        
        vec = [i;j;dd];
        nvec = R*single(vec) + t;
        
        nvec = single(int16(nvec));
       
       
        
        
        for k=1:3
            if nvec(k) == 0
                nvec(k) = 1;
            elseif nvec(k) < 0
                nvec(k) = 1;
            
            end
        end
        I1n(nvec(2),nvec(1),:) = Image1(j,i,:);
        D1n(j,i) = nvec(3);
    end
 end
% Displaying the images.

figure;

imshow(Image1);
title('Image-1');
figure;

imshow(D1);
title('Depth Map-1');

figure;

imshow(Image2);
title('Image2');
figure;

imshow(D2);
title('Depth Map-2');

figure;

imshow(I1n);
title('Rotated&Translated Image');
figure;

imshow(D1n);
title('Rotated&Translated Depth Map');
 
 
     
     
     
 
