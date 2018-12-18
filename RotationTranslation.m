% This function take input two data sets: p and q and their corresponding 
% weight vector w and outputs P, Q, A ,R , t  
% Here P is the same dataset p, Q is the modified dataset q, 
% A is a vector which contains order of the datapoints in the data set Q.
% R is the rotational matrix.
% t is the translation vector.


function [P,Q,A,R,t] = RotationTranslation(p,q,w)
p1 = single(zeros(size(p,1),size(p,2)));
q1 = single(zeros(size(q,1),size(q,2)));

for i = 1:size(p,2)
    p1(:,i) = p(:,i)*w(i);
    q1(:,i) = q(:,i)*w(i);
end
    
    
sp1 = single(zeros(size(p,1),1));
sq1 = single(zeros(size(q,1),1));

pm = single(zeros(size(p,1),1));
qm = single(zeros(size(q,1),1));



for i = 1:size(p,1)
    sp1(i) = sum(p1(i,:));
    sq1(i) = sum(q1(i,:));
    
end
for i = 1:size(p,1)
    pm(i) = sp1(i)/sum(w);
    qm(i) = sq1(i)/sum(w);
end



x = single(pm*ones(1,size(p,2)));
y = single(qm*ones(1,size(q,2)));

X = p - x;

Y = q - y;


W = diag(w);

A = X*W*transpose(Y);
[U,S,V] = svd(A);

d = det(V*transpose(U));

I = eye(size(A,1));
I(size(I,1),size(I,2)) = d;
R = V*I*transpose(U);
t = qm - R*pm;


t1  = single(t*single(ones(1,size(p,2))));
P1 = R*p + t1;


ar = single(zeros(size(p,2),1));
index = 1;
% This loop is for rematching of the data points of the both the sets.
for i = 1:size(p,2)
    m = Inf;
    for j = 1:size(q,2)
         if norm(P1(:,i)-q(:,j))<m
             m = norm(P1(:,i)-q(:,j));
             index = j;
         end
    end
    ar(i) = index;
end
P = p;
% qn is modified q
qn = single(zeros(size(q,1),size(q,2)));
for i = 1:size(ar,1)
    qn(:,i) = q(:,ar(i));
end
Q = qn;
A = ar;

         
        
        

















