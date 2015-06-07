clear all;
A=textread("../test.csv", "%f", "delimiter", ",");
S=reshape(A, 3, rows(A)/3);
scatter3(S(1,:)', S(2,:)', S(3,:)');
xlabel('X');
ylabel('Y');
zlabel('Z');