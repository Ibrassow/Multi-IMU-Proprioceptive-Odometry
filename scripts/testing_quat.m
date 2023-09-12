

yawk = (-2*pi:0.1:2*pi)';

yy = (-2*pi:0.1:2*pi)';
s = size(yy);
N = s(1);
yy = yy + 0.5 * randn(N, 1);


quat_list = zeros(N,4);
for i=1:N
    quat_list(i, :) = euler_to_quat( [0, 0, yy(i)] );
end


vm = [1 1 1]';
vm = vm / norm(vm);

res = zeros(N,2);
g = zeros(N,1)

for i=1:N

    Rm = quat_to_rot(quat_list(i,:));
    
    vquat = Rm * vm;
    
    Ry = euler_to_rot([0, 0, yawk(i,:)]);
    
    vy = Ry * vm;
    %ss = sign(sum(vy - vquat));
    %res(i) = ss * norm(vy - vquat);
    res(i,:) = vy(1:2) - vquat(1:2);
end



plot(yawk-yy)
hold on
plot(res)





