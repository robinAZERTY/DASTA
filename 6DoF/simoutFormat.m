function [t,x,y,z,vx,vy,vz,ax,ay,az,qw,qx,qy,qz,wxx,wyy,wzz,axx,ayy,azz,gyr_xx,gyr_yy,gyr_zz,acc_xx,acc_yy,acc_zz,temperature, gps_xx, gps_yy, gps_zz, l1c1x,l1c1y, l2c1x,l2c1y, l1c2x,l1c2y, l2c2x,l2c2y] = simoutFormat(simout)

    t = simout.time;
    
    data = squeeze(simout.signals.values);
    x = transpose(data(1,:));
    y = transpose(data(2,:));
    z = transpose(data(3,:));
    
    vx = transpose(data(4,:));
    vy = transpose(data(5,:));
    vz = transpose(data(6,:));
    
    ax = transpose(data(7,:));
    ay = transpose(data(8,:));
    az = transpose(data(9,:));
    
    qw = transpose(data(10,:));
    qx = transpose(data(11,:));
    qy = transpose(data(12,:));
    qz = transpose(data(13,:));
    
    wxx = transpose(data(14,:));
    wyy = transpose(data(15,:));
    wzz = transpose(data(16,:));
    
    axx = transpose(data(17,:));
    ayy = transpose(data(18,:));
    azz = transpose(data(19,:));
    
    gyr_xx = transpose(data(20,:));
    gyr_yy = transpose(data(21,:));
    gyr_zz = transpose(data(22,:));
    
    acc_xx = transpose(data(23,:));
    acc_yy = transpose(data(24,:));
    acc_zz = transpose(data(25,:));
    
    temperature = transpose(data(26,:));

    gps_xx = transpose(data(27,:));
    gps_yy = transpose(data(28,:));
    gps_zz = transpose(data(29,:));

    l1c1x = transpose(data(30,:));
    l1c1y = transpose(data(31,:));

    l2c1x = transpose(data(32,:));
    l2c1y = transpose(data(33,:));

    l1c2x = transpose(data(34,:));
    l1c2y = transpose(data(35,:));

    l2c2x = transpose(data(36,:));
    l2c2y = transpose(data(37,:));
end