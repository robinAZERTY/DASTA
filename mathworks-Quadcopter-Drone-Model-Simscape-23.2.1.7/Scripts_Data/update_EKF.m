function [X,P]=update_EKF(U,Z)
    global ekf
    ekf.Un = U;
    ekf = ekf.predict();
    ekf.Zn = Z;
    ekf = ekf.update();
    X = ekf.Xn;
    P = ekf.Pn;
end