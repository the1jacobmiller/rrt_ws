#include <slam/slam.h>
#include <Eigen/Dense>

void EKFSLAMPropagate(Eigen::VectorXd x_hat_t, Eigen::MatrixXd Sigma_x_t, Eigen::VectorXd u, Eigen::MatrixXd Sigma_n, double dt,
                      Eigen::VectorXd& x_hat_tpdt, Eigen::MatrixXd& Sigma_x_tpdt) {

  // propagate x(t)
  double x = x_hat_t[0] + dt * (u[0]) * cos(x_hat_t[2]);
  double y = x_hat_t[1] + dt * (u[0]) * sin(x_hat_t[2]);
  double theta = x_hat_t[2] + dt * (u[1]);
  x_hat_tpdt = x_hat_t;
  x_hat_tpdt[0] = x;
  x_hat_tpdt[1] = y;
  x_hat_tpdt[2] = theta;

  // propagate Sigma
  Eigen::MatrixXd A(3,3);
  A <<  1,    0,    -dt*u[0]*sin(x_hat_t[2]),
        0,    1,    dt*u[0]*cos(x_hat_t[2]),
        0,    0,    1;

  Eigen::MatrixXd N(3,2);
  N <<  dt*cos(x_hat_t[2]),    0,
        dt*sin(x_hat_t[2]),    0,
        0,                     dt;

  Sigma_x_tpdt = Sigma_x_t;
  Eigen::MatrixXd Sigma_RR(3,3);
  Sigma_RR <<  Sigma_x_t(0,0),  Sigma_x_t(0,1), Sigma_x_t(0,2),
              Sigma_x_t(1,0),  Sigma_x_t(1,1), Sigma_x_t(1,2),
              Sigma_x_t(2,0),  Sigma_x_t(2,1), Sigma_x_t(2,2);
  Eigen::MatrixXd Sigma_RR_new(3,3);
  Sigma_RR_new = A*Sigma_RR*A.transpose() + N*Sigma_n*N.transpose();
  Sigma_x_tpdt(0,0)=Sigma_RR_new(0,0);  Sigma_x_tpdt(0,1)=Sigma_RR_new(0,1);  Sigma_x_tpdt(0,2)=Sigma_RR_new(0,2);
  Sigma_x_tpdt(1,0)=Sigma_RR_new(1,0);  Sigma_x_tpdt(1,1)=Sigma_RR_new(1,1);  Sigma_x_tpdt(1,2)=Sigma_RR_new(1,2);
  Sigma_x_tpdt(2,0)=Sigma_RR_new(2,0);  Sigma_x_tpdt(2,1)=Sigma_RR_new(2,1);  Sigma_x_tpdt(2,2)=Sigma_RR_new(2,2);

  for (int col = 3; col < Sigma_x_t.cols(); col+= 2) {
    Eigen::MatrixXd Sigma_RL(3,2);
    Sigma_RL <<  Sigma_x_t(0,col),  Sigma_x_t(0,col+1),
                Sigma_x_t(1,col),  Sigma_x_t(1,col+1),
                Sigma_x_t(2,col),  Sigma_x_t(2,col+1);
    Eigen::MatrixXd Sigma_RL_new(3,2);
    Sigma_RL_new = A*Sigma_RL;
    Sigma_x_tpdt(0,col)=Sigma_RL_new(0,0);  Sigma_x_tpdt(0,col+1)=Sigma_RL_new(0,1);
    Sigma_x_tpdt(1,col)=Sigma_RL_new(1,0);  Sigma_x_tpdt(1,col+1)=Sigma_RL_new(1,1);
    Sigma_x_tpdt(2,col)=Sigma_RL_new(2,0);  Sigma_x_tpdt(2,col+1)=Sigma_RL_new(2,1);
  }

  for (int row = 3; row < Sigma_x_t.rows(); row+=2) {
    Eigen::MatrixXd Sigma_LR(2,3);
    Sigma_LR <<  Sigma_x_t(row,0),   Sigma_x_t(row,1),   Sigma_x_t(row,2),
                Sigma_x_t(row+1,0),  Sigma_x_t(row+1,1), Sigma_x_t(row+1,2);
    Eigen::MatrixXd Sigma_LR_new(2,3);
    Sigma_LR_new = Sigma_LR*A.transpose();
    Sigma_x_tpdt(row,0)=Sigma_LR_new(0,0);  Sigma_x_tpdt(row,1)=Sigma_LR_new(0,1);  Sigma_x_tpdt(row,2)=Sigma_LR_new(0,2);
    Sigma_x_tpdt(row+1,0)=Sigma_LR_new(1,0);  Sigma_x_tpdt(row+1,1)=Sigma_LR_new(1,1);  Sigma_x_tpdt(row+1,2)=Sigma_LR_new(1,2);
  }

}

void EKFSLAMRelPosUpdate(Eigen::VectorXd x_hat_t, Eigen::MatrixXd Sigma_x_t, std::vector<Eigen::VectorXd> zs, std::vector<Eigen::MatrixXd> Sigma_ms,
                         Eigen::VectorXd& x_hat_tpdt, Eigen::MatrixXd& Sigma_x_tpdt) {

  x_hat_tpdt = x_hat_t;
  Sigma_x_tpdt = Sigma_x_t;
  Eigen::Vector3d x_hat;
  x_hat << x_hat_tpdt[0], x_hat_tpdt[1], x_hat_tpdt[2];

  // For each measurement, check if it matches any already in the state, and run an update for it.
  Eigen::Matrix2d C;
  C << cos(x_hat_tpdt[2]), sin(x_hat_tpdt[2]),
       -sin(x_hat_tpdt[2]), cos(x_hat_tpdt[2]);

  // printf("Start EKF SLAM Update\n");
  std::vector<Eigen::VectorXd> unmatched;
  std::vector<Eigen::MatrixXd> unmatched_sigmas;
  for (int i = 0; i < zs.size(); i++) {
    Eigen::VectorXd z = zs[i]; // relative measurement

    Eigen::MatrixXd M = Eigen::MatrixXd::Identity(2,2);
    Eigen::MatrixXd M_Sigma_M= M*Sigma_ms[i]*M;

    bool matched = false;
    double min_distance = std::numeric_limits<double>::max();

    // printf("\tMeasurement %d\n", i);
    int landmark_iter = 1;
    for (int row = 3; row < x_hat_tpdt.rows(); row+=2) {
      Eigen::Vector2d G_p_L, G_p_R;
      G_p_R << x_hat_tpdt[0], x_hat_tpdt[1];
      G_p_L << x_hat_tpdt[row], x_hat_tpdt[row+1];

      Eigen::MatrixXd H_R(2,3);
      H_R <<  -cos(x_hat_tpdt[2]),  -sin(x_hat_tpdt[2]),    -sin(x_hat_tpdt[2])*(G_p_L[0] - x_hat_tpdt[0]) + cos(x_hat_tpdt[2])*(G_p_L[1] - x_hat_tpdt[1]),
              sin(x_hat_tpdt[2]),   -cos(x_hat_tpdt[2]),    -cos(x_hat_tpdt[2])*(G_p_L[0] - x_hat_tpdt[0]) - sin(x_hat_tpdt[2])*(G_p_L[1] - x_hat_tpdt[1]);
      Eigen::MatrixXd H_L(2,2);
      H_L << cos(x_hat_tpdt[2]), sin(x_hat_tpdt[2]),
             -sin(x_hat_tpdt[2]), cos(x_hat_tpdt[2]);
      Eigen::MatrixXd H(2,x_hat_tpdt.rows());
      H(0,0) = H_R(0,0);  H(0,1) = H_R(0,1);  H(0,2) = H_R(0,2);
      H(1,0) = H_R(1,0);  H(1,1) = H_R(1,1);  H(1,2) = H_R(1,2);
      for (int j = 3; j < H_R.cols(); j+=2) {
        if (j == row) {
          H(0,j) = H_L(0,0);  H(0,j+1) = H_L(0,1);
          H(1,j) = H_L(1,0);  H(1,j+1) = H_L(1,1);
        }
        else {
          H(0,j) = 0;  H(0,j+1) = 0;
          H(1,j) = 0;  H(1,j+1) = 0;
        }
      }

      Eigen::Matrix2d S = H*Sigma_x_tpdt*H.transpose() + M_Sigma_M;
      Eigen::MatrixXd S_inv = S.inverse();

      Eigen::VectorXd h_x = C * (G_p_L - G_p_R); // transform to relative frame
      Eigen::VectorXd r = z - h_x;
      double d = sqrt(r.transpose()*S_inv*r);
      // double d = r.transpose()*S_inv*r;
      // printf("\t\tDistance from landmark %d: %f\n", landmark_iter, d);

      if (d < 10*0.1026) {
      // if (d < 0.8) {
        printf("\tUpdating measurement %d\n", i);
        Eigen::MatrixXd K = Sigma_x_tpdt*H.transpose()*S_inv;

        matched = true;
        x_hat_tpdt = x_hat_tpdt + K*(z - h_x);
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(Sigma_x_tpdt.rows(),H.cols());
        Sigma_x_tpdt = (I - K*H) * Sigma_x_tpdt * (I - K*H).transpose() + K*M_Sigma_M*K.transpose();

        break;
      }
      if (d < min_distance) {
        min_distance = d;
      }

      landmark_iter++;
    }

    if (!matched && min_distance > 2*5.9915) {
      unmatched.push_back(z);
      unmatched_sigmas.push_back(Sigma_ms[i]);
    }
  }

  // For every unmatched measurement make sure it's sufficiently novel, then add to the state.
  for (int i = 0; i < unmatched.size(); i++) {
    // printf("Adding new landmark %d\n", i);
    Eigen::VectorXd z = unmatched[i];

    Eigen::MatrixXd M = Eigen::MatrixXd::Identity(2,2);
    Eigen::MatrixXd M_Sigma_M= M*unmatched_sigmas[i]*M;

    Eigen::Vector2d R_p_L, G_p_R, G_p_L;
    G_p_R << x_hat_tpdt[0], x_hat_tpdt[1];
    R_p_L << z[0], z[1];

    G_p_L = C.transpose()*R_p_L + G_p_R;

    Eigen::MatrixXd H_R(2,3);
    H_R <<  -cos(x_hat_tpdt[2]),  -sin(x_hat_tpdt[2]),    -sin(x_hat_tpdt[2])*(G_p_L[0] - x_hat_tpdt[0]) + cos(x_hat_tpdt[2])*(G_p_L[1] - x_hat_tpdt[1]),
            sin(x_hat_tpdt[2]),   -cos(x_hat_tpdt[2]),    -cos(x_hat_tpdt[2])*(G_p_L[0] - x_hat_tpdt[0]) - sin(x_hat_tpdt[2])*(G_p_L[1] - x_hat_tpdt[1]);

    Eigen::Matrix2d H_L;
    H_L <<  cos(x_hat_tpdt[2]),  sin(x_hat_tpdt[2]),
            -sin(x_hat_tpdt[2]), cos(x_hat_tpdt[2]);
    Eigen::MatrixXd H_L_inv = H_L.inverse();

    // feature initialization - x
    Eigen::VectorXd h_x = C * (R_p_L);
    Eigen::VectorXd x_L_new = G_p_L; // H_L_inv*(z - h_x);

    Eigen::MatrixXd tmp = x_hat_tpdt;
    x_hat_tpdt.resize(x_hat_tpdt.rows()+2,1);
    x_hat_tpdt << tmp, x_L_new;

    // debug - print x_hat_tpdt
    // for (int r = 0; r < x_hat_tpdt.rows(); r++) {
    //   for (int c = 0; c < x_hat_tpdt.cols(); c++) {
    //     printf("\t%f", x_hat_tpdt(r,c));
    //   }
    //   printf("\n");
    // }

    // feature initialization - Sigma
    Eigen::MatrixXd Sigma_RR(3,3);
    Sigma_RR <<   Sigma_x_t(0,0),  Sigma_x_t(0,1), Sigma_x_t(0,2),
                  Sigma_x_t(1,0),  Sigma_x_t(1,1), Sigma_x_t(1,2),
                  Sigma_x_t(2,0),  Sigma_x_t(2,1), Sigma_x_t(2,2);

    Eigen::MatrixXd Sigma_tmp = Sigma_x_tpdt;
    int rows = Sigma_x_tpdt.rows()+2;
    int cols = Sigma_x_tpdt.cols()+2;
    Sigma_x_tpdt.resize(rows, cols);

    for (int row = 0; row < Sigma_tmp.rows(); row++) {
      for (int col = 0; col < Sigma_tmp.cols(); col++) {
        Sigma_x_tpdt(row,col) = Sigma_tmp(row,col);
      }
    }

    // printf("\tInitializing columns\n");
    Eigen::MatrixXd Sigma_RL_new(2,3);
    Sigma_RL_new = -H_L_inv*H_R*Sigma_RR;
    Sigma_x_tpdt(rows-2,0)=Sigma_RL_new(0,0);  Sigma_x_tpdt(rows-2,1)=Sigma_RL_new(0,1);  Sigma_x_tpdt(rows-2,2)=Sigma_RL_new(0,2);
    Sigma_x_tpdt(rows-1,0)=Sigma_RL_new(1,0);  Sigma_x_tpdt(rows-1,1)=Sigma_RL_new(1,1);   Sigma_x_tpdt(rows-1,2)=Sigma_RL_new(1,2);

    for (int col = 3; col < cols-2; col+=2) {
      Eigen::MatrixXd Sigma_RL(3,2);
      Sigma_RL <<  Sigma_tmp(0,col),  Sigma_tmp(0,col+1),
                  Sigma_tmp(1,col),  Sigma_tmp(1,col+1),
                  Sigma_tmp(2,col),  Sigma_tmp(2,col+1);
      Eigen::MatrixXd Sigma_new(2,2);
      Sigma_new = -H_L_inv*H_R*Sigma_RL;
      Sigma_x_tpdt(rows-2,col)=Sigma_new(0,0);  Sigma_x_tpdt(rows-2,col+1)=Sigma_new(0,1);
      Sigma_x_tpdt(rows-1,col)=Sigma_new(1,0);  Sigma_x_tpdt(rows-1,col+1)=Sigma_new(1,1);
    }

    // printf("\tInitializing rows\n");
    Eigen::MatrixXd Sigma_LR_new(3,2);
    Sigma_LR_new = -Sigma_RR*H_R.transpose()*H_L_inv.transpose();
    Sigma_x_tpdt(0,cols-2)=Sigma_LR_new(0,0);  Sigma_x_tpdt(0,cols-1)=Sigma_LR_new(0,1);
    Sigma_x_tpdt(1,cols-2)=Sigma_LR_new(1,0);  Sigma_x_tpdt(1,cols-1)=Sigma_LR_new(1,1);
    Sigma_x_tpdt(2,cols-2)=Sigma_LR_new(2,0);  Sigma_x_tpdt(2,cols-1)=Sigma_LR_new(2,1);

    for (int row = 3; row < rows-2; row+=2) {
      Eigen::MatrixXd Sigma_LR(2,3);
      Sigma_LR <<  Sigma_tmp(row,0),   Sigma_tmp(row,1),   Sigma_tmp(row,2),
                  Sigma_tmp(row+1,0),  Sigma_tmp(row+1,1), Sigma_tmp(row+1,2);
      Eigen::MatrixXd Sigma_new(2,2);
      Sigma_new = -Sigma_LR*H_R.transpose()*H_L_inv.transpose();
      Sigma_x_tpdt(row,cols-2)=Sigma_new(0,0);  Sigma_x_tpdt(row,cols-1)=Sigma_new(0,1);
      Sigma_x_tpdt(row+1,cols-2)=Sigma_new(1,0);  Sigma_x_tpdt(row+1,cols-1)=Sigma_new(1,1);
    }

    // printf("\tInitializing corner\n");
    Eigen::MatrixXd Sigma_new(2,2);
    Sigma_new = H_L_inv*(H_R*Sigma_RR*H_R.transpose() + M_Sigma_M)*H_L_inv.transpose();
    Sigma_x_tpdt(rows-2,cols-2)=Sigma_new(0,0);  Sigma_x_tpdt(rows-2,cols-1)=Sigma_new(0,1);
    Sigma_x_tpdt(rows-1,cols-2)=Sigma_new(1,0);  Sigma_x_tpdt(rows-1,cols-1)=Sigma_new(1,1);

    // for (int r = 0; r < Sigma_x_tpdt.rows(); r++) {
    //   for (int c = 0; c < Sigma_x_tpdt.cols(); c++) {
    //     printf("\t%f", Sigma_x_tpdt(r,c));
    //   }
    //   printf("\n");
    // }
  }

  // printf("Finished EKF SLAM Update\n");
}
