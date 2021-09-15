//Kalman filter publisher 
#include <Eigen/Dense>

class KalmanFilter
{   
    public:
    /**
     * Create a Kalman filter with the specified matrices.
     *   F - System dynamics matrix
     *   H - Output matrix
     *   Q - Process noise covariance
     *   R - Measurement noise covariance
     *   P - Estimate error covariance
     */
    KalmanFilter(
        double dt,
        const Eigen::MatrixXd& F,
        const Eigen::MatrixXd& H,
        const Eigen::MatrixXd& Q,
        const Eigen::MatrixXd& R,
        const Eigen::MatrixXd& P
    );

    private:
    // Matrices for computation
    Eigen::MatrixXd F, P, Q, R, P, K, P0;

    // System dimensions
    int m, n;

    // Initial and current time seconds
    double t0, t;

    // Discrete time step
    double dt;

    //check if filter initalized
    bool filter_init;

    // n-size identity
    Eigen::MatrixXd I;

    // Estimated states
    Eigen::VectorXd x_hat, x_hat_new;
};