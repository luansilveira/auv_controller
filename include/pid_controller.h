#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <vector>

class PIDController
{
public:
    PIDController(double kp, double kd, double ki, int window_size = 10);
    double get(double error);


private:
    double kp_;
    double kd_;
    double ki_;

    std::vector<double> errors_;
    int index_error_;
    int window_size_;
    double last_error_;
};

#endif // PID_CONTROLLER_H
