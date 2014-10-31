#include "pid_controller.h"

PIDController::PIDController(double kp, double kd, double ki, int window_size):
    kp_(kp), kd_(kd), ki_(ki), window_size_(window_size), errors_(window_size)
{
    index_error_ = 0;
    std::fill(errors_.begin(),errors_.end(),0.0);
    last_error_ = 0.0;
}


double PIDController::get(double error)
{
    errors_[index_error_] = error;
    index_error_ =  (index_error_+1)%window_size_;

    double sum =0;
    for(int i=0;i<window_size_;i++)
    {
        sum += errors_[i];
    }

    double control =  kp_ * error + kd_ * (error - last_error_) + ki_ * sum;

    return control;
}


