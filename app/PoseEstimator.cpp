#include "../include/PoseEstimator.hpp"

// Class Constructor
PoseEstimator::PoseEstimator(double& wheel_rad,
                             double& distance_between_front_back_wheels,
                             int& ticks_per_revolution, double& gain) {
    // Initialising all the variables
    this->counter = 0;
    this->wheel_rad = wheel_rad;
    this->distance_between_front_back_wheels =
        distance_between_front_back_wheels;
    this->ticks_per_revolution = ticks_per_revolution;
    this->dt = 0;
    this->t_curr = 0;
    this->t_prev = 0;
    this->steering_angle = 0;
    this->encoder_ticks_curr = 0;
    this->encoder_ticks_prev = 0;
    this->x_encoder = 0;
    this->y_encoder = 0;
    this->theta_encoder = 0;
    this->x_encoder_prev = 0;
    this->y_encoder_prev = 0;
    this->theta_encoder_prev = 0;
    this->x_encoder_dot = 0;
    this->y_encoder_dot = 0;
    this->theta_encoder_dot = 0;
    this->steering_velocity_encoder = 0;
    this->theta_gyro_dot = 0;
    this->theta_gyro = 0;
    this->steering_velocity_gyro = 0;
    this->x_gyro_dot = 0;
    this->y_gyro_dot = 0;
    this->x_gyro_prev = 0;
    this->y_gyro_prev = 0;
    this->theta_gyro_prev = 0;
    this->x_gyro = 0;
    this->y_gyro = 0;
    this->filter_gain = gain;
    this->steering_angle_limit = M_PI / 2;
}

/* --------------------------------------------------------------------------*/
/**
 * @brief  To preprocess steering angle
 */
/* --------------------------------------------------------------------------*/
void PoseEstimator::preprocessSteeringAngle() {
    if (this->steering_angle < -this->steering_angle_limit) {
        this->steering_angle = -this->steering_angle_limit;
    } else if (this->steering_angle > this->steering_angle_limit) {
        this->steering_angle = this->steering_angle_limit;
    }
}

/* --------------------------------------------------------------------------*/
/**
 * @brief  Calculating difference in time
 *
 * @Returns   difference
 */
/* --------------------------------------------------------------------------*/
double PoseEstimator::calculateDt() {
    this->dt = this->t_curr - this->t_prev;
    return this->dt;
}

/* --------------------------------------------------------------------------*/
/**
 * @brief  To calculate steering velocity
 * = (tick difference)*2pi * r/( ticks_per_rev *dt)
 *
 * @Returns
 */
/* --------------------------------------------------------------------------*/
double PoseEstimator::calculateSteeringVelocity() {
    if ((this->encoder_ticks_curr - this->encoder_ticks_prev) < 0) {
        this->steering_velocity_encoder = 0;
        return steering_velocity_encoder;
    } else {
        this->steering_velocity_encoder =
            (this->encoder_ticks_curr - this->encoder_ticks_prev) * 2 * M_PI *
            this->wheel_rad / (this->ticks_per_revolution * (dt));
        return this->steering_velocity_encoder;
    }
}

/* --------------------------------------------------------------------------*/
/**
 * @brief  Theta Encoder dot calculation = Vs sin(steering angle) /l
 * l = distance between front and back wheel
 *
 * @Returns
 */
/* --------------------------------------------------------------------------*/
double PoseEstimator::calculateThetaEncoderDot() {
    this->theta_encoder_dot = this->steering_velocity_encoder *
                              sin(this->steering_angle) /
                              distance_between_front_back_wheels;
    return this->theta_encoder_dot;
}

/* --------------------------------------------------------------------------*/
/**
 * @brief  To integrate variables
 * integral_X in discrete = x_old + dt* x_dot
 *
 *  @Param val      variable
 *  @Param val_dot  differential of the variable1
 *
 * @Returns
 */
/* --------------------------------------------------------------------------*/
double PoseEstimator::integrateVariable(double val, double val_dot) {
    auto integrated_val = val + this->dt * val_dot;
    return integrated_val;
}

/* --------------------------------------------------------------------------*/
/**
 * @brief  Xencoderdot = Vs * cos(steering angle) * cos(theta_encoder)
 *
 * @Returns   Xencoder_dot ,velocity calculated using encoder values in
 * x -direction
 */
/* --------------------------------------------------------------------------*/
double PoseEstimator::calculateXEncoderDot() {
    this->x_encoder_dot = this->steering_velocity_encoder *
                          cos(this->steering_angle) * cos(this->theta_encoder);
    return this->x_encoder_dot;
}
/* --------------------------------------------------------------------------*/
/**
 * @brief  Y_encoderdot = Vs * cos(steering angle) * sin(theta_encoder)
 *
 * @Returns   Yencoder_dot ,velocity calculated using encoder values in
 * x -direction
 */
/* --------------------------------------------------------------------------*/
double PoseEstimator::calculateYEncoderDot() {
    this->y_encoder_dot = this->steering_velocity_encoder *
                          cos(this->steering_angle) * sin(this->theta_encoder);
    return this->y_encoder_dot;
}

/* --------------------------------------------------------------------------*/
/**
 * @brief  Vs = theta_gyro_dot * l /sin(steering angle)
 * l = distance between front and back wheels
 *
 * @Returns   Steering Velocity as per gyroscope measurement values
 */
/* --------------------------------------------------------------------------*/
double PoseEstimator::calculateSteeringVelocityGyro() {
    if ((this->steering_angle < 0.01) & (this->steering_angle > -0.01)) {
        this->steering_velocity_gyro = 0;
        return this->steering_velocity_gyro;
    } else {
        this->steering_velocity_gyro = this->theta_gyro_dot *
                                       distance_between_front_back_wheels /
                                       sin(this->steering_angle);
        return this->steering_velocity_gyro;
    }
}

/* --------------------------------------------------------------------------*/
/**
 * @brief  Xgyrodot = Vs * cos(steering angle) * cos(theta_gyro)
 *
 * @Returns   double Xgyro_Dot
 */
/* --------------------------------------------------------------------------*/
double PoseEstimator::calculateXGyroDot() {
    this->x_gyro_dot = this->steering_velocity_gyro *
                       cos(this->steering_angle) * cos(this->theta_gyro);
    return this->x_gyro_dot;
}
/* --------------------------------------------------------------------------*/
/**
 * @brief  Ygyrodot = Vs * cos(steering angle) * sin(theta_gyro)
 *
 * @Returns   double Ygyro_Dot
 */
/* --------------------------------------------------------------------------*/
double PoseEstimator::calculateYGyroDot() {
    this->y_gyro_dot = this->steering_velocity_gyro *
                       cos(this->steering_angle) * sin(this->theta_gyro);
    return this->y_gyro_dot;
}

/* --------------------------------------------------------------------------*/
/**
 * @brief  Complimentary Filter
 * output = gain(Measurement 1) + (1-gain)*(measurement2)
 * @Param encoder_val encoder predicted values
 * @Param gyro_val gyroscope predicted values
 *
 * @Returns   double
 */
/* --------------------------------------------------------------------------*/
double PoseEstimator::complimentaryFilter(double encoder_val, double gyro_val) {
    double gain = 100 * this->filter_gain;
    double output = (gain * encoder_val) + ((100 - gain) * gyro_val);
    return output / 100;
}

// Main Estimator

/* --------------------------------------------------------------------------*/
/**
 * @brief  Estimates the pose of the tricycle
 *
 * @Param time_c = current time
 * @Param steering_angle
 * @Param encoder_ticks
 * @Param theta_gyro_dot
 *
 * @Returns   vector containing Heading/Pose [ x, y,theta]
 */
/* --------------------------------------------------------------------------*/
vector<double> PoseEstimator::estimate(double& time_c, double& steering_angle,
                                       int& encoder_ticks,
                                       double& theta_gyro_dot) {
    vector<double> pose;
    pose.clear();

    if (this->counter == 0) {
        this->t_prev = time_c;
        this->encoder_ticks_prev = encoder_ticks;
        this->theta_gyro_dot = theta_gyro_dot;
        this->counter++;

        pose.push_back(0);
        pose.push_back(0);
        pose.push_back(0);
        return pose;
    } else {
        this->t_curr = time_c;
        this->encoder_ticks_curr = encoder_ticks;
        this->theta_gyro_dot = theta_gyro_dot;
        this->steering_angle = steering_angle;

        PoseEstimator::calculateDt();

        PoseEstimator::calculateSteeringVelocity();

        PoseEstimator::calculateThetaEncoderDot();

        // theta_encoder
        this->theta_encoder = PoseEstimator::integrateVariable(
            this->theta_encoder_prev, theta_encoder_dot);

        PoseEstimator::calculateYEncoderDot();

        PoseEstimator::calculateXEncoderDot();

        this->x_encoder = PoseEstimator::integrateVariable(this->x_encoder_prev,
                                                           this->x_encoder_dot);
        this->y_encoder = PoseEstimator::integrateVariable(this->y_encoder_prev,
                                                           this->y_encoder_dot);

        this->theta_gyro = PoseEstimator::integrateVariable(
            this->theta_gyro_prev, this->theta_gyro_dot);

        PoseEstimator::calculateSteeringVelocityGyro();

        this->x_gyro_dot = PoseEstimator::calculateXGyroDot();
        this->y_gyro_dot = PoseEstimator::calculateYGyroDot();

        this->x_gyro = PoseEstimator::integrateVariable(
            this->x_gyro_prev, PoseEstimator::calculateXGyroDot());
        this->y_gyro = PoseEstimator::integrateVariable(
            this->y_gyro_prev, PoseEstimator::calculateYGyroDot());

        double pred_x =
            PoseEstimator::complimentaryFilter(this->x_encoder, this->x_gyro);

        double pred_y =
            PoseEstimator::complimentaryFilter(this->y_encoder, this->y_gyro);
        double pred_theta = PoseEstimator::complimentaryFilter(
            this->theta_encoder, this->theta_gyro);

        pose.push_back(pred_x);
        pose.push_back(pred_y);
        pose.push_back(pred_theta);

        this->x_encoder_prev = pred_x;           // this->x_encoder;
        this->y_encoder_prev = pred_y;           // this->y_encoder;
        this->theta_encoder_prev = pred_theta;   // this->theta_encoder;

        this->x_gyro_prev = pred_x;           // this->x_gyro;
        this->y_gyro_prev = pred_y;           // this->y_gyro;
        this->theta_gyro_prev = pred_theta;   // this->theta_gyro;
        return pose;
    }
}
// Defining all Set Functions
//
//
double PoseEstimator::setXEncoder(double x_encoder) {
    this->x_encoder = x_encoder;
    return this->x_encoder;
}

double PoseEstimator::setYEncoder(double y_encoder) {
    this->y_encoder = y_encoder;
    return this->y_encoder;
}

double PoseEstimator::setThetaEncoder(double theta_encoder) {
    return this->theta_encoder = theta_encoder;
}

double PoseEstimator::setXEncoderPrev(double x_encoder_prev) {
    return this->x_encoder_prev = x_encoder_prev;
}

double PoseEstimator::setYEncoderPrev(double y_encoder_prev) {
    return this->y_encoder_prev = y_encoder_prev;
}

double PoseEstimator::setThetaEncoderPrev(double theta_encoder_prev) {
    return this->theta_encoder_prev = theta_encoder_prev;
}

double PoseEstimator::setXEncoderDot(double x_encoder_dot) {
    return this->x_encoder_dot = x_encoder_dot;
}

double PoseEstimator::setYEncoderDot(double y_encoder_dot) {
    return this->y_encoder_dot = y_encoder_dot;
}

double PoseEstimator::setThetaEncoderDot(double theta_encoder_dot) {
    return this->theta_encoder_dot = theta_encoder_dot;
}

// Definitions of Gyro set functions
//
//
double PoseEstimator::setXGyro(double x_gyro) { return this->x_gyro = x_gyro; }

double PoseEstimator::setYGyro(double y_gyro) { return this->y_gyro = y_gyro; }

double PoseEstimator::setThetaGyro(double theta_gyro) {
    return this->theta_gyro = theta_gyro;
}

double PoseEstimator::setXGyroPrev(double x_gyro_prev) {
    return this->x_gyro_prev = x_gyro_prev;
}
int PoseEstimator::setCounter(int count) { return this->counter = count; }

double PoseEstimator::setYGyroPrev(double y_gyro_prev) {
    return this->y_gyro_prev = y_gyro_prev;
}

double PoseEstimator::setThetaGyroPrev(double theta_gyro_prev) {
    return this->theta_gyro_prev = theta_gyro_prev;
}

double PoseEstimator::setXGyroDot(double x_gyro_dot) {
    return this->x_gyro_dot = x_gyro_dot;
}

double PoseEstimator::setYGyroDot(double y_gyro_dot) {
    return this->y_gyro_dot = y_gyro_dot;
}

double PoseEstimator::setThetaGyroDot(double theta_gyro_dot) {
    return this->theta_gyro_dot = theta_gyro_dot;
}
// set functions for variables and constructors;
//
double PoseEstimator::setTimeCurr(double time_c) {
    return this->t_curr = time_c;
}

double PoseEstimator::setTimePrev(double time_prev) {
    return this->t_prev = time_prev;
}

double PoseEstimator::setWheelRad(double wheel_rad) {
    return this->wheel_rad = wheel_rad;
}

int PoseEstimator::setEncoderTicksCurr(int enco_ticks) {
    return this->encoder_ticks_curr = enco_ticks;
}
int PoseEstimator::setEncoderTicksPrev(int enco_ticks_prev) {
    return this->encoder_ticks_prev = enco_ticks_prev;
}
double PoseEstimator::setDistaceBetWheels(
    double distance_between_front_back_wheels) {
    return this->distance_between_front_back_wheels =
               distance_between_front_back_wheels;
}
double PoseEstimator::setSteeringAngle(double steering_angle) {
    return this->steering_angle = steering_angle;
}
double PoseEstimator::setAngularVelocity(double ang_vel) {
    return this->theta_encoder_dot = ang_vel;
}

double PoseEstimator::setFiltergain(double filter_gain) {
    return this->filter_gain = filter_gain;
}
