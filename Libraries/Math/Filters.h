///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       Math
//  Workfile:        Filters.h
//  Revision:        1.0 - 8 June, 2016
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Collection of filtering functions
//
///////////////////////////////////////////////////////////////////////////////

#ifndef FILTER_H
#define FILTER_H

//#include "../../portable.h"
#include "NumericalMath.h"
#include "MatrixMath.h"

using namespace std;

namespace Math
{
  //! @ingroup Math
  //!
  //! @brief Simple (posterior, controlless) discrete (linear) Kalman Filter
  //!
  //!
  class LIBRARY_API SimpleKalman
  {
  public:
    //! @brief Default constructor
    //!
    SimpleKalman();

    //! @brief Default destructor
    //!
    ~SimpleKalman();

    //! @brief Add a new time-series reading to the Kalman filter equations, update the covariance
    //!        matrices, and present an updated estimate of the new state
    //!
    //! @param newEst     Output of the function providing an updated state estimate
    //! @param curReading The current (noisy) sensor reading
    //! @param noise      The sensor noise covariance for updating our covariance estimates
    //!
    //! @return True if the state estimation function completes successfully and provides a valid
    //!         value, false otherwise
    //!
    bool updateEstimate(vector<double> &newEst,
                        vector<double> &curReading,
                        vector<double> &noise);

    //! @brief Clear the state and estimate history and restart the filter
    //!
    //! @return True if the reset was completed successfully, false otherwise
    //!
    bool reset();

  private:
    //! @brief Whether or not the variables have been initialized, set to true during intialization
    //!        and during a reset() command call
    //!
    bool cleanSlate_;

    //! @brief Covariance matrix for the Kalman filter
    //!
    vector<double> covariance_;

    //! @brief Previous state estimates
    //!
    vector<double> prevEst_;
  };



  //! @ingroup Math
  //!
  //! @brief   Discrete (linear) Kalman Filter 
  //!
  class LIBRARY_API Kalman
  {
  public:
    //! @brief Default constructor
    //!
    Kalman();

    //! @brief Default destructor
    //!
    ~Kalman();

    //! @brief Initialization of the Kalman filter with initial estimates of the prediction and
    //!        control matrices as well as the noise model.  Defaults to unit matrices if NULL.
    //!        Assumes Kalman filter model of x_est_k = (F * x_est_k-1) + (B * u_k) + w_k-1
    //!
    //! @param prediction Prediction matrix (F)
    //! @param control    Control matrix (B)
    //! @param noise      The noise covariance (Q) for updating our covariance matrix (Optional,
    //!                   assumed gaussian with stdev 0.25)
    //! @param stateMes   Tranformation matrix mapping state vectors to the measurement domain
    //!
    void init(matrix *prediction, matrix *control, matrix *noise, matrix *stateMes);

    //! @brief Add a new time-series reading to the Kalman filter equations, update the covariance
    //!        matrices, and present an updated estimate of the new state
    //!
    //! @param newEst     Output of the function providing an updated state estimate
    //! @param curReading The current (noisy) sensor reading
    //! @param control    The control vector (if |control| == 0, there is no external influence)
    //! @param procNoise  The process noise (w) added when updating the a priori state estimates
    //! @param measNoise  The measurement noise from the state observation
    //!
    //! @return True if the state estimation function completes successfully and provides a valid
    //!         value, false otherwise
    //!
    bool updateEstimate(vector<double> &newEst,
                        vector<double> &curReading, 
                        vector<double> &control,
                        vector<double> &procNoise,
                        vector<double> &measNoise);

    //! @brief Clear the state and estimate history and restart the filter
    //!
    //! @return True if the reset was completed successfully, false otherwise
    //!
    bool reset();

  private:

    //! @brief Whether or not the variables have been initialized, set to true during intialization
    //!        and during a reset() command call
    //!
    bool cleanSlate_;

    //! @brief TODO
    //!
    matrix *prediction_;

    //! @brief TODO
    //!
    matrix *control_;

    //! @brief Covariance matrix for the Kalman filter
    //!
    matrix *covariance_;

    //! @brief The previous state estimate (initialized to 0 on construction and during reset)
    //!
    vector<double> prevEst_;

    //! @brief The mapping from the state variables to the measured outputs
    //!
    matrix *stateMes_;

    //! @brief Noise model for the Kalman filter
    //!
    matrix *noise_;
  }; // Kalman
} // namespace Math


#endif