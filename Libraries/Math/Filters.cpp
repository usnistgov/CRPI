///////////////////////////////////////////////////////////////////////////////
//
//  Original System: Collaborative Robot Programming Interface
//  Subsystem:       Math
//  Workfile:        Filters.cpp
//  Revision:        1.0 - 8 June, 2016
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Collection of math functions applicable to filtering algorithms
//
///////////////////////////////////////////////////////////////////////////////

#include "Filters.h"


namespace Math
{
  ///////////////////////////////////////////////////////////////////////////////
  //!         Simple (Posterior, Controlless)  Discrete Kalman Filter         !//
  ///////////////////////////////////////////////////////////////////////////////

  LIBRARY_API SimpleKalman::SimpleKalman()
  {
    cleanSlate_ = true;
  }


  LIBRARY_API SimpleKalman::~SimpleKalman()
  {
    prevEst_.clear();
    covariance_.clear();
  }


  LIBRARY_API bool SimpleKalman::updateEstimate(vector<double> &newEst,
                                                vector<double> &curReading,
                                                vector<double> &noise)
  {
    double gain;
    double ns;

    newEst.resize(curReading.size());
 
    if (cleanSlate_)
    {
      covariance_.clear();
      prevEst_.clear();
      for (int i = 0; i < curReading.size(); ++i)
      {
        covariance_.push_back(1.0f);
        prevEst_.push_back(0.0f);
      }
      cleanSlate_ = false;
    }

    //! Verify that we are expecting an appropriate ammount of data
    if (curReading.size() != covariance_.size())
    {
      return false;
    }

    for (int i = 0; i < curReading.size(); ++i)
    {
      if (noise.size() == 0)
      {
        ns = 0.1f;
      }
      else if ((noise.size() == 1) || (noise.size() != curReading.size()))
      {
        ns = noise.at(0);
      }
      else
      {
        ns = noise.at(i);
      }

      //! Calculate gains
      gain = covariance_.at(i) / (covariance_.at(i) + ns);

      //! Update the pose estimate
      newEst.at(i) = prevEst_.at(i) + (gain * (curReading.at(i) - prevEst_.at(i)));

      //! Update the covariance
      covariance_.at(i) = ((1 - gain) * covariance_.at(i));
    } // for (int i = 0; i < curReading.size(); ++i)

    return true;
  }


  LIBRARY_API bool SimpleKalman::reset()
  {
    cleanSlate_ = true;
    return true;
  }



  ///////////////////////////////////////////////////////////////////////////////
  //!                        Discrete Kalman Filter                           !//
  ///////////////////////////////////////////////////////////////////////////////

  LIBRARY_API Kalman::Kalman()
  {
    cleanSlate_ = true;
    covariance_ = new matrix(3, 3);
    prediction_ = new matrix(3, 3);
    control_ = new matrix(3, 3);
    noise_ = new matrix(3, 3);
    stateMes_ = new matrix(3, 3);
  }


  LIBRARY_API Kalman::~Kalman()
  {
    delete covariance_;
    delete prediction_;
    delete control_;
    delete noise_;
    delete stateMes_;
  }


  LIBRARY_API void Kalman::init(matrix *prediction, matrix *control, matrix *noise, matrix *stateMes)
  {
    if (prediction == NULL)
    {
      prediction_->valid = false;
    } // if (prediction == NULL)
    else
    {
      *prediction_ = *prediction;
    } // if (prediction == NULL) ... else


    if (control == NULL)
    {
      control_->valid = false;
    } // if (control == NULL)
    else
    {
      *control_ = *control;
    } // if (control == NULL) ... else


    if (noise == NULL)
    {
      noise_->valid = false;
    }
    else
    {
      *noise_ = *noise;
    }

    if (stateMes == NULL)
    {
      stateMes_->valid = false;
    }
    else
    {
      *stateMes_ = *stateMes;
    }

    cleanSlate_ = true;
  }


  //! JAM:  TODO:  need checks in here to verify that things have completed successfully
  LIBRARY_API bool Kalman::updateEstimate(vector<double> &newEst,
                                          vector<double> &curReading,
                                          vector<double> &control,
                                          vector<double> &procNoise,
                                          vector<double> &measNoise)
  {
    int i;

    if (prediction_ == NULL)
    {
      return false;
    }

    if (cleanSlate_)
    {
      covariance_->covariance(curReading, curReading);
      if (noise_ == NULL)
      {
        if (procNoise.size() > 0)
        {
          noise_->covariance(procNoise, procNoise);
        }
        else
        {
          noise_->resize(curReading.size(), curReading.size());
          noise_->setAll(0.1f);
        }
      }
    }

    //! Compute a priori state estimate
    //! w_k = process noise
    //! x_(k) = (F_(k) * x_(k-1)) + (B_(k) * u_(k)) + w_(k)
    matrix xknew, xkold, u, w;
    xkold = prevEst_;
    if (control.size() > 0)
    {
      u = control;
    }
    if (procNoise.size() > 0)
    {
      w = procNoise;
    }

    xknew = (*prediction_ * xkold);
    if (control_ != NULL && (control.size() > 0))
    {
      u = control;
      xknew = xknew + (*control_ * u);
    }
    if (procNoise.size() > 0)
    {
      w = procNoise;
      xknew = xknew + w;
    }

    //! Compute a priori estimate error
    matrix PKnew;
    PKnew = ((*prediction_) * (*covariance_) * prediction_->trans()) + *noise_;

    //! Compute a posteriori error blending factor (Kalman gain)
    matrix Kk, temp, R;
    R.covariance(measNoise, measNoise);
    temp = (*stateMes_ * PKnew * stateMes_->trans() + R);
    Kk = (PKnew * stateMes_->trans()) *  temp.inv();

    //! Update the state estimate based on the error factors
    matrix xknew2, z;
    z = measNoise;
    xknew2 = xknew + (Kk * (z - (*stateMes_ * xknew)));
    newEst.clear();
    for (i = 0; i < xknew2.rows; ++i)
    {
      newEst.push_back(xknew2.at(i, 0));
    }

    //! Update the estimate error covariance matrix
    matrix I;
    I.identity(Kk.rows);
    *covariance_ = (I - (Kk * *stateMes_)) * PKnew;

    return covariance_->valid;
  }


  LIBRARY_API bool Kalman::reset()
  {
    cleanSlate_ = true;
    //! TODO
    return true;
  }



}

