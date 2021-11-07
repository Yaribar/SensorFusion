#ifndef _KALMAN_FILTER_h
#define _KALMAN_FILTER_h

#if defined(ARDUINO) && ARDUINO >= 100
    #include "arduino.h"
#else
    #include "WProgram.h"
#endif

#include <BasicLinearAlgebra.h>
using namespace BLA;

class Kalman{
    //HOLA BRO 
    public:
    Kalman();
    ~Kalman();

    BLA::Matrix<2,1> xk_pred(BLA::Matrix<2,2> F, BLA::Matrix<2,1> G, BLA::Matrix<2,1> xk_1, BLA::Matrix<1,1> uk_1);
    BLA::Matrix<2,2> Pk_pred(BLA::Matrix<2,2> F,BLA::Matrix<2,2> Pk_1_correc,BLA::Matrix<2,2> Qk_1);
    BLA::Matrix<2,1> K(BLA::Matrix<2,2> P, BLA::Matrix<1,2> H, BLA::Matrix<1,1> R);
    BLA::Matrix<2,1> xk_correc(BLA::Matrix<2,1> X,BLA::Matrix<2,1> K,BLA::Matrix<1,1> y, BLA::Matrix<1,2> H);

    private:

};

#endif