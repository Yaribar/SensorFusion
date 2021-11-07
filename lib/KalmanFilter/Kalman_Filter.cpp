#include "Kalman_Filter.h"
#include <BasicLinearAlgebra.h>
using namespace BLA;

Kalman::Kalman()
{
};
Kalman::~Kalman()
{
};

BLA::Matrix<2,1> Kalman::xk_pred(BLA::Matrix<2,2> F, BLA::Matrix<2,1> G, BLA::Matrix<2,1> xk_1, BLA::Matrix<1,1> uk_1)
{
    return (F*xk_1+G*uk_1);
}

BLA::Matrix<2,2> Kalman::Pk_pred(BLA::Matrix<2,2> F,BLA::Matrix<2,2> Pk_1_correc,BLA::Matrix<2,2> Qk_1)
{

    return (F*Pk_1_correc)*(~F)+Qk_1;
}

BLA::Matrix<2,1> Kalman::K(BLA::Matrix<2,2> P, BLA::Matrix<1,2> H, BLA::Matrix<1,1> R){
    /*BLA::Matrix<1,1> operation = H*P*(~H)+R;
    BLA::Matrix<1,1> operation_inv = operation.Inverse();
    return P*(~H)*operation_inv;*/
    return P*(~H);

}

BLA::Matrix<2,1> Kalman::xk_correc(BLA::Matrix<2,1> X,BLA::Matrix<2,1> K,BLA::Matrix<1,1> y, BLA::Matrix<1,2> H){
    return X+K*(y-H*X);
}