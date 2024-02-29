//
// Created by jjgomez on 30/01/23.
//

#include <iostream>

#include <Regresion/Regression.h>

using namespace std;

int main() {
    // Creates a new regression.
    Regression* regression = new Regression("Lab0/Data/PisosTrain.txt");

    // Computes the optimal weights.
    regression->ComputeRegressionWeights();

    // Retrieve the optimal weights.
    auto weights = regression->GetComputedWeights();

    // Compute the RMSE of the solution computed.
    const float rmse = regression->ComputeRMSE();
    std::cout << "RMSE = " << rmse << endl;

    // What happens if you delete the memory associated to the regression here?
    // delete regression;

    // Estimate the value of an apartment of 100 m² and 4 rooms
    Eigen::Vector3f v = Eigen::Vector3f(1.0, 100, 4);
    const float pred = regression->Estimate(v);
    std::cout << "Predicted price = " << pred << endl;

    delete regression;

    return 0;
}
