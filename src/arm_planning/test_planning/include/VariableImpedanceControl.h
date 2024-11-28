/**
 * @file VariableImpedanceControl.h
 * @brief head file of Variable Impedance Controller
 * @version 1.0.0
 * @date 11.21 2024
 */

#include <ImpedanceControl.h>

class VariableImpedanceControl : public ImpedanceControl{

public:
    /** destructor */
    virtual ~VariableImpedanceControl() override = default;  

    bool updateState(const ImpedanceControlState& state) override;
private:

};