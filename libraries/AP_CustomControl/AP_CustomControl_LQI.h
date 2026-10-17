#pragma once
#include "AP_CustomControl_Backend.h"

#if AP_PLANE_CUSTOMCONTROL_LQI_ENABLED
class AP_CustomControl_LQI : public AP_CustomControl_Backend {
public:
    using AP_CustomControl_Backend::AP_CustomControl_Backend;
    bool is_mimo() const override { return true; }
    bool can_run() override;
    void update() override {}
    void reset() override {}
    bool engage(const AP_CustomControl_MIMO::Input &input) override;
    bool update_mimo(const AP_CustomControl_MIMO::Input &input,
                     AP_CustomControl_MIMO::Demand &demand) override;
    void allocation_feedback(const AP_CustomControl_MIMO::Allocation &allocation) override;

private:
    bool valid(const AP_CustomControl_MIMO::Input &input) const;
    void measurements(const AP_CustomControl_MIMO::Input &input, float (&y)[9]) const;
    float _state[16];
    float _integral[3];
    float _matching[3];
    float _applied[3];
    float _requested[3];
    float _height_origin;
    float _height_reference;
    float _turn_reference;
    float _elapsed;
    uint32_t _reset_sequence;
    AP_CustomControl_MIMO::Demand _held;
};
#endif
