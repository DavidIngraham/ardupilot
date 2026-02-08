#pragma once

/*
 Generic PI for systems like heater control, no filtering
*/

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

class AC_PI {
public:
    // Constructor
    // bidirectional: if true, integrator can go negative; if false (default), clamps to [0, imax] for heater use
    AC_PI(float initial_p, float initial_i, float initial_imax, bool bidirectional = false);

    CLASS_NO_COPY(AC_PI);

    // update controller
    float update(float measurement, float target, float dt);
    float update(float meassurement, float target, float df, bool limit_neg, bool limit_pos);

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

    float get_P() const {
        return output_P;
    }
    float get_I() const {
        return integrator;
    }

    // Resets the integrator to zero.
    void reset_I();

protected:
    AP_Float        kP;
    AP_Float        kI;
    AP_Float        imax;
    float           integrator;
    float           output_P;

private:
    const float _default_kp;
    const float _default_ki;
    const float _default_imax;
    bool _bidirectional; 
};
