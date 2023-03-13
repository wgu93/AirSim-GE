// Weibin Gu
// Politecnico di Torino @ CSL
// Contact: weibin.gu@polito.it / weibin_gu@yeah.net
// June 21, 2021


#ifndef msr_airlib_MultiRotorGroundEffect_hpp
#define msr_airlib_MultiRotorGroundEffect_hpp

// AirSim
#include "common/Common.hpp"
// Math
#include <cmath>
// Timer
#include <ctime> 
#include <ratio>
#include <chrono>
// Data logging
#include "common/common_utils/CSVWriter.hpp"

namespace msr
{
namespace airlib
{

    class MultiRotorGroundEffect
    {

    /************************* Public Attributes *************************/
    public:
        enum GE_MODEL
        {
            NO_GROUND_EFFECT = 0, // no ground effect
            CHEESEMAN_BENNETT = 1, // Cheeseman-Bennett ground effect model
            CONYERS_PARAMETRIC = 2, // Parametric model developed by Stephen Conyers
        };

        real_T GRAVITY_CONST = 0.0f; // (scalar) gravity constant [N/kg] 
        real_T MASS = 0.0f; // multirotor mass [kg]
        real_T PROP_RADIUS_IN_MM = 0.0f; // propeller radius [mm]
        real_T PROP_SPACING_IN_MM = 0.0f; // propeller spacing = 2 times of arm length [mm]

        real_T alt_above_gnd_ = 0.0f; // multirotor altitude above ground [m]
        real_T delta_thrust_ = 0.0f; // increment or reduction of total thrust (along vertical/z axis) due to ground effect [N]
        real_T ratio_thrust_ = 0.0f; // thrust ratio between IGE and OGE 
        real_T ratio_Z_over_R_ = 0.0f; // ratio between altitude above ground and propeller radius (should be in same units!)

    /************************* Public Functions *************************/
    public:

        MultiRotorGroundEffect()
        {
            csv_writer_ = new CSVWriter("/home/weibin/Documents/AirSim/ground_effect.csv"); // TODO: CHANGE HERE WITH YOUR OWN FILE SAVING DIR
        }

        ~MultiRotorGroundEffect()
        {
            delete csv_writer_;
        }

        // Retrieve multirotor data
        void init(real_T mass, real_T prop_radius_in_mm, real_T prop_spacing_in_mm, real_T gravity)
        {
            MASS = mass;
            PROP_RADIUS_IN_MM = prop_radius_in_mm;
            PROP_SPACING_IN_MM = prop_spacing_in_mm;
            GRAVITY_CONST = fabs(gravity);

            // Set up csv file
            std::vector<std::string> col_title = { "IGE_over_OGE", "Z_over_R", "delta_thrust", "Z", "R"};
            csv_writer_->addDatainRow(col_title.begin(), col_title.end());
        }

        // Retrieve multirotor altitude above the ground from distance sensor/ground truth
        void getAltitudeAboveGround(real_T alt_above_gnd)
        {
            alt_above_gnd_ = alt_above_gnd;
        }

        // Retrieve output body and drag wrench along z axis of multirotor at current time step
        void getCurrentBodyAndDragWrench(real_T body, real_T drag)
        {
            cur_body_wrench_ = body;
            cur_drag_wrench_ = drag;
        }

        // Calculate thrust increment or reduction (along z axis) due to ground effect
        void update(GE_MODEL model)
        {
            real_T ratio_ige_over_oge;

            // Select ground effect model
            switch(model)
            {
                case CHEESEMAN_BENNETT:
                {
                    ratio_ige_over_oge = getRatioCheesemanBennettModel();
                    model_ = CHEESEMAN_BENNETT;
                    // Utils::log("Cheeseman-Bennett model is used.");
                    break;
                }
                case CONYERS_PARAMETRIC:
                {
                    ratio_ige_over_oge = getRatioConyersParametricModel();
                    model_ = CONYERS_PARAMETRIC;
                    // Utils::log("Stephen Conyers's parametric model is used.");
                    break;
                }
                default: // no ground effect
                {
                    ratio_ige_over_oge = 1;
                    model_ = NO_GROUND_EFFECT;
                    // Utils::log("No ground effect.");
                }
            }

            // T_OGE calculation (scalar)
            //real_T T_OGE = MASS * GRAVITY_CONST; // simple implementation by assuming T_OGE = multirotor weight = const.
            real_T T_OGE = fabs(cur_body_wrench_ + cur_drag_wrench_); // realistic implementation by taking summation of total body and drag wrench in z axis as T_OGE
            
            // T_IGE calculation (scalar)
            real_T T_IGE = T_OGE * ratio_ige_over_oge; 

            // Output calculation
            delta_thrust_ = T_IGE - T_OGE;
            ratio_thrust_ = ratio_ige_over_oge;
            ratio_Z_over_R_ = alt_above_gnd_ / (0.001f * PROP_RADIUS_IN_MM);

            // Write data into csv file
            float plot_data [] = {ratio_thrust_, ratio_Z_over_R_, delta_thrust_, alt_above_gnd_, PROP_RADIUS_IN_MM};
            csv_writer_->addDatainRow(plot_data , plot_data + sizeof(plot_data) / sizeof(float));
        }

        // Return total thrust increment or reduction (along vertical axis) due to ground effect
        real_T getThrustChange() const
        {
            return delta_thrust_;
        }  

        // Return thrust ratio between IGE and OGE 
        real_T getThrustRatio() const
        {
            return ratio_thrust_;
        }    

        // Print message to the terminal
        void print_info() const
        {
            static std::chrono::steady_clock::time_point t_ref = std::chrono::steady_clock::now();
            std::chrono::steady_clock::time_point t_now = std::chrono::steady_clock::now();
            std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t_now - t_ref);
            if (time_span.count() > 1) // print every second
            {
                Utils::log("=============================================");
                switch (model_)
                {
                    case NO_GROUND_EFFECT: {Utils::log("No ground effect model is taking effect."); break;}
                    case CHEESEMAN_BENNETT: {Utils::log("Ground effect is taking effect using Cheeseman-Bennett model!"); break;}
                    case CONYERS_PARAMETRIC: {Utils::log("Ground effect is taking effect using Stephen Conyers's parametric model!"); break;}
                    default: {Utils::log("Error: Unknown ground effect model => Please check 'MultiRotorGroundEffect.hpp'.");}
                }
                // Utils::log("++++++");
                // Utils::log(Utils::stringf("Mass is %s [kg]", std::to_string(MASS).c_str()));
                // Utils::log(Utils::stringf("Prop radius is %s [mm]", std::to_string(PROP_RADIUS_IN_MM).c_str()));
                // Utils::log(Utils::stringf("Prop spacing is %s [mm]", std::to_string(PROP_SPACING_IN_MM).c_str()));
                // Utils::log(Utils::stringf("Gravity constant is %s [N/kg]", std::to_string(GRAVITY_CONST).c_str()));
                Utils::log("++++++");
                Utils::log(Utils::stringf("Alt above gnd is %s [m]", std::to_string(alt_above_gnd_).c_str()));
                Utils::log(Utils::stringf("Body wrench is %s [N]", std::to_string(cur_body_wrench_).c_str()));
                Utils::log(Utils::stringf("Drag wrench is %s [N]", std::to_string(cur_drag_wrench_).c_str()));
                Utils::log(Utils::stringf("Sum of body and drag wrench is %s [N]", std::to_string(cur_body_wrench_+ cur_drag_wrench_).c_str()));
                Utils::log("++++++");
                Utils::log(Utils::stringf("Thrust change is %s [N]", std::to_string(delta_thrust_).c_str()));
                //Utils::log(Utils::stringf("Thrust ratio is %s", std::to_string(ratio_thrust_).c_str()));
                //Utils::log(Utils::stringf("Distance ratio is %s", std::to_string(ratio_Z_over_R_).c_str()));
                Utils::log("=============================================");

                t_ref = std::chrono::steady_clock::now();
            }
            
        }

    /************************* Private Attributes *************************/
    private:

        CSVWriter* csv_writer_;
        GE_MODEL model_;
        real_T MAX_Z_OVER_R = 10.0f; // =Z/R, maximum functional value used in Conyers's parametric model
        real_T MIN_Z_OVER_R = 0.25f; // =Z/R, minimum functional value used in both two models
        real_T cur_body_wrench_; // total body wrench along z axis of multirotor at current time step
        real_T cur_drag_wrench_; // total drag wrench along z axis of multirotor at current time step

    /************************* Private Functions *************************/
    private:

        // Calculate thrust ratio between IGE and OGE based on Cheeseman-Bennett model
        real_T getRatioCheesemanBennettModel()
        {
            real_T ratio_ige_over_oge; // = (Thrust_{IGE}) / (Thrust_{OGE})
            real_T prop_radius_in_m = 0.001f * PROP_RADIUS_IN_MM;
            if (alt_above_gnd_ > MIN_Z_OVER_R * prop_radius_in_m)
            {
                // T_{IGE} / T_{OGE} = 1 / ( 1 - ( R / (4Z) )^2 )
                ratio_ige_over_oge = 1.0f / (1.0f - pow(prop_radius_in_m / (4.0f * alt_above_gnd_), 2)); 
            }  
            else
            {
                ratio_ige_over_oge = 1.0f; // it is assumed that there is no ground effect when the multirotor is very close to the ground
            }
            return ratio_ige_over_oge;
        }

        // Calculate thrust ratio between IGE and OGE based on Parametric Model developed by Stephen Conyers (refer to Conyers's PhD dissertation)
        real_T getRatioConyersParametricModel()
        {
            real_T L = PROP_SPACING_IN_MM;
            real_T R = PROP_RADIUS_IN_MM;
            // Coefficients of the parameteric model: eq 6.7 on pg 101
            real_T p1 = -0.0000002794 * pow(L, 2) + 0.0003761 * L + 0.00003197 * pow(R, 2) - 0.0075479 * R + 0.3357119;
            real_T p2 = 0.0000053178 * pow(L, 2) - 0.0071388 * L - 0.0006703 * pow(R, 2) + 0.1643906 * R - 7.0388236;
            real_T p3 = -0.000022529 * pow(L, 2) + 0.0219849 * L + 0.0104943 * pow(R, 2) - 2.6402066 * R + 163.3198037;
            real_T p4 = 0.000032426 * pow(L, 2) - 0.0289233 * L - 0.0156903 * pow(R, 2) + 3.9426111 * R - 248.2897537;
            real_T p5 = -0.00018056 * pow(L, 2) + 0.2518899 * L + 0.0458834 * pow(R, 2) - 11.4527617 * R + 636.8788317;
            real_T q1 = 0.000010736 * pow(L, 2) - 0.0231523 * L + 0.0053983 * pow(R, 2) - 1.3630188 * R + 96.3637597;
            real_T q2 = -0.000044379 * pow(L, 2) + 0.0780762 * L - 0.0024483 * pow(R, 2) + 0.6141837 * R - 71.9829075;
            real_T q3 = -0.00013482 * pow(L, 2) + 0.1883013 * L + 0.0359764 * pow(R, 2) - 8.9660684 * R + 500.2544495;

            real_T prop_radius_in_m = 0.001f * PROP_RADIUS_IN_MM;
            real_T x = alt_above_gnd_ / prop_radius_in_m;
            real_T ratio_ige_over_oge; // = (Thrust_{IGE}) / (Thrust_{OGE})
            ratio_ige_over_oge = (p1 * pow(x, 4) + p2 * pow(x, 3) + p3 * pow(x, 2) + p4 * x + p5) / (pow(x, 3) + q1 * pow(x, 2) + q2 * x + q3); // eq 6.8 on pg 101

            
            if (x > MAX_Z_OVER_R || x < MIN_Z_OVER_R) ratio_ige_over_oge = 1; // when the multirotor is far away from or too close to the ground,
                                                                            // set thrust ratio to 1; this is to avoid numerical instability.

            return ratio_ige_over_oge;
        }

    };
}
} //namespace
#endif
