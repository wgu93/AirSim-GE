// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_multirotorphysicsbody_hpp
#define msr_airlib_multirotorphysicsbody_hpp

#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "RotorActuator.hpp"
#include "api/VehicleApiBase.hpp"
#include "api/VehicleSimApiBase.hpp"
#include "MultiRotorParams.hpp"
#include <vector>
#include "physics/PhysicsBody.hpp"

#include "MultiRotorGroundEffect.hpp" // Weibin

namespace msr
{
namespace airlib
{

    class MultiRotorPhysicsBody : public PhysicsBody
    {
    public:
        MultiRotorPhysicsBody(MultiRotorParams* params, VehicleApiBase* vehicle_api,
                              Kinematics* kinematics, Environment* environment) 
            : params_(params), environment_(environment), vehicle_api_(vehicle_api) // Weibin: environment
        {
            setName("MultiRotorPhysicsBody");
            vehicle_api_->setParent(this);
            initialize(kinematics, environment);
            multirotor_ge_ = new MultiRotorGroundEffect; // Weibin
        }

        //*** Start: UpdatableState implementation ***//
        virtual void resetImplementation() override
        {
            //reset rotors, kinematics and environment
            PhysicsBody::resetImplementation();

            //reset sensors last after their ground truth has been reset
            resetSensors();
        }

        virtual void update() override
        {
            //update forces on vertices that we will use next
            PhysicsBody::update();

            // Weibin
            test_ground_truth_z_ = - environment_->getState().position.z(); // take negative to compare with distance sensor data

            //Note that controller gets updated after kinematics gets updated in updateKinematics
            //otherwise sensors will have values from previous cycle causing lags which will appear
            //as crazy jerks whenever commands like velocity is issued
        }
        virtual void reportState(StateReporter& reporter) override
        {
            //call base
            PhysicsBody::reportState(reporter);

            reportSensors(*params_, reporter);

            //report rotors
            for (uint rotor_index = 0; rotor_index < rotors_.size(); ++rotor_index) {
                reporter.startHeading("", 1);
                reporter.writeValue("Rotor", rotor_index);
                reporter.endHeading(false, 1);
                rotors_.at(rotor_index).reportState(reporter);
            }
        }
        //*** End: UpdatableState implementation ***//

        //Fast Physics engine calls this method to set next kinematics
        virtual void updateKinematics(const Kinematics::State& kinematics) override
        {
            PhysicsBody::updateKinematics(kinematics);

            updateSensorsAndController();
        }

        //External Physics engine calls this method to keep physics bodies updated and move rotors
        virtual void updateKinematics() override
        {
            PhysicsBody::updateKinematics();

            updateSensorsAndController();
        }

        void updateSensorsAndController()
        {
            updateSensors(*params_, getKinematics(), getEnvironment());

            //update controller which will update actuator control signal
            vehicle_api_->update();

            //transfer new input values from controller to rotors
            for (uint rotor_index = 0; rotor_index < rotors_.size(); ++rotor_index) {
                rotors_.at(rotor_index).setControlSignal(vehicle_api_->getActuation(rotor_index));

                if (rotors_.size() == 4) // Weibin
                {
                    control_input_[rotor_index] = vehicle_api_->getActuation(rotor_index);
                }
            }

            // Utils::log(Utils::stringf("Control input is %s, %s, %s, %s", 
            //             std::to_string(control_input_[0]).c_str(),
            //             std::to_string(control_input_[1]).c_str(),
            //             std::to_string(control_input_[2]).c_str(),
            //             std::to_string(control_input_[3]).c_str()));
        }

        //sensor getter
        const SensorCollection& getSensors() const
        {
            return params_->getSensors();
        }

        //physics body interface
        virtual uint wrenchVertexCount() const override
        {
            return params_->getParams().rotor_count;
        }
        virtual PhysicsBodyVertex& getWrenchVertex(uint index) override
        {
            return rotors_.at(index);
        }
        virtual const PhysicsBodyVertex& getWrenchVertex(uint index) const override
        {
            return rotors_.at(index);
        }

        virtual uint dragVertexCount() const override
        {
            return static_cast<uint>(drag_faces_.size());
        }
        virtual PhysicsBodyVertex& getDragVertex(uint index) override
        {
            return drag_faces_.at(index);
        }
        virtual const PhysicsBodyVertex& getDragVertex(uint index) const override
        {
            return drag_faces_.at(index);
        }

        virtual real_T getRestitution() const override
        {
            return params_->getParams().restitution;
        }
        virtual real_T getFriction() const override
        {
            return params_->getParams().friction;
        }

        // Weibin
        virtual real_T getDistanceCustomData() const override
        {
            return vehicle_api_->getDistanceSensorData(DISTANCE_SENSOR_NAME).distance;
        }

        // Weibin
        virtual bool getLidarCustomData(real_T min_dist[3]) const override
        {
            vector<real_T> point_cloud = vehicle_api_->getLidarData(LIDAR_NAME).point_cloud;
            if (point_cloud.size() > 3)
            {
                static int counter = 0;
                static real_T x_min = fabs(point_cloud.at(0));
                static real_T y_min = fabs(point_cloud.at(1));
                static real_T z_min = fabs(point_cloud.at(2));
                x_min = fabs(point_cloud.at(0));
                y_min = fabs(point_cloud.at(1));
                z_min = fabs(point_cloud.at(2));
                for (auto it = point_cloud.begin(); it != point_cloud.end(); it++)
                {
                    switch (counter % 3)
                    {
                    case 0: // x
                    {
                        x_min = (fabs(*it) < x_min) ? fabs(*it) : x_min;
                        break;
                    }
                    case 1: // y
                    {
                        y_min = (fabs(*it) < y_min) ? fabs(*it) : y_min;
                        break;
                    }
                    case 2: // z
                    {
                        z_min = (fabs(*it) < z_min) ? fabs(*it) : z_min;
                        break;
                    }
                    }
                    counter++;
                }
                counter = 0;
                min_dist[0] = x_min;
                min_dist[1] = y_min;
                min_dist[2] = z_min;
                return true;
            }
            else
            {
                real_T dummy_val = 0.0f/0.0f;
                min_dist[0] = dummy_val;
                min_dist[1] = dummy_val;
                min_dist[2] = dummy_val;
                return false;
            }
            
        }

        // Weibin
        virtual real_T getGroundEffect() override 
        {
            // Init ground effect for once
            static bool is_ge_init = false; // set true if ground effect is initialized
            if (is_ge_init == false)
            {
                real_T mass = params_->getParams().mass;
                real_T prop_radius_in_mm = 1000.0f * 0.5f * params_->getParams().rotor_params.propeller_diameter;
                real_T prop_spacing_in_mm = 1000.0f * 2.0f * params_->getParams().arm_length;

                multirotor_ge_->init(mass, prop_radius_in_mm, prop_spacing_in_mm, ENV_GRAVITY_CONST);
                is_ge_init = true;
            }
            
            // Get ground effect model from APIs
            MultiRotorGroundEffect::GE_MODEL model;
            if (vehicle_api_->ground_effect_model_ == 1) 
                model = MultiRotorGroundEffect::GE_MODEL::CHEESEMAN_BENNETT;
            else if (vehicle_api_->ground_effect_model_ == 2) 
                model = MultiRotorGroundEffect::GE_MODEL::CONYERS_PARAMETRIC;
            else
                model = MultiRotorGroundEffect::GE_MODEL::NO_GROUND_EFFECT;
            ground_effect_model_ = vehicle_api_->ground_effect_model_;

            // Get altitude above ground
            real_T alt_dist_sensor = vehicle_api_->getDistanceSensorData(DISTANCE_SENSOR_NAME).distance; // NOT WORKING: has bias
            // real_T alt_ground_truth = test_ground_truth_z_ - SIM_GROUND_TRUTH_Z_BIAS; // NOT WORKING: has bias. Minus because in NED frame
            real_T sim_alt_ground_truth = sim_ground_truth_z_; // unbiased ground truth of altitude above ground
            // multirotor_ge_->getAltitudeAboveGround(alt_dist_sensor); // from distance sensor
            multirotor_ge_->getAltitudeAboveGround(-sim_alt_ground_truth);  // from ground truth in NED, more robust for modeling to attitude changes
            // Utils::log(Utils::stringf("TEST: Lidar Gnd Truth = %s [m], Alt = %s [m], Gnd Truth = %s [m]", 
            //                         std::to_string(sim_alt_ground_truth).c_str(), 
            //                         std::to_string(alt_ground_truth).c_str(), 
            //                         std::to_string(test_ground_truth_z_).c_str()));

            // Get total body and drag wrench in z axis
            multirotor_ge_->getCurrentBodyAndDragWrench(total_body_wrench_z_, total_drag_wrench_z_);

            // Update ground effect calculation
            multirotor_ge_->update(model); 

            // Get thrust changes and ratio between IGE and OGE 
            real_T delta_thrust = multirotor_ge_->getThrustChange();
            //real_T ratio_thrust = multirotor_ge_->getThrustRatio();

            // Print info to the terminal
            multirotor_ge_->print_info();

            return delta_thrust;
        }

        RotorActuator::Output getRotorOutput(uint rotor_index) const
        {
            return rotors_.at(rotor_index).getOutput();
        }

        virtual ~MultiRotorPhysicsBody() = default;

    private: //methods
        void initialize(Kinematics* kinematics, Environment* environment)
        {
            PhysicsBody::initialize(params_->getParams().mass, params_->getParams().inertia, kinematics, environment);

            createRotors(*params_, rotors_, environment);
            createDragVertices();

            initSensors(*params_, getKinematics(), getEnvironment());
        }

        static void createRotors(const MultiRotorParams& params, vector<RotorActuator>& rotors, const Environment* environment)
        {
            rotors.clear();
            //for each rotor pose
            for (uint rotor_index = 0; rotor_index < params.getParams().rotor_poses.size(); ++rotor_index) {
                const MultiRotorParams::RotorPose& rotor_pose = params.getParams().rotor_poses.at(rotor_index);
                rotors.emplace_back(rotor_pose.position, rotor_pose.normal, rotor_pose.direction, params.getParams().rotor_params, environment, rotor_index);
            }
        }

        void reportSensors(MultiRotorParams& params, StateReporter& reporter)
        {
            params.getSensors().reportState(reporter);
        }

        void updateSensors(MultiRotorParams& params, const Kinematics::State& state, const Environment& environment)
        {
            unused(state);
            unused(environment);
            params.getSensors().update();
        }

        void initSensors(MultiRotorParams& params, const Kinematics::State& state, const Environment& environment)
        {
            params.getSensors().initialize(&state, &environment);
        }

        void resetSensors()
        {
            params_->getSensors().reset();
        }

        void createDragVertices()
        {
            const auto& params = params_->getParams();

            //Drone is seen as central body that is connected to propellers via arm. We approximate central body as box of size x, y, z.
            //The drag depends on area exposed so we also add area of propellers to approximate drag they may introduce due to their area.
            //while moving along any axis, we find area that will be exposed in that direction
            real_T propeller_area = M_PIf * params.rotor_params.propeller_diameter * params.rotor_params.propeller_diameter;
            real_T propeller_xsection = M_PIf * params.rotor_params.propeller_diameter * params.rotor_params.propeller_height;

            real_T top_bottom_area = params.body_box.x() * params.body_box.y();
            real_T left_right_area = params.body_box.x() * params.body_box.z();
            real_T front_back_area = params.body_box.y() * params.body_box.z();
            Vector3r drag_factor_unit = Vector3r(
                                            front_back_area + rotors_.size() * propeller_xsection,
                                            left_right_area + rotors_.size() * propeller_xsection,
                                            top_bottom_area + rotors_.size() * propeller_area) *
                                        params.linear_drag_coefficient / 2;

            //add six drag vertices representing 6 sides
            drag_faces_.clear();
            drag_faces_.emplace_back(Vector3r(0, 0, -params.body_box.z() / 2.0f), Vector3r(0, 0, -1), drag_factor_unit.z());
            drag_faces_.emplace_back(Vector3r(0, 0, params.body_box.z() / 2.0f), Vector3r(0, 0, 1), drag_factor_unit.z());
            drag_faces_.emplace_back(Vector3r(0, -params.body_box.y() / 2.0f, 0), Vector3r(0, -1, 0), drag_factor_unit.y());
            drag_faces_.emplace_back(Vector3r(0, params.body_box.y() / 2.0f, 0), Vector3r(0, 1, 0), drag_factor_unit.y());
            drag_faces_.emplace_back(Vector3r(-params.body_box.x() / 2.0f, 0, 0), Vector3r(-1, 0, 0), drag_factor_unit.x());
            drag_faces_.emplace_back(Vector3r(params.body_box.x() / 2.0f, 0, 0), Vector3r(1, 0, 0), drag_factor_unit.x());
        }

    private: //fields
        MultiRotorParams* params_;

        //let us be the owner of rotors object
        vector<RotorActuator> rotors_;
        vector<PhysicsBodyVertex> drag_faces_;

        std::unique_ptr<Environment> environment_; // unused
        VehicleApiBase* vehicle_api_;

        MultiRotorGroundEffect* multirotor_ge_; // Weibin
        std::string DISTANCE_SENSOR_NAME = "DistanceCustom"; // Weibin
        std::string LIDAR_NAME = "LidarCustom"; // Weibin
        real_T test_ground_truth_z_; // Weibin
    };
}
} //namespace
#endif
