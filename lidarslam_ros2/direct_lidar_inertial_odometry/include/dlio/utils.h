/***********************************************************
 *                                                         *
 * Copyright (c)                                           *
 *                                                         *
 * The Verifiable & Control-Theoretic Robotics (VECTR) Lab *
 * University of California, Los Angeles                   *
 *                                                         *
 * Authors: Kenny J. Chen, Ryan Nemiroff, Brett T. Lopez   *
 * Contact: {kennyjchen, ryguyn, btlopez}@ucla.edu         *
 *                                                         *
 ***********************************************************/

/*
Provides small ROS parameter helpers shared by the DLIO nodes.
*/

#include "rclcpp/rclcpp.hpp"

namespace dlio {

    /*
    Preserves a template type so a default parameter value can participate in deduction.
    */
    template <typename T>
    struct identity { typedef T type; };

    /*
    Declares a ROS parameter with its default and immediately reads its effective value.
    */
    template <typename T>
    void declare_param(rclcpp::Node* node, const std::string param_name, T& param, const typename identity<T>::type& default_value) {
        node->declare_parameter(param_name, default_value);
        node->get_parameter(param_name, param);
    }

}
