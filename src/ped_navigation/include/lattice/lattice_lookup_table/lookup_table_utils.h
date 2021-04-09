#ifndef __LOOKUP_TABLE_UTILS_H
#define __LOOKUP_TABLE_UTILS_H

#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <Eigen/Dense>

#include "lattice/diff_drive_generator/motion_model_diff_drive.h"

namespace LookupTableUtils
{
    class StateWithControlParams
    {
    public:
        StateWithControlParams(void);

        Eigen::Vector3d state;// x, y, yaw
        MotionModelDiffDrive::ControlParams control;
    private:
    };

    typedef std::map<double, std::map<double, std::vector<StateWithControlParams> > > LookupTable;

    bool load_lookup_table(const std::string&, LookupTable&);

    void get_optimized_param_from_lookup_table(const LookupTable&, const Eigen::Vector3d, const double, const double, MotionModelDiffDrive::ControlParams&);
    void get_optimized_params_from_lookup_table(const LookupTable&, const Eigen::Vector3d, const double, const double, vector<MotionModelDiffDrive::ControlParams> &params_);
}

#endif// __LOOKUP_TABLE_UTILS_H
