#include "lattice_lookup_table/lookup_table_generator.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lookup_table_generator");
    LookupTableGenerator lookup_table_generator;
    std::string data = lookup_table_generator.state_process();
    lookup_table_generator.state_save(data);
    return 0;
}
