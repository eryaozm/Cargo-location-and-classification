
#ifndef SIMOBJECT_H
#define SIMOBJECT_H

#include "RemoteAPIClient.h"

class SimObject {
public:
    int handle;
    std::string type;
    std::string path;
    std::vector<double> position;


    SimObject();
};



#endif //SIMOBJECT_H
