
#ifndef DISPATCH_H
#define DISPATCH_H

#include "Cluster.h"
#include "Position.h"
#include "RemoteAPIClient.h"

void Dispatch(Cluster* cluster_array,bool* act, Position* position,std::string type,RemoteAPIObject::sim sim);

#endif //DISPATCH_H
