#ifndef DDSROS_UTILS_H
#define DDSROS_UTILS_H

#include <ndds/ndds_cpp.h>

namespace px
{

DDSDomainParticipant* createDDSParticipant(int domainId, DDS_TransportBuiltinKindMask transport = DDS_TRANSPORTBUILTIN_UDPv4 | DDS_TRANSPORTBUILTIN_SHMEM);

int ddsShutdown(DDSDomainParticipant* participant);

}

#endif
