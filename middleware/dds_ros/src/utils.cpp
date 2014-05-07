#include "dds_ros/utils.h"

#include <ros/ros.h>

namespace px
{

DDSDomainParticipant*
createDDSParticipant(int domainId, DDS_TransportBuiltinKindMask transport)
{
    DDS_DomainParticipantQos participant_qos;
    DDS_ReturnCode_t retcode = DDSTheParticipantFactory->get_default_participant_qos(participant_qos);

    if (retcode != DDS_RETCODE_OK)
    {
        ROS_ERROR("get_default_participant_qos error %d", retcode);
        return 0;
    }

    participant_qos.discovery.initial_peers.maximum(0);

    switch (transport)
    {
    case DDS_TRANSPORTBUILTIN_UDPv4:
    {
        participant_qos.discovery.initial_peers.maximum(2);
        participant_qos.discovery.initial_peers.length(2);

        participant_qos.discovery.initial_peers[0] = DDS_String_dup("239.255.0.1");
        participant_qos.discovery.initial_peers[1] = DDS_String_dup("4@builtin.udpv4://127.0.0.1");

        participant_qos.discovery.multicast_receive_addresses.maximum(0);
        participant_qos.discovery.multicast_receive_addresses.maximum(1);
        participant_qos.discovery.multicast_receive_addresses.length(1);
        participant_qos.discovery.multicast_receive_addresses[0] = DDS_String_dup("239.255.0.1");

        break;
    }
    case DDS_TRANSPORTBUILTIN_UDPv4 | DDS_TRANSPORTBUILTIN_SHMEM:
    {
        participant_qos.discovery.initial_peers.maximum(3);
        participant_qos.discovery.initial_peers.length(3);

        participant_qos.discovery.initial_peers[0] = DDS_String_dup("239.255.0.1");
        participant_qos.discovery.initial_peers[1] = DDS_String_dup("4@builtin.udpv4://127.0.0.1");
        participant_qos.discovery.initial_peers[2] = DDS_String_dup("builtin.shmem://");

        participant_qos.discovery.multicast_receive_addresses.maximum(0);
        participant_qos.discovery.multicast_receive_addresses.maximum(1);
        participant_qos.discovery.multicast_receive_addresses.length(1);
        participant_qos.discovery.multicast_receive_addresses[0] = DDS_String_dup("239.255.0.1");

        break;
    }
    }

    participant_qos.transport_builtin.mask = transport;

    participant_qos.receiver_pool.buffer_size = 1048576; // 1 MB

    retcode = DDSPropertyQosPolicyHelper::assert_property(
        participant_qos.property,
        "dds.transport.UDPv4.builtin.parent.allow_interfaces",
        "192.168.*", DDS_BOOLEAN_FALSE);

    if (retcode != DDS_RETCODE_OK)
    {
        ROS_ERROR("assert_property %d", retcode);
        return 0;
    }

    retcode = DDSPropertyQosPolicyHelper::assert_property(
        participant_qos.property,
        "dds.transport.UDPv4.builtin.parent.message_size_max",
        "65535", DDS_BOOLEAN_FALSE);

    if (retcode != DDS_RETCODE_OK)
    {
        ROS_ERROR("assert_property %d", retcode);
        return 0;
    }

    retcode = DDSPropertyQosPolicyHelper::assert_property(
        participant_qos.property,
        "dds.transport.UDPv4.builtin.recv_socket_buffer_size",
        "8388608", DDS_BOOLEAN_FALSE);

    if (retcode != DDS_RETCODE_OK)
    {
        ROS_ERROR("assert_property %d", retcode);
        return 0;
    }

    retcode = DDSPropertyQosPolicyHelper::assert_property(
        participant_qos.property,
        "dds.transport.UDPv4.builtin.send_socket_buffer_size",
        "8388608", DDS_BOOLEAN_FALSE);

    if (retcode != DDS_RETCODE_OK)
    {
        ROS_ERROR("assert_property %d", retcode);
        return 0;
    }

    retcode = DDSPropertyQosPolicyHelper::assert_property(
        participant_qos.property,
        "dds.transport.shmem.builtin.parent.message_size_max",
        "1048576", DDS_BOOLEAN_FALSE);

    if (retcode != DDS_RETCODE_OK)
    {
        ROS_ERROR("assert_property %d", retcode);
        return 0;
    }

    retcode = DDSPropertyQosPolicyHelper::assert_property(
        participant_qos.property,
        "dds.transport.shmem.builtin.receive_buffer_size",
        "8388608", DDS_BOOLEAN_FALSE);

    if (retcode != DDS_RETCODE_OK)
    {
        ROS_ERROR("assert_property %d", retcode);
        return 0;
    }

    retcode = DDSPropertyQosPolicyHelper::assert_property(
        participant_qos.property,
        "dds.transport.shmem.builtin.received_message_count_max",
        "8", DDS_BOOLEAN_FALSE);

    if (retcode != DDS_RETCODE_OK)
    {
        ROS_ERROR("assert_property %d", retcode);
        return 0;
    }

    DDSDomainParticipant* ddsParticipant =
        DDSTheParticipantFactory->create_participant(domainId, participant_qos, 0, DDS_STATUS_MASK_NONE);
    if (ddsParticipant == 0)
    {
        ROS_ERROR("create_participant error");
        return 0;
    }

    return ddsParticipant;
}

int
ddsShutdown(DDSDomainParticipant* participant)
{
    int status = 0;

    if (participant != 0)
    {
        DDS_ReturnCode_t retcode = participant->delete_contained_entities();
        if (retcode != DDS_RETCODE_OK)
        {
            ROS_ERROR("delete_contained_entities error %d", retcode);
            status = -1;
        }

        retcode = DDSTheParticipantFactory->delete_participant(participant);
        if (retcode != DDS_RETCODE_OK)
        {
            ROS_ERROR("delete_participant error %d", retcode);
            status = -1;
        }
    }

    return status;
}

}
