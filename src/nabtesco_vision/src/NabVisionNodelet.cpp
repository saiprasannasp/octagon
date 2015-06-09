// this should really be in the implementation (.cpp file)
#include <pluginlib/class_list_macros.h>
#include <NabVisionNodelet.h>

// watch the capitalization carefully
PLUGINLIB_EXPORT_CLASS(octagon::NabVisionNodelet, nodelet::Nodelet) 

namespace octagon
{
    void NabVisionNodelet::onInit() 
    {
        NODELET_DEBUG("Initializing Nab vision nodelet...");
        m_node.reset(new NabVision(getNodeHandle(), getPrivateNodeHandle()));
    }
}