#include <nodelet/nodelet.h>
#include <NabVision.h>

namespace octagon
{

    class NabVisionNodelet : public nodelet::Nodelet
    {
        public:
            virtual void onInit();

        private:
  			boost::shared_ptr<NabVision> m_node;
    };

}