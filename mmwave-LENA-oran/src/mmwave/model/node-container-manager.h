#include "ns3/node-container.h"
namespace ns3 {

class NodeContainerManager {
public:
    static NodeContainerManager &GetInstance() {
        static NodeContainerManager instance;
        return instance;
    }

    void SetMmWaveEnbNodes(NodeContainer nodes) {
        m_mmWaveEnbNodes = nodes;
    }

    NodeContainer &GetMmWaveEnbNodes() {
        return m_mmWaveEnbNodes;
    }

private:
    NodeContainerManager() {}
    NodeContainer m_mmWaveEnbNodes;
};
}
