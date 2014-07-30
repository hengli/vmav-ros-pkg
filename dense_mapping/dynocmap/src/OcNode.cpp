#include "dynocmap/OcNode.h"

#include <boost/make_shared.hpp>
#include <cmath>

namespace px
{

OcNode::OcNode()
 : m_leaf(true)
 , m_child(8)
 , m_index(-1)
 , m_logOdds(0.0)
 , m_interimLogOdds(0.0)
 , m_isUpdated(false)
{

}

OcNode::OcNode(OcNodePtr parent, int index)
 : m_leaf(true)
 , m_child(8)
 , m_parent(parent)
 , m_index(index)
 , m_logOdds(0.0)
 , m_interimLogOdds(0.0)
 , m_isUpdated(false)
{

}

int
OcNode::index(void) const
{
    return m_index;
}

void
OcNode::setLogOdds(double logOdds)
{
    m_logOdds = logOdds;
}

double
OcNode::getLogOdds(void) const
{
    return m_logOdds;
}

void
OcNode::setInterimLogOdds(double logOdds)
{
    m_interimLogOdds = logOdds;
}

double
OcNode::getInterimLogOdds(void) const
{
    return m_interimLogOdds;
}

double
OcNode::getProbability(void) const
{
    double e = exp(m_logOdds);

    return e / (1 + e);
}

void
OcNode::split(void)
{
    if (m_leaf)
    {
        for (int i = 0; i < 8; ++i)
        {
            m_child.at(i) = boost::make_shared<OcNode>(sharedPtr(), i);
        }
        m_leaf = false;
    }
}

bool
OcNode::isLeaf(void) const
{
    return m_leaf;
}

std::vector<OcNodePtr>&
OcNode::children(void)
{
    return m_child;
}

const std::vector<OcNodePtr>&
OcNode::children(void) const
{
    return m_child;
}

OcNodePtr
OcNode::child(int index) const
{
    return m_child[index];
}

void
OcNode::setChild(int index, const OcNodePtr& childNode)
{
    m_child[index] = childNode;

    childNode->m_parent = shared_from_this();
    childNode->m_index = index;
}

OcNodeWPtr
OcNode::parent(void) const
{
    return m_parent;
}

void
OcNode::setUpdated(bool updated)
{
    m_isUpdated = updated;
}

bool
OcNode::isUpdated(void) const
{
    return m_isUpdated;
}

OcNodePtr
OcNode::sharedPtr(void)
{
    return shared_from_this();
}

}
