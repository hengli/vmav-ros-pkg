#ifndef OCNODE_H_
#define OCNODE_H_

#include <boost/enable_shared_from_this.hpp>
#include <boost/shared_ptr.hpp>
#include <vector>

namespace px
{

class OcNode;
typedef boost::shared_ptr<OcNode> OcNodePtr;
typedef boost::weak_ptr<OcNode> OcNodeWPtr;

class OcNode: public boost::enable_shared_from_this<OcNode>
{
public:
    OcNode();
    OcNode(OcNodePtr parent, int index);

    int index(void) const;

    void setLogOdds(double logOdds);
    double getLogOdds(void) const;

    void setInterimLogOdds(double logOdds);
    double getInterimLogOdds(void) const;

    double getProbability(void) const;

    void split(void);
    bool isLeaf(void) const;

    std::vector<OcNodePtr>& children(void);
    const std::vector<OcNodePtr>& children(void) const;
    OcNodePtr child(int index) const;
    void setChild(int index, const OcNodePtr& childNode);

    OcNodeWPtr parent(void) const;

    void setUpdated(bool updated);
    bool isUpdated(void) const;

private:
    OcNodePtr sharedPtr(void);

    bool m_leaf;
    std::vector<OcNodePtr> m_child;
    OcNodeWPtr m_parent;

    int m_index;
    double m_logOdds;
    double m_interimLogOdds;

    bool m_isUpdated;
};

}

#endif
