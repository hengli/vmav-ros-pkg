#ifndef OCTREECACHE_H
#define OCTREECACHE_H

#include <boost/thread/thread.hpp>

#include "dynocmap/OcTree.h"

namespace px
{

class OcTreeCache
{
public:
    OcTreeCache(int cacheSize);
    ~OcTreeCache();

    void cache(const std::string& filename, const OcTreePtr& data);

    OcTreePtr get(const std::string& filename);
    int lookup(const std::string& filename);

private:
    void cacheThread(void);

    enum Status
    {
        UNINITIALIZED,
        READ_REQUESTED,
        READY,
        REPLACE_REQUESTED
    };

    class CacheItem
    {
    public:
        CacheItem(): exists(false), status(UNINITIALIZED) {};

        OcTreePtr data;
        std::string filename;
        OcTreePtr newData;
        std::string newFilename;
        bool exists;
        Status status;
        unsigned int lastReference;
    };

    boost::shared_ptr<boost::thread> m_cacheThread;
    std::vector<CacheItem> m_cache;
    boost::mutex m_cacheMutex;

    unsigned int m_currentReference;

    bool m_requestDestroy;
};

}

#endif
