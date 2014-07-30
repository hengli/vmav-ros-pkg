#include "OcTreeCache.h"

#include <boost/filesystem.hpp>
#include <boost/make_shared.hpp>
#include <iostream>

namespace px
{

OcTreeCache::OcTreeCache(int cacheSize)
 : m_currentReference(0)
 , m_requestDestroy(false)
{
    m_cache.reserve(cacheSize);
    for (int i = 0; i < cacheSize; ++i)
    {
        m_cache.push_back(CacheItem());
    }

    m_cache.resize(cacheSize);

    m_cacheThread = boost::make_shared<boost::thread>(&OcTreeCache::cacheThread, this);
}

OcTreeCache::~OcTreeCache()
{
    m_requestDestroy = true;

    m_cacheThread->join();
}

void
OcTreeCache::cache(const std::string& filename, const OcTreePtr& data)
{
    boost::lock_guard<boost::mutex> lock(m_cacheMutex);

    for (size_t i = 0; i < m_cache.size(); ++i)
    {
        CacheItem& item = m_cache.at(i);

        if (item.status != UNINITIALIZED &&
            item.filename == filename)
        {
            if (item.data != data)
            {
                item.data = data;
            }
            item.exists = true;

            return;
        }
    }

    int mark = -1;

    for (size_t i = 0; i < m_cache.size(); ++i)
    {
        CacheItem& item = m_cache.at(i);

        // get uninitialized octree
        if (item.status == UNINITIALIZED)
        {
            mark = i;
            break;
        }
        // get oldest octree
        else if (item.status == READY &&
                 (mark == -1 ||
                  item.lastReference < m_cache.at(mark).lastReference))
        {
            mark = i;
        }
    }

    if (mark == -1)
    {
       return;
    }
    else
    {
        CacheItem& item = m_cache.at(mark);

        if (item.status == READY)
        {
            item.newData = data;
            item.newFilename = filename;
            item.status = REPLACE_REQUESTED;
        }
        else
        {
            item.data = data;
            item.filename = filename;
            item.status = READY;
        }

        item.exists = true;

        item.lastReference = m_currentReference;
        ++m_currentReference;
    }
}

OcTreePtr
OcTreeCache::get(const std::string& filename)
{
    while (true)
    {
        int mark = lookup(filename);

        if (mark == -1)
        {
            usleep(100);
            continue;
        }

        {
            boost::lock_guard<boost::mutex> lock(m_cacheMutex);

            CacheItem& item = m_cache.at(mark);
            if (!item.exists)
            {
                return OcTreePtr();
            }
            else if (item.status == READY)
            {
                return item.data;
            }
        }

        usleep(100);
    }
}

int
OcTreeCache::lookup(const std::string& filename)
{
    boost::lock_guard<boost::mutex> lock(m_cacheMutex);

    int mark = -1;

    for (size_t i = 0; i < m_cache.size(); ++i)
    {
        CacheItem& item = m_cache.at(i);

        if (item.status != UNINITIALIZED &&
            item.filename == filename)
        {
            mark = i;
            break;
        }
    }

    if (mark == -1)
    {
        for (size_t i = 0; i < m_cache.size(); ++i)
        {
            CacheItem& item = m_cache.at(i);

            // get uninitialized octree
            if (item.status == UNINITIALIZED)
            {
                mark = i;
                break;
            }
            // get oldest octree
            else if (item.status == READY &&
                     (mark == -1 ||
                      item.lastReference < m_cache.at(mark).lastReference))
            {
                mark = i;
            }
        }

        if (mark == -1)
        {
           return -1;
        }
        else
        {
            CacheItem& item = m_cache.at(mark);

            item.exists = boost::filesystem::exists(filename.c_str());

            if (item.status == READY)
            {
                item.newFilename = filename;
                item.status = REPLACE_REQUESTED;
            }
            else
            {
                item.filename = filename;

                if (!item.exists)
                {
                    item.status = READY;
                }
                else
                {
                    item.status = READ_REQUESTED;
                }
            }

            item.lastReference = m_currentReference;
            ++m_currentReference;

            return mark;
        }
    }
    else
    {
        CacheItem& item = m_cache.at(mark);

        if (item.status == READY)
        {
            item.lastReference = m_currentReference;
            ++m_currentReference;

            return mark;
        }
        else
        {
            return -1;
        }
    }
}

void
OcTreeCache::cacheThread(void)
{
    while (!m_requestDestroy)
    {
        m_cacheMutex.lock();

        for (size_t i = 0; i < m_cache.size(); ++i)
        {
            CacheItem& item = m_cache.at(i);
            switch (item.status)
            {
            case READ_REQUESTED:
            {
                item.data = boost::make_shared<OcTree>();
                item.data->read(item.filename);
                item.status = READY;

                break;
            }
            case REPLACE_REQUESTED:
            {
                item.data->write(item.filename);

                if (item.newData)
                {
                    item.data = item.newData;
                }
                else
                {
                    if (item.exists)
                    {
                        item.data = boost::make_shared<OcTree>();
                        item.data->read(item.newFilename);
                    }
                    else
                    {
                        item.data.reset();
                    }
                }

                item.filename = item.newFilename;

                item.newData.reset();
                item.newFilename.clear();

                item.status = READY;

                break;
            }
            }
        }

        m_cacheMutex.unlock();

        usleep(100);
    }
}

}
