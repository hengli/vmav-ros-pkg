#ifndef DATABUFFER_H
#define DATABUFFER_H

#include <boost/thread/mutex.hpp>
#include <vector>

namespace px
{

template <class T>
class DataBuffer
{
public:
    explicit DataBuffer(size_t size = 100);

    void clear(void);
    bool empty(void);
    size_t size(void);

    bool before(const ros::Time& stamp, T& data);
    bool after(const ros::Time& stamp, T& data);

    void nearest(const ros::Time& stamp, T& data);
    bool nearest(const ros::Time& stamp, T& dataBefore, T& dataAfter);

    bool current(T& data);
    void push(const ros::Time& stamp, const T& data);

    bool find(const ros::Time& stamp, T& data);

private:
    std::vector<std::pair<ros::Time, T> > m_buffer;
    int m_index;

    boost::mutex m_globalMutex;
};

template <class T>
DataBuffer<T>::DataBuffer(size_t size)
 : m_index(-1)
{
    m_buffer.reserve(size);
}

template <class T>
void
DataBuffer<T>::clear(void)
{
    boost::lock_guard<boost::mutex> lock(m_globalMutex);

    m_index = -1;
    m_buffer.clear();
}

template <class T>
bool
DataBuffer<T>::empty(void)
{
    boost::lock_guard<boost::mutex> lock(m_globalMutex);

    return m_buffer.empty();
}

template <class T>
size_t
DataBuffer<T>::size(void)
{
    boost::lock_guard<boost::mutex> lock(m_globalMutex);

    return m_buffer.size();
}

template <class T>
bool
DataBuffer<T>::before(const ros::Time& stamp, T& data)
{
    boost::lock_guard<boost::mutex> lock(m_globalMutex);

    if (m_buffer.empty())
    {
        return false;
    }

    int endMark = 0;
    if (m_buffer.size() == m_buffer.capacity())
    {
        endMark = m_index;
    }
    else
    {
        endMark = m_buffer.capacity() - 1;
    }

    int mark = m_index;
    do
    {
        if ((m_buffer.at(mark).first - stamp).toSec() < 0.0)
        {
            if (mark == m_index)
            {
                // no data after timestamp
                return false;
            }
            else
            {
                data = m_buffer.at(mark).second;

                return true;
            }
        }

        --mark;
        if (mark < 0)
        {
            mark += m_buffer.capacity();
        }
    }
    while (mark != endMark);

    return false;
}

template <class T>
bool
DataBuffer<T>::after(const ros::Time& stamp, T& data)
{
    boost::lock_guard<boost::mutex> lock(m_globalMutex);

    if (m_buffer.empty())
    {
        return false;
    }

    int endMark = 0;
    if (m_buffer.size() == m_buffer.capacity())
    {
        endMark = m_index;
    }
    else
    {
        endMark = m_buffer.capacity() - 1;
    }

    int mark = m_index;
    do
    {
        if ((m_buffer.at(mark).first - stamp).toSec() < 0.0)
        {
            if (mark == m_index)
            {
                // no data after timestamp
                return false;
            }
            else
            {
                data = m_buffer.at((mark + 1) % m_buffer.capacity()).second;

                return true;
            }
        }

        --mark;
        if (mark < 0)
        {
            mark += m_buffer.capacity();
        }
    }
    while (mark != endMark);

    return false;
}

template <class T>
void
DataBuffer<T>::nearest(const ros::Time& stamp, T& data)
{
    boost::lock_guard<boost::mutex> lock(m_globalMutex);

    if (m_buffer.empty())
    {
        return false;
    }

    int endMark = 0;
    if (m_buffer.size() == m_buffer.capacity())
    {
        endMark = m_index;
    }
    else
    {
        endMark = m_buffer.capacity() - 1;
    }

    long int tsDiffMin = std::numeric_limits<int64_t>::max();
    int mark = m_index;
    do
    {
        int64_t tsDiff = std::abs((m_buffer.at(mark).first - stamp).toNSec());
        if (tsDiff < tsDiffMin)
        {
            tsDiffMin = tsDiff;
        }
        else
        {
            data = m_buffer.at(mark).second;
            return true;
        }

        --mark;
        if (mark < 0)
        {
            mark += m_buffer.capacity();
        }
    }
    while (mark != endMark);

    return false;
}

template <class T>
bool
DataBuffer<T>::nearest(const ros::Time& stamp, T& dataBefore, T& dataAfter)
{
    boost::lock_guard<boost::mutex> lock(m_globalMutex);

    if (m_buffer.empty())
    {
        return false;
    }

    int endMark = 0;
    if (m_buffer.size() == m_buffer.capacity())
    {
        endMark = m_index;
    }
    else
    {
        endMark = m_buffer.capacity() - 1;
    }

    int mark = m_index;
    do
    {
        if ((m_buffer.at(mark).first - stamp).toSec() < 0.0)
        {
            if (mark == m_index)
            {
                // no data after timestamp
                return false;
            }
            else
            {
                dataBefore = m_buffer.at(mark).second;
                dataAfter = m_buffer.at((mark + 1) % m_buffer.capacity()).second;

                return true;
            }
        }

        --mark;
        if (mark < 0)
        {
            mark += m_buffer.capacity();
        }
    }
    while (mark != endMark);

    return false;
}

template <class T>
bool
DataBuffer<T>::current(T& data)
{
    boost::lock_guard<boost::mutex> lock(m_globalMutex);

    if (m_buffer.empty())
    {
        return false;
    }
    else
    {
        data = m_buffer.at(m_index).second;
        return true;
    }
}

template <class T>
void
DataBuffer<T>::push(const ros::Time& stamp, const T& data)
{
    boost::lock_guard<boost::mutex> lock(m_globalMutex);

    if (!m_buffer.empty() && stamp == m_buffer.at(m_index).first)
    {
        return;
    }

    if (m_buffer.size() == m_buffer.capacity())
    {
        m_index = (m_index + 1) % m_buffer.capacity();
        m_buffer.at(m_index).first = stamp;
        m_buffer.at(m_index).second = data;
    }
    else
    {
        m_buffer.push_back(std::make_pair(stamp, data));
        ++m_index;
    }
}

template <class T>
bool
DataBuffer<T>::find(const ros::Time& stamp, T& data)
{
    boost::lock_guard<boost::mutex> lock(m_globalMutex);

    if (m_buffer.empty())
    {
        return false;
    }

    int endMark = 0;
    if (m_buffer.size() == m_buffer.capacity())
    {
        endMark = m_index;
    }
    else
    {
        endMark = m_buffer.capacity() - 1;
    }

    int mark = m_index;
    do
    {
        if (m_buffer.at(mark).first == stamp)
        {
            data = m_buffer.at(mark).second;

            return true;
        }
        else if (m_buffer.at(mark).first < stamp)
        {
            return false;
        }

        --mark;
        if (mark < 0)
        {
            mark += m_buffer.capacity();
        }
    }
    while (mark != endMark);

    return false;
}

}

#endif
