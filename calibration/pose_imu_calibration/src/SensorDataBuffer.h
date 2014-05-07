#ifndef SENSORDATABUFFER_H
#define SENSORDATABUFFER_H

#include <boost/thread.hpp>
#include <vector>

namespace px
{

template <class T>
class SensorDataBuffer
{
public:
    explicit SensorDataBuffer(size_t size = 100);

    void clear(void);
    bool empty(void);
    size_t size(void);

    void front(ros::Time& timestamp, T& data);
    void at(size_t n, ros::Time& timestamp, T& data);

    bool before(const ros::Time& timestamp, T& data);
    bool after(const ros::Time& timestamp, T& data);

    bool nearest(const ros::Time& timestamp, T& data);
    bool nearest(const ros::Time& timestamp, T& dataBefore, T& dataAfter);

    bool current(T& data);
    void push(const ros::Time& timestamp, const T& data);

    bool find(const ros::Time& timestamp, T& data);

private:
    std::vector< std::pair<ros::Time, T> > m_buffer;
    int m_index;

    boost::mutex m_globalMutex;
};

template <class T>
SensorDataBuffer<T>::SensorDataBuffer(size_t size)
 : m_index(-1)
{
    m_buffer.reserve(size);
}

template <class T>
void
SensorDataBuffer<T>::clear(void)
{
    boost::lock_guard<boost::mutex> lock(m_globalMutex);

    m_index = -1;
    m_buffer.clear();
}

template <class T>
bool
SensorDataBuffer<T>::empty(void)
{
    boost::lock_guard<boost::mutex> lock(m_globalMutex);

    return m_buffer.empty();
}

template <class T>
size_t
SensorDataBuffer<T>::size(void)
{
    boost::lock_guard<boost::mutex> lock(m_globalMutex);

    return m_buffer.size();
}

template <class T>
void
SensorDataBuffer<T>::front(ros::Time& timestamp, T& data)
{
    int index = 1 + m_index - static_cast<int>(m_buffer.size());
    if (index < 0)
    {
        index += m_buffer.size();
    }

    timestamp = m_buffer.at(index).first;
    data = m_buffer.at(index).second;
}

template <class T>
void
SensorDataBuffer<T>::at(size_t n, ros::Time& timestamp, T& data)
{
    int index = n + 1 + m_index;
    if (index >= m_buffer.size())
    {
        index -= m_buffer.size();
    }

    timestamp = m_buffer.at(index).first;
    data = m_buffer.at(index).second;
}

template <class T>
bool
SensorDataBuffer<T>::before(const ros::Time& timestamp, T& data)
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
        double tsDiff = (m_buffer.at(mark).first - timestamp).toSec();
        if (tsDiff < 0.0)
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
SensorDataBuffer<T>::after(const ros::Time& timestamp, T& data)
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
        double tsDiff = (m_buffer.at(mark).first - timestamp).toSec();
        if (tsDiff < 0.0)
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
bool
SensorDataBuffer<T>::nearest(const ros::Time& timestamp, T& data)
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

    double tsDiffMin = std::numeric_limits<double>::max();
    int mark = m_index;
    do
    {
        double tsDiff = fabs((m_buffer.at(mark).first - timestamp).toSec());
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
SensorDataBuffer<T>::nearest(const ros::Time& timestamp, T& dataBefore, T& dataAfter)
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
        double tsDiff = (m_buffer.at(mark).first - timestamp).toSec();
        if (tsDiff < 0.0)
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
SensorDataBuffer<T>::current(T& data)
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
SensorDataBuffer<T>::push(const ros::Time& timestamp, const T& data)
{
    boost::lock_guard<boost::mutex> lock(m_globalMutex);

    if (!m_buffer.empty() && timestamp == m_buffer.back().first)
    {
        return;
    }

    if (m_buffer.size() == m_buffer.capacity())
    {
        m_index = (m_index + 1) % m_buffer.capacity();
        m_buffer.at(m_index).first = timestamp;
        m_buffer.at(m_index).second = data;
    }
    else
    {
        m_buffer.push_back(std::make_pair(timestamp, data));
        ++m_index;
    }
}

template <class T>
bool
SensorDataBuffer<T>::find(const ros::Time& timestamp, T& data)
{
    boost::lock_guard<boost::mutex> lock(m_globalMutex);

    for (typename std::vector< std::pair<ros::Time, T> >::iterator it = m_buffer.begin(); it != m_buffer.end(); ++it)
    {
        if (it->first == timestamp)
        {
            data = it->second;

            return true;
        }
        else if (it->first > timestamp)
        {
            return false;
        }
    }

    return false;
}

}

#endif
