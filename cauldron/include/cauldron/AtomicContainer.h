#ifndef ATOMICCONTAINER_H
#define ATOMICCONTAINER_H

#include <boost/thread/condition_variable.hpp>
#include <boost/thread/mutex.hpp>

namespace px
{

template<class T>
class AtomicContainer
{
public:
    AtomicContainer();
    AtomicContainer(const AtomicContainer<T>& frame);

    T& data(void);
    const T& data(void) const;

    ros::Time& timestamp(void);
    const ros::Time& timestamp(void) const;

    bool& available(void);
    bool available(void) const;

    void lockData(void);
    bool tryLockData(void);
    void unlockData(void);

    void notifyData(void);

    boost::condition_variable& dataCondition(void);
    boost::mutex& dataMutex(void);

private:
    T m_image;
    ros::Time m_timestamp;
    bool m_available;

    boost::condition_variable m_dataCond;
    boost::mutex m_dataMutex;
};

template<class T>
AtomicContainer<T>::AtomicContainer()
 : m_available(false)
{

}

template<class T>
AtomicContainer<T>::AtomicContainer(const AtomicContainer<T>& frame)
 : m_timestamp(frame.m_timestamp)
 , m_available(frame.m_available)
{
    frame.m_image.copyTo(m_image);
}

template<class T>
T&
AtomicContainer<T>::data(void)
{
    return m_image;
}

template<class T>
ros::Time&
AtomicContainer<T>::timestamp(void)
{
    return m_timestamp;
}

template<class T>
const ros::Time&
AtomicContainer<T>::timestamp(void) const
{
    return m_timestamp;
}

template<class T>
bool&
AtomicContainer<T>::available(void)
{
    return m_available;
}

template<class T>
bool
AtomicContainer<T>::available(void) const
{
    return m_available;
}

template<class T>
void
AtomicContainer<T>::lockData(void)
{
    m_dataMutex.lock();
}

template<class T>
bool
AtomicContainer<T>::tryLockData(void)
{
    return m_dataMutex.try_lock();
}

template<class T>
void
AtomicContainer<T>::unlockData(void)
{
    m_dataMutex.unlock();
}

template<class T>
void
AtomicContainer<T>::notifyData(void)
{
    boost::lock_guard<boost::mutex> lock(m_dataMutex);

    m_available = true;

    m_dataCond.notify_one();
}

template<class T>
boost::condition_variable&
AtomicContainer<T>::dataCondition(void)
{
    return m_dataCond;
}

template<class T>
boost::mutex&
AtomicContainer<T>::dataMutex(void)
{
    return m_dataMutex;
}

}

#endif
