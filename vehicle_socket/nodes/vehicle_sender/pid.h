#include <cmath>

enum {LLAST = 0,LAST = 1,NOW = 2};

class PID
{
public:
    float m_Kp,m_Ti,m_Td;
    float m_error[3] = {0};

    PID(const float p,const float i = 0,const float d = 0):
        m_Kp(p),m_Ti(i),m_Td(d)
    {}

    float delta(const float error)
    {
        m_error[LLAST] = m_error[LAST];
        m_error[LAST] = m_error[NOW];
        m_error[NOW] = error;

        return m_Kp*(m_error[NOW] - m_error[LAST]);
    }

    float position(float error)
    {
        return m_Kp*error;
    }

    void change(const float Kp)
    {
        m_Kp = Kp;
    }
};