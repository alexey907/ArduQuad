#ifndef __LPF_h__
#define __LPF_h__


class LPF
{


  public:
    LPF();
    
    LPF(float feedback);
    float filter(float val);
    float get();
    void setFeedback(float feedback);
  private:
    float m_output = 0;
    float m_feedback = 0.2;
    bool m_init = true;
};
#endif

