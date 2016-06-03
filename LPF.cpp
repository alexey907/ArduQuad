#include "Arduino.h"
#include "LPF.h"

LPF::LPF(){
}

LPF::LPF(float feedback)
{
	m_feedback = feedback;

}

void LPF::setFeedback(float feedback)
{
  m_feedback = feedback;

}
float LPF::filter(float input){
  if (m_init) {
    m_output = input;
    m_init = false;
  } else {
	  m_output += (input - m_output) * m_feedback;
  }
	return m_output;
}

float LPF::get(){
	return m_output;
}

