#ifndef __LOGGER_h__
#define __LOGGER_h__


class Logger : public Print
{


  public:

    Logger(Print* p1, Print* p2);     
	size_t write(uint8_t byte);
	


   private:
  	Print* p1;
  	Print* p2;
  	
  	
};
#endif

