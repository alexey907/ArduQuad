#ifndef __AVG_h__
#define __AVG_h__


class Avg
{


  public:

    Avg(int size);     
	void add(double next); 
    double get();
	void reset();

   private:
  	double* values;
  	int size;
  	int pos;
	bool full;
	double summ;
};
#endif

