#include "Arduino.h"
#include "Avg.h"

Avg::Avg(int _size)
{
	values = NULL;
	size = _size;
	reset();
}

void Avg::reset(){
	if (values != NULL) {
		free(values);
	}
	values = (double*) malloc(size * sizeof(double));
	memset(values, 0, size * sizeof(double));
	pos = 0;
	full = false;
	summ = 0;
}

void Avg::add(double next){
	summ -= values[pos];
	values[pos] = next;
	summ += values[pos];
	pos++;
	
	if (pos == size){
		pos = 0;
		full = true;
	}
	
}
/*
void Avg::filterHarm(){
	int n = 0;
	double max = 0,min = 0;
	double last0 = 0;
	double lastVal = 0
	while (n < size){
		int idx = (pos + n) % size;
		double val = values[idx]
		if (val * lastVal > 0) { //changed sign
			double zero = (double) n - val / (val + lastVal);
			lastZero = zero;
		}
	}
	
}*/

double Avg::get(){
	
	return summ / ((full) ? size : pos);
	
}

