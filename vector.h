#ifndef __VECTOR__H__
#define __VECTOR__H__
#include <math.h>
#include <Arduino.h>

template <class T>class Vector3D
{
  public:
    Vector3D<T>();
    Vector3D<T>(T x, T y, T z);
    Vector3D<T>(Vector3D<T>& v);

    //  Vector3D& operation= (Vector3D& v);

    void set(T x, T y, T z);


    T mag();
    void norm();
    void mult (T n);
    void div (T n);
    void rotateX (float angle);
    void rotateY (float angle);
    
    void add (Vector3D<T>& v);
    void set (Vector3D<T>& v);
    void sub (Vector3D<T>& v);
    float dot (Vector3D<T>& v);
    void inline lpf (Vector3D<T>& v, float factor);
    
    void print(Print* p);

    float x;
    float y;
    float z;


};
class TrigFn {
  public:
    inline float get(float angle) {
      int n = (int) (angle * 180.0 / PI) % 360;
      if (n < 0) n +=360;
      return _val[n];
    };

  protected:
    float _val[360];
};

class TrigCos: public TrigFn {
  public:
    TrigCos() {
      for (float n = 0; n < 360; n++) {
        _val[(int) n] = cos((float) n * PI / 180.0);
      }
    }
};

class TrigSin: public TrigFn {
public:
    TrigSin() {
      for (int n = 0; n < 360; n++) {
        _val[n] = sin((float) n * PI / 180.0);
      }
    }
};
#define _PI_  3.1415926536f
#define _PI12_  1.5707963268f

class TrigAtan{
public:
    float vals[201];
    TrigAtan() {
      for (int n = -100; n <= 100; n++) {
        vals[n + 100] = (float) atan(n / 100.0);
      }

    }

    inline float get(float y, float x) {
      if (abs(x) > abs(y)) {
        int tg = (int) (100.0 * y / x) + 100;
        return (x > 0) ? vals[tg]
               : //0 to pi/4 and 0 to -pi/4
               (y > 0) ? (PI + vals[tg])
               : //3/4pi to pi
               (-_PI_ + vals[tg]); //-pi to -3/4pi

      } else {
        int tg = (int) (100.0 * x / y) + 100;

        return (y > 0) ? (_PI12_ - vals[tg]) //1/4pi to 3/4pi
               : (-_PI12_ - vals[tg]); //-3/4pi to -pi/4
      }

    }
};
extern TrigAtan atan2_;
extern TrigCos cos_;
extern TrigSin sin_;



typedef Vector3D<float> FVector3D;
typedef Vector3D<int16_t> IVector3D;



#endif
