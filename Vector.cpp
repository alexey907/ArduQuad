#include "Vector.h"

template <class T>
Vector3D<T>::Vector3D() {
  x = 0;
  y = 0;
  z = 0;
}

template <class T>
Vector3D<T>::Vector3D(T x, T y, T z) {
  this->x = x;
  this->y = y;
  this->z = z;
}

template <class T>
Vector3D<T>::Vector3D(Vector3D<T>& v) {
  this->x = v.x;
  this->y = v.y;
  this->z = v.z;
}

template <class T>
void Vector3D<T>::set(T x, T y, T z) {
  this->x = x;
  this->y = y;
  this->z = z;
}

template <class T>
void Vector3D<T>::set(Vector3D<T>& v) {
  this->x = v.x;
  this->y = v.y;
  this->z = v.z;
}

template <class T> T Vector3D<T>::mag() {
  return sqrt(x * x + y * y + z * z);
}

template <class T> void Vector3D<T>::norm() {
  div(mag());
}

template <class T> void Vector3D<T>::mult(T n) {
  this->x *= n;
  this->y *= n;
  this->z *= n;
}

template <class T> void Vector3D<T>::div(T n) { 
  this->x /= n;
  this->y /= n;
  this->z /= n;
}

template <class T> void Vector3D<T>::add(Vector3D<T>& v) {
  this->x += v.x;
  this->y += v.y;
  this->z += v.z;
}

template <class T> void Vector3D<T>::sub(Vector3D<T>& v) {
  this->x -= v.x;
  this->y -= v.y;
  this->z -= v.z;
}

template <class T> float Vector3D<T>::dot(Vector3D<T>& v) {
  return this->x * v.x +
         this->y * v.y +
         this->z * v.z;
}

template <class T> void inline Vector3D<T>::lpf(Vector3D<T>& v, float factor) {
  this->x += (v.x - this->x) * factor;
  this->y += (v.y - this->y) * factor;
  this->z += (v.z - this->z) * factor;
}

template <class T> void Vector3D<T>::print(Print* p) {
  p->print("(");
  p->print(x);
  p->print(", ");
  p->print(y);
  p->print(", ");
  p->print(z);
  p->print(")");

}


template class Vector3D<float>;
template class Vector3D<int>;



