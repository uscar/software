#ifndef STRUCTURES
#define STRUCTURES


typedef struct{
  float x;
  float y;
  float z;
} 
f3;
typedef struct{
  f3 v;
  float m;
} 
norm3;

typedef struct{
  norm3 up;
  float rot;
}
qState;

typedef struct{
  float m0;
  float m1;
  float m2;
  float m3;
} 
mState;



#endif




