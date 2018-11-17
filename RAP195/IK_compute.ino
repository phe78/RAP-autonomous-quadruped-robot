//-----------------------------------------------------------------------------------
//   IK computation
//-----------------------------------------------------------------------------------

void IK(float x, float y, float z) {

  ca=atanf(y/z);
  z1=z/cosf(ca);
  ll=sqrtf(x*x+(z1-cl)*(z1-cl));
  a1=atan2f(x,z1-cl);
  a2=acosf((ll*ll+fl*fl-tl*tl)/(2*fl*ll));
  fa=a1+a2;
  b1=acosf((fl*fl+tl*tl-ll*ll)/(2*fl*tl));
  ta=PI-b1;
  x1=tl*sinf(ta-fa);
  z2=tl*cosf(ta-fa);
  z3=fl*cosf(fa);
  z4=cosf(ca)*cl;
  z5=cosf(ca)*(cl+z3)-z4;
  z6=cosf(ca)*z1-z4-z5;
  y11=sinf(ca)*(z3+cl);
  y2=sinf(ca)*cl;
/*  Serial.println("IK");
  print_float("X=",x,8);  
  print_float("Y=",y,8);  
  print_float("Z=",z,8);  
  print_float("CA=",ca*rad_to_deg,8);
  print_float("FA=",fa*rad_to_deg,8);
  print_float("TA=",ta*rad_to_deg,8);*/
}


