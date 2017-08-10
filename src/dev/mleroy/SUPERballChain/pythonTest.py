# -*- coding: utf-8 -*-
from pyswarm import pso
import subprocess
import numpy as np

appFile = "../../../../build/dev/mleroy/SUPERballChain/AppSUPERballChain"
yamlFile = "../../../../resources/YamlStructures/mleroy/rot2SUPERballV2.yaml"

def banana(x):
	x1=str(x[0])
	x2=str(x[1])
	x3=str(x[2])
	x4=str(x[3])
	x5=str(x[4])
	x6=str(x[5])
	x7=str(x[6])
	x8=str(x[7])
	x9=str(x[8])
	x10=str(x[9])
	x11=str(x[10])
	x12=str(x[11])
	x13=str(x[12])
	x14=str(x[13])
	x15=str(x[14])
	x16=str(x[15])
	x17=str(x[16])
	x18=str(x[17])
	x19=str(x[18])
	x20=str(x[19])
	x21=str(x[20])
	x22=str(x[21])
	x23=str(x[22])
	x24=str(x[23])
	x25=str(x[24])
	x26=str(x[25])
	x27=str(x[26])
	x28=str(x[27])
	x29=str(x[28])
	x30=str(x[29])
	x31=str(x[30])
	x32=str(x[31])
	
	#try:
	out = subprocess.check_output([appFile,yamlFile,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16,x17,x18,x19,x20,x21,x22,x23,x24,x25,x26,x27,x28,x29,x30,x31,x32])
	#out = subprocess.check_output([appFile,yamlFile])
	
	out1, out2 = out.split("DONE, traveled ")
	out2 = out2[0:6]
	
	#except:
	#	out2 = "-1000.0"
	
	out2 = float(out2)
	print(out2)
	return -out2	

def con(x):
	x1=x[0] #omega
	x2=x[1] #mu
	x3=x[2] #up
	x4=x[3] #down
	x5=x[4] #ne
	x6=x[5] #se
	x7=x[6] #even
	x8=x[7] #odd

	x9=x[8]
	x10=x[9]
	x11=x[10]
	x12=x[11]
	x13=x[12]
	x14=x[13]
	x15=x[14]
	x16=x[15]
	
	x17=x[16]
	x18=x[17]
	x19=x[18]
	x20=x[19]
	x21=x[20]
	x22=x[21]
	x23=x[22]
	x24=x[23]
	
	x25=x[24]
	x26=x[25]
	x27=x[26]
	x28=x[27]
	x29=x[28]
	x30=x[29]
	x31=x[30]
	x32=x[31]
	
	a31 =  x4
  	a41 =  x6
   	a13 = x11
   	a23 = x13
   	a53 = x12
   	a63 = x14
   	a35 = x19
   	a45 = x21
   	a75 = x20
   	a85 = x22
   	a57 = x27
   	a67 = x29
   	w13 =  x1 -  x9;
   	w35 =  x9 - x17;
   	w57 = x17 - x25;

   	sigma  = 0.0
	sigma  =   a13*a35*a57 -   a13*a35*a67 -   a13*a45*a57 -   a23*a35*a57 + 2*a13*a35*a75 +   a31*a35*a57 + a13*a45*a67 +   a23*a35*a67
  	sigma +=   a23*a45*a57 - 2*a13*a35*a85 - 2*a13*a45*a75 - 2*a23*a35*a75 -   a31*a35*a67 -   a31*a45*a57 - a35*a41*a57 -   a23*a45*a67
  	sigma += 2*a31*a35*a75 -   a31*a53*a57 + 2*a13*a45*a85 + 2*a23*a35*a85 + 2*a23*a45*a75 +   a31*a45*a67 + a35*a41*a67
  	sigma +=   a41*a45*a57 - 2*a31*a35*a85 - 2*a31*a45*a75 +   a31*a53*a67 +   a31*a57*a63 - 2*a35*a41*a75 + a41*a53*a57 - 2*a23*a45*a85 - a41*a45*a67 - a31*a53*a75
  	sigma += 2*a31*a45*a85 -   a31*a63*a67 + 2*a35*a41*a85 + 2*a41*a45*a75 -   a41*a53*a67 -   a41*a57*a63 + a31*a53*a85 +   a31*a63*a75
  	sigma +=   a41*a53*a75 - 2*a41*a45*a85 +   a41*a63*a67 -   a31*a63*a85 -   a41*a53*a85 -   a41*a63*a75 + a41*a63*a85
  	
  	s1  = 0.0
  	s1  = w13*(a35*a57-a35*a67-a45*a57+2*a35*a75-a53*a57+a45*a67-2*a35*a85-2*a45*a75+a53*a67+a57*a63-a53*a75+2*a45*a85-a63*a67+a53*a85+a63*a75-a63*a85)
  	s1 -= w57*(a53-a63)*(a75-a85)
  	s1 -= w35*(a53-a63)*(a57-a67+a75-a85)
  	s1 /= sigma;

  	s2 =  0.0
  	s2  = -w57 * (a75-a85) * (a13-a23+a31-a41)
  	s2 -= w13 * (a13-a23) * (a57-a67+a75-a85)
  	s2 -= w35 * (a13-a23+a31-a41) * (a57-a67+a75-a85)
  	s2 /= sigma;

  	s3  = 0.0
  	s3  = w57*(a13*a35-a13*a45-a23*a35+a31*a35+a23*a45-a31*a45-a35*a41-a31*a53+a41*a45+a31*a63+a41*a53-a41*a63)
  	s3 -= w13 * (a13-a23) * (a35-a45)
  	s3 -= w35 * (a35-a45) * (a13-a23+a31-a41)
  	s3 /= sigma
  	# The condition would be float(abs(s1)<1 and abs(s2)<1 and abs(s3)<1) so as con(x)=0
  	return [1-abs(s1),1-abs(s2),1-abs(s3)]

lb1 = 0.0#999999999 #0.1
ub1 = 1.0 #5.0            omega

lb2 = -0.5#999999999 #0.1
ub2 =  0.5 #1.0            mu/amp

lb3 = -1.0#999999999 #-1.0
ub3 =  1.0 #1.0            up

lb4 = -1.0#999999999 #-1.0
ub4 =  1.0 #1.0            down

lb5 = -1.0#999999999 #-1.0
ub5 =  1.0 #1.0            ne

lb6 = -1.0#999999999 #-1.0
ub6 =  1.0 #1.0            se

lb7 = 0.0#999999999 #-0.1
ub7 = 1.0 #0.1            even

lb8 = 0.0#999999999 #-0.1
ub8 = 1.0 #0.1            odd

lb=[lb1,lb2,lb3,lb4,lb5,lb6,lb7,lb8,lb1,lb2,lb3,lb4,lb5,lb6,lb7,lb8,lb1,lb2,lb3,lb4,lb5,lb6,lb7,lb8,lb1,lb2,lb3,lb4,lb5,lb6,lb7,lb8]
ub=[ub1,ub2,ub3,ub4,ub5,ub6,ub7,ub8,ub1,ub2,ub3,ub4,ub5,ub6,ub7,ub8,ub1,ub2,ub3,ub4,ub5,ub6,ub7,ub8,ub1,ub2,ub3,ub4,ub5,ub6,ub7,ub8]

xopt2, fopt2 = pso(banana, lb, ub, f_ieqcons=con, args=(), kwargs={}, swarmsize=40, omega=1.0, phip=1.0, phig=1.0, maxiter=40, minstep=1e-8, minfunc=1e-8, debug=True)
print xopt2
print fopt2