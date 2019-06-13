import os

file = [2,3,4]
num=[50,100,200,500]
degree=[5,10,20,50]

for i in file:
	for j in num:
		for k in degree:
			#left_arm
			os.system("/home/rajat/melodic_ws/devel/bin/createGraph -d 7 -i /home/rajat/Desktop/MINT/graphs/"+str(i)+"/herb_halton_"+str(j)+".graphml -o /home/rajat/Desktop/MINT/graphs/"+str(i)+"/left_arm/herb_halton_l_"+str(j)+"_"+str(k)+".graphml -k "+str(k)+" -a left")
			#right_arm
			os.system("/home/rajat/melodic_ws/devel/bin/createGraph -d 7 -i /home/rajat/Desktop/MINT/graphs/"+str(i)+"/herb_halton_"+str(j)+".graphml -o /home/rajat/Desktop/MINT/graphs/"+str(i)+"/right_arm/herb_halton_r_"+str(j)+"_"+str(k)+".graphml -k "+str(k)+" -a right")
			print(str(i)+str(j)+str(k))