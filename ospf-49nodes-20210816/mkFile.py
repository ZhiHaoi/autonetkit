import os
cwd = os.getcwd()
for i in range(1,50):
    os.mknod("/home/zzh/nsdi22/pybatfish/jupyter_notebooks/networks/ospf-test-49nodes/configs/r"+ str(i) + ".cfg") 
    # fp = open("/home/zzh/nsdi22/pybatfish/jupyter_notebooks/networks/ospf-test-48nodes/configs/r%i.cfg", "w")
    # fp.close()