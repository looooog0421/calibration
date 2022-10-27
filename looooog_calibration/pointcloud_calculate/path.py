import sys

paths=sys.path
 
for path_m in paths:
    if "python" in path_m:
        print("python path",path_m)