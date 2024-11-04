import os

file_names = os.listdir("/home/zezhousun/Downloads/agribot_data/rgb")
file_names.sort(key=lambda x: int(x[:-4]))

f= open("/home/zezhousun/Downloads/agribot_data/rgb.txt", 'w')
for name in file_names:
    f.write(name[0:10]+'.'+name[10:-4]+" rgb/"+name+"\n")
f.close()