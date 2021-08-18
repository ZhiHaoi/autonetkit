# Read from topology file
edge = "edge"
source = "source"
target = "target"
fp = open("/home/zzh/Documents/networkTopology/Kdl.gml", "r")
text_line = fp.readlines();
pairs = []

r = []
for num in range(0, 754):
    r.append(1)

for num in range(0,len(text_line)):
    currentLine = text_line[num];
    if(edge in currentLine):
        if(source in text_line[num+1]):
            if(target in text_line[num+2]):
                # print(text_line[num+1][11:((len(text_line[num+1])-1))])
                # print(text_line[num+2][11:((len(text_line[num+2])-1))])
                pairs.append((r[int(text_line[num+1][11:((len(text_line[num+1])-1))])],
                              r[int(text_line[num+2][11:((len(text_line[num+2])-1))])]))

fp.close()