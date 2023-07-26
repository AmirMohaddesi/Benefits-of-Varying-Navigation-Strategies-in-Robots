import os
import pandas as pd
header = ['Robot Number','number of goals assigned','Time taken','Number of wall obstacles','Number of Robot obstacles','']
def findfiles (path, filter):
    for root, dirs, files in os.walk(path):
        for file in files:
            yield os.path.join(root, file)

for textfile in findfiles(r'C:\Users\Amix\Desktop\Project\PR2Maze\controllers\SpikeWave\Results', '*.txt'):
##    print(textfile);
    filename = os.path.basename(textfile)
    if filename.find('Assigned'):
        count = 0;
        f = pd.read_csv(textfile, sep=" ")
        for i in f:
            count+=1
        print(count);
