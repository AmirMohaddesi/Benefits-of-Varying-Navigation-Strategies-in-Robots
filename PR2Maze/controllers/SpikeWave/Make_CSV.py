import pandas as pd
import os

df = pd.read_csv("edited map2.txt", sep='\t', header = None)
map = (df < 241).sum().sum()

def read_trail(file_name):
    df = pd.read_csv(file_name, header=None)
    time = float(df.iloc[0, 0].split("goals: ")[1])
    wall = int(df.iloc[1, 0].split("Obstacles ")[1])
    robot = int(df.iloc[2, 0].split("Obstacles ")[1])
    
    info = [time, wall, robot]
    
    return info
  
def read_efficiency(file_names):
    df = None
    for idx, file_name in enumerate(file_names):
        if idx == 0:
            df = pd.read_csv(file_name, header=None, sep =" ")
            df.drop([64], axis=1, inplace=True)
            df[df != 0] = 1
        else:
            tmp = pd.read_csv(file_name, header=None, sep =" ")
            tmp.drop([64], axis=1, inplace=True)
            tmp[tmp != 0] = 1
            df = df + tmp
            
    overall = (df >= 1).sum().sum()
    overlap = (df >= 2).sum().sum()
    return overall, overlap
   
def read_occupancy(file_name):
    df = pd.read_csv(file_name, header=None, sep =" ")
    df.drop([64], axis=1, inplace=True)
    occupancy = (df > 0).sum().sum()
    occupancy = occupancy / map
    return occupancy
    
files = os.listdir("Results")
trial = []
for file in files:
    trial1 = [f"{file}_1Robot_trial{i+1}_robot1" for i in range(5)]
    trial2 = [f"{file}_3Robot_trial{i+1}_robot{j+1}" for i in range(5) for j in range(3)]
    trial3 = [f"{file}_5Robot_trial{i+1}_robot{j+1}" for i in range(5) for j in range(5)]
    trial = trial + trial1 + trial2 + trial3
index = ["Trial Number", 
         "Robot Number", 
         "Time taken (s)", 
         "Wall Obstacles (#)", 
         "Number of Collisions (#)", 
         "Area occupancy (%)", 
         "Efficiency (Overlap)(%)", 
         "Efficiency (Overall area covered by all robots) (%)"]

print(trial)


df = pd.DataFrame(0, 
                  columns=trial,
                  index=index)

for file in files:
    for i in [1,3,5]:
        for trial in range(1,6):
            robots = []
            path_robots = []
            for robot in range(1,i+1):
                path = f"Results/{file}/Result {i}/{trial}"
                info = read_trail(f"{path}/outputLogFor{robot}with{i}Robots.txt")
                df.loc["Trial Number", f"{file}_{i}Robot_trial{trial}_robot{robot}"] = trial
                df.loc["Robot Number", f"{file}_{i}Robot_trial{trial}_robot{robot}"] = robot
                df.loc["Time taken (s)", f"{file}_{i}Robot_trial{trial}_robot{robot}"] = info[0]
                df.loc["Wall Obstacles (#)", f"{file}_{i}Robot_trial{trial}_robot{robot}"] = info[1]
                df.loc["Number of Collisions (#)", f"{file}_{i}Robot_trial{trial}_robot{robot}"] = info[2]
                occupancy = read_occupancy(f"{path}/outputLogFor{robot}with{i}RobotsAreaOccupancy.txt")
                df.loc["Area occupancy (%)", f"{file}_{i}Robot_trial{trial}_robot{robot}"] = occupancy
                path_robots.append(f"{path}/outputLogFor{robot}with{i}RobotsAreaOccupancy.txt")
                robots.append(f"{file}_{i}Robot_trial{trial}_robot{robot}")
            overall, overlap = read_efficiency(file_names=path_robots)
            df.loc["Efficiency (Overlap)(%)", robots] = overlap
            df.loc["Efficiency (Overall area covered by all robots) (%)", robots] = overall
df.to_csv("BOV.csv")
