import yaml

with open('map_8by8_obst12_agents15_ex0.yaml') as file:
    obj = yaml.safe_load(file)
    #print(obj['agents'])
    robot_size = len(obj['agents'])
    print (robot_size)
    print(obj['agents'][0]['goal'])
