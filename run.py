import subprocess
import json
import time
# Define the range of values for m and k
m_range = list(range(20, 361, 20))
k_range = range(20)
num_cases=len(k_range)
map_name = "orz201d"
output_name = "tmp.json"
map_file="./maps/"+map_name+".map"
csv_files="./data/PIBT_COMPLETE"+map_name+".csv"

def read_data():
    with open(output_name,"r") as f:
        data_dict=json.load(f)
        return data_dict["mkpn"],data_dict["soc"],data_dict["runtime"]



def write_multiple_csv(file_name,itemList,dataList):
    num_items=len(itemList)
    data_length=len(dataList[0])
    with open(file_name,'w') as file:
        for i,itemName in enumerate(itemList):
            file.write(itemName)
            if i==num_items-1:
                file.write('\n')
            else:
                file.write(',')
        for i in range(data_length):
            for j,data in enumerate(dataList):
                file.write(str(data[i]))
                if j==num_items-1:
                    file.write('\n')
                else:
                    file.write(',')


success_rates=[]
agents_data=[]
mkpn_data=[]
soc_data=[]
time_data=[]
itemList=['num_agents', 'mkpn', 'soc','runtime', 'success_rate']

for m in m_range:
    mkpn_total=0
    soc_total=0
    time_total=0
    success_rate=0
    for k in k_range:
        # Create the instance name by formatting the k value into the instance_template
        instance_name = "./instances/"+map_name+"/agents"+f"{m}_{k}.json"
        
        # Call the main executable with the arguments
        cmd = ["./main", map_file, instance_name, output_name]
        t0=time.time()
        try:
            # Run the command using subprocess and wait for it to complete
            subprocess.run(cmd, check=True,timeout=60)
            print(f"Completed run for m={m}, k={k}")
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired):
            # If the program is killed, it is counted as failed
            print(f"Failed to run {cmd}")
        t1=time.time()
        try:
            mkpn,soc,runtime=read_data()
            if mkpn<1:
                raise RuntimeError("failed")
            mkpn_total+=mkpn
            soc_total+=soc
            time_total+=(t1-t0)
            success_rate+=1
        except:
            time_total+=(t1-t0)
            pass
    mkpn_data.append(mkpn_total/success_rate)
    agents_data.append(m)
    soc_data.append(soc_total/success_rate)
    time_data.append(time_total/num_cases)
    success_rates.append(success_rate/num_cases)
    dataList = [agents_data, mkpn_data, soc_data, time_data, success_rates]
    write_multiple_csv(csv_files,itemList,dataList)


