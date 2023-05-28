import  numpy as np
import matplotlib.pyplot as plt
import pandas as pd

LINE_WIDTH=3
MARKER_SIZE=3
# plt.rcParams['text.usetex'] = True
plt.rcParams['font.family'] = "Times New Roman"
# plt.rcParams['font.serif'] = "Times"
plt.rcParams.update(
        {
            'xtick.labelsize': 22,
            'ytick.labelsize': 22,
            "legend.borderpad": 0.2,  ## border whitespace
            "legend.labelspacing": 0.2,  ## the vertical space between the legend entries
            "legend.handlelength": 1,  ## the length of the legend lines
            "legend.handleheight": 0.7,  ## the height of the legend handle
            "legend.handletextpad": 0.2,  ## the space between the legend line and legend text
            "legend.borderaxespad": 0.5,  ## the border between the axes and legend edge
            "legend.columnspacing": 1.0,  ## column separation
            "legend.framealpha": 0.8
        }
    )
font1 = {'family' : 'Serif',
'weight' : 'normal',
'size'   : 20,
}



def plot_item(dataFrame,iterName,plotMarker="o"):
    num_agents=dataFrame["num_agents"].to_numpy()
    item_data=dataFrame[iterName].to_numpy()
    l1,=plt.plot(num_agents,item_data,linewidth=LINE_WIDTH,marker=plotMarker)
    return l1


def plot_item2(dataFrame,iterName,plotMarker="o"):
    num_agents=dataFrame["dist"].to_numpy()
    item_data=dataFrame[iterName].to_numpy()
    l1,=plt.plot(num_agents,item_data,linewidth=LINE_WIDTH,marker=plotMarker)
    return l1


####################################################################
df1=pd.read_csv("./orz201dHCA.csv")
df2=pd.read_csv("./orz201dPIBT.csv")
# df3=pd.read_csv("./orz201dECBSx.csv")
# df4=pd.read_csv("./orz201dPPS.csv")
df5=pd.read_csv("./orz201dUNPPc.csv")


# plt.figure(figsize=(15,10))
# plt.subplot(2,2,1)
# l1=plot_item(df1,"runtime")
# l2=plot_item(df2,'runtime')
# # l3=plot_item(df3,'runtime')
# l5=plot_item(df5,"runtime")
# # l4=plot_item(df4,"runtime")

# # plt.xlabel("num of robots",fontsize=28)
# plt.ylabel("runtime",fontsize=28)

# plt.subplot(2,2,2)
# l1=plot_item(df1,"success_rate")
# l2=plot_item(df2,'success_rate')
# # l3=plot_item(df3,'success_rate')
# l5=plot_item(df5,'success_rate')
# # l4=plot_item(df4,'success_rate')

# # plt.xlabel("num of robots",fontsize=28)
# plt.ylabel("success rate",fontsize=28)
# plt.subplot(2,2,3)
# l1=plot_item(df1,"mkpn")
# l2=plot_item(df2,'mkpn')
# # l3=plot_item(df3,'mkpn')
# l5=plot_item(df5,'mkpn')
# # l4=plot_item(df4,'mkpn')
# plt.legend(handles=[l1,l2,l5],labels=["HCA","PIBT","Proposed"],prop=font1)
# plt.xlabel("num of robots",fontsize=28)
# plt.ylabel("mkpn optimality",fontsize=28)
# # plt.legend(handles=[l1,l2],labels=["ECBS","trans+ECBS"],prop=font1)
# # plt.savefig("halfdense_ecbs.svg",bbox_inches="tight",pad_inches=0.05)
# # plt.show()
# # plt.ylabel("soc optimality",fontsize=28)
# plt.subplot(2,2,4)
# l1=plot_item(df1,"soc")
# l2=plot_item(df2,'soc')
# # l3=plot_item(df3,'soc')
# l5=plot_item(df5,'soc')
# # l4=plot_item(df4,'soc')

# plt.xlabel("num of robots",fontsize=28)
# plt.ylabel("soc optimality",fontsize=28)
# # plt.legend(handles=[l1,l2],labels=["ECBS","trans+ECBS"],prop=font1)
# plt.savefig("orz201d_data.pdf",bbox_inches="tight",pad_inches=0.05)
# plt.show()
#########################################################################

# df1=pd.read_csv("./hrt002dHCA.csv")
# df2=pd.read_csv("./hrt002dPIBT.csv")
# # df3=pd.read_csv("./hrt002dECBS.csv")
# df4=pd.read_csv("./hrt002dUNPPc.csv")


# plt.figure(figsize=(15,10))
# plt.subplot(2,2,1)
# l1=plot_item(df1,"runtime")
# l2=plot_item(df2,'runtime')
# # l3=plot_item(df3,'runtime')
# l4=plot_item(df4,"runtime")

# plt.ylabel("runtime",fontsize=28)

# plt.subplot(2,2,2)
# l1=plot_item(df1,"success_rate")
# l2=plot_item(df2,'success_rate')
# # l3=plot_item(df3,'success rate')
# l4=plot_item(df4,'success_rate')


# plt.ylabel("success rate",fontsize=28)
# plt.subplot(2,2,3)
# l1=plot_item(df1,"mkpn")
# l2=plot_item(df2,'mkpn')
# # l3=plot_item(df3,'mkpn')
# l4=plot_item(df4,'mkpn')
# # l4=plot_item(df4,'mkpn')
# plt.legend(handles=[l1,l2,l4],labels=["HCA","PIBT","Proposed"],prop=font1)
# plt.xlabel("num of robots",fontsize=28)
# plt.ylabel("mkpn optimality",fontsize=28)
# # plt.legend(handles=[l1,l2],labels=["ECBS","trans+ECBS"],prop=font1)
# # plt.savefig("halfdense_ecbs.svg",bbox_inches="tight",pad_inches=0.05)
# # plt.show()
# # plt.ylabel("soc optimality",fontsize=28)
# plt.subplot(2,2,4)
# l1=plot_item(df1,"soc")
# l2=plot_item(df2,'soc')
# # l3=plot_item(df3,'soc')
# l4=plot_item(df4,'soc')
# # l4=plot_item(df4,'soc')

# plt.xlabel("num of robots",fontsize=28)
# plt.ylabel("soc optimality",fontsize=28)
# # plt.legend(handles=[l1,l2],labels=["ECBS","trans+ECBS"],prop=font1)
# plt.savefig("hrt002d_data.pdf",bbox_inches="tight",pad_inches=0.05)
# plt.show()


########################################################################

# df1=pd.read_csv("./d10HCA.csv")
# df2=pd.read_csv("./d10PIBT.csv")
# df3=pd.read_csv("./d10ECBS.csv")
# df4=pd.read_csv("./d10PPS.csv")


# plt.figure(figsize=(15,12))
# plt.subplot(2,2,1)
# l1=plot_item(df1,"runtime")
# l2=plot_item(df2,'runtime')
# l3=plot_item(df3,'runtime')
# l4=plot_item(df4,'runtime')


# plt.xlabel("num of robots",fontsize=28)
# plt.ylabel("runtime",fontsize=28)
# plt.legend(handles=[l1,l2,l3,l4],labels=["HCA","PIBT","ECBS","PPS"],prop=font1)
# plt.subplot(2,2,2)
# l1=plot_item(df1,"success_rate")
# l2=plot_item(df2,'success_rate')
# l3=plot_item(df3,'success_rate')
# l4=plot_item(df4,'success_rate')



# plt.xlabel("num of robots",fontsize=28)

# plt.ylabel("success_rate",fontsize=28)
# plt.subplot(2,2,3)
# l1=plot_item(df1,"mkpn")
# l2=plot_item(df2,'mkpn')
# l3=plot_item(df3,'mkpn')

# l4=plot_item(df4,'mkpn')

# plt.xlabel("num of robots",fontsize=28)
# plt.ylabel("mkpn optimality",fontsize=28)
# # plt.legend(handles=[l1,l2],labels=["ECBS","trans+ECBS"],prop=font1)
# # plt.savefig("halfdense_ecbs.svg",bbox_inches="tight",pad_inches=0.05)
# # plt.show()
# # plt.ylabel("soc optimality",fontsize=28)
# plt.subplot(2,2,4)
# l1=plot_item(df1,"soc")
# l2=plot_item(df2,'soc')
# l3=plot_item(df3,'soc')

# l4=plot_item(df4,'soc')

# plt.xlabel("num of robots",fontsize=28)
# plt.ylabel("soc optimality",fontsize=28)
# # plt.legend(handles=[l1,l2],labels=["ECBS","trans+ECBS"],prop=font1)
# plt.savefig("d10_splot.svg",bbox_inches="tight",pad_inches=0.05)
# plt.show()

###############################################################################

# df1=pd.read_csv("./m100HCA.csv")
# df2=pd.read_csv("./m100PIBT.csv")
# # df3=pd.read_csv("./m100ECBS.csv")



# plt.figure(figsize=(15,12))
# plt.subplot(2,2,1)
# l1=plot_item2(df1,"runtime")
# l2=plot_item2(df2,'runtime')
# # l3=plot_item2(df3,'runtime')



# plt.xlabel("d",fontsize=28)
# plt.ylabel("runtime",fontsize=28)
# plt.legend(handles=[l1,l2],labels=["HCA","PIBT"],prop=font1)
# plt.subplot(2,2,2)
# l1=plot_item2(df1,"success_rate")
# l2=plot_item2(df2,'success_rate')
# # l3=plot_item2(df3,'success_rate')



# plt.xlabel("d",fontsize=28)

# plt.ylabel("success_rate",fontsize=28)
# plt.subplot(2,2,3)
# l1=plot_item2(df1,"mkpn")
# l2=plot_item2(df2,'mkpn')
# # l3=plot_item2(df3,'mkpn')


# plt.xlabel("d",fontsize=28)
# plt.ylabel("mkpn optimality",fontsize=28)
# # plt.legend(handles=[l1,l2],labels=["ECBS","trans+ECBS"],prop=font1)
# # plt.savefig("halfdense_ecbs.svg",bbox_inches="tight",pad_inches=0.05)
# # plt.show()
# # plt.ylabel("soc optimality",fontsize=28)
# plt.subplot(2,2,4)
# l1=plot_item2(df1,"soc")
# l2=plot_item2(df2,'soc')
# # l3=plot_item2(df3,'soc')


# plt.xlabel("d",fontsize=28)
# plt.ylabel("soc optimality",fontsize=28)
# # plt.legend(handles=[l1,l2],labels=["ECBS","trans+ECBS"],prop=font1)
# plt.savefig("m100_splot.svg",bbox_inches="tight",pad_inches=0.05)
# plt.show()

#####################################

# df1=pd.read_csv("./60x60lacam.csv")

# df3=pd.read_csv("./m100ECBS.csv")



# plt.figure(figsize=(15,12))
# plt.subplot(2,2,1)
# l1=plot_item(df1,"runtime")

# # l3=plot_item2(df3,'runtime')



# plt.xlabel("agents",fontsize=28)
# plt.ylabel("runtime",fontsize=28)
# plt.legend(handles=[l1],labels=["Lacam"],prop=font1)
# plt.subplot(2,2,2)
# l1=plot_item(df1,"success_rate")

# # l3=plot_item2(df3,'success_rate')



# plt.xlabel("agents",fontsize=28)

# plt.ylabel("success_rate",fontsize=28)
# plt.subplot(2,2,3)
# l1=plot_item(df1,"mkpn")

# # l3=plot_item2(df3,'mkpn')


# plt.xlabel("agents",fontsize=28)
# plt.ylabel("mkpn optimality",fontsize=28)
# # plt.legend(handles=[l1,l2],labels=["ECBS","trans+ECBS"],prop=font1)
# # plt.savefig("halfdense_ecbs.svg",bbox_inches="tight",pad_inches=0.05)
# # plt.show()
# # plt.ylabel("soc optimality",fontsize=28)
# plt.subplot(2,2,4)
# l1=plot_item(df1,"soc")

# # l3=plot_item2(df3,'soc')


# plt.xlabel("agents",fontsize=28)
# plt.ylabel("soc optimality",fontsize=28)
# # plt.legend(handles=[l1,l2],labels=["ECBS","trans+ECBS"],prop=font1)
# plt.suptitle('Lacam scalibility test in 60x60 grid (random)',fontsize=30)
# plt.savefig("60x60Lacam_splot.svg",bbox_inches="tight",pad_inches=0.05)
# plt.show()