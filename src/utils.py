from geographiclib.geodesic import Geodesic
import datetime
import numpy as np
import math
from ast import literal_eval
import pandas as pd




def to_local_runway_frame(lat1,lon1):
    R1 = [40.774548, -79.959237] ##Runway 08
    R2 = [40.778630, -79.942803] ##Runway 26
    geod = Geodesic.WGS84
    g = geod.Inverse(R1[0],R1[1],lat1,lon1)
    r = g['s12']/1000.0
    b = g['azi1']
    x = np.multiply(r,np.sin(np.deg2rad(b)))
    y = np.multiply(r,np.cos(np.deg2rad(b)))

   
    ang = np.deg2rad(72)


    rot = np.array([[np.cos(ang),-np.sin(ang)],[np.sin(ang),np.cos(ang)]])
    p = np.matmul(rot,np.array([x,y]))
    return p[1],-p[0]

def read_traffic_file(path):

    df = pd.read_csv(path, sep = ' ', names = ["Frame","ID","x","y","z","w_x","w_y"])
    counts = df["ID"].value_counts()
    ownship_id = counts.first_valid_index()
    first_frame = df.loc[df["ID"]==ownship_id]["Frame"].iloc[0]
    last_frame = df.loc[df["ID"]==ownship_id]["Frame"].iloc[-1]
    df_pruned = df.loc[(df["Frame"]>=first_frame) & (df["Frame"]<=last_frame)]
    df_based = df_pruned.copy()
    df_based["Frame"] = df_pruned["Frame"] - df_pruned["Frame"].iloc[0]
    # df_pruned.index = df_pruned["Frame"]
    # df_pruned = df_pruned.drop(columns = ["Frame"])
    data_dict = df_based.set_index('ID').groupby('Frame').apply(lambda x: x.to_dict('index')).to_dict()
    print(data_dict[122].keys())
    return data_dict


    