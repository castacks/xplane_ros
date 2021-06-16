import rotation
import numpy as np

def sphericalToCartesian(rho, lat, lon):
    phi = lat
    the = lon
    x = rho * np.cos(phi) * np.cos(the) 
    y = rho * np.cos(phi) * np.sin(the)
    z = rho * np.sin(phi)
    return np.array([[x],
                    [y],
                    [z]])

def cartesianToSpherical(x,y,z):
    pho = np.sqrt(x**2 + y**2 + z**2)
    lon = np.arctan2(y,x)
    base = np.sqrt(x**2 + y**2)
    lat = np.arctan2(z, base)

    return (pho, (180/np.pi)*lat, (180/np.pi)*lon)
    
def localToWorldMatrix(lat, lon):
    lamda = lon * np.pi/180
    phi = lat * np.pi/180
    R = np.array([[-np.sin(lamda), -np.sin(phi) * np.cos(lamda), np.cos(phi) * np.cos(lamda)],
                [np.cos(lamda), -np.sin(phi) * np.sin(lamda), np.cos(phi) * np.sin(lamda)],
                [0, np.cos(phi), np.sin(phi)]])
    return R

def worldToLocalMatrix(lat, lon):
    # phi = lat
    # the = lon
    # R = rotation.RotX(180) * rotation.RotZ(90) * rotation.RotX(phi) * rotation.RotZ(the)
    R = localToWorldMatrix(lat, lon)
    return np.linalg.inv(R)

def pointWorldToLocal(p1_w, p0_w, R_wl):
    return R_wl * (p1_w - p0_w)
    
def pointLocalToWorld(p1_l, p0_w, R_lw):    
    return R_lw * p1_l + p0_w

# if __name__ == "__main__":
#     R = localToWorldMatrix(30,60)
#     print(R)
    
