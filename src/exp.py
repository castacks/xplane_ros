import xpc 

with xpc.XPlaneConnect() as client:
    drefs = []
    drefs.append("sim/flightmodel/position/lat_ref")
    drefs.append("sim/flightmodel/position/lon_ref")

    data = client.getDREFs(drefs)
    print(data)

    drefs = []
    drefs.append("sim/flightmodel/position/local_x")
    drefs.append("sim/flightmodel/position/local_y")
    drefs.append("sim/flightmodel/position/local_z")

    values = [4794.64550781, 304.482330322, 31016.2910156 ]
    client.sendDREFs(drefs, values)

    drefs = []
    drefs.append("sim/flightmodel/position/lat_ref")
    drefs.append("sim/flightmodel/position/lon_ref")

    data = client.getDREFs(drefs)
    print(data)

