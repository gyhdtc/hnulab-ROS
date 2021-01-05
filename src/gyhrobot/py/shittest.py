# # coding:utf-8

# import serial.tools.list_ports

# plist = list(serial.tools.list_ports.comports())

# if len(plist) <= 0:
#     print("没有发现端口!")
# else:
#     # plist_0 = list(plist[0])
#     # serialName = plist_0[0]
#     # serialFd = serial.Serial(serialName, 115200, timeout=60)
#     # print("可用端口名>>>", serialFd.name)
#     # print(serialFd.read_all())
#     for i in plist:
#         print("1")
#         print(i)
#         for j in i:
#             print("2")
#             print (j)

# import bluetooth

# nearby_devices = bluetooth.discover_devices(lookup_names=True)
# for addr, name in nearby_devices:
#     print("  %s - %s" % (addr, name))

import bluetooth

nearby_devices = bluetooth.discover_devices(lookup_names=True)
for addr, name in nearby_devices:
    print("  %s - %s" % (addr, name))

    services = bluetooth.find_service(address=addr)
    for svc in services:
        print("Service Name: %s"    % svc["name"])
        print("    Host:        %s" % svc["host"])
        print("    Description: %s" % svc["description"])
        print("    Provided By: %s" % svc["provider"])
        print("    Protocol:    %s" % svc["protocol"])
        print("    channel/PSM: %s" % svc["port"])
        print("    svc classes: %s "% svc["service-classes"])
        print("    profiles:    %s "% svc["profiles"])
        print("    service id:  %s "% svc["service-id"])
        print("")