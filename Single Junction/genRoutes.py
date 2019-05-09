from __future__ import absolute_import
from __future__ import print_function

# import os
# import sys
# import optparse
# import subprocess
import random

random.seed(42)  # make tests reproducible
N = 20000  # number of time steps, initially 3600
# demand per second from different directions
pE = 1. / 20  # 19
pW = 1. / 20  # 21
pN = 1. / 20  # 23
pS = 1. / 20  # 25
with open("data/sq.rou.xml", "w") as routes:
    print("""<routes>

    <vType id="typeW" accel="0.8" decel="4.5" sigma="0.3" length="5" minGap="2.0" maxSpeed="25" guiShape="passenger"/>
    <vType id="typeN" accel="0.8" decel="4.5" sigma="0.3" length="5" minGap="2.0" maxSpeed="25" guiShape="passenger"/>
    <vType id="typeS" accel="0.8" decel="4.5" sigma="0.3" length="5" minGap="2.0" maxSpeed="25" guiShape="passenger"/>
    <vType id="typeE" accel="0.8" decel="4.5" sigma="0.3" length="5" minGap="2.0" maxSpeed="25" guiShape="passenger"/>

    <route id="north-south" edges="ne-n nc cs s-se " />
    <route id="north-west" edges="ne-n nc cw w-we" />
    <route id="east-north" edges="ee-e ec cn n-ne" />
    <route id="east-west" edges="ee-e ec cw w-we" />
    <route id="south-north" edges="se-s sc cn n-ne" />
    <route id="south-east" edges="se-s sc ce e-ee" />
    <route id="west-east" edges="we-w wc ce e-ee" />
    <route id="west-south" edges="we-w wc cs s-se" />
    """, file=routes)
    lastVeh = 0
    vehNr = 0
    for i in range(N):
        if random.uniform(0, 1) < pE:
            print('    <vehicle id="en_%i" type="typeE" route="east-north" depart="%i" color="#800000"/>' % (vehNr, lastVeh), file=routes)
            vehNr += 1
            lastVeh += 1
            # print('    <vehicle id="es_%i" type="typeE" route="east-south" depart="%i" color="#F0E68C"/>' % (vehNr, lastVeh), file=routes)
            # vehNr += 1
            # lastVeh += 1
            print('    <vehicle id="ew_%i" type="typeE" route="east-west" depart="%i" color="#2E8B57"/>' % (vehNr, lastVeh), file=routes)
            vehNr += 1
            lastVeh += 1
        if random.uniform(0, 1) < pW:
            # print('    <vehicle id="wn_%i" type="typeW" route="west-north" depart="%i" color="#800000"/>' % (vehNr, lastVeh), file=routes)
            # vehNr += 1
            print('    <vehicle id="we_%i" type="typeW" route="west-east" depart="%i" color="#4169E1"/>' % (vehNr, lastVeh), file=routes)
            vehNr += 1
            lastVeh += 1
            print('    <vehicle id="ws_%i" type="typeW" route="west-south" depart="%i" color="#F0E68C"/>' % (vehNr, lastVeh), file=routes)
            vehNr += 1
            lastVeh += 1
        if random.uniform(0, 1) < pN:
            # print('    <vehicle id="ne_%i" type="typeN" route="north-east" depart="%i" color="#4169E1"/>' % (vehNr, lastVeh), file=routes)
            # vehNr += 1
            print('    <vehicle id="ns_%i" type="typeN" route="north-south" depart="%i" color="#F0E68C"/>' % (vehNr, lastVeh), file=routes)
            vehNr += 1
            lastVeh += 1
            print('    <vehicle id="nw_%i" type="typeN" route="north-west" depart="%i" color="#2E8B57"/>' % (vehNr, lastVeh), file=routes)
            vehNr += 1
            lastVeh += 1
        if random.uniform(0, 1) < pS:
            print('    <vehicle id="sn_%i" type="typeS" route="south-north" depart="%i" color="#800000"/>' % (vehNr, lastVeh), file=routes)
            vehNr += 1
            lastVeh += 1
            print('    <vehicle id="se_%i" type="typeS" route="south-east" depart="%i" color="#4169E1"/>' % (vehNr, lastVeh), file=routes)
            vehNr += 1
            lastVeh += 1
            # print('    <vehicle id="sw_%i" type="typeS" route="south-west" depart="%i" color="#2E8B57"/>' % (vehNr, lastVeh), file=routes)
            # vehNr += 1
            # lastVeh += 1
    print("""\n</routes>""", file=routes)
